#pragma once
#include <array>
#include <cmath>

#include <algorithm>

namespace SoftBody
{
#if defined(__clang__)
#pragma clang fp contract(off)
#endif
using V = std::array<double,2>;
template<class T> struct Body
{
    using Vec = std::array<T,2>;
    std::array<Vec,16> x{}, v{}, rest{};
    std::array<T,16> mass{}, lengths{};
    mutable const char* failure=nullptr;
    int kind; // 0: local frames, 1: global frame, 2: pressure tire, 3: recoverable
    mutable double recoveryCosine=1, recoverySine=0;
    T restArea;
    T preload=T(0);
    T materialScale=T(1);
    bool active=true;
    Vec forceInput{};T torqueInput=0;
    T stiffness,frequency,damping,frameOverlap;
    T bulk=T(300);
    Body(int model, T totalMass, T rx, T ry, T edgeStiffness, T hz,
         T dampingRate, T pressure, T scale, T overlap) : kind(model),
        stiffness(edgeStiffness),frequency(hz),damping(dampingRate),frameOverlap(overlap)
    {
        const T pi=T(3.14159265358979323846);
        materialScale=scale;preload=pressure;
        for(int i=0;i<16;++i)
        {
            T a=T(2)*pi*T(i)/T(16);
            rest[i]=x[i]={rx*std::cos(a),ry*std::sin(a)};
            mass[i]=totalMass/T(16);
        }
        for(int i=0;i<16;++i)
        { auto a=x[i],b=x[(i+1)%16];lengths[i]=std::hypot(a[0]-b[0],a[1]-b[1]); }
        restArea=area();
        if(kind==2 && preload!=T(0))
        {
            // Initial pressure-balanced pretension currently requires a circle.
            T tension=preload*rx*std::cos(pi/T(16));
            for(auto& l:lengths)l-=tension/stiffness;
        }
    }
    T area() const
    {
        T a=0;
        // Local origin reduces cancellation under large translations.
        for (int i=1;i<15;++i)
            a+=(x[i][0]-x[0][0])*(x[i+1][1]-x[0][1])-(x[i][1]-x[0][1])*(x[i+1][0]-x[0][0]);
        return a/T(2);
    }
    // Finite corotated quadratic energy. Unlike pressure, this material is
    // defined for signed/zero area and overlapping nodes. Node identities and
    // rest coordinates never change; forces, not a rest-pose teleport, unfold it.
    T recoverableForce(std::array<Vec,16>& forces) const
    {
        double totalMass=0;
        V center{}, restCenter{};
        for(int node=0;node<16;++node)
        {
            totalMass+=mass[node];
            for(int axis=0;axis<2;++axis)
            {
                center[axis]+=double(mass[node])*x[node][axis];
                restCenter[axis]+=double(mass[node])*rest[node][axis];
            }
        }
        for(int axis=0;axis<2;++axis)
        {
            center[axis]/=totalMass;
            restCenter[axis]/=totalMass;
        }
        double dot=0, cross=0;
        for(int node=0;node<16;++node)
        {
            double rx=rest[node][0]-restCenter[0], ry=rest[node][1]-restCenter[1];
            double px=x[node][0]-center[0], py=x[node][1]-center[1];
            dot+=mass[node]*(rx*px+ry*py);
            cross+=mass[node]*(rx*py-ry*px);
        }
        double norm=std::hypot(dot,cross);
        if(norm>1e-12)
        {
            // Proper rotation only: never fit a reflection to an inverted body.
            recoveryCosine=dot/norm;
            recoverySine=cross/norm;
        }
        // A completely ambiguous covariance retains the last defined rotation.
        double omega=2*3.14159265358979323846*double(frequency);
        double energy=0;
        for(int node=0;node<16;++node)
        {
            double rx=rest[node][0]-restCenter[0], ry=rest[node][1]-restCenter[1];
            V error{x[node][0]-center[0]-recoveryCosine*rx+recoverySine*ry,
                    x[node][1]-center[1]-recoverySine*rx-recoveryCosine*ry};
            double stiffness=omega*omega*mass[node]*materialScale;
            for(int axis=0;axis<2;++axis)
            {
                forces[node][axis]=T(-stiffness*error[axis]);
                energy+=.5*stiffness*error[axis]*error[axis];
            }
        }
        return T(energy);
    }
    T force(std::array<Vec,16>& f) const
    {
        f={}; T energy=0;
        if(kind==3)return recoverableForce(f);
        for (int i=0;i<16;++i)
        {
            int j=(i+1)%16;
            Vec d{x[j][0]-x[i][0],x[j][1]-x[i][1]};
            T l=std::hypot(d[0],d[1]);
            if (l<T(1e-8)) { failure="collapsed_edge";return T(0); }
            T extension=l-lengths[i]; energy+=(stiffness/T(2))*extension*extension;
            for (int k=0;k<2;++k)
            { T q=stiffness*extension*d[k]/l; f[i][k]+=q; f[j][k]-=q; }
        }
        if (kind==2)
        {
            T a=area();
            if (!(a>T(.2)*restArea)) { failure="pressure_area_domain";return T(0); }
            energy+=preload==T(0) ? bulk*(a-restArea-restArea*std::log(a/restArea))
                : bulk*(a-restArea)-(bulk+preload)*restArea*std::log(a/restArea);
            T pressure=preload==T(0) ? bulk*(restArea/a-T(1))
                : (bulk+preload)*restArea/a-bulk;
            for (int i=0;i<16;++i)
            {
                auto prev=x[(i+15)%16],next=x[(i+1)%16];
                f[i][0]+=pressure*(next[1]-prev[1])/T(2);
                f[i][1]+=pressure*(prev[0]-next[0])/T(2);
            }
            return energy;
        }
        // Small frame reductions use double even for float state; no force balancing patch.
        const double omega=2*3.14159265358979323846*double(frequency);
        for (int frame=0;frame<(kind==0 ? 2 : 1);++frame)
        {
            std::array<double,16> weight{};
            std::array<double,2> c{},r{}; double total=0;
            for (int i=0;i<16;++i)
            {
                int membership=(rest[i][0]<=frameOverlap)+(rest[i][0]>=-frameOverlap);
                bool included=kind==1 || (frame==0 ? rest[i][0]<=frameOverlap : rest[i][0]>=-frameOverlap);
                weight[i]=included ? double(mass[i])/double(kind==1 ? 1 : membership) : T(0);
                total+=weight[i];
                for (int k=0;k<2;++k) { c[k]+=weight[i]*x[i][k]; r[k]+=weight[i]*rest[i][k]; }
            }
            for (int k=0;k<2;++k) { c[k]/=total; r[k]/=total; }
            double dot=0,cross=0;
            for (int i=0;i<16;++i)
            {
                std::array<double,2> a{double(rest[i][0])-r[0],double(rest[i][1])-r[1]},b{double(x[i][0])-c[0],double(x[i][1])-c[1]};
                dot+=weight[i]*(a[0]*b[0]+a[1]*b[1]);
                cross+=weight[i]*(a[0]*b[1]-a[1]*b[0]);
            }
            double norm=std::hypot(dot,cross);
            if (norm<T(1e-10)) { failure="ambiguous_frame";return T(0); }
            double co=dot/norm,si=cross/norm;
            for (int i=0;i<16;++i)
            {
                std::array<double,2> a{double(rest[i][0])-r[0],double(rest[i][1])-r[1]};
                std::array<double,2> d{x[i][0]-c[0]-co*a[0]+si*a[1],x[i][1]-c[1]-si*a[0]-co*a[1]};
                for (int k=0;k<2;++k)
                { f[i][k]-=T(omega*omega*weight[i]*d[k]); energy+=T(.5*omega*omega*weight[i]*d[k]*d[k]); }
            }
        }
        if (materialScale!=T(1))
        {
            for (auto& force:f) for (T& component:force) component*=materialScale;
            energy*=materialScale;
        }
        return energy;
    }
    T kinetic() const
    {
        T e=0; for (int i=0;i<16;++i) e+=T(.5)*mass[i]*(v[i][0]*v[i][0]+v[i][1]*v[i][1]);
        return e;
    }
    // Mass-orthogonal projection onto translation and rotation at fixed positions.
    // Only the residual velocity decays; reductions use double for float state.
    void dampDeformation(T dt)
    {
        double total=0, inertia=0, angular=0;
        V center{}, velocity{};
        for (int i=0;i<16;++i)
        {
            total+=mass[i];
            for (int k=0;k<2;++k) { center[k]+=double(mass[i])*x[i][k]; velocity[k]+=double(mass[i])*v[i][k]; }
        }
        for (int k=0;k<2;++k) { center[k]/=total; velocity[k]/=total; }
        for (int i=0;i<16;++i)
        {
            double rx=double(x[i][0])-center[0], ry=double(x[i][1])-center[1];
            inertia+=double(mass[i])*(rx*rx+ry*ry);
            angular+=double(mass[i])*(rx*(double(v[i][1])-velocity[1])-ry*(double(v[i][0])-velocity[0]));
        }
        if (!(inertia>1e-12))
        {
            if(kind!=3) { failure="degenerate_damping_inertia";return; }
            // At a point there is no defined rigid angular velocity. Dampen
            // only velocity relative to translation until the shape opens.
            double decay=-std::expm1(-double(damping)*double(dt));
            for(auto& nodeVelocity:v)for(int axis=0;axis<2;++axis)
                nodeVelocity[axis]=T(nodeVelocity[axis]-decay*(nodeVelocity[axis]-velocity[axis]));
            return;
        }
        double omega=angular/inertia, decay=-std::expm1(-double(damping)*double(dt));
        for (int i=0;i<16;++i)
        {
            V rigid{velocity[0]-omega*(double(x[i][1])-center[1]),velocity[1]+omega*(double(x[i][0])-center[0])};
            for (int k=0;k<2;++k) v[i][k]=T(double(v[i][k])-decay*(double(v[i][k])-rigid[k]));
        }
    }
    void damp(T dt)
    {
        T decay=-std::expm1(-damping*dt);
        for (int i=0;i<16;++i)
        {
            int j=(i+1)%16; Vec d{x[j][0]-x[i][0],x[j][1]-x[i][1]};
            T l=std::hypot(d[0],d[1]);
            if (l<T(1e-8)) { failure="collapsed_edge";return; }
            for (T& q:d) q/=l;
            T speed=(v[j][0]-v[i][0])*d[0]+(v[j][1]-v[i][1])*d[1];
            T impulse=decay*speed/(T(1)/mass[i]+T(1)/mass[j]);
            for (int k=0;k<2;++k) { v[i][k]+=impulse*d[k]/mass[i]; v[j][k]-=impulse*d[k]/mass[j]; }
        }
    }
};
}
