#pragma once
// Private experimental fixed-node soft-body solver. No Box2D or Lua dependency.
#include "sb_contact.h"
#include <vector>

namespace SoftBody
{
#if defined(__clang__)
#pragma clang fp contract(off)
#endif
template<class T> struct World
{
    using B=Body<T>;using Vec=typename B::Vec;
    struct Pin
    {
        int a,b;std::array<T,16> wa{},wb{};
        Vec worldAnchor{}; // b == -1 denotes a fixed world endpoint.
        bool active=true;T speed=0,maxTorque=0,gain=0;
    };
    struct TouchJoint
    {
        int body;
        std::array<T,16> weights{};
        Vec target{};
        T frequency=T(3);
        T dampingRatio=T(1);
        T maxForce=T(50);
        T lastForce=0;
        unsigned long long generation=1;
        bool active=true;
    };
    struct Boundary
    {
        // Plane: dot(normal,p)>=offset. Corner: solid x>left && y<top.
        bool active=true,enabled=true;int kind=0;
        Vec normal{T(0),T(1)};T offset=0,target=0,left=0,top=0,friction=T(.8);
    };
    struct Metrics
    {
        double penetration=0,pin=0,nodeSpeed=0,centerSpeed=0,minArea=1,energy=0;
        bool selfCross=false, strictSelfCross=false;
    };
    std::vector<B> bodies;
    std::vector<Pin> pins;
    std::vector<TouchJoint> touchJoints;
    std::vector<Boundary> boundaries;
    // IDs never reused. Explicit collision exclusions, no hidden car part rules.
    std::vector<std::array<int,2>> exclusions;
    Vec gravity{T(0),T(-20)};
    T dt=T(1)/T(480);int iterations=8,substeps=8;
    mutable const char* failure=nullptr;
    double motorWork=0,boundaryWork=0,appliedWork=0,frictionLoss=0;
    V external{};int contacts=0,strictEvents=0,cornerContacts=0;
    Metrics peak;
    std::vector<T> minimumY;
    double maxUnforcedEnergy=0;
    unsigned long long completedSubsteps=0;
    bool pairEnabled(int a,int b) const
    {
        if(!bodies[a].active || !bodies[b].active)return false;
        for(auto p:exclusions)if((p[0]==a&&p[1]==b)||(p[0]==b&&p[1]==a))return false;
        return true;
    }
    Vec anchor(int id,const std::array<T,16>& weights) const
    {
        Vec result{};
        for(int i=0;i<16;++i)for(int k=0;k<2;++k)result[k]+=weights[i]*bodies[id].x[i][k];
        return result;
    }
    Vec pinTarget(const Pin& pin) const
    { return pin.b<0 ? pin.worldAnchor : anchor(pin.b,pin.wb); }
    Vec center(int id) const
    { Vec result{};for(auto p:bodies[id].x)for(int k=0;k<2;++k)result[k]+=p[k]/T(16);return result; }
    double kinetic(int id) const
    { double e=0;for(int i=0;i<16;++i)for(T v:bodies[id].v[i])e+=.5*bodies[id].mass[i]*double(v)*v;return e; }
    double omega(int id) const
    {
        auto c=center(id);const auto& b=bodies[id];double angular=0,inertia=0;
        for(int i=0;i<16;++i)
        { double x=double(b.x[i][0])-c[0],y=double(b.x[i][1])-c[1];inertia+=b.mass[i]*(x*x+y*y);angular+=b.mass[i]*(x*b.v[i][1]-y*b.v[i][0]); }
        if(inertia<1e-12){if(b.kind!=3)failure="degenerate_motor_inertia";return 0;}return angular/inertia;
    }
    void torque(int id,double value)
    {
        auto c=center(id);auto& b=bodies[id];double inertia=0;
        for(int i=0;i<16;++i){double x=double(b.x[i][0])-c[0],y=double(b.x[i][1])-c[1];inertia+=b.mass[i]*(x*x+y*y);}
        if(inertia<1e-12)
        {
            // A coincident recoverable body has no lever arm. Ignore torque
            // until material forces open it; never divide by zero inertia.
            if(b.kind!=3)failure="degenerate_motor_inertia";
            return;
        }
        for(int i=0;i<16;++i)
        { double x=double(b.x[i][0])-c[0],y=double(b.x[i][1])-c[1];b.v[i][0]-=T(double(dt)*value*y/inertia);b.v[i][1]+=T(double(dt)*value*x/inertia); }
    }
    void boundary(int id,int node,Vec normal,T depth,T velocity,T friction)
    {
        auto& part=bodies[id];T j=part.mass[node]*depth/dt;
        for(int k=0;k<2;++k){T delta=depth*normal[k];part.x[node][k]+=delta;part.v[node][k]+=delta/dt;external[k]+=double(j)*normal[k];}
        boundaryWork+=double(j)*velocity;
        Vec tangent{-normal[1],normal[0]};T speed=part.v[node][0]*tangent[0]+part.v[node][1]*tangent[1];
        T jt=std::max(-friction*j,std::min(friction*j,-part.mass[node]*speed));double before=kinetic(id);
        for(int k=0;k<2;++k){T dv=jt*tangent[k]/part.mass[node];part.v[node][k]+=dv;part.x[node][k]+=dt*dv;external[k]+=double(jt)*tangent[k];}
        frictionLoss+=before-kinetic(id);++contacts;
    }
    void corner(int id,const Boundary& boundary)
    {
        auto& part=bodies[id];Vec point{boundary.left,boundary.top};
        if(!inside<T>(point,part))return;
        auto hit=nearest<T>(point,part);if(hit.edge<0){failure="degenerate_contact_edge";return;}if(hit.depth<T(1e-8))return;
        int a=hit.edge,b=(a+1)%16;T wa=T(1)-hit.t,wb=hit.t;
        Vec normal{-hit.normal[0],-hit.normal[1]},tangent{-normal[1],normal[0]};
        T inv=wa*wa/part.mass[a]+wb*wb/part.mass[b],j=hit.depth/(dt*inv);
        auto apply=[&](Vec direction,T impulse)
        {
            for(int k=0;k<2;++k)
            {
                T da=wa*impulse*direction[k]/part.mass[a],db=wb*impulse*direction[k]/part.mass[b];
                part.v[a][k]+=da;part.v[b][k]+=db;part.x[a][k]+=dt*da;part.x[b][k]+=dt*db;
                external[k]+=double(impulse)*direction[k];
            }
        };
        apply(normal,j);T speed=0;
        for(int k=0;k<2;++k)speed+=(wa*part.v[a][k]+wb*part.v[b][k])*tangent[k];
        T jt=std::max(-boundary.friction*j,std::min(boundary.friction*j,-speed/inv));
        double before=kinetic(id);apply(tangent,jt);frictionLoss+=before-kinetic(id);++contacts;++cornerContacts;
    }
    void solveTouchJoint(TouchJoint& joint)
    {
        auto& body=bodies[joint.body];
        Vec position=anchor(joint.body,joint.weights);
        Vec velocity{};
        T inverseMass=0;
        for(int node=0;node<16;++node)
        {
            T weight=joint.weights[node];
            inverseMass+=weight*weight/body.mass[node];
            for(int axis=0;axis<2;++axis)
            {
                velocity[axis]+=weight*body.v[node][axis];
            }
        }

        // Implicit damped spring at a material point. The target is prescribed;
        // its reaction belongs to the external controller, not another body.
        T omega=T(2*3.14159265358979323846)*joint.frequency;
        T damping=T(2)*joint.dampingRatio*omega;
        T denominator=inverseMass*(T(1)+damping*dt+omega*omega*dt*dt);
        Vec force{};
        for(int axis=0;axis<2;++axis)
        {
            T error=joint.target[axis]-position[axis];
            T velocityCoefficient=damping+omega*omega*dt;
            force[axis]=(omega*omega*error-velocityCoefficient*velocity[axis])/denominator;
        }
        T magnitude=std::hypot(force[0],force[1]);
        if(magnitude>joint.maxForce)
        {
            T scale=joint.maxForce/magnitude;
            for(T& value:force)value*=scale;
        }
        joint.lastForce=std::min(magnitude,joint.maxForce);

        double before=kinetic(joint.body);
        for(int node=0;node<16;++node)
        {
            for(int axis=0;axis<2;++axis)
            {
                T impulse=dt*joint.weights[node]*force[axis];
                body.v[node][axis]+=impulse/body.mass[node];
                external[axis]+=impulse;
            }
        }
        appliedWork+=kinetic(joint.body)-before;
    }
    void solveSubstep(const std::vector<T>& offsets,const std::vector<T>& velocities)
    {
        auto previous=bodies;
        for(auto& b:bodies)if(b.active)
        {
            // For the finite recovery spring, damp the incoming motion before
            // adding elastic impulses. Damping those impulses afterwards would
            // reduce the wall reaction without reducing stored elastic energy.
            if(b.kind==3)
            {
                b.dampDeformation(dt);
                if(b.failure){failure=b.failure;return;}
            }
            std::array<Vec,16> f{};b.force(f);if(b.failure){failure=b.failure;return;}
            for(int i=0;i<16;++i)for(int k=0;k<2;++k)b.v[i][k]+=dt*(f[i][k]/b.mass[i]+gravity[k]);
        }
        for(size_t id=0;id<bodies.size();++id)if(bodies[id].active)
        {
            auto& b=bodies[id];double before=kinetic(int(id));
            for(int i=0;i<16;++i)for(int k=0;k<2;++k)
            { T impulse=dt*b.forceInput[k]/T(16);b.v[i][k]+=impulse/b.mass[i];external[k]+=impulse; }
            if(b.torqueInput!=0)torque(int(id),b.torqueInput);
            appliedWork+=kinetic(int(id))-before;
        }
        for(auto& joint:touchJoints)
        {
            if(joint.active)solveTouchJoint(joint);
        }
        std::vector<double> torques(bodies.size(),0);double before=0;
        for(const auto& p:pins)if(p.active && p.maxTorque>0)
        {
            if(p.b<0)
            {
                double value=std::max(-double(p.maxTorque),std::min(double(p.maxTorque),
                    double(p.gain)*(double(p.speed)-omega(p.a))));
                torques[p.a]+=value;
                continue;
            }
            double value=std::max(-double(p.maxTorque),std::min(double(p.maxTorque),double(p.gain)*(double(p.speed)-(omega(p.b)-omega(p.a)))));
            torques[p.b]+=value;torques[p.a]-=value;
        }
        for(size_t i=0;i<bodies.size();++i)if(bodies[i].active)before+=kinetic(int(i));
        for(size_t i=0;i<bodies.size();++i)if(bodies[i].active && torques[i]!=0)torque(int(i),torques[i]);
        double after=0;for(size_t i=0;i<bodies.size();++i)if(bodies[i].active)after+=kinetic(int(i));motorWork+=after-before;
        if(failure)return;
        for(auto& b:bodies)if(b.active)
        { if(b.kind!=3)b.dampDeformation(dt);if(b.failure){failure=b.failure;return;}for(int i=0;i<16;++i)for(int k=0;k<2;++k)b.x[i][k]+=dt*b.v[i][k]; }
        for(int iteration=0;iteration<iterations;++iteration)
        {
            for(size_t id=0;id<bodies.size();++id)if(bodies[id].active)for(int i=0;i<16;++i)
            {
                for(size_t j=0;j<boundaries.size();++j)
                {
                    const auto& b=boundaries[j];if(!b.active || !b.enabled)continue;auto p=bodies[id].x[i];
                    if(b.kind==0)
                    { T depth=offsets[j]-b.normal[0]*p[0]-b.normal[1]*p[1];if(depth>0)boundary(int(id),i,b.normal,depth,velocities[j],b.friction); }
                    else
                    {
                        T left=p[0]-b.left,up=b.top-p[1];if(left>0&&up>0)
                        { if(left<up)boundary(int(id),i,{T(-1),T(0)},left,0,b.friction);else boundary(int(id),i,{T(0),T(1)},up,0,b.friction); }
                    }
                }
            }
            for(const auto& b:boundaries)if(b.active&&b.enabled&&b.kind==1)
                for(size_t id=0;id<bodies.size();++id)if(bodies[id].active)corner(int(id),b);
            for(size_t a=0;a<bodies.size();++a)for(size_t b=a+1;b<bodies.size();++b)if(pairEnabled(int(a),int(b)))
            {
                if(guardEdgeOnly(bodies[a],bodies[b]))++strictEvents;
                double impulse=0;int count=projectPair(bodies[a],bodies[b],dt,impulse,false);
                if(count<0){failure="degenerate_contact_edge";return;}contacts+=count;
            }
            for(const auto& p:pins)if(p.active)for(int k=0;k<2;++k)
            {
                T error=anchor(p.a,p.wa)[k]-pinTarget(p)[k],inv=0;
                for(int i=0;i<16;++i)inv+=p.wa[i]*p.wa[i]/bodies[p.a].mass[i]+(p.b<0 ? T(0) : p.wb[i]*p.wb[i]/bodies[p.b].mass[i]);
                if(!(inv>0)){failure="invalid_pin_mass";return;}T lambda=-error/inv;
                for(int i=0;i<16;++i)
                {
                    T da=p.wa[i]*lambda/bodies[p.a].mass[i],db=p.b<0 ? T(0) : -p.wb[i]*lambda/bodies[p.b].mass[i];
                    bodies[p.a].x[i][k]+=da;bodies[p.a].v[i][k]+=da/dt;
                    if(p.b>=0){bodies[p.b].x[i][k]+=db;bodies[p.b].v[i][k]+=db/dt;}
                    else external[k]+=double(bodies[p.a].mass[i])*da/dt;
                }
            }
        }
        for(size_t a=0;a<bodies.size();++a)for(size_t b=a+1;b<bodies.size();++b)if(pairEnabled(int(a),int(b)))
            if(guardTransit(previous[a],bodies[a],previous[b],bodies[b])||guardTransit(previous[b],bodies[b],previous[a],bodies[a]))++strictEvents;
    }
    Metrics metrics(const std::vector<T>& offsets) const
    {
        Metrics m;
        for(size_t id=0;id<bodies.size();++id)if(bodies[id].active)
        {
            const auto& b=bodies[id];std::array<Vec,16> f{};m.energy+=b.force(f)+kinetic(int(id));
            if(b.failure){failure=b.failure;return m;}m.minArea=std::min(m.minArea,double(b.area()/b.restArea));V cv{};
            for(int i=0;i<16;++i)
            {
                auto p=b.x[i],q=b.x[(i+1)%16];
                for(int k=0;k<2;++k)
                { if(!std::isfinite(double(p[k]))||!std::isfinite(double(b.v[i][k]))){failure="nonfinite";return m;}m.energy-=double(b.mass[i])*gravity[k]*p[k];cv[k]+=double(b.v[i][k])/16; }
                m.nodeSpeed=std::max(m.nodeSpeed,double(std::hypot(b.v[i][0],b.v[i][1])));
                for(size_t j=0;j<boundaries.size();++j)
                {
                    const auto& edge=boundaries[j];if(!edge.active||!edge.enabled)continue;
                    if(edge.kind==0)m.penetration=std::max(m.penetration,double(offsets[j]-edge.normal[0]*p[0]-edge.normal[1]*p[1]));
                    else
                    {
                        double ax=double(p[0])-edge.left,ay=double(edge.top)-p[1],dx=double(q[0])-p[0],dy=double(p[1])-q[1];
                        m.penetration=std::max(m.penetration,std::min(ax,ay));m.penetration=std::max(m.penetration,std::min(ax+dx,ay+dy));
                        if(std::abs(dx-dy)>1e-15){double t=(ay-ax)/(dx-dy);if(t>0&&t<1)m.penetration=std::max(m.penetration,std::min(ax+t*dx,ay+t*dy));}
                    }
                }
                for(int j=i+2;j<16;++j)if(!(i==0&&j==15))
                {
                    bool crossing=strictCross<T>(p,q,b.x[j],b.x[(j+1)%16]);
                    m.selfCross=m.selfCross || crossing;
                    m.strictSelfCross=m.strictSelfCross || (crossing && b.kind!=3);
                }
            }
            m.centerSpeed=std::max(m.centerSpeed,std::hypot(cv[0],cv[1]));
        }
        for(size_t a=0;a<bodies.size();++a)for(size_t b=a+1;b<bodies.size();++b)if(pairEnabled(int(a),int(b)))
        {
            for(auto p:bodies[a].x)if(inside<T>(p,bodies[b]))m.penetration=std::max(m.penetration,double(nearest<T>(p,bodies[b]).depth));
            for(auto p:bodies[b].x)if(inside<T>(p,bodies[a]))m.penetration=std::max(m.penetration,double(nearest<T>(p,bodies[a]).depth));
        }
        for(const auto& p:pins)if(p.active){auto a=anchor(p.a,p.wa),b=pinTarget(p);m.pin=std::max(m.pin,double(std::hypot(a[0]-b[0],a[1]-b[1])));}
        return m;
    }
    Metrics currentMetrics() const
    { std::vector<T> offsets;for(const auto& b:boundaries)offsets.push_back(b.offset);return metrics(offsets); }
    void step()
    {
        if(failure)return;peak=Metrics();
        minimumY.assign(bodies.size(),std::numeric_limits<T>::max());
        maxUnforcedEnergy=-std::numeric_limits<double>::infinity();
        std::vector<T> starts,offsets,velocities;
        for(const auto& b:boundaries){starts.push_back(b.offset);offsets.push_back(b.offset);velocities.push_back((b.target-b.offset)/(dt*T(substeps)));}
        for(int s=1;s<=substeps;++s)
        {
            for(size_t j=0;j<boundaries.size();++j)offsets[j]=starts[j]+(boundaries[j].target-starts[j])*T(s)/T(substeps);
            solveSubstep(offsets,velocities);++completedSubsteps;if(failure)return;
            auto m=metrics(offsets);if(failure)return;
            maxUnforcedEnergy=std::max(maxUnforcedEnergy,m.energy-motorWork-boundaryWork-appliedWork);
            for(size_t id=0;id<bodies.size();++id)if(bodies[id].active)
                for(auto p:bodies[id].x)minimumY[id]=std::min(minimumY[id],p[1]);
            peak.penetration=std::max(peak.penetration,m.penetration);peak.pin=std::max(peak.pin,m.pin);
            peak.nodeSpeed=std::max(peak.nodeSpeed,m.nodeSpeed);peak.centerSpeed=std::max(peak.centerSpeed,m.centerSpeed);
            peak.minArea=std::min(peak.minArea,m.minArea);peak.selfCross=peak.selfCross||m.selfCross;peak.energy=m.energy;
            if(m.strictSelfCross){failure="self_crossing";return;}
        }
        for(auto& b:boundaries)b.offset=b.target;
        for(auto& b:bodies){b.forceInput={};b.torqueInput=0;}
    }
    void removeBody(int id)
    {
        bodies[id].active=false;
        for(auto& pin:pins)
        {
            if(pin.a==id||pin.b==id)pin.active=false;
        }
        for(auto& joint:touchJoints)
        {
            if(joint.body==id)joint.active=false;
        }
    }
};
}
