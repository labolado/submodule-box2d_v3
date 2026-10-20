#pragma once
#include "sb_body.h"
#include <limits>
namespace SoftBody
{
#if defined(__clang__)
#pragma clang fp contract(off)
#endif
template<class T> T cross(typename Body<T>::Vec a,typename Body<T>::Vec b)
{ return a[0]*b[1]-a[1]*b[0]; }
template<class T> typename Body<T>::Vec sub(typename Body<T>::Vec a,typename Body<T>::Vec b)
{ return {a[0]-b[0],a[1]-b[1]}; }
template<class T> bool inside(typename Body<T>::Vec p,const Body<T>& b)
{
    bool result=false;
    for (int i=0,j=15;i<16;j=i++)
    {
        auto a=b.x[i],c=b.x[j];
        if ((a[1]>p[1])!=(c[1]>p[1]) && double(p[0])<(double(c[0])-a[0])*(double(p[1])-a[1])/(double(c[1])-a[1])+a[0]) result=!result;
    }
    return result;
}
template<class T> struct Hit { int edge; T depth,t; typename Body<T>::Vec normal; Hit(int e=-1,T d=0,T f=0,typename Body<T>::Vec n={}) : edge(e),depth(d),t(f),normal(n) {} };
template<class T> Hit<T> nearest(typename Body<T>::Vec p,const Body<T>& b)
{
    Hit<T> hit; T best=std::numeric_limits<T>::max();
    for (int e=0;e<16;++e)
    {
        auto a=b.x[e],d=sub<T>(b.x[(e+1)%16],a),pa=sub<T>(p,a);
        T length2=d[0]*d[0]+d[1]*d[1];
        if (!(length2>T(1e-12))) return Hit<T>();
        T t=std::max(T(0),std::min(T(1),(pa[0]*d[0]+pa[1]*d[1])/length2));
        typename Body<T>::Vec delta{a[0]+t*d[0]-p[0],a[1]+t*d[1]-p[1]};
        T distance=std::hypot(delta[0],delta[1]);
        if (distance<best) { best=distance;hit={e,distance,t,{d[1]/std::sqrt(length2),-d[0]/std::sqrt(length2)}}; if (distance>T(1e-8)) for (int k=0;k<2;++k) hit.normal[k]=delta[k]/distance; }
    }
    return hit;
}
template<class T> bool overlapAabb(const Body<T>& a,const Body<T>& b)
{
    for (int k=0;k<2;++k)
    {
        T al=a.x[0][k],ah=al,bl=b.x[0][k],bh=bl;
        for (int i=1;i<16;++i) { al=std::min(al,a.x[i][k]);ah=std::max(ah,a.x[i][k]);bl=std::min(bl,b.x[i][k]);bh=std::max(bh,b.x[i][k]); }
        if (ah<bl || bh<al) return false;
    }
    return true;
}
template<class T> bool strictCross(typename Body<T>::Vec a,typename Body<T>::Vec b,typename Body<T>::Vec c,typename Body<T>::Vec d)
{
    auto orient=[](typename Body<T>::Vec p,typename Body<T>::Vec q,typename Body<T>::Vec r)
    { return (double(q[0])-p[0])*(double(r[1])-p[1])-(double(q[1])-p[1])*(double(r[0])-p[0]); };
    return orient(a,b,c)*orient(a,b,d)<-1e-16 && orient(c,d,a)*orient(c,d,b)<-1e-16;
}
template<class T> const char* guardEdgeOnly(const Body<T>& a,const Body<T>& b)
{
    if (!overlapAabb(a,b)) return nullptr;
    for (int i=0;i<16;++i) if (inside<T>(a.x[i],b) || inside<T>(b.x[i],a)) return nullptr;
    for (int i=0;i<16;++i) for (int j=0;j<16;++j)
        if (strictCross<T>(a.x[i],a.x[(i+1)%16],b.x[j],b.x[(j+1)%16])) return "edge_only_crossing";
    return nullptr;
}
// Crossing guard for linearly moving point and linearly moving segment.
// Detect an unresolved transit; this is detection, not a CCD collision response.
template<class T> const char* guardTransit(const Body<T>& oldA,const Body<T>& a,const Body<T>& oldB,const Body<T>& b)
{
    for (int i=0;i<16;++i)
    {
        if (inside<T>(oldA.x[i],oldB) || inside<T>(a.x[i],b)) continue;
        for (int e=0;e<16;++e)
        {
            int j=(e+1)%16;
            auto u=sub<T>(oldA.x[i],oldB.x[e]),v=sub<T>(oldB.x[j],oldB.x[e]);
            auto du=sub<T>(sub<T>(a.x[i],b.x[e]),u),dv=sub<T>(sub<T>(b.x[j],b.x[e]),v);
            double aa=cross<T>(du,dv),bb=cross<T>(du,v)+cross<T>(u,dv),cc=cross<T>(u,v);
            double roots[2]{};int count=0;
            if (std::abs(aa)<1e-12) { if (std::abs(bb)>1e-12) roots[count++]=-cc/bb; }
            else
            {
                double disc=bb*bb-4*aa*cc;
                if (disc>=0) { roots[count++]=(-bb-std::sqrt(disc))/(2*aa);roots[count++]=(-bb+std::sqrt(disc))/(2*aa); }
            }
            for (int r=0;r<count;++r)
            {
                double t=roots[r];if (t<=1e-6 || t>=1-1e-6) continue;
                double px=u[0]+t*du[0],py=u[1]+t*du[1],ex=v[0]+t*dv[0],ey=v[1]+t*dv[1];
                double length2=ex*ex+ey*ey;
                if (length2<1e-12) return "degenerate_swept_edge";
                double fraction=(px*ex+py*ey)/length2;
                if (fraction>1e-6 && fraction<1-1e-6) return "unresolved_swept_transit";
            }
        }
    }
    return nullptr;
}
template<class T> int projectPair(Body<T>& a,Body<T>& b,T dt,double& impulse,bool strict=true)
{
    if (!overlapAabb(a,b)) return 0;
    if (strict && guardEdgeOnly(a,b)) return -1;int count=0;
    for (int side=0;side<2;++side)
    {
        auto& point=side==0 ? a : b;auto& edge=side==0 ? b : a;
        for (int i=0;i<16;++i)
        {
            if (!inside<T>(point.x[i],edge)) continue;
            auto hit=nearest<T>(point.x[i],edge);if(hit.edge<0)return -1;if (hit.depth<T(1e-8)) continue;
            int j=hit.edge,k=(j+1)%16;T wa=T(1)-hit.t,wb=hit.t;
            T inv=T(1)/point.mass[i]+wa*wa/edge.mass[j]+wb*wb/edge.mass[k];
            T lambda=hit.depth/inv;impulse+=double(lambda/dt);++count;
            auto apply=[&](Body<T>& body,int n,T weight)
            {
                for (int d=0;d<2;++d) { T delta=weight*lambda*hit.normal[d]/body.mass[n];body.x[n][d]+=delta;body.v[n][d]+=delta/dt; }
            };
            apply(point,i,T(1));apply(edge,j,-wa);apply(edge,k,-wb);
        }
    }
    return count;
}
}
