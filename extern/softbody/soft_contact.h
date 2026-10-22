#pragma once
#include "soft_body.h"
#include <limits>
namespace SoftBody
{
#if defined(__clang__)
#pragma clang fp contract(off)
#endif
template<class T> T cross(typename Body<T>::Vec a,typename Body<T>::Vec b);
template<class T> typename Body<T>::Vec sub(typename Body<T>::Vec a,typename Body<T>::Vec b);
template<class T> bool inside(typename Body<T>::Vec p,const Body<T>& b);
template<class T> struct Hit { int edge; T depth,t; typename Body<T>::Vec normal; Hit(int e=-1,T d=0,T f=0,typename Body<T>::Vec n={}) : edge(e),depth(d),t(f),normal(n) {} };
template<class T> Hit<T> nearest(typename Body<T>::Vec p,const Body<T>& b);
template<class T> bool overlapAabb(const Body<T>& a,const Body<T>& b);
template<class T> bool strictCross(typename Body<T>::Vec a,typename Body<T>::Vec b,typename Body<T>::Vec c,typename Body<T>::Vec d);
template<class T> const char* guardEdgeOnly(const Body<T>& a,const Body<T>& b);
// Crossing guard for linearly moving point and linearly moving segment.
// Detect an unresolved transit; this is detection, not a CCD collision response.
template<class T> const char* guardTransit(const Body<T>& oldA,const Body<T>& a,const Body<T>& oldB,const Body<T>& b);
// Called after normal projection, when the point and weighted edge coincide.
// Velocity-only impulses avoid moving the contact out of the resolved boundary.
template<class T> T applyPairFriction(Body<T>& point, int i, Body<T>& edge,
    int j, int k, T wa, T wb, const typename Body<T>::Vec& normal, T normalImpulse);

template<class T> int projectPair(Body<T>& a,Body<T>& b,T dt,double& impulse,bool strict=true, double* frictionLoss=nullptr);
}
