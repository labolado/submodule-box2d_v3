#include "soft_box2d_static.h"

namespace SoftBody
{
#if defined(__clang__)
#pragma clang fp contract(off)
#endif
bool Box2DStaticContacts::allows(b2Filter a, b2Filter b)
{
    if(a.groupIndex != 0 && a.groupIndex == b.groupIndex)return a.groupIndex > 0;
    return (a.maskBits & b.categoryBits) != 0 && (b.maskBits & a.categoryBits) != 0;
}

const char* Box2DStaticContacts::validate() const
{
    for(const auto& entry : shapes)
    {
        if(!b2Shape_IsValid(entry.id))continue; // Deleted shape/world: never dereference stale IDs.
        if(b2World_IsLocked(b2Shape_GetWorld(entry.id)))return "static_world_locked";
        b2BodyId body = b2Shape_GetBody(entry.id);
        if(!b2Body_IsEnabled(body) || b2Shape_IsSensor(entry.id))continue;
        if(b2Body_GetType(body) != b2_staticBody)return "static_bridge_requires_static_body";
        if(b2Shape_GetRestitution(entry.id) != 0)return "static_bounce_not_supported";
        b2ShapeType type = b2Shape_GetType(entry.id);
        if(type != b2_circleShape && type != b2_polygonShape)return "unsupported_static_shape";
        if(type == b2_polygonShape && b2Shape_GetPolygon(entry.id).radius != 0)
            return "rounded_polygon_not_supported";
    }
    return nullptr;
}

namespace
{
using Vec = Box2DStaticContacts::Vec;
// Maximum depth along a segment inside a convex polygon. The depth is the
// minimum of affine face distances; its maximum occurs at an endpoint or at
// an intersection of two such affine functions.
float segmentDepth(Vec a,Vec b,const std::array<Vec,B2_MAX_POLYGON_VERTICES>& points,int count)
{
    std::array<float,B2_MAX_POLYGON_VERTICES> start{},slope{};
    for(int i=0;i<count;++i)
    {
        Vec edge=sub<float>(points[(i+1)%count],points[i]);
        float length=std::hypot(edge[0],edge[1]);
        Vec n{edge[1]/length,-edge[0]/length};
        start[i]=(points[i][0]-a[0])*n[0]+(points[i][1]-a[1])*n[1];
        slope[i]=(a[0]-b[0])*n[0]+(a[1]-b[1])*n[1];
    }
    auto depth=[&](float t)
    { float d=std::numeric_limits<float>::max();for(int i=0;i<count;++i)d=std::min(d,start[i]+t*slope[i]);return d; };
    float result=std::max(0.f,std::max(depth(0),depth(1)));
    for(int i=0;i<count;++i)for(int j=i+1;j<count;++j)
    {
        float denominator=slope[i]-slope[j];
        if(std::abs(denominator)<1e-12f)continue;
        float t=(start[j]-start[i])/denominator;
        if(t>0 && t<1)result=std::max(result,depth(t));
    }
    return result;
}
float circleSegmentDepth(Vec a,Vec b,Vec center,float radius)
{
    Vec d=sub<float>(b,a),p=sub<float>(center,a);
    float length=d[0]*d[0]+d[1]*d[1];
    float t=length>1e-12f ? std::max(0.f,std::min(1.f,(p[0]*d[0]+p[1]*d[1])/length)) : 0;
    return std::max(0.f,radius-std::hypot(a[0]+t*d[0]-center[0],a[1]+t*d[1]-center[1]));
}
}

bool Box2DStaticContacts::enabled(const Shape& entry,int body) const
{
    if(!b2Shape_IsValid(entry.id))return false;
    if(!b2Body_IsEnabled(b2Shape_GetBody(entry.id)) || b2Shape_IsSensor(entry.id))return false;
    if(std::find(entry.excludedBodies.begin(),entry.excludedBodies.end(),body)!=entry.excludedBodies.end())return false;
    return allows(body<int(filters.size()) ? filters[body] : b2DefaultFilter(),b2Shape_GetFilter(entry.id));
}

std::vector<size_t> Box2DStaticContacts::candidates(const Core& world,int body,bool swept) const
{
    b2AABB bounds={{1e30f,1e30f},{-1e30f,-1e30f}};
    auto include=[&](Vec p)
    {
        if(flipY)p[1]=-p[1];
        bounds.lowerBound.x=std::min(bounds.lowerBound.x,p[0]);bounds.lowerBound.y=std::min(bounds.lowerBound.y,p[1]);
        bounds.upperBound.x=std::max(bounds.upperBound.x,p[0]);bounds.upperBound.y=std::max(bounds.upperBound.y,p[1]);
    };
    for(auto p:world.bodies[body].x)include(p);
    if(swept && body<int(previous.size()))for(auto p:previous[body])include(p);
    std::vector<uint32_t> worlds;
    std::vector<uint64_t> found;
    for(const auto& entry:shapes)if(b2Shape_IsValid(entry.id))
    {
        b2WorldId id=b2Shape_GetWorld(entry.id);uint32_t key=b2StoreWorldId(id);
        if(std::find(worlds.begin(),worlds.end(),key)!=worlds.end())continue;
        worlds.push_back(key);
        b2QueryFilter filter;filter.categoryBits=UINT64_MAX;filter.maskBits=UINT64_MAX;
        b2World_OverlapAABB(id,b2Pos_zero,bounds,filter,[](b2ShapeId shape,void* context)
        { static_cast<std::vector<uint64_t>*>(context)->push_back(b2StoreShapeId(shape));return true; },&found);
        ++queryCount;
    }
    std::vector<size_t> result;
    // Preserve registration order; broadphase traversal order is not solver order.
    for(size_t i=0;i<shapes.size();++i)if(enabled(shapes[i],body))
    {
        auto filter=b2Shape_GetFilter(shapes[i].id);
        // Query filters cannot express positive groupIndex overrides and may
        // omit zero-mask/category shapes. Explicit registration supplies those.
        if(std::find(found.begin(),found.end(),b2StoreShapeId(shapes[i].id))!=found.end() ||
            filter.maskBits==0 || filter.categoryBits==0)result.push_back(i);
    }
    candidateCount+=result.size();return result;
}

void Box2DStaticContacts::polygonPoints(b2ShapeId shape,std::array<Vec,B2_MAX_POLYGON_VERTICES>& points,int& count) const
{
    b2Polygon polygon=b2Shape_GetPolygon(shape);count=polygon.count;
    for(int i=0;i<count;++i)
    {
        b2Pos p=b2Body_GetWorldPoint(b2Shape_GetBody(shape),polygon.vertices[i]);
        points[flipY ? count-1-i : i]={float(p.x),float(flipY ? -p.y : p.y)};
    }
}
Box2DStaticContacts::Vec Box2DStaticContacts::circleCenter(b2ShapeId shape) const
{
    b2Pos p=b2Body_GetWorldPoint(b2Shape_GetBody(shape),b2Shape_GetCircle(shape).center);
    return {float(p.x),float(flipY ? -p.y : p.y)};
}

bool Box2DStaticContacts::step(Core& world) const
{
    if(world.failure)return false;
    if(const char* error=validate()){world.failure=error;return false;}
    queryCount=candidateCount=0;previous.clear();
    for(const auto& body:world.bodies)previous.push_back(body.x);
    world.step([](Core& target,const void* context)
    { (*static_cast<const Box2DStaticContacts*>(context))(target); },this,
        [](const Core& target,const void* context)
        { return static_cast<const Box2DStaticContacts*>(context)->penetration(target); });
    previous.clear();return !world.failure;
}

void Box2DStaticContacts::operator()(Core& world) const
{
    for(size_t id=0;id<world.bodies.size();++id)if(world.bodies[id].active)
    {
        for(size_t index:candidates(world,int(id),true))
        {
            b2ShapeId shape=shapes[index].id;
            float friction=std::sqrt(world.bodies[id].friction*b2Shape_GetFriction(shape));
            bool circle=b2Shape_GetType(shape)==b2_circleShape;
            std::array<Vec,B2_MAX_POLYGON_VERTICES> points{};int count=0;
            Vec center{};float radius=0;
            if(circle){center=circleCenter(shape);radius=b2Shape_GetCircle(shape).radius;}
            else polygonPoints(shape,points,count);
            // This is a transit guard, not CCD response. Reject an unsupported
            // completed crossing rather than quietly accepting an empty end pose.
            auto depth=[&](Vec p,Vec q)
            { return circle ? circleSegmentDepth(p,q,center,radius) : segmentDepth(p,q,points,count); };
            bool endOverlap=false;
            for(int node=0;node<16;++node)
                endOverlap=endOverlap || depth(world.bodies[id].x[node],world.bodies[id].x[(node+1)%16])>0;
            if(circle)endOverlap=endOverlap || inside<float>(center,world.bodies[id]);
            else for(int i=0;i<count;++i)endOverlap=endOverlap || inside<float>(points[i],world.bodies[id]);
            if(!endOverlap && id<previous.size())
            {
                Vec a{},b{};
                bool startOverlap=false;
                for(int node=0;node<16;++node)
                {
                    for(int axis=0;axis<2;++axis)
                    { a[axis]+=previous[id][node][axis]/16;b[axis]+=world.bodies[id].x[node][axis]/16; }
                    startOverlap=startOverlap || depth(previous[id][node],previous[id][(node+1)%16])>0;
                }
                // Diagnose complete-body transit. A single contour node can
                // sweep a corner while the body legitimately leaves a finite
                // platform; that is not a complete-body crossing certificate.
                if(!startOverlap && depth(a,a)<=0 && depth(b,b)<=0 && depth(a,b)>1e-5f)
                { world.failure="static_swept_transit";return; }
            }
            if(circle)projectCircle(world,int(id),center,radius,friction);
            else projectPolygon(world,int(id),points,count,friction);
            if(world.failure)return;
        }
    }
    if(!previous.empty())for(size_t i=0;i<world.bodies.size();++i)previous[i]=world.bodies[i].x;
}

double Box2DStaticContacts::penetration(const Core& world) const
{
    if(shapes.empty() || validate())return 0;
    float maximum=0;
    for(size_t id=0;id<world.bodies.size();++id)if(world.bodies[id].active)
    for(size_t index:candidates(world,int(id),false))
    {
        const auto& body=world.bodies[id];b2ShapeId shape=shapes[index].id;
        if(b2Shape_GetType(shape)==b2_circleShape)
        {
            Vec center=circleCenter(shape);float radius=b2Shape_GetCircle(shape).radius;
            for(int i=0;i<16;++i)maximum=std::max(maximum,circleSegmentDepth(body.x[i],body.x[(i+1)%16],center,radius));
            if(inside<float>(center,body))maximum=std::max(maximum,radius+nearest<float>(center,body).depth);
        }
        else
        {
            std::array<Vec,B2_MAX_POLYGON_VERTICES> points;int count;polygonPoints(shape,points,count);
            for(int i=0;i<16;++i)maximum=std::max(maximum,segmentDepth(body.x[i],body.x[(i+1)%16],points,count));
            for(int i=0;i<count;++i)if(inside<float>(points[i],body))maximum=std::max(maximum,nearest<float>(points[i],body).depth);
        }
    }
    return maximum;
}

void Box2DStaticContacts::edgeContact(Core& world,int id,int edge,float fraction,Vec normal,float depth,float friction)
{
    if(depth <= 1e-8f)return;
    auto& body = world.bodies[id];
    int next = (edge + 1) % 16;
    float a = 1 - fraction, b = fraction;
    float inverseMass = a*a/body.mass[edge] + b*b/body.mass[next];
    float impulse = depth / (world.dt * inverseMass);
    Vec tangent{-normal[1],normal[0]};
    auto apply = [&](Vec direction,float value,bool move)
    {
        for(int k = 0; k < 2; ++k)
        {
            float va = a*value*direction[k]/body.mass[edge];
            float vb = b*value*direction[k]/body.mass[next];
            body.v[edge][k] += va;body.v[next][k] += vb;
            if(move){body.x[edge][k] += world.dt*va;body.x[next][k] += world.dt*vb;}
            world.external[k] += value*direction[k];
        }
    };
    apply(normal,impulse,true);
    float speed = 0;
    for(int k = 0; k < 2; ++k)speed += (a*body.v[edge][k]+b*body.v[next][k])*tangent[k];
    float limit = friction*impulse;
    float jt = std::max(-limit,std::min(limit,-speed/inverseMass));
    double before = world.kinetic(id);
    apply(tangent,jt,false);
    world.frictionLoss += before-world.kinetic(id);
    ++world.contacts;
}

void Box2DStaticContacts::projectCircle(Core& world,int id,Vec center,float radius,float friction)
{
    // Closest contour point covers node-circle and edge-circle contacts,
    // including a small circle touching between two soft nodes.
    auto& body = world.bodies[id];
    auto hit = nearest<float>(center,body);
    if(hit.edge < 0){world.failure = "degenerate_static_contact_edge";return;}
    bool contained = inside<float>(center,body);
    float depth = contained ? radius+hit.depth : radius-hit.depth;
    Vec normal = hit.normal;
    if(contained)for(float& value : normal)value = -value;
    edgeContact(world,id,hit.edge,hit.t,normal,depth,friction);
}

void Box2DStaticContacts::projectPolygon(Core& world,int id,
    const std::array<Vec,B2_MAX_POLYGON_VERTICES>& points,int count,float friction)
{
    auto& body = world.bodies[id];
    for(int node = 0; node < 16; ++node)
    {
        float closest = -std::numeric_limits<float>::max();
        Vec normal{};
        for(int edge = 0; edge < count; ++edge)
        {
            Vec delta = sub<float>(points[(edge+1)%count],points[edge]);
            float length = std::hypot(delta[0],delta[1]);
            Vec n{delta[1]/length,-delta[0]/length};
            Vec relative = sub<float>(body.x[node],points[edge]);
            float separation = relative[0]*n[0]+relative[1]*n[1];
            if(separation > closest){closest = separation;normal = n;}
        }
        if(closest < 0)world.boundary(id,node,normal,-closest,0,friction);
    }
    // A convex vertex can enter a soft edge without containing any node.
    for(int i = 0; i < count; ++i)world.boundaryVertex(id,points[i],friction);
    // Use one separating face for a body/polygon pair. Independent nearest-face
    // choices can send opposite ends to opposite sides of a thin wall forever.
    // For initial overlap without motion history, choose the polygon face with
    // the smallest whole-body translation. Equal candidates keep face order.
    float bestSeparation = -std::numeric_limits<float>::max();
    Vec separatingNormal{};
    float separatingOffset = 0;
    for(int face = 0; face < count; ++face)
    {
        Vec delta = sub<float>(points[(face+1)%count],points[face]);
        float length = std::hypot(delta[0],delta[1]);
        Vec normal{delta[1]/length,-delta[0]/length};
        float offset = points[face][0]*normal[0]+points[face][1]*normal[1];
        float minimum = std::numeric_limits<float>::max();
        for(auto point : body.x)
            minimum = std::min(minimum,point[0]*normal[0]+point[1]*normal[1]-offset);
        if(minimum > bestSeparation)
        { bestSeparation = minimum;separatingNormal = normal;separatingOffset = offset; }
    }
    const auto edgePositions = body.x;
    // Clip each material edge against the convex interior, then constrain its
    // endpoint support values to the selected face. A midpoint constraint alone
    // does not prevent the rest of the segment from remaining inside the solid.
    for(int edge = 0; edge < 16; ++edge)
    {
        Vec a = edgePositions[edge], b = edgePositions[(edge+1)%16];
        float enter = 0, leave = 1;
        bool intersects = true;
        for(int face = 0; face < count; ++face)
        {
            Vec delta = sub<float>(points[(face+1)%count],points[face]);
            float length = std::hypot(delta[0],delta[1]);
            Vec normal{delta[1]/length,-delta[0]/length};
            auto separation = [&](Vec p)
            { return (p[0]-points[face][0])*normal[0]+(p[1]-points[face][1])*normal[1]; };
            float start = separation(a), finish = separation(b);
            if(start >= 0 && finish >= 0){intersects = false;break;}
            if(start > 0)enter = std::max(enter,start/(start-finish));
            if(finish > 0)leave = std::min(leave,start/(start-finish));
        }
        if(!intersects || leave <= enter)continue;
        for(int node : {edge,(edge+1)%16})
        {
            float separation = body.x[node][0]*separatingNormal[0]+
                body.x[node][1]*separatingNormal[1]-separatingOffset;
            if(separation >= 0)continue;
            // Split positional correction from the impact impulse. In particular,
            // an initially overlapping stationary body must not acquire depth/dt
            // launch velocity merely to untangle its contour.
            for(int axis = 0; axis < 2; ++axis)
                body.x[node][axis] += (1e-6f-separation)*separatingNormal[axis];
            float speed = body.v[node][0]*separatingNormal[0]+body.v[node][1]*separatingNormal[1];
            if(speed < 0)
            {
                float impulse = -body.mass[node]*speed;
                Vec tangent{-separatingNormal[1],separatingNormal[0]};
                float tangentSpeed = body.v[node][0]*tangent[0]+body.v[node][1]*tangent[1];
                float limit = friction*impulse;
                float tangentImpulse = std::max(-limit,std::min(limit,-body.mass[node]*tangentSpeed));
                for(int axis = 0; axis < 2; ++axis)
                {
                    float value = impulse*separatingNormal[axis]+tangentImpulse*tangent[axis];
                    body.v[node][axis] += value/body.mass[node];
                    world.external[axis] += value;
                }
                world.frictionLoss -= double(tangentImpulse)*tangentSpeed+
                    .5*double(tangentImpulse)*tangentImpulse/body.mass[node];
            }
            ++world.contacts;
        }
    }

}
}
