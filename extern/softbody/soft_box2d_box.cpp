#include "soft_box2d_box.h"
#include <limits>

namespace SoftBody
{
#if defined(__clang__)
#pragma clang fp contract(off)
#endif
namespace
{
using Core=Box2DBoxContacts::Core;
const b2Vec2 normals[4]={{-1,0},{1,0},{0,-1},{0,1}};

const char* dimensions(b2ShapeId id,float& hx,float& hy)
{
    if(b2Shape_GetType(id)!=b2_polygonShape)return "box_bridge_requires_polygon";
    auto polygon=b2Shape_GetPolygon(id);
    if(polygon.count!=4 || polygon.radius!=0)return "box_bridge_requires_sharp_rectangle";
    hx=hy=0;
    for(int i=0;i<4;++i)
    {
        auto p=polygon.vertices[i];
        if(!std::isfinite(p.x)||!std::isfinite(p.y))return "box_bridge_invalid_rectangle";
        hx=std::max(hx,std::abs(p.x));hy=std::max(hy,std::abs(p.y));
    }
    if(!(hx>0 && hy>0))return "box_bridge_invalid_rectangle";
    unsigned mask=0;
    const float tolerance=8*std::numeric_limits<float>::epsilon()*std::max(hx,hy);
    for(int i=0;i<4;++i)
    {
        auto p=polygon.vertices[i];
        if(std::abs(std::abs(p.x)-hx)>tolerance || std::abs(std::abs(p.y)-hy)>tolerance)
            return "box_bridge_requires_centered_rectangle";
        mask|=1u<<((p.x>0?1:0)+(p.y>0?2:0));
    }
    return mask==15 ? nullptr : "box_bridge_invalid_rectangle";
}
struct Contact { int edge;float fraction;b2Pos point;b2Vec2 normal;float depth; };
struct Box
{
    Core& world;Core::B& b;b2BodyId rigid;float hx,hy,sign;unsigned long& contacts;
    float separation(b2Vec2 p,int face) const
    {return p.x*normals[face].x+p.y*normals[face].y-(face<2?hx:hy);}
    b2Vec2 local(int node) const
    {auto p=b.x[node];return b2Body_GetLocalPoint(rigid,{p[0],sign*p[1]});}
    b2Pos corner(int i) const
    {return b2Body_GetWorldPoint(rigid,{i&1?hx:-hx,i&2?hy:-hy});}
    Contact nodeContact(int node,int face,float depth) const
    {
        auto p=b.x[node];
        return {node,0,{p[0],sign*p[1]},b2Body_GetWorldVector(rigid,normals[face]),depth};
    }
    void resolve(const Contact& c)
    {
        ++contacts;
        int edge=c.edge,next=(edge+1)%16;float weights[2]={1-c.fraction,c.fraction};
        auto center=b2Body_GetWorldCenter(rigid);
        float lever=(c.point.x-center.x)*c.normal.y-(c.point.y-center.y)*c.normal.x;
        float softInverse=weights[0]*weights[0]/b.mass[edge]+weights[1]*weights[1]/b.mass[next];
        bool dynamic=b2Body_GetType(rigid)==b2_dynamicBody;
        float im=dynamic?1/b2Body_GetMass(rigid):0,ii=dynamic?1/b2Body_GetRotationalInertia(rigid):0;
        auto rv=b2Body_GetLocalPointVelocity(rigid,b2Body_GetLocalPoint(rigid,c.point));
        float vx=weights[0]*b.v[edge][0]+weights[1]*b.v[next][0];
        float vy=weights[0]*b.v[edge][1]+weights[1]*b.v[next][1];
        float speed=(vx-rv.x)*c.normal.x+(sign*vy-rv.y)*c.normal.y;
        if(speed<0)
        {
            float impulse=-speed/(softInverse+im+lever*lever*ii);
            for(int n=0;n<2;++n)
            {
                int node=n==0?edge:next;
                b.v[node][0]+=weights[n]*impulse*c.normal.x/b.mass[node];
                b.v[node][1]+=sign*weights[n]*impulse*c.normal.y/b.mass[node];
            }
            if(dynamic)b2Body_ApplyLinearImpulse(rigid,{-impulse*c.normal.x,-impulse*c.normal.y},c.point,true);
        }
        // Mode1: split position correction, no depth/dt launch or rigid teleport.
        float lambda=c.depth/softInverse;
        for(int n=0;n<2;++n)
        {
            int node=n==0?edge:next;
            b.x[node][0]+=weights[n]*lambda*c.normal.x/b.mass[node];
            b.x[node][1]+=sign*weights[n]*lambda*c.normal.y/b.mass[node];
        }
    }
    bool clip(b2Vec2 a,b2Vec2 b)
    {
        float enter=0,leave=1;
        for(int face=0;face<4;++face)
        {
            float start=separation(a,face),finish=separation(b,face);
            if(start>=0 && finish>=0)return false;
            if(start>0)enter=std::max(enter,start/(start-finish));
            if(finish>0)leave=std::min(leave,start/(start-finish));
        }
        return leave>enter;
    }
    // A segment entering and leaving adjacent faces encloses a box corner, not
    // a full wall crossing. Use that local feature instead of a whole-body face.
    bool cornerCrossing(b2Vec2 a,b2Vec2 b,b2Vec2& vertex)
    {
        float enter=0,leave=1;int entry=-1,exit=-1;
        for(int face=0;face<4;++face)
        {
            float start=separation(a,face),finish=separation(b,face);
            if(start>=0 && finish>=0)return false;
            if(start>0){float t=start/(start-finish);if(t>enter){enter=t;entry=face;}}
            if(finish>0){float t=start/(start-finish);if(t<leave){leave=t;exit=face;}}
        }
        if(leave<=enter || entry<0 || exit<0 || entry/2==exit/2)return false;
        int x=entry<2?entry:exit,y=entry>=2?entry:exit;
        vertex={normals[x].x*hx,normals[y].y*hy};return true;
    }
    // Exact maximum of minimum affine face depths along each material segment.
    float segmentDepth(b2Vec2 a,b2Vec2 b)
    {
        float start[4],slope[4];
        for(int i=0;i<4;++i){start[i]=-separation(a,i);slope[i]=-separation(b,i)-start[i];}
        auto depth=[&](float t){float d=1e30f;for(int i=0;i<4;++i)d=std::min(d,start[i]+t*slope[i]);return d;};
        float result=std::max(0.f,std::max(depth(0),depth(1)));
        for(int i=0;i<4;++i)for(int j=i+1;j<4;++j)
        {
            float denominator=slope[i]-slope[j];if(std::abs(denominator)<1e-12f)continue;
            float t=(start[j]-start[i])/denominator;
            if(t>0 && t<1)result=std::max(result,depth(t));
        }
        return result;
    }

    void project()
    {
        // Nearest face for nodes, then box vertices inside the soft contour.
        for(int node=0;node<16;++node)
        {
            auto point=local(node);float closest=-1e30f;int face=0;
            for(int i=0;i<4;++i){float d=separation(point,i);if(d>closest){closest=d;face=i;}}
            if(closest<0)resolve(nodeContact(node,face,-closest));
        }
        for(int i=0;i<4;++i)
        {
            auto vertex=corner(i);Core::Vec v{float(vertex.x),sign*float(vertex.y)};
            if(!SoftBody::inside<float>(v,b))continue;
            auto hit=SoftBody::nearest<float>(v,b);
            if(hit.edge<0){world.failure="degenerate_contact_edge";return;}
            if(hit.depth<1e-8f)continue;
            auto a=b.x[hit.edge],d=b.x[(hit.edge+1)%16];
            b2Pos point={(1-hit.t)*a[0]+hit.t*d[0],sign*((1-hit.t)*a[1]+hit.t*d[1])};
            resolve({hit.edge,hit.t,point,{-hit.normal[0],-sign*hit.normal[1]},hit.depth});
        }
        // C1's whole-body separating-face fallback prevents an edge straddling a
        // thin solid from keeping its endpoints on opposite sides indefinitely.
        int face=0;float best=-1e30f;
        for(int i=0;i<4;++i)
        {
            float minimum=1e30f;
            for(int node=0;node<16;++node)minimum=std::min(minimum,separation(local(node),i));
            if(minimum>best){best=minimum;face=i;}
        }
        b2Vec2 positions[16];for(int i=0;i<16;++i)positions[i]=local(i);
        for(int edge=0;edge<16;++edge)
        {
            if(!clip(positions[edge],positions[(edge+1)%16]))continue;
            {
                // Recompute using the current edge: earlier constraints may move it.
                auto a=local(edge),d=local((edge+1)%16);b2Vec2 vertex;
                if(cornerCrossing(a,d,vertex))
                {
                    auto worldVertex=b2Body_GetWorldPoint(rigid,vertex);
                    Core::Vec corner{float(worldVertex.x),sign*float(worldVertex.y)};
                    // If the corner is outside the soft solid, this is not a local
                    // corner exclusion constraint (e.g. initial enclosure).
                    if(SoftBody::inside<float>(corner,b))
                    {
                        float dx=d.x-a.x,dy=d.y-a.y,length2=dx*dx+dy*dy;
                        if(length2<1e-12f){world.failure="degenerate_contact_edge";return;}
                        float fraction=std::max(0.f,std::min(1.f,((vertex.x-a.x)*dx+(vertex.y-a.y)*dy)/length2));
                        b2Vec2 closest={a.x+fraction*dx,a.y+fraction*dy};
                        float nx=vertex.x-closest.x,ny=vertex.y-closest.y,depth=std::hypot(nx,ny);
                        if(depth>1e-8f)
                            resolve({edge,fraction,b2Body_GetWorldPoint(rigid,closest),
                                b2Body_GetWorldVector(rigid,{nx/depth,ny/depth}),depth});
                        continue;
                    }
                }
            }
            for(int node:{edge,(edge+1)%16})
            {
                float d=separation(local(node),face);
                if(d<0)resolve(nodeContact(node,face,-d));
            }
        }
    }
    double penetration()
    {
        double result=0;
        for(int i=0;i<16;++i)result=std::max(result,double(segmentDepth(local(i),local((i+1)%16))));
        for(int i=0;i<4;++i)
        {
            auto p=corner(i);Core::Vec v{float(p.x),sign*float(p.y)};
            if(inside<float>(v,b))
            {
                auto hit=nearest<float>(v,b);
                if(hit.edge<0)return std::numeric_limits<double>::infinity();
                result=std::max(result,double(hit.depth));
            }
        }
        return result;
    }
};
static void projectBoxes(Core& world,const void* context)
{static_cast<const Box2DBoxContacts*>(context)->project(world);}
}

bool Box2DBoxContacts::enabled(const Shape& entry,int id) const
{
    if(!b2Shape_IsValid(entry.id))return false;
    auto body=b2Shape_GetBody(entry.id);
    if(!b2Body_IsEnabled(body) || b2Shape_IsSensor(entry.id))return false;
    if(std::find(entry.excludedBodies.begin(),entry.excludedBodies.end(),id)!=entry.excludedBodies.end())return false;
    auto a=id<int(filters.size())?filters[id]:b2DefaultFilter(),bfilter=b2Shape_GetFilter(entry.id);
    if(a.groupIndex!=0 && a.groupIndex==bfilter.groupIndex)return a.groupIndex>0;
    return (a.maskBits&bfilter.categoryBits)!=0 && (bfilter.maskBits&a.categoryBits)!=0;
}
const char* Box2DBoxContacts::validate(const Core& world) const
{
    for(const auto& b:world.bodies)if(b.active)
    {
        if(b.kind!=3 || b.friction!=0)return "box_bridge_requires_frictionless_recoverable_body";
        for(int i=0;i<16;++i)
        {
            if(!(b.mass[i]>0) || !std::isfinite(b.mass[i]))return "box_bridge_invalid_soft_mass";
            for(int k=0;k<2;++k)if(!std::isfinite(b.x[i][k])||!std::isfinite(b.v[i][k]))return "box_bridge_nonfinite_soft_state";
        }
    }
    b2WorldId owner=b2_nullWorldId;
    for(size_t i=0;i<shapes.size();++i)
    {
        auto id=shapes[i].id;
        if(!b2Shape_IsValid(id))continue; // Deleted bodies/worlds are inert attachments.
        auto worldId=b2Shape_GetWorld(id);
        if(B2_IS_NON_NULL(owner) && (owner.index1!=worldId.index1 || owner.generation!=worldId.generation))return "box_bridge_mixed_worlds";
        owner=worldId;
        if(b2World_IsLocked(owner))return "box_bridge_world_locked";
        auto body=b2Shape_GetBody(id);
        for(size_t j=0;j<i;++j)if(b2Shape_IsValid(shapes[j].id) && B2_ID_EQUALS(b2Shape_GetBody(shapes[j].id),body))return "box_bridge_duplicate_body";
        if(!b2Body_IsEnabled(body) || b2Shape_IsSensor(id))continue;
        if(b2Body_GetShapeCount(body)!=1)return "box_bridge_requires_single_shape";
        float hx,hy;const char* error=dimensions(id,hx,hy);if(error)return error;
        if(b2Shape_GetFriction(id)!=0 || b2Shape_GetRestitution(id)!=0)return "box_bridge_requires_zero_friction_and_bounce";
        if(b2Body_GetType(body)==b2_dynamicBody)
        {
            auto locks=b2Body_GetMotionLocks(body);
            if(locks.linearX || locks.linearY || locks.angularZ)return "box_bridge_motion_locks_unsupported";
            float m=b2Body_GetMass(body),inertia=b2Body_GetRotationalInertia(body);
            if(!(m>0 && inertia>0) || !std::isfinite(m) || !std::isfinite(inertia))return "box_bridge_invalid_rigid_mass";
        }
    }
    for(const auto& pin:pins)if(pin.active)
    {
        if(pin.softBody<0 || pin.softBody>=int(world.bodies.size()))return "rigid_pin_invalid_soft_body";
        if(!pinActive(world,pin))continue;
        if(b2Body_GetType(pin.rigid)!=b2_dynamicBody)return "rigid_pin_requires_dynamic_body";
        bool attached=false;
        for(const auto& shape:shapes)if(b2Shape_IsValid(shape.id) && B2_ID_EQUALS(b2Shape_GetBody(shape.id),pin.rigid))attached=true;
        if(!attached)return "rigid_pin_requires_attachment";
        for(const auto& shape:shapes)if(b2Shape_IsValid(shape.id) && B2_ID_EQUALS(b2Shape_GetBody(shape.id),pin.rigid) && b2Shape_IsSensor(shape.id))
            return "rigid_pin_sensor_unsupported";
        if(!(pin.frequency>0 && pin.frequency<=60 && pin.dampingRatio>=0 && pin.dampingRatio<=10))return "rigid_pin_invalid_spring";
        if(!std::isfinite(pin.localAnchor.x)||!std::isfinite(pin.localAnchor.y))return "rigid_pin_invalid_anchor";
        double sum=0;
        for(float weight:pin.weights)
        {if(!(weight>=0 && weight<=1))return "rigid_pin_invalid_weights";sum+=weight;}
        if(std::abs(sum-1)>1e-6)return "rigid_pin_invalid_weights";
    }
    return nullptr;
}
bool Box2DBoxContacts::step(Core& world) const
{
    if(world.failure)return false;
    if(world.substeps!=1){world.failure="box_bridge_requires_single_soft_step";return false;}
    if(const char* error=validate(world)){world.failure=error;return false;}
    for(const auto& pin:pins)pin.impulse={0,0};
    world.step(projectBoxes,this,[](const Core& target,const void* context)
        {return static_cast<const Box2DBoxContacts*>(context)->penetration(target);});
    return !world.failure;
}
void Box2DBoxContacts::project(Core& world) const
{
    if(world.failure)return;
    if(const char* error=validate(world)){world.failure=error;return;}
    for(size_t i=0;i<world.bodies.size();++i)if(world.bodies[i].active)
        for(const auto& entry:shapes)if(enabled(entry,int(i)))
        {
            float hx,hy;dimensions(entry.id,hx,hy);
            Box box{world,world.bodies[i],b2Shape_GetBody(entry.id),hx,hy,flipY?-1.f:1.f,contacts};
            box.project();if(world.failure)return;
        }
    solvePins(world);
}
bool Box2DBoxContacts::pinActive(const Core& world,const Pin& pin) const
{
    return pin.active && pin.softBody>=0 && pin.softBody<int(world.bodies.size()) &&
        world.bodies[pin.softBody].active && b2Body_IsValid(pin.rigid) && b2Body_IsEnabled(pin.rigid);
}
double Box2DBoxContacts::pinError(const Core& world,const Pin& pin) const
{
    if(!pinActive(world,pin))return 0;
    auto anchor=world.anchor(pin.softBody,pin.weights);
    auto target=b2Body_GetWorldPoint(pin.rigid,pin.localAnchor);
    return std::hypot(double(anchor[0])-target.x,double(flipY?-anchor[1]:anchor[1])-target.y);
}
void Box2DBoxContacts::solvePins(Core& world) const
{
    for(const auto& pin:pins)if(pinActive(world,pin))
    {
        auto& body=world.bodies[pin.softBody];float sign=flipY?-1.f:1.f;
        auto anchor=world.anchor(pin.softBody,pin.weights);
        auto target=b2Body_GetWorldPoint(pin.rigid,pin.localAnchor);
        auto center=b2Body_GetWorldCenter(pin.rigid);
        double rx=target.x-center.x,ry=target.y-center.y;
        double inverseSoft=0,vx=0,vy=0;
        for(int i=0;i<16;++i)
        {
            double weight=pin.weights[i];inverseSoft+=weight*weight/body.mass[i];
            vx+=weight*body.v[i][0];vy+=weight*sign*body.v[i][1];
        }
        double im=1.0/b2Body_GetMass(pin.rigid),ii=1.0/b2Body_GetRotationalInertia(pin.rigid);
        // K = (inverse soft anchor mass + inverse rigid mass) I + I^-1 perp(r) perp(r)^T.
        double a=inverseSoft+im+ii*ry*ry,b=-ii*rx*ry,d=inverseSoft+im+ii*rx*rx;
        double determinant=a*d-b*b;
        if(!(determinant>0) || !std::isfinite(determinant)){world.failure="rigid_pin_invalid_effective_mass";return;}
        auto rigidVelocity=b2Body_GetLocalPointVelocity(pin.rigid,pin.localAnchor);
        double omega=2*3.14159265358979323846*pin.frequency;
        double denominator=2*pin.dampingRatio+world.dt*omega;
        double spring=world.dt*omega*denominator;
        double massScale=spring/(1+spring),impulseScale=1/(1+spring),bias=omega/denominator;
        double ex=double(anchor[0])-target.x,ey=double(sign*anchor[1])-target.y;
        double cx=vx-rigidVelocity.x+bias*ex,cy=vy-rigidVelocity.y+bias*ey;
        b2Vec2 impulse={float(-massScale*(d*cx-b*cy)/determinant-impulseScale*pin.impulse.x),
                        float(-massScale*(a*cy-b*cx)/determinant-impulseScale*pin.impulse.y)};
        if(!std::isfinite(impulse.x)||!std::isfinite(impulse.y)){world.failure="rigid_pin_nonfinite_impulse";return;}
        pin.impulse.x+=impulse.x;pin.impulse.y+=impulse.y;
        for(int i=0;i<16;++i)
        {
            body.v[i][0]+=pin.weights[i]*impulse.x/body.mass[i];
            body.v[i][1]+=sign*pin.weights[i]*impulse.y/body.mass[i];
        }
        b2Body_ApplyLinearImpulse(pin.rigid,{-impulse.x,-impulse.y},target,true);
    }
}
double Box2DBoxContacts::penetration(const Core& world) const
{
    if(validate(world))return std::numeric_limits<double>::infinity();
    double result=0;
    // Geometry helpers share the project view but this path does not mutate it.
    auto& view=const_cast<Core&>(world);
    for(size_t i=0;i<world.bodies.size();++i)if(world.bodies[i].active)
        for(const auto& entry:shapes)if(enabled(entry,int(i)))
        {
            float hx,hy;dimensions(entry.id,hx,hy);
            Box box{view,view.bodies[i],b2Shape_GetBody(entry.id),hx,hy,flipY?-1.f:1.f,contacts};
            result=std::max(result,box.penetration());
        }
    return result;
}
}
