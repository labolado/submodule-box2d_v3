#pragma once
// C1 native prototype: explicit static shape registration, public Box2D API only.
// Coordinates are metres in the same frame as Box2D. No Lua, scheduler or body writes.
#include "sb_world.h"
#include "box2d/box2d.h"

namespace SoftBody
{
class Box2DStaticContacts
{
public:
    using Core = World<float>;
    using Vec = Core::Vec;
    struct Shape
    {
        b2ShapeId id;
        std::vector<int> excludedBodies;
    };
    std::vector<Shape> shapes;
    // Missing entries use Box2D's default filter. This is separate from soft-soft filters.
    std::vector<b2Filter> filters;

    static bool allows(b2Filter a, b2Filter b)
    {
        if(a.groupIndex != 0 && a.groupIndex == b.groupIndex)return a.groupIndex > 0;
        return (a.maskBits & b.categoryBits) != 0 && (b.maskBits & a.categoryBits) != 0;
    }

    // Validate the entire registration before advancing any soft state. Unsupported
    // active geometry is an error, even when a filter currently excludes it.
    const char* validate() const
    {
        for(const auto& entry : shapes)
        {
            if(!b2Shape_IsValid(entry.id))continue; // Deleted shape/world: never dereference stale IDs.
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

    bool step(Core& world) const
    {
        if(world.failure)return false;
        if(const char* error = validate()){world.failure = error;return false;}
        world.step(*this);
        return !world.failure;
    }

    void operator()(Core& world) const
    {
        for(const auto& entry : shapes)
        {
            if(!b2Shape_IsValid(entry.id))continue;
            b2BodyId rigid = b2Shape_GetBody(entry.id);
            if(!b2Body_IsEnabled(rigid) || b2Shape_IsSensor(entry.id))continue;
            b2Filter rigidFilter = b2Shape_GetFilter(entry.id);
            for(size_t id = 0; id < world.bodies.size(); ++id)
            {
                auto& soft = world.bodies[id];
                if(!soft.active)continue;
                if(std::find(entry.excludedBodies.begin(),entry.excludedBodies.end(),int(id)) != entry.excludedBodies.end())continue;
                b2Filter filter = id < filters.size() ? filters[id] : b2DefaultFilter();
                if(!allows(filter,rigidFilter))continue;
                float friction = std::sqrt(soft.friction * b2Shape_GetFriction(entry.id));
                if(b2Shape_GetType(entry.id) == b2_circleShape)
                {
                    b2Circle circle = b2Shape_GetCircle(entry.id);
                    b2Pos center = b2Body_GetWorldPoint(rigid,circle.center);
                    projectCircle(world,int(id),{float(center.x),float(center.y)},circle.radius,friction);
                }
                else
                {
                    b2Polygon polygon = b2Shape_GetPolygon(entry.id);
                    std::array<Vec,B2_MAX_POLYGON_VERTICES> points;
                    for(int i = 0; i < polygon.count; ++i)
                    {
                        b2Pos p = b2Body_GetWorldPoint(rigid,polygon.vertices[i]);
                        points[i] = {float(p.x),float(p.y)};
                    }
                    projectPolygon(world,int(id),points,polygon.count,friction);
                }
                if(world.failure)return;
            }
        }
    }

private:
    // Weighted edge correction; no rigid reaction in the static-only phase.
    static void edgeContact(Core& world,int id,int edge,float fraction,Vec normal,float depth,float friction)
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

    static void projectCircle(Core& world,int id,Vec center,float radius,float friction)
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

    static void projectPolygon(Core& world,int id,
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
    }
};
}
