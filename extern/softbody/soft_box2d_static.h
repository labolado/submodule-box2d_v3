#pragma once
// C1 native prototype: explicit static shape registration, public Box2D API only.
// Coordinates are metres in the same frame as Box2D. No Lua, scheduler or body writes.
#include "soft_world.h"
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
    bool flipY=false; // Solar2D uses down-positive coordinates; the experiment uses up.
    mutable unsigned long queryCount=0,candidateCount=0;
    double penetration(const Core& world) const;

    static bool allows(b2Filter a, b2Filter b);

    // Validate the entire registration before advancing any soft state. Unsupported
    // active geometry is an error, even when a filter currently excludes it.
    const char* validate() const;

    bool step(Core& world) const;

    void operator()(Core& world) const;

private:
    mutable std::vector<std::array<Vec,16>> previous;
    std::vector<size_t> candidates(const Core& world,int body,bool swept) const;
    bool enabled(const Shape& shape,int body) const;
    void polygonPoints(b2ShapeId shape,std::array<Vec,B2_MAX_POLYGON_VERTICES>& points,int& count) const;
    Vec circleCenter(b2ShapeId shape) const;
    // Weighted edge correction; no rigid reaction in the static-only phase.
    static void edgeContact(Core& world,int id,int edge,float fraction,Vec normal,float depth,float friction);

    static void projectCircle(Core& world,int id,Vec center,float radius,float friction);

    static void projectPolygon(Core& world,int id,
        const std::array<Vec,B2_MAX_POLYGON_VERTICES>& points,int count,float friction);
};
}
