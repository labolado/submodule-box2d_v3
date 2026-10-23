#pragma once
#include "soft_world.h"
#include "box2d/box2d.h"

namespace SoftBody
{
// Restricted experimental bridge: explicitly registered centred, sharp boxes.
// Caller steps Box2D first. This class never steps or owns the rigid world.
// No friction, restitution, CCD, motion locks, compound fixtures or Lua callbacks.
class Box2DBoxContacts
{
public:
    using Core=World<float>;
    struct Shape { b2ShapeId id;std::vector<int> excludedBodies; };
    std::vector<Shape> shapes;
    std::vector<b2Filter> filters;
    bool flipY=false;
    mutable unsigned long contacts=0;
    const char* validate(const Core& world) const;
    bool step(Core& world) const;
    void project(Core& world) const;
    double penetration(const Core& world) const;
private:
    bool enabled(const Shape& shape,int body) const;
};
}
