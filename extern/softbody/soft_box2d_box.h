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
    struct Pin
    {
        int softBody=-1;
        b2BodyId rigid=b2_nullBodyId;
        std::array<float,16> weights{};
        b2Vec2 localAnchor{}; // Box2D local coordinates, metres.
        float frequency=30,dampingRatio=1;
        bool active=true;
        mutable b2Vec2 impulse{}; // Reset each complete step; no warm start.
    };
    // Private experimental phase-local contact view. No public body API calls.
    struct StateView
    {
        b2Pos center; b2Rot rotation; b2Vec2 velocity; float angularVelocity;
        float inverseMass,inverseInertia;
        b2Vec2 localCenter{};
        b2Pos origin{}; // Geometry origin, distinct from the mass centre.
        StateView()=default;
        StateView(b2Pos c,b2Rot q,b2Vec2 v,float w,float im,float ii,b2Vec2 local={})
            :center(c),rotation(q),velocity(v),angularVelocity(w),inverseMass(im),inverseInertia(ii),localCenter(local),
             origin(b2OffsetPos(c,b2Neg(b2RotateVector(q,local)))){}

    };
    static const char* validateStepBody(b2WorldId world,b2BodyId body);
    static bool readStepBody(void* step,b2BodyId body,StateView& state,int& index);
    static void writeStepBody(void* step,int index,const StateView& state);
    static void projectState(Core& world,int body,StateView& state,float hx,float hy,unsigned long& contacts,bool flipY=false);
    static double statePenetration(Core& world,int body,StateView& state,float hx,float hy,bool flipY=false);
    // Private single-step owner. Preflight failures do not advance either world.
    // A failure after entering the rigid solver is non-transactional and poisons
    // this owner; callers must discard the pair instead of silently resuming.
    struct StepGuard
    {
        const char* error=nullptr;
        bool faulted=false;
        unsigned long long completed=0,entered=0;
        using Stage=bool (*)(void* step,int phase,void* user);
        bool advance(b2WorldId world,Core& soft,const b2BodyId* targets,int count,Stage stage,void* user);
    };
    // Restricted native owner: one soft body, one attachment, optional box contact.
    // Owns neither world. Configuration is explicit; no user stage callbacks.
    struct AttachmentOwner
    {
        Pin attachment;
        bool flipY=false; // Soft coordinates stay unchanged; rigid y is reflected.
        b2ShapeId contact=b2_nullShapeId;
        b2Filter filter=b2DefaultFilter();
        StepGuard guard;
        b2Vec2 impulse{};
        unsigned long contacts=0;
        double angularResidual=0,preDepth=0,postDepth=0;
        bool advance(b2WorldId world,Core& soft);
    private:
        b2WorldId boundWorld=b2_nullWorldId;
        Core* boundSoft=nullptr;
        bool boundFlipY=false;
        struct Context;
        static bool stage(void* step,int phase,void* user);
    };
    std::vector<Pin> pins;
    bool pinActive(const Core& world,const Pin& pin) const;
    double pinError(const Core& world,const Pin& pin) const;
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
    void solvePins(Core& world) const;
};
}
