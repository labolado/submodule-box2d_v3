#include "soft_game_assembly.h"
extern "C" {
#include "../../src/physics_world.h"
}

namespace SoftBody { namespace Game
{
#if defined(__clang__)
#pragma clang fp contract(off)
#endif
namespace
{
bool validWeights(const std::array<float, 16>& weights)
{
    double sum = 0;
    for (float weight : weights)
    {
        if (!(weight >= 0 && weight <= 1)) return false;
        sum += weight;
    }
    return std::abs(sum - 1) <= 1e-6;
}
}

bool Assembly::prepare()
{
    if (error) return false;
    if (entered && (b2StoreWorldId(world) != b2StoreWorldId(boundWorld) || flipY != boundFlipY))
    { error = "assembly_owner_changed"; return false; }
    if (soft.failure) { error = soft.failure; return false; }
    if (soft.bodies.size() != PartCount || soft.pins.size() > LinkCount || !soft.boundaries.empty() ||
        soft.dt != Step || soft.substeps != 1 || soft.iterations != Sweeps)
    { error = "unsupported_assembly_configuration"; return false; }
    for (int a = 0; a < PartCount; ++a)
        for (int b = a + 1; b < PartCount; ++b)
            if (soft.pairEnabled(a, b))
            { error = "assembly_requires_explicit_soft_collision_exclusions"; return false; }
    for (size_t id = 0; id < soft.pins.size(); ++id)
    {
        const auto& pin = soft.pins[id];
        const auto& link = links[id];
        if (!pin.active) continue;
        if (pin.a < 0 || pin.a >= PartCount || pin.b < 0 || pin.b >= PartCount || pin.a == pin.b ||
            !validWeights(pin.wa) || !validWeights(pin.wb) ||
            !(link.frequency >= .01f && link.frequency <= 60) || !(link.dampingRatio >= 0 && link.dampingRatio <= 10) ||
            !std::isfinite(pin.speed) || !(pin.maxTorque >= 0 && pin.maxTorque <= 1e6f) ||
            !(pin.gain >= 0 && pin.gain <= 1e6f))
        { error = "invalid_assembly_connection"; return false; }
    }
    int touches = 0;
    for (const auto& touch : soft.touchJoints) if (touch.active)
    {
        if (++touches > 1 || touch.body < 0 || touch.body >= PartCount || !validWeights(touch.weights) ||
            !std::isfinite(touch.target[0]) || !std::isfinite(touch.target[1]) ||
            !(touch.frequency >= .01f && touch.frequency <= 60) || !(touch.dampingRatio >= 0 && touch.dampingRatio <= 10) ||
            !(touch.maxForce >= 0 && touch.maxForce <= 1e5f))
        { error = "invalid_assembly_touch"; return false; }
    }
    for (int id = 0; id < PartCount; ++id)
    {
        auto& part = parts[id];
        if (part.core.bodies.empty()) part.core.bodies.push_back(soft.bodies[id]);
        else part.core.bodies[0] = soft.bodies[id];
        part.core.gravity = soft.gravity;
        auto& contact = part.contact;
        contact.world = world;
        contact.flipY = flipY;
        contact.friction = soft.bodies[id].friction;
        if (!contact.validate() || !contact.prepare() || !contact.observeRecovery())
        { error = contact.error; return false; }
        for (int i = 0; i < contact.boxCount; ++i)
            if (b2Body_GetType(contact.boxes[i].body) == b2_dynamicBody)
            { error = "assembly_shared_dynamic_contacts_unsupported"; return false; }
    }
    return true;
}

void Assembly::motors()
{
    double torques[PartCount] = {}, before = 0, after = 0;
    for (const auto& pin : soft.pins) if (pin.active && pin.maxTorque > 0)
    {
        double relative = parts[pin.b].core.omega(0) - parts[pin.a].core.omega(0);
        double torque = std::max(-double(pin.maxTorque), std::min(double(pin.maxTorque), double(pin.gain) * (double(pin.speed) - relative)));
        torques[pin.b] += torque;
        torques[pin.a] -= torque;
    }
    for (auto& part : parts) before += part.core.kinetic(0);
    for (int id = 0; id < PartCount; ++id) if (torques[id] != 0) parts[id].core.torque(0, torques[id]);
    for (auto& part : parts) after += part.core.kinetic(0);
    soft.motorWork += after - before;
}

void Assembly::prepareLinks()
{
    for (size_t id = 0; id < soft.pins.size(); ++id)
    {
        const auto& pin = soft.pins[id];
        if (!pin.active) continue;
        auto& link = links[id];
        auto& a = parts[pin.a].core;
        auto& b = parts[pin.b].core;
        link.inverseMass = 0;
        link.impulse = {0, 0};
        for (int i = 0; i < 16; ++i)
            link.inverseMass += pin.wa[i] * pin.wa[i] / a.bodies[0].mass[i] + pin.wb[i] * pin.wb[i] / b.bodies[0].mass[i];
        float omega = 2 * 3.14159265358979323846f * link.frequency;
        float denominator = 2 * link.dampingRatio * omega + Step * omega * omega;
        // Implicit spring compliance: gamma=1/(h*(d+h*k)), where k=m*w*w.
        // Accumulated impulses make repeated sweeps solve the same spring,
        // rather than applying the full spring force eight times.
        link.gamma = link.inverseMass / (Step * denominator);
        float biasRate = omega * omega / denominator;
        auto x = a.anchor(0, pin.wa), y = b.anchor(0, pin.wb);
        link.bias = {biasRate * (y[0] - x[0]), biasRate * (y[1] - x[1])};
    }
}

void Assembly::solveLinks()
{
    for (size_t id = 0; id < soft.pins.size(); ++id)
    {
        const auto& pin = soft.pins[id];
        if (!pin.active) continue;
        auto& link = links[id];
        auto& a = parts[pin.a].core.bodies[0];
        auto& b = parts[pin.b].core.bodies[0];
        b2Vec2 relative{};
        for (int i = 0; i < 16; ++i)
        {
            relative.x += pin.wb[i] * b.v[i][0] - pin.wa[i] * a.v[i][0];
            relative.y += pin.wb[i] * b.v[i][1] - pin.wa[i] * a.v[i][1];
        }
        auto impulse = b2MulSV(-1 / (link.inverseMass + link.gamma),
            b2Add(b2Add(relative, link.bias), b2MulSV(link.gamma, link.impulse)));
        link.impulse = b2Add(link.impulse, impulse);
        for (int i = 0; i < 16; ++i)
        {
            a.v[i][0] -= pin.wa[i] * impulse.x / a.mass[i];
            a.v[i][1] -= pin.wa[i] * impulse.y / a.mass[i];
            b.v[i][0] += pin.wb[i] * impulse.x / b.mass[i];
            b.v[i][1] += pin.wb[i] * impulse.y / b.mass[i];
        }
    }
}

void Assembly::stage(void* step, int phase, void* context)
{
    auto& owner = *static_cast<Assembly*>(context);
    if (owner.error) return;
    if (phase != owner.nextPhase++) { owner.error = "assembly_phase_order"; return; }
    if (phase == 0)
    {
        for (auto& part : owner.parts)
        {
            part.core.advanceMaterialVelocity();
            if (part.core.failure) { owner.error = part.core.failure; return; }
        }
        for (auto& touch : owner.soft.touchJoints) if (touch.active)
        {
            auto local = touch;
            local.body = 0;
            auto& core = owner.parts[touch.body].core;
            double before = core.appliedWork;
            core.solveTouchJoint(local);
            owner.soft.appliedWork += core.appliedWork - before;
            touch.lastForce = local.lastForce;
        }
        owner.motors();
        return;
    }
    if (phase == 1)
    {
        owner.prepareLinks();
        owner.solveLinks();
        return;
    }
    for (auto& part : owner.parts)
    {
        auto& contact = part.contact;
        auto& body = part.core.bodies[0];
        for (int i = 0; i < 16; ++i)
            for (int axis = 0; axis < 2; ++axis) body.x[i][axis] += Step * body.v[i][axis];
        if (!contact.observeRecovery()) { owner.error = contact.error; return; }
        for (int i = 0; i < contact.boxCount; ++i)
        {
            auto& box = contact.boxes[i];
            if (box.enabled && b2Body_GetType(box.body) != b2_staticBody && !contact.read(step, box.body, box.state, box.index))
            { owner.error = "assembly_contact_state"; return; }
        }
        if (!contact.projectGeometry()) { owner.error = contact.error; return; }
        contact.gather();
        if (contact.error) { owner.error = contact.error; return; }
        for (int i = 0; i < contact.rowCount; ++i)
        {
            auto& row = contact.rows[i];
            b2Vec2 tangent = {-row.normal.y, row.normal.x};
            contact.apply(row, b2Add(b2MulSV(row.normalImpulse, row.normal), b2MulSV(row.tangentImpulse, tangent)));
        }
    }
    for (int sweep = 0; sweep < Sweeps; ++sweep)
    {
        for (auto& part : owner.parts)
        {
            auto& contact = part.contact;
            for (int i = 0; i < contact.rowCount && !contact.error; ++i) contact.solve(contact.rows[i]);
            if (contact.error) { owner.error = contact.error; return; }
        }
        owner.solveLinks();
    }
    for (auto& part : owner.parts)
    {
        auto& contact = part.contact;
        contact.previous = contact.rows;
        contact.previousCount = contact.rowCount;
        contact.lastDepth = 0;
        for (int i = 0; i < contact.boxCount; ++i) if (contact.boxes[i].enabled)
        {
            auto& box = contact.boxes[i];
            BoxProjection geometry(part.core.bodies[0], box.state, box.hx, box.hy);
            contact.lastDepth = std::max(contact.lastDepth, geometry.boundaryDepth());
        }
    }
}

bool Assembly::advance()
{
    if (!prepare()) return false;
    for (auto& part : parts)
    {
        auto& contact = part.contact;
        contact.recovery.remaining.fill(contact.recovery.maxCorrection);
        if (contact.recovery.active) contact.clearCache();
    }
    nextPhase = 0;
    boundWorld = world; boundFlipY = flipY;
    ++entered;
    b2World_StepWithSoftStages(world, Step, 1, nullptr, nullptr, stage, this);
    bool staticOnly = true;
    for (const auto& part : parts)
        for (int i = 0; i < part.contact.boxCount; ++i)
            if (part.contact.boxes[i].enabled && b2Body_GetType(part.contact.boxes[i].body) != b2_staticBody) staticOnly = false;
    if (nextPhase == 0 && staticOnly)
        for (int phase = 0; phase < 3; ++phase) stage(nullptr, phase, this);
    if (!error && nextPhase != 3) error = "assembly_missing_stage";
    // There is no rollback after native entry. Publish partial material state
    // even on failure, and leave the owner faulted until it is rebuilt.
    for (int id = 0; id < PartCount; ++id) soft.bodies[id] = parts[id].core.bodies[0];
    if (error) return false;
    for (auto& part : parts)
    {
        auto& contact = part.contact;
        if (!contact.observeRecovery(true)) { error = contact.error; return false; }
        for (int i = 0; i < contact.boxCount; ++i)
            contact.boxes[i].cachedTransform = b2Body_GetTransform(contact.boxes[i].body);
        ++part.core.completedSubsteps;
        peakSpeed = std::max(peakSpeed, part.core.currentMetrics().nodeSpeed);
        peakDepth = std::max(peakDepth, contact.lastDepth);
    }
    soft.peak = soft.currentMetrics();
    peakGap = std::max(peakGap, soft.peak.pin);
    ++soft.completedSubsteps; ++completed;
    return true;
}
}}
