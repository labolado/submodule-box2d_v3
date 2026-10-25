#pragma once
#include "soft_game_world.h"

namespace SoftBody { namespace Game
{
// Bounded assembly owner. Existing Solver/World layouts and standalone paths
// stay unchanged. Parts retain their contact caches; material state is synchronized
// with three fixed single-body workspaces at preflight and completion. Only
// this owner advances time; a workspace never steps the native world.
struct Assembly
{
    static constexpr int PartCount = 3;
    static constexpr int LinkCount = 2;
    struct Part
    {
        Core core;
        Solver contact;
        Part() : contact(core) { core.dt = Step; core.substeps = 1; }
    };
    struct Link
    {
        b2Vec2 impulse{}, bias{};
        float inverseMass = 0, gamma = 0;
        float frequency = 12, dampingRatio = 1;
    };
    Core& soft;
    std::array<Part, PartCount> parts;
    std::array<Link, LinkCount> links;
    b2WorldId world = b2_nullWorldId;
    b2WorldId boundWorld = b2_nullWorldId;
    bool flipY = false, boundFlipY = false;
    const char* error = nullptr;
    unsigned long long entered = 0, completed = 0;
    int nextPhase = 0;
    double peakGap = 0, peakSpeed = 0, peakDepth = 0;

    explicit Assembly(Core& core) : soft(core) {}
    Assembly(const Assembly&) = delete;
    Assembly& operator=(const Assembly&) = delete;
    bool prepare();
    bool advance();
    void motors();
    void prepareLinks();
    void solveLinks();
    static void stage(void* step, int phase, void* context);
};
}}
