#pragma once
// Private restricted game solver. No Box2D public API or existing solver changes.
#include "soft_box2d_box.h"
#include "soft_game_geometry.h"
#include <array>
#include <algorithm>
#include <cmath>

namespace SoftBody { namespace Game
{
using Core = SoftBody::World<float>;
using Bridge = SoftBody::Box2DBoxContacts;
using View = Bridge::StateView;
constexpr float Step = 1.f / 480;
constexpr float Margin = .001f; // 0.16 content units at scale 160.
constexpr int Sweeps = 8;
constexpr int MaxBoxes = 4;
constexpr int MaxRows = MaxBoxes * 20;

struct Box
{
    b2ShapeId shape = b2_nullShapeId;
    b2ShapeId cachedShape = b2_nullShapeId;
    b2BodyId body = b2_nullBodyId;
    View state{};
    int index = -1;
    float hx = 0, hy = 0, friction = 0;
    b2Filter filter = b2DefaultFilter();
    bool enabled = false;
    bool stateCached = false;
    b2WorldTransform cachedTransform{};
    b2BodyType cachedType = b2_staticBody;
    float cachedMass = 0, cachedInertia = 0;
    b2Vec2 cachedCenter{};
};

struct Row
{
    int box = 0, edge = 0, feature = 0;
    float fraction = 0, normalImpulse = 0, tangentImpulse = 0;
    b2Vec2 normal{}, localAnchor{};
};

struct Solver
{
    Core& soft;
    bool flipY = false;
    unsigned long long entered = 0;
    b2WorldId boundWorld = b2_nullWorldId;
    bool boundFlipY = false;
    Bridge::Pin cachedPin;
    b2Vec2 impulse{};
    b2WorldId world = b2_nullWorldId;
    Bridge::Pin pin;
    b2Filter filter = b2DefaultFilter();
    Recovery recovery;
    void* activeStep = nullptr; // Valid only during this call's phase 2.
    double pinInverseXX = 0, pinInverseXY = 0, pinInverseYY = 0;
    std::array<Box, MaxBoxes> boxes{};
    int boxCount = 0;
    float friction = .8f;
    std::array<Row, MaxRows> rows{}, previous{};
    int rowCount = 0, previousCount = 0;
    unsigned long long steps = 0, rowSolves = 0, pinSolves = 0, warmHits = 0, cacheResets = 0;
    std::array<unsigned long long, MaxBoxes> contactSteps{};
    unsigned long geometryCorrections = 0;
    double maxDepth = 0, lastDepth = 0, maxGap = 0, normalTotal = 0, tangentTotal = 0;
    unsigned long long overlapSteps = 0;
    double contactMomentumResidual = 0;
    const char* error = nullptr;
    int nextPhase = 0;

    explicit Solver(Core& core) : soft(core) { pin.active = cachedPin.active = false; }
    Solver(const Solver&) = delete;
    Solver& operator=(const Solver&) = delete;

    // The adapter owns shape registration and native-coordinate pin anchors.
    // Views below always use the soft world's metre/up-positive convention.
    bool read(void* step, b2BodyId id, View& state, int& index) const;
    void write(void* step, int index, View state) const;
    void reflect(View& state) const;
    b2Vec2 localPinAnchor() const;
    bool validate();

    bool add(b2ShapeId shape);

    void clearCache();

    bool enabled(b2Filter other) const;

    bool prepare();

    bool prepareTargets();

    b2Vec2 point(const Row& row) const;

    float softInverse(const Row& row) const;

    void append(Row row);

    void gather();

    bool projectGeometry();

    bool observeRecovery(bool endOfStep = false);

    double anchorCoupling(const Row& row) const;

    b2Vec2 compensatingImpulse(const Row& row, b2Vec2 impulse) const;

    bool preparePinResponse(void* step);

    float inverseMass(const Row& row, b2Vec2 direction) const;

    float relativeSpeed(const Row& row, b2Vec2 direction) const;

    void apply(const Row& row, b2Vec2 impulse);

    void solve(Row& row);

    bool attachment(void* step, bool bias);

    static void stage(void* step, int phase, void* context);

    bool advance();
};
}}
