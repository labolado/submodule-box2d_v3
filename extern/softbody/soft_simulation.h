#pragma once
// Native operation boundary. Values are metres/seconds/kg, y up. This header
// exposes definitions and snapshots, never solver storage, Lua or Box2D types.
#include <array>
#include <cstdint>
#include <vector>

namespace SoftBody
{
using Point = std::array<float, 2>;
using Weights = std::vector<float>;
enum class ObjectKind
{
	None,
	Body,
	Joint,
	Touch,
	Attachment
};
struct ObjectId
{
	std::uint64_t owner = 0, generation = 0;
	int index = -1;
	ObjectKind kind = ObjectKind::None;
};
struct SimulationDef
{
	int maxBodies = 32, maxJoints = 64, maxTouches = 1;
	// Runtime budgets, not per-shape storage limits. Removed bodies release nodes.
	int maxNodes = 512, maxSoftContactRows = 512;
	float timeStep = 1.f / 480;
	int iterations = 8;
	Point gravity{ { 0, -20 } };
	bool enableSoftContacts = false;
};
enum class BodySupport
{
	Free,
	World
};
struct SoftShapeDef
{
	// Local metres, y up. Empty nodes retain the existing ellipse preset.
	// At least 3 nodes, one simple contour using every node exactly once.
	// Creation must fit the simulation's remaining maxNodes budget.
	// Empty boundary means input order. Indices are zero based in the native API.
	// Construction copies the definition; changing it never edits a live body.
	std::vector<Point> nodes;
	std::vector<int> boundary;
};
struct SoftBodyDef
{
	SoftShapeDef shape;
	// Total kg, distributed equally across the active nodes. This is not an
	// areal-density material; changing sampling changes the mass distribution.
	// halfWidth/halfHeight apply only to the omitted-shape ellipse preset.
	float mass = 1, halfWidth = .35f, halfHeight = .35f;
	Point position{};
	float frequency = 6, damping = 40, materialScale = 1, friction = 0;
	// World support fixes the elastic reference, not the nodes or the centre.
	// Its reaction is external; mass and local contact response stay finite.
	BodySupport support = BodySupport::Free;
};
struct SoftJointDef
{
	ObjectId bodyA, bodyB;
	// Exactly one weight per input node. Defaults suit the omitted-shape ellipse.
	Weights weightsA = Weights( 16 ), weightsB = Weights( 16 );
	float frequency = 12, dampingRatio = 1, motorGain = 8;
};
struct SoftTouchDef
{
	ObjectId body;
	Weights weights = Weights( 16 );
	Point target{};
	float frequency = 3, dampingRatio = 1, maxForce = 50;
};
// A compliant positional pin to an immovable world point. Rotation stays free;
// its optional motor drives the body's angular velocity relative to the world.
struct SoftWorldJointDef
{
	ObjectId body;
	Weights weights = Weights( 16 );
	Point anchor{};
	float frequency = 12, dampingRatio = 1, motorGain = 8;
};
struct SoftBodyState
{
	std::vector<Point> vertices, velocities;
	int nodeCount = 0;
	// Normalized CCW contour indexing vertices in the original input order.
	std::vector<int> boundary;
	BodySupport support = BodySupport::Free;
	Point referencePosition{}; // Creation origin; a fixed target only for World.
};
struct SoftTouchState
{
	Point position{}, target{};
	float force = 0;
};
struct SimulationState
{
	unsigned long long completedSteps = 0;
	double energy = 0, motorWork = 0, appliedWork = 0;
	double penetration = 0, pinError = 0, nodeSpeed = 0, centerSpeed = 0, minAreaRatio = 1;
	bool selfCross = false;
	const char* failure = nullptr;
	std::vector<float> vertices;
};
class Box2DCoupling;
class Simulation
{
public:
	explicit Simulation( const SimulationDef& def = SimulationDef() );
	~Simulation();
	Simulation( const Simulation& ) = delete;
	Simulation& operator=( const Simulation& ) = delete;
	bool ready() const;
	const char* lastError() const;
	bool isValid( ObjectId id ) const;
	int availableNodes() const;
	int getBodyNodeCount( ObjectId body ) const; // Zero for an invalid handle.
	ObjectId createBody( const SoftBodyDef& def );
	ObjectId createJoint( const SoftJointDef& def );
	ObjectId createWorldJoint( const SoftWorldJointDef& def );
	ObjectId createTouch( const SoftTouchDef& def );
	bool remove( ObjectId id );
	bool setMotor( ObjectId joint, float speed, float maxTorque );
	bool setTouchTarget( ObjectId touch, Point target );
	// Total impulse in kg*m/s, distributed by node mass. Translation only:
	// shape, position and angular momentum about the centre are unchanged.
	bool applyLinearImpulse( ObjectId body, Point impulse );
	bool setCollisionEnabled( ObjectId a, ObjectId b, bool enabled );
	bool getBodyState( ObjectId body, SoftBodyState& state ) const;
	bool getTouchState( ObjectId touch, SoftTouchState& state ) const;
	SimulationState getState() const;

private:
	struct Impl;
	Impl* impl_;
	friend class Box2DCoupling;
};
} // namespace SoftBody
