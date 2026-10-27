#pragma once
// Box2D-specific boundary. Only this adapter owns registrations and stage state.
#include "soft_simulation.h"

#include "box2d/id.h"

namespace SoftBody
{
struct RigidFilter
{
	std::uint64_t categoryBits = 1, maskBits = ~std::uint64_t( 0 );
	int groupIndex = 0;
};
struct Box2DCouplingDef
{
	b2WorldId world = b2_nullWorldId;
	bool flipY = false, profile = false;
	// Restricted game experiment: awake dynamic contacts; polygon compounds share one rigid state.
	bool enableDynamicContacts = false;
	int maxAttachments = 64;
	int maxRigidJoints = 64;
	int maxChainSegments = 256; // Nearby candidates per soft body, not total terrain length.
};
// Positional spring with optional relative-speed motor. Local anchors use soft metres/y-up, even
// when flipY is enabled. Connections never implicitly change collision filtering.
struct SoftRigidJointDef
{
	ObjectId body, rigid;
	Weights weights = Weights( 16 );
	Point localAnchor{};
	float frequency = 30, dampingRatio = 1, motorGain = 8;
};
struct SoftRigidJointState
{
	bool enabled = false;
	double errorM = 0;
	Point impulse{};						// On the soft side, in the last completed step, kg*m/s.
	double relativeSpeed = 0;				// Soft minus rigid, rad/s in y-up coordinates.
	double motorImpulse = 0, motorWork = 0; // Last-step angular impulse and actuator work.
};
struct CouplingState
{
	unsigned long long entered = 0, completed = 0, contactRows = 0, warmHits = 0;
	unsigned long long recoveryEntries = 0, recoveryExits = 0, recoverySteps = 0;
	unsigned long long softContactRows = 0, softRecoverySteps = 0;
	unsigned long long softRecoveryEntries = 0, softRecoveryExits = 0;
	int softRecoveringPairs = 0;
	// Accumulated magnitude of whole-body position assistance, in metres.
	double softRecoveryDistance = 0, softContactDepth = 0;
	double contactDepth = 0, peakGap = 0;
	unsigned long long chainQueries = 0;
	int chainCandidates = 0;
	bool recovering = false;
	const char* failure = nullptr;
};
struct TimingSample
{
	double wallMs = 0, cpuMs = 0;
	long long calls = 0;
};
struct CouplingProfile
{
	enum Stage
	{
		Prepare,
		MaterialMotor,
		InitialLinks,
		ContactGeometry,
		ContactSolve,
		Observe,
		Finalize,
		NativeWorld,
		Advance,
		Adapter,
		Count
	};
	bool enabled = false, clockFailed = false;
	std::array<TimingSample, Count> samples{};
};
class Box2DCoupling
{
public:
	// The simulation and rigid world must outlive the adapter. Calls are
	// serialized on their owner thread; no Lua callbacks run in soft phases.
	Box2DCoupling( Simulation& simulation, const Box2DCouplingDef& def );
	~Box2DCoupling();
	Box2DCoupling( const Box2DCoupling& ) = delete;
	Box2DCoupling& operator=( const Box2DCoupling& ) = delete;
	bool ready() const;
	const char* lastError() const;
	ObjectId attachBody( b2BodyId body );
	bool isValid( ObjectId attachment ) const;
	bool detach( ObjectId attachment );
	ObjectId createRigidJoint( const SoftRigidJointDef& def );
	bool removeRigidJoint( ObjectId joint );
	bool setRigidMotor( ObjectId joint, float speed, float maxTorque );
	bool getRigidJointState( ObjectId joint, SoftRigidJointState& state ) const;
	bool setCollisionEnabled( ObjectId attachment, ObjectId body, bool enabled );
	bool setFilter( ObjectId body, RigidFilter filter );
	bool prepare();
	bool advance( bool* entered = nullptr ); // Exactly one complete rigid step with soft phases.
	CouplingState getState() const;
	CouplingProfile getProfile() const;
	static bool profilingSupported();

private:
	struct Impl;
	Impl* impl_;
};
} // namespace SoftBody
