#pragma once
// Per-body rigid contact constraints; assembly owns the step and all joints.
#include "soft_box2d_state.h"
#include "soft_game_geometry.h"
#include "soft_profile.h"
#include "soft_world.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace SoftBody
{
namespace Game
{
using Core = SoftBody::World<float>;
using Bridge = SoftBody::Box2DState;
using View = Bridge::StateView;
constexpr float Step = 1.f / 480;
constexpr float Margin = .001f; // 0.16 content units at scale 160.
constexpr int Sweeps = 8;
constexpr int MaxCompoundShapes = 16; // Assembly registration budget per soft body.

struct Box
{
	b2ShapeId shape = b2_nullShapeId;
	b2ShapeId cachedShape = b2_nullShapeId;
	b2BodyId body = b2_nullBodyId;
	View state{};
	RigidGeometry geometry;
	int index = -1;
	int stateSlot = 0; // First enabled fixture of the same rigid body.
	bool compound = false;
	bool curved = false;
	std::vector<BoundarySegment> boundary; // Exposed lines/arcs, only on the state owner.
	float hx = 0, hy = 0, friction = 0;
	b2Filter filter = b2DefaultFilter();
	bool enabled = false;
	bool stateCached = false;
	std::vector<int> nodeFaces;
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
	Profile* profile =
		nullptr; // Non-owning assembly diagnostics; never used by the solver equations.
	int bodyIndex = 0;
	bool retainBoundarySides = false;
	SoftBody::Body<float>& body()
	{
		return soft.bodies[bodyIndex];
	}
	const SoftBody::Body<float>& body() const
	{
		return soft.bodies[bodyIndex];
	}
	bool flipY = false;
	b2WorldId world = b2_nullWorldId;
	b2Filter filter = b2DefaultFilter();
	Recovery recovery;
	std::vector<Box> boxes = std::vector<Box>( MaxCompoundShapes );
	int boxCount = 0;
	int maxChainSegments = 0; // Only assembly enables nearby static chain candidates.
	bool chainQueryActive = false;
	b2AABB chainQueryBounds{}; // Soft coordinates; reject motion escaping this query envelope.
	std::vector<Core::Vec> chainStart;
	bool boundaryDirty = true;
	float friction = .8f;
	std::vector<Row> rows, previous;
	std::vector<Row> chainPatches; // Two extreme barycentric contacts per coplanar edge patch.
	struct Plane
	{
		b2Vec2 normal;
		float gap;
	};
	std::vector<Plane> translationPlanes;
	std::vector<float> curveCuts; // Reused material-edge/analytic-boundary intersections.
	int rowCount = 0, previousCount = 0;
	unsigned long long rowSolves = 0, warmHits = 0, cacheResets = 0;
	std::vector<unsigned long long> contactSteps =
		std::vector<unsigned long long>( MaxCompoundShapes );
	unsigned long geometryCorrections = 0;
	double maxDepth = 0, lastDepth = 0, normalTotal = 0, tangentTotal = 0;
	unsigned long long overlapSteps = 0;
	double contactMomentumResidual = 0;
	const char* error = nullptr;

	explicit Solver( Core& core )
		: soft( core )
	{
	}
	Solver( Core& core, int index )
		: soft( core )
		, bodyIndex( index )
	{
	}
	Solver( const Solver& ) = delete;
	Solver& operator=( const Solver& ) = delete;

	// The adapter owns shape registration and native-coordinate pin anchors.
	// Views below always use the soft world's metre/up-positive convention.
	bool read( void* step, b2BodyId id, View& state, int& index ) const;
	void write( void* step, int index, View state ) const;
	void reflect( View& state ) const;
	bool validate();

	bool add( b2ShapeId shape );

	void clearCache();

	bool enabled( b2Filter other ) const;

	bool prepare();

	bool prepareTargets();
	void prepareCompounds();
	View& rigidState( int shape )
	{
		return boxes[boxes[shape].stateSlot].state;
	}
	const View& rigidState( int shape ) const
	{
		return boxes[boxes[shape].stateSlot].state;
	}
	bool insideCompound( int owner, b2Vec2 p ) const;
	bool nearestCompound( int owner, b2Vec2 p, b2Vec2& q, int& segment ) const;
	void gatherCompound( int owner );
	bool projectCompound( int owner );
	void gatherCurved( int owner );
	bool projectCurved( int owner );
	bool curvedInterior( int owner, b2Vec2 a, b2Vec2 b, float& fraction, b2Vec2& surface,
						 int& feature );
	double curvedDepth( int owner );
	void chainEdgeRows( int edge );
	void gatherChains();
	bool projectChains();
	double chainDepth( int shape ) const;
	bool withinChainQuery();

	b2Vec2 point( const Row& row ) const;

	float softInverse( const Row& row ) const;

	void append( Row row );

	void gather();

	bool projectGeometry();
	void captureBoundarySides();
	float limitCorrection( int node, Core::Vec delta );
	Core::Vec projectTranslation( Core::Vec desired );

	bool observeRecovery( bool endOfStep = false );

	float inverseMass( const Row& row, b2Vec2 direction ) const;

	float relativeSpeed( const Row& row, b2Vec2 direction ) const;

	void apply( const Row& row, b2Vec2 impulse );

	void solve( Row& row );
};
} // namespace Game
} // namespace SoftBody
