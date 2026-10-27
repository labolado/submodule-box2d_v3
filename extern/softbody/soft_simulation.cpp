#include "soft_simulation_internal.h"

#include <atomic>
#include <limits>
#include <new>
#include <set>

namespace SoftBody
{
namespace
{
std::atomic<std::uint64_t> nextOwner{ 1 };
bool range( float value, float low, float high )
{
	return std::isfinite( value ) && value >= low && value <= high;
}
bool point( Point p )
{
	return range( p[0], -1e4f, 1e4f ) && range( p[1], -1e4f, 1e4f );
}
bool weights( const Weights& value, int count )
{
	if ( value.size() != size_t( count ) )
		return false;
	double sum = 0;
	for ( float v : value )
	{
		if ( !range( v, 0, 1 ) )
			return false;
		sum += v;
	}
	return std::abs( sum - 1 ) <= 1e-6;
}
// Validate once at creation, before allocating a body slot. Preserve public
// node identities even when boundary winding or traversal order changes.
bool configureShape( Body<float>& body, const SoftShapeDef& shape, float totalMass )
{
	if ( shape.nodes.empty() )
		return shape.boundary.empty();
	int count = int( shape.nodes.size() );
	if ( count < 3 || ( !shape.boundary.empty() && int( shape.boundary.size() ) != count ) )
		return false;
	std::vector<int> order( count );
	std::vector<bool> seen( count );
	for ( int i = 0; i < count; ++i )
	{
		int node = shape.boundary.empty() ? i : shape.boundary[i];
		if ( node < 0 || node >= count || seen[node] )
			return false;
		seen[node] = true;
		order[i] = node;
		auto p = shape.nodes[node];
		if ( !range( p[0], -100, 100 ) || !range( p[1], -100, 100 ) )
			return false;
	}
	auto orientation = []( Point a, Point b, Point c ) {
		return ( double( b[0] ) - a[0] ) * ( double( c[1] ) - a[1] ) -
			   ( double( b[1] ) - a[1] ) * ( double( c[0] ) - a[0] );
	};
	auto vertex = [&]( int i ) { return shape.nodes[order[i % count]]; };
	double area = 0;
	for ( int i = 0; i < count; ++i )
	{
		auto a = vertex( i ), b = vertex( i + 1 );
		if ( std::hypot( double( b[0] ) - a[0], double( b[1] ) - a[1] ) < .001 )
			return false;
		area += orientation( vertex( 0 ), a, b );
		// Adjacent edges may be collinear, but may not fold back over each other.
		auto c = vertex( i + 2 );
		if ( std::abs( orientation( a, b, c ) ) < 1e-10 &&
			 ( double( a[0] ) - b[0] ) * ( double( c[0] ) - b[0] ) +
					 ( double( a[1] ) - b[1] ) * ( double( c[1] ) - b[1] ) >
				 0 )
			return false;
		for ( int j = i + 2; j < count; ++j )
		{
			if ( i == 0 && j == count - 1 )
				continue;
			auto c = vertex( j ), d = vertex( j + 1 );
			if ( std::max( a[0], b[0] ) < std::min( c[0], d[0] ) ||
				 std::max( c[0], d[0] ) < std::min( a[0], b[0] ) ||
				 std::max( a[1], b[1] ) < std::min( c[1], d[1] ) ||
				 std::max( c[1], d[1] ) < std::min( a[1], b[1] ) )
				continue;
			double abC = orientation( a, b, c ), abD = orientation( a, b, d );
			double cdA = orientation( c, d, a ), cdB = orientation( c, d, b );
			if ( abC * abD <= 0 && cdA * cdB <= 0 )
				return false; // Includes nonadjacent touching and overlapping edges.
		}
	}
	if ( std::abs( area ) < 2e-6 )
		return false;
	if ( area < 0 )
		std::reverse( order.begin() + 1, order.begin() + count );
	body.resizeNodes( count );
	body.sourceNodes = order;
	for ( int i = 0; i < count; ++i )
	{
		body.x[i] = body.rest[i] = vertex( i );
		body.mass[i] = totalMass / float( count );
		auto a = vertex( i ), b = vertex( i + 1 );
		body.lengths[i] = std::hypot( b[0] - a[0], b[1] - a[1] );
		body.concaveReference =
			body.concaveReference || orientation( a, b, vertex( i + 2 ) ) < -1e-10;
	}
	body.restArea = body.area();
	return true;
}
bool configureSprings( Body<float>& body, const std::vector<SoftSpringDef>& springs, float dt )
{
	if ( springs.empty() )
		return true;
	std::vector<int> solverNodes( body.nodeCount );
	std::vector<double> stiffness( body.nodeCount );
	std::set<std::pair<int, int>> pairs;
	for ( int i = 0; i < body.nodeCount; ++i )
		solverNodes[body.sourceNodes[i]] = i;
	body.internalSprings.reserve( springs.size() );
	for ( const auto& spring : springs )
	{
		int a = spring.nodeA, b = spring.nodeB;
		if ( a < 0 || b < 0 || a >= body.nodeCount || b >= body.nodeCount || a == b ||
			 !range( spring.stiffness, .001f, 1e8f ) || !range( spring.dampingRatio, 0, 10 ) ||
			 !pairs.insert( { std::min( a, b ), std::max( a, b ) } ).second )
			return false;
		a = solverNodes[a];
		b = solverNodes[b];
		double length = std::hypot( double( body.rest[b][0] ) - body.rest[a][0],
									double( body.rest[b][1] ) - body.rest[a][1] );
		if ( length < .001 )
			return false;
		double epsilon = length * 1e-4;
		double effectiveMass = 1. / ( 1. / body.mass[a] + 1. / body.mass[b] );
		body.internalSprings.push_back(
			{ a, b, spring.stiffness,
			  2 * spring.dampingRatio * std::sqrt( spring.stiffness * effectiveMass ),
			  std::hypot( length, epsilon ), epsilon } );
		stiffness[a] += spring.stiffness;
		stiffness[b] += spring.stiffness;
	}
	double omega = 2 * 3.14159265358979323846 * body.frequency;
	for ( int i = 0; i < body.nodeCount; ++i )
		// Conservative rest-state stiffness budget, including all incident edges.
		// This does not promise stability under arbitrary collapse or contact.
		if ( double( dt ) * dt *
				 ( omega * omega * body.materialScale + 2 * stiffness[i] / body.mass[i] ) >
			 1 )
			return false;
	return true;
}
Weights orderWeights( const Weights& input, const Body<float>& body )
{
	Weights result( body.nodeCount );
	for ( int i = 0; i < body.nodeCount; ++i )
		result[i] = input[body.sourceNodes[i]];
	return result;
}
} // namespace
std::uint64_t AllocateSimulationOwner()
{
	return nextOwner.fetch_add( 1 );
}

Simulation::Impl::Impl( const SimulationDef& def )
	: limits( def )
	, owner( AllocateSimulationOwner() )
{
	if ( def.maxBodies < 1 || def.maxBodies > 32 || def.maxJoints < 0 || def.maxJoints > 64 ||
		 def.maxTouches < 0 || def.maxTouches > 64 || def.maxNodes < 3 ||
		 def.maxNodes > std::numeric_limits<int>::max() / 4 - 4 || def.maxSoftContactRows < 1 ||
		 def.maxInternalSprings < 0 ||
		 def.maxInternalSprings > std::numeric_limits<int>::max() / 64 ||
		 def.timeStep != 1.f / 480 || def.iterations != 8 || !point( def.gravity ) )
	{
		error = "unsupported soft simulation definition";
		return;
	}
	core.dt = def.timeStep;
	core.substeps = 1;
	core.iterations = def.iterations;
	core.gravity = def.gravity;
	core.maxSoftContactRows = def.maxSoftContactRows;
	core.bodies.reserve( def.maxBodies );
	core.pins.reserve( def.maxJoints );
	core.touchJoints.reserve( def.maxTouches );
	valid = true;
}
bool Simulation::Impl::editable()
{
	if ( !valid )
		return false;
	if ( stepping )
	{
		error = "soft simulation is executing a native step";
		return false;
	}
	error = nullptr;
	return true;
}
ObjectId Simulation::Impl::id( ObjectKind kind, int index, std::uint64_t generation ) const
{
	ObjectId result;
	result.owner = owner;
	result.index = index;
	result.generation = generation;
	result.kind = kind;
	return result;
}
bool Simulation::Impl::check( ObjectId id, ObjectKind expected ) const
{
	if ( !valid || id.owner != owner || id.kind != expected || id.index < 0 )
		return false;
	size_t i = size_t( id.index );
	if ( expected == ObjectKind::Body )
		return i < core.bodies.size() && core.bodies[i].active &&
			   core.bodies[i].generation == id.generation;
	if ( expected == ObjectKind::Joint )
		return i < core.pins.size() && core.pins[i].active &&
			   core.pins[i].generation == id.generation;
	if ( expected == ObjectKind::Touch )
		return i < core.touchJoints.size() && core.touchJoints[i].active &&
			   core.touchJoints[i].generation == id.generation;
	return false;
}
Simulation::Simulation( const SimulationDef& def )
	: impl_( new ( std::nothrow ) Impl( def ) )
{
}
Simulation::~Simulation()
{
	delete impl_;
}
bool Simulation::ready() const
{
	return impl_ && impl_->valid;
}
const char* Simulation::lastError() const
{
	return impl_ ? impl_->error : "cannot allocate soft simulation";
}
bool Simulation::isValid( ObjectId id ) const
{
	return impl_ && impl_->check( id, id.kind );
}
int Simulation::availableNodes() const
{
	if ( !ready() )
		return 0;
	int remaining = impl_->limits.maxNodes;
	for ( const auto& body : impl_->core.bodies )
		if ( body.active )
			remaining -= body.nodeCount;
	return remaining;
}
int Simulation::availableInternalSprings() const
{
	if ( !impl_ || !impl_->valid )
		return 0;
	int remaining = impl_->limits.maxInternalSprings;
	for ( const auto& body : impl_->core.bodies )
		if ( body.active )
			remaining -= int( body.internalSprings.size() );
	return remaining;
}
int Simulation::getBodyNodeCount( ObjectId id ) const
{
	return impl_ && impl_->check( id, ObjectKind::Body ) ? impl_->core.bodies[id.index].nodeCount
														 : 0;
}
ObjectId Simulation::createBody( const SoftBodyDef& def )
{
	if ( !impl_ || !impl_->editable() )
		return {};
	auto& s = *impl_;
	auto& bodies = s.core.bodies;
	if ( !range( def.mass, .001f, 1000 ) || !range( def.halfWidth, .01f, 100 ) ||
		 !range( def.halfHeight, .01f, 100 ) || !point( def.position ) ||
		 !range( def.frequency, .001f, 1000 ) || !range( def.damping, 0, 1e5f ) ||
		 !range( def.materialScale, .001f, 1e4f ) || !range( def.friction, 0, 10 ) ||
		 ( def.support != BodySupport::Free && def.support != BodySupport::World ) ||
		 2 * 3.14159265358979323846 * def.frequency * std::sqrt( def.materialScale ) * s.core.dt >
			 1 )
	{
		s.error = "invalid recoverable body definition";
		return {};
	}
	int slot = 0;
	while ( slot < int( bodies.size() ) && bodies[slot].active )
		++slot;
	if ( slot >= s.limits.maxBodies )
	{
		s.error = "soft body capacity reached";
		return {};
	}
	size_t count = def.shape.nodes.empty() ? 16 : def.shape.nodes.size();
	if ( count > size_t( availableNodes() ) )
	{
		s.error = "soft node capacity reached (maxNodes)";
		return {};
	}
	if ( def.internalSprings.size() > size_t( availableInternalSprings() ) )
	{
		s.error = "soft internal spring capacity reached (maxInternalSprings)";
		return {};
	}
	Body<float> body( 3, def.mass, def.halfWidth, def.halfHeight, 300, def.frequency, def.damping,
					  0, def.materialScale, .2f );
	if ( !configureShape( body, def.shape, def.mass ) )
	{
		s.error = "invalid soft shape: expected one simple contour with at least 3 nodes";
		return {};
	}
	if ( !configureSprings( body, def.internalSprings, s.core.dt ) )
	{
		s.error = "invalid internal springs or fixed-step stiffness budget exceeded";
		return {};
	}
	body.friction = def.friction;
	body.worldSupported = def.support == BodySupport::World;
	body.referencePosition = def.position;
	for ( int i = 0; i < body.nodeCount; ++i )
	{
		auto& p = body.x[i];
		p[0] += def.position[0];
		p[1] += def.position[1];
	}
	if ( slot == int( bodies.size() ) )
		bodies.push_back( std::move( body ) );
	else
	{
		body.generation = bodies[slot].generation + 1;
		bodies[slot] = std::move( body );
		auto& exclusions = s.core.exclusions;
		exclusions.erase( std::remove_if( exclusions.begin(), exclusions.end(),
										  [slot]( const std::array<int, 2>& p ) {
											  return p[0] == slot || p[1] == slot;
										  } ),
						  exclusions.end() );
	}
	return s.id( ObjectKind::Body, slot, bodies[slot].generation );
}
ObjectId Simulation::createJoint( const SoftJointDef& def )
{
	if ( !impl_ || !impl_->editable() )
		return {};
	auto& s = *impl_;
	auto& joints = s.core.pins;
	if ( !s.check( def.bodyA, ObjectKind::Body ) || !s.check( def.bodyB, ObjectKind::Body ) ||
		 def.bodyA.index == def.bodyB.index ||
		 !weights( def.weightsA, s.core.bodies[def.bodyA.index].nodeCount ) ||
		 !weights( def.weightsB, s.core.bodies[def.bodyB.index].nodeCount ) ||
		 !range( def.frequency, .01f, 60 ) || !range( def.dampingRatio, 0, 10 ) ||
		 !range( def.motorGain, 0, 1e6f ) )
	{
		s.error = "invalid soft joint definition";
		return {};
	}
	int slot = 0;
	while ( slot < int( joints.size() ) && joints[slot].active )
		++slot;
	if ( slot >= s.limits.maxJoints )
	{
		s.error = "soft joint capacity reached";
		return {};
	}
	World<float>::Pin pin;
	pin.a = def.bodyA.index;
	pin.b = def.bodyB.index;
	pin.wa = orderWeights( def.weightsA, s.core.bodies[pin.a] );
	pin.wb = orderWeights( def.weightsB, s.core.bodies[pin.b] );
	pin.gain = def.motorGain;
	pin.frequency = def.frequency;
	pin.dampingRatio = def.dampingRatio;
	if ( slot == int( joints.size() ) )
		joints.push_back( pin );
	else
	{
		pin.generation = joints[slot].generation + 1;
		joints[slot] = pin;
	}
	return s.id( ObjectKind::Joint, slot, joints[slot].generation );
}
ObjectId Simulation::createWorldJoint( const SoftWorldJointDef& def )
{
	if ( !impl_ || !impl_->editable() )
		return {};
	auto& s = *impl_;
	if ( !s.check( def.body, ObjectKind::Body ) ||
		 !weights( def.weights, s.core.bodies[def.body.index].nodeCount ) || !point( def.anchor ) ||
		 !range( def.frequency, .01f, 60 ) || !range( def.dampingRatio, 0, 10 ) ||
		 !range( def.motorGain, 0, 1e6f ) )
	{
		s.error = "invalid soft world joint definition";
		return {};
	}
	auto& joints = s.core.pins;
	int slot = 0;
	while ( slot < int( joints.size() ) && joints[slot].active )
		++slot;
	if ( slot >= s.limits.maxJoints )
	{
		s.error = "soft joint capacity reached";
		return {};
	}
	World<float>::Pin pin;
	pin.a = def.body.index;
	pin.b = -1;
	pin.wa = orderWeights( def.weights, s.core.bodies[pin.a] );
	pin.worldAnchor = def.anchor;
	pin.frequency = def.frequency;
	pin.dampingRatio = def.dampingRatio;
	pin.gain = def.motorGain;
	if ( slot == int( joints.size() ) )
		joints.push_back( pin );
	else
	{
		pin.generation = joints[slot].generation + 1;
		joints[slot] = pin;
	}
	return s.id( ObjectKind::Joint, slot, joints[slot].generation );
}
ObjectId Simulation::createTouch( const SoftTouchDef& def )
{
	if ( !impl_ || !impl_->editable() )
		return {};
	auto& s = *impl_;
	auto& touches = s.core.touchJoints;
	if ( !s.check( def.body, ObjectKind::Body ) ||
		 !weights( def.weights, s.core.bodies[def.body.index].nodeCount ) || !point( def.target ) ||
		 !range( def.frequency, .01f, 60 ) || !range( def.dampingRatio, 0, 10 ) ||
		 !range( def.maxForce, 0, 1e5f ) )
	{
		s.error = "invalid soft touch definition";
		return {};
	}
	int slot = 0;
	while ( slot < int( touches.size() ) && touches[slot].active )
		++slot;
	if ( slot >= s.limits.maxTouches )
	{
		s.error = "active touch joint limit reached";
		return {};
	}
	World<float>::TouchJoint touch;
	touch.body = def.body.index;
	touch.weights = orderWeights( def.weights, s.core.bodies[touch.body] );
	touch.target = def.target;
	touch.frequency = def.frequency;
	touch.dampingRatio = def.dampingRatio;
	touch.maxForce = def.maxForce;
	if ( slot == int( touches.size() ) )
		touches.push_back( touch );
	else
	{
		touch.generation = touches[slot].generation + 1;
		touches[slot] = touch;
	}
	return s.id( ObjectKind::Touch, slot, touches[slot].generation );
}
bool Simulation::remove( ObjectId id )
{
	if ( !impl_ || !impl_->editable() )
		return false;
	if ( !isValid( id ) )
	{
		impl_->error = "soft object was removed or belongs to another world";
		return false;
	}
	if ( id.kind == ObjectKind::Body )
		impl_->core.removeBody( id.index );
	else if ( id.kind == ObjectKind::Joint )
		impl_->core.pins[id.index].active = false;
	else
		impl_->core.touchJoints[id.index].active = false;
	return true;
}
bool Simulation::setMotor( ObjectId joint, float speed, float maxTorque )
{
	if ( !impl_ || !impl_->editable() )
		return false;
	if ( !impl_->check( joint, ObjectKind::Joint ) || !range( speed, -1e4f, 1e4f ) ||
		 !range( maxTorque, 0, 1e6f ) )
	{
		impl_->error = "invalid soft motor input";
		return false;
	}
	auto& pin = impl_->core.pins[joint.index];
	pin.speed = speed;
	pin.maxTorque = maxTorque;
	return true;
}
bool Simulation::setTouchTarget( ObjectId touch, Point target )
{
	if ( !impl_ || !impl_->editable() )
		return false;
	if ( !impl_->check( touch, ObjectKind::Touch ) || !point( target ) )
	{
		impl_->error = "invalid soft touch target";
		return false;
	}
	impl_->core.touchJoints[touch.index].target = target;
	return true;
}
bool Simulation::applyLinearImpulse( ObjectId id, Point impulse )
{
	if ( !impl_ || !impl_->editable() )
		return false;
	if ( !impl_->check( id, ObjectKind::Body ) || !point( impulse ) )
	{
		impl_->error = "invalid soft linear impulse";
		return false;
	}
	auto& body = impl_->core.bodies[id.index];
	double mass = 0;
	for ( float value : body.mass )
		mass += value;
	auto next = body.v;
	double work = 0;
	for ( int i = 0; i < body.nodeCount; ++i )
		for ( int axis = 0; axis < 2; ++axis )
		{
			double before = body.v[i][axis];
			next[i][axis] = float( before + impulse[axis] / mass );
			if ( !range( next[i][axis], -1e4f, 1e4f ) )
			{
				impl_->error = "soft impulse exceeds supported velocity";
				return false;
			}
			work +=
				.5 * body.mass[i] * ( double( next[i][axis] ) * next[i][axis] - before * before );
		}
	// Validate the complete update before changing any node. Equal velocity
	// increments give a total impulse independent of the node mass layout.
	body.v = next;
	impl_->core.appliedWork += work;
	return true;
}
bool Simulation::setCollisionEnabled( ObjectId a, ObjectId b, bool enabled )
{
	if ( !impl_ || !impl_->editable() )
		return false;
	if ( !impl_->check( a, ObjectKind::Body ) || !impl_->check( b, ObjectKind::Body ) ||
		 a.index == b.index )
	{
		impl_->error = "invalid soft collision pair";
		return false;
	}
	auto& pairs = impl_->core.exclusions;
	for ( auto it = pairs.begin(); it != pairs.end(); ++it )
		if ( ( ( *it )[0] == a.index && ( *it )[1] == b.index ) ||
			 ( ( *it )[1] == a.index && ( *it )[0] == b.index ) )
		{
			if ( enabled )
				pairs.erase( it );
			return true;
		}
	if ( !enabled )
		pairs.push_back( { a.index, b.index } );
	return true;
}
bool Simulation::getBodyState( ObjectId id, SoftBodyState& result ) const
{
	if ( !impl_ || !impl_->check( id, ObjectKind::Body ) || impl_->stepping )
		return false;
	const auto& body = impl_->core.bodies[id.index];
	result.nodeCount = body.nodeCount;
	result.internalSpringScale = body.internalSpringScale;
	result.internalSpringWork = body.internalSpringWork;
	result.internalSpringRecoverySteps = body.internalSpringRecoverySteps;
	result.vertices.resize( body.nodeCount );
	result.velocities.resize( body.nodeCount );
	result.boundary = body.sourceNodes;
	for ( int i = 0; i < body.nodeCount; ++i )
	{
		result.vertices[body.sourceNodes[i]] = body.x[i];
		result.velocities[body.sourceNodes[i]] = body.v[i];
	}
	result.support = body.worldSupported ? BodySupport::World : BodySupport::Free;
	result.referencePosition = body.referencePosition;
	return true;
}
bool Simulation::getTouchState( ObjectId id, SoftTouchState& result ) const
{
	if ( !impl_ || !impl_->check( id, ObjectKind::Touch ) || impl_->stepping )
		return false;
	const auto& touch = impl_->core.touchJoints[id.index];
	result.position = impl_->core.anchor( touch.body, touch.weights );
	result.target = touch.target;
	result.force = touch.lastForce;
	return true;
}
SimulationState Simulation::getState() const
{
	SimulationState result;
	if ( !ready() || impl_->stepping )
	{
		result.failure = "soft state unavailable";
		return result;
	}
	auto& core = impl_->core;
	auto metrics = core.currentMetrics();
	result.completedSteps = core.completedSubsteps;
	result.energy = metrics.energy;
	result.motorWork = core.motorWork;
	result.appliedWork = core.appliedWork;
	result.failure = core.failure;
	result.penetration = core.peak.penetration;
	result.pinError = core.peak.pin;
	result.nodeSpeed = core.peak.nodeSpeed;
	result.centerSpeed = core.peak.centerSpeed;
	result.minAreaRatio = core.peak.minArea;
	result.selfCross = core.peak.selfCross;
	for ( const auto& body : core.bodies )
		if ( body.active )
			for ( int i = 0; i < body.nodeCount; ++i )
				for ( float v : body.x[i] )
					result.vertices.push_back( v );
	return result;
}
} // namespace SoftBody
