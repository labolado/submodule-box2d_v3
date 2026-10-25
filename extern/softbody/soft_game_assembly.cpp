#include "soft_game_assembly.h"
extern "C"
{
#include "../../src/physics_world.h"
}

namespace SoftBody
{
namespace Game
{
#if defined( __clang__ )
#pragma clang fp contract( off )
#endif
namespace
{
bool validWeights( const std::array<float, 16>& weights )
{
	double sum = 0;
	for ( float weight : weights )
	{
		if ( !( weight >= 0 && weight <= 1 ) )
			return false;
		sum += weight;
	}
	return std::abs( sum - 1 ) <= 1e-6;
}
} // namespace

bool Assembly::syncParts()
{
	if ( soft.bodies.size() > MaxBodies || soft.pins.size() > MaxLinks )
	{
		error = "assembly_capacity_exceeded";
		return false;
	}
	parts.resize( soft.bodies.size() );
	links.resize( soft.pins.size() );
	for ( size_t id = 0; id < parts.size(); ++id )
	{
		if ( !soft.bodies[id].active )
		{
			parts[id].reset();
			continue;
		}
		if ( !parts[id] || parts[id]->generation != soft.bodies[id].generation )
			parts[id].reset( new Part( soft, int( id ) ) );
	}
	return true;
}

bool Assembly::prepare()
{
	Profile::Scope timing( profile, Profile::Prepare );
	if ( error )
		return false;
	if ( entered &&
		 ( b2StoreWorldId( world ) != b2StoreWorldId( boundWorld ) || flipY != boundFlipY ) )
	{
		error = "assembly_owner_changed";
		return false;
	}
	if ( soft.failure )
	{
		error = soft.failure;
		return false;
	}
	if ( !syncParts() )
		return false;
	if ( !soft.boundaries.empty() || soft.dt != Step || soft.substeps != 1 ||
		 soft.iterations != Sweeps )
	{
		error = "unsupported_assembly_configuration";
		return false;
	}
	for ( int a = 0; a < int( parts.size() ); ++a )
		for ( int b = a + 1; b < int( parts.size() ); ++b )
			if ( soft.pairEnabled( a, b ) )
			{
				error = "assembly_requires_explicit_soft_collision_exclusions";
				return false;
			}
	for ( size_t id = 0; id < soft.pins.size(); ++id )
	{
		const auto& pin = soft.pins[id];
		if ( !pin.active )
			continue;
		if ( pin.a < 0 || pin.a >= int( parts.size() ) || pin.b < 0 ||
			 pin.b >= int( parts.size() ) || pin.a == pin.b || !parts[pin.a] || !parts[pin.b] ||
			 !validWeights( pin.wa ) || !validWeights( pin.wb ) ||
			 !( pin.frequency >= .01f && pin.frequency <= 60 ) ||
			 !( pin.dampingRatio >= 0 && pin.dampingRatio <= 10 ) || !std::isfinite( pin.speed ) ||
			 !( pin.maxTorque >= 0 && pin.maxTorque <= 1e6f ) ||
			 !( pin.gain >= 0 && pin.gain <= 1e6f ) )
		{
			error = "invalid_assembly_connection";
			return false;
		}
	}
	int touches = 0;
	for ( const auto& touch : soft.touchJoints )
		if ( touch.active )
		{
			if ( ++touches > 64 || touch.body < 0 || touch.body >= int( parts.size() ) ||
				 !parts[touch.body] || !validWeights( touch.weights ) ||
				 !std::isfinite( touch.target[0] ) || !std::isfinite( touch.target[1] ) ||
				 !( touch.frequency >= .01f && touch.frequency <= 60 ) ||
				 !( touch.dampingRatio >= 0 && touch.dampingRatio <= 10 ) ||
				 !( touch.maxForce >= 0 && touch.maxForce <= 1e5f ) )
			{
				error = "invalid_assembly_touch";
				return false;
			}
		}
	for ( int id = 0; id < int( parts.size() ); ++id )
	{
		if ( !parts[id] )
			continue;
		auto& contact = parts[id]->contact;
		contact.world = world;
		contact.flipY = flipY;
		contact.friction = soft.bodies[id].friction;
		if ( !contact.validate() || !contact.prepare() || !contact.observeRecovery() )
		{
			error = contact.error;
			return false;
		}
		for ( int i = 0; i < contact.boxCount; ++i )
			if ( b2Body_GetType( contact.boxes[i].body ) == b2_dynamicBody )
			{
				error = "assembly_shared_dynamic_contacts_unsupported";
				return false;
			}
	}
	return true;
}

void Assembly::motors()
{
	double torques[MaxBodies] = {}, before = 0, after = 0;
	for ( const auto& pin : soft.pins )
		if ( pin.active && pin.maxTorque > 0 )
		{
			double relative = soft.omega( pin.b ) - soft.omega( pin.a );
			double torque =
				std::max( -double( pin.maxTorque ),
						  std::min( double( pin.maxTorque ),
									double( pin.gain ) * ( double( pin.speed ) - relative ) ) );
			torques[pin.b] += torque;
			torques[pin.a] -= torque;
		}
	for ( int id = 0; id < int( parts.size() ); ++id )
		if ( parts[id] )
			before += soft.kinetic( id );
	for ( int id = 0; id < int( parts.size() ); ++id )
		if ( torques[id] != 0 )
			soft.torque( id, torques[id] );
	for ( int id = 0; id < int( parts.size() ); ++id )
		if ( parts[id] )
			after += soft.kinetic( id );
	soft.motorWork += after - before;
}

void Assembly::prepareLinks()
{
	for ( size_t id = 0; id < soft.pins.size(); ++id )
	{
		const auto& pin = soft.pins[id];
		if ( !pin.active )
			continue;
		auto& link = links[id];
		auto& a = soft.bodies[pin.a];
		auto& b = soft.bodies[pin.b];
		link.inverseMass = 0;
		link.impulse = { 0, 0 };
		for ( int i = 0; i < 16; ++i )
			link.inverseMass +=
				pin.wa[i] * pin.wa[i] / a.mass[i] + pin.wb[i] * pin.wb[i] / b.mass[i];
		float omega = 2 * 3.14159265358979323846f * pin.frequency;
		float denominator = 2 * pin.dampingRatio * omega + Step * omega * omega;
		// Implicit spring compliance: gamma=1/(h*(d+h*k)), where k=m*w*w.
		// Accumulated impulses make repeated sweeps solve the same spring,
		// rather than applying the full spring force eight times.
		link.gamma = link.inverseMass / ( Step * denominator );
		float biasRate = omega * omega / denominator;
		auto x = soft.anchor( pin.a, pin.wa ), y = soft.anchor( pin.b, pin.wb );
		link.bias = { biasRate * ( y[0] - x[0] ), biasRate * ( y[1] - x[1] ) };
	}
}

void Assembly::solveLinks()
{
	for ( size_t id = 0; id < soft.pins.size(); ++id )
	{
		const auto& pin = soft.pins[id];
		if ( !pin.active )
			continue;
		auto& link = links[id];
		auto& a = soft.bodies[pin.a];
		auto& b = soft.bodies[pin.b];
		b2Vec2 relative{};
		for ( int i = 0; i < 16; ++i )
		{
			relative.x += pin.wb[i] * b.v[i][0] - pin.wa[i] * a.v[i][0];
			relative.y += pin.wb[i] * b.v[i][1] - pin.wa[i] * a.v[i][1];
		}
		auto impulse =
			b2MulSV( -1 / ( link.inverseMass + link.gamma ),
					 b2Add( b2Add( relative, link.bias ), b2MulSV( link.gamma, link.impulse ) ) );
		link.impulse = b2Add( link.impulse, impulse );
		for ( int i = 0; i < 16; ++i )
		{
			a.v[i][0] -= pin.wa[i] * impulse.x / a.mass[i];
			a.v[i][1] -= pin.wa[i] * impulse.y / a.mass[i];
			b.v[i][0] += pin.wb[i] * impulse.x / b.mass[i];
			b.v[i][1] += pin.wb[i] * impulse.y / b.mass[i];
		}
	}
}

void Assembly::stage( void* step, int phase, void* context )
{
	auto& owner = *static_cast<Assembly*>( context );
	if ( owner.error )
		return;
	if ( phase != owner.nextPhase++ )
	{
		owner.error = "assembly_phase_order";
		return;
	}
	if ( phase == 0 )
	{
		Profile::Scope timing( owner.profile, Profile::MaterialMotor );
		owner.soft.advanceMaterialVelocity();
		if ( owner.soft.failure )
		{
			owner.error = owner.soft.failure;
			return;
		}
		for ( auto& touch : owner.soft.touchJoints )
			if ( touch.active )
				owner.soft.solveTouchJoint( touch );
		owner.motors();
		return;
	}
	if ( phase == 1 )
	{
		Profile::Scope timing( owner.profile, Profile::InitialLinks );
		owner.prepareLinks();
		owner.solveLinks();
		return;
	}
	{
		Profile::Scope timing( owner.profile, Profile::ContactGeometry );
		for ( auto& part : owner.parts )
			if ( part )
			{
				auto& contact = part->contact;
				auto& body = part->contact.body();
				for ( int i = 0; i < 16; ++i )
					for ( int axis = 0; axis < 2; ++axis )
						body.x[i][axis] += Step * body.v[i][axis];
				if ( !contact.observeRecovery() )
				{
					owner.error = contact.error;
					return;
				}
				for ( int i = 0; i < contact.boxCount; ++i )
				{
					auto& box = contact.boxes[i];
					if ( box.enabled && b2Body_GetType( box.body ) != b2_staticBody &&
						 !contact.read( step, box.body, box.state, box.index ) )
					{
						owner.error = "assembly_contact_state";
						return;
					}
				}
				if ( !contact.projectGeometry() )
				{
					owner.error = contact.error;
					return;
				}
				contact.gather();
				if ( contact.error )
				{
					owner.error = contact.error;
					return;
				}
				for ( int i = 0; i < contact.rowCount; ++i )
				{
					auto& row = contact.rows[i];
					b2Vec2 tangent = { -row.normal.y, row.normal.x };
					contact.apply( row, b2Add( b2MulSV( row.normalImpulse, row.normal ),
											   b2MulSV( row.tangentImpulse, tangent ) ) );
				}
			}
	}
	{
		Profile::Scope timing( owner.profile, Profile::ContactSolve );
		for ( int sweep = 0; sweep < Sweeps; ++sweep )
		{
			for ( auto& part : owner.parts )
				if ( part )
				{
					auto& contact = part->contact;
					for ( int i = 0; i < contact.rowCount && !contact.error; ++i )
						contact.solve( contact.rows[i] );
					if ( contact.error )
					{
						owner.error = contact.error;
						return;
					}
				}
			owner.solveLinks();
		}
	}
	{
		Profile::Scope timing( owner.profile, Profile::Observe );
		for ( auto& part : owner.parts )
			if ( part )
			{
				auto& contact = part->contact;
				contact.previous = contact.rows;
				contact.previousCount = contact.rowCount;
				contact.lastDepth = 0;
				for ( int i = 0; i < contact.boxCount; ++i )
					if ( contact.boxes[i].enabled )
					{
						auto& box = contact.boxes[i];
						BoxProjection geometry( part->contact.body(), box.state, box.hx, box.hy );
						contact.lastDepth = std::max( contact.lastDepth, geometry.boundaryDepth() );
					}
			}
	}
}

bool Assembly::advance()
{
	Profile::Scope timing( profile, Profile::Advance );
	if ( !prepare() )
		return false;
	for ( auto& part : parts )
		if ( part )
		{
			auto& contact = part->contact;
			contact.recovery.remaining.fill( contact.recovery.maxCorrection );
			if ( contact.recovery.active )
				contact.clearCache();
		}
	nextPhase = 0;
	boundWorld = world;
	boundFlipY = flipY;
	++entered;
	{
		Profile::Scope nativeTiming( profile, Profile::NativeWorld );
		b2World_StepWithSoftStages( world, Step, 1, nullptr, nullptr, stage, this );
		bool staticOnly = true;
		for ( const auto& part : parts )
			if ( part )
				for ( int i = 0; i < part->contact.boxCount; ++i )
					if ( part->contact.boxes[i].enabled &&
						 b2Body_GetType( part->contact.boxes[i].body ) != b2_staticBody )
						staticOnly = false;
		if ( nextPhase == 0 && staticOnly )
			for ( int phase = 0; phase < 3; ++phase )
				stage( nullptr, phase, this );
	}
	if ( !error && nextPhase != 3 )
		error = "assembly_missing_stage";
	// There is no rollback after native entry. Material state is already in
	// the authoritative world; a failed step remains latched.
	if ( error )
		return false;
	Profile::Scope observationTiming( profile, Profile::Finalize );
	for ( auto& part : parts )
		if ( part )
		{
			auto& contact = part->contact;
			if ( !contact.observeRecovery( true ) )
			{
				error = contact.error;
				return false;
			}
			for ( int i = 0; i < contact.boxCount; ++i )
				contact.boxes[i].cachedTransform = b2Body_GetTransform( contact.boxes[i].body );
			peakDepth = std::max( peakDepth, contact.lastDepth );
		}
	soft.peak = soft.currentMetrics();
	peakSpeed = std::max( peakSpeed, soft.peak.nodeSpeed );
	peakGap = std::max( peakGap, soft.peak.pin );
	++soft.completedSubsteps;
	++completed;
	return true;
}
} // namespace Game
} // namespace SoftBody
