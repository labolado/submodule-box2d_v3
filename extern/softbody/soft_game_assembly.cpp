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
bool validWeights( const std::vector<float>& weights, int count )
{
	if ( weights.size() != size_t( count ) )
		return false;
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
		parts[id]->contact.profile = &profile;
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
	if ( soft.dt != Step || soft.substeps != 1 ||
		 soft.iterations != Sweeps )
	{
		error = "unsupported_assembly_configuration";
		return false;
	}
	for ( int a = 0; a < int( parts.size() ); ++a )
		for ( int b = a + 1; b < int( parts.size() ); ++b )
			if ( !enableSoftContacts && soft.pairEnabled( a, b ) )
			{
				error = "assembly_requires_explicit_soft_collision_exclusions";
				return false;
			}
	for ( size_t id = 0; id < soft.pins.size(); ++id )
	{
		const auto& pin = soft.pins[id];
		if ( !pin.active )
			continue;
		bool worldPin = pin.b == -1;
		bool validTarget =
			worldPin ? std::isfinite( pin.worldAnchor[0] ) && std::isfinite( pin.worldAnchor[1] )
					 : pin.b >= 0 && pin.b < int( parts.size() ) && parts[pin.b] &&
						   validWeights( pin.wb, soft.bodies[pin.b].nodeCount );
		if ( pin.a < 0 || pin.a >= int( parts.size() ) || !validTarget || pin.a == pin.b ||
			 !parts[pin.a] || !validWeights( pin.wa, soft.bodies[pin.a].nodeCount ) ||
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
				 !parts[touch.body] ||
				 !validWeights( touch.weights, soft.bodies[touch.body].nodeCount ) ||
				 !std::isfinite( touch.target[0] ) || !std::isfinite( touch.target[1] ) ||
				 !( touch.frequency >= .01f && touch.frequency <= 60 ) ||
				 !( touch.dampingRatio >= 0 && touch.dampingRatio <= 10 ) ||
				 !( touch.maxForce >= 0 && touch.maxForce <= 1e5f ) )
			{
				error = "invalid_assembly_touch";
				return false;
			}
		}
	// A dynamic target must receive soft impulses before Box2D's biased
	// constraints and position integration. Late impulses let a stiff soft
	// contact fight the rigid support without a same-step position response.
	// Static/kinematic-only assemblies retain their established ordering.
	earlyContacts = false;
	for ( auto& link : rigidLinks )
	{
		link.enabled = false;
		if ( !link.active )
			continue;
		if ( link.body < 0 || link.body >= int( parts.size() ) || !parts[link.body] ||
			 !validWeights( link.weights, soft.bodies[link.body].nodeCount ) ||
			 !b2Body_IsValid( link.rigid ) )
		{
			error = "invalid_assembly_rigid_connection";
			return false;
		}
		if ( !b2Body_IsEnabled( link.rigid ) )
			continue;
		auto& contact = parts[link.body]->contact;
		contact.flipY = flipY;
		if ( b2Body_GetType( link.rigid ) == b2_staticBody )
		{
			auto transform = b2Body_GetTransform( link.rigid );
			link.state = View{ b2Body_GetWorldCenter( link.rigid ), transform.q, { 0, 0 }, 0, 0, 0,
							   b2Body_GetLocalCenter( link.rigid ) };
			link.state.origin = transform.p;
			contact.reflect( link.state );
			link.index = -1;
		}
		else if ( const char* reason = Bridge::validateStepBody( world, link.rigid ) )
		{
			error = reason;
			return false;
		}
		link.enabled = true;
		// The rigid joints/contacts must see the pin's reaction before integration.
		earlyContacts = true;
	}
	for ( int id = 0; id < int( parts.size() ); ++id )
	{
		if ( !parts[id] )
			continue;
		auto& contact = parts[id]->contact;
		contact.retainBoundarySides = enableSoftContacts;
		contact.world = world;
		contact.flipY = flipY;
		contact.friction = soft.bodies[id].friction;
		if ( !contact.validate() || !contact.prepare() || !contact.observeRecovery() )
		{
			error = contact.error;
			return false;
		}
		for ( int i = 0; i < contact.boxCount; ++i )
			if ( enableDynamicContacts && contact.boxes[i].enabled &&
				 b2Body_GetType( contact.boxes[i].body ) == b2_dynamicBody )
				earlyContacts = true;
		for ( int i = 0; i < contact.boxCount; ++i )
			if ( !enableDynamicContacts &&
				 b2Body_GetType( contact.boxes[i].body ) == b2_dynamicBody )
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
			if ( pin.b == -1 )
			{
				// A fixed world absorbs the reaction torque. Preserve the old API's
				// positive=counterclockwise convention in soft metres/y-up space.
				double torque = pin.gain * ( double( pin.speed ) - soft.omega( pin.a ) );
				torques[pin.a] += std::max( -double( pin.maxTorque ),
											std::min( double( pin.maxTorque ), torque ) );
				continue;
			}
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

bool Assembly::rigidMotors( void* step )
{
	for ( auto& link : rigidLinks )
	{
		link.motorImpulse = link.motorWork = 0;
		if ( !link.enabled || link.maxMotorTorque == 0 || link.motorGain == 0 )
			continue;
		auto& contact = parts[link.body]->contact;
		if ( b2Body_GetType( link.rigid ) != b2_staticBody &&
			 !contact.read( step, link.rigid, link.state, link.index ) )
		{
			error = "assembly_rigid_motor_state";
			return false;
		}
		const auto& body = soft.bodies[link.body];
		auto center = soft.center( link.body );
		double inertia = 0;
		for ( int i = 0; i < body.nodeCount; ++i )
		{
			double x = double( body.x[i][0] ) - center[0];
			double y = double( body.x[i][1] ) - center[1];
			inertia += body.mass[i] * ( x * x + y * y );
		}
		// Match Core::torque's collapsed-body guard on BOTH ends: a body
		// with no lever arm must not leave an unmatched rigid reaction.
		if ( inertia < 1e-12 )
			continue;
		double inverse = 1 / inertia + link.state.inverseInertia;
		double relative = soft.omega( link.body ) - link.state.angularVelocity;
		// Implicit proportional speed servo. The denominator accounts for
		// both angular responses and avoids overshoot at small inertia.
		double gain = double( Step ) * link.motorGain;
		double impulse = gain * ( link.motorSpeed - relative ) / ( 1 + gain * inverse );
		double cap = double( Step ) * link.maxMotorTorque;
		impulse = std::max( -cap, std::min( cap, impulse ) );
		if ( !std::isfinite( impulse ) )
		{
			error = "nonfinite_assembly_rigid_motor";
			return false;
		}
		double before = soft.kinetic( link.body );
		soft.torque( link.body, impulse / Step );
		soft.motorWork += soft.kinetic( link.body ) - before;
		link.state.angularVelocity -= float( impulse * link.state.inverseInertia );
		if ( link.index >= 0 )
			contact.write( step, link.index, link.state );
		link.motorImpulse = impulse;
		// Relative actuator work; a prescribed rigid target can supply or
		// absorb work externally. This is not the whole world's energy.
		link.motorWork = impulse * relative + .5 * impulse * impulse * inverse;
	}
	return true;
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
		if ( pin.b == -1 )
		{
			link.inverseMass = 0;
			link.impulse = { 0, 0 };
			for ( int i = 0; i < a.nodeCount; ++i )
				link.inverseMass += pin.wa[i] * pin.wa[i] / a.mass[i];
			float omega = 2 * 3.14159265358979323846f * pin.frequency;
			float denominator = 2 * pin.dampingRatio * omega + Step * omega * omega;
			link.gamma = link.inverseMass / ( Step * denominator );
			float biasRate = omega * omega / denominator;
			auto x = soft.anchor( pin.a, pin.wa );
			link.bias = { biasRate * ( pin.worldAnchor[0] - x[0] ),
						  biasRate * ( pin.worldAnchor[1] - x[1] ) };
			continue;
		}
		auto& b = soft.bodies[pin.b];
		link.inverseMass = 0;
		link.impulse = { 0, 0 };
		for ( int i = 0; i < std::max( a.nodeCount, b.nodeCount ); ++i )
			link.inverseMass += ( i < a.nodeCount ? pin.wa[i] * pin.wa[i] / a.mass[i] : 0 ) +
								( i < b.nodeCount ? pin.wb[i] * pin.wb[i] / b.mass[i] : 0 );
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
		if ( pin.b == -1 )
		{
			b2Vec2 relative{};
			for ( int i = 0; i < a.nodeCount; ++i )
			{
				relative.x -= pin.wa[i] * a.v[i][0];
				relative.y -= pin.wa[i] * a.v[i][1];
			}
			auto impulse = b2MulSV(
				-1 / ( link.inverseMass + link.gamma ),
				b2Add( b2Add( relative, link.bias ), b2MulSV( link.gamma, link.impulse ) ) );
			link.impulse = b2Add( link.impulse, impulse );
			for ( int i = 0; i < a.nodeCount; ++i )
			{
				a.v[i][0] -= pin.wa[i] * impulse.x / a.mass[i];
				a.v[i][1] -= pin.wa[i] * impulse.y / a.mass[i];
			}
			continue;
		}
		auto& b = soft.bodies[pin.b];
		b2Vec2 relative{};
		for ( int i = 0; i < std::max( a.nodeCount, b.nodeCount ); ++i )
		{
			relative.x += ( i < b.nodeCount ? pin.wb[i] * b.v[i][0] : 0 ) -
						  ( i < a.nodeCount ? pin.wa[i] * a.v[i][0] : 0 );
			relative.y += ( i < b.nodeCount ? pin.wb[i] * b.v[i][1] : 0 ) -
						  ( i < a.nodeCount ? pin.wa[i] * a.v[i][1] : 0 );
		}
		auto impulse =
			b2MulSV( -1 / ( link.inverseMass + link.gamma ),
					 b2Add( b2Add( relative, link.bias ), b2MulSV( link.gamma, link.impulse ) ) );
		link.impulse = b2Add( link.impulse, impulse );
		for ( int i = 0; i < a.nodeCount; ++i )
		{
			a.v[i][0] -= pin.wa[i] * impulse.x / a.mass[i];
			a.v[i][1] -= pin.wa[i] * impulse.y / a.mass[i];
		}
		for ( int i = 0; i < b.nodeCount; ++i )
		{
			b.v[i][0] += pin.wb[i] * impulse.x / b.mass[i];
			b.v[i][1] += pin.wb[i] * impulse.y / b.mass[i];
		}
	}
}

bool Assembly::prepareRigidLinks( void* step )
{
	for ( auto& link : rigidLinks )
	{
		link.impulse = { 0, 0 };
		if ( !link.enabled )
			continue;
		auto& contact = parts[link.body]->contact;
		if ( b2Body_GetType( link.rigid ) != b2_staticBody &&
			 !contact.read( step, link.rigid, link.state, link.index ) )
		{
			error = "assembly_rigid_pin_state";
			return false;
		}
		const auto& state = link.state;
		link.arm = b2RotateVector( state.rotation, b2Sub( link.localAnchor, state.localCenter ) );
		float inverse = state.inverseMass;
		const auto& body = soft.bodies[link.body];
		for ( int i = 0; i < body.nodeCount; ++i )
			inverse += link.weights[i] * link.weights[i] / body.mass[i];
		float xx = inverse + state.inverseInertia * link.arm.y * link.arm.y;
		float xy = -state.inverseInertia * link.arm.x * link.arm.y;
		float yy = inverse + state.inverseInertia * link.arm.x * link.arm.x;
		float omega = 2 * 3.14159265358979323846f * link.frequency;
		float denominator = 2 * link.dampingRatio * omega + Step * omega * omega;
		float compliance = 1 / ( Step * denominator );
		link.gamma = { { compliance * xx, compliance * xy }, { compliance * xy, compliance * yy } };
		link.mass = b2GetInverse22( { { xx + link.gamma.cx.x, xy + link.gamma.cx.y },
									  { xy + link.gamma.cy.x, yy + link.gamma.cy.y } } );
		auto anchor = soft.anchor( link.body, link.weights );
		auto target =
			b2OffsetPos( state.origin, b2RotateVector( state.rotation, link.localAnchor ) );
		link.bias = { float( anchor[0] - target.x ), float( anchor[1] - target.y ) };
		link.bias = b2MulSV( omega * omega / denominator, link.bias );
	}
	return true;
}

bool Assembly::solveRigidLinks( void* step )
{
	for ( auto& link : rigidLinks )
	{
		if ( !link.enabled )
			continue;
		auto& contact = parts[link.body]->contact;
		// Read every visit: another pin or a contact may share this rigid body.
		if ( link.index >= 0 && !contact.read( step, link.rigid, link.state, link.index ) )
		{
			error = "assembly_rigid_pin_state";
			return false;
		}
		auto& state = link.state;
		auto& body = soft.bodies[link.body];
		b2Vec2 relative =
			b2Neg( b2Add( state.velocity, b2CrossSV( state.angularVelocity, link.arm ) ) );
		for ( int i = 0; i < body.nodeCount; ++i )
		{
			relative.x += link.weights[i] * body.v[i][0];
			relative.y += link.weights[i] * body.v[i][1];
		}
		auto impulse = b2Neg( b2MulMV( link.mass, b2Add( b2Add( relative, link.bias ),
														 b2MulMV( link.gamma, link.impulse ) ) ) );
		if ( !std::isfinite( impulse.x ) || !std::isfinite( impulse.y ) )
		{
			error = "nonfinite_assembly_rigid_pin_impulse";
			return false;
		}
		link.impulse = b2Add( link.impulse, impulse );
		for ( int i = 0; i < body.nodeCount; ++i )
		{
			body.v[i][0] += link.weights[i] * impulse.x / body.mass[i];
			body.v[i][1] += link.weights[i] * impulse.y / body.mass[i];
		}
		state.velocity = b2MulSub( state.velocity, state.inverseMass, impulse );
		state.angularVelocity -= state.inverseInertia * b2Cross( link.arm, impulse );
		if ( link.index >= 0 )
			contact.write( step, link.index, state );
	}
	return true;
}

bool Assembly::refreshDynamic( Solver& contact, void* step )
{
	if ( !enableDynamicContacts )
		return true;
	// Box2D's step state is authoritative across parts. Each local contact
	// batch observes previous batches, including their warm-start impulses.
	for ( int i = 0; i < contact.boxCount; ++i )
	{
		auto& box = contact.boxes[i];
		if ( box.enabled && box.stateSlot == i && box.state.inverseMass > 0 &&
			 !contact.read( step, box.body, box.state, box.index ) )
		{
			error = "assembly_dynamic_state";
			return false;
		}
	}
	return true;
}

void Assembly::publishDynamic( Solver& contact, void* step )
{
	if ( !enableDynamicContacts )
		return;
	for ( int i = 0; i < contact.boxCount; ++i )
	{
		const auto& box = contact.boxes[i];
		if ( box.enabled && box.stateSlot == i && box.state.inverseMass > 0 )
			contact.write( step, box.index, box.state );
	}
}

bool Assembly::pairGeometry( void* step )
{
	pairs.positionContext = this;
	pairs.projectTranslation = []( void* context, int id, Core::Vec desired ) {
		return static_cast<Assembly*>( context )->parts[id]->contact.projectTranslation( desired );
	};
	pairs.positionLimit = []( void* context, int id, int node, Core::Vec delta ) {
		return static_cast<Assembly*>( context )->parts[id]->contact.limitCorrection( node, delta );
	};
	// All positions must be current before gathering any pair or rigid rows.
	for ( auto& part : parts )
		if ( part )
		{
			auto& contact = part->contact;
			auto& body = contact.body();
			for ( int i = 0; i < contact.boxCount; ++i )
			{
				auto& box = contact.boxes[i];
				if ( box.enabled && b2Body_GetType( box.body ) != b2_staticBody &&
					 !contact.read( step, box.body, box.state, box.index ) )
				{
					error = "assembly_contact_state";
					return false;
				}
			}
			contact.captureBoundarySides();
			for ( int i = 0; i < body.nodeCount; ++i )
				for ( int axis = 0; axis < 2; ++axis )
					body.x[i][axis] += Step * body.v[i][axis];
		}
	if ( !earlyContacts )
		pairs.beginStep();
	// Fixed passes share the per-node correction budgets. End with rigid
	// projection to retain the existing wall/press containment policy.
	for ( int pass = 0; pass < 2; ++pass )
	{
		if ( !pairs.project() )
		{
			error = pairs.error;
			return false;
		}
		for ( auto& part : parts )
			if ( part && ( !part->contact.observeRecovery() || !part->contact.projectGeometry() ) )
			{
				error = part->contact.error;
				return false;
			}
	}
	return earlyContacts || gatherContacts( step );
}

bool Assembly::gatherContacts( void* step )
{
	if ( enableSoftContacts && !pairs.gather() )
	{
		error = pairs.error;
		return false;
	}
	if ( enableSoftContacts )
		pairs.warmStart();
	for ( auto& part : parts )
		if ( part )
		{
			auto& contact = part->contact;
			if ( !refreshDynamic( contact, step ) )
				return false;
			contact.gather();
			if ( contact.error )
			{
				error = contact.error;
				return false;
			}
			for ( int i = 0; i < contact.rowCount; ++i )
			{
				auto& row = contact.rows[i];
				b2Vec2 tangent = { -row.normal.y, row.normal.x };
				contact.apply( row, b2Add( b2MulSV( row.normalImpulse, row.normal ),
										   b2MulSV( row.tangentImpulse, tangent ) ) );
			}
			publishDynamic( contact, step );
		}
	return true;
}

bool Assembly::solveContacts( void* step )
{
	Profile::Scope timing( profile, Profile::ContactSolve );
	for ( int sweep = 0; sweep < Sweeps; ++sweep )
	{
		if ( enableSoftContacts )
			pairs.solve();
		for ( auto& part : parts )
			if ( part )
			{
				auto& contact = part->contact;
				if ( !refreshDynamic( contact, step ) )
					return false;
				for ( int i = 0; i < contact.rowCount && !contact.error; ++i )
					contact.solve( contact.rows[i] );
				if ( contact.error )
				{
					error = contact.error;
					return false;
				}
				publishDynamic( contact, step );
			}
		solveLinks();
		if ( !solveRigidLinks( step ) )
			return false;
	}

	return true;
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
		if ( !owner.rigidMotors( step ) )
			return;
		return;
	}
	if ( phase == 1 )
	{
		{
			Profile::Scope timing( owner.profile, Profile::InitialLinks );
			owner.prepareLinks();
			owner.solveLinks();
			if ( !owner.prepareRigidLinks( step ) || !owner.solveRigidLinks( step ) )
				return;
		}
		if ( owner.earlyContacts )
		{
			{
				Profile::Scope timing( owner.profile, Profile::ContactGeometry );
				for ( auto& part : owner.parts )
					if ( part )
					{
						auto& contact = part->contact;
						for ( int i = 0; i < contact.boxCount; ++i )
						{
							auto& shape = contact.boxes[i];
							if ( shape.enabled && b2Body_GetType( shape.body ) != b2_staticBody &&
								 !contact.read( step, shape.body, shape.state, shape.index ) )
							{
								owner.error = "assembly_contact_state";
								return;
							}
						}
						contact.captureBoundarySides();
					}
				if ( owner.enableSoftContacts )
					owner.pairs.beginStep();
				if ( !owner.gatherContacts( step ) )
					return;
			}
			if ( !owner.solveContacts( step ) )
				return;
		}
		return;
	}
	{
		Profile::Scope timing( owner.profile, Profile::ContactGeometry );
		if ( owner.enableSoftContacts )
		{
			if ( !owner.pairGeometry( step ) )
				return;
		}
		else
			for ( auto& part : owner.parts )
				if ( part )
				{
					auto& contact = part->contact;
					auto& body = part->contact.body();
					for ( int i = 0; i < body.nodeCount; ++i )
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
					if ( !owner.earlyContacts )
					{
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
						owner.publishDynamic( contact, step );
					}
				}
	}
	if ( !owner.earlyContacts && !owner.solveContacts( step ) )
		return;

	{
		Profile::Scope timing( owner.profile, Profile::Observe );
		if ( owner.enableSoftContacts )
			owner.pairs.finish();
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
						if ( box.geometry.kind == RigidGeometry::Chain )
						{
							contact.lastDepth =
								std::max( contact.lastDepth, contact.chainDepth( i ) );
							continue;
						}
						BoxProjection geometry( part->contact.body(), box.state, box.geometry );
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
			contact.recovery.remaining.assign( contact.body().nodeCount,
											   contact.recovery.maxCorrection );
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
		for ( const auto& link : rigidLinks )
			if ( link.enabled && b2Body_GetType( link.rigid ) != b2_staticBody )
				staticOnly = false;
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
			if ( !contact.observeRecovery( true ) || !contact.withinChainQuery() )
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
