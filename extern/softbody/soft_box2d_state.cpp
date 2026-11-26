#include "soft_box2d_state.h"
extern "C"
{
#include "../../src/body.h"
#include "../../src/physics_world.h"
#include "../../src/solver.h"
}

namespace SoftBody
{
#if defined( __clang__ )
#pragma clang fp contract( off )
#endif
const char* Box2DState::validateStepBody( b2WorldId world, b2BodyId body )
{
	if ( !b2World_IsValid( world ) )
		return "invalid_world";
	if ( b2World_IsLocked( world ) )
		return "world_locked";
	if ( !b2Body_IsValid( body ) )
		return "invalid_body";
	auto owner = b2Body_GetWorld( body );
	if ( owner.index1 != world.index1 || owner.generation != world.generation )
		return "foreign_world";
	if ( !b2Body_IsEnabled( body ) )
		return "disabled_body";
	if ( b2Body_GetType( body ) == b2_staticBody )
		return "static_body_unsupported";
	if ( !b2Body_IsAwake( body ) )
		return "sleeping_body";
	if ( b2Body_IsFixedRotation( body ) )
		return "motion_lock_unsupported";
	return nullptr;
}
bool Box2DState::readStepBody( void* step, b2BodyId id, StateView& state, int& index )
{
	auto& c = *static_cast<b2StepContext*>( step );
	index = -1;
	// IDs were validated before locking. No create/delete/wake is allowed in a stage.
	if ( id.world0 != c.world->worldId )
		return false;
	auto body = b2GetBodyFullId( c.world, id );
	if ( body->setIndex != b2_awakeSet )
		return false;
	int i = body->localIndex;
	if ( i < 0 || i >= c.world->solverSets.data[b2_awakeSet].bodyStates.count ||
		 c.sims[i].bodyId != body->id )
		return false;
	auto& sim = c.sims[i];
	auto& current = c.states[i];
	state = { b2OffsetPos( sim.center, current.deltaPosition ),
			  b2MulRot( current.deltaRotation, sim.transform.q ),
			  current.linearVelocity,
			  current.angularVelocity,
			  sim.invMass,
			  sim.invInertia,
			  sim.localCenter };
	// Preserve the original geometry transform before any integration. Rebuilding
	// it through the mass centre can perturb strict feature classifications.
	if ( current.deltaPosition.x == 0 && current.deltaPosition.y == 0 &&
		 current.deltaRotation.c == 1 && current.deltaRotation.s == 0 )
		state.origin = sim.transform.p;
	index = i;
	return true;
}
void Box2DState::writeStepBody( void* step, int index, const StateView& state )
{
	auto& c = *static_cast<b2StepContext*>( step );
	if ( c.sims[index].invMass > 0 )
	{
		c.states[index].linearVelocity = state.velocity;
		c.states[index].angularVelocity = state.angularVelocity;
	}
}

} // namespace SoftBody
