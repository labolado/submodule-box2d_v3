#pragma once
#include "box2d/box2d.h"

namespace SoftBody
{
// Private bridge to an active Box2D step. Owns no world, geometry or joints.
// IDs are validated before locking; reads/writes are restricted to that step.
class Box2DState
{
public:
	struct StateView
	{
		b2Pos center;
		b2Rot rotation;
		b2Vec2 velocity;
		float angularVelocity;
		float inverseMass, inverseInertia;
		b2Vec2 localCenter{};
		b2Pos origin{}; // Geometry origin, distinct from the mass centre.
		StateView() = default;
		StateView( b2Pos c, b2Rot q, b2Vec2 v, float w, float im, float ii, b2Vec2 local = {} )
			: center( c )
			, rotation( q )
			, velocity( v )
			, angularVelocity( w )
			, inverseMass( im )
			, inverseInertia( ii )
			, localCenter( local )
			, origin( b2OffsetPos( c, b2Neg( b2RotateVector( q, local ) ) ) )
		{
		}
	};
	static const char* validateStepBody( b2WorldId world, b2BodyId body );
	static bool readStepBody( void* step, b2BodyId body, StateView& state, int& index );
	static void writeStepBody( void* step, int index, const StateView& state );
};
} // namespace SoftBody
