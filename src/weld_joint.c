// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "body.h"
#include "core.h"
#include "joint.h"
#include "physics_world.h"
#include "solver.h"
#include "solver_set.h"

// needed for dll export
#include "box2d/box2d.h"

#include <float.h>

typedef struct
{
	float x, y, z;
} b2WeldVec3;

// A 3-by-3 matrix. Stored in column-major order.
typedef struct
{
	b2WeldVec3 cx, cy, cz;
} b2WeldMat33;

static inline float b2DotWeldVec3( b2WeldVec3 a, b2WeldVec3 b )
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

static inline b2WeldVec3 b2CrossWeldVec3( b2WeldVec3 a, b2WeldVec3 b )
{
	return (b2WeldVec3){ a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
}

static bool b2SolveWeld33( const b2WeldMat33* m, b2WeldVec3 b, b2WeldVec3* result )
{
	float scale = b2AbsFloat( m->cx.x );
	scale = b2MaxFloat( scale, b2AbsFloat( m->cx.y ) );
	scale = b2MaxFloat( scale, b2AbsFloat( m->cx.z ) );
	scale = b2MaxFloat( scale, b2AbsFloat( m->cy.x ) );
	scale = b2MaxFloat( scale, b2AbsFloat( m->cy.y ) );
	scale = b2MaxFloat( scale, b2AbsFloat( m->cy.z ) );
	scale = b2MaxFloat( scale, b2AbsFloat( m->cz.x ) );
	scale = b2MaxFloat( scale, b2AbsFloat( m->cz.y ) );
	scale = b2MaxFloat( scale, b2AbsFloat( m->cz.z ) );

	if ( scale == 0.0f )
	{
		return false;
	}

	float invScale = 1.0f / scale;
	b2WeldMat33 n = {
		{ invScale * m->cx.x, invScale * m->cx.y, invScale * m->cx.z },
		{ invScale * m->cy.x, invScale * m->cy.y, invScale * m->cy.z },
		{ invScale * m->cz.x, invScale * m->cz.y, invScale * m->cz.z },
	};

	float det = b2DotWeldVec3( n.cx, b2CrossWeldVec3( n.cy, n.cz ) );
	if ( b2AbsFloat( det ) <= 100.0f * FLT_EPSILON )
	{
		return false;
	}

	float invDetScale = invScale / det;
	result->x = invDetScale * b2DotWeldVec3( b, b2CrossWeldVec3( n.cy, n.cz ) );
	result->y = invDetScale * b2DotWeldVec3( n.cx, b2CrossWeldVec3( b, n.cz ) );
	result->z = invDetScale * b2DotWeldVec3( n.cx, b2CrossWeldVec3( n.cy, b ) );

	return b2IsValidFloat( result->x ) && b2IsValidFloat( result->y ) && b2IsValidFloat( result->z );
}

void b2WeldJoint_SetLinearHertz( b2JointId jointId, float hertz )
{
	B2_ASSERT( b2IsValidFloat( hertz ) && hertz >= 0.0f );
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_weldJoint );
	joint->weldJoint.linearHertz = hertz;
}

float b2WeldJoint_GetLinearHertz( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_weldJoint );
	return joint->weldJoint.linearHertz;
}

void b2WeldJoint_SetLinearDampingRatio( b2JointId jointId, float dampingRatio )
{
	B2_ASSERT( b2IsValidFloat( dampingRatio ) && dampingRatio >= 0.0f );
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_weldJoint );
	joint->weldJoint.linearDampingRatio = dampingRatio;
}

float b2WeldJoint_GetLinearDampingRatio( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_weldJoint );
	return joint->weldJoint.linearDampingRatio;
}

void b2WeldJoint_SetAngularHertz( b2JointId jointId, float hertz )
{
	B2_ASSERT( b2IsValidFloat( hertz ) && hertz >= 0.0f );
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_weldJoint );
	joint->weldJoint.angularHertz = hertz;
}

float b2WeldJoint_GetAngularHertz( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_weldJoint );
	return joint->weldJoint.angularHertz;
}

void b2WeldJoint_SetAngularDampingRatio( b2JointId jointId, float dampingRatio )
{
	B2_ASSERT( b2IsValidFloat( dampingRatio ) && dampingRatio >= 0.0f );
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_weldJoint );
	joint->weldJoint.angularDampingRatio = dampingRatio;
}

float b2WeldJoint_GetAngularDampingRatio( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_weldJoint );
	return joint->weldJoint.angularDampingRatio;
}

void b2WeldJoint_EnableBlockSolve( b2JointId jointId, bool enableBlockSolve )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_weldJoint );
	joint->weldJoint.enableBlockSolve = enableBlockSolve;
}

bool b2WeldJoint_IsBlockSolveEnabled( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_weldJoint );
	return joint->weldJoint.enableBlockSolve;
}

b2Vec2 b2GetWeldJointForce( b2World* world, b2JointSim* base )
{
	b2Vec2 force = b2MulSV( world->inv_h, base->weldJoint.linearImpulse );
	return force;
}

float b2GetWeldJointTorque( b2World* world, b2JointSim* base )
{
	return world->inv_h * base->weldJoint.angularImpulse;
}

// Point-to-point constraint
// C = p2 - p1
// Cdot = v2 - v1
//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
// J = [-E -r1_skew E r2_skew ]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

// Angle constraint
// C = angle2 - angle1 - referenceAngle
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
// K = invI1 + invI2

// 3x3 Block
// K = [J1] * invM * [J1T J2T]
//     [J2]
//   = [J1] * [invM * J1T invM * J2T]
//     [J2]
//   = [J1 * invM * J1T J1 * invM * J2T]
//     [J2 * invM * J1T J2 * invM * J2T]

void b2PrepareWeldJoint( b2JointSim* base, b2StepContext* context )
{
	B2_ASSERT( base->type == b2_weldJoint );

	// chase body id to the solver set where the body lives
	int idA = base->bodyIdA;
	int idB = base->bodyIdB;

	b2World* world = context->world;

	b2Body* bodyA = b2Array_Get( world->bodies, idA );
	b2Body* bodyB = b2Array_Get( world->bodies, idB );

	B2_ASSERT( bodyA->setIndex == b2_awakeSet || bodyB->setIndex == b2_awakeSet );
	b2SolverSet* setA = b2Array_Get( world->solverSets, bodyA->setIndex );
	b2SolverSet* setB = b2Array_Get( world->solverSets, bodyB->setIndex );

	int localIndexA = bodyA->localIndex;
	int localIndexB = bodyB->localIndex;

	b2BodySim* bodySimA = b2Array_Get( setA->bodySims, localIndexA );
	b2BodySim* bodySimB = b2Array_Get( setB->bodySims, localIndexB );

	float mA = bodySimA->invMass;
	float iA = bodySimA->invInertia;
	float mB = bodySimB->invMass;
	float iB = bodySimB->invInertia;

	base->invMassA = mA;
	base->invMassB = mB;
	base->invIA = iA;
	base->invIB = iB;

	b2WeldJoint* joint = &base->weldJoint;
	joint->indexA = bodyA->setIndex == b2_awakeSet ? localIndexA : B2_NULL_INDEX;
	joint->indexB = bodyB->setIndex == b2_awakeSet ? localIndexB : B2_NULL_INDEX;

	// Compute joint anchor frames with world space rotation, relative to center of mass
	joint->frameA.q = b2MulRot( bodySimA->transform.q, base->localFrameA.q );
	joint->frameA.p = b2RotateVector( bodySimA->transform.q, b2Sub( base->localFrameA.p, bodySimA->localCenter ) );
	joint->frameB.q = b2MulRot( bodySimB->transform.q, base->localFrameB.q );
	joint->frameB.p = b2RotateVector( bodySimB->transform.q, b2Sub( base->localFrameB.p, bodySimB->localCenter ) );

	// Compute the initial center delta. Incremental position updates are relative to this.
	joint->deltaCenter = b2Sub( bodySimB->center, bodySimA->center );

	float ka = iA + iB;
	joint->axialMass = ka > 0.0f ? 1.0f / ka : 0.0f;

	if ( joint->linearHertz == 0.0f )
	{
		joint->linearSpring = base->constraintSoftness;
	}
	else
	{
		joint->linearSpring = b2MakeSoft( joint->linearHertz, joint->linearDampingRatio, context->h );
	}

	if ( joint->angularHertz == 0.0f )
	{
		joint->angularSpring = base->constraintSoftness;
	}
	else
	{
		joint->angularSpring = b2MakeSoft( joint->angularHertz, joint->angularDampingRatio, context->h );
	}

	if ( context->enableWarmStarting == false )
	{
		joint->linearImpulse = b2Vec2_zero;
		joint->angularImpulse = 0.0f;
	}
}

void b2WarmStartWeldJoint( b2JointSim* base, b2StepContext* context )
{
	float mA = base->invMassA;
	float mB = base->invMassB;
	float iA = base->invIA;
	float iB = base->invIB;

	// dummy state for static bodies
	b2BodyState dummyState = b2_identityBodyState;

	b2WeldJoint* joint = &base->weldJoint;

	b2BodyState* stateA = joint->indexA == B2_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b2BodyState* stateB = joint->indexB == B2_NULL_INDEX ? &dummyState : context->states + joint->indexB;

	b2Vec2 rA = b2RotateVector( stateA->deltaRotation, joint->frameA.p );
	b2Vec2 rB = b2RotateVector( stateB->deltaRotation, joint->frameB.p );

	if ( stateA->flags & b2_dynamicFlag )
	{
		stateA->linearVelocity = b2MulSub( stateA->linearVelocity, mA, joint->linearImpulse );
		stateA->angularVelocity -= iA * ( b2Cross( rA, joint->linearImpulse ) + joint->angularImpulse );
	}

	if ( stateB->flags & b2_dynamicFlag )
	{
		stateB->linearVelocity = b2MulAdd( stateB->linearVelocity, mB, joint->linearImpulse );
		stateB->angularVelocity += iB * ( b2Cross( rB, joint->linearImpulse ) + joint->angularImpulse );
	}
}

void b2SolveWeldJoint( b2JointSim* base, b2StepContext* context, bool useBias )
{
	B2_ASSERT( base->type == b2_weldJoint );

	float mA = base->invMassA;
	float mB = base->invMassB;
	float iA = base->invIA;
	float iB = base->invIB;

	// dummy state for static bodies
	b2BodyState dummyState = b2_identityBodyState;

	b2WeldJoint* joint = &base->weldJoint;

	b2BodyState* stateA = joint->indexA == B2_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b2BodyState* stateB = joint->indexB == B2_NULL_INDEX ? &dummyState : context->states + joint->indexB;

	b2Vec2 vA = stateA->linearVelocity;
	float wA = stateA->angularVelocity;
	b2Vec2 vB = stateB->linearVelocity;
	float wB = stateB->angularVelocity;

	bool solved = false;
	if ( joint->enableBlockSolve && joint->linearHertz == 0.0f && joint->angularHertz == 0.0f )
	{
		// Block solving requires matching softness for all three constraint rows.
		// Hard-hard welds use the same base constraint softness for linear and angular correction.
		B2_ASSERT( joint->linearSpring.biasRate == joint->angularSpring.biasRate );
		B2_ASSERT( joint->linearSpring.massScale == joint->angularSpring.massScale );
		B2_ASSERT( joint->linearSpring.impulseScale == joint->angularSpring.impulseScale );

		// J = [-I -r1_skew I r2_skew]
		//     [ 0       -1 0       1]
		// r_skew = [-ry; rx]
		b2Vec2 rA = b2RotateVector( stateA->deltaRotation, joint->frameA.p );
		b2Vec2 rB = b2RotateVector( stateB->deltaRotation, joint->frameB.p );

		b2WeldMat33 K;
		K.cx.x = mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB;
		K.cy.x = -rA.y * rA.x * iA - rB.y * rB.x * iB;
		K.cz.x = -rA.y * iA - rB.y * iB;
		K.cx.y = K.cy.x;
		K.cy.y = mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB;
		K.cz.y = rA.x * iA + rB.x * iB;
		K.cx.z = K.cz.x;
		K.cy.z = K.cz.y;
		K.cz.z = iA + iB;

		b2WeldVec3 bias = { 0.0f, 0.0f, 0.0f };
		float massScale = 1.0f;
		float impulseScale = 0.0f;
		if ( useBias )
		{
			b2Vec2 dcA = stateA->deltaPosition;
			b2Vec2 dcB = stateB->deltaPosition;
			b2Vec2 jointTranslation = b2Add( b2Add( b2Sub( dcB, dcA ), b2Sub( rB, rA ) ), joint->deltaCenter );
			bias.x = joint->linearSpring.biasRate * jointTranslation.x;
			bias.y = joint->linearSpring.biasRate * jointTranslation.y;

			b2Rot qA = b2MulRot( stateA->deltaRotation, joint->frameA.q );
			b2Rot qB = b2MulRot( stateB->deltaRotation, joint->frameB.q );
			b2Rot relQ = b2InvMulRot( qA, qB );
			bias.z = joint->angularSpring.biasRate * b2Rot_GetAngle( relQ );

			massScale = joint->linearSpring.massScale;
			impulseScale = joint->linearSpring.impulseScale;
		}

		b2Vec2 Cdot1 = b2Sub( b2Add( vB, b2CrossSV( wB, rB ) ), b2Add( vA, b2CrossSV( wA, rA ) ) );
		float Cdot2 = wB - wA;
		b2WeldVec3 Cdot = { Cdot1.x + bias.x, Cdot1.y + bias.y, Cdot2 + bias.z };

		b2WeldVec3 blockImpulse;
		if ( b2SolveWeld33( &K, Cdot, &blockImpulse ) )
		{
			b2Vec2 linearImpulse = {
				-massScale * blockImpulse.x - impulseScale * joint->linearImpulse.x,
				-massScale * blockImpulse.y - impulseScale * joint->linearImpulse.y,
			};
			float angularImpulse = -massScale * blockImpulse.z - impulseScale * joint->angularImpulse;

			joint->linearImpulse = b2Add( joint->linearImpulse, linearImpulse );
			joint->angularImpulse += angularImpulse;

			vA = b2MulSub( vA, mA, linearImpulse );
			wA -= iA * ( b2Cross( rA, linearImpulse ) + angularImpulse );
			vB = b2MulAdd( vB, mB, linearImpulse );
			wB += iB * ( b2Cross( rB, linearImpulse ) + angularImpulse );
			solved = true;
		}
	}

	if ( solved == false )
	{
		// angular constraint
		{
			b2Rot qA = b2MulRot( stateA->deltaRotation, joint->frameA.q );
			b2Rot qB = b2MulRot( stateB->deltaRotation, joint->frameB.q );
			b2Rot relQ = b2InvMulRot( qA, qB );
			float jointAngle = b2Rot_GetAngle( relQ );

			float bias = 0.0f;
			float massScale = 1.0f;
			float impulseScale = 0.0f;
			if ( useBias || joint->angularHertz > 0.0f )
			{
				float C = jointAngle;
				bias = joint->angularSpring.biasRate * C;
				massScale = joint->angularSpring.massScale;
				impulseScale = joint->angularSpring.impulseScale;
			}

			float Cdot = wB - wA;
			float impulse = -massScale * joint->axialMass * ( Cdot + bias ) - impulseScale * joint->angularImpulse;
			joint->angularImpulse += impulse;

			wA -= iA * impulse;
			wB += iB * impulse;
		}

		// linear constraint
		{
			b2Vec2 rA = b2RotateVector( stateA->deltaRotation, joint->frameA.p );
			b2Vec2 rB = b2RotateVector( stateB->deltaRotation, joint->frameB.p );

			b2Vec2 bias = b2Vec2_zero;
			float massScale = 1.0f;
			float impulseScale = 0.0f;
			if ( useBias || joint->linearHertz > 0.0f )
			{
				b2Vec2 dcA = stateA->deltaPosition;
				b2Vec2 dcB = stateB->deltaPosition;
				b2Vec2 C = b2Add( b2Add( b2Sub( dcB, dcA ), b2Sub( rB, rA ) ), joint->deltaCenter );

				bias = b2MulSV( joint->linearSpring.biasRate, C );
				massScale = joint->linearSpring.massScale;
				impulseScale = joint->linearSpring.impulseScale;
			}

			b2Vec2 Cdot = b2Sub( b2Add( vB, b2CrossSV( wB, rB ) ), b2Add( vA, b2CrossSV( wA, rA ) ) );

			b2Mat22 K;
			K.cx.x = mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB;
			K.cy.x = -rA.y * rA.x * iA - rB.y * rB.x * iB;
			K.cx.y = K.cy.x;
			K.cy.y = mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB;
			b2Vec2 b = b2Solve22( K, b2Add( Cdot, bias ) );

			b2Vec2 impulse = {
				-massScale * b.x - impulseScale * joint->linearImpulse.x,
				-massScale * b.y - impulseScale * joint->linearImpulse.y,
			};

			joint->linearImpulse = b2Add( joint->linearImpulse, impulse );

			vA = b2MulSub( vA, mA, impulse );
			wA -= iA * b2Cross( rA, impulse );
			vB = b2MulAdd( vB, mB, impulse );
			wB += iB * b2Cross( rB, impulse );
		}
	}

	B2_ASSERT( b2IsValidVec2( vA ) );
	B2_ASSERT( b2IsValidFloat( wA ) );
	B2_ASSERT( b2IsValidVec2( vB ) );
	B2_ASSERT( b2IsValidFloat( wB ) );

	if ( stateA->flags & b2_dynamicFlag )
	{
		stateA->linearVelocity = vA;
		stateA->angularVelocity = wA;
	}

	if ( stateB->flags & b2_dynamicFlag )
	{
		stateB->linearVelocity = vB;
		stateB->angularVelocity = wB;
	}
}

#if 0
void b2DumpWeldJoint()
{
	int32 indexA = bodyA->islandIndex;
	int32 indexB = bodyB->islandIndex;

	b2Dump("  b2WeldJointDef jd;\n");
	b2Dump("  jd.bodyA = sims[%d];\n", indexA);
	b2Dump("  jd.bodyB = sims[%d];\n", indexB);
	b2Dump("  jd.collideConnected = bool(%d);\n", collideConnected);
	b2Dump("  jd.localAnchorA.Set(%.9g, %.9g);\n", localAnchorA.x, localAnchorA.y);
	b2Dump("  jd.localAnchorB.Set(%.9g, %.9g);\n", localAnchorB.x, localAnchorB.y);
	b2Dump("  jd.referenceAngle = %.9g;\n", referenceAngle);
	b2Dump("  jd.stiffness = %.9g;\n", stiffness);
	b2Dump("  jd.damping = %.9g;\n", damping);
	b2Dump("  joints[%d] = world->CreateJoint(&jd);\n", index);
}
#endif

void b2DrawWeldJoint( b2DebugDraw* draw, b2JointSim* base, b2Transform transformA, b2Transform transformB, float drawScale )
{
	B2_ASSERT( base->type == b2_weldJoint );

	b2Transform frameA = b2MulTransforms( transformA, base->localFrameA );
	b2Transform frameB = b2MulTransforms( transformB, base->localFrameB );

	b2Polygon box = b2MakeBox( 0.25f * drawScale, 0.125f * drawScale );

	b2Vec2 points[4];

	for ( int i = 0; i < 4; ++i )
	{
		points[i] = b2TransformPoint( frameA, box.vertices[i] );
	}
	draw->DrawPolygonFcn( points, 4, b2_colorDarkOrange, draw->context );

	for ( int i = 0; i < 4; ++i )
	{
		points[i] = b2TransformPoint( frameB, box.vertices[i] );
	}

	draw->DrawPolygonFcn( points, 4, b2_colorDarkCyan, draw->context );
}
