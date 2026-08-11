// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"

#include "body.h"
#include "joint.h"
#include "solver.h"

#include "box2d/box2d.h"

static b2JointSim MakeWeldJointSim( bool enableBlockSolve )
{
	b2JointSim base = { 0 };
	base.type = b2_weldJoint;
	base.invMassA = 1.0f;
	base.invMassB = 0.5f;
	base.invIA = 2.0f;
	base.invIB = 1.0f;

	b2WeldJoint* joint = &base.weldJoint;
	joint->indexA = 0;
	joint->indexB = 1;
	joint->frameA = (b2Transform){ { 1.2f, -0.4f }, b2Rot_identity };
	joint->frameB = (b2Transform){ { -0.7f, 0.8f }, b2Rot_identity };
	joint->axialMass = 1.0f / ( base.invIA + base.invIB );
	joint->enableBlockSolve = enableBlockSolve;

	return base;
}

static b2BodyState MakeBodyState( b2Vec2 linearVelocity, float angularVelocity )
{
	b2BodyState state = b2_identityBodyState;
	state.linearVelocity = linearVelocity;
	state.angularVelocity = angularVelocity;
	state.flags = b2_dynamicFlag;
	return state;
}

static b2Vec2 GetWeldPointVelocity( const b2BodyState* state, b2Vec2 localAnchor )
{
	b2Vec2 r = b2RotateVector( state->deltaRotation, localAnchor );
	return b2Add( state->linearVelocity, b2CrossSV( state->angularVelocity, r ) );
}

static int WeldBlockSolveResidualTest( void )
{
	b2JointSim base = MakeWeldJointSim( true );
	b2JointSim splitBase = MakeWeldJointSim( false );

	b2BodyState states[2];
	states[0] = MakeBodyState( (b2Vec2){ 3.0f, -2.0f }, 4.0f );
	states[1] = MakeBodyState( (b2Vec2){ -1.0f, 5.0f }, -2.0f );
	b2BodyState splitStates[2] = { states[0], states[1] };

	b2StepContext context = { 0 };
	context.states = states;

	b2SolveWeldJoint( &base, &context, false );

	context.states = splitStates;
	b2SolveWeldJoint( &splitBase, &context, false );

	b2Vec2 velocityA = GetWeldPointVelocity( states + 0, base.weldJoint.frameA.p );
	b2Vec2 velocityB = GetWeldPointVelocity( states + 1, base.weldJoint.frameB.p );
	b2Vec2 linearResidual = b2Sub( velocityB, velocityA );
	float angularResidual = states[1].angularVelocity - states[0].angularVelocity;
	float splitAngularResidual = splitStates[1].angularVelocity - splitStates[0].angularVelocity;

	ENSURE_SMALL( linearResidual.x, 0.00001f );
	ENSURE_SMALL( linearResidual.y, 0.00001f );
	ENSURE_SMALL( angularResidual, 0.00001f );
	ENSURE( b2AbsFloat( splitAngularResidual ) > 0.01f );

	return 0;
}

static int WeldBlockSolveFallbackTest( void )
{
	b2JointSim base = MakeWeldJointSim( true );
	base.invIA = 0.0f;
	base.invIB = 0.0f;
	base.weldJoint.axialMass = 0.0f;

	b2BodyState states[2];
	states[0] = MakeBodyState( (b2Vec2){ 3.0f, -2.0f }, 0.0f );
	states[1] = MakeBodyState( (b2Vec2){ -1.0f, 5.0f }, 0.0f );

	b2StepContext context = { 0 };
	context.states = states;

	b2SolveWeldJoint( &base, &context, false );

	b2Vec2 velocityA = GetWeldPointVelocity( states + 0, base.weldJoint.frameA.p );
	b2Vec2 velocityB = GetWeldPointVelocity( states + 1, base.weldJoint.frameB.p );
	b2Vec2 linearResidual = b2Sub( velocityB, velocityA );

	ENSURE_SMALL( linearResidual.x, 0.00001f );
	ENSURE_SMALL( linearResidual.y, 0.00001f );
	ENSURE( b2IsValidVec2( states[0].linearVelocity ) );
	ENSURE( b2IsValidVec2( states[1].linearVelocity ) );

	return 0;
}

static int WeldBlockSolveApiTest( void )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	worldDef.gravity = b2Vec2_zero;
	b2WorldId worldId = b2CreateWorld( &worldDef );

	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	b2BodyId bodyIdA = b2CreateBody( worldId, &bodyDef );
	bodyDef.position = (b2Vec2){ 1.0f, 0.0f };
	b2BodyId bodyIdB = b2CreateBody( worldId, &bodyDef );

	b2ShapeDef shapeDef = b2DefaultShapeDef();
	shapeDef.density = 1.0f;
	b2Circle circle = { b2Vec2_zero, 0.5f };
	b2CreateCircleShape( bodyIdA, &shapeDef, &circle );
	b2CreateCircleShape( bodyIdB, &shapeDef, &circle );

	b2WeldJointDef jointDef = b2DefaultWeldJointDef();
	jointDef.base.bodyIdA = bodyIdA;
	jointDef.base.bodyIdB = bodyIdB;
	jointDef.base.localFrameA.p = (b2Vec2){ 0.5f, 0.0f };
	jointDef.base.localFrameB.p = (b2Vec2){ -0.5f, 0.0f };
	b2JointId jointId = b2CreateWeldJoint( worldId, &jointDef );

	ENSURE( b2WeldJoint_IsBlockSolveEnabled( jointId ) == false );
	b2WeldJoint_EnableBlockSolve( jointId, true );
	ENSURE( b2WeldJoint_IsBlockSolveEnabled( jointId ) == true );

	// The setting remains enabled but is inactive while either axis uses a spring.
	b2WeldJoint_SetAngularHertz( jointId, 2.0f );
	b2World_Step( worldId, 1.0f / 60.0f, 4, NULL, NULL );
	ENSURE( b2WeldJoint_IsBlockSolveEnabled( jointId ) == true );

	b2WeldJoint_SetAngularHertz( jointId, 0.0f );
	b2World_Step( worldId, 1.0f / 60.0f, 4, NULL, NULL );

	b2WeldJoint_EnableBlockSolve( jointId, false );
	ENSURE( b2WeldJoint_IsBlockSolveEnabled( jointId ) == false );

	b2DestroyWorld( worldId );
	return 0;
}

int JointTest( void )
{
	RUN_SUBTEST( WeldBlockSolveResidualTest );
	RUN_SUBTEST( WeldBlockSolveFallbackTest );
	RUN_SUBTEST( WeldBlockSolveApiTest );

	return 0;
}
