// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"
#include "benchmarks.h"

#include "box2d/box2d.h"
#include "box2d/collision.h"
#include "box2d/constants.h"
#include "box2d/math_functions.h"

#include <stdio.h>

// This is a simple example of building and running a simulation
// using Box2D. Here we create a large ground box and a small dynamic
// box.
// There are no graphics for this example. Box2D is meant to be used
// with your rendering engine in your game engine.
int HelloWorld( void )
{
	// Construct a world object, which will hold and simulate the rigid bodies.
	b2WorldDef worldDef = b2DefaultWorldDef();
	worldDef.gravity = (b2Vec2){ 0.0f, -10.0f };

	b2WorldId worldId = b2CreateWorld( &worldDef );
	ENSURE( b2World_IsValid( worldId ) );

	// Define the ground body.
	b2BodyDef groundBodyDef = b2DefaultBodyDef();
	groundBodyDef.position = (b2Pos){ 0.0f, -10.0f };

	// Call the body factory which allocates memory for the ground body
	// from a pool and creates the ground box shape (also from a pool).
	// The body is also added to the world.
	b2BodyId groundId = b2CreateBody( worldId, &groundBodyDef );
	ENSURE( b2Body_IsValid( groundId ) );

	// Define the ground box shape. The extents are the half-widths of the box.
	b2Polygon groundBox = b2MakeBox( 50.0f, 10.0f );

	// Add the box shape to the ground body.
	b2ShapeDef groundShapeDef = b2DefaultShapeDef();
	b2CreatePolygonShape( groundId, &groundShapeDef, &groundBox );

	// Define the dynamic body. We set its position and call the body factory.
	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	bodyDef.position = (b2Pos){ 0.0f, 4.0f };

	b2BodyId bodyId = b2CreateBody( worldId, &bodyDef );

	// Define another box shape for our dynamic body.
	b2Polygon dynamicBox = b2MakeBox( 1.0f, 1.0f );

	// Define the dynamic body shape
	b2ShapeDef shapeDef = b2DefaultShapeDef();

	// Set the box density to be non-zero, so it will be dynamic.
	shapeDef.density = 1.0f;

	// Override the default friction.
	shapeDef.material.friction = 0.3f;

	// Add the shape to the body.
	b2CreatePolygonShape( bodyId, &shapeDef, &dynamicBox );

	// Prepare for simulation. Typically we use a time step of 1/60 of a
	// second (60Hz) and 4 sub-steps. This provides a high quality simulation
	// in most game scenarios.
	float timeStep = 1.0f / 60.0f;
	int subStepCount = 4;

	b2Pos position = b2Body_GetPosition( bodyId );
	b2Rot rotation = b2Body_GetRotation( bodyId );

	// This is our little game loop.
	for ( int i = 0; i < 90; ++i )
	{
		// Instruct the world to perform a single step of simulation.
		// It is generally best to keep the time step and iterations fixed.
		b2World_Step( worldId, timeStep, subStepCount, NULL, NULL );

		// Now print the position and angle of the body.
		position = b2Body_GetPosition( bodyId );
		rotation = b2Body_GetRotation( bodyId );

		// printf("%4.2f %4.2f %4.2f\n", position.x, position.y, b2Rot_GetAngle(rotation));
	}

	// When the world destructor is called, all bodies and joints are freed. This can
	// create orphaned ids, so be careful about your world management.
	b2DestroyWorld( worldId );

	ENSURE( b2AbsFloat( position.x ) < 0.01f );
	ENSURE( b2AbsFloat( position.y - 1.00f ) < 0.01f );
	ENSURE( b2AbsFloat( b2Rot_GetAngle( rotation ) ) < 0.01f );

	return 0;
}

int EmptyWorld( void )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	b2WorldId worldId = b2CreateWorld( &worldDef );
	ENSURE( b2World_IsValid( worldId ) == true );

	float timeStep = 1.0f / 60.0f;
	int32_t subStepCount = 1;

	for ( int32_t i = 0; i < 60; ++i )
	{
		b2World_Step( worldId, timeStep, subStepCount, NULL, NULL );
	}

	b2DestroyWorld( worldId );

	ENSURE( b2World_IsValid( worldId ) == false );

	return 0;
}

#define BODY_COUNT 10
int DestroyAllBodiesWorld( void )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	b2WorldId worldId = b2CreateWorld( &worldDef );
	ENSURE( b2World_IsValid( worldId ) == true );

	int count = 0;
	bool creating = true;

	b2BodyId bodyIds[BODY_COUNT];
	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	b2Polygon square = b2MakeSquare( 0.5f );

	for ( int32_t i = 0; i < 2 * BODY_COUNT + 10; ++i )
	{
		if ( creating )
		{
			if ( count < BODY_COUNT )
			{
				bodyIds[count] = b2CreateBody( worldId, &bodyDef );

				b2ShapeDef shapeDef = b2DefaultShapeDef();
				b2CreatePolygonShape( bodyIds[count], &shapeDef, &square );
				count += 1;
			}
			else
			{
				creating = false;
			}
		}
		else if ( count > 0 )
		{
			b2DestroyBody( bodyIds[count - 1] );
			bodyIds[count - 1] = b2_nullBodyId;
			count -= 1;
		}

		b2World_Step( worldId, 1.0f / 60.0f, 3, NULL, NULL );
	}

	b2Counters counters = b2World_GetCounters( worldId );
	ENSURE( counters.bodyCount == 0 );

	b2DestroyWorld( worldId );

	ENSURE( b2World_IsValid( worldId ) == false );

	return 0;
}

static int TestIsValid( void )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	b2WorldId worldId = b2CreateWorld( &worldDef );
	ENSURE( b2World_IsValid( worldId ) );

	b2BodyDef bodyDef = b2DefaultBodyDef();

	b2BodyId bodyId1 = b2CreateBody( worldId, &bodyDef );
	ENSURE( b2Body_IsValid( bodyId1 ) == true );

	b2BodyId bodyId2 = b2CreateBody( worldId, &bodyDef );
	ENSURE( b2Body_IsValid( bodyId2 ) == true );

	b2DestroyBody( bodyId1 );
	ENSURE( b2Body_IsValid( bodyId1 ) == false );

	b2DestroyBody( bodyId2 );
	ENSURE( b2Body_IsValid( bodyId2 ) == false );

	b2DestroyWorld( worldId );

	ENSURE( b2World_IsValid( worldId ) == false );
	ENSURE( b2Body_IsValid( bodyId2 ) == false );
	ENSURE( b2Body_IsValid( bodyId1 ) == false );

	return 0;
}

#define WORLD_COUNT ( B2_MAX_WORLDS / 2 )

int TestWorldRecycle( void )
{
	_Static_assert( WORLD_COUNT > 0, "world count" );

	int count = 100;

	b2WorldId worldIds[WORLD_COUNT];

	for ( int i = 0; i < count; ++i )
	{
		b2WorldDef worldDef = b2DefaultWorldDef();
		for ( int j = 0; j < WORLD_COUNT; ++j )
		{
			worldIds[j] = b2CreateWorld( &worldDef );
			ENSURE( b2World_IsValid( worldIds[j] ) == true );

			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2CreateBody( worldIds[j], &bodyDef );
		}

		for ( int j = 0; j < WORLD_COUNT; ++j )
		{
			float timeStep = 1.0f / 60.0f;
			int subStepCount = 1;

			for ( int k = 0; k < 10; ++k )
			{
				b2World_Step( worldIds[j], timeStep, subStepCount, NULL, NULL );
			}
		}

		for ( int j = WORLD_COUNT - 1; j >= 0; --j )
		{
			b2DestroyWorld( worldIds[j] );
			ENSURE( b2World_IsValid( worldIds[j] ) == false );
			worldIds[j] = b2_nullWorldId;
		}
	}

	return 0;
}

static bool CustomFilter( b2ShapeId shapeIdA, b2ShapeId shapeIdB, void* context )
{
	(void)shapeIdA;
	(void)shapeIdB;
	ENSURE( context == NULL );
	return true;
}

static bool PreSolveStatic( b2ShapeId shapeIdA, b2ShapeId shapeIdB, b2Pos point, b2Vec2 normal, float separation,
							void* context )
{
	(void)shapeIdA;
	(void)shapeIdB;
	(void)point;
	(void)normal;
	(void)separation;
	ENSURE( context == NULL );
	return false;
}

#if defined( _MSC_VER )
__declspec( thread ) static bool sPreSolveStepThread;
#else
static _Thread_local bool sPreSolveStepThread;
#endif

typedef struct PreSolveThreadContext
{
	int callbackCount;
	bool wrongThread;
	float minimumSeparation;
	bool disableGlobalOnFirstCallback;
	b2WorldId worldId;
} PreSolveThreadContext;

static bool PreSolveThreadCheck( b2ShapeId shapeIdA, b2ShapeId shapeIdB, b2Vec2 point, b2Vec2 normal, float separation,
								 void* context )
{
	(void)shapeIdA;
	(void)shapeIdB;
	(void)point;
	(void)normal;
	PreSolveThreadContext* threadContext = context;
	threadContext->callbackCount += 1;
	threadContext->wrongThread |= sPreSolveStepThread == false;
	threadContext->minimumSeparation = b2MinFloat( threadContext->minimumSeparation, separation );
	if ( threadContext->disableGlobalOnFirstCallback && threadContext->callbackCount == 1 )
	{
		b2World_EnableGlobalPreSolveEvents( threadContext->worldId, false );
	}
	return true;
}

static int PreSolveCallingThreadTest( void )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	worldDef.workerCount = 4;
	worldDef.gravity = (b2Vec2){ 0.0f, -10.0f };
	b2WorldId worldId = b2CreateWorld( &worldDef );
	ENSURE( b2World_GetWorkerCount( worldId ) == 4 );

	PreSolveThreadContext context = { 0 };
	b2World_SetPreSolveCallback( worldId, PreSolveThreadCheck, &context );

	b2BodyDef groundDef = b2DefaultBodyDef();
	b2BodyId groundId = b2CreateBody( worldId, &groundDef );
	b2Polygon groundBox = b2MakeBox( 5.0f, 0.5f );
	b2ShapeDef groundShapeDef = b2DefaultShapeDef();
	b2CreatePolygonShape( groundId, &groundShapeDef, &groundBox );

	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	bodyDef.position = (b2Vec2){ 0.0f, 0.75f };
	b2BodyId bodyId = b2CreateBody( worldId, &bodyDef );
	b2Polygon box = b2MakeBox( 0.5f, 0.5f );
	b2ShapeDef shapeDef = b2DefaultShapeDef();
	shapeDef.enablePreSolveEvents = true;
	b2ShapeId shapeId = b2CreatePolygonShape( bodyId, &shapeDef, &box );

	sPreSolveStepThread = true;
	b2World_Step( worldId, 1.0f / 60.0f, 4, NULL, NULL );
	sPreSolveStepThread = false;

	ENSURE( context.callbackCount > 0 );
	ENSURE( context.wrongThread == false );
	ENSURE( context.minimumSeparation < 0.0f );

	// Verify the world-wide mode invokes the same calling-thread callback even
	// when no individual shape has pre-solve events enabled.
	b2Shape_EnablePreSolveEvents( shapeId, false );
	b2World_EnableGlobalPreSolveEvents( worldId, true );
	ENSURE( b2World_AreGlobalPreSolveEventsEnabled( worldId ) == true );
	context.callbackCount = 0;
	sPreSolveStepThread = true;
	b2World_Step( worldId, 1.0f / 60.0f, 4, NULL, NULL );
	sPreSolveStepThread = false;

	ENSURE( context.callbackCount > 0 );
	ENSURE( context.wrongThread == false );

	b2World_EnableGlobalPreSolveEvents( worldId, false );
	ENSURE( b2World_AreGlobalPreSolveEventsEnabled( worldId ) == false );
	b2DestroyWorld( worldId );
	return 0;
}

static int PreSolveCcdCallingThreadTest( void )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	worldDef.workerCount = 4;
	worldDef.gravity = b2Vec2_zero;
	b2WorldId worldId = b2CreateWorld( &worldDef );

	PreSolveThreadContext context = { 0 };
	b2World_SetPreSolveCallback( worldId, PreSolveThreadCheck, &context );
	b2World_EnableGlobalPreSolveEvents( worldId, true );

	// A thin wall and a fast bullet exercise the continuous-collision path. No
	// shape has enabled pre-solve events, so this also covers world-wide mode in
	// CCD rather than only the discrete contact update.
	b2BodyDef wallDef = b2DefaultBodyDef();
	b2BodyId wallId = b2CreateBody( worldId, &wallDef );
	b2Polygon wall = b2MakeBox( 0.05f, 2.0f );
	b2ShapeDef shapeDef = b2DefaultShapeDef();
	b2CreatePolygonShape( wallId, &shapeDef, &wall );

	b2BodyDef bulletDef = b2DefaultBodyDef();
	bulletDef.type = b2_dynamicBody;
	bulletDef.isBullet = true;
	bulletDef.position = (b2Vec2){ -1.0f, 0.0f };
	bulletDef.linearVelocity = (b2Vec2){ 100.0f, 0.0f };
	b2BodyId bulletId = b2CreateBody( worldId, &bulletDef );
	b2Circle circle = { b2Vec2_zero, 0.1f };
	b2CreateCircleShape( bulletId, &shapeDef, &circle );

	sPreSolveStepThread = true;
	b2World_Step( worldId, 1.0f / 60.0f, 4, NULL, NULL );
	sPreSolveStepThread = false;

	ENSURE( context.callbackCount > 0 );
	ENSURE( context.wrongThread == false );

	b2DestroyWorld( worldId );
	return 0;
}

static int PreSolveGlobalToggleTest( void )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	worldDef.workerCount = 4;
	worldDef.gravity = b2Vec2_zero;
	b2WorldId worldId = b2CreateWorld( &worldDef );

	PreSolveThreadContext context = { 0 };
	context.disableGlobalOnFirstCallback = true;
	context.worldId = worldId;
	b2World_SetPreSolveCallback( worldId, PreSolveThreadCheck, &context );
	b2World_EnableGlobalPreSolveEvents( worldId, true );

	b2BodyDef groundDef = b2DefaultBodyDef();
	b2BodyId groundId = b2CreateBody( worldId, &groundDef );
	b2Polygon ground = b2MakeBox( 5.0f, 0.5f );
	b2ShapeDef groundShapeDef = b2DefaultShapeDef();
	b2CreatePolygonShape( groundId, &groundShapeDef, &ground );

	b2Polygon box = b2MakeBox( 0.5f, 0.5f );
	b2ShapeDef shapeDef = b2DefaultShapeDef();
	shapeDef.enableContactEvents = true;
	for ( int i = 0; i < 2; ++i )
	{
		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.position = (b2Vec2){ i == 0 ? -1.0f : 1.0f, 0.75f };
		b2BodyId bodyId = b2CreateBody( worldId, &bodyDef );
		b2CreatePolygonShape( bodyId, &shapeDef, &box );
	}

	sPreSolveStepThread = true;
	b2World_Step( worldId, 1.0f / 60.0f, 4, NULL, NULL );
	sPreSolveStepThread = false;

	b2ContactEvents events = b2World_GetContactEvents( worldId );
	ENSURE( context.callbackCount == 1 );
	ENSURE( context.wrongThread == false );
	ENSURE( events.beginCount == 2 );
	ENSURE( b2World_AreGlobalPreSolveEventsEnabled( worldId ) == false );

	b2DestroyWorld( worldId );
	return 0;
}

// This test is here to ensure all API functions link correctly.
int TestWorldCoverage( void )
{
	b2WorldDef worldDef = b2DefaultWorldDef();

	b2WorldId worldId = b2CreateWorld( &worldDef );
	ENSURE( b2World_IsValid( worldId ) );

	b2World_EnableSleeping( worldId, true );
	b2World_EnableSleeping( worldId, false );
	bool flag = b2World_IsSleepingEnabled( worldId );
	ENSURE( flag == false );

	b2World_EnableContinuous( worldId, false );
	b2World_EnableContinuous( worldId, true );
	flag = b2World_IsContinuousEnabled( worldId );
	ENSURE( flag == true );

	b2World_SetRestitutionThreshold( worldId, 0.0f );
	b2World_SetRestitutionThreshold( worldId, 2.0f );
	float value = b2World_GetRestitutionThreshold( worldId );
	ENSURE( value == 2.0f );

	b2World_SetHitEventThreshold( worldId, 0.0f );
	b2World_SetHitEventThreshold( worldId, 100.0f );
	value = b2World_GetHitEventThreshold( worldId );
	ENSURE( value == 100.0f );

	b2World_SetCustomFilterCallback( worldId, CustomFilter, NULL );
	b2World_SetPreSolveCallback( worldId, PreSolveStatic, NULL );

	b2Vec2 g = { 1.0f, 2.0f };
	b2World_SetGravity( worldId, g );
	b2Vec2 v = b2World_GetGravity( worldId );
	ENSURE( v.x == g.x );
	ENSURE( v.y == g.y );

	b2ExplosionDef explosionDef = b2DefaultExplosionDef();
	b2World_Explode( worldId, &explosionDef );

	b2World_SetContactTuning( worldId, 10.0f, 2.0f, 4.0f );

	b2World_SetMaximumLinearSpeed( worldId, 10.0f );
	value = b2World_GetMaximumLinearSpeed( worldId );
	ENSURE( value == 10.0f );

	b2World_EnableWarmStarting( worldId, true );
	flag = b2World_IsWarmStartingEnabled( worldId );
	ENSURE( flag == true );

	int count = b2World_GetAwakeBodyCount( worldId );
	ENSURE( count == 0 );

	b2World_SetUserData( worldId, &value );
	void* userData = b2World_GetUserData( worldId );
	ENSURE( userData == &value );

	b2World_Step( worldId, 1.0f, 1, NULL, NULL);

	b2DestroyWorld( worldId );

	return 0;
}

static int TestSensor( void )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	b2WorldId worldId = b2CreateWorld( &worldDef );

	// Wall from x = 1 to x = 2
	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_staticBody;
	bodyDef.position.x = 1.5f;
	bodyDef.position.y = 11.0f;
	b2BodyId wallId = b2CreateBody( worldId, &bodyDef );
	b2Polygon box = b2MakeBox( 0.5f, 10.0f );
	b2ShapeDef shapeDef = b2DefaultShapeDef();
	shapeDef.enableSensorEvents = true;
	b2CreatePolygonShape( wallId, &shapeDef, &box );

	// Bullet fired towards the wall
	bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	bodyDef.isBullet = true;
	bodyDef.gravityScale = 0.0f;
	bodyDef.position = (b2Pos){ 7.39814f, 4.0f };
	bodyDef.linearVelocity = (b2Vec2){ -20.0f, 0.0f };
	b2BodyId bulletId = b2CreateBody( worldId, &bodyDef );
	shapeDef = b2DefaultShapeDef();
	shapeDef.isSensor = true;
	shapeDef.enableSensorEvents = true;
	b2Circle circle = { { 0.0f, 0.0f }, 0.1f };
	b2CreateCircleShape( bulletId, &shapeDef, &circle );

	int beginCount = 0;
	int endCount = 0;

	while ( true )
	{
		float timeStep = 1.0f / 60.0f;
		int subStepCount = 4;
		b2World_Step( worldId, timeStep, subStepCount, NULL, NULL );

		b2Pos bulletPos = b2Body_GetPosition( bulletId );
		// printf( "Bullet pos: %g %g\n", bulletPos.x, bulletPos.y );

		b2SensorEvents events = b2World_GetSensorEvents( worldId );

		if ( events.beginCount > 0 )
		{
			beginCount += 1;
		}

		if ( events.endCount > 0 )
		{
			endCount += 1;
		}

		if ( bulletPos.x < -1.0f )
		{
			break;
		}
	}

	b2DestroyWorld( worldId );

	ENSURE( beginCount == 1 );
	ENSURE( endCount == 1 );

	return 0;
}

static int TestSetWorkerCount( void )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	worldDef.workerCount = 1;
	b2WorldId worldId = b2CreateWorld( &worldDef );
	ENSURE( b2World_IsValid( worldId ) );
	ENSURE( b2World_GetWorkerCount( worldId ) == 1 );

	CreateJunkyard( worldId );
	StepJunkyard( worldId, 1 );

	b2World_SetWorkerCount( worldId, 4 );
	ENSURE( b2World_GetWorkerCount( worldId ) == 4 );

	StepJunkyard( worldId, 2 );

	b2World_SetWorkerCount( worldId, 4 );
	ENSURE( b2World_GetWorkerCount( worldId ) == 4 );

	StepJunkyard( worldId, 3 );

	b2World_SetWorkerCount( worldId, 0 );
	ENSURE( b2World_GetWorkerCount( worldId ) == 1 );

	StepJunkyard( worldId, 4 );

	b2World_SetWorkerCount( worldId, -5 );
	ENSURE( b2World_GetWorkerCount( worldId ) == 1 );

	StepJunkyard( worldId, 5 );

	b2World_SetWorkerCount( worldId, B2_MAX_WORKERS + 10 );
	ENSURE( b2World_GetWorkerCount( worldId ) == B2_MAX_WORKERS );

	StepJunkyard( worldId, 2 );

	b2DestroyWorld( worldId );

	return 0;
}

static int ChainSegmentShapeTest( void )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	worldDef.gravity = (b2Vec2){ 0.0f, -10.0f };
	b2WorldId worldId = b2CreateWorld( &worldDef );

	b2BodyDef bodyDef = b2DefaultBodyDef();
	b2BodyId groundId = b2CreateBody( worldId, &bodyDef );

	b2ChainSegment cs = { 0 };
	cs.ghost1 = (b2Vec2){ 2.0f, 0.0f };
	cs.segment.point1 = (b2Vec2){ 1.0f, 0.0f };
	cs.segment.point2 = (b2Vec2){ -1.0f, 0.0f };
	cs.ghost2 = (b2Vec2){ -2.0f, 0.0f };
	cs.chainId = 99;

	b2ShapeDef shapeDef = b2DefaultShapeDef();
	b2ShapeId orphanShape = b2CreateChainSegmentShape( groundId, &shapeDef, &cs );
	ENSURE( B2_IS_NON_NULL( orphanShape ) );

	ENSURE( b2Shape_GetType( orphanShape ) == b2_chainSegmentShape );

	b2ChainId parentChain = b2Shape_GetParentChain( orphanShape );
	ENSURE( B2_IS_NULL( parentChain ) );

	b2ChainSegment got = b2Shape_GetChainSegment( orphanShape );
	ENSURE_SMALL( got.ghost1.x - cs.ghost1.x, 1e-5f );
	ENSURE_SMALL( got.ghost1.y - cs.ghost1.y, 1e-5f );
	ENSURE_SMALL( got.segment.point1.x - cs.segment.point1.x, 1e-5f );
	ENSURE_SMALL( got.segment.point1.y - cs.segment.point1.y, 1e-5f );
	ENSURE_SMALL( got.segment.point2.x - cs.segment.point2.x, 1e-5f );
	ENSURE_SMALL( got.segment.point2.y - cs.segment.point2.y, 1e-5f );
	ENSURE_SMALL( got.ghost2.x - cs.ghost2.x, 1e-5f );
	ENSURE_SMALL( got.ghost2.y - cs.ghost2.y, 1e-5f );
	ENSURE( got.chainId == B2_NULL_INDEX );

	b2BodyDef dynamicDef = b2DefaultBodyDef();
	dynamicDef.type = b2_dynamicBody;
	dynamicDef.position = (b2Pos){ 0.0f, 2.0f };
	b2BodyId circleBodyId = b2CreateBody( worldId, &dynamicDef );
	b2Circle circle = { { 0.0f, 0.0f }, 0.5f };
	b2ShapeDef circleShapeDef = b2DefaultShapeDef();
	b2CreateCircleShape( circleBodyId, &circleShapeDef, &circle );

	for ( int i = 0; i < 120; ++i )
	{
		b2World_Step( worldId, 1.0f / 60.0f, 4, NULL, NULL );
	}

	b2Pos circlePos = b2Body_GetPosition( circleBodyId );
	ENSURE( circlePos.y > 0.0f );

	b2ChainSegment cs2 = { 0 };
	cs2.ghost1 = (b2Vec2){ 3.0f, 0.0f };
	cs2.segment.point1 = (b2Vec2){ 2.0f, 0.0f };
	cs2.segment.point2 = (b2Vec2){ -2.0f, 0.0f };
	cs2.ghost2 = (b2Vec2){ -3.0f, 0.0f };
	cs2.chainId = B2_NULL_INDEX;

	b2Shape_SetChainSegment( orphanShape, &cs2 );

	b2ChainSegment got2 = b2Shape_GetChainSegment( orphanShape );
	ENSURE_SMALL( got2.segment.point1.x - cs2.segment.point1.x, 1e-5f );
	ENSURE_SMALL( got2.segment.point1.y - cs2.segment.point1.y, 1e-5f );
	ENSURE_SMALL( got2.segment.point2.x - cs2.segment.point2.x, 1e-5f );
	ENSURE_SMALL( got2.segment.point2.y - cs2.segment.point2.y, 1e-5f );
	ENSURE_SMALL( got2.ghost1.x - cs2.ghost1.x, 1e-5f );
	ENSURE_SMALL( got2.ghost1.y - cs2.ghost1.y, 1e-5f );
	ENSURE_SMALL( got2.ghost2.x - cs2.ghost2.x, 1e-5f );
	ENSURE_SMALL( got2.ghost2.y - cs2.ghost2.y, 1e-5f );
	ENSURE( got2.chainId == B2_NULL_INDEX );

	b2ChainId parentChain2 = b2Shape_GetParentChain( orphanShape );
	ENSURE( B2_IS_NULL( parentChain2 ) );

	b2BodyId convBody = b2CreateBody( worldId, &bodyDef );
	b2Circle convCircle = { { 0.0f, 0.0f }, 0.25f };
	b2ShapeId convShape = b2CreateCircleShape( convBody, &shapeDef, &convCircle );
	ENSURE( b2Shape_GetType( convShape ) == b2_circleShape );

	b2Shape_SetChainSegment( convShape, &cs2 );

	ENSURE( b2Shape_GetType( convShape ) == b2_chainSegmentShape );
	b2ChainSegment got3 = b2Shape_GetChainSegment( convShape );
	ENSURE_SMALL( got3.ghost1.x - cs2.ghost1.x, 1e-5f );
	ENSURE_SMALL( got3.ghost1.y - cs2.ghost1.y, 1e-5f );
	ENSURE_SMALL( got3.ghost2.x - cs2.ghost2.x, 1e-5f );
	ENSURE_SMALL( got3.ghost2.y - cs2.ghost2.y, 1e-5f );
	ENSURE( got3.chainId == B2_NULL_INDEX );

	b2ChainId parentChain3 = b2Shape_GetParentChain( convShape );
	ENSURE( B2_IS_NULL( parentChain3 ) );

	b2DestroyShape( orphanShape, true );
	b2DestroyWorld( worldId );

	return 0;
}

static int DeferredMassFlagSyncTest( void )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	b2WorldId worldId = b2CreateWorld( &worldDef );

	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	b2BodyId bodyId = b2CreateBody( worldId, &bodyDef );

	b2ShapeDef shapeDef = b2DefaultShapeDef();
	shapeDef.updateBodyMass = false;

	b2Circle circle = { { 0.0f, 0.0f }, 0.5f };
	b2CreateCircleShape( bodyId, &shapeDef, &circle );

	b2Body_ApplyMassFromShapes( bodyId );

	b2World_Step( worldId, 1.0f / 60.0f, 4, NULL, NULL );

	b2DestroyWorld( worldId );
	return 0;
}

static int EnableSleepFlagSyncTest( void )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	b2WorldId worldId = b2CreateWorld( &worldDef );

	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	bodyDef.enableSleep = false;
	b2BodyId bodyId = b2CreateBody( worldId, &bodyDef );

	ENSURE( b2Body_IsSleepEnabled( bodyId ) == false );

	b2Body_EnableSleep( bodyId, true );
	ENSURE( b2Body_IsSleepEnabled( bodyId ) == true );

	b2World_Step( worldId, 1.0f / 60.0f, 4, NULL, NULL );

	b2DestroyWorld( worldId );
	return 0;
}

static int EnableContactRecyclingTest( void )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	b2WorldId worldId = b2CreateWorld( &worldDef );

	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;

	// Default is enabled
	b2BodyId bodyA = b2CreateBody( worldId, &bodyDef );
	ENSURE( b2Body_IsContactRecyclingEnabled( bodyA ) == true );

	b2Body_EnableContactRecycling( bodyA, false );
	ENSURE( b2Body_IsContactRecyclingEnabled( bodyA ) == false );

	b2Body_EnableContactRecycling( bodyA, true );
	ENSURE( b2Body_IsContactRecyclingEnabled( bodyA ) == true );

	// Per-def opt-out at creation
	bodyDef.enableContactRecycling = false;
	b2BodyId bodyB = b2CreateBody( worldId, &bodyDef );
	ENSURE( b2Body_IsContactRecyclingEnabled( bodyB ) == false );

	// Stepping after toggling must not trip the flag-sync validator
	b2World_Step( worldId, 1.0f / 60.0f, 4, NULL, NULL );

	b2DestroyWorld( worldId );
	return 0;
}

static int SetBulletDriftTest( void )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	b2WorldId worldId = b2CreateWorld( &worldDef );

	{
		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.isBullet = false;
		b2BodyId bodyId = b2CreateBody( worldId, &bodyDef );

		ENSURE( b2Body_IsBullet( bodyId ) == false );

		b2Body_SetBullet( bodyId, true );
		ENSURE( b2Body_IsBullet( bodyId ) == true );

		b2MotionLocks locks = { 0 };
		locks.linearX = true;
		b2Body_SetMotionLocks( bodyId, locks );

		ENSURE( b2Body_IsBullet( bodyId ) == true );
	}

	{
		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.isBullet = true;
		b2BodyId bodyId = b2CreateBody( worldId, &bodyDef );

		ENSURE( b2Body_IsBullet( bodyId ) == true );

		b2Body_SetBullet( bodyId, false );
		ENSURE( b2Body_IsBullet( bodyId ) == false );

		b2MotionLocks locks = { 0 };
		locks.linearX = true;
		b2Body_SetMotionLocks( bodyId, locks );

		ENSURE( b2Body_IsBullet( bodyId ) == false );
	}

	b2DestroyWorld( worldId );
	return 0;
}

// Slide a rotation-locked box (or circle) across 10 flush tiles towards a wall on two lanes (floor at y = 0 and
// y = 3). bodyFilter enables the per body seam filter for the slider on each lane. Writes the final positions.
static int SlideAcrossTiles( bool worldFilter, const bool bodyFilter[2], bool useCircle, float speed, b2Vec2 positions[2] )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	worldDef.workerCount = 1;
	b2WorldId worldId = b2CreateWorld( &worldDef );
	b2World_EnableSeamContactFilter( worldId, worldFilter );

	b2BodyDef groundDef = b2DefaultBodyDef();
	b2BodyId groundId = b2CreateBody( worldId, &groundDef );
	b2ShapeDef shapeDef = b2DefaultShapeDef();
	shapeDef.material.friction = 0.0f;

	b2BodyId bodyIds[2];
	for ( int lane = 0; lane < 2; ++lane )
	{
		float floorY = 3.0f * lane;
		for ( int i = 0; i < 10; ++i )
		{
			b2Polygon tile = b2MakeOffsetBox( 1.0f, 0.25f, (b2Vec2){ -9.0f + 2.0f * i, floorY - 0.25f }, b2Rot_identity );
			b2CreatePolygonShape( groundId, &shapeDef, &tile );
		}
		b2Polygon wall = b2MakeOffsetBox( 0.25f, 1.25f, (b2Vec2){ 10.25f, floorY + 1.25f }, b2Rot_identity );
		b2CreatePolygonShape( groundId, &shapeDef, &wall );

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.position = (b2Vec2){ -9.0f, floorY + 0.5f };
		bodyDef.linearVelocity = (b2Vec2){ speed, 0.0f };
		bodyDef.motionLocks.angularZ = true;
		bodyIds[lane] = b2CreateBody( worldId, &bodyDef );
		ENSURE( b2Body_IsSeamContactFilterEnabled( bodyIds[lane] ) == false );
		b2Body_EnableSeamContactFilter( bodyIds[lane], bodyFilter[lane] );
		ENSURE( b2Body_IsSeamContactFilterEnabled( bodyIds[lane] ) == bodyFilter[lane] );

		b2ShapeDef bodyShapeDef = shapeDef;
		bodyShapeDef.density = 1.0f;
		if ( useCircle )
		{
			b2Circle circle = { { 0.0f, 0.0f }, 0.5f };
			b2CreateCircleShape( bodyIds[lane], &bodyShapeDef, &circle );
		}
		else
		{
			b2Polygon box = b2MakeBox( 0.5f, 0.5f );
			b2CreatePolygonShape( bodyIds[lane], &bodyShapeDef, &box );
		}
	}

	for ( int i = 0; i < 180; ++i )
	{
		b2World_Step( worldId, 1.0f / 60.0f, 4, NULL, NULL );
	}

	for ( int lane = 0; lane < 2; ++lane )
	{
		positions[lane] = b2Body_GetPosition( bodyIds[lane] );
		positions[lane].y -= 3.0f * lane;
	}
	b2DestroyWorld( worldId );
	return 0;
}

static bool CrossedAllSeams( b2Vec2 p )
{
	// Reached the wall (box center stops at x = 9.5), still resting on the floor.
	return p.x > 8.0f && p.x < 9.5f + 0.01f && b2AbsFloat( p.y - 0.5f ) < 0.01f;
}

static int SeamContactFilterTest( void )
{
	{
		b2WorldDef worldDef = b2DefaultWorldDef();
		b2WorldId worldId = b2CreateWorld( &worldDef );

		// Default is disabled
		ENSURE( b2World_IsSeamContactFilterEnabled( worldId ) == false );
		b2World_EnableSeamContactFilter( worldId, true );
		ENSURE( b2World_IsSeamContactFilterEnabled( worldId ) == true );
		b2World_EnableSeamContactFilter( worldId, false );
		ENSURE( b2World_IsSeamContactFilterEnabled( worldId ) == false );

		b2DestroyWorld( worldId );
	}

	const bool noBodies[2] = { false, false };
	const bool firstBody[2] = { true, false };
	b2Vec2 p[2];

	// Without the filter the boxes catch the first tile seam (ghost collision).
	ENSURE( SlideAcrossTiles( false, noBodies, false, 10.0f, p ) == 0 );
	ENSURE( p[0].x < -6.0f && p[1].x < -6.0f );

	// World-wide filter: every box crosses every seam and is still stopped by the wall.
	b2Vec2 worldOnly[2];
	ENSURE( SlideAcrossTiles( true, noBodies, false, 10.0f, worldOnly ) == 0 );
	ENSURE( CrossedAllSeams( worldOnly[0] ) && CrossedAllSeams( worldOnly[1] ) );

	// Per body filter with the world filter disabled: only the enabled body is filtered.
	ENSURE( SlideAcrossTiles( false, firstBody, false, 10.0f, p ) == 0 );
	ENSURE( CrossedAllSeams( p[0] ) );
	ENSURE( p[1].x < -6.0f );

	// World or body: the body flag adds nothing when the world filter is already enabled.
	b2Vec2 q[2];
	ENSURE( SlideAcrossTiles( true, firstBody, false, 10.0f, q ) == 0 );
	for ( int lane = 0; lane < 2; ++lane )
	{
		ENSURE( q[lane].x == worldOnly[lane].x && q[lane].y == worldOnly[lane].y );
	}

	// One point manifolds (circles) are not changed by the filter.
	b2Vec2 circleOff[2], circleOn[2];
	ENSURE( SlideAcrossTiles( false, noBodies, true, 10.0f, circleOff ) == 0 );
	ENSURE( SlideAcrossTiles( true, firstBody, true, 10.0f, circleOn ) == 0 );
	for ( int lane = 0; lane < 2; ++lane )
	{
		ENSURE( circleOff[lane].x == circleOn[lane].x && circleOff[lane].y == circleOn[lane].y );
	}

	return 0;
}

// b2Shape_ComputeDistance (used by LiquidFun) must return the world distance and a world normal
// pointing from the shape to the target, also for a rotated body with an offset shape.
static int ShapeComputeDistanceTest( void )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	b2WorldId worldId = b2CreateWorld( &worldDef );

	// Rotated 90 degrees: the local x axis maps to world y. The box covers x in [2.75, 3.25] and
	// y in [1.5, 3.5] in world space.
	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.position = (b2Vec2){ 3.0f, 2.0f };
	bodyDef.rotation = b2MakeRot( 0.5f * B2_PI );
	b2BodyId bodyId = b2CreateBody( worldId, &bodyDef );

	b2ShapeDef shapeDef = b2DefaultShapeDef();
	b2Polygon box = b2MakeOffsetBox( 1.0f, 0.25f, (b2Vec2){ 0.5f, 0.0f }, b2Rot_identity );
	b2ShapeId shapeId = b2CreatePolygonShape( bodyId, &shapeDef, &box );

	struct
	{
		b2Vec2 target;
		float distance;
		b2Vec2 normal;
	} cases[] = {
		{ { 5.0f, 2.5f }, 1.75f, { 1.0f, 0.0f } },
		{ { 3.0f, 5.0f }, 1.5f, { 0.0f, 1.0f } },
		{ { 1.0f, 2.0f }, 1.75f, { -1.0f, 0.0f } },
		{ { 3.0f, 0.5f }, 1.0f, { 0.0f, -1.0f } },
	};

	for ( int i = 0; i < (int)( sizeof( cases ) / sizeof( cases[0] ) ); ++i )
	{
		float distance = -1.0f;
		b2Vec2 normal = b2Vec2_zero;
		b2Shape_ComputeDistance( shapeId, cases[i].target, &distance, &normal );
		ENSURE_SMALL( distance - cases[i].distance, 1e-5f );
		ENSURE_SMALL( normal.x - cases[i].normal.x, 1e-5f );
		ENSURE_SMALL( normal.y - cases[i].normal.y, 1e-5f );

		// Consistent with the closest point query
		b2Vec2 closest = b2Shape_GetClosestPoint( shapeId, cases[i].target );
		ENSURE_SMALL( b2Distance( closest, cases[i].target ) - distance, 1e-5f );
	}

	b2DestroyWorld( worldId );
	return 0;
}

// b2Body_GetPreviousTransform (used by LiquidFun) returns the transform from before the last step,
// and equals the current transform after creation and after b2Body_SetTransform.
static int BodyPreviousTransformTest( void )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	b2WorldId worldId = b2CreateWorld( &worldDef );

	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	bodyDef.position = (b2Vec2){ 1.0f, 5.0f };
	bodyDef.rotation = b2MakeRot( 0.3f );
	bodyDef.linearVelocity = (b2Vec2){ 2.0f, 0.0f };
	bodyDef.angularVelocity = 1.0f;
	b2BodyId bodyId = b2CreateBody( worldId, &bodyDef );

	b2ShapeDef shapeDef = b2DefaultShapeDef();
	b2Polygon box = b2MakeBox( 0.5f, 0.5f );
	b2CreatePolygonShape( bodyId, &shapeDef, &box );

	b2Transform created = b2Body_GetTransform( bodyId );
	b2Transform previous = b2Body_GetPreviousTransform( bodyId );
	ENSURE( previous.p.x == created.p.x && previous.p.y == created.p.y );
	ENSURE( previous.q.c == created.q.c && previous.q.s == created.q.s );

	b2Transform before = created;
	for ( int i = 0; i < 3; ++i )
	{
		b2World_Step( worldId, 1.0f / 60.0f, 4, NULL, NULL );
		b2Transform current = b2Body_GetTransform( bodyId );
		previous = b2Body_GetPreviousTransform( bodyId );
		ENSURE( previous.p.x == before.p.x && previous.p.y == before.p.y );
		ENSURE( previous.q.c == before.q.c && previous.q.s == before.q.s );
		ENSURE( current.p.x != before.p.x );
		before = current;
	}

	b2Body_SetTransform( bodyId, (b2Vec2){ -2.0f, 3.0f }, b2MakeRot( -0.7f ) );
	b2Transform moved = b2Body_GetTransform( bodyId );
	previous = b2Body_GetPreviousTransform( bodyId );
	ENSURE( previous.p.x == moved.p.x && previous.p.y == moved.p.y );
	ENSURE( previous.q.c == moved.q.c && previous.q.s == moved.q.s );

	b2DestroyWorld( worldId );
	return 0;
}

int WorldTest( void )
{
	RUN_SUBTEST( HelloWorld );
	RUN_SUBTEST( EmptyWorld );
	RUN_SUBTEST( DestroyAllBodiesWorld );
	RUN_SUBTEST( TestIsValid );
	RUN_SUBTEST( TestWorldRecycle );
	RUN_SUBTEST( TestWorldCoverage );
	RUN_SUBTEST( PreSolveCallingThreadTest );
	RUN_SUBTEST( PreSolveCcdCallingThreadTest );
	RUN_SUBTEST( PreSolveGlobalToggleTest );
	RUN_SUBTEST( TestSensor );
	RUN_SUBTEST( TestSetWorkerCount );
	RUN_SUBTEST( ChainSegmentShapeTest );
	RUN_SUBTEST( SetBulletDriftTest );
	RUN_SUBTEST( DeferredMassFlagSyncTest );
	RUN_SUBTEST( EnableSleepFlagSyncTest );
	RUN_SUBTEST( EnableContactRecyclingTest );
	RUN_SUBTEST( SeamContactFilterTest );
	RUN_SUBTEST( ShapeComputeDistanceTest );
	RUN_SUBTEST( BodyPreviousTransformTest );

	return 0;
}
