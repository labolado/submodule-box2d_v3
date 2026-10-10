// Internal, short-lived shape query data for LiquidFun. Not part of the public API.
#pragma once

#include "box2d/box2d.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct b2LiquidShapeQuery
{
	b2DistanceInput input;
	b2WorldTransform transform;
} b2LiquidShapeQuery;

// Valid only while the shape and its body transform remain unchanged. In
// particular, do not reuse this data across user contact filter callbacks.
bool b2PrepareLiquidShapeQuery( b2ShapeId shapeId, b2LiquidShapeQuery* query );

// Internal bridge for read-only LiquidFun contact jobs during a locked step.
// Jobs must not access world state, invoke user callbacks, or allocate through
// the shared LiquidFun allocators. All jobs finish before this call returns.
int b2GetLiquidWorkerCount( b2WorldId worldId );
void b2RunLiquidTasks( b2WorldId worldId, b2TaskCallback* callback, void** contexts, int count );

static inline void b2ComputeLiquidShapeDistance( b2LiquidShapeQuery* query, b2Vec2 target,
											   float* distance, b2Vec2* normal )
{
	b2WorldTransform targetTransform = { b2ToPos( target ), b2Rot_identity };
	query->input.transform = b2InvMulWorldTransforms( query->transform, targetTransform );

	// Keep the original cold GJK cache and arithmetic: only query preparation is
	// reused, not the simplex or results from a different particle.
	b2SimplexCache cache = { 0 };
	b2DistanceOutput output = b2ShapeDistance( &query->input, &cache, NULL, 0 );
	*distance = output.distance;
	*normal = b2RotateVector( query->transform.q, b2Normalize( b2Sub( output.pointB, output.pointA ) ) );
}

#ifdef __cplusplus
}
#endif
