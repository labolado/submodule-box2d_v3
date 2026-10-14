// Internal, short-lived shape query data for LiquidFun. Not part of the public API.
#pragma once

#include "box2d/box2d.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct b2LiquidShapeQuery
{
	b2ShapeType type;
	union
	{
		b2Circle circle;
		b2Capsule capsule;
		b2Polygon polygon;
		b2Segment segment;
	};
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

// Snapshot geometry only; results and particle-specific state are never cached.
void b2ComputeLiquidShapeDistance( const b2LiquidShapeQuery* query, b2Vec2 target,
								  float* distance, b2Vec2* normal );

#ifdef __cplusplus
}
#endif
