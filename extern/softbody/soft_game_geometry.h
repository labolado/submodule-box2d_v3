#pragma once
#include "soft_box2d_state.h"
#include "soft_game_recovery.h"

#include <algorithm>
#include <cmath>

namespace SoftBody
{
namespace Game
{
using Core = SoftBody::World<float>;
using View = SoftBody::Box2DState::StateView;
// Shape data copied at preflight into soft-space local coordinates. Rectangles
// retain the original feature order and arithmetic for the frozen car baseline.
struct RigidGeometry
{
	enum Kind
	{
		Rectangle,
		Polygon,
		Circle,
		Capsule,
		RoundedPolygon,
		Chain
	};
	Kind kind = Rectangle;
	float hx = 0, hy = 0;
	b2Polygon polygon{};
	b2Circle circle{};
	b2Capsule capsule{};
	b2ChainSegment chain{};
	bool read( b2ShapeId shape, bool flipY );
	bool equals( const RigidGeometry& other ) const;
	float distance( b2Vec2 point ) const;
};

// Unfiltered static-tree query: public world queries discard zero masks before
// a positive collision group can override them. Kept in the thin adapter.
b2TreeStats queryStaticShapes( b2WorldId world, b2AABB bounds, b2OverlapResultCallbackFcn* callback,
							   void* context );

// Exposed polygon-union boundary in a single rigid body's local coordinates.
// Each segment/arc retains its source fixture for material and cache identity.
struct BoundaryPolygon
{
	b2Polygon polygon;
	int shape;
};
struct BoundarySegment
{
	b2Vec2 a, b, normal;
	int shape, feature;
	bool cornerA = true, cornerB = true;
	// radius == 0 is the historical straight segment. Otherwise the boundary
	// is an exact counterclockwise circular arc, not a sampled polygon.
	b2Vec2 center{};
	float radius = 0, angle = 0, sweep = 0;
	b2Vec2 point( float t ) const;
	b2Vec2 normalAt( b2Vec2 p ) const;
	b2Vec2 closest( b2Vec2 p ) const;
	bool parameter( b2Vec2 p, float& t ) const;
	void intersections( const BoundarySegment& other, std::vector<float>& parameters ) const;
	float closestEdge( b2Vec2 p, b2Vec2 q, b2Vec2& surface ) const;
};
void buildCompoundBoundary( const std::vector<BoundaryPolygon>& polygons,
							std::vector<BoundarySegment>& boundary );
struct BoundaryShape
{
	RigidGeometry geometry;
	int shape;
};
void buildCurvedBoundary( const std::vector<BoundaryShape>& shapes,
						  std::vector<BoundarySegment>& boundary );

// Geometry-only extension of our C1 projection (historical name retained). Keep it separate from
// the accumulated velocity constraints: moving a node must refresh subsequent
// corner/edge queries, without adding depth / dt to physical velocity.
struct BoxProjection
{
	Core::B& body;
	b2WorldTransform transform;
	float hx, hy;
	const RigidGeometry* geometry = nullptr;
	const char* error = nullptr;
	unsigned corrections = 0;
	bool pointsOnly = false, stopOnInvalid = false, invalidated = false;
	std::vector<float>* remaining = nullptr;
	double* correctionDistance = nullptr;
	const std::vector<int>* nodeFaces = nullptr;
	float faceMargin = 0;

	BoxProjection( Core::B& b, const View& view, float x, float y )
		: body( b )
		, transform{ view.origin, view.rotation }
		, hx( x )
		, hy( y )
	{
	}

	BoxProjection( Core::B& b, const View& view, const RigidGeometry& shape )
		: BoxProjection( b, view, shape.hx, shape.hy )
	{
		geometry = &shape;
	}
	bool circular() const
	{
		return geometry && geometry->kind == RigidGeometry::Circle;
	}
	int faceCount() const;
	b2Vec2 faceNormal( int face ) const;
	b2Vec2 vertex( int corner ) const;
	bool withinFace( b2Vec2 p, int face ) const;
	b2Vec2 radialNormal( b2Vec2 p ) const;
	void projectCircle();

	static b2Vec2 normal( int face );

	float separation( b2Vec2 p, int face ) const;

	b2Vec2 local( int node ) const;

	int retainedFace( int node ) const;

	bool clip( b2Vec2 a, b2Vec2 b, int& entry, int& exit ) const;

	void move( int edge, float fraction, b2Vec2 direction, float depth );

	void project();

	double boundaryDepth() const;
};
} // namespace Game
} // namespace SoftBody
