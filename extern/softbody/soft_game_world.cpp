#include "soft_game_world.h"

#if defined( __clang__ )
#pragma clang fp contract( off )
#endif

namespace SoftBody
{
namespace Game
{
bool Solver::add( b2ShapeId shape )
{
	if ( boxCount == ( MaxCompoundShapes + maxChainSegments ) )
		return false;
	if ( size_t( boxCount ) == boxes.size() )
	{
		boxes.resize( boxCount + 1 );
		if ( contactSteps.size() < boxes.size() )
			contactSteps.resize( boxes.size() );
	}
	boundaryDirty = true;
	boxes[boxCount++].shape = shape;
	return true;
}

void Solver::clearCache()
{
	previousCount = 0;
	++cacheResets;
}

bool Solver::enabled( b2Filter other ) const
{
	if ( filter.groupIndex != 0 && filter.groupIndex == other.groupIndex )
		return filter.groupIndex > 0;
	return ( filter.maskBits & other.categoryBits ) && ( other.maskBits & filter.categoryBits );
}

bool Solver::prepare()
{
	if ( prepareTargets() )
		return true;
	// A rejected shape must never leave reusable impulses behind.
	clearCache();
	return false;
}

bool Solver::prepareTargets()
{
	if ( !b2World_IsValid( world ) || b2World_IsLocked( world ) )
	{
		error = "invalid_or_locked_world";
		return false;
	}
	for ( int i = 0; i < boxCount; ++i )
	{
		auto& box = boxes[i];
		if ( !b2Shape_IsValid( box.shape ) )
		{
			clearCache();
			error = "invalid_contact_shape";
			return false;
		}
		auto owner = b2Shape_GetWorld( box.shape );
		if ( owner.index1 != world.index1 || owner.generation != world.generation )
		{
			error = "foreign_contact_world";
			return false;
		}
		if ( !B2_ID_EQUALS( box.cachedShape, box.shape ) )
			clearCache();
		box.cachedShape = box.shape;
		box.body = b2Shape_GetBody( box.shape );
		box.stateSlot = i;
		auto transform = b2Body_GetTransform( box.body );
		auto type = b2Body_GetType( box.body );
		float mass = b2Body_GetMass( box.body ), inertia = b2Body_GetRotationalInertia( box.body );
		auto center = b2Body_GetLocalCenter( box.body );
		if ( !box.stateCached || box.cachedType != type || box.cachedMass != mass ||
			 box.cachedInertia != inertia || box.cachedCenter.x != center.x ||
			 box.cachedCenter.y != center.y || box.cachedTransform.p.x != transform.p.x ||
			 box.cachedTransform.p.y != transform.p.y || box.cachedTransform.q.c != transform.q.c ||
			 box.cachedTransform.q.s != transform.q.s )
			clearCache();
		box.stateCached = true;
		box.cachedTransform = transform;
		box.cachedType = type;
		box.cachedMass = mass;
		box.cachedInertia = inertia;
		box.cachedCenter = center;
		RigidGeometry geometry;

		// Rigid materials retain restitution for Box2D contacts. Soft contacts
		// currently solve only non-penetration and friction (zero restitution).
		if ( !geometry.read( box.shape, flipY ) )
		{
			error = "game_requires_supported_closed_shape";
			return false;
		}
		if ( geometry.kind == RigidGeometry::Chain &&
			 ( maxChainSegments == 0 || !chainQueryActive ||
			   chainStart.size() != body().x.size() ) )
		{
			error = "chain_requires_assembly_query";
			return false;
		}
		float hx = geometry.hx, hy = geometry.hy;
		if ( !box.geometry.equals( geometry ) )
		{
			boundaryDirty = true;
			clearCache();
			std::fill( box.nodeFaces.begin(), box.nodeFaces.end(), -1 );
		}
		box.geometry = geometry;
		auto nextFilter = b2Shape_GetFilter( box.shape );
		float nextFriction = b2Shape_GetFriction( box.shape );
		bool nextEnabled =
			b2Body_IsEnabled( box.body ) && !b2Shape_IsSensor( box.shape ) && enabled( nextFilter );
		if ( box.hx != hx || box.hy != hy || box.friction != nextFriction ||
			 box.enabled != nextEnabled || box.filter.categoryBits != nextFilter.categoryBits ||
			 box.filter.maskBits != nextFilter.maskBits ||
			 box.filter.groupIndex != nextFilter.groupIndex )
			clearCache();
		if ( box.enabled != nextEnabled )
			boundaryDirty = true;
		box.hx = hx;
		box.hy = hy;
		box.friction = nextFriction;
		box.filter = nextFilter;
		box.enabled = nextEnabled;
		if ( !nextEnabled )
			continue;
		if ( b2Body_GetType( box.body ) == b2_staticBody )
		{
			auto transform = b2Body_GetTransform( box.body );
			box.state = View{ b2Body_GetWorldCenter( box.body ), transform.q, { 0, 0 }, 0, 0, 0,
							  b2Body_GetLocalCenter( box.body ) };
			box.state.origin = transform.p;
			reflect( box.state );
			box.index = -1;
		}
		else if ( const char* reason = Bridge::validateStepBody( world, box.body ) )
		{
			error = reason;
			return false;
		}
		for ( int j = 0; j < i; ++j )
			if ( boxes[j].enabled && B2_ID_EQUALS( boxes[j].body, box.body ) )
			{
				if ( box.geometry.kind == RigidGeometry::Chain ||
					 boxes[j].geometry.kind == RigidGeometry::Chain )
					continue; // Static segments are surfaces, never a filled polygon union.
				box.stateSlot = boxes[j].stateSlot;
				break;
			}
	}
	prepareCompounds();
	return true;
}

void Solver::prepareCompounds()
{
	for ( int i = 0; i < boxCount; ++i )
	{
		boxes[i].compound = false;
		boxes[i].curved = false;
	}
	for ( int i = 0; i < boxCount; ++i )
		if ( boxes[i].enabled && boxes[i].stateSlot != i )
			boxes[boxes[i].stateSlot].compound = true;
	for ( int i = 0; i < boxCount; ++i )
		if ( boxes[i].enabled )
			boxes[i].compound = boxes[boxes[i].stateSlot].compound;
	for ( int i = 0; i < boxCount; ++i )
		if ( boxes[i].enabled &&
			 ( boxes[i].geometry.kind == RigidGeometry::Capsule ||
			   boxes[i].geometry.kind == RigidGeometry::RoundedPolygon ||
			   ( boxes[i].compound && boxes[i].geometry.kind == RigidGeometry::Circle ) ) )
			boxes[boxes[i].stateSlot].curved = true;
	for ( int i = 0; i < boxCount; ++i )
		if ( boxes[i].enabled )
			boxes[i].curved = boxes[boxes[i].stateSlot].curved;
	if ( !boundaryDirty )
		return;
	for ( int i = 0; i < boxCount; ++i )
	{
		auto& box = boxes[i];
		box.boundary.clear();
		if ( ( !box.compound && !box.curved ) || box.stateSlot != i )
			continue;
		if ( box.curved )
		{
			std::vector<BoundaryShape> shapes;
			for ( int j = 0; j < boxCount; ++j )
				if ( boxes[j].enabled && boxes[j].stateSlot == i )
					shapes.push_back( { boxes[j].geometry, j } );
			buildCurvedBoundary( shapes, box.boundary );
			curveCuts.reserve( 2 * box.boundary.size() + 2 );
			continue;
		}
		std::vector<BoundaryPolygon> polygons;
		for ( int j = 0; j < boxCount; ++j )
			if ( boxes[j].enabled && boxes[j].stateSlot == i )
				polygons.push_back( { boxes[j].geometry.polygon, j } );
		buildCompoundBoundary( polygons, box.boundary );
	}
	boundaryDirty = false;
	// Candidate registration validation may run before any soft body exists.
	if ( bodyIndex >= 0 && size_t( bodyIndex ) < soft.bodies.size() )
	{
		size_t capacity = 0;
		for ( int i = 0; i < boxCount; ++i )
			if ( boxes[i].enabled && boxes[i].stateSlot == i )
				capacity +=
					boxes[i].curved ? ( 4 * boxes[i].boundary.size() + 2 ) * body().nodeCount
					: boxes[i].compound
						? size_t( boxCount + 1 ) * body().nodeCount + 2 * boxes[i].boundary.size()
						: 2 * body().nodeCount + B2_MAX_POLYGON_VERTICES;
		if ( rows.size() < capacity )
			rows.resize( capacity );
		if ( previous.size() < capacity )
			previous.resize( capacity );
	}
}

// Convex clipping depends only on the material segment, not contour winding.
static bool clipCompoundSegment( const b2Polygon& polygon, b2Vec2 a, b2Vec2 b, float& lo,
								 float& hi )
{
	lo = 0;
	hi = 1;
	auto d = b2Sub( b, a );
	for ( int f = 0; f < polygon.count; ++f )
	{
		float start = b2Dot( b2Sub( a, polygon.vertices[f] ), polygon.normals[f] );
		float slope = b2Dot( d, polygon.normals[f] );
		if ( std::abs( slope ) < 1e-9f )
		{
			if ( start >= 0 )
				return false;
		}
		else if ( slope > 0 )
			hi = std::min( hi, -start / slope );
		else
			lo = std::max( lo, -start / slope );
	}
	return hi > lo;
}

bool Solver::insideCompound( int owner, b2Vec2 p ) const
{
	for ( int j = 0; j < boxCount; ++j )
		if ( boxes[j].enabled && boxes[j].stateSlot == owner )
		{
			if ( boxes[owner].curved )
			{
				if ( boxes[j].geometry.distance( p ) <= 0 )
					return true;
				continue;
			}
			const auto& polygon = boxes[j].geometry.polygon;
			bool inside = true;
			for ( int f = 0; f < polygon.count; ++f )
				if ( b2Dot( b2Sub( p, polygon.vertices[f] ), polygon.normals[f] ) > 0 )
				{
					inside = false;
					break;
				}
			if ( inside )
				return true;
		}
	return false;
}
bool Solver::nearestCompound( int owner, b2Vec2 p, b2Vec2& q, int& segment ) const
{
	float best = 1e30f;
	segment = -1;
	const auto& edges = boxes[owner].boundary;
	for ( size_t i = 0; i < edges.size(); ++i )
	{
		auto point = edges[i].closest( p );
		float distance = b2LengthSquared( b2Sub( point, p ) );
		if ( distance < best )
		{
			best = distance;
			q = point;
			segment = int( i );
		}
	}
	return segment >= 0;
}
// Find a material interval inside the union, including crossings with both
// endpoints outside. Split at exact line/arc intersections; no contour sampling.
bool Solver::curvedInterior( int owner, b2Vec2 a, b2Vec2 b, float& fraction, b2Vec2& surface,
							 int& feature )
{
	BoundarySegment material;
	material.a = a;
	material.b = b;
	if ( b2DistanceSquared( a, b ) < 1e-12f )
		return false;
	curveCuts.clear();
	curveCuts.push_back( 0 );
	curveCuts.push_back( 1 );
	for ( const auto& curve : boxes[owner].boundary )
		material.intersections( curve, curveCuts );
	std::sort( curveCuts.begin(), curveCuts.end() );
	float best = 0;
	feature = -1;
	for ( size_t i = 1; i < curveCuts.size(); ++i )
	{
		if ( curveCuts[i] - curveCuts[i - 1] < 1e-7f )
			continue;
		float t = .5f * ( curveCuts[i] + curveCuts[i - 1] );
		auto p = b2Lerp( a, b, t );
		if ( !insideCompound( owner, p ) )
			continue;
		b2Vec2 q;
		int edge;
		if ( !nearestCompound( owner, p, q, edge ) )
			continue;
		float depth = b2DistanceSquared( p, q );
		if ( depth > best )
		{
			best = depth;
			fraction = t;
			surface = q;
			feature = edge;
		}
	}
	return feature >= 0;
}

void Solver::gatherCurved( int owner )
{
	auto& box = boxes[owner];
	BoxProjection geometry( body(), box.state, box.geometry );
	auto appendContact = [&]( int node, float fraction, b2Vec2 q, int curve, int kind,
							  b2Vec2 normal ) {
		const auto& surface = box.boundary[curve];
		Row row;
		row.box = surface.shape;
		row.edge = node;
		row.fraction = fraction;
		row.feature = 4 * surface.feature + kind;
		row.localAnchor = q;
		row.normal = b2RotateVector( geometry.transform.q, normal );
		append( row );
		++contactSteps[row.box];
	};
	for ( int node = 0; node < body().nodeCount; ++node )
	{
		auto p = geometry.local( node );
		b2Vec2 q;
		int edge;
		if ( nearestCompound( owner, p, q, edge ) &&
			 ( b2DistanceSquared( p, q ) <= Margin * Margin || insideCompound( owner, p ) ) )
			appendContact( node, 0, q, edge, 0, box.boundary[edge].normalAt( q ) );
	}
	for ( int edge = 0; edge < body().nodeCount; ++edge )
	{
		auto a = geometry.local( edge ), b = geometry.local( ( edge + 1 ) % body().nodeCount );
		if ( b2DistanceSquared( a, b ) < 1e-12f )
			continue;
		for ( size_t i = 0; i < box.boundary.size(); ++i )
		{
			const auto& curve = box.boundary[i];
			b2Vec2 q;
			float t = curve.closestEdge( a, b, q );
			auto p = b2Lerp( a, b, t );
			if ( t <= 1e-6f || t >= 1 - 1e-6f )
				continue;
			auto normal = curve.normalAt( q );
			bool contained = false;
			if ( !recovery.active )
			{
				auto world = b2TransformWorldPoint( geometry.transform, q );
				Core::Vec point{ float( world.x ), float( world.y ) };
				contained = SoftBody::inside<float>( point, body() ) &&
							SoftBody::nearest<float>( point, body() ).edge == edge;
				if ( contained )
				{
					auto d = b2Normalize( b2Sub( b, a ) );
					normal = { -d.y, d.x };
				}
			}
			if ( contained || b2DistanceSquared( p, q ) <= Margin * Margin )
				appendContact( edge, t, q, int( i ), 1, normal );
		}
		float t;
		b2Vec2 q;
		int curve;
		if ( curvedInterior( owner, a, b, t, q, curve ) )
			appendContact( edge, t, q, curve, 2, box.boundary[curve].normalAt( q ) );
	}
}

bool Solver::projectCurved( int owner )
{
	if ( !observeRecovery() )
		return false;
	auto& box = boxes[owner];
	BoxProjection projection( body(), box.state, box.geometry );
	projection.stopOnInvalid = !recovery.active;
	if ( recovery.active )
	{
		projection.remaining = &recovery.remaining;
		projection.correctionDistance = &recovery.correctionDistance;
	}
	auto move = [&]( int edge, float t, b2Vec2 p, b2Vec2 q ) {
		auto delta = b2Sub( q, p );
		float depth = b2Length( delta );
		if ( depth > 1e-7f )
			projection.move( edge, t,
							 b2RotateVector( projection.transform.q, b2MulSV( 1 / depth, delta ) ),
							 depth );
	};
	for ( int node = 0; node < body().nodeCount && !projection.invalidated; ++node )
	{
		auto p = projection.local( node );
		b2Vec2 q;
		int curve;
		if ( insideCompound( owner, p ) && nearestCompound( owner, p, q, curve ) )
			move( node, 0, p, q );
	}
	for ( int edge = 0; edge < body().nodeCount && !projection.invalidated; ++edge )
	{
		auto a = projection.local( edge ), b = projection.local( ( edge + 1 ) % body().nodeCount );
		float t;
		b2Vec2 q;
		int curve;
		if ( curvedInterior( owner, a, b, t, q, curve ) )
			move( edge, t, b2Lerp( a, b, t ), q );
	}
	// A closed rigid component may lie entirely inside a healthy soft contour.
	// Only its nearest material edge supplies an escape direction. During
	// folding, there is no oriented solid interior; retain finite edge crossings.
	if ( !recovery.active )
		for ( const auto& curve : box.boundary )
		{
			if ( projection.invalidated )
				break;
			auto world = b2TransformWorldPoint( projection.transform, curve.point( .5f ) );
			Core::Vec point{ float( world.x ), float( world.y ) };
			if ( !SoftBody::inside<float>( point, body() ) )
				continue;
			auto hit = SoftBody::nearest<float>( point, body() );
			if ( hit.edge >= 0 )
				projection.move( hit.edge, hit.t, { -hit.normal[0], -hit.normal[1] }, hit.depth );
		}
	geometryCorrections += projection.corrections;
	if ( profile && profile->enabled && profile->geometryDepth )
		profile->work[Profile::ProjectionCorrections] += projection.corrections;
	return observeRecovery();
}

// Node and interior-interval witnesses, not a certified continuous maximum.
// Regression fixtures additionally sample complete material edges independently.
double Solver::curvedDepth( int owner )
{
	BoxProjection geometry( body(), boxes[owner].state, boxes[owner].geometry );
	double depth = 0;
	for ( int edge = 0; edge < body().nodeCount; ++edge )
	{
		auto a = geometry.local( edge ), b = geometry.local( ( edge + 1 ) % body().nodeCount );
		b2Vec2 q;
		int curve;
		float t;
		if ( insideCompound( owner, a ) && nearestCompound( owner, a, q, curve ) )
			depth = std::max( depth, double( b2Distance( a, q ) ) );
		if ( curvedInterior( owner, a, b, t, q, curve ) )
			depth = std::max( depth, double( b2Distance( b2Lerp( a, b, t ), q ) ) );
	}
	return depth;
}

void Solver::gatherCompound( int owner )
{
	auto& box = boxes[owner];
	auto& body = this->body();
	b2WorldTransform transform{ box.state.origin, box.state.rotation };
	for ( int node = 0; node < body.nodeCount; ++node )
	{
		auto p = b2InvTransformWorldPoint( transform, { body.x[node][0], body.x[node][1] } );
		b2Vec2 q;
		int edge;
		if ( !nearestCompound( owner, p, q, edge ) )
			continue;
		float distance = b2Length( b2Sub( q, p ) );
		if ( distance > Margin && !insideCompound( owner, p ) )
			continue;
		const auto& surface = box.boundary[edge];
		Row row;
		row.box = surface.shape;
		row.edge = node;
		row.feature = surface.feature;
		row.localAnchor = q;
		row.normal = b2RotateVector( transform.q, surface.normal );
		append( row );
		++contactSteps[row.box];
	}
	if ( recovery.active )
	{
		// Folded material has no oriented interior, but each finite segment can
		// still cross a real solid. Constrain those crossings as well as nodes.
		for ( int node = 0; node < body.nodeCount; ++node )
		{
			auto a = b2InvTransformWorldPoint( transform, { body.x[node][0], body.x[node][1] } );
			auto next = body.x[( node + 1 ) % body.nodeCount];
			auto b = b2InvTransformWorldPoint( transform, { next[0], next[1] } );
			for ( int shape = 0; shape < boxCount; ++shape )
			{
				if ( !boxes[shape].enabled || boxes[shape].stateSlot != owner )
					continue;
				float lo, hi;
				if ( !clipCompoundSegment( boxes[shape].geometry.polygon, a, b, lo, hi ) )
					continue;
				float fraction = ( lo + hi ) * .5f;
				auto p = b2Lerp( a, b, fraction );
				b2Vec2 q;
				int segment;
				if ( !nearestCompound( owner, p, q, segment ) )
					continue;
				const auto& surface = box.boundary[segment];
				Row row;
				row.box = surface.shape;
				row.edge = node;
				row.fraction = fraction;
				row.feature = 4096 + surface.feature;
				row.localAnchor = q;
				row.normal = b2RotateVector( transform.q, surface.normal );
				append( row );
				++contactSteps[row.box];
			}
		}
		return;
	}
	for ( const auto& edge : box.boundary )
		for ( int side = 0; side < 2; ++side )
		{
			if ( !( side ? edge.cornerB : edge.cornerA ) )
				continue;
			auto local = side ? edge.b : edge.a;
			auto world = b2TransformWorldPoint( transform, local );
			Core::Vec point{ float( world.x ), float( world.y ) };
			auto hit = SoftBody::nearest<float>( point, body );
			if ( hit.edge < 0 )
			{
				error = "degenerate_edge";
				return;
			}
			if ( !SoftBody::inside<float>( point, body ) && hit.depth > Margin )
				continue;
			Row row;
			row.box = edge.shape;
			row.edge = hit.edge;
			row.fraction = hit.t;
			row.feature = 1024 + 2 * edge.feature + side;
			row.localAnchor = local;
			// Use the exterior face, not a fictitious internal seam normal.
			row.normal = b2RotateVector( transform.q, edge.normal );
			append( row );
			++contactSteps[row.box];
		}
}
bool Solver::projectCompound( int owner )
{
	auto& box = boxes[owner];
	if ( !observeRecovery() )
		return false;
	BoxProjection projection( body(), box.state, box.geometry );
	projection.stopOnInvalid = !recovery.active;
	if ( recovery.active )
	{
		projection.remaining = &recovery.remaining;
		projection.correctionDistance = &recovery.correctionDistance;
	}
	auto movePoint = [&]( int node, float t, b2Vec2 p ) {
		if ( !insideCompound( owner, p ) )
			return;
		b2Vec2 q;
		int edge;
		if ( !nearestCompound( owner, p, q, edge ) )
			return;
		auto delta = b2Sub( q, p );
		float depth = b2Length( delta );
		if ( depth > 1e-7f )
			projection.move( node, t,
							 b2RotateVector( projection.transform.q, b2MulSV( 1 / depth, delta ) ),
							 depth );
	};
	for ( int node = 0; node < body().nodeCount && !projection.invalidated; ++node )
		movePoint( node, 0, projection.local( node ) );
	if ( !recovery.active )
	{
		for ( const auto& edge : box.boundary )
			for ( int side = 0; side < 2 && !projection.invalidated; ++side )
			{
				if ( !( side ? edge.cornerB : edge.cornerA ) )
					continue;
				auto world = b2TransformWorldPoint( projection.transform, side ? edge.b : edge.a );
				Core::Vec point{ float( world.x ), float( world.y ) };
				if ( !SoftBody::inside<float>( point, body() ) )
					continue;
				auto hit = SoftBody::nearest<float>( point, body() );
				if ( hit.edge >= 0 )
					projection.move( hit.edge, hit.t, { -hit.normal[0], -hit.normal[1] },
									 hit.depth );
			}
	}
	// Detect a long soft edge crossing an entire component even when neither
	// material endpoint nor a rigid corner lies in the other solid.
	for ( int node = 0; node < body().nodeCount && !projection.invalidated; ++node )
		for ( int j = 0; j < boxCount && !projection.invalidated; ++j )
		{
			if ( !boxes[j].enabled || boxes[j].stateSlot != owner )
				continue;
			auto a = projection.local( node ),
				 d = b2Sub( projection.local( ( node + 1 ) % body().nodeCount ), a );
			float lo, hi;
			bool crossed =
				clipCompoundSegment( boxes[j].geometry.polygon, a, b2Add( a, d ), lo, hi );
			if ( crossed )
			{
				float t = ( lo + hi ) * .5f;
				movePoint( node, t, b2MulAdd( a, t, d ) );
			}
		}

	geometryCorrections += projection.corrections;
	if ( profile && profile->enabled && profile->geometryDepth )
		profile->work[Profile::ProjectionCorrections] += projection.corrections;
	return observeRecovery();
}

namespace
{
struct ChainHit
{
	Row row;
	float gap;
};
// Use Box2D's public narrow phase for the adjacency normal cone. Each material
// edge is a thin capsule, not a separately simulated rigid body. A front-side
// edge may continue through the plane within this step; back-side edges pass.
int chainHits( const Solver& solver, int shape, int edge, ChainHit hits[2], float reach = Margin )
{
	const auto& box = solver.boxes[shape];
	const auto& body = solver.body();
	const auto& chain = box.geometry.chain;
	b2WorldTransform transform{ box.state.origin, box.state.rotation };
	int next = ( edge + 1 ) % body.nodeCount;
	auto local = [&]( Core::Vec p ) {
		return b2InvTransformWorldPoint( transform, { p[0], p[1] } );
	};
	auto a = local( body.x[edge] ), b = local( body.x[next] );
	auto start = b2Lerp( local( solver.chainStart[edge] ), local( solver.chainStart[next] ), .5f );
	auto normal = b2Normalize( b2RightPerp( b2Sub( chain.segment.point2, chain.segment.point1 ) ) );
	if ( b2Dot( normal, b2Sub( start, chain.segment.point1 ) ) < -Margin )
		return 0;
	float side = b2Dot( normal, b2Sub( b2Lerp( a, b, .5f ), chain.segment.point1 ) );
	auto shift = b2MulSV( std::max( 0.f, Margin - side ), normal );
	float radius = Margin + std::max( 0.f, reach );
	b2LocalManifold manifold;
	auto d = b2Sub( b, a );
	float length2 = b2LengthSquared( d );
	if ( length2 < 1e-12f )
	{
		b2Circle point{ b2Add( a, shift ), radius };
		manifold = b2CollideChainSegmentAndCircle( &chain, &point, b2Transform_identity );
	}
	else
	{
		b2Capsule edgeShape{ b2Add( a, shift ), b2Add( b, shift ), radius };
		b2SimplexCache cache{};
		manifold =
			b2CollideChainSegmentAndCapsule( &chain, &edgeShape, b2Transform_identity, &cache );
	}
	int count = 0;
	for ( int i = 0; i < manifold.pointCount; ++i )
	{
		const auto& contact = manifold.points[i];
		auto n = manifold.normal;
		// The capsule is only a query proxy for a closed, CCW material contour.
		// Its back-facing side must not support the body: after going around a
		// chain endpoint, upper edges can otherwise hang on the terrain while
		// lower edges correctly pass through the noncolliding back side.
		// During unfolding the contour orientation is not reliable; retain the
		// existing recovery path until a valid boundary has been restored.
		if ( !solver.recovery.active && b2Dot( b2RightPerp( d ), n ) > 0 )
			continue;
		auto surface = b2MulAdd( contact.point, -.5f * contact.separation, n );
		auto center =
			b2Sub( b2MulAdd( contact.point, .5f * contact.separation + radius, n ), shift );
		float fraction =
			length2 > 1e-12f
				? std::max( 0.f, std::min( 1.f, b2Dot( b2Sub( center, a ), d ) / length2 ) )
				: 0;
		float gap = b2Dot( b2Sub( b2Lerp( a, b, fraction ), surface ), n ) - Margin;
		if ( gap > reach )
			continue;
		Row row;
		row.box = shape;
		row.edge = edge;
		row.fraction = fraction;
		row.feature = contact.id;
		row.normal = b2RotateVector( transform.q, n );
		row.localAnchor = b2MulAdd( surface, Margin, n );
		hits[count++] = { row, gap };
	}
	return count;
}
} // namespace

bool Solver::withinChainQuery()
{
	if ( !chainQueryActive )
		return true;
	for ( auto p : body().x )
		if ( p[0] < chainQueryBounds.lowerBound.x || p[1] < chainQueryBounds.lowerBound.y ||
			 p[0] > chainQueryBounds.upperBound.x || p[1] > chainQueryBounds.upperBound.y )
		{
			error = "chain_motion_exceeded_query_bounds";
			return false;
		}
	return true;
}
void Solver::chainEdgeRows( int edge )
{
	// On a straight material edge, coplanar contacts with the same friction
	// impose redundant constraints. Retain their extreme material coordinates
	// so support does not depend on how finely a flat road is segmented.
	// Different normals and materials keep separate rows and actual anchors.
	chainPatches.clear();
	auto anchor = [&]( const Row& row ) {
		const auto& box = boxes[row.box];
		auto p = b2TransformWorldPoint( { box.state.origin, box.state.rotation }, row.localAnchor );
		return b2Dot( { float( p.x ), float( p.y ) }, row.normal );
	};
	for ( int shape = 0; shape < boxCount; ++shape )
	{
		if ( !boxes[shape].enabled || boxes[shape].geometry.kind != RigidGeometry::Chain )
			continue;
		ChainHit hits[2];
		int count = chainHits( *this, shape, edge, hits );
		for ( int i = 0; i < count; ++i )
		{
			const auto& row = hits[i].row;
			bool merged = false;
			for ( size_t j = 0; j < chainPatches.size(); j += 2 )
			{
				auto& low = chainPatches[j];
				auto& high = chainPatches[j + 1];
				if ( b2LengthSquared( b2Sub( low.normal, row.normal ) ) > 1e-12f ||
					 std::abs( anchor( low ) - anchor( row ) ) > 1e-6f ||
					 boxes[low.box].friction != boxes[row.box].friction )
					continue;
				if ( row.fraction < low.fraction )
					low = row;
				if ( row.fraction > high.fraction )
					high = row;
				merged = true;
				break;
			}
			if ( !merged )
			{
				chainPatches.push_back( row );
				chainPatches.push_back( row );
			}
		}
	}
}
void Solver::gatherChains()
{
	for ( int edge = 0; edge < body().nodeCount; ++edge )
	{
		chainEdgeRows( edge );
		for ( size_t i = 0; i < chainPatches.size(); ++i )
		{
			const auto& row = chainPatches[i];
			if ( i % 2 && row.fraction == chainPatches[i - 1].fraction )
				continue;
			append( row );
			++contactSteps[row.box];
		}
	}
}
bool Solver::projectChains()
{
	if ( !observeRecovery() )
		return false;
	// The projection needs only body mass and a world-space normal. Individual
	// row anchors below retain their actual segment transform and material.
	View origin{};
	BoxProjection projection( body(), origin, 0, 0 );
	projection.stopOnInvalid = !recovery.active;
	if ( recovery.active )
	{
		projection.remaining = &recovery.remaining;
		projection.correctionDistance = &recovery.correctionDistance;
	}
	for ( int edge = 0; edge < body().nodeCount && !projection.invalidated; ++edge )
	{
		chainEdgeRows( edge );
		for ( size_t i = 0; i < chainPatches.size() && !projection.invalidated; ++i )
		{
			const auto& row = chainPatches[i];
			if ( i % 2 && row.fraction == chainPatches[i - 1].fraction )
				continue;
			const auto& box = boxes[row.box];
			auto anchor =
				b2TransformWorldPoint( { box.state.origin, box.state.rotation }, row.localAnchor );
			auto p = point( row );
			float gap = b2Dot( { float( p.x - anchor.x ), float( p.y - anchor.y ) }, row.normal );
			projection.move( edge, row.fraction, row.normal, -gap );
		}
	}
	geometryCorrections += projection.corrections;
	if ( profile && profile->enabled && profile->geometryDepth )
		profile->work[Profile::ProjectionCorrections] += projection.corrections;
	return observeRecovery() && withinChainQuery();
}
double Solver::chainDepth( int shape ) const
{
	double depth = 0;
	for ( int edge = 0; edge < body().nodeCount; ++edge )
	{
		ChainHit hits[2];
		int count = chainHits( *this, shape, edge, hits );
		for ( int i = 0; i < count; ++i )
			depth = std::max( depth, double( -hits[i].gap ) );
	}
	return depth;
}

b2Vec2 Solver::point( const Row& row ) const
{
	const auto& body = this->body();
	const auto& a = body.x[row.edge];
	const auto& b = body.x[( row.edge + 1 ) % body.nodeCount];
	return { ( 1 - row.fraction ) * a[0] + row.fraction * b[0],
			 ( 1 - row.fraction ) * a[1] + row.fraction * b[1] };
}

float Solver::softInverse( const Row& row ) const
{
	const auto& mass = body().mass;
	return ( 1 - row.fraction ) * ( 1 - row.fraction ) / mass[row.edge] +
		   row.fraction * row.fraction / mass[( row.edge + 1 ) % body().nodeCount];
}

void Solver::append( Row row )
{
	if ( size_t( rowCount ) == rows.size() )
	{
		error = "contact_capacity";
		return;
	}
	if ( !recovery.active )
	{
		Profile::Scope timing( profile, Profile::GeometryCache );
		for ( int i = 0; i < previousCount; ++i )
		{
			if ( timing.profile )
				++timing.profile->work[Profile::CacheComparisons];
			const auto& old = previous[i];
			if ( old.box == row.box && old.edge == row.edge && old.feature == row.feature &&
				 std::abs( old.fraction - row.fraction ) < .1f &&
				 b2Dot( old.normal, row.normal ) > .98f &&
				 b2Length( b2Sub( old.localAnchor, row.localAnchor ) ) < .02f )
			{
				row.normalImpulse = old.normalImpulse;
				row.tangentImpulse = old.tangentImpulse;
				++warmHits;
				break;
			}
		}
	}
	if ( profile && profile->enabled && profile->geometryDepth )
		++profile->work[Profile::Rows];
	rows[rowCount++] = row;
}

void Solver::gather()
{
	Profile::Scope timing( profile, Profile::GeometryGather );
	bool chainsGathered = false;
	rowCount = 0;
	auto& body = this->body();
	for ( int id = 0; id < boxCount; ++id )
	{
		auto& box = boxes[id];
		if ( !box.enabled )
			continue;
		if ( timing.profile )
			++timing.profile->work[Profile::GatherCandidates];
		b2WorldTransform transform = { box.state.origin, box.state.rotation };
		if ( box.geometry.kind == RigidGeometry::Chain )
		{
			if ( !chainsGathered )
				gatherChains();
			chainsGathered = true;
			continue;
		}
		if ( box.curved )
		{
			if ( box.stateSlot == id )
				gatherCurved( id );
			continue;
		}
		if ( box.compound )
		{
			if ( box.stateSlot == id )
				gatherCompound( id );
			continue;
		}
		int start = rowCount;
		BoxProjection geometry( body, box.state, box.geometry );
		geometry.nodeFaces = &box.nodeFaces;
		geometry.faceMargin = Margin;
		for ( int node = 0; node < body.nodeCount; ++node )
		{
			auto p = b2InvTransformWorldPoint( transform, { body.x[node][0], body.x[node][1] } );
			float distance = -1e30f;
			int face = 0;
			for ( int j = 0; j < geometry.faceCount(); ++j )
			{
				float d = geometry.separation( p, j );
				if ( d > distance )
				{
					distance = d;
					face = j;
				}
			}
			if ( distance > Margin )
			{
				// A retained side is considered below even if another box face
				// has a tiny positive gap at a seam.
				if ( !retainBoundarySides )
					continue;
			}
			if ( retainBoundarySides )
			{
				int retained = geometry.retainedFace( node );
				if ( retained >= 0 )
				{
					face = retained;
					distance = geometry.separation( p, face );
				}
			}
			if ( distance > Margin )
				continue;
			Row row;
			row.box = id;
			row.edge = node;
			row.feature = face;
			auto normal =
				geometry.circular() ? geometry.radialNormal( p ) : geometry.faceNormal( face );
			row.normal = b2RotateVector( transform.q, normal );
			row.localAnchor = b2MulAdd( p, -distance, normal );
			append( row );
		}
		if ( geometry.circular() )
		{
			// Interior edge contacts complement node contacts without duplicate endpoints.
			if ( !recovery.active )
			{
				auto center = b2TransformWorldPoint( transform, box.geometry.circle.center );
				bool contained =
					SoftBody::inside<float>( { float( center.x ), float( center.y ) }, body );
				auto nearest =
					contained
						? SoftBody::nearest<float>( { float( center.x ), float( center.y ) }, body )
						: Hit<float>{};
				for ( int edge = 0; edge < body.nodeCount; ++edge )
				{
					auto a = geometry.local( edge ),
						 d = b2Sub( geometry.local( ( edge + 1 ) % body.nodeCount ), a );
					float length2 = b2LengthSquared( d );
					if ( length2 < 1e-12f )
						continue;
					float t = std::max(
						0.f, std::min( 1.f, b2Dot( b2Sub( box.geometry.circle.center, a ), d ) /
												length2 ) );
					if ( t <= 0 || t >= 1 )
						continue;
					auto p = b2MulAdd( a, t, d );
					float distance = geometry.separation( p, 0 );
					bool interior = contained && edge == nearest.edge;
					if ( !interior && distance > Margin )
						continue;
					auto n = geometry.radialNormal( p );
					if ( interior ||
						 b2LengthSquared( b2Sub( p, box.geometry.circle.center ) ) < 1e-16f )
						n = b2MulSV( 1 / std::sqrt( length2 ), b2Vec2{ -d.y, d.x } );
					Row row;
					row.box = id;
					row.edge = edge;
					row.feature = 1;
					row.fraction = t;
					row.normal = b2RotateVector( transform.q, n );
					row.localAnchor =
						b2MulAdd( box.geometry.circle.center, box.geometry.circle.radius, n );
					append( row );
				}
			}
			if ( rowCount != start )
				++contactSteps[id];
			continue;
		}
		for ( int corner = 0; corner < geometry.faceCount(); ++corner )
		{
			// A folded contour is not an oriented solid. Its material
			// nodes still collide with real box faces; never ask inside
			// or nearest-edge queries to invent an interior in this mode.
			if ( recovery.active )
				break;
			b2Vec2 local = geometry.vertex( corner );
			auto vertex = b2TransformWorldPoint( transform, local );
			Core::Vec p = { float( vertex.x ), float( vertex.y ) };
			bool inside = SoftBody::inside<float>( p, body );
			auto hit = SoftBody::nearest<float>( p, body );
			if ( hit.edge < 0 )
			{
				error = "degenerate_edge";
				return;
			}
			if ( !inside && hit.depth > Margin )
				continue;
			Row row;
			row.box = id;
			row.edge = hit.edge;
			row.feature = geometry.faceCount() + corner;
			row.fraction = hit.t;
			row.localAnchor = local;
			row.normal = { -hit.normal[0], -hit.normal[1] };
			{
				// The point may be just outside after split projection.
				// Use the oriented material edge, not the sign of its
				// nearly-zero distance to the box corner.
				auto a = body.x[hit.edge], b = body.x[( hit.edge + 1 ) % body.nodeCount];
				float length = std::hypot( b[0] - a[0], b[1] - a[1] );
				row.normal = { -( b[1] - a[1] ) / length, ( b[0] - a[0] ) / length };
			}
			append( row );
		}
		if ( rowCount != start )
			++contactSteps[id];
	}
}

bool Solver::projectGeometry()
{
	Profile::Scope timing( profile, Profile::GeometryProject );
	if ( !withinChainQuery() )
		return false;
	// Within this call only BoxProjection::move can change node positions;
	// it increments corrections before every write. Velocities stay unchanged.
	// Repeated endOfStep=false observations of unchanged topology are idempotent.
	// Keep the observation local so edits between calls cannot reuse stale data.
	bool observed = false;
	for ( int sweep = 0; sweep < Sweeps; ++sweep )
	{
		bool chainsProjected = false;
		for ( int j = 0; j < boxCount; ++j )
		{
			int id = j;
			auto& box = boxes[id];
			if ( !box.enabled )
				continue;
			if ( timing.profile )
				++timing.profile->work[Profile::ProjectionCandidates];
			if ( box.geometry.kind == RigidGeometry::Chain )
			{
				if ( !chainsProjected && !projectChains() )
					return false;
				chainsProjected = true;
				observed = false;
				continue;
			}
			if ( box.curved )
			{
				if ( box.stateSlot == id && !projectCurved( id ) )
					return false;
				observed = false;
				continue;
			}
			if ( box.compound )
			{
				if ( box.stateSlot == id && !projectCompound( id ) )
					return false;
				observed = false;
				continue;
			}
			BoxProjection geometry( body(), box.state, box.geometry );
			if ( retainBoundarySides )
			{
				geometry.nodeFaces = &box.nodeFaces;
				geometry.faceMargin = Margin;
			}
			// Folded boundaries have no well-defined interior.
			{
				if ( !observed && !observeRecovery() )
					return false;
				observed = true;
				geometry.pointsOnly = recovery.active;
				geometry.stopOnInvalid = !recovery.active;
				if ( recovery.active )
				{
					geometry.remaining = &recovery.remaining;
					geometry.correctionDistance = &recovery.correctionDistance;
				}
			}
			geometry.project();
			geometryCorrections += geometry.corrections;
			if ( profile && profile->enabled && profile->geometryDepth )
				profile->work[Profile::ProjectionCorrections] += geometry.corrections;
			if ( geometry.error )
			{
				error = geometry.error;
				return false;
			}
			if ( geometry.corrections != 0 && !observeRecovery() )
				return false;
		}
	}
	return withinChainQuery();
}

void Solver::captureBoundarySides()
{
	Profile::Scope timing( profile, Profile::GeometrySides );
	for ( int j = 0; j < boxCount; ++j )
	{
		auto& box = boxes[j];
		if ( !box.enabled )
		{
			std::fill( box.nodeFaces.begin(), box.nodeFaces.end(), -1 );
			continue;
		}
		BoxProjection geometry( body(), box.state, box.geometry );
		geometry.nodeFaces = &box.nodeFaces;
		geometry.faceMargin = Margin;
		if ( box.curved || box.compound || geometry.circular() ||
			 box.geometry.kind == RigidGeometry::Chain )
		{
			std::fill( box.nodeFaces.begin(), box.nodeFaces.end(), -1 );
			continue;
		}
		auto center = soft.center( bodyIndex );
		auto centerLocal = b2InvTransformWorldPoint( geometry.transform, { center[0], center[1] } );
		int centerFace = 0;
		for ( int i = 1; i < geometry.faceCount(); ++i )
			if ( geometry.separation( centerLocal, i ) >
				 geometry.separation( centerLocal, centerFace ) )
				centerFace = i;
		for ( int node = 0; node < body().nodeCount; ++node )
		{
			auto p = geometry.local( node );
			int face = geometry.retainedFace( node );
			if ( face >= 0 && geometry.separation( p, face ) <= Margin )
				continue;
			face = 0;
			for ( int i = 1; i < geometry.faceCount(); ++i )
				if ( geometry.separation( p, i ) > geometry.separation( p, face ) )
					face = i;
			// A newly registered body may already overlap a box. Its exterior
			// centre supplies a coherent initial side, rather than pinning its
			// nodes to opposite faces of the same thin plate.
			if ( box.nodeFaces[node] < 0 && geometry.separation( p, face ) < 0 &&
				 geometry.separation( centerLocal, centerFace ) > 0 )
				face = centerFace;
			box.nodeFaces[node] = face;
		}
	}
}

float Solver::limitCorrection( int node, Core::Vec delta )
{
	float fraction = 1;
	for ( int j = 0; j < boxCount; ++j )
	{
		const auto& box = boxes[j];
		if ( !box.enabled )
			continue;
		if ( box.geometry.kind == RigidGeometry::Chain )
		{
			float reach = std::hypot( delta[0], delta[1] );
			for ( int edge : { node, ( node + body().nodeCount - 1 ) % body().nodeCount } )
			{
				ChainHit hits[2];
				int count = chainHits( *this, j, edge, hits, reach );
				for ( int i = 0; i < count; ++i )
				{
					const auto& hit = hits[i];
					float weight = edge == node ? 1 - hit.row.fraction : hit.row.fraction;
					float toward = weight * b2Dot( { delta[0], delta[1] }, hit.row.normal );
					if ( toward < 0 )
						fraction = std::min( fraction, std::max( 0.f, hit.gap ) / -toward );
				}
			}
			continue;
		}
		BoxProjection geometry( body(), box.state, box.geometry );
		geometry.faceMargin = Margin;
		if ( box.compound || box.curved )
		{
			if ( box.stateSlot != j )
				continue;
			auto p = geometry.local( node );
			b2Vec2 q;
			int edge;
			if ( !nearestCompound( j, p, q, edge ) )
				continue;
			if ( b2Length( b2Sub( p, q ) ) > std::hypot( delta[0], delta[1] ) + Margin &&
				 !insideCompound( j, p ) )
				continue;
			auto n = box.boundary[edge].normalAt( q );
			auto d = b2InvRotateVector( geometry.transform.q, { delta[0], delta[1] } );
			float toward = b2Dot( d, n ), gap = std::max( 0.f, b2Dot( b2Sub( p, q ), n ) );
			if ( toward < 0 )
				fraction = std::min( fraction, gap / -toward );
			continue;
		}
		int face = geometry.circular() ? 0 : box.nodeFaces[node];
		if ( face < 0 )
			continue;
		auto p = geometry.local( node );
		if ( !geometry.withinFace( p, face ) )
			continue;
		auto d = b2InvRotateVector( geometry.transform.q, { delta[0], delta[1] } );
		auto normal =
			geometry.circular() ? geometry.radialNormal( p ) : geometry.faceNormal( face );
		float toward = b2Dot( d, normal );
		if ( toward >= 0 )
			continue;
		fraction = std::min( fraction, std::max( 0.f, geometry.separation( p, face ) / -toward ) );
	}
	return fraction;
}

Core::Vec Solver::projectTranslation( Core::Vec desired )
{
	// Project onto exterior support planes. Circle normals are local to each
	// material point, so they are recomputed rather than retained as face IDs.
	auto& planes = translationPlanes;
	int count = 0;
	for ( int j = 0; j < boxCount; ++j )
	{
		const auto& box = boxes[j];
		if ( !box.enabled )
			continue;
		if ( box.geometry.kind == RigidGeometry::Chain )
		{
			float reach = std::hypot( desired[0], desired[1] );
			for ( int edge = 0; edge < body().nodeCount; ++edge )
			{
				ChainHit hits[2];
				int hitCount = chainHits( *this, j, edge, hits, reach );
				for ( int i = 0; i < hitCount; ++i )
					planes[count++] = { hits[i].row.normal, std::max( 0.f, hits[i].gap ) };
			}
			continue;
		}
		BoxProjection geometry( body(), box.state, box.geometry );
		geometry.faceMargin = Margin;
		if ( box.compound || box.curved )
		{
			if ( box.stateSlot != j )
				continue;
			for ( int node = 0; node < body().nodeCount; ++node )
			{
				auto p = geometry.local( node );
				b2Vec2 q;
				int edge;
				if ( !nearestCompound( j, p, q, edge ) )
					continue;
				float gap = b2Length( b2Sub( p, q ) );
				if ( gap > std::hypot( desired[0], desired[1] ) + Margin &&
					 !insideCompound( j, p ) )
					continue;
				auto n = box.boundary[edge].normalAt( q );
				planes[count++] = { b2RotateVector( geometry.transform.q, n ),
									std::max( 0.f, b2Dot( b2Sub( p, q ), n ) ) };
			}
			continue;
		}
		if ( geometry.circular() )
		{
			for ( int node = 0; node < body().nodeCount; ++node )
			{
				auto p = geometry.local( node );
				float gap = std::max( 0.f, geometry.separation( p, 0 ) );
				// Projection onto a convex set containing zero cannot increase
				// the target norm; more distant planes cannot affect this result.
				if ( gap > std::hypot( desired[0], desired[1] ) + Margin )
					continue;
				planes[count++] = {
					b2RotateVector( geometry.transform.q, geometry.radialNormal( p ) ),
					std::max( 0.f, geometry.separation( p, 0 ) ) };
			}
			continue;
		}
		for ( int face = 0; face < geometry.faceCount(); ++face )
		{
			float gap = 1e30f;
			for ( int node = 0; node < body().nodeCount; ++node )
			{
				if ( box.nodeFaces[node] != face )
					continue;
				auto p = geometry.local( node );
				if ( !geometry.withinFace( p, face ) )
					continue;
				gap = std::min( gap, std::max( 0.f, geometry.separation( p, face ) ) );
			}
			if ( gap < 1e30f )
				planes[count++] = {
					b2RotateVector( geometry.transform.q, geometry.faceNormal( face ) ), gap };
		}
	}
	b2Vec2 target{ desired[0], desired[1] }, best{};
	float bestError = b2LengthSquared( target );
	auto consider = [&]( b2Vec2 candidate ) {
		for ( int i = 0; i < count; ++i )
			if ( b2Dot( planes[i].normal, candidate ) + planes[i].gap < -1e-6f )
				return;
		float error = b2LengthSquared( b2Sub( candidate, target ) );
		if ( error < bestError )
		{
			best = candidate;
			bestError = error;
		}
	};
	consider( target );
	for ( int i = 0; i < count; ++i )
	{
		const auto& a = planes[i];
		consider( b2MulAdd( target, -a.gap - b2Dot( a.normal, target ), a.normal ) );
		for ( int j = i + 1; j < count; ++j )
		{
			const auto& b = planes[j];
			float det = b2Cross( a.normal, b.normal );
			if ( std::abs( det ) < 1e-6f )
				continue;
			consider( { ( -a.gap * b.normal.y + b.gap * a.normal.y ) / det,
						( -b.gap * a.normal.x + a.gap * b.normal.x ) / det } );
		}
	}
	return { best.x, best.y };
}

bool Solver::observeRecovery( bool endOfStep )
{
	Profile::Scope timing( profile, Profile::GeometryRecovery );
	auto topology = inspectTopology( body() );
	if ( !topology.finite )
	{
		error = "nonfinite_state";
		return false;
	}
	if ( recovery.observe( topology, endOfStep ) )
		clearCache();
	return true;
}

float Solver::inverseMass( const Row& row, b2Vec2 direction ) const
{
	const auto& rigid = rigidState( row.box );
	auto p = point( row );
	float lever =
		b2Cross( { float( p.x - rigid.center.x ), float( p.y - rigid.center.y ) }, direction );
	double result = softInverse( row ) + rigid.inverseMass + rigid.inverseInertia * lever * lever;
	return float( result );
}

float Solver::relativeSpeed( const Row& row, b2Vec2 direction ) const
{
	const auto& body = this->body();
	const auto& rigid = rigidState( row.box );
	auto p = point( row );
	float vx = ( 1 - row.fraction ) * body.v[row.edge][0] +
			   row.fraction * body.v[( row.edge + 1 ) % body.nodeCount][0];
	float vy = ( 1 - row.fraction ) * body.v[row.edge][1] +
			   row.fraction * body.v[( row.edge + 1 ) % body.nodeCount][1];
	return b2Dot( { vx - rigid.velocity.x + rigid.angularVelocity * float( p.y - rigid.center.y ),
					vy - rigid.velocity.y - rigid.angularVelocity * float( p.x - rigid.center.x ) },
				  direction );
}

void Solver::apply( const Row& row, b2Vec2 impulse )
{
	auto& body = this->body();
	auto& rigid = rigidState( row.box );
	auto p = point( row );
	double px = 0, py = 0;
	for ( int side = 0; side < 2; ++side )
	{
		int node = ( row.edge + side ) % body.nodeCount;
		float weight = side ? row.fraction : 1 - row.fraction;
		float vx = body.v[node][0], vy = body.v[node][1];
		body.v[node][0] += weight * impulse.x / body.mass[node];
		body.v[node][1] += weight * impulse.y / body.mass[node];
		px += body.mass[node] * double( body.v[node][0] - vx );
		py += body.mass[node] * double( body.v[node][1] - vy );
	}
	if ( rigid.inverseMass > 0 )
	{
		auto before = rigid.velocity;
		rigid.velocity = b2MulAdd( rigid.velocity, -rigid.inverseMass, impulse );
		rigid.angularVelocity -=
			rigid.inverseInertia *
			b2Cross( { float( p.x - rigid.center.x ), float( p.y - rigid.center.y ) }, impulse );
		px += ( rigid.velocity.x - before.x ) / rigid.inverseMass;
		py += ( rigid.velocity.y - before.y ) / rigid.inverseMass;
		contactMomentumResidual = std::max( contactMomentumResidual, std::hypot( px, py ) );
	}
}

void Solver::solve( Row& row )
{
	++rowSolves;
	auto& rigid = rigidState( row.box );
	auto surface = b2TransformWorldPoint( { rigid.origin, rigid.rotation }, row.localAnchor );
	auto p = point( row );
	float separation = b2Dot( { float( p.x - surface.x ), float( p.y - surface.y ) }, row.normal );
	// Positive separation admits only the velocity needed to reach the face.
	float bias = std::max( 0.f, separation ) / soft.dt;
	float normalMass = inverseMass( row, row.normal );
	if ( !( normalMass > 0 ) )
	{
		error = "contact_effective_mass";
		return;
	}
	float normal = std::max( 0.f, row.normalImpulse -
									  ( relativeSpeed( row, row.normal ) + bias ) / normalMass );
	apply( row, b2MulSV( normal - row.normalImpulse, row.normal ) );
	row.normalImpulse = normal;
	b2Vec2 tangent = { -row.normal.y, row.normal.x };
	float limit = std::sqrt( friction * boxes[row.box].friction ) * normal;
	float tangentMass = inverseMass( row, tangent );
	if ( !( tangentMass > 0 ) )
	{
		error = "contact_effective_mass";
		return;
	}
	float next =
		std::max( -limit, std::min( limit, row.tangentImpulse -
											   relativeSpeed( row, tangent ) / tangentMass ) );
	apply( row, b2MulSV( next - row.tangentImpulse, tangent ) );
	row.tangentImpulse = next;
}

void Solver::reflect( View& state ) const
{
	if ( !flipY )
		return;
	state.center.y = -state.center.y;
	state.origin.y = -state.origin.y;
	state.rotation.s = -state.rotation.s;
	state.velocity.y = -state.velocity.y;
	state.angularVelocity = -state.angularVelocity;
	state.localCenter.y = -state.localCenter.y;
}

bool Solver::read( void* step, b2BodyId id, View& state, int& index ) const
{
	if ( !Bridge::readStepBody( step, id, state, index ) )
		return false;
	reflect( state );
	return true;
}

void Solver::write( void* step, int index, View state ) const
{
	reflect( state );
	Bridge::writeStepBody( step, index, state );
}

bool Solver::validate()
{
	if ( soft.failure )
	{
		error = soft.failure;
		return false;
	}
	if ( !( soft.dt > 0 ) || !std::isfinite( 1.f / soft.dt ) || soft.substeps < 1 ||
		 soft.substeps > 128 || soft.iterations != Sweeps )
	{
		error = "unsupported_game_step";
		return false;
	}
	if ( bodyIndex < 0 || size_t( bodyIndex ) >= soft.bodies.size() || !body().active )
	{
		error = "unsupported_game_body";
		return false;
	}
	const auto& body = this->body();
	// A circle contributes node and edge rows; a polygon contributes node and vertex rows.
	// Allocate during preflight, before entering Box2D; retain across steps.
	size_t capacity = size_t( std::max( 1, boxCount ) ) *
					  ( 2 * size_t( body.nodeCount ) + B2_MAX_POLYGON_VERTICES );
	if ( maxChainSegments )
		chainPatches.reserve( 4 * size_t( boxCount ) );
	if ( rows.size() < capacity )
		rows.resize( capacity );
	if ( previous.size() < capacity )
		previous.resize( capacity );
	translationPlanes.resize( size_t( std::max( 1, boxCount ) ) *
							  ( 2 * size_t( body.nodeCount ) + B2_MAX_POLYGON_VERTICES ) );
	recovery.remaining.resize( body.nodeCount );
	for ( auto& box : boxes )
		box.nodeFaces.resize( body.nodeCount, -1 );

	if ( !( friction >= 0 && friction <= 10 ) )
	{
		error = "invalid_game_friction";
		return false;
	}
	for ( int i = 0; i < body.nodeCount; ++i )
	{
		if ( !( body.mass[i] > 0 ) || !std::isfinite( body.mass[i] ) )
		{
			error = "invalid_game_mass";
			return false;
		}
	}
	return true;
}
} // namespace Game
} // namespace SoftBody
