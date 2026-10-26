#include "soft_game_world.h"
extern "C"
{
#include "../../src/physics_world.h"
}

#if defined( __clang__ )
#pragma clang fp contract( off )
#endif

namespace SoftBody
{
namespace Game
{
bool Solver::add( b2ShapeId shape )
{
	if ( boxCount == ( extendedGeometry ? MaxCompoundShapes : MaxBoxes ) )
		return false;
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
		if ( pin.active && B2_ID_EQUALS( pin.rigid, box.body ) )
		{
			error = "game_support_contact_unsupported";
			return false;
		}
		RigidGeometry geometry;
		if ( extendedGeometry )
		{
			if ( b2Shape_GetRestitution( box.shape ) != 0 )
			{
				error = "game_requires_zero_bounce_shape";
				return false;
			}
			if ( !geometry.read( box.shape, flipY ) )
			{
				error = "game_requires_circle_or_sharp_convex_polygon";
				return false;
			}
		}
		else
		{
			if ( b2Shape_GetType( box.shape ) != b2_polygonShape )
			{
				error = "game_requires_box";
				return false;
			}
			if ( b2Body_GetShapeCount( box.body ) != 1 || b2Shape_GetRestitution( box.shape ) != 0 )
			{
				error = "game_requires_single_zero_bounce_box";
				return false;
			}
			auto polygon = b2Shape_GetPolygon( box.shape );
			if ( polygon.count != 4 || polygon.radius != 0 )
			{
				error = "game_requires_sharp_box";
				return false;
			}
			float hx = 0, hy = 0;
			for ( int j = 0; j < 4; ++j )
			{
				hx = std::max( hx, std::abs( polygon.vertices[j].x ) );
				hy = std::max( hy, std::abs( polygon.vertices[j].y ) );
			}
			unsigned corners = 0;
			for ( int j = 0; j < 4; ++j )
			{
				auto p = polygon.vertices[j];
				if ( std::abs( std::abs( p.x ) - hx ) > 1e-6f ||
					 std::abs( std::abs( p.y ) - hy ) > 1e-6f )
				{
					error = "game_requires_centered_box";
					return false;
				}
				corners |= 1u << ( ( p.x > 0 ? 1 : 0 ) + ( p.y > 0 ? 2 : 0 ) );
			}
			if ( !( hx > 0 && hy > 0 ) || corners != 15 )
			{
				error = "invalid_box";
				return false;
			}
			geometry.hx = hx;
			geometry.hy = hy;
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
				if ( !extendedGeometry )
				{
					error = "game_duplicate_body";
					return false;
				}
				if ( box.geometry.kind == RigidGeometry::Circle ||
					 boxes[j].geometry.kind == RigidGeometry::Circle )
				{
					error = "compound_requires_sharp_polygons";
					return false;
				}
				box.stateSlot = boxes[j].stateSlot;
				break;
			}
	}
	prepareCompounds();
	if ( pin.active && ( error = Bridge::validateStepBody( world, pin.rigid ) ) )
		return false;
	return true;
}

void Solver::prepareCompounds()
{
	for ( int i = 0; i < boxCount; ++i )
		boxes[i].compound = false;
	for ( int i = 0; i < boxCount; ++i )
		if ( boxes[i].enabled && boxes[i].stateSlot != i )
			boxes[boxes[i].stateSlot].compound = true;
	for ( int i = 0; i < boxCount; ++i )
		if ( boxes[i].enabled )
			boxes[i].compound = boxes[boxes[i].stateSlot].compound;
	if ( !boundaryDirty )
		return;
	for ( int i = 0; i < boxCount; ++i )
	{
		auto& box = boxes[i];
		box.boundary.clear();
		if ( !box.compound || box.stateSlot != i )
			continue;
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
				capacity += boxes[i].compound ? size_t( boxCount + 1 ) * body().nodeCount +
													2 * boxes[i].boundary.size()
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
		auto d = b2Sub( edges[i].b, edges[i].a );
		float t = std::max(
			0.f, std::min( 1.f, b2Dot( b2Sub( p, edges[i].a ), d ) / b2LengthSquared( d ) ) );
		auto point = b2MulAdd( edges[i].a, t, d );
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
		return;

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
	if ( !recovery.active )
	{
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
	}

	geometryCorrections += projection.corrections;
	return observeRecovery();
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
		for ( int i = 0; i < previousCount; ++i )
		{
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
	rows[rowCount++] = row;
}

void Solver::gather()
{
	rowCount = 0;
	auto& body = this->body();
	for ( int id = 0; id < boxCount; ++id )
	{
		auto& box = boxes[id];
		if ( !box.enabled )
			continue;
		b2WorldTransform transform = { box.state.origin, box.state.rotation };
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
	// Within this call only BoxProjection::move can change node positions;
	// it increments corrections before every write. Velocities stay unchanged.
	// Repeated endOfStep=false observations of unchanged topology are idempotent.
	// Keep the observation local so edits between calls cannot reuse stale data.
	bool observed = false;
	for ( int sweep = 0; sweep < Sweeps; ++sweep )
		for ( int j = 0; j < boxCount; ++j )
		{
			int id = j;
			auto& box = boxes[id];
			if ( !box.enabled )
				continue;
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
			if ( geometry.error )
			{
				error = geometry.error;
				return false;
			}
			if ( geometry.corrections != 0 && !observeRecovery() )
				return false;
		}
	return true;
}

void Solver::captureBoundarySides()
{
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
		if ( box.compound || geometry.circular() )
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
		BoxProjection geometry( body(), box.state, box.geometry );
		geometry.faceMargin = Margin;
		if ( box.compound )
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
			auto n = box.boundary[edge].normal;
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
		BoxProjection geometry( body(), box.state, box.geometry );
		geometry.faceMargin = Margin;
		if ( box.compound )
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
				auto n = box.boundary[edge].normal;
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

double Solver::anchorCoupling( const Row& row ) const
{
	if ( !pin.active )
		return 0;
	const auto& body = this->body();
	return pin.weights[row.edge] * double( 1 - row.fraction ) / body.mass[row.edge] +
		   pin.weights[( row.edge + 1 ) % body.nodeCount] * double( row.fraction ) /
			   body.mass[( row.edge + 1 ) % body.nodeCount];
}

b2Vec2 Solver::compensatingImpulse( const Row& row, b2Vec2 impulse ) const
{
	double coupling = anchorCoupling( row );
	return { float( -coupling * ( pinInverseXX * impulse.x + pinInverseXY * impulse.y ) ),
			 float( -coupling * ( pinInverseXY * impulse.x + pinInverseYY * impulse.y ) ) };
}

bool Solver::preparePinResponse( void* step )
{
	if ( !pin.active )
		return true;
	View rigid{};
	int index;
	if ( !read( step, pin.rigid, rigid, index ) )
		return false;
	auto anchor = b2TransformWorldPoint( { rigid.origin, rigid.rotation }, localPinAnchor() );
	double rx = anchor.x - rigid.center.x, ry = anchor.y - rigid.center.y;
	double inverse = rigid.inverseMass;
	for ( int i = 0; i < body().nodeCount; ++i )
		inverse += double( pin.weights[i] ) * pin.weights[i] / body().mass[i];
	double xx = inverse + rigid.inverseInertia * ry * ry;
	double xy = -rigid.inverseInertia * rx * ry;
	double yy = inverse + rigid.inverseInertia * rx * rx;
	double determinant = xx * yy - xy * xy;
	if ( !( determinant > 0 ) )
		return false;
	pinInverseXX = yy / determinant;
	pinInverseXY = -xy / determinant;
	pinInverseYY = xx / determinant;
	return true;
}

float Solver::inverseMass( const Row& row, b2Vec2 direction ) const
{
	const auto& rigid = rigidState( row.box );
	auto p = point( row );
	float lever =
		b2Cross( { float( p.x - rigid.center.x ), float( p.y - rigid.center.y ) }, direction );
	double result = softInverse( row ) + rigid.inverseMass + rigid.inverseInertia * lever * lever;
	double coupling = anchorCoupling( row );
	// Schur complement of the two-dimensional bilateral attachment:
	// K_contact - G * inverse(K_pin) * transpose(G).
	result -=
		coupling * coupling *
		( direction.x * direction.x * pinInverseXX + 2 * direction.x * direction.y * pinInverseXY +
		  direction.y * direction.y * pinInverseYY );
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
	if ( pin.active )
	{
		// The contact impulse must include its attachment reaction as well
		// as its reduced effective mass. Keep the rigid response local to
		// the existing solver-state bridge; never integrate this body here.
		auto correction = compensatingImpulse( row, impulse );
		this->impulse = b2Add( this->impulse, correction );
		View support{};
		int index;
		if ( !read( activeStep, pin.rigid, support, index ) )
		{
			error = "attachment_state";
			return;
		}
		auto anchor =
			b2TransformWorldPoint( { support.origin, support.rotation }, localPinAnchor() );
		b2Vec2 r = { float( anchor.x - support.center.x ), float( anchor.y - support.center.y ) };
		for ( int i = 0; i < body.nodeCount; ++i )
		{
			body.v[i][0] += pin.weights[i] * correction.x / body.mass[i];
			body.v[i][1] += pin.weights[i] * correction.y / body.mass[i];
		}
		support.velocity = b2MulAdd( support.velocity, -support.inverseMass, correction );
		support.angularVelocity -= support.inverseInertia * b2Cross( r, correction );
		write( activeStep, index, support );
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
	float bias = std::max( 0.f, separation ) / Step;
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

bool Solver::attachment( void* step, bool bias )
{
	if ( !pin.active )
		return true;
	View rigid{};
	int index;
	if ( !read( step, pin.rigid, rigid, index ) )
		return false;
	++pinSolves;
	auto& body = this->body();
	auto anchor = b2TransformWorldPoint( { rigid.origin, rigid.rotation }, localPinAnchor() );
	b2Vec2 r = { float( anchor.x - rigid.center.x ), float( anchor.y - rigid.center.y ) };
	double inverse = rigid.inverseMass, x = 0, y = 0, vx = 0, vy = 0;
	for ( int i = 0; i < body.nodeCount; ++i )
	{
		double weight = pin.weights[i];
		inverse += weight * weight / body.mass[i];
		x += weight * body.x[i][0];
		y += weight * body.x[i][1];
		vx += weight * body.v[i][0];
		vy += weight * body.v[i][1];
	}
	double kxx = inverse + rigid.inverseInertia * r.y * r.y;
	double kxy = -rigid.inverseInertia * r.x * r.y;
	double kyy = inverse + rigid.inverseInertia * r.x * r.x;
	double cx = vx - rigid.velocity.x + rigid.angularVelocity * r.y +
				( bias ? ( x - anchor.x ) / Step : 0 );
	double cy = vy - rigid.velocity.y - rigid.angularVelocity * r.x +
				( bias ? ( y - anchor.y ) / Step : 0 );
	double determinant = kxx * kyy - kxy * kxy;
	if ( !( determinant > 0 ) )
		return false;
	b2Vec2 impulse = { float( -( kyy * cx - kxy * cy ) / determinant ),
					   float( -( kxx * cy - kxy * cx ) / determinant ) };
	for ( int i = 0; i < body.nodeCount; ++i )
	{
		body.v[i][0] += pin.weights[i] * impulse.x / body.mass[i];
		body.v[i][1] += pin.weights[i] * impulse.y / body.mass[i];
	}
	rigid.velocity = b2MulAdd( rigid.velocity, -rigid.inverseMass, impulse );
	rigid.angularVelocity -= rigid.inverseInertia * b2Cross( r, impulse );
	write( step, index, rigid );
	this->impulse = b2Add( this->impulse, impulse );
	return true;
}

void Solver::stage( void* step, int phase, void* context )
{
	auto& solver = *static_cast<Solver*>( context );
	if ( solver.error )
		return;
	if ( phase != solver.nextPhase++ )
	{
		solver.error = "phase_order";
		return;
	}
	if ( phase == 0 )
	{
		solver.soft.advanceMaterialVelocity();
		for ( auto& touch : solver.soft.touchJoints )
			if ( touch.active )
				solver.soft.solveTouchJoint( touch );
		{
			if ( solver.soft.failure )
				solver.error = solver.soft.failure;
			else if ( !inspectTopology( solver.body() ).finite )
				solver.error = "nonfinite_state";
		}
		return;
	}
	if ( phase == 1 )
	{
		if ( !solver.attachment( step, true ) )
			solver.error = "attachment_state";
		return;
	}
	for ( int i = 0; i < solver.body().nodeCount; ++i )
		for ( int axis = 0; axis < 2; ++axis )
			solver.body().x[i][axis] += Step * solver.body().v[i][axis];
	if ( !solver.observeRecovery() )
		return;
	solver.lastDepth = 0;
	for ( int i = 0; i < solver.boxCount; ++i )
	{
		auto& box = solver.boxes[i];
		if ( !box.enabled )
			continue;
		if ( b2Body_GetType( box.body ) != b2_staticBody &&
			 !solver.read( step, box.body, box.state, box.index ) )
		{
			solver.error = "contact_state";
			return;
		}
	}
	solver.activeStep = step;
	if ( !solver.preparePinResponse( step ) ||
		 ( solver.pin.active && !solver.attachment( step, false ) ) )
	{
		solver.error = "attachment_response";
		return;
	}
	if ( !solver.projectGeometry() )
		return;
	solver.gather();
	if ( solver.error )
		return;
	for ( int i = 0; i < solver.rowCount; ++i )
	{
		auto& row = solver.rows[i];
		b2Vec2 tangent = { -row.normal.y, row.normal.x };
		solver.apply( row, b2Add( b2MulSV( row.normalImpulse, row.normal ),
								  b2MulSV( row.tangentImpulse, tangent ) ) );
	}
	for ( int sweep = 0; sweep < Sweeps && !solver.error; ++sweep )
		for ( int j = 0; j < solver.rowCount && !solver.error; ++j )
			solver.solve( solver.rows[j] );
	if ( solver.error )
		return;
	for ( int i = 0; i < solver.boxCount; ++i )
	{
		auto& box = solver.boxes[i];
		if ( !box.enabled )
			continue;
		double depth;
		if ( !inspectTopology( solver.body() ).valid() )
		{
			// Only boundary segments have a defined overlap while folded;
			// this metric is not an interior penetration certificate.
			BoxProjection geometry( solver.body(), box.state, box.geometry );
			depth = geometry.boundaryDepth();
		}
		else
			depth = Bridge::statePenetration( solver.soft, 0, box.state, box.hx, box.hy );
		solver.maxDepth = std::max( solver.maxDepth, depth );
		solver.lastDepth = std::max( solver.lastDepth, depth );
		if ( box.index >= 0 )
			solver.write( step, box.index, box.state );
	}
	for ( int i = 0; i < solver.rowCount; ++i )
	{
		solver.normalTotal += solver.rows[i].normalImpulse;
		solver.tangentTotal += std::abs( solver.rows[i].tangentImpulse );
	}
	solver.previous = solver.rows;
	solver.previousCount = solver.rowCount;
}

bool Solver::advance()
{
	if ( error || !validate() )
		return false;
	if ( !observeRecovery() )
		return false;
	recovery.remaining.assign( body().nodeCount, recovery.maxCorrection );
	if ( recovery.active )
		clearCache();
	if ( error || !prepare() )
		return false;
	nextPhase = 0;
	impulse = b2Vec2_zero;
	boundWorld = world;
	boundFlipY = flipY;
	++entered;
	b2World_StepWithSoftStages( world, Step, 1, nullptr, nullptr, stage, this );
	// Box2D does not run its solver with an empty awake set. If every
	// registered target is static, no rigid solver state is needed at all.
	// Complete the soft-only phases after the rigid step, exactly once.
	// Never manufacture a hidden awake rigid body just to obtain callbacks.
	bool staticOnly = !pin.active;
	for ( int i = 0; i < boxCount; ++i )
		if ( boxes[i].enabled && b2Body_GetType( boxes[i].body ) != b2_staticBody )
			staticOnly = false;
	if ( nextPhase == 0 && staticOnly )
		for ( int phase = 0; phase < 3; ++phase )
			stage( nullptr, phase, this );
	activeStep = nullptr;
	// Compare next prepare against the completed public pose, not the old
	// pose or a velocity prediction. Ordinary integration is not teleport.
	for ( int i = 0; i < boxCount; ++i )
		boxes[i].cachedTransform = b2Body_GetTransform( boxes[i].body );
	++steps;
	if ( soft.failure )
		error = soft.failure;
	if ( nextPhase != 3 && !error )
		error = "missing_phase";
	auto metrics = soft.currentMetrics();
	if ( !error )
		observeRecovery( true );
	if ( lastDepth > 1e-6 )
		++overlapSteps;
	for ( int i = 0; i < body().nodeCount; ++i )
		for ( int axis = 0; axis < 2; ++axis )
			if ( !std::isfinite( body().x[i][axis] ) || !std::isfinite( body().v[i][axis] ) )
				error = "nonfinite_state";
	if ( pin.active )
	{
		auto a = soft.anchor( bodyIndex, pin.weights );
		auto b = b2Body_GetWorldPoint( pin.rigid, pin.localAnchor );
		maxGap = std::max(
			maxGap, std::hypot( double( a[0] ) - b.x, double( a[1] ) - ( flipY ? -b.y : b.y ) ) );
	}
	if ( !error )
	{
		++soft.completedSubsteps;
		soft.peak = metrics;
	}
	return error == nullptr;
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

b2Vec2 Solver::localPinAnchor() const
{
	return { pin.localAnchor.x, flipY ? -pin.localAnchor.y : pin.localAnchor.y };
}

bool Solver::validate()
{
	if ( entered &&
		 ( b2StoreWorldId( boundWorld ) != b2StoreWorldId( world ) || boundFlipY != flipY ) )
	{
		error = "game_owner_changed";
		return false;
	}
	if ( pin.active != cachedPin.active || !B2_ID_EQUALS( pin.rigid, cachedPin.rigid ) ||
		 pin.weights != cachedPin.weights || pin.localAnchor.x != cachedPin.localAnchor.x ||
		 pin.localAnchor.y != cachedPin.localAnchor.y )
	{
		clearCache();
		cachedPin = pin;
	}
	if ( soft.failure )
	{
		error = soft.failure;
		return false;
	}
	if ( soft.dt != Step || soft.substeps != 1 || soft.iterations != Sweeps )
	{
		error = "unsupported_game_step";
		return false;
	}
	if ( bodyIndex < 0 || size_t( bodyIndex ) >= soft.bodies.size() || !body().active ||
		 body().kind != 3 ||
		 ( !sharedWorld &&
		   ( soft.bodies.size() != 1 || !soft.pins.empty() || !soft.boundaries.empty() ) ) )
	{
		error = "unsupported_game_body";
		return false;
	}
	const auto& body = this->body();
	// A circle contributes node and edge rows; a polygon contributes node and vertex rows.
	// Allocate during preflight, before entering Box2D; retain across steps.
	size_t capacity = size_t( std::max( 1, boxCount ) ) *
					  ( 2 * size_t( body.nodeCount ) + B2_MAX_POLYGON_VERTICES );
	if ( rows.size() < capacity )
		rows.resize( capacity );
	if ( previous.size() < capacity )
		previous.resize( capacity );
	translationPlanes.resize( size_t( std::max( 1, boxCount ) ) *
							  ( size_t( body.nodeCount ) + B2_MAX_POLYGON_VERTICES ) );
	recovery.remaining.resize( body.nodeCount );
	for ( auto& box : boxes )
		box.nodeFaces.resize( body.nodeCount, -1 );
	if ( body.forceInput[0] != 0 || body.forceInput[1] != 0 || body.torqueInput != 0 )
	{
		error = "unsupported_game_load";
		return false;
	}
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
	int touches = 0;
	for ( const auto& touch : soft.touchJoints )
		if ( !sharedWorld && touch.active )
		{
			if ( ++touches > 1 || touch.body != 0 ||
				 touch.weights.size() != size_t( body.nodeCount ) ||
				 !std::isfinite( touch.target[0] ) || !std::isfinite( touch.target[1] ) ||
				 !( touch.frequency >= .01f && touch.frequency <= 60 ) ||
				 !( touch.dampingRatio >= 0 && touch.dampingRatio <= 10 ) ||
				 !( touch.maxForce >= 0 && touch.maxForce <= 1e5f ) )
			{
				error = "invalid_game_touch";
				return false;
			}
			double total = 0;
			for ( float weight : touch.weights )
			{
				if ( !( weight >= 0 && weight <= 1 ) )
				{
					error = "invalid_game_weights";
					return false;
				}
				total += weight;
			}
			if ( std::abs( total - 1 ) > 1e-6 )
			{
				error = "invalid_game_weights";
				return false;
			}
		}
	if ( pin.active )
	{
		double total = 0;
		for ( float weight : pin.weights )
		{
			if ( !( weight >= 0 && weight <= 1 ) )
			{
				error = "invalid_game_pin";
				return false;
			}
			total += weight;
		}
		if ( pin.softBody != 0 || pin.weights.size() != size_t( body.nodeCount ) ||
			 std::abs( total - 1 ) > 1e-6 || !std::isfinite( pin.localAnchor.x ) ||
			 !std::isfinite( pin.localAnchor.y ) )
		{
			error = "invalid_game_pin";
			return false;
		}
	}
	return true;
}
} // namespace Game
} // namespace SoftBody
