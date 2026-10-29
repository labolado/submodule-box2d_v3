#include "soft_game_geometry.h"

#include <limits>
extern "C"
{
#include "../../src/physics_world.h"
#include "../../src/shape.h"
}

#if defined( __clang__ )
#pragma clang fp contract( off )
#endif

namespace SoftBody
{
namespace Game
{

b2TreeStats queryStaticShapes( b2WorldId id, b2AABB bounds, b2OverlapResultCallbackFcn* callback,
							   void* context )
{
	struct Query
	{
		b2World* world;
		b2OverlapResultCallbackFcn* callback;
		void* context;
	} query{ b2GetWorldFromId( id ), callback, context };
	B2_ASSERT( !query.world->locked );
	if ( query.world->locked )
		return {};
	return b2DynamicTree_OverlapAABBAll(
		query.world->broadPhase.trees + b2_staticBody, bounds,
		[]( int, uint64_t data, void* user ) {
			auto& q = *static_cast<Query*>( user );
			auto* shape = b2Array_Get( q.world->shapes, int( data ) );
			return q.callback( { int( data ) + 1, q.world->worldId, shape->generation },
							   q.context );
		},
		&query );
}

bool RigidGeometry::read( b2ShapeId shape, bool flipY )
{
	if ( b2Shape_GetType( shape ) == b2_chainSegmentShape )
	{
		if ( b2Body_GetType( b2Shape_GetBody( shape ) ) != b2_staticBody )
			return false;
		kind = Chain;
		chain = b2Shape_GetChainSegment( shape );
		if ( flipY )
		{
			// Reflection reverses handedness. Reverse endpoints AND ghosts so
			// the chain's right-facing collision side is reflected correctly.
			std::swap( chain.segment.point1, chain.segment.point2 );
			std::swap( chain.ghost1, chain.ghost2 );
			chain.segment.point1.y = -chain.segment.point1.y;
			chain.segment.point2.y = -chain.segment.point2.y;
			chain.ghost1.y = -chain.ghost1.y;
			chain.ghost2.y = -chain.ghost2.y;
		}
		return b2LengthSquared( b2Sub( chain.segment.point2, chain.segment.point1 ) ) > 1e-6f;
	}
	if ( b2Shape_GetType( shape ) == b2_circleShape )
	{
		kind = Circle;
		circle = b2Shape_GetCircle( shape );
		if ( flipY )
			circle.center.y = -circle.center.y;
		return circle.radius > 0;
	}
	if ( b2Shape_GetType( shape ) != b2_polygonShape )
	{
		if ( b2Shape_GetType( shape ) != b2_capsuleShape )
			return false;
		kind = Capsule;
		capsule = b2Shape_GetCapsule( shape );
		if ( flipY )
		{
			capsule.center1.y = -capsule.center1.y;
			capsule.center2.y = -capsule.center2.y;
		}
		return capsule.radius > 0 && b2DistanceSquared( capsule.center1, capsule.center2 ) > 1e-12f;
	}
	auto source = b2Shape_GetPolygon( shape );
	if ( source.radius < 0 || source.count < 3 || source.count > B2_MAX_POLYGON_VERTICES )
		return false;
	polygon = source;
	if ( flipY )
		for ( int i = 0; i < source.count; ++i )
		{
			auto v = source.vertices[source.count - 1 - i];
			auto n = source.normals[( 2 * source.count - 2 - i ) % source.count];
			polygon.vertices[i] = { v.x, -v.y };
			polygon.normals[i] = { n.x, -n.y };
		}
	hx = hy = 0;
	for ( int i = 0; i < source.count; ++i )
	{
		hx = std::max( hx, std::abs( source.vertices[i].x ) );
		hy = std::max( hy, std::abs( source.vertices[i].y ) );
	}
	unsigned corners = 0;
	for ( int i = 0; i < source.count; ++i )
	{
		auto v = source.vertices[i];
		if ( std::abs( std::abs( v.x ) - hx ) > 1e-6f || std::abs( std::abs( v.y ) - hy ) > 1e-6f )
			break;
		corners |= 1u << ( ( v.x > 0 ? 1 : 0 ) + ( v.y > 0 ? 2 : 0 ) );
	}
	kind = source.count == 4 && hx > 0 && hy > 0 && corners == 15 ? Rectangle : Polygon;
	if ( source.radius > 0 )
		kind = RoundedPolygon;
	return true;
}

bool RigidGeometry::equals( const RigidGeometry& other ) const
{
	if ( kind != other.kind )
		return false;
	if ( kind == Rectangle )
		return hx == other.hx && hy == other.hy;
	if ( kind == Circle )
		return circle.center.x == other.circle.center.x &&
			   circle.center.y == other.circle.center.y && circle.radius == other.circle.radius;
	if ( kind == Capsule )
		return capsule.center1.x == other.capsule.center1.x &&
			   capsule.center1.y == other.capsule.center1.y &&
			   capsule.center2.x == other.capsule.center2.x &&
			   capsule.center2.y == other.capsule.center2.y &&
			   capsule.radius == other.capsule.radius;
	if ( kind == Chain )
	{
		const b2Vec2 a[] = { chain.ghost1, chain.segment.point1, chain.segment.point2,
							 chain.ghost2 };
		const auto& c = other.chain;
		const b2Vec2 b[] = { c.ghost1, c.segment.point1, c.segment.point2, c.ghost2 };
		for ( int i = 0; i < 4; ++i )
			if ( a[i].x != b[i].x || a[i].y != b[i].y )
				return false;
		return true;
	}
	if ( polygon.count != other.polygon.count || polygon.radius != other.polygon.radius )
		return false;
	for ( int i = 0; i < polygon.count; ++i )
		if ( polygon.vertices[i].x != other.polygon.vertices[i].x ||
			 polygon.vertices[i].y != other.polygon.vertices[i].y ||
			 polygon.normals[i].x != other.polygon.normals[i].x ||
			 polygon.normals[i].y != other.polygon.normals[i].y )
			return false;
	return true;
}

namespace
{
constexpr float Pi = 3.14159265358979323846f;
float segmentParameter( b2Vec2 p, b2Vec2 a, b2Vec2 b )
{
	auto d = b2Sub( b, a );
	float length = b2LengthSquared( d );
	return length > 1e-16f ? std::max( 0.f, std::min( 1.f, b2Dot( b2Sub( p, a ), d ) / length ) )
						   : 0;
}
float positiveAngle( float a )
{
	a = std::fmod( a, 2 * Pi );
	return a < 0 ? a + 2 * Pi : a;
}
} // namespace

float RigidGeometry::distance( b2Vec2 p ) const
{
	if ( kind == Circle )
		return b2Distance( p, circle.center ) - circle.radius;
	if ( kind == Capsule )
	{
		auto q = b2Lerp( capsule.center1, capsule.center2,
						 segmentParameter( p, capsule.center1, capsule.center2 ) );
		return b2Distance( p, q ) - capsule.radius;
	}
	float plane = -1e30f, distanceSquared = 1e30f;
	for ( int i = 0; i < polygon.count; ++i )
	{
		auto a = polygon.vertices[i], b = polygon.vertices[( i + 1 ) % polygon.count];
		plane = std::max( plane, b2Dot( b2Sub( p, a ), polygon.normals[i] ) );
		auto q = b2Lerp( a, b, segmentParameter( p, a, b ) );
		distanceSquared = std::min( distanceSquared, b2DistanceSquared( p, q ) );
	}
	return ( plane <= 0 ? plane : std::sqrt( distanceSquared ) ) - polygon.radius;
}

b2Vec2 BoundarySegment::point( float t ) const
{
	return radius > 0 ? b2MulAdd( center, radius,
								  { std::cos( angle + t * sweep ), std::sin( angle + t * sweep ) } )
					  : b2Lerp( a, b, t );
}
b2Vec2 BoundarySegment::normalAt( b2Vec2 p ) const
{
	return radius > 0 ? b2Normalize( b2Sub( p, center ) ) : normal;
}
bool BoundarySegment::parameter( b2Vec2 p, float& t ) const
{
	if ( radius == 0 )
	{
		auto d = b2Sub( b, a );
		t = b2Dot( b2Sub( p, a ), d ) / b2LengthSquared( d );
		return t >= -1e-6f && t <= 1 + 1e-6f;
	}
	float offset = positiveAngle( std::atan2( p.y - center.y, p.x - center.x ) - angle );
	if ( offset > 2 * Pi - 1e-6f )
		offset = 0;
	t = offset / sweep;
	return t <= 1 + 1e-6f;
}
b2Vec2 BoundarySegment::closest( b2Vec2 p ) const
{
	if ( radius == 0 )
		return b2MulAdd( a, segmentParameter( p, a, b ), b2Sub( b, a ) );
	float t;
	if ( parameter( p, t ) )
		return point( std::min( 1.f, t ) );
	return b2DistanceSquared( p, a ) < b2DistanceSquared( p, b ) ? a : b;
}
void BoundarySegment::intersections( const BoundarySegment& other,
									 std::vector<float>& parameters ) const
{
	auto add = [&]( b2Vec2 p ) {
		float t, u;
		if ( parameter( p, t ) && other.parameter( p, u ) )
			parameters.push_back( std::max( 0.f, std::min( 1.f, t ) ) );
	};
	if ( radius == 0 && other.radius == 0 )
	{
		auto d = b2Sub( b, a ), e = b2Sub( other.b, other.a );
		float cross = b2Cross( d, e );
		if ( std::abs( cross ) > 1e-10f )
			add( b2MulAdd( a, b2Cross( b2Sub( other.a, a ), e ) / cross, d ) );
		else if ( std::abs( b2Cross( b2Sub( other.a, a ), d ) ) < 1e-7f )
		{
			add( other.a );
			add( other.b );
		}
		return;
	}
	if ( radius == 0 || other.radius == 0 )
	{
		const auto& line = radius == 0 ? *this : other;
		const auto& arc = radius == 0 ? other : *this;
		auto d = b2Sub( line.b, line.a ), offset = b2Sub( line.a, arc.center );
		double aa = b2Dot( d, d ), bb = b2Dot( offset, d );
		double cc = double( offset.x ) * offset.x + double( offset.y ) * offset.y -
					double( arc.radius ) * arc.radius;
		double discriminant = bb * bb - aa * cc;
		if ( aa <= 1e-16 || discriminant < 0 )
			return;
		double root = std::sqrt( discriminant );
		add( b2MulAdd( line.a, float( ( -bb - root ) / aa ), d ) );
		add( b2MulAdd( line.a, float( ( -bb + root ) / aa ), d ) );
		return;
	}
	auto delta = b2Sub( other.center, center );
	float length = b2Length( delta );
	if ( length < 1e-7f )
	{
		if ( std::abs( radius - other.radius ) < 1e-7f )
		{
			add( other.a );
			add( other.b );
		}
		return;
	}
	if ( length > radius + other.radius || length < std::abs( radius - other.radius ) )
		return;
	float along =
		( radius * radius - other.radius * other.radius + length * length ) / ( 2 * length );
	float height = std::sqrt( std::max( 0.f, radius * radius - along * along ) );
	auto unit = b2MulSV( 1 / length, delta ), base = b2MulAdd( center, along, unit );
	add( b2MulAdd( base, height, { -unit.y, unit.x } ) );
	add( b2MulAdd( base, -height, { -unit.y, unit.x } ) );
}
float BoundarySegment::closestEdge( b2Vec2 p, b2Vec2 q, b2Vec2& surface ) const
{
	float best = 1e30f, fraction = 0;
	auto consider = [&]( float t, b2Vec2 s ) {
		float distance = b2DistanceSquared( b2Lerp( p, q, t ), s );
		if ( distance < best )
		{
			best = distance;
			fraction = t;
			surface = s;
		}
	};
	consider( 0, closest( p ) );
	consider( 1, closest( q ) );
	consider( segmentParameter( a, p, q ), a );
	consider( segmentParameter( b, p, q ), b );
	if ( radius > 0 )
	{
		float t = segmentParameter( center, p, q );
		consider( t, closest( b2Lerp( p, q, t ) ) );
	}
	return fraction;
}

void buildCurvedBoundary( const std::vector<BoundaryShape>& shapes,
						  std::vector<BoundarySegment>& boundary )
{
	std::vector<BoundarySegment> curves;
	for ( const auto& source : shapes )
	{
		int feature = 0;
		auto line = [&]( b2Vec2 a, b2Vec2 b, b2Vec2 normal ) {
			BoundarySegment c;
			c.a = a;
			c.b = b;
			c.normal = normal;
			c.shape = source.shape;
			c.feature = feature++;
			curves.push_back( c );
		};
		auto arc = [&]( b2Vec2 center, float radius, b2Vec2 start, float sweep ) {
			BoundarySegment c;
			c.center = center;
			c.radius = radius;
			c.angle = std::atan2( start.y, start.x );
			c.sweep = sweep;
			c.a = c.point( 0 );
			c.b = c.point( 1 );
			c.normal = start;
			c.shape = source.shape;
			c.feature = feature++;
			c.cornerA = c.cornerB = false;
			curves.push_back( c );
		};
		const auto& g = source.geometry;
		if ( g.kind == RigidGeometry::Circle )
			arc( g.circle.center, g.circle.radius, { 1, 0 }, 2 * Pi );
		else if ( g.kind == RigidGeometry::Capsule )
		{
			auto a = g.capsule.center1, b = g.capsule.center2;
			auto u = b2Normalize( b2Sub( b, a ) );
			b2Vec2 n{ u.y, -u.x };
			float r = g.capsule.radius;
			line( b2MulAdd( a, r, n ), b2MulAdd( b, r, n ), n );
			arc( b, r, n, Pi );
			line( b2MulAdd( b, -r, n ), b2MulAdd( a, -r, n ), b2Neg( n ) );
			arc( a, r, b2Neg( n ), Pi );
		}
		else
		{
			const auto& p = g.polygon;
			for ( int i = 0; i < p.count; ++i )
			{
				int next = ( i + 1 ) % p.count;
				line( b2MulAdd( p.vertices[i], p.radius, p.normals[i] ),
					  b2MulAdd( p.vertices[next], p.radius, p.normals[i] ), p.normals[i] );
				if ( p.radius > 0 )
				{
					float sweep = std::atan2( b2Cross( p.normals[i], p.normals[next] ),
											  b2Dot( p.normals[i], p.normals[next] ) );
					arc( p.vertices[next], p.radius, p.normals[i], sweep );
				}
			}
		}
	}
	boundary.clear();
	std::vector<float> cuts;
	for ( const auto& source : curves )
	{
		cuts.clear();
		cuts.push_back( 0 );
		cuts.push_back( 1 );
		for ( const auto& other : curves )
			if ( source.shape != other.shape )
				source.intersections( other, cuts );
		std::sort( cuts.begin(), cuts.end() );
		int piece = 0;
		for ( size_t i = 1; i < cuts.size(); ++i )
		{
			float lo = cuts[i - 1], hi = cuts[i];
			if ( hi - lo < 1e-6f )
				continue;
			auto p = source.point( ( lo + hi ) * .5f );
			auto n = source.normalAt( p );
			bool hidden = false;
			for ( const auto& other : shapes )
				if ( source.shape != other.shape )
				{
					float distance = other.geometry.distance( p );
					// A coincident exterior has one deterministic owner; an
					// adjacent interior seam is hidden on both source fixtures.
					if ( distance < -1e-6f ||
						 ( distance < 1e-6f &&
						   ( other.shape < source.shape ||
							 other.geometry.distance( b2MulAdd( p, 4e-6f, n ) ) < 0 ) ) )
					{
						hidden = true;
						break;
					}
				}
			if ( hidden )
				continue;
			auto c = source;
			c.a = source.point( lo );
			c.b = source.point( hi );
			c.angle += lo * c.sweep;
			c.sweep *= hi - lo;
			c.feature = source.feature * 1024 + piece++;
			boundary.push_back( c );
		}
	}
}

void buildCompoundBoundary( const std::vector<BoundaryPolygon>& polygons,
							std::vector<BoundarySegment>& boundary )
{
	boundary.clear();
	// Split each source edge at occluder half-plane intersections. Clipping a
	// tiny outward offset removes shared internal edges, not touching exteriors.
	constexpr float epsilon = 1e-6f;
	for ( const auto& source : polygons )
		for ( int face = 0; face < source.polygon.count; ++face )
		{
			auto a = source.polygon.vertices[face];
			auto b = source.polygon.vertices[( face + 1 ) % source.polygon.count];
			auto n = source.polygon.normals[face], d = b2Sub( b, a );
			std::vector<std::pair<float, float>> visible{ { 0, 1 } };
			for ( const auto& other : polygons )
			{
				if ( other.shape == source.shape )
					continue;
				float lo = 0, hi = 1;
				for ( int j = 0; j < other.polygon.count; ++j )
				{
					auto normal = other.polygon.normals[j];
					float start = b2Dot( b2Sub( a, other.polygon.vertices[j] ), normal );
					float slope = b2Dot( d, normal );
					// Coincident exterior faces have one deterministic source fixture.
					bool samePlane = std::abs( start ) < epsilon && b2Dot( n, normal ) > .99999f;
					float offset = samePlane && other.shape < source.shape
									   ? -epsilon
									   : 4 * epsilon * b2Dot( n, normal );
					start += offset;
					if ( std::abs( slope ) < epsilon )
					{
						if ( start > 0 )
						{
							hi = -1;
							break;
						}
					}
					else if ( slope > 0 )
						hi = std::min( hi, -start / slope );
					else
						lo = std::max( lo, -start / slope );
				}
				if ( hi <= lo )
					continue;
				std::vector<std::pair<float, float>> next;
				for ( auto interval : visible )
				{
					if ( hi <= interval.first || lo >= interval.second )
						next.push_back( interval );
					else
					{
						if ( lo > interval.first )
							next.push_back( { interval.first, lo } );
						if ( hi < interval.second )
							next.push_back( { hi, interval.second } );
					}
				}
				visible.swap( next );
			}
			int piece = 0;
			for ( auto interval : visible )
			{
				if ( ( interval.second - interval.first ) * b2Length( d ) < 8 * epsilon )
					continue;
				BoundarySegment segment;
				segment.a = b2MulAdd( a, interval.first, d );
				segment.b = b2MulAdd( a, interval.second, d );
				segment.normal = n;
				segment.shape = source.shape;
				segment.feature = face * 64 + piece++;
				boundary.push_back( segment );
			}
		}
	// A seam endpoint in a straight exposed surface is not a physical corner.
	for ( auto& segment : boundary )
		for ( const auto& other : boundary )
		{
			if ( &segment == &other || b2Dot( segment.normal, other.normal ) < .99999f )
				continue;
			if ( b2LengthSquared( b2Sub( segment.a, other.b ) ) < 1e-9f )
				segment.cornerA = false;
			if ( b2LengthSquared( b2Sub( segment.b, other.a ) ) < 1e-9f )
				segment.cornerB = false;
		}
}

int BoxProjection::faceCount() const
{
	if ( circular() )
		return 1;
	if ( geometry && geometry->kind == RigidGeometry::Polygon )
		return geometry->polygon.count;
	return 4;
}
b2Vec2 BoxProjection::faceNormal( int face ) const
{
	return geometry && geometry->kind == RigidGeometry::Polygon ? geometry->polygon.normals[face]
																: normal( face );
}
b2Vec2 BoxProjection::vertex( int corner ) const
{
	return geometry && geometry->kind == RigidGeometry::Polygon
			   ? geometry->polygon.vertices[corner]
			   : b2Vec2{ corner & 1 ? hx : -hx, corner & 2 ? hy : -hy };
}
b2Vec2 BoxProjection::radialNormal( b2Vec2 p ) const
{
	auto offset = b2Sub( p, geometry->circle.center );
	float length = b2Length( offset );
	return length > 1e-8f ? b2MulSV( 1 / length, offset ) : b2Vec2{ 1, 0 };
}
bool BoxProjection::withinFace( b2Vec2 p, int face ) const
{
	if ( circular() )
		return true;
	if ( geometry && geometry->kind == RigidGeometry::Polygon )
	{
		auto a = vertex( face ), d = b2Sub( vertex( ( face + 1 ) % faceCount() ), a );
		float length = b2Length( d ), along = b2Dot( b2Sub( p, a ), d ) / length;
		return along >= -faceMargin && along <= length + faceMargin;
	}
	return std::abs( face < 2 ? p.y : p.x ) <= ( face < 2 ? hy : hx ) + faceMargin;
}

b2Vec2 BoxProjection::normal( int face )
{
	const b2Vec2 normals[4] = { { -1, 0 }, { 1, 0 }, { 0, -1 }, { 0, 1 } };
	return normals[face];
}

float BoxProjection::separation( b2Vec2 p, int face ) const
{
	if ( circular() )
		return b2Length( b2Sub( p, geometry->circle.center ) ) - geometry->circle.radius;
	if ( geometry && geometry->kind == RigidGeometry::Polygon )
		return b2Dot( b2Sub( p, vertex( face ) ), faceNormal( face ) );
	return b2Dot( p, normal( face ) ) - ( face < 2 ? hx : hy );
}

b2Vec2 BoxProjection::local( int node ) const
{
	return b2InvTransformWorldPoint( transform, { body.x[node][0], body.x[node][1] } );
}

int BoxProjection::retainedFace( int node ) const
{
	if ( circular() || !nodeFaces || ( *nodeFaces )[node] < 0 )
		return -1;
	int face = ( *nodeFaces )[node];
	return face < faceCount() && withinFace( local( node ), face ) ? face : -1;
}

bool BoxProjection::clip( b2Vec2 a, b2Vec2 b, int& entry, int& exit ) const
{
	float enter = 0, leave = 1;
	entry = exit = -1;
	for ( int face = 0; face < faceCount(); ++face )
	{
		float start = separation( a, face ), finish = separation( b, face );
		if ( start >= 0 && finish >= 0 )
			return false;
		if ( start > 0 && start / ( start - finish ) > enter )
		{
			enter = start / ( start - finish );
			entry = face;
		}
		if ( finish > 0 && start / ( start - finish ) < leave )
		{
			leave = start / ( start - finish );
			exit = face;
		}
	}
	return leave > enter;
}

void BoxProjection::move( int edge, float fraction, b2Vec2 direction, float depth )
{
	if ( !( depth > 0 ) )
		return;
	++corrections;
	int next = ( edge + 1 ) % body.nodeCount;
	float inverse = ( 1 - fraction ) * ( 1 - fraction ) / body.mass[edge] +
					fraction * fraction / body.mass[next];
	float factor = 1;
	if ( remaining )
		for ( int side = 0; side < 2; ++side )
		{
			int node = side ? next : edge;
			float weight = side ? fraction : 1 - fraction;
			float distance = weight * ( depth + 1e-6f ) / ( inverse * body.mass[node] );
			if ( distance > 0 )
				factor = std::min( factor, ( *remaining )[node] / distance );
		}
	for ( int side = 0; side < 2; ++side )
	{
		int node = side ? next : edge;
		float weight = side ? fraction : 1 - fraction;
		// C1 leaves a 1e-6 m numerical gap to avoid alternating sides of
		// a float-rounded plane. This is 0.00016 content units, not skin.
		float distance = weight * ( depth + 1e-6f ) / ( inverse * body.mass[node] );
		if ( remaining )
		{
			distance *= factor;
			( *remaining )[node] = std::max( 0.f, ( *remaining )[node] - distance );
		}
		if ( correctionDistance )
			*correctionDistance += distance;
		body.x[node][0] += distance * direction.x;
		body.x[node][1] += distance * direction.y;
	}
	if ( stopOnInvalid && !inspectTopology( body ).valid() )
		invalidated = true;
}

void BoxProjection::project()
{
	if ( circular() )
	{
		projectCircle();
		return;
	}
	// For rectangles only, retain faces separating every original node. The
	// existing node/retained-side projection still runs first. Any correction
	// invalidates this proof, including a correction limited by its budget.
	unsigned outside = !geometry || geometry->kind == RigidGeometry::Rectangle ? 15u : 0u;
	unsigned initialCorrections = corrections;
	float scale = 1 + float( std::abs( transform.p.x ) + std::abs( transform.p.y ) ) + hx + hy;
	for ( int node = 0; node < body.nodeCount; ++node )
	{
		auto p = local( node );
		unsigned nodeOutside = 0;
		float margin = 32 * std::numeric_limits<float>::epsilon() *
					   ( scale + std::abs( p.x ) + std::abs( p.y ) );
		float closest = -1e30f;
		int face = 0;
		for ( int i = 0; i < faceCount(); ++i )
		{
			float distance = separation( p, i );
			if ( outside && distance > margin )
				nodeOutside |= 1u << i;
			if ( distance > closest )
			{
				closest = distance;
				face = i;
			}
		}
		outside &= nodeOutside;
		int retained = retainedFace( node );
		if ( retained >= 0 )
		{
			face = retained;
			closest = separation( p, face );
		}
		if ( closest < 0 )
			move( node, 0, b2RotateVector( transform.q, faceNormal( face ) ), -closest );
		if ( invalidated )
			return;
	}
	if ( pointsOnly )
		return;
	// A convex half-plane contains the whole contour (including concave or
	// folded contours) when it contains all nodes. No corner/edge can cross
	// this rectangle. Keep near-plane cases on the original path to cover
	// float roundoff in inverse/world transforms; this is not a contact skin.
	if ( outside && corrections == initialCorrections )
		return;
	for ( int corner = 0; corner < faceCount(); ++corner )
	{
		auto vertex = b2TransformWorldPoint( transform, this->vertex( corner ) );
		Core::Vec p = { float( vertex.x ), float( vertex.y ) };
		if ( !SoftBody::inside<float>( p, body ) )
			continue;
		auto hit = SoftBody::nearest<float>( p, body );
		if ( hit.edge < 0 )
		{
			error = "degenerate_contact_edge";
			return;
		}
		if ( hit.depth > 1e-8f )
			move( hit.edge, hit.t, { -hit.normal[0], -hit.normal[1] }, hit.depth );
		if ( invalidated )
			return;
	}
	int face = 0;
	float best = -1e30f;
	for ( int i = 0; i < faceCount(); ++i )
	{
		float minimum = 1e30f;
		for ( int node = 0; node < body.nodeCount; ++node )
			minimum = std::min( minimum, separation( local( node ), i ) );
		if ( minimum > best )
		{
			best = minimum;
			face = i;
		}
	}
	auto& positions = body.positionScratch;
	for ( int i = 0; i < body.nodeCount; ++i )
		positions[i] = { local( i ).x, local( i ).y };
	for ( int edge = 0; edge < body.nodeCount; ++edge )
	{
		int entry, exit;
		if ( !clip( { positions[edge][0], positions[edge][1] },
					{ positions[( edge + 1 ) % body.nodeCount][0],
					  positions[( edge + 1 ) % body.nodeCount][1] },
					entry, exit ) )
			continue;
		auto a = local( edge ), b = local( ( edge + 1 ) % body.nodeCount );
		bool crossed = clip( a, b, entry, exit );
		bool rectangle = !geometry || geometry->kind == RigidGeometry::Rectangle;
		int common = -1;
		if ( entry >= 0 && exit >= 0 && !rectangle )
		{
			if ( ( entry + 1 ) % faceCount() == exit )
				common = exit;
			if ( ( exit + 1 ) % faceCount() == entry )
				common = entry;
		}
		if ( crossed && entry >= 0 && exit >= 0 &&
			 ( rectangle ? entry / 2 != exit / 2 : common >= 0 ) )
		{
			int x = entry < 2 ? entry : exit, y = entry >= 2 ? entry : exit;
			b2Vec2 vertex = rectangle ? b2Vec2{ normal( x ).x * hx, normal( y ).y * hy }
									  : this->vertex( common );
			auto world = b2TransformWorldPoint( transform, vertex );
			if ( SoftBody::inside<float>( { float( world.x ), float( world.y ) }, body ) )
			{
				auto delta = b2Sub( b, a );
				float length2 = b2LengthSquared( delta );
				if ( length2 < 1e-12f )
				{
					error = "degenerate_contact_edge";
					return;
				}
				float fraction =
					std::max( 0.f, std::min( 1.f, b2Dot( b2Sub( vertex, a ), delta ) / length2 ) );
				auto offset = b2Sub( vertex, b2MulAdd( a, fraction, delta ) );
				float depth = b2Length( offset );
				if ( depth > 1e-8f )
					move( edge, fraction,
						  b2RotateVector( transform.q, b2MulSV( 1 / depth, offset ) ), depth );
				if ( invalidated )
					return;
				continue;
			}
		}
		for ( int node : { edge, ( edge + 1 ) % body.nodeCount } )
		{
			float distance = separation( local( node ), face );
			if ( distance < 0 )
				move( node, 0, b2RotateVector( transform.q, faceNormal( face ) ), -distance );
			if ( invalidated )
				return;
		}
	}
}

void BoxProjection::projectCircle()
{
	for ( int node = 0; node < body.nodeCount; ++node )
	{
		auto p = local( node );
		float depth = -separation( p, 0 );
		if ( depth > 0 )
			move( node, 0, b2RotateVector( transform.q, radialNormal( p ) ), depth );
		if ( invalidated )
			return;
	}
	if ( pointsOnly )
		return;
	auto center = b2TransformWorldPoint( transform, geometry->circle.center );
	Core::Vec p = { float( center.x ), float( center.y ) };
	if ( SoftBody::inside<float>( p, body ) )
	{
		auto hit = SoftBody::nearest<float>( p, body );
		if ( hit.edge < 0 )
		{
			error = "degenerate_contact_edge";
			return;
		}
		move( hit.edge, hit.t, { -hit.normal[0], -hit.normal[1] },
			  geometry->circle.radius + hit.depth );
		if ( invalidated )
			return;
	}
	// A circle can intersect the middle of a long edge with both nodes outside.
	for ( int edge = 0; edge < body.nodeCount; ++edge )
	{
		auto a = local( edge ), d = b2Sub( local( ( edge + 1 ) % body.nodeCount ), a );
		float length2 = b2LengthSquared( d );
		if ( length2 < 1e-12f )
			continue;
		float t = std::max(
			0.f, std::min( 1.f, b2Dot( b2Sub( geometry->circle.center, a ), d ) / length2 ) );
		if ( t <= 0 || t >= 1 )
			continue;
		auto point = b2MulAdd( a, t, d );
		float depth = -separation( point, 0 );
		if ( depth > 0 )
		{
			auto n = radialNormal( point );
			if ( b2LengthSquared( b2Sub( point, geometry->circle.center ) ) < 1e-16f )
				n = b2MulSV( 1 / std::sqrt( length2 ), b2Vec2{ -d.y, d.x } );
			move( edge, t, b2RotateVector( transform.q, n ), depth );
		}
		if ( invalidated )
			return;
	}
}

double BoxProjection::boundaryDepth() const
{
	if ( circular() )
	{
		double depth = 0;
		for ( int i = 0; i < body.nodeCount; ++i )
		{
			auto a = local( i ), d = b2Sub( local( ( i + 1 ) % body.nodeCount ), a );
			float length2 = b2LengthSquared( d );
			float t =
				length2 > 1e-12f
					? std::max( 0.f,
								std::min( 1.f, b2Dot( b2Sub( geometry->circle.center, a ), d ) /
												   length2 ) )
					: 0;
			depth = std::max( depth, double( -separation( b2MulAdd( a, t, d ), 0 ) ) );
		}
		// This is boundary overlap only; containment is reported separately by tests.
		return depth;
	}
	double result = 0;
	for ( int edge = 0; edge < body.nodeCount; ++edge )
	{
		auto a = local( edge ), b = local( ( edge + 1 ) % body.nodeCount );
		float start[B2_MAX_POLYGON_VERTICES], slope[B2_MAX_POLYGON_VERTICES];
		for ( int face = 0; face < faceCount(); ++face )
		{
			start[face] = -separation( a, face );
			slope[face] = -separation( b, face ) - start[face];
		}
		auto depth = [&]( float t ) {
			float value = 1e30f;
			for ( int face = 0; face < faceCount(); ++face )
				value = std::min( value, start[face] + t * slope[face] );
			return double( value );
		};
		result = std::max( result, std::max( depth( 0 ), depth( 1 ) ) );
		for ( int i = 0; i < faceCount(); ++i )
			for ( int j = i + 1; j < faceCount(); ++j )
			{
				float denominator = slope[i] - slope[j];
				if ( std::abs( denominator ) < 1e-12f )
					continue;
				float t = ( start[j] - start[i] ) / denominator;
				if ( t > 0 && t < 1 )
					result = std::max( result, depth( t ) );
			}
	}
	return result;
}
} // namespace Game
} // namespace SoftBody
