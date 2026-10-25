#include "soft_game_geometry.h"

#if defined( __clang__ )
#pragma clang fp contract( off )
#endif

namespace SoftBody
{
namespace Game
{

b2Vec2 BoxProjection::normal( int face )
{
	const b2Vec2 normals[4] = { { -1, 0 }, { 1, 0 }, { 0, -1 }, { 0, 1 } };
	return normals[face];
}

float BoxProjection::separation( b2Vec2 p, int face ) const
{
	return b2Dot( p, normal( face ) ) - ( face < 2 ? hx : hy );
}

b2Vec2 BoxProjection::local( int node ) const
{
	return b2InvTransformWorldPoint( transform, { body.x[node][0], body.x[node][1] } );
}

int BoxProjection::retainedFace( int node ) const
{
	if ( !nodeFaces || ( *nodeFaces )[node] < 0 )
		return -1;
	int face = ( *nodeFaces )[node];
	auto p = local( node );
	float tangent = face < 2 ? p.y : p.x;
	float extent = face < 2 ? hy : hx;
	// Keep the approached side across a contact skin at adjacent box seams.
	// Once the point has gone around the finite end, release this half-plane.
	return std::abs( tangent ) <= extent + faceMargin ? face : -1;
}

bool BoxProjection::clip( b2Vec2 a, b2Vec2 b, int& entry, int& exit ) const
{
	float enter = 0, leave = 1;
	entry = exit = -1;
	for ( int face = 0; face < 4; ++face )
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
	int next = ( edge + 1 ) % 16;
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
	for ( int node = 0; node < 16; ++node )
	{
		auto p = local( node );
		float closest = -1e30f;
		int face = 0;
		for ( int i = 0; i < 4; ++i )
		{
			float distance = separation( p, i );
			if ( distance > closest )
			{
				closest = distance;
				face = i;
			}
		}
		int retained = retainedFace( node );
		if ( retained >= 0 )
		{
			face = retained;
			closest = separation( p, face );
		}
		if ( closest < 0 )
			move( node, 0, b2RotateVector( transform.q, normal( face ) ), -closest );
		if ( invalidated )
			return;
	}
	if ( pointsOnly )
		return;
	for ( int corner = 0; corner < 4; ++corner )
	{
		auto vertex =
			b2TransformWorldPoint( transform, { corner & 1 ? hx : -hx, corner & 2 ? hy : -hy } );
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
	for ( int i = 0; i < 4; ++i )
	{
		float minimum = 1e30f;
		for ( int node = 0; node < 16; ++node )
			minimum = std::min( minimum, separation( local( node ), i ) );
		if ( minimum > best )
		{
			best = minimum;
			face = i;
		}
	}
	b2Vec2 positions[16];
	for ( int i = 0; i < 16; ++i )
		positions[i] = local( i );
	for ( int edge = 0; edge < 16; ++edge )
	{
		int entry, exit;
		if ( !clip( positions[edge], positions[( edge + 1 ) % 16], entry, exit ) )
			continue;
		auto a = local( edge ), b = local( ( edge + 1 ) % 16 );
		if ( clip( a, b, entry, exit ) && entry >= 0 && exit >= 0 && entry / 2 != exit / 2 )
		{
			int x = entry < 2 ? entry : exit, y = entry >= 2 ? entry : exit;
			b2Vec2 vertex = { normal( x ).x * hx, normal( y ).y * hy };
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
		for ( int node : { edge, ( edge + 1 ) % 16 } )
		{
			float distance = separation( local( node ), face );
			if ( distance < 0 )
				move( node, 0, b2RotateVector( transform.q, normal( face ) ), -distance );
			if ( invalidated )
				return;
		}
	}
}

double BoxProjection::boundaryDepth() const
{
	double result = 0;
	for ( int edge = 0; edge < 16; ++edge )
	{
		auto a = local( edge ), b = local( ( edge + 1 ) % 16 );
		float start[4], slope[4];
		for ( int face = 0; face < 4; ++face )
		{
			start[face] = -separation( a, face );
			slope[face] = -separation( b, face ) - start[face];
		}
		auto depth = [&]( float t ) {
			float value = 1e30f;
			for ( int face = 0; face < 4; ++face )
				value = std::min( value, start[face] + t * slope[face] );
			return double( value );
		};
		result = std::max( result, std::max( depth( 0 ), depth( 1 ) ) );
		for ( int i = 0; i < 4; ++i )
			for ( int j = i + 1; j < 4; ++j )
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
