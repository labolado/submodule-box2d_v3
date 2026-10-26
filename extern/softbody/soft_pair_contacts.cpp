#include "soft_pair_contacts.h"

#include "soft_game_recovery.h"

#include <limits>

namespace SoftBody
{
#if defined( __clang__ )
#pragma clang fp contract( off )
#endif
namespace
{
using Vec = PairContacts::Vec;
constexpr float Margin = .002f;
constexpr float Correction = .02f; // Per node, shared across pairs and passes in a complete step.
float dot( Vec a, Vec b )
{
	return a[0] * b[0] + a[1] * b[1];
}
Vec scale( Vec a, float s )
{
	return { a[0] * s, a[1] * s };
}
Vec add( Vec a, Vec b )
{
	return { a[0] + b[0], a[1] + b[1] };
}
Vec difference( Vec a, Vec b )
{
	return { a[0] - b[0], a[1] - b[1] };
}
float mass( const Body<float>& body )
{
	float sum = 0;
	for ( float m : body.mass )
		sum += m;
	return sum;
}
Vec centre( const Body<float>& body )
{
	Vec p{};
	for ( int i = 0; i < 16; ++i )
		p = add( p, scale( body.x[i], body.mass[i] ) );
	return scale( p, 1 / mass( body ) );
}
Vec point( const Body<float>& body, int edge, float t )
{
	return add( scale( body.x[edge], 1 - t ), scale( body.x[( edge + 1 ) % 16], t ) );
}
} // namespace

PairContacts::PairContacts( Core& world )
	: core( world )
{
}
void PairContacts::beginStep()
{
	rows.reserve( MaxRows );
	previous.reserve( MaxRows );
	remaining.reserve( MaxBodies );
	states.resize( MaxBodies * MaxBodies );
	remaining.resize( core.bodies.size() );
	for ( auto& budget : remaining )
		budget.fill( Correction );
	recoveryUsed = false;
}
bool PairContacts::broadPhase( const Body<float>& a, const Body<float>& b ) const
{
	for ( int axis = 0; axis < 2; ++axis )
	{
		float al = a.x[0][axis], ah = al, bl = b.x[0][axis], bh = bl;
		for ( int i = 1; i < 16; ++i )
		{
			al = std::min( al, a.x[i][axis] );
			ah = std::max( ah, a.x[i][axis] );
			bl = std::min( bl, b.x[i][axis] );
			bh = std::max( bh, b.x[i][axis] );
		}
		if ( ah + Margin < bl || bh + Margin < al )
			return false;
	}
	return true;
}
bool PairContacts::append( Row row )
{
	if ( rows.size() == MaxRows )
	{
		error = "soft_pair_contact_capacity";
		return false;
	}
	row.generationA = core.bodies[row.a].generation;
	row.generationB = core.bodies[row.b].generation;
	rows.push_back( row );
	return true;
}
void PairContacts::sample( int a, int b, int edge, float fraction )
{
	auto p = point( core.bodies[a], edge, fraction );
	const auto& target = core.bodies[b];
	float best = std::numeric_limits<float>::max();
	Row row;
	row.a = a;
	row.b = b;
	row.edgeA = edge;
	row.tA = fraction;
	bool interior = inside<float>( p, target );
	for ( int e = 0; e < 16; ++e )
	{
		auto d = difference( target.x[( e + 1 ) % 16], target.x[e] );
		float length2 = dot( d, d );
		if ( length2 < 1e-12f )
			continue;
		float t =
			std::max( 0.f, std::min( 1.f, dot( difference( p, target.x[e] ), d ) / length2 ) );
		auto delta = difference( p, point( target, e, t ) );
		float distance = std::sqrt( dot( delta, delta ) );
		if ( distance >= best )
			continue;
		best = distance;
		row.edgeB = e;
		row.tB = t;
		row.normal = distance > 1e-7f
						 ? scale( delta, ( interior ? -1.f : 1.f ) / distance )
						 : Vec{ d[1] / std::sqrt( length2 ), -d[0] / std::sqrt( length2 ) };
	}
	if ( best == std::numeric_limits<float>::max() )
		return;
	if ( !interior && best > Margin )
		return;
	row.gap = interior ? -best : best;
	append( row );
}
void PairContacts::direction( int a, int b )
{
	const auto& first = core.bodies[a];
	const auto& second = core.bodies[b];
	for ( int e = 0; e < 16 && !error; ++e )
	{
		sample( a, b, e, 0 );
		// Interior portions of an edge can cross the other contour even when
		// neither endpoint is inside. Sample intervals between intersections.
		float cuts[18] = { 0, 1 };
		int count = 2;
		auto p = first.x[e], d = difference( first.x[( e + 1 ) % 16], p );
		for ( int j = 0; j < 16; ++j )
		{
			auto q = second.x[j], v = difference( second.x[( j + 1 ) % 16], q );
			float denominator = cross<float>( d, v );
			if ( std::abs( denominator ) < 1e-12f )
				continue;
			float t = cross<float>( difference( q, p ), v ) / denominator;
			float u = cross<float>( difference( q, p ), d ) / denominator;
			if ( t > 1e-5f && t < 1 - 1e-5f && u >= 0 && u <= 1 )
				cuts[count++] = t;
		}
		if ( count == 2 )
			continue;
		std::sort( cuts, cuts + count );
		for ( int i = 1; i < count; ++i )
		{
			if ( cuts[i] - cuts[i - 1] < 1e-5f )
				continue;
			float t = .5f * ( cuts[i] + cuts[i - 1] );
			if ( inside<float>( point( first, e, t ), second ) )
				sample( a, b, e, t );
		}
	}
}
float PairContacts::separation( int a, int b, Vec normal ) const
{
	const auto& first = core.bodies[a];
	const auto& second = core.bodies[b];
	float amin = dot( first.x[0], normal ), bmax = dot( second.x[0], normal );
	for ( int i = 1; i < 16; ++i )
	{
		amin = std::min( amin, dot( first.x[i], normal ) );
		bmax = std::max( bmax, dot( second.x[i], normal ) );
	}
	return amin - bmax;
}
void PairContacts::fallback( int a, int b, Vec normal )
{
	recoveryUsed = true;
	Row row;
	row.a = a;
	row.b = b;
	row.centres = true;
	row.normal = normal;
	row.gap = separation( a, b, normal );
	if ( row.gap > Margin )
		return;
	append( row );
}
bool PairContacts::collect()
{
	rows.clear();
	if ( core.bodies.size() > MaxBodies )
	{
		error = "soft_pair_body_capacity";
		return false;
	}
	for ( int a = 0; a < int( core.bodies.size() ); ++a )
		for ( int b = a + 1; b < int( core.bodies.size() ) && !error; ++b )
		{
			auto& state = states[a * MaxBodies + b];
			const auto& bodyA = core.bodies[a];
			const auto& bodyB = core.bodies[b];
			if ( !core.pairEnabled( a, b ) )
			{
				state = PairState{};
				continue;
			}
			auto delta = difference( centre( bodyA ), centre( bodyB ) );
			float length = std::sqrt( dot( delta, delta ) );
			Vec direction = length > 1e-5f ? scale( delta, 1 / length ) : Vec{ 1, 0 };
			if ( state.generationA != bodyA.generation || state.generationB != bodyB.generation )
			{
				state = PairState{};
				state.generationA = bodyA.generation;
				state.generationB = bodyB.generation;
				state.normal = direction;
			}
			if ( !state.active && separation( a, b, direction ) >= 0 )
				state.normal = direction;
			if ( !broadPhase( bodyA, bodyB ) )
				continue;
			auto first = Game::inspectTopology( core.bodies[a] );
			auto second = Game::inspectTopology( core.bodies[b] );
			if ( !first.finite || !second.finite )
			{
				error = "nonfinite_soft_pair";
				break;
			}
			bool deep = !first.valid() || !second.valid() ||
						first.signedArea < .05 * core.bodies[a].restArea ||
						second.signedArea < .05 * core.bodies[b].restArea ||
						inside<float>( centre( bodyA ), bodyB ) ||
						inside<float>( centre( bodyB ), bodyA );
			if ( deep && !state.active )
			{
				state.active = true;
				state.separatedSteps = 0;
				++recoveryEntries;
			}
			if ( state.active )
				fallback( a, b, state.normal );
			else
			{
				this->direction( a, b );
				this->direction( b, a );
			}
		}
	return !error;
}
float PairContacts::inverse( const Row& r ) const
{
	const auto& a = core.bodies[r.a];
	const auto& b = core.bodies[r.b];
	if ( r.centres )
		return 1 / mass( a ) + 1 / mass( b );
	return ( 1 - r.tA ) * ( 1 - r.tA ) / a.mass[r.edgeA] +
		   r.tA * r.tA / a.mass[( r.edgeA + 1 ) % 16] +
		   ( 1 - r.tB ) * ( 1 - r.tB ) / b.mass[r.edgeB] +
		   r.tB * r.tB / b.mass[( r.edgeB + 1 ) % 16];
}
PairContacts::Vec PairContacts::velocity( int id, int edge, float t, bool whole ) const
{
	const auto& b = core.bodies[id];
	if ( !whole )
		return add( scale( b.v[edge], 1 - t ), scale( b.v[( edge + 1 ) % 16], t ) );
	Vec v{};
	for ( int i = 0; i < 16; ++i )
		v = add( v, scale( b.v[i], b.mass[i] ) );
	return scale( v, 1 / mass( b ) );
}
void PairContacts::apply( const Row& r, Vec impulse, bool position )
{
	float factor = 1;
	// Use one scale for the whole correction so its mass-weighted sum stays zero.
	if ( position )
		for ( int side = 0; side < 2; ++side )
		{
			int id = side ? r.b : r.a, edge = side ? r.edgeB : r.edgeA;
			float t = side ? r.tB : r.tA;
			const auto& body = core.bodies[id];
			for ( int j = 0; j < ( r.centres ? 16 : 2 ); ++j )
			{
				int node = r.centres ? j : ( edge + j ) % 16;
				float w = r.centres ? 1 / mass( body ) : ( j ? t : 1 - t ) / body.mass[node];
				float distance = w * std::sqrt( dot( impulse, impulse ) );
				if ( distance > 0 )
					factor = std::min( factor, remaining[id][node] / distance );
				if ( positionLimit )
					factor = std::min(
						factor, positionLimit( positionContext, id, node,
											   scale( impulse, ( side ? -1.f : 1.f ) * w ) ) );
			}
		}
	for ( int side = 0; side < 2; ++side )
	{
		int id = side ? r.b : r.a, edge = side ? r.edgeB : r.edgeA;
		float t = side ? r.tB : r.tA;
		auto& body = core.bodies[id];
		for ( int j = 0; j < ( r.centres ? 16 : 2 ); ++j )
		{
			int node = r.centres ? j : ( edge + j ) % 16;
			float w = r.centres ? 1 / mass( body ) : ( j ? t : 1 - t ) / body.mass[node];
			auto delta = scale( impulse, ( side ? -1.f : 1.f ) * factor * w );
			if ( position )
			{
				body.x[node] = add( body.x[node], delta );
				remaining[id][node] =
					std::max( 0.f, remaining[id][node] - std::sqrt( dot( delta, delta ) ) );
			}
			else
				body.v[node] = add( body.v[node], delta );
		}
	}
}
bool PairContacts::project()
{
	if ( !collect() )
		return false;
	for ( const auto& row : rows )
	{
		if ( row.centres )
		{
			projectRecovery( row );
			continue;
		}
		const auto& a = core.bodies[row.a];
		const auto& b = core.bodies[row.b];
		// Earlier rows move shared nodes. Reusing their original depths applies
		// the same overlap correction several times and stores artificial strain.
		float gap = dot( difference( point( a, row.edgeA, row.tA ), point( b, row.edgeB, row.tB ) ),
						 row.normal );
		if ( gap < -.0001f )
			apply( row, scale( row.normal, -( gap + .0001f ) / inverse( row ) ), true );
	}
	return true;
}

void PairContacts::projectRecovery( const Row& row )
{
	float need = std::max( 0.f, Margin - separation( row.a, row.b, row.normal ) );
	Vec motion[2]{};
	float progress[2]{};
	for ( int side = 0; side < 2; ++side )
	{
		int id = side ? row.b : row.a;
		// World support permits local deformation, but recovery must not move
		// its whole reference frame. The other side receives the spare budget.
		if ( core.bodies[id].worldSupported )
			continue;
		float budget = *std::min_element( remaining[id].begin(), remaining[id].end() );
		Vec direction = scale( row.normal, side ? -1.f : 1.f );
		motion[side] = scale( direction, budget );
		if ( projectTranslation )
			motion[side] = projectTranslation( positionContext, id, motion[side] );
		float length = std::sqrt( dot( motion[side], motion[side] ) );
		if ( length > budget )
			motion[side] = scale( motion[side], budget / length );
		progress[side] = std::max( 0.f, dot( direction, motion[side] ) );
	}
	float inverseA = 1 / mass( core.bodies[row.a] ), inverseB = 1 / mass( core.bodies[row.b] );
	float shareA = std::min( progress[0], need * inverseA / ( inverseA + inverseB ) );
	float shareB = std::min( progress[1], need - shareA );
	shareA += std::min( progress[0] - shareA, need - shareA - shareB );
	const float shares[2] = { shareA, shareB };
	for ( int side = 0; side < 2; ++side )
	{
		if ( progress[side] <= 0 )
			continue;
		int id = side ? row.b : row.a;
		auto delta = scale( motion[side], shares[side] / progress[side] );
		float distance = std::sqrt( dot( delta, delta ) );
		for ( int node = 0; node < 16; ++node )
		{
			core.bodies[id].x[node] = add( core.bodies[id].x[node], delta );
			remaining[id][node] = std::max( 0.f, remaining[id][node] - distance );
		}
		recoveryDistance += distance;
	}
}
bool PairContacts::gather()
{
	if ( !collect() )
		return false;
	depth = 0;
	for ( auto& row : rows )
	{
		depth = std::max( depth, -double( row.gap ) );
		if ( row.centres )
			continue;
		for ( const auto& old : previous )
			if ( old.a == row.a && old.b == row.b && old.generationA == row.generationA &&
				 old.generationB == row.generationB && !old.centres && old.edgeA == row.edgeA &&
				 old.edgeB == row.edgeB && std::abs( old.tA - row.tA ) < .05f &&
				 std::abs( old.tB - row.tB ) < .05f && dot( old.normal, row.normal ) > .98f )
			{
				row.normalImpulse = old.normalImpulse;
				row.tangentImpulse = old.tangentImpulse;
				++warmHits;
				break;
			}
	}
	return true;
}
void PairContacts::warmStart()
{
	for ( const auto& r : rows )
		apply( r,
			   add( scale( r.normal, r.normalImpulse ),
					scale( Vec{ -r.normal[1], r.normal[0] }, r.tangentImpulse ) ),
			   false );
}
void PairContacts::solve()
{
	for ( auto& r : rows )
	{
		float inv = inverse( r );
		auto relative = [&]() {
			return difference( velocity( r.a, r.edgeA, r.tA, r.centres ),
							   velocity( r.b, r.edgeB, r.tB, r.centres ) );
		};
		float next = std::max(
			0.f, r.normalImpulse -
					 ( dot( relative(), r.normal ) + std::max( r.gap, 0.f ) / core.dt ) / inv );
		apply( r, scale( r.normal, next - r.normalImpulse ), false );
		r.normalImpulse = next;
		Vec tangent{ -r.normal[1], r.normal[0] };
		float limit =
			r.centres ? 0
					  : std::sqrt( core.bodies[r.a].friction * core.bodies[r.b].friction ) * next;
		next = std::max( -limit,
						 std::min( limit, r.tangentImpulse - dot( relative(), tangent ) / inv ) );
		apply( r, scale( tangent, next - r.tangentImpulse ), false );
		r.tangentImpulse = next;
		++rowSolves;
	}
}
void PairContacts::finish()
{
	previous = rows;
	recoverySteps += recoveryUsed;
	recoveringPairs = 0;
	for ( int a = 0; a < int( core.bodies.size() ); ++a )
		for ( int b = a + 1; b < int( core.bodies.size() ); ++b )
		{
			auto& state = states[a * MaxBodies + b];
			if ( !state.active )
				continue;
			// A separating plane certifies disjoint filled regions, including
			// nearly coincident contours that defeat nearest-boundary depth.
			bool separated = separation( a, b, state.normal ) >= 0 &&
							 Game::inspectTopology( core.bodies[a] ).valid() &&
							 Game::inspectTopology( core.bodies[b] ).valid();
			state.separatedSteps = separated ? state.separatedSteps + 1 : 0;
			if ( state.separatedSteps >= 8 )
			{
				state.active = false;
				++recoveryExits;
			}
			else
				++recoveringPairs;
		}
}
} // namespace SoftBody
