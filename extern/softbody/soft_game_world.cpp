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
	if ( boxCount == MaxBoxes )
		return false;
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
		auto nextFilter = b2Shape_GetFilter( box.shape );
		float nextFriction = b2Shape_GetFriction( box.shape );
		bool nextEnabled =
			b2Body_IsEnabled( box.body ) && !b2Shape_IsSensor( box.shape ) && enabled( nextFilter );
		if ( box.hx != hx || box.hy != hy || box.friction != nextFriction ||
			 box.enabled != nextEnabled || box.filter.categoryBits != nextFilter.categoryBits ||
			 box.filter.maskBits != nextFilter.maskBits ||
			 box.filter.groupIndex != nextFilter.groupIndex )
			clearCache();
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
			if ( B2_ID_EQUALS( boxes[j].body, box.body ) )
			{
				error = "game_duplicate_body";
				return false;
			}
	}
	if ( pin.active && ( error = Bridge::validateStepBody( world, pin.rigid ) ) )
		return false;
	return true;
}

b2Vec2 Solver::point( const Row& row ) const
{
	const auto& body = this->body();
	const auto& a = body.x[row.edge];
	const auto& b = body.x[( row.edge + 1 ) % 16];
	return { ( 1 - row.fraction ) * a[0] + row.fraction * b[0],
			 ( 1 - row.fraction ) * a[1] + row.fraction * b[1] };
}

float Solver::softInverse( const Row& row ) const
{
	const auto& mass = body().mass;
	return ( 1 - row.fraction ) * ( 1 - row.fraction ) / mass[row.edge] +
		   row.fraction * row.fraction / mass[( row.edge + 1 ) % 16];
}

void Solver::append( Row row )
{
	if ( rowCount == MaxRows )
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
	const b2Vec2 normals[4] = { { -1, 0 }, { 1, 0 }, { 0, -1 }, { 0, 1 } };
	auto& body = this->body();
	for ( int id = 0; id < boxCount; ++id )
	{
		auto& box = boxes[id];
		if ( !box.enabled )
			continue;
		b2WorldTransform transform = { box.state.origin, box.state.rotation };
		int start = rowCount;
		for ( int node = 0; node < 16; ++node )
		{
			auto p = b2InvTransformWorldPoint( transform, { body.x[node][0], body.x[node][1] } );
			float distance = -1e30f;
			int face = 0;
			for ( int j = 0; j < 4; ++j )
			{
				float d = b2Dot( p, normals[j] ) - ( j < 2 ? box.hx : box.hy );
				if ( d > distance )
				{
					distance = d;
					face = j;
				}
			}
			if ( distance > Margin )
				continue;
			Row row;
			row.box = id;
			row.edge = node;
			row.feature = face;
			row.normal = b2RotateVector( transform.q, normals[face] );
			row.localAnchor = b2MulAdd( p, -distance, normals[face] );
			append( row );
		}
		for ( int corner = 0; corner < 4; ++corner )
		{
			// A folded contour is not an oriented solid. Its material
			// nodes still collide with real box faces; never ask inside
			// or nearest-edge queries to invent an interior in this mode.
			if ( recovery.active )
				break;
			b2Vec2 local = { corner & 1 ? box.hx : -box.hx, corner & 2 ? box.hy : -box.hy };
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
			row.feature = 4 + corner;
			row.fraction = hit.t;
			row.localAnchor = local;
			row.normal = { -hit.normal[0], -hit.normal[1] };
			{
				// The point may be just outside after split projection.
				// Use the oriented material edge, not the sign of its
				// nearly-zero distance to the box corner.
				auto a = body.x[hit.edge], b = body.x[( hit.edge + 1 ) % 16];
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
			BoxProjection geometry( body(), box.state, box.hx, box.hy );
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
		   pin.weights[( row.edge + 1 ) % 16] * double( row.fraction ) /
			   body.mass[( row.edge + 1 ) % 16];
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
	for ( int i = 0; i < 16; ++i )
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
	const auto& rigid = boxes[row.box].state;
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
	const auto& rigid = boxes[row.box].state;
	auto p = point( row );
	float vx = ( 1 - row.fraction ) * body.v[row.edge][0] +
			   row.fraction * body.v[( row.edge + 1 ) % 16][0];
	float vy = ( 1 - row.fraction ) * body.v[row.edge][1] +
			   row.fraction * body.v[( row.edge + 1 ) % 16][1];
	return b2Dot( { vx - rigid.velocity.x + rigid.angularVelocity * float( p.y - rigid.center.y ),
					vy - rigid.velocity.y - rigid.angularVelocity * float( p.x - rigid.center.x ) },
				  direction );
}

void Solver::apply( const Row& row, b2Vec2 impulse )
{
	auto& body = this->body();
	auto& rigid = boxes[row.box].state;
	auto p = point( row );
	double px = 0, py = 0;
	for ( int side = 0; side < 2; ++side )
	{
		int node = ( row.edge + side ) % 16;
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
		for ( int i = 0; i < 16; ++i )
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
	auto& rigid = boxes[row.box].state;
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
	for ( int i = 0; i < 16; ++i )
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
	for ( int i = 0; i < 16; ++i )
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
	for ( int i = 0; i < 16; ++i )
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
			BoxProjection geometry( solver.body(), box.state, box.hx, box.hy );
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
	recovery.remaining.fill( recovery.maxCorrection );
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
	for ( int i = 0; i < 16; ++i )
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
	for ( int i = 0; i < 16; ++i )
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
			if ( ++touches > 1 || touch.body != 0 || !std::isfinite( touch.target[0] ) ||
				 !std::isfinite( touch.target[1] ) ||
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
		if ( pin.softBody != 0 || std::abs( total - 1 ) > 1e-6 ||
			 !std::isfinite( pin.localAnchor.x ) || !std::isfinite( pin.localAnchor.y ) )
		{
			error = "invalid_game_pin";
			return false;
		}
	}
	return true;
}
} // namespace Game
} // namespace SoftBody
