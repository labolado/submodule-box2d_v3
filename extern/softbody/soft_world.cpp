#include "soft_world.h"

namespace SoftBody
{
#if defined( __clang__ )
#pragma clang fp contract( off )
#endif
template <class T> bool World<T>::pairEnabled( int a, int b ) const
{
	if ( !bodies[a].active || !bodies[b].active )
		return false;
	for ( auto p : exclusions )
		if ( ( p[0] == a && p[1] == b ) || ( p[0] == b && p[1] == a ) )
			return false;
	return true;
}

template <class T> bool World<T>::boundaryEnabled( int body, const Boundary& boundary ) const
{
	if ( !bodies[body].active || !boundary.active || !boundary.enabled )
		return false;
	for ( int excluded : boundary.excludedBodies )
		if ( excluded == body )
			return false;
	return true;
}

template <class T> typename World<T>::Vec World<T>::anchor( int id, const T* weights ) const
{
	Vec result{};
	for ( int i = 0; i < bodies[id].nodeCount; ++i )
		for ( int k = 0; k < 2; ++k )
			result[k] += weights[i] * bodies[id].x[i][k];
	return result;
}

template <class T> typename World<T>::Vec World<T>::pinTarget( const Pin& pin ) const
{
	return pin.b < 0 ? pin.worldAnchor : anchor( pin.b, pin.wb );
}

template <class T> typename World<T>::Vec World<T>::boxLocal( const Boundary& box, Vec point ) const
{
	T x = point[0] - box.center[0], y = point[1] - box.center[1];
	return { box.axis[0] * x + box.axis[1] * y, -box.axis[1] * x + box.axis[0] * y };
}

template <class T>
typename World<T>::Vec World<T>::boxVertex( const Boundary& box, int corner ) const
{
	T x = ( corner & 1 ? T( 1 ) : T( -1 ) ) * box.halfSize[0];
	T y = ( corner & 2 ? T( 1 ) : T( -1 ) ) * box.halfSize[1];
	return { box.center[0] + box.axis[0] * x - box.axis[1] * y,
			 box.center[1] + box.axis[1] * x + box.axis[0] * y };
}

template <class T> typename World<T>::Vec World<T>::center( int id ) const
{
	Vec result{};
	for ( int i = 0; i < bodies[id].nodeCount; ++i )
		for ( int k = 0; k < 2; ++k )
			result[k] += bodies[id].x[i][k] / T( bodies[id].nodeCount );
	return result;
}

template <class T> double World<T>::kinetic( int id ) const
{
	double e = 0;
	for ( int i = 0; i < bodies[id].nodeCount; ++i )
		for ( T v : bodies[id].v[i] )
			e += .5 * bodies[id].mass[i] * double( v ) * v;
	return e;
}

template <class T> double World<T>::omega( int id ) const
{
	auto c = center( id );
	const auto& b = bodies[id];
	double angular = 0, inertia = 0;
	for ( int i = 0; i < b.nodeCount; ++i )
	{
		double x = double( b.x[i][0] ) - c[0], y = double( b.x[i][1] ) - c[1];
		inertia += b.mass[i] * ( x * x + y * y );
		angular += b.mass[i] * ( x * b.v[i][1] - y * b.v[i][0] );
	}
	if ( inertia < 1e-12 )
	{
		if ( b.kind != 3 )
			failure = "degenerate_motor_inertia";
		return 0;
	}
	return angular / inertia;
}

template <class T> void World<T>::torque( int id, double value )
{
	auto c = center( id );
	auto& b = bodies[id];
	double inertia = 0;
	for ( int i = 0; i < b.nodeCount; ++i )
	{
		double x = double( b.x[i][0] ) - c[0], y = double( b.x[i][1] ) - c[1];
		inertia += b.mass[i] * ( x * x + y * y );
	}
	if ( inertia < 1e-12 )
	{
		// A coincident recoverable body has no lever arm. Ignore torque
		// until material forces open it; never divide by zero inertia.
		if ( b.kind != 3 )
			failure = "degenerate_motor_inertia";
		return;
	}
	for ( int i = 0; i < b.nodeCount; ++i )
	{
		double x = double( b.x[i][0] ) - c[0], y = double( b.x[i][1] ) - c[1];
		b.v[i][0] -= T( double( dt ) * value * y / inertia );
		b.v[i][1] += T( double( dt ) * value * x / inertia );
	}
}

template <class T>
void World<T>::boundary( int id, int node, Vec normal, T depth, T velocity, T friction )
{
	auto& part = bodies[id];
	T j = part.mass[node] * depth / dt;
	for ( int k = 0; k < 2; ++k )
	{
		T delta = depth * normal[k];
		part.x[node][k] += delta;
		part.v[node][k] += delta / dt;
		external[k] += double( j ) * normal[k];
	}
	boundaryWork += double( j ) * velocity;
	Vec tangent{ -normal[1], normal[0] };
	T speed = part.v[node][0] * tangent[0] + part.v[node][1] * tangent[1];
	T jt = std::max( -friction * j, std::min( friction * j, -part.mass[node] * speed ) );
	double before = kinetic( id );
	for ( int k = 0; k < 2; ++k )
	{
		T dv = jt * tangent[k] / part.mass[node];
		part.v[node][k] += dv;
		part.x[node][k] += dt * dv;
		external[k] += double( jt ) * tangent[k];
	}
	frictionLoss += before - kinetic( id );
	++contacts;
}

template <class T> void World<T>::corner( int id, const Boundary& boundary )
{
	boundaryVertex( id, { boundary.left, boundary.top }, boundary.friction );
}

template <class T> void World<T>::boundaryVertex( int id, Vec point, T friction )
{
	auto& part = bodies[id];
	if ( !inside<T>( point, part ) )
		return;
	auto hit = nearest<T>( point, part );
	if ( hit.edge < 0 )
	{
		failure = "degenerate_contact_edge";
		return;
	}
	if ( hit.depth < T( 1e-8 ) )
		return;
	int a = hit.edge, b = ( a + 1 ) % part.nodeCount;
	T wa = T( 1 ) - hit.t, wb = hit.t;
	Vec normal{ -hit.normal[0], -hit.normal[1] }, tangent{ -normal[1], normal[0] };
	T inv = wa * wa / part.mass[a] + wb * wb / part.mass[b], j = hit.depth / ( dt * inv );
	auto apply = [&]( Vec direction, T impulse ) {
		for ( int k = 0; k < 2; ++k )
		{
			T da = wa * impulse * direction[k] / part.mass[a],
			  db = wb * impulse * direction[k] / part.mass[b];
			part.v[a][k] += da;
			part.v[b][k] += db;
			part.x[a][k] += dt * da;
			part.x[b][k] += dt * db;
			external[k] += double( impulse ) * direction[k];
		}
	};
	apply( normal, j );
	T speed = 0;
	for ( int k = 0; k < 2; ++k )
		speed += ( wa * part.v[a][k] + wb * part.v[b][k] ) * tangent[k];
	T jt = std::max( -friction * j, std::min( friction * j, -speed / inv ) );
	double before = kinetic( id );
	apply( tangent, jt );
	frictionLoss += before - kinetic( id );
	++contacts;
	++cornerContacts;
}

template <class T> void World<T>::solveTouchJoint( TouchJoint& joint )
{
	auto& body = bodies[joint.body];
	Vec position = anchor( joint.body, joint.weights );
	Vec velocity{};
	T inverseMass = 0;
	for ( int node = 0; node < body.nodeCount; ++node )
	{
		T weight = joint.weights[node];
		inverseMass += weight * weight / body.mass[node];
		for ( int axis = 0; axis < 2; ++axis )
		{
			velocity[axis] += weight * body.v[node][axis];
		}
	}

	// Implicit damped spring at a material point. The target is prescribed;
	// its reaction belongs to the external controller, not another body.
	T omega = T( 2 * 3.14159265358979323846 ) * joint.frequency;
	T damping = T( 2 ) * joint.dampingRatio * omega;
	T denominator = inverseMass * ( T( 1 ) + damping * dt + omega * omega * dt * dt );
	Vec force{};
	for ( int axis = 0; axis < 2; ++axis )
	{
		T error = joint.target[axis] - position[axis];
		T velocityCoefficient = damping + omega * omega * dt;
		force[axis] =
			( omega * omega * error - velocityCoefficient * velocity[axis] ) / denominator;
	}
	T magnitude = std::hypot( force[0], force[1] );
	if ( magnitude > joint.maxForce )
	{
		T scale = joint.maxForce / magnitude;
		for ( T& value : force )
			value *= scale;
	}
	joint.lastForce = std::min( magnitude, joint.maxForce );

	double before = kinetic( joint.body );
	for ( int node = 0; node < body.nodeCount; ++node )
	{
		for ( int axis = 0; axis < 2; ++axis )
		{
			T impulse = dt * joint.weights[node] * force[axis];
			body.v[node][axis] += impulse / body.mass[node];
			external[axis] += impulse;
		}
	}
	appliedWork += kinetic( joint.body ) - before;
}

template <class T> void World<T>::advanceMaterialVelocity()
{
	for ( auto& b : bodies )
		if ( b.active )
		{
			// For the finite recovery spring, damp the incoming motion before
			// adding elastic impulses. Damping those impulses afterwards would
			// reduce the wall reaction without reducing stored elastic energy.
			if ( b.kind == 3 )
			{
				if ( b.referenceScale.enabled )
					appliedWork += b.updateReferenceScale( dt );
				b.dampDeformation( dt );
				if ( !b.internalSprings.empty() )
				{
					b.updateInternalSpringRecovery( dt );
					b.dampInternalSprings( dt );
				}
				if ( b.failure )
				{
					failure = b.failure;
					return;
				}
			}
			auto& f = b.forceScratch;
			b.force( f );
			if ( b.failure )
			{
				failure = b.failure;
				return;
			}
			if ( b.kind == 3 && b.pressure.maxPressure > 0 )
			{
				b.updatePressure( dt );
				double work = 0;
				for ( int i = 0; i < b.nodeCount; ++i )
					for ( int k = 0; k < 2; ++k )
					{
						double before = b.v[i][k];
						T pressureForce = b.pressure.forces[i][k];
						b.v[i][k] += dt * ( ( f[i][k] + pressureForce ) / b.mass[i] + gravity[k] );
						// Attribute work using the average velocity of the COMBINED
						// material/gravity/pressure update. Separate kicks would report
						// fictitious pressure work even at exact force equilibrium.
						work += .5 * ( before + b.v[i][k] ) * dt * pressureForce;
					}
				b.pressure.work += work;
				appliedWork += work;
			}
			else
				for ( int i = 0; i < b.nodeCount; ++i )
					for ( int k = 0; k < 2; ++k )
						b.v[i][k] += dt * ( f[i][k] / b.mass[i] + gravity[k] );
		}
}

template <class T>
void World<T>::solveSubstep( const std::vector<T>& offsets, const std::vector<T>& velocities )
{
	solveSubstep( offsets, velocities, nullptr, nullptr );
}

template <class T>
void World<T>::solveSubstep( const std::vector<T>& offsets, const std::vector<T>& velocities,
							 ContactProjector project, const void* context )
{
	auto previous = bodies;
	advanceMaterialVelocity();
	for ( size_t id = 0; id < bodies.size(); ++id )
		if ( bodies[id].active )
		{
			auto& b = bodies[id];
			double before = kinetic( int( id ) );
			for ( int i = 0; i < b.nodeCount; ++i )
				for ( int k = 0; k < 2; ++k )
				{
					T impulse = dt * b.forceInput[k] / T( b.nodeCount );
					b.v[i][k] += impulse / b.mass[i];
					external[k] += impulse;
				}
			if ( b.torqueInput != 0 )
				torque( int( id ), b.torqueInput );
			appliedWork += kinetic( int( id ) ) - before;
		}
	for ( auto& joint : touchJoints )
	{
		if ( joint.active )
			solveTouchJoint( joint );
	}
	std::vector<double> torques( bodies.size(), 0 );
	double before = 0;
	for ( const auto& p : pins )
		if ( p.active && p.maxTorque > 0 )
		{
			if ( p.b < 0 )
			{
				double value =
					std::max( -double( p.maxTorque ),
							  std::min( double( p.maxTorque ),
										double( p.gain ) * ( double( p.speed ) - omega( p.a ) ) ) );
				torques[p.a] += value;
				continue;
			}
			double value =
				std::max( -double( p.maxTorque ),
						  std::min( double( p.maxTorque ),
									double( p.gain ) *
										( double( p.speed ) - ( omega( p.b ) - omega( p.a ) ) ) ) );
			torques[p.b] += value;
			torques[p.a] -= value;
		}
	for ( size_t i = 0; i < bodies.size(); ++i )
		if ( bodies[i].active )
			before += kinetic( int( i ) );
	for ( size_t i = 0; i < bodies.size(); ++i )
		if ( bodies[i].active && torques[i] != 0 )
			torque( int( i ), torques[i] );
	double after = 0;
	for ( size_t i = 0; i < bodies.size(); ++i )
		if ( bodies[i].active )
			after += kinetic( int( i ) );
	motorWork += after - before;
	if ( failure )
		return;
	for ( auto& b : bodies )
		if ( b.active )
		{
			if ( b.kind != 3 )
				b.dampDeformation( dt );
			if ( b.failure )
			{
				failure = b.failure;
				return;
			}
			for ( int i = 0; i < b.nodeCount; ++i )
				for ( int k = 0; k < 2; ++k )
					b.x[i][k] += dt * b.v[i][k];
		}
	for ( int iteration = 0; iteration < iterations; ++iteration )
	{
		for ( size_t id = 0; id < bodies.size(); ++id )
			if ( bodies[id].active )
				for ( int i = 0; i < bodies[id].nodeCount; ++i )
				{
					for ( size_t j = 0; j < boundaries.size(); ++j )
					{
						const auto& b = boundaries[j];
						if ( !boundaryEnabled( int( id ), b ) )
							continue;
						auto p = bodies[id].x[i];
						if ( b.kind == 0 )
						{
							T depth = offsets[j] - b.normal[0] * p[0] - b.normal[1] * p[1];
							if ( depth > 0 )
								boundary( int( id ), i, b.normal, depth, velocities[j],
										  b.friction );
						}
						else if ( b.kind == 2 )
						{
							auto local = boxLocal( b, p );
							T dx = b.halfSize[0] - std::abs( local[0] );
							T dy = b.halfSize[1] - std::abs( local[1] );
							if ( dx > 0 && dy > 0 )
							{
								T sign = ( dx < dy ? local[0] : local[1] ) < 0 ? T( -1 ) : T( 1 );
								Vec normal = dx < dy ? Vec{ sign * b.axis[0], sign * b.axis[1] }
													 : Vec{ -sign * b.axis[1], sign * b.axis[0] };
								boundary( int( id ), i, normal, std::min( dx, dy ), 0, b.friction );
							}
						}
						else
						{
							T left = p[0] - b.left, up = b.top - p[1];
							if ( left > 0 && up > 0 )
							{
								if ( left < up )
									boundary( int( id ), i, { T( -1 ), T( 0 ) }, left, 0,
											  b.friction );
								else
									boundary( int( id ), i, { T( 0 ), T( 1 ) }, up, 0, b.friction );
							}
						}
					}
				}
		for ( const auto& b : boundaries )
			if ( b.active && b.enabled && b.kind == 1 )
				for ( size_t id = 0; id < bodies.size(); ++id )
					if ( boundaryEnabled( int( id ), b ) )
						corner( int( id ), b );
		for ( const auto& box : boundaries )
			if ( box.active && box.enabled && box.kind == 2 )
				for ( size_t id = 0; id < bodies.size(); ++id )
					if ( boundaryEnabled( int( id ), box ) )
						for ( int vertex = 0; vertex < 4; ++vertex )
							boundaryVertex( int( id ), boxVertex( box, vertex ), box.friction );
		if ( project )
			project( *this, context );
		if ( failure )
			return;
		for ( size_t a = 0; a < bodies.size(); ++a )
			for ( size_t b = a + 1; b < bodies.size(); ++b )
				if ( pairEnabled( int( a ), int( b ) ) )
				{
					if ( guardEdgeOnly( bodies[a], bodies[b] ) )
						++strictEvents;
					double impulse = 0;
					int count =
						projectPair( bodies[a], bodies[b], dt, impulse, false, &frictionLoss );
					if ( count < 0 )
					{
						failure = "degenerate_contact_edge";
						return;
					}
					contacts += count;
				}
		for ( const auto& p : pins )
			if ( p.active )
				for ( int k = 0; k < 2; ++k )
				{
					T error = anchor( p.a, p.wa )[k] - pinTarget( p )[k], inv = 0;
					int countA = bodies[p.a].nodeCount,
						countB = p.b < 0 ? 0 : bodies[p.b].nodeCount;
					for ( int i = 0; i < std::max( countA, countB ); ++i )
						inv += ( i < countA ? p.wa[i] * p.wa[i] / bodies[p.a].mass[i] : T( 0 ) ) +
							   ( i < countB ? p.wb[i] * p.wb[i] / bodies[p.b].mass[i] : T( 0 ) );
					if ( !( inv > 0 ) )
					{
						failure = "invalid_pin_mass";
						return;
					}
					T lambda = -error / inv;
					for ( int i = 0; i < countA; ++i )
					{
						T da = p.wa[i] * lambda / bodies[p.a].mass[i];
						bodies[p.a].x[i][k] += da;
						bodies[p.a].v[i][k] += da / dt;
						if ( p.b < 0 )
							external[k] += double( bodies[p.a].mass[i] ) * da / dt;
					}
					for ( int i = 0; i < countB; ++i )
					{
						T db = -p.wb[i] * lambda / bodies[p.b].mass[i];
						bodies[p.b].x[i][k] += db;
						bodies[p.b].v[i][k] += db / dt;
					}
				}
	}
	for ( size_t a = 0; a < bodies.size(); ++a )
		for ( size_t b = a + 1; b < bodies.size(); ++b )
			if ( pairEnabled( int( a ), int( b ) ) )
				if ( guardTransit( previous[a], bodies[a], previous[b], bodies[b] ) ||
					 guardTransit( previous[b], bodies[b], previous[a], bodies[a] ) )
					++strictEvents;
}

template <class T>
typename World<T>::Metrics World<T>::metrics( const std::vector<T>& offsets ) const
{
	Metrics m;
	for ( size_t id = 0; id < bodies.size(); ++id )
		if ( bodies[id].active )
		{
			const auto& b = bodies[id];
			auto& f = b.forceScratch;
			m.energy += b.force( f ) + kinetic( int( id ) );
			if ( b.failure )
			{
				failure = b.failure;
				return m;
			}
			m.minArea = std::min( m.minArea, double( b.area() / b.restArea ) );
			V cv{};
			for ( int i = 0; i < b.nodeCount; ++i )
			{
				auto p = b.x[i], q = b.x[( i + 1 ) % b.nodeCount];
				for ( int k = 0; k < 2; ++k )
				{
					if ( !std::isfinite( double( p[k] ) ) || !std::isfinite( double( b.v[i][k] ) ) )
					{
						failure = "nonfinite";
						return m;
					}
					m.energy -= double( b.mass[i] ) * gravity[k] * p[k];
					cv[k] += double( b.v[i][k] ) / b.nodeCount;
				}
				m.nodeSpeed = std::max( m.nodeSpeed, double( std::hypot( b.v[i][0], b.v[i][1] ) ) );
				for ( size_t j = 0; j < boundaries.size(); ++j )
				{
					const auto& edge = boundaries[j];
					if ( !boundaryEnabled( int( id ), edge ) )
						continue;
					if ( edge.kind == 0 )
						m.penetration =
							std::max( m.penetration, double( offsets[j] - edge.normal[0] * p[0] -
															 edge.normal[1] * p[1] ) );
					else if ( edge.kind == 2 )
					{
						auto local = boxLocal( edge, p );
						m.penetration = std::max(
							m.penetration,
							double( std::min( edge.halfSize[0] - std::abs( local[0] ),
											  edge.halfSize[1] - std::abs( local[1] ) ) ) );
					}
					else
					{
						double ax = double( p[0] ) - edge.left, ay = double( edge.top ) - p[1],
							   dx = double( q[0] ) - p[0], dy = double( p[1] ) - q[1];
						m.penetration = std::max( m.penetration, std::min( ax, ay ) );
						m.penetration = std::max( m.penetration, std::min( ax + dx, ay + dy ) );
						if ( std::abs( dx - dy ) > 1e-15 )
						{
							double t = ( ay - ax ) / ( dx - dy );
							if ( t > 0 && t < 1 )
								m.penetration =
									std::max( m.penetration, std::min( ax + t * dx, ay + t * dy ) );
						}
					}
				}
				for ( int j = i + 2; j < b.nodeCount; ++j )
					if ( !( i == 0 && j == b.nodeCount - 1 ) )
					{
						bool crossing =
							strictCross<T>( p, q, b.x[j], b.x[( j + 1 ) % b.nodeCount] );
						m.selfCross = m.selfCross || crossing;
						m.strictSelfCross = m.strictSelfCross || ( crossing && b.kind != 3 );
					}
			}
			m.centerSpeed = std::max( m.centerSpeed, std::hypot( cv[0], cv[1] ) );
		}
	for ( size_t a = 0; a < bodies.size(); ++a )
		for ( size_t b = a + 1; b < bodies.size(); ++b )
			if ( pairEnabled( int( a ), int( b ) ) )
			{
				for ( int i = 0; i < bodies[a].nodeCount; ++i )
					if ( inside<T>( bodies[a].x[i], bodies[b] ) )
						m.penetration =
							std::max( m.penetration,
									  double( nearest<T>( bodies[a].x[i], bodies[b] ).depth ) );
				for ( int i = 0; i < bodies[b].nodeCount; ++i )
					if ( inside<T>( bodies[b].x[i], bodies[a] ) )
						m.penetration =
							std::max( m.penetration,
									  double( nearest<T>( bodies[b].x[i], bodies[a] ).depth ) );
			}
	for ( const auto& p : pins )
		if ( p.active )
		{
			auto a = anchor( p.a, p.wa ), b = pinTarget( p );
			m.pin = std::max( m.pin, double( std::hypot( a[0] - b[0], a[1] - b[1] ) ) );
		}
	return m;
}

template <class T> typename World<T>::Metrics World<T>::currentMetrics() const
{
	std::vector<T> offsets;
	for ( const auto& b : boundaries )
		offsets.push_back( b.offset );
	return metrics( offsets );
}

template <class T> void World<T>::step()
{
	step( nullptr, nullptr );
}

template <class T>
void World<T>::step( ContactProjector project, const void* context, ContactMeasure measure )
{
	if ( failure )
		return;
	peak = Metrics();
	minimumY.assign( bodies.size(), std::numeric_limits<T>::max() );
	maxUnforcedEnergy = -std::numeric_limits<double>::infinity();
	std::vector<T> starts, offsets, velocities;
	for ( const auto& b : boundaries )
	{
		starts.push_back( b.offset );
		offsets.push_back( b.offset );
		velocities.push_back( ( b.target - b.offset ) / ( dt * T( substeps ) ) );
	}
	for ( int s = 1; s <= substeps; ++s )
	{
		for ( size_t j = 0; j < boundaries.size(); ++j )
			offsets[j] = starts[j] + ( boundaries[j].target - starts[j] ) * T( s ) / T( substeps );
		solveSubstep( offsets, velocities, project, context );
		++completedSubsteps;
		if ( failure )
			return;
		auto m = metrics( offsets );
		if ( failure )
			return;
		if ( measure )
			m.penetration = std::max( m.penetration, measure( *this, context ) );
		maxUnforcedEnergy =
			std::max( maxUnforcedEnergy, m.energy - motorWork - boundaryWork - appliedWork );
		for ( size_t id = 0; id < bodies.size(); ++id )
			if ( bodies[id].active )
				for ( int i = 0; i < bodies[id].nodeCount; ++i )
					minimumY[id] = std::min( minimumY[id], bodies[id].x[i][1] );
		peak.penetration = std::max( peak.penetration, m.penetration );
		peak.pin = std::max( peak.pin, m.pin );
		peak.nodeSpeed = std::max( peak.nodeSpeed, m.nodeSpeed );
		peak.centerSpeed = std::max( peak.centerSpeed, m.centerSpeed );
		peak.minArea = std::min( peak.minArea, m.minArea );
		peak.selfCross = peak.selfCross || m.selfCross;
		peak.energy = m.energy;
		if ( m.strictSelfCross )
		{
			failure = "self_crossing";
			return;
		}
	}
	for ( auto& b : boundaries )
		b.offset = b.target;
	for ( auto& b : bodies )
	{
		b.forceInput = {};
		b.torqueInput = 0;
	}
}

template <class T> void World<T>::removeBody( int id )
{
	bodies[id].active = false;
	for ( auto& pin : pins )
	{
		if ( pin.a == id || pin.b == id )
			pin.active = false;
	}
	for ( auto& joint : touchJoints )
	{
		if ( joint.body == id )
			joint.active = false;
	}
}

template struct World<float>;
template struct World<double>;
} // namespace SoftBody
