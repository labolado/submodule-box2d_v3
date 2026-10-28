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
		return;
	}
	for ( int i = 0; i < b.nodeCount; ++i )
	{
		double x = double( b.x[i][0] ) - c[0], y = double( b.x[i][1] ) - c[1];
		b.v[i][0] -= T( double( dt ) * value * y / inertia );
		b.v[i][1] += T( double( dt ) * value * x / inertia );
	}
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

			auto& f = b.forceScratch;
			b.force( f );
			if ( b.failure )
			{
				failure = b.failure;
				return;
			}
			if ( b.pressure.maxPressure > 0 )
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

template <class T> typename World<T>::Metrics World<T>::currentMetrics() const
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

				for ( int j = i + 2; j < b.nodeCount; ++j )
					if ( !( i == 0 && j == b.nodeCount - 1 ) )
					{
						bool crossing =
							strictCross<T>( p, q, b.x[j], b.x[( j + 1 ) % b.nodeCount] );
						m.selfCross = m.selfCross || crossing;
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
