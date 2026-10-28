#include "soft_body.h"

#include "soft_contact.h"

namespace SoftBody
{
#if defined( __clang__ )
#pragma clang fp contract( off )
#endif
template <class T>
Body<T>::Body( T totalMass, T rx, T ry, T hz, T dampingRate, T scale )
	: frequency( hz )
	, damping( dampingRate )
{
	resizeNodes( nodeCount );
	const T pi = T( 3.14159265358979323846 );
	materialScale = scale;
	for ( int i = 0; i < nodeCount; ++i )
	{
		T a = T( 2 ) * pi * T( i ) / T( nodeCount );
		sourceNodes[i] = i;
		rest[i] = x[i] = { rx * std::cos( a ), ry * std::sin( a ) };
		mass[i] = totalMass / T( nodeCount );
	}
	for ( int i = 0; i < nodeCount; ++i )
	{
		auto a = x[i], b = x[( i + 1 ) % nodeCount];
		lengths[i] = std::hypot( a[0] - b[0], a[1] - b[1] );
	}
	restArea = area();
}

template <class T> void Body<T>::resizeNodes( int count )
{
	nodeCount = count;
	x.assign( count, Vec{} );
	v.assign( count, Vec{} );
	rest.assign( count, Vec{} );
	mass.assign( count, T( 0 ) );
	lengths.assign( count, T( 0 ) );
	sourceNodes.resize( count );
	forceScratch.resize( count );
	positionScratch.resize( count );
}

template <class T> void Body<T>::initializeReferenceScale()
{
	auto& scale = referenceScale;
	scale.baseRest = rest;
	scale.baseLengths = lengths;
	scale.area = restArea;
	double totalMass = 0;
	for ( int i = 0; i < nodeCount; ++i )
	{
		totalMass += mass[i];
		for ( int axis = 0; axis < 2; ++axis )
			scale.center[axis] += double( mass[i] ) * rest[i][axis];
	}
	for ( auto& axis : scale.center )
		axis /= totalMass;
	for ( auto p : rest )
		scale.radius =
			std::max( scale.radius, std::hypot( p[0] - scale.center[0], p[1] - scale.center[1] ) );
	for ( const auto& spring : internalSprings )
		scale.baseSprings.push_back( { spring.restLength, spring.epsilon } );
}

template <class T> double Body<T>::updateReferenceScale( T dt )
{
	auto& scale = referenceScale;
	if ( !scale.enabled || scale.value == scale.target )
		return 0;
	double distance = scale.target - scale.value;
	double rateStep = scale.rate * dt;
	double step = std::min( rateStep, scale.maxSpeed * dt / scale.radius );
	if ( step < std::min( rateStep, std::abs( distance ) ) )
		++scale.limitedSteps;
	double next =
		std::abs( distance ) <= step ? scale.target : scale.value + std::copysign( step, distance );
	// Account for active reference changes at unchanged positions and velocity.
	// Structural stiffness recovery is a separate parameter-work ledger.
	double before = double( recoverableForce( forceScratch.data() ) ) +
					internalSpringForce( nullptr, internalSpringScale );
	for ( int i = 0; i < nodeCount; ++i )
	{
		for ( int axis = 0; axis < 2; ++axis )
			rest[i][axis] = next == 1 ? scale.baseRest[i][axis]
									  : T( scale.center[axis] + next * ( scale.baseRest[i][axis] -
																		 scale.center[axis] ) );
		lengths[i] = T( scale.baseLengths[i] * next );
	}
	for ( size_t i = 0; i < internalSprings.size(); ++i )
	{
		internalSprings[i].restLength = scale.baseSprings[i][0] * next;
		internalSprings[i].epsilon = scale.baseSprings[i][1] * next;
	}
	restArea = T( scale.area * next * next );
	scale.value = next;
	double work = double( recoverableForce( forceScratch.data() ) ) +
				  internalSpringForce( nullptr, internalSpringScale ) - before;
	scale.work += work;
	return work;
}

template <class T> T Body<T>::area() const
{
	T a = 0;
	// Local origin reduces cancellation under large translations.
	for ( int i = 1; i < nodeCount - 1; ++i )
		a += ( x[i][0] - x[0][0] ) * ( x[i + 1][1] - x[0][1] ) -
			 ( x[i][1] - x[0][1] ) * ( x[i + 1][0] - x[0][0] );
	return a / T( 2 );
}

template <class T> T Body<T>::supportedForce( Vec* forces ) const
{
	// Distributed world support uses the same material stiffness. The target
	// never follows contact corrections or a folded/inverted configuration.
	double omega = 2 * 3.14159265358979323846 * double( frequency );
	double energy = 0;
	for ( int node = 0; node < nodeCount; ++node )
	{
		double stiffness = omega * omega * mass[node] * materialScale;
		for ( int axis = 0; axis < 2; ++axis )
		{
			double error = double( x[node][axis] ) - referencePosition[axis] - rest[node][axis];
			forces[node][axis] = T( -stiffness * error );
			energy += .5 * stiffness * error * error;
		}
	}
	return T( energy );
}

template <class T> T Body<T>::recoverableForce( Vec* forces ) const
{
	if ( worldSupported )
		return supportedForce( forces );
	double totalMass = 0;
	V center{}, restCenter{};
	for ( int node = 0; node < nodeCount; ++node )
	{
		totalMass += mass[node];
		for ( int axis = 0; axis < 2; ++axis )
		{
			center[axis] += double( mass[node] ) * x[node][axis];
			restCenter[axis] += double( mass[node] ) * rest[node][axis];
		}
	}
	for ( int axis = 0; axis < 2; ++axis )
	{
		center[axis] /= totalMass;
		restCenter[axis] /= totalMass;
	}
	double dot = 0, cross = 0;
	for ( int node = 0; node < nodeCount; ++node )
	{
		double rx = rest[node][0] - restCenter[0], ry = rest[node][1] - restCenter[1];
		double px = x[node][0] - center[0], py = x[node][1] - center[1];
		dot += mass[node] * ( rx * px + ry * py );
		cross += mass[node] * ( rx * py - ry * px );
	}
	double norm = std::hypot( dot, cross );
	if ( norm > 1e-12 )
	{
		// Proper rotation only: never fit a reflection to an inverted body.
		recoveryCosine = dot / norm;
		recoverySine = cross / norm;
	}
	// A completely ambiguous covariance retains the last defined rotation.
	double omega = 2 * 3.14159265358979323846 * double( frequency );
	double energy = 0;
	for ( int node = 0; node < nodeCount; ++node )
	{
		double rx = rest[node][0] - restCenter[0], ry = rest[node][1] - restCenter[1];
		V error{ x[node][0] - center[0] - recoveryCosine * rx + recoverySine * ry,
				 x[node][1] - center[1] - recoverySine * rx - recoveryCosine * ry };
		double stiffness = omega * omega * mass[node] * materialScale;
		for ( int axis = 0; axis < 2; ++axis )
		{
			forces[node][axis] = T( -stiffness * error[axis] );
			energy += .5 * stiffness * error[axis] * error[axis];
		}
	}
	return T( energy );
}

template <class T> void Body<T>::updateInternalSpringRecovery( T dt )
{
	bool valid = area() > 0;
	for ( int i = 0; i < nodeCount && valid; ++i )
	{
		auto a = x[i], b = x[( i + 1 ) % nodeCount];
		valid = std::hypot( double( a[0] ) - b[0], double( a[1] ) - b[1] ) > 1e-6;
		for ( int j = i + 2; j < nodeCount && valid; ++j )
			if ( !( i == 0 && j == nodeCount - 1 ) )
				valid = !strictCross<T>( a, b, x[j], x[( j + 1 ) % nodeCount] );
	}
	double before = internalSpringScale;
	if ( !valid )
	{
		internalSpringScale = 0;
		internalSpringHealthySteps = 0;
	}
	else if ( internalSpringHealthySteps < 8 )
		++internalSpringHealthySteps;
	else
		internalSpringScale = std::min( 1., internalSpringScale + double( dt ) * frequency );
	if ( internalSpringScale < 1 )
		++internalSpringRecoverySteps;
	if ( before != internalSpringScale )
		// Recovery changes structural stiffness, hence stored elastic energy.
		// Record that signed work; do not hide it in gravity or external input.
		internalSpringWork += ( internalSpringScale - before ) * internalSpringForce( nullptr );
}

template <class T> T Body<T>::internalSpringForce( Vec* forces, double scale ) const
{
	double energy = 0;
	for ( const auto& spring : internalSprings )
	{
		V d{ double( x[spring.b][0] ) - x[spring.a][0], double( x[spring.b][1] ) - x[spring.a][1] };
		// Smooth distance has a finite gradient even when endpoints coincide.
		// At exact coincidence this edge supplies no preferred direction; the
		// recoverable material, not a rest-direction impulse, unfolds the body.
		double length = std::sqrt( d[0] * d[0] + d[1] * d[1] + spring.epsilon * spring.epsilon );
		double extension = length - spring.restLength;
		double factor = scale * spring.stiffness * extension / length;
		energy += .5 * scale * spring.stiffness * extension * extension;
		for ( int axis = 0; forces && axis < 2; ++axis )
		{
			T force = T( factor * d[axis] );
			forces[spring.a][axis] += force;
			forces[spring.b][axis] -= force;
		}
	}
	return T( energy );
}

template <class T> void Body<T>::dampInternalSprings( T dt )
{
	if ( internalSpringScale == 0 )
		return;
	for ( const auto& spring : internalSprings )
	{
		if ( spring.damping == 0 )
			continue;
		V d{ double( x[spring.b][0] ) - x[spring.a][0], double( x[spring.b][1] ) - x[spring.a][1] };
		double length = std::hypot( d[0], d[1] );
		if ( length <= spring.epsilon )
			continue;
		for ( auto& component : d )
			component /= length;
		double inverseMass = 1. / mass[spring.a] + 1. / mass[spring.b];
		double speed = ( double( v[spring.b][0] ) - v[spring.a][0] ) * d[0] +
					   ( double( v[spring.b][1] ) - v[spring.a][1] ) * d[1];
		// Exact pairwise axial decay at the current positions. Equal/opposite,
		// central impulses preserve momentum and cannot add kinetic energy.
		double impulse =
			-std::expm1( -spring.damping * std::sqrt( internalSpringScale ) * inverseMass * dt ) *
			speed / inverseMass;
		for ( int axis = 0; axis < 2; ++axis )
		{
			v[spring.a][axis] = T( v[spring.a][axis] + impulse * d[axis] / mass[spring.a] );
			v[spring.b][axis] = T( v[spring.b][axis] - impulse * d[axis] / mass[spring.b] );
		}
	}
}

template <class T> bool Body<T>::pressureBoundaryValid() const
{
	if ( !( area() > .05 * restArea ) )
		return false;
	auto orient = []( Vec a, Vec b, Vec c ) {
		return ( double( b[0] ) - a[0] ) * ( double( c[1] ) - a[1] ) -
			   ( double( b[1] ) - a[1] ) * ( double( c[0] ) - a[0] );
	};
	auto onSegment = []( Vec a, Vec b, Vec p ) {
		return p[0] >= std::min( a[0], b[0] ) && p[0] <= std::max( a[0], b[0] ) &&
			   p[1] >= std::min( a[1], b[1] ) && p[1] <= std::max( a[1], b[1] );
	};
	for ( int i = 0; i < nodeCount; ++i )
	{
		auto a = x[i], b = x[( i + 1 ) % nodeCount];
		if ( std::hypot( double( a[0] ) - b[0], double( a[1] ) - b[1] ) <= 1e-6 )
			return false;
		for ( int j = i + 2; j < nodeCount; ++j )
		{
			if ( i == 0 && j == nodeCount - 1 )
				continue;
			auto c = x[j], d = x[( j + 1 ) % nodeCount];
			double abC = orient( a, b, c ), abD = orient( a, b, d );
			double cdA = orient( c, d, a ), cdB = orient( c, d, b );
			if ( ( abC * abD < 0 && cdA * cdB < 0 ) ||
				 ( std::abs( abC ) < 1e-12 && onSegment( a, b, c ) ) ||
				 ( std::abs( abD ) < 1e-12 && onSegment( a, b, d ) ) ||
				 ( std::abs( cdA ) < 1e-12 && onSegment( c, d, a ) ) ||
				 ( std::abs( cdB ) < 1e-12 && onSegment( c, d, b ) ) )
				return false;
		}
	}
	return true;
}

template <class T> double Body<T>::pressureForce( Vec* forces ) const
{
	std::fill_n( forces, nodeCount, Vec{} );
	if ( pressure.maxPressure == 0 || pressure.scale == 0 || pressure.level == 0 )
		return 0;
	double a = area();
	if ( !( a > .05 * restArea ) )
		return 0;
	// A fixed amount of virtual gas has p*A = level*creationArea. No vacuum
	// suction: the recoverable material supplies the restoring membrane force.
	double gasArea = referenceScale.enabled ? referenceScale.area : double( restArea );
	double applied =
		pressure.scale * std::min( pressure.maxPressure, pressure.level * gasArea / a );
	double inverseMassGradient = 0;
	for ( int i = 0; i < nodeCount; ++i )
	{
		auto prev = x[( i + nodeCount - 1 ) % nodeCount], next = x[( i + 1 ) % nodeCount];
		double gx = .5 * ( double( next[1] ) - prev[1] );
		double gy = .5 * ( double( prev[0] ) - next[0] );
		inverseMassGradient = std::max( inverseMassGradient, std::hypot( gx, gy ) / mass[i] );
	}
	if ( inverseMassGradient > 0 )
		applied = std::min( applied, pressure.maxAcceleration / inverseMassGradient );
	for ( int i = 0; i < nodeCount; ++i )
	{
		auto prev = x[( i + nodeCount - 1 ) % nodeCount], next = x[( i + 1 ) % nodeCount];
		forces[i] = { T( .5 * applied * ( double( next[1] ) - prev[1] ) ),
					  T( .5 * applied * ( double( prev[0] ) - next[0] ) ) };
	}
	return applied;
}

template <class T> void Body<T>::updatePressure( T dt )
{
	if ( pressure.maxPressure == 0 )
		return;
	pressure.level += std::max( -pressure.rate * dt,
								std::min( pressure.rate * dt, pressure.target - pressure.level ) );
	if ( !pressureBoundaryValid() )
	{
		pressure.scale = 0;
		pressure.healthySteps = 0;
	}
	else if ( pressure.healthySteps < 8 )
	{
		// Hysteresis: a nearly flat body must recover beyond 10% area before
		// collecting eight consecutive healthy steps. No position reset.
		pressure.healthySteps = area() > .1 * restArea ? pressure.healthySteps + 1 : 0;
	}
	else
		pressure.scale = std::min( 1., pressure.scale + double( dt ) * frequency );
	if ( pressure.scale < 1 )
		++pressure.recoverySteps;
	pressure.forces.resize( nodeCount );
	pressure.applied = pressureForce( pressure.forces.data() );
	double gasArea = referenceScale.enabled ? referenceScale.area : double( restArea );
	if ( pressure.scale > 0 && pressure.level > 0 &&
		 pressure.applied < pressure.scale * pressure.level * gasArea / area() )
		++pressure.limitedSteps;
}

template <class T> T Body<T>::force( Vec* f ) const
{
	std::fill_n( f, nodeCount, Vec{} );
	T energy = recoverableForce( f );
	if ( !internalSprings.empty() )
		energy += internalSpringForce( f, internalSpringScale );
	return energy;
}

template <class T> T Body<T>::kinetic() const
{
	T e = 0;
	for ( int i = 0; i < nodeCount; ++i )
		e += T( .5 ) * mass[i] * ( v[i][0] * v[i][0] + v[i][1] * v[i][1] );
	return e;
}

template <class T> void Body<T>::dampDeformation( T dt )
{
	if ( worldSupported )
	{
		// A stationary reference also absorbs rigid translation and rotation.
		double decay = std::exp( -double( damping ) * double( dt ) );
		for ( auto& velocity : v )
			for ( auto& component : velocity )
				component = T( double( component ) * decay );
		return;
	}
	double total = 0, inertia = 0, angular = 0;
	V center{}, velocity{};
	for ( int i = 0; i < nodeCount; ++i )
	{
		total += mass[i];
		for ( int k = 0; k < 2; ++k )
		{
			center[k] += double( mass[i] ) * x[i][k];
			velocity[k] += double( mass[i] ) * v[i][k];
		}
	}
	for ( int k = 0; k < 2; ++k )
	{
		center[k] /= total;
		velocity[k] /= total;
	}
	for ( int i = 0; i < nodeCount; ++i )
	{
		double rx = double( x[i][0] ) - center[0], ry = double( x[i][1] ) - center[1];
		inertia += double( mass[i] ) * ( rx * rx + ry * ry );
		angular += double( mass[i] ) * ( rx * ( double( v[i][1] ) - velocity[1] ) -
										 ry * ( double( v[i][0] ) - velocity[0] ) );
	}
	if ( !( inertia > 1e-12 ) )
	{

		// At a point there is no defined rigid angular velocity. Dampen
		// only velocity relative to translation until the shape opens.
		double decay = -std::expm1( -double( damping ) * double( dt ) );
		for ( int node = 0; node < nodeCount; ++node )
			for ( int axis = 0; axis < 2; ++axis )
				v[node][axis] = T( v[node][axis] - decay * ( v[node][axis] - velocity[axis] ) );
		return;
	}
	double omega = angular / inertia, decay = -std::expm1( -double( damping ) * double( dt ) );
	for ( int i = 0; i < nodeCount; ++i )
	{
		V rigid{ velocity[0] - omega * ( double( x[i][1] ) - center[1] ),
				 velocity[1] + omega * ( double( x[i][0] ) - center[0] ) };
		for ( int k = 0; k < 2; ++k )
			v[i][k] = T( double( v[i][k] ) - decay * ( double( v[i][k] ) - rigid[k] ) );
	}
}

template struct Body<float>;
template struct Body<double>;
} // namespace SoftBody
