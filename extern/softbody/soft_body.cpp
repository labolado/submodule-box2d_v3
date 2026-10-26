#include "soft_body.h"

namespace SoftBody
{
#if defined( __clang__ )
#pragma clang fp contract( off )
#endif
template <class T>
Body<T>::Body( int model, T totalMass, T rx, T ry, T edgeStiffness, T hz, T dampingRate, T pressure,
			   T scale, T overlap )
	: kind( model )
	, stiffness( edgeStiffness )
	, frequency( hz )
	, damping( dampingRate )
	, frameOverlap( overlap )
{
	resizeNodes( nodeCount );
	const T pi = T( 3.14159265358979323846 );
	materialScale = scale;
	preload = pressure;
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
	if ( kind == 2 && preload != T( 0 ) )
	{
		// Initial pressure-balanced pretension currently requires a circle.
		T tension = preload * rx * std::cos( pi / T( nodeCount ) );
		for ( auto& l : lengths )
			l -= tension / stiffness;
	}
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
	weightScratch.resize( count );
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

template <class T> T Body<T>::force( Vec* f ) const
{
	std::fill_n( f, nodeCount, Vec{} );
	T energy = 0;
	if ( kind == 3 )
		return recoverableForce( f );
	for ( int i = 0; i < nodeCount; ++i )
	{
		int j = ( i + 1 ) % nodeCount;
		Vec d{ x[j][0] - x[i][0], x[j][1] - x[i][1] };
		T l = std::hypot( d[0], d[1] );
		if ( l < T( 1e-8 ) )
		{
			failure = "collapsed_edge";
			return T( 0 );
		}
		T extension = l - lengths[i];
		energy += ( stiffness / T( 2 ) ) * extension * extension;
		for ( int k = 0; k < 2; ++k )
		{
			T q = stiffness * extension * d[k] / l;
			f[i][k] += q;
			f[j][k] -= q;
		}
	}
	if ( kind == 2 )
	{
		T a = area();
		if ( !( a > T( .2 ) * restArea ) )
		{
			failure = "pressure_area_domain";
			return T( 0 );
		}
		energy += preload == T( 0 ) ? bulk * ( a - restArea - restArea * std::log( a / restArea ) )
									: bulk * ( a - restArea ) -
										  ( bulk + preload ) * restArea * std::log( a / restArea );
		T pressure = preload == T( 0 ) ? bulk * ( restArea / a - T( 1 ) )
									   : ( bulk + preload ) * restArea / a - bulk;
		for ( int i = 0; i < nodeCount; ++i )
		{
			auto prev = x[( i + nodeCount - 1 ) % nodeCount], next = x[( i + 1 ) % nodeCount];
			f[i][0] += pressure * ( next[1] - prev[1] ) / T( 2 );
			f[i][1] += pressure * ( prev[0] - next[0] ) / T( 2 );
		}
		return energy;
	}
	// Small frame reductions use double even for float state; no force balancing patch.
	const double omega = 2 * 3.14159265358979323846 * double( frequency );
	for ( int frame = 0; frame < ( kind == 0 ? 2 : 1 ); ++frame )
	{
		auto& weight = weightScratch;
		std::array<double, 2> c{}, r{};
		double total = 0;
		for ( int i = 0; i < nodeCount; ++i )
		{
			int membership = ( rest[i][0] <= frameOverlap ) + ( rest[i][0] >= -frameOverlap );
			bool included = kind == 1 || ( frame == 0 ? rest[i][0] <= frameOverlap
													  : rest[i][0] >= -frameOverlap );
			weight[i] =
				included ? double( mass[i] ) / double( kind == 1 ? 1 : membership ) : T( 0 );
			total += weight[i];
			for ( int k = 0; k < 2; ++k )
			{
				c[k] += weight[i] * x[i][k];
				r[k] += weight[i] * rest[i][k];
			}
		}
		for ( int k = 0; k < 2; ++k )
		{
			c[k] /= total;
			r[k] /= total;
		}
		double dot = 0, cross = 0;
		for ( int i = 0; i < nodeCount; ++i )
		{
			std::array<double, 2> a{ double( rest[i][0] ) - r[0], double( rest[i][1] ) - r[1] },
				b{ double( x[i][0] ) - c[0], double( x[i][1] ) - c[1] };
			dot += weight[i] * ( a[0] * b[0] + a[1] * b[1] );
			cross += weight[i] * ( a[0] * b[1] - a[1] * b[0] );
		}
		double norm = std::hypot( dot, cross );
		if ( norm < T( 1e-10 ) )
		{
			failure = "ambiguous_frame";
			return T( 0 );
		}
		double co = dot / norm, si = cross / norm;
		for ( int i = 0; i < nodeCount; ++i )
		{
			std::array<double, 2> a{ double( rest[i][0] ) - r[0], double( rest[i][1] ) - r[1] };
			std::array<double, 2> d{ x[i][0] - c[0] - co * a[0] + si * a[1],
									 x[i][1] - c[1] - si * a[0] - co * a[1] };
			for ( int k = 0; k < 2; ++k )
			{
				f[i][k] -= T( omega * omega * weight[i] * d[k] );
				energy += T( .5 * omega * omega * weight[i] * d[k] * d[k] );
			}
		}
	}
	if ( materialScale != T( 1 ) )
	{
		for ( int node = 0; node < nodeCount; ++node )
			for ( T& component : f[node] )
				component *= materialScale;
		energy *= materialScale;
	}
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
		if ( kind != 3 )
		{
			failure = "degenerate_damping_inertia";
			return;
		}
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

template <class T> void Body<T>::damp( T dt )
{
	T decay = -std::expm1( -damping * dt );
	for ( int i = 0; i < nodeCount; ++i )
	{
		int j = ( i + 1 ) % nodeCount;
		Vec d{ x[j][0] - x[i][0], x[j][1] - x[i][1] };
		T l = std::hypot( d[0], d[1] );
		if ( l < T( 1e-8 ) )
		{
			failure = "collapsed_edge";
			return;
		}
		for ( T& q : d )
			q /= l;
		T speed = ( v[j][0] - v[i][0] ) * d[0] + ( v[j][1] - v[i][1] ) * d[1];
		T impulse = decay * speed / ( T( 1 ) / mass[i] + T( 1 ) / mass[j] );
		for ( int k = 0; k < 2; ++k )
		{
			v[i][k] += impulse * d[k] / mass[i];
			v[j][k] -= impulse * d[k] / mass[j];
		}
	}
}

template struct Body<float>;
template struct Body<double>;
} // namespace SoftBody
