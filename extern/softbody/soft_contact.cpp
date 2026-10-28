#include "soft_contact.h"
namespace SoftBody
{
#if defined( __clang__ )
#pragma clang fp contract( off )
#endif
template <class T> T cross( typename Body<T>::Vec a, typename Body<T>::Vec b )
{
	return a[0] * b[1] - a[1] * b[0];
}

template <class T> typename Body<T>::Vec sub( typename Body<T>::Vec a, typename Body<T>::Vec b )
{
	return { a[0] - b[0], a[1] - b[1] };
}

template <class T> bool inside( typename Body<T>::Vec p, const Body<T>& b )
{
	bool result = false;
	for ( int i = 0, j = b.nodeCount - 1; i < b.nodeCount; j = i++ )
	{
		auto a = b.x[i], c = b.x[j];
		if ( ( a[1] > p[1] ) != ( c[1] > p[1] ) &&
			 double( p[0] ) <
				 ( double( c[0] ) - a[0] ) * ( double( p[1] ) - a[1] ) / ( double( c[1] ) - a[1] ) +
					 a[0] )
			result = !result;
	}
	return result;
}

template <class T> Hit<T> nearest( typename Body<T>::Vec p, const Body<T>& b )
{
	Hit<T> hit;
	T best = std::numeric_limits<T>::max();
	for ( int e = 0; e < b.nodeCount; ++e )
	{
		auto a = b.x[e], d = sub<T>( b.x[( e + 1 ) % b.nodeCount], a ), pa = sub<T>( p, a );
		T length2 = d[0] * d[0] + d[1] * d[1];
		if ( !( length2 > T( 1e-12 ) ) )
			return Hit<T>();
		T t = std::max( T( 0 ), std::min( T( 1 ), ( pa[0] * d[0] + pa[1] * d[1] ) / length2 ) );
		typename Body<T>::Vec delta{ a[0] + t * d[0] - p[0], a[1] + t * d[1] - p[1] };
		T distance = std::hypot( delta[0], delta[1] );
		if ( distance < best )
		{
			best = distance;
			hit = { e, distance, t, { d[1] / std::sqrt( length2 ), -d[0] / std::sqrt( length2 ) } };
			if ( distance > T( 1e-8 ) )
				for ( int k = 0; k < 2; ++k )
					hit.normal[k] = delta[k] / distance;
		}
	}
	return hit;
}

template <class T> bool overlapAabb( const Body<T>& a, const Body<T>& b )
{
	for ( int k = 0; k < 2; ++k )
	{
		T al = a.x[0][k], ah = al, bl = b.x[0][k], bh = bl;
		for ( int i = 1; i < a.nodeCount; ++i )
		{
			al = std::min( al, a.x[i][k] );
			ah = std::max( ah, a.x[i][k] );
		}
		for ( int i = 1; i < b.nodeCount; ++i )
		{
			bl = std::min( bl, b.x[i][k] );
			bh = std::max( bh, b.x[i][k] );
		}
		if ( ah < bl || bh < al )
			return false;
	}
	return true;
}

template <class T>
bool strictCross( typename Body<T>::Vec a, typename Body<T>::Vec b, typename Body<T>::Vec c,
				  typename Body<T>::Vec d )
{
	auto orient = []( typename Body<T>::Vec p, typename Body<T>::Vec q, typename Body<T>::Vec r ) {
		return ( double( q[0] ) - p[0] ) * ( double( r[1] ) - p[1] ) -
			   ( double( q[1] ) - p[1] ) * ( double( r[0] ) - p[0] );
	};
	return orient( a, b, c ) * orient( a, b, d ) < -1e-16 &&
		   orient( c, d, a ) * orient( c, d, b ) < -1e-16;
}

template float cross<float>( typename Body<float>::Vec a, typename Body<float>::Vec b );
template double cross<double>( typename Body<double>::Vec a, typename Body<double>::Vec b );
template typename Body<float>::Vec sub<float>( typename Body<float>::Vec a,
											   typename Body<float>::Vec b );
template typename Body<double>::Vec sub<double>( typename Body<double>::Vec a,
												 typename Body<double>::Vec b );
template bool inside<float>( typename Body<float>::Vec p, const Body<float>& b );
template bool inside<double>( typename Body<double>::Vec p, const Body<double>& b );
template Hit<float> nearest<float>( typename Body<float>::Vec p, const Body<float>& b );
template Hit<double> nearest<double>( typename Body<double>::Vec p, const Body<double>& b );
template bool overlapAabb<float>( const Body<float>& a, const Body<float>& b );
template bool overlapAabb<double>( const Body<double>& a, const Body<double>& b );
template bool strictCross<float>( typename Body<float>::Vec a, typename Body<float>::Vec b,
								  typename Body<float>::Vec c, typename Body<float>::Vec d );
template bool strictCross<double>( typename Body<double>::Vec a, typename Body<double>::Vec b,
								   typename Body<double>::Vec c, typename Body<double>::Vec d );
} // namespace SoftBody
