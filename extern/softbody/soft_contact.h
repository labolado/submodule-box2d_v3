#pragma once
#include "soft_body.h"

#include <limits>
namespace SoftBody
{
#if defined( __clang__ )
#pragma clang fp contract( off )
#endif
template <class T> T cross( typename Body<T>::Vec a, typename Body<T>::Vec b );
template <class T> typename Body<T>::Vec sub( typename Body<T>::Vec a, typename Body<T>::Vec b );
template <class T> bool inside( typename Body<T>::Vec p, const Body<T>& b );
template <class T> struct Hit
{
	int edge;
	T depth, t;
	typename Body<T>::Vec normal;
	Hit( int e = -1, T d = 0, T f = 0, typename Body<T>::Vec n = {} )
		: edge( e )
		, depth( d )
		, t( f )
		, normal( n )
	{
	}
};
template <class T> Hit<T> nearest( typename Body<T>::Vec p, const Body<T>& b );
template <class T> bool overlapAabb( const Body<T>& a, const Body<T>& b );
template <class T>
bool strictCross( typename Body<T>::Vec a, typename Body<T>::Vec b, typename Body<T>::Vec c,
				  typename Body<T>::Vec d );
} // namespace SoftBody
