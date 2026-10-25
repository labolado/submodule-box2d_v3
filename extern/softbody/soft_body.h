#pragma once
#include <algorithm>
#include <array>
#include <cmath>

namespace SoftBody
{
#if defined( __clang__ )
#pragma clang fp contract( off )
#endif
using V = std::array<double, 2>;
template <class T> struct Body
{
	using Vec = std::array<T, 2>;
	std::array<Vec, 16> x{}, v{}, rest{};
	std::array<T, 16> mass{}, lengths{};
	mutable const char* failure = nullptr;
	int kind; // 0: local frames, 1: global frame, 2: pressure tire, 3: recoverable
	mutable double recoveryCosine = 1, recoverySine = 0;
	T restArea;
	T preload = T( 0 );
	T materialScale = T( 1 );
	T friction = T( 0 ); // Soft/soft Coulomb friction; boundaries have their own coefficient.
	unsigned long long generation = 1;
	bool active = true;
	Vec forceInput{};
	T torqueInput = 0;
	T stiffness, frequency, damping, frameOverlap;
	T bulk = T( 300 );
	Body( int model, T totalMass, T rx, T ry, T edgeStiffness, T hz, T dampingRate, T pressure,
		  T scale, T overlap );
	T area() const;
	// Finite corotated quadratic energy. Unlike pressure, this material is
	// defined for signed/zero area and overlapping nodes. Node identities and
	// rest coordinates never change; forces, not a rest-pose teleport, unfold it.
	T recoverableForce( std::array<Vec, 16>& forces ) const;
	T force( std::array<Vec, 16>& f ) const;
	T kinetic() const;
	// Mass-orthogonal projection onto translation and rotation at fixed positions.
	// Only the residual velocity decays; reductions use double for float state.
	void dampDeformation( T dt );
	void damp( T dt );
};
extern template struct Body<float>;
extern template struct Body<double>;
} // namespace SoftBody
