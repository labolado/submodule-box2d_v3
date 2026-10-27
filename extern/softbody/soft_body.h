#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace SoftBody
{
#if defined( __clang__ )
#pragma clang fp contract( off )
#endif
using V = std::array<double, 2>;
template <class T> struct Body
{
	using Vec = std::array<T, 2>;
	std::vector<Vec> x, v, rest;
	std::vector<T> mass, lengths;
	struct InternalSpring
	{
		int a, b;
		double stiffness, damping, restLength, epsilon;
	};
	std::vector<InternalSpring> internalSprings;
	double internalSpringScale = 1, internalSpringWork = 0;
	int internalSpringHealthySteps = 8;
	unsigned long long internalSpringRecoverySteps = 0;
	struct Pressure
	{
		double maxPressure = 0; // Zero disables the optional recoverable-body module.
		double target = 0, level = 0, rate = 500, maxAcceleration = 200;
		double scale = 1, applied = 0, work = 0;
		int healthySteps = 8;
		unsigned long long recoverySteps = 0, limitedSteps = 0;
		std::vector<Vec> forces; // Allocated only for an enabled pressure module.
	} pressure;
	int nodeCount = 16;
	// Solver nodes follow the normalized boundary; public node identities do not.
	std::vector<int> sourceNodes;
	// Sized with the body. Reused by material, diagnostics and box projection.
	mutable std::vector<Vec> forceScratch, positionScratch;
	mutable std::vector<double> weightScratch;
	bool concaveReference = false;
	mutable const char* failure = nullptr;
	int kind; // 0: local frames, 1: global frame, 2: pressure tire, 3: recoverable
	mutable double recoveryCosine = 1, recoverySine = 0;
	T restArea;
	T preload = T( 0 );
	T materialScale = T( 1 );
	T friction = T( 0 ); // Soft/soft Coulomb friction; boundaries have their own coefficient.
	unsigned long long generation = 1;
	bool active = true;
	bool worldSupported = false;
	Vec referencePosition{};
	Vec forceInput{};
	T torqueInput = 0;
	T stiffness, frequency, damping, frameOverlap;
	T bulk = T( 300 );
	Body( int model, T totalMass, T rx, T ry, T edgeStiffness, T hz, T dampingRate, T pressure,
		  T scale, T overlap );
	T area() const;
	void resizeNodes( int count );
	// Finite corotated quadratic energy. Unlike pressure, this material is
	// defined for signed/zero area and overlapping nodes. Node identities and
	// rest coordinates never change; forces, not a rest-pose teleport, unfold it.
	T recoverableForce( Vec* forces ) const;
	T supportedForce( Vec* forces ) const;
	T internalSpringForce( Vec* forces, double scale = 1 ) const;
	void updateInternalSpringRecovery( T dt );
	void dampInternalSprings( T dt );
	bool pressureBoundaryValid() const;
	// Uniform gauge pressure times the signed-area gradient. The returned
	// pressure includes the global acceleration cap, preserving net force/torque.
	double pressureForce( Vec* forces ) const;
	void updatePressure( T dt ); // Updates the bounded force; never moves nodes.
	T force( Vec* forces ) const;
	// Keep fixed-size native research fixtures source-compatible. The caller's
	// buffer must contain at least nodeCount entries.
	template <class Buffer> T force( Buffer& forces ) const
	{
		return force( forces.data() );
	}
	T kinetic() const;
	// Free bodies damp only the residual outside translation/rotation modes.
	// World-supported bodies damp relative to their stationary reference.
	// Reductions use double for float state.
	void dampDeformation( T dt );
	void damp( T dt );
};
extern template struct Body<float>;
extern template struct Body<double>;
} // namespace SoftBody
