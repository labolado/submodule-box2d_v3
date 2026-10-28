#pragma once
// Private experimental soft-body solver. No Box2D or Lua dependency.
#include "soft_contact.h"

#include <vector>

namespace SoftBody
{
#if defined( __clang__ )
#pragma clang fp contract( off )
#endif
template <class T> struct World
{
	using B = Body<T>;
	using Vec = typename B::Vec;
	struct Pin
	{
		int a, b;
		std::vector<T> wa = std::vector<T>( 16 ), wb = std::vector<T>( 16 );
		Vec worldAnchor{}; // b == -1 denotes a fixed world endpoint.
		bool active = true;
		T speed = 0, maxTorque = 0, gain = 0;
		T frequency = T( 12 ), dampingRatio = T( 1 );
		unsigned long long generation = 1;
	};
	struct TouchJoint
	{
		int body;
		std::vector<T> weights = std::vector<T>( 16 );
		Vec target{};
		T frequency = T( 3 );
		T dampingRatio = T( 1 );
		T maxForce = T( 50 );
		T lastForce = 0;
		unsigned long long generation = 1;
		bool active = true;
	};
	struct Metrics
	{
		double penetration = 0, pin = 0, nodeSpeed = 0, centerSpeed = 0, minArea = 1, energy = 0;
		bool selfCross = false;
	};
	std::vector<B> bodies;
	std::vector<Pin> pins;
	std::vector<TouchJoint> touchJoints;
	// IDs never reused. Explicit collision exclusions, no hidden car part rules.
	std::vector<std::array<int, 2>> exclusions;
	Vec gravity{ T( 0 ), T( -20 ) };
	T dt = T( 1 ) / T( 480 );
	int iterations = 8, substeps = 8;
	int maxSoftContactRows = 512;
	mutable const char* failure = nullptr;
	double motorWork = 0, appliedWork = 0;
	V external{}; // Accumulated touch impulse.
	Metrics peak;
	unsigned long long completedSubsteps = 0;
	bool pairEnabled( int a, int b ) const;
	Vec anchor( int id, const T* weights ) const;
	template <class Buffer> Vec anchor( int id, const Buffer& weights ) const
	{
		return anchor( id, weights.data() );
	}
	Vec pinTarget( const Pin& pin ) const;
	Vec center( int id ) const;
	double kinetic( int id ) const;
	double omega( int id ) const;
	void torque( int id, double value );
	void solveTouchJoint( TouchJoint& joint );
	void advanceMaterialVelocity();
	Metrics currentMetrics() const;
	void removeBody( int id );
};
extern template struct World<float>;
extern template struct World<double>;
} // namespace SoftBody
