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
	struct Boundary
	{
		// Body IDs are stable and never reused. Empty means all active bodies.
		std::vector<int> excludedBodies;
		// Plane: dot(normal,p)>=offset. Corner: solid x>left && y<top.
		bool active = true, enabled = true;
		int kind = 0;
		Vec center{}, axis{ T( 1 ), T( 0 ) }, halfSize{}; // Static oriented box (kind 2).
		Vec normal{ T( 0 ), T( 1 ) };
		T offset = 0, target = 0, left = 0, top = 0, friction = T( .8 );
	};
	struct Metrics
	{
		double penetration = 0, pin = 0, nodeSpeed = 0, centerSpeed = 0, minArea = 1, energy = 0;
		bool selfCross = false, strictSelfCross = false;
	};
	std::vector<B> bodies;
	std::vector<Pin> pins;
	std::vector<TouchJoint> touchJoints;
	std::vector<Boundary> boundaries;
	// IDs never reused. Explicit collision exclusions, no hidden car part rules.
	std::vector<std::array<int, 2>> exclusions;
	Vec gravity{ T( 0 ), T( -20 ) };
	T dt = T( 1 ) / T( 480 );
	int iterations = 8, substeps = 8;
	int maxSoftContactRows = 512;
	mutable const char* failure = nullptr;
	double motorWork = 0, boundaryWork = 0, appliedWork = 0, frictionLoss = 0;
	V external{};
	int contacts = 0, strictEvents = 0, cornerContacts = 0;
	Metrics peak;
	std::vector<T> minimumY;
	double maxUnforcedEnergy = 0;
	unsigned long long completedSubsteps = 0;
	bool pairEnabled( int a, int b ) const;
	bool boundaryEnabled( int body, const Boundary& boundary ) const;
	Vec anchor( int id, const T* weights ) const;
	template <class Buffer> Vec anchor( int id, const Buffer& weights ) const
	{
		return anchor( id, weights.data() );
	}
	Vec pinTarget( const Pin& pin ) const;
	Vec boxLocal( const Boundary& box, Vec point ) const;
	Vec boxVertex( const Boundary& box, int corner ) const;
	Vec center( int id ) const;
	double kinetic( int id ) const;
	double omega( int id ) const;
	void torque( int id, double value );
	void boundary( int id, int node, Vec normal, T depth, T velocity, T friction );
	void corner( int id, const Boundary& boundary );
	void boundaryVertex( int id, Vec point, T friction );
	void solveTouchJoint( TouchJoint& joint );
	// Optional external constraints run synchronously inside each soft iteration.
	// No callback is stored; context must remain valid for this step only.
	using ContactProjector = void ( * )( World&, const void* );
	void advanceMaterialVelocity(); // Experimental no-contact phase split.
	void solveSubstep( const std::vector<T>& offsets, const std::vector<T>& velocities );
	void solveSubstep( const std::vector<T>& offsets, const std::vector<T>& velocities,
					   ContactProjector project, const void* context );
	Metrics metrics( const std::vector<T>& offsets ) const;
	Metrics currentMetrics() const;
	void step();
	using ContactMeasure = double ( * )( const World&, const void* );
	void step( ContactProjector project, const void* context, ContactMeasure measure = nullptr );
	void removeBody( int id );
};
extern template struct World<float>;
extern template struct World<double>;
} // namespace SoftBody
