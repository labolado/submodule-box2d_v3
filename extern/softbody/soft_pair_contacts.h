#pragma once
// Private soft/soft constraints. No Lua, rigid-body state or Box2D types.
#include "soft_world.h"

#include <vector>

namespace SoftBody
{
class PairContacts
{
public:
	using Core = World<float>;
	using Vec = Body<float>::Vec;
	static constexpr int MaxRows = 512;
	static constexpr int MaxBodies = 32;
	explicit PairContacts( Core& core );
	void beginStep();
	bool project();
	bool gather();
	void warmStart();
	void solve();
	void finish();
	const char* error = nullptr;
	double depth = 0;
	unsigned long long rowSolves = 0, warmHits = 0, recoverySteps = 0;
	unsigned long long recoveryEntries = 0, recoveryExits = 0;
	int recoveringPairs = 0;
	double recoveryDistance = 0;
	// Optional admissible-motion query supplied by a boundary adapter.
	// One common fraction preserves bilateral mass weighting; no rigid types.
	float ( *positionLimit )( void*, int, int, Vec ) = nullptr;
	void* positionContext = nullptr;
	Vec ( *projectTranslation )( void*, int, Vec ) = nullptr;

private:
	struct Row
	{
		int a = 0, b = 0, edgeA = 0, edgeB = 0;
		unsigned long long generationA = 0, generationB = 0;
		float tA = 0, tB = 0, gap = 0, normalImpulse = 0, tangentImpulse = 0;
		Vec normal{};
		bool centres = false;
	};
	struct PairState
	{
		unsigned long long generationA = 0, generationB = 0;
		Vec normal{ { 1, 0 } }; // Last separated direction, points from B to A.
		int separatedSteps = 0;
		bool active = false;
	};
	Core& core;
	std::vector<Row> rows, previous;
	std::vector<std::array<float, 16>> remaining;
	std::vector<PairState> states;
	bool recoveryUsed = false;
	bool collect();
	bool append( Row row );
	bool broadPhase( const Body<float>& a, const Body<float>& b ) const;
	void sample( int a, int b, int edge, float fraction );
	void direction( int a, int b );
	void fallback( int a, int b, Vec normal );
	float separation( int a, int b, Vec normal ) const;
	void projectRecovery( const Row& row );
	float inverse( const Row& row ) const;
	Vec velocity( int body, int edge, float fraction, bool centre ) const;
	void apply( const Row& row, Vec impulse, bool position );
};
} // namespace SoftBody
