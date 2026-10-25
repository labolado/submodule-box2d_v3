#pragma once
#include "soft_game_world.h"
#include "soft_pair_contacts.h"
#include "soft_profile.h"

#include <memory>

namespace SoftBody
{
namespace Game
{
// Internal bounded multi-body solver. Core owns all material state; each part
// owns only its contact/recovery cache and references a stable body slot.
// This owner alone advances the coupled rigid world.
struct Assembly
{
	static constexpr int MaxBodies = 32;
	static constexpr int MaxLinks = 64;
	struct Part
	{
		unsigned long long generation;
		Solver contact;
		Part( Core& core, int id )
			: generation( core.bodies[id].generation )
			, contact( core, id )
		{
		}
	};
	struct Link
	{
		b2Vec2 impulse{}, bias{};
		float inverseMass = 0, gamma = 0;
	};
	Core& soft;
	PairContacts pairs;
	std::vector<std::unique_ptr<Part>> parts;
	std::vector<Link> links;
	b2WorldId world = b2_nullWorldId;
	b2WorldId boundWorld = b2_nullWorldId;
	bool flipY = false, boundFlipY = false;
	bool enableDynamicContacts = false;
	bool enableSoftContacts = false;
	const char* error = nullptr;
	unsigned long long entered = 0, completed = 0;
	int nextPhase = 0;
	double peakGap = 0, peakSpeed = 0, peakDepth = 0;
	Profile profile;

	explicit Assembly( Core& core )
		: soft( core )
		, pairs( core )
	{
	}
	Assembly( const Assembly& ) = delete;
	Assembly& operator=( const Assembly& ) = delete;
	bool syncParts();
	bool prepare();
	bool advance();
	void motors();
	void prepareLinks();
	void solveLinks();
	bool refreshDynamic( Solver& contact, void* step );
	void publishDynamic( Solver& contact, void* step );
	bool pairGeometry( void* step );
	static void stage( void* step, int phase, void* context );
};
} // namespace Game
} // namespace SoftBody
