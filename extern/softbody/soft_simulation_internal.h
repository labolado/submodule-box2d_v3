#pragma once
// Private implementation shared only by the native API and Box2D adapter.
#include "soft_simulation.h"
#include "soft_world.h"

namespace SoftBody
{
std::uint64_t AllocateSimulationOwner();

struct Simulation::Impl
{
	World<float> core;
	SimulationDef limits;
	std::uint64_t owner;
	mutable const char* error = nullptr;
	bool valid = false, stepping = false, coupled = false;
	explicit Impl( const SimulationDef& def );
	bool editable();
	ObjectId id( ObjectKind kind, int index, std::uint64_t generation ) const;
	bool check( ObjectId id, ObjectKind expected ) const;
};
} // namespace SoftBody
