#pragma once
#include "soft_world.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace SoftBody
{
namespace Game
{
using Core = SoftBody::World<float>;

struct Topology
{
	bool finite = true, crossing = false;
	double shortestEdge = 1e30, signedArea = 0;

	bool valid() const
	{
		return finite && !crossing && shortestEdge > 1e-6 && signedArea > 0;
	}
};

Topology inspectTopology( const Core::B& body );

struct Recovery
{
	bool active = false;
	int healthySteps = 0;
	unsigned long long entries = 0, exits = 0, activeSteps = 0, crossingSteps = 0,
					   degenerateSteps = 0;
	double correctionDistance = 0;
	std::vector<float> remaining;
	// Explicit recovery-only position budget, shared by all boxes and sweeps
	// in a substep. It is not a velocity clamp or a changed material stiffness.
	float maxCorrection = .02f;

	bool observe( const Topology& topology, bool endOfStep = false );
};
} // namespace Game
} // namespace SoftBody
