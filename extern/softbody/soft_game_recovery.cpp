#include "soft_game_recovery.h"

#if defined(__clang__)
#pragma clang fp contract(off)
#endif

namespace SoftBody { namespace Game {
Topology inspectTopology(const Core::B& body)
{
    Topology result;
    for (int i = 0; i < 16; ++i)
        for (int axis = 0; axis < 2; ++axis)
            result.finite = result.finite && std::isfinite(body.x[i][axis]) && std::isfinite(body.v[i][axis]);
    if (!result.finite) return result;
    for (int i = 0; i < 16; ++i)
    {
        auto a = body.x[i], b = body.x[(i + 1) % 16], origin = body.x[0];
        result.shortestEdge = std::min(result.shortestEdge, std::hypot(double(b[0]) - a[0], double(b[1]) - a[1]));
        result.signedArea += .5 * ((double(a[0]) - origin[0]) * (double(b[1]) - origin[1]) -
                                  (double(a[1]) - origin[1]) * (double(b[0]) - origin[0]));
        for (int j = i + 2; j < 16; ++j)
            if (!(i == 0 && j == 15) && SoftBody::strictCross<float>(a, b, body.x[j], body.x[(j + 1) % 16]))
                result.crossing = true;
    }
    return result;
}

bool Recovery::observe(const Topology& topology, bool endOfStep)
{
    bool wasActive = active;
    if (!topology.valid())
    {
        active = true;
        healthySteps = 0;
        if (!wasActive) ++entries;
    }
    else if (endOfStep && active && ++healthySteps >= 8)
    {
        active = false;
        healthySteps = 0;
        ++exits;
    }
    if (endOfStep)
    {
        activeSteps += wasActive || active;
        crossingSteps += topology.crossing;
        degenerateSteps += topology.shortestEdge <= 1e-6 || topology.signedArea <= 0;
    }
    return active != wasActive;
}
}}
