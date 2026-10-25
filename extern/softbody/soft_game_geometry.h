#pragma once
#include "soft_box2d_box.h"
#include "soft_game_recovery.h"

#include <algorithm>
#include <cmath>

namespace SoftBody
{
namespace Game
{
using Core = SoftBody::World<float>;
using View = SoftBody::Box2DBoxContacts::StateView;
// Geometry-only form of our existing C1 box projection. Keep it separate from
// the accumulated velocity constraints: moving a node must refresh subsequent
// corner/edge queries, without adding depth / dt to physical velocity.
struct BoxProjection
{
	Core::B& body;
	b2WorldTransform transform;
	float hx, hy;
	const char* error = nullptr;
	unsigned corrections = 0;
	bool pointsOnly = false, stopOnInvalid = false, invalidated = false;
	std::array<float, 16>* remaining = nullptr;
	double* correctionDistance = nullptr;
	const std::array<int, 16>* nodeFaces = nullptr;
	float faceMargin = 0;

	BoxProjection( Core::B& b, const View& view, float x, float y )
		: body( b )
		, transform{ view.origin, view.rotation }
		, hx( x )
		, hy( y )
	{
	}

	static b2Vec2 normal( int face );

	float separation( b2Vec2 p, int face ) const;

	b2Vec2 local( int node ) const;

	int retainedFace( int node ) const;

	bool clip( b2Vec2 a, b2Vec2 b, int& entry, int& exit ) const;

	void move( int edge, float fraction, b2Vec2 direction, float depth );

	void project();

	double boundaryDepth() const;
};
} // namespace Game
} // namespace SoftBody
