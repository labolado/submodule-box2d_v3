#ifndef B2_LIQUID_CALLBACKS_H
#define B2_LIQUID_CALLBACKS_H

#include "box2d/id.h"
#include "settings.h"

class b2ParticleSystem;
struct b2ParticleBodyContact;
struct b2ParticleContact;

/// Implement this class to provide collision filtering. In other words, you can implement
/// this class if you want finer control over contact creation.
class b2ContactFilter
{
public:
	virtual ~b2ContactFilter() {}

	/// Return true if contact calculations should be performed between these two shapes.
	/// @warning for performance reasons this is only called when the AABBs begin to overlap.
	virtual bool ShouldCollide(b2ShapeId fixtureA, b2ShapeId fixtureB);

	/// Return true if contact calculations should be performed between a
	/// fixture and particle.  This is only called if the
	/// b2_fixtureContactListenerParticle flag is set on the particle.
	virtual bool ShouldCollide(b2ShapeId fixture,
							   b2ParticleSystem* particleSystem,
							   int32 particleIndex);
	// {
	// 	B2_NOT_USED(fixture);
	// 	B2_NOT_USED(particleIndex);
	// 	B2_NOT_USED(particleSystem);
	// 	return true;
	// }

	/// Return true if contact calculations should be performed between two
	/// particles.  This is only called if the
	/// b2_particleContactListenerParticle flag is set on the particle.
	virtual bool ShouldCollide(b2ParticleSystem* particleSystem,
							   int32 particleIndexA, int32 particleIndexB)
	{
		B2_NOT_USED(particleSystem);
		B2_NOT_USED(particleIndexA);
		B2_NOT_USED(particleIndexB);
		return true;
	}
};

/// Implement this class to get contact information. You can use these results for
/// things like sounds and game logic. You can also get contact results by
/// traversing the contact lists after the time step. However, you might miss
/// some contacts because continuous physics leads to sub-stepping.
/// Additionally you may receive multiple callbacks for the same contact in a
/// single time step.
/// You should strive to make your callbacks efficient because there may be
/// many callbacks per time step.
/// @warning You cannot create/destroy Box2D entities inside these callbacks.
class b2ContactListener
{
public:
	virtual ~b2ContactListener() {}

	// /// Called when two fixtures begin to touch.
	// virtual void BeginContact(b2Contact* contact) { B2_NOT_USED(contact); }

	// /// Called when two fixtures cease to touch.
	// virtual void EndContact(b2Contact* contact) { B2_NOT_USED(contact); }

	/// Called when a fixture and particle start touching if the
	/// b2_fixtureContactFilterParticle flag is set on the particle.
	virtual void BeginParticleContact(b2ParticleSystem* particleSystem,
							  b2ParticleBodyContact* particleBodyContact)
	{
		B2_NOT_USED(particleSystem);
		B2_NOT_USED(particleBodyContact);
	}

	/// Called when a fixture and particle stop touching if the
	/// b2_fixtureContactFilterParticle flag is set on the particle.
	virtual void EndParticleContact(b2ShapeId fixture,
							b2ParticleSystem* particleSystem, int32 index)
	{
		B2_NOT_USED(fixture);
		B2_NOT_USED(particleSystem);
		B2_NOT_USED(index);
	}

	/// Called when two particles start touching if
	/// b2_particleContactFilterParticle flag is set on either particle.
	virtual void BeginParticleSystemContact(b2ParticleSystem* particleSystem,
							  b2ParticleContact* particleContact)
	{
		B2_NOT_USED(particleSystem);
		B2_NOT_USED(particleContact);
	}

	/// Called when two particles start touching if
	/// b2_particleContactFilterParticle flag is set on either particle.
	virtual void EndParticleSystemContact(b2ParticleSystem* particleSystem,
							int32 indexA, int32 indexB)
	{
		B2_NOT_USED(particleSystem);
		B2_NOT_USED(indexA);
		B2_NOT_USED(indexB);
	}
};

/// Callback class for ray casts.
/// See b2World::RayCast
class b2LiquidRayCastCallback
{
public:
	virtual ~b2LiquidRayCastCallback() {}

	/// Called for each fixture found in the query. You control how the ray cast
	/// proceeds by returning a float:
	/// return -1: ignore this fixture and continue
	/// return 0: terminate the ray cast
	/// return fraction: clip the ray to this point
	/// return 1: don't clip the ray and continue
	/// @param fixture the fixture hit by the ray
	/// @param point the point of initial intersection
	/// @param normal the normal vector at the point of intersection
	/// @return -1 to filter, 0 to terminate, fraction to clip the ray for
	/// closest hit, 1 to continue
	virtual float32 ReportFixture(	b2ShapeId fixture, const b2Vec2& point,
									const b2Vec2& normal, float32 fraction) = 0;

	/// Called for each particle found in the query. You control how the ray
	/// cast proceeds by returning a float:
	/// return <=0: ignore the remaining particles in this particle system
	/// return fraction: ignore particles that are 'fraction' percent farther
	///   along the line from 'point1' to 'point2'. Note that 'point1' and
	///   'point2' are parameters to b2World::RayCast.
	/// @param particleSystem the particle system containing the particle
	/// @param index the index of the particle in particleSystem
	/// @param point the point of intersection bt the ray and the particle
	/// @param normal the normal vector at the point of intersection
	/// @param fraction percent (0.0~1.0) from 'point0' to 'point1' along the
	///   ray. Note that 'point1' and 'point2' are parameters to
	///   b2World::RayCast.
	/// @return <=0 to ignore rest of particle system, fraction to ignore
	/// particles that are farther away.
	virtual float32 ReportParticle(const b2ParticleSystem* particleSystem,
								   int32 index, const b2Vec2& point,
								   const b2Vec2& normal, float32 fraction)
	{
		B2_NOT_USED(particleSystem);
		B2_NOT_USED(index);
		B2_NOT_USED(&point);
		B2_NOT_USED(&normal);
		B2_NOT_USED(fraction);
		return 0;
	}

	/// Cull an entire particle system from b2World::RayCast. Ignored in
	/// b2ParticleSystem::RayCast.
	/// @return true if you want to include particleSystem in the RayCast, or
	/// false to cull particleSystem from the RayCast.
	virtual bool ShouldQueryParticleSystem(
		const b2ParticleSystem* particleSystem)
	{
		B2_NOT_USED(particleSystem);
		return true;
	}
};

#endif
