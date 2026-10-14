// #include "Rtt_Assert.h"
#include "liquid_world.h"
#include "liquid_callbacks.h"
#include "particle_group.h"
#include <cmath>
#include <mutex>
#include <set>

// Creation-only options live outside ABI-visible object layouts.
static std::mutex s_rotationCCDMutex;
static std::set<const b2ParticleSystem*> s_rotationCCDSystems;

static bool UsesRotatingBoundaryCCD( const b2ParticleSystem* system )
{
	std::lock_guard<std::mutex> lock( s_rotationCCDMutex );
	return s_rotationCCDSystems.count( system ) != 0;
}

static b2ContactFilter b2_liquid_defaultFilter;

bool operator<( b2ShapeId a, b2ShapeId b ) {
	return a.index1 < b.index1;
}

static b2WorldCastOutput CastMovingBoundary( b2ShapeId shape, b2Vec2 start, b2Vec2 end )
{
	b2Vec2 translation = end - start;
	b2WorldCastOutput hit = b2Shape_RayCast( shape, start, translation );
	hit.point = start + hit.fraction * translation;
	if ( hit.hit && b2LengthSquared( hit.normal ) > 0.0f )
	{
		return hit;
	}
	// Rounded polygon casts may report an initial overlap without a normal,
	// or miss a short segment within the cast tolerance. Advance using signed
	// distance for closed shapes; retain the existing one-sided chain casts.
	b2ShapeType type = b2Shape_GetType( shape );
	if ( type != b2_polygonShape && type != b2_circleShape && type != b2_capsuleShape )
	{
		return hit;
	}
	hit.hit = false;
	float length = b2Length( translation );
	if ( length == 0.0f )
	{
		return hit;
	}
	float fraction = 0.0f;
	const float tolerance = 0.5f * b2_linearSlop;
	for ( int iteration = 0; iteration < 20; ++iteration )
	{
		b2Vec2 point = start + fraction * translation;
		float distance;
		b2Vec2 normal;
		b2Shape_ComputeDistance( shape, point, &distance, &normal );
		if ( distance <= tolerance + 1e-6f )
		{
			if ( b2Dot( translation, normal ) < 0.0f )
			{
				hit.hit = true;
				hit.fraction = fraction;
				hit.normal = normal;
				hit.point = point + b2MaxFloat( 0.0f, -distance ) * normal;
			}
			break;
		}
		fraction += ( distance - tolerance ) / length;
		if ( fraction > 1.0f )
		{
			break;
		}
	}
	return hit;
}

b2LiquidWorld::b2LiquidWorld( b2WorldId worldId )
{
	m_worldId = worldId;
	m_particleSystemList = NULL;

	m_contactFilter = &b2_liquid_defaultFilter;

	m_liquidFunVersion = &b2_liquidFunVersion;
	m_liquidFunVersionString = b2_liquidFunVersionString;
}

b2LiquidWorld::~b2LiquidWorld()
{
	// DestroyParticleSystem checks the underlying world lock. Its ID must
	// remain valid until all particle systems have been released.
	while (m_particleSystemList)
	{
		DestroyParticleSystem(m_particleSystemList);
	}
	m_worldId = b2_nullWorldId;

	// Even though the block allocator frees them for us, for safety,
	// we should ensure that all buffers have been freed.
	// b2Assert(m_blockAllocator.GetNumGiantAllocations() == 0);
}

void b2LiquidWorld::SetContactListener( b2ContactListener *listener ) {
	m_contactListener = listener;
}

b2ParticleSystem* b2LiquidWorld::CreateParticleSystem(const b2ParticleSystemDef* def)
{
	return CreateParticleSystem( def, false );
}

b2ParticleSystem* b2LiquidWorld::CreateParticleSystem(const b2ParticleSystemDef* def, bool enableRotatingBoundaryCCD)
{
	b2Assert(IsLocked() == false);
	if (IsLocked())
	{
		return NULL;
	}

	void* mem = m_blockAllocator.Allocate(sizeof(b2ParticleSystem));
	b2ParticleSystem* p = new (mem) b2ParticleSystem(def, this);

	// Add to world doubly linked list.
	p->m_prev = NULL;
	p->m_next = m_particleSystemList;
	if (m_particleSystemList)
	{
		m_particleSystemList->m_prev = p;
	}
	m_particleSystemList = p;
	if ( enableRotatingBoundaryCCD )
	{
		std::lock_guard<std::mutex> lock( s_rotationCCDMutex );
		s_rotationCCDSystems.insert( p );
	}

	return p;
}


void b2LiquidWorld::DestroyParticleSystem(b2ParticleSystem* p)
{
	b2Assert(m_particleSystemList != NULL);
	b2Assert(IsLocked() == false);
	if (IsLocked())
	{
		return;
	}

	// Remove world particleSystem list.
	if (p->m_prev)
	{
		p->m_prev->m_next = p->m_next;
	}

	if (p->m_next)
	{
		p->m_next->m_prev = p->m_prev;
	}

	if (p == m_particleSystemList)
	{
		m_particleSystemList = p->m_next;
	}

	{
		std::lock_guard<std::mutex> lock( s_rotationCCDMutex );
		s_rotationCCDSystems.erase( p );
	}
	p->~b2ParticleSystem();
	m_blockAllocator.Free(p, sizeof(b2ParticleSystem));
}

bool b2LiquidWorld::IsLocked() const
{
	return b2World_IsLocked( m_worldId );
}

void b2LiquidWorld::QueryAABB( b2AABB aabb, b2QueryFilter filter, b2OverlapResultFcn* fcn, void* context )
{
	b2World_OverlapAABBForLiquidFun( m_worldId, aabb, filter, fcn, context );
}

void b2LiquidWorld::OnStep( float dt, float invDT )
{
	OnStep( dt, invDT, NULL );
}

void b2LiquidWorld::OnStep( float dt, float invDT, b2Vec2** startPositions )
{
	b2TimeStep step = { dt, invDT, 0.0f, 1, false };
	int index = 0;
	for (b2ParticleSystem* p = m_particleSystemList; p; p = p->GetNext(), ++index)
	{
		p->Solve(step, startPositions ? startPositions[index] : NULL); // Particle Simulation
	}
}

struct LiquidStepContext
{
	b2LiquidWorld* world;
	b2Vec2** startPositions;
};

static void LiquidFunStep( float dt, float invDT, void* context )
{
	LiquidStepContext* step = (LiquidStepContext*) context;
	step->world->OnStep( dt, invDT, step->startPositions );
}

void b2LiquidWorld::Step( float timeStep, int subStepCount )
{
	if ( IsLocked() )
	{
		return;
	}
	int count = 0;
	int particleCount = 0;
	for ( b2ParticleSystem* p = m_particleSystemList; p; p = p->GetNext() )
	{
		++count;
		if ( UsesRotatingBoundaryCCD( p ) && !p->GetPaused() )
		{
			particleCount += p->GetParticleCount();
		}
	}
	b2Vec2* buffer = NULL;
	b2Vec2** positions = NULL;
	if ( particleCount > 0 && timeStep > 0.0f )
	{
		positions = (b2Vec2**)m_stackAllocator.Allocate( count * sizeof( b2Vec2* ) );
		buffer = (b2Vec2*)m_stackAllocator.Allocate( particleCount * sizeof( b2Vec2 ) );
		b2Vec2* next = buffer;
		int i = 0;
		for ( b2ParticleSystem* p = m_particleSystemList; p; p = p->GetNext(), ++i )
		{
			positions[i] = NULL;
			if ( UsesRotatingBoundaryCCD( p ) && !p->GetPaused() && p->GetParticleCount() > 0 )
			{
				positions[i] = next;
				next += p->GetParticleCount();
			}
		}
	}
	LiquidStepContext context = { this, positions };
	b2World_Step( m_worldId, timeStep, subStepCount, LiquidFunStep, &context );
	if ( positions )
	{
		PostStep( positions );
	}
	if ( positions )
	{
		m_stackAllocator.Free( buffer );
		m_stackAllocator.Free( positions );
	}
}

void b2LiquidWorld::PostStep( b2Vec2* const* startPositions )
{
	// The rigid solver has now advanced. Sweep its actual rotation against the
	// particle paths from OnStep, rather than waiting for next frame's contacts.
	if ( m_particleSystemList == NULL )
	{
		return;
	}
	b2BodyEvents events = b2World_GetBodyEvents( m_worldId );
	for ( int e = 0; e < events.moveCount; ++e )
	{
		b2BodyId body = events.moveEvents[e].bodyId;
		b2Transform previous = b2Body_GetPreviousTransform( body );
		b2Transform current = b2Body_GetTransform( body );
		float angle = std::atan2( previous.q.c * current.q.s - previous.q.s * current.q.c,
			previous.q.c * current.q.c + previous.q.s * current.q.s );
		if ( std::fabs( angle ) < 1e-6f )
		{
			continue;
		}
		int capacity = b2Body_GetShapeCount( body );
		if ( capacity == 0 )
		{
			continue;
		}
		b2ShapeId* shapes = (b2ShapeId*)m_stackAllocator.Allocate( capacity * sizeof( b2ShapeId ) );
		int count = b2Body_GetShapes( body, shapes, capacity );
		b2AABB* bounds = (b2AABB*)m_stackAllocator.Allocate( count * sizeof( b2AABB ) );
		int solidCount = 0;
		float reach = 0.0f;
		b2QueryFilter queryFilter = b2DefaultQueryFilter();
		for ( int j = 0; j < count; ++j )
		{
			b2ShapeId shape = shapes[j];
			b2Filter filter = b2Shape_GetFilter( shape );
			if ( b2Shape_IsSensor( shape ) || ( filter.categoryBits & queryFilter.maskBits ) == 0 ||
				( filter.maskBits & queryFilter.categoryBits ) == 0 )
			{
				continue;
			}
			b2AABB aabb = b2Shape_GetAABB( shape );
			b2Vec2 lo = aabb.lowerBound - current.p;
			b2Vec2 hi = aabb.upperBound - current.p;
			reach = b2MaxFloat( reach, b2Length( { b2MaxFloat( std::fabs( lo.x ), std::fabs( hi.x ) ),
				b2MaxFloat( std::fabs( lo.y ), std::fabs( hi.y ) ) } ) );
			shapes[solidCount] = shape;
			bounds[solidCount++] = aabb;
		}
		b2Vec2 extent = { reach, reach };
		b2AABB swept = { b2Min( previous.p, current.p ) - extent, b2Max( previous.p, current.p ) + extent };
		int segments = b2MinInt( 64, b2MaxInt( 1, (int)std::ceil( std::fabs( angle ) / 0.025f ) ) );
		float invMass = 0.0f, invInertia = 0.0f;
		if ( b2Body_GetType( body ) == b2_dynamicBody )
		{
			float mass = b2Body_GetMass( body );
			float inertia = b2Body_GetRotationalInertia( body );
			invMass = mass > 0.0f ? 1.0f / mass : 0.0f;
			invInertia = inertia > 0.0f ? 1.0f / inertia : 0.0f;
		}
		int systemIndex = 0;
		for ( b2ParticleSystem* system = m_particleSystemList; solidCount > 0 && system; system = system->GetNext(), ++systemIndex )
		{
			if ( system->GetPaused() ||
				( startPositions ? startPositions[systemIndex] == NULL : !UsesRotatingBoundaryCCD( system ) ) )
			{
				continue;
			}
			b2Vec2* positions = system->GetPositionBuffer();
			b2Vec2* velocities = system->GetVelocityBuffer();
			const uint32* flags = system->GetFlagsBuffer();
			b2ParticleGroup* const* groups = system->GetGroupBuffer();
			float particleInvMass = system->GetParticleInvMass();
			bool changed = false;
			for ( int i = 0; i < system->GetParticleCount(); ++i )
			{
				// Keep fixed/rigid particles and user callback timing on their existing path.
				if ( flags[i] & ( b2_zombieParticle | b2_wallParticle | b2_fixtureContactFilterParticle ) ||
					( groups[i] && ( groups[i]->GetGroupFlags() & b2_rigidParticleGroup ) ) )
				{
					continue;
				}
				b2Vec2 target = positions[i];
				b2Vec2 p = startPositions ? startPositions[systemIndex][i] : target;
				b2AABB travel = { b2Min( p, target ), b2Max( p, target ) };
				if ( !b2AABB_Overlaps( travel, swept ) )
				{
					continue;
				}
				b2Vec2 displacement = ( 1.0f / segments ) * ( target - p );
				bool hitParticle = false;
				b2Transform from = previous;
				for ( int k = 1; k <= segments; ++k )
				{
					float t = (float)k / segments;
					b2Transform to = { previous.p + t * ( current.p - previous.p ),
						b2MulRot( previous.q, b2MakeRot( t * angle ) ) };
					b2Vec2 start = b2TransformPoint( current, b2InvTransformPoint( from, p ) );
					b2Vec2 end = b2TransformPoint( current, b2InvTransformPoint( to, p + displacement ) );
					bool hitSegment = false;
					// Resolve up to four successive surfaces (including a box corner).
					for ( int contact = 0; contact < 4; ++contact )
					{
						b2AABB path = { b2Min( start, end ), b2Max( start, end ) };
						b2WorldCastOutput closest = {};
						closest.fraction = 1.0f;
						for ( int j = 0; j < solidCount; ++j )
						{
							if ( !b2AABB_Overlaps( path, bounds[j] ) )
							{
								continue;
							}
							b2WorldCastOutput hit = CastMovingBoundary( shapes[j], start, end );
							if ( hit.hit && b2LengthSquared( hit.normal ) > 0.0f && hit.fraction <= closest.fraction )
							{
								closest = hit;
							}
						}
						if ( !closest.hit )
						{
							break;
						}
						hitSegment = true;
						hitParticle = true;
						changed = true;
						b2Vec2 hit = closest.point;
						b2Vec2 normal = closest.normal;
						float inward = b2Dot( end - hit, normal );
						end += ( 2.0f * b2_linearSlop - b2MinFloat( 0.0f, inward ) ) * normal;
						start = hit + 2.0f * b2_linearSlop * normal;
						b2Vec2 worldNormal = b2RotateVector( to.q, b2InvRotateVector( current.q, normal ) );
						b2Vec2 point = b2TransformPoint( to, b2InvTransformPoint( current, hit ) );
						b2Vec2 bodyVelocity = b2Body_GetWorldPointVelocity( body, point );
						float vn = b2Dot( velocities[i] - bodyVelocity, worldNormal );
						if ( vn < 0.0f )
						{
							b2Vec2 center = b2TransformPoint( current, b2Body_GetLocalCenter( body ) );
							float rn = b2Cross( point - center, worldNormal );
							float impulse = -vn / ( particleInvMass + invMass + invInertia * rn * rn );
							velocities[i] += particleInvMass * impulse * worldNormal;
							if ( invMass > 0.0f )
							{
								b2Body_ApplyLinearImpulse( body, -impulse * worldNormal, point, true );
							}
						}
					}
					if ( hitSegment )
					{
						p = b2TransformPoint( to, b2InvTransformPoint( current, end ) );
					}
					else
					{
						p += displacement;
					}
					from = to;
				}
				if ( hitParticle )
				{
					positions[i] = p;
				}
			}
			if ( changed )
			{
				system->UpdatePositionProxies();
			}
		}
		m_stackAllocator.Free( bounds );
		m_stackAllocator.Free( shapes );
	}
}
