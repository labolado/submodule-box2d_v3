// #include "Rtt_Assert.h"
#include "liquid_world.h"
#include "liquid_callbacks.h"

static b2ContactFilter b2_liquid_defaultFilter;

bool operator<( b2ShapeId a, b2ShapeId b ) {
	return a.index1 < b.index1;
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
	m_worldId = b2_nullWorldId;

	while (m_particleSystemList)
	{
		DestroyParticleSystem(m_particleSystemList);
	}

	// Even though the block allocator frees them for us, for safety,
	// we should ensure that all buffers have been freed.
	// b2Assert(m_blockAllocator.GetNumGiantAllocations() == 0);
}

void b2LiquidWorld::SetContactListener( b2ContactListener *listener ) {
	m_contactListener = listener;
}

b2ParticleSystem* b2LiquidWorld::CreateParticleSystem(const b2ParticleSystemDef* def)
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
	b2TimeStep step = { dt, invDT, 0.0f, 1, false };
	for (b2ParticleSystem* p = m_particleSystemList; p; p = p->GetNext())
	{
		p->Solve(step); // Particle Simulation
	}
}

static void LiquidFunStep( float dt, float invDT, void* context )
{
	b2LiquidWorld* world = (b2LiquidWorld*) context;
	world->OnStep( dt, invDT );
}

void b2LiquidWorld::Step( float timeStep, int subStepCount )
{
	b2World_Step( m_worldId, timeStep, subStepCount, LiquidFunStep, this );
}
