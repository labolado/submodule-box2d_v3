#include "soft_box2d_simulation.h"

#include "soft_game_assembly.h"
#include "soft_simulation_internal.h"

#include <new>

namespace SoftBody
{
namespace
{
bool same( ObjectId a, ObjectId b )
{
	return a.owner == b.owner && a.index == b.index && a.generation == b.generation &&
		   a.kind == b.kind;
}
} // namespace
struct Box2DCoupling::Impl
{
	struct Attachment
	{
		b2BodyId body = b2_nullBodyId;
		b2ShapeId shape = b2_nullShapeId;
		bool active = true;
		std::uint64_t generation = 1;
		std::vector<ObjectId> excluded;
	};
	struct Filter
	{
		ObjectId body;
		RigidFilter value;
	};
	Simulation& simulation;
	Simulation::Impl& state;
	Box2DCouplingDef def;
	const std::uint64_t owner = AllocateSimulationOwner();
	Game::Assembly solver;
	std::vector<Attachment> attachments;
	std::array<Filter, 32> filters{};
	const char* error = nullptr;
	bool valid = false;
	Impl( Simulation& simulation, const Box2DCouplingDef& def )
		: simulation( simulation )
		, state( *simulation.impl_ )
		, def( def )
		, solver( state.core )
	{
		if ( !state.valid || state.coupled || !b2World_IsValid( def.world ) ||
			 b2World_IsLocked( def.world ) || def.maxAttachments < 1 || def.maxAttachments > 64 ||
			 ( def.profile && !Game::Profile::supported() ) )
		{
			error = "invalid soft Box2D coupling definition";
			return;
		}
		state.coupled = true;
		solver.world = def.world;
		solver.flipY = def.flipY;
		solver.enableDynamicContacts = def.enableDynamicContacts;
		solver.profile.enabled = def.profile;
		attachments.reserve( def.maxAttachments );
		valid = true;
	}
	bool editable()
	{
		if ( !valid )
			return false;
		if ( state.stepping )
		{
			error = "soft coupling is executing a native step";
			return false;
		}
		error = nullptr;
		return true;
	}
	bool owns( ObjectId id ) const
	{
		return id.owner == owner && id.kind == ObjectKind::Attachment && id.index >= 0 &&
			   size_t( id.index ) < attachments.size() && attachments[id.index].active &&
			   attachments[id.index].generation == id.generation;
	}
};
Box2DCoupling::Box2DCoupling( Simulation& simulation, const Box2DCouplingDef& def )
	: impl_( simulation.ready() ? new ( std::nothrow ) Impl( simulation, def ) : nullptr )
{
}
Box2DCoupling::~Box2DCoupling()
{
	if ( impl_ && impl_->valid )
		impl_->state.coupled = false;
	delete impl_;
}
bool Box2DCoupling::ready() const
{
	return impl_ && impl_->valid;
}
const char* Box2DCoupling::lastError() const
{
	return impl_ ? ( impl_->solver.error ? impl_->solver.error : impl_->error )
				 : "cannot create soft Box2D coupling";
}
bool Box2DCoupling::profilingSupported()
{
	return Game::Profile::supported();
}
ObjectId Box2DCoupling::attachBody( b2BodyId body )
{
	if ( !impl_ || !impl_->editable() )
		return {};
	auto& s = *impl_;
	if ( !b2Body_IsValid( body ) ||
		 b2StoreWorldId( b2Body_GetWorld( body ) ) != b2StoreWorldId( s.def.world ) ||
		 b2Body_GetShapeCount( body ) != 1 )
	{
		s.error = "attachment requires one centred sharp box in this world";
		return {};
	}
	b2ShapeId shape;
	b2Body_GetShapes( body, &shape, 1 );
	Game::Solver candidate( s.state.core );
	candidate.world = s.def.world;
	candidate.flipY = s.def.flipY;
	candidate.add( shape );
	if ( !candidate.prepare() )
	{
		s.error = candidate.error;
		return {};
	}
	for ( const auto& item : s.attachments )
		if ( item.active && B2_ID_EQUALS( item.body, body ) )
		{
			s.error = "body already attached";
			return {};
		}
	int slot = 0;
	while ( slot < int( s.attachments.size() ) && s.attachments[slot].active )
		++slot;
	if ( slot >= s.def.maxAttachments )
	{
		s.error = "soft attachment capacity reached";
		return {};
	}
	Impl::Attachment item;
	item.body = body;
	item.shape = shape;
	if ( slot == int( s.attachments.size() ) )
		s.attachments.push_back( item );
	else
	{
		item.generation = s.attachments[slot].generation + 1;
		s.attachments[slot] = item;
	}
	ObjectId id = s.state.id( ObjectKind::Attachment, slot, s.attachments[slot].generation );
	id.owner = s.owner;
	return id;
}
bool Box2DCoupling::isValid( ObjectId id ) const
{
	return impl_ && impl_->owns( id ) && b2Body_IsValid( impl_->attachments[id.index].body );
}
bool Box2DCoupling::detach( ObjectId id )
{
	if ( !impl_ || !impl_->editable() )
		return false;
	if ( !impl_->owns( id ) )
	{
		impl_->error = "soft attachment was removed or belongs to another world";
		return false;
	}
	impl_->attachments[id.index].active = false;
	return true;
}
bool Box2DCoupling::setCollisionEnabled( ObjectId attachment, ObjectId body, bool enabled )
{
	if ( !impl_ || !impl_->editable() )
		return false;
	if ( !impl_->owns( attachment ) || !impl_->state.check( body, ObjectKind::Body ) )
	{
		impl_->error = "invalid soft attachment collision pair";
		return false;
	}
	auto& excluded = impl_->attachments[attachment.index].excluded;
	for ( auto it = excluded.begin(); it != excluded.end(); ++it )
		if ( same( *it, body ) )
		{
			if ( enabled )
				excluded.erase( it );
			return true;
		}
	if ( !enabled )
		excluded.push_back( body );
	return true;
}
bool Box2DCoupling::setFilter( ObjectId body, RigidFilter filter )
{
	if ( !impl_ || !impl_->editable() )
		return false;
	if ( !impl_->state.check( body, ObjectKind::Body ) || filter.groupIndex < -32768 ||
		 filter.groupIndex > 32767 )
	{
		impl_->error = "invalid soft rigid filter";
		return false;
	}
	auto& target = impl_->filters[body.index];
	target.body = body;
	target.value = filter;
	return true;
}
bool Box2DCoupling::prepare()
{
	if ( !impl_ || !impl_->editable() )
		return false;
	auto& s = *impl_;
	auto& solver = s.solver;
	Game::Profile::Scope timing( solver.profile, Game::Profile::Adapter );
	if ( solver.error )
		return false;
	if ( !b2World_IsValid( s.def.world ) || b2World_IsLocked( s.def.world ) )
	{
		s.error = "invalid or locked coupled world";
		return false;
	}
	auto gravity = b2World_GetGravity( s.def.world );
	s.state.core.gravity = { gravity.x, s.def.flipY ? -gravity.y : gravity.y };
	if ( !solver.syncParts() )
		return false;
	for ( const auto& item : s.attachments )
		if ( item.active )
			if ( !b2Body_IsValid( item.body ) || !b2Shape_IsValid( item.shape ) ||
				 b2Body_GetShapeCount( item.body ) != 1 )
			{
				s.error = "coupled fixtures changed; detach and reattach the body";
				return false;
			}
	for ( size_t index = 0; index < solver.parts.size(); ++index )
	{
		if ( !solver.parts[index] )
			continue;
		auto& contact = solver.parts[index]->contact;
		ObjectId body =
			s.state.id( ObjectKind::Body, int( index ), s.state.core.bodies[index].generation );
		const auto& filter = s.filters[index];
		contact.filter = b2DefaultFilter();
		if ( same( filter.body, body ) )
			contact.filter = { filter.value.categoryBits, filter.value.maskBits,
							   filter.value.groupIndex };
		b2ShapeId targets[Game::MaxBoxes];
		int count = 0;
		for ( const auto& item : s.attachments )
			if ( item.active )
			{
				bool excluded = false;
				for ( ObjectId id : item.excluded )
					if ( same( id, body ) )
					{
						excluded = true;
						break;
					}
				if ( excluded )
					continue;
				if ( count == Game::MaxBoxes )
				{
					s.error = "soft coupling supports four contact boxes per body";
					return false;
				}
				targets[count++] = item.shape;
			}
		bool changed = count != contact.boxCount;
		for ( int i = 0; i < count && !changed; ++i )
			changed = !B2_ID_EQUALS( targets[i], contact.boxes[i].shape );
		if ( changed )
		{
			contact.clearCache();
			contact.boxes = {};
			contact.boxCount = 0;
			for ( int i = 0; i < count; ++i )
				contact.add( targets[i] );
		}
	}
	return solver.prepare();
}
bool Box2DCoupling::advance( bool* entered )
{
	if ( entered )
		*entered = false;
	if ( !prepare() )
		return false;
	impl_->state.stepping = true;
	auto before = impl_->solver.entered;
	bool success = impl_->solver.advance();
	if ( entered )
		*entered = impl_->solver.entered != before;
	impl_->state.stepping = false;
	return success;
}
CouplingState Box2DCoupling::getState() const
{
	CouplingState result;
	if ( !impl_ )
	{
		result.failure = lastError();
		return result;
	}
	const auto& solver = impl_->solver;
	result.entered = solver.entered;
	result.completed = solver.completed;
	result.peakGap = solver.peakGap;
	result.failure = lastError();
	for ( const auto& part : solver.parts )
		if ( part )
		{
			const auto& c = part->contact;
			result.recoveryEntries += c.recovery.entries;
			result.recoveryExits += c.recovery.exits;
			result.recoverySteps += c.recovery.activeSteps;
			result.warmHits += c.warmHits;
			result.contactRows += c.rowSolves;
			result.contactDepth = std::max( result.contactDepth, c.lastDepth );
			result.recovering = result.recovering || c.recovery.active;
		}
	return result;
}
CouplingProfile Box2DCoupling::getProfile() const
{
	CouplingProfile result;
	if ( !impl_ )
		return result;
	const auto& profile = impl_->solver.profile;
	result.enabled = profile.enabled;
	result.clockFailed = profile.clockFailed;
	for ( int i = 0; i < Game::Profile::Count; ++i )
	{
		result.samples[i].wallMs = profile.samples[i].wallMs;
		result.samples[i].cpuMs = profile.samples[i].cpuMs;
		result.samples[i].calls = profile.samples[i].calls;
	}
	return result;
}
} // namespace SoftBody
