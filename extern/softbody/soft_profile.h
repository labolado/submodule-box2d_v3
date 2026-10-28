#pragma once
#include <array>
#include <time.h>

namespace SoftBody
{
namespace Game
{
// Opt-in diagnostics for the private assembly path. Timings are cumulative
// and inclusive: nativeWorld includes its soft-stage callbacks, and advance
// includes nativeWorld. Disabled scopes do not read a clock.
struct Profile
{
	enum Bucket
	{
		Prepare,
		MaterialMotor,
		InitialLinks,
		ContactGeometry,
		ContactSolve,
		Observe,
		Finalize,
		NativeWorld,
		Advance,
		Adapter,
		GeometryProject,
		GeometryGather,
		GeometryCache,
		GeometryRecovery,
		GeometrySides,
		Count
	};
	enum Counter
	{
		ProjectionCandidates,
		ProjectionCorrections,
		GatherCandidates,
		CacheComparisons,
		Rows,
		CounterCount
	};
	struct Sample
	{
		double wallMs = 0, cpuMs = 0;
		unsigned long long calls = 0;
	};
	std::array<Sample, Count> samples{};
	bool enabled = false, clockFailed = false;
	std::array<unsigned long long, CounterCount> work{};
	unsigned geometryDepth =
		0; // Detail scopes belong only to ContactGeometry, not prepare/finalize.

	static bool supported()
	{
#if defined( CLOCK_MONOTONIC ) && defined( CLOCK_THREAD_CPUTIME_ID )
		return true;
#else
		return false;
#endif
	}
	static double read( bool cpu, bool& failed )
	{
#if defined( CLOCK_MONOTONIC ) && defined( CLOCK_THREAD_CPUTIME_ID )
		timespec value{};
		if ( clock_gettime( cpu ? CLOCK_THREAD_CPUTIME_ID : CLOCK_MONOTONIC, &value ) == 0 )
			return double( value.tv_sec ) * 1000 + double( value.tv_nsec ) * 1e-6;
#endif
		failed = true;
		return 0;
	}
	struct Scope
	{
		Profile* profile;
		Bucket bucket;
		double wall = 0, cpu = 0;
		Scope( Profile& p, Bucket b )
			: Scope( &p, b )
		{
		}
		Scope( Profile* p, Bucket b )
			: profile( p && p->enabled && ( b < GeometryProject || p->geometryDepth ) ? p
																					  : nullptr )
			, bucket( b )
		{
			if ( profile )
			{
				wall = read( false, profile->clockFailed );
				cpu = read( true, profile->clockFailed );
				if ( bucket == ContactGeometry )
					++profile->geometryDepth;
			}
		}
		~Scope()
		{
			if ( !profile )
				return;
			double endCpu = read( true, profile->clockFailed );
			double endWall = read( false, profile->clockFailed );
			auto& sample = profile->samples[bucket];
			sample.cpuMs += endCpu - cpu;
			sample.wallMs += endWall - wall;
			++sample.calls;
			if ( bucket == ContactGeometry )
				--profile->geometryDepth;
		}
		Scope( const Scope& ) = delete;
		Scope& operator=( const Scope& ) = delete;
	};
};
} // namespace Game
} // namespace SoftBody
