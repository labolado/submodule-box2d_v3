# Experimental soft-body world

Current recoverable assembly consumers use [the native operation API](API.md):
`Simulation` plus `Box2DCoupling`. Car layout and controls stay in application Lua.
macOS and Android compile this path; other engine platforms are not yet integrated.
The older raw `World` interfaces described below are retained for historical
experimental modes, not the supported boundary for new assembly consumers.

When editing softbody C++ files, use the Box2D repository's `.clang-format`
(`clang-format -i extern/softbody/<file>` from the submodule root). Do not introduce
an independent softbody style. Solar2D bindings follow their neighbouring engine
style; this formatter rule does not apply to the rest of the engine.

## Historical kernel and prototype interfaces

Private C++11 solver with compiled implementations and float/double explicit
instantiations. The engine includes `soft_world.h` through its Lua adapter.
The core has no Box2D dependency. `soft_box2d_static` is a separate experimental
adapter using Box2D public APIs. No rigid solver, worker scheduler, SIMD flags or
LiquidFun behavior are modified.

The `.h` files declare data and operations; the matching `.cpp` files implement
materials, contacts, stepping and the static bridge. Only float and double are
instantiated. `World::step(projector, context, measure)` calls an optional synchronous
projector each soft constraint iteration; it stores neither pointer and invokes
no worker tasks. The context must outlive the call. No callback is registered by
the default `step()`. This private C++ entry is not a Box2D substep API.

CMake consumers add this directory and link `softbody`; when the `box2d` target
exists, `softbody_box2d` is also available. Both disable FP contraction and C++
exceptions on Clang/GCC. The current macOS and Android engine projects compile the core and coupling sources.
Other platform engine projects still need these compilation units and validation.

- `soft_body.h/.cpp`: fixed 16-node local-frame and pressure materials, deformation damping.
- `soft_contact.h/.cpp`: point/edge contact and sweep diagnostics (not general CCD).
- `soft_world.h/.cpp`: body storage, mass-weighted pins, equal/opposite motors, force-limited touch springs, planes,
  static solid quadrants, contact iterations and per-substep diagnostics.

- `soft_box2d_static.h/.cpp`: registered static circle/polygon prototype; no CCD,
  dynamic reaction. Solar2D exposes an experimental static attachment. Polygon segment clipping and coherent endpoint
  separation cover the tested discrete edge crossings; high-speed transit and
  arbitrary deeply interlocked geometry are not certified.

Units are meters, kilograms and seconds, with no display scaling. The validated
profile is 60 Hz with 8 substeps and 8 iterations, gravity (0,-20), and a restricted
three-body assembly. Sample dimensions, connection weights, collision exclusions,
drive timing and moving-boundary paths belong to Lua/test fixtures, not this core.
Parameters outside that profile are not certified stable.

Planes interpolate their scalar offset over each world step; work includes their
normal motion. Forces and torques accumulate for the next outer step and then clear.
Bodies have uniform node mass. Pressure pretension is supported for circles only.
Static Box2D coupling uses public AABB queries in the separate bridge. There is no
arbitrary mesh, moving rigid boundary response, threading or SIMD implementation.

Body removal disables attached pins; IDs are not reused during the world's lifetime.
Lua owns the world lifetime. Core callers must validate definitions and IDs before
mutation; the Solar2D adapter performs those checks. Solver failures are sticky and
require world recreation. A failed step can have partially advanced state; it is not
transactional. The completed substep count records progress, including the failing
substep. Product thresholds/time limits are owned by tests, not the solver.

Metrics aggregate over every substep of the last call; body positions represent the
current state. Sweep warnings remain diagnostics and do not establish robust general
soft/soft collision. Future Box2D coupling should live in a separate adapter rather
than leaking rigid-engine state into the material kernel.

Touch springs act on weighted material points with an implicit damped update at
substep frequency. Their target is an external controller; appliedWork records
its kinetic work, and total force is capped. Removal invalidates the handle;
reused touch slots carry a generation to prevent stale-handle mutation. Touch
IDs are the exception to the non-reused body/pin/boundary allocation policy.

## Recoverable material

Material kind 3 uses finite, mass-weighted corotated quadratic energy instead of
pressure or edge-length springs. A proper 2D rotation aligns rest nodes to current
nodes, without fitting reflection or changing rest topology. At ambiguous zero
covariance the last defined rotation is retained. Forces continuously open folded,
inverted, collinear and coincident states; positions are not replaced by a rest pose.
Frequency, materialScale and deformation damping determine the response. Pressure
preload, bulk and edge stiffness are not part of this material.

Zero/signed area and self-crossing are diagnostic states for kind 3, not fatal
errors. Other materials retain their previous guards. At zero inertia, recoverable
bodies use translation-only damping and ignore torque until they regain a lever
arm. Normal nonfinite-state checks remain active. Deformation damping precedes
elastic impulses for this material, to avoid attenuating boundary reactions while
retaining the full elastic energy. This is an explicit force integrator, not an
unconditionally stable implementation of position-based shape matching.

External boundary contact and pins remain enabled, even when area reaches zero.
The earlier uncommitted `allowYield` boundary prototype was rejected and removed.
The validated profile remains 60Hz, 8 substeps, 8 iterations. Lua rejects
2*pi*frequency*sqrt(materialScale)*substep > 1 for this material; core callers must
validate their own definitions. This guard is not a universal stability guarantee.
Normal three-part driving plus extreme parallel-plane compression and touch release
are tested; arbitrary folded soft/soft collision, complex terrain CCD, incompatible
negative plane gaps, and mobile performance remain outside that evidence.

World pins use `Pin::b == -1` and a fixed `worldAnchor`; their motor target is
absolute counterclockwise angular speed. No surrogate heavy body is created.
Pins with two bodies retain their relative-speed convention. World pin reactions
contribute to external linear impulse and motor work remains explicitly recorded.

Static oriented boxes (boundary kind 2) use a center, unit x-axis and half sizes.
Contact uses node/face projection and box-vertex/soft-edge response; the current
box penetration metric samples nodes. This is not a general edge-edge CCD claim.
Only planes support prescribed motion.

Soft/soft friction is opt-in with `Body::friction` (Lua `createBody{friction=...}`,
range 0..10, default 0). Pair coefficients mix as sqrt(muA * muB), so either zero
keeps the pair frictionless. Boundary coefficients remain independent.
After each normal projection, the point/edge relative tangent velocity is opposed
by an impulse limited to mu * (normal positional impulse / dt). Barycentric edge
reactions preserve linear momentum and, at the coincident corrected contact,
angular momentum. Only velocities change, keeping normal projection intact.
`World::frictionLoss` includes soft/soft tangent kinetic-energy loss as well as
boundary friction. It is not total system energy loss: motors, elastic forces and
normal projection can add energy.
This is local Coulomb-like friction, not a persistent static-contact manifold:
no penetration correction means no friction budget, and each iteration uses only
its own new normal impulse. Holding/sticking can depend on substeps and iterations.
It does not repair arbitrary folded polygon contact, supply CCD, or guarantee
passage through an overlapping rotor gap. The frozen C1 rotor fixture clears both
rotors; this is a bounded scene result, not a general transport guarantee.

Boundary/body collision exclusions are independent of soft/soft exclusions.
`Boundary::excludedBodies` contains stable body IDs; an empty list preserves the
original behavior. `boundaryEnabled` gates node projection, boundary-vertex
response and penetration metrics. Lua `boundary:setCollisionEnabled(body, bool)`
is pair-local, idempotent and rejects foreign-world or removed handles. The
boundary's global `setEnabled` state still takes precedence. No Box2D rigid-body
filter or LiquidFun behavior is changed.

## C1 static bridge contract

Only registered static circles and zero-radius convex polygons with zero bounce
respond. Disabled bodies and sensors do not respond. Unsupported active shapes,
nonstatic bodies, and locked worlds fail validation. Lua reflects Box2D y-down
coordinates to the existing metre/y-up soft world; it does not change its step API.
Shape filters and geometric-mean friction are read live. Native custom material
and filter callbacks are unsupported. Registrations retain deterministic order
following public AABB queries; zero-bit filters retain positive-group overrides.

Static penetration is measured over edges as well as nodes, after each substep,
and merged into the core peak. Query/candidate counters include metric queries.
The sweep guard detects some complete-body transits and fails with
`static_swept_transit`; it neither supplies collision response nor guarantees
pointwise CCD. A failed step may have partially advanced state. Recreate the
world after failure. Deep interlocking and arbitrary folded contact are not certified.

Current acceptance and Lua contracts live in the independent test repository:
`doc/jellycar-soft-body/static-c1-acceptance.md` and
`physics_soft_body_static_bridge/README.md`. No Box2D src or LiquidFun scheduler
changes are required for this static stage. Dynamic coupling requires a separate
synchronization decision; manual soft stepping is not that protocol.

## Restricted box bridge (C2b-1)

`soft_box2d_box.h/.cpp` adds `Box2DBoxContacts` to `softbody_box2d`.
This is separate from the C1 static bridge; it has no engine scheduling or Lua registration.
The caller steps Box2D first, then calls `step` with exactly one soft substep.
`project` is the synchronous per-iteration callback operation. Calls must be serialized
while the rigid world is unlocked. No rigid world is owned or advanced by this adapter.

Register explicit shape IDs in deterministic order. Active non-sensor attachments must be
single centred sharp rectangular fixtures, with zero friction/restitution and no dynamic
motion locks. Dimensions, mass, centre of mass and inertia are read from Box2D.
Only recoverable, frictionless soft bodies are supported. Unsupported live data fails
before soft advancement; a failure during contact can leave a partial step, so callers
must latch the fault rather than retry it. Deleted IDs are inert; an engine wrapper still
needs world-generation and handle lifecycle guards. Duplicate bodies and mixed worlds
are rejected. Filters, group overrides, exclusions, sensors and disabled bodies are handled.

`flipY` maps the existing metre/y-up soft frame to Box2D's frame, including normals,
point velocities and reaction impulses. Position correction remains soft-only and does
not turn penetration into velocity. Penetration metrics cover segments and contained
corners; this is discrete collision response, not CCD or arbitrary overlap recovery.
The known deeply interlocked/degenerate contour cases are not fixed by this extraction.

macOS and native CMake build this adapter. Other engine platform projects still need the
pre-existing soft core/C1 sources as well as this file; they remain unvalidated.
Regression evidence is in the independent test repository at
`doc/jellycar-soft-body/native-box-bridge.md` and `physics_soft_body_sync/box_bridge.cpp`.

### Assembly timing diagnostics

`Assembly::profile.enabled` defaults to false. `soft_profile.h` records inclusive
wall and current-thread CPU intervals only when enabled; it does not change the
solver configuration. `nativeWorld` includes soft callbacks (including the static
world fallback), and `advance` includes `nativeWorld`. Do not add nested buckets.
`adapter` is populated by the native `Box2DCoupling` layer. Unsupported clock targets
can use the normal disabled path. This internal header is not a public plugin API.
