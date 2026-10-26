# Native operation boundary

`soft_simulation.h` is the Box2D-independent operation interface for the current
recoverable material. `soft_box2d_simulation.h` adds the coupled complete-step
owner. Consumers include these two headers, not `soft_world.h`, `soft_game_world.h`
or `soft_game_assembly.h`. This remains an experimental API, not a stable SDK ABI.

## Responsibilities

| Layer | Owns | Does not own |
|---|---|---|
| `Simulation` | Validated definitions, canonical body state, generic joints/touches, handles and snapshots | Lua, display objects, vehicle roles, rigid bodies |
| Internal material/assembly solver | Forces, integration, link/contact iterations, recovery caches | Car layout, controls, test thresholds |
| `Box2DCoupling` | Rigid registrations, filters, contact targets, caches and phase dispatch | Lifetime of the supplied rigid world |
| Solar2D binding | Lua validation/userdata, scale/orientation conventions, scheduler/lifetime bridge | Material formulas or contact-cache edits |
| Application Lua | Shapes within supported capabilities, masses, connections, terrain, motors, input and rendering | Internal solver arrays |

All nodes have a single authoritative state. Per-body contact workspaces refer to
stable slots; they do not own or copy another body. Definitions and returned state
are values. Editing a snapshot never edits simulation storage.

## Units and stepping

Native units are metres, kilograms, seconds, radians, with y up. Gravity is read
from the coupled Box2D world; `flipY` reflects its coordinate system when needed.
`SimulationDef.gravity` initializes an uncoupled definition only. The operation
API currently advances through `Box2DCoupling::advance`, not a standalone step.
It executes exactly one complete Box2D step with soft phases at 1/480 s and eight
contact/link iterations. The application supplies eight complete steps per 60 Hz
frame. This is not Box2D's amortized eight-substep algorithm.

The material, motor and contact formulas are unchanged by this API extraction.
Recoverable bodies accept one simple hole-free contour with at least three nodes
(`SoftBodyDef.shape.nodes`, optional `boundary`); the omitted shape remains the
16-node ellipse. Nodes, weights and snapshots are sized dynamically. Joint and
touch anchors require exactly one nonnegative weight per input node, summing to one.
Native `Weights` is a vector: resize it for custom shapes. Definition defaults
still contain 16 zero weights for the ellipse; zero-padded extra entries are rejected.
`SoftBodyState.vertices`, `velocities` and `boundary` are variable-size values.
Boundary winding is normalized without renumbering public node identities. Positive motor speed is relative angular
velocity of B with respect to A. Frequency is Hz; dampingRatio is dimensionless;
material `damping` retains the existing deformation-velocity damping coefficient.
Mass is total mass, distributed uniformly over nodes. There is no density API yet.

## Lifetime and errors

`ObjectId` contains owner, kind, slot and generation. It is a process-local handle,
not serialization data. Bodies, joints, touches and rigid registrations reuse
inactive slots with new generations. Removing a body invalidates its joints and
touches. Collision exclusions and rigid filters do not leak into reused bodies.
Replacing the adapter also invalidates all of its old registration handles.
A stale or foreign handle cannot modify an object; check `isValid`/return values.

Simulation must outlive its adapter; Box2D world lifetime remains the caller's
responsibility. Destroy the adapter before Simulation. Operations are serialized
on the owner thread. Mutations during the native complete step are rejected.
Rejected definitions/capacity changes do not partially create objects.
`lastError` reports operation failure. Solver failure is sticky: a failed entered
step may have changed state and cannot be retried; recreate the owner.
`advance(&entered)` distinguishes failure before native entry from failure after
entry. Counters report entered rigid steps and successfully completed soft steps.

## Current limits

- Configurable bounds: 1..32 body slots, 0..64 joints, 0..64 touches (default one).
  These are allocation limits, not certified mobile scene capacities.
- `SimulationDef.maxNodes` (Lua `maxNodes`) defaults to 512 **active nodes in the
  whole simulation**, not 16 per body. Increase at world creation as needed;
  no rebuild is required. Removing bodies releases the logical budget. Inactive
  slots/caches can retain storage until reuse, synchronization or world destruction.
  `availableNodes()` reports the native remaining budget. Creation checks it before
  allocating body storage or consuming slots; `getBodyNodeCount()` avoids snapshots.
- `maxSoftContactRows` defaults to 512 gathered soft/soft rows. Exhaustion reports
  `soft_pair_contact_capacity`, never silently drops contacts. It can happen after
  entering a step, so the normal sticky-failure rule applies. Rigid rows are sized
  from the node count (node/vertex rows for polygons, node/edge rows for circles).
  Scratch is reused after preflight/first-step warmup. These budgets are not a
  memory quota or a certified mobile limit. No new parallelism or SIMD is implied.
- Soft-soft contacts are off by default: all active pairs must explicitly
  disable collision. `SimulationDef.enableSoftContacts = true` enables the
  experimental path below; it does not change joint collision exclusions.
- Static/kinematic bodies accept a single circle or 1–16 sharp convex polygon
  fixtures (3–8 vertices each), with zero restitution. Local offsets/rotations
  are supported. Multi-fixture circles/mixed circle-polygon bodies, capsules,
  segments/chains and rounded polygons remain rejected. Single circles retain
  analytic distances; centered single boxes retain their historical arithmetic.
- Up to sixteen registered contact shapes per soft body after attachment
  exclusions, at most 64 body registrations. This is a preflight capacity limit,
  not a certified mobile budget. Per-soft-body filtering determines which
  fixtures contribute to the polygon union; sensors never hide solid faces.
- Compound polygon exteriors are clipped in rigid-local coordinates. Interior
  faces and collinear seam endpoints do not produce contact walls. Segments keep
  fixture friction and stable source features; coincident exterior segments use
  one deterministic fixture in registration order (overlapping different
  materials do not get averaged). Bodies keep their full Box2D mass/inertia.
- Each rigid body has one authoritative velocity view per soft contact batch;
  its fixtures accumulate into that view and publish once. Across soft bodies,
  the existing native step state remains authoritative. No rigid solver changes.
- Supported fixture geometry, filters, mass/type and enabled changes invalidate
  impulses as needed. Adding/removing/replacing polygon fixtures is discovered
  at preflight without reattaching. Removing all fixtures leaves an empty valid
  registration. Destroying the rigid body requires detaching its registration.
  Unsupported changes fail before stepping and can be repaired. Mutation during
  a native step is forbidden. Lua `boxContacts` retains its historical name.
- `Box2DCouplingDef.enableDynamicContacts` separately opts into awake, unlocked
  dynamic contacts, including the polygon compounds above. Jointed mechanisms,
  general CCD, arbitrary topology, world/rigid pins and direct force loading are
  not certified in this path.
- Historical standalone, static, attachment and single-body game modes retain
  their internal interfaces and geometry restrictions for regression. The new
  geometry is enabled only by `Box2DCoupling` (Lua `couplingMode='assembly'`).

The P4j local fixture covers static support, long-edge contacts, rotating circle
friction, offset dynamic reaction, mirrored local geometry, bounded touch and
curved pressing/release. This does not certify arbitrary initial containment,
interlocked geometry, continuous collision detection or mobile cost. Native
positions/forces retain metre, kilogram, second and y-up units.

Assemblies with an enabled dynamic rigid target solve contact impulses at the
existing pre-rigid-constraint stage. Box2D then handles its own contacts/joints
and integrates rigid positions; soft geometry projection still follows that
integration. The same impulses are not applied a second time in the late stage.
Static/kinematic-only assemblies retain their previous ordering. This fixes the
observed stiff-contact feedback when dragging a rigid circle against soft bodies
trapped near static terrain; it does not create a unified rigid/soft solver or CCD.

`getProfile()` is disabled by default. Inclusive buckets retain their prior
meaning; `adapter` is now populated by the native coupling layer. Do not add
nested timing buckets. Local CMake and macOS/Android engine lists compile the two
new source files. Other engine platforms still need integration and validation.

The independent test repository contains `physics_soft_body_game_car/api_checks.cpp`
(public-header consumer), `guards.cpp` (deliberate internal corruption), frozen
five-scenario migration, and `physics_soft_body_api` (Lua topology and lifecycle).

## Experimental soft-soft contacts (P4d)

`enableSoftContacts` is immutable at creation and defaults to false. Ordinary
contacts use two-sided node/edge and interior edge-crossing rows, accumulated
normal impulses and Coulomb friction. `setCollisionEnabled(a, b, false)` excludes
the pair, including its recovery. No Box2D rigid solver changes are required.

Deep overlap uses a persistent pair state keyed by both body generations. It
retains the last separated direction and leaves recovery only after eight complete
steps with valid contours and disjoint projection intervals. For concave reference
shapes, whole-contour containment replaces the centre test, and actual clear
contours can retire recovery without a separating plane. These are restricted
certificates, not exact overlap-volume solving; arbitrary interlocking remains
uncertified. Adding boundary samples changes the equal-node mass distribution
and can change deformation/contact behavior. It is not a converged continuum model.

Position assistance shares a 0.02 m per-node budget across pairs and passes in
one 1/480 s step. During deep recovery only, the boundary adapter projects desired
whole-body translation onto retained rigid-face constraints. A blocked body can
cede its share to the other. Assistance changes positions without adding delta/dt
to velocity or rewriting rest geometry. This deliberately sacrifices exact
physical energy/centre-of-mass evolution for recoverability after crushing;
ordinary contact momentum is checked separately. Retained boundary sides are
not continuous collision detection. There is no guarantee when the retained
separation direction has no available space.

`CouplingState` (and Lua `getState()`) expose:

| Native field | Lua field | Meaning |
|---|---|---|
| `softContactRows` | same | Cumulative velocity row solves, not unique contact count |
| `softContactDepth` | `softContactDepthM` | Latest gathered row depth in metres, not overlap volume |
| `softRecoverySteps` | same | Complete steps that used pair recovery rows |
| `softRecoveryEntries`, `softRecoveryExits` | same | Cumulative entries and successful healthy exits |
| `softRecoveringPairs` | same | Active recovery pairs at the last completed step |
| `softRecoveryDistance` | `softRecoveryDistanceM` | Cumulative whole-body translation lengths in metres |

Removal/filtering discards pair state without counting a successful recovery exit.
Counters therefore need not balance. Existing `recovering` describes individual
body recovery only; also check `softRecoveringPairs` for pairs. Snapshots may stay
stale until the next completed step after an edit.

The local native and Simulator gate covers two free bodies: ordinary impact,
confined support, shared extreme compression/release and lifecycle. It does not
certify four-body stacking, contacts between jointed assemblies, combined dynamic
rigid/soft-pair contacts, mobile cost or arbitrary user geometry. The supported
car still excludes collisions between its own connected parts. The body-slot bound remains
32; node and soft-row budgets are runtime settings, not a performance promise. Only macOS is built
and run for P4d; Android source lists are updated but the new path is untested there.

## Dynamic-node validation (P4i follow-up)

Local tests cover 17/32/64-node contacts and constraints, 96-node slot reuse,
513-node definition creation with maxNodes=1024, edge-intersection scratch beyond
18 entries, and warm cache reuse. 32/64-node ellipses recover after a .02m plate gap.
The 24-node concave tray recovers its released shape but does **not** meet the .04m
flattening gate (minimum about .0631m). This limitation is retained as an explicit
diagnostic; materials and recovery parameters were not tuned. See the independent
`test/doc/jellycar-soft-body/dynamic-node-results.md` for actual coverage and failures.

This changes experimental private C++ definitions/storage; direct native users of
these headers must rebuild and size custom-shape weights explicitly. It does not
change the Solar2D public plugin SDK, existing engine class layouts or virtuals.
Historical Lua modes still accept only their original 16-node ellipse. Arbitrary
counts are exposed through `couplingMode='assembly'` only.
