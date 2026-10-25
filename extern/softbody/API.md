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
Only recoverable 16-node ellipses are exposed. Joint and touch anchors use 16
nonnegative weights whose sum is one. Positive motor speed is relative angular
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
- All active soft pairs must explicitly disable soft-soft collision. Generic
  assembly means arbitrary supported connection graphs, not new contact support.
- Static/kinematic, single centred sharp box fixtures only, zero restitution;
  up to four registered contact boxes per soft body, at most 64 registrations.
  Pair exclusions allow different bodies to use different registered boxes.
- Shared dynamic rigid contacts, arbitrary topology, general CCD, world/rigid pins,
  direct force loading and soft-soft contact remain unsupported in this path.
- Historical standalone, static, attachment and single-body game modes retain
  their internal interfaces for regression. Their capabilities are not implicitly
  available through this new operation boundary.

`getProfile()` is disabled by default. Inclusive buckets retain their prior
meaning; `adapter` is now populated by the native coupling layer. Do not add
nested timing buckets. Local CMake and macOS/Android engine lists compile the two
new source files. Other engine platforms still need integration and validation.

The independent test repository contains `physics_soft_body_game_car/api_checks.cpp`
(public-header consumer), `guards.cpp` (deliberate internal corruption), frozen
five-scenario migration, and `physics_soft_body_api` (Lua topology and lifecycle).
