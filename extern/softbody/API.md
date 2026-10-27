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

## Optional internal springs

`SoftBodyDef.internalSprings` is a copied list of `SoftSpringDef` connections
between existing input nodes (zero-based native IDs, one-based Lua IDs). Lua uses
`body = world:createBody { ..., internalSprings = {
{nodeA=1, nodeB=9, stiffness=600, dampingRatio=0.3} } }` in assembly mode.
The list belongs to the body definition, not `shape`. Omission/empty lists keep
the previous material arithmetic. No mass points, collision edges, pins or mesh
triangles are created; crossing spring lines do not create a shared node.
Connections can include boundary neighbours. Internal mass points and runtime
topology/rest-length editing are not supported.

Stiffness is N/m (default300, accepted0.001..1e8); dampingRatio is dimensionless
(default0.3, accepted0..10). Reference length comes from the initial shape and
must be at least1mm. Input range acceptance does not certify stability: creation
also rejects duplicates, self edges and invalid endpoints, and checks each node's
total incident stiffness against the fixed1/480s rest-state budget
`h^2 * ((2*pi*frequency)^2*materialScale + 2*sum(k)/nodeMass) <= 1`.
This is not a bound for arbitrary compression/contact. The world budget
`maxInternalSprings` defaults to2048; removal releases capacity.

The added elastic energy is `0.5*k*(sqrt(|xb-xa|^2+epsilon^2)-restLength)^2`,
where epsilon is1e-4 times reference separation and restLength uses the same
regularization. Its negative gradient gives equal/opposite central forces, zero
at rest and finite at coincident endpoints. Pairwise axial damping uses
`c=2*zeta*sqrt(k/(1/ma+1/mb))` and an exponential velocity decay; it cannot add
pair kinetic energy and leaves rigid translation/rotation unchanged.

An inverted/self-crossed boundary, nonpositive area or boundary edge below1um
temporarily disables the structure network. The existing recoverable material
unfolds it; after8 healthy complete steps, structural strength ramps at
`frequency` per second back to1. The damping coefficient follows sqrt(strength).
This is an active **recovery material policy**, not a globally passive spring
network: stiffness changes can add or remove elastic energy. Positions, reference
shape and contacts are not reset. `SoftBodyState` / `body:getState()` report:

- `internalSpringScale`: current structural strength,0..1.
- `internalSpringWork`: signed cumulative stored-energy change due only to that
  schedule,J. This is separate from `appliedWork`; neither is a full coupled ledger.
- `internalSpringRecoverySteps`: complete steps spent below full strength.

Force/damping traversal is O(edges); the opted-in boundary validity check is
O(nodes^2) per material step. Mobile cost is not yet validated. Local evidence is
a free18-node bridge with23 springs, real rigid press/release, inversion/collapse
material checks and unchanged no-network car regressions. Extreme press speeds
and loaded negative-control overlap remain documented limitations. These tests
do not certify every network, general CCD or arbitrary highly stiff materials.

## Fixed-world pins

`Simulation::createWorldJoint(SoftWorldJointDef)` accepts a body, its node weights,
an immutable world anchor, frequency/dampingRatio and motorGain. Lua assembly uses
`world:createWorldPinJoint{body=body, weights=weights, x=x, y=y, frequency=12,
dampingRatio=1, motorGain=8}`. Coordinates remain metres/y-up, independent of the
display camera. The returned joint has `setMotor(speed, maxTorque)` and
`removeSelf()` like a soft/soft pin. Speed is rad/s; positive rotates the body
counterclockwise relative to the fixed world, whereas a soft/soft motor drives B
relative to A. Torque is Nm.

This is a compliant positional constraint, not an infinitely rigid pin or an
orientation lock. Its reaction is external to the simulated bodies. Motor torque
is limited once per complete step; disabling torque allows coasting, not immediate
stopping. There are no angle limits or moving-world-target setters. Use touch for
interactive targets; a moving rigid anchor uses the separate coupling pin API below.
Collapsed bodies retain the existing finite-inertia guard; no rigid teleport is used.

World and soft/soft pins share `maxJoints`, slot generations and removal rules.
Body removal invalidates dependent pins. Weights follow public source-node IDs,
including reordered contours. Existing soft/soft arithmetic is unchanged.
The local roller fixture validates centre-distributed pins. Single-node gravity
bearing remains a diagnostic failure (93.6mm transient gap versus its80mm gate);
accepted parameter syntax is not a promise of tight anchoring for every weight set.

## Soft-rigid pins (assembly)

`Box2DCoupling::createRigidJoint(SoftRigidJointDef)` connects one weighted soft
anchor to an attached Box2D body. Lua exposes:

```lua
local attachment = world:attachPhysicsBody(rigidBody)
attachment:setCollisionEnabled(softBody, false) -- explicit application choice
local pin = world:createRigidPinJoint {
    body = softBody, rigid = attachment, weights = nodeWeights,
    localAnchorX = 0, localAnchorY = 0, -- metres, rigid-local y UP
    frequency = 30, dampingRatio = 1, motorGain = 8,
}
local state = pin:getState() -- enabled, errorM, impulseX, impulseY on soft side
pin:removeSelf()
```

Initial anchors must coincide within 1mm. Frequency is 1..60Hz, damping ratio
0..10. Weights use public input-node identities, including custom contours.
This is a compliant positional spring, not an orientation lock. An optional
relative-speed motor is described below; default torque is zero. There are no
angle limits, target setters, breaking force or hard-attachment mode. An off-centre
anchor follows a rigid pivot arm; a passive centred point does not transmit axle rotation.
Static and prescribed kinematic targets absorb reaction externally. Dynamic
bodies receive the opposite linear impulse and the correct moment about their
mass centre. Shifted mass centres and reflected local coordinates are supported.

The full 2x2 effective inverse mass is
`K = (sum(weights[i]^2/mass[i]) + inverseRigidMass) I + inverseInertia perp(r) perp(r)^T`.
With `omega=2*pi*frequency`, compliance is `K/(h*(2*zeta*omega+h*omega^2))`.
Accumulated impulses reset each complete step. Pins solve with the assembly's
contacts before rigid constraints and integration, reading/writing the current
rigid state on every visit. Shared pins/contacts cannot overwrite one another's
reaction. No Box2D position edits, hidden rigid bodies or new core hooks are used.
The no-pin path retains its previous ordering and arithmetic.

`Box2DCouplingDef.maxRigidJoints` / Lua `maxRigidJoints` is a separate 0..64 budget
(default64); it does not consume the soft/world `maxJoints` slots. The coupling
owns `ObjectKind::RigidJoint` handles; query/remove them through the coupling,
not `Simulation`. Slot reuse increments generations. Removing the soft body or
registration invalidates dependent pins. Destroying the rigid body invalidates
the pin immediately; detach the dead registration before the next step, as for
ordinary contacts. The pin keeps the Lua world alive, not the rigid display object.
Disabled rigid targets skip the pin and report zero impulse; re-enabling a widely
separated target is not a recovery guarantee. Sleeping or motion-locked dynamic
and kinematic targets are rejected by the existing bridge; keep them awake.
Contact filters and exclusions do not disable the pin. Registration geometry
limits below still apply, including zero restitution. No collision is implicitly
excluded by creating a connection.

`getState()` reports current geometric gap and last-step soft impulse (kg*m/s).
World energy does not include these springs' stored energy or external support
work; it is not a complete coupled energy ledger. A failed entered step is still
non-transactional. Tight anchoring is not guaranteed: the frozen 0.3kg welded
support / 3kg soft body / single-node load reaches about25.9mm transient gap at
30Hz, although its 50mm diagnostic gate passes. Visual/mobile acceptance remains
separate from native load/reaction/lifecycle checks.

## Soft-rigid relative-speed motor

`Box2DCoupling::setRigidMotor(joint, speed, maxTorque)` / Lua
`pin:setMotor(speed, maxTorque)` drives **soft angular velocity minus rigid angular
velocity**. Units are rad/s and Nm, positive counterclockwise in soft y-up space.
For a tyre rolling right on horizontal terrain, use a negative target speed.
The optional creation field `motorGain` defaults to8 (Nm per rad/s), range0..1e6.
Speed must be finite within±1e4rad/s; torque0..1e6Nm. These are input guard limits,
not validated game parameters. The local car fixture uses±6rad/s and10Nm.

No motor runs until maxTorque>0 and gain>0. `setMotor(0,0)` coasts;
`setMotor(0,positiveTorque)` brakes relative rotation, not absolute world motion.
Changing the motor never changes positional stiffness or collision exclusions.
No instantaneous angle lock or wheel-specific logic is added to the solver.

Once per complete step, after soft material/touch velocity and before Box2D warm
start/constraints, the motor computes `K=1/Isoft+inverseIrigid` and
`J = clamp(h*gain*(target-relative)/(1+h*gain*K), -h*maxTorque, h*maxTorque)`.
This implicit response prevents the isolated motor from overshooting a target at
small inertia. External contacts/materials can still change speed later in the step.
Soft nodes receive a distributed angular impulse; a dynamic rigid target receives
`-J`. The current rigid velocity is refreshed/published for each motor, including
multiple wheels on one chassis. A zero-inertia collapsed soft body skips both
sides until material recovery creates a lever arm. Static/kinematic endpoints
absorb the reaction externally. No Box2D core hook or rigid-position correction.

`pin:getState()` adds `relativeSpeed` (current, after all phases), `motorImpulse`
(last step on the soft side, Nm*s) and `motorWork` (last-step actuator work,J).
The latter is `J*relativeBefore+0.5*J^2*K`; it includes the opposite dynamic reaction
and is relative work against prescribed endpoints. It does not include pin spring,
contact/friction, gravity, other joints, or a complete coupled energy ledger.
The older world `motorWork` remains the soft-side kinetic contribution.

The fixed 3kg rigid chassis / two1kg soft tyres has local forward/reverse,8° incline,
coast and bounded press/recovery tests. Maximum measured terrain overlap on the
slope is6.44mm; press pin gap reaches10.39mm. This does not certify extreme crushing,
CCD, arbitrary mass ratios, suspension equivalence or mobile performance.

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
  dynamic contacts, including the polygon compounds above. A bounded joint-driven
  rigid press and centre world-pinned soft rollers have local fixtures; general
  jointed mechanisms, CCD, arbitrary topology and direct force loading are not
  certified. Soft/rigid positional pins have the bounded checks described above.
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
Assemblies without an enabled soft/rigid pin and without dynamic contacts retain
their previous ordering. This fixes the
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
