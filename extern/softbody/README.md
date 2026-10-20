# Experimental soft-body world

Private C++11, header-only solver, isolated from Box2D and LiquidFun. The engine
includes `sb_world.h` through its Lua adapter. No Box2D public headers, rigid solver,
worker scheduler, SIMD flags or LiquidFun behavior are modified.

- `sb_body.h`: fixed 16-node local-frame and pressure materials, deformation damping.
- `sb_contact.h`: point/edge contact and sweep diagnostics (not general CCD).
- `sb_world.h`: body storage, mass-weighted pins, equal/opposite motors, force-limited touch springs, planes,
  static solid quadrants, contact iterations and per-substep diagnostics.

Units are meters, kilograms and seconds, with no display scaling. The validated
profile is 60 Hz with 8 substeps and 8 iterations, gravity (0,-20), and a restricted
three-body assembly. Sample dimensions, connection weights, collision exclusions,
drive timing and moving-boundary paths belong to Lua/test fixtures, not this core.
Parameters outside that profile are not certified stable.

Planes interpolate their scalar offset over each world step; work includes their
normal motion. Forces and torques accumulate for the next outer step and then clear.
Bodies have uniform node mass. Pressure pretension is supported for circles only.
There is no arbitrary mesh, rotating-boundary response, Box2D coupling, broad-phase
acceleration, threading or SIMD implementation in this module yet.

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
