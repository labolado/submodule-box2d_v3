# Experimental soft-body world

Private C++11, header-only solver, isolated from Box2D and LiquidFun. The engine
includes `sb_world.h` through its Lua adapter. No Box2D public headers, rigid solver,
worker scheduler, SIMD flags or LiquidFun behavior are modified.

- `sb_body.h`: fixed 16-node local-frame and pressure materials, deformation damping.
- `sb_contact.h`: point/edge contact and sweep diagnostics (not general CCD).
- `sb_world.h`: body storage, mass-weighted pins, equal/opposite motors, planes,
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
