# Softbody runtime

Private experimental softbody library, isolated from LiquidFun and Box2D rigid
solver formulas. The current runtime is `Simulation` + `Box2DCoupling`;
Solar2D exposes it through `physics.newExperimentalSoftWorld` with
`coupled=true, couplingMode='assembly'`. This is not a stable public C++ ABI.

## Responsibilities

| Source | Responsibility |
|---|---|
| `soft_simulation.h/.cpp` | Validated definitions, generational handles, generic body/joint operations and snapshots |
| `soft_simulation_internal.h` | Private storage shared with the coupling owner; not a consumer header |
| `soft_body.h/.cpp` | Recoverable material, optional internal springs, bounded pressure and reference scaling |
| `soft_world.h/.cpp` | Shared state, material velocity update, touch and diagnostics; no standalone prototype integrator |
| `soft_contact.h/.cpp` | Contour geometry predicates used by current contacts/recovery |
| `soft_game_assembly.h/.cpp` | Generic multi-body step, soft/world/rigid links and motors; no car layout |
| `soft_game_world.h/.cpp` | Per-soft-body rigid contact constraints, friction, cache and boundary recovery |
| `soft_game_geometry.h/.cpp` | Circle/polygon/chain geometry, compound exterior and projections |
| `soft_game_recovery.h/.cpp` | Finite topology diagnostics and recovery state |
| `soft_pair_contacts.h/.cpp` | Optional soft/soft contact and recovery |
| `soft_box2d_state.h/.cpp` | Private phase-local Box2D velocity view; no world/geometry/joint ownership |
| `soft_box2d_simulation.h/.cpp` | Public native coupling operations, registrations, lifetime and complete-step dispatch |
| `soft_profile.h` | Opt-in diagnostics; does not choose solver equations |

`soft_game_*` names describe the current game-oriented algorithms, not a vehicle
preset or the removed single-body Lua `game` mode. Cars, materials, shape presets,
controls and level layouts belong to application Lua or test fixtures.

Consumers include `soft_simulation.h` and `soft_box2d_simulation.h` only. See
[API.md](API.md) for contracts, units and limitations. Native units are m/kg/s,
y up. The coupled owner executes one complete configured Box2D step, with soft
phases in every internal substep. Test configuration: timeStep=1/60, subSteps=8;
each 1/480 s substep performs eight soft constraint sweeps. Native defaults remain
1/480 + 1 for regression compatibility. See API.md for timing and counter semantics.
Soft phases remain serial; using Box2D internal substeps does not parallelize them.

## Build and verification

CMake builds `softbody` without a Box2D dependency. If target `box2d` exists,
it also builds `softbody_box2d`. Current Solar2D integration is macOS and Android;
other platforms need explicit integration and validation. Sources are private,
not installed as part of the public plugin SDK.

In the independent `test/` repository, start with `doc/softbody-api/README.md`
and `TEST_INDEX.md`. Current regression groups cover API/lifetime, vehicle/touch,
soft/soft and soft/rigid joints, compound/chain terrain, arbitrary contours,
internal springs, pressure and reference scaling. Frozen car composition lives
in `test/physics_soft_body_game_car/car.h`, not in the runtime.

## Retired experiments

The 2026-10-28 cleanup removes local/global-frame and old pressure materials,
virtual boundaries, `World::step()`, serial static/box bridges, attachment owners,
single-body game stepping and their Lua bindings. Do not add compatibility
wrappers which silently substitute assembly semantics.

Full historical source remains in Box2D Git commit `0a35da29`; matching engine
`33015f6b` and test `9daa5dd` reproduce the old entrypoints. Instructions and
actual cleanup evidence are in `test/doc/jellycar-soft-body/runtime-cleanup.md`.
Historical failures remain failures; removal does not certify unsupported physics.
