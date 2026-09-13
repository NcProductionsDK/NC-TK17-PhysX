# Collision profiling — 2026-09-13

The initial diagnostic build preserved the approved wind/activation checkpoint and all
physics algorithms and settings. It adds detail to the existing
`performance_profile` option without editing the installed config or defaults.
The resulting optimization is documented in [BODY-CONTACT-PERFORMANCE.md](BODY-CONTACT-PERFORMANCE.md).

Four `collision profile` records accompany each existing one-second profile
window: `avg_ms`, `max_call_ms`, `calls`, and `counts`.

The first gameplay capture exposed a logging integration defect: the normal
logger filters the format string before expansion, so the original generic
`%s` emission discarded the detail records with debug disabled. The corrected
build uses a literal `collision profile ` format prefix and allows that prefix
when profiling is enabled. A regression now exercises the actual filter and
file writer with debug off and verifies all four records reach the log.
The initial capture remains saved at
`build/collision-profile-tests/capture-missing-details.log`.

| Field | Scope |
|---|---|
| `body_collection` | Penis/testicle projection setup and contact gathering, ending before the joint solver; includes early exits and room queries |
| `body_solve` | Complete shared penis/testicle contact solver |
| `body_position` | Initial positional solver passes within `body_solve` |
| `body_refine` | Combined-support refinement within `body_solve` |
| `body_velocity` | Contact velocity projection within `body_solve` |
| `addon_prepare_query` | Addon body-contact setup, geometry tests and trace preparation, ending before manifold resolution |
| `addon_coherent_solve` | Coupled addon joint contact solver |
| `addon_visible_solve` | Visible-output correction fallback, also used by room contact response |
| `room_sphere` | Room sphere query and response calculation |
| `room_sweep` | Complete swept query; includes its `room_sphere` calls |
| `room_single` | Room contact collection for single-bone body systems |

`avg_ms` divides accumulated time by the same tick count as the existing profile,
including all people, chains and substeps. `max_call_ms` is the longest single
invocation in the window, not the worst full frame. `calls` counts invocations,
including early returns. Timings overlap where noted and must not be summed as
independent costs. The three body solver subphases exclude some solver setup.

Workload counters record contacts supplied to the body/coherent-addon solvers,
initial positional passes actually entered, body Jacobian and pose prediction
evaluations, addon gradient evaluations, and room BVH nodes and triangles tested.
`body_passes` excludes refinement and velocity passes; `addon_passes` excludes
velocity passes. Contacts are counted on every solver invocation, so counts can
include the same physical support across substeps. Geometry counts cover sphere
and single-bone room queries, including sphere queries nested inside sweeps.

Counters and clock reads are inactive when profiling is disabled. Enabled
instrumentation adds measurement overhead, especially counters inside frequently
called geometry routines. Use it to locate expensive work; use comparable builds
and workloads for final FPS measurements. Logging occurs after the existing total
tick timer ends, so its file-writing overhead is not included in that timer.

## Next gameplay capture

Restart the game to load the DLL. With the existing `performance_profile` option
enabled, use the same room, actors, equipment and camera for two periods:

1. Allow loading and settling to finish, then hold a pose with little contact
   for about 20 seconds.
2. Hold a demanding pose with body/addon contact for about 30 seconds. Keep the
   camera still for most of it, then briefly move the pose to exercise release
   and renewed contact.

Record which part was contact-heavy and retain the log before the next launch.
The log stays at `The Klub 17/Logs/NC-TK17-PhysX.log`. There are no new config keys.

## Validation and rollback

- Production 32-bit DLL builds successfully.
- Body-contact, collision, single-bone-contact and force/activation suites pass.
- `run_collision_profile_tests.py` checks nested timers, early returns, explicit
  end without double counting, disabled clock reads, report normalization and
  reset behavior. It also recompiles and runs the original body-contact, room
  query and sidecar-contact fixtures with the production profiler enabled.
- Removing the added profiling statements reproduces the five changed physics
  modules from the checkpoint exactly. No solver mathematics or limits changed.

Run the new suite after `run_body_contact_tests.py` and `run_collision_tests.py`,
which generate its production-source fixtures. The previous installed DLL,
source modules, config hash and profile log are backed up locally in
`build/collision-profile-baseline/`; generated artifacts are ignored by Git.

The corrected logging build captured 263 complete detail windows, identifying
repeated body-solver geometry work. The initial capture contains the ordinary
profile only and cannot establish the detailed collision bottleneck.
