# Body contact solver optimization — 2026-09-13

The corrected collision profile captured 263 complete windows. The body contact
solver accounted for most of the measured penis/testicle collision cost during
the busier section. All measured addon and room collision paths recorded zero
calls, so this capture does not justify changes to those paths.

Frame-weighted measurements from the saved pre-change capture:

| Period | Collection | Complete body solve | Position | Refinement | Velocity |
|---|---:|---:|---:|---:|---:|
| 01:37:30–01:37:39, 1,068 ticks | 0.00937 ms | 0.00125 ms | 0.00004 ms | 0.00080 ms | 0.00004 ms |
| 01:38:50–01:39:40, 5,072 ticks | 0.01314 ms | 0.27899 ms | 0.12655 ms | 0.02841 ms | 0.12358 ms |

The largest individual window's average body-solver cost was 0.71750 ms/tick.
The three solver subphases are nested in the complete solve. The quieter/busier
labels describe the measured workload, not independently recorded user actions.
These timings include the enabled profiler's overhead.

## Change

During the eight velocity projection passes, the final positional correction,
joint angles, contact normals, sampled pose and base geometry remain fixed.
Only velocities change. Their geometric Jacobian can therefore be computed on
the first pass and reused for the next seven passes.

The cache lives on the current solve's stack and holds at most 24 × 3 × 2 floats
(576 bytes). It cannot survive into another pose, frame, person or solver call.
All eight passes still run in the same contact order and read current velocities.
Inertia calculations, joint-limit handling, positional solving, refinement,
response strengths, timestep handling and configured iteration counts remain
unchanged. No config or default-setting changes accompany this optimization.

## Validation

- The existing body-contact suite passes, including coupled supports, inward
  velocity removal, limits, deep overlap, release and variable-rate settling.
- The collision profiler suite passes its actual file-logging test and reruns
  the body, room and addon fixtures with the production profiler enabled.
- `run_body_contact_performance_tests.py` compares against a frozen pre-change
  solver over 2,400 cases: 0–24 contacts, 1–3 links, legacy/composed geometry,
  inertia, joint limits, response settings and timesteps. Observed maximum
  correction and velocity differences were both zero. Other state and contact
  inputs were byte-identical. Each enabled solve eliminated exactly seven
  Jacobian evaluations per contact.
- The production 32-bit DLL builds successfully.

The isolated 32-bit `-O2` benchmark measured approximately **18–54% less time per
complete solve** for its contact workloads (1, 3, 8 and 24 contacts). For example,
2,000 composed three-contact solves took 1,186.922 ms before and 958.294 ms after;
the single-contact case took 76.613 ms before and 37.738 ms after. These are
synthetic helper timings, not measured FPS improvements. The positional phase
still dominates difficult multi-contact cases, so total savings vary with pose.

Generate the current fixture with `python run_body_contact_tests.py`, then run
`python run_body_contact_performance_tests.py`. Results are saved locally in
`build/body-contact-performance/results.txt`. The pre-change DLL, source, config
hash, captured log and analysis are in `build/body-contact-optimization-baseline/`.

## Next gameplay comparison

Restart and repeat the same contact-heavy pose with profiling enabled. Compare
`body_velocity`, `body_solve`, `body_jacobians` and `body_predictions` alongside
contact counts and total plugin time. Check settling and release too.
Binding/refresh costs and overall frame spikes remain separate opportunities;
this change does not claim to resolve them. In-game confirmation is pending.
