# Resting-contact investigation, 2026-09-08

Implementation follow-up: the coordinated body-segment trial described at the
top of `COLLISION-NOTES.md` now addresses the two reproductions on the new body
path. The investigation below records the preceding baseline. The retained
legacy helpers still reproduce the old defects; the new candidate-pose/contact
regressions cover their replacement. Whole-chain simultaneous constraints and
coordinated room response remain future work.

The user confirmed the contact-location build substantially improves Fiesta,
with minor buzzing remaining, and that body movement no longer disables its
collisions. This investigation preserves that baseline. No production source,
installed DLL, or INI settings were changed during this investigation.

Snapshot of source, DLL and the latest game log:
`build/collision-backup-analysis-20260908-191112/`.

## Reproduced: contact depth and solver state describe different poses

`run_chain_simulations` captures the evaluated orientation, then integrates each
target's spring/gravity/drive into `sim_offset`, then queries body collisions
using evaluated engine pivots. `addon_chain_solve_body_contact` maps the sampled
point through the frozen live basis, predicts it with the **current** offset,
and sets its goal to `dot(base, normal) + depth`. Thus the depth measured at the
sampled pose is added to a position that may already have moved inward or
outward. The response solves an incremental displacement, not the original
contact plane relative to the sampled geometry.

`python analyze_collision_solver.py` runs the existing regression suite and an
additional executable using extracted production functions. Eight output
configurations cover rotated parent frames, legacy/full angles, inverted roots,
and scaling. Each receives a small inward and outward solver step while the
sampled basis/point stays fixed, reproducing the phase mismatch without TK17.

Representative results (game coordinate units, not assumed metres):

| Case | Required separation from sampled point | Prior integration movement | Current final separation | Experimental final separation |
| --- | ---: | ---: | ---: | ---: |
| Inward step | 0.000106466 | -0.000214405 | -0.000107817 | 0.000107798 |
| Outward step | 0.000106466 | 0.000212426 | 0.000319003 | 0.000212426 |

All eight inward cases report solver success yet finish inward of the sampled
point. All eight outward cases add an unnecessary push after the contact is
already satisfied. An offline experiment subtracts prior movement along the
normal from the requested separation. It satisfies the inward cases and skips
the already-satisfied outward ones. This validates an absolute-plane goal for
one joint; it is **not** a complete fix for a chain with shared ancestors.

The earlier regression fixtures began from a solver pose consistent with the
sampled basis, so passing those tests did not cover this failure.

## Reproduced: normal-only deduplication loses angular constraints

`physx_contact_store` merges contacts with normal dot product above 0.9999 and
retains the deeper support. `addon_chain_body_manifold_store` now retains the
point belonging to that winning support. However, when different points move
through rotation, equal normals do not imply equivalent constraints.

The additional experiment places a support at one-quarter of the lever length
with 80% of the farther support's depth. Both have the same normal. Production
storage retains only the farther, deeper support. Solving that support moves
the near point through just one-quarter of the displacement; it needs 80%.
In fixture zero, the near point needs 0.000085173 but receives 0.000026646,
about 31% of its required separation. The loss reproduces in all eight poses.

For addon angular response, support equivalence must account for point and
joint influence. The existing deduplication remains appropriate for truly
shared translation constraints; changing it globally would affect room and
body consumers unnecessarily. More retained points also need a solver that
does not add duplicate impulses.

## Further concerns established by source inspection

These are not measured causes of the remaining in-game buzz:

- `addon_chain_apply_visible_body_correction` gives each eligible ancestor an
  equal displacement share and predicts its contribution independently using
  frozen parent bases and live pivots. It sums those contributions; it does not
  compose updated ancestor transforms into descendant transforms.
- Body re-query updates `end_override` by predicted correction but leaves the
  segment start unchanged within the iteration loop, even when an ancestor
  rotation moves both ends. Room/self/addon responses subsequently share this
  partly updated geometry. Additional iterations are therefore not guaranteed
  to improve the actual displayed shape.
- Velocity/friction response projects each joint separately against each
  contact gradient. It does not solve the sum of all influencing joints'
  contact-normal velocities. A later non-orthogonal constraint can invalidate
  an earlier one. Moving-support relative velocity is also not represented in
  this local projection.
- Springs/drive and collision operate once per variable simulation step
  (capped at 0.05 seconds), while positional response gains are per call. The
  current 30/60/144 Hz tests cover component behavior, not an entire animated
  chain with evaluated transforms and moving contacts. Frame-rate sensitivity
  remains an integration-test gap, not an established instability measurement.

## What the latest game trace does and does not establish

The latest log includes a 19:02:45.794 same-feature neck contact with endpoint
request 0.0016043, observed outward motion 0.0004230, and essentially unchanged
gap (-0.0000393 change). This is compatible with incomplete response, but is
not a controlled stationary experiment. Body animation, different closest
points and multi-joint motion affect those measurements. The log alone cannot
attribute a percentage of buzzing to either reproduced defect.

The point probe's `applied=0` field checks the contacted target, while the body
response modifies its ancestors. It does **not** mean no correction was applied;
`sampled_joints` and movement predictions demonstrate this. That label should be
corrected during the next diagnostic change.

## Recommended implementation order

1. Capture one coherent chain pose and local material points before integration.
   Predict the whole candidate pose after integration, accounting for ancestor
   rotations in both segment endpoints. Validate against evaluated engine output.
2. Retain distinct angular supports and define their plane goals against that
   coherent pose. Update residual separation from the current candidate geometry
   after each correction, including changes already made by other joints.
3. Solve the shared joint/contact constraints with bounded accumulated responses,
   then handle relative normal velocity and friction against the same geometry.
   Publish the resulting chain consistently.
4. Test resting/moving support, release/sliding, corners, order changes, joint
   limits, and 30/60/144 Hz on a complete chain. Compare to the saved Fiesta build
   in-game before generalizing the approach to room and body-body response.

The isolated absolute-goal experiment is a useful regression target, but simply
subtracting a separate movement estimate in every existing per-joint call could
double-count shared ancestor motion. A coordinated pose representation is the
next implementation step, rather than another scalar tuning trial.

Artifacts: `analyze_collision_solver.py` and generated
`build/collision_phase_analysis.json`. The experiment asserts the current defects
to document the reproduction; update those assertions when production is fixed.
