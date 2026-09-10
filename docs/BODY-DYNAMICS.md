# Confirmed gravity and link-inertia checkpoint — 2026-09-09

Status: accepted in-game; the user is pleased with the combined gravity and
link-inertia changes for penis and testicle chains. No configuration edits are needed.
The accepted limit-stop DLL (`D776A68A...6F7E7023`), source, tests and game log are
preserved in `build/collision-backup-confirmed-body-limits-20260909/`.
The exact accepted DLL, source, tests, game log and preceding collision release ZIP
are preserved in `build/collision-backup-confirmed-body-dynamics-20260909/`.

Gameplay-confirmed DLL SHA256:
`E6B9A09A4C217EB4984E3B4560FD9401543AD1F6988EAB0B4BDF870F0340FAC3`.
All seven regression suites and the complete DLL build passed.

## Gravity

The plugin fits the same composed joint model already validated against engine
poses. Each measured segment is treated as a uniform rod with relative mass
proportional to length. For each bend coordinate, it measures how rotation moves
the mass of that link and all downstream links. Projecting that derivative onto
the local gravity direction supplies the joint's gravitational leverage.

The resulting geometric contribution changes as the chain bends. It decreases
as a lever lines up with gravity and changes sign when the lever crosses to the
other side. It no longer applies the same gravity response at every orientation.

The accepted implementation blends 50% of the existing configured gravity contribution
with 50% of the geometric contribution. Its force magnitude is derived from the
existing gravity-only targets and normalized by the reference chain's leverage;
it is not a new SI-unit mass/gravity calibration. Profile gravity strength,
curves and gains therefore still influence the result. Zero configured gravity
drive produces zero geometric drive. Body motion and wind are separated from
the gravity-only contribution before replacement, then the existing joint and
total-chain target limits are applied.

Gravity is transformed from the configured world direction through the existing
camera/body basis conversion. Its direction is smoothed using the configured
gravity response time. A reversal can fade through zero without normalizing a
tiny vector into a sudden full-strength force. During camera quarantine the last
trusted direction is retained. Missing/unready body bases use the existing gravity
mapping rather than reading an unavailable frame.

## Effective inertia

Two quadrature samples along each rod capture both the movement of its center
and its distributed rotational inertia. Each joint's diagonal effective inertia
includes all downstream rods. A reference pose is used so inertia does not flicker
as contacts move the chain. Relative inertias are normalized and blended with an
identity floor: `I = 0.65 + 0.35 * normalized_rod_inertia`. This avoids nearly zero
distal inertia. Uniformly scaling the entire rig preserves the relative response.

Spring acceleration uses inverse inertia. Damping is scaled to retain the
configured damping ratio, so the different links have different response periods
without introducing arbitrary per-link damping settings. Contact normal-velocity
projection uses the same inverse inertias. Positional depenetration is still not
converted into velocity. Progressive joint stops and bounded substeps remain.

This is a reduced diagonal inertia model, not a full articulated-body solver:
there is no off-diagonal inertial coupling or Coriolis force. Only the configured
horizontal/vertical bend coordinates use it; the remaining twist coordinate keeps
its existing response. No anatomical density or soft-tissue deformation is modeled.
The two dynamic chains still run sequentially. These are practical approximations
intended to improve motion, not a claim of physical completeness.

## Activation and diagnostics

Validated live geometry establishes the reference, including when individual
contact-response toggles are disabled and collider sampling remains available.
The last valid reference is retained through temporary held/invalid samples;
chain reinitialization clears it. Unsupported rigs or configurations that never
yield a valid reference retain the previous dynamics. A ready body frame is also
required for geometric gravity. The health log reports `inertia=1` and
`geometry_gravity=1` when each path is active.

New source dependency: `physx_body_dynamics.h`.

## Verification

`python run_body_dynamics_tests.py` checks:

- Single-rod inertia against `L²/3` and gravitational leverage against `L/2`.
- Gravity direction and leverage changes, hanging alignment, reversal through
  zero, zero configured drive, and invalid geometry rejection.
- Heavier effective proximal inertia, distinct link response, and rig-scale
  invariance for two- and three-link chains.
- Combined free-motion settling at 20/30/60/144 Hz. In the synthetic three-link
  case, the settled root/distal angles are about 19.787°/10.8804° at each rate.
- The actual integration helpers preserve non-gravity movement contributions,
  gravity disable, camera-held direction, missing-frame fallback and damping ratio.

`run_body_contact_tests.py` exercises the combined gravity/inertia/contact path
for two- and three-link resting/releasing chains at multiple rates, uneven frames
and capped stalls. It checks that inertia-weighted contact projection cancels
inward speed without increasing kinetic energy in an isolated contact case.
Existing body-pose, body-motion, collision, free-motion and binding suites pass.

The standalone fixtures do not execute TK17's complete render/animation traversal.
Gameplay confirmation covers the user's tested setup. Different rigs, profiles,
moving supports and simultaneous body/room contacts still benefit from broader
gameplay coverage. Earlier accepted DLLs remain available for comparison.

## Checkpoint cleanup

- Removed the redundant one-iteration wrapper around the penis contact solve;
  contact sampling and response still execute once per substep.
- Gravity requests leverage without accumulating unused inertia values. Reference
  preparation still computes the same effective inertias. A comparison with the
  saved implementation across 10,000 varied one-, two- and three-link poses found
  bit-identical gravity output and reference inertia under the build toolchain.
- No tuning, collision equations or user configuration changed. This is a small
  arithmetic cleanup; no gameplay performance improvement has been measured.

The cumulative release package includes accepted timing, progressive stops,
gravity and inertia alongside the preceding collision/free-motion fixes.

Cleanup build: all seven regression suites and the full DLL build passed.
Installed to `The Klub 17/Binaries/NC-TK17-PhysX.dll` and packaged with matching
SHA256: `2A14414F05589A56883B0311039101871EA96BFAA59A2E3A204783BE0D6810C1`.
The game configuration hash is unchanged. Gameplay confirmation above applies
to the saved pre-cleanup DLL; the cleanup build has automated validation.
