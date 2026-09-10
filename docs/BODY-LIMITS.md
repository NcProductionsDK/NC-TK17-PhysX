# Progressive body-joint stops — 2026-09-09

Status: accepted in-game; the user reported that it looks fine. It builds
on the accepted movement-timing DLL `CDF54E5A...E3251AC`, backed up with source,
tests and user-test log in `build/collision-backup-confirmed-body-timing-20260909/`.
The cumulative release now includes these changes and the subsequent accepted
[gravity/inertia implementation](BODY-DYNAMICS.md).

Built and installed to `The Klub 17/Binaries/NC-TK17-PhysX.dll`, with matching
SHA256: `D776A68ABFBDE0D7E7F6E5F44835957000B86D017AE4241D479DBF6B6F7E7023`.
All six regression suites and the complete DLL build passed.

## Behavior

The previous spring update hit each configured angular limit by clamping the
position and then cancelling outward velocity. This implementation adds progressive,
directional braking as a free-moving joint approaches its stop. It affects both
penis and testicle chains through the shared `physx_body_motion.h` helpers.

The final quarter of the angular distance from neutral to each limit is a braking
band, capped at 12 degrees. Each side uses its own range, so a narrow asymmetric
testicle root is treated differently from a wide distal joint. For an interval
excluding zero, the midpoint supplies the reference. Braking strength grows
quadratically with predicted proximity and scales with the square root of the
existing spring stiffness. Implicit damping reduces the proposed outward speed
without reversing it or adding kinetic energy.

This is a motion-dependent stop, not an added elastic spring or a reduced angular
range. It adds no torque at rest and does not change stationary target angles.
Movement away from the nearby stop is unmodified, unless the proposed step reaches
the braking band at the opposite stop. The existing hard bounds still catch extreme
motion and locked joints. Contact corrections are applied afterward and retain the
full configured range; they are not fed through the braking helper.

No INI changes are required. Gravity targets, per-link gains, ordinary free-motion
stiffness/damping and collider geometry are unchanged. Total-chain bend/twist
limits still use their existing hard constraint; this implementation softens individual
joint stops only. Mass-based gravity and coupled link inertia remain separate work.

## Verification

`run_body_motion_tests.py` checks:

- Exact ordinary spring behavior outside the braking band, continuity at its
  entry, stationary equilibrium throughout the range, and immediate reversal.
- Braking does not increase or reverse proposed velocity across 5,000 combinations
  of angle, speed, symmetric/asymmetric and offset limits.
- Locked joints, extreme zero-damping swings and reachability of the full range.
- For a synthetic testicle-root swing (stiffness 115, damping 4, limit 7.5 degrees),
  final-stop speed falls from approximately 58 to 12–18 degrees/second across
  20/30/60/144 Hz schedules. Peak speed earlier in the swing remains approximately
  62 degrees/second. These are fixture measurements, not game telemetry.

`run_body_contact_tests.py` additionally places a support inside the narrow limit's
braking band. It checks resting stability, first-step release on target reversal,
and that collision escape can still use angles past the start of the band. Both
chains' existing rest/release tests use the new spring helper at 20/30/60/144 Hz,
uneven intervals and capped stalls. Recorded-pose, collision, sidecar free-motion
and startup-binding suites pass as well.

Gameplay should compare larger swings, sudden body stops, reversals, and the
confirmed upward-facing contact case. Small motions far from joint limits should
feel the same. Whether the easing looks more natural requires in-game assessment;
this is not a claim of a complete anatomical or soft-tissue simulation.
