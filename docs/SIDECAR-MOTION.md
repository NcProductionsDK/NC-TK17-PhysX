# Confirmed sidecar motion checkpoint — 2026-09-09

Status: accepted in-game; the user reports that the motion looks very good.
This follow-up changes sidecar parent-to-child motion generally,
with no addon-name checks or configuration edits. Body physics is unchanged.

The preceding accepted DLL (`2A14414F...D6810C1`), source, tests, log and release ZIP
are saved in `build/collision-backup-before-sidecar-motion-20260909/`.
The newly accepted DLL, source, tests, log, current game/addon test settings and
preceding release ZIP are saved in
`build/collision-backup-confirmed-sidecar-motion-20260909/`.
The cumulative release package now includes the accepted sidecar motion changes.

All seven regression suites and the full 32-bit DLL build passed. Built and
installed to `The Klub 17/Binaries/NC-TK17-PhysX.dll`, with matching SHA256:
`8FBC79B8A01A723B745B02BC352811DF579E5B0A2E6D67F7D30BE961086D1841`.
Checkpoint cleanup updates documentation and packaging only. The source, DLL
and live game/addon configuration are unchanged from the accepted test. No
performance improvement is claimed; remaining solver work is listed below.

## Findings

The sidecar solver already integrates combined forces on a fixed-length sphere,
using substeps of at most 1/240 second within the accepted 50 ms frame interval.
It projects acceleration and velocity tangent to the current link and keeps
camera quarantine out of spring coefficients. Those are the accepted fixes
described in [FREE-MOTION.md](FREE-MOTION.md).

Two weaknesses remained in how links pass motion to their children:

1. Inherited bend and velocity used the parent's raw displacement. The same parent
   rotation therefore drove a short child more strongly when the parent link was
   longer. Authored lengths unintentionally acted as an extra response multiplier.
2. Free-motion publication estimated velocity from frame displacement and zeroed
   it below 0.00001 local units. Contact publication used solved velocity instead.
   Small motion could disappear from inheritance, and contact/release switched
   between an averaged displacement estimate and the current solved velocity.

These are source and standalone-test findings, not a diagnosis of a new game log
or a claim that all remaining sidecar motion issues have the same cause.

## Implemented

Inherited bend and speed are scaled by `child_length / parent_length` before use.
This expresses the parent's angular motion at the child's link radius. Existing
joint gain, chain taper and drive strength remain in effect. The velocity cap is
applied after conversion. An unavailable/invalid parent length keeps the existing
displacement-based fallback. Equal-length links retain the same inheritance scale.

The fixed-length bone/matrix and object-transform paths now publish their solved
tangent velocity through free motion, contact and release. The small-displacement
cutoff is not used on those paths. Older rigid paths keep their previous velocity
estimate; contact correction still never becomes inherited momentum. The final
upstream-contact publication continues to publish the corrected solver velocity.

This is kinematic motion inheritance, not added downstream mass or a complete
articulated-body solver. An unequal-length chain's resting bend and motion feel
can change because the unintended length multiplier is gone. Existing profiles
may have been tuned around it; broader gameplay comparison remains useful.

## What should be considered next

- **Gravity:** tangent projection already gives some dependence on the current
  link direction. The input still uses authored axis mappings, a projection
  relative to the rest direction, optional inverted-gravity shaping and stiffness
  scaling. It is not a direct distributed-mass gravity model. A replacement needs
  to preserve custom coordinate mappings and distinguish authoring choices from
  physics errors; copying the body-chain gravity helper is not sufficient.
- **Inertia:** inherited spring/velocity following is not physical mass coupling.
  A fuller model needs a verified chain hierarchy, outgoing segment geometry and
  coordinate transforms, including terminal links and object-transform sidecars.
  Target rest lengths alone are insufficient evidence for a full mass model.
- **Limits:** published Euler angles are clamped, while free-motion state lives
  on a vector sphere. This can permit hidden solver motion beyond a visible limit.
  Progressive stops should constrain the same output mapping used for contacts,
  including full-angle/legacy modes, inverted root roll and output scaling. A
  naive Cartesian clamp would not solve that mismatch. This build does not alter
  joint limits or contact geometry.
- **Scheduling:** free motion substeps each link, but the chain still advances in
  parent-first order with contacts outside those substeps. Longer-frame and
  moving-support behavior deserves a chain-wide fixture before changing scheduling.

## Verification and gameplay check

`python run_free_motion_tests.py` now also exercises:

- Identical angular input with parent lengths from 0.025 to 0.4 units.
- Equal and unequal three-link chains, including actual velocity publication
  and a small root movement reaching the tip after settling.
- Tiny velocity, exact rest, contact correction and release publication.
- A supported unequal-length chain settling and releasing using contact math.
- Fixed 30/60/144 Hz, alternating 16/17 ms and occasional 50 ms updates.

A comparison against the saved implementation held the parent at the same angle:
the old child settled at 1.35147°, 5.39092° and 20.6801° for the three parent
lengths; the new child settled at 5.39092° in all three cases. A 0.00002-unit/s
velocity previously published as zero now remains 0.00002-unit/s.

The standalone tests do not execute TK17's full animation traversal or reproduce
every addon coordinate mapping. In-game, compare gentle movement and stopping,
freely hanging motion, body/ground support and release, and camera orbit. Fiesta
and NcHat008 are useful available examples; the implementation applies generally.
