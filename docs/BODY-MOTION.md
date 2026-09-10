# Accepted body movement timing — 2026-09-09

Status: accepted in-game; the user reported that the timing change looks fine.
The accepted [joint-limit changes](BODY-LIMITS.md) build on this version.
The cumulative release now also includes the accepted [gravity/inertia changes](BODY-DYNAMICS.md).
The preceding DLL and edited source are saved in
`build/collision-backup-body-timing-20260909/`.

Built and installed to `The Klub 17/Binaries/NC-TK17-PhysX.dll`, with matching
SHA256: `CDF54E5A78C77BA1E5A67DADCE1AB46CE6D64E663F3509A99B0D58B78E3251AC`.
The complete DLL build and all six regression suites passed.
The accepted timing DLL, edited source, tests and user-test log are saved in
`build/collision-backup-confirmed-body-timing-20260909/`.

## Changes

- Ordinary update intervals up to 100 ms advance their full elapsed duration.
  Previously the body paths discarded time beyond 25 ms. Each accepted interval
  is divided into steps no larger than 16 ms, with at most seven steps.
- Spring integration, gravity filtering, limits and contact response run on each
  substep using the same captured engine pose and current predicted angles.
  Final engine output is published after all substeps. Animated support geometry
  is sampled once per update; it is not interpolated between substeps.
- Translation and rotation displacement inputs are scaled to a 16 ms reference
  interval before driving the spring targets. Existing scale settings therefore
  retain their reference meaning, instead of changing strength with sample rate.
  Translation deadzones are applied after normalization. Camera calibration
  continues to receive raw observations in its existing units.
- Translation smoothing uses three fixed 16 ms windows with the established
  60/28/12 weights. At 16 ms it reproduces the previous three-sample filter.
  At other intervals the windows average the available samples over elapsed time.
  Untrusted camera samples clear the history rather than replaying old movement.
- Startup and intervals longer than 100 ms discard displacement drive and clear
  its history. The springs advance at most 100 ms without accumulating a catch-up
  backlog. This is an explicit stall policy, not preservation of arbitrarily
  long elapsed time.
- Existing health diagnostics now include substep count and substep duration.

The spring equation and 16 ms angular update remain unchanged. This timing change
does not introduce mass-based gravity, link inertia coupling or soft joint limits.
Those remain separate potential improvements after timing is tested in-game.
Breast, butt and sidecar integration and user configuration are unchanged.

## Validation and limits

`python run_body_motion_tests.py` checks production timing/filter/spring helpers:
all 1–100 ms intervals, bounded stalls, exact 16 ms reference behavior, fixed and
variable-rate steady movement, history wrap, camera rejection and recovery,
invalid inputs, full elapsed-time advancement and common spring equilibrium.

`python run_body_contact_tests.py` uses production substeps, spring integration,
pose prediction and contact response. Two- and three-link chains rest and release
from synthetic supports at 20/30/60/144 Hz, uneven updates and capped stalls.
The largest settled tip range in these fixtures is below 0.00000003 scene units.
The recorded body-pose, collision, free-motion and binding suites also pass.

These standalone tests do not run the complete TK17 render/animation scheduler,
room traversal or camera calibration. Movement changing within a sampled frame
is approximated by that interval's average; transient responses are not claimed
to be identical across all rates. Both dynamic chains still run sequentially.
Extra contact work at low frame rates can increase CPU cost; measure with the
existing performance counters if needed.

In-game validation: slow body translation and rotation, abrupt movement followed
by stopping, upward-facing thigh/testicle contact, ground support, and camera
orbit. Compare at normal frame rate and a lower frame rate if available. Check
resting stability and release as well as free-motion strength and timing.
