# Gravity sampling overhaul — 2026-09-10

Status: accepted in-game. The user confirmed responsive pose switching, pose
location changes and Ctrl+Z, with no problems reported. The short confirmation
delay after moving the camera was accepted as a reasonable safeguard.

The gameplay-confirmed DLL, source, configuration, log and preceding release ZIP
are preserved in `build/collision-backup-confirmed-gravity-sampling-20260910/`.

The previous tested build, source,
configuration and log are preserved in
`build/collision-backup-before-gravity-overhaul-20260910/`.
Previous DLL SHA256:
`2589A63A57CC8A71E5ACB139331EBA1C38FB65D87025A8489EAEFF8440A52F4A`.
The cumulative public release now includes the accepted sampling overhaul.

Gameplay-confirmed DLL SHA256 before release comment cleanup:
`AF10DAEBBB3262932E2216F5D726F6E5F69922DAC15828227E8CBE03DB358E86`.
Release cleanup updates comments in the game configuration and embedded defaults
without changing settings. No add-on INIs were edited. The resource rebuild is
verified against the tested DLL: every non-resource section is identical after
excluding the export timestamp. Embedded defaults match the reference INI.
Release DLL SHA256:
`E930024E4CA2DBF96A69EECFC9937E2950A4124085EE820157209FF793DC4464`.

## What changed

Gravity sampling now uses `physx_gravity_sample.h` for the main body drive,
geometric penis/testicle gravity, breast spacing gravity, and person-owned
sidecar parent gravity. The body path no longer uses its extra 1.6-second
camera hold or releases on view-space root displacement. Sidecars no longer
unconditionally accept live parent gravity while leaving an unreachable older
hold implementation underneath.

The shared sampler retains the last accepted direction and queues a candidate.
A later simulation frame must confirm it against the same complete camera
version, covering both camera translation and rotation. Camera movement clears
pending samples. Sampling resumes after at least 48 ms of camera quiet, or the
larger configured probe/root quarantine. Two agreeing samples are required;
the older of them is accepted. Simulation substeps cannot confirm each other.

A changed pose does not have to move the root far enough to trigger a heuristic.
With a quiet camera, a settled pose flip is accepted on its next confirming
frame. Actual movement still follows the configured smoothing, speed cap,
springs and inertia; this is not an instant bone snap.

Missing/invalid samples retain the previous accepted force. A changed parent
object resets sampling, and sidecar warmup supplies zero until confirmed rather
than falling through to a gravity vector expressed in another frame. Room
reference restoration preserves the neutral pose but resets sample history.
Breast neutral-reference capture and initial body filtering wait for accepted
gravity instead of capturing temporary warmup zeros.

Geometric gravity gathers candidates while the primary drive is held, avoiding
a circular wait on release. Its direction updates only when both paths permit
it. The existing geometric force blend, joint inertia and collision solvers
are unchanged.

Body gravity smoothing now uses an exponential response based on elapsed time,
matching the geometric direction filter. Disabling `gravity_zero_at_start` no
longer resets that smoothing every accepted update. `gravity_response_ms = 0`
still removes only smoothing: sample checks and the configured speed cap remain.
Sidecar forces still use their existing spring integration without an added
global smoothing layer.

## Validation and limits

All eight standalone regression suites pass. The gravity suite exercises the
production sampler, camera capture hook/projection, runtime sampling adapter,
sidecar adapter and body filter. Cases include a 180-degree pose flip,
one-frame spikes, a disagreeing late body sample after a camera change,
camera changes inside one simulation frame, panning/orbiting, invalid directions,
parent rebinding, stalls and tick/frame wrap. Smooth pose motion and filtering
are checked at 20/30/60/144 Hz. The dynamics suite checks the production geometric
helper's warmup, simultaneous hold and release behavior.

The agreement bound is a vector difference of 0.08 (about 4.6 degrees for unit
directions); signed/scaled body channels use that same vector bound. Samples
more than 100 ms apart cannot confirm each other. Continuous very rapid pose
rotation, a very low update rate, or continuous camera motion can therefore
retain the older direction until samples qualify. The test checks smooth
rotation at one radian per second; it does not establish an unrestricted
angular-speed range.

This is conservative sample validation, not an engine guarantee that a matrix
was evaluated during the current frame. Repeated stale-but-consistent matrices
can still pass. Gameplay and the new log counters must verify the chosen quiet
period in TK17. No claim of complete camera isolation follows from synthetic
tests alone.

The existing initial room/startup calibration remains. Authored local-gravity
room sidecars and optional breast translation sag retain their separate paths.
Wind sampling and motion-camera protection are unchanged. Explicitly disabling
camera compensation retains the legacy profile behavior and does not provide
the world-gravity isolation of the default compensated path.

## Gameplay acceptance procedure

For future regression checks, restart TK17, keeping settings fixed for comparison.
Switch/undo a pose and change pose location with a stationary camera, then pan
and orbit while the body is stationary. Finally combine camera movement and
pose changes, then stop the camera. Check both body chains and an add-on.

Existing `gravity responsiveness` logs now include `sample_reason` and cumulative
`sample_counts` for accepted, camera-held and waiting/invalid sampling calls.
Reason values: 0 accepted, 1 camera quiet period, 2 invalid sample/camera,
3 awaiting confirmation, 4 disagreeing direction. Counters reset on source/state
replacement; repeated held substeps can contribute multiple calls. Body trace
state is printed before that call's sampling, so it describes the prior update.
These counters supplement the throttled trace; they do not record every pose.
