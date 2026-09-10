# Pose/undo gravity responsiveness investigation — 2026-09-10

The newer [gravity sampling overhaul](GRAVITY-SAMPLING.md) supersedes the hold
described below. This document preserves the investigation and earlier builds.

## Previous: camera regression correction

The subsequent [second-room log review](GRAVITY-RESPONSE-ROOMTEST.md) supports
the hold correction in sampled data and records remaining uncertainty, sidecar
observations and options for a snappier response. No tuning changed in that review.

All eight regression suites and the full DLL build pass. Built and installed to
`The Klub 17/Binaries/NC-TK17-PhysX.dll`, with matching SHA256:
`2589A63A57CC8A71E5ACB139331EBA1C38FB65D87025A8489EAEFF8440A52F4A`.
The user's configuration, including debug, and the accepted ZIP are unchanged.

The first responsiveness build was rejected in-game: faster response came with
camera contamination. Its DLL, source, tests and 22 MB log are preserved in
`build/collision-backup-gravity-response-regression-20260910/`.

The correction restores the full-camera gravity guard from the confirmed
checkpoint. Rotation-only versions remain diagnostic. Geometric gravity now
also honors the per-body `gravity_camera_hold_active` flag. Previously it checked
only the global collider pivot hold, whose production implementation returns
false. Thus it could refresh while the primary gravity target was held. A new
regression fails on the prior implementation and passes with this correction,
including direction refresh on release. Springs and inertia keep running.

This prioritizes camera isolation. The original delay can return; safely
replacing the long hold still requires coherent pose/camera sampling evidence.
Gameplay validation is pending.

### Findings in the 14:56-15:03 debug session

- At 15:03:01.365 Person02 gravity releases on a 0.193841-unit view-space root
  displacement with rotation age 516 ms, while full camera movement was still
  being recorded. Person01 similarly releases on 0.203879 units. The rotation-only
  gate let the raw-root release heuristic run during camera translation. Camera
  translation can cause that displacement; the log cannot prove whether the body
  also moved at that instant.
- At 15:02:58.913-15:02:59.913 Person02 has zero reported body input/contact
  correction and unchanged main gravity target, yet root bend changes from
  -35.562 to -34.074 degrees during camera activity. This is consistent with
  another force path or residual motion. Source review and the regression prove
  the geometric-gravity hold gap; the old log did not record its direction, so
  it cannot attribute every movement.
- Of 600 diagnostic rows, 533 were sidecar rows, predominantly room palms. Only
  three Fiesta-tail rows were captured (14:58:43-45), all valid and unheld, before
  the later test. The trace now excludes ownerless room chains and placeholder
  roots. With explicit debug enabled it continues at one-second intervals beyond
  the normal budget. Body rows also record retained geometric-gravity direction.
- Eight of 98 sampled testicle-health rows have update gaps above 100 ms; the
  largest is 4953 ms. Loading, UI pauses and logging overhead are possibilities,
  not established causes. This is not a measurement of steady-state FPS.

Restart TK17 and compare camera pan/orbit with the body stationary, then pose/undo.
With debug enabled, the test need not fit into three minutes.

## Historical first attempt (rejected)

Status of first attempt: rejected after the user reported camera contamination.
The user reports delayed response after pose changes/undo/location changes,
affecting body physics and sidecars. Moving the body may restore response.

All eight regression suites and the full DLL build passed. Installed to
`The Klub 17/Binaries/NC-TK17-PhysX.dll`, matching the build SHA256:
`5E53A642C56B64098C6E0A5632EDE01CF95FA926E04C89F857DB6CF304D9445B`.
Game configuration and the accepted release ZIP remain unchanged.

## Evidence and scope

The current game log contains startup/activity events only. It cannot establish
the cause of a particular stalled pose. Source review found a live body-gravity
hold of `max(camera_quiet_ms, root_quarantine_ms) + 1600 ms`. Its early release
depends on root displacement exceeding a threshold. Camera version changes
previously included translation, although projecting a gravity direction depends
only on rotation. Camera repositioning could therefore needlessly start or extend
this hold. This applies to consumers of the common body gravity probe, including
sidecar fallback. The sidecar's primary parent-gravity path already bypasses its
old hold code, so that timer cannot explain primary-path sidecar delays.

## Targeted change

- Camera capture now tracks rotation changes separately using the nine rotation
  entries, with the same change threshold as the existing camera signal.
- The promoted body-gravity hold uses that rotation version and timestamp.
  Camera translation does not start a gravity hold or extend an existing one.
- Position/root-drive guards still use the full camera version. Actual camera
  rotation retains its previous quarantine, hold duration and body-release path.
- Startup baseline probing, gravity smoothing, springs, contact response and
  primary sidecar gravity behavior are unchanged. This does not claim to resolve
  every reported pose-transition delay, particularly during camera rotation.

The preceding DLL, source, tests, log, config and accepted release ZIP are saved
in `build/collision-backup-before-gravity-response-20260910/`. The accepted release
ZIP remains unchanged while this build is evaluated.

## Bounded test trace

This test build admits `gravity responsiveness` rows into the normal log without
requiring debug mode. There are at most 600 rows, one per second per sampled body
state or sidecar, for the first three minutes after gravity sampling begins.
Restarting TK17 starts a fresh trace. The trace is diagnostic only and should be
removed or made debug-only after the remaining delay is diagnosed.

Body rows record promotion/hold state, full-camera and rotation-only versions and
ages, root length, a read-only live root projection, accepted target and filtered
target. `live_root_unsigned` omits configured signs/curves/reference subtraction;
it is an orientation observation, not a direct expected target value. Custom
gravity basis nodes can differ from the traced root. Rows are sampled before the
current probe update, so an ordinary one-frame difference is expected.
Sidecar rows record gravity validity, binding and selected drive. A flat drive
despite a pose change, or a changing drive with unchanged visible motion, points
to different follow-up investigations. One-second sampling cannot identify every
short transient.

## Verification and next game test

`python run_gravity_response_tests.py` compiles the actual camera capture hook,
direction projection and promoted-gravity hold block. It covers translation-only
camera changes, retained rotation holds, movement release, directional projection
and quiet-camera updates. It does not reproduce engine traversal timing.

After restarting TK17, load a scene and reproduce pose switching, Ctrl+Z and pose
location changes within the first three minutes of active physics. Compare one
change with a stationary camera, then camera panning and orbiting. If gravity
appears stuck, briefly leave the body still before moving it so the log can show
the transition. Note which sidecar/body part appeared delayed. The existing
camera-isolation harness remains available; this change does not drive the camera
or modify the pose automatically.
