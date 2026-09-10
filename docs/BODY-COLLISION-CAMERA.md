# Body collision camera protection - 2026-09-10

Status: fixed and accepted by the user after in-game testing of the 160 ms
follow-up. Slight penis-chain movement can remain during unusually aggressive
camera motion; the user accepts this limitation and requested no further tuning.

The user confirmed substantially improved breast/butt collisions, but reported
camera-driven motion in breasts, penis and testicles (butt uncertain).
The tested DLL and debug log are preserved in
`build/collision-backup-single-bone-camera-regression-20260910/`.

## Evidence and limits

In that log at 19:50:56.043, Person02's penis receives three room contacts with
maximum reported penetration 0.05 and collider motion 0.05325. The subsequent
write shows substantial angular velocity even though movement input is zero.
At 19:50:56.079-080, breast and both butt systems also report strong corrections.
Gravity targets and geometric gravity directions remain effectively constant
through this camera-active interval. Thus the recorded force burst comes from
contacts; it is not evidence of a gravity-direction jump.

The old body-to-room conversion combined live model-view skeleton coordinates
with the latest independently captured camera inverse. The single-bone path
also used that conversion for its source/target centers and parent axes.
The log does not record the exact camera epoch of every engine matrix, so it
cannot prove that every visible disturbance has this one cause. A regression
fixture reproduces false movement when these updates arrive at different times.

## Change

`physx_collision_frame.c`/`.h` retain each person's trusted TRS_group placement
in room space. Live bone coordinates are converted relative to the live TRS
matrix and then through that trusted placement. Source centers, target centers,
translation axes and room correction vectors use matching conversions.

Placement acquisition waits for at least 160 ms of camera quiet and two agreeing
samples from different simulation frames in the same camera epoch. Once
confirmed, ordinary placement motion with a stationary camera updates directly.
While the camera moves, local skeletal animation and physics remain live;
only the person's global placement is held. A simultaneous change of pose
location is therefore reflected after camera quiet and confirmation. Missing
initial data does not fall back to using view coordinates as world coordinates.
Source or named-node generation changes invalidate the cached placement.

The new mapping is used for breast/butt contacts and the penis/testicle room
queries. Chain dynamics, body-local chain contact solving, gravity, sidecar
paths, shared room geometry/sweep routines, config and addon files are unchanged.
The prepared release ZIP includes the accepted follow-up DLL and this document.

With debug enabled, `body collision frame` logs availability, camera version,
hold state and the difference between raw and retained room placement, once per
second per person. This gives the next game test evidence absent from the old log.

## Validation and next game test

Ten regression suites pass. `run_collision_frame_tests.py` extracts production
sampling and point/vector conversion code. During simulated asynchronous camera
orbit/pan, the old conversion has over 0.05 scene units of error; the protected
conversion remains within 0.00002 units, including live bone movement. It also
checks confirmation, same-frame rejection, global placement changes, source
loss and scene generation changes. Single-bone contact integration, chain
contacts, gravity, motion, binding and sidecar regression fixtures pass.

These tests use synthetic engine matrices. The subsequent user gameplay
confirmation and accepted limitation are recorded above. TRS and bone model-view matrices are assumed
to belong to the same skeleton update; the new diagnostics help investigate
remaining runtime timing problems if that assumption does not hold.

Installed candidate SHA256: 66947B4DCE74460C3FAFB160A6BAC4DFCF43BF44FF628EE2E9E5B42BABF39C35

## Aggressive camera follow-up

The user confirmed no obvious camera contamination with the above candidate,
but saw slight penis movement during aggressive camera movement. That candidate
and the new log are backed up in
`build/collision-backup-before-camera-settle-followup-20260910/`.

At 20:15:43.412 the log records Person02 accepting room placement
`(2.44681,0.54086,-0.05561)` at camera age 141 ms. Once quiet, it returns to
`(2.46247,0.60423,-0.02935)`. In this interval, penis gravity and movement input
stay steady, with room-contact corrections and subsequent residual velocity.
This suggests that the original 48 ms quiet period can allow late skeleton
updates through; two agreeing samples alone can both be stale. The throttled
log cannot establish the exact settling duration or prove that every contact
in the session was camera-driven.

The follow-up changes only the collision placement quiet minimum from 48 to
160 ms, retaining the two-sample confirmation. This is a conservative timing
adjustment, not an exact engine frame synchronization fix. It adds 112 ms to
the earliest placement reacquisition after camera movement. Local bone motion,
gravity timing and normal pose-location tracking with a stationary camera are
unchanged. If camera movement and a global pose-location change coincide, that
placement change waits for the longer quiet period.

A new synthetic late-update fixture fails with the old minimum and passes with
the new one: agreeing displaced samples through 144 ms cannot move the collision
frame, and correct samples at 160/176 ms resume tracking. Camera-frame,
single-bone, gravity-response and body-contact suites were rerun successfully.
Gameplay verification is complete: the user reported further improvement and
accepted the remaining slight movement under unusually aggressive camera motion.

Installed follow-up SHA256: 1BD25F675D103F494CE73D43A7AEE94264ECC4EC2CAEBCAA9CC75CC1F15297E6
