# Breast/butt contact candidate — 2026-09-10

Status: improved contacts and the subsequent camera fix are accepted in game.
Slight penis-chain movement during unusually aggressive camera motion remains
an accepted limitation.
The camera-protection follow-up and current validation are described in
[BODY-COLLISION-CAMERA.md](BODY-COLLISION-CAMERA.md). Details below record the
original single-bone candidate before that follow-up.
The confirmed release, DLL, sources and game settings are preserved in
`build/collision-backup-before-single-bone-fix-20260910/`.
The release ZIP now includes the accepted follow-up. The DLL hash below identifies the original
single-bone candidate; the accepted follow-up hash is in BODY-COLLISION-CAMERA.md.

Built and installed to `The Klub 17/Binaries/NC-TK17-PhysX.dll` with matching
SHA256: `D8E7CEF6F8759EF96D22D7977C8BE480C9507999E8E8170EEC667D3715B5C265`.

## Implementation

`physx_single_bone_contact.c` and `.h` replace the old breast/butt displacement
sum and contact-damped spring targets with a translation-only constraint solve.
They are separate from the accepted chain and sidecar contact implementations.

- Body spheres and room planes meet in world space. Contact normals are mapped
  into the real bone parent's local translation coordinates using its full
  scaled matrix. This removes the former h/v/s versus authored-axis mismatch,
  and avoids the fallback that copied an unconverted offset into local axes.
- Free translation targets remain separate from positional collision recovery.
  Contacts impose separation constraints on the predicted sphere position.
  Body and room constraints are solved together, with equivalent normals
  deduplicated. Room contacts retain their individual planes instead of supplying
  one summed displacement. Body contacts retain individual finger/hand supports;
  the former averaged hand direction is no longer needed.
- Recovery respects each configured translation limit. Contradictory or
  unreachable contacts retain a bounded, low-residual result rather than growing
  the allowed deformation. Residual overlap is logged. Normalized body response
  strength 1 gives full recovery where feasible; 0 disables body response.
  Values below 1 give gradual recovery; values above 1 are capped at 1 for this
  single-bone response, avoiding an exaggerated push past separation.
- Contact removes inward translation velocity. Tangential and separating
  velocity survive. Positional recovery does not generate launch velocity.
  The old 120 ms all-axis critical-damping hold is removed.
- Translation integration uses bounded substeps over up to 100 ms of elapsed
  time, with implicit damping and velocity rebasing after longer stalls. Contact
  constraints are recomputed from predicted translation in each substep, with
  up to three geometry passes. Angular/free-rotation integration and its older
  time cap are unchanged; that broader motion work is outside this collision fix.
- Single-bone pairs solved later in the frame can see earlier pair positions
  through a private cache, avoiding two full recoveries against the same stale
  overlap. Global body colliders and chain geometry are never overwritten by it.
- Room sweeps start from the previous observed center, not a requested correction
  that the engine might never have reached. History is rejected after 120 ms,
  parent/generation changes or movement above 0.75 scene units. Existing room
  sphere/sweep functions used by chains and sidecars are unchanged.
- Contact-only translation is published even if ordinary bone translation is
  disabled; restore/handoff still returns the original authored translation.

The old single-bone summed-offset functions, hand-grouping helper, damping-hold
fields, tracked-room wrapper and unused world-vector conversion were removed.
Movement-input conversion helpers remain unchanged for their original purpose.
The historical audit still reproduces the old invalid calling convention;
it does not exercise the replacement runtime path.

## Validation

Nine regression suites pass, including `run_single_bone_contact_tests.py`.
The new suite extracts the production integration, body query, parent mapping
and output functions. It checks duplicate/corner supports, body and room resting,
moving-support release, combined supports, impossible displacement limits,
contact-only output and restoration, stale room history, source generation
changes, scaled/mirrored parent mapping and camera rotation invariance.
Timing cases include 50/33/16/7 ms and alternating 11/23 ms updates.
The existing room fixture also verifies that the new production room-plane
collector preserves distinct floor/wall supports and deduplicates triangles.

Source comparison against the backup confirms unchanged penis/testicle update
functions, chain/contact/motion/dynamics helpers, sidecar code, and shared room
sphere/sweep queries. The accepted eight regression suites continue to pass.
These tests use synthetic engine geometry; they do not prove runtime matrix
freshness, final mesh placement or visual quality in every pose.

## Game test and limits

The game configuration's `breasts_collision_enabled` and
`butt_collision_enabled` are enabled for this test. Their former values were
false. Existing room-collision flags, gravity, scope, radii, movement limits,
debug setting and addon configurations are retained. Default/public config and
the accepted release ZIP are not changed.

Restart TK17. Test breast and butt contact with hands/other body regions allowed
by the configured scope, then resting against room surfaces, release, pose undo
and camera movement. Contact records use the `single-bone contact` prefix and
are throttled to once per second per system/person while touching, even with
debug off. They include predicted versus observed world centers, translation,
recovery and combined remaining violation in local output units. With debug on,
they also report no-contact/invalid-source frames at the same throttle.

The solver treats external supports as kinematic within an update; it does not
transfer physical momentum from a moving hand or couple all breast/butt masses
in a simultaneous dynamics solve. Displacement limits, sphere proxies, excluded
attachment regions, contact-capacity bounds (32 local / 8 room) and unreachable
poses can still leave mesh/proxy penetration. Very fast body-sphere crossings
do not gain continuous body collision detection in this change. Room sweeps
retain the existing endpoint fallback when their starting sphere is embedded.
The previous sampled center is assumed to correspond to the prior published
translation; the new observed/predicted log fields help audit that assumption
in game. No full soft-tissue deformation is implied.
