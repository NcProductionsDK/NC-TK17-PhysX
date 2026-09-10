# Collision response revision

## Contact-group error refinement rejected and reverted

The user reported the contact-group error trial performed worse in game.
Restored the exact previous DLL, solver source, and baseline test fixture from
`build/collision-backup-contact-balance-20260908-194942/`. The installed and build
DLL hashes both match
`0D866868E64D5AA285C2CCFA5E845EEB06C5862900278565C3F400EB9E96B6EA`.
All baseline collision tests pass. Terminal body coverage, the liveness fix,
and Fiesta's upper-spine configuration are retained.

The rejected implementation, its added experiments, and the latest game log
are preserved in `build/collision-backup-rejected-balance-20260908-195729/`.
The section below is historical; its group-error acceptance rule is no longer
active. Restoring the baseline test fixture also removes that rule's specific
acceptance assertions from the active suite; they remain in the archived trial.

The latest log contains ongoing three-support ordinary contacts and terminal
contacts, including a terminal contact at chain_t=0 on the left clavicle/shoulder
capsule at 19:56:30.415. It does not provide a controlled before/after comparison
or identify the exact visual regression frame. User feedback establishes the
regression; the log does not establish its sole cause. A plausible limitation
of the rejected rule is that it can block useful intermediate corrections in
an animated system whose segments are solved sequentially. Stationary
single-group error reduction proved insufficient as a live quality criterion.
Do not reintroduce this change merely because its isolated tests pass.

## Reject corrections that worsen the current contact group

The user confirmed upper-back clipping was a scope issue and is resolved. The
remaining complaint is a small resting nudge. Current build/source/log snapshot:
`build/collision-backup-contact-balance-20260908-194942/`.

The latest log shows ongoing ordinary/terminal contacts and occasional larger
corrections (for example the ordinary endpoint change at 19:48:58.732). It does
not identify the user's exact nudge frame or distinguish animation from every
solver contribution. The investigation found an independently reproducible
issue: the positional line search checked only the active support, allowing
one accepted correction to increase the group's total squared positive plane
residual. Balanced opposing supports could therefore move away from their best
compromise despite a stationary input.

A new production helper computes that error for all contacts of the current
segment. A candidate now must improve the active contact AND decrease the group
error before it is accepted. Existing step bounds, six-pass budget, velocity
handling, friction, springs, configuration and room algorithms are unchanged.
The comparison uses double accumulation with a 1e-14 numerical improvement
floor in squared coordinate units. This is a line-search convergence condition,
not a sleep timer or a stored resting pose.

The new regression failed before the change: fixture zero shifted 0.000059184
coordinate units and increased squared residual from 0.000242000 to 0.000242007.
After the change, all eight balanced fixtures report zero shift. Additional
regressions verify motion remains possible for unequal opposing contacts and
compatible oblique contacts. Existing settling/release at 30/60/144 Hz, final-tip
response, length/limit tests, and the full collision suite pass.

This safeguard applies to general coordinated sidecar/body contact groups,
including the terminal segment. Separate segments/persons and room response
remain sequential and can still interact. Contradictory contacts may retain
penetration at a lower-error compromise; the bounded solver does not guarantee
the global minimum. More candidate evaluations add some work in multi-contact
cases. Live reduction of the remaining nudge is unverified. Compare rest, small
movement, and rest on the same body pose; watch for both nudging and increased
overlap/sticking.

## Terminal hair segment now participates in body collision

The coordinated-body trial was reported better, with minor buzzing and possible
tail03 clipping while the upper links rested on the body. Snapshot and test log:
`build/collision-backup-terminal-body-20260908-193651/`.

The body loop previously stopped at the final real pivot. Its final segment,
named for tail03, ended at tail03 and could affect only tail01/tail02. The
synthetic extension after tail03 was used by room queries and collision/debug
point collection, but was not submitted by the body response loop. This is a
confirmed coverage gap, not proof of the exact visible overlap the user saw.

The current-frame capture now freezes the existing terminal-axis helper's tip.
The coordinated pose can represent that virtual segment and include the final
joint in its response. Each final-link body pass also queries the extension,
using the same body filters, radius, and configured terminal scale. Ordinary
endpoints are refreshed if terminal response changes ancestors. The tip skips
the ordinary link's previous-segment history, avoiding mixed segment sweep data.
No new terminal sweep/CCD is implemented. Missing/stale terminal snapshots skip
this new query, rather than fabricating a direction or using an invalid fallback.

New `addon terminal body contact` records separately report the terminal
segment's hits, supports, nearest feature, gap, and endpoints every 500 ms.
The regular tail03 record continues to describe the link ending at its pivot.

Tests cover the terminal pose under eight rotated/full/legacy configurations,
its dependence on the final joint (whose rotation does not move its own pivot),
contact separation with the upstream joint fixed, and absent/stale snapshot
rejection. All existing collision tests and DLL compilation pass. Engine-only
terminal-axis discovery still needs live validation; the tests supply a known
terminal axis. This trial fixes body coverage, not the remaining settling buzz.

The user clarified that the apparent clipping was through the upper back/torso.
Fiesta's sidecar previously limited body contacts to head, neck, clavicles, and
shoulders, excluding that area. This trial also adds `spine03, spine04` to its
custom targets to cover the upper back. The previous INI is in the checkpoint.
Its radius 0.025 and terminal scale 1.5 are unchanged. Lower torso and other body
regions remain outside the custom scope. The test log contains
repeated ordinary tail03 corrections and large neck/head overlaps near the
softened attachment region (chain_t about 0.092), so residual multi-contact and
attachment behavior remains under investigation. Room response is unchanged.

## Coordinated body-segment contact trial

Checkpoint of the user-confirmed Fiesta build:
`build/collision-backup-coherent-20260908-191627/`.

`physx_chain_contact.c` adds a pure candidate-pose evaluator. It freezes evaluated
joint pivots and bases before integration, composes each influencing ancestor's
rotation change, and predicts both endpoints of a contacted segment. Body queries
and their re-queries now use that candidate geometry, so contact depths account
for spring integration and earlier corrections. The original per-joint path
remains available when a complete current-frame snapshot cannot be obtained.

The addon angular manifold preserves same-normal supports at distinct points.
The new solve uses the summed derivatives of all influencing joints, updates
each support's residual against the current composed pose, and limits both each
step and total per-joint displacement per call. Normal velocity response uses
the sum of joint contributions, with bounded per-joint friction and subsequent
normal projections. No pose pinning, sleeping, spring-strength, or INI changes
are added. The body visibility/liveness fix is retained.

This is coordinated per contacted segment, not a simultaneous solve of every
body/room/self constraint across the entire chain. Separate segments and collider
persons are still processed sequentially. It uses six positional passes and four
velocity passes; contradictory/limited contacts can remain unresolved. Tangential
friction remains an approximation in joint coordinates, and explicit moving-body
surface velocity is not modeled. Up to eight supports are retained. Long chains
or dense manifolds cost more than the old response; engine performance needs
in-game validation.

Room response algorithms remain on the preceding path. Where the candidate pose
is available, the room query receives both newly predicted segment endpoints,
avoiding a predicted end paired with an old start. This does not yet make room
response a coordinated chain solve. Body-body response is unchanged.

Validation: all existing collision regressions pass. `chain_contact_test.c` also
checks a two-joint candidate against an independent root-to-tip FK calculation
across eight rotated/scaled/legacy/full/inverted configurations; same-direction
supports at distinct lever arms; combined inward-velocity removal; separating
motion; bounded contradictory constraints; fixed output limits; stale snapshot
rejection; fixed link lengths; spring-driven resting and release at 30/60/144 Hz.
The latter harness uses simplified springs and omits TK17 animation/child-drive
evaluation. Its settled endpoint range was 0, 0, and 0.000001737 coordinate units
respectively; these are offline results, not measured game jitter.

The offline analyzer still characterizes defects in retained legacy helpers and
labels them as such. Its main regression suite now verifies the replacement.
Next trial: Fiesta on the body, rest/move/rest, checking buzz, clipping, large
swings, and frame-rate changes. A brief ground check is useful because room-query
endpoints now follow the same candidate pose. No live improvement is claimed yet.

## Offline analysis of the improved baseline

Fiesta's contact-location trial was reported substantially better, with minor
buzz remaining. The confirmed build and latest log are preserved in
`build/collision-backup-analysis-20260908-191112/`. No production code, DLL, or
configuration changed during this investigation.

See `COLLISION-ANALYSIS.md` for two production-function reproductions: contact
depth sampled before integration is applied relative to the advanced solver
pose, and normal-only support merging can discard the stricter angular contact.
`python analyze_collision_solver.py` runs all existing regressions plus sixteen
pose-mismatch cases and eight angular-support cases. All completed successfully.
An isolated absolute-goal experiment demonstrates the first correction principle;
it is not installed or claimed as a full-chain fix.

## Body contact-location response trial

The user confirmed the visible-body liveness fix kept hair responsive through
movement. That build is preserved at
`build/collision-backup-contact-location-20260908-185442/`.

Body segment manifolds now retain the closest hair point with each stored
support. Duplicate/deeper-support replacement updates the matching point. For
the visible addon response, each active support's displacement is solved at
that point through the production output-angle mapping. Endpoint movement is
predicted separately for subsequent queries; contact movement is not mistakenly
used as endpoint movement. The contact gradient still drives velocity response.

The existing manifold dual solve supplies the support shares. Their total is
bounded to the old net displacement magnitude, so opposing/cancelling supports
cannot generate large separate requests. The existing 0.55 response gain and
per-joint step limits remain. This conservative distribution can soften corner
response. Independent-joint predictions and sequential support application are
still approximations, not a coupled full-chain constraint solve; residual buzz,
contact ordering, and multi-support penetration require live evaluation. The
startup path without an evaluated joint basis retains endpoint mapping.

Production-function regressions check half-lever separation and separately
predicted endpoint movement across eight output configurations, fixed link
length, support-point deduplication/replacement, and bounded corner/opposing
requests. Full collision tests pass. Body-body and room response algorithms are
unchanged; shared contact-store return values and dual-weight access preserve
their existing numerical behavior. The confirmed liveness fix is retained.

Next game test: Fiesta tail on the body, rest, small movement, and rest again.
Check for reduced buzz, excessive swings, and visible penetration. No in-game
improvement is claimed until this trial is tested.

## Visible-body false despawn

The 18:41 test lost body collision reports immediately after the 18:41:33.009
`body-chain-colliders despawn` record for Person02. The camera heuristic counted
three almost-stationary root samples (last delta 0.00002977) as stale scene data.
Hair simulation/output continued through 18:42:01. This supports a collider
availability failure, rather than the hair simply sleeping. The log and previous
build are preserved in `build/collision-backup-liveness-20260908-184355/`.

An explicit positive PersonVisible result now overrides this camera heuristic
and releases its quarantine even with a stationary camera. Caller checks for
hidden people, stale scene tracks and invalid root data remain. If engine
visibility is unavailable, the existing heuristic remains; this patch cannot
guarantee recovery in that case. Production-function tests reproduce the tiny
root deltas across camera versions, stationary-camera recovery, and preservation
of the unknown-visibility fallback. Full collision tests pass.

The contact probe also showed contact movement approximately half the endpoint
movement on several neck contacts. That solver issue remains open; this change
addresses collider availability only. Live validation is still required.

## Contact-location diagnostic build

The attachment-fade trial was reported only possibly/slightly better, with
residual buzzing. Its behavior is preserved in this build. Checkpoint:
`build/collision-backup-contact-probe-20260908-183605/`.

Body detection uses a closest point along the segment, but the visible response
currently targets the endpoint. The new `addon contact point probe` debug record
compares predicted movement at the nearest active contact with predicted endpoint
movement under the same joint corrections. It runs at the existing 250 ms trace
cadence and makes no simulation or engine writes. No new collision tuning,
sleeping, body-body response, or room response changes are included.

These are summed independent-joint predictions, not full-chain forward kinematics
or measured engine movement. The nearest candidate must be an accepted hit to
emit a record; it need not represent every support in a multi-contact manifold.
`sampled_joints=0` means no evaluated-basis predictions were available. Use these
records alongside existing effectiveness logs, not as proof of the jitter cause.

Regression checks cover the production output mapping in eight pose/configuration
fixtures: a halfway contact moves half as far as the endpoint under the same
joint rotation, the pivot stays fixed, and probes do not mutate target state.
The full collision test suite passes. Next live test: Fiesta hair resting against
the body for about 15 seconds, followed by a small wearer movement and rest.
This build is diagnostic; no visual improvement is expected from it alone.

## Attachment-contact transition trial

The current body and ground versions were reported improved but still mildly
buzzing. They are preserved at
`build/collision-backup-attachment-20260908-182332/`.

For addon contacts marked as parent attachments, the response previously jumped
from excluded to full strength at segment parameter 0.08. Prior traces included
neck/head contacts around 0.083–0.085. The response now transitions smoothly from
zero at 0.08 to full strength at 0.12. The excluded first links remain excluded;
non-attachment contacts and non-addon handling are unchanged. This deliberately
softens attachment contact within that short band and may permit more overlap
there. It is not a general solution for shoulder-contact oscillation.

Tests verify continuity and monotonicity through the boundary, preservation of
fixed-anchor exclusion, and unchanged unrelated/non-addon contact weights. Tests
and DLL build pass. Ground, friction, and spring settings are unchanged. Another
body-only reproduction is needed to assess visual results.

## Remove legacy escape from the output-aware ground path

The ground trial was reported still violently buzzing. The trace showed a
0.00026 surface correction paired with 0.00303 of artificial sideways escape.
The terminal request now uses the original surface correction for each
output-aware joint; only joints using the legacy fallback receive the escape.
This avoids tilting the numerical solver's contact plane toward a fabricated
lateral displacement. The trace reports zero tangent escape when all applied
joints used the output-aware path. Body response is unchanged.

A regression using those logged values verifies zero lateral request in the new
path and unchanged escape in the legacy path. Tests and DLL build pass; live
ground stability remains unverified. Checkpoint:
`build/collision-backup-escape-20260908-181156/`.

## Output-aware ground-contact trial

The output-aware body build was reported better, with minor buzzing remaining.
It is preserved at `build/collision-backup-ground-20260908-180432/`.

Room joint contacts and terminal contacts now convert world-space lever arms
and correction vectors into the frozen body's coordinates and use the same
production-output numerical solve as body contacts. Terminal correction shares
are applied to the requested displacement before solving, instead of scaling
the resulting nonlinear solver offset. The resulting contact gradient drives
velocity/friction response. Without an evaluated basis, the previous mapping
remains available; with a valid basis, an unsuccessful solve does not silently
substitute a different mapping.

Tests cover the production room vector conversion and output-aware wrapper under
camera rotation and translated world pivots, and missing-camera rejection. They
match equivalent body-space corrections across all eight existing output-mode
fixtures. The complete terminal-chain/engine traversal remains an in-game test.
Body response and spring settings are unchanged in this trial. Residual body
buzzing is still unresolved; spring/contact interaction has not been changed.

## Output-aware body-contact solve

The body-contact trial now uses numerical derivatives of the production output
mapping, rather than assuming that solver direction maps rigidly onto a visible
segment. A pure output evaluator accepts candidate solver offsets and preserves
the configured channels, full/legacy angle modes, root roll inversion, root and
matrix scales, and angle limits. Its rotation matrix function is shared with the
SJoint/TJoint publication writers.

Before integration, the evaluated TJoint basis and previously published local
rotation reconstruct a frozen parent basis in body coordinates. For each
upstream joint, a bounded four-iteration solve searches for improved predicted
separation. Candidate offsets preserve link length; backtracking rejects steps
that worsen the predicted plane residual. Velocity response uses the contact
gradient in solver coordinates. The query's temporary endpoint is advanced by
predicted achieved movement, rather than the entire requested correction.
Missing startup/binding bases retain the previous mapping. No live engine writes
or target mutations occur while probing candidate poses.

The regression suite checks the production angle functions and basis
reconstruction across rotated parent frames, full/legacy modes, inverted roots,
root/matrix scale, blocked angle limits, preserved link length, and immutable
probes. Build and tests pass. The live evaluated-basis timing and multi-joint
composition still need verification in TK17; individual predicted improvements
are not a guarantee of full-chain separation. Runtime performance has not been
benchmarked. Ground response is unchanged in this trial.

Checkpoint: `build/collision-backup-jacobian-20260908-174823/`. The existing
effectiveness trace remains available for a body-only resting-contact test.

## Body-contact effectiveness measurement

The body-mapping trial was reported visually about the same. Response remains
unchanged in this diagnostic build. With debug enabled, the new
`addon contact effectiveness` record compares the first live wearer-body query
of a frame against the accumulated relaxed requests from the previous frame.
It reports requested movement, observed outward/lateral pivot movement, nearest
gap before/after with sweep inflation removed, and whether the nearest collider
label matches. Later iterative predicted endpoints are excluded as observations.
Records expire after 120 ms or a scene-generation change and are logged at most
every 250 ms per target. This measures combined frame behaviour, not an isolated
collision response: animation and closest-point changes also affect the result.
The nearest diagnostic feature can include an ignored attachment contact; its
gap is diagnostic and must not be interpreted as a solved contact unconditionally.

The new record requires another short body-only resting reproduction. Prior
source/DLL checkpoint: `build/collision-backup-response-20260908-134642/`.

## Visible body-pivot response trial

The live trace showed millimetres of solver/pivot movement even when final
velocity was close to zero. The body path queried visible pivots but distributed
body-space correction directly into solver offsets, including the contacted
bone, which cannot move its own pivot. The visible-endpoint path now maps the
correction into preceding joints' solver coordinates using the existing visible
segment mapping. It excludes the contacted bone and retains friction and 0.55
relaxation. Temporary query endpoint advance uses that relaxation too; it is
still a prediction rather than a measured engine result.

Tests verify preceding-bone movement, preservation of the contacted bone and
link length, and fixed-anchor rejection. Full live-skeleton behaviour remains
unverified. Ground response is unchanged in this trial and remains unresolved.
The diagnostic trace remains enabled by `defaults.debug` for comparison.
Checkpoint: `build/collision-backup-bodymap-20260908-133821/`.

## Live-motion measurement build

This build keeps the friction trial's response unchanged and adds debug-only
`addon contact motion begin/final` records for colliding skinned chains. Records
are throttled to one pair per target per 250 ms. They compare the engine pivot
step, previous published rotation, final solver displacement, corrected flag,
velocity, and new published rotation. Pivot sampling occurs before integration;
it does not assume engine pivots update synchronously after a matrix write.
No forced engine evaluation or pose writes are added by the trace.

Reason: body collision's temporary end override advances by the requested push,
while solver corrections are distributed and length-constrained. This endpoint
is not a measurement of the resulting visible pivot. Ground contacts follow a
different mapping, and repeated ground penetration remains in the latest log.
The new measurements are intended to distinguish mapping/publishing issues from
spring/contact oscillation before changing solver behaviour again.

With `defaults.debug=true`, reproduce body resting contact for about ten seconds,
then ground resting contact for about ten seconds. The existing log is sufficient;
no upload is required in this shared workspace. Prior DLL/source checkpoint:
`build/collision-backup-measure-20260908-133153/`.

## Contact friction trial

After continued buzzing was reported on both body and ground, the latest log
showed repeated floor support corrections alongside sideways tip motion. The
link velocity response had no friction. It now applies Coulomb-style friction
with coefficient 0.25, capped by the inward impulse removed during that contact.
This is a targeted trial for the residual tangential oscillation, not a confirmed
explanation of every remaining nudge. No pose is pinned and no sleep is added.

Tests verify the exact friction bound, no velocity reversal, preservation of
separating motion, no extra damping from a duplicate contact query, and matching
one-second sliding deceleration at 30/60/144 Hz. Existing regression checks pass.
Small sliding motion can now stop under support; larger sliding motion loses
only the bounded friction impulse. Live moving-collider response still needs
in-game validation. Dedicated body solver policies are unchanged in this trial.

The pre-friction source and DLL are preserved at
`build/collision-backup-friction-20260908-132218/`.

## Resting-contact length calculation fix

The subsequent debug log showed tiny sidecar corrections reporting lengths near
0.00391. The production `physx_sqrtf` performed eight Newton iterations starting
at one for small inputs. For tiny positive squared lengths, it returned roughly
1/256 rather than the actual square root. This distorted distances, contact
normal normalization, small-motion thresholds, and velocity response at rest.

It now uses the standard `sqrtf`. This shared helper is also used by body physics;
the change is not restricted to sidecars. No damping or sleep policies were added.

The earlier room/sidecar test fixtures substituted an accurate length function,
which concealed this defect. They now extract the production length helpers.
With the old helper, the room corner regression fails; with the corrected helper,
it passes. Added checks cover lengths from 1e-8 to 10000, room corrections of
2/10/100 microunits, and shallow sidecar contacts cancelling inward velocity while
preserving sliding. All collision regressions pass with the production helper.
The complete live-game resting behaviour still needs verification.

The DLL and source before this fix are saved at
`build/collision-backup-length-20260908-131036/`. This is the most recent rollback
checkpoint, preserving the second version reported improved in-game.

## Follow-up after the first in-game trial

The first revision was reported substantially better in-game, with small
remaining hair nudges against both body and room surfaces. The follow-up makes
two targeted sidecar changes:

- Remove a remaining margin gate that disabled the entire shallow body-contact
  band immediately above the slop threshold. The continuous depth curve now
  receives the actual margin throughout that band.
- Mark contact-corrected solver targets and use their solved velocity for child
  motion inheritance. Previously the ordinary output path divided corrected
  displacement by frame time, turning depenetration into a new child impulse.
  This applies to sidecar body, room, and terminal corrections. Free-motion
  velocity estimation retains its existing behaviour.

The production-path regressions cover both changes, including contact release
and velocity inheritance at 30/60/144 Hz. Tests and syntax checks pass; visible
results of this follow-up still require another in-game trial. No dedicated body
solver changes or sleep settings were added in this follow-up.

The first improved DLL and its source checkpoint are saved under
`build/collision-backup-improved-20260908-125706/`. Copy that directory's
`NC-TK17-PhysX.dll` back to the game Binaries directory with TK17 closed to return
to the version confirmed better in-game.

## Findings and changes

The Fiesta ponytail uses three simulated tail bones, a 0.025 collision radius,
and a terminal extension of 1.5 link lengths. Its custom body selection includes
the head, neck, clavicles, and shoulders. The Backyard Pool room selects its
grass and floor meshes for collision. These addon configurations were not changed.

Several response problems were present in the source:

- The sidecar/body-chain soft-contact curve dropped from a nonzero correction
  to almost zero when penetration crossed the configured slop threshold.
  The shared response curve is now continuous and increases with penetration.
- Sidecar contacts merged substantially different normals and summed the
  resulting pushes. The replacement retains up to eight supporting planes and
  solves for a small displacement satisfying them, deduplicating coplanar
  contacts. Position corrections no longer blend in previous-frame normals.
- A link was pushed out of contact and then normalized to its authored length,
  which could put it back inside the surface. The new projection satisfies a
  contact plane on the fixed-length sphere. Room visible-to-solver mapping uses
  that projection before converting the desired direction back to solver space.
- Removing normal velocity and subsequently removing radial link velocity could
  recreate inward motion. Velocity response now removes radial motion first,
  then removes inward contact motion in the link's tangent plane. Outward motion
  and sliding are retained instead of damping all components per contact query.
- Room sphere queries previously returned only the deepest triangle. They now
  solve multiple supporting planes in one BVH traversal, including floor/wall
  corners. Existing sweep sampling and entry-side recovery remain in place.
- Sidecar room contacts automatically pinned poses and cleared velocity after
  a few contact frames. That automatic pose pinning has been removed. Terminal
  response distributes a normalized share of the correction through upstream
  joints instead of each joint applying 38–72% of the same correction.
- Late contacts changed upstream solver offsets after their visual poses had
  already been published. Skinned chains now republish changed upstream poses
  after the chain solve, using the same output mapping and angle limits as normal
  output. The late positional correction is not converted into inherited velocity.

Body-chain/body contact benefits from the shared response-curve correction.
Dedicated breast, butt, and other body solvers retain their existing response and
rest policies; all users of room sphere queries receive the multi-plane query.
This is not a replacement of every physics solver in the plugin.

## Automated validation

Run from this directory with the existing MinGW32 toolchain and Python:

```powershell
python run_collision_tests.py
.\compile-physx.bat
```

The test runner compiles the shared contact math and extracts the actual room
query, sidecar correction, visible mapping, and final-publication functions from
the production source. Engine services are replaced by small test fixtures.
It checks response continuity/monotonicity, duplicate and oblique supports,
corner separation, contact order, link length, sliding, release, singular cases,
thin-floor sweep, and ten seconds of resting contact at 30/60/144 Hz without
sleep or contact damping. Publication tests check late upstream updates, stale
target rejection, duplicate-write avoidance, and velocity handling.

The tests and 32-bit DLL build pass. They do **not** simulate the complete live
TK17 skeleton, animation hooks, skin deformation, moving collider history, or
the entire terminal-chain solve. In-game visual stability remains unverified.

## In-game checks still needed

The built DLL was installed to `The Klub 17/Binaries/NC-TK17-PhysX.dll` on
2026-09-08. The previous installed DLL is preserved at
`build/collision-backup-20260908-124707/NC-TK17-PhysX.dll`. Build and installed
DLL SHA-256 hashes matched after copying. To roll back, close TK17 and copy
that backup over the installed DLL.

1. Restart TK17 with the revised DLL and Fiesta hair. Rest the ponytail against
   the head/shoulder colliders, then move the head slowly and release the contact.
2. In Backyard Pool, lower the ponytail onto the floor and an edge. Watch for
   persistent penetration, sudden folding, or frame-to-frame motion while still.
3. Move the person slowly along and away from the floor contact. The tail should
   respond without waiting for the previous room sleep threshold.
4. Repeat body/body contacts and body/room contacts to check regressions, then
   repeat the hair case at different frame rates and with another person nearby.

Impossible contacts (an anchored link with no reachable separating pose), deeply
overlapping opposite-facing surfaces, the eight-plane contact limit, and the
existing sampled sweeps still impose limits. In particular, removing pose pinning
can expose errors in live collider/visual mapping that the old sleep hid. The
numerical tests establish the listed solver properties, not a guarantee that all
visible buzzing in the game has been eliminated.
