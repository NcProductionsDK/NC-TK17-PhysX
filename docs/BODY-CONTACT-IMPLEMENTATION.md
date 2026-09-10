# Confirmed body-chain contact response

The current working DLL additionally contains accepted body movement timing and
progressive joint stops, plus the accepted [gravity/link-inertia changes](BODY-DYNAMICS.md).
See also [BODY-MOTION.md](BODY-MOTION.md) and
[BODY-LIMITS.md](BODY-LIMITS.md). The confirmation and hashes below describe
the preceding collision checkpoint. Its release ZIP is preserved in the
gravity/inertia checkpoint backup; the current package includes the motion changes.

Status: confirmed in-game on 2026-09-09. The user reports that the last remaining
body collision problem is resolved. This extends the confirmed sidecar/free-motion
checkpoint. The updated release package contains both sets of improvements.

The exact confirmed source, DLL, tests and game log, plus the earlier release
ZIP, are saved in `build/collision-backup-confirmed-body-20260909/`. Cleanup
removes unused `collision_contact_filter` storage/resets and raw matrix rows
from pose diagnostics. Those rows were never validated as rotation bases and
are not used by the composed solver. Bounded pose/output diagnostics and their
replay tools remain useful and are retained. No solver equations, collision
limits, camera behavior or user configuration are changed by this cleanup.

Cleanup build installed with matching source-build/game DLL SHA256:
`BF89A22429DFA4CB75A556692FF9E846EDB81936A26D9FF2ADD3C28A335F4FC4`.
Body-contact, recorded body-pose, collision, free-motion and binding suites pass,
as does the complete DLL build. The gameplay confirmation applies to the exact
pre-cleanup DLL below; the cleanup build is regression-tested.

The first candidate (`820654DB...F2E5A5`) failed the upward-facing game test.
The user confirmed the bounded-response follow-up was much better, with
occasional large reactions and testicle buzzing remaining. The subsequent
testicle/cross-sample follow-up was also confirmed much better, with occasional
thigh entrapment and minor testicle buzzing. The contact-stall follow-up still
failed in-game. The subsequent diagnostic capture identified a rotation-model
error; the composed-pose correction below was confirmed in-game.

## Composed body-joint prediction (2026-09-09)

Built and installed to `The Klub 17/Binaries/NC-TK17-PhysX.dll`, with matching
SHA256: `23D561D30C85CFCE094DBDA12DBD349E271F849EE619C853C80B2B0816A0AFC1`.

The diagnostic game test contains 256 penis pose comparisons and four testicle
comparisons. Horizontal-only bends agree with the old prediction, but combined
horizontal/sideways bends can move in the opposite direction. The largest
recorded discrepancy is 0.0770663 scene units. A solver can reduce its predicted
contact error and still drive the actual chain deeper into a thigh when the
rotation mapping is wrong.

`physx_body_pose.h` now composes each joint's actual local Euler rotation,
authored planar orientation and parent transform. Link lengths and authored
orientations are recovered from the pre-step engine pivots and published Euler
values, without constants fitted to this person's dimensions or angles. Contact
queries, finite-difference Jacobians, correction trials and normal-velocity
response share this prediction. The position correction still never becomes a
velocity impulse; limits and correction budgets are unchanged.

`python run_body_pose_tests.py` checks 16 sanitized recorded samples retained in
`tests/body_pose_recordings.json`. Passing a diagnostic log replays every complete
sample. The full capture's 260 poses match their subsequent observed engine
points with a largest error of 0.000000572 scene units. Only the pre-step sample
is used to recover the model; the later observation is used for verification.
This establishes pose-prediction agreement for the captured bends, not complete
collision recovery or resting behavior in-game. Testicle coverage is limited to
four captured poses.

`run_body_contact_tests.py` additionally checks that composed proximal/distal
support corrections produce the predicted pose when published. Two- and
three-link composed chains under synthetic spring pressure settle on a plane
at 30/60/144 Hz and uneven timesteps while bent sideways; maximum settled tip
variation is below 0.00000002 scene units. The legacy fallback tests and existing
collision, free-motion and binding suites also pass.

The composed model activates only for live samples in the standard root Y/Z/X
collider basis, when the observed link directions agree with the model's pitch.
Held/camera-quarantined samples, unsupported rigs/bases, degenerate lengths and
near-singular pitch use the existing approximation. These fallback cases and
sequential coupling between the two dynamic chains remain limitations. The
pose-audit log includes `composed=1` when the preceding prediction used the new
model. Historical raw rows are not used as joint bases and were removed from
new diagnostic records during checkpoint cleanup.

Source distributions must include `physx_body_pose.h`. No configuration or
sidecar behavior is changed by this correction. The prior diagnostic DLL,
source and full capture are preserved in
`build/collision-backup-body-composed-pose-20260909/`.

## Engine-pose audit (2026-09-09)

Built and installed diagnostic DLL SHA256:
`207CFF6DF63772022A1D6AFF526D5E2E13807DDBEDC869FEBCEF7DAD764D4DEA`.

The contact-stall candidate did not resolve the reported thigh penetration or
testicle resting difficulty. The log ending at 09:01:05 repeatedly predicts a
large reduction in contact-plane error while the subsequent engine samples
remain deeply embedded. At 09:01:04.178 the nearest thigh margin is -0.09441,
and the predicted squared error falls from 0.005581091 to 0.002139199. Neither
this nor the standalone tests establish correct rendered joint motion.

That build was diagnostic only: it retained the current response and added
`body-contact pose-audit`/`pose-joint` records before replacing each step's
snapshot. It compares the previous step's prediction (including the applied
angle change) against the new live/held sample. Records contain sample ticks,
camera hold status, points, simulation/output angles and available raw joint
basis rows. The rows are observations; their interpretation as an output frame
has not been validated. Tracing requires debug/collider diagnostics, valid
engine samples less than 100 ms apart and previous penetration >=0.01. It is
bounded to 256 records per chain state, plus two/three joint lines per record.
The comparison is observational and introduces no pose writes or solver tuning.

`analyze_body_pose.py [log]` summarizes these records without modifying files.
The body test suite verifies the metric distinguishes opposite motion, exact
agreement and held geometry without mutating inputs. Full engine traversal
remains outside these tests. The next game capture established the combined
rotation mismatch described above.

Previous source, DLL and failing log:
`build/collision-backup-body-pose-audit-20260909/`.

The improved bounded-response DLL had SHA256
`47942241F54027DF42DE824929B9AB4A0F857B7DE5CE436FCA5817958FA0561E`.

## Testicle and cross-sample follow-up (2026-09-09)

Built and installed to `The Klub 17/Binaries/NC-TK17-PhysX.dll`, with matching
SHA256: `480B6F429A96752391F3B25298E835C7CF0B9E5402C3E75645E809B86C02E337`.

## Contact-stall follow-up (2026-09-09)

Built and installed to `The Klub 17/Binaries/NC-TK17-PhysX.dll`, with matching
SHA256: `64E5811CD75E6F334A428F304BCD888EDF08C95EAEBD6DC042B10421E913C523`.

The log ending at 08:36:14 contains six penis samples with substantial contact
error and effectively zero correction. At 08:35:02.833 there are six contacts,
squared error 0.015349885, and no accepted correction. The nearest collider is
the left hip/thigh capsule. The user reported the chain getting stuck in a
thigh. These records establish a solver stall, but cannot reconstruct the
complete contact manifold or every visible testicle nudge.

- After the existing plane projections, the solver now tries up to eight
  refinements along the combined contact-error gradient. Each trial respects
  the same joint and whole-chain limits, takes at most a two-degree local step,
  and backtracks if actual predicted overlap does not decrease. This lets
  conflicting contacts make a smaller combined move when each individual full
  correction would be rejected. Velocity response uses the accepted result.
- A prior contact point may reverse the radial normal only if it was outside
  the collider (allowing collision slop). An already embedded history point
  cannot reverse the nearest outward direction. The existing exterior-entry
  protection and degenerate-centre fallback remain. This rule applies to both
  body chains; no sidecar solver or configuration was changed.

A two-contact production-solver fixture previously stayed at squared error
0.000052 although a combined step could reduce it to 0.000050. It now makes
that improvement. An independent synthetic fixture using the recorded chain
shape and two thigh capsules escapes in four bounded updates. This recovery
fixture isolates geometry and correction; it does not include engine traversal,
gravity or moving supports. Normal-history, joint limits, release, deep overlap,
resting at multiple rates, collision, free-motion and binding tests pass.

This does not establish complete removal of game penetration/buzzing. The
sampled geometry, existing Euler mapping and sequential dynamic-chain updates
remain limitations. Previous DLL, source and this game log are preserved in
`build/collision-backup-body-stall-20260909/`.

## Testicle/cross-sample evidence and changes

The log ending at 08:16:21 records a testicle second-joint angle changing from
-25.477 to +13.309 degrees in successive one-second records while the root is
stationary. Joint-limit saturation and repeated contacts are visible. The log
is sampled; it does not identify every visible nudge or establish a complete
frame-by-frame causal sequence.

Three source defects were addressed:

- Active testicle geometry treated the two passive sphere centres as rotation
  pivots, then extrapolated farther to form its second segment. Those centres
  already contain fine offsets. The active chain now uses the validated real
  joint01, joint02 and terminal pivots cached with the passive sample. A held
  sample holds both representations. The unused fourth point repeats the tip.
  Passive sphere geometry and its INI fine offsets are unchanged. Active-chain
  queries wait for valid sampled geometry instead of inventing an angle-only
  shape if it expires. The first 45% of the first testicle segment is excluded
  from active contact response near the fixed attachment, including when it is
  the opposing chain; the pre-existing penis cross-attachment exclusion remains.
- Velocity into an individual joint limit remained after angle clamping. The
  contact projection could transfer that unavailable motion to a child joint.
  A production-solver fixture at the testicle root's 7.5-degree limit showed
  100 degrees/second of blocked root velocity becoming 20 at the root and -40
  at the initially stationary child. Both chains now discard blocked velocity
  after spring/limit integration, before collision response, and after contact
  velocity projections. Velocity away from the limit remains available.
- The active penis cross-query used angle-only geometry when its candidate was
  from another tick, despite a valid engine sample being available. With this
  session's 0.5-unit fallback link lengths, that could replace the visible chain
  with a 1.5-unit chain. It now uses the current sampled candidate or a valid
  live/held engine centreline. Expired geometry is rejected. This fixes the
  fallback discontinuity; the two dynamic chains still update sequentially.

`run_body_contact_tests.py` exercises actual query/solver functions with the
recorded testicle pivots, real child rotation, held/expired/invalid samples,
staggered chain ticks and the configured long fallback lengths, and blocked
root velocity/release. The existing rest/rate/overlap tests and the collision,
free-motion and binding suites pass. These checks do not reproduce full game
traversal or validate the remaining root-local Euler approximation.

Previous source, DLL and test log are preserved in
`build/collision-backup-body-testicles-20260909/`. The confirmed sidecar release
archive remains separate. Retest testicle support against body and penis,
slow movement/release, and the upward-facing pose.

## Upward-facing instability follow-up (2026-09-09)

The game log ending at 23:44:19 records stationary root inputs with large
penis angle excursions (including -132.460 degrees), and repeated reversing
contact corrections near the active testicle chain. The loaded configuration
permits 45 degrees per joint per tick. The removed response handoff had imposed
much smaller effective correction caps; the first replacement used the full
INI limit directly. This exposed a large angular update to a locally predicted
contact manifold. The sampled log does not establish every contributing cause.

The follow-up bounds the combined angular correction for the entire chain at
360 degrees/second (6 degrees at 60 Hz), while retaining lower configured
per-joint limits. The budget uses the chain's integration timestep, bounded to
1/240 through 1/20 second. It neither sleeps the chain nor damps free motion.
Each candidate is evaluated against all retained contact planes. The solver
keeps the result with least squared penetration, preferring less movement
within numerical tolerance, and never publishes a larger residual than the
uncorrected candidate. Velocity projection uses this accepted correction.
The error guarantee applies to sampled planes and predicted geometry, not the
full curved collider surface or rendered engine pose.

A stress fixture uses the logged chain shape with constructed deep/opposing
supports and the 45-degree setting. The rejected solver produced a combined
57.9558-degree correction in its first case. All 200 cases now stay within the
6-degree budget at 60 Hz and do not increase contact error. This is a stress
test informed by the log, not a complete replay of engine frames.

With debug or collider diagnostics enabled, `body-contact bounded-solve` logs
each target independently (at most every 250 ms during contact), including
the timestep, all joint corrections and squared residual before/after.
The failed DLL, solver sources and game log are preserved in
`build/collision-backup-body-instability-20260909/`.

## Implemented

- Both body chains capture a frame reference before spring integration. Contact
  queries apply the candidate angle difference to that reference. Active
  penis/testicle cross queries use current-frame candidates when available;
  the second chain can see the first chain's corrected geometry.
- Contacts with different lever arms remain distinct. Near-duplicate normals
  merge only at nearly identical segment fractions. Storage is bounded at eight
  contacts per segment and 24 per chain.
- `physx_body_contact.c` solves the retained supports together, using all useful
  upstream joints and per-joint limits. Accumulated unilateral corrections can
  decrease as another support resolves, preventing unnecessary lift away from
  a surface. The older rule freezing proximal joints is no longer used.
- Contact velocity is projected through the influencing joints, removing inward
  normal velocity without converting position correction into a bounce impulse.
  Tangential motion and separating velocity are retained.
- The old penis/testicle correction filters, contact-history spring suppression,
  and rest/impact damping handoff were removed from these response paths. Body
  and room geometry queries retain their scope, validity and camera safeguards.

The existing per-tick correction cap and per-joint angular limits remain.
`collision_iterations` selects eight inner constraint sweeps per configured
iteration (up to 48), with early exit. Response strength zero disables response;
values below one relax the solve. Values above one use full correction without
over-relaxation, which previously risked overshooting support.

The module is included by `physx_colliders.c`; include it in source distributions.
No sidecar solver or user configuration was changed in this follow-up.

## Verification

`python run_body_contact_tests.py` compiles the actual contact solver, point
prediction, contact storage and angular-limit functions. It covers distinct
lever arms, duplicate contacts, proximal/distal/corner supports, inward velocity,
sliding, separation, fixed joint limits and disabled response. Two- and
three-segment angular springs resting on a plane are exercised at 30/60/144 Hz
and with occasional 50 ms frames. The largest settled tip variation in those
fixtures was approximately 3.4e-8 scene units. These synthetic values are not
measurements of visible in-game movement.

The existing collision, free-motion and startup-binding suites also passed.
`compile-physx.bat` built the DLL. The earlier diagnostic
`analyze_body_contacts.py` distinguishes corrected contact retention from the
retained legacy helper examples; use the new suite for the active solver.

## Scope and limits of this candidate

The two dynamic chains still run sequentially. Shared candidate geometry is an
improvement, but this is not yet a mass-weighted simultaneous solve of both
chains. Surface-relative velocities and physical contact friction are not
modeled here. Tangential motion is allowed rather than suppressed by the old
whole-chain rest damping, so sliding may be more noticeable.

Snapshot prediction is relative to the pre-step simulation state. If engine
pivots are held or engine traversal supplies an older pose, that reference can
still lag; the existing freshness/ghost protections remain. Joint rotation
prediction uses the composed mapping above for validated live samples, and
retains the existing approximation for unsupported/held samples. Large angle
changes, overlapping body geometry and unreachable joint limits need gameplay
coverage. Body gravity targets and free-motion/camera-coast integration were
not retuned during this contact-response change.

Test the reported upward-facing pose with support on pelvis, thighs and
testicles, then small body movements and release from each support. Also check
room support and camera orbit. Neither complete removal of in-game bounce nor
equivalent performance across all body configurations is established by the
standalone tests.

Rollback source and DLL:
`build/collision-backup-body-response-20260908-232244/`.
The confirmed release ZIP and full checkpoint remain available separately.
