# Clothing-hidden genital PhysX — implementation and validation

Updated 2026-09-15. The user confirmed pause/resume and collider visibility for
body02 on Person01/03 and body03 on Person02. They report approximately 1–2 FPS
gained per person while hidden genital PhysX is paused. This is a gameplay
observation, not a controlled benchmark or guaranteed scaling result. The user
requested no further benchmarking. They subsequently confirmed that continuous
clothing resume removes the distracting snap and is their preferred behavior.

## Configuration and behavior

`[defaults] pause_hidden_genitals = true` enables the feature. Omitted/false
disables suspension and visibility lookups. The user's current INI value is
preserved during builds and DLL installation.

The ConfigEditor control `NCPhysXPauseHiddenGenitals` maps to this global key.
GUI ON/OFF writes true/false through the existing settings handler and normal
INI hot reload applies it. When the controls are built/reopened, the existing
spinbox sync reads the INI value, overriding the script's initial OFF default.
External INI edits are reflected in the GUI on reopening the controls.

`physx_update_genital_pause_state` handles all four people independently:

- A confirmed body02/body03 type and validated genital visibility of zero pause
  that person's penis and testicle PhysX. Unknown type, missing geometry or an
  unsupported getter leave physics enabled. User enable flags remain unchanged.
- Active people are checked every 250 ms, or sooner on named-node generation
  changes. Paused people are checked before the solvers every physics tick so
  uncovering does not wait for the slow poll. The normal rendering path runs one
  physics tick per presented frame. No scene pointer is reused for field reads.
- Existing disable paths release runtime bone ownership, reconnect PoseEditor
  tracks and reset solver timing/velocity. Testicle track restoration schedules
  next-frame pose evaluation. Late writes, traversal ownership and native
  animation filtering respect the same pause state. Resume initializes from live
  state, retaining validated axis references and a skeleton stability ticket
  for a clothing-only suspension. No previous simulation pose is restored.
- Native TK17 animation/inertia is restored. This pauses plugin simulation;
  it does not eliminate all engine skeleton work.
- Hook5, D3D8 and OpenGL debug paths omit the paused genital shapes. Shared body
  collision data stays available to other people and add-on solvers.

There is no change to Hook5-Extended for this feature.

## Continuous clothing resume

Resume preserves the pose visible when clothing is removed, then lets PhysX
settle naturally. This applies to both chains, all four people and both
mode-state arrays. The user confirmed that the transition looks smooth and
shows no distracting snap in their tested setup.

- Existing AppBase pre-animation (runtime) and PoseEditor post-track hooks attempt
  pending clothing resumes earlier; EndScene remains the fallback. Per-chain,
  per-person masks prevent a second solver call at frame end. A phase guard
  blocks recursion during native track handoff. Present/SwapBuffers resets masks.
- Suspension still releases native ownership and clears old motion and contacts.
  A saved skeleton ticket bypasses the testicle startup delay only after live
  resolution matches every pointer and the named-node generation is unchanged.
  Changed or unknown skeletons retain the original three observations / 50 ms
  validation. Hidden solvers remain paused.
- Both chains capture current output before acquiring native ownership. On
  initialization, that output becomes the initial simulation pose with zero
  velocity. Penis mapping includes its existing PoseEditor compensation. The
  first update returns with that pose, rather than jumping to the force target.
- Gravity-readiness waits preserve the captured pose. Once ready, normal spring
  integration settles it toward gravity/wind/contact response. Old translation
  and rotation impulses are discarded on the first resumed integration step.
- Initial angles obey joint/chain limits. Any residual needed to preserve an
  out-of-limit authored pose is stored as a temporary rest offset, which decays
  with a 60 ms time constant during simulation only. This avoids a limit-clamp
  cut while restoring the normal rest over a short transition. Contact prediction
  includes this changing offset. Testicles still simulate two links: their end
  joint's temporary rest offset decays without adding an end-joint spring.

No clothing visibility writes, hidden catch-up simulation, new settings or
Hook5-Extended changes are involved. The result is a continuous settling motion,
not a promise that the first visible frame is already at physics equilibrium.

Automated checks cover skeleton/generation rejection, old-motion reset, exact
initial output (including out-of-limit poses and pose compensation), fixed
end-joint angles, rest-offset convergence and step-size independence, contact
prediction deltas, actor/mode isolation and exclusion of duplicate frame-end
attempts. A fixture runs the actual penis solver to check initial publication,
holding during gravity readiness, and subsequent movement toward the target.

**Runtime status:** user-confirmed smooth transition on 2026-09-15 with DLL
SHA256 `AC2764E0A8400C5F875947AA1CAC29446828BF63F8595C651C6BB51257961D15`.
The report confirms the preferred behavior in their setup; it does not establish
every mode, body replacement or Person04 scenario. No further FPS benchmark is
requested.

Cleanup limits the testicle starting-pose snapshot to a pending, uncaptured
clothing resume, avoiding that copy on ordinary active updates and readiness
retries. It does not change the approved simulation or transition timing.

## Body-type detection

Explicit per-person body loads are retained independently of optional PhysX
sidecar binding by `body_profile_note_virtual_body_scene_a`. For example, TK17's
`good-bye-txx.log` contains `Execute3 'Shared/Body/body03___02'` for Person02.
The existing log observer and direct virtual-file paths use the same parser.
A subsequent body load replaces the type; UI selection intent and collision
scene loads do not. An exact profile path is a fallback if no load was observed.
No fixed person-to-body assignments or additional file polling are used.

Two rejected approaches explain this choice: profile paths alone excluded bodies
without a bound .physx.ini; body root Object.Name strings were not exposed by
the runtime named-object lookup. Neither is used to infer a custom sidecar.

## Visibility reader evidence

`DcDressVX_NcPants11.bs` hides the named geometry with `.Show I32(-1)`; the lowered
pants variant uses a different command array. The body scripts define it as
TSkinPolygonGeometry linked to SSkinPolygonGeometry through TNode.SNode.
The runtime name is `PersonNNBody:body_subdiv_cageShape__body_genital01_SG`.

The reader validates both complete getter implementations before following their
reads, without invoking engine dispatch:

- TNode.Visibility (0x06FFF043): SYS getter RVA 0xED8F0 reads SNode at object+0x10
  and dispatches SNode.Visibility. Metadata is at object-0x18; the class table
  is metadata+0x10C, getter slot +0x180.
- SNode.Visibility (0x01FFF042): getter RVA 0xE0C40 is `8B 41 10 C2 04 00`, reading
  DWORD snode+0x10. Class table: metadata+0x108, getter slot +0x40. Both tables
  use a supported marker at table-0x1C.
- SYS registration sequences at RVA 0x120AF6 and 0x120933 tie these getters to
  their property IDs. `Development/patch-sources/pemod/classes.h` also maps them.

ImageLayer.Visibility (0x02FFF0B3) is a different property with the same display
name; the initial diagnostic incorrectly selected it. The tests therefore check
installed SYS registration bytes as well as synthetic object layouts.

Analyzed SHA256:

- TK17-158.001.exe: `280F288015A224790D21676A36DB249D7D70A559524242E22B7307C37A9B592A`
- ThriXXX010278-SYS.dll: `C56FBF8C3256BC21B631B1A3F3AA6341150549B6994747E178D9E44DC223F8EC`

## Logging and automated checks

Normal logs retain concise `genital physics state` pause/resume transitions.
Detailed `genital visibility probe` messages (object, raw value, body source and
validation reason) require `debug = true` and are emitted only on changes.
`performance_profile` does not enable these detailed diagnostics.

Run `test_genital_visibility.cmd`. It checks installed SYS signatures and
registration IDs, reader rejection paths, normal/debug log filtering, both body
types in every person slot, mixed clothing states, inactive people, body-load
replacement and unrelated-event rejection. It also exercises actual solver
disable dispatch in both modes, motion/timing reset, ownership gates, unchanged
enable settings, next-tick resume/object-loss handling and slow polling for
active people. The executable optionally accepts a TK17 log path for replay.

## Gameplay evidence and limits

Body02's geometry was correlated through 1 → 0 → 1 on the same object at
00:42:18, 00:42:53 and 00:43:35. Person03's pause/resume was subsequently logged
at 11:57:07–18. After the body-load fix, Person02 resolved body03 and recorded
pause → resume → pause at 12:04:31.769, 12:04:53.825 and 12:04:56.295. The user
confirmed the collider shapes disappear while covered and return with working
penis/testicle physics when uncovered.

The user-tested functional DLL SHA256 was
`ECE3382652182C4B1CF3784C1A37E293EEB482839A302C28229EDA5515CD34B9`.
Body-load evidence and the prior DLL are preserved in
`build/collision-backup-genital-loadtype-20260915-120158/`.

Cleanup build: detailed diagnostics gated by debug, concise transition logging,
update-function rename, and consolidated notes. Existing checks and production
build passed; installed with TK17 closed and the copied DLL hash verified:
`01140316DDE894A4B9A07B50788A9187280E8E74B687ADF9AEA5AEC2F5906F1D`.
The user-tested functional DLL is backed up in
`build/collision-backup-genital-cleanup-20260915-122244/`.

Person04 was inactive during gameplay tests; its slot has automated coverage.
FreeMode handoff, editing poses while covered, live person/body replacement and
option hot reload remain broader-release validation items. The original resume
jump has since been replaced by the user-confirmed continuous transition above.

The feature does not enumerate every skinning or attachment consumer of these
bones. Keep it disabled for unverified custom attachments that require genital
PhysX while the body geometry is hidden. Dependency detection remains work
before considering default-on use.
