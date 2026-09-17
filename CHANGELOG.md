# Breast/butt collision offsets and setting aliases - 2026-09-16

- Add optional XYZ `collision_min_offset` and `collision_max_offset` for
  breasts and butt. Bound displacement from the normal translation spring
  for other-person, self and room contacts, allowing overlap at the limit.
  An independent collision-free spring preserves normal jiggle/gravity motion
  and prevents the allowance accumulating across frames. Omitted bounds keep
  existing behavior; existing total translation limits still apply.
- Accept `min_angle`, `max_angle` and `gain` for these single-bone sections,
  with per-key `joint01_*` fallbacks. Support both naming schemes and optional
  offset overrides in body profiles. Keep chain section names unchanged.
- Update the supplied config to the simpler names and add commented offset
  examples. Reject malformed, non-finite or incorrectly signed offset tuples.
- Regression coverage includes all XYZ directions, reflected parents, self,
  other-person and room contacts, sustained contact, live bounds/strength
  changes and zero-bound spring motion at 20/30/60/144 Hz. Windows INI tests
  cover aliases, body-profile inheritance and missing/invalid settings.
  In-game validation of the new offsets remains pending.

# Incoming collision strength - 2026-09-16

- Add `collision_strength` (float, clamped to 0.1-1.0, default 1.0) to the
  breast, penis, testicle and butt physics sections, including body profiles.
  Only the receiver's response to other people is reduced; self/room contacts
  and outgoing collider geometry retain their existing behavior.
- Anchor weak contact separation to the collision-free spring target so
  sustained contact stays weak instead of accumulating full displacement.
  Preserve the original `1.0` path and hard self/room supports.
- Bind all four settings sliders to their global INI values. Capture the
  numeric payload at the ConfigEditor callback adapter before it discards
  that payload when forwarding to the text handler. Suppress initialization
  callbacks and stale widget writes; restore INI values when the page opens.
- Skip duplicate slider saves to avoid redundant INI writes and log entries.
  Compare the persisted value so external edits are still respected. Normal
  logs report saved values and loaded strengths; breast/butt contact traces
  include the effective receiver strength.
- Automated tests cover configuration, both contact solvers, sustained contact
  at 20/30/60/144 Hz, live strength changes, mixed hard/soft supports and the
  native x86 callback adapter. Settings tests include missing widgets, bounds,
  initialization, reopening, duplicate events and external INI edits.
- User confirmed the intended weak breast/butt collision behavior and working
  in-game slider updates, and approved the feature. Duplicate-save cleanup
  is covered by automated regression tests.

# Hook5 collision wireframes - 2026-09-14

- Replace the CPU bitmap overlay with one batched D3D11 line draw and a reusable
  vertex buffer. Upload only visible candidate line vertices, with no full-screen
  image clear, GDI objects or texture upload. Empty batches issue no draw.
- Draw oriented 3D ellipsoid rings and tapered capsules from collider positions,
  radii and per-body scaling. Keep physical target volumes separate from the
  moving object's radius added by contact queries.
- Preserve homogeneous perspective and clip individual lines at the frustum;
  remove the Hook5 path's screen-radius clamps and axis-aligned oval approximation.
- Preserve the confirmed layer below the GUI. Native D3D8/OpenGL and collision
  response remain unchanged. This follow-up needs only the new PhysX DLL.
- Geometry, WARP rendering/state/clipping and actual collider-data tests cover
  the new path. In-game appearance and performance comparison remain pending.

# Hook5 collision visualization layering - 2026-09-14

- Replace the topmost desktop collision window with a debug layer drawn into
  Hook5's completed scene before TK17's GUI. Collision outlines can no longer
  cover other applications, and the game's GUI is drawn above them.
- Requires the updated Hook5-Extended debug scene callback. An older or missing
  Hook5-Extended skips Hook5 collision visualization and logs the requirement.
- Preserve native D3D8/OpenGL rendering and collision simulation. The new
  callback coexists with Liquids and also supports Liquids being disabled.
- Builds, WARP pixel/state/resize tests and scene-hook regression checks pass;
  in-game GUI, Alt+Tab and fullscreen validation remains pending.

# Gravity during collider warmup - 2026-09-13

- Follow-up in-game testing showed that the filter change below did not resolve
  the visible transition. The next capture identified geometric gravity being
  disabled by the collider readiness gate, despite strengths already applying.
- Let confirmed local engine geometry initialize chain dynamics during collider
  warmup, and keep geometric gravity independent of collision readiness.
- Preserve collision settling/response gates and reject unconfirmed, held,
  synthetic or stale samples when initializing during warmup. Config unchanged.
- Regression fixtures cover both chains before/after promotion and repeated
  warmups; visual validation of this follow-up remains pending.

# Penis/testicle gravity transition - 2026-09-13

- On a confirmed large orientation change, seed the mapped and geometric
  gravity filters from the new pose instead of blending from the previous pose.
- Retain sample confirmation and camera holds, including changes confirmed while
  a body hold is active. Ordinary animated motion keeps its existing smoothing.
- Preserve spring state, contacts, neutral references and all config values.
- Regression tests reproduce the logged direction change at 30/60/90/144 Hz.
  Visual pose-load validation remains pending. See
  [POSE-GRAVITY-TRANSITION.md](docs/POSE-GRAVITY-TRANSITION.md).

# Penis gravity strength controls - 2026-09-13

- Added `gravity_horizontal_strength` / `gravity_vertical_strength` (0..4,
  default 1) under penis_physics, with global reload and body-sidecar overrides.
- Scale final gravity after geometric shaping so full-tilt and inverted mapped
  gravity remain adjustable; movement and wind retain their existing inputs.
- Preserve the existing partial-tilt curve semantics and exact unit-strength path.
- Added both controls at 1 in the installed/default config; existing values remain.
- Full-tilt geometry-transition and config regression tests pass. In-game checking
  is pending. Details: [PENIS-GRAVITY-STRENGTH.md](docs/PENIS-GRAVITY-STRENGTH.md).

# Smoother breast and butt updates - 2026-09-13

- Extended `update_rate_hz` to breasts and butt, including global config,
  body-sidecar inheritance/overrides, collider refresh and cadence logging.
- Nonzero mode normalizes movement to the 16 ms reference response and uses
  bounded angular substeps; gravity, wind and constant sag retain their strength.
- Zero/omitted retains legacy timing and integration. Installed config unchanged.
- Scheduling, paired motion and single-bone contact regressions pass; in-game
  smoothness and FPS cost await testing. See [BODY-UPDATE-RATE.md](docs/BODY-UPDATE-RATE.md).

# Optional smoother body updates - 2026-09-13

- Added per-section `update_rate_hz` for penis/testicles: 0 retains existing
  timing, -1 follows render frames, and 1..240 targets a configurable rate.
- Shared precise frame timing, coordinated collider refresh, bounded stalls
  and rate-switch rebasing; existing motion and solver settings are retained.
- Added actual body update frequency to the existing performance profile log.
- Scheduling, INI inheritance, collider cache and body physics regressions pass;
  the 32-bit DLL builds. In-game smoothness and FPS cost await testing.
- Installed config unchanged. See [BODY-UPDATE-RATE.md](docs/BODY-UPDATE-RATE.md).

# Body contact velocity optimization - 2026-09-13

- The detailed gameplay capture identified repeated body-solver geometry work.
- Reused each contact's fixed Jacobian across the eight velocity passes within
  one solve; retained pass order, live velocities, inertia and limit handling.
- 2,400 old/new comparisons produced identical correction and velocity outputs.
  Existing contact regressions and instrumented fixtures pass; DLL builds.
- Isolated contact workloads took 18-54% less solver time; in-game validation
  and FPS comparison of this change are pending. Config files are unchanged.
- Details: [BODY-CONTACT-PERFORMANCE.md](docs/BODY-CONTACT-PERFORMANCE.md).

# Collision profiling build - 2026-09-13

- Fixed detail records being filtered out with debug disabled; an integration
  test now verifies the actual file logger emits them under performance_profile.
- Preserved the approved wind/activation checkpoint and physics behavior.
- Added opt-in collision collection/solve/room timings and workload counters
  under the existing performance_profile switch; no config files were changed.
- Collision regressions pass with profiling both erased and enabled. The
  32-bit DLL builds; a contact-heavy in-game capture is the next step.
- Field definitions and test steps: [COLLISION-PROFILING.md](docs/COLLISION-PROFILING.md).

# Code performance and continuous wind - 2026-09-12

- Added validated positive lookup hints for addon/equipment registries and
  removed unused activation queries and gravity-only report calculations.
- Reused wind phase/sample calculations and shared chain sway; skipped sine
  evaluations when turbulence is zero.
- Removed the hourly wind phase reset and improved clock precision while
  retaining existing wind settings and body/clothing/room response mappings.
- Ten existing regression suites and two new differential suites pass; the DLL
  builds. In-game behavior and FPS validation of this build are pending.
- Details, isolated timing results and rollback location:
  [PERFORMANCE-WIND.md](docs/PERFORMANCE-WIND.md).

# Breast/butt contacts and body collision camera fix - 2026-09-10

- Replaced breast/butt summed collision offsets with bounded translation
  constraints, correctly mapped to the bone parent, for improved separation
  and more natural pushing against body and room supports.
- Protected body collision coordinates from mismatched camera/skeleton updates
  while retaining live local bone movement. The room-placement check waits
  160 ms after camera movement and confirms across two samples.
- User testing accepted the collision improvements and camera fix. Slight
  penis-chain movement under unusually aggressive camera motion remains an
  accepted limitation; no further tuning is planned for this checkpoint.
- Regression coverage includes asynchronous camera updates and delayed settling.
  Release packaging includes the exact gameplay-confirmed DLL and updated notes.
  Details: [BODY-COLLISION-CAMERA.md](docs/BODY-COLLISION-CAMERA.md).

# Gravity sampling checkpoint — 2026-09-10

- Replaced the extra 1.6-second body gravity hold and root-displacement release
  with deferred direction confirmation using the full camera version.
- Shared that sampling protocol with geometric body gravity, breast spacing
  gravity and person-owned sidecar parent gravity; retained trusted forces while
  samples are held and prevented a sidecar fallback from switching force frames.
- Corrected repeated filter resets with `gravity_zero_at_start` disabled and
  made body gravity smoothing consistent with elapsed time.
- Added sampling diagnostics and regression coverage. All eight suites and the
  DLL build pass. In-game testing confirmed responsive pose switching, pose
  location changes and Ctrl+Z, with no problems reported. The brief gravity
  confirmation delay after camera movement was accepted.
- Release cleanup clarifies existing configuration options without changing
  settings or physics code. The bundled default comments are updated too.
  Details and limitations: [GRAVITY-SAMPLING.md](docs/GRAVITY-SAMPLING.md).

# Sidecar motion checkpoint — 2026-09-09

- Normalized inherited bend and speed by child/parent link length, so the same
  parent rotation no longer disproportionately drives a shorter child.
- Published solved velocity for fixed-length sidecar motion, preserving small
  movements and using the same velocity convention through contact and release.
- Retained joint gain, taper, drive settings and the older rigid-path fallback.
  These are general sidecar changes, with no addon-specific conditions.
- User testing accepted the result. All seven regression suites and the DLL
  build passed before gameplay testing, including unequal-length chains,
  small inherited motion, resting supports and release.

Checkpoint cleanup updates documentation and packaging only. The cumulative
package contains the exact gameplay-confirmed DLL; no additional solver changes
or game/addon configuration edits were made. This improves motion inheritance,
not downstream mass coupling. Further gravity and joint-limit work remains.

# Body motion checkpoint — 2026-09-09

- Preserved elapsed time for penis and testicle simulation using bounded substeps,
  consistent movement-input scaling and explicit handling of long stalls.
- Added progressive braking near joint limits while keeping the configured range
  and immediate response when moving away from a limit.
- Added pose-dependent gravity based on measured chain geometry, blended with
  existing gravity settings. Retained camera protection and zero-gravity behavior.
- Added relative link inertia from the mass and leverage of downstream segments,
  with matching damping and contact-velocity response.
- User testing accepted the timing, stops and combined gravity/inertia changes.
  The inertia model is a reduced diagonal approximation, not a complete
  articulated-body or soft-tissue simulation.
- Checkpoint cleanup removes a redundant contact-loop wrapper and skips unused
  inertia accumulation during gravity evaluation. A 10,000-pose comparison found
  bit-identical gravity and reference inertia against the accepted implementation.

All seven regression suites and the full DLL build pass. The cumulative package
includes the preceding body collision and sidecar fixes. Existing game settings
are retained. The exact gameplay-confirmed build is backed up before cleanup.

# Body collision checkpoint — 2026-09-09

- Corrected combined-axis body-joint prediction to match the engine's composed
  Euler rotations, authored orientations and parent transforms.
- Coordinated contacts across upstream body joints while retaining distinct
  support locations and bounding the complete chain correction.
- Separated position correction from contact velocity response, preserving
  sliding and release without injecting a depenetration bounce.
- Used actual testicle pivots and current active-chain geometry for cross
  collisions; prevented blocked joint velocity from leaking into child joints.
- Improved competing-support progress and outward recovery from embedded
  contacts without reversing normals from already embedded history.
- In-game testing confirmed resolution of the reported thigh penetration and
  testicle resting problem. This confirmation covers the tested setup, not
  every rig, configuration or frame rate.
- Checkpoint cleanup removes the unused body contact-filter storage/resets and
  unvalidated raw matrix rows from diagnostic records. Useful bounded pose
  diagnostics, compatibility fallbacks and regression tests are retained.

The exact gameplay-confirmed DLL and source are backed up locally before
cleanup. The solver equations, settings and user configuration are unchanged by
the cleanup. The release package includes the earlier sidecar/free-motion fixes.

# Collision checkpoint — 2026-09-08

- Fixed sidecar free-motion nudging by combining spring forces before movement
  and integrating tangent to fixed-length links with bounded substeps.
- Removed camera-triggered sidecar stiffness changes and child pose holds while
  retaining protection against camera-contaminated movement inputs. In-game
  testing confirmed improved free motion and sidecar contact stability.
- Fixed startup layout detection near allocation boundaries and added guarded
  retries for rejected sidecar joint layouts.
- Improved sidecar/body resting contacts using predicted segment positions and
  coordinated corrections across influencing joints.
- Preserved contacts at different lever arms and added body collision for the
  configured extension beyond a chain's final joint.
- Fixed small-vector square-root accuracy and discontinuous shallow-contact
  response that could contribute to nudging.
- Improved overlapping-contact handling, fixed-length response, friction and
  velocity handling; collision corrections no longer become inherited motion.
- Corrected collision-to-joint rotation mapping and published late upstream
  corrections in the same frame.
- Improved room contact response and removed artificial sideways escape from
  the output-aware path.
- Fixed false body-collider removal during movement when the engine still
  confirms that the person is visible.

The latest combined-contact-error rejection experiment was reverted after worse
in-game results and is not included. The later free-motion fix resolved the
reported nudging and further improved contacts in the tested cases. Other
sidecars and mixed body/room contacts still need
broader gameplay coverage. Dedicated body/body physics was not overhauled.

Fiesta's local test configuration also gained `spine03` and `spine04` collision
targets. That addon-specific INI edit is not part of this general plugin package.
