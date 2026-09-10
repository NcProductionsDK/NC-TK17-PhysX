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
