# NC-TK17-PhysX — physics checkpoint

This package contains the cumulative collision, body-motion, sidecar-motion and
gravity-sampling checkpoint confirmed through 2026-09-10. It includes body
movement timing, progressive joint stops, pose-dependent gravity and link inertia.
It also includes length-consistent sidecar inheritance and continuous publication
of solved sidecar velocity. Pose changes and undo now refresh gravity through
consecutive-frame sample checks instead of the old long camera hold. Breast and
butt contacts now use bounded translation constraints for improved separation
and more natural pushing. Body collision coordinates are protected against
mismatched camera/skeleton updates. The package contains the exact DLL accepted
in the latest gameplay test.
It is an experimental 32-bit Windows extension for compatible TK17 installations.
See CHANGELOG.md for the changes and remaining limitations.

## Installation

1. Close TK17 and keep a copy of your current `Binaries/NC-TK17-PhysX.dll`.
2. Extract the package's `Binaries` folder into the TK17 game directory, replacing
   that DLL.
3. Start the game. Existing `Extensions/PhysX/Config.ini` settings are retained.
   If the configuration is missing, the plugin generates its documented defaults.

`Config.default.ini` is a reference copy. It is not installed over your existing
configuration. Keep `[defaults] debug = false` for normal play; enable diagnostic
logging temporarily when investigating a problem.

No settings migration is required. The gravity sampling safeguard waits for at
least 48 ms of camera quiet, followed by agreeing samples on consecutive frames.
A brief delay when changing or undoing a pose immediately after camera movement
is expected; springs continue using the last trusted gravity direction.
`gravity_response_ms` controls body target smoothing separately. Setting it to
zero does not disable camera checks or the configured speed limit. Startup probe
settings and compatibility fallback scales remain supported.

Body collision placement has a separate 160 ms camera-quiet check, followed by
two agreeing samples. Local bone motion remains live. Moving the camera and
changing the body's global pose location together can briefly delay its new
collision placement. This does not change `gravity_response_ms`.

The existing breast/butt collision switches and room-collision settings still
apply; this update does not turn disabled collision systems on automatically.

Addon `.physx.ini` files still determine their collision scopes, radii and terminal
lengths. A body region excluded by an addon's custom scope will not collide.
This package does not modify individual addon files or include the Fiesta addon.

## Validation and limits

All ten automated suites passed during development: collision, body-contact,
body-pose, body-motion, body-dynamics, startup-binding, free-motion,
gravity-response, single-bone-contact and collision-frame. The four relevant
suites were rerun after the final camera-settling adjustment. They cover pose calculations,
contact/velocity constraints, terminal collision, joint limits, gravity and inertia,
and simplified settling/release at multiple rates including 20/30/60/144 Hz,
uneven frame durations and capped stalls. In-game testing
confirmed the free-motion nudging fix and further improvement to sidecar
contacts. Subsequent testing confirmed that composed body-joint prediction
resolved the reported penis/thigh penetration and testicle resting difficulty.
Further user testing accepted the body timing, joint stops and combined gravity
and inertia improvements. Earlier cleanup removed a redundant loop and unused
arithmetic; 10,000 sampled poses retain bit-identical gravity and reference inertia.
The gravity/inertia model uses uniform rods and a diagonal approximation; it does
not simulate full inertial coupling or soft-tissue deformation.
The subsequent sidecar inheritance changes were also accepted in-game. Tests
cover equal/unequal link lengths, small inherited motion and supported-chain
release. Gravity testing subsequently confirmed snappy pose switching, pose
location changes and Ctrl+Z, with no problems reported. The short confirmation
delay after camera movement was accepted. Gravity regression checks cover camera
pan/orbit, pose flips, invalid samples, rebinding and frame-rate variation.
Continuous camera movement, rapid direction changes or update gaps above 100 ms
can retain the previous direction until samples qualify. These checks cannot
prove that every engine matrix is fresh; see `docs/GRAVITY-SAMPLING.md` for details.
User testing also confirmed substantially improved breast/butt collisions and
accepted the body camera fix. Slight penis-chain movement can remain under
unusually aggressive camera motion; this is an accepted limitation. The camera
follow-up applies to body physics; sidecar camera handling is unchanged.
See `docs/BODY-COLLISION-CAMERA.md` and `docs/SINGLE-BONE-CONTACT.md` for details.
Final release cleanup changes documentation and packaging only.
Different joint layouts and simultaneous
body/room contacts may behave differently. This is a tested checkpoint, not a
claim that collision behavior is finished for every addon.

`SHA256SUMS.txt` lists the hashes of the package files. Restart the game after
changing DLL versions. To roll back, restore your saved DLL while the game is closed.
