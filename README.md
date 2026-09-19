# NC-TK17-PhysX

NC-TK17-PhysX is an experimental 32-bit Windows extension for The Klub 17. It adds configurable secondary body physics, collision handling, and support for per-body and per-addon physics profiles.

## Features

- Physics simulation for breasts, buttocks, penis, and testicles
- Body and addon collision support
- Per-person and sidecar configuration profiles
- Runtime configuration reloading
- Optional collider visualization and diagnostic logging
- Contact-plane solving for sidecar/body and room contacts, length-constrained
  sidecar response, and same-frame publication of late upstream corrections
- A larger, collision-resistant per-frame cache for repeated Windows
  memory-readability checks, preserving the existing safety checks while
  avoiding redundant `VirtualQuery` calls
- Cached runtime-root hints for body and addon target resolution. The extension
  still asks TK17 for the current live object on every lookup, but avoids
  rebuilding and rescanning unchanged root prefixes throughout a stable room

The extension uses version-specific runtime hooks and is intended only for compatible TK17 installations. Back up your game files before installing or testing it.

## Incoming collision strength

`collision_strength` in `[breasts_physics]`, `[penis_physics]`,
`[testicle_physics]` and `[butt_physics]` controls how strongly that body part
responds to **other people**. It accepts floats from `0.1` to `1.0`, clamps values
outside that range, and defaults to `1.0` when absent. Self-collisions and room
collisions retain their existing response. The setting does not weaken the
person's colliders as obstacles for others; the receiving person's setting applies.

Global values in `Extensions/PhysX/Config.ini` apply to all people. A body's
`.physx.ini` profile can override the same four section keys for its wearer.
Changes use the existing configuration reload mechanism. Collision enable switches
and scopes still apply; a scope without `_all` has no contacts from other people
to weaken. The legacy `[body_colliders] collision_strength` is a separate setting.

The four collision-strength sliders on the PhysX settings page read these global
INI values when the page opens and save changes as floats. Opening the page
does not write TK17's previously saved slider values back into the INI. Reopen
the page after editing the INI externally to refresh the sliders.
Slider events capture the numeric callback payload before TK17's adapter drops
it when forwarding to its text handler. Saving does not require a cached widget.
Repeated notifications for the same saved value do not rewrite the INI.
Normal logs report `settings slider saved` and the loaded collision strengths;
breast/butt contact traces show the receiving person's `incoming_strength`.

Strength scales each external contact's displacement relative to the body's
collision-free spring target, including its motion/gravity drive. Lower strengths
therefore allow persistent overlap, instead of gradually converging to full
separation. Inward-speed reduction is also weakened. Joint limits and full-strength
self/room supports still apply; simultaneous constraints can affect the result.

## Breast and butt collision movement limits

`[breasts_physics]` and `[butt_physics]` accept optional `collision_min_offset`
and `collision_max_offset` values, each containing three X,Y,Z floats:

```ini
collision_min_offset = -0.02,-0.01,-0.03
collision_max_offset = 0.02,0.04,0.03
```

These are distances in the output bone parent's local coordinates, in scene
units. They bound collision displacement relative to an independent copy of
the normal translation spring, including jiggle and gravity sag. They apply
to **all contacts: other people, self and room**. Bone orientation and parent
scale determine their world directions; they are not camera axes.

Minimum components must be zero or negative, and maximum components zero or
positive. Zero on both sides of an axis blocks collision displacement on that
axis while normal spring motion continues. Each omitted bound adds no new
restriction. Leaving both keys out preserves the existing collision response.
Malformed tuples are ignored; existing per-body inherited values are retained.

The existing `bone_translation_max_offset` still caps total translation.
Collision limits take priority over separation: contact can remain overlapping
when it would require movement outside the permitted range. Incoming
`collision_strength` continues to weaken only other-person contacts before
the displacement bounds apply. Global values can be overridden independently
in body `.physx.ini` profiles and use the usual configuration reload mechanism.
The supplied config contains commented examples so no new bounds are imposed
until enabled.

For these two single-bone sections, use `min_angle`, `max_angle` and `gain`
for rotational physics. Each new key takes precedence over its corresponding
legacy `joint01_*` key in the same file. Old configs and body profiles continue
to work; penis and testicle chains keep their numbered keys. Angles still
accept XYZ values or a single value for all axes. If a minimum is omitted,
it defaults to the negative maximum, as before.

## Build

Install the MSYS2 MinGW 32-bit toolchain, then run:

```powershell
.\compile-physx.bat
```

The compiled DLL is written to `build\NC-TK17-PhysX.dll`.

Hook5 collision visualization requires Hook5-Extended with the
`nc_hook5_extended_register_debug_composite_callback` export (2026-09-14 or
later). Install both updated DLLs. Debug outlines are composited into the
completed scene before the GUI; no desktop overlay window is created. If the
bridge is unavailable, PhysX logs this and skips Hook5 visualization. Native
D3D8/OpenGL visualization and collision simulation do not require this bridge.
Hook5 now draws batched 3D wireframes with reusable vertex storage instead of
uploading a full-screen bitmap. The rings show physical collider volumes;
contact queries additionally account for the moving object's radius. Outlines
remain visible through scene geometry and below the GUI.
Run `test_debug_composite.cmd` for geometry, D3D11 WARP rendering and actual
collider-data tests.

Run collision checks with Python and the same MinGW toolchain:

```powershell
python run_collision_tests.py
python run_body_contact_tests.py
python run_body_pose_tests.py
python run_body_motion_tests.py
python run_body_dynamics_tests.py
python run_free_motion_tests.py
python run_binding_tests.py
python run_gravity_response_tests.py
python run_single_bone_contact_tests.py
python run_collision_strength_config_tests.py
python run_paired_config_tests.py
python run_settings_slider_tests.py
python run_collision_frame_tests.py
```

Keep all source modules in source checkouts, including `physx_chain_contact.c`,
`physx_body_contact.c`, `physx_contact_math.h` and `physx_body_pose.h`.
The body movement implementation also requires `physx_body_motion.h`; see
[BODY-MOTION.md](docs/BODY-MOTION.md) for changes and validation.
The accepted progressive joint stops are described in
[BODY-LIMITS.md](docs/BODY-LIMITS.md). The accepted gravity/link-inertia implementation
requires `physx_body_dynamics.h`; see [BODY-DYNAMICS.md](docs/BODY-DYNAMICS.md).
Gravity sampling also requires `physx_gravity_sample.h`; the confirmed checkpoint
and its validation are described in [GRAVITY-SAMPLING.md](docs/GRAVITY-SAMPLING.md).

The accepted sidecar motion follow-up normalizes parent-to-child angular motion
and preserves small inherited velocities. See [SIDECAR-MOTION.md](docs/SIDECAR-MOTION.md)
for the review, validation and remaining work. The cumulative release ZIP includes
this follow-up and the confirmed gravity sampling overhaul.

## Release package

The current source/build includes the gameplay-confirmed breast/butt contact
improvements in `physx_single_bone_contact.c` and `.h`; see
[SINGLE-BONE-CONTACT.md](docs/SINGLE-BONE-CONTACT.md). Body collision camera
protection requires `physx_collision_frame.c` and `.h`; see
[BODY-COLLISION-CAMERA.md](docs/BODY-COLLISION-CAMERA.md). The user accepted this
fix, including slight residual penis-chain movement under unusually aggressive
camera motion. The release ZIP includes these changes and the exact
gameplay-confirmed DLL; see [RELEASE.md](RELEASE.md) for installation and limits.

The confirmed [gravity sampling checkpoint](docs/GRAVITY-SAMPLING.md) replaces the
long camera hold with deferred sample confirmation shared by body and sidecar
gravity. In-game testing confirmed responsive pose switching, location changes
and Ctrl+Z. The cumulative release ZIP includes this checkpoint, with clarified
configuration comments and matching executable code to the tested DLL.

After validating the built DLL, run:

```powershell
python package-release.py
```

This packages the existing DLL into
`build/NC-TK17-PhysX-collision-checkpoint.zip` with installation notes, reference
defaults and SHA256 checksums. It verifies the archive and 32-bit DLL format,
but does not rebuild, install, or publish it. Test executables, logs, historical
DLLs and individual addon files are excluded. Installation retains existing
game configuration.

See [CHANGELOG.md](CHANGELOG.md) for fixes and [RELEASE.md](RELEASE.md) for
installation and limitations. The 2026-09-09 body-chain checkpoint was confirmed
in-game to resolve the reported thigh penetration and testicle resting problem.
Subsequent timing, progressive joint stops, gravity and link-inertia improvements
were also accepted in-game and are included in the cumulative release package.
See [the plugin review](docs/PLUGIN-REVIEW.md) for remaining engineering work.

## Configuration

Global settings are read from `Extensions/PhysX/Config.ini`. The plugin creates the `Extensions/PhysX` directories and writes its documented default configuration when the file is missing. Body and addon-specific `.physx.ini` sidecars can override relevant settings and are reloaded when changed. Some global controls are also available through the in-game configuration editor.

For an add-on body, place `body01.physx.ini`, `body02.physx.ini`, or
`body03.physx.ini` beside the corresponding `bodyXX.bs` in
`Addons/<your add-on>/Scenes/Shared/Body/`. The sidecar overrides only the
person loading that body; unspecified settings inherit the global configuration.
Binding uses the loaded file path and TK17's per-person body-load or body-selection
events. It does not inspect blendshapes, mesh names, or the contents of the body
file, and needs no special morphs or identification entries in the sidecar.
Loading a replacement body clears the previous profile while its exact path is
resolved; a body without a sidecar uses global settings. Ambiguous file-open
candidates are left unbound. Run `python run_body_profile_binding_tests.py` from
this directory for the binding regression tests.

To isolate a body collider while adjusting its radius or offset, use:

```ini
[body_colliders]
debug_draw = true
debug_draw_person = 1
debug_draw_filter = spine02
debug_draw_capsules = false
```

`debug_draw_person` is global only: `0` shows all people, `1` through `4` select
Person01 through Person04. `debug_draw`, `debug_draw_filter`, and
`debug_draw_capsules` can also be set in a body sidecar; omitted keys inherit
the global settings. Changes use the existing INI live reload. Set the filter to
`all` and capsules to `true` to restore the full view. The defaults preserve
existing drawing. These controls affect body debug visuals only; physics and
the separate room/accessory debug controls are unchanged.

Filter names are case-insensitive: `pelvis`, `spine01` through `spine04`, `neck`,
`head`, `testicles01`, `testicles02`, `testicles`, or `penis` (the capsule chain).
Paired names `hip`, `thigh`, `knee`, `ankle`, `ball`, `breast`, `butt`, `clavicle`,
`shoulder`, `elbow`, `forearm`, `wrist`, `palm`, and `finger01` through `finger05`
select both sides. Prefix a paired name with `left_` or `right_` for one side.
Finger joints can be selected individually, e.g. `left_finger02_03` or
`right_finger01_end`. One name is accepted at a time; an unknown name hides
body shapes and logs a diagnostic. With capsules enabled, connections touching
the selected node(s) remain visible. Disable capsules to see just the selected
spheres/ovals. Filtering works in Hook5, Direct3D 8, and OpenGL drawing paths.

Run `python run_collider_debug_filter_tests.py` to check INI inheritance,
selection, and the production Hook5 geometry collector.

Penis collision capsules support these global and body-sidecar settings:

```ini
[body_colliders]
penis_radius = 0.0275
penis01_fine_offset = 0,0,0
penis02_fine_offset = 0,0,0
penis03_fine_offset = 0,0,0
```

`penis_radius` is a circular capsule radius, clamped to 0.001-0.25. Each offset
is a body-local XYZ displacement of that joint's collision point. The terminal
point also receives `penis03_fine_offset`, preserving the final segment's length.
The capsules share their adjusted endpoints. Raw animation pivots, gravity and
inertia samples remain unchanged; contact prediction, passive colliders and all
three drawing paths use the adjusted geometry. Edits reload live.

`collision_margin_radius` retains the old shared/testicle probe thickness
independently of the penis's dimensions. It is not an additional margin on top of
`penis_radius`. A legacy `chain_radius` value supplies either radius when that
replacement key is absent in the same INI. Sidecar keys override inherited global
values. The supplied configs preserve both previous radius values during this
migration, with zero offsets. Missing global keys default to 0.018 and zero offsets.
XYZ oval radii and separate per-segment radii are not implemented.

Run `python run_penis_collider_tests.py` for config, geometry, contact prediction,
joint-space solve and wire-drawing regressions.

For troubleshooting, check `Logs/NC-TK17-PhysX.log` and enable debug options only when needed, as verbose diagnostics can generate substantial output.

See [COLLISION-NOTES.md](COLLISION-NOTES.md) for the collision changes, automated
test command, limitations, and in-game checks still needed.
