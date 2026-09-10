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

## Build

Install the MSYS2 MinGW 32-bit toolchain, then run:

```powershell
.\compile-physx.bat
```

The compiled DLL is written to `build\NC-TK17-PhysX.dll`.

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

For troubleshooting, check `Logs/NC-TK17-PhysX.log` and enable debug options only when needed, as verbose diagnostics can generate substantial output.

See [COLLISION-NOTES.md](COLLISION-NOTES.md) for the collision changes, automated
test command, limitations, and in-game checks still needed.
