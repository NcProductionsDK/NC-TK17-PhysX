# Sidecar free-motion follow-up (2026-09-08)

The subsequent accepted [sidecar motion follow-up](SIDECAR-MOTION.md) adds
length-consistent inheritance and continuous solver-velocity publication. The
confirmation and hashes below describe this preceding free-motion checkpoint.

Status: automated checks passed; the user confirmed that free-motion nudging
was fixed and sidecar collisions improved as well. This applies to
sidecar bone/matrix and object-transform local-bend simulation generally.

Built and installed DLL SHA256:
`A206FE70BD67CABF6BD67F8C08C606750CFD56A6DFB7D5DA1004488633DE42B9`.

## Evidence and change

The test session beginning 20:27:22 had global wind disabled. NcHat008 also
had collisions disabled. Its second ear joint showed small changes in pose,
with larger changes around camera-quarantine events. Fiesta had moving samples
with no contact correction. The log is sampled, so it cannot identify the cause
of every individual nudge.

Two implementation problems were addressed:

- Child spring inheritance previously changed velocity after position had
  already advanced. Gravity and inherited forces thus acted at different
  integration stages. A diagnostic replay settled at fixed frame durations,
  but changed its resting angle when frame duration changed.
- Camera quarantine changed sidecar stiffness/damping and held then released
  child poses. Child inherited stiffness was not scaled with the main spring,
  so guard transitions could change the balance of forces.

`addon_chain_integrate_free_motion` combines the spring, gravity/wind resultant,
inherited spring and parent-velocity following in one update. It projects
acceleration and velocity tangent to the fixed-length link before movement,
normalizes afterward, and uses substeps no longer than 1/240 second (at most
12 under the existing 50 ms timestep cap). Damping and velocity following are
combined into a stable drag term. No settling deadzone or free-motion sleep
was added.

Camera guards still filter suspect parent/root inputs. Camera state no longer
changes sidecar spring coefficients or freezes child poses. The body-physics
camera-coast settings and body solver are unchanged. Existing collision
constraints, contact sleep, and startup binding checks remain in place.

## Verification

```powershell
python run_free_motion_tests.py
python run_binding_tests.py
python run_collision_tests.py
.\compile-physx.bat
```

The new suite compiles the production integrator and camera quarantine helper.
It verifies the same resting position with/without inheritance at 30/60/144 Hz,
alternating 16/17 ms, and occasional 50 ms frames. All tested settled angular
ranges printed zero to eight decimal places. It checks fixed link lengths,
tangent velocities, continued momentum, small parent motion, changing forces,
and plane contact/rest/release using production contact math. Camera tests
exercise quarantine toggling with fixed compensated force inputs; they do not
reproduce the full engine camera-capture pipeline.

The resting pose and motion feel can differ slightly from the former
timestep-dependent update. Test both NcHat008 and Fiesta freely hanging, then
camera orbit/pause, small body movement, and body/room contact. Residual noise
from engine transform timing or force sampling remains possible.

The previous working source and DLL are preserved in
`build/collision-backup-free-motion-20260908-203827/`. After in-game confirmation,
the release ZIP was refreshed to this build. The newly confirmed source and DLL
are saved in `build/collision-backup-confirmed-free-motion-20260908/`, alongside
the previous release ZIP.
