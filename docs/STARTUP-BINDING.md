# Sidecar startup layout recovery (2026-09-08)

This follow-up changes sidecar joint initialization, not the confirmed collision
solver. It applies to compatible sidecars generally; no Fiesta-specific rule
was added. In-game confirmation is pending.

## Evidence

In the 20:10 session, `NcHair15_tail01` received rotation/translation offsets
`0x06c/0x07c` on the initial binding. The write guard accepts that layout only
for room targets. The children received `0x038/0x048` and started simulation,
while the root did not. Re-equipping at 20:15:17 produced `0x038/0x048` for
the root and all three joints started.

The failed root's scene-pose scans reported no candidate offsets. Those scans
require a readable 4 KB window, whereas the live matrix needs only bytes
`0x018..0x053`. An allocation boundary can therefore reject a valid matrix;
the previous bounded check covered raw pointers only, and this binding had
no raw pointer. The log establishes the rejected layout, but does not contain
a memory dump proving the matrix contents at the original address.

## Change

- Check the bounded matrix through `s_object` as well: require sane basis
  vectors, an approximately orthogonal normalized basis, and translation
  matching the scene file before selecting `0x038/0x048`.
- Recheck rejected cached layouts at most once per second. Preserve the write
  guard's accepted offsets and its three-sample, 32 ms stabilization period.
- Log `addon safety layout retry` and `addon safety layout recovered` so a
  persistently rejected binding is visible. Healthy siblings keep running.

The existing raw-pointer and native room layouts remain supported. Retry does
not force an unsafe mapping, change collision settings, or re-equip an addon.
An invalid layout that cannot be identified safely remains quarantined.

## Validation

`python run_binding_tests.py` compiles the actual production layout and guard
functions. It exercises a valid wrapper matrix immediately before a protected,
inaccessible page, unreadable pointers, invalid matrix vectors, translation
mismatch, recovery and stabilization, retry throttling across tick-counter wrap,
independent sibling readiness, and preservation of native room layouts.

Binding and collision regressions passed, and `compile-physx.bat` built the DLL.
The build and installed DLL both have SHA256
`3869A266548B8551DD3F08C08F71A786D7132CCCE6BEA35740C1F6B519178E7F`.
Test a fresh room load with the hair already equipped, then body movement,
camera movement, and re-equipping. Confirm the root's simulation starts and the
resting-collision behavior remains at the checkpoint.

The prior source, DLL and evidence log are preserved locally in
`build/collision-backup-startup-binding-20260908-202120/`. The confirmed collision
release ZIP was not regenerated for this unconfirmed startup follow-up.

## Camera protections observed

`physx_sidecar.c` measures parent movement relative to the model and uses the
captured camera transform to compensate world movement and forces. Its root
drive quarantine discards suspect impulses while continuing spring/gravity
simulation. `physx_physics.c` contains corresponding body-chain and testicle
root-drive safeguards.

The evidence log records both `camera_relative=1` parent sampling and
`root-drive camera-rebased` for sidecars and body physics. This establishes that
the safeguards ran, not that every camera/update-order combination is immune.
The startup write-guard rejection occurs before root-drive processing.

The older hold/release code after the unconditional `return 1` in
`addon_chain_camera_safe_gravity_drive` is unreachable. The active path consumes
the same-tick camera-compensated gravity candidate directly. No camera behavior
was changed in this follow-up.
