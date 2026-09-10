# Plugin review after the confirmed body checkpoint

Reviewed 2026-09-09: simulation scheduling and integration, body and sidecar
contact integration, configuration/reload, diagnostics, public API and hook
installation/teardown. This is a targeted source review, not exhaustive engine
compatibility testing. The latest body collision fix is confirmed in-game;
the items below are separate improvement opportunities, not explanations for
an unresolved collision complaint.

## 1. Preserve elapsed time with bounded body-physics substeps

Follow-up: [BODY-MOTION.md](BODY-MOTION.md) describes the implemented penis and
testicle timing changes, now accepted in-game. The observations below describe the reviewed checkpoint;
breast and butt paths still retain the original timing behavior.

**Confirmed implementation limitation; highest-value physics follow-up.**
The body updates in `physx_physics.c` and `physx_butt.c` set `last_tick = now`
and clamp `dt` to 0.025 seconds. `physx_render_hooks.c` runs the simulation once
per presented frame. When an update is eligible on every frame, 30 FPS therefore
advances only 0.75 seconds of body simulation per wall-clock second; 20 FPS
advances 0.5 seconds. Larger configured update intervals have the same issue.
There is no remainder accumulator in these body paths.

This affects response timing and motion-input scaling, even though the small
step cap helps keep large frame stalls bounded. The sidecar integrator already
divides its accepted interval into smaller steps. A body follow-up should use
bounded substeps with explicit stall handling and consistent sampled animation
inputs; simply removing the cap would expose the angular springs to large steps.
Keep collision solving and output publication coherent with those substeps.

Validate real body update functions at 20/30/60/144 FPS, long stalls, camera
movement, configured intervals and release from moving contacts. The current
contact fixtures exercise solver timesteps, but do not execute the complete
engine update scheduling path.

## 2. Validate configuration before replacing live values

**Confirmed parser/reload weakness; good isolated robustness follow-up.**
`profile_float` in `physx_config.c` uses `atof`: malformed text becomes zero,
numeric prefixes accept trailing garbage, and non-finite values are not rejected
there. `parse_vec3` similarly accepts parsed components without a finite check.
The comparison-based `physx_clampf` in `physx_sidecar.c` does not sanitize NaN.
These helpers feed stiffness, damping, gravity and other simulation parameters.

Use checked scalar/vector parsing that preserves the previous/default value on
invalid input and reports the affected key once. Keep intended range clamping
and existing valid INI syntax compatible. Test malformed values, overflow,
NaN/infinity, comments/whitespace and partial vector input.

`refresh_global_config` debounces the global INI for 1500 ms, but immediately
reloads changed person body sidecars. Bring that path under an equivalent
settled-write check and validate a temporary configuration before publishing it.
No malformed-INI failure was reproduced in the user's gameplay capture.

## 3. Reduce the cost of diagnostic logging

**Confirmed synchronous work; actual frame-time cost needs measurement.**
Every accepted `log_line` in `NC-TK17-PhysX.c` rebuilds the path, takes the log
lock, opens the file, writes a timestamp/message and closes the file. Simulation
and hook paths call this directly. Debug mode enables frequent per-chain records,
so diagnostics can perturb the timing being investigated. Normal-mode filtering
already avoids much of this work and should remain.

First cache the path and batch bounded diagnostic records per frame with a clear
flush policy. Consider a writer queue only if measurements justify its shutdown
and concurrency complexity. Compare debug on/off using the existing performance
counters, and verify that actionable errors and final records remain available.

## 4. Make the DLL's lifetime contract explicit

**Source-level unload hazard; not a reproduced gameplay crash.**
`DllMain`'s explicit-unload path restores selected engine hooks and resets physics.
However, `install_inline_hook` also patches ConfigEditor, AppMain and PersonContext
entry points, and `patch_iat`/renderer patching install additional callbacks. The
detach path does not restore all those call sites; several patch paths do not
retain a complete restoration record. A continuing process could call unloaded
plugin code after a real `FreeLibrary` unload. Normal process termination takes
the separate early-return path and is outside this finding.

Decide whether the extension is process-lifetime-only or needs supported runtime
unload. For runtime unload, track owned patches and original bytes/slots, stop
callbacks before teardown, and restore only entries still owned by this plugin
so another extension's hook chain is preserved. Exercise this in an isolated
host before attempting it in-game. Keep restart-based DLL installation for the
current checkpoint.

## 5. Retain the new pose evidence and expose fallback reasons

**Known compatibility boundary, not a reason to overhaul confirmed contacts.**
`body_contact_capture_step` uses composed prediction only for a validated live
sample in the standard collider basis. Camera-held geometry, incompatible rigs
and singular fits retain the old approximation. The `composed` diagnostic flag
shows which model ran, but not why fitting was rejected. Add inexpensive counters
by reason if another rig exposes a problem; collect its engine/output pose pairs
before extending the fit. Preserve the regression recordings that caught the
combined-axis mismatch.

Penis and testicle chains still solve sequentially rather than simultaneously.
The confirmed result does not justify replacing that coupling now. Revisit it
only with a reproducible moving-support/order-dependence case and a regression.

## Cleanup completed at this checkpoint

`Bugs.md` also retains a known PoseEditor track-handoff problem: loading an
animated pose, starting a blank pose and disabling penis physics can restore
the previous pose's tracks. This predates the collision work. It needs a
separate ownership/restoration trace across scene transitions; this review has
not reproduced or diagnosed it, and the collision checkpoint does not fix it.

- Backed up the exact confirmed source, DLL, tests and log before edits.
- Removed write-only body contact-filter storage and its obsolete reset writes.
- Removed unvalidated raw matrix reads/rows from body pose diagnostics; kept
  the useful bounded point/Euler audit and replay tests.
- Updated checkpoint, build/test and release documentation to include body fixes.
- Retained live compatibility fallback code, historical analysis, and rollback
  binaries under ignored backup directories. No broad deletion or physics
  retuning was performed.

Recommended next work: configuration validation as an isolated robustness fix,
then body timestep handling as the next physics experiment. Measure diagnostic
overhead before choosing a logging redesign.
