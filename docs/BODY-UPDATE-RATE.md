# Configurable body update rate

The existing body interval gate can publish fewer poses than the game renders.
Solver substeps improve integration within one update but only publish its final
pose. This opt-in setting schedules more actual body updates, using one shared
high-resolution clock sample per render frame.

Add `update_rate_hz` to any of the four existing body physics sections in
`Extensions/PhysX/Config.ini`:

```ini
[penis_physics]
update_rate_hz = -1

[testicle_physics]
update_rate_hz = -1

[breasts_physics]
update_rate_hz = -1

[butt_physics]
update_rate_hz = -1
```

Add these keys inside the existing sections; retain their other settings.

| Value | Behavior |
| --- | --- |
| `0` or omitted | Existing `interval_ms` scheduling, including its current minimum of 16 ms. |
| `-1` | Update on every rendered frame at ordinary game frame rates. Best starting point for visual smoothness. |
| `1`–`240` | Target this many updates per second, limited by rendered frames. `60` is a lower-cost option. |

Nonzero values override `interval_ms` for that section. At 90 render FPS, `-1`
can produce approximately 90 physics updates per second. A target of 60 produces
60 updates distributed over render frames, so some frames still hold a pose.
Targets above render FPS do not create extra unseen solves. All four sections are
independent; body sidecars can override the key, inheriting it when omitted.
Values below -1 fall back to 0; values above 240 clamp to 240.

The installed config is left unchanged. The bundled default documents the new
key with value 0. Restart the game after installing the DLL and selecting a rate.
Returning a section's key to 0 restores its original scheduling.

## Physics and timing

- Penis/testicle movement gain and their existing 48 ms input history remain time-normalized.
  Stiffness, damping, wind, gravity, limits and collision settings are unchanged.
  Smaller integration steps can slightly change motion/contact transients.
- Each accepted update runs the real solver and publishes its result. Collider
  refresh uses the same due decision and cannot reuse the previous render frame
  just because `GetTickCount` has not advanced.
- Lifecycle, binding and ownership timestamps retain their original clock domain.
- Positive rates carry scheduling deadlines across frames, avoiding the repeated
  interval-gate undershoot (for example, 60 Hz becoming 45 Hz at 90 render FPS).
  Missed deadlines do not queue catch-up solves.
- Existing millisecond motion samples retain accumulated time without drift;
  sub-millisecond frames wait until a nonzero millisecond sample is available.
  Existing stall protection still bounds simulation to 100 ms and clears stale
  movement input. Very low rates below 10 Hz encounter that protection normally.
- Breasts and butt use the same precise schedule and collider due checks. In
  nonzero mode, translation and rotation movement samples are normalized to
  the established 16 ms response before axis mapping and deadzones, including
  optional bone translation. Gravity, wind and constant sag are not scaled as
  movement. No extra movement smoothing/filter delay is added.
- Their nonzero mode advances up to 100 ms in angular substeps of at most 16 ms,
  matching the existing translation/contact time cap. Legacy mode retains its
  original 25 ms angular cap and unscaled movement samples. Faster sampling and
  removing that cap in the new mode can change transients compared with a slow
  legacy update; the reference movement gain and spring settings are preserved.
- Addon sidecar simulation rates are unchanged.

## Validation and in-game comparison

`python run_body_update_tests.py` tests the production scheduler at 30/60/90/120/
144/240 render FPS, legacy equivalence including DWORD wrap, rate switches,
pauses, actual Windows INI reads, actual QPC frame sampling, collider due checks
and collider cache reuse across repeated coarse ticks. The simulated 90 FPS loop
produces 90 updates/s with -1, and 60 with a target of 60.

`python run_paired_body_update_tests.py` checks exact legacy paired-body timing
and angular output equivalence, breast/butt movement gain and spring equilibrium
at multiple sample rates, bounded stalls/limits, and all four profile labels.
`python run_single_bone_contact_tests.py` exercises breast/butt translation,
body/room contact, moving supports, release, and output/handoff restoration.

Body motion, dynamics, gravity response, contact and collision-frame regressions
also pass. These establish scheduling and solver behavior in fixtures; visual
smoothness and FPS cost still require an in-game comparison.

With the existing `performance_profile = true`, active chains now emit a record
approximately every five seconds:

```text
performance profile body-update person=Person01 system=penis update_rate_hz=-1 published_hz=90.0 sample_ms=5000
```

`published_hz` counts completed body updates, not solver substeps or game FPS.
Compare the same animation, camera and collision workload with 0, 60 and -1.
Use the existing total physics timings and in-game FPS to judge the cost. More
body updates also mean more setup and collision work; a particular FPS loss
cannot be guaranteed from a standalone test.
