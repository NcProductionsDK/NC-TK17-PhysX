# PhysX code optimization and continuous wind — 2026-09-12

This change reduces repeated sidecar setup and wind calculations. It does not
edit the installed config or bundled defaults, change solver cadence, reduce
collision quality, or change gravity/body response settings. Gameplay testing
of this build is still required.

## Changes

- Active-addon, equipment-zone and equipment-definition lookups use bounded,
  thread-local positive hints. Each hit checks the actual record's current keys;
  ownership, selected sidecars, roots and timestamps are still read live. Missing
  keys take the original search path. Replaced entries cannot satisfy stale hints.
- Invalid chains, room chains and chains in their existing settle window avoid
  unused PoseEditor visibility queries. Established chains with the same live
  owner root avoid an unused vector length calculation. Activation rules,
  placeholder protection, room write validation and readiness remain intact.
- The sidecar force path computes its separate gravity-only report vector only
  when debug or gravity diagnostics consume it. Simulation still maps the combined
  gravity/wind resultant once, including the existing inverted-gravity response.
- Wind caches the hash-derived phase constants, reuses identical same-time force
  samples, and shares the sway sine among targets with the same chain identity
  and sway frequency. Zero turbulence skips its two sine evaluations. All cache
  hits validate the relevant hashes and parameters; live setting changes take
  effect even within the same timestamp. Name hashing itself remains unchanged.

## Wind behavior

The original sampler used `now % 3600000`, jumping back to zero every hour.
The new sampler starts at that legacy phase but advances an integer millisecond
clock continuously, including through the 32-bit system-clock wrap. Double
precision time reduces phase quantization and avoids drift across frame rates.
As with any unsigned 32-bit elapsed-time calculation, successive samples must
be less than one full clock wrap (about 49.7 days) apart.

Wind direction, strength, gust frequencies, per-target variation, signed sway,
clamps and body/clothing/room response mappings remain the same. Existing shared
chain sway already provides coherent movement of connected room joints, so this
change does not introduce a new wind model or retune the user's materials.
The intended motion difference is smooth progression through the former hourly
reset and more precise phase timing. Body and sidecar wind use the same sampler;
per-chain sway overrides and ownerless room chains retain their existing routing.

## Evidence and limits

The supplied pre-change profile's last 30 windows cover 30.109 seconds and 1,526
physics ticks. Weighted average plugin time was 3.7348 ms/tick: addons 1.6316 ms
(including activation/setup 1.1174 ms), body systems about 1.4898 ms, and collider
refresh 0.4178 ms. Nested phases must not be added to their parent phase. These
numbers describe that scene and plugin timing, not an attributable FPS gain.

The isolated benchmark compares frozen original functions with current
production code using the same 32-bit MinGW `-O2` build. Initial runs measured
roughly 55–56% less time for repeated active-slot lookups, about 50% less time for
paired wind samples, about 17% less time for a 32-target wind chain, and about
55% less time for wind with zero turbulence. Cache misses, actual equipment
counts and enabled wind channels affect savings. These percentages apply only
to the named helper workloads, not the entire plugin or game.

Validation:

- `python run_optimization_tests.py`: compares registry records/results across
  insertion, misses, case changes, capacity eviction, clock wrap and resets;
  exercises wind cache collisions, same-time parameter edits, enable/disable,
  body/clothing/room routing, overrides, clamping and 20/30/60/144 Hz schedules.
  Wind stays continuous across the old hourly boundary and system-clock wrap.
  Against the old float-time sampler, the first-hour fixture's largest force
  difference was 0.00144243 units; differences are expected from higher precision.
- `python run_force_activation_tests.py`: 50,000 activation fixtures have exactly
  matching decisions, outputs and state. 30,000 force fixtures have identical
  target state and agree within 0.000001 force units (observed maximum
  0.000000238419), including debug reporting and uninitialized gravity mappings.
  The small force rounding difference is consistent with 32-bit x87 evaluation.
- Existing binding, free-motion, gravity-response, body-motion, body-dynamics,
  collision, body-pose, body-contact, collision-frame and single-bone-contact
  suites pass. The production DLL builds successfully.

For an in-game comparison, repeat the previous scene, camera, equipment and
actions with the same config. Compare settled profile windows after loading,
especially total, addon activation and addon drive time; record game FPS too.
Check clothing changes, room transitions and body/clothing/room wind motion.
No in-game FPS improvement has been measured for this build yet.

The previous installed DLL, edited source files, config hash and original
profile log are backed up locally in `build/before-wind-optimization/`. The
backup and generated benchmark files are ignored by Git.
