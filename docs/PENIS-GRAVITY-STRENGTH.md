# Penis gravity strength at every tilt

`[penis_physics]` now has two independent multipliers:

```ini
gravity_horizontal_strength = 1.0
gravity_vertical_strength = 1.0
```

Both default to 1.0, preserving the existing calculation. Set a value to 0.5 to
halve that component of the gravity target, 0 to disable it, or 2 to double it.
The allowed range is 0 through 4; negative values clamp to 0 and do not reverse
gravity. Joint limits, collisions and spring response still constrain the visible
pose, so half a gravity target does not necessarily produce half the final angle.

These controls remain effective at full tilt. They scale the final gravity-only
result after geometric shaping, including inverted-gravity contributions mapped
to that axis. Movement and wind are not multiplied. A weaker horizontal setting
therefore cannot be replaced by full geometric gravity once the pose settles.
The ordinary path is used when both values are 1, preserving its exact limit
ordering and floating-point calculation.

Horizontal and vertical refer to the existing gravity output mappings in
`[physics_environment]`: `gravity_horizontal_tail_axis` and
`gravity_vertical_tail_axis`. With the current mapping, the user's face-down
test produces mainly horizontal gravity, so start by trying
`gravity_horizontal_strength = 0.5`. These are body-local channels, not screen
directions. If advanced mappings assign both channels to one axis, their
multipliers combine by multiplication. An independently remapped inverted axis
outside those two axes retains its existing strength.

The existing `gravity_horizontal_curve` and `gravity_vertical_curve` controls
keep their meaning. They shape partial-tilt response; recovery starts at primary
gravity magnitude 0.90 and curves are bypassed at 0.98. Their valid range remains
0.1 through 8. Changing them is not a way to tune full-tilt strength.

Both new keys support global config reload and per-body sidecar overrides.
Omitted sidecar keys inherit the global values; removing a global key restores
1.0. They currently apply only to penis physics. Other body systems and the
approved smoother update modes are unchanged.

The installed config adds both controls at 1.0 without changing existing values.
Restart the game for the new DLL, then edit and save a strength value. Existing
config polling/debounce applies edits after the save settles. With debug enabled,
`body-chain gravity-strength` records show the effective per-person multipliers,
whether geometry is active, the unscaled gravity result and the combined target.

Validation: `run_body_dynamics_tests.py` exercises both axes/signs at full tilt,
startup before geometry is available, geometry confirmation and repeated updates
at 7/11/16/33 ms. It checks exact unit-strength equivalence, half/zero gravity,
retained movement, and real Windows INI loading/inheritance/clamping. In-game
visual verification of the new strength controls remains necessary.
