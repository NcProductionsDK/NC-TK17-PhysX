Physics presets
===============

Put INI files in `Extensions/PhysX/Presets`. Open the PhysX settings screen to
refresh the list. Default always comes first; the remaining names are sorted
filenames without `.ini`. Selection takes effect on the next physics update.

Only these sections are read from presets:

- `[breasts_physics]`
- `[penis_physics]`
- `[testicle_physics]`
- `[butt_physics]`

Each option uses the add-on body's sidecar first, the selected preset second,
and `Config.ini` last. An omitted option inherits the value below it. Default
skips the preset layer and keeps sidecar overrides. Existing angle aliases
such as breast/butt `joint01_max_angle` remain supported.

`Config.ini` remembers the filename, including `.ini`, in `[presets] selected`.
`DEFAULT` is the built-in selection; a file named `Default.ini` is a separate
preset. Physics controls and per-person physics toggles save to the selected
preset, or to `Config.ini` under Default. Other controls always save globally.
The controls show the global/preset values; a body's sidecar can still override
those values for that body.

Edits to the selected file reload after the normal polling interval and a
1.5-second save delay. Switching presets never copies physics values into the
global file. A missing selected file resets the selection to Default and logs
the reason. Reopen the settings screen after adding or removing preset files.

Validation: `python Development/NC-TK17-PhysX/run_preset_config_tests.py` exercises
the real config loader using temporary files, without starting TK17.
