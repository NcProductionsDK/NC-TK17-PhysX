# DriverBra1 equip crash

The September 13, 17:42 reproduction recorded access violation `c0000005`
inside `ThriXXX010278-SYS.dll`, RVA `0x8aec7`, during
`addon-body-local-pivot`. The supplied raw/resolved addresses were
`89982e20` / `3268d080`. The fault read address was `0x10c`, with EAX zero.

Disassembly of the installed engine's `GetModelViewRotationPivot` confirms
that the entry loads metadata from `[object-0x18]`, then its dispatch table
from `[metadata+0x10c]`. The supplied resolved pointer had null metadata.
The PhysX call therefore passed an incompatible object to the engine API.
This was not a numeric gravity failure or the automated-test completion log.

The addon pivot caller now checks that metadata/dispatch path and the first
getter's executable address. It retains a valid resolved pointer, otherwise
tries the raw script object from the same exact target binding. If neither
passes, it returns an unavailable sample without calling the engine. Other
binding fields and transform output pointers are not rewritten.

The existing diagnostic observer remains enabled at startup with debug=true,
and writes first-chance faults to `Logs/NC-TK17-PhysX-fault.log`. These records
are observations, not necessarily unhandled crashes; exception handling is
unchanged. Raw stack words in that report are not a reconstructed call stack.

Validation: `run_addon_pivot_tests.py` exercises the production caller using
the recorded null-metadata condition, raw fallback, unchanged valid resolved
selection, invalid/unreadable metadata, missing/non-executable getters and
owner gating. Collision and fault-observer regressions pass. The 32-bit DLL
builds. In-game confirmation of raw binding compatibility and restored addon
collision is pending; keep debug enabled for the next equip test.

The earlier v1.10 scene-reference correction did not resolve the crash.
The friend's v1.11 is a different asset version, but its final log reaches
the same activation stage. A successful local retest does not by itself prove
the friend's build is fixed.
