# Key Editor scrolling capture

The reported frame rates were 108 FPS with the Key Editor closed, 94 FPS
open and idle, and about 30 FPS during scrolling. FPS returns to 94 when
scrolling stops. These are user observations, not measurements from the
CPU sampler.

Three 30-second captures of the same scene are stored under
`The Klub 17/Logs/NC-TK17-Hook5-Extended/CPU-Capture/`:

| State | Capture stem | Samples | Context failures | Stack-read failures |
|---|---|---:|---:|---:|
| Closed | TK17-CPU-20260914-154508 | 1930 | 0 | 1 |
| Open, idle | TK17-CPU-20260914-154625 | 1928 | 0 | 0 |
| Scrolling | TK17-CPU-20260914-154743 | 1928 | 0 | 0 |

All captures selected thread 23216 in process 14268. The sampler briefly
suspends the selected thread to read its instruction pointer and frame-pointer
chain. Samples indicate where that thread spends time; they are not call
counts, GPU timings, or exact per-frame durations. Optimized callers can be
missing from the chain.

## Identified scrolling bottleneck

The installed PhysX DLL before the fix has SHA256
`385682A0A6D8EF946DA750E479B943102C2B9CE64CA4D604896930BE54B88AD1`.
Its disassembly at RVA `0xB600` matches
`addon_declared_name_matches_alias`: a direct `_stricmp`, then three
`_snprintf` calls constructing `S%s`, `local_%s`, and `local_S%s`, each followed
by a case-insensitive comparison. The formatting calls return at RVAs
`0xB676`, `0xB6AA`, and `0xB6ED`; the binary's format strings verify the source
association without relying on a linker map from another build.

671 distinct scrolling samples (34.8%) include one of those three return
addresses. Neither the closed nor idle capture has a sample at those calls.
The sampled runtime instructions are in MSVCRT's formatting machinery; this
is alias string construction, not physics integration or configuration reads.

The source path is:

`hook_Object_iNameSet` -> `addon_chain_name_interesting` ->
`addon_declared_sidecar_name` -> `addon_declared_name_matches_alias`.

The object-name hook is global, including UI objects. Each incoming name is
checked against parents and simulated targets in every enabled addon chain.
An unrelated name previously caused three formatted strings per candidate.
This is a concrete contributor to the scrolling slowdown. The capture does
not establish that it accounts for all of the slowdown, or explain all of
the smaller cost of leaving the window open.

## Fix and validation

Match recognized prefixes directly against the runtime string, retaining
case-insensitive exact matches for all four alias forms. No configuration
changes, physics disablement, or cached lookup results are introduced.
Declared names that themselves begin with `S` or `local_` retain their meaning.

`python Development/NC-TK17-PhysX/run_name_alias_tests.py` passes differential
checks against the previous algorithm, mixed case, prefix boundaries,
null/empty names, names up to the 127-character target limit, and active
sidecar discovery/filtering including immediate name changes. Its isolated
one-million-unrelated-name benchmark measured 477.860 ms before and 10.463 ms
after; these timings do not imply a corresponding whole-game FPS increase.
The full PhysX DLL builds successfully and `git diff --check` passes.

The original DLL is preserved in
`build/before-key-editor-alias-20260914/NC-TK17-PhysX.dll`.
After the user closed TK17, the built DLL was copied into the game binaries;
both copies have SHA256
`EA892C68201E1F5E46D8EEA9C1AB246AA71F89DCEAD3AF8ADE89F636C4EE913A`.
After restarting, the user reported about 108 FPS closed, 94 FPS open/idle,
and 54 FPS while scrolling, with PhysX still behaving normally. Scrolling
improved from approximately 33.3 to 18.5 ms per frame (30 to 54 FPS).
The idle-window cost is unchanged. Scrolling still costs about 7.9 ms above
open/idle, so a further capture is needed to identify the remaining work.

## Follow-up scrolling capture and Liquids string validation

`TK17-CPU-20260914-160503` records 1,929 samples of thread 21008 in process
19084 after the PhysX fix, with zero context and stack-read failures. MSVCRT
instruction samples fell from 759 to 130. The earlier alias-formatting spike
is absent. These are equal-duration captures, not equal numbers of frames.

155 sampled caller chains include Liquids RVA `0x1B21`, the return from
`VirtualQuery` in `ptr_readable`, verified against the installed DLL's
disassembly. Its caller chain is often incomplete, so this count does not
attribute every query to a particular higher-level function. Source inspection
shows `safe_cstr_a`, used by the global object-name hook's `stringref_cstr_a`,
queries memory once per character. This is avoidable without caching permissions
across calls. The separate Liquids name-formatting loop investigated earlier
does not dominate this capture.

The Liquids change scans only within the currently verified memory region,
checks the next region before crossing it, and preserves nonempty printable
ASCII, NUL termination, maximum length, and guard/uncommitted/inaccessible
memory rejection. Permissions are queried anew on every call.

`python Development/NC-TK17-Liquids/run_string_scan_tests.py` passes differential
byte/length checks, Windows page-boundary tests, direct/indirect StringRef tests,
and permission-change tests. A 255-character valid string goes from 256 queries
to one. The isolated 100,000 typical UI-name benchmark measured 1,030.997 ms
and 2,700,000 queries before, versus 39.348 ms and 100,000 queries after.
This is not a whole-game FPS measurement. The full Liquids DLL builds;
compiler warnings concern existing API casts, unused render functions and
frame-address access, outside the changed function.

Original Liquids SHA256:
`8B543264D39214C04180E3E3871714EF30F470A92E9D929C2CE53CAA31A52D35`.
Backup: `Development/NC-TK17-Liquids/build/before-key-editor-string-scan-20260914/NC-TK17-Liquids.dll`.
After the user closed TK17, the new Liquids DLL was installed and its SHA256
verified against the build:
`20952BD5FCE413BC28725663B31C04E65A9F50D6989CD432651DC451D7987242`.
The user retested and reported about 108 FPS closed, 94 FPS open/idle, and
64 FPS scrolling, with Liquids working normally. The second fix improved
scrolling from about 18.5 to 15.6 ms per frame (54 to 64 FPS). Compared with
the original 30 FPS, the two fixes together removed about 17.7 ms per scrolling
frame. The approximately 1.4 ms idle Key Editor cost remains unchanged, and
scrolling still adds about 5.0 ms above open/idle.

The user reports about 150 FPS when pressing J to hide the general GUI,
regardless of whether the Key Editor was open. Relative to that 6.7 ms
frame time, the normal visible GUI adds about 2.6 ms, and the GUI with an
idle Key Editor adds about 4.0 ms. A paired capture is pending to identify
the remaining work; the FPS comparison alone does not distinguish widget
processing, rendering, and extension overhead.

## Paired normal-GUI versus J-hidden capture

The normal GUI was visible with the Key Editor closed in
`TK17-CPU-20260914-161905` (1,928 samples, zero context failures, two stack-read
failures). J hid the GUI for `TK17-CPU-20260914-162005` (1,930 samples, zero
context or stack-read failures). Both selected thread 13104 in process 14876.
No runtime code or setting was changed between these captures.

| Instruction module | Visible samples | Hidden samples |
|---|---:|---:|
| TK17 SYS | 554 | 459 |
| Hook5 d3d8.dll | 78 | 30 |
| D3D11 runtime | 79 | 27 |
| Hook Effects | 19 | 3 |
| NVIDIA driver | 35 | 22 |
| Hook5 Extended | 322 | 425 |
| PhysX | 64 | 78 |
| Liquids | 2 | 2 |

These are fixed-duration instruction samples, not per-frame costs. A higher
share for an unchanged scene task with the GUI hidden does not imply that the
task became more expensive. GPU execution time is not measured here.

Disassembly ties the disappearing native paths to widget processing:

- SYS RVA `0x513D0` loops over children, resolves the `Visibility` member
  (`0x05FFF0D8`), and invokes each visible child's virtual method at slot +0x18.
  The +0x5142E return falls within the +0x51420 sample bucket.
- SYS RVA `0x51ED0` and its surrounding entry perform child positioning and
  size/layout work, with horizontal/vertical/reversed layout modes
  (`0x01FFF0E6` through `0x04FFF0E6`). Sampled return +0x51FCF is a recursive
  child-method dispatch. The recovered member IDs are listed in
  `Development/patch-sources/pemod/classes.h`.
- SYS +0x4ADF3 returns from a virtual call after scaling a four-component
  context rectangle and restores the rectangle afterward. Its visible
  descendant stacks include native UI code, Hook5, Hook Effects, D3D11,
  and extension state hooks.

Count each sample once per path; recursive occurrences must not be summed:

| Observed instruction/caller path | Visible samples | Hidden samples |
|---|---:|---:|
| SYS +0x4ADF3 | 211 | 1 |
| SYS +0x51420 bucket | 161 | 0 |
| SYS +0x51FCF layout dispatch | 50 | 0 |
| Hook5 +0x2E320..+0x2E493 interface batch upload/submission | 51 | 0 |

Rows overlap and are not additive. Optimized or deep call chains can be
truncated, so these counts are not complete timing attribution.

Hook5's recovered `flush_immediate_quad_batch` at +0x2E320 maps the dynamic
vertex buffer with WRITE_DISCARD, copies queued quads, unmaps, binds geometry,
and invokes Hook Effects. The queue used by DrawPrimitiveUP flushes at 32
quads, and SetTransform / private render state 0x89 can flush it earlier.
These behaviors are documented in `Development/Hook5/docs/SYMBOL_MAP.md` and
the decompiled functions. The captures do not measure actual batch occupancy
or establish that any particular flush is redundant.

Conclusion: the idle visible GUI adds ongoing native widget traversal/layout
and rendering work, plus the extension hooks along that rendering path.
J removes most of the observed UI paths; this is not merely the pixel cost of
an overlay. No single remaining hotspot comparable to the original alias
formatting spike is established. Further optimization should measure UI
invalidation frequency and batch/state-change counts before attempting to
cache widgets or suppress renderer calls. Such changes must retain animated
controls, text changes, tooltips, focus, clipping and scrolling. No additional
DLL modification or FPS improvement is claimed from this comparison.

## Scrolling after both fixes and the UI submission experiment

The diagnostic submission comparison is recorded in
`Development/NC-TK17-Hook5-Extended/UI-PROFILE.md`. Scrolling added roughly
5 ms/frame with almost unchanged submission counts and downstream API times.
The normal Hook5 Extended DLL was restored before this CPU capture.

`TK17-CPU-20260914-170554` sampled thread 4972 in process 13448: 1,930 samples,
zero context failures and one stack-read failure. PhysX and Liquids were both
the previously validated fixed builds. The GUI-scroll interval was announced
to the user immediately before starting the sampler.

The remaining cost is distributed: native SYS has 504 instruction samples,
NTDLL 229, Extended 206, MSVCRT 151, NVIDIA 144, MSVCR110 104, WIN32U 99,
native APP 89, PhysX 89, Hook5 79 and D3D11 73. These are samples of one thread,
not complete per-frame CPU or GPU timing. The earlier normal-GUI capture is
a different process and GUI state; it is not an idle-editor control for this
capture. Do not subtract its module totals to estimate scrolling overhead.

53 samples include Liquids return RVA `0x5890`, verified in the installed
binary as the memory-region query in the already fixed `safe_cstr_a`. This
does not mean the per-character query bug returned. The alias comparisons
remaining in PhysX at returns `0xA6B9` and `0xA6F0` are `_stricmp` and
`_strnicmp`, not the original formatting calls.

18 distinct samples include Liquids return `0x1C59B`. Disassembly verifies a
four-iteration `_snprintf` loop with format `Person%02dSpermray`, inlined from
`liquid_person_spermray_root_name` into the global name-hook path. It is reached
through `remember_liquid_transform_node` even for unrelated UI object names.
This is a smaller confirmed avoidable cost, not an explanation of all remaining
scrolling or idle-GUI overhead.

The lookup now compares the same four fixed string literals using the same
case-insensitive comparison and person-index result. No formatting is needed.
`python Development/NC-TK17-Liquids/run_root_name_tests.py` passes differential
tests against the prior implementation: persons 00-99, all ASCII case
combinations, byte mutations, truncations, extra suffixes, null and empty
strings, and prefixed names used by callers. The isolated 100,000-name mismatch
benchmark measured 90.121 ms before and 1.908 ms after. This is not a gameplay
FPS measurement. The full DLL builds successfully with existing warnings;
`git diff --check` passes.

Candidate SHA256:
`AB4D0C2DA67BEDC258AB75C687E932CAB97DD29A48005439594A0282673FCFC0`.
The current installed DLL is backed up under
`Development/NC-TK17-Liquids/build/before-key-editor-root-names-20260914/`,
SHA256 `20952BD5FCE413BC28725663B31C04E65A9F50D6989CD432651DC451D7987242`.
After the user confirmed TK17 was closed, the candidate was installed and its
SHA256 verified against the built DLL. The installed PhysX hash still matches
the validated alias fix, and Hook5 Extended still matches the restored normal
build. The user then reported approximately 110 FPS with the editor closed,
94 FPS open/idle, and 64 FPS scrolling, with Liquids still working normally.
The previous readings were 108/94/64: there is no measurable improvement in
idle-editor or scrolling performance from this last patch. The small closed
GUI difference cannot confidently be attributed to it. Retain the simpler
equivalent lookup, but do not treat its isolated benchmark as a gameplay gain.

### Next distinction: pointer input versus scrolling

The existing scrolling capture contains 40 instruction samples at SYS
`+0x4A7D0` and two at `+0x4A7BB`, with recurring caller frames
`+0x45EB8`, `+0x4ACBF` and `+0x4A7D7`. Disassembly of the installed SYS verifies
that `+0x4A790` iterates a child array at object +0x14 in reverse order and
dispatches virtual slot +0x1C; the return is +0x4A7D7. The rest of that routine
handles input/event fields. This is a concrete native input-traversal path,
but the samples do not establish whether mouse motion, wheel events, or a
per-frame input pass accounts for its cost. Repeated recursive frames must
not be counted as separate samples. Before considering a native patch, compare
pointer motion over the idle Key Editor without scrolling against its stationary
idle FPS, keeping the scene and editor contents fixed.

The user performed that comparison: idle Key Editor about 94 FPS versus
86 FPS while moving the mouse without clicking or scrolling. The same motion
over the normal PoseEditor GUI, without the Key Editor, changes about 112 FPS
to 98 FPS. These correspond to approximately 0.99 and 1.28 ms/frame additional
cost, respectively. This establishes a shared GUI mouse-motion penalty,
separate from the larger scrolling penalty. It does not alone establish that
native widget dispatch, Hook5, window-message handling, or another hook owns
that time. No mouse-motion-only CPU capture has yet been taken.

The archived `Scripts/Shared/GUI/uiPoseEditKeyEditor.bs` defines the two scroll
frames and EditKey handlers for mouse button down/up/hold events. It contains
no obvious script mouse-motion loop explaining the general GUI observation.
No script dimensions, input behavior, polling rate, or clipping rules were
changed. The next capture should isolate mouse motion over the normal GUI
with the Key Editor closed, rather than mix motion with scrolling again.

## Mouse motion over normal PoseEditor GUI

`TK17-CPU-20260914-172707` sampled thread 24200 in process 20576 during the
announced normal-GUI mouse-motion test: 1,929 samples, zero context failures
and zero stack-read failures. The Key Editor was closed; no click or scroll
was requested.

120 instruction samples land at WIN32U +0x11AC. The local 32-bit WIN32U export
table identifies the containing stub as `NtUserPeekMessage` (+0x11A0). The
stacks pass through USER32 +0x32661 and gameoverlayrenderer +0x91C0D to APP
+0x6387 (83 samples) or +0x63CA (37). Disassembly and imports of the installed
APP identify both returns as `PeekMessageW` calls in its queue-draining loop
at +0x6330, with PM_REMOVE and no HWND/message filter. Messages are dispatched
between those calls. This establishes the API path, not its event rate or
whether the calls are redundant.

33 samples land in `NtUserMessageCall` (+0x11E0 stub) via SYS +0x92B7C; the SYS
import at +0x153408 is `DefWindowProcW`. 12 samples land in `NtUserSetCursor`
(+0x1310 stub) via the executable +0x6E3A9. These are sampled CPU/system-call
paths, not draw submissions. Native child input-dispatch return SYS +0x4A7D7
appears in 53 distinct samples.

For orientation only, the earlier stationary normal-GUI run had two samples
in this PeekMessage path and 41 containing the native child-dispatch return;
WIN32U instruction samples were 17 versus 191 during mouse motion. These were
different processes, with a small Liquids lookup change between them, so this
is not a precise paired timing experiment. It points more strongly toward
Windows message handling for the additional motion cost than toward the
previously suspected native widget traversal alone.

The SYS WM_MOUSEMOVE branch at +0x92726 handles position/button data, invokes
the input callbacks, then joins the common DefWindowProcW tail. No messages
were suppressed and no input callback was bypassed. The Steam overlay hook's
presence in the PeekMessage stack is not evidence that Steam causes the cost.
Message rates and the user's mouse polling configuration are not yet known;
do not assume a high-rate device or introduce event dropping based on samples.

### Polling-rate comparison

The user's Logitech G HUB screenshot identifies a PRO 2 LIGHTSPEED with both
wired and wireless report rates set to 1,000 Hz. At that setting, the reported
normal PoseEditor GUI readings were approximately 112 FPS stationary and
98 FPS during pointer motion. At the requested temporary 125 Hz setting,
the user reports approximately 112 FPS stationary and 103 FPS moving.

| Mouse report rate | Stationary | Moving | Added frame time during motion |
|---|---:|---:|---:|
| 1,000 Hz | 112 FPS | 98 FPS | 1.276 ms |
| 125 Hz | 112 FPS | 103 FPS | 0.780 ms |

Reducing the rate recovered about 0.495 ms/frame during motion (roughly
39 percent of the reported motion penalty), with no observed stationary
change. These are approximate user readings, not repeated timed trials.
The result supports report-rate-sensitive input overhead as one contributor;
it does not establish a specific redundant handler or identify an overlay as
the cause. Approximately 0.78 ms/frame of the motion penalty remains at the
lower rate. This test did not measure Key Editor scrolling at 125 Hz and must
not be used to claim a scrolling improvement. The 125 Hz setting was only a
diagnostic comparison; returning to the user's original 1,000 Hz setting
keeps subsequent results comparable with the earlier game tests.

## Shared frame-work investigation: PhysX readability cache

The user authorized investigating performance with the GUI either visible or
hidden. In the existing paired captures, 127 of 1,928 visible-GUI samples and
144 of 1,930 hidden-GUI samples contain PhysX return RVA +0x3AE1. Disassembly
of the installed PhysX DLL verifies this return is the `VirtualQuery` call in
the cached `ptr_readable` function. This is shared overhead, not a UI-only path.
These counts are not call counts or exact per-frame timings.

For comparison, verified Extended AVX skinning instructions account for 145
visible and 224 hidden instruction samples; its scene-child traversal loop
accounts for 93 and 132. Those are separate routines: the hot +0x1A5C0 bucket
is scene traversal, not skinning. Both already have specialized implementations.
The first bounded change targets the avoidable memory-query cache misses.

The readability cache stores whole VirtualQuery regions but selects an entry
by the address's page hash. Successive accesses to different pages within the
same region can therefore repeat the same query, even though the preceding
entry already covers that address. The change checks the most recently used
entry before the hash lookup. It stores only an index into the existing
thread-local table, retains the original range checks, and honors the same
frame epoch on both lookup paths. A replaced hash slot cannot leave an extra
stale copy of a region. No cache lifetime is extended beyond the existing
frame policy. As before, this cache does not detect protection changes within
a frame; changes are observed after the frame epoch advances.

`python Development/NC-TK17-PhysX/run_readable_cache_tests.py` extracts the
production implementation and passes Windows tests for region boundaries,
read-only/guard/uncommitted pages, hash-slot collisions, failed lookups,
null/zero/overflow ranges, per-thread entries, cross-thread epoch advancement,
and differential mixed-range results against the previous implementation.
The 64-page contiguous-region fixture goes from 64 queries to one per frame.
Across 4,000 synthetic frames (256,000 checks), the page-striding benchmark
measured 3,968.398 ms/256,000 queries before and 65.101 ms/4,000 queries after.
Repeated same-page checks measured 64.871 versus 65.081 ms, with 4,000 queries
in both cases. These synthetic access patterns do not predict in-game gains.

The full PhysX DLL builds and `git diff --check` passes. Candidate SHA256:
`EA1564846ED0DCDEBA15265DD21D3C1193127F27FF10055311D2AF045A838FA9`.
The previous installed DLL, including the validated alias fix, is backed up at
`Development/NC-TK17-PhysX/build/before-region-cache-20260914/NC-TK17-PhysX.dll`,
SHA256 `EA892C68201E1F5E46D8EEA9C1AB246AA71F89DCEAD3AF8ADE89F636C4EE913A`.
The installed DLL was available for exclusive write access, so the candidate
was installed and its hash verified. Hook5 Extended remains the normal build
and Liquids retains its latest validated lookup change. In-game validation
and FPS results for this PhysX cache change are pending.
