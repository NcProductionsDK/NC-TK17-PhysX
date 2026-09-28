# Same-frame memory-region reuse

The September 20 gameplay capture observed 66 of 1,936 main-thread samples
inside `ZwQueryVirtualMemory` returning to the PhysX readability checker.
Eight additional query samples came from the executable-address check, which
this change does not modify.

The existing readability cache has a last-region shortcut and a 256-slot
page-address hash. After accessing a different region, reading a new page of
an already-checked allocation can miss both shortcuts and query Windows
again. Hash collisions also evict reusable whole-region information.

The new fallback retains 16 positive region descriptions per thread. Only
misses in the existing shortcuts search those entries; a match repopulates
the page slot. Entries retain the existing frame epoch and are invalidated
by the same D3D8/OpenGL presentation hooks. The change adds 196 bytes of
thread-local storage on Win32. No mapping is cached across frame epochs.

The integer range check also rejects overflowing address ranges before
constructing the end pointer. Null and non-null zero-length behavior is
unchanged. Writable and executable checks, physics equations, update rates,
collisions and configuration values are unchanged.

## Validation

`python Development/NC-TK17-PhysX/run_readable_cache_tests.py` passed:

- Differential mixed-region reads against the immediate previous algorithm,
  including its last-region optimization.
- Guard, read-only, no-access and uncommitted memory; spanning protected
  boundaries; zero-length, null and overflowing ranges.
- Page-hash collisions, more regions than the new fallback can retain,
  thread-local separation and cross-thread frame-epoch invalidation.
- Two or 16 alternating regions, each traversed over 64 pages, require only
  two or 16 Windows queries respectively within the frame.

Seven alternating-order benchmark trials produced these median batch times:

| Synthetic workload | Previous | Candidate |
|---|---:|---:|
| Same page, 64 checks/frame over 1,000 frames | 1.306 ms | 1.322 ms |
| Alternating two allocations/pages, same check count | 42.288 ms | 2.672 ms |
| Alternating 16 allocations/pages, same check count | 42.286 ms | 15.653 ms |
| One cold query/frame over 1,000 frames | 1.012 ms | 1.017 ms |

These are synthetic CPU batch timings, not in-game frame times or predicted
FPS. Actual benefit depends on reuse and contention in the running scene.
The full DLL builds successfully and source whitespace checks pass.

## Deployment and rollback

Candidate installed DLL SHA256:
`B787841CCD075130924697B200BDFE4266037FED952A5C4D2C210B9A466AB16E`.

The previous installed PhysX DLL, source, test script and user config are
backed up in `build/before-region-reuse-20260920-010800`.
To roll back the runtime candidate with TK17 closed, restore that directory's
`NC-TK17-PhysX.dll` into `The Klub 17/Binaries`. No INI change is required.

Hook5-Extended stays at SHA256
`8A84CB45A6B88A503A44CAEBF84FA6A72FFA105E86D31899A9CADE78A9A1F9D7`,
with unified wetness enabled. Transparency calls were not deduplicated:
the verified APP routine performs dynamic member resolution and dispatches
object-update notifications; equality of the requested layer alone is not
sufficient to prove that bypassing a call preserves behavior.

## In-game follow-up

The user reported approximately 80 FPS after restarting with the candidate,
compared with approximately 60 before. The subsequent same-view capture is
`TK17-CPU-20260920-011224.csv`, with its summary and matching configuration
copies in the same CPU-Capture directory as the earlier capture.

Both captures ran for 30 seconds. The main thread had 1,936 observations
before and 1,932 after. PhysX readability-query observations were 66 before
and 77 after; executable-query observations were eight and seven respectively.
Together these account for 3.82% and 4.35% of main-thread observations. PhysX
self samples were 69 before and 60 after (3.56% and 3.11%). Neither capture
had thread-context read failures. These are sample locations, not query
counts or per-frame timings, and the possible frame rate differs between runs.

Thus the synthetic tests establish fewer queries for the targeted access
patterns, but the gameplay samples do not establish a reduced query-time
share or prove that this patch caused the entire reported FPS recovery.
The live video workload also varied: the decoder thread used about 4.56
billion cycles in the initial baseline measurement and 3.71 billion in the
follow-up. No alternating baseline/candidate gameplay trial was performed.
The tested candidate remains installed; use the preserved baseline for any
further controlled comparison.
