# Genital reveal investigation (historical)

2026-09-15. This records the original investigation into making the first
visible frame show a prepared PhysX pose. That approach was superseded: the
user preferred preserving the visible starting pose and letting simulation
settle smoothly, and has now confirmed that it works without distracting snaps.

Current behavior, implementation and validation are maintained in
[GENITAL-VISIBILITY.md](GENITAL-VISIBILITY.md). The findings and proposed
experiment below are historical; they are not outstanding work or descriptions
of the current solver.

## Findings

The remaining transition has identifiable causes beyond visibility polling:

1. `physx_render_hooks.c`: the only full `physx_tick()` caller is
   `physx_tick_once_for_render_frame`. Its callers are EndScene, Present and
   SwapBuffers. Visibility is therefore sampled at a graphics frame-end hook;
   there is no guarantee that the newly revealed mesh has not already been
   evaluated/submitted. Exact Hook5 display latency was not measured.
2. `physx_physics.c`, `run_body_chain_physics_for_person`: the initialization
   branch writes rest minus pose compensation, zeroes angles/velocities, probes
   gravity, optionally publishes runtime output, then returns before integration.
   An early visibility check alone cannot supply a settled PhysX pose.
3. `run_testicle_physics_for_person` also initializes and returns before
   integration. Before initialization, `testicle_physics_candidate_is_stable`
   requires three matching root/joint observations over at least 50 ms.
   The first observation returns false. This is a skeleton-safety delay, not
   the former 250 ms visibility poll.
4. `reset_body_chain_person_state_for_reactivation` delegates to the reset
   which clears motion, timing and ownership-candidate information. It preserves
   only the validated axis reference. Clothing suspension currently uses these
   same disable/reset paths, so resume repeats startup behavior.
5. Existing traversal and post-animation ownership hooks mainly reapply already
   prepared output. They do not prepare an uninitialized revealed genital chain.
   The body-chain traversal overlay is disabled in PoseEditor. PoseEditor has a
   separate UpdateObjectsFromTracks hook; AppBase ProcessAnimation alone is not
   sufficient to establish the final pose in every mode.

The code establishes these behaviors; it does not establish how much of the
user's perceived jump comes from each one. There was no live phase capture.

## Engine entry points

The installed SYS DLL's TNode.Visibility setter at RVA 0xED910 delegates through
the linked SNode at object+0x10 to SNode.Visibility (0x01FFF042), using the getter's
already investigated class layout. Its disassembly was checked locally. This is
a candidate for observing an exact reveal request on a validated genital object,
not justification for changing visibility setters globally.

Existing plugin hooks provide candidate timing observations:

- AppBase ProcessAnimation entry/exit;
- PoseEdit UpdateObjectsFromTracks exit;
- UpdateTraverse before calling the engine;
- EndScene/Present and the existing genital pause transition.

These are candidates, not a verified before-skinning insertion point. Running a
solver from a visibility setter can encounter a partially updated outfit/scene;
running it on every traversal can repeat simulation or use incomplete matrices.
Calling the full physics tick again would also repeat unrelated plugin work.

## Original prototype proposal

Use a separate, per-person clothing-resume state rather than weakening ordinary
startup or restoring a whole stale solver state:

1. Preserve only enough identity information to recognize a previously valid
   skeleton. While paused, cheaply validate it at the existing paused-person
   checks. On reveal, re-resolve and validate the actual live root/joints. A
   changed/uncertain skeleton must still take the normal startup checks.
2. Observe a reveal request and queue preparation. Do not run physics directly
   inside a generic visibility setter. Establish the final current pose and a
   before-skinning phase for each mode with a short event trace.
3. Initialize from the current pose/gravity with fresh velocities and contact
   state. Prepare a bounded initial solution for the affected chains only,
   without advancing unrelated solvers or falsifying frame timestamps. A few
   integration steps alone do not guarantee equilibrium or collision stability.
4. Apply the prepared output and ensure the engine propagates changed transforms
   before the first visible skinning/draw. If that cannot be guaranteed within
   the reveal frame, the alternative is briefly deferring reveal until ready.
   That avoids the visible rest pose but introduces reveal latency, a different
   tradeoff from an immediate seamless reveal.
5. Preserve normal failure behavior for missing geometry, changed bodies,
   disabled options and per-person settings. Any deferred visibility mechanism
   would need to release promptly on cancellation/error and follow the latest
   clothing request, including another hide request.

Do not restore the last displayed PhysX output unconditionally: the person may
have moved, changed pose/body or changed contact conditions while covered. A
fully paused solver also cannot reconstruct the exact motion history it would
have had if it had run continuously. The attainable goal is a plausible prepared
first visible pose, not an identical counterfactual simulation history.

## Outcome

The immediate-force-pose prototype retained a visible cut in user testing.
Continuous resume now preserves the live initial pose with fresh velocity,
holds it through gravity initialization, and settles through normal simulation.
The user confirmed and approved that behavior. No visibility-setter hook,
forced hiding or further phase-trace experiment is planned for this task.

Installed DLL at investigation time, unchanged:
`97167752CDD2678F3CC77CBE21B7CBC91E97CCDD146F99AF25729A57425E09CA`.
