# Current collision checkpoint

The body-chain follow-up is now confirmed in-game: the reported thigh
penetration and testicle resting problem were resolved. See
[BODY-CONTACT-IMPLEMENTATION.md](docs/BODY-CONTACT-IMPLEMENTATION.md) for the
confirmed binary, cleanup and validation. The release package now includes
these body fixes. The earlier release ZIP is backed up with the new checkpoint.
The cumulative package also includes the accepted body movement timing,
progressive stops and [gravity/link-inertia changes](docs/BODY-DYNAMICS.md).
It also includes the accepted [sidecar motion follow-up](docs/SIDECAR-MOTION.md),
which normalizes inheritance between different link lengths and preserves small
solved velocities through free motion and contact release.

The following records the earlier sidecar/free-motion checkpoint.

The confirmed build includes the collision improvements, startup binding
recovery, and the free-motion integrator fix. The user confirmed that the
free-motion nudging was fixed and sidecar collision behavior improved too.
Unstable free motion was contributing to the remaining contact disturbance.

Confirmed DLL SHA256:
`A206FE70BD67CABF6BD67F8C08C606750CFD56A6DFB7D5DA1004488633DE42B9`.

The test log also showed Fiesta's root starting with the bounded matrix check. See
[STARTUP-BINDING.md](docs/STARTUP-BINDING.md) and
[FREE-MOTION.md](docs/FREE-MOTION.md) for changes, tests and preserved builds.
The release ZIP contains the confirmed DLL above. The source, DLL, settings,
and previous release are preserved locally in
`build/collision-backup-confirmed-free-motion-20260908/`.

## Active changes

- Combined free-motion spring forces, tangent integration and bounded substeps
  remove the reproduced dependence of resting pose on frame duration.
- Camera guards retain input protection without changing sidecar stiffness or
  freezing and releasing child poses.
- Bounded startup layout detection and guarded retries recover rejected joints.
- Accurate small-vector lengths and continuous shallow-contact response.
- Contact planes, bounded friction/velocity response, fixed link lengths, and
  same-frame publication of upstream corrections.
- Candidate chain poses account for influencing joint rotations before body
  queries. Contacts at different lever arms remain distinct.
- Body collision covers the configured extension beyond the final joint.
- Improved room correction mapping without artificial sideways escape on the
  output-aware path.
- Engine visibility overrides false camera-based body-despawn detection.

The solver changes apply to compatible sidecars generally. Fiesta's local
addition of `spine03, spine04` to its custom targets is an addon configuration
change, not a special case in the plugin or part of the release package.

## Code retained intentionally

The older per-joint output-aware solver remains in use for room contacts and
as a fallback when a complete current-frame chain snapshot is unavailable.
Other mappings support startup and targets without evaluated bases. Removing
these active compatibility paths would change behavior.

`physx_chain_contact.c` is included by `physx_sidecar.c`, rather than compiled as
a separate translation unit. `physx_contact_math.h` is shared by collision
consumers. Both must be included when committing or distributing source.

The rejected `physx_chain_contact_error` acceptance rule is absent. Diagnostics
remain gated by debug options; release defaults disable verbose logging.

## Verification

```powershell
python run_collision_tests.py
.\compile-physx.bat
```

The tests compile extracted production functions and standalone contact math.
Coverage includes candidate poses against independent forward kinematics,
distinct lever arms, bounded conflicting contacts, joint limits, separating
velocity, terminal response, body-liveness recovery, room queries, and simplified
spring settling/release at 30/60/144 Hz. Tests do not reproduce TK17's complete
animation pipeline or every addon layout.

`python analyze_collision_solver.py` runs the suite and characterizes known
limitations in retained legacy helpers. Those legacy reproductions are
diagnostic experiments, not release criteria for the coordinated path.

## Remaining limitations

The reported free-motion nudging is fixed in the tested cases; other addons
and poses still require coverage. Separate segments, people, and room surfaces are
processed sequentially, so later corrections can affect earlier contacts.
Friction uses joint-space approximations. Explicit moving-body surface velocity
is not modeled. Terminal body collision has no dedicated continuous sweep.
At most eight supports are retained per segment; complex chains require broader
performance and gameplay validation.

Dedicated body/body physics was not overhauled. Collision scope, radii, and
terminal lengths still depend on each sidecar's configuration.

## Release and history

See [CHANGELOG.md](CHANGELOG.md) for fixes and [RELEASE.md](RELEASE.md) for
installation. `python package-release.py` packages the current built DLL,
reference defaults, documentation, and checksums. It does not compile or publish
anything and does not overwrite user configurations.

Detailed trials, the rejected refinement, rollback, and checkpoint locations are
preserved in [docs/COLLISION-HISTORY.md](docs/COLLISION-HISTORY.md).
[COLLISION-ANALYSIS.md](COLLISION-ANALYSIS.md) records the earlier investigation;
its description of the original baseline is historical.
