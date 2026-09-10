# Body-chain resting-contact review

This document records the pre-change audit. The subsequent implementation and
its remaining limits are in [BODY-CONTACT-IMPLEMENTATION.md](BODY-CONTACT-IMPLEMENTATION.md).

The reported case is a body facing upward, with the penis repeatedly nudging
or bouncing against the pelvis, thighs or simulated testicle chain. The user
confirmed that repeated movement against the support is the main symptom.

Recommendation: rework the body-chain contact response and coordination, using
the existing collider geometry and engine integration as a starting point.
This review does not change physics code, configuration or the installed DLL.
The confirmed sidecar checkpoint remains intact.

## Confirmed implementation limitations

1. **Contact merging loses useful supports.** `body_chain_store_contact` merges
   contacts on the same segment when their normals have dot product >= 0.70,
   without comparing their position along the segment. Only two directions
   per segment are retained. Distinct lever arms can require distinct angular
   constraints even when their normals match.

2. **Support preservation freezes joints too broadly.** The hierarchical
   contact solver sets `first_joint` after an earlier supported segment.
   This prevents later contacts from using upstream joints, including motions
   that would preserve or safely lift off the earlier support. At a crease
   between body surfaces, the remaining joints may have no useful response.

3. **The queried pose can lag the simulated pose.** The penis spring advances
   `state->angle` before collision queries, but the fresh-engine-point branch
   of `body_chain_collision_points_local` returns cached pivots unchanged.
   It does not apply the difference between sampled and candidate joint angles.
   Contact corrections are then applied to the updated state. How much lag
   this introduces depends on engine traversal timing; the current log cannot
   quantify it for the reported pose.

4. **The two dynamic chains are not solved as one contact system.** The main
   update calls penis physics before testicle physics. Each collision query
   obtains the other chain's geometry and changes only its own chain's angles.
   The reverse query uses different geometry paths: simulated penis points,
   but live/held testicle points where available. There is no joint solve of
   both chains' responses or contact-point relative velocity in this path.
   Sequential updates alone are not necessarily a bug; the absent common
   predicted state and coordinated contact response are the concern here.

5. **Contact response has many state-dependent filters.** `physx_physics.c`
   attenuates, suppresses, smooths and caps angular corrections after the
   geometric solver. Rest/impact/grace state changes those rules. The penis
   spring also uses remembered angular correction direction to suppress spring
   force; the testicle path relies more heavily on post-step damping. Several
   damping factors and grace durations are per tick. These mechanisms can
   produce delayed correction or repeated spring/contact disturbance and can
   hinder sliding. Their individual contribution needs gameplay capture.

6. **Body free motion still needs independent verification.** Both body-chain
   spring paths retain variable-timestep integration and camera-related
   stiffness/damping changes. The specific sidecar inheritance ordering defect
   is not present in those angular spring loops. Do not assume the entire
   sidecar fix should be copied into the body solver unchanged.

The shared contact Jacobian also routes through helpers using the penis output
axis mapping. However, `physx_config.c` currently copies that mapping into the
testicle configuration. This is a coupling to clean up, not an established
cause of the present resting problem.

## Production-code reproductions

Run `python analyze_body_contacts.py`. It extracts and compiles the relevant
production functions into an isolated diagnostic executable. On this revision:

- Contacts at segment fractions 0.25 and 0.90 with equal normals retain just
  one support.
- A bent chain's distal upward correction is rejected when upstream joints
  are protected. Allowing the upstream joint produces 0.0018749 units of
  separation and also lifts the proximal joint safely above its support plane.
- With a fresh cached straight chain, changing the candidate angle by 20 degrees
  leaves queried engine points unchanged. The separate candidate calculation
  moves the tip by 0.1026061 units in the test geometry.

These deliberately isolated fixtures establish behavior of the code. Their
distances are synthetic engine units, not measured penetration in the game.
The script characterizes the old behavior; it is not a regression test whose
observed defects should be preserved after a rewrite.

The available log ending at 22:30:37 has no `body-chain-physics write`,
`testicle-physics write`, or `response-simple` records. Its collider health
records show no ready bodies. It does not capture the reported resting case.

## Proposed implementation sequence

1. Establish consistent sampled and predicted chain poses, including rendered
   axis mapping and angle limits. Query the pose that will actually be written.
2. Keep distinct contact locations and solve supports together instead of
   freezing all upstream joints. Preserve nonpenetration at previous supports
   while allowing useful motion and separation.
3. Separate positional overlap correction from velocity response. Remove inward
   contact-point relative velocity while allowing tangential sliding; then
   replace redundant angular correction filters as the new solve takes over.
4. Coordinate penis/testicle contacts using a shared predicted state and both
   chains' response. Check rest, slow support movement and separation.
5. Verify gravity-only rest and camera-only movement independently. Make body
   integration timing and guard transitions consistent where tests show a defect.

Validation should cover pelvis/thigh creases, dynamic chain-on-chain support,
different body orientations, small movements, joint limits, room contact and
30/60/144 Hz plus uneven frames. Capture the upward-facing gameplay case before
and after to check motion, penetration, support switching and camera inputs.
