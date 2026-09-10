# Breast and butt collision review — 2026-09-10

This is the pre-fix review. The subsequent candidate implementation and tests
are described in [SINGLE-BONE-CONTACT.md](SINGLE-BONE-CONTACT.md).

Scope: independent breast/butt bones and their body/room contact response.
The user reports penetration or unnatural pushing. Penis/testicle chains are
accepted and explicitly excluded from changes. This review changes no runtime
code, settings, installed DLL or release archive.

## Findings

### 1. Body-contact correction uses the wrong coordinate convention

`paired_bone_physics_compute_collision_offsets` in `physx_colliders.c` computes
offsets in the collider's h/v/s coordinate system. The current/default basis
offsets are 0x088, 0x098 and 0x078 respectively. The caller then passes this
vector to `breasts_physics_body_translation_to_parent_local` or its butt
equivalent. Those helpers expect authored body coordinates in matrix-row order
0x078, 0x088, 0x098. There is no conversion between these conventions.

Confirmed using extracted production matrix and conversion functions:

```
python analyze_single_bone_contacts.py
```

For an identity body frame, a requested X displacement of 0.020 scene units
becomes a Z displacement of 0.020. Y becomes X and Z becomes Y. All 12 cases
(both systems, three axes, identity/rotated parent) reproduced the mismatch,
with vector error 0.028284. This is a synthetic reproduction of the current
conversion, not a recorded in-game incident.

The movement-input helpers expect their existing authored coordinate convention;
changing those shared helpers would risk breaking accepted movement. Fix the
single-bone collision call sites by converting from the collider basis through
view/world space to the bone parent's local axes. Room contacts already use a
different world-to-parent conversion and do not pass through this mismatch.

Both callers also copy the unconverted body offset straight into local axes if
conversion fails. That fallback should retain/revalidate contact instead of
applying a vector in a different frame.

### 2. Contact is a spring target, not a separation constraint

The breast and butt update functions add the residual collision offset to their
translation spring target. They integrate toward that target, then clamp the
translation range. They do not enforce the contact boundary after integration
or remove inward velocity along the support normal.

In a simplified aligned static contact, let initial overlap be p and actual bone
displacement be x. With unit response strength, the residual correction is p-x.
The current spring has equilibrium x=p-x, hence x=p/2. A 0.020-unit overlap can
settle with 0.010 remaining. An independent scalar simulation of that update
settles at the predicted value. Actual engine behavior additionally depends on
the sampled pose, strength, translation limits and coordinate mapping; this is
an explanation of the response structure, not a measurement of game penetration.

There is also a 120 ms critical-damping hold applied on all three translation
axes after a contact. It can suppress bouncing, but does not enforce separation
and also damps sliding/release motion. A replacement should keep an unconstrained
spring target, solve feasible contact constraints for the translated sphere,
and remove inward relative velocity while preserving tangential/outward motion.
Any deliberately soft contact should have explicit compliance rather than depend
on feedback from residual penetration.

### 3. Multiple contacts are added without a combined solve

Body contact adds each non-hand sphere's penetration vector, then caps total
length at 0.060. Hands have a useful grouping rule that limits each hand to its
deepest penetration, but other overlapping proxies do not. Similar contacts can
multiply the push; opposing ones can cancel despite both remaining penetrated.
Body and room corrections are then added independently, so neither side checks
whether the combined result still satisfies its contacts.

Preserve contact normals and depths, deduplicate equivalent supports and solve
body/room constraints together in the single bone's translation coordinates.
Retain attachment exclusions, collision scope and authored displacement limits.
When limits make complete separation impossible, report the residual overlap;
do not silently increase the permitted deformation or move a chain bone.

### 4. Room sweep history assumes a correction was fully applied

`body_room_collision_resolve_tracked_sphere` stores `center + correction` as the
next sweep origin. The breast/butt caller applies that correction through a
spring and displacement clamp, so the stored position need not have been reached.
Future sweeps can therefore start from a predicted separated point instead of
the actual sampled/output point. This needs an explicit history policy matched
to the final single-bone output, including stale samples and pose teleports.

### 5. Timing and output conditions need coverage

Both single-bone update functions cap elapsed time at 25 ms and discard the
remainder. At eligible updates of 30/20 Hz, they advance only 0.75/0.50 seconds
per second. Use bounded substeps specific to these paths, with contact refreshed
against predicted translated geometry; the accepted chain timing is out of scope.

Collision offsets are calculated even when `bone_translation_enabled` is false,
but the output functions only publish them when bone translation is enabled.
That coupling should be made explicit or separated so a collision switch is
not silently ineffective. Current game settings enable translation for both.

## Log and configuration limits

The inspected game configuration has breast and butt physics/room collisions
enabled, but `breasts_collision_enabled` and `butt_collision_enabled` are false
under body colliders. Debug is false. The available normal-play log has gravity
traces but no detailed single-bone body/room contact or translation-write records.
It cannot identify the user's exact contact incident or prove which switches
were active earlier in the playthrough. No settings were changed for this review.

## Recommended follow-up

A focused replacement of the single-bone contact response is justified. First
correct coordinate conversion and cover identity, rotated, mirrored and scaled
parents. Then introduce a small translation-only contact solve with coherent
velocity and sweep history, and bounded integration. Test body-only and room-only
supports, combined corners, duplicate contacts, moving hands, release, limits,
pose changes and camera movement. Protect the accepted chain implementation and
release package throughout. This remains a proxy-sphere bone simulation, not a
soft-tissue deformation model.
