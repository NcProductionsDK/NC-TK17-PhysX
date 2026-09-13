# Penis/testicle gravity on pose changes

The initial filter-history change did not resolve the user's visible pause.
The follow-up capture and code inspection identified a separate dependency:
geometric gravity returned early whenever the collider readiness flag was false.
This flag includes a whole-body placement settling delay. Dynamics initialization
also waited for it, despite only needing the local chain's geometry.

For example, at 12:37:34.879 the log reports horizontal strength 1.5 already
applied, but `geometry=0`, while collider settling is active. At 12:37:35.326
the collider becomes ready. The subsequent geometric calculation produces a
different gravity target. Strengths were not being overwritten; the gravity
model itself was switching during collision warmup.

Gravity now uses confirmed body orientation independently of collider readiness.
During temporary basis loss it retains its last trusted geometric direction,
as it already does during camera holds. Local dynamics can initialize before
collision promotion, using current engine pivots only, with valid basis and
no body/camera hold. Synthetic or held samples cannot initialize the model
during warmup. Collision response retains its existing readiness gate.

The sampler now records a confirmed direction jump greater than 30 degrees,
or the first confirmed sample after initialization/rebinding. Penis and testicle
consumers use that event to seed their mapped-drive and geometric-direction
filters from the confirmed direction. The event stays pending through a body
camera hold and is consumed once, including when several solver substeps run
in one frame. Smaller continuous changes retain the configured smoothing.

Sampling still requires agreeing observations across frames and a quiet camera.
This change removes the extra filter transition after confirmation; it does not
remove camera validation time or teleport the chain to its final resting angle.
The spring solver still handles motion and collision normally. Config files,
gravity strengths/curves, neutral references, and other body systems are unchanged.

Validation:

- `python run_gravity_response_tests.py`: logged old/new direction at
  30/60/90/144 Hz, confirmation, holds, rebinding, repeated substeps, ordinary
  animated rotation, camera validation and unchanged sidecar sample routing.
- `python run_body_dynamics_tests.py`: production geometric filter releases a
  confirmed held change directly, retains small-motion smoothing and chain
  angles, and preserves gravity-strength behavior through geometry activation.
  Repeated collider readiness toggles now produce identical gravity targets to
  the ready path for both chains, including zero/unit/1.5 horizontal strengths.
- `python run_body_contact_tests.py`: actual capture/begin functions initialize
  both chain models from current pivots before collision promotion, retain the
  model at promotion, and reject invalid, stale, held and synthetic warmup input.

In-game check: restart with the new DLL and load the same Person01/Person02 pose.
Check whether the old-direction pause is gone or reduced, and also rotate the
camera and switch back to a different pose. Visual validation is pending.
