# Gravity response: second room test, 2026-09-10

Reviewed the 15:10:48-15:21:33 session: 48,816 log rows, 18,220,647 bytes.
The user reports no obvious camera contamination and faster penis/testicle
response. Tested DLL: `2589A63A57CC8A71E5ACB139331EBA1C38FB65D87025A8489EAEFF8440A52F4A`.
Log, configuration and DLL are preserved in
`build/collision-backup-gravity-response-roomtest-20260910/`.

## Camera protection

There are 2,760 body gravity trace rows. For penis/testicle states with valid
geometric gravity, 241 pairs of consecutive trace rows both report a hold.
237 pairs have exactly unchanged geometric direction at logged precision.
Each of the four changed pairs contains an explicit release event for that person
between the samples. None shows geometric direction changing without a logged
release between those held samples.

This supports the hold correction; it does not prove the absence of brief camera
contamination. Sampling is throttled to about one second, and no automated
camera-isolation test ran in this session. The trace cannot distinguish deliberate
body movement from every camera-related change in the engine's view-space data.

The remaining weak point is early release based on raw root displacement. There
are 64 release records (some represent different physics systems for the same
person/event); 26 have camera quiet times below 100 ms. Those times are
31/32/47/62/63/78 ms. For example, Person01 releases at 15:14:48.619 on a
0.080335-unit root displacement after 32 ms, and at 15:14:49.323 on 0.179403 units
after 47 ms. These are follow-up candidates, not proven contamination: actual
body movement is not independently tagged in this log.

## Sidecars and other observations

- All 2,874 recorded sidecar gravity samples are valid and unheld. They cover
  two earring chains and Fiesta's tail/front/side bangs. This does not show a
  primary-path gravity timer blocking those sidecars. It does not prove that
  sampled directions are free of camera error or that rendered output followed
  immediately. Sidecar springs, limits and sampling still affect visible motion.
- 26 of 284 sampled physics-health rows have update gaps above 100 ms. The
  largest is 4812 ms at 15:14:39. Some records describe multiple systems in the
  same update. Loading, modal UI or stalls are possible explanations; debug I/O
  is another possibility, not a measured cause. This is not a steady-frame-rate
  measurement. Long stalls are intentionally capped by the integrator.
- No standalone NaN/Inf values or explicit failed/overflow tokens were found
  in the log. This is a narrow log check, not exhaustive correctness validation.

## Snappier response

The live configuration uses `gravity_response_ms = 100.0` and
`gravity_max_degrees_per_second = 500.0`. The body gravity filter and the geometric
direction filter use that response time, and physical springs still integrate
toward the resulting target. These filters are distinct from the approximately
1.61-second camera hold.

A conservative next experiment would reduce accepted-sample smoothing to 50 ms
while retaining the camera guard, speed cap and spring settings. This can make
motion start/adjust more sharply after acceptance; it will not remove a hold.
It may also make any residual accepted sampling error more visible. Setting
response time to zero would remove that smoothing, not make the entire chain
settle instantly. Snapping the chain itself would discard natural momentum and
can disrupt contacts, so it is not the intended improvement.

For immediate response during pose changes that coincide with camera activity,
the higher-value engineering work is a trustworthy pose-change signal or coherent
pose/camera sampling, rather than lowering hold timers. No code, tuning, installed
DLL or release package was changed during this review.
