#ifndef PHYSX_GRAVITY_SAMPLE_H
#define PHYSX_GRAVITY_SAMPLE_H
#include <stdint.h>
#include <string.h>
#include <math.h>

/* Trust direction samples, never view-space root displacement. A candidate
   waits for a later simulation frame with the same complete camera version.
   After camera movement or a discontinuity, require two agreeing directions.
   These are sampling checks, separate from the profile's physical smoothing. */
typedef struct gravity_sample_t {
    float pending[3], trusted[3];
    uintptr_t source;
    uint32_t pending_tick, pending_frame, pending_camera, last_frame, last_camera;
    uint32_t accepted_count, camera_count, waiting_count;
    int pending_valid, trusted_valid, processed, accepted, reason;
} gravity_sample_t;

enum { GRAVITY_SAMPLE_ACCEPTED, GRAVITY_SAMPLE_CAMERA,
       GRAVITY_SAMPLE_INVALID, GRAVITY_SAMPLE_CONFIRMING,
       GRAVITY_SAMPLE_DISCONTINUITY };

static int gravity_sample_update(gravity_sample_t *s, uintptr_t source,
    const float candidate[3], int valid, uint32_t frame, uint32_t now,
    uint32_t camera, int camera_valid, uint32_t camera_age, uint32_t quiet_ms,
    float out[3])
{
    int i, agrees = 1;
    float delta2 = 0.0f, norm2 = 0.0f;
    if (s->source != source) {
        memset(s, 0, sizeof(*s));
        s->source = source;
    }
    if (quiet_ms < 48u) quiet_ms = 48u;
    if (valid && candidate) for (i = 0; i < 3; i++) {
        if (!isfinite(candidate[i]) || fabsf(candidate[i]) > 8.0f) valid = 0;
        norm2 += candidate[i]*candidate[i];
    }
    else valid = 0;
    /* Callers provide normalized directions, optionally signed/scaled.
       Zero is allowed for authored channels whose configured gain is zero. */
    if (!isfinite(norm2)) valid = 0;
    if (s->processed && s->last_frame == frame && s->last_camera == camera &&
        camera_valid && valid && camera_age >= quiet_ms) {
        if (s->trusted_valid) memcpy(out, s->trusted, sizeof(s->trusted));
        return s->trusted_valid;
    }
    s->processed = 1; s->last_frame = frame; s->last_camera = camera; s->accepted = 0;
    if (!valid || !camera_valid) {
        s->pending_valid = 0; s->reason = GRAVITY_SAMPLE_INVALID;
    } else if (camera_age < quiet_ms) {
        s->pending_valid = 0; s->reason = GRAVITY_SAMPLE_CAMERA;
    } else {
        if (s->pending_valid) for (i = 0; i < 3; i++) {
            float d = candidate[i] - s->pending[i];
            delta2 += d*d;
        }
        /* About 4.6 degrees for unit directions. Reject one-frame spikes,
           but keep a persistent new pose eligible on its next sample. */
        agrees = delta2 <= 0.0064f;
        if (s->pending_valid && s->pending_camera == camera &&
            frame != s->pending_frame && now - s->pending_tick > 0u &&
            now - s->pending_tick <= 100u && agrees) {
            memcpy(s->trusted, s->pending, sizeof(s->trusted));
            s->trusted_valid = s->accepted = 1;
            s->reason = GRAVITY_SAMPLE_ACCEPTED;
        } else s->reason = agrees ? GRAVITY_SAMPLE_CONFIRMING : GRAVITY_SAMPLE_DISCONTINUITY;
        memcpy(s->pending, candidate, sizeof(s->pending));
        s->pending_tick = now; s->pending_frame = frame; s->pending_camera = camera;
        s->pending_valid = 1;
    }
    if (s->accepted) s->accepted_count++;
    else if (s->reason == GRAVITY_SAMPLE_CAMERA) s->camera_count++;
    else s->waiting_count++;
    if (s->trusted_valid) memcpy(out, s->trusted, sizeof(s->trusted));
    return s->trusted_valid;
}
#endif
