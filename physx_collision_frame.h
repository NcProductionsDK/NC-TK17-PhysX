#ifndef PHYSX_COLLISION_FRAME_H
#define PHYSX_COLLISION_FRAME_H
#include <stdint.h>
#include <string.h>
#include <math.h>

/* A recorded aggressive camera test still had a displaced room-placement
   estimate at 141 ms. Agreement alone can confirm two equally stale samples.
   This applies only to collision placement; gravity and live local bone motion
   keep their existing timing. Still require confirmation after this interval. */
#define COLLISION_FRAME_CAMERA_QUIET_MS 160u

/* Only the person's placement in the room is sampled here. Live bone motion
   relative to TRS_group is never frozen. Camera and skeleton matrices can
   belong to different render updates; accept a new camera epoch only after
   it is quiet and two placement samples agree. */
typedef struct collision_frame_sample_t {
    float trusted[12], pending[12]; /* three rows, then origin */
    uintptr_t source;
    uint32_t generation, camera, pending_camera, pending_frame, pending_tick;
    int trusted_valid, pending_valid, held;
} collision_frame_sample_t;

static int collision_frame_sample_update(collision_frame_sample_t *s,
    uintptr_t source, uint32_t generation, const float candidate[12], int valid,
    uint32_t frame, uint32_t now, uint32_t camera, int camera_valid,
    uint32_t camera_age)
{
    int i, agrees = 1, tracking;
    if (s->source != source || s->generation != generation) {
        memset(s, 0, sizeof(*s));
        s->source = source; s->generation = generation;
    }
    for (i = 0; valid && i < 12; i++)
        if (!isfinite(candidate[i]) || fabsf(candidate[i]) > (i < 9 ? 8.f : 4096.f)) valid = 0;
    tracking = s->trusted_valid && s->camera == camera && !s->held;
    s->held = 1;
    if (!valid || !camera_valid || camera_age < COLLISION_FRAME_CAMERA_QUIET_MS) {
        s->pending_valid = 0;
    } else {
        for (i = 0; i < 12; i++)
            if (fabsf(candidate[i] - s->pending[i]) > (i < 9 ? .04f : .02f)) agrees = 0;
        if (tracking ||
            (s->pending_valid && s->pending_camera == camera &&
             s->pending_frame != frame && now - s->pending_tick > 0u &&
             now - s->pending_tick <= 100u && agrees)) {
            memcpy(s->trusted, candidate, sizeof(s->trusted));
            s->trusted_valid = 1; s->camera = camera; s->held = 0;
        }
        memcpy(s->pending, candidate, sizeof(s->pending));
        s->pending_valid = 1; s->pending_camera = camera;
        s->pending_frame = frame; s->pending_tick = now;
    }
    return s->trusted_valid;
}
#endif
