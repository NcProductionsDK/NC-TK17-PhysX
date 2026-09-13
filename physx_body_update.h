#ifndef PHYSX_BODY_UPDATE_H
#define PHYSX_BODY_UPDATE_H
#include <stdint.h>

/* 0 preserves interval_ms scheduling; -1 publishes once per rendered frame.
   Positive rates are sampled on render frames, without catch-up solves. */
typedef struct body_update_clock_t {
    uint64_t last_us, next_us;
    int rate_hz, initialized;
} body_update_clock_t;

static int body_update_clamp_rate(int rate)
{
    return rate < -1 ? 0 : rate > 240 ? 240 : rate;
}

static int body_update_due(const body_update_clock_t *clock, int rate,
    int interval_ms, uint32_t last_tick, uint32_t now, uint64_t precise_us)
{
    if (!rate) return !last_tick || (uint32_t)(now-last_tick)>=(uint32_t)interval_ms;
    if (!clock->initialized || clock->rate_hz!=rate || precise_us<clock->last_us)
        return 1;
    /* The existing motion filter uses integral milliseconds. At extreme
       render rates retain sub-ms time until it can form a real sample. */
    if (precise_us/1000==clock->last_us/1000) return 0;
    return rate<0 || precise_us>=clock->next_us;
}

static uint32_t body_update_elapsed(body_update_clock_t *clock, int rate,
    uint32_t last_tick, uint32_t now, uint64_t precise_us)
{
    uint64_t elapsed=0;
    int rebase=!clock->initialized || clock->rate_hz!=rate || precise_us<clock->last_us;
    if (!rate) {
        /* Keep legacy wraparound, startup and elapsed values unchanged. */
        elapsed=clock->rate_hz ? 0 : last_tick ? (uint32_t)(now-last_tick) : 0;
        clock->initialized=0;
    } else {
        if (!rebase) elapsed=precise_us/1000-clock->last_us/1000;
        if (rate>0) {
            uint64_t period=1000000u/(unsigned int)rate;
            if (rebase || elapsed>100) clock->next_us=precise_us+period;
            else if (precise_us>=clock->next_us)
                clock->next_us+=((precise_us-clock->next_us)/period+1)*period;
        }
        clock->initialized=1;
    }
    clock->last_us=precise_us;
    clock->rate_hz=rate;
    /* Preserve the >100 ms stall signal without truncation after long pauses. */
    return elapsed>UINT32_MAX ? UINT32_MAX : (uint32_t)elapsed;
}
#endif
