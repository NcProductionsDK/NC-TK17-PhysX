#ifndef PHYSX_WIND_H
#define PHYSX_WIND_H

#include <math.h>
#include <stdint.h>

/* Render-thread wind state. Hashes are the same mathematical identities used
   by the original wind field; no engine objects or ownership are cached. */
typedef struct physx_wind_parameters_t {
    float strength, turbulence, gust_strength, gust_frequency;
    float variation, wind_scale, sway_strength, sway_frequency;
} physx_wind_parameters_t;

typedef struct physx_wind_sample_t {
    int valid, result_valid;
    uint32_t chain_hash, target_hash;
    float phase, signed_unit, frequency_scale;
    float sway_phase, sway_frequency_scale;
    double seconds;
    physx_wind_parameters_t parameters;
    float result;
} physx_wind_sample_t;

typedef struct physx_wind_sway_t {
    int valid;
    uint32_t chain_hash;
    double seconds;
    float frequency, value;
} physx_wind_sway_t;

typedef struct physx_wind_state_t {
    int clock_valid;
    uint32_t last_tick;
    uint64_t elapsed_ms;
    double seconds;
    physx_wind_sample_t samples[256];
    physx_wind_sway_t sways[64];
} physx_wind_state_t;

static double physx_wind_seconds(physx_wind_state_t *state, uint32_t now)
{
    if (state->clock_valid && state->last_tick == now) return state->seconds;
    if (!state->clock_valid) {
        /* Start at the legacy phase, then advance continuously through both
           the old hourly boundary and the unsigned millisecond-clock wrap. */
        state->elapsed_ms = now % 3600000u;
        state->clock_valid = 1;
    } else {
        state->elapsed_ms += (uint32_t)(now - state->last_tick);
    }
    state->last_tick = now;
    state->seconds = (double)state->elapsed_ms * 0.001;
    return state->seconds;
}

static int physx_wind_parameters_equal(const physx_wind_parameters_t *a,
                                       const physx_wind_parameters_t *b)
{
    return a->strength == b->strength && a->turbulence == b->turbulence &&
        a->gust_strength == b->gust_strength &&
        a->gust_frequency == b->gust_frequency &&
        a->variation == b->variation && a->wind_scale == b->wind_scale &&
        a->sway_strength == b->sway_strength &&
        a->sway_frequency == b->sway_frequency;
}

static float physx_wind_sample(physx_wind_state_t *state,
                               uint32_t chain_hash, uint32_t target_hash,
                               const physx_wind_parameters_t *p, uint32_t now)
{
    const float two_pi = 6.2831853071795864769f;
    double seconds = physx_wind_seconds(state, now);
    unsigned int index = (target_hash ^ (target_hash >> 16)) & 255u;
    physx_wind_sample_t *sample = &state->samples[index];
    float gust = 0.0f, turbulence = 0.0f, sway = 0.0f, individual, result;
    if (!sample->valid || sample->chain_hash != chain_hash ||
        sample->target_hash != target_hash) {
        float unit = (float)(target_hash & 0xffffu) / 65535.0f;
        sample->valid = 1;
        sample->result_valid = 0;
        sample->chain_hash = chain_hash;
        sample->target_hash = target_hash;
        sample->phase = unit * two_pi;
        sample->signed_unit = unit * 2.0f - 1.0f;
        sample->frequency_scale = 0.85f +
            ((float)((target_hash >> 16) & 0xffu) / 255.0f) * 0.30f;
        sample->sway_phase = ((float)(chain_hash & 0xffffu) / 65535.0f) * two_pi;
        sample->sway_frequency_scale = 0.94f +
            ((float)((chain_hash >> 16) & 0xffu) / 255.0f) * 0.12f;
    }
    if (sample->result_valid && sample->seconds == seconds &&
        physx_wind_parameters_equal(&sample->parameters, p)) return sample->result;

    if (p->gust_frequency > 0.000001f && p->gust_strength > 0.000001f) {
        gust = p->gust_strength * (0.5f + 0.5f * (float)sin(
            seconds * two_pi * p->gust_frequency * sample->frequency_scale +
            sample->phase));
    }
    if (p->turbulence != 0.0f) {
        turbulence = p->turbulence *
            (0.65f * (float)sin(seconds * two_pi *
                (0.37f + p->gust_frequency * 1.70f) *
                sample->frequency_scale + sample->phase * 1.73f) +
             0.35f * (float)sin(seconds * two_pi *
                (0.83f + p->gust_frequency * 2.90f) *
                (1.15f - (sample->frequency_scale - 0.85f)) +
                sample->phase * 2.41f));
    }
    individual = 1.0f + p->variation * sample->signed_unit;
    if (p->sway_strength > 0.000001f && p->sway_frequency > 0.000001f) {
        physx_wind_sway_t *shared = &state->sways[
            (chain_hash ^ (chain_hash >> 16)) & 63u];
        if (!shared->valid || shared->chain_hash != chain_hash ||
            shared->seconds != seconds || shared->frequency != p->sway_frequency) {
            shared->valid = 1;
            shared->chain_hash = chain_hash;
            shared->seconds = seconds;
            shared->frequency = p->sway_frequency;
            shared->value = (float)sin(seconds * two_pi * p->sway_frequency *
                sample->sway_frequency_scale + sample->sway_phase);
        }
        sway = p->sway_strength * p->wind_scale * individual * shared->value;
    }
    result = p->strength * p->wind_scale * individual * (1.0f + gust + turbulence) + sway;
    if (result < -20.0f) result = -20.0f;
    if (result > 20.0f) result = 20.0f;
    sample->parameters = *p;
    sample->seconds = seconds;
    sample->result = result;
    sample->result_valid = 1;
    return result;
}

#endif
