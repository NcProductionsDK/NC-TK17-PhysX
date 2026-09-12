/* Original wind sampler for comparison; intentionally retains its old clock. */
static float reference_wind_strength(const char *owner_name,
                                       const char *system_name,
                                       const char *target_name,
                                       float wind_scale,
                                       float sway_strength,
                                       float sway_frequency,
                                       DWORD now)
{
    const float two_pi = 6.2831853071795864769f;
    unsigned int hash = 2166136261u;
    float unit;
    float signed_unit;
    float phase;
    float seconds;
    float frequency_scale;
    float sway_phase;
    float sway_frequency_scale;
    float gust = 0.0f;
    float turbulence;
    float individual;
    float amplitude;
    float sway = 0.0f;
    if (!room_wind_is_enabled() || wind_scale <= 0.000001f) {
        return 0.0f;
    }
    hash = room_wind_hash_add(hash, owner_name);
    hash = room_wind_hash_add(hash, system_name);
    /* Every target in one chain shares a slow phase so the trunk bends as a
       unit. Target hashing below still varies gusts and turbulence. */
    sway_phase = ((float)(hash & 0xffffu) / 65535.0f) * two_pi;
    sway_frequency_scale = 0.94f +
        ((float)((hash >> 16) & 0xffu) / 255.0f) * 0.12f;
    hash = room_wind_hash_add(hash, target_name);
    unit = (float)(hash & 0xffffu) / 65535.0f;
    signed_unit = unit * 2.0f - 1.0f;
    phase = unit * two_pi;
    seconds = (float)(now % 3600000u) * 0.001f;
    frequency_scale = 0.85f +
        ((float)((hash >> 16) & 0xffu) / 255.0f) * 0.30f;
    if (room_wind_cfg.gust_frequency > 0.000001f &&
        room_wind_cfg.gust_strength > 0.000001f) {
        gust = room_wind_cfg.gust_strength *
            (0.5f + 0.5f * (float)sin((double)(
                seconds * two_pi * room_wind_cfg.gust_frequency *
                frequency_scale + phase)));
    }
    turbulence = room_wind_cfg.turbulence *
        (0.65f * (float)sin((double)(
            seconds * two_pi *
            (0.37f + room_wind_cfg.gust_frequency * 1.70f) *
            frequency_scale + phase * 1.73f)) +
         0.35f * (float)sin((double)(
            seconds * two_pi *
            (0.83f + room_wind_cfg.gust_frequency * 2.90f) *
            (1.15f - (frequency_scale - 0.85f)) + phase * 2.41f)));
    individual = 1.0f + room_wind_cfg.variation * signed_unit;
    amplitude = room_wind_cfg.strength * wind_scale * individual *
                (1.0f + gust + turbulence);
    if (sway_strength > 0.000001f && sway_frequency > 0.000001f) {
        /* Unlike gusts, this contribution is signed. It can move a flexible
           room chain through rest for genuine back-and-forth motion. */
        sway = sway_strength * wind_scale * individual *
            (float)sin((double)(seconds * two_pi * sway_frequency *
                                sway_frequency_scale + sway_phase));
    }
    return physx_clampf(amplitude + sway, -20.0f, 20.0f);
}
