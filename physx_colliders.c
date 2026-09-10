static int body_collider_draw_view_point(const float stored[3], float out[3])
{
    if (!stored || !out) return 0;
    out[0] = stored[0];
    out[1] = stored[1];
    out[2] = stored[2];
    return 1;
}

static float body_collider_distance(const float a[3], const float b[3])
{
    float d[3];
    d[0] = a[0] - b[0];
    d[1] = a[1] - b[1];
    d[2] = a[2] - b[2];
    return physx_vec3_len(d);
}

static DWORD body_chain_penis_pivot_quarantine_until[4];
static float body_chain_penis_reconnect_candidate[4][4][3];
static DWORD body_chain_penis_reconnect_candidate_tick[4];
static unsigned int body_chain_penis_reconnect_candidate_count[4];
static int body_chain_penis_reconnect_candidate_ready[4];

static void body_chain_penis_clear_reconnect_candidate(int person_index)
{
    if (person_index < 0 || person_index >= 4) return;
    body_chain_penis_reconnect_candidate_ready[person_index] = 0;
    body_chain_penis_reconnect_candidate_tick[person_index] = 0;
    body_chain_penis_reconnect_candidate_count[person_index] = 0;
}

static int body_chain_penis_reconnect_candidate_stable(
    int person_index,
    const float local[4][3],
    DWORD now,
    float *candidate_delta_out,
    unsigned int *candidate_count_out,
    DWORD *candidate_age_out)
{
    float max_delta = 0.0f;
    int i;

    if (candidate_delta_out) *candidate_delta_out = 0.0f;
    if (candidate_count_out) *candidate_count_out = 0;
    if (candidate_age_out) *candidate_age_out = 0;
    if (person_index < 0 || person_index >= 4 || !local) return 0;

    if (!body_chain_penis_reconnect_candidate_ready[person_index]) {
        memcpy(body_chain_penis_reconnect_candidate[person_index], local,
               sizeof(body_chain_penis_reconnect_candidate[person_index]));
        body_chain_penis_reconnect_candidate_ready[person_index] = 1;
        body_chain_penis_reconnect_candidate_tick[person_index] = now;
        body_chain_penis_reconnect_candidate_count[person_index] = 1;
        if (candidate_count_out) *candidate_count_out = 1;
        return 0;
    }

    for (i = 0; i < 4; i++) {
        float delta = body_collider_distance(
            local[i], body_chain_penis_reconnect_candidate[person_index][i]);
        if (sane_probe_float(delta) && delta > max_delta) {
            max_delta = delta;
        }
    }

    if (max_delta > 0.018f) {
        memcpy(body_chain_penis_reconnect_candidate[person_index], local,
               sizeof(body_chain_penis_reconnect_candidate[person_index]));
        body_chain_penis_reconnect_candidate_ready[person_index] = 1;
        body_chain_penis_reconnect_candidate_tick[person_index] = now;
        body_chain_penis_reconnect_candidate_count[person_index] = 1;
        if (candidate_delta_out) *candidate_delta_out = max_delta;
        if (candidate_count_out) *candidate_count_out = 1;
        return 0;
    }

    body_chain_penis_reconnect_candidate_count[person_index]++;
    if (candidate_delta_out) *candidate_delta_out = max_delta;
    if (candidate_count_out) {
        *candidate_count_out =
            body_chain_penis_reconnect_candidate_count[person_index];
    }
    if (candidate_age_out) {
        *candidate_age_out =
            now - body_chain_penis_reconnect_candidate_tick[person_index];
    }
    return (now - body_chain_penis_reconnect_candidate_tick[person_index] >=
                180u ||
            body_chain_penis_reconnect_candidate_count[person_index] >= 4);
}

static void body_chain_collider_capture_settle_sample(
    body_chain_collider_person_state_t *state)
{
    int i;
    if (!state) return;
    for (i = 0; i < BODY_COLLIDER_NODE_COUNT; i++) {
        state->settle_prev_local[i][0] = state->local_position[i][0];
        state->settle_prev_local[i][1] = state->local_position[i][1];
        state->settle_prev_local[i][2] = state->local_position[i][2];
    }
    for (i = 0; i < 4; i++) {
        state->settle_prev_chain[i][0] = state->chain_local_point[i][0];
        state->settle_prev_chain[i][1] = state->chain_local_point[i][1];
        state->settle_prev_chain[i][2] = state->chain_local_point[i][2];
    }
    state->settle_prev_chain_ready = state->chain_points_ready;
}

static int body_chain_collider_ready_node_required(
    const body_chain_collider_person_state_t *state,
    int node_index)
{
    int scope_mask = state ? state->active_scope_mask :
        BODY_CHAIN_COLLIDER_GROUP_ALL;
    if (!body_chain_collider_cfg.breasts_collision_enabled &&
        !body_chain_collider_cfg.penis_collision_enabled &&
        !body_chain_collider_cfg.testicle_collision_enabled &&
        node_index >= BODY_COLLIDER_TESTICLES_01 &&
        node_index <= BODY_COLLIDER_TESTICLES_MID) {
        return 0;
    }
    if (scope_mask & BODY_CHAIN_COLLIDER_GROUP_FULL_BODY) return 1;
    if (node_index <= BODY_COLLIDER_TESTICLES_MID &&
        (scope_mask & (BODY_CHAIN_COLLIDER_GROUP_GENITALS |
                       BODY_CHAIN_COLLIDER_GROUP_PELVIS))) {
        return 1;
    }
    if ((scope_mask & BODY_CHAIN_COLLIDER_GROUP_HANDS) &&
        body_chain_collider_node_is_hand(node_index)) {
        return 1;
    }
    if ((scope_mask & BODY_CHAIN_COLLIDER_GROUP_BREAST_SOURCE) &&
        (node_index == BODY_COLLIDER_BREAST_L ||
         node_index == BODY_COLLIDER_BREAST_R)) {
        return 1;
    }
    if ((scope_mask & BODY_CHAIN_COLLIDER_GROUP_BUTT_SOURCE) &&
        (node_index == BODY_COLLIDER_BUTT_L ||
         node_index == BODY_COLLIDER_BUTT_R)) {
        return 1;
    }
    return 0;
}

static int body_chain_collider_sample_stable(
    const body_chain_collider_person_state_t *state,
    int require_chain_points,
    float *max_delta_out)
{
    float max_delta = 0.0f;
    int i;

    if (max_delta_out) *max_delta_out = 0.0f;
    if (!state) return 0;
    if (((state->active_scope_mask &
          (BODY_CHAIN_COLLIDER_GROUP_PELVIS |
           BODY_CHAIN_COLLIDER_GROUP_FULL_BODY)) != 0 &&
         !state->limb_points_ready) ||
        (require_chain_points && !state->chain_points_ready)) {
        return 0;
    }
    for (i = 0; i < BODY_COLLIDER_NODE_COUNT; i++) {
        float delta;
        if (!state->valid[i]) {
            if (body_chain_collider_ready_node_required(state, i)) return 0;
            continue;
        }
        delta = body_collider_distance(state->local_position[i],
                                       state->settle_prev_local[i]);
        if (!sane_probe_float(delta)) return 0;
        if (delta > max_delta) max_delta = delta;
    }
    if (max_delta_out) *max_delta_out = max_delta;
    return max_delta <= BODY_CHAIN_COLLIDER_STABLE_DELTA;
}

static DWORD body_chain_collider_root_quiet_required_ms(void)
{
    int ms = physics_environment_cfg.gravity_probe_settle_ms;
    if (ms < BODY_CHAIN_COLLIDER_ROOT_QUIET_MS) {
        ms = BODY_CHAIN_COLLIDER_ROOT_QUIET_MS;
    }
    if (ms > 30000) ms = 30000;
    return (DWORD)ms;
}

static int body_chain_collider_runtime_mode_active(void)
{
    return InterlockedCompareExchange(&body_chain_poseeditor_mode_active,
                                      0, 0) == 0;
}

static int body_chain_collider_root_quiet(
    body_chain_collider_person_state_t *state,
    const float *root_view,
    DWORD now,
    int force_reset,
    float *delta_out,
    unsigned long *quiet_ms_out)
{
    float delta = 0.0f;
    float epsilon = physics_environment_cfg.gravity_probe_motion_epsilon;
    DWORD required_ms = body_chain_collider_root_quiet_required_ms();

    if (delta_out) *delta_out = 0.0f;
    if (quiet_ms_out) *quiet_ms_out = 0ul;
    if (!state || !root_view) return 0;

    if (!state->settle_prev_root_ready || force_reset) {
        state->settle_prev_root_view[0] = root_view[0];
        state->settle_prev_root_view[1] = root_view[1];
        state->settle_prev_root_view[2] = root_view[2];
        state->settle_prev_root_ready = 1;
        state->settle_root_quiet_tick = 0;
        return 0;
    }

    delta = body_collider_distance(root_view, state->settle_prev_root_view);
    state->settle_prev_root_view[0] = root_view[0];
    state->settle_prev_root_view[1] = root_view[1];
    state->settle_prev_root_view[2] = root_view[2];
    if (delta_out) *delta_out = delta;

    if (!sane_probe_float(delta) || delta > epsilon) {
        state->settle_root_quiet_tick = 0;
        return 0;
    }

    if (!state->settle_root_quiet_tick) {
        state->settle_root_quiet_tick = now;
    }
    if (quiet_ms_out) {
        *quiet_ms_out = (unsigned long)(now - state->settle_root_quiet_tick);
    }
    return now - state->settle_root_quiet_tick >= required_ms;
}

static int body_chain_collider_update_settle_gate(
    body_chain_collider_person_state_t *state,
    const char *person,
    DWORD now,
    int sample_ready,
    int force_reset,
    int require_chain_points,
    const float *root_view)
{
    float max_delta = 0.0f;
    float root_delta = 0.0f;
    unsigned long root_quiet_ms = 0ul;
    int stable;
    int root_quiet;
    int runtime_mode;

    if (!state) return 0;
    runtime_mode = body_chain_collider_runtime_mode_active();
    if (sample_ready &&
        (((state->active_scope_mask &
           (BODY_CHAIN_COLLIDER_GROUP_PELVIS |
            BODY_CHAIN_COLLIDER_GROUP_FULL_BODY)) != 0 &&
          !state->limb_points_ready) ||
         (require_chain_points && !state->chain_points_ready))) {
        sample_ready = 0;
    }
    if (!sample_ready) {
        state->sample_ready = 0;
        state->ready = 0;
        state->settle_start_tick = 0;
        state->settle_sample_count = 0;
        state->settle_logged = 0;
        state->settle_prev_root_ready = 0;
        state->settle_root_quiet_tick = 0;
        return 0;
    }

    if (state->ready && !force_reset) {
        state->sample_ready = 1;
        body_chain_collider_capture_settle_sample(state);
        return 1;
    }

    if (force_reset || !state->sample_ready || !state->settle_start_tick) {
        state->sample_ready = 1;
        state->ready = 0;
        state->settle_start_tick = now;
        state->settle_sample_count = 1;
        state->settle_logged = 0;
        body_chain_collider_capture_settle_sample(state);
        body_chain_collider_root_quiet(state, root_view, now, 1,
                                       NULL, NULL);
        if ((body_chain_collider_cfg.diagnostic || defaults_cfg.debug) &&
            !state->settle_logged &&
            (!state->last_settle_log_tick ||
             now - state->last_settle_log_tick >=
                (DWORD)body_chain_collider_cfg.health_log_ms)) {
            state->settle_logged = 1;
            state->last_settle_log_tick = now;
            log_line("body-chain-colliders settling person=\"%s\" reason=\"%s\" settle_ms=%d root_quiet_ms=%lu stable_samples_required=%d note=\"collider draw/response is gated until room-load samples and root motion stop jumping\"",
                     person ? person : "",
                     force_reset ? "root-or-source-changed" : "sample-regained",
                     BODY_CHAIN_COLLIDER_SETTLE_MS,
                     (unsigned long)body_chain_collider_root_quiet_required_ms(),
                     BODY_CHAIN_COLLIDER_STABLE_SAMPLES);
        }
        return 0;
    }

    stable = body_chain_collider_sample_stable(state, require_chain_points,
                                               &max_delta);
    if (!stable) {
        state->ready = 0;
        state->settle_start_tick = now;
        state->settle_sample_count = 1;
        state->settle_logged = 0;
        body_chain_collider_capture_settle_sample(state);
        body_chain_collider_root_quiet(state, root_view, now, 1,
                                       NULL, NULL);
        if ((body_chain_collider_cfg.diagnostic || defaults_cfg.debug) &&
            (!state->last_settle_log_tick ||
             now - state->last_settle_log_tick >=
                (DWORD)body_chain_collider_cfg.health_log_ms)) {
            state->last_settle_log_tick = now;
            log_line("body-chain-colliders settling-reset person=\"%s\" max_delta=%.5f threshold=%.5f live_points_ready=%d note=\"transient collider sample jump rejected before it can flicker draw/response\"",
                     person ? person : "",
                     max_delta,
                     BODY_CHAIN_COLLIDER_STABLE_DELTA,
                     state->chain_points_ready && state->limb_points_ready);
        }
        return 0;
    }

    if (state->settle_sample_count < 1000000) {
        state->settle_sample_count++;
    }
    if (runtime_mode &&
        state->settle_sample_count >= BODY_CHAIN_COLLIDER_STABLE_SAMPLES) {
        /* FreeMode bodies are normally animated continuously, so waiting for
           their root to become quiet can suppress collision indefinitely.
           Three coherent body-local samples are enough to reject a transient
           load/source jump while restoring collision promptly. Missing or
           discontinuous live nodes still fail before reaching this branch. */
        body_chain_collider_capture_settle_sample(state);
        state->ready = 1;
        if ((body_chain_collider_cfg.diagnostic || defaults_cfg.debug) &&
            (!state->last_settle_log_tick ||
             now - state->last_settle_log_tick >=
                (DWORD)body_chain_collider_cfg.health_log_ms)) {
            state->last_settle_log_tick = now;
            log_line("body-chain-colliders runtime-ready person=\"%s\" stable_samples=%d note=\"FreeMode collision resumed from coherent body-local samples without waiting for the animated root to become quiet\"",
                     person ? person : "",
                     state->settle_sample_count);
        }
        return 1;
    }
    root_quiet = body_chain_collider_root_quiet(
        state, root_view, now, 0, &root_delta, &root_quiet_ms);
    body_chain_collider_capture_settle_sample(state);
    if (!root_quiet &&
        now - state->settle_start_tick >=
            (DWORD)(BODY_CHAIN_COLLIDER_SETTLE_MS + 500) &&
        state->settle_sample_count >= BODY_CHAIN_COLLIDER_STABLE_SAMPLES) {
        root_quiet = 1;
        if ((body_chain_collider_cfg.diagnostic || defaults_cfg.debug) &&
            (!state->last_settle_log_tick ||
             now - state->last_settle_log_tick >=
                (DWORD)body_chain_collider_cfg.health_log_ms)) {
            state->last_settle_log_tick = now;
            log_line("body-chain-colliders settling-timeout person=\"%s\" settle_ms=%lu root_delta=%.6f quiet_ms=%lu note=\"stable collider samples promoted despite ongoing root/camera motion so collision cannot remain disabled during live poses\"",
                     person ? person : "",
                     (unsigned long)(now - state->settle_start_tick),
                     root_delta,
                     root_quiet_ms);
        }
    }
    if (now - state->settle_start_tick >= BODY_CHAIN_COLLIDER_SETTLE_MS &&
        state->settle_sample_count >= BODY_CHAIN_COLLIDER_STABLE_SAMPLES &&
        root_quiet) {
        state->ready = 1;
        return 1;
    }
    if ((body_chain_collider_cfg.diagnostic || defaults_cfg.debug) &&
        !root_quiet &&
        (!state->last_settle_log_tick ||
         now - state->last_settle_log_tick >=
            (DWORD)body_chain_collider_cfg.health_log_ms)) {
        state->last_settle_log_tick = now;
        log_line("body-chain-colliders settling-root person=\"%s\" root_delta=%.6f quiet_ms=%lu required_ms=%lu epsilon=%.6f note=\"root/body still settling; holding collider draw/response to prevent load-time flicker\"",
                 person ? person : "",
                 root_delta,
                 root_quiet_ms,
                 (unsigned long)body_chain_collider_root_quiet_required_ms(),
                 physics_environment_cfg.gravity_probe_motion_epsilon);
    }
    state->ready = 0;
    return 0;
}

static int body_chain_collider_scene_liveness(
    body_chain_collider_person_state_t *state,
    const char *person,
    void *root_raw,
    const float root_view[3],
    int root_changed,
    int scene_person_visible,
    DWORD now)
{
    float delta;
    const float stale_epsilon = 0.00005f;
    const float live_epsilon = 0.00050f;
    if (!state || !root_view) {
        return 1;
    }

    /* An explicit engine visibility result outranks the camera heuristic.
       Orbiting around a live body can leave its root almost stationary.
       Hidden-person, scene-track and root validity checks run in the caller. */
    if (scene_person_visible > 0) {
        if (state->scene_liveness_quarantined) {
            log_line("body-chain-colliders respawn person=\"%s\" reason=\"engine confirms visible body; releasing camera heuristic quarantine\"",
                     person ? person : "");
        }
        state->scene_liveness_quarantined = 0;
        state->scene_liveness_quarantined_root_raw = NULL;
        state->scene_liveness_static_camera_samples = 0;
        state->scene_liveness_camera_version = captured_camera_version;
        memcpy(state->scene_liveness_root_view, root_view,
               sizeof(state->scene_liveness_root_view));
        return 1;
    }

    if (state->scene_liveness_quarantined) {
        if (root_raw != state->scene_liveness_quarantined_root_raw) {
            log_line("body-chain-colliders respawn person=\"%s\" old_root_raw=%p new_root_raw=%p reason=\"TK17 supplied a different body root object\" note=\"releasing stale-collider quarantine and settling the new body\"",
                     person ? person : "",
                     state->scene_liveness_quarantined_root_raw,
                     root_raw);
            state->scene_liveness_quarantined = 0;
            state->scene_liveness_quarantined_root_raw = NULL;
            state->scene_liveness_static_camera_samples = 0;
            state->scene_liveness_camera_version = captured_camera_version;
            memcpy(state->scene_liveness_root_view, root_view,
                   sizeof(state->scene_liveness_root_view));
            return 1;
        }
        if (!captured_camera_inverse_valid || captured_camera_version <= 0 ||
            state->scene_liveness_camera_version == captured_camera_version) {
            return 0;
        }
        delta = body_collider_distance(state->scene_liveness_root_view,
                                       root_view);
        state->scene_liveness_camera_version = captured_camera_version;
        memcpy(state->scene_liveness_root_view, root_view,
               sizeof(state->scene_liveness_root_view));
        if (sane_probe_float(delta) && delta > live_epsilon) {
            log_line("body-chain-colliders respawn person=\"%s\" root_raw=%p camera_version=%ld root_delta=%.8f reason=\"body root responded to camera-space transform again\" note=\"releasing stale-collider quarantine and settling the live body\"",
                     person ? person : "",
                     root_raw,
                     captured_camera_version,
                     delta);
            state->scene_liveness_quarantined = 0;
            state->scene_liveness_quarantined_root_raw = NULL;
            state->scene_liveness_static_camera_samples = 0;
            return 1;
        }
        return 0;
    }

    if (!captured_camera_inverse_valid || captured_camera_version <= 0 ||
        root_changed) {
        if (state && root_view) {
            state->scene_liveness_camera_version = captured_camera_version;
            state->scene_liveness_root_view[0] = root_view[0];
            state->scene_liveness_root_view[1] = root_view[1];
            state->scene_liveness_root_view[2] = root_view[2];
            state->scene_liveness_static_camera_samples = 0;
        }
        return 1;
    }

    if (!state->scene_liveness_camera_version) {
        state->scene_liveness_camera_version = captured_camera_version;
        state->scene_liveness_root_view[0] = root_view[0];
        state->scene_liveness_root_view[1] = root_view[1];
        state->scene_liveness_root_view[2] = root_view[2];
        state->scene_liveness_static_camera_samples = 0;
        return 1;
    }

    if (state->scene_liveness_camera_version == captured_camera_version) {
        return 1;
    }

    delta = body_collider_distance(state->scene_liveness_root_view,
                                   root_view);
    state->scene_liveness_camera_version = captured_camera_version;
    state->scene_liveness_root_view[0] = root_view[0];
    state->scene_liveness_root_view[1] = root_view[1];
    state->scene_liveness_root_view[2] = root_view[2];

    if (state->ready && sane_probe_float(delta) && delta <= stale_epsilon) {
        if (state->scene_liveness_static_camera_samples < 1000000) {
            state->scene_liveness_static_camera_samples++;
        }
    } else {
        state->scene_liveness_static_camera_samples = 0;
    }

    if (state->scene_liveness_static_camera_samples >= 3) {
        state->scene_liveness_quarantined = 1;
        state->scene_liveness_quarantined_root_raw = root_raw;
        if (!state->last_scene_liveness_log_tick ||
            now - state->last_scene_liveness_log_tick >=
                (DWORD)body_chain_collider_cfg.health_log_ms) {
            state->last_scene_liveness_log_tick = now;
            log_line("body-chain-colliders despawn person=\"%s\" reason=\"view-space body root stayed frozen through camera changes\" camera_version=%ld static_camera_samples=%d root_delta=%.8f note=\"TK17 still resolves old named nodes after scene removal; clearing stale collision for this person\"",
                     person ? person : "",
                     captured_camera_version,
                     state->scene_liveness_static_camera_samples,
                     delta);
        }
        return 0;
    }

    return 1;
}

static int ptr_executable(const void *p)
{
    MEMORY_BASIC_INFORMATION mbi;
    DWORD protect;
    if (!p || !VirtualQuery(p, &mbi, sizeof(mbi)) ||
        mbi.State != MEM_COMMIT ||
        (mbi.Protect & (PAGE_NOACCESS | PAGE_GUARD))) {
        return 0;
    }
    protect = mbi.Protect & 0xff;
    return protect == PAGE_EXECUTE ||
           protect == PAGE_EXECUTE_READ ||
           protect == PAGE_EXECUTE_READWRITE ||
           protect == PAGE_EXECUTE_WRITECOPY;
}

static float body_collider_det3(const float a[3], const float b[3], const float c[3])
{
    return a[0] * (b[1] * c[2] - b[2] * c[1]) -
           b[0] * (a[1] * c[2] - a[2] * c[1]) +
           c[0] * (a[1] * b[2] - a[2] * b[1]);
}

static int body_collider_view_delta_to_local(const float delta[3],
                                             const float basis_h[3],
                                             const float basis_v[3],
                                             const float basis_s[3],
                                             float local[3])
{
    float det;
    if (!delta || !basis_h || !basis_v || !basis_s || !local) return 0;
    det = body_collider_det3(basis_h, basis_v, basis_s);
    if (physx_absf(det) < 0.000001f) {
        local[0] = vec3_dot(delta, basis_h);
        local[1] = vec3_dot(delta, basis_v);
        local[2] = vec3_dot(delta, basis_s);
        return 0;
    }
    local[0] = body_collider_det3(delta, basis_v, basis_s) / det;
    local[1] = body_collider_det3(basis_h, delta, basis_s) / det;
    local[2] = body_collider_det3(basis_h, basis_v, delta) / det;
    return sane_probe_float(local[0]) &&
           sane_probe_float(local[1]) &&
           sane_probe_float(local[2]);
}

static const char *body_collider_variant_node(int node_index, int variant)
{
    switch (node_index) {
    case BODY_COLLIDER_ROOT:
        return variant == 0 ? "root" : variant == 1 ? "Sroot" : NULL;
    case BODY_COLLIDER_STOMACH_01:
        return variant == 0 ? "Sspine_joint01" : variant == 1 ? "spine_joint01" : NULL;
    case BODY_COLLIDER_STOMACH_02:
        return variant == 0 ? "Sspine_joint02" : variant == 1 ? "spine_joint02" : NULL;
    case BODY_COLLIDER_HIP_L:
        return variant == 0 ? "Ship_L_joint" : variant == 1 ? "hip_L_joint" : NULL;
    case BODY_COLLIDER_HIP_R:
        return variant == 0 ? "Ship_R_joint" : variant == 1 ? "hip_R_joint" : NULL;
    case BODY_COLLIDER_KNEE_L:
        return variant == 0 ? "Sknee_L_joint" : variant == 1 ? "knee_L_joint" : NULL;
    case BODY_COLLIDER_KNEE_R:
        return variant == 0 ? "Sknee_R_joint" : variant == 1 ? "knee_R_joint" : NULL;
    case BODY_COLLIDER_THIGH_L:
    case BODY_COLLIDER_THIGH_R:
        return NULL;
    case BODY_COLLIDER_TESTICLES_01:
        return variant == 0 ? "Stesticles_joint01" : variant == 1 ? "testicles_joint01" : NULL;
    case BODY_COLLIDER_TESTICLES_02:
        return variant == 0 ? "Stesticles_joint02" : variant == 1 ? "testicles_joint02" : NULL;
    case BODY_COLLIDER_TESTICLES_MID:
        return NULL;
    default:
        return NULL;
    }
}

static float body_collider_target_distance(int node_index)
{
    switch (node_index) {
    case BODY_COLLIDER_STOMACH_01:
    case BODY_COLLIDER_STOMACH_02:
        return 0.55f;
    case BODY_COLLIDER_HIP_L:
    case BODY_COLLIDER_HIP_R:
        return 0.65f;
    case BODY_COLLIDER_KNEE_L:
    case BODY_COLLIDER_KNEE_R:
        return 0.95f;
    case BODY_COLLIDER_THIGH_L:
    case BODY_COLLIDER_THIGH_R:
        return 0.80f;
    case BODY_COLLIDER_TESTICLES_01:
    case BODY_COLLIDER_TESTICLES_02:
    case BODY_COLLIDER_TESTICLES_MID:
        return 0.35f;
    default:
        return 0.0f;
    }
}

static int read_body_collider_position(void *raw, int offset, float out[3])
{
    float *v;
    float len;
    if (!raw || offset < 0 || !out ||
        !ptr_readable((BYTE*)raw + offset, sizeof(float) * 3)) {
        return 0;
    }
    v = (float*)((BYTE*)raw + offset);
    if (!sane_probe_float(v[0]) ||
        !sane_probe_float(v[1]) ||
        !sane_probe_float(v[2])) {
        return 0;
    }
    len = physx_vec3_len(v);
    if (len < 0.000001f || len > 10000.0f) return 0;
    out[0] = v[0];
    out[1] = v[1];
    out[2] = v[2];
    return 1;
}

static int resolve_body_collider_source(body_chain_collider_person_state_t *state,
                                        const char *person,
                                        int node_index)
{
    static const int candidate_offsets[] = {
        0x06c, 0x070, 0x074, 0x078, 0x088, 0x098, 0x0a8,
        0x0b0, 0x0d8, 0x0e8, 0x208, 0x338, 0x348, 0x358,
        0x388, 0x3a8
    };
    float best_score = 1000000.0f;
    float best_pos[3] = {0.0f, 0.0f, 0.0f};
    void *best_raw = NULL;
    int best_offset = -1;
    char best_name[128] = "";
    int variant;

    if (!state || !person || node_index < 0 ||
        node_index >= BODY_COLLIDER_NODE_COUNT) {
        return 0;
    }

    for (variant = 0; variant < 4; variant++) {
        const char *node = body_collider_variant_node(node_index, variant);
        char runtime_name[256];
        void *raw;
        int offset_index;
        if (!node) continue;
        make_body_runtime_name(runtime_name, sizeof(runtime_name), person, node);
        raw = resolve_axis_map_raw(runtime_name);
        if (!raw) continue;

        for (offset_index = 0;
             offset_index < (int)(sizeof(candidate_offsets) / sizeof(candidate_offsets[0]));
             offset_index++) {
            int offset = candidate_offsets[offset_index];
            float pos[3];
            float score = 0.0f;
            int j;
            if (body_chain_collider_cfg.position_offset >= 0 &&
                offset != body_chain_collider_cfg.position_offset) {
                continue;
            }
            if (!read_body_collider_position(raw, offset, pos)) continue;
            if (node_index != BODY_COLLIDER_ROOT &&
                state->valid[BODY_COLLIDER_ROOT]) {
                float root_dist = body_collider_distance(pos,
                    state->view_position[BODY_COLLIDER_ROOT]);
                float target_dist = body_collider_target_distance(node_index);
                if (root_dist < 0.010f) score += 1000.0f;
                if (root_dist > 3.000f) score += 100.0f + root_dist;
                score += physx_absf(root_dist - target_dist);
            } else if (node_index == BODY_COLLIDER_ROOT) {
                score += (offset == 0x0e8) ? 0.0f : 0.25f;
                score += (variant == 0) ? 0.0f : 0.10f;
            }
            for (j = 0; j < node_index; j++) {
                if (state->valid[j]) {
                    float dist = body_collider_distance(pos,
                                                        state->view_position[j]);
                    if (dist < 0.010f) score += 250.0f;
                }
            }
            if (variant == 0) score -= 0.05f;
            if (offset == 0x0e8 && node_index != BODY_COLLIDER_ROOT) {
                score += 0.50f;
            }
            if (score < best_score) {
                best_score = score;
                best_raw = raw;
                best_offset = offset;
                best_pos[0] = pos[0];
                best_pos[1] = pos[1];
                best_pos[2] = pos[2];
                lstrcpynA(best_name, node, sizeof(best_name));
            }
        }
    }

    if (!best_raw || best_offset < 0 || best_score > 999.0f) return 0;
    state->raw[node_index] = best_raw;
    state->position_offset[node_index] = best_offset;
    lstrcpynA(state->source_name[node_index], best_name,
              sizeof(state->source_name[node_index]));
    state->view_position[node_index][0] = best_pos[0];
    state->view_position[node_index][1] = best_pos[1];
    state->view_position[node_index][2] = best_pos[2];
    log_line("body-chain-colliders source person=\"%s\" node_index=%d source=\"%s\" raw=%p offset=0x%03x score=%.5f view=(%.5f,%.5f,%.5f) configured_offset=%s",
             person,
             node_index,
             state->source_name[node_index],
             state->raw[node_index],
             state->position_offset[node_index],
             best_score,
             state->view_position[node_index][0],
             state->view_position[node_index][1],
             state->view_position[node_index][2],
             body_chain_collider_cfg.position_offset < 0 ? "auto" : "fixed");
    return 1;
}

static int update_live_testicle_collider(
    body_chain_collider_person_state_t *state,
    const char *person,
    int node_index,
    const float root_pos[3],
    const float basis_h[3],
    const float basis_v[3],
    const float basis_s[3])
{
    static const int candidate_offsets[] = {
        0x06c, 0x070, 0x074, 0x078, 0x088, 0x098, 0x0a8,
        0x0b0, 0x0d8, 0x0e8, 0x208, 0x338, 0x348, 0x358,
        0x388, 0x3a8
    };
    const float *expected;
    float best_score = 1000000.0f;
    float best_view[3] = { 0.0f, 0.0f, 0.0f };
    float best_local[3] = { 0.0f, 0.0f, 0.0f };
    void *best_raw = NULL;
    int best_offset = -1;
    char best_name[128] = "";
    int variant;

    if (!state || !person || !root_pos || !basis_h || !basis_v || !basis_s ||
        (node_index != BODY_COLLIDER_TESTICLES_01 &&
         node_index != BODY_COLLIDER_TESTICLES_02)) {
        return 0;
    }
    expected = body_chain_collider_cfg.local_offset[node_index];

    if (state->raw[node_index] && state->position_offset[node_index] >= 0) {
        float view[3];
        float delta[3];
        float local[3];
        float local_len;
        if (read_body_collider_position(state->raw[node_index],
                                        state->position_offset[node_index],
                                        view)) {
            delta[0] = view[0] - root_pos[0];
            delta[1] = view[1] - root_pos[1];
            delta[2] = view[2] - root_pos[2];
            local[0] = vec3_dot(delta, basis_h);
            local[1] = vec3_dot(delta, basis_v);
            local[2] = vec3_dot(delta, basis_s);
            local_len = physx_vec3_len(local);
            if (local_len >= 0.010f && local_len <= 0.500f &&
                body_collider_distance(local, expected) < 0.400f) {
                state->view_position[node_index][0] = view[0];
                state->view_position[node_index][1] = view[1];
                state->view_position[node_index][2] = view[2];
                state->local_position[node_index][0] = local[0];
                state->local_position[node_index][1] = local[1];
                state->local_position[node_index][2] = local[2];
                if (!camera_view_to_world_point(
                        view, state->world_position[node_index])) {
                    state->world_position[node_index][0] = view[0];
                    state->world_position[node_index][1] = view[1];
                    state->world_position[node_index][2] = view[2];
                }
                state->valid[node_index] = 1;
                return 1;
            }
        }
    }

    for (variant = 0; variant < 2; variant++) {
        const char *node = body_collider_variant_node(node_index, variant);
        char runtime_name[256];
        void *raw;
        int offset_index;
        if (!node) continue;
        make_body_runtime_name(runtime_name, sizeof(runtime_name), person, node);
        raw = resolve_axis_map_raw(runtime_name);
        if (!raw) continue;

        for (offset_index = 0;
             offset_index < (int)(sizeof(candidate_offsets) /
                                  sizeof(candidate_offsets[0]));
             offset_index++) {
            int offset = candidate_offsets[offset_index];
            float view[3];
            float delta[3];
            float local[3];
            float local_len;
            float score;
            if (!read_body_collider_position(raw, offset, view)) continue;
            delta[0] = view[0] - root_pos[0];
            delta[1] = view[1] - root_pos[1];
            delta[2] = view[2] - root_pos[2];
            local[0] = vec3_dot(delta, basis_h);
            local[1] = vec3_dot(delta, basis_v);
            local[2] = vec3_dot(delta, basis_s);
            local_len = physx_vec3_len(local);
            if (local_len < 0.010f || local_len > 0.500f) continue;
            score = body_collider_distance(local, expected);
            if (variant == 0) score -= 0.005f;
            if (offset == 0x0e8) score -= 0.002f;
            if (score < best_score) {
                best_score = score;
                best_raw = raw;
                best_offset = offset;
                best_view[0] = view[0];
                best_view[1] = view[1];
                best_view[2] = view[2];
                best_local[0] = local[0];
                best_local[1] = local[1];
                best_local[2] = local[2];
                lstrcpynA(best_name, node, sizeof(best_name));
            }
        }
    }

    if (!best_raw || best_offset < 0) return 0;
    if (state->raw[node_index] != best_raw ||
        state->position_offset[node_index] != best_offset) {
        log_line("body-chain-colliders live-testicle person=\"%s\" node=\"%s\" raw=%p offset=0x%03x score=%.5f local=(%.5f,%.5f,%.5f) configured_fallback=(%.5f,%.5f,%.5f) note=\"live bone center converted from view space into camera-independent root-local space\"",
                 person,
                 best_name,
                 best_raw,
                 best_offset,
                 best_score,
                 best_local[0], best_local[1], best_local[2],
                 expected[0], expected[1], expected[2]);
    }
    state->raw[node_index] = best_raw;
    state->position_offset[node_index] = best_offset;
    lstrcpynA(state->source_name[node_index], best_name,
              sizeof(state->source_name[node_index]));
    state->view_position[node_index][0] = best_view[0];
    state->view_position[node_index][1] = best_view[1];
    state->view_position[node_index][2] = best_view[2];
    state->local_position[node_index][0] = best_local[0];
    state->local_position[node_index][1] = best_local[1];
    state->local_position[node_index][2] = best_local[2];
    if (!camera_view_to_world_point(best_view,
                                    state->world_position[node_index])) {
        state->world_position[node_index][0] = best_view[0];
        state->world_position[node_index][1] = best_view[1];
        state->world_position[node_index][2] = best_view[2];
    }
    state->valid[node_index] = 1;
    return 1;
}

static int read_body_collider_rotation(void *raw, int offset, float out[3])
{
    float *v;
    if (!raw || offset < 0 || !out ||
        !ptr_readable((BYTE*)raw + offset, sizeof(float) * 3)) {
        return 0;
    }
    v = (float*)((BYTE*)raw + offset);
    if (!sane_probe_float(v[0]) || !sane_probe_float(v[1]) ||
        !sane_probe_float(v[2])) {
        return 0;
    }
    out[0] = v[0];
    out[1] = v[1];
    out[2] = v[2];
    return 1;
}

static float body_collider_wrap_degrees(float value)
{
    while (value > 180.0f) value -= 360.0f;
    while (value < -180.0f) value += 360.0f;
    return value;
}

/* TK17 exposes the animated joint channels as three Euler values, while the
   body-chain solver uses root-local X forward, Y vertical, and Z horizontal.
   Map the configured body-chain output channels onto those local planes so
   collider motion stays independent from the renderer/camera matrices. */
static void body_collider_rotate_local_vector(const float in[3],
                                              const float rotation_delta[3],
                                              float out[3])
{
    const float deg_to_rad = 0.01745329251994329577f;
    int h_axis = body_chain_physics_cfg.horizontal_output_axis;
    int v_axis = body_chain_physics_cfg.vertical_output_axis;
    int roll_axis = 0;
    float h;
    float v;
    float roll;
    float x = in[0];
    float y = in[1];
    float z = in[2];
    float c;
    float s;
    float next;

    while (roll_axis == h_axis || roll_axis == v_axis) roll_axis++;
    if (roll_axis > 2) roll_axis = 0;
    h = body_collider_wrap_degrees(rotation_delta[h_axis]) * deg_to_rad;
    v = body_collider_wrap_degrees(rotation_delta[v_axis]) * deg_to_rad;
    roll = body_collider_wrap_degrees(rotation_delta[roll_axis]) * deg_to_rad;

    c = (float)cos((double)roll);
    s = (float)sin((double)roll);
    next = y * c - z * s;
    z = y * s + z * c;
    y = next;

    c = (float)cos((double)h);
    s = (float)sin((double)h);
    next = x * c + z * s;
    z = -x * s + z * c;
    x = next;

    c = (float)cos((double)-v);
    s = (float)sin((double)-v);
    next = x * c - y * s;
    y = x * s + y * c;
    x = next;

    out[0] = x;
    out[1] = y;
    out[2] = z;
}

static void body_collider_rotate_point_about_pivot(const float point[3],
                                                   const float pivot[3],
                                                   const float rotation_delta[3],
                                                   float out[3])
{
    float relative[3];
    float rotated[3];
    relative[0] = point[0] - pivot[0];
    relative[1] = point[1] - pivot[1];
    relative[2] = point[2] - pivot[2];
    body_collider_rotate_local_vector(relative, rotation_delta, rotated);
    out[0] = pivot[0] + rotated[0];
    out[1] = pivot[1] + rotated[1];
    out[2] = pivot[2] + rotated[2];
}

static int body_collider_engine_pivot_local(const char *person,
                                            const char *node,
                                            const char *fallback_node,
                                            const float root_pos[3],
                                            const float basis_h[3],
                                            const float basis_v[3],
                                            const float basis_s[3],
                                            float local[3],
                                            float view[3])
{
    float pivot_view[3];
    float delta[3];
    if (!person || !node || !root_pos || !basis_h || !basis_v ||
        !basis_s || !local) {
        return 0;
    }
    if (!body_collider_engine_pivot_view(person, node, fallback_node,
                                         pivot_view)) {
        return 0;
    }
    delta[0] = pivot_view[0] - root_pos[0];
    delta[1] = pivot_view[1] - root_pos[1];
    delta[2] = pivot_view[2] - root_pos[2];
    if (!body_collider_view_delta_to_local(delta, basis_h, basis_v, basis_s,
                                           local)) {
        return 0;
    }
    if (view) {
        view[0] = pivot_view[0];
        view[1] = pivot_view[1];
        view[2] = pivot_view[2];
    }
    return 1;
}

static int body_collider_engine_pivot_local_object(
    void *object,
    const float root_pos[3],
    const float basis_h[3],
    const float basis_v[3],
    const float basis_s[3],
    float local[3],
    float view[3])
{
    float pivot_view[3];
    float delta[3];
    if (!object || !root_pos || !basis_h || !basis_v || !basis_s ||
        !local ||
        !body_collider_engine_pivot_view_object(object, pivot_view)) {
        return 0;
    }
    delta[0] = pivot_view[0] - root_pos[0];
    delta[1] = pivot_view[1] - root_pos[1];
    delta[2] = pivot_view[2] - root_pos[2];
    if (!body_collider_view_delta_to_local(delta, basis_h, basis_v, basis_s,
                                           local)) {
        return 0;
    }
    if (view) {
        view[0] = pivot_view[0];
        view[1] = pivot_view[1];
        view[2] = pivot_view[2];
    }
    return 1;
}

static int body_chain_engine_points_plausible(const float local[4][3])
{
    float seg_len[3] = {0.0f, 0.0f, 0.0f};
    float total = 0.0f;
    int i;
    if (!local) return 0;
    if (physx_vec3_len(local[0]) > 0.030f) return 0;
    for (i = 0; i < 4; i++) {
        if (!sane_probe_float(local[i][0]) ||
            !sane_probe_float(local[i][1]) ||
            !sane_probe_float(local[i][2])) {
            return 0;
        }
        if (physx_absf(local[i][0]) > 0.50f ||
            physx_absf(local[i][1]) > 0.50f ||
            physx_absf(local[i][2]) > 0.50f) {
            return 0;
        }
    }
    for (i = 1; i < 4; i++) {
        float dx = local[i][0] - local[i - 1][0];
        float dy = local[i][1] - local[i - 1][1];
        float dz = local[i][2] - local[i - 1][2];
        float len = (float)sqrt((double)(dx * dx + dy * dy + dz * dz));
        if (len < 0.003f || len > 0.180f) return 0;
        seg_len[i - 1] = len;
        total += len;
    }
    if (seg_len[0] < 0.035f && seg_len[1] > 0.120f) return 0;
    if (seg_len[0] > 0.0001f &&
        seg_len[1] > 0.100f &&
        seg_len[1] / seg_len[0] > 3.25f) {
        return 0;
    }
    if (seg_len[2] > 0.0001f &&
        seg_len[1] > 0.120f &&
        seg_len[1] / seg_len[2] > 3.50f) {
        return 0;
    }
    return total >= 0.060f && total <= 0.360f;
}

static int body_chain_engine_points_continuous(
    const body_chain_collider_person_state_t *state,
    const float local[4][3],
    float *max_delta_out,
    float *worst_dot_out)
{
    float max_delta = 0.0f;
    float worst_dot = 1.0f;
    int i;

    if (max_delta_out) *max_delta_out = 0.0f;
    if (worst_dot_out) *worst_dot_out = 1.0f;
    if (!state || !local || !state->chain_points_ready ||
        !state->chain_points_update_tick) {
        return 1;
    }
    for (i = 0; i < 4; i++) {
        float delta;
        if (!state->chain_point_valid[i]) return 1;
        delta = body_collider_distance(local[i], state->chain_local_point[i]);
        if (sane_probe_float(delta) && delta > max_delta) {
            max_delta = delta;
        }
    }
    for (i = 1; i < 4; i++) {
        float old_v[3];
        float new_v[3];
        float old_len;
        float new_len;
        float dot;
        old_v[0] = state->chain_local_point[i][0] -
                   state->chain_local_point[i - 1][0];
        old_v[1] = state->chain_local_point[i][1] -
                   state->chain_local_point[i - 1][1];
        old_v[2] = state->chain_local_point[i][2] -
                   state->chain_local_point[i - 1][2];
        new_v[0] = local[i][0] - local[i - 1][0];
        new_v[1] = local[i][1] - local[i - 1][1];
        new_v[2] = local[i][2] - local[i - 1][2];
        old_len = physx_vec3_len(old_v);
        new_len = physx_vec3_len(new_v);
        if (old_len <= 0.003f || new_len <= 0.003f ||
            !sane_probe_float(old_len) || !sane_probe_float(new_len)) {
            continue;
        }
        dot = (old_v[0] * new_v[0] +
               old_v[1] * new_v[1] +
               old_v[2] * new_v[2]) / (old_len * new_len);
        dot = physx_clampf(dot, -1.0f, 1.0f);
        if (dot < worst_dot) worst_dot = dot;
    }
    if (max_delta_out) *max_delta_out = max_delta;
    if (worst_dot_out) *worst_dot_out = worst_dot;

    return !((max_delta > 0.055f && worst_dot < 0.10f) ||
             (max_delta > 0.095f && worst_dot < 0.45f) ||
             max_delta > 0.140f);
}

static int body_chain_camera_pivot_hold_active(DWORD now)
{
    (void)now;
    /*
       Do not freeze all model-view pivot samples during camera movement.
       The live collider chain must keep following animated/posed TK17 bones.
       Camera-related protection is handled by the continuity gates below,
       which reject only discontinuous samples while allowing coherent motion.
    */
    return 0;
}

static int body_chain_limb_points_continuous(
    const body_chain_collider_person_state_t *state,
    const float local[6][3],
    float *max_delta_out)
{
    float max_delta = 0.0f;
    int i;

    if (max_delta_out) *max_delta_out = 0.0f;
    if (!state || !local || !state->limb_points_ready ||
        !state->limb_points_update_tick) {
        return 1;
    }
    for (i = 0; i < 6; i++) {
        float delta = body_collider_distance(
            local[i], state->limb_local_position[i]);
        if (sane_probe_float(delta) && delta > max_delta) {
            max_delta = delta;
        }
    }
    if (max_delta_out) *max_delta_out = max_delta;
    return max_delta <= 0.120f;
}

static int body_chain_testicle_colliders_plausible(const float local[2][3])
{
    float dx;
    float dy;
    float dz;
    float dist;
    int i;
    if (!local) return 0;
    for (i = 0; i < 2; i++) {
        if (!sane_probe_float(local[i][0]) ||
            !sane_probe_float(local[i][1]) ||
            !sane_probe_float(local[i][2])) {
            return 0;
        }
        if (physx_absf(local[i][0]) > 0.75f ||
            physx_absf(local[i][1]) > 0.75f ||
            physx_absf(local[i][2]) > 0.75f) {
            return 0;
        }
    }
    dx = local[1][0] - local[0][0];
    dy = local[1][1] - local[0][1];
    dz = local[1][2] - local[0][2];
    dist = (float)sqrt((double)(dx * dx + dy * dy + dz * dz));
    return dist >= 0.005f && dist <= 0.350f;
}

static int body_chain_testicle_joint_pair_plausible(
    const float a[3],
    const float b[3])
{
    float dx;
    float dy;
    float dz;
    float dist;
    int i;

    if (!a || !b) return 0;
    for (i = 0; i < 3; i++) {
        if (!sane_probe_float(a[i]) || !sane_probe_float(b[i]) ||
            physx_absf(a[i]) > 0.75f || physx_absf(b[i]) > 0.75f) {
            return 0;
        }
    }
    dx = b[0] - a[0];
    dy = b[1] - a[1];
    dz = b[2] - a[2];
    dist = (float)sqrt((double)(dx * dx + dy * dy + dz * dz));
    return dist >= 0.005f && dist <= 0.350f;
}

static int update_engine_pivot_body_chain_points(
    body_chain_collider_person_state_t *state,
    int person_index,
    const char *person,
    const float root_pos[3],
    const float basis_h[3],
    const float basis_v[3],
    const float basis_s[3],
    DWORD now)
{
    static const char *nodes[4] = {
        "penis_joint01", "penis_joint02", "penis_joint03", "penis_jointEnd"
    };
    float local[4][3];
    float view[4][3];
    int i;
    int end_from_engine = 1;

    if (!state || !person || !root_pos || !basis_h || !basis_v || !basis_s) {
        return 0;
    }

    if (person_index >= 0 && person_index < 4 &&
        body_chain_penis_pivot_quarantine_until[person_index] &&
        now < body_chain_penis_pivot_quarantine_until[person_index]) {
        state->chain_points_fresh = 0;
        state->chain_points_update_tick = now;
        if ((body_chain_collider_cfg.diagnostic || defaults_cfg.debug) &&
            (!state->last_chain_point_reject_log_tick ||
             now - state->last_chain_point_reject_log_tick >= 500u)) {
            state->last_chain_point_reject_log_tick = now;
            log_line("body-chain-colliders chain-engine-pivots quarantine-held person=\"%s\" remaining_ms=%lu note=\"holding last stable penis centerline after camera-attached ghost contact; live pivots resume after the short quarantine\"",
                     person,
                     (unsigned long)
                        (body_chain_penis_pivot_quarantine_until
                            [person_index] - now));
        }
        return state->chain_points_ready ? 1 : 0;
    }

    if (body_chain_camera_pivot_hold_active(now) &&
        state->chain_points_ready) {
        state->chain_points_fresh = 0;
        state->chain_points_update_tick = now;
        if ((body_chain_collider_cfg.diagnostic || defaults_cfg.debug) &&
            (!state->last_chain_point_reject_log_tick ||
             now - state->last_chain_point_reject_log_tick >= 750u)) {
            state->last_chain_point_reject_log_tick = now;
            log_line("body-chain-colliders chain-engine-pivots camera-held person=\"%s\" camera_version=%ld note=\"camera matrix changed recently; holding last stable local penis centerline instead of accepting a model-view pivot sample that can attach to the camera\"",
                     person,
                     captured_camera_version);
        }
        return 1;
    }

    for (i = 0; i < 3; i++) {
        if (!body_collider_engine_pivot_local(person, nodes[i],
                                             NULL, root_pos,
                                             basis_h, basis_v, basis_s,
                                             local[i], view[i])) {
            state->chain_points_ready = 0;
            state->chain_points_fresh = 0;
            state->chain_points_update_tick = 0;
            return 0;
        }
    }

    if (!body_collider_engine_pivot_local(person, nodes[3],
                                         NULL, root_pos,
                                         basis_h, basis_v, basis_s,
                                         local[3], view[3])) {
        float dir[3] = {
            local[2][0] - local[1][0],
            local[2][1] - local[1][1],
            local[2][2] - local[1][2]
        };
        float len = physx_vec3_len(dir);
        float fallback_len = body_chain_collider_cfg.link_length[2];
        if (fallback_len <= 0.0001f) fallback_len = 0.040f;
        if (len > 0.0001f) {
            local[3][0] = local[2][0] + dir[0] / len * fallback_len;
            local[3][1] = local[2][1] + dir[1] / len * fallback_len;
            local[3][2] = local[2][2] + dir[2] / len * fallback_len;
        } else {
            local[3][0] = local[2][0] - fallback_len;
            local[3][1] = local[2][1];
            local[3][2] = local[2][2];
        }
        view[3][0] = view[2][0];
        view[3][1] = view[2][1];
        view[3][2] = view[2][2];
        end_from_engine = 0;
    }

    if (!body_chain_engine_points_plausible(local)) {
        int keep_previous =
            state->chain_points_ready &&
            state->chain_points_update_tick &&
            now - state->chain_points_update_tick <=
                BODY_CHAIN_ENGINE_POINT_STALE_MS;
        DWORD reject_log_interval = body_chain_collider_cfg.diagnostic ?
            (DWORD)body_chain_collider_cfg.health_log_ms : 30000u;
        state->chain_point_reject_count++;
        if ((body_chain_collider_cfg.diagnostic || defaults_cfg.debug) &&
            (!state->last_chain_point_reject_log_tick ||
             now - state->last_chain_point_reject_log_tick >=
                reject_log_interval || !keep_previous)) {
            state->last_chain_point_reject_log_tick = now;
            log_line("body-chain-colliders chain-engine-pivots rejected person=\"%s\" total_rejected=%u local01=(%.5f,%.5f,%.5f) local02=(%.5f,%.5f,%.5f) local03=(%.5f,%.5f,%.5f) localEnd=(%.5f,%.5f,%.5f) kept_previous=%d stale_ms=%lu note=\"invalid engine sample discarded; cached valid centerline is visual-only until a fresh sample arrives\"",
                     person,
                     state->chain_point_reject_count,
                     local[0][0], local[0][1], local[0][2],
                     local[1][0], local[1][1], local[1][2],
                     local[2][0], local[2][1], local[2][2],
                     local[3][0], local[3][1], local[3][2],
                     keep_previous,
                     state->chain_points_update_tick ?
                         (unsigned long)(now - state->chain_points_update_tick) :
                         0ul);
        }
        state->chain_points_fresh = 0;
        if (keep_previous) {
            state->chain_points_update_tick = now;
            return 1;
        }
        state->chain_points_ready = 0;
        for (i = 0; i < 4; i++) state->chain_point_valid[i] = 0;
        return 0;
    }

    {
        float max_delta = 0.0f;
        float worst_dot = 1.0f;
        int recent_reload =
            config_hot_reload_tick &&
            now - config_hot_reload_tick <= 6000u;
        int recent_camera =
            captured_camera_change_tick &&
            now - captured_camera_change_tick <= 2500u;
        int keep_previous =
            state->chain_points_ready &&
            state->chain_points_update_tick &&
            now - state->chain_points_update_tick <=
                BODY_CHAIN_ENGINE_POINT_STALE_MS;

        if (keep_previous &&
            !body_chain_engine_points_continuous(
                state, local, &max_delta, &worst_dot)) {
            float candidate_delta = 0.0f;
            unsigned int candidate_count = 0;
            DWORD candidate_age = 0;
            if (body_chain_penis_reconnect_candidate_stable(
                    person_index, local, now, &candidate_delta,
                    &candidate_count, &candidate_age)) {
                for (i = 0; i < 4; i++) {
                    state->chain_local_point[i][0] = local[i][0];
                    state->chain_local_point[i][1] = local[i][1];
                    state->chain_local_point[i][2] = local[i][2];
                    state->chain_point_valid[i] = 1;
                }
                state->chain_points_ready = 1;
                state->chain_points_fresh = 1;
                state->chain_points_update_tick = now;
                body_chain_penis_clear_reconnect_candidate(person_index);
                if ((body_chain_collider_cfg.diagnostic || defaults_cfg.debug) &&
                    (!state->last_chain_point_reject_log_tick ||
                     now - state->last_chain_point_reject_log_tick >= 500u)) {
                    state->last_chain_point_reject_log_tick = now;
                    log_line("body-chain-colliders chain-engine-pivots reconnected person=\"%s\" max_delta=%.5f worst_dot=%.5f candidate_delta=%.5f candidate_count=%u candidate_age_ms=%lu note=\"accepted a stable discontinuous live penis centerline so colliders reconnect to TK17 bones after ghost quarantine\"",
                             person,
                             max_delta,
                             worst_dot,
                             candidate_delta,
                             candidate_count,
                             (unsigned long)candidate_age);
                }
                return 1;
            }
            state->chain_point_reject_count++;
            state->chain_points_fresh = 0;
            state->chain_points_update_tick = now;
            if ((body_chain_collider_cfg.diagnostic || defaults_cfg.debug) &&
                (!state->last_chain_point_reject_log_tick ||
                 now - state->last_chain_point_reject_log_tick >= 750u)) {
                state->last_chain_point_reject_log_tick = now;
                log_line("body-chain-colliders chain-engine-pivots held person=\"%s\" recent_reload=%d recent_camera=%d max_delta=%.5f worst_dot=%.5f candidate_delta=%.5f candidate_count=%u candidate_age_ms=%lu total_rejected=%u note=\"rejecting plausible but discontinuous live penis pivot sample; keeping previous live centerline until a stable reconnect candidate is confirmed\"",
                         person,
                         recent_reload,
                         recent_camera,
                         max_delta,
                         worst_dot,
                         candidate_delta,
                         candidate_count,
                         (unsigned long)candidate_age,
                         state->chain_point_reject_count);
            }
            return 1;
        }
    }

    for (i = 0; i < 4; i++) {
        state->chain_local_point[i][0] = local[i][0];
        state->chain_local_point[i][1] = local[i][1];
        state->chain_local_point[i][2] = local[i][2];
        state->chain_point_valid[i] = 1;
    }
    state->chain_points_ready = 1;
    state->chain_points_fresh = 1;
    state->chain_points_update_tick = now;
    body_chain_penis_clear_reconnect_candidate(person_index);

    {
        int recent_reload =
            config_hot_reload_tick &&
            now - config_hot_reload_tick <= 6000u;
        DWORD log_interval = recent_reload ? 750u :
            (DWORD)body_chain_collider_cfg.health_log_ms;
        if ((body_chain_collider_cfg.diagnostic ||
             defaults_cfg.debug ||
             recent_reload) &&
            (!state->last_chain_point_log_tick ||
             now - state->last_chain_point_log_tick >= log_interval)) {
            state->last_chain_point_log_tick = now;
            log_line("body-chain-colliders chain-engine-pivots person=\"%s\" recent_reload=%d source=\"penis-engine-pivots\" local01=(%.5f,%.5f,%.5f) local02=(%.5f,%.5f,%.5f) local03=(%.5f,%.5f,%.5f) localEnd=(%.5f,%.5f,%.5f) end_source=\"%s\" note=\"penis collision follows live TK17 pivots; invalid samples hold the previous valid centerline briefly\"",
                     person,
                     recent_reload,
                     state->chain_local_point[0][0],
                     state->chain_local_point[0][1],
                     state->chain_local_point[0][2],
                     state->chain_local_point[1][0],
                     state->chain_local_point[1][1],
                     state->chain_local_point[1][2],
                     state->chain_local_point[2][0],
                     state->chain_local_point[2][1],
                     state->chain_local_point[2][2],
                     state->chain_local_point[3][0],
                     state->chain_local_point[3][1],
                     state->chain_local_point[3][2],
                     end_from_engine ? "engine" : "extrapolated");
        }
    }
    return 1;
}

static int update_engine_pivot_testicle_colliders(
    body_chain_collider_person_state_t *state,
    const char *person,
    const float root_pos[3],
    const float basis_h[3],
    const float basis_v[3],
    const float basis_s[3],
    DWORD now)
{
    static const char *nodes[3] = {
        "testicles_joint01", "testicles_joint02", "testicles_jointEnd"
    };
    float joint_view[3][3] = {{0}};
    float joint_local[3][3] = {{0}};
    float local[2][3] = {{0}};
    int end_found = 1;
    int complete = 1;
    int sample_valid = 0;
    int keep_previous = 0;
    int joint02_from_end_midpoint = 0;
    int i;

    if (!state || !person || !root_pos || !basis_h || !basis_v || !basis_s ||
        !body_chain_collider_cfg.live_testicle_bones) {
        return 0;
    }

    if (body_chain_camera_pivot_hold_active(now) &&
        state->testicle_points_ready) {
        state->testicle_points_update_tick = now;
        if ((body_chain_collider_cfg.diagnostic || defaults_cfg.debug) &&
            (!state->last_testicle_reject_log_tick ||
             now - state->last_testicle_reject_log_tick >= 750u)) {
            state->last_testicle_reject_log_tick = now;
            log_line("body-chain-colliders testicle-engine-bones camera-held person=\"%s\" camera_version=%ld note=\"camera matrix changed recently; holding last stable local testicle capsules instead of accepting model-view pivots that can attach to the camera\"",
                     person,
                     captured_camera_version);
        }
        for (i = 0; i < 2; i++) {
            state->local_position[BODY_COLLIDER_TESTICLES_01 + i][0] =
                state->testicle_local_position[i][0];
            state->local_position[BODY_COLLIDER_TESTICLES_01 + i][1] =
                state->testicle_local_position[i][1];
            state->local_position[BODY_COLLIDER_TESTICLES_01 + i][2] =
                state->testicle_local_position[i][2];
            state->valid[BODY_COLLIDER_TESTICLES_01 + i] = 1;
        }
        state->local_position[BODY_COLLIDER_TESTICLES_MID][0] =
            (state->testicle_local_position[0][0] +
             state->testicle_local_position[1][0]) * 0.5f;
        state->local_position[BODY_COLLIDER_TESTICLES_MID][1] =
            (state->testicle_local_position[0][1] +
             state->testicle_local_position[1][1]) * 0.5f;
        state->local_position[BODY_COLLIDER_TESTICLES_MID][2] =
            (state->testicle_local_position[0][2] +
             state->testicle_local_position[1][2]) * 0.5f;
        state->valid[BODY_COLLIDER_TESTICLES_MID] = 1;
        return 1;
    }

    for (i = 0; i < 2; i++) {
        if (!body_collider_engine_pivot_local(person, nodes[i],
                                             NULL, root_pos,
                                             basis_h, basis_v, basis_s,
                                             joint_local[i], joint_view[i])) {
            complete = 0;
            break;
        }
    }
    if (complete &&
        !body_collider_engine_pivot_local(person, nodes[2], NULL, root_pos,
                                          basis_h, basis_v, basis_s,
                                          joint_local[2], joint_view[2])) {
        end_found = 0;
        joint_local[2][0] = joint_local[1][0] +
            (joint_local[1][0] - joint_local[0][0]);
        joint_local[2][1] = joint_local[1][1] +
            (joint_local[1][1] - joint_local[0][1]);
        joint_local[2][2] = joint_local[1][2] +
            (joint_local[1][2] - joint_local[0][2]);
        joint_view[2][0] = joint_view[1][0];
        joint_view[2][1] = joint_view[1][1];
        joint_view[2][2] = joint_view[1][2];
    }

    if (complete && end_found &&
        body_chain_testicle_joint_pair_plausible(
            joint_local[0], joint_local[2]) &&
        (!body_chain_testicle_joint_pair_plausible(
             joint_local[0], joint_local[1]) ||
         !body_chain_testicle_joint_pair_plausible(
             joint_local[1], joint_local[2]))) {
        joint_local[1][0] = (joint_local[0][0] + joint_local[2][0]) * 0.5f;
        joint_local[1][1] = (joint_local[0][1] + joint_local[2][1]) * 0.5f;
        joint_local[1][2] = (joint_local[0][2] + joint_local[2][2]) * 0.5f;
        joint_view[1][0] = (joint_view[0][0] + joint_view[2][0]) * 0.5f;
        joint_view[1][1] = (joint_view[0][1] + joint_view[2][1]) * 0.5f;
        joint_view[1][2] = (joint_view[0][2] + joint_view[2][2]) * 0.5f;
        joint02_from_end_midpoint = 1;
    }

    if (complete) {
        local[0][0] = (joint_local[0][0] + joint_local[1][0]) * 0.5f +
                      body_chain_collider_cfg.testicle_fine_offset[0][0];
        local[0][1] = (joint_local[0][1] + joint_local[1][1]) * 0.5f +
                      body_chain_collider_cfg.testicle_fine_offset[0][1];
        local[0][2] = (joint_local[0][2] + joint_local[1][2]) * 0.5f +
                      body_chain_collider_cfg.testicle_fine_offset[0][2];
        local[1][0] = (joint_local[1][0] + joint_local[2][0]) * 0.5f +
                      body_chain_collider_cfg.testicle_fine_offset[1][0];
        local[1][1] = (joint_local[1][1] + joint_local[2][1]) * 0.5f +
                      body_chain_collider_cfg.testicle_fine_offset[1][1];
        local[1][2] = (joint_local[1][2] + joint_local[2][2]) * 0.5f +
                      body_chain_collider_cfg.testicle_fine_offset[1][2];
        sample_valid = body_chain_testicle_colliders_plausible(local);
    }

    if (sample_valid) {
        int recent_reload =
            config_hot_reload_tick &&
            now - config_hot_reload_tick <= 6000u;
        int recent_camera =
            captured_camera_change_tick &&
            now - captured_camera_change_tick <= 2500u;
        keep_previous =
            state->testicle_points_ready &&
            state->testicle_points_update_tick &&
            now - state->testicle_points_update_tick <=
                BODY_CHAIN_ENGINE_POINT_STALE_MS;
        if ((recent_reload || recent_camera) && keep_previous) {
            float max_delta = 0.0f;
            for (i = 0; i < 2; i++) {
                float delta = body_collider_distance(
                    local[i], state->testicle_local_position[i]);
                if (sane_probe_float(delta) && delta > max_delta) {
                    max_delta = delta;
                }
            }
            if (max_delta > 0.095f) {
                sample_valid = 0;
                if ((body_chain_collider_cfg.diagnostic || defaults_cfg.debug) &&
                    (!state->last_testicle_reject_log_tick ||
                     now - state->last_testicle_reject_log_tick >= 750u)) {
                    state->last_testicle_reject_log_tick = now;
                    log_line("body-chain-colliders testicle-engine-bones held person=\"%s\" recent_reload=%d recent_camera=%d max_delta=%.5f note=\"rejecting plausible but discontinuous live testicle pivot sample; keeping previous capsule so camera-attached pivots cannot become collision\"",
                             person,
                             recent_reload,
                             recent_camera,
                             max_delta);
                }
            }
        }
    }

    if (sample_valid) {
        memcpy(state->testicle_local_position, local,
               sizeof(state->testicle_local_position));
        memcpy(state->testicle_joint_position, joint_local,
               sizeof(state->testicle_joint_position));
        state->testicle_points_ready = 1;
        state->testicle_points_update_tick = now;
    } else {
        keep_previous =
            state->testicle_points_ready &&
            state->testicle_points_update_tick &&
            now - state->testicle_points_update_tick <=
                BODY_CHAIN_ENGINE_POINT_STALE_MS;
        if ((body_chain_collider_cfg.diagnostic || defaults_cfg.debug) &&
            (!state->last_testicle_reject_log_tick ||
             now - state->last_testicle_reject_log_tick >=
                (DWORD)body_chain_collider_cfg.health_log_ms ||
             !keep_previous)) {
            state->last_testicle_reject_log_tick = now;
            log_line("body-chain-colliders testicle-engine-bones rejected person=\"%s\" complete=%d joint01_local=(%.5f,%.5f,%.5f) joint02_local=(%.5f,%.5f,%.5f) jointEnd_local=(%.5f,%.5f,%.5f) local01=(%.5f,%.5f,%.5f) local02=(%.5f,%.5f,%.5f) kept_previous=%d note=\"discarding transient testicle pivot failure and retaining the last coherent pair\"",
                     person,
                     complete,
                     joint_local[0][0], joint_local[0][1], joint_local[0][2],
                     joint_local[1][0], joint_local[1][1], joint_local[1][2],
                     joint_local[2][0], joint_local[2][1], joint_local[2][2],
                     local[0][0], local[0][1], local[0][2],
                     local[1][0], local[1][1], local[1][2],
                     keep_previous);
        }
        if (!keep_previous) {
            state->testicle_points_ready = 0;
            state->testicle_points_update_tick = 0;
            return 0;
        }
    }

    for (i = 0; i < 2; i++) {
        state->raw[BODY_COLLIDER_TESTICLES_01 + i] = NULL;
        state->position_offset[BODY_COLLIDER_TESTICLES_01 + i] = -4;
        lstrcpynA(state->source_name[BODY_COLLIDER_TESTICLES_01 + i],
                  i == 0 ? "engine-bone:testicles_joint01->02" :
                           "engine-bone:testicles_joint02->End",
                  sizeof(state->source_name[BODY_COLLIDER_TESTICLES_01 + i]));
        state->local_position[BODY_COLLIDER_TESTICLES_01 + i][0] =
            state->testicle_local_position[i][0];
        state->local_position[BODY_COLLIDER_TESTICLES_01 + i][1] =
            state->testicle_local_position[i][1];
        state->local_position[BODY_COLLIDER_TESTICLES_01 + i][2] =
            state->testicle_local_position[i][2];
        state->valid[BODY_COLLIDER_TESTICLES_01 + i] = 1;
    }

    state->local_position[BODY_COLLIDER_TESTICLES_MID][0] =
        (state->testicle_local_position[0][0] +
         state->testicle_local_position[1][0]) * 0.5f;
    state->local_position[BODY_COLLIDER_TESTICLES_MID][1] =
        (state->testicle_local_position[0][1] +
         state->testicle_local_position[1][1]) * 0.5f;
    state->local_position[BODY_COLLIDER_TESTICLES_MID][2] =
        (state->testicle_local_position[0][2] +
         state->testicle_local_position[1][2]) * 0.5f;
    state->raw[BODY_COLLIDER_TESTICLES_MID] = NULL;
    state->position_offset[BODY_COLLIDER_TESTICLES_MID] = -4;
    lstrcpynA(state->source_name[BODY_COLLIDER_TESTICLES_MID],
              "engine-pivot:testicles_mid",
              sizeof(state->source_name[BODY_COLLIDER_TESTICLES_MID]));
    state->valid[BODY_COLLIDER_TESTICLES_MID] = 1;

    if (sample_valid &&
        (body_chain_collider_cfg.diagnostic || defaults_cfg.debug) &&
        (!state->last_testicle_rotation_log_tick ||
         now - state->last_testicle_rotation_log_tick >=
            (DWORD)body_chain_collider_cfg.health_log_ms)) {
        state->last_testicle_rotation_log_tick = now;
        log_line("body-chain-colliders testicle-engine-bones person=\"%s\" joint01_local=(%.5f,%.5f,%.5f) joint02_local=(%.5f,%.5f,%.5f) jointEnd_local=(%.5f,%.5f,%.5f) local01=(%.5f,%.5f,%.5f) local02=(%.5f,%.5f,%.5f) local_mid=(%.5f,%.5f,%.5f) end_source=\"%s\" fine_offsets=(%.5f,%.5f,%.5f;%.5f,%.5f,%.5f) note=\"testicle proxies follow live TK17 bone segments, so PoseEditor rotations move the spheres\"",
                 person,
                 joint_local[0][0], joint_local[0][1], joint_local[0][2],
                 joint_local[1][0], joint_local[1][1], joint_local[1][2],
                 joint_local[2][0], joint_local[2][1], joint_local[2][2],
                 state->local_position[BODY_COLLIDER_TESTICLES_01][0],
                 state->local_position[BODY_COLLIDER_TESTICLES_01][1],
                 state->local_position[BODY_COLLIDER_TESTICLES_01][2],
                 state->local_position[BODY_COLLIDER_TESTICLES_02][0],
                 state->local_position[BODY_COLLIDER_TESTICLES_02][1],
                 state->local_position[BODY_COLLIDER_TESTICLES_02][2],
                 state->local_position[BODY_COLLIDER_TESTICLES_MID][0],
                 state->local_position[BODY_COLLIDER_TESTICLES_MID][1],
                 state->local_position[BODY_COLLIDER_TESTICLES_MID][2],
                 joint02_from_end_midpoint ? "engine-end-midpoint" :
                     (end_found ? "engine" : "extrapolated"),
                 body_chain_collider_cfg.testicle_fine_offset[0][0],
                 body_chain_collider_cfg.testicle_fine_offset[0][1],
                 body_chain_collider_cfg.testicle_fine_offset[0][2],
                 body_chain_collider_cfg.testicle_fine_offset[1][0],
                 body_chain_collider_cfg.testicle_fine_offset[1][1],
                 body_chain_collider_cfg.testicle_fine_offset[1][2]);
    }
    return 1;
}

static void body_collider_apply_mirrored_fine_offset(float local[3],
                                                     const float fine[3],
                                                     int side_sign)
{
    if (!local || !fine) return;
    local[0] += fine[0];
    local[1] += fine[1] * (float)side_sign;
    local[2] += fine[2];
}

static int body_chain_extra_collider_point_plausible(const float local[3])
{
    if (!local) return 0;
    if (!sane_probe_float(local[0]) ||
        !sane_probe_float(local[1]) ||
        !sane_probe_float(local[2])) {
        return 0;
    }
    return physx_absf(local[0]) <= 2.5f &&
           physx_absf(local[1]) <= 2.5f &&
           physx_absf(local[2]) <= 2.5f;
}

static void body_chain_set_extra_node_from_cache(
    body_chain_collider_person_state_t *state,
    int node_index,
    const char *source_name)
{
    if (!state || node_index < 0 || node_index >= BODY_COLLIDER_NODE_COUNT ||
        !state->extra_point_ready[node_index]) {
        return;
    }
    state->raw[node_index] = NULL;
    state->position_offset[node_index] = -4;
    lstrcpynA(state->source_name[node_index],
              source_name ? source_name : "engine-pivot:extra",
              sizeof(state->source_name[node_index]));
    state->source_name[node_index][sizeof(state->source_name[node_index]) - 1] = 0;
    state->local_position[node_index][0] =
        state->extra_local_position[node_index][0];
    state->local_position[node_index][1] =
        state->extra_local_position[node_index][1];
    state->local_position[node_index][2] =
        state->extra_local_position[node_index][2];
    state->valid[node_index] = 1;
}

static int body_chain_update_derived_palm_node(
    body_chain_collider_person_state_t *state,
    int side_sign,
    int wrist_node,
    const int finger_base_nodes[5],
    int palm_node,
    DWORD now)
{
    float average[3] = { 0.0f, 0.0f, 0.0f };
    float palm[3];
    int i;
    int count = 0;
    if (!state || !finger_base_nodes ||
        wrist_node < 0 || wrist_node >= BODY_COLLIDER_NODE_COUNT ||
        palm_node < 0 || palm_node >= BODY_COLLIDER_NODE_COUNT ||
        !state->valid[wrist_node]) {
        return 0;
    }
    for (i = 0; i < 5; i++) {
        int node = finger_base_nodes[i];
        if (node >= 0 && node < BODY_COLLIDER_NODE_COUNT &&
            state->valid[node]) {
            average[0] += state->local_position[node][0];
            average[1] += state->local_position[node][1];
            average[2] += state->local_position[node][2];
            count++;
        }
    }
    if (count < 3) return 0;
    average[0] /= (float)count;
    average[1] /= (float)count;
    average[2] /= (float)count;
    palm[0] = (state->local_position[wrist_node][0] + average[0]) * 0.5f;
    palm[1] = (state->local_position[wrist_node][1] + average[1]) * 0.5f;
    palm[2] = (state->local_position[wrist_node][2] + average[2]) * 0.5f;
    body_collider_apply_mirrored_fine_offset(
        palm, body_chain_collider_cfg.node_fine_offset[palm_node],
        side_sign);
    if (!body_chain_extra_collider_point_plausible(palm)) {
        return 0;
    }
    state->extra_local_position[palm_node][0] = palm[0];
    state->extra_local_position[palm_node][1] = palm[1];
    state->extra_local_position[palm_node][2] = palm[2];
    state->extra_point_ready[palm_node] = 1;
    state->extra_point_update_tick[palm_node] = now;
    body_chain_set_extra_node_from_cache(
        state, palm_node,
        palm_node == BODY_COLLIDER_PALM_L ?
            "derived:palm_L" : "derived:palm_R");
    return 1;
}

static int update_engine_pivot_extra_colliders(
    body_chain_collider_person_state_t *state,
    const char *person,
    const float root_pos[3],
    const float basis_h[3],
    const float basis_v[3],
    const float basis_s[3],
    int scope_mask,
    DWORD now)
{
    static const int palm_left_bases[5] = {
        BODY_COLLIDER_FINGER01_L_01,
        BODY_COLLIDER_FINGER02_L_01,
        BODY_COLLIDER_FINGER03_L_01,
        BODY_COLLIDER_FINGER04_L_01,
        BODY_COLLIDER_FINGER05_L_01
    };
    static const int palm_right_bases[5] = {
        BODY_COLLIDER_FINGER01_R_01,
        BODY_COLLIDER_FINGER02_R_01,
        BODY_COLLIDER_FINGER03_R_01,
        BODY_COLLIDER_FINGER04_R_01,
        BODY_COLLIDER_FINGER05_R_01
    };
    int i;
    int updated = 0;
    if (!state || !person || !root_pos || !basis_h || !basis_v || !basis_s) {
        return 0;
    }
    for (i = 0; i < BODY_COLLIDER_DIRECT_NODE_COUNT; i++) {
        const body_collider_direct_node_def_t *def =
            &body_collider_direct_nodes[i];
        void *object;
        float candidate[3];
        float view[3];
        int keep_previous;
        if (!def->node_name ||
            def->node_index < 0 ||
            def->node_index >= BODY_COLLIDER_NODE_COUNT) {
            continue;
        }
        /* Resolve only groups requested by the active scope union. Breast
           and Butt source pivots are sampled separately from collision
           targets so hands_only still has the simulated pair's live center. */
        if (!(scope_mask & BODY_CHAIN_COLLIDER_GROUP_FULL_BODY) &&
            !((scope_mask & BODY_CHAIN_COLLIDER_GROUP_HANDS) &&
              body_chain_collider_node_is_hand(def->node_index)) &&
            !((scope_mask & BODY_CHAIN_COLLIDER_GROUP_BREAST_SOURCE) &&
              (def->node_index == BODY_COLLIDER_BREAST_L ||
               def->node_index == BODY_COLLIDER_BREAST_R)) &&
            !((scope_mask & BODY_CHAIN_COLLIDER_GROUP_BUTT_SOURCE) &&
              (def->node_index == BODY_COLLIDER_BUTT_L ||
               def->node_index == BODY_COLLIDER_BUTT_R))) {
            continue;
        }
        object = state->extra_object[def->node_index];
        if (!object || !ptr_readable(object, sizeof(void*)) ||
            is_nil_engine_object(object, object) ||
            !state->extra_object_verify_tick[def->node_index] ||
            now - state->extra_object_verify_tick[def->node_index] >=
                BODY_CHAIN_COLLIDER_OBJECT_VERIFY_MS) {
            object = body_collider_engine_pivot_object(
                person, def->node_name, def->fallback_name);
            state->extra_object[def->node_index] = object;
            state->extra_object_verify_tick[def->node_index] = now;
        }
        if (body_chain_camera_pivot_hold_active(now) &&
            state->extra_point_ready[def->node_index]) {
            state->extra_point_update_tick[def->node_index] = now;
        } else if (body_collider_engine_pivot_local_object(
                object, root_pos, basis_h, basis_v, basis_s,
                candidate, view)) {
            body_collider_apply_mirrored_fine_offset(
                candidate,
                body_chain_collider_cfg.node_fine_offset[def->node_index],
                def->side_sign);
            if (body_chain_extra_collider_point_plausible(candidate)) {
                state->extra_local_position[def->node_index][0] =
                    candidate[0];
                state->extra_local_position[def->node_index][1] =
                    candidate[1];
                state->extra_local_position[def->node_index][2] =
                    candidate[2];
                state->extra_point_ready[def->node_index] = 1;
                state->extra_point_update_tick[def->node_index] = now;
            }
        }
        keep_previous =
            state->extra_point_ready[def->node_index] &&
            state->extra_point_update_tick[def->node_index] &&
            now - state->extra_point_update_tick[def->node_index] <=
                BODY_CHAIN_ENGINE_POINT_STALE_MS;
        if (keep_previous) {
            body_chain_set_extra_node_from_cache(
                state, def->node_index, def->source_name);
            updated++;
        } else {
            state->extra_point_ready[def->node_index] = 0;
            state->extra_point_update_tick[def->node_index] = 0;
            state->valid[def->node_index] = 0;
        }
    }
    if (scope_mask & (BODY_CHAIN_COLLIDER_GROUP_HANDS |
                      BODY_CHAIN_COLLIDER_GROUP_FULL_BODY)) {
        updated += body_chain_update_derived_palm_node(
            state, 1, BODY_COLLIDER_WRIST_L, palm_left_bases,
            BODY_COLLIDER_PALM_L, now);
        updated += body_chain_update_derived_palm_node(
            state, -1, BODY_COLLIDER_WRIST_R, palm_right_bases,
            BODY_COLLIDER_PALM_R, now);
    }
    return updated;
}

static int update_engine_pivot_stomach_colliders(
    body_chain_collider_person_state_t *state,
    const char *person,
    const float root_pos[3],
    const float basis_h[3],
    const float basis_v[3],
    const float basis_s[3],
    DWORD now)
{
    static const char *nodes[2] = {
        "spine_joint01", "spine_joint02"
    };
    static const char *fallback_nodes[2] = {
        "Sspine_joint01", "Sspine_joint02"
    };
    static const int node_indices[2] = {
        BODY_COLLIDER_STOMACH_01, BODY_COLLIDER_STOMACH_02
    };
    static const char *source_names[2] = {
        "engine-pivot:spine_joint01",
        "engine-pivot:spine_joint02"
    };
    float candidate[2][3];
    float spine_len = 0.0f;
    int i;
    int complete = 1;
    int sample_valid = 0;

    if (!state || !person || !root_pos || !basis_h || !basis_v || !basis_s) {
        return 0;
    }

    if (body_chain_camera_pivot_hold_active(now) &&
        state->stomach_points_ready) {
        state->stomach_points_update_tick = now;
        if ((body_chain_collider_cfg.diagnostic || defaults_cfg.debug) &&
            (!state->last_stomach_reject_log_tick ||
             now - state->last_stomach_reject_log_tick >= 750u)) {
            state->last_stomach_reject_log_tick = now;
            log_line("body-chain-colliders spine-engine-pivots camera-held person=\"%s\" camera_version=%ld note=\"camera matrix changed recently; holding last stable local spine capsule instead of accepting model-view pivots that can attach to the camera\"",
                     person,
                     captured_camera_version);
        }
        for (i = 0; i < 2; i++) {
            int node_index = node_indices[i];
            state->local_position[node_index][0] =
                state->stomach_local_position[i][0];
            state->local_position[node_index][1] =
                state->stomach_local_position[i][1];
            state->local_position[node_index][2] =
                state->stomach_local_position[i][2];
            state->valid[node_index] = 1;
        }
        return 1;
    }

    for (i = 0; i < 2; i++) {
        float view[3];
        if (!body_collider_engine_pivot_local(person, nodes[i],
                                             fallback_nodes[i],
                                             root_pos, basis_h, basis_v,
                                             basis_s, candidate[i], view)) {
            complete = 0;
            break;
        }
        candidate[i][0] += body_chain_collider_cfg.stomach_fine_offset[i][0];
        candidate[i][1] += body_chain_collider_cfg.stomach_fine_offset[i][1];
        candidate[i][2] += body_chain_collider_cfg.stomach_fine_offset[i][2];
    }

    if (complete) {
        spine_len = body_collider_distance(candidate[0], candidate[1]);
        sample_valid = spine_len >= 0.015f && spine_len <= 0.450f;
        for (i = 0; i < 2 && sample_valid; i++) {
            if (physx_vec3_len(candidate[i]) > 1.25f) {
                sample_valid = 0;
            }
        }
    }

    if (sample_valid) {
        int recent_reload =
            config_hot_reload_tick &&
            now - config_hot_reload_tick <= 6000u;
        int recent_camera =
            captured_camera_change_tick &&
            now - captured_camera_change_tick <= 2500u;
        int keep_previous =
            state->stomach_points_ready &&
            state->stomach_points_update_tick &&
            now - state->stomach_points_update_tick <=
                BODY_CHAIN_ENGINE_POINT_STALE_MS;
        if ((recent_reload || recent_camera) && keep_previous) {
            float max_delta = 0.0f;
            for (i = 0; i < 2; i++) {
                float delta = body_collider_distance(
                    candidate[i], state->stomach_local_position[i]);
                if (sane_probe_float(delta) && delta > max_delta) {
                    max_delta = delta;
                }
            }
            if (max_delta > 0.120f) {
                sample_valid = 0;
                if ((body_chain_collider_cfg.diagnostic || defaults_cfg.debug) &&
                    (!state->last_stomach_reject_log_tick ||
                     now - state->last_stomach_reject_log_tick >= 750u)) {
                    state->last_stomach_reject_log_tick = now;
                    log_line("body-chain-colliders spine-engine-pivots held person=\"%s\" recent_reload=%d recent_camera=%d max_delta=%.5f note=\"rejecting plausible but discontinuous live spine pivot sample; keeping previous capsule so camera-attached pivots cannot become collision\"",
                             person,
                             recent_reload,
                             recent_camera,
                             max_delta);
                }
            }
        }
    }

    if (sample_valid) {
        memcpy(state->stomach_local_position, candidate,
               sizeof(state->stomach_local_position));
        state->stomach_points_ready = 1;
        state->stomach_points_update_tick = now;
    } else {
        int keep_previous =
            state->stomach_points_ready &&
            state->stomach_points_update_tick &&
            now - state->stomach_points_update_tick <=
                BODY_CHAIN_ENGINE_POINT_STALE_MS;
        if (!keep_previous) {
            state->stomach_points_ready = 0;
            state->stomach_points_update_tick = 0;
            for (i = 0; i < 2; i++) {
                state->valid[node_indices[i]] = 0;
            }
        }
        if ((body_chain_collider_cfg.diagnostic || defaults_cfg.debug) &&
               (!state->last_stomach_reject_log_tick ||
                now - state->last_stomach_reject_log_tick >=
                    (DWORD)body_chain_collider_cfg.health_log_ms ||
                !keep_previous)) {
            state->last_stomach_reject_log_tick = now;
            log_line("body-chain-colliders spine-engine-pivots rejected person=\"%s\" complete=%d spine_len=%.5f kept_previous=%d note=\"discarding incoherent live spine sample so spine collision cannot flicker or outlive a removed person\"",
                     person, complete, spine_len, keep_previous);
        }
    }

    if (!state->stomach_points_ready) {
        return 0;
    }

    for (i = 0; i < 2; i++) {
        int node_index = node_indices[i];
        state->raw[node_index] = NULL;
        state->position_offset[node_index] = -4;
        lstrcpynA(state->source_name[node_index], source_names[i],
                  sizeof(state->source_name[node_index]));
        state->source_name[node_index]
            [sizeof(state->source_name[node_index]) - 1] = 0;
        state->local_position[node_index][0] =
            state->stomach_local_position[i][0];
        state->local_position[node_index][1] =
            state->stomach_local_position[i][1];
        state->local_position[node_index][2] =
            state->stomach_local_position[i][2];
        state->valid[node_index] = 1;
    }

    if (sample_valid &&
        (body_chain_collider_cfg.diagnostic || defaults_cfg.debug) &&
        (!state->last_health_log_tick ||
         now - state->last_health_log_tick >=
            (DWORD)body_chain_collider_cfg.health_log_ms)) {
        log_line("body-chain-colliders spine-engine-pivots person=\"%s\" spine01=(%.5f,%.5f,%.5f) spine02=(%.5f,%.5f,%.5f) fine_offsets=(%.5f,%.5f,%.5f;%.5f,%.5f,%.5f) radii=(%.5f,%.5f,%.5f;%.5f,%.5f,%.5f) note=\"spine oval capsule follows live spine_joint01->spine_joint02 pivots\"",
                 person,
                 state->local_position[BODY_COLLIDER_STOMACH_01][0],
                 state->local_position[BODY_COLLIDER_STOMACH_01][1],
                 state->local_position[BODY_COLLIDER_STOMACH_01][2],
                 state->local_position[BODY_COLLIDER_STOMACH_02][0],
                 state->local_position[BODY_COLLIDER_STOMACH_02][1],
                 state->local_position[BODY_COLLIDER_STOMACH_02][2],
                 body_chain_collider_cfg.stomach_fine_offset[0][0],
                 body_chain_collider_cfg.stomach_fine_offset[0][1],
                 body_chain_collider_cfg.stomach_fine_offset[0][2],
                 body_chain_collider_cfg.stomach_fine_offset[1][0],
                 body_chain_collider_cfg.stomach_fine_offset[1][1],
                 body_chain_collider_cfg.stomach_fine_offset[1][2],
                 body_chain_collider_cfg.stomach_radius[0][0],
                 body_chain_collider_cfg.stomach_radius[0][1],
                 body_chain_collider_cfg.stomach_radius[0][2],
                 body_chain_collider_cfg.stomach_radius[1][0],
                 body_chain_collider_cfg.stomach_radius[1][1],
                 body_chain_collider_cfg.stomach_radius[1][2]);
    }

    return 1;
}

static void body_chain_clear_limb_collider_state(
    body_chain_collider_person_state_t *state);

static int update_engine_pivot_limb_colliders(
    body_chain_collider_person_state_t *state,
    const char *person,
    const float root_pos[3],
    const float basis_h[3],
    const float basis_v[3],
    const float basis_s[3],
    DWORD now)
{
    static const char *nodes[4] = {
        "hip_L_joint", "hip_R_joint", "knee_L_joint", "knee_R_joint"
    };
    static const char *fallback_nodes[4] = {
        "Ship_L_joint", "Ship_R_joint", "Sknee_L_joint", "Sknee_R_joint"
    };
    static const int node_indices[6] = {
        BODY_COLLIDER_HIP_L, BODY_COLLIDER_HIP_R,
        BODY_COLLIDER_KNEE_L, BODY_COLLIDER_KNEE_R,
        BODY_COLLIDER_THIGH_L, BODY_COLLIDER_THIGH_R
    };
    static const char *source_names[6] = {
        "engine-pivot:hip_L_joint",
        "engine-pivot:hip_R_joint",
        "engine-pivot:knee_L_joint",
        "engine-pivot:knee_R_joint",
        "engine-pivot:hip_L_to_knee_L_mid",
        "engine-pivot:hip_R_to_knee_R_mid"
    };
    static const int side_signs[4] = { 1, -1, 1, -1 };
    float candidate[6][3];
    float left_length = 0.0f;
    float right_length = 0.0f;
    int i;
    int complete = 1;
    int sample_valid = 0;

    if (!state || !person || !root_pos || !basis_h || !basis_v || !basis_s) {
        return 0;
    }

    if (body_chain_camera_pivot_hold_active(now) &&
        state->limb_points_ready) {
        state->limb_points_update_tick = now;
        if ((body_chain_collider_cfg.diagnostic || defaults_cfg.debug) &&
            (!state->last_limb_reject_log_tick ||
             now - state->last_limb_reject_log_tick >= 750u)) {
            state->last_limb_reject_log_tick = now;
            log_line("body-chain-colliders limb-engine-pivots camera-held person=\"%s\" camera_version=%ld note=\"camera matrix changed recently; holding last stable local hip/thigh/knee capsules instead of accepting model-view pivots that can attach to the camera\"",
                     person,
                     captured_camera_version);
        }
        for (i = 0; i < 6; i++) {
            int node_index = node_indices[i];
            state->local_position[node_index][0] =
                state->limb_local_position[i][0];
            state->local_position[node_index][1] =
                state->limb_local_position[i][1];
            state->local_position[node_index][2] =
                state->limb_local_position[i][2];
            state->valid[node_index] = 1;
        }
        return 1;
    }

    for (i = 0; i < 4; i++) {
        float view[3];
        int node_index = node_indices[i];
        const float *fine =
            (node_index == BODY_COLLIDER_HIP_L ||
             node_index == BODY_COLLIDER_HIP_R) ?
                body_chain_collider_cfg.hip_fine_offset :
                body_chain_collider_cfg.knee_fine_offset;

        if (!body_collider_engine_pivot_local(person, nodes[i],
                                             fallback_nodes[i],
                                             root_pos, basis_h, basis_v,
                                             basis_s, candidate[i], view)) {
            complete = 0;
            break;
        }

        body_collider_apply_mirrored_fine_offset(candidate[i], fine,
                                                 side_signs[i]);
    }

    if (complete) {
        candidate[4][0] = (candidate[0][0] + candidate[2][0]) * 0.5f;
        candidate[4][1] = (candidate[0][1] + candidate[2][1]) * 0.5f;
        candidate[4][2] = (candidate[0][2] + candidate[2][2]) * 0.5f;
        candidate[5][0] = (candidate[1][0] + candidate[3][0]) * 0.5f;
        candidate[5][1] = (candidate[1][1] + candidate[3][1]) * 0.5f;
        candidate[5][2] = (candidate[1][2] + candidate[3][2]) * 0.5f;
        body_collider_apply_mirrored_fine_offset(
            candidate[4], body_chain_collider_cfg.thigh_fine_offset, 1);
        body_collider_apply_mirrored_fine_offset(
            candidate[5], body_chain_collider_cfg.thigh_fine_offset, -1);
        left_length = body_collider_distance(candidate[0], candidate[2]);
        right_length = body_collider_distance(candidate[1], candidate[3]);
        sample_valid = left_length >= 0.15f && left_length <= 0.75f &&
                       right_length >= 0.15f && right_length <= 0.75f;
        for (i = 0; i < 6 && sample_valid; i++) {
            if (physx_vec3_len(candidate[i]) > 1.0f) {
                sample_valid = 0;
            }
        }
    }

    if (sample_valid) {
        float max_delta = 0.0f;
        int recent_reload =
            config_hot_reload_tick &&
            now - config_hot_reload_tick <= 6000u;
        int recent_camera =
            captured_camera_change_tick &&
            now - captured_camera_change_tick <= 2500u;
        int keep_previous =
            state->limb_points_ready &&
            state->limb_points_update_tick &&
            now - state->limb_points_update_tick <=
                BODY_CHAIN_ENGINE_POINT_STALE_MS;

        if ((recent_reload || recent_camera) && keep_previous &&
            !body_chain_limb_points_continuous(
                state, candidate, &max_delta)) {
            sample_valid = 0;
            if ((body_chain_collider_cfg.diagnostic || defaults_cfg.debug) &&
                (!state->last_limb_reject_log_tick ||
                 now - state->last_limb_reject_log_tick >= 750u)) {
                state->last_limb_reject_log_tick = now;
            log_line("body-chain-colliders limb-engine-pivots held person=\"%s\" recent_reload=%d recent_camera=%d max_delta=%.5f note=\"rejecting plausible but discontinuous live limb pivot sample; clearing hip/thigh/knee collision for this frame so stale capsules cannot push the chain\"",
                         person,
                         recent_reload,
                         recent_camera,
                         max_delta);
            }
        }
    }

    if (sample_valid) {
        memcpy(state->limb_local_position, candidate,
               sizeof(state->limb_local_position));
        state->limb_points_ready = 1;
        state->limb_points_update_tick = now;
    } else {
        int had_previous = state->limb_points_ready;
        body_chain_clear_limb_collider_state(state);
        if ((body_chain_collider_cfg.diagnostic || defaults_cfg.debug) &&
               (!state->last_limb_reject_log_tick ||
                now - state->last_limb_reject_log_tick >=
                    (DWORD)body_chain_collider_cfg.health_log_ms ||
                had_previous)) {
            state->last_limb_reject_log_tick = now;
            log_line("body-chain-colliders limb-engine-pivots rejected person=\"%s\" complete=%d left_length=%.5f right_length=%.5f kept_previous=0 previous_ready=%d note=\"discarding incoherent live limb sample so stale hip/thigh/knee collision cannot remain attached to an old pose\"",
                     person, complete, left_length, right_length,
                     had_previous);
        }
    }

    if (!state->limb_points_ready) {
        return 0;
    }

    for (i = 0; i < 6; i++) {
        int node_index = node_indices[i];
        state->raw[node_index] = NULL;
        state->position_offset[node_index] = -4;
        lstrcpynA(state->source_name[node_index], source_names[i],
                  sizeof(state->source_name[node_index]));
        state->source_name[node_index]
            [sizeof(state->source_name[node_index]) - 1] = 0;
        state->local_position[node_index][0] =
            state->limb_local_position[i][0];
        state->local_position[node_index][1] =
            state->limb_local_position[i][1];
        state->local_position[node_index][2] =
            state->limb_local_position[i][2];
        state->valid[node_index] = 1;
    }

    if (sample_valid &&
        (body_chain_collider_cfg.diagnostic || defaults_cfg.debug) &&
        (!state->last_health_log_tick ||
         now - state->last_health_log_tick >=
            (DWORD)body_chain_collider_cfg.health_log_ms)) {
        log_line("body-chain-colliders limb-engine-pivots person=\"%s\" updated=%d hipL=(%.5f,%.5f,%.5f) hipR=(%.5f,%.5f,%.5f) thighL=(%.5f,%.5f,%.5f) thighR=(%.5f,%.5f,%.5f) kneeL=(%.5f,%.5f,%.5f) kneeR=(%.5f,%.5f,%.5f) hip_fine=(%.5f,%.5f,%.5f) thigh_fine=(%.5f,%.5f,%.5f) knee_fine=(%.5f,%.5f,%.5f) note=\"hip/thigh/knee collision follows live TK17 body pivots; only the middle fine-offset component mirrors per side\"",
                 person,
                 6,
                 state->local_position[BODY_COLLIDER_HIP_L][0],
                 state->local_position[BODY_COLLIDER_HIP_L][1],
                 state->local_position[BODY_COLLIDER_HIP_L][2],
                 state->local_position[BODY_COLLIDER_HIP_R][0],
                 state->local_position[BODY_COLLIDER_HIP_R][1],
                 state->local_position[BODY_COLLIDER_HIP_R][2],
                 state->local_position[BODY_COLLIDER_THIGH_L][0],
                 state->local_position[BODY_COLLIDER_THIGH_L][1],
                 state->local_position[BODY_COLLIDER_THIGH_L][2],
                 state->local_position[BODY_COLLIDER_THIGH_R][0],
                 state->local_position[BODY_COLLIDER_THIGH_R][1],
                 state->local_position[BODY_COLLIDER_THIGH_R][2],
                 state->local_position[BODY_COLLIDER_KNEE_L][0],
                 state->local_position[BODY_COLLIDER_KNEE_L][1],
                 state->local_position[BODY_COLLIDER_KNEE_L][2],
                 state->local_position[BODY_COLLIDER_KNEE_R][0],
                 state->local_position[BODY_COLLIDER_KNEE_R][1],
                 state->local_position[BODY_COLLIDER_KNEE_R][2],
                 body_chain_collider_cfg.hip_fine_offset[0],
                 body_chain_collider_cfg.hip_fine_offset[1],
                 body_chain_collider_cfg.hip_fine_offset[2],
                 body_chain_collider_cfg.thigh_fine_offset[0],
                 body_chain_collider_cfg.thigh_fine_offset[1],
                 body_chain_collider_cfg.thigh_fine_offset[2],
                 body_chain_collider_cfg.knee_fine_offset[0],
                 body_chain_collider_cfg.knee_fine_offset[1],
                 body_chain_collider_cfg.knee_fine_offset[2]);
    }

    return 1;
}

static int update_testicle_rotation_colliders(
    body_chain_collider_person_state_t *state,
    const char *person,
    DWORD now)
{
    static const char *nodes[2] = {
        "Stesticles_joint01", "Stesticles_joint02"
    };
    float current[2][3];
    float delta[2][3];
    float pivot02_live[3];
    float center02_parent[3];
    int i;

    if (!state || !person || !body_chain_collider_cfg.live_testicle_bones) {
        return 0;
    }

    for (i = 0; i < 2; i++) {
        char runtime_name[256];
        void *raw;
        make_body_runtime_name(runtime_name, sizeof(runtime_name), person, nodes[i]);
        raw = resolve_axis_map_raw(runtime_name);
        if (!raw ||
            !read_body_collider_rotation(raw,
                body_chain_collider_cfg.testicle_rotation_offset,
                current[i])) {
            state->testicle_rotation_ready[i] = 0;
            state->testicle_rotation_raw[i] = NULL;
            return 0;
        }
        if (!state->testicle_rotation_ready[i] ||
            state->testicle_rotation_raw[i] != raw) {
            state->testicle_rotation_raw[i] = raw;
            state->testicle_rotation_rest[i][0] = current[i][0];
            state->testicle_rotation_rest[i][1] = current[i][1];
            state->testicle_rotation_rest[i][2] = current[i][2];
            state->testicle_rotation_ready[i] = 1;
            log_line("body-chain-colliders testicle-rotation-source person=\"%s\" node=\"%s\" raw=%p offset=0x%03x rest=(%.4f,%.4f,%.4f) note=\"camera-independent local rotation channel\"",
                     person, nodes[i], raw,
                     body_chain_collider_cfg.testicle_rotation_offset,
                     current[i][0], current[i][1], current[i][2]);
        }
        delta[i][0] = body_collider_wrap_degrees(
            current[i][0] - state->testicle_rotation_rest[i][0]);
        delta[i][1] = body_collider_wrap_degrees(
            current[i][1] - state->testicle_rotation_rest[i][1]);
        delta[i][2] = body_collider_wrap_degrees(
            current[i][2] - state->testicle_rotation_rest[i][2]);
        state->raw[BODY_COLLIDER_TESTICLES_01 + i] = raw;
        state->position_offset[BODY_COLLIDER_TESTICLES_01 + i] =
            body_chain_collider_cfg.testicle_rotation_offset;
        lstrcpynA(state->source_name[BODY_COLLIDER_TESTICLES_01 + i],
                  nodes[i],
                  sizeof(state->source_name[BODY_COLLIDER_TESTICLES_01 + i]));
    }

    body_collider_rotate_point_about_pivot(
        body_chain_collider_cfg.local_offset[BODY_COLLIDER_TESTICLES_01],
        body_chain_collider_cfg.testicle_pivot[0], delta[0],
        state->local_position[BODY_COLLIDER_TESTICLES_01]);

    body_collider_rotate_point_about_pivot(
        body_chain_collider_cfg.testicle_pivot[1],
        body_chain_collider_cfg.testicle_pivot[0], delta[0], pivot02_live);
    body_collider_rotate_point_about_pivot(
        body_chain_collider_cfg.local_offset[BODY_COLLIDER_TESTICLES_02],
        body_chain_collider_cfg.testicle_pivot[0], delta[0], center02_parent);
    body_collider_rotate_point_about_pivot(
        center02_parent, pivot02_live, delta[1],
        state->local_position[BODY_COLLIDER_TESTICLES_02]);

    state->local_position[BODY_COLLIDER_TESTICLES_MID][0] =
        (state->local_position[BODY_COLLIDER_TESTICLES_01][0] +
         state->local_position[BODY_COLLIDER_TESTICLES_02][0]) * 0.5f;
    state->local_position[BODY_COLLIDER_TESTICLES_MID][1] =
        (state->local_position[BODY_COLLIDER_TESTICLES_01][1] +
         state->local_position[BODY_COLLIDER_TESTICLES_02][1]) * 0.5f;
    state->local_position[BODY_COLLIDER_TESTICLES_MID][2] =
        (state->local_position[BODY_COLLIDER_TESTICLES_01][2] +
         state->local_position[BODY_COLLIDER_TESTICLES_02][2]) * 0.5f;
    state->raw[BODY_COLLIDER_TESTICLES_MID] = state->raw[BODY_COLLIDER_TESTICLES_01];
    state->position_offset[BODY_COLLIDER_TESTICLES_MID] =
        body_chain_collider_cfg.testicle_rotation_offset;
    lstrcpynA(state->source_name[BODY_COLLIDER_TESTICLES_MID],
              "derived:testicles_mid",
              sizeof(state->source_name[BODY_COLLIDER_TESTICLES_MID]));

    if ((body_chain_collider_cfg.diagnostic || defaults_cfg.debug) &&
        (!state->last_testicle_rotation_log_tick ||
         now - state->last_testicle_rotation_log_tick >=
            (DWORD)body_chain_collider_cfg.health_log_ms)) {
        state->last_testicle_rotation_log_tick = now;
        log_line("body-chain-colliders testicle-rotation person=\"%s\" delta01=(%.3f,%.3f,%.3f) delta02=(%.3f,%.3f,%.3f) local01=(%.5f,%.5f,%.5f) local02=(%.5f,%.5f,%.5f) local_mid=(%.5f,%.5f,%.5f)",
                 person,
                 delta[0][0], delta[0][1], delta[0][2],
                 delta[1][0], delta[1][1], delta[1][2],
                 state->local_position[BODY_COLLIDER_TESTICLES_01][0],
                 state->local_position[BODY_COLLIDER_TESTICLES_01][1],
                 state->local_position[BODY_COLLIDER_TESTICLES_01][2],
                 state->local_position[BODY_COLLIDER_TESTICLES_02][0],
                 state->local_position[BODY_COLLIDER_TESTICLES_02][1],
                 state->local_position[BODY_COLLIDER_TESTICLES_02][2],
                 state->local_position[BODY_COLLIDER_TESTICLES_MID][0],
                 state->local_position[BODY_COLLIDER_TESTICLES_MID][1],
                 state->local_position[BODY_COLLIDER_TESTICLES_MID][2]);
    }
    return 1;
}

static void body_chain_clear_penis_collider_state(
    body_chain_collider_person_state_t *state)
{
    int i;
    if (!state) return;
    state->chain_points_ready = 0;
    state->chain_points_fresh = 0;
    state->chain_points_update_tick = 0;
    for (i = 0; i < 4; i++) {
        state->chain_point_valid[i] = 0;
    }
}

static void body_chain_clear_testicle_collider_state(
    body_chain_collider_person_state_t *state)
{
    int i;
    if (!state) return;
    state->testicle_points_ready = 0;
    state->testicle_points_update_tick = 0;
    for (i = BODY_COLLIDER_TESTICLES_01; i <= BODY_COLLIDER_TESTICLES_MID; i++) {
        state->valid[i] = 0;
        state->raw[i] = NULL;
        state->position_offset[i] = -4;
    }
    for (i = 0; i < 2; i++) {
        state->testicle_rotation_ready[i] = 0;
        state->testicle_rotation_raw[i] = NULL;
    }
}

static void body_chain_clear_limb_collider_state(
    body_chain_collider_person_state_t *state)
{
    static const int node_indices[6] = {
        BODY_COLLIDER_HIP_L, BODY_COLLIDER_HIP_R,
        BODY_COLLIDER_KNEE_L, BODY_COLLIDER_KNEE_R,
        BODY_COLLIDER_THIGH_L, BODY_COLLIDER_THIGH_R
    };
    int i;
    if (!state) return;
    state->limb_points_ready = 0;
    state->limb_points_update_tick = 0;
    for (i = 0; i < 6; i++) {
        int node_index = node_indices[i];
        state->valid[node_index] = 0;
        state->raw[node_index] = NULL;
        state->position_offset[node_index] = -4;
    }
}

#include "physx_collision_frame.c"

static void update_body_chain_colliders_for_person_scope(
    int person_index, DWORD now, int required_scope_mask)
{
    static DWORD refresh_tick[4];
    static int refresh_tick_valid[4];
    static int refresh_scope_mask[4];
    body_chain_collider_person_state_t *state;
    char person[32];
    int i;
    int ready = 1;
    int scene_person_visible;

    if ((!body_chain_collider_cfg.enabled && !room_collision_is_enabled()) ||
        person_index < 0 || person_index >= 4) {
        return;
    }
    if (required_scope_mask < 0) required_scope_mask = 0;
    required_scope_mask &= BODY_CHAIN_COLLIDER_GROUP_ALL;
    if (body_chain_collider_cfg.debug_draw) {
        required_scope_mask = BODY_CHAIN_COLLIDER_GROUP_ALL;
    }
    if (refresh_tick_valid[person_index] &&
        refresh_tick[person_index] == now &&
        (refresh_scope_mask[person_index] & required_scope_mask) ==
            required_scope_mask) {
        return;
    }
    refresh_tick_valid[person_index] = 1;
    refresh_tick[person_index] = now;
    refresh_scope_mask[person_index] = required_scope_mask;
    state = &body_chain_collider_states[person_index];
    state->active_scope_mask = required_scope_mask;
    _snprintf(person, sizeof(person), "Person%02d", person_index + 1);

    scene_person_visible = poseedit_scene_person_visible(person_index);
    if (scene_person_visible == 0) {
        if (!state->scene_liveness_engine_invisible) {
            log_line("body-chain-colliders despawn person=\"%s\" reason=\"TK17 PersonVisible is false\" poseedit=%p person_slot=%d property_id=0x%08lx note=\"camera-independent engine visibility; collision is removed immediately\"",
                     person,
                     captured_poseedit_this,
                     person_index + 1,
                     (unsigned long)SCRIPT_PROPERTY_PERSON_VISIBLE);
        }
        state->scene_liveness_quarantined = 1;
        if (!state->scene_liveness_quarantined_root_raw) {
            state->scene_liveness_quarantined_root_raw =
                state->raw[BODY_COLLIDER_ROOT];
        }
        state->scene_liveness_engine_invisible = 1;
        if (state->ready || state->sample_ready || state->basis_valid) {
            reset_body_chain_collider_state_preserve_scene_liveness(state);
            clear_body_chain_prev_collider_for_person(person_index);
        }
        return;
    }
    if (scene_person_visible > 0 &&
        state->scene_liveness_engine_invisible) {
        log_line("body-chain-colliders respawn person=\"%s\" reason=\"TK17 PersonVisible is true again\" person_slot=%d note=\"releasing engine-visibility quarantine and settling the live body\"",
                 person,
                 person_index + 1);
        state->scene_liveness_quarantined = 0;
        state->scene_liveness_quarantined_root_raw = NULL;
        state->scene_liveness_engine_invisible = 0;
        state->scene_liveness_static_camera_samples = 0;
        state->scene_liveness_camera_version = captured_camera_version;
    }

    /* PoseEditor exposes stable track identity for despawn validation.
       FreeMode and the other runtime modes can legitimately replace or omit
       those tracks while keeping the same live skeleton. Their liveness is
       validated below from PersonVisible, the live root/basis, and the live
       limb/chain pivots instead. */
    if (!body_chain_collider_runtime_mode_active() &&
        !body_chain_collider_person_tracks_current(person_index, person)) {
        if (state->ready || state->sample_ready || state->basis_valid) {
            log_line("body-chain-colliders despawn person=\"%s\" reason=\"current PoseEdit tracks no longer match live body nodes\" note=\"clearing stale passive collision after person was removed from the scene\"",
                     person);
        }
        reset_body_chain_collider_state(state);
        clear_body_chain_prev_collider_for_person(person_index);
        return;
    }

    if (body_chain_collider_cfg.root_local_offsets) {
        char root_name[256];
        void *root_raw;
        int root_offset;
        float root_pos[3];
        float basis_h[3];
        float basis_v[3];
        float basis_s[3];
        int h_off = physics_environment_cfg.gravity_horizontal_basis_offset;
        int v_off = physics_environment_cfg.gravity_vertical_basis_offset;
        int s_off = physics_environment_cfg.gravity_horizontal_secondary_basis_offset;

        make_body_runtime_name(root_name, sizeof(root_name), person, "root");
        root_raw = resolve_axis_map_raw(root_name);
        root_offset = body_chain_collider_cfg.position_offset >= 0 ?
            body_chain_collider_cfg.position_offset :
            body_chain_physics_cfg.root_offset;

        if (!root_raw ||
            !read_body_collider_position(root_raw, root_offset, root_pos) ||
            !read_normalized_basis_vector(root_raw, h_off, basis_h) ||
            !read_normalized_basis_vector(root_raw, v_off, basis_v) ||
            !read_normalized_basis_vector(root_raw, s_off, basis_s)) {
            ready = 0;
            state->basis_valid = 0;
            for (i = 0; i < BODY_COLLIDER_NODE_COUNT; i++) {
                state->valid[i] = 0;
            }
            body_chain_collider_update_settle_gate(
                state, person, now, 0, 0, 1, NULL);
        } else {
            float draw_root_pos[3];
            int draw_anchor_found = 0;
            int chain_points_ok = 0;
            int limb_points_ok = 0;
            int stomach_points_ok = 0;
            int extra_points = 0;
            int root_changed = state->raw[BODY_COLLIDER_ROOT] != root_raw ||
                               state->position_offset[BODY_COLLIDER_ROOT] != root_offset;
            if (root_changed) {
                /* Body nodes share the root lifecycle.  A new root invalidates
                   every cached direct collider object before any pivot call. */
                memset(state->extra_object, 0,
                       sizeof(state->extra_object));
                memset(state->extra_object_verify_tick, 0,
                       sizeof(state->extra_object_verify_tick));
            }
            /* Collision math is in body-chain local space, whose origin is the
               penis-chain base rather than TK17's skeleton root pivot. Anchor
               debug draw there so the visible proxy matches the solver frame. */
            if (body_collider_engine_pivot_view(
                    person,
                    "penis_joint01",
                    NULL,
                    draw_root_pos)) {
                draw_anchor_found = 1;
                root_pos[0] = draw_root_pos[0];
                root_pos[1] = draw_root_pos[1];
                root_pos[2] = draw_root_pos[2];
            }
            if (!body_chain_collider_cfg.penis_collision_enabled) {
                draw_anchor_found = 1;
            }
            if (!body_chain_collider_scene_liveness(
                    state, person, root_raw, root_pos, root_changed,
                    scene_person_visible, now)) {
                if (state->ready || state->sample_ready || state->basis_valid) {
                    reset_body_chain_collider_state_preserve_scene_liveness(
                        state);
                    clear_body_chain_prev_collider_for_person(person_index);
                }
                return;
            }
            memcpy(state->basis_h, basis_h, sizeof(state->basis_h));
            memcpy(state->basis_v, basis_v, sizeof(state->basis_v));
            memcpy(state->basis_s, basis_s, sizeof(state->basis_s));
            state->basis_valid = 1;
            if (body_chain_collider_cfg.penis_collision_enabled) {
                chain_points_ok = update_engine_pivot_body_chain_points(
                    state, person_index, person, root_pos,
                    basis_h, basis_v, basis_s, now);
            } else {
                chain_points_ok = 1;
                body_chain_clear_penis_collider_state(state);
            }
            for (i = 0; i < BODY_COLLIDER_NODE_COUNT; i++) {
                const float *local = body_chain_collider_cfg.local_offset[i];
                state->raw[i] = root_raw;
                state->position_offset[i] = root_offset;
                lstrcpynA(state->source_name[i],
                          body_chain_collider_node_label(i),
                          sizeof(state->source_name[i]));
                state->local_position[i][0] = local[0];
                state->local_position[i][1] = local[1];
                state->local_position[i][2] = local[2];
                state->valid[i] =
                    i <= BODY_COLLIDER_TESTICLES_MID &&
                    (required_scope_mask &
                     (BODY_CHAIN_COLLIDER_GROUP_GENITALS |
                      BODY_CHAIN_COLLIDER_GROUP_PELVIS |
                      BODY_CHAIN_COLLIDER_GROUP_FULL_BODY));
            }
            if (!(required_scope_mask &
                  (BODY_CHAIN_COLLIDER_GROUP_GENITALS |
                   BODY_CHAIN_COLLIDER_GROUP_PELVIS |
                   BODY_CHAIN_COLLIDER_GROUP_FULL_BODY)) ||
                !body_chain_collider_cfg.testicle_collision_enabled) {
                body_chain_clear_testicle_collider_state(state);
            }
            if (required_scope_mask &
                (BODY_CHAIN_COLLIDER_GROUP_PELVIS |
                 BODY_CHAIN_COLLIDER_GROUP_FULL_BODY)) {
                limb_points_ok = update_engine_pivot_limb_colliders(
                    state, person, root_pos, basis_h, basis_v, basis_s, now);
                stomach_points_ok = update_engine_pivot_stomach_colliders(
                    state, person, root_pos, basis_h, basis_v, basis_s, now);
            } else {
                /* Genitals-only response never consumes pelvis/spine edges. */
                limb_points_ok = 1;
                stomach_points_ok = 1;
            }
            (void)stomach_points_ok;
            if (required_scope_mask &
                (BODY_CHAIN_COLLIDER_GROUP_HANDS |
                 BODY_CHAIN_COLLIDER_GROUP_FULL_BODY |
                 BODY_CHAIN_COLLIDER_GROUP_BREAST_SOURCE |
                 BODY_CHAIN_COLLIDER_GROUP_BUTT_SOURCE)) {
                extra_points = update_engine_pivot_extra_colliders(
                    state, person, root_pos, basis_h, basis_v, basis_s,
                    required_scope_mask, now);
            }
            (void)extra_points;
            if ((required_scope_mask &
                 (BODY_CHAIN_COLLIDER_GROUP_GENITALS |
                  BODY_CHAIN_COLLIDER_GROUP_PELVIS |
                  BODY_CHAIN_COLLIDER_GROUP_FULL_BODY)) &&
                body_chain_collider_cfg.testicle_collision_enabled &&
                body_chain_collider_cfg.live_testicle_bones) {
                int testicle_physx_owner =
                    testicle_physics_cfg.enabled &&
                    testicle_physics_cfg.enabled_person[person_index];
                if (testicle_physx_owner) {
                    /* Enabled Testicle PhysX writes Stesticles_joint01/02, but
                       TK17 exposes the rendered armature pivots through the
                       regular testicles_joint TNodes.  Do not query the
                       Stesticles SJoint objects with the model-view pivot API. */
                    if (!update_engine_pivot_testicle_colliders(
                            state, person, root_pos, basis_h, basis_v,
                            basis_s, now)) {
                        update_testicle_rotation_colliders(state, person, now);
                    }
                } else if (!body_chain_collider_cfg.testicles_bone_head_mode ||
                           !update_engine_pivot_testicle_colliders(
                               state, person, root_pos, basis_h, basis_v,
                               basis_s, now)) {
                    update_testicle_rotation_colliders(state, person, now);
                }
            }
            if (!(required_scope_mask &
                  (BODY_CHAIN_COLLIDER_GROUP_GENITALS |
                   BODY_CHAIN_COLLIDER_GROUP_PELVIS |
                   BODY_CHAIN_COLLIDER_GROUP_FULL_BODY)) ||
                !body_chain_collider_cfg.testicle_collision_enabled) {
                body_chain_clear_testicle_collider_state(state);
            }
            for (i = 0; i < BODY_COLLIDER_NODE_COUNT; i++) {
                const float *local = state->local_position[i];
                if (!state->valid[i]) {
                    continue;
                }
                state->view_position[i][0] =
                    root_pos[0] +
                    local[0] * basis_h[0] +
                    local[1] * basis_v[0] +
                    local[2] * basis_s[0];
                state->view_position[i][1] =
                    root_pos[1] +
                    local[0] * basis_h[1] +
                    local[1] * basis_v[1] +
                    local[2] * basis_s[1];
                state->view_position[i][2] =
                    root_pos[2] +
                    local[0] * basis_h[2] +
                    local[1] * basis_v[2] +
                    local[2] * basis_s[2];
                if (!camera_view_to_world_point(state->view_position[i],
                                                state->world_position[i])) {
                    state->world_position[i][0] = state->view_position[i][0];
                    state->world_position[i][1] = state->view_position[i][1];
                    state->world_position[i][2] = state->view_position[i][2];
                }
            }
            if (!draw_anchor_found || !chain_points_ok || !limb_points_ok) {
                ready = 0;
            }
            ready = body_chain_collider_update_settle_gate(
                state, person, now, ready, root_changed,
                body_chain_collider_cfg.penis_collision_enabled, root_pos);
            if (!ready || root_changed) state->ready_logged = 0;
            if (ready && !state->ready_logged) {
                state->ready_logged = 1;
                log_line("body-chain-colliders ready person=\"%s\" mode=\"root-local\" root_raw=%p root_offset=0x%03x basis_offsets=(h=0x%03x,v=0x%03x,s=0x%03x) draw_anchor=\"penis_joint01\" draw_anchor_found=%d draw_origin_view=(%.5f,%.5f,%.5f) local_offsets=(pelvis=%.3f,%.3f,%.3f hipL=%.3f,%.3f,%.3f hipR=%.3f,%.3f,%.3f thighL=%.3f,%.3f,%.3f thighR=%.3f,%.3f,%.3f kneeL=%.3f,%.3f,%.3f kneeR=%.3f,%.3f,%.3f test01=%.3f,%.3f,%.3f test02=%.3f,%.3f,%.3f test_mid=%.3f,%.3f,%.3f) response_enabled=%d penis_collision=%d testicle_collision=%d note=\"local-space collision response uses live engine pivots when available; debug draw is anchored to the body-chain base\"",
                         person,
                         root_raw,
                         root_offset,
                         h_off,
                         v_off,
                         s_off,
                         draw_anchor_found,
                         root_pos[0],
                         root_pos[1],
                         root_pos[2],
                         body_chain_collider_cfg.local_offset[BODY_COLLIDER_ROOT][0],
                         body_chain_collider_cfg.local_offset[BODY_COLLIDER_ROOT][1],
                         body_chain_collider_cfg.local_offset[BODY_COLLIDER_ROOT][2],
                         body_chain_collider_cfg.local_offset[BODY_COLLIDER_HIP_L][0],
                         body_chain_collider_cfg.local_offset[BODY_COLLIDER_HIP_L][1],
                         body_chain_collider_cfg.local_offset[BODY_COLLIDER_HIP_L][2],
                         body_chain_collider_cfg.local_offset[BODY_COLLIDER_HIP_R][0],
                         body_chain_collider_cfg.local_offset[BODY_COLLIDER_HIP_R][1],
                         body_chain_collider_cfg.local_offset[BODY_COLLIDER_HIP_R][2],
                         body_chain_collider_cfg.local_offset[BODY_COLLIDER_THIGH_L][0],
                         body_chain_collider_cfg.local_offset[BODY_COLLIDER_THIGH_L][1],
                         body_chain_collider_cfg.local_offset[BODY_COLLIDER_THIGH_L][2],
                         body_chain_collider_cfg.local_offset[BODY_COLLIDER_THIGH_R][0],
                         body_chain_collider_cfg.local_offset[BODY_COLLIDER_THIGH_R][1],
                         body_chain_collider_cfg.local_offset[BODY_COLLIDER_THIGH_R][2],
                         body_chain_collider_cfg.local_offset[BODY_COLLIDER_KNEE_L][0],
                         body_chain_collider_cfg.local_offset[BODY_COLLIDER_KNEE_L][1],
                         body_chain_collider_cfg.local_offset[BODY_COLLIDER_KNEE_L][2],
                         body_chain_collider_cfg.local_offset[BODY_COLLIDER_KNEE_R][0],
                         body_chain_collider_cfg.local_offset[BODY_COLLIDER_KNEE_R][1],
                         body_chain_collider_cfg.local_offset[BODY_COLLIDER_KNEE_R][2],
                         body_chain_collider_cfg.local_offset[BODY_COLLIDER_TESTICLES_01][0],
                         body_chain_collider_cfg.local_offset[BODY_COLLIDER_TESTICLES_01][1],
                         body_chain_collider_cfg.local_offset[BODY_COLLIDER_TESTICLES_01][2],
                         body_chain_collider_cfg.local_offset[BODY_COLLIDER_TESTICLES_02][0],
                         body_chain_collider_cfg.local_offset[BODY_COLLIDER_TESTICLES_02][1],
                         body_chain_collider_cfg.local_offset[BODY_COLLIDER_TESTICLES_02][2],
                         body_chain_collider_cfg.local_offset[BODY_COLLIDER_TESTICLES_MID][0],
                         body_chain_collider_cfg.local_offset[BODY_COLLIDER_TESTICLES_MID][1],
                         body_chain_collider_cfg.local_offset[BODY_COLLIDER_TESTICLES_MID][2],
                         body_chain_collider_cfg.response_enabled,
                         body_chain_collider_cfg.penis_collision_enabled,
                         body_chain_collider_cfg.testicle_collision_enabled);
            }
        }
    } else {
        state->basis_valid = 0;
        for (i = 0; i < BODY_COLLIDER_NODE_COUNT; i++) {
            float position[3];
            if (state->position_offset[i] < -2 || !state->raw[i]) {
                state->valid[i] = 0;
                if (!resolve_body_collider_source(state, person, i)) {
                    ready = 0;
                    continue;
                }
            }
            if (!read_body_collider_position(state->raw[i],
                                             state->position_offset[i],
                                             position)) {
                state->valid[i] = 0;
                state->raw[i] = NULL;
                state->position_offset[i] = -3;
                ready = 0;
                continue;
            }
            state->view_position[i][0] = position[0];
            state->view_position[i][1] = position[1];
            state->view_position[i][2] = position[2];
            if (!camera_view_to_world_point(state->view_position[i],
                                            state->world_position[i])) {
                state->world_position[i][0] = state->view_position[i][0];
                state->world_position[i][1] = state->view_position[i][1];
                state->world_position[i][2] = state->view_position[i][2];
            }
            state->valid[i] = 1;
        }
        if (!body_chain_collider_cfg.testicle_collision_enabled) {
            body_chain_clear_testicle_collider_state(state);
        }

        ready = body_chain_collider_update_settle_gate(
            state, person, now, ready, 0, 0,
            state->valid[BODY_COLLIDER_ROOT] ?
                state->view_position[BODY_COLLIDER_ROOT] : NULL);
        if (ready && !state->ready_logged) {
            state->ready_logged = 1;
            log_line("body-chain-colliders ready person=\"%s\" offsets=(root=0x%03x,hipL=0x%03x,hipR=0x%03x,kneeL=0x%03x,kneeR=0x%03x,test01=0x%03x,test02=0x%03x) sources=(%s,%s,%s,%s,%s,%s,%s) mode=\"raw-scan validation-only; collision response requires root_local_offsets=true\"",
                     person,
                     state->position_offset[BODY_COLLIDER_ROOT],
                     state->position_offset[BODY_COLLIDER_HIP_L],
                     state->position_offset[BODY_COLLIDER_HIP_R],
                     state->position_offset[BODY_COLLIDER_KNEE_L],
                     state->position_offset[BODY_COLLIDER_KNEE_R],
                     state->position_offset[BODY_COLLIDER_TESTICLES_01],
                     state->position_offset[BODY_COLLIDER_TESTICLES_02],
                     state->source_name[BODY_COLLIDER_ROOT],
                     state->source_name[BODY_COLLIDER_HIP_L],
                     state->source_name[BODY_COLLIDER_HIP_R],
                     state->source_name[BODY_COLLIDER_KNEE_L],
                     state->source_name[BODY_COLLIDER_KNEE_R],
                     state->source_name[BODY_COLLIDER_TESTICLES_01],
                     state->source_name[BODY_COLLIDER_TESTICLES_02]);
        }
    }
    body_collision_frame_update(state, person, now);
    if (defaults_cfg.debug &&
        (!state->last_health_log_tick ||
         now - state->last_health_log_tick >=
             (DWORD)body_chain_collider_cfg.health_log_ms)) {
        state->last_health_log_tick = now;
        log_line("body-chain-colliders health person=\"%s\" ready=%d sample_ready=%d settling_ms=%lu root_quiet_ms=%lu camera_version=%ld camera_valid=%d root_view=(%.5f,%.5f,%.5f) hip_L_view=(%.5f,%.5f,%.5f) hip_R_view=(%.5f,%.5f,%.5f) thigh_L_view=(%.5f,%.5f,%.5f) thigh_R_view=(%.5f,%.5f,%.5f) knee_L_view=(%.5f,%.5f,%.5f) knee_R_view=(%.5f,%.5f,%.5f) testicles01_view=(%.5f,%.5f,%.5f) testicles02_view=(%.5f,%.5f,%.5f) distances_view=(hipL_to_thighL=%.5f,thighL_to_kneeL=%.5f,hipR_to_thighR=%.5f,thighR_to_kneeR=%.5f,testicles=%.5f)",
                 person,
                 ready,
                 state->sample_ready,
                 (!state->ready && state->settle_start_tick) ?
                    (unsigned long)(now - state->settle_start_tick) : 0ul,
                 (!state->ready && state->settle_root_quiet_tick) ?
                    (unsigned long)(now - state->settle_root_quiet_tick) : 0ul,
                 captured_camera_version,
                 captured_camera_inverse_valid,
                 state->view_position[BODY_COLLIDER_ROOT][0],
                 state->view_position[BODY_COLLIDER_ROOT][1],
                 state->view_position[BODY_COLLIDER_ROOT][2],
                 state->view_position[BODY_COLLIDER_HIP_L][0],
                 state->view_position[BODY_COLLIDER_HIP_L][1],
                 state->view_position[BODY_COLLIDER_HIP_L][2],
                 state->view_position[BODY_COLLIDER_HIP_R][0],
                 state->view_position[BODY_COLLIDER_HIP_R][1],
                 state->view_position[BODY_COLLIDER_HIP_R][2],
                 state->view_position[BODY_COLLIDER_THIGH_L][0],
                 state->view_position[BODY_COLLIDER_THIGH_L][1],
                 state->view_position[BODY_COLLIDER_THIGH_L][2],
                 state->view_position[BODY_COLLIDER_THIGH_R][0],
                 state->view_position[BODY_COLLIDER_THIGH_R][1],
                 state->view_position[BODY_COLLIDER_THIGH_R][2],
                 state->view_position[BODY_COLLIDER_KNEE_L][0],
                 state->view_position[BODY_COLLIDER_KNEE_L][1],
                 state->view_position[BODY_COLLIDER_KNEE_L][2],
                 state->view_position[BODY_COLLIDER_KNEE_R][0],
                 state->view_position[BODY_COLLIDER_KNEE_R][1],
                 state->view_position[BODY_COLLIDER_KNEE_R][2],
                 state->view_position[BODY_COLLIDER_TESTICLES_01][0],
                 state->view_position[BODY_COLLIDER_TESTICLES_01][1],
                 state->view_position[BODY_COLLIDER_TESTICLES_01][2],
                 state->view_position[BODY_COLLIDER_TESTICLES_02][0],
                 state->view_position[BODY_COLLIDER_TESTICLES_02][1],
                 state->view_position[BODY_COLLIDER_TESTICLES_02][2],
                 body_collider_distance(state->view_position[BODY_COLLIDER_HIP_L],
                                        state->view_position[BODY_COLLIDER_THIGH_L]),
                 body_collider_distance(state->view_position[BODY_COLLIDER_THIGH_L],
                                        state->view_position[BODY_COLLIDER_KNEE_L]),
                 body_collider_distance(state->view_position[BODY_COLLIDER_HIP_R],
                                        state->view_position[BODY_COLLIDER_THIGH_R]),
                 body_collider_distance(state->view_position[BODY_COLLIDER_THIGH_R],
                                        state->view_position[BODY_COLLIDER_KNEE_R]),
                 body_collider_distance(state->view_position[BODY_COLLIDER_TESTICLES_01],
                                        state->view_position[BODY_COLLIDER_TESTICLES_02]));
    }
}

/* Callers without a narrower collision scope retain the historical full-body
   refresh.  Scoped penis/testicle refreshes use the function above directly. */
static void update_body_chain_colliders_for_person(int person_index, DWORD now)
{
    update_body_chain_colliders_for_person_scope(
        person_index, now, BODY_CHAIN_COLLIDER_GROUP_ALL);
}

static float body_chain_collider_radius_for_node(int node_index)
{
    if (node_index < 0 || node_index >= BODY_COLLIDER_NODE_COUNT) {
        return body_chain_collider_cfg.chain_radius;
    }
    return body_chain_radius_vec3_max(
        body_chain_collider_cfg.node_radius[node_index]);
}

static float body_chain_collider_visual_radius_for_node(int node_index)
{
    float radius;
    if (node_index < 0 || node_index >= BODY_COLLIDER_NODE_COUNT) {
        return body_chain_collider_cfg.chain_radius;
    }
    radius = body_chain_radius_vec3_max(
        body_chain_collider_cfg.node_radius[node_index]) *
        body_chain_collider_cfg.response_radius_scale;
    return physx_clampf(radius, 0.0001f, 4.0f);
}

static float body_chain_collider_effective_radius_for_node(int node_index)
{
    return body_chain_collider_radius_for_node(node_index) *
           body_chain_collider_cfg.response_radius_scale +
           body_chain_collider_cfg.chain_radius;
}

static void body_chain_collider_effective_stomach_radius_axes(
    int stomach_index,
    float out[3])
{
    int axis;
    if (!out) return;
    stomach_index = stomach_index ? 1 : 0;
    for (axis = 0; axis < 3; axis++) {
        out[axis] =
            body_chain_collider_cfg.stomach_radius[stomach_index][axis] *
            body_chain_collider_cfg.response_radius_scale +
            body_chain_collider_cfg.chain_radius;
        out[axis] = physx_clampf(out[axis], 0.001f, 4.0f);
    }
}

static void body_chain_collider_visual_stomach_radius_axes(
    int stomach_index,
    float out[3])
{
    int axis;
    if (!out) return;
    stomach_index = stomach_index ? 1 : 0;
    for (axis = 0; axis < 3; axis++) {
        out[axis] =
            body_chain_collider_cfg.stomach_radius[stomach_index][axis] *
            body_chain_collider_cfg.response_radius_scale;
        out[axis] = physx_clampf(out[axis], 0.0001f, 4.0f);
    }
}

static void body_chain_collider_effective_radius_axes_for_node(
    int node_index,
    float out[3])
{
    int axis;
    if (!out) return;
    if (node_index < 0 || node_index >= BODY_COLLIDER_NODE_COUNT) {
        body_chain_set_vec3(out, body_chain_collider_cfg.chain_radius,
                            body_chain_collider_cfg.chain_radius,
                            body_chain_collider_cfg.chain_radius);
        return;
    }
    for (axis = 0; axis < 3; axis++) {
        out[axis] =
            body_chain_collider_cfg.node_radius[node_index][axis] *
            body_chain_collider_cfg.response_radius_scale +
            body_chain_collider_cfg.chain_radius;
        out[axis] = physx_clampf(out[axis], 0.001f, 4.0f);
    }
}

static void body_chain_collider_visual_radius_axes_for_node(
    int node_index,
    float out[3])
{
    int axis;
    if (!out) return;
    if (node_index < 0 || node_index >= BODY_COLLIDER_NODE_COUNT) {
        body_chain_set_vec3(out, body_chain_collider_cfg.chain_radius,
                            body_chain_collider_cfg.chain_radius,
                            body_chain_collider_cfg.chain_radius);
        return;
    }
    for (axis = 0; axis < 3; axis++) {
        out[axis] =
            body_chain_collider_cfg.node_radius[node_index][axis] *
            body_chain_collider_cfg.response_radius_scale;
        out[axis] = physx_clampf(out[axis], 0.0001f, 4.0f);
    }
}

static float body_chain_collider_node_sweep_radius(
    const float sweep[BODY_COLLIDER_NODE_COUNT],
    int node_index)
{
    if (!sweep ||
        node_index < 0 || node_index >= BODY_COLLIDER_NODE_COUNT) {
        return 0.0f;
    }
    return sweep[node_index];
}

static float body_chain_collider_pair_sweep_radius(
    const float sweep[BODY_COLLIDER_NODE_COUNT],
    int start_node,
    int end_node)
{
    float a = body_chain_collider_node_sweep_radius(sweep, start_node);
    float b = body_chain_collider_node_sweep_radius(sweep, end_node);
    return a > b ? a : b;
}

static const char *body_chain_collider_node_label(int node_index)
{
    switch (node_index) {
    case BODY_COLLIDER_ROOT: return "pelvis";
    case BODY_COLLIDER_STOMACH_01: return "spine01";
    case BODY_COLLIDER_STOMACH_02: return "spine02";
    case BODY_COLLIDER_HIP_L: return "hip_L";
    case BODY_COLLIDER_HIP_R: return "hip_R";
    case BODY_COLLIDER_KNEE_L: return "knee_L";
    case BODY_COLLIDER_KNEE_R: return "knee_R";
    case BODY_COLLIDER_THIGH_L: return "thigh_L";
    case BODY_COLLIDER_THIGH_R: return "thigh_R";
    case BODY_COLLIDER_TESTICLES_01: return "testicles01";
    case BODY_COLLIDER_TESTICLES_02: return "testicles02";
    case BODY_COLLIDER_TESTICLES_MID: return "testicles_mid";
    case BODY_COLLIDER_STOMACH_03: return "spine03";
    case BODY_COLLIDER_STOMACH_04: return "spine04";
    case BODY_COLLIDER_NECK_01: return "neck01";
    case BODY_COLLIDER_ANKLE_L: return "ankle_L";
    case BODY_COLLIDER_ANKLE_R: return "ankle_R";
    case BODY_COLLIDER_BALL_L: return "ball_L";
    case BODY_COLLIDER_BALL_R: return "ball_R";
    case BODY_COLLIDER_BREAST_L: return "breast_L";
    case BODY_COLLIDER_BREAST_R: return "breast_R";
    case BODY_COLLIDER_HEAD_02: return "head02";
    case BODY_COLLIDER_CLAVICLE_L: return "clavicle_L";
    case BODY_COLLIDER_CLAVICLE_R: return "clavicle_R";
    case BODY_COLLIDER_SHOULDER_L: return "shoulder_L";
    case BODY_COLLIDER_SHOULDER_R: return "shoulder_R";
    case BODY_COLLIDER_ELBOW_L: return "elbow_L";
    case BODY_COLLIDER_ELBOW_R: return "elbow_R";
    case BODY_COLLIDER_FOREARM_L: return "forearm_L";
    case BODY_COLLIDER_FOREARM_R: return "forearm_R";
    case BODY_COLLIDER_WRIST_L: return "wrist_L";
    case BODY_COLLIDER_WRIST_R: return "wrist_R";
    case BODY_COLLIDER_PALM_L: return "palm_L";
    case BODY_COLLIDER_PALM_R: return "palm_R";
    default: return "body_extra";
    }
}

static int body_chain_collider_node_radius_is_oval(int node_index)
{
    const float *r;
    if (node_index < 0 || node_index >= BODY_COLLIDER_NODE_COUNT) return 0;
    r = body_chain_collider_cfg.node_radius[node_index];
    return physx_absf(r[0] - r[1]) > 0.0001f ||
           physx_absf(r[0] - r[2]) > 0.0001f ||
           physx_absf(r[1] - r[2]) > 0.0001f;
}

static DWORD body_chain_collider_node_color_d3d(int node_index)
{
    if (node_index == BODY_COLLIDER_TESTICLES_01 ||
        node_index == BODY_COLLIDER_TESTICLES_02 ||
        node_index == BODY_COLLIDER_TESTICLES_MID) {
        return 0xffff40ff;
    }
    if (node_index == BODY_COLLIDER_HIP_L ||
        node_index == BODY_COLLIDER_HIP_R ||
        node_index == BODY_COLLIDER_THIGH_L ||
        node_index == BODY_COLLIDER_THIGH_R ||
        node_index == BODY_COLLIDER_KNEE_L ||
        node_index == BODY_COLLIDER_KNEE_R ||
        node_index == BODY_COLLIDER_ANKLE_L ||
        node_index == BODY_COLLIDER_ANKLE_R ||
        node_index == BODY_COLLIDER_BALL_L ||
        node_index == BODY_COLLIDER_BALL_R) {
        return 0xff20ffff;
    }
    if (node_index == BODY_COLLIDER_HEAD_02 ||
        (node_index >= BODY_COLLIDER_FINGER01_L_01 &&
         node_index <= BODY_COLLIDER_FINGER05_R_END)) {
        return 0xffff4040;
    }
    if (node_index == BODY_COLLIDER_BREAST_L ||
        node_index == BODY_COLLIDER_BREAST_R) {
        return 0xffff80c0;
    }
    if (node_index == BODY_COLLIDER_BUTT_L ||
        node_index == BODY_COLLIDER_BUTT_R) {
        return 0xffc080ff;
    }
    if (node_index >= BODY_COLLIDER_CLAVICLE_L &&
        node_index <= BODY_COLLIDER_PALM_R) {
        return 0xffffffff;
    }
    return 0xff80ff40;
}

static void body_chain_collider_node_color_rgb(int node_index,
                                               unsigned char *r,
                                               unsigned char *g,
                                               unsigned char *b)
{
    DWORD color = body_chain_collider_node_color_d3d(node_index);
    if (r) *r = (unsigned char)((color >> 16) & 0xff);
    if (g) *g = (unsigned char)((color >> 8) & 0xff);
    if (b) *b = (unsigned char)(color & 0xff);
}

static int body_chain_simulated_points_local(
    const body_chain_person_state_t *chain_state,
    const body_chain_physics_config_t *cfg,
    const float angle_delta[3][2],
    float points[4][3])
{
    const float deg_to_rad = 0.01745329251994329577f;
    float pos[3] = { 0.0f, 0.0f, 0.0f };
    float cumulative_h = 0.0f;
    float cumulative_v = 0.0f;
    int i;

    if (!chain_state || !cfg || !points) return 0;
    memset(points, 0, sizeof(float) * 4 * 3);

    for (i = 0; i < 3; i++) {
        float len = body_chain_collider_cfg.link_length[i];
        float h_rad;
        float v_rad;
        float cv;
        cumulative_h += chain_state->angle[i][cfg->rotation_tail_axis[0]] +
            (angle_delta ? angle_delta[i][0] : 0.0f);
        cumulative_v += chain_state->angle[i][cfg->rotation_tail_axis[1]] +
            (angle_delta ? angle_delta[i][1] : 0.0f);
        h_rad = cumulative_h * deg_to_rad;
        v_rad = cumulative_v * deg_to_rad;
        cv = (float)cos((double)v_rad);
        pos[0] += -len * cv * (float)cos((double)h_rad);
        pos[1] +=  len * (float)sin((double)v_rad);
        pos[2] +=  len * cv * (float)sin((double)h_rad);
        points[i + 1][0] = pos[0];
        points[i + 1][1] = pos[1];
        points[i + 1][2] = pos[2];
    }
    return 1;
}

static int body_chain_collider_local_to_view(
    const body_chain_collider_person_state_t *state,
    const float local[3],
    float view[3])
{
    const float *root;
    if (!state || !local || !view || !state->basis_valid ||
        !state->valid[BODY_COLLIDER_ROOT]) {
        return 0;
    }
    root = state->view_position[BODY_COLLIDER_ROOT];
    view[0] = root[0] +
              local[0] * state->basis_h[0] +
              local[1] * state->basis_v[0] +
              local[2] * state->basis_s[0];
    view[1] = root[1] +
              local[0] * state->basis_h[1] +
              local[1] * state->basis_v[1] +
              local[2] * state->basis_s[1];
    view[2] = root[2] +
              local[0] * state->basis_h[2] +
              local[1] * state->basis_v[2] +
              local[2] * state->basis_s[2];
    return sane_probe_float(view[0]) &&
           sane_probe_float(view[1]) &&
           sane_probe_float(view[2]);
}

static void body_chain_closest_segment_pair(const float p1[3],
                                            const float q1[3],
                                            const float p2[3],
                                            const float q2[3],
                                            float *s_out,
                                            float *t_out,
                                            float c1[3],
                                            float c2[3],
                                            float *dist_out)
{
    const float eps = 0.000001f;
    float d1[3] = { q1[0] - p1[0], q1[1] - p1[1], q1[2] - p1[2] };
    float d2[3] = { q2[0] - p2[0], q2[1] - p2[1], q2[2] - p2[2] };
    float r[3] = { p1[0] - p2[0], p1[1] - p2[1], p1[2] - p2[2] };
    float a = d1[0] * d1[0] + d1[1] * d1[1] + d1[2] * d1[2];
    float e = d2[0] * d2[0] + d2[1] * d2[1] + d2[2] * d2[2];
    float f = d2[0] * r[0] + d2[1] * r[1] + d2[2] * r[2];
    float s = 0.0f;
    float t = 0.0f;
    float dx, dy, dz;

    if (a <= eps && e <= eps) {
        s = 0.0f;
        t = 0.0f;
    } else if (a <= eps) {
        s = 0.0f;
        t = physx_clampf(f / e, 0.0f, 1.0f);
    } else {
        float c = d1[0] * r[0] + d1[1] * r[1] + d1[2] * r[2];
        if (e <= eps) {
            t = 0.0f;
            s = physx_clampf(-c / a, 0.0f, 1.0f);
        } else {
            float b = d1[0] * d2[0] + d1[1] * d2[1] + d1[2] * d2[2];
            float denom = a * e - b * b;
            if (denom != 0.0f) {
                s = physx_clampf((b * f - c * e) / denom, 0.0f, 1.0f);
            } else {
                s = 0.0f;
            }
            t = (b * s + f) / e;
            if (t < 0.0f) {
                t = 0.0f;
                s = physx_clampf(-c / a, 0.0f, 1.0f);
            } else if (t > 1.0f) {
                t = 1.0f;
                s = physx_clampf((b - c) / a, 0.0f, 1.0f);
            }
        }
    }

    c1[0] = p1[0] + d1[0] * s;
    c1[1] = p1[1] + d1[1] * s;
    c1[2] = p1[2] + d1[2] * s;
    c2[0] = p2[0] + d2[0] * t;
    c2[1] = p2[1] + d2[1] * t;
    c2[2] = p2[2] + d2[2] * t;
    dx = c1[0] - c2[0];
    dy = c1[1] - c2[1];
    dz = c1[2] - c2[2];
    if (s_out) *s_out = s;
    if (t_out) *t_out = t;
    if (dist_out) *dist_out = (float)sqrt((double)(dx * dx + dy * dy + dz * dz));
}

static int body_chain_collider_pair_margin(const float start[3],
                                           const float end[3],
                                           float chain_len_at_start,
                                           float min_chain_distance,
                                           const float pair_a[3],
                                           const float pair_b[3],
                                           float radius,
                                           float *chain_t,
                                           float *pair_t,
                                           float closest_chain[3],
                                           float closest_pair[3],
                                           float *dist,
                                           float *margin,
                                           float *closest_chain_distance)
{
    float seg[3] = { end[0] - start[0], end[1] - start[1], end[2] - start[2] };
    float seg_len_sq = seg[0] * seg[0] + seg[1] * seg[1] + seg[2] * seg[2];
    float seg_len;
    float segment_end_distance;
    float active_start[3];
    float active_start_distance = chain_len_at_start;
    float active_len;
    float local_s = 0.0f;
    float local_t = 0.0f;
    if (seg_len_sq < 0.000001f) return 0;
    seg_len = (float)sqrt((double)seg_len_sq);
    segment_end_distance = chain_len_at_start + seg_len;
    if (min_chain_distance >= segment_end_distance) return 0;

    active_start[0] = start[0];
    active_start[1] = start[1];
    active_start[2] = start[2];
    if (min_chain_distance > chain_len_at_start) {
        float trim_t = (min_chain_distance - chain_len_at_start) / seg_len;
        trim_t = physx_clampf(trim_t, 0.0f, 1.0f);
        active_start[0] = start[0] + seg[0] * trim_t;
        active_start[1] = start[1] + seg[1] * trim_t;
        active_start[2] = start[2] + seg[2] * trim_t;
        active_start_distance = chain_len_at_start + seg_len * trim_t;
    }
    active_len = segment_end_distance - active_start_distance;
    if (active_len <= 0.000001f) return 0;

    body_chain_closest_segment_pair(active_start, end, pair_a, pair_b,
                                    &local_s, &local_t,
                                    closest_chain, closest_pair, dist);
    if (margin) *margin = *dist - radius;
    if (chain_t) {
        *chain_t = physx_clampf(
            ((active_start_distance - chain_len_at_start) +
             active_len * local_s) / seg_len,
            0.0f, 1.0f);
    }
    if (pair_t) *pair_t = local_t;
    if (closest_chain_distance) {
        *closest_chain_distance = active_start_distance + active_len * local_s;
    }
    return 1;
}

static int body_chain_collider_local_point_in_chain_space(
    const body_chain_collider_person_state_t *chain_frame,
    const body_chain_collider_person_state_t *body_state,
    const float local[3],
    float out[3]);

static int body_chain_collider_node_in_chain_space(
    const body_chain_collider_person_state_t *chain_frame,
    const body_chain_collider_person_state_t *body_state,
    int node_index,
    float out[3])
{
    if (!chain_frame || !body_state || !out ||
        node_index < 0 || node_index >= BODY_COLLIDER_NODE_COUNT ||
        !body_state->valid[node_index]) {
        return 0;
    }
    return body_chain_collider_local_point_in_chain_space(
        chain_frame, body_state, body_state->local_position[node_index], out);
}

static int body_chain_collider_local_point_in_chain_space(
    const body_chain_collider_person_state_t *chain_frame,
    const body_chain_collider_person_state_t *body_state,
    const float local[3],
    float out[3])
{
    float view[3];
    float delta[3];
    const float *root;
    if (!chain_frame || !body_state || !local || !out ||
        !chain_frame->basis_valid ||
        !chain_frame->valid[BODY_COLLIDER_ROOT]) {
        return 0;
    }
    if (chain_frame == body_state) {
        out[0] = local[0];
        out[1] = local[1];
        out[2] = local[2];
        return 1;
    }
    if (!body_chain_collider_local_to_view(body_state, local, view)) {
        return 0;
    }
    root = chain_frame->view_position[BODY_COLLIDER_ROOT];
    delta[0] = view[0] - root[0];
    delta[1] = view[1] - root[1];
    delta[2] = view[2] - root[2];
    return body_collider_view_delta_to_local(
        delta, chain_frame->basis_h, chain_frame->basis_v,
        chain_frame->basis_s, out);
}

static int body_chain_collider_local_vector_in_chain_space(
    const body_chain_collider_person_state_t *chain_frame,
    const body_chain_collider_person_state_t *body_state,
    const float local_vector[3],
    float out[3])
{
    float origin_view[3];
    float end_view[3];
    float delta[3];
    float origin_local[3] = { 0.0f, 0.0f, 0.0f };
    if (!chain_frame || !body_state || !local_vector || !out ||
        !chain_frame->basis_valid ||
        !chain_frame->valid[BODY_COLLIDER_ROOT]) {
        return 0;
    }
    if (chain_frame == body_state) {
        out[0] = local_vector[0];
        out[1] = local_vector[1];
        out[2] = local_vector[2];
        return 1;
    }
    if (!body_chain_collider_local_to_view(body_state, origin_local,
                                          origin_view) ||
        !body_chain_collider_local_to_view(body_state, local_vector,
                                          end_view)) {
        return 0;
    }
    delta[0] = end_view[0] - origin_view[0];
    delta[1] = end_view[1] - origin_view[1];
    delta[2] = end_view[2] - origin_view[2];
    return body_collider_view_delta_to_local(
        delta, chain_frame->basis_h, chain_frame->basis_v,
        chain_frame->basis_s, out);
}

static int body_chain_normalize_axis(float axis[3])
{
    float len;
    if (!axis) return 0;
    len = physx_vec3_len(axis);
    if (len <= 0.000001f || !sane_probe_float(len)) return 0;
    axis[0] /= len;
    axis[1] /= len;
    axis[2] /= len;
    return 1;
}

static int body_chain_passive_chain_points_in_frame(
    const body_chain_collider_person_state_t *chain_frame,
    const body_chain_collider_person_state_t *body_state,
    float points[4][3],
    DWORD now)
{
    int i;
    if (!chain_frame || !body_state || !points ||
        !body_state->chain_points_ready ||
        !body_state->chain_points_update_tick ||
        now - body_state->chain_points_update_tick >
            (body_state->chain_points_fresh ?
             BODY_CHAIN_ENGINE_POINT_STALE_MS :
             BODY_CHAIN_COLLISION_POINT_HOLD_MS)) {
        return 0;
    }
    for (i = 0; i < 4; i++) {
        if (!body_state->chain_point_valid[i] ||
            !body_chain_collider_local_point_in_chain_space(
                chain_frame, body_state,
                body_state->chain_local_point[i], points[i])) {
            return 0;
        }
    }
    return 1;
}

static int body_chain_collision_points_local(
    const body_chain_collider_person_state_t *collider_state,
    const body_chain_person_state_t *chain_state,
    float points[4][3],
    int *engine_points,
    DWORD now)
{
    int i;
    int live_points_fresh = 0;

    if (!points) return 0;
    memset(points, 0, sizeof(float) * 4 * 3);
    if (engine_points) *engine_points = 0;

    if (collider_state && collider_state->chain_points_ready) {
        live_points_fresh =
            !now ||
            (collider_state->chain_points_update_tick &&
             now - collider_state->chain_points_update_tick <=
                (collider_state->chain_points_fresh ?
                 BODY_CHAIN_ENGINE_POINT_STALE_MS :
                 BODY_CHAIN_COLLISION_POINT_HOLD_MS));
    }

    if (collider_state && collider_state->chain_points_ready &&
        live_points_fresh) {
        for (i = 0; i < 4; i++) {
            if (!collider_state->chain_point_valid[i]) break;
            points[i][0] = collider_state->chain_local_point[i][0];
            points[i][1] = collider_state->chain_local_point[i][1];
            points[i][2] = collider_state->chain_local_point[i][2];
        }
        if (i == 4) {
            if (engine_points) {
                *engine_points = collider_state->chain_points_fresh ? 1 : 2;
            }
            return 1;
        }
    }

    if (!body_chain_simulated_points_local(chain_state,
                                           &body_chain_physics_cfg,
                                           NULL, points)) {
        return 0;
    }
    return body_chain_engine_points_plausible(points);
}

static int body_chain_live_points_with_delta(
    const float live_points[4][3],
    const float angle_delta[3][2],
    float out[4][3]);
static int body_chain_live_points_with_delta_cfg(
    const body_chain_physics_config_t *cfg,
    const float live_points[4][3],
    const float angle_delta[3][2],
    float out[4][3]);

/* Active-chain kinematics need the real rotation pivots and terminal point.
   The passive spheres are offset segment midpoints, not joints: treating them
   as pivots both changes the lever arms and invents a segment beyond the tip.
   Use the same validated/held sample as the passive proxies; no synthetic
   angle-only shape is substituted when that sample expires. */
static int body_chain_testicle_collision_points_local(
    const body_chain_collider_person_state_t *collider_state,
    const body_chain_person_state_t *chain_state,
    float points[4][3], int *engine_points, DWORD now)
{
    int i,a;
    if (engine_points) *engine_points=0;
    if (!points) return 0;
    memset(points,0,sizeof(float)*12);
    if (!collider_state || !chain_state || !collider_state->testicle_points_ready ||
        !collider_state->testicle_points_update_tick ||
        now-collider_state->testicle_points_update_tick>BODY_CHAIN_ENGINE_POINT_STALE_MS) return 0;
    for(i=0;i<3;i++) for(a=0;a<3;a++) {
        float value=collider_state->testicle_joint_position[i][a];
        if (!sane_probe_float(value)) return 0;
        points[i][a]=value;
    }
    /* Two physical segments; the unused fourth point has no extension. */
    memcpy(points[3],points[2],sizeof(float)*3);
    if(engine_points) *engine_points=1;
    return 1;
}


static int body_contact_candidate_points(const body_chain_person_state_t *state,
    const body_chain_physics_config_t *cfg, DWORD now, float points[4][3]);

static int body_chain_penis_cross_sample(
    const body_chain_collider_person_state_t *collider,
    const body_chain_person_state_t *state,float points[4][3],DWORD now)
{
    int engine=0;
    if (state->collision_step_engine_points &&
        body_contact_candidate_points(state,&body_chain_physics_cfg,now,points)) return 1;
    /* Independent update intervals can leave the other chain's candidate one
       tick old. Keep its real live/held shape. Configured fallback link lengths
       are not a substitute for the visible chain during mutual contact. */
    if (!body_chain_collision_points_local(collider,state,points,&engine,now)) return 0;
    return engine!=0;
}

static int body_chain_active_penis_cross_points_local(int person_index,
                                                      float points[4][3], DWORD now)
{
    body_chain_person_state_t *state;
    if (!points || person_index < 0 || person_index >= 4 ||
        !body_chain_physics_cfg.enabled ||
        !body_chain_physics_cfg.enabled_person[person_index] ||
        !body_chain_collider_cfg.penis_collision_enabled) {
        return 0;
    }
    state = InterlockedCompareExchange(
                &body_chain_poseeditor_mode_active, 0, 0)
        ? &body_chain_person_states[person_index]
        : &runtime_body_chain_person_states[person_index];
    if (!state->initialized || !state->active_logged) {
        return 0;
    }
    return body_chain_penis_cross_sample(&body_chain_collider_states[person_index],state,points,now);
}

static int body_chain_active_testicle_cross_points_local(int person_index,
                                                        float points[4][3],
                                                        DWORD now)
{
    body_chain_person_state_t *state;
    body_chain_collider_person_state_t *collider_state;
    if (!points || person_index < 0 || person_index >= 4 ||
        !testicle_physics_cfg.enabled ||
        !testicle_physics_cfg.enabled_person[person_index] ||
        !body_chain_collider_cfg.testicle_collision_enabled) {
        return 0;
    }
    state = InterlockedCompareExchange(
                &body_chain_poseeditor_mode_active, 0, 0)
        ? &testicle_physics_states[person_index]
        : &runtime_testicle_physics_states[person_index];
    if (!state->initialized || !state->active_logged) {
        return 0;
    }
    collider_state = &body_chain_collider_states[person_index];
    if (body_contact_candidate_points(state,&testicle_physics_cfg,now,points)) return 1;
    return body_chain_testicle_collision_points_local(
        collider_state, state, points, NULL, now);
}

static void body_chain_collider_limb_pair_nodes(int pair_index,
                                                int *start_node,
                                                int *end_node)
{
    if (!start_node || !end_node) return;
    if (pair_index == BODY_COLLIDER_LIMB_PAIR_R_HIP_THIGH) {
        *start_node = BODY_COLLIDER_HIP_R;
        *end_node = BODY_COLLIDER_THIGH_R;
    } else if (pair_index == BODY_COLLIDER_LIMB_PAIR_R_THIGH_KNEE) {
        *start_node = BODY_COLLIDER_THIGH_R;
        *end_node = BODY_COLLIDER_KNEE_R;
    } else if (pair_index == BODY_COLLIDER_LIMB_PAIR_L_THIGH_KNEE) {
        *start_node = BODY_COLLIDER_THIGH_L;
        *end_node = BODY_COLLIDER_KNEE_L;
    } else {
        *start_node = BODY_COLLIDER_HIP_L;
        *end_node = BODY_COLLIDER_THIGH_L;
    }
}

static const char *body_chain_collider_limb_pair_name(int pair_index)
{
    switch (pair_index) {
    case BODY_COLLIDER_LIMB_PAIR_L_THIGH_KNEE:
        return "thigh_L_to_knee_L";
    case BODY_COLLIDER_LIMB_PAIR_R_HIP_THIGH:
        return "hip_R_to_thigh_R";
    case BODY_COLLIDER_LIMB_PAIR_R_THIGH_KNEE:
        return "thigh_R_to_knee_R";
    default:
        return "hip_L_to_thigh_L";
    }
}

static float body_chain_collider_limb_pair_radius(int pair_index,
                                                  float pair_t)
{
    int start_node;
    int end_node;
    float r0;
    float r1;
    body_chain_collider_limb_pair_nodes(pair_index, &start_node, &end_node);
    pair_t = physx_clampf(pair_t, 0.0f, 1.0f);
    r0 = body_chain_collider_effective_radius_for_node(start_node);
    r1 = body_chain_collider_effective_radius_for_node(end_node);
    return r0 + (r1 - r0) * pair_t;
}

static int body_chain_collider_limb_pair_margin(
    const float start[3],
    const float end[3],
    float chain_len_at_start,
    float min_chain_distance,
    body_chain_collider_person_state_t *state,
    int pair_index,
    float *chain_t,
    float *pair_t,
    float closest_chain[3],
    float closest_pair[3],
    float *dist,
    float *radius,
    float *margin,
    float *closest_chain_distance)
{
    int start_node;
    int end_node;
    float raw_pair_t = 0.0f;
    float pair_radius;
    if (!state || pair_index < 0 ||
        pair_index >= BODY_COLLIDER_LIMB_PAIR_COUNT) {
        return 0;
    }
    body_chain_collider_limb_pair_nodes(pair_index, &start_node, &end_node);
    if (!state->valid[start_node] || !state->valid[end_node]) return 0;
    if (!body_chain_collider_pair_margin(
            start, end, chain_len_at_start, min_chain_distance,
            state->local_position[start_node],
            state->local_position[end_node],
            0.0f, chain_t, &raw_pair_t, closest_chain,
            closest_pair, dist, margin, closest_chain_distance)) {
        return 0;
    }
    pair_radius = body_chain_collider_limb_pair_radius(pair_index,
                                                       raw_pair_t);
    if (pair_t) *pair_t = raw_pair_t;
    if (radius) *radius = pair_radius;
    if (margin && dist) *margin = *dist - pair_radius;
    return 1;
}

static int body_chain_collider_limb_pair_margin_in_frame(
    const float start[3],
    const float end[3],
    float chain_len_at_start,
    float min_chain_distance,
    const body_chain_collider_person_state_t *chain_frame,
    const body_chain_collider_person_state_t *body_state,
    int pair_index,
    float *chain_t,
    float *pair_t,
    float closest_chain[3],
    float closest_pair[3],
    float *dist,
    float *radius,
    float *margin,
    float *closest_chain_distance,
    float pair_a[3],
    float pair_b[3])
{
    int start_node;
    int end_node;
    float raw_pair_t = 0.0f;
    float pair_radius;
    if (!chain_frame || !body_state || !pair_a || !pair_b ||
        pair_index < 0 || pair_index >= BODY_COLLIDER_LIMB_PAIR_COUNT) {
        return 0;
    }
    body_chain_collider_limb_pair_nodes(pair_index, &start_node, &end_node);
    if (!body_state->valid[start_node] || !body_state->valid[end_node]) {
        return 0;
    }
    if (!body_chain_collider_node_in_chain_space(
            chain_frame, body_state, start_node, pair_a) ||
        !body_chain_collider_node_in_chain_space(
            chain_frame, body_state, end_node, pair_b)) {
        return 0;
    }
    if (!body_chain_collider_pair_margin(
            start, end, chain_len_at_start, min_chain_distance,
            pair_a, pair_b, 0.0f, chain_t, &raw_pair_t,
            closest_chain, closest_pair, dist, margin,
            closest_chain_distance)) {
        return 0;
    }
    pair_radius = body_chain_collider_limb_pair_radius(pair_index,
                                                       raw_pair_t);
    if (pair_t) *pair_t = raw_pair_t;
    if (radius) *radius = pair_radius;
    if (margin && dist) *margin = *dist - pair_radius;
    return 1;
}

static int body_chain_collider_stomach_pair_margin_in_frame(
    const float start[3],
    const float end[3],
    float chain_len_at_start,
    float min_chain_distance,
    const body_chain_collider_person_state_t *chain_frame,
    const body_chain_collider_person_state_t *body_state,
    float *chain_t,
    float *pair_t,
    float closest_chain[3],
    float closest_pair[3],
    float *dist,
    float *radius,
    float *margin,
    float *closest_chain_distance,
    float pair_a[3],
    float pair_b[3])
{
    static const float local_h[3] = { 1.0f, 0.0f, 0.0f };
    static const float local_v[3] = { 0.0f, 1.0f, 0.0f };
    static const float local_s[3] = { 0.0f, 0.0f, 1.0f };
    float raw_pair_t = 0.0f;
    float axis_h[3];
    float axis_v[3];
    float axis_s[3];
    float normal[3];
    float radius0[3];
    float radius1[3];
    float radius_at[3];
    float effective_radius;
    float normal_len;
    float nh, nv, ns;
    int axis;

    if (!chain_frame || !body_state || !pair_a || !pair_b ||
        !body_state->stomach_points_ready ||
        !body_state->valid[BODY_COLLIDER_STOMACH_01] ||
        !body_state->valid[BODY_COLLIDER_STOMACH_02]) {
        return 0;
    }
    if (!body_chain_collider_node_in_chain_space(
            chain_frame, body_state, BODY_COLLIDER_STOMACH_01, pair_a) ||
        !body_chain_collider_node_in_chain_space(
            chain_frame, body_state, BODY_COLLIDER_STOMACH_02, pair_b)) {
        return 0;
    }
    if (!body_chain_collider_pair_margin(
            start, end, chain_len_at_start, min_chain_distance,
            pair_a, pair_b, 0.0f, chain_t, &raw_pair_t,
            closest_chain, closest_pair, dist, margin,
            closest_chain_distance)) {
        return 0;
    }

    if (!dist || !margin) return 0;
    normal[0] = closest_chain[0] - closest_pair[0];
    normal[1] = closest_chain[1] - closest_pair[1];
    normal[2] = closest_chain[2] - closest_pair[2];
    normal_len = physx_vec3_len(normal);
    if (normal_len > 0.000001f) {
        normal[0] /= normal_len;
        normal[1] /= normal_len;
        normal[2] /= normal_len;
    } else {
        normal[0] = 0.0f;
        normal[1] = 1.0f;
        normal[2] = 0.0f;
    }

    if (!body_chain_collider_local_vector_in_chain_space(
            chain_frame, body_state, local_h, axis_h) ||
        !body_chain_collider_local_vector_in_chain_space(
            chain_frame, body_state, local_v, axis_v) ||
        !body_chain_collider_local_vector_in_chain_space(
            chain_frame, body_state, local_s, axis_s) ||
        !body_chain_normalize_axis(axis_h) ||
        !body_chain_normalize_axis(axis_v) ||
        !body_chain_normalize_axis(axis_s)) {
        axis_h[0] = 1.0f; axis_h[1] = 0.0f; axis_h[2] = 0.0f;
        axis_v[0] = 0.0f; axis_v[1] = 1.0f; axis_v[2] = 0.0f;
        axis_s[0] = 0.0f; axis_s[1] = 0.0f; axis_s[2] = 1.0f;
    }

    body_chain_collider_effective_stomach_radius_axes(0, radius0);
    body_chain_collider_effective_stomach_radius_axes(1, radius1);
    raw_pair_t = physx_clampf(raw_pair_t, 0.0f, 1.0f);
    for (axis = 0; axis < 3; axis++) {
        radius_at[axis] =
            radius0[axis] + (radius1[axis] - radius0[axis]) * raw_pair_t;
    }
    nh = vec3_dot(normal, axis_h);
    nv = vec3_dot(normal, axis_v);
    ns = vec3_dot(normal, axis_s);
    effective_radius =
        (float)sqrt((double)(
            radius_at[0] * radius_at[0] * nh * nh +
            radius_at[1] * radius_at[1] * nv * nv +
            radius_at[2] * radius_at[2] * ns * ns));
    if (!sane_probe_float(effective_radius) || effective_radius <= 0.0f) {
        effective_radius = radius_at[0];
        if (radius_at[1] > effective_radius) effective_radius = radius_at[1];
        if (radius_at[2] > effective_radius) effective_radius = radius_at[2];
    }

    if (pair_t) *pair_t = raw_pair_t;
    if (radius) *radius = effective_radius;
    *margin = *dist - effective_radius;
    return 1;
}

static int body_chain_simulated_segment_point(
    const body_chain_person_state_t *chain_state,
    int segment_index,
    float segment_t,
    const float angle_delta[3][2],
    float out[3])
{
    float points[4][3];
    float t;
    if (!chain_state || !out || segment_index < 0 || segment_index >= 3) {
        return 0;
    }
    if (!body_chain_simulated_points_local(chain_state,
                                           &body_chain_physics_cfg,
                                           angle_delta, points)) {
        return 0;
    }
    t = physx_clampf(segment_t, 0.0f, 1.0f);
    out[0] = points[segment_index][0] +
             (points[segment_index + 1][0] -
              points[segment_index][0]) * t;
    out[1] = points[segment_index][1] +
             (points[segment_index + 1][1] -
              points[segment_index][1]) * t;
    out[2] = points[segment_index][2] +
             (points[segment_index + 1][2] -
              points[segment_index][2]) * t;
    return sane_probe_float(out[0]) &&
           sane_probe_float(out[1]) &&
           sane_probe_float(out[2]);
}

static int body_chain_live_points_with_delta(
    const float live_points[4][3],
    const float angle_delta[3][2],
    float out[4][3]);

static int body_chain_live_segment_point(
    const float live_points[4][3],
    int segment_index,
    float segment_t,
    const float angle_delta[3][2],
    float out[3])
{
    float points[4][3];
    float t;
    if (!body_chain_live_points_with_delta(live_points, angle_delta,
                                           points) ||
        !out || segment_index < 0 || segment_index >= 3) {
        return 0;
    }
    t = physx_clampf(segment_t, 0.0f, 1.0f);
    out[0] = points[segment_index][0] +
             (points[segment_index + 1][0] -
              points[segment_index][0]) * t;
    out[1] = points[segment_index][1] +
             (points[segment_index + 1][1] -
              points[segment_index][1]) * t;
    out[2] = points[segment_index][2] +
             (points[segment_index + 1][2] -
              points[segment_index][2]) * t;
    return sane_probe_float(out[0]) &&
           sane_probe_float(out[1]) &&
           sane_probe_float(out[2]);
}

static int body_chain_live_points_with_delta_cfg(
    const body_chain_physics_config_t *cfg,
    const float live_points[4][3],
    const float angle_delta[3][2],
    float out[4][3])
{
    int joint;
    int point_index;
    if (!live_points || !out) {
        return 0;
    }
    memcpy(out, live_points, sizeof(float) * 4 * 3);

    if (angle_delta) {
        for (joint = 0; joint < 3; joint++) {
            float rot[3] = { 0.0f, 0.0f, 0.0f };
            int h_axis = cfg ?
                cfg->horizontal_output_axis :
                body_chain_physics_cfg.horizontal_output_axis;
            int v_axis = cfg ?
                cfg->vertical_output_axis :
                body_chain_physics_cfg.vertical_output_axis;
            if (h_axis >= 0 && h_axis < 3) {
                rot[h_axis] += angle_delta[joint][0];
            }
            if (v_axis >= 0 && v_axis < 3) {
                rot[v_axis] += angle_delta[joint][1];
            }
            if (physx_absf(rot[0]) < 0.000001f &&
                physx_absf(rot[1]) < 0.000001f &&
                physx_absf(rot[2]) < 0.000001f) {
                continue;
            }
            for (point_index = joint + 1; point_index < 4; point_index++) {
                float rotated[3];
                body_collider_rotate_point_about_pivot(
                    out[point_index], out[joint], rot, rotated);
                out[point_index][0] = rotated[0];
                out[point_index][1] = rotated[1];
                out[point_index][2] = rotated[2];
            }
        }
    }

    for (point_index = 0; point_index < 4; point_index++) {
        if (!sane_probe_float(out[point_index][0]) ||
            !sane_probe_float(out[point_index][1]) ||
            !sane_probe_float(out[point_index][2])) {
            return 0;
        }
    }
    return 1;
}

static int body_chain_live_points_with_delta(
    const float live_points[4][3],
    const float angle_delta[3][2],
    float out[4][3])
{
    return body_chain_live_points_with_delta_cfg(
        &body_chain_physics_cfg, live_points, angle_delta, out);
}

static int body_chain_apply_contact_constraint_correction(
    const float live_points[4][3],
    int segment_index,
    int first_joint,
    float segment_t,
    const float contact_point[3],
    const float contact_anchor[3],
    const float normal[3],
    float penetration,
    float response_scale,
    float correction[3][2])
{
    const float epsilon_degrees = 0.25f;
    const float solver_softness = 0.000005f;
    float jacobian[3][2] = {
        { 0.0f, 0.0f },
        { 0.0f, 0.0f },
        { 0.0f, 0.0f }
    };
    float proposed[3][2] = {
        { 0.0f, 0.0f },
        { 0.0f, 0.0f },
        { 0.0f, 0.0f }
    };
    float plus_delta[3][2] = {
        { 0.0f, 0.0f },
        { 0.0f, 0.0f },
        { 0.0f, 0.0f }
    };
    float minus_delta[3][2] = {
        { 0.0f, 0.0f },
        { 0.0f, 0.0f },
        { 0.0f, 0.0f }
    };
    float current_point[3];
    float plus_point[3];
    float minus_point[3];
    float current_sep;
    float plus_sep;
    float minus_sep;
    int correction_sign = 0;
    float denominator = 0.0f;
    float gain;
    int joint;
    int axis;

    if (!live_points || !normal || !correction ||
        segment_index < 0 || segment_index >= 3 ||
        first_joint < 0 || first_joint > segment_index ||
        !contact_point || !contact_anchor ||
        penetration <= 0.0f || response_scale <= 0.0f) {
        return 0;
    }

    for (joint = first_joint;
         joint <= segment_index && joint < 3;
         joint++) {
        for (axis = 0; axis < 2; axis++) {
            float plus_delta[3][2] = {
                { 0.0f, 0.0f },
                { 0.0f, 0.0f },
                { 0.0f, 0.0f }
            };
            float minus_delta[3][2] = {
                { 0.0f, 0.0f },
                { 0.0f, 0.0f },
                { 0.0f, 0.0f }
            };
            float plus[3];
            float minus[3];
            float derivative[3];
            float j;

            plus_delta[joint][axis] = epsilon_degrees;
            minus_delta[joint][axis] = -epsilon_degrees;
            if (!body_chain_live_segment_point(
                    live_points, segment_index, segment_t,
                    plus_delta, plus) ||
                !body_chain_live_segment_point(
                    live_points, segment_index, segment_t,
                    minus_delta, minus)) {
                continue;
            }

            derivative[0] = (plus[0] - minus[0]) /
                            (epsilon_degrees * 2.0f);
            derivative[1] = (plus[1] - minus[1]) /
                            (epsilon_degrees * 2.0f);
            derivative[2] = (plus[2] - minus[2]) /
                            (epsilon_degrees * 2.0f);
            j = vec3_dot(derivative, normal);
            if (!sane_probe_float(j)) continue;
            jacobian[joint][axis] = j;
            denominator += j * j;
        }
    }

    if (denominator < 0.00000001f) return 0;

    gain = physx_clampf(body_chain_collider_cfg.response_strength *
                        response_scale, 0.0f, 4.0f);
    if (gain <= 0.0f) return 0;

    for (joint = first_joint;
         joint <= segment_index && joint < 3;
         joint++) {
        for (axis = 0; axis < 2; axis++) {
            proposed[joint][axis] =
                penetration * gain * jacobian[joint][axis] /
                (denominator + solver_softness);
            plus_delta[joint][axis] = proposed[joint][axis];
            minus_delta[joint][axis] = -proposed[joint][axis];
        }
    }

    if (!body_chain_live_segment_point(
            live_points, segment_index, segment_t,
            plus_delta, plus_point) ||
        !body_chain_live_segment_point(
            live_points, segment_index, segment_t,
            minus_delta, minus_point)) {
        return 0;
    }

    current_point[0] = contact_point[0] - contact_anchor[0];
    current_point[1] = contact_point[1] - contact_anchor[1];
    current_point[2] = contact_point[2] - contact_anchor[2];
    plus_point[0] -= contact_anchor[0];
    plus_point[1] -= contact_anchor[1];
    plus_point[2] -= contact_anchor[2];
    minus_point[0] -= contact_anchor[0];
    minus_point[1] -= contact_anchor[1];
    minus_point[2] -= contact_anchor[2];

    current_sep = vec3_dot(current_point, normal);
    plus_sep = vec3_dot(plus_point, normal);
    minus_sep = vec3_dot(minus_point, normal);
    if (!sane_probe_float(current_sep) ||
        !sane_probe_float(plus_sep) ||
        !sane_probe_float(minus_sep)) {
        return 0;
    }

    if (plus_sep > current_sep + 0.000001f &&
        plus_sep >= minus_sep) {
        correction_sign = 1;
    } else if (minus_sep > current_sep + 0.000001f) {
        correction_sign = -1;
    } else {
        return 0;
    }

    for (joint = first_joint;
         joint <= segment_index && joint < 3;
         joint++) {
        for (axis = 0; axis < 2; axis++) {
            correction[joint][axis] +=
                proposed[joint][axis] * (float)correction_sign;
        }
    }

    return 1;
}

static int body_chain_testicle_self_upper_edge_node(int node)
{
    switch (node) {
    case BODY_COLLIDER_STOMACH_02:
    case BODY_COLLIDER_STOMACH_03:
    case BODY_COLLIDER_STOMACH_04:
    case BODY_COLLIDER_NECK_01:
    case BODY_COLLIDER_HEAD_02:
    case BODY_COLLIDER_BREAST_L:
    case BODY_COLLIDER_BREAST_R:
        return 1;
    default:
        return 0;
    }
}

static const char *body_chain_collider_node_diagnostic_name(int node_index)
{
    int i;
    for (i = 0; i < BODY_COLLIDER_DIRECT_NODE_COUNT; i++) {
        const body_collider_direct_node_def_t *def =
            &body_collider_direct_nodes[i];
        if (def->node_index == node_index && def->node_name) {
            return def->node_name;
        }
    }
    return body_chain_collider_node_label(node_index);
}

typedef struct body_chain_collider_projection_cache_t {
    DWORD simulation_serial;
    float point[BODY_COLLIDER_NODE_COUNT][3];
    unsigned char valid[BODY_COLLIDER_NODE_COUNT];
    unsigned char active_node[BODY_COLLIDER_NODE_COUNT];
    int active_node_count;
} body_chain_collider_projection_cache_t;

static body_chain_collider_projection_cache_t
    body_chain_collider_projection_cache[4][4];
static unsigned char
    body_chain_hand_edge_index[BODY_COLLIDER_EXTRA_EDGE_COUNT];
static int body_chain_hand_edge_count = -1;

static void body_chain_prepare_hand_edge_index(void)
{
    int edge_index;
    if (body_chain_hand_edge_count >= 0) return;
    body_chain_hand_edge_count = 0;
    for (edge_index = 0;
         edge_index < BODY_COLLIDER_EXTRA_EDGE_COUNT;
         edge_index++) {
        const body_collider_extra_edge_def_t *edge =
            &body_collider_extra_edges[edge_index];
        if (edge &&
            body_chain_collider_node_is_hand(edge->start_node) &&
            body_chain_collider_node_is_hand(edge->end_node)) {
            body_chain_hand_edge_index[body_chain_hand_edge_count++] =
                (unsigned char)edge_index;
        }
    }
}

static const body_chain_collider_projection_cache_t *
body_chain_get_collider_projection_cache(
    int owner_person_index,
    int collider_person_index,
    const body_chain_collider_person_state_t *owner_state,
    const body_chain_collider_person_state_t *collider_state)
{
    body_chain_collider_projection_cache_t *cache;
    int node;
    if (owner_person_index < 0 || owner_person_index >= 4 ||
        collider_person_index < 0 || collider_person_index >= 4 ||
        !owner_state || !collider_state) {
        return NULL;
    }
    cache = &body_chain_collider_projection_cache
        [owner_person_index][collider_person_index];
    if (cache->simulation_serial == physx_simulation_serial) {
        return cache;
    }
    cache->simulation_serial = physx_simulation_serial;
    cache->active_node_count = 0;
    memset(cache->valid, 0, sizeof(cache->valid));
    for (node = 0; node < BODY_COLLIDER_NODE_COUNT; node++) {
        int mask = collider_state->active_scope_mask;
        int selected = 0;
        if (mask & BODY_CHAIN_COLLIDER_GROUP_FULL_BODY) {
            selected = 1;
        } else if (node <= BODY_COLLIDER_TESTICLES_MID &&
                   (mask & (BODY_CHAIN_COLLIDER_GROUP_GENITALS |
                            BODY_CHAIN_COLLIDER_GROUP_PELVIS))) {
            selected = 1;
        } else if ((mask & BODY_CHAIN_COLLIDER_GROUP_HANDS) &&
                   body_chain_collider_node_is_hand(node)) {
            selected = 1;
        } else if ((mask & BODY_CHAIN_COLLIDER_GROUP_BREAST_SOURCE) &&
                   (node == BODY_COLLIDER_BREAST_L ||
                    node == BODY_COLLIDER_BREAST_R)) {
            selected = 1;
        } else if ((mask & BODY_CHAIN_COLLIDER_GROUP_BUTT_SOURCE) &&
                   (node == BODY_COLLIDER_BUTT_L ||
                    node == BODY_COLLIDER_BUTT_R)) {
            selected = 1;
        }
        if (!selected) {
            continue;
        }
        if (!collider_state->valid[node] ||
            !body_chain_collider_node_in_chain_space(
                owner_state, collider_state, node, cache->point[node])) {
            continue;
        }
        cache->valid[node] = 1;
        cache->active_node[cache->active_node_count++] =
            (unsigned char)node;
    }
    return cache;
}

static int paired_bone_physics_collider_node_in_scope(int node, int scope)
{
    int scope_mask = body_chain_collision_scope_collider_mask(scope);
    if (node < 0 || node >= BODY_COLLIDER_NODE_COUNT) return 0;
    if (scope_mask & BODY_CHAIN_COLLIDER_GROUP_FULL_BODY) return 1;
    if (body_chain_collider_node_is_hand(node)) {
        return (scope_mask & BODY_CHAIN_COLLIDER_GROUP_HANDS) != 0;
    }
    if (node <= BODY_COLLIDER_TESTICLES_MID) {
        return (scope_mask & (BODY_CHAIN_COLLIDER_GROUP_GENITALS |
                              BODY_CHAIN_COLLIDER_GROUP_PELVIS)) != 0;
    }
    return 0;
}



static int breasts_physics_owner_attachment_node(int node)
{
    switch (node) {
    case BODY_COLLIDER_ROOT:
    case BODY_COLLIDER_STOMACH_01:
    case BODY_COLLIDER_STOMACH_02:
    case BODY_COLLIDER_STOMACH_03:
    case BODY_COLLIDER_STOMACH_04:
    case BODY_COLLIDER_BREAST_L:
    case BODY_COLLIDER_BREAST_R:
    case BODY_COLLIDER_CLAVICLE_L:
    case BODY_COLLIDER_CLAVICLE_R:
    case BODY_COLLIDER_SHOULDER_L:
    case BODY_COLLIDER_SHOULDER_R:
        return 1;
    default:
        return 0;
    }
}

static int butt_physics_owner_attachment_node(int node)
{
    switch (node) {
    case BODY_COLLIDER_ROOT:
    case BODY_COLLIDER_STOMACH_01:
    case BODY_COLLIDER_STOMACH_02:
    case BODY_COLLIDER_STOMACH_03:
    case BODY_COLLIDER_STOMACH_04:
    case BODY_COLLIDER_HIP_L:
    case BODY_COLLIDER_HIP_R:
    case BODY_COLLIDER_THIGH_L:
    case BODY_COLLIDER_THIGH_R:
    case BODY_COLLIDER_TESTICLES_01:
    case BODY_COLLIDER_TESTICLES_02:
    case BODY_COLLIDER_TESTICLES_MID:
    case BODY_COLLIDER_BUTT_L:
    case BODY_COLLIDER_BUTT_R:
        return 1;
    default:
        return 0;
    }
}

#include "physx_body_contact.c"

static void body_chain_compute_collider_projection(int person_index,
                                                   body_chain_person_state_t *chain_state,
                                                   float correction[3][2],
                                                   DWORD now,
                                                   int emit_log,
                                                   float *max_penetration_out,
                                                   float *sweep_radius_out,
                                                   float *collider_motion_out,
                                                   int collision_target)
{
    static const char *passive_chain_names[3] = {
        "passive_chain_joint01_to_02",
        "passive_chain_joint02_to_03",
        "passive_chain_joint03_to_end"
    };
    static const char *active_penis_chain_names[3] = {
        "active_penis_joint01_to_02",
        "active_penis_joint02_to_03",
        "active_penis_joint03_to_end"
    };
    static const char *active_testicle_chain_names[2] = {
        "active_testicle_joint01_to_02",
        "active_testicle_joint02_to_end"
    };
    body_chain_collider_person_state_t *chain_collider_state;
    body_chain_collider_person_state_t *collider_state;
    float endpoint[3][3];
    float chain_points[4][3];
    float working_points[4][3];
    float cumulative_len[3];
    float projected_h = 0.0f;
    float projected_v = 0.0f;
    float max_penetration = 0.0f;
    float sweep_radius = 0.0f;
    float largest_sweep_radius = 0.0f;
    float largest_collider_motion = 0.0f;
    float nearest_margin = 99999.0f;
    float nearest_dist = 0.0f;
    float nearest_radius = 0.0f;
    float nearest_t = 0.0f;
    float nearest_closest[3] = { 0.0f, 0.0f, 0.0f };
    float chain_sweep_move = 0.0f;
    const char *nearest_label = NULL;
    int nearest_segment = -1;
    int nearest_node = -1;
    int nearest_collider_person = -1;
    body_chain_contact_t contacts[BODY_CHAIN_MAX_CONTACTS];
    int contact_count = 0;
    int hit_count = 0;
    int chain_points_from_engine = 0;
    const char *pose_source = "simulated-angle";
    int i, j, node;
    int collider_person_index;
    int passive_segment;
    int solver_iterations;
    int hierarchical_support = 0;
    int target_is_testicle =
        collision_target == BODY_CHAIN_COLLISION_TARGET_TESTICLES;
    const char *target_name = target_is_testicle ? "testicle" : "penis";
    int segment_count = target_is_testicle ? 2 : 3;
    int target_response_enabled;
    int body_response_enabled;
    int room_response_enabled;
    int collision_scope;
    int collision_scope_mask;
    int collision_scope_all_persons;
    int collision_scope_includes_hands;
    int collision_scope_is_full_body;
    float owner_chain_radius;
    float owner_collision_slop;
    const float contact_soft_margin = 0.0020f;
    const float sweep_activation_move = 0.0180f;

    if (max_penetration_out) *max_penetration_out = 0.0f;
    if (collider_motion_out) *collider_motion_out = 0.0f;
    if (chain_state) {
        chain_state->collision_manifold_contacts = 0;
        chain_state->collision_room_contacts = 0;
        if (chain_state->collision_multi_support_grace_ticks > 0) {
            chain_state->collision_multi_support_grace_ticks--;
        }
    }
    body_profile_set_active_person_config(person_index);
    body_response_enabled = target_is_testicle ?
        body_chain_collider_cfg.testicle_collision_enabled :
        body_chain_collider_cfg.penis_collision_enabled;
    room_response_enabled = room_collision_is_enabled() &&
        (target_is_testicle ?
         testicle_physics_cfg.room_collision_enabled :
         body_chain_physics_cfg.room_collision_enabled);
    target_response_enabled = body_response_enabled || room_response_enabled;
    collision_scope = target_is_testicle ?
        testicle_physics_cfg.collision_scope :
        body_chain_physics_cfg.collision_scope;
    if (!body_chain_collision_scope_valid(collision_scope)) {
        collision_scope = BODY_CHAIN_COLLISION_SCOPE_FULL_BODY_ALL;
    }
    collision_scope_mask =
        body_chain_collision_scope_collider_mask(collision_scope);
    collision_scope_all_persons =
        body_chain_collision_scope_all_persons(collision_scope);
    collision_scope_includes_hands =
        body_chain_collision_scope_includes_hands(collision_scope);
    collision_scope_is_full_body =
        body_chain_collision_scope_is_full_body(collision_scope);
    if (collision_scope_includes_hands &&
        !collision_scope_is_full_body) {
        body_chain_prepare_hand_edge_index();
    }
    owner_chain_radius = body_chain_collider_cfg.chain_radius;
    owner_collision_slop = body_chain_collider_cfg.collision_slop;
    if ((!body_chain_collider_cfg.enabled || !body_response_enabled) &&
        !room_response_enabled) {
        return;
    }
    if (!target_response_enabled ||
        !body_chain_collider_cfg.root_local_offsets ||
        !chain_state ||
        !correction ||
        person_index < 0 || person_index >= 4) {
        return;
    }

    chain_collider_state = &body_chain_collider_states[person_index];
    if (!target_is_testicle) {
        int chain_points_stale =
            !chain_collider_state->ready ||
            !chain_collider_state->chain_points_ready ||
            !chain_collider_state->chain_points_update_tick ||
            now - chain_collider_state->chain_points_update_tick >
                (chain_collider_state->chain_points_fresh ?
                 BODY_CHAIN_ENGINE_POINT_STALE_MS :
                 BODY_CHAIN_COLLISION_POINT_HOLD_MS);
        if ((!chain_state->collision_step_valid || chain_state->collision_step_tick != now) &&
            (chain_points_stale || !body_chain_collider_cfg.debug_draw)) {
            update_body_chain_colliders_for_person_scope(
                person_index, now,
                body_chain_collision_scope_collider_mask(collision_scope));
        }
    }
    if (!chain_collider_state->ready) return;
    if (body_contact_candidate_points(chain_state,
            target_is_testicle ? &testicle_physics_cfg : &body_chain_physics_cfg,
            now, chain_points)) {
        chain_points_from_engine=chain_state->collision_step_engine_points;
    } else if (target_is_testicle ?
        !body_chain_testicle_collision_points_local(chain_collider_state,
                                                   chain_state,
                                                   chain_points,
                                                   &chain_points_from_engine,
                                                   now) :
        !body_chain_collision_points_local(chain_collider_state, chain_state,
                                           chain_points,
                                           &chain_points_from_engine,
                                           now)) {
        chain_state->collision_prev_chain_points_ready = 0;
        return;
    }
    pose_source = chain_points_from_engine == 1 ?
        "engine-pivots-fresh" :
        (chain_points_from_engine == 2 ?
         "engine-pivots-held" : "simulated-angle-fallback");
    memcpy(working_points, chain_points, sizeof(working_points));
    for (i = 0; i < segment_count; i++) {
        float dx;
        float dy;
        float dz;
        endpoint[i][0] = chain_points[i + 1][0];
        endpoint[i][1] = chain_points[i + 1][1];
        endpoint[i][2] = chain_points[i + 1][2];
        dx = chain_points[i + 1][0] - chain_points[i][0];
        dy = chain_points[i + 1][1] - chain_points[i][1];
        dz = chain_points[i + 1][2] - chain_points[i][2];
        cumulative_len[i] =
            (i == 0 ? 0.0f : cumulative_len[i - 1]) +
            (float)sqrt((double)(dx * dx + dy * dy + dz * dz));
    }
    for (; i < 3; i++) {
        endpoint[i][0] = chain_points[i + 1][0];
        endpoint[i][1] = chain_points[i + 1][1];
        endpoint[i][2] = chain_points[i + 1][2];
    }
    if (chain_state->collision_prev_chain_points_ready) {
        for (i = 1; i <= segment_count; i++) {
            float move =
                body_collider_distance(chain_points[i],
                                       chain_state->collision_prev_chain_points[i]);
            if (sane_probe_float(move) && move > chain_sweep_move) {
                chain_sweep_move = move;
            }
        }
        if (chain_sweep_move > sweep_activation_move) {
            sweep_radius = physx_clampf(
                (chain_sweep_move - sweep_activation_move) * 0.85f,
                0.0f, 0.065f);
        }
    }
    largest_sweep_radius = sweep_radius;

#define BODY_CHAIN_ACCUMULATE_CONTACT(LABEL_VALUE, NODE_VALUE, SEG_INDEX, SEG_T_VALUE, DIST_VALUE, RADIUS_VALUE, MARGIN_VALUE, CLOSEST_CHAIN_VALUE, CLOSEST_BODY_VALUE, BODY_A_VALUE, BODY_B_VALUE) do { \
        float margin_value = (MARGIN_VALUE); \
        float dist_value = (DIST_VALUE); \
        float radius_value = (RADIUS_VALUE); \
        const float *closest_chain_value = (CLOSEST_CHAIN_VALUE); \
        const float *closest_body_value = (CLOSEST_BODY_VALUE); \
        const float *body_a_value = (BODY_A_VALUE); \
        const float *body_b_value = (BODY_B_VALUE); \
        if (margin_value < nearest_margin) { \
            nearest_margin = margin_value; \
            nearest_dist = dist_value; \
            nearest_radius = radius_value; \
            nearest_t = (SEG_T_VALUE); \
            nearest_closest[0] = closest_chain_value[0]; \
            nearest_closest[1] = closest_chain_value[1]; \
            nearest_closest[2] = closest_chain_value[2]; \
            nearest_segment = (SEG_INDEX); \
            nearest_node = (NODE_VALUE); \
            nearest_label = (LABEL_VALUE); \
            nearest_collider_person = collider_person_index; \
        } \
            if (margin_value < contact_soft_margin) { \
                float normal[3]; \
                float normal_len; \
                float previous_side[3] = { 0.0f, 0.0f, 0.0f }; \
                float previous_side_len = 0.0f; \
                int held_to_previous_side = 0; \
                float penetration; \
                penetration = physx_contact_depth(margin_value, \
                    owner_collision_slop, contact_soft_margin); \
                penetration = physx_clampf(penetration, 0.0f, 0.050f); \
                if ((SEG_INDEX) == 0 && (SEG_T_VALUE) < (target_is_testicle ? 0.45f : 0.080f)) { \
                    penetration = 0.0f; \
                } \
                if (penetration <= 0.000001f) { \
                    break; \
                } \
            normal[0] = closest_chain_value[0] - closest_body_value[0]; \
            normal[1] = closest_chain_value[1] - closest_body_value[1]; \
            normal[2] = closest_chain_value[2] - closest_body_value[2]; \
            normal_len = physx_vec3_len(normal); \
            if (chain_state->collision_prev_chain_points_ready && \
                largest_collider_motion <= 0.0120f) { \
                float previous_contact[3]; \
                float body_axis[3]; \
                float body_axis_len; \
                float along; \
                previous_contact[0] = \
                    chain_state->collision_prev_chain_points[(SEG_INDEX)][0] + \
                    (chain_state->collision_prev_chain_points[(SEG_INDEX) + 1][0] - \
                     chain_state->collision_prev_chain_points[(SEG_INDEX)][0]) * \
                    (SEG_T_VALUE); \
                previous_contact[1] = \
                    chain_state->collision_prev_chain_points[(SEG_INDEX)][1] + \
                    (chain_state->collision_prev_chain_points[(SEG_INDEX) + 1][1] - \
                     chain_state->collision_prev_chain_points[(SEG_INDEX)][1]) * \
                    (SEG_T_VALUE); \
                previous_contact[2] = \
                    chain_state->collision_prev_chain_points[(SEG_INDEX)][2] + \
                    (chain_state->collision_prev_chain_points[(SEG_INDEX) + 1][2] - \
                     chain_state->collision_prev_chain_points[(SEG_INDEX)][2]) * \
                    (SEG_T_VALUE); \
                previous_side[0] = previous_contact[0] - closest_body_value[0]; \
                previous_side[1] = previous_contact[1] - closest_body_value[1]; \
                previous_side[2] = previous_contact[2] - closest_body_value[2]; \
                body_axis[0] = body_b_value[0] - body_a_value[0]; \
                body_axis[1] = body_b_value[1] - body_a_value[1]; \
                body_axis[2] = body_b_value[2] - body_a_value[2]; \
                body_axis_len = physx_vec3_len(body_axis); \
                if (body_axis_len > 0.0001f) { \
                    body_axis[0] /= body_axis_len; \
                    body_axis[1] /= body_axis_len; \
                    body_axis[2] /= body_axis_len; \
                    along = previous_side[0] * body_axis[0] + \
                            previous_side[1] * body_axis[1] + \
                            previous_side[2] * body_axis[2]; \
                    previous_side[0] -= body_axis[0] * along; \
                    previous_side[1] -= body_axis[1] * along; \
                    previous_side[2] -= body_axis[2] * along; \
                } \
                previous_side_len = physx_vec3_len(previous_side); \
            } \
            if (normal_len > 0.0001f) { \
                normal[0] /= normal_len; \
                normal[1] /= normal_len; \
                normal[2] /= normal_len; \
                held_to_previous_side = body_contact_orient_from_history( \
                    normal, previous_side, previous_side_len, radius_value, owner_collision_slop); \
            } else if (previous_side_len > 0.0001f) { \
                normal[0] = previous_side[0] / previous_side_len; \
                normal[1] = previous_side[1] / previous_side_len; \
                normal[2] = previous_side[2] / previous_side_len; \
                normal_len = 1.0f; \
                held_to_previous_side = 1; \
            } else { \
                float chain_axis[3]; \
                float body_axis[3]; \
                float chain_mid[3]; \
                float body_mid[3]; \
                float orient[3]; \
                chain_axis[0] = end[0] - start[0]; \
                chain_axis[1] = end[1] - start[1]; \
                chain_axis[2] = end[2] - start[2]; \
                body_axis[0] = body_b_value[0] - body_a_value[0]; \
                body_axis[1] = body_b_value[1] - body_a_value[1]; \
                body_axis[2] = body_b_value[2] - body_a_value[2]; \
                normal[0] = body_axis[1] * chain_axis[2] - \
                            body_axis[2] * chain_axis[1]; \
                normal[1] = body_axis[2] * chain_axis[0] - \
                            body_axis[0] * chain_axis[2]; \
                normal[2] = body_axis[0] * chain_axis[1] - \
                            body_axis[1] * chain_axis[0]; \
                normal_len = physx_vec3_len(normal); \
                chain_mid[0] = (start[0] + end[0]) * 0.5f; \
                chain_mid[1] = (start[1] + end[1]) * 0.5f; \
                chain_mid[2] = (start[2] + end[2]) * 0.5f; \
                body_mid[0] = (body_a_value[0] + body_b_value[0]) * 0.5f; \
                body_mid[1] = (body_a_value[1] + body_b_value[1]) * 0.5f; \
                body_mid[2] = (body_a_value[2] + body_b_value[2]) * 0.5f; \
                orient[0] = chain_mid[0] - body_mid[0]; \
                orient[1] = chain_mid[1] - body_mid[1]; \
                orient[2] = chain_mid[2] - body_mid[2]; \
                if (normal_len > 0.0001f && \
                    vec3_dot(normal, orient) < 0.0f) { \
                    normal[0] = -normal[0]; \
                    normal[1] = -normal[1]; \
                    normal[2] = -normal[2]; \
                } \
                if (normal_len <= 0.0001f) { \
                    normal[0] = closest_chain_value[0] - body_mid[0]; \
                    normal[1] = closest_chain_value[1] - body_mid[1]; \
                    normal[2] = closest_chain_value[2] - body_mid[2]; \
                    normal_len = physx_vec3_len(normal); \
                } \
                if (normal_len <= 0.0001f) { \
                    normal[0] = chain_mid[0] - body_mid[0]; \
                    normal[1] = chain_mid[1] - body_mid[1]; \
                    normal[2] = chain_mid[2] - body_mid[2]; \
                    normal_len = physx_vec3_len(normal); \
                } \
                if (normal_len > 0.0001f) { \
                    normal[0] /= normal_len; \
                    normal[1] /= normal_len; \
                    normal[2] /= normal_len; \
                } else { \
                    normal[0] = 0.0f; \
                    normal[1] = 1.0f; \
                    normal[2] = 0.0f; \
                } \
            } \
            if (held_to_previous_side) { \
                float signed_separation = \
                    (closest_chain_value[0] - closest_body_value[0]) * normal[0] + \
                    (closest_chain_value[1] - closest_body_value[1]) * normal[1] + \
                    (closest_chain_value[2] - closest_body_value[2]) * normal[2]; \
                if (signed_separation < 0.0f) { \
                    penetration = physx_clampf( \
                        penetration - signed_separation * 2.0f, \
                        0.0f, 0.100f); \
                } \
            } \
            hit_count++; \
            if (penetration > max_penetration) { \
                max_penetration = penetration; \
            } \
            body_chain_store_contact(contacts, &contact_count, \
                BODY_CHAIN_MAX_CONTACTS, (SEG_INDEX), (SEG_T_VALUE), \
                penetration, closest_chain_value, closest_body_value, \
                normal); \
        } \
    } while (0)

    if (body_response_enabled && body_chain_collider_cfg.enabled)
    for (collider_person_index = 0;
         collider_person_index < 4;
         collider_person_index++) {
        float passive_chain_points[4][3];
        const body_chain_collider_projection_cache_t *projection_cache;
        const float (*current_collider_points)[3];
        const unsigned char *current_collider_valid;
        float current_collider_sweep[BODY_COLLIDER_NODE_COUNT];
        int active_node_index;
        int passive_chain_ready = 0;
        if (!collision_scope_all_persons &&
            collider_person_index != person_index) {
            chain_state->collision_prev_collider_ready
                [collider_person_index] = 0;
            continue;
        }
        body_profile_set_active_person_config(collider_person_index);
        collider_state = &body_chain_collider_states[collider_person_index];
        if (!collider_state->ready || !collider_state->basis_valid) {
            chain_state->collision_prev_collider_ready[collider_person_index] = 0;
            continue;
        }
        projection_cache = body_chain_get_collider_projection_cache(
            person_index, collider_person_index,
            chain_collider_state, collider_state);
        if (!projection_cache) {
            chain_state->collision_prev_collider_ready[collider_person_index] = 0;
            continue;
        }
        current_collider_points = projection_cache->point;
        current_collider_valid = projection_cache->valid;
        if (collider_person_index != person_index) {
            passive_chain_ready = body_chain_passive_chain_points_in_frame(
                chain_collider_state, collider_state, passive_chain_points,
                now);
        } else if (target_is_testicle) {
            passive_chain_ready =
                body_chain_active_penis_cross_points_local(
                    person_index, passive_chain_points, now);
        } else if (body_chain_active_testicle_cross_points_local(
                       person_index, passive_chain_points, now)) {
            passive_chain_ready = 2;
        }
        memset(current_collider_sweep, 0, sizeof(current_collider_sweep));
        for (active_node_index = 0;
             active_node_index < projection_cache->active_node_count;
             active_node_index++) {
            node = projection_cache->active_node[active_node_index];
            if (chain_state->collision_prev_collider_ready[collider_person_index] &&
                chain_state->collision_prev_collider_valid[collider_person_index][node]) {
                float move = body_collider_distance(
                    current_collider_points[node],
                    chain_state->collision_prev_collider_points
                        [collider_person_index][node]);
                if (sane_probe_float(move) &&
                    move > largest_collider_motion) {
                    largest_collider_motion = move;
                }
                if (sane_probe_float(move) && move > sweep_activation_move) {
                    float node_sweep =
                        physx_clampf(
                            (move - sweep_activation_move) * 0.85f,
                            0.0f, 0.065f);
                    current_collider_sweep[node] = node_sweep;
                    if (node_sweep > largest_sweep_radius) {
                        largest_sweep_radius = node_sweep;
                    }
                }
            }
        }

        for (i = 0; i < segment_count; i++) {
            float *start = chain_points[i];
            float *end = chain_points[i + 1];
            float chain_len_at_start = i == 0 ? 0.0f : cumulative_len[i - 1];

            if ((collision_scope_mask &
                 BODY_CHAIN_COLLIDER_GROUP_GENITALS) &&
                passive_chain_ready) {
                int same_person_active_penis_cross =
                    collider_person_index == person_index &&
                    target_is_testicle &&
                    passive_chain_ready == 1;
                int same_person_active_testicle_cross =
                    collider_person_index == person_index &&
                    !target_is_testicle &&
                    passive_chain_ready == 2;
                int same_person_cross =
                    same_person_active_penis_cross ||
                    same_person_active_testicle_cross;
                float collider_chain_radius =
                    body_chain_collider_cfg.chain_radius;
                float same_person_cross_radius =
                    owner_chain_radius * 0.75f;
                int passive_segment_count =
                    passive_chain_ready == 2 ? 2 : 3;
                if (same_person_cross_radius >
                    owner_chain_radius) {
                    same_person_cross_radius =
                        owner_chain_radius;
                }
                if (same_person_cross_radius < 0.001f) {
                    same_person_cross_radius =
                        owner_chain_radius;
                }
                for (passive_segment = 0;
                     passive_segment < passive_segment_count;
                     passive_segment++) {
                    float passive_chain_t;
                    float passive_pair_t;
                    float passive_closest_chain[3];
                    float passive_closest[3];
                    float passive_dist;
                    float passive_margin;
                    float passive_chain_distance;
                    float radius = same_person_cross ?
                        (same_person_cross_radius + sweep_radius * 0.35f) :
                        (owner_chain_radius + collider_chain_radius +
                         sweep_radius);
                    if (body_chain_collider_pair_margin(
                            start, end, chain_len_at_start, 0.0f,
                            passive_chain_points[passive_segment],
                            passive_chain_points[passive_segment + 1],
                            radius, &passive_chain_t, &passive_pair_t,
                            passive_closest_chain, passive_closest,
                            &passive_dist, &passive_margin,
                            &passive_chain_distance)) {
                        const char *passive_label =
                            passive_chain_names[passive_segment];
                        if (same_person_active_penis_cross &&
                            passive_segment == 0 &&
                            passive_pair_t < 0.45f) {
                            continue;
                        }
                        if (same_person_active_testicle_cross &&
                            passive_segment == 0 &&
                            passive_pair_t < 0.45f) {
                            continue;
                        }
                        if (same_person_cross &&
                            passive_margin >
                            -owner_collision_slop) {
                            continue;
                        }
                        if (passive_chain_ready == 2) {
                            passive_label =
                                active_testicle_chain_names[passive_segment];
                        } else if (collider_person_index == person_index &&
                                   target_is_testicle) {
                            passive_label =
                                active_penis_chain_names[passive_segment];
                        }
                        BODY_CHAIN_ACCUMULATE_CONTACT(
                            passive_label, -1,
                            i, passive_chain_t, passive_dist, radius,
                            passive_margin, passive_closest_chain,
                            passive_closest,
                            passive_chain_points[passive_segment],
                            passive_chain_points[passive_segment + 1]);
                    }
                }
            }

            if ((collision_scope_mask &
                 BODY_CHAIN_COLLIDER_GROUP_GENITALS) &&
                !(collider_person_index == person_index &&
                  ((target_is_testicle) ||
                   (testicle_physics_cfg.enabled &&
                    testicle_physics_cfg.enabled_person[person_index] &&
                    body_chain_collider_cfg.testicle_collision_enabled))) &&
                collider_state->valid[BODY_COLLIDER_TESTICLES_01] &&
                collider_state->valid[BODY_COLLIDER_TESTICLES_02]) {
                float pair_a[3];
                float pair_b[3];
                float pair_chain_t;
                float pair_t;
                float pair_closest_chain[3];
                float pair_closest[3];
                float pair_dist;
                float pair_margin;
                float pair_chain_distance;
                float radius =
                    body_chain_collider_effective_radius_for_node(
                        BODY_COLLIDER_TESTICLES_MID);
                radius += body_chain_collider_pair_sweep_radius(
                    current_collider_sweep,
                    BODY_COLLIDER_TESTICLES_01,
                    BODY_COLLIDER_TESTICLES_02);
                if (body_chain_collider_node_in_chain_space(
                        chain_collider_state, collider_state,
                        BODY_COLLIDER_TESTICLES_01, pair_a) &&
                    body_chain_collider_node_in_chain_space(
                        chain_collider_state, collider_state,
                        BODY_COLLIDER_TESTICLES_02, pair_b) &&
                    body_chain_collider_pair_margin(
                        start, end, chain_len_at_start, 0.0f,
                        pair_a, pair_b, radius, &pair_chain_t, &pair_t,
                        pair_closest_chain, pair_closest, &pair_dist,
                        &pair_margin, &pair_chain_distance)) {
                    BODY_CHAIN_ACCUMULATE_CONTACT(
                        "testicles_pair", BODY_COLLIDER_TESTICLES_MID,
                        i, pair_chain_t, pair_dist, radius, pair_margin,
                        pair_closest_chain, pair_closest,
                        pair_a, pair_b);
                }
            }

            if (!(collision_scope_mask &
                  (BODY_CHAIN_COLLIDER_GROUP_PELVIS |
                   BODY_CHAIN_COLLIDER_GROUP_HANDS |
                   BODY_CHAIN_COLLIDER_GROUP_FULL_BODY))) {
                continue;
            }

            if (!(target_is_testicle &&
                  collider_person_index == person_index) &&
                collider_state->stomach_points_ready &&
                collider_state->valid[BODY_COLLIDER_STOMACH_01] &&
                collider_state->valid[BODY_COLLIDER_STOMACH_02]) {
                float stomach_a[3];
                float stomach_b[3];
                float stomach_chain_t;
                float stomach_t;
                float stomach_closest_chain[3];
                float stomach_closest[3];
                float stomach_dist;
                float stomach_radius;
                float stomach_margin;
                float stomach_chain_distance;
                if (body_chain_collider_stomach_pair_margin_in_frame(
                        start, end, chain_len_at_start, 0.0f,
                        chain_collider_state, collider_state,
                        &stomach_chain_t, &stomach_t,
                        stomach_closest_chain, stomach_closest,
                        &stomach_dist, &stomach_radius, &stomach_margin,
                        &stomach_chain_distance, stomach_a, stomach_b)) {
                    float stomach_sweep =
                        body_chain_collider_pair_sweep_radius(
                            current_collider_sweep,
                            BODY_COLLIDER_STOMACH_01,
                            BODY_COLLIDER_STOMACH_02);
                    stomach_radius += stomach_sweep;
                    stomach_margin -= stomach_sweep;
                    BODY_CHAIN_ACCUMULATE_CONTACT(
                        "spine_pair", BODY_COLLIDER_STOMACH_01,
                        i, stomach_chain_t, stomach_dist, stomach_radius,
                        stomach_margin, stomach_closest_chain,
                        stomach_closest, stomach_a, stomach_b);
                }
            }

            for (j = 0; j < BODY_COLLIDER_LIMB_PAIR_COUNT; j++) {
                float limb_a[3];
                float limb_b[3];
                float limb_chain_t;
                float limb_t;
                float limb_closest_chain[3];
                float limb_closest[3];
                float limb_dist;
                float limb_radius;
                float limb_margin;
                float limb_chain_distance;
                int limb_start_node = -1;
                int limb_end_node = -1;
                if (!collision_scope_is_full_body &&
                    j != BODY_COLLIDER_LIMB_PAIR_L_HIP_THIGH &&
                    j != BODY_COLLIDER_LIMB_PAIR_R_HIP_THIGH) {
                    continue;
                }
                if (body_chain_collider_limb_pair_margin_in_frame(
                        start, end, chain_len_at_start, 0.0f,
                        chain_collider_state, collider_state, j,
                        &limb_chain_t, &limb_t,
                        limb_closest_chain, limb_closest, &limb_dist,
                        &limb_radius, &limb_margin, &limb_chain_distance,
                        limb_a, limb_b)) {
                    float limb_sweep;
                    body_chain_collider_limb_pair_nodes(
                        j, &limb_start_node, &limb_end_node);
                    limb_sweep = body_chain_collider_pair_sweep_radius(
                        current_collider_sweep,
                        limb_start_node, limb_end_node);
                    limb_radius += limb_sweep;
                    limb_margin -= limb_sweep;
                    BODY_CHAIN_ACCUMULATE_CONTACT(
                        body_chain_collider_limb_pair_name(j), -1,
                        i, limb_chain_t, limb_dist, limb_radius,
                        limb_margin, limb_closest_chain, limb_closest,
                        limb_a, limb_b);
                }
            }
            if (!collision_scope_includes_hands) {
                continue;
            }
            for (j = 0;
                 j < (collision_scope_is_full_body ?
                      BODY_COLLIDER_EXTRA_EDGE_COUNT :
                      body_chain_hand_edge_count);
                 j++) {
                int edge_index = collision_scope_is_full_body ?
                    j : (int)body_chain_hand_edge_index[j];
                const body_collider_extra_edge_def_t *edge =
                    &body_collider_extra_edges[edge_index];
                float edge_chain_t;
                float edge_t;
                float edge_closest_chain[3];
                float edge_closest[3];
                float edge_dist;
                float edge_radius;
                float edge_margin;
                float edge_chain_distance;
                float r0;
                float r1;
                if (!edge ||
                    edge->start_node < 0 ||
                    edge->start_node >= BODY_COLLIDER_NODE_COUNT ||
                    edge->end_node < 0 ||
                    edge->end_node >= BODY_COLLIDER_NODE_COUNT ||
                    !current_collider_valid[edge->start_node] ||
                    !current_collider_valid[edge->end_node]) {
                    continue;
                }
                if (target_is_testicle &&
                    collider_person_index == person_index &&
                    (body_chain_testicle_self_upper_edge_node(
                         edge->start_node) ||
                     body_chain_testicle_self_upper_edge_node(
                         edge->end_node))) {
                    continue;
                }
                if (!body_chain_collider_pair_margin(
                        start, end, chain_len_at_start, 0.0f,
                        current_collider_points[edge->start_node],
                        current_collider_points[edge->end_node],
                        0.0f, &edge_chain_t, &edge_t,
                        edge_closest_chain, edge_closest,
                        &edge_dist, &edge_margin,
                        &edge_chain_distance)) {
                    continue;
                }
                r0 = body_chain_collider_effective_radius_for_node(
                    edge->start_node);
                r1 = body_chain_collider_effective_radius_for_node(
                    edge->end_node);
                edge_radius = r0 + (r1 - r0) *
                    physx_clampf(edge_t, 0.0f, 1.0f);
                edge_radius += body_chain_collider_pair_sweep_radius(
                    current_collider_sweep,
                    edge->start_node, edge->end_node);
                edge_margin = edge_dist - edge_radius;
                BODY_CHAIN_ACCUMULATE_CONTACT(
                    edge->name, -1, i, edge_chain_t, edge_dist,
                    edge_radius, edge_margin, edge_closest_chain,
                    edge_closest,
                    current_collider_points[edge->start_node],
                    current_collider_points[edge->end_node]);
            }
        }
        memset(chain_state->collision_prev_collider_valid
                   [collider_person_index],
               0,
               sizeof(chain_state->collision_prev_collider_valid
                          [collider_person_index]));
        for (active_node_index = 0;
             active_node_index < projection_cache->active_node_count;
             active_node_index++) {
            node = projection_cache->active_node[active_node_index];
            chain_state->collision_prev_collider_valid
                [collider_person_index][node] = 1;
            memcpy(chain_state->collision_prev_collider_points
                       [collider_person_index][node],
                   current_collider_points[node],
                   sizeof(current_collider_points[node]));
        }
        chain_state->collision_prev_collider_ready[collider_person_index] = 1;
    }

    if (room_response_enabled) {
        LONG room_track_generation =
            InterlockedCompareExchange(&named_node_generation, 0, 0);
        collider_person_index = -2;
        for (i = 0; i < segment_count; i++) {
            float *start = chain_points[i];
            float *end = chain_points[i + 1];
            float world_start[3];
            float world_end[3];
            float world_correction[3];
            float local_correction[3];
            float closest_room[3];
            float correction_len;
            float margin;
            float dist;
            float radius = owner_chain_radius;
            int temporal_contact = 0;
            if (!body_collision_local_point_to_world(
                    chain_collider_state, start, world_start) ||
                !body_collision_local_point_to_world(
                    chain_collider_state, end, world_end)) {
                chain_state->room_collision_track_valid[i] = 0;
                continue;
            }
            if (chain_state->room_collision_track_valid[i] &&
                chain_state->room_collision_track_generation[i] ==
                    room_track_generation &&
                now - chain_state->room_collision_track_tick[i] <= 160u) {
                float move[3];
                float move_len;
                move[0] = world_end[0] -
                    chain_state->room_collision_track_world[i][0];
                move[1] = world_end[1] -
                    chain_state->room_collision_track_world[i][1];
                move[2] = world_end[2] -
                    chain_state->room_collision_track_world[i][2];
                move_len = physx_vec3_len(move);
                if (sane_probe_float(move_len) && move_len <= 0.750f) {
                    temporal_contact = room_collision_resolve_swept_sphere(
                        chain_state->room_collision_track_world[i],
                        world_end, radius, world_correction, NULL);
                }
            }
            if (!temporal_contact &&
                !room_collision_resolve_swept_sphere(
                    world_start, world_end, radius,
                    world_correction, NULL)) {
                chain_state->room_collision_track_valid[i] = 1;
                chain_state->room_collision_track_generation[i] =
                    room_track_generation;
                chain_state->room_collision_track_tick[i] = now;
                memcpy(chain_state->room_collision_track_world[i],
                       world_end, sizeof(world_end));
                continue;
            }
            {
                float world_correction_len =
                    physx_vec3_len(world_correction);
                float separation_slop = physx_clampf(
                    radius * 0.03f, 0.00050f, 0.00100f);
                int room_axis;
                for (room_axis = 0; room_axis < 3; room_axis++) {
                    float normal = world_correction_len > 0.000001f ?
                        world_correction[room_axis] /
                            world_correction_len : 0.0f;
                    chain_state->room_collision_track_world[i][room_axis] =
                        world_end[room_axis] +
                        world_correction[room_axis] +
                        normal * separation_slop;
                }
                chain_state->room_collision_track_valid[i] = 1;
                chain_state->room_collision_track_generation[i] =
                    room_track_generation;
                chain_state->room_collision_track_tick[i] = now;
            }
            if (!body_collision_world_vector_to_local(
                    chain_collider_state, world_correction,
                    local_correction)) {
                continue;
            }
            chain_state->collision_room_contacts++;
            correction_len = physx_vec3_len(local_correction);
            if (correction_len <= 0.000001f) continue;
            closest_room[0] = chain_points[i + 1][0] - local_correction[0];
            closest_room[1] = chain_points[i + 1][1] - local_correction[1];
            closest_room[2] = chain_points[i + 1][2] - local_correction[2];
            margin = -correction_len;
            dist = radius - correction_len;
            if (dist < 0.0f) dist = 0.0f;
            BODY_CHAIN_ACCUMULATE_CONTACT(
                "room_mesh", -2, i, 1.0f, dist, radius, margin,
                chain_points[i + 1], closest_room,
                closest_room, closest_room);
        }
    }

#undef BODY_CHAIN_ACCUMULATE_CONTACT

    body_profile_set_active_person_config(person_index);
    {
        int fresh_sweep_ghost =
            chain_points_from_engine == 1 &&
            chain_state->collision_prev_chain_points_ready &&
            chain_sweep_move > 0.060f &&
            largest_sweep_radius >= 0.060f &&
            largest_collider_motion <= 0.008f &&
            max_penetration >= 0.010f &&
            contact_count >= 2;

    if (!target_is_testicle &&
        fresh_sweep_ghost) {
        for (i = 0; i < 3; i++) {
            correction[i][0] = 0.0f;
            correction[i][1] = 0.0f;
            chain_state->collision_contact_direction[i][0] = 0.0f;
            chain_state->collision_contact_direction[i][1] = 0.0f;
        }
        chain_state->collision_prev_max_penetration = 0.0f;
        chain_state->collision_rest_penetration = 0.0f;
        chain_state->collision_rest_ticks = 0;
        chain_state->collision_rest_valid = 0;
        chain_state->collision_rest_grace_ticks = 0;
        chain_state->collision_impact_ticks = 0;
        chain_state->collision_multi_support_grace_ticks = 0;
        chain_state->collision_prev_chain_points_ready = 0;
        memset(chain_state->collision_prev_chain_points, 0,
               sizeof(chain_state->collision_prev_chain_points));
        if (person_index >= 0 && person_index < 4) {
            body_chain_penis_pivot_quarantine_until[person_index] =
                now + 1000u;
        }
        if (max_penetration_out) {
            *max_penetration_out = 0.0f;
        }
        if (sweep_radius_out) {
            *sweep_radius_out = largest_sweep_radius;
        }
        if (collider_motion_out) {
            *collider_motion_out = largest_collider_motion;
        }
        chain_state->collision_manifold_contacts = 0;
        chain_collider_state->chain_points_ready = 0;
        chain_collider_state->chain_points_fresh = 0;
        chain_collider_state->chain_points_update_tick = 0;
        for (i = 0; i < 4; i++) {
            chain_collider_state->chain_point_valid[i] = 0;
        }
        body_chain_penis_clear_reconnect_candidate(person_index);
        if (emit_log &&
            (body_chain_collider_cfg.diagnostic || defaults_cfg.debug) &&
            (!chain_collider_state->last_response_log_tick ||
             now - chain_collider_state->last_response_log_tick >= 250u)) {
            chain_collider_state->last_response_log_tick = now;
            log_line("body-chain-colliders response-suppressed target=\"%s\" person_index=%d collider_person_index=%d hits=%d contacts=%d nearest_segment=seg%02d nearest_node=\"%s\" nearest_t=%.3f nearest_margin=%.5f max_penetration=%.5f chain_sweep_move=%.5f sweep_radius=%.5f collider_motion=%.5f pose_source=\"%s\" quarantine_ms=%lu note=\"penis centerline matched camera-attached ghost contact; clearing collision handoff and keeping live pivot cache out of collision history\"",
                     target_name,
                     person_index + 1,
                     nearest_collider_person + 1,
                     hit_count,
                     contact_count,
                     nearest_segment + 1,
                     nearest_label ? nearest_label :
                        (nearest_node >= 0 ?
                         body_chain_collider_node_label(nearest_node) : "none"),
                     nearest_t,
                     nearest_margin,
                     max_penetration,
                     chain_sweep_move,
                     largest_sweep_radius,
                     largest_collider_motion,
                     pose_source,
                     1000ul);
        }
        return;
    }
    }
    chain_state->collision_manifold_contacts = contact_count;
    if (contact_count > 1) {
        chain_state->collision_multi_support_grace_ticks = 3;
    }
    solver_iterations = (int)physx_clampf((float)body_chain_collider_cfg.collision_iterations,1.0f,6.0f)*8;
    body_contact_solve(target_is_testicle ? &testicle_physics_cfg : &body_chain_physics_cfg,
        chain_state, chain_points, contacts, contact_count, segment_count, correction);
    if (emit_log && contact_count && (defaults_cfg.debug || body_chain_collider_cfg.diagnostic) &&
        (!chain_state->collision_solve_log_tick || now-chain_state->collision_solve_log_tick>=250u)) {
        float solved[4][3];
        body_contact_predict(
            target_is_testicle ? &testicle_physics_cfg : &body_chain_physics_cfg,
            chain_state, chain_points, correction, solved);
        chain_state->collision_solve_log_tick=now;
        log_line("body-contact bounded-solve target=\"%s\" person_index=%d contacts=%d dt=%.5f overlap_error=(%.9f,%.9f) correction=(%.3f,%.3f;%.3f,%.3f;%.3f,%.3f) note=\"squared contact-plane residual before/after; independent target diagnostics\"",
            target_name,person_index+1,contact_count,chain_state->collision_step_dt,
            body_contact_overlap_error(chain_points,contacts,contact_count),
            body_contact_overlap_error(solved,contacts,contact_count),
            correction[0][0],correction[0][1],correction[1][0],correction[1][1],
            correction[2][0],correction[2][1]);
    }

    projected_h = 0.0f;
    projected_v = 0.0f;
    for (i = 0; i < segment_count; i++) {
        correction[i][0] = physx_clampf(
            correction[i][0],
            -body_chain_collider_cfg.response_max_degrees_per_tick,
             body_chain_collider_cfg.response_max_degrees_per_tick);
        correction[i][1] = physx_clampf(
            correction[i][1],
            -body_chain_collider_cfg.response_max_degrees_per_tick,
             body_chain_collider_cfg.response_max_degrees_per_tick);
        projected_h += correction[i][0];
        projected_v += correction[i][1];
    }
    if (max_penetration_out) {
        *max_penetration_out = max_penetration;
    }
    if (sweep_radius_out) {
        *sweep_radius_out = largest_sweep_radius;
    }
    if (collider_motion_out) {
        *collider_motion_out = largest_collider_motion;
    }
    memcpy(chain_state->collision_prev_chain_points, chain_points,
           sizeof(chain_state->collision_prev_chain_points));
    chain_state->collision_prev_chain_points_ready = 1;
    if (emit_log && (body_chain_collider_cfg.diagnostic ||
         (hit_count && defaults_cfg.debug)) &&
        (!chain_collider_state->last_response_log_tick ||
         now - chain_collider_state->last_response_log_tick >=
            (DWORD)body_chain_collider_cfg.response_log_ms)) {
        chain_collider_state->last_response_log_tick = now;
        log_line("body-chain-colliders response-simple target=\"%s\" person_index=%d collider_person_index=%d hits=%d contacts=%d solver_iterations=%d hierarchical_support=%d nearest_segment=seg%02d nearest_node=\"%s\" nearest_t=%.3f nearest_dist=%.5f nearest_radius=%.5f nearest_margin=%.5f nearest_closest=(%.5f,%.5f,%.5f) max_penetration=%.5f sweep_radius=%.5f collider_motion=%.5f correction=(h=%.3f,v=%.3f) endpoint01_local=(%.5f,%.5f,%.5f) endpoint02_local=(%.5f,%.5f,%.5f) endpoint03_local=(%.5f,%.5f,%.5f) pose_source=\"%s\" note=\"iterative compact contact manifold; passive colliders from all ready persons share the same INI settings\"",
                 target_name,
                 person_index + 1,
                 nearest_collider_person + 1,
                 hit_count,
                 contact_count,
                 solver_iterations,
                 hierarchical_support,
                 nearest_segment + 1,
                 nearest_label ? nearest_label :
                    (nearest_node >= 0 ?
                     body_chain_collider_node_label(nearest_node) : "none"),
                 nearest_t,
                 nearest_dist,
                 nearest_radius,
                 nearest_margin,
                 nearest_closest[0],
                 nearest_closest[1],
                 nearest_closest[2],
                 max_penetration,
                 largest_sweep_radius,
                 largest_collider_motion,
                 projected_h,
                 projected_v,
                 endpoint[0][0],
                 endpoint[0][1],
                 endpoint[0][2],
                 endpoint[1][0],
                 endpoint[1][1],
                 endpoint[1][2],
                 endpoint[2][0],
                 endpoint[2][1],
                 endpoint[2][2],
                 pose_source);
    }
    return;
}

