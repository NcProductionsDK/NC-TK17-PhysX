/* Isolated two-joint butt solver. This deliberately shares no mutable
   configuration or runtime state with breast PhysX. */

static int resolve_butt_physics_raws(
    const char *person, void **root_raw_out, void *source_raw_out[2],
    void *animation_raw_out[2], void *translation_parent_raw_out[2])
{
    static const char *source_names[2] = {
        "Sbutt_L_joint01", "Sbutt_R_joint01"
    };
    static const char *animation_names[2] = {
        "butt_L_joint01", "butt_R_joint01"
    };
    char name[256];
    int side;
    if (!person || !root_raw_out || !source_raw_out ||
        !animation_raw_out || !translation_parent_raw_out) return 0;
    make_body_runtime_name(name, sizeof(name), person, "root");
    *root_raw_out = resolve_axis_map_raw(name);
    if (!*root_raw_out ||
        !body_chain_runtime_tjoint_layout_valid(*root_raw_out)) return 0;
    for (side = 0; side < 2; side++) {
        make_body_runtime_name(name, sizeof(name), person,
                               source_names[side]);
        source_raw_out[side] = resolve_axis_map_raw(name);
        make_body_runtime_name(name, sizeof(name), person,
                               animation_names[side]);
        animation_raw_out[side] = resolve_axis_map_raw(name);
        translation_parent_raw_out[side] = *root_raw_out;
        if (!source_raw_out[side] || !animation_raw_out[side] ||
            !body_chain_runtime_tjoint_layout_valid(
                animation_raw_out[side])) return 0;
    }
    return 1;
}

static int butt_physics_values_sane(
    void *root_raw, void *source_raw[2], void *animation_raw[2],
    void *translation_parent_raw[2])
{
    int side;
    if (!root_raw ||
        !ptr_readable((BYTE*)root_raw +
                          butt_physics_cfg.root_offset,
                      sizeof(float) * 3)) return 0;
    for (side = 0; side < 2; side++) {
        float *rotation;
        float *translation;
        if (!source_raw[side] || !animation_raw[side] ||
            !translation_parent_raw[side] ||
            !ptr_readable((BYTE*)source_raw[side] +
                              butt_physics_cfg.output_offset,
                          sizeof(float) * 3) ||
            !ptr_readable((BYTE*)source_raw[side] +
                              butt_physics_bone_translation_offset,
                          sizeof(float) * 3) ||
            !body_chain_runtime_tjoint_layout_valid(animation_raw[side]) ||
            !body_chain_runtime_tjoint_layout_valid(
                translation_parent_raw[side])) return 0;
        rotation = (float*)((BYTE*)source_raw[side] +
                            butt_physics_cfg.output_offset);
        translation = (float*)((BYTE*)source_raw[side] +
                               butt_physics_bone_translation_offset);
        if (!body_chain_vec3_sane_limit(rotation, 720.0f) ||
            !body_chain_vec3_sane_limit(translation, 64.0f)) return 0;
    }
    return 1;
}

static int butt_physics_candidate_is_stable(
    breasts_physics_person_state_t *state, void *root_raw,
    void *source_raw[2], void *animation_raw[2],
    void *translation_parent_raw[2], DWORD now)
{
    int same;
    if (!state || !root_raw || !source_raw[0] || !source_raw[1] ||
        !animation_raw[0] || !animation_raw[1] ||
        !translation_parent_raw[0] || !translation_parent_raw[1]) return 0;
    if (state->initialized) return 1;
    same = state->ownership_candidate_root_raw == root_raw &&
        state->ownership_candidate_source_raw[0] == source_raw[0] &&
        state->ownership_candidate_source_raw[1] == source_raw[1] &&
        state->ownership_candidate_animation_raw[0] == animation_raw[0] &&
        state->ownership_candidate_animation_raw[1] == animation_raw[1] &&
        state->ownership_candidate_translation_parent_raw[0] ==
            translation_parent_raw[0] &&
        state->ownership_candidate_translation_parent_raw[1] ==
            translation_parent_raw[1];
    if (!same) {
        state->ownership_candidate_root_raw = root_raw;
        state->ownership_candidate_source_raw[0] = source_raw[0];
        state->ownership_candidate_source_raw[1] = source_raw[1];
        state->ownership_candidate_animation_raw[0] = animation_raw[0];
        state->ownership_candidate_animation_raw[1] = animation_raw[1];
        state->ownership_candidate_translation_parent_raw[0] =
            translation_parent_raw[0];
        state->ownership_candidate_translation_parent_raw[1] =
            translation_parent_raw[1];
        state->ownership_candidate_tick = now;
        state->ownership_candidate_samples = 1;
        return 0;
    }
    if (state->ownership_candidate_samples < 0xffffffffu)
        state->ownership_candidate_samples++;
    return state->ownership_candidate_samples >= 3u &&
           now - state->ownership_candidate_tick >= 50u;
}

static int butt_physics_live_ownership_matches(
    int person_index, breasts_physics_person_state_t *state)
{
    void *root_raw = NULL;
    void *source_raw[2] = { NULL, NULL };
    void *animation_raw[2] = { NULL, NULL };
    void *translation_parent_raw[2] = { NULL, NULL };
    LONG node_generation;
    if (person_index < 0 || person_index >= 4 || !state ||
        !state->initialized) return 0;
    node_generation = InterlockedCompareExchange(
        &named_node_generation, 0, 0);
    if (InterlockedCompareExchange(&physx_late_ownership_active, 0, 0) &&
        physx_simulation_serial &&
        state->live_ownership_simulation_serial ==
            physx_simulation_serial &&
        state->live_ownership_node_generation == node_generation) {
        return 1;
    }
    if (!resolve_butt_physics_raws(body_chain_person_name(person_index),
                                   &root_raw, source_raw, animation_raw,
                                   translation_parent_raw)) return 0;
    if (!(root_raw == state->motion.root_raw &&
        source_raw[0] == state->source_joint_raw[0] &&
        source_raw[1] == state->source_joint_raw[1] &&
        animation_raw[0] == state->animation_joint_raw[0] &&
        animation_raw[1] == state->animation_joint_raw[1] &&
        translation_parent_raw[0] ==
            state->translation_parent_joint_raw[0] &&
        translation_parent_raw[1] ==
            state->translation_parent_joint_raw[1] &&
        butt_physics_values_sane(root_raw, source_raw, animation_raw,
                                 translation_parent_raw))) return 0;
    state->live_ownership_simulation_serial = physx_simulation_serial;
    state->live_ownership_node_generation = node_generation;
    return 1;
}

static void butt_physics_capture_animation_rows(
    breasts_physics_person_state_t *state)
{
    int side;
    if (!state) return;
    for (side = 0; side < 2; side++) {
        void *joint = state->animation_joint_raw[side];
        if (!body_chain_runtime_tjoint_layout_valid(joint)) return;
        memcpy(&state->animation_rows[side][0],
               (BYTE*)joint + 0x078, sizeof(float) * 3);
        memcpy(&state->animation_rows[side][3],
               (BYTE*)joint + 0x088, sizeof(float) * 3);
        memcpy(&state->animation_rows[side][6],
               (BYTE*)joint + 0x098, sizeof(float) * 3);
    }
    state->animation_rows_valid = 1;
}

static int butt_physics_apply_output(
    breasts_physics_person_state_t *state, int restore_handoff,
    int capture_animation)
{
    int side, axis;
    int wrote_translation = 0;
    if (!state || !state->initialized) return 0;
    (void)capture_animation;
    for (side = 0; side < 2; side++) {
        float *output;
        float *translation_output;
        int write_translation = restore_handoff
            ? state->bone_translation_applied
            : (butt_physics_bone_translation_enabled ||
               state->contact_translation_active || state->bone_translation_applied);
        if (!state->source_joint_raw[side] ||
            !ptr_readable((BYTE*)state->source_joint_raw[side] +
                              butt_physics_cfg.output_offset,
                          sizeof(float) * 3) ||
            (write_translation &&
             !ptr_readable((BYTE*)state->source_joint_raw[side] +
                               butt_physics_bone_translation_offset,
                           sizeof(float) * 3))) return 0;
        output = (float*)((BYTE*)state->source_joint_raw[side] +
                          butt_physics_cfg.output_offset);
        for (axis = 0; axis < 3; axis++) {
            output[axis] = restore_handoff
                ? state->source_handoff[side][axis]
                : state->rest_rotation[side][axis] +
                      state->rotation[side][axis];
        }
        if (write_translation) {
            translation_output =
                (float*)((BYTE*)state->source_joint_raw[side] +
                         butt_physics_bone_translation_offset);
            for (axis = 0; axis < 3; axis++) {
                translation_output[axis] =
                    state->source_translation_handoff[side][axis] +
                    ((!restore_handoff &&
                      (butt_physics_bone_translation_enabled || state->contact_translation_active))
                         ? state->bone_translation[side][axis] : 0.0f);
            }
            wrote_translation = 1;
        }
        body_chain_mark_runtime_transform_dirty(
            state->source_joint_raw[side]);
        if (butt_physics_cfg.override_animation &&
            state->animation_rows_valid) {
            memcpy((BYTE*)state->animation_joint_raw[side] + 0x078,
                   &state->animation_rows[side][0], sizeof(float) * 3);
            memcpy((BYTE*)state->animation_joint_raw[side] + 0x088,
                   &state->animation_rows[side][3], sizeof(float) * 3);
            memcpy((BYTE*)state->animation_joint_raw[side] + 0x098,
                   &state->animation_rows[side][6], sizeof(float) * 3);
        }
    }
    state->output_applied = restore_handoff ? 0 : 1;
    if (wrote_translation) {
        state->bone_translation_applied =
            (!restore_handoff &&
             (butt_physics_bone_translation_enabled || state->contact_translation_active)) ? 1 : 0;
    }
    return 1;
}

static const poseeditor_paired_bone_track_def_t
butt_physics_poseeditor_tracks[] = {
    { POSEEDIT_TRACK_BUTT_SQUEEZE_LEFT,
      "body_size_poseBlend_buttcheek_L_squeeze", "Squeeze Left", 1 },
    { POSEEDIT_TRACK_BUTT_SQUEEZE_RIGHT,
      "body_size_poseBlend_buttcheek_R_squeeze", "Squeeze Right", 1 },
    { POSEEDIT_TRACK_BUTT_LEFT, "butt_L_joint01", "Butt L", 0 },
    { POSEEDIT_TRACK_BUTT_RIGHT, "butt_R_joint01", "Butt R", 0 }
};

static void reset_butt_physics_person_state(
    int person_index, breasts_physics_person_state_t *state,
    int restore_output)
{
    unsigned int restored_tracks;
    if (!state) return;
    if (restore_output && state->output_applied &&
        butt_physics_live_ownership_matches(person_index, state))
        butt_physics_apply_output(state, 1, 0);
    restored_tracks = restore_poseeditor_paired_bone_tracks(
        &state->pose_tracks,
        (int)(sizeof(butt_physics_poseeditor_tracks) /
              sizeof(butt_physics_poseeditor_tracks[0])));
    if (restored_tracks) {
        schedule_poseeditor_track_handoff_refresh(
            person_index, restored_tracks, GetTickCount());
    }
    memset(state, 0, sizeof(*state));
}

static void butt_physics_build_side_drive(
    const body_chain_physics_config_t *cfg, int side,
    const float translation_step[3], const float rotation_step[3],
    float out[3])
{
    int channel;
    out[0] = out[1] = out[2] = 0.0f;
    if (!cfg || side < 0 || side > 1) return;
    for (channel = 0; channel < 3; channel++) {
        int tail_axis = cfg->translation_tail_axis[channel];
        if (tail_axis >= 0 && tail_axis <= 2) {
            out[tail_axis] += translation_step[channel] *
                cfg->translation_scale[channel] *
                butt_physics_translation_sign[side][channel] *
                BODY_CHAIN_TRANSLATION_RESPONSE;
        }
    }
    for (channel = 0; channel < 3; channel++) {
        int source_axis = cfg->rotation_source_axis[channel];
        int tail_axis = cfg->rotation_tail_axis[channel];
        if (source_axis >= 0 && source_axis <= 2 &&
            tail_axis >= 0 && tail_axis <= 2) {
            float value = rotation_step[source_axis];
            if (physx_absf(value) >= cfg->rotation_deadzone) {
                out[tail_axis] += value * cfg->rotation_scale[channel] *
                    butt_physics_rotation_sign[side][channel];
            }
        }
    }
}

static int butt_physics_body_translation_to_parent_local(
    const breasts_physics_person_state_t *state, int side,
    const float body_target[3], float out[3])
{
    float parent_view[9], trs_view[9], trs_inverse[9];
    float parent_relative[9], parent_inverse[9], body_relative[9];
    float target_relative[3];
    if (!state || side < 0 || side > 1 || !body_target || !out ||
        !state->motion.camera_relative_live_current_valid ||
        !state->motion.camera_relative_trs_raw ||
        !state->translation_parent_joint_raw[side] ||
        !body_chain_read_mat3_rows(
            state->translation_parent_joint_raw[side], parent_view) ||
        !body_chain_read_mat3_rows(
            state->motion.camera_relative_trs_raw, trs_view) ||
        !body_chain_mat3_inverse(trs_view, trs_inverse)) return 0;
    memcpy(body_relative, state->motion.camera_relative_live_current,
           sizeof(body_relative));
    if (!body_chain_normalize_basis_rows(body_relative)) return 0;
    body_chain_mat3_multiply(parent_view, trs_inverse, parent_relative);
    if (!body_chain_normalize_basis_rows(parent_relative) ||
        !body_chain_mat3_inverse(parent_relative, parent_inverse)) return 0;
    body_chain_transform_row_vector3(body_target, body_relative,
                                     target_relative);
    body_chain_transform_row_vector3(target_relative, parent_inverse, out);
    return body_chain_vec3_sane_limit(out, 0.25f);
}

static void butt_physics_build_bone_translation_target(
    const body_chain_physics_config_t *cfg,
    const breasts_physics_person_state_t *state, int side,
    const float local_step[3], float out[3])
{
    float mapped[3] = { 0.0f, 0.0f, 0.0f };
    int channel;
    if (!cfg || !state || side < 0 || side > 1 ||
        !local_step || !out) return;
    out[0] = out[1] = out[2] = 0.0f;
    for (channel = 0; channel < 3; channel++) {
        int source_axis = butt_physics_bone_translation_source_axis[channel];
        int tail_axis = butt_physics_bone_translation_tail_axis[channel];
        float drive = local_step[source_axis];
        if (physx_absf(drive) <= cfg->translation_deadzone) drive = 0.0f;
        mapped[tail_axis] += drive *
            butt_physics_bone_translation_scale[channel] *
            butt_physics_bone_translation_sign[side][channel];
    }
    if (butt_physics_bone_translation_space &&
        butt_physics_body_translation_to_parent_local(
            state, side, mapped, out)) return;
    memcpy(out, mapped, sizeof(mapped));
}

static void butt_physics_add_gravity_target(
    const body_chain_physics_config_t *cfg, int side,
    const float gravity_drive[3], float out[3])
{
    int channel;
    float inversion_blend, planar_scale;
    if (!cfg || !gravity_drive || !out || side < 0 || side > 1) return;
    inversion_blend = paired_bone_inverted_gravity_amount(gravity_drive);
    planar_scale = 1.0f - inversion_blend;
    for (channel = 0; channel < 2; channel++) {
        int tail_axis = butt_physics_gravity_tail_axis[channel];
        if (tail_axis >= 0 && tail_axis <= 2) {
            out[tail_axis] += gravity_drive[channel] *
                cfg->gravity_angle *
                butt_physics_gravity_sign[side][channel] * planar_scale;
        }
    }
    if (inversion_blend > 0.0f) {
        int tail_axis = butt_physics_gravity_tail_axis[2];
        if (tail_axis >= 0 && tail_axis <= 2) {
            out[tail_axis] +=
                inversion_blend * cfg->gravity_inverted_strength *
                cfg->gravity_inverted_sign *
                butt_physics_gravity_sign[side][2];
        }
    }
}

static void run_butt_physics_for_person(int person_index, DWORD now)
{
    const char *person = body_chain_person_name(person_index);
    breasts_physics_person_state_t *state =
        &butt_physics_states[person_index];
    body_chain_physics_config_t *cfg = &butt_physics_cfg;
    void *root_raw = NULL;
    void *source_raw[2] = { NULL, NULL };
    void *animation_raw[2] = { NULL, NULL };
    void *translation_parent_raw[2] = { NULL, NULL };
    float *root;
    float root_pivot[3];
    float local_step[3] = { 0.0f, 0.0f, 0.0f };
    float translation_step[3] = { 0.0f, 0.0f, 0.0f };
    float rotation_delta[9] = { 0.0f };
    float parent_rotation_step[3] = { 0.0f, 0.0f, 0.0f };
    float gravity_drive[3] = { 0.0f, 0.0f, 0.0f };
    float wind_channels[2][3] = {
        { 0.0f, 0.0f, 0.0f },
        { 0.0f, 0.0f, 0.0f }
    };
    float target[2][3] = { { 0.0f }, { 0.0f } };
    float bone_target[2][3] = { { 0.0f }, { 0.0f } };
    float collision_local_offset[2][3] = { { 0.0f }, { 0.0f } };
    float dt;
    DWORD elapsed_ms;
    int wind_active[2] = { 0, 0 };
    int side, axis, channel;

    if (!state->initialized && state->resolve_retry_tick &&
        now - state->resolve_retry_tick <
            (DWORD)BUTT_PHYSICS_MISSING_RETRY_MS) return;
    if (state->last_tick &&
        now - state->last_tick < (DWORD)cfg->interval_ms) return;
    elapsed_ms = state->last_tick ? now - state->last_tick : 0;
    state->last_tick = now;
    dt = elapsed_ms ? (float)elapsed_ms / 1000.0f : 0.016f;
    if (dt <= 0.0f) dt = 0.016f;
    if (dt > 0.025f) dt = 0.025f;

    if (state->initialized && state->cache_verify_tick &&
        now - state->cache_verify_tick <
            (DWORD)BUTT_PHYSICS_CACHE_VERIFY_MS) {
        root_raw = state->motion.root_raw;
        for (side = 0; side < 2; side++) {
            source_raw[side] = state->source_joint_raw[side];
            animation_raw[side] = state->animation_joint_raw[side];
            translation_parent_raw[side] =
                state->translation_parent_joint_raw[side];
        }
    } else {
        if (!resolve_butt_physics_raws(
                person, &root_raw, source_raw, animation_raw,
                translation_parent_raw) ||
            !butt_physics_values_sane(
                root_raw, source_raw, animation_raw,
                translation_parent_raw)) {
            reset_butt_physics_person_state(person_index, state, 0);
            state->resolve_retry_tick = now;
            restore_tk17_butt_inertia_for_person(person_index, now);
            return;
        }
        if (state->initialized &&
            (root_raw != state->motion.root_raw ||
             source_raw[0] != state->source_joint_raw[0] ||
             source_raw[1] != state->source_joint_raw[1] ||
             animation_raw[0] != state->animation_joint_raw[0] ||
             animation_raw[1] != state->animation_joint_raw[1])) {
            reset_butt_physics_person_state(person_index, state, 0);
            restore_tk17_butt_inertia_for_person(person_index, now);
            return;
        }
        state->cache_verify_tick = now;
    }
    state->resolve_retry_tick = 0;
    root = (float*)((BYTE*)root_raw + cfg->root_offset);
    if (!butt_physics_candidate_is_stable(
            state, root_raw, source_raw, animation_raw,
            translation_parent_raw, now)) return;
    state->live_ownership_simulation_serial = physx_simulation_serial;
    state->live_ownership_node_generation = InterlockedCompareExchange(
        &named_node_generation, 0, 0);

    if (!state->initialized) {
        for (side = 0; side < 2; side++) {
            float *output = (float*)((BYTE*)source_raw[side] +
                                     cfg->output_offset);
            float *translation_output =
                (float*)((BYTE*)source_raw[side] +
                         butt_physics_bone_translation_offset);
            state->source_joint_raw[side] = source_raw[side];
            state->animation_joint_raw[side] = animation_raw[side];
            state->translation_parent_joint_raw[side] =
                translation_parent_raw[side];
            for (axis = 0; axis < 3; axis++) {
                state->source_handoff[side][axis] = output[axis];
                state->source_translation_handoff[side][axis] =
                    translation_output[axis];
                state->rest_rotation[side][axis] =
                    cfg->zero_output_rest ? 0.0f : output[axis];
                state->rotation[side][axis] = 0.0f;
                state->angular_velocity[side][axis] = 0.0f;
                state->bone_translation[side][axis] = 0.0f;
                state->bone_translation_velocity[side][axis] = 0.0f;
            }
        }
        butt_physics_capture_animation_rows(state);
    }

    if (!body_chain_runtime_mode_active() && cfg->override_animation &&
        cfg->poseeditor_track_override &&
        !suppress_poseeditor_paired_bone_tracks(
            person_index, person, "butt-physics", &state->pose_tracks,
            butt_physics_poseeditor_tracks,
            (int)(sizeof(butt_physics_poseeditor_tracks) /
                  sizeof(butt_physics_poseeditor_tracks[0])))) {
        return;
    }

    if (!suppress_tk17_butt_inertia_for_person(person_index, now)) return;

    if (!state->initialized) {
        memset(&state->motion, 0, sizeof(state->motion));
        state->motion.initialized = 1;
        state->motion.root_raw = root_raw;
        state->motion.init_tick = now;
        state->initialized = 1;
        body_chain_camera_relative_orientation_step(
            person, &state->motion, root_raw, rotation_delta, NULL, NULL);
        butt_physics_apply_output(state, 0, 0);
        return;
    }

    root_pivot[0] = root[0];
    root_pivot[1] = root[1];
    root_pivot[2] = root[2];
    body_collider_engine_pivot_view(person, "root", NULL, root_pivot);
    body_chain_camera_relative_orientation_step(
        person, &state->motion, root_raw, rotation_delta,
        parent_rotation_step, NULL);
    body_chain_camera_neutral_pivot_step(
        person, person_index, &state->motion, root_pivot,
        cfg->root_offset, NULL, local_step);
    for (channel = 0; channel < 3; channel++) {
        translation_step[channel] =
            local_step[cfg->translation_source_axis[channel]];
    }
    if ((physx_absf(cfg->gravity_angle) > 0.000001f ||
         cfg->gravity_inverted_strength > 0.000001f) &&
        physics_environment_cfg.world_gravity_probe &&
        physics_environment_cfg.gravity_apply_to_body_chain) {
        run_body_chain_gravity_probe(
            person, &state->motion, root_raw, root, now,
            &butt_physics_room_gravity_cache[person_index],
            "butt_physics");
        if (state->motion.gravity_probe_promoted) {
            const float *active_gravity;
            float relative_raw[3];
            update_body_chain_gravity_filter(&state->motion, dt);
            active_gravity = state->motion.gravity_drive_filtered_valid
                ? state->motion.gravity_drive_filtered
                : state->motion.gravity_drive;
            if (!state->gravity_reference_valid) {
                for (channel = 0; channel < 3; channel++) {
                    state->gravity_reference[channel] =
                        active_gravity[channel];
                    state->gravity_relative[channel] = 0.0f;
                }
                state->gravity_reference_valid = 1;
                log_line("butt-physics gravity-reference captured person=\"%s\" reference=(%.5f,%.5f,%.5f) strength=%.2f",
                         person, state->gravity_reference[0],
                         state->gravity_reference[1],
                         state->gravity_reference[2], cfg->gravity_angle);
            } else {
                body_gravity_direction_drive(
                    active_gravity, state->gravity_reference,
                    state->gravity_relative, relative_raw);
                memcpy(state->gravity_relative, relative_raw,
                       sizeof(relative_raw));
                body_chain_apply_gravity_curve(
                    relative_raw, cfg->gravity_horizontal_curve,
                    cfg->gravity_vertical_curve, gravity_drive);
            }
        }
    }
    for (side = 0; side < 2; side++) {
        wind_active[side] = body_chain_room_wind_channels(
            cfg, &state->motion,
            state->translation_parent_joint_raw[side],
            person, "butt_physics", now, wind_channels[side]);
        butt_physics_build_side_drive(
            cfg, side, translation_step, parent_rotation_step,
            target[side]);
        butt_physics_add_gravity_target(
            cfg, side, gravity_drive, target[side]);
        if (wind_active[side]) {
            body_chain_add_room_wind_target(
                cfg,
                wind_channels[side],
                butt_physics_wind_sign[side],
                target[side]);
        }
        for (axis = 0; axis < 3; axis++) {
            float acceleration;
            target[side][axis] = body_chain_clamp_link_axis_angle(
                cfg, 0, axis, target[side][axis] * cfg->link_gain[0]);
            acceleration =
                (target[side][axis] - state->rotation[side][axis]) *
                    cfg->stiffness -
                state->angular_velocity[side][axis] * cfg->damping;
            state->angular_velocity[side][axis] += acceleration * dt;
            state->rotation[side][axis] +=
                state->angular_velocity[side][axis] * dt;
            state->rotation[side][axis] =
                body_chain_clamp_link_axis_angle(
                    cfg, 0, axis, state->rotation[side][axis]);
        }
    }
    /* Free translation targets remain independent of contact recovery. */
    for (side = 0; side < 2; side++) {
        if (butt_physics_bone_translation_enabled) {
            butt_physics_build_bone_translation_target(
                cfg, state, side, local_step, bone_target[side]);
        }
    }
    state->contact_translation_active = butt_physics_bone_translation_enabled ||
        (body_chain_collider_cfg.enabled && body_chain_collider_cfg.butt_collision_enabled) ||
        (cfg->room_collision_enabled && room_collision_is_enabled()) || state->bone_translation_applied;
    if (state->contact_translation_active) {
        single_bone_contact_step(person_index, 1, state, cfg,
            now, elapsed_ms, bone_target, butt_physics_bone_translation_stiffness,
            butt_physics_bone_translation_damping, butt_physics_bone_translation_max_offset,
            collision_local_offset);
    } else {
        memset(state->bone_translation,0,sizeof(state->bone_translation));
        memset(state->bone_translation_velocity,0,sizeof(state->bone_translation_velocity));
    }
    butt_physics_apply_output(state, 0, 0);
    if (!state->active_logged) {
        state->active_logged = 1;
        log_line("butt-physics active person=\"%s\" targets=\"butt_L_joint01,butt_R_joint01\" gravity=(strength=%.2f,inverted_strength=%.2f,inverted_axis=%d,inverted_sign=%.2f) spring=(%.2f,%.2f) bone_translation=(enabled=%d,space=%s,stiffness=%.2f,damping=%.2f,max=%.4f/%.4f/%.4f) collision=(enabled=%d,scope=%s) translation_gravity_sag=not-supported note=\"two independent single-bone springs; child joints follow naturally\"",
                 person, cfg->gravity_angle,
                 cfg->gravity_inverted_strength,
                 butt_physics_gravity_tail_axis[2],
                 cfg->gravity_inverted_sign,
                 cfg->stiffness, cfg->damping,
                 butt_physics_bone_translation_enabled,
                 butt_physics_bone_translation_space ? "body" : "local",
                 butt_physics_bone_translation_stiffness,
                 butt_physics_bone_translation_damping,
                 butt_physics_bone_translation_max_offset[0],
                 butt_physics_bone_translation_max_offset[1],
                 butt_physics_bone_translation_max_offset[2],
                 body_chain_collider_cfg.butt_collision_enabled,
                 body_chain_collision_scope_name(cfg->collision_scope));
    }
    if (defaults_cfg.debug &&
        (!state->log_tick || now - state->log_tick >= 1000u)) {
        state->log_tick = now;
        log_line("butt-physics write person=\"%s\" local_step=(%.5f,%.5f,%.5f) gravity=(%.4f,%.4f,%.4f) gravity_sign=(left:%.1f/%.1f/%.1f,right:%.1f/%.1f/%.1f) inverted_blend=%.4f left=(%.3f,%.3f,%.3f) right=(%.3f,%.3f,%.3f) bone_translation=(%.5f,%.5f,%.5f;%.5f,%.5f,%.5f)",
                 person, local_step[0], local_step[1], local_step[2],
                 gravity_drive[0], gravity_drive[1], gravity_drive[2],
                 butt_physics_gravity_sign[0][0],
                 butt_physics_gravity_sign[0][1],
                 butt_physics_gravity_sign[0][2],
                 butt_physics_gravity_sign[1][0],
                 butt_physics_gravity_sign[1][1],
                 butt_physics_gravity_sign[1][2],
                 paired_bone_inverted_gravity_amount(gravity_drive),
                 state->rotation[0][0], state->rotation[0][1],
                 state->rotation[0][2], state->rotation[1][0],
                 state->rotation[1][1], state->rotation[1][2],
                 state->bone_translation[0][0],
                 state->bone_translation[0][1],
                 state->bone_translation[0][2],
                 state->bone_translation[1][0],
                 state->bone_translation[1][1],
                 state->bone_translation[1][2]);
    }
}

static void run_butt_physics(DWORD now)
{
    int i;
    if (!butt_physics_global_cfg.enabled || !engine_FindObjC) {
        for (i = 0; i < 4; i++) {
            body_profile_set_active_person_config(i);
            reset_butt_physics_person_state(
                i, &butt_physics_states[i], 1);
            restore_tk17_butt_inertia_for_person(i, now);
        }
        body_profile_set_active_person_config(-1);
        return;
    }
    for (i = 0; i < 4; i++) {
        body_profile_set_active_person_config(i);
        if (butt_physics_person_enabled(i)) {
            LONGLONG perf_start = physx_perf_counter();
            run_butt_physics_for_person(i, now);
            physx_perf_add(PHYSX_PERF_BUTT_ACTIVE, perf_start);
        } else {
            reset_butt_physics_person_state(
                i, &butt_physics_states[i], 1);
            restore_tk17_butt_inertia_for_person(i, now);
        }
    }
    body_profile_set_active_person_config(-1);
}

static int butt_physics_apply_all_outputs(int capture_animation)
{
    int applied = 0;
    int i;
    if (!butt_physics_global_cfg.enabled) return 0;
    for (i = 0; i < 4; i++) {
        breasts_physics_person_state_t *state =
            &butt_physics_states[i];
        body_profile_set_active_person_config(i);
        if (butt_physics_person_enabled(i) && state->initialized &&
            butt_physics_live_ownership_matches(i, state)) {
            applied += butt_physics_apply_output(
                state, 0, capture_animation);
        }
    }
    body_profile_set_active_person_config(-1);
    return applied;
}

static void run_butt_physics_late_ownership(DWORD now)
{
    int i;
    if (!butt_physics_global_cfg.enabled) {
        restore_tk17_butt_inertia_all(now);
        return;
    }
    for (i = 0; i < 4; i++) {
        breasts_physics_person_state_t *state =
            &butt_physics_states[i];
        body_profile_set_active_person_config(i);
        if (butt_physics_person_enabled(i) && state->initialized &&
            butt_physics_live_ownership_matches(i, state) &&
            suppress_tk17_butt_inertia_for_person(i, now)) {
            butt_physics_apply_output(state, 0, 0);
        } else if (!butt_physics_person_enabled(i)) {
            restore_tk17_butt_inertia_for_person(i, now);
        }
    }
    body_profile_set_active_person_config(-1);
}

static void reset_butt_physics_all(int restore_output)
{
    int i;
    for (i = 0; i < 4; i++) {
        body_profile_set_active_person_config(i);
        reset_butt_physics_person_state(
            i, &butt_physics_states[i], restore_output);
    }
    body_profile_set_active_person_config(-1);
    restore_tk17_butt_inertia_all(GetTickCount());
}
