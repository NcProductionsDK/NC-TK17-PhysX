/* Camera-safe world coordinates for body contacts. Relative bone motion uses
   the current skeleton's TRS frame, not the independently captured camera. */
static void body_collision_frame_update(body_chain_collider_person_state_t *state,
    const char *person, DWORD now)
{
    char name[192];
    void *raw;
    float view[9], inverse[9], origin[3], candidate[12] = {0}, camera[9];
    float offset[3];
    int i, row, col, valid;
    LONG generation = InterlockedCompareExchange(&named_node_generation, 0, 0);
    DWORD age = captured_camera_change_tick ? now - captured_camera_change_tick : 0xffffffffu;
    state->contact_frame_valid = 0;
    make_body_runtime_name(name, sizeof(name), person, "TRS_group");
    raw = resolve_axis_map_raw(name);
    valid = raw && body_chain_read_mat3_rows(raw, view) &&
        body_chain_mat3_inverse(view, inverse) &&
        body_collider_engine_pivot_view(person, "TRS_group", NULL, origin) &&
        body_chain_vec3_sane_limit(origin, 4096.f);
    if (valid && captured_camera_inverse_valid) {
        for (row = 0; row < 3; row++) for (col = 0; col < 3; col++)
            camera[row*3+col] = captured_camera_inverse[row*4+col];
        body_chain_mat3_multiply(view, camera, candidate);
        valid = camera_view_to_world_point(origin, candidate+9);
    }
    if (!collision_frame_sample_update(&state->contact_frame_sample,
            (uintptr_t)raw, (uint32_t)generation, candidate, valid,
            physx_simulation_serial, now, captured_camera_version,
            captured_camera_inverse_valid, age) || !valid) {
        if (defaults_cfg.debug && (!state->contact_frame_log_tick ||
            now - state->contact_frame_log_tick >= 1000u)) {
            state->contact_frame_log_tick = now;
            log_line("body collision frame person=\"%s\" available=0 source_valid=%d camera_valid=%d camera=%ld age_ms=%lu confirming=%d",
                person, valid, captured_camera_inverse_valid, captured_camera_version,
                (unsigned long)age, state->contact_frame_sample.pending_valid);
        }
        return;
    }
    body_chain_mat3_multiply(inverse, state->contact_frame_sample.trusted,
                            state->contact_view_to_world);
    if (!body_chain_mat3_inverse(state->contact_view_to_world,
                                state->contact_world_to_view)) return;
    body_chain_transform_row_vector3(origin, state->contact_view_to_world, offset);
    for (i = 0; i < 3; i++) state->contact_world_origin[i] =
        state->contact_frame_sample.trusted[9+i] - offset[i];
    state->contact_frame_valid = 1;
    if (defaults_cfg.debug && (!state->contact_frame_log_tick ||
        now - state->contact_frame_log_tick >= 1000u)) {
        float delta[3];
        for (i = 0; i < 3; i++) delta[i] = candidate[9+i] - state->contact_frame_sample.trusted[9+i];
        state->contact_frame_log_tick = now;
        log_line("body collision frame person=\"%s\" available=1 camera=%ld age_ms=%lu held=%d raw_placement_delta=%.6f trusted_origin=(%.5f,%.5f,%.5f)",
            person, captured_camera_version, (unsigned long)age,
            state->contact_frame_sample.held, physx_vec3_len(delta),
            state->contact_frame_sample.trusted[9], state->contact_frame_sample.trusted[10],
            state->contact_frame_sample.trusted[11]);
    }
}

static int body_collision_view_to_world(const body_chain_collider_person_state_t *state,
    const float view[3], float world[3])
{
    int a;
    if (!state || !state->contact_frame_valid) return 0;
    body_chain_transform_row_vector3(view, state->contact_view_to_world, world);
    for (a = 0; a < 3; a++) world[a] += state->contact_world_origin[a];
    return body_chain_vec3_sane_limit(world, 4096.f);
}
