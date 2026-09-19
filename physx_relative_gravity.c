/* Shared by body and add-on gravity. Learn room-down in the placement frame
   while the camera is quiet, then follow bone * inverse(TRS_group) even while
   the camera moves. Never assume the placement frame is world identity. */
static int gravity_normalize_basis(float rows[9])
{
    int row, col;
    for (row = 0; row < 3; row++) {
        float len = physx_vec3_len(&rows[row * 3]);
        if (!isfinite(len) || len < 0.000001f || len > 8.0f) return 0;
        for (col = 0; col < 3; col++) rows[row * 3 + col] /= len;
    }
    return fabsf(vec3_dot(rows, rows + 3)) < .28f &&
           fabsf(vec3_dot(rows, rows + 6)) < .28f &&
           fabsf(vec3_dot(rows + 3, rows + 6)) < .28f;
}

/* -1: unavailable basis (caller may use the guarded legacy path).
    0: handled but still warming. 1: a trusted direction is available. */
static int gravity_sample_relative(gravity_sample_t *sample,
    gravity_sample_t *reference_sample, const void *bone_raw, const void *trs_raw,
    const float bone_matrix[9], const float trs_matrix[9],
    const float gravity_view[3], DWORD now, float out[3])
{
    float inverse[9], relative[9], reference_basis[9];
    float reference_candidate[3], reference[3] = {0}, candidate[3];
    int offsets[3] = {physics_environment_cfg.gravity_horizontal_basis_offset,
        physics_environment_cfg.gravity_vertical_basis_offset,
        physics_environment_cfg.gravity_horizontal_secondary_basis_offset};
    int fallback[3] = {1, 2, 0};
    float signs[3] = {physics_environment_cfg.gravity_horizontal_basis_sign,
        physics_environment_cfg.gravity_vertical_basis_sign,
        physics_environment_cfg.gravity_horizontal_secondary_basis_sign};
    int i, reference_valid;
    /* A replacement bone can belong to a new pose/scene even when TRS has
       been reused. Reconfirm both sources before accepting its first force. */
    if (reference_sample->source != (uintptr_t)trs_raw ||
        sample->source != (uintptr_t)bone_raw) {
        memset(sample, 0, sizeof(*sample));
        memset(reference_sample, 0, sizeof(*reference_sample));
        sample->source = (uintptr_t)bone_raw;
        reference_sample->source = (uintptr_t)trs_raw;
    }
    if (!body_chain_mat3_inverse(trs_matrix, inverse)) return -1;
    body_chain_mat3_multiply(bone_matrix, inverse, relative);
    memcpy(reference_basis, trs_matrix, sizeof(reference_basis));
    if (!gravity_normalize_basis(relative) ||
        !gravity_normalize_basis(reference_basis)) return -1;
    for (i = 0; i < 3; i++)
        reference_candidate[i] = vec3_dot(gravity_view, &reference_basis[i * 3]);
    reference_valid = gravity_sample_live(reference_sample, trs_raw,
        reference_candidate, 1, now, reference);
    for (i = 0; i < 3; i++) {
        int row = offsets[i] == 0x078 ? 0 : offsets[i] == 0x088 ? 1 :
                  offsets[i] == 0x098 ? 2 : fallback[i];
        candidate[i] = vec3_dot(reference, &relative[row * 3]) * signs[i];
        if (!isfinite(candidate[i]) || fabsf(candidate[i]) > 4.0f)
            reference_valid = 0;
    }
    /* Keep frame confirmation and spike rejection; the common camera basis
       has cancelled, so camera-version changes need not restart this sample. */
    memset(out, 0, sizeof(float) * 3);
    return gravity_sample_update(sample, (uintptr_t)bone_raw, candidate,
        reference_valid, physx_simulation_serial, now, 0,
        captured_camera_inverse_valid, 0xffffffffu, 48u, out);
}

static int body_gravity_sample_live(const char *person, void **trs_cache,
    gravity_sample_t *sample, gravity_sample_t *reference_sample,
    void *bone_raw, const float gravity_view[3], const float candidate[3],
    int valid, DWORD now, float out[3])
{
    float bone[9], trs[9];
    void *trs_raw = *trs_cache;
    int offsets[3] = {physics_environment_cfg.gravity_horizontal_basis_offset,
        physics_environment_cfg.gravity_vertical_basis_offset,
        physics_environment_cfg.gravity_horizontal_secondary_basis_offset};
    int i;
    if (!physics_environment_cfg.gravity_dynamic_body_basis ||
        !physics_environment_cfg.body_chain_camera_relative_orientation ||
        !physics_environment_cfg.gravity_basis_camera_compensate ||
        !valid || !gravity_view)
        goto fallback;
    for (i = 0; i < 3; i++)
        if (offsets[i] != 0x078 && offsets[i] != 0x088 && offsets[i] != 0x098)
            goto fallback;
    if (!body_chain_read_mat3_rows(trs_raw, trs)) {
        char name[256];
        make_body_runtime_name(name, sizeof(name), person, "TRS_group");
        trs_raw = resolve_axis_map_raw(name);
        if (!body_chain_read_mat3_rows(trs_raw, trs)) {
            *trs_cache = NULL;
            goto fallback;
        }
        *trs_cache = trs_raw;
    }
    if (!body_chain_read_mat3_rows(bone_raw, bone)) goto fallback;
    /* Both body callers already normalized room gravity and projected it
       into view space for their fallback candidate. Reuse that same sample;
       do not cache it across simulation frames or engine traversal points. */
    {
        body_placement_cache_t *placement = body_placement_cache_for(person, trs_raw);
        if (placement && placement->gravity_revision && placement->gravity.trusted_valid &&
            sample->source == (uintptr_t)bone_raw &&
            reference_sample->source == (uintptr_t)trs_raw && reference_sample->trusted_valid &&
            memcmp(reference_sample->trusted,placement->gravity.trusted,sizeof(reference_sample->trusted))) {
            *reference_sample=placement->gravity;
            reference_sample->pending_valid=reference_sample->processed=reference_sample->accepted=0;
        }
        if (placement && !sample->trusted_valid && !reference_sample->trusted_valid &&
            placement->gravity.trusted_valid) {
            memset(sample, 0, sizeof(*sample));
            sample->source = (uintptr_t)bone_raw;
            *reference_sample = placement->gravity;
            reference_sample->pending_valid = reference_sample->processed = 0;
            reference_sample->accepted = 0;
            if (defaults_cfg.debug)
                log_line("physics-environment placement-reuse person=\"%s\" system=gravity note=\"same live body, generation and authored orientation; translation does not change down; fresh bone sample still required\"", person);
        }
        int available = gravity_sample_relative(sample, reference_sample,
            bone_raw, trs_raw, bone, trs, gravity_view, now, out);
        if (placement && reference_sample->accepted)
            placement->gravity = *reference_sample;
        if (available >= 0) return available;
    }
fallback:
    reference_sample->pending_valid = 0;
    return gravity_sample_live(sample, bone_raw, candidate, valid, now, out);
}

/* A confirmed room-down reference and fresh bone/TRS samples establish
   readiness without requiring a camera-dependent root POSITION to stop. */
static int body_gravity_relative_startup_ready(const char *person,
    body_chain_person_state_t *state, void *root_raw, DWORD now)
{
    float world[3], view[3], out[3], length;
    int i;
    if (!physics_environment_cfg.gravity_apply_to_body_chain ||
        !physics_environment_cfg.gravity_dynamic_body_basis ||
        !physics_environment_cfg.body_chain_camera_relative_orientation ||
        !physics_environment_cfg.gravity_basis_camera_compensate ||
        _stricmp(physics_environment_cfg.gravity_basis_node, "root")) return 0;
    length = physx_vec3_len(physics_environment_cfg.world_gravity);
    if (!isfinite(length) || length < .000001f) return 0;
    for (i = 0; i < 3; ++i) world[i] = physics_environment_cfg.world_gravity[i] / length;
    if (!camera_world_to_view_direction(world, view)) return 0;
    if (!body_gravity_sample_live(person, &state->camera_relative_trs_raw,
            &state->gravity_sample, &state->gravity_reference_sample,
            root_raw, view, world, 1, now, out)) return 0;
    return state->gravity_sample.accepted && state->gravity_reference_sample.trusted_valid &&
        state->gravity_reference_sample.source == (uintptr_t)state->camera_relative_trs_raw;
}
