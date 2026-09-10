/* Built-in root translation is stored in TK17 scene units.  This preserves
   the previous 300-degree response at a public scale of 0.1. */
#define BODY_CHAIN_TRANSLATION_RESPONSE 3000.0f
#define BODY_CHAIN_WIND_RESPONSE_DEGREES 10.0f

static void body_chain_apply_link_inertia(const body_chain_person_state_t *state,
    int joint,int axis,float *stiffness,float *damping)
{
    int a;
    if(!state->dynamics_valid || joint>=state->dynamics.segments) return;
    for(a=0;a<2;a++) if(axis==state->dynamics.axis[a]) {
        float inverse=state->dynamics.inverse_inertia[joint][a];
        *stiffness*=inverse;
        /* Keep the user's damping ratio while changing the natural period. */
        *damping*=sqrtf(inverse);
    }
}

static int body_chain_limit_total_rotation(const body_chain_physics_config_t *cfg,
    float value[3][3],float velocity[3][3],int link_count);

static void body_chain_shape_gravity(int person,body_chain_person_state_t *state,
    const body_chain_physics_config_t *cfg,DWORD now,float dt,
    const float configured[3][3],float target[3][3])
{
    float euler[3][3],shaped[3][3];int j,a;
    state->dynamics_gravity_active=0;
    if(!state->dynamics_valid || !physics_environment_cfg.gravity_apply_to_body_chain ||
       !state->gravity_probe_promoted) return;
    if(!body_chain_collider_states[person].ready ||
       !body_chain_collider_states[person].basis_valid) return;
    /* Geometric gravity uses the same sample protocol as the primary drive.
       Gather while held too, so both can confirm promptly after the camera stops. */
    {
        float candidate[3], trusted[3];
        int valid = !body_chain_camera_pivot_hold_active(now) &&
            room_collision_world_vector_to_body_local(&body_chain_collider_states[person],
                physics_environment_cfg.world_gravity, candidate);
        if (valid) {
            float length = physx_vec3_len(candidate);
            valid = length > .00001f && isfinite(length);
            if (valid) for (a=0;a<3;a++) candidate[a] /= length;
        }
        if (gravity_sample_live(&state->geometry_sample,state->root_raw,
                candidate,valid,now,trusted) && state->geometry_sample.accepted &&
            !state->gravity_camera_hold_active) {
            float response=physics_environment_cfg.gravity_response_ms*.001f;
            float alpha=response>0?1.0f-expf(-dt/response):1.0f;
            for(a=0;a<3;a++)
                state->dynamics_gravity_direction[a]=state->dynamics_gravity_valid?
                    state->dynamics_gravity_direction[a]+alpha*(trusted[a]-state->dynamics_gravity_direction[a]):trusted[a];
            state->dynamics_gravity_valid=1;
        }
    }
    if(!state->dynamics_gravity_valid) return;
    for(j=0;j<3;j++) for(a=0;a<3;a++)
        euler[j][a]=state->dynamics.reference.euler[j][a]+state->angle[j][a];
    if(!body_dynamics_gravity(&state->dynamics.reference,&state->dynamics,euler,
        state->dynamics_gravity_direction,configured,shaped)) return;
    for(j=0;j<state->dynamics.segments;j++) for(a=0;a<3;a++)
        target[j][a]=body_chain_clamp_link_axis_angle(cfg,j,a,
            target[j][a]+shaped[j][a]-configured[j][a]);
    body_chain_limit_total_rotation(cfg,target,NULL,state->dynamics.segments);
    state->dynamics_gravity_active=1;
}

static int body_chain_release_runtime_ownership_for_person(
    int person_index, int testicle_chain, const char *reason);
static int body_chain_runtime_suppress_penis_pose_tracks(
    int person_index, const char *person,
    body_chain_person_state_t *state);

static int body_chain_runtime_mode_active(void)
{
    return InterlockedCompareExchange(&body_chain_poseeditor_mode_active,
                                      0, 0) == 0;
}

static body_chain_person_state_t *body_chain_active_person_state(
    int person_index, int testicle_chain)
{
    if (person_index < 0 || person_index >= 4) return NULL;
    if (body_chain_runtime_mode_active()) {
        return testicle_chain
            ? &runtime_testicle_physics_states[person_index]
            : &runtime_body_chain_person_states[person_index];
    }
    return testicle_chain
        ? &testicle_physics_states[person_index]
        : &body_chain_person_states[person_index];
}

static body_chain_gravity_snapshot_t *body_chain_active_gravity_cache(
    int person_index, int testicle_chain)
{
    if (person_index < 0 || person_index >= 4) return NULL;
    if (body_chain_runtime_mode_active()) {
        return testicle_chain
            ? &runtime_testicle_room_gravity_cache[person_index]
            : &runtime_body_chain_room_gravity_cache[person_index];
    }
    return testicle_chain
        ? &testicle_room_gravity_cache[person_index]
        : &body_chain_room_gravity_cache[person_index];
}

static body_chain_axis_reference_cache_t *
body_chain_active_axis_reference_cache(int person_index)
{
    if (person_index < 0 || person_index >= 4) return NULL;
    return body_chain_runtime_mode_active()
        ? &runtime_body_chain_axis_reference_cache[person_index]
        : &poseeditor_body_chain_axis_reference_cache[person_index];
}

static body_chain_axis_reference_cache_t *
breasts_physics_active_axis_reference_cache(int person_index)
{
    if (person_index < 0 || person_index >= 4) return NULL;
    return body_chain_runtime_mode_active()
        ? &runtime_breasts_axis_reference_cache[person_index]
        : &poseeditor_breasts_axis_reference_cache[person_index];
}

/* TK17 publishes a named-node generation whenever scene objects are created
   or replaced.  That event makes axis validation immediate.  During a stable
   room, validate each cache periodically as a fail-safe, staggered across the
   eight body/breast person slots so resolution work cannot bunch into one
   rendered frame. */
#define BODY_CHAIN_AXIS_VALIDATION_MS 1000u
#define BODY_CHAIN_AXIS_VALIDATION_STAGGER_MS 125u

static int body_chain_axis_reference_validation_due(
    body_chain_axis_reference_cache_t *axis_cache,
    int validation_slot,
    DWORD now)
{
    LONG generation;
    DWORD stagger;
    if (!axis_cache) return 0;
    generation = InterlockedCompareExchange(&named_node_generation, 0, 0);
    if (axis_cache->node_generation == generation &&
        axis_cache->next_validation_tick &&
        (LONG)(now - axis_cache->next_validation_tick) < 0) {
        return 0;
    }
    if (validation_slot < 0) validation_slot = 0;
    if (validation_slot > 7) validation_slot = 7;
    stagger = (DWORD)validation_slot *
              BODY_CHAIN_AXIS_VALIDATION_STAGGER_MS;
    axis_cache->node_generation = generation;
    axis_cache->next_validation_tick =
        now + BODY_CHAIN_AXIS_VALIDATION_MS + stagger;
    return 1;
}

/* Capture the authored body-axis frame even while every body-physics feature
   is disabled.  Previously this baseline was first sampled by whichever of
   penis, testicle, or breast PhysX was enabled.  Enabling one later in an
   already-running room could therefore mistake the person's current animated
   orientation for the authored axes.  This observer is read-only: it resolves
   root/TRS_group, stores their camera-neutral relative basis, and never takes
   animation ownership or writes a transform. */
static void body_chain_prime_axis_reference_for_person(int person_index,
                                                       DWORD now)
{
    body_chain_axis_reference_cache_t *axis_cache;
    const char *person;
    char name[256];
    void *root_raw;
    void *trs_raw;
    float root_matrix[9];
    float trs_matrix[9];
    float trs_inverse[9];
    float basis[9];

    if (person_index < 0 || person_index >= 4 || !engine_FindObjC) return;
    axis_cache = body_chain_active_axis_reference_cache(person_index);
    person = body_chain_person_name(person_index);
    if (!axis_cache || !person) return;
    if (!body_chain_axis_reference_validation_due(
            axis_cache, person_index * 2, now)) {
        return;
    }

    make_body_runtime_name(name, sizeof(name), person, "root");
    root_raw = resolve_axis_map_raw(name);
    make_body_runtime_name(name, sizeof(name), person, "TRS_group");
    trs_raw = resolve_axis_map_raw(name);
    if (!root_raw || !trs_raw) return;

    /* A valid cache belongs to these exact live scene objects.  Do not
       replace it as the person subsequently moves or rotates. */
    if (axis_cache->valid &&
        axis_cache->root_raw == root_raw &&
        axis_cache->trs_raw == trs_raw) {
        return;
    }
    if (!body_chain_read_mat3_rows(root_raw, root_matrix) ||
        !body_chain_read_mat3_rows(trs_raw, trs_matrix) ||
        !body_chain_mat3_inverse(trs_matrix, trs_inverse)) {
        return;
    }
    body_chain_mat3_multiply(root_matrix, trs_inverse, basis);
    if (!body_chain_normalize_basis_rows(basis)) return;

    axis_cache->root_raw = root_raw;
    axis_cache->trs_raw = trs_raw;
    memcpy(axis_cache->basis, basis, sizeof(basis));
    axis_cache->valid = 1;
    if (defaults_cfg.debug) {
        log_line("body-chain-physics translation-axis reference primed person=\"%s\" source=\"room-observer\" root_raw=%p trs_raw=%p reference=(%.4f,%.4f,%.4f;%.4f,%.4f,%.4f;%.4f,%.4f,%.4f) tick=%lu note=\"available before penis, testicle, or breast PhysX activation\"",
                 person, root_raw, trs_raw,
                 basis[0], basis[1], basis[2],
                 basis[3], basis[4], basis[5],
                 basis[6], basis[7], basis[8],
                 (unsigned long)now);
    }
}

/* Breast motion is driven from the upper torso rather than the pelvis/root.
   Prime its authored spine basis independently so a later breast-physics
   activation cannot treat an already-posed torso as the neutral axes, and
   cannot replace the root cache shared by penis and testicle physics. */
static void breasts_physics_prime_axis_reference_for_person(
    int person_index, DWORD now)
{
    body_chain_axis_reference_cache_t *axis_cache;
    const char *person;
    char name[256];
    void *spine_raw;
    void *trs_raw;
    float spine_matrix[9];
    float trs_matrix[9];
    float trs_inverse[9];
    float basis[9];

    if (person_index < 0 || person_index >= 4 || !engine_FindObjC) return;
    axis_cache =
        breasts_physics_active_axis_reference_cache(person_index);
    person = body_chain_person_name(person_index);
    if (!axis_cache || !person) return;
    if (!body_chain_axis_reference_validation_due(
            axis_cache, person_index * 2 + 1, now)) {
        return;
    }

    make_body_runtime_name(name, sizeof(name), person, "spine_joint04");
    spine_raw = resolve_axis_map_raw(name);
    make_body_runtime_name(name, sizeof(name), person, "TRS_group");
    trs_raw = resolve_axis_map_raw(name);
    if (!spine_raw || !trs_raw) return;

    if (axis_cache->valid &&
        axis_cache->root_raw == spine_raw &&
        axis_cache->trs_raw == trs_raw) {
        return;
    }
    if (!body_chain_read_mat3_rows(spine_raw, spine_matrix) ||
        !body_chain_read_mat3_rows(trs_raw, trs_matrix) ||
        !body_chain_mat3_inverse(trs_matrix, trs_inverse)) {
        return;
    }
    body_chain_mat3_multiply(spine_matrix, trs_inverse, basis);
    if (!body_chain_normalize_basis_rows(basis)) return;

    axis_cache->root_raw = spine_raw;
    axis_cache->trs_raw = trs_raw;
    memcpy(axis_cache->basis, basis, sizeof(basis));
    axis_cache->valid = 1;
    if (defaults_cfg.debug) {
        log_line("breasts-physics translation-axis reference primed person=\"%s\" source=\"room-observer:spine_joint04\" spine_raw=%p trs_raw=%p reference=(%.4f,%.4f,%.4f;%.4f,%.4f,%.4f;%.4f,%.4f,%.4f) tick=%lu note=\"available before breast PhysX activation; isolated from penis/testicle root axes\"",
                 person, spine_raw, trs_raw,
                 basis[0], basis[1], basis[2],
                 basis[3], basis[4], basis[5],
                 basis[6], basis[7], basis[8],
                 (unsigned long)now);
    }
}

static void body_chain_prime_axis_references(DWORD now)
{
    int i;
    for (i = 0; i < 4; i++) {
        body_chain_prime_axis_reference_for_person(i, now);
        breasts_physics_prime_axis_reference_for_person(i, now);
    }
}

static runtime_body_chain_ownership_state_t *
body_chain_runtime_ownership_state(int person_index, int testicle_chain)
{
    if (person_index < 0 || person_index >= 4) return NULL;
    return testicle_chain
        ? &runtime_testicle_ownership_states[person_index]
        : &runtime_body_chain_ownership_states[person_index];
}

static void body_chain_refresh_runtime_penis_native_filter(void)
{
    int active = 0;
    int i;
    if (body_chain_runtime_mode_active()) {
        for (i = 0; i < 4; i++) {
            if (runtime_body_chain_ownership_states[i].ownership_active &&
                body_chain_physics_person_cfg[i].override_animation) {
                active = 1;
                break;
            }
        }
    }
    InterlockedExchange(&body_chain_runtime_penis_native_filter_active,
                        active ? 1 : 0);
}

static int body_chain_runtime_tjoint_layout_valid(void *object)
{
    float *row0;
    float *row1;
    float *row2;
    if (!object || is_nil_engine_object(NULL, object) ||
        !ptr_readable((BYTE*)object + 0x078, 0x030)) {
        return 0;
    }
    row0 = (float*)((BYTE*)object + 0x078);
    row1 = (float*)((BYTE*)object + 0x088);
    row2 = (float*)((BYTE*)object + 0x098);
    return body_chain_vec3_sane_limit(row0, 4.0f) &&
           body_chain_vec3_sane_limit(row1, 4.0f) &&
           body_chain_vec3_sane_limit(row2, 4.0f);
}

static int body_chain_resolve_runtime_animation_joints(
    const char *person, int testicle_chain, void *joint_raw_out[3])
{
    static const char *penis_joints[3] = {
        "penis_joint01", "penis_joint02", "penis_joint03"
    };
    static const char *testicle_joints[3] = {
        "testicles_joint01", "testicles_joint02", "testicles_jointEnd"
    };
    const char **names = testicle_chain ? testicle_joints : penis_joints;
    char runtime_name[256];
    int i;
    if (!person || !joint_raw_out) return 0;
    for (i = 0; i < 3; i++) {
        void *raw = NULL;
        void *object;
        make_body_runtime_name(runtime_name, sizeof(runtime_name),
                               person, names[i]);
        object = resolve_find_obj(runtime_name, &raw);
        if ((!object || is_nil_engine_object(raw, object)) &&
            captured_script_engine) {
            object = resolve_script_engine_obj(runtime_name, &raw);
        }
        if (!raw || !object || is_nil_engine_object(raw, object) ||
            !body_chain_runtime_tjoint_layout_valid(raw)) {
            return 0;
        }
        joint_raw_out[i] = raw;
    }
    return 1;
}

static int body_chain_sample_runtime_mapping(
    int person_index, int testicle_chain, void *source_joint_raw[3],
    DWORD now)
{
    runtime_body_chain_ownership_state_t *ownership =
        body_chain_runtime_ownership_state(person_index, testicle_chain);
    void *animation_joint_raw[3] = { NULL, NULL, NULL };
    const char *person = body_chain_person_name(person_index);
    int changed = 0;
    int i;
    if (!body_chain_runtime_mode_active() || !ownership ||
        !source_joint_raw ||
        !body_chain_resolve_runtime_animation_joints(
            person, testicle_chain, animation_joint_raw)) {
        if (ownership) {
            ownership->mapping_samples = 0;
            ownership->mapping_ready = 0;
            ownership->pose_valid = 0;
        }
        return 0;
    }
    for (i = 0; i < 3; i++) {
        if (!source_joint_raw[i] ||
            ownership->source_joint_raw[i] != source_joint_raw[i] ||
            ownership->animation_joint_raw[i] != animation_joint_raw[i]) {
            changed = 1;
        }
    }
    if (changed) {
        for (i = 0; i < 3; i++) {
            ownership->source_joint_raw[i] = source_joint_raw[i];
            ownership->animation_joint_raw[i] = animation_joint_raw[i];
        }
        ownership->mapping_samples = 1;
        ownership->mapping_first_tick = now;
        ownership->mapping_last_tick = now;
        ownership->mapping_ready = 0;
        ownership->pose_valid = 0;
        ownership->animation_value_valid_mask = 0;
        ownership->native_output_animation_valid_mask = 0;
        ownership->native_source_axis_animation_valid_mask = 0;
        ownership->ownership_logged = 0;
        ownership->runtime_writer_logged = 0;
        ownership->native_output_writer_logged = 0;
        ownership->native_source_axis_writer_logged = 0;
        return 0;
    }
    if (ownership->mapping_last_tick != now) {
        ownership->mapping_last_tick = now;
        if (ownership->mapping_samples < 3u) {
            ownership->mapping_samples++;
        }
    }
    if (ownership->mapping_samples >= 3u &&
        now - ownership->mapping_first_tick >= 32u) {
        ownership->mapping_ready = 1;
    }
    return ownership->mapping_ready;
}

static void body_chain_publish_runtime_pose(
    int person_index, int testicle_chain, float *output[3])
{
    runtime_body_chain_ownership_state_t *ownership =
        body_chain_runtime_ownership_state(person_index, testicle_chain);
    int i, axis;
    if (!body_chain_runtime_mode_active() || !ownership ||
        !ownership->mapping_ready || !output) {
        return;
    }
    for (i = 0; i < 3; i++) {
        if (!output[i] ||
            !body_chain_vec3_sane_limit(output[i], 720.0f)) {
            ownership->pose_valid = 0;
            return;
        }
        for (axis = 0; axis < 3; axis++) {
            ownership->rotation[i][axis] = output[i][axis];
        }
    }
    ownership->pose_valid = 1;
}

static int body_chain_face_down_translation_active(
    const body_chain_person_state_t *state)
{
    const float *gravity_drive;
    if (!state || !state->gravity_probe_promoted) return 0;
    gravity_drive = state->gravity_drive_filtered_valid ?
        state->gravity_drive_filtered : state->gravity_drive;

    /* Runtime captures show the owner's forward basis aligned toward world
       gravity at about -1.0 while face-down. Keep face-up/upright movement
       unchanged and apply the configured response only in this hemisphere. */
    return gravity_drive[0] < -0.5f;
}

static int body_chain_camera_neutral_pivot_step(
    const char *person,
    int person_index,
    body_chain_person_state_t *state,
    const float root_pivot_view[3],
    int root_offset,
    body_chain_axis_reference_cache_t *axis_cache_override,
    float out[3])
{
    float trs_basis[9];
    float trs_inverse[9];
    float trs_pivot_view[3];
    float relative_view[3];
    float relative_local[3];
    float local_step[3];
    float live_basis[9];
    float rest_inverse[9];
    float live_delta[9];
    float owner_local_step[3];
    float *trs_raw_position;
    float step_len;
    body_chain_axis_reference_cache_t *axis_cache;
    if (out) out[0] = out[1] = out[2] = 0.0f;
    if (!person || !state || !root_pivot_view || !out || root_offset < 0 ||
        !body_chain_read_mat3_rows(state->camera_relative_trs_raw,
                                   trs_basis) ||
        !body_chain_mat3_inverse(trs_basis, trs_inverse)) {
        if (state) state->root_prev_world_valid = 0;
        return 0;
    }
    if (!body_collider_engine_pivot_view(person, "TRS_group", NULL,
                                         trs_pivot_view)) {
        if (!ptr_readable((BYTE*)state->camera_relative_trs_raw + root_offset,
                          sizeof(float) * 3)) {
            state->root_prev_world_valid = 0;
            return 0;
        }
        trs_raw_position = (float*)((BYTE*)state->camera_relative_trs_raw +
                                    root_offset);
        trs_pivot_view[0] = trs_raw_position[0];
        trs_pivot_view[1] = trs_raw_position[1];
        trs_pivot_view[2] = trs_raw_position[2];
    }
    relative_view[0] = root_pivot_view[0] - trs_pivot_view[0];
    relative_view[1] = root_pivot_view[1] - trs_pivot_view[1];
    relative_view[2] = root_pivot_view[2] - trs_pivot_view[2];
    body_chain_transform_row_vector3(relative_view, trs_inverse,
                                     relative_local);
    if (!body_chain_vec3_sane_limit(relative_local, 64.0f)) {
        state->root_prev_world_valid = 0;
        state->root_translation_initialized = 0;
        return 0;
    }
    if (!state->camera_relative_live_current_valid) {
        state->root_prev_world_valid = 0;
        state->root_translation_initialized = 0;
        return 0;
    }
    memcpy(live_basis, state->camera_relative_live_current,
           sizeof(live_basis));
    if (!body_chain_normalize_basis_rows(live_basis)) {
        state->root_prev_world_valid = 0;
        state->root_translation_initialized = 0;
        return 0;
    }
    if (!state->root_prev_world_valid ||
        !state->root_translation_initialized ||
        state->root_translation_raw != state->root_raw) {
        int reused_axis_reference = 0;
        const char *axis_source = "current-skeleton";
        axis_cache = axis_cache_override ? axis_cache_override :
            body_chain_active_axis_reference_cache(person_index);
        memcpy(state->root_prev_world, relative_local,
               sizeof(relative_local));
        state->root_prev_world_valid = 1;
        state->root_translation_initialized = 1;
        state->root_translation_raw = state->root_raw;

        if (axis_cache && axis_cache->valid &&
            axis_cache->root_raw == state->root_raw &&
            axis_cache->trs_raw == state->camera_relative_trs_raw) {
            memcpy(state->root_translation_parent_rest,
                   axis_cache->basis, sizeof(axis_cache->basis));
            state->root_translation_parent_rest_valid = 1;
            state->root_translation_parent_rest_root_raw = state->root_raw;
            state->root_translation_parent_rest_trs_raw =
                state->camera_relative_trs_raw;
            reused_axis_reference = 1;
            axis_source = "person-shared-cache";
        } else if (state->root_translation_parent_rest_valid &&
                   state->root_translation_parent_rest_root_raw ==
                       state->root_raw &&
                   state->root_translation_parent_rest_trs_raw ==
                       state->camera_relative_trs_raw) {
            reused_axis_reference = 1;
            axis_source = "reactivation-cache";
        }
        if (!reused_axis_reference) {
            memcpy(state->root_translation_parent_rest, live_basis,
                   sizeof(live_basis));
            state->root_translation_parent_rest_valid = 1;
            state->root_translation_parent_rest_root_raw = state->root_raw;
            state->root_translation_parent_rest_trs_raw =
                state->camera_relative_trs_raw;
        }
        if (axis_cache) {
            axis_cache->root_raw = state->root_raw;
            axis_cache->trs_raw = state->camera_relative_trs_raw;
            memcpy(axis_cache->basis,
                   state->root_translation_parent_rest,
                   sizeof(axis_cache->basis));
            axis_cache->valid = 1;
        }
        if (defaults_cfg.debug) {
            log_line("body-chain-physics translation-axis reference person=\"%s\" source=\"%s\" root_raw=%p trs_raw=%p reference=(%.4f,%.4f,%.4f;%.4f,%.4f,%.4f;%.4f,%.4f,%.4f) note=\"off/on reactivation retains the authored body axes; live orientation continues updating relative to them\"",
                     person,
                     axis_source,
                     state->root_raw,
                     state->camera_relative_trs_raw,
                     state->root_translation_parent_rest[0],
                     state->root_translation_parent_rest[1],
                     state->root_translation_parent_rest[2],
                     state->root_translation_parent_rest[3],
                     state->root_translation_parent_rest[4],
                     state->root_translation_parent_rest[5],
                     state->root_translation_parent_rest[6],
                     state->root_translation_parent_rest[7],
                     state->root_translation_parent_rest[8]);
        }
        return 0;
    }
    local_step[0] = relative_local[0] - state->root_prev_world[0];
    local_step[1] = relative_local[1] - state->root_prev_world[1];
    local_step[2] = relative_local[2] - state->root_prev_world[2];
    memcpy(state->root_prev_world, relative_local, sizeof(relative_local));
    if (!body_chain_vec3_sane_limit(local_step, 8.0f)) return 0;
    step_len = physx_vec3_len(local_step);
    if (step_len > 4.0f) return 0;

    /* The pivot delta above is expressed in TRS_group space. Rotate that
       stable, camera-neutral sample by the owner's live orientation change
       since initialization so configured source axes remain body-local after
       the whole person is turned. This mirrors the working sidecar parent
       translation frame while retaining the initial pose as the authored
       axis baseline. */
    if (!body_chain_mat3_inverse(state->root_translation_parent_rest,
                                 rest_inverse)) {
        state->root_translation_initialized = 0;
        return 0;
    }
    body_chain_mat3_multiply(rest_inverse, live_basis, live_delta);
    if (!body_chain_normalize_basis_rows(live_delta)) {
        state->root_translation_initialized = 0;
        return 0;
    }
    owner_local_step[0] = vec3_dot(local_step, &live_delta[0]);
    owner_local_step[1] = vec3_dot(local_step, &live_delta[3]);
    owner_local_step[2] = vec3_dot(local_step, &live_delta[6]);
    if (!body_chain_vec3_sane_limit(owner_local_step, 8.0f)) return 0;
    out[0] = owner_local_step[0];
    out[1] = owner_local_step[1];
    out[2] = owner_local_step[2];
    return body_chain_vec3_sane_limit(out, 8.0f);
}

static void body_chain_build_mapped_drive(
    const body_chain_physics_config_t *cfg,
    float horizontal_step,
    float vertical_step,
    float depth_step,
    const float rotation_step[3],
    float gravity_horizontal,
    float gravity_vertical,
    float out[3])
{
    const float translation_step[3] = {
        horizontal_step, vertical_step, depth_step
    };
    float translation_drive[3] = { 0.0f, 0.0f, 0.0f };
    int channel;
    out[0] = 0.0f;
    out[1] = 0.0f;
    out[2] = 0.0f;
    if (!cfg) return;
    for (channel = 0; channel < 3; channel++) {
        int tail_axis = cfg->translation_tail_axis[channel];
        if (tail_axis >= 0 && tail_axis <= 2) {
            translation_drive[tail_axis] +=
                translation_step[channel] *
                cfg->translation_scale[channel] *
                BODY_CHAIN_TRANSLATION_RESPONSE;
        }
    }
    for (channel = 0; channel < 3; channel++) {
        out[channel] += translation_drive[channel];
    }
    for (channel = 0; channel < 3; channel++) {
        int tail_axis;
        tail_axis = cfg->rotation_tail_axis[channel];
        if (rotation_step && tail_axis >= 0 && tail_axis <= 2) {
            float value = rotation_step[cfg->rotation_source_axis[channel]];
            if (physx_absf(value) >= cfg->rotation_deadzone) {
                out[tail_axis] += value * cfg->rotation_scale[channel];
            }
        }
    }
    out[physics_environment_cfg.gravity_horizontal_tail_axis] +=
        gravity_horizontal;
    out[physics_environment_cfg.gravity_vertical_tail_axis] +=
        gravity_vertical + cfg->gravity_angle;
}

/* Curve the combined primary world-gravity amount while preserving its
   direction.  The recovery band prevents slightly imperfect engine basis
   readings near a completed orientation from weakening full gravity. */
static void body_chain_apply_gravity_curve(const float drive[3],
                                           float horizontal_curve,
                                           float vertical_curve,
                                           float out[3])
{
    const float recovery_start = 0.90f;
    const float recovery_end = 0.98f;
    float magnitude;
    float magnitude_sq;
    float curve;
    float shaped;
    float scale;

    out[0] = drive[0];
    out[1] = drive[1];
    out[2] = drive[2];
    if (!(horizontal_curve > 0.0f) || !(vertical_curve > 0.0f) ||
        (physx_absf(horizontal_curve - 1.0f) <= 0.000001f &&
         physx_absf(vertical_curve - 1.0f) <= 0.000001f)) {
        return;
    }

    magnitude_sq = drive[0] * drive[0] + drive[1] * drive[1];
    magnitude = (float)sqrt((double)magnitude_sq);
    if (magnitude <= 0.000001f || magnitude >= recovery_end) {
        return;
    }

    /* Select the curve by gravity direction, then scale both primary
       channels together so different curve values cannot skew direction. */
    curve = (horizontal_curve * drive[0] * drive[0] +
             vertical_curve * drive[1] * drive[1]) / magnitude_sq;

    shaped = (float)pow((double)physx_clampf(magnitude, 0.0f, 1.0f),
                        (double)curve);
    if (magnitude > recovery_start) {
        float blend = (magnitude - recovery_start) /
                      (recovery_end - recovery_start);
        blend = physx_clampf(blend, 0.0f, 1.0f);
        blend = blend * blend * (3.0f - 2.0f * blend);
        shaped += (magnitude - shaped) * blend;
    }
    scale = shaped / magnitude;
    out[0] = drive[0] * scale;
    out[1] = drive[1] * scale;
}

/* Keep the configured wind fixed in room/world space. TK17's live joint
   bases are model-view matrices, so first rotate the room direction into
   view space and then project it onto the live parent rows. Do not cancel
   TRS_group here: that also cancels a pose location's authored placement
   rotation and makes wind follow the previous pose instead of the room. */
static int body_chain_room_wind_channels(
    const body_chain_physics_config_t *cfg,
    body_chain_person_state_t *state,
    void *parent_raw,
    const char *person,
    const char *system_name,
    DWORD now,
    float out[3])
{
    float world_direction[3];
    float view_direction[3];
    float parent_matrix[9];
    float basis_h[3];
    float basis_v[3];
    float basis_depth[3];
    float strength;
    unsigned int generation;
    int h_off;
    int v_off;
    int depth_off;
    out[0] = out[1] = out[2] = 0.0f;
    if (!cfg || !state || !parent_raw || !person || !system_name ||
        camera_contamination_test_isolating_forces() ||
        !cfg->wind_enabled || cfg->wind_scale <= 0.000001f ||
        !room_wind_direction(world_direction) ||
        !camera_world_to_view_direction(world_direction, view_direction) ||
        !body_chain_read_mat3_rows(parent_raw, parent_matrix) ||
        !body_chain_normalize_basis_rows(parent_matrix)) {
        return 0;
    }
    h_off = physics_environment_cfg.gravity_horizontal_basis_offset;
    v_off = physics_environment_cfg.gravity_vertical_basis_offset;
    depth_off =
        physics_environment_cfg.gravity_horizontal_secondary_basis_offset;
    if (h_off != 0x078 && h_off != 0x088 && h_off != 0x098) {
        return 0;
    }
    if (v_off != 0x078 && v_off != 0x088 && v_off != 0x098) {
        return 0;
    }
    if (depth_off != 0x078 && depth_off != 0x088 &&
        depth_off != 0x098) {
        return 0;
    }
    memcpy(basis_h, &parent_matrix[((h_off - 0x078) / 0x10) * 3],
           sizeof(basis_h));
    memcpy(basis_v, &parent_matrix[((v_off - 0x078) / 0x10) * 3],
           sizeof(basis_v));
    memcpy(basis_depth,
           &parent_matrix[((depth_off - 0x078) / 0x10) * 3],
           sizeof(basis_depth));
    strength = room_wind_body_strength(person, system_name,
                                       cfg->wind_scale, now) *
               BODY_CHAIN_WIND_RESPONSE_DEGREES;
    if (physx_absf(strength) <= 0.000001f) return 0;
    out[0] = vec3_dot(view_direction, basis_h) *
             physics_environment_cfg.gravity_horizontal_basis_sign *
             strength;
    out[1] = vec3_dot(view_direction, basis_v) *
             physics_environment_cfg.gravity_vertical_basis_sign *
             strength;
    out[2] = vec3_dot(view_direction, basis_depth) *
             physics_environment_cfg.gravity_horizontal_secondary_basis_sign *
             strength;
    if (!body_chain_vec3_sane_limit(out, 200.0f)) {
        out[0] = out[1] = out[2] = 0.0f;
        return 0;
    }
    generation = room_wind_generation();
    if (state->wind_generation != generation) {
        state->wind_generation = generation;
        state->wind_logged = 0;
    }
    if (defaults_cfg.debug && !state->wind_logged) {
        state->wind_logged = 1;
        log_line("body room wind active person=\"%s\" system=\"%s\" wind_scale=%.3f parent_raw=%p room_direction=(%.4f,%.4f,%.4f) view_direction=(%.4f,%.4f,%.4f) parent_local_channels=(%.4f,%.4f,%.4f)",
                 person, system_name, cfg->wind_scale, parent_raw,
                 world_direction[0], world_direction[1], world_direction[2],
                 view_direction[0], view_direction[1], view_direction[2],
                 out[0], out[1], out[2]);
    }
    return 1;
}

static void body_chain_add_room_wind_target(
    const body_chain_physics_config_t *cfg,
    const float wind_channels[3],
    const float output_signs[3],
    float out[3])
{
    float mapped[3] = { 0.0f, 0.0f, 0.0f };
    int channel;
    int axis;
    if (!cfg || !wind_channels || !out) return;

    /* Each body rig has authored rotation axes that do not necessarily match
       an add-on chain. Keep room/world projection shared, but make the final
       force-channel -> rotation-axis conversion explicitly configurable. */
    for (channel = 0; channel < 3; channel++) {
        int source_axis = cfg->wind_source_axis[channel];
        int tail_axis = cfg->wind_tail_axis[channel];
        if (source_axis < 0 || source_axis > 2 ||
            tail_axis < 0 || tail_axis > 2) continue;
        mapped[tail_axis] += wind_channels[source_axis] *
                             cfg->wind_axis_scale[channel];
    }

    for (axis = 0; axis < 3; axis++) {
        out[axis] += mapped[axis] *
            (output_signs ? output_signs[axis] : 1.0f);
    }
}

/* The axial gravity channel is negative while upright and positive while
   inverted. Smoothstep keeps upright/sideways neutral and avoids a sudden
   force as the body crosses the halfway orientation. */
static float body_chain_inverted_gravity_amount(
    const float gravity_drive[3])
{
    float amount;
    if (!gravity_drive) return 0.0f;
    amount = physx_clampf(gravity_drive[2], 0.0f, 1.0f);
    return amount * amount * (3.0f - 2.0f * amount);
}

/* Paired breast/butt solvers capture their own neutral gravity reference.
   Their axial channel therefore runs from 0 upright, through 1 sideways, to
   2 upside down. Remove the first half so their inverted response stays
   neutral through upright and sideways orientations. */
static float paired_bone_inverted_gravity_amount(
    const float gravity_drive[3])
{
    float amount;
    if (!gravity_drive) return 0.0f;
    amount = physx_clampf(gravity_drive[2] - 1.0f, 0.0f, 1.0f);
    return amount * amount * (3.0f - 2.0f * amount);
}

/* Individual joint limits cannot stop a hierarchical chain from curling:
   three legal same-direction rotations can still add up to an implausible
   whole-chain arc. Limit the combined bend while preserving the configured
   per-joint shape. When limiting live state, remove only velocity that keeps
   pushing the total farther through the boundary. */
static int body_chain_limit_total_rotation(
    const body_chain_physics_config_t *cfg,
    float value[3][3],
    float velocity[3][3],
    int link_count)
{
    int horizontal_axis;
    int vertical_axis;
    int twist_axis;
    int i;
    int limited = 0;
    float total_h = 0.0f;
    float total_v = 0.0f;
    float bend_length;

    if (!cfg || !value || link_count < 1 || link_count > 3) return 0;
    horizontal_axis = cfg->horizontal_output_axis;
    vertical_axis = cfg->vertical_output_axis;
    if (horizontal_axis < 0 || horizontal_axis > 2 ||
        vertical_axis < 0 || vertical_axis > 2 ||
        horizontal_axis == vertical_axis) {
        return 0;
    }
    twist_axis = 3 - horizontal_axis - vertical_axis;

    for (i = 0; i < link_count; i++) {
        total_h += value[i][horizontal_axis];
        total_v += value[i][vertical_axis];
    }
    bend_length = (float)sqrt((double)(total_h * total_h +
                                       total_v * total_v));
    if (cfg->chain_total_bend_max > 0.0001f &&
        bend_length > cfg->chain_total_bend_max) {
        float scale = cfg->chain_total_bend_max / bend_length;
        float normal_h = total_h / bend_length;
        float normal_v = total_v / bend_length;
        for (i = 0; i < link_count; i++) {
            value[i][horizontal_axis] *= scale;
            value[i][vertical_axis] *= scale;
        }
        if (velocity) {
            float total_velocity_h = 0.0f;
            float total_velocity_v = 0.0f;
            float outward_velocity;
            for (i = 0; i < link_count; i++) {
                total_velocity_h += velocity[i][horizontal_axis];
                total_velocity_v += velocity[i][vertical_axis];
            }
            outward_velocity = total_velocity_h * normal_h +
                               total_velocity_v * normal_v;
            if (outward_velocity > 0.0f) {
                float per_link = outward_velocity / (float)link_count;
                for (i = 0; i < link_count; i++) {
                    velocity[i][horizontal_axis] -= normal_h * per_link;
                    velocity[i][vertical_axis] -= normal_v * per_link;
                }
            }
        }
        limited = 1;
    }

    if (twist_axis >= 0 && twist_axis <= 2 &&
        cfg->chain_total_twist_max > 0.0001f) {
        float total_twist = 0.0f;
        for (i = 0; i < link_count; i++) {
            total_twist += value[i][twist_axis];
        }
        if (physx_absf(total_twist) > cfg->chain_total_twist_max) {
            float scale = cfg->chain_total_twist_max /
                          physx_absf(total_twist);
            float direction = total_twist < 0.0f ? -1.0f : 1.0f;
            for (i = 0; i < link_count; i++) {
                value[i][twist_axis] *= scale;
            }
            if (velocity) {
                float total_velocity = 0.0f;
                float outward_velocity;
                for (i = 0; i < link_count; i++) {
                    total_velocity += velocity[i][twist_axis];
                }
                outward_velocity = total_velocity * direction;
                if (outward_velocity > 0.0f) {
                    float per_link = direction * outward_velocity /
                                     (float)link_count;
                    for (i = 0; i < link_count; i++) {
                        velocity[i][twist_axis] -= per_link;
                    }
                }
            }
            limited = 1;
        }
    }
    return limited;
}

static int body_chain_gravity_effective_settle_ms(body_chain_person_state_t *state,
                                                  DWORD now,
                                                  int *reactivation_fast)
{
    const int fast_settle_ms = 300;
    int settle_ms = physics_environment_cfg.gravity_probe_settle_ms;
    DWORD camera_age_ms;
    DWORD quiet_ms;
    if (reactivation_fast) *reactivation_fast = 0;
    if (!state || !state->gravity_probe_reactivation_ready ||
        settle_ms <= fast_settle_ms) {
        return settle_ms;
    }

    quiet_ms = (DWORD)physics_environment_cfg.body_chain_camera_quarantine_ms;
    if (quiet_ms < (DWORD)physics_environment_cfg.gravity_probe_camera_quiet_ms) {
        quiet_ms = (DWORD)physics_environment_cfg.gravity_probe_camera_quiet_ms;
    }
    camera_age_ms = captured_camera_change_tick ?
        (now - captured_camera_change_tick) : 0xffffffffu;
    if (captured_camera_inverse_valid && camera_age_ms < quiet_ms) {
        return settle_ms;
    }

    if (reactivation_fast) *reactivation_fast = 1;
    return fast_settle_ms;
}

static void body_chain_gravity_mark_reactivation_ready(body_chain_person_state_t *state,
                                                       void *root_raw,
                                                       DWORD now)
{
    if (!state) return;
    state->gravity_probe_reactivation_ready = 1;
    state->gravity_probe_reactivation_tick = now;
    state->gravity_probe_reactivation_root_raw = root_raw;
}

static int body_chain_runtime_gravity_basis_coherent(
    const char *person,
    body_chain_person_state_t *state,
    void *root_raw,
    DWORD now)
{
    float basis_h[3];
    float basis_v[3];
    float basis_dot;
    void *basis_raw = root_raw;
    DWORD camera_quiet_ms;
    DWORD camera_age_ms;
    int h_off;
    int v_off;

    if (!body_chain_runtime_mode_active() || !root_raw) return 0;

    camera_quiet_ms =
        (DWORD)physics_environment_cfg.gravity_probe_camera_quiet_ms;
    if (camera_quiet_ms <
        (DWORD)physics_environment_cfg.body_chain_camera_quarantine_ms) {
        camera_quiet_ms =
            (DWORD)physics_environment_cfg.body_chain_camera_quarantine_ms;
    }
    camera_age_ms = captured_camera_change_tick ?
        (now - captured_camera_change_tick) : 0xffffffffu;
    if (captured_camera_inverse_valid && camera_age_ms < camera_quiet_ms) {
        return 0;
    }

    h_off = physics_environment_cfg.gravity_horizontal_basis_offset;
    v_off = physics_environment_cfg.gravity_vertical_basis_offset;
    if (physics_environment_cfg.gravity_dynamic_body_basis &&
        state &&
        state->camera_relative_live_current_valid &&
        _stricmp(physics_environment_cfg.gravity_basis_node, "root") == 0 &&
        body_chain_relative_basis_vector(state, h_off, basis_h) &&
        body_chain_relative_basis_vector(state, v_off, basis_v)) {
        basis_dot = physx_absf(vec3_dot(basis_h, basis_v));
        return sane_probe_float(basis_dot) && basis_dot <= 0.35f;
    }

    if (_stricmp(physics_environment_cfg.gravity_basis_node, "root") != 0) {
        char basis_name[256];
        make_body_runtime_name(basis_name, sizeof(basis_name),
                               person,
                               physics_environment_cfg.gravity_basis_node);
        basis_raw = resolve_axis_map_raw(basis_name);
        if (!basis_raw) basis_raw = root_raw;
    }
    if (!read_normalized_basis_vector(basis_raw, h_off, basis_h) ||
        !read_normalized_basis_vector(basis_raw, v_off, basis_v)) {
        return 0;
    }
    basis_dot = physx_absf(vec3_dot(basis_h, basis_v));
    return sane_probe_float(basis_dot) && basis_dot <= 0.35f;
}

static void run_body_chain_gravity_probe(const char *person,
                                         body_chain_person_state_t *state,
                                         void *root_raw,
                                         const float root[3],
                                         DWORD now,
                                         body_chain_gravity_snapshot_t *room_cache,
                                         const char *system_name)
{
    float root_delta[3];
    float move_delta[3];
    float root_world[3] = { 0.0f, 0.0f, 0.0f };
    float move_len;
    float root_len;
    int settle_ms;
    int reactivation_fast = 0;
    int root_nonzero;
    int root_world_valid;
    int capture_initial_gravity_reference = 0;
    int runtime_mode;
    int runtime_basis_coherent = 0;
    if (!physics_environment_cfg.world_gravity_probe ||
        !person || !state || !root) {
        return;
    }
    runtime_mode = body_chain_runtime_mode_active();
    if (!state->init_tick) {
        state->init_tick = now;
    }
    root_len = physx_vec3_len(root);
    if (root_len > physics_environment_cfg.gravity_probe_motion_epsilon &&
        gravity_response_trace_due(now, &state->gravity_response_trace_tick)) {
        float view[3], basis[3], live[3] = {0};
        int channel, valid = camera_world_to_view_direction(
            physics_environment_cfg.world_gravity, view);
        int offsets[3] = {physics_environment_cfg.gravity_horizontal_basis_offset,
            physics_environment_cfg.gravity_vertical_basis_offset,
            physics_environment_cfg.gravity_horizontal_secondary_basis_offset};
        for (channel = 0; channel < 3 && valid; channel++) {
            valid = read_normalized_basis_vector(root_raw, offsets[channel], basis);
            if (valid) live[channel] = vec3_dot(view, basis);
        }
        log_line("gravity responsiveness body system=%s person=%s promoted=%d held=%d camera=%ld rotation=%ld camera_age=%lu rotation_age=%lu root_len=%.6f live_root_valid=%d live_root_unsigned=(%.5f,%.5f,%.5f) target=(%.5f,%.5f,%.5f) filtered=(%.5f,%.5f,%.5f) geometry_valid=%d geometry_direction=(%.5f,%.5f,%.5f) sample_reason=%d sample_counts=(accepted=%lu,camera=%lu,waiting=%lu)",
            system_name, person, state->gravity_probe_promoted, state->gravity_camera_hold_active,
            captured_camera_version, captured_camera_rotation_version,
            captured_camera_change_tick ? (unsigned long)(now-captured_camera_change_tick) : 0xffffffffUL,
            captured_camera_rotation_change_tick ? (unsigned long)(now-captured_camera_rotation_change_tick) : 0xffffffffUL,
            root_len, valid, live[0],live[1],live[2],state->gravity_drive[0],state->gravity_drive[1],state->gravity_drive[2],
            state->gravity_drive_filtered[0],state->gravity_drive_filtered[1],state->gravity_drive_filtered[2],
            state->dynamics_gravity_valid,state->dynamics_gravity_direction[0],state->dynamics_gravity_direction[1],state->dynamics_gravity_direction[2],
            state->gravity_sample.reason,(unsigned long)state->gravity_sample.accepted_count,
            (unsigned long)state->gravity_sample.camera_count,(unsigned long)state->gravity_sample.waiting_count);
    }
    root_nonzero =
        root_len > physics_environment_cfg.gravity_probe_motion_epsilon;
    root_world_valid = 0;
    if (physics_environment_cfg.gravity_probe_require_nonzero_root &&
        !root_nonzero) {
        if (!state->gravity_probe_log_tick ||
            (int)(now - state->gravity_probe_log_tick) >=
            physics_environment_cfg.gravity_probe_log_ms) {
            state->gravity_probe_log_tick = now;
            log_line("physics-environment gravity-probe waiting person=\"%s\" elapsed_ms=%lu root=(%.5f,%.5f,%.5f) root_len=%.6f epsilon=%.6f require_nonzero_root=%d world_gravity=(%.5f,%.5f,%.5f) note=\"waiting for non-placeholder root; read-only; no chain write\"",
                     person,
                     (unsigned long)(now - state->init_tick),
                     root[0], root[1], root[2],
                     root_len,
                     physics_environment_cfg.gravity_probe_motion_epsilon,
                     physics_environment_cfg.gravity_probe_require_nonzero_root,
                     physics_environment_cfg.world_gravity[0],
                     physics_environment_cfg.world_gravity[1],
                     physics_environment_cfg.world_gravity[2]);
        }
        state->gravity_probe_captured = 0;
        state->gravity_probe_promoted = 0;
        state->gravity_probe_sampled = 0;
        state->gravity_probe_stable_tick = now;
        state->gravity_probe_candidate_tick = 0;
        state->gravity_probe_candidate_camera_version = 0;
        reset_body_chain_gravity_state(state);
        state->gravity_probe_prev_root[0] = root[0];
        state->gravity_probe_prev_root[1] = root[1];
        state->gravity_probe_prev_root[2] = root[2];
        state->gravity_probe_last_root[0] = root[0];
        state->gravity_probe_last_root[1] = root[1];
        state->gravity_probe_last_root[2] = root[2];
        if (root_world_valid) {
            body_chain_copy_vec3(state->gravity_probe_prev_root_world, root_world);
            body_chain_copy_vec3(state->gravity_probe_last_root_world, root_world);
            state->gravity_probe_world_valid = 1;
        } else {
            state->gravity_probe_world_valid = 0;
        }
        return;
    }
    settle_ms = body_chain_gravity_effective_settle_ms(
        state, now, &reactivation_fast);
    if (runtime_mode) {
        runtime_basis_coherent =
            body_chain_runtime_gravity_basis_coherent(
                person, state, root_raw, now);
    }
    if (state->gravity_probe_captured) {
        root_delta[0] = root[0] - state->gravity_probe_root[0];
        root_delta[1] = root[1] - state->gravity_probe_root[1];
        root_delta[2] = root[2] - state->gravity_probe_root[2];
        move_len = physx_vec3_len(root_delta);
        if (!runtime_mode &&
            (!physics_environment_cfg.gravity_apply_to_body_chain ||
             !state->gravity_probe_promoted) &&
            move_len > physics_environment_cfg.gravity_probe_invalidate_epsilon) {
            log_line("physics-environment gravity-probe invalidated person=\"%s\" root=(%.5f,%.5f,%.5f) old_ref=(%.5f,%.5f,%.5f) root_delta=(%.5f,%.5f,%.5f) delta_len=%.6f invalidate_epsilon=%.6f note=\"reference changed after capture; waiting for fresh stable reference; read-only; no chain write\"",
                     person,
                     root[0], root[1], root[2],
                     state->gravity_probe_root[0],
                     state->gravity_probe_root[1],
                     state->gravity_probe_root[2],
                     root_delta[0], root_delta[1], root_delta[2],
                     move_len,
                     physics_environment_cfg.gravity_probe_invalidate_epsilon);
            state->gravity_probe_captured = 0;
            state->gravity_probe_promoted = 0;
            state->gravity_probe_sampled = 0;
            state->gravity_probe_stable_tick = now;
            state->gravity_probe_candidate_tick = 0;
            state->gravity_probe_candidate_camera_version = 0;
            state->gravity_probe_log_tick = now;
            reset_body_chain_gravity_state(state);
            state->gravity_probe_prev_root[0] = root[0];
            state->gravity_probe_prev_root[1] = root[1];
            state->gravity_probe_prev_root[2] = root[2];
            state->gravity_probe_last_root[0] = root[0];
            state->gravity_probe_last_root[1] = root[1];
            state->gravity_probe_last_root[2] = root[2];
            if (root_world_valid) {
                body_chain_copy_vec3(state->gravity_probe_prev_root_world, root_world);
                body_chain_copy_vec3(state->gravity_probe_last_root_world, root_world);
                state->gravity_probe_world_valid = 1;
            } else {
                state->gravity_probe_world_valid = 0;
            }
            return;
        }
    }
    if (physics_environment_cfg.gravity_apply_to_body_chain &&
        state->gravity_probe_promoted) {
        goto compute_gravity_drive;
    }
    if (!state->gravity_probe_sampled) {
        state->gravity_probe_sampled = 1;
        state->gravity_probe_stable_tick = now;
        state->gravity_probe_prev_root[0] = root[0];
        state->gravity_probe_prev_root[1] = root[1];
        state->gravity_probe_prev_root[2] = root[2];
        if (root_world_valid) {
            body_chain_copy_vec3(state->gravity_probe_prev_root_world, root_world);
            state->gravity_probe_world_valid = 1;
        } else {
            state->gravity_probe_world_valid = 0;
        }
        if (!state->gravity_probe_log_tick ||
            (int)(now - state->gravity_probe_log_tick) >=
            physics_environment_cfg.gravity_probe_log_ms) {
            state->gravity_probe_log_tick = now;
            log_line("physics-environment gravity-probe waiting person=\"%s\" elapsed_ms=%lu root=(%.5f,%.5f,%.5f) root_len=%.6f settle_ms=%d epsilon=%.6f note=\"first non-placeholder root sample; waiting for stability; read-only\"",
                     person,
                     (unsigned long)(now - state->init_tick),
                     root[0], root[1], root[2],
                     root_len,
                     settle_ms,
                     physics_environment_cfg.gravity_probe_motion_epsilon);
        }
        return;
    }
    if (root_world_valid && state->gravity_probe_world_valid) {
        move_delta[0] = root_world[0] - state->gravity_probe_prev_root_world[0];
        move_delta[1] = root_world[1] - state->gravity_probe_prev_root_world[1];
        move_delta[2] = root_world[2] - state->gravity_probe_prev_root_world[2];
    } else {
        move_delta[0] = root[0] - state->gravity_probe_prev_root[0];
        move_delta[1] = root[1] - state->gravity_probe_prev_root[1];
        move_delta[2] = root[2] - state->gravity_probe_prev_root[2];
    }
    move_len = physx_vec3_len(move_delta);
    if (runtime_mode && !runtime_basis_coherent) {
        state->gravity_probe_stable_tick = now;
        state->gravity_probe_prev_root[0] = root[0];
        state->gravity_probe_prev_root[1] = root[1];
        state->gravity_probe_prev_root[2] = root[2];
        if (root_world_valid) {
            body_chain_copy_vec3(state->gravity_probe_prev_root_world,
                                 root_world);
            state->gravity_probe_world_valid = 1;
        } else {
            state->gravity_probe_world_valid = 0;
        }
        if (!state->gravity_probe_log_tick ||
            (int)(now - state->gravity_probe_log_tick) >=
            physics_environment_cfg.gravity_probe_log_ms) {
            state->gravity_probe_log_tick = now;
            log_line("physics-environment gravity-probe waiting person=\"%s\" elapsed_ms=%lu root=(%.5f,%.5f,%.5f) settle_ms=%d note=\"runtime root may animate, but a coherent camera-quiet body basis is not available yet; read-only; no chain write\"",
                     person,
                     (unsigned long)(now - state->init_tick),
                     root[0], root[1], root[2],
                     settle_ms);
        }
        return;
    }
    if (!runtime_mode &&
        move_len > physics_environment_cfg.gravity_probe_motion_epsilon) {
        state->gravity_probe_stable_tick = now;
        state->gravity_probe_prev_root[0] = root[0];
        state->gravity_probe_prev_root[1] = root[1];
        state->gravity_probe_prev_root[2] = root[2];
        if (root_world_valid) {
            body_chain_copy_vec3(state->gravity_probe_prev_root_world, root_world);
            state->gravity_probe_world_valid = 1;
        } else {
            state->gravity_probe_world_valid = 0;
        }
        if (!state->gravity_probe_log_tick ||
            (int)(now - state->gravity_probe_log_tick) >=
            physics_environment_cfg.gravity_probe_log_ms) {
            state->gravity_probe_log_tick = now;
            log_line("physics-environment gravity-probe waiting person=\"%s\" elapsed_ms=%lu root=(%.5f,%.5f,%.5f) root_move_delta=(%.6f,%.6f,%.6f) move_len=%.6f settle_ms=%d epsilon=%.6f note=\"root still changing; read-only; no chain write\"",
                     person,
                     (unsigned long)(now - state->init_tick),
                     root[0], root[1], root[2],
                     move_delta[0], move_delta[1], move_delta[2],
                     move_len,
                     settle_ms,
                     physics_environment_cfg.gravity_probe_motion_epsilon);
        }
        return;
    }
    if (runtime_mode &&
        move_len > physics_environment_cfg.gravity_probe_motion_epsilon) {
        /* Runtime animation is allowed to translate the owner continuously.
           A valid, camera-quiet body basis is the calibration signal here;
           root translation alone must not keep PhysX gated forever. */
        state->gravity_probe_prev_root[0] = root[0];
        state->gravity_probe_prev_root[1] = root[1];
        state->gravity_probe_prev_root[2] = root[2];
        if (root_world_valid) {
            body_chain_copy_vec3(state->gravity_probe_prev_root_world,
                                 root_world);
            state->gravity_probe_world_valid = 1;
        } else {
            state->gravity_probe_world_valid = 0;
        }
    }
    if ((int)(now - state->gravity_probe_stable_tick) <
        settle_ms) {
        if (!state->gravity_probe_log_tick ||
            (int)(now - state->gravity_probe_log_tick) >=
            physics_environment_cfg.gravity_probe_log_ms) {
            state->gravity_probe_log_tick = now;
            log_line("physics-environment gravity-probe waiting person=\"%s\" elapsed_ms=%lu stable_ms=%lu settle_ms=%d root=(%.5f,%.5f,%.5f) move_len=%.6f epsilon=%.6f note=\"root quiet but not long enough; read-only\"",
                     person,
                     (unsigned long)(now - state->init_tick),
                     (unsigned long)(now - state->gravity_probe_stable_tick),
                     settle_ms,
                     root[0], root[1], root[2],
                     move_len,
                     physics_environment_cfg.gravity_probe_motion_epsilon);
        }
        return;
    }
    if (!state->gravity_probe_captured) {
        state->gravity_probe_captured = 1;
        state->gravity_probe_promoted = 1;
        state->gravity_probe_candidate_tick = now;
        state->gravity_probe_candidate_camera_version = captured_camera_version;
        state->gravity_probe_root[0] = root[0];
        state->gravity_probe_root[1] = root[1];
        state->gravity_probe_root[2] = root[2];
        state->gravity_probe_last_root[0] = root[0];
        state->gravity_probe_last_root[1] = root[1];
        state->gravity_probe_last_root[2] = root[2];
        if (root_world_valid) {
            body_chain_copy_vec3(state->gravity_probe_root_world, root_world);
            body_chain_copy_vec3(state->gravity_probe_last_root_world, root_world);
            state->gravity_probe_world_valid = 1;
        } else {
            state->gravity_probe_world_valid = 0;
        }
        state->camera_seen_version = captured_camera_version;
        state->gravity_camera_hold_active = 0;
        state->gravity_probe_log_tick = now;
        log_line("physics-environment gravity-probe promoted person=\"%s\" elapsed_ms=%lu stable_ms=%lu settle_ms=%d confirm_ms=%d root_ref=(%.5f,%.5f,%.5f) root_len=%.6f world_gravity=(%.5f,%.5f,%.5f) root_offset=0x%03x note=\"stable body baseline promoted; camera movement is ignored by body-chain simulation\"",
                 person,
                 (unsigned long)(now - state->init_tick),
                 (unsigned long)(now - state->gravity_probe_stable_tick),
                 settle_ms,
                 physics_environment_cfg.gravity_probe_confirm_ms,
                 state->gravity_probe_root[0],
                 state->gravity_probe_root[1],
                 state->gravity_probe_root[2],
                 root_len,
                 physics_environment_cfg.world_gravity[0],
                 physics_environment_cfg.world_gravity[1],
                 physics_environment_cfg.world_gravity[2],
                 body_chain_physics_cfg.root_offset);
        if (reactivation_fast && defaults_cfg.debug) {
            log_line("physics-environment gravity-probe reactivation-fast person=\"%s\" settle_ms=%d previous_root_raw=%p current_root_raw=%p note=\"prior trusted baseline allows a shorter camera-quiet reactivation settle\"",
                     person,
                     settle_ms,
                     state->gravity_probe_reactivation_root_raw,
                     root_raw);
        }
        body_chain_gravity_mark_reactivation_ready(state, root_raw, now);
        capture_initial_gravity_reference = 1;
        goto compute_gravity_drive;
    }
    if (!state->gravity_probe_promoted) {
        if ((int)(now - state->gravity_probe_candidate_tick) <
            physics_environment_cfg.gravity_probe_confirm_ms) {
            if (!state->gravity_probe_log_tick ||
                (int)(now - state->gravity_probe_log_tick) >=
                physics_environment_cfg.gravity_probe_log_ms) {
                state->gravity_probe_log_tick = now;
                log_line("physics-environment gravity-probe confirming person=\"%s\" elapsed_ms=%lu confirm_ms=%d confirm_elapsed_ms=%lu root=(%.5f,%.5f,%.5f) root_ref=(%.5f,%.5f,%.5f) note=\"candidate still valid; read-only; no chain write\"",
                         person,
                         (unsigned long)(now - state->init_tick),
                         physics_environment_cfg.gravity_probe_confirm_ms,
                         (unsigned long)(now - state->gravity_probe_candidate_tick),
                         root[0], root[1], root[2],
                         state->gravity_probe_root[0],
                         state->gravity_probe_root[1],
                         state->gravity_probe_root[2]);
            }
            return;
        }
        state->gravity_probe_promoted = 1;
        state->gravity_probe_log_tick = now;
        log_line("physics-environment gravity-probe promoted person=\"%s\" elapsed_ms=%lu confirm_ms=%d root_ref=(%.5f,%.5f,%.5f) world_gravity=(%.5f,%.5f,%.5f) note=\"confirmed baseline ready for future gravity; read-only; no chain write\"",
                 person,
                 (unsigned long)(now - state->init_tick),
                 physics_environment_cfg.gravity_probe_confirm_ms,
                 state->gravity_probe_root[0],
                 state->gravity_probe_root[1],
                 state->gravity_probe_root[2],
                 physics_environment_cfg.world_gravity[0],
                 physics_environment_cfg.world_gravity[1],
                 physics_environment_cfg.world_gravity[2]);
        body_chain_gravity_mark_reactivation_ready(state, root_raw, now);
        return;
    }
compute_gravity_drive:
    root_delta[0] = root[0] - state->gravity_probe_root[0];
    root_delta[1] = root[1] - state->gravity_probe_root[1];
    root_delta[2] = root[2] - state->gravity_probe_root[2];
    move_len = physx_vec3_len(root_delta);
    if (!physics_environment_cfg.gravity_apply_to_body_chain &&
        move_len > physics_environment_cfg.gravity_probe_invalidate_epsilon) {
        log_line("physics-environment gravity-probe invalidated person=\"%s\" root=(%.5f,%.5f,%.5f) old_ref=(%.5f,%.5f,%.5f) root_delta=(%.5f,%.5f,%.5f) delta_len=%.6f invalidate_epsilon=%.6f note=\"reference changed after capture; waiting for fresh stable reference; read-only; no chain write\"",
                 person,
                 root[0], root[1], root[2],
                 state->gravity_probe_root[0],
                 state->gravity_probe_root[1],
                 state->gravity_probe_root[2],
                 root_delta[0], root_delta[1], root_delta[2],
                 move_len,
                 physics_environment_cfg.gravity_probe_invalidate_epsilon);
        state->gravity_probe_captured = 0;
        state->gravity_probe_promoted = 0;
        state->gravity_probe_sampled = 0;
        state->gravity_probe_stable_tick = now;
        state->gravity_probe_candidate_tick = 0;
        state->gravity_probe_candidate_camera_version = 0;
        state->gravity_probe_log_tick = now;
        reset_body_chain_gravity_state(state);
        state->gravity_probe_prev_root[0] = root[0];
        state->gravity_probe_prev_root[1] = root[1];
        state->gravity_probe_prev_root[2] = root[2];
        state->gravity_probe_last_root[0] = root[0];
        state->gravity_probe_last_root[1] = root[1];
        state->gravity_probe_last_root[2] = root[2];
        if (root_world_valid) {
            body_chain_copy_vec3(state->gravity_probe_prev_root_world, root_world);
            body_chain_copy_vec3(state->gravity_probe_last_root_world, root_world);
            state->gravity_probe_world_valid = 1;
        } else {
            state->gravity_probe_world_valid = 0;
        }
        return;
    }
    /* Gravity direction is accepted below, after its basis is projected.
       No timer-only release or view-space root-motion heuristic. */
    state->gravity_camera_hold_active = 0;
    {
        float gravity_len = physx_vec3_len(physics_environment_cfg.world_gravity);
        float gravity_n[3] = { 0.0f, -1.0f, 0.0f };
        float gravity_basis_n[3] = { 0.0f, -1.0f, 0.0f };
        float gravity_h;
        float gravity_v;
        float gravity_h_secondary = 0.0f;
        float chain_gravity_h;
        float chain_gravity_v;
        float root_ref_h;
        float root_ref_v;
        float basis_h[3] = { 0.0f, 0.0f, 0.0f };
        float basis_v[3] = { 0.0f, 0.0f, 0.0f };
        float basis_h_secondary[3] = { 0.0f, 0.0f, 0.0f };
        float basis_h_raw[3] = { 0.0f, 0.0f, 0.0f };
        float basis_v_raw[3] = { 0.0f, 0.0f, 0.0f };
        float basis_h_secondary_raw[3] = { 0.0f, 0.0f, 0.0f };
        void *basis_raw = root_raw;
        int dynamic_basis_used = 0;
        int gravity_camera_compensated = 0;
        int camera_free_basis_available =
            physics_environment_cfg.gravity_dynamic_body_basis &&
            physics_environment_cfg.body_chain_camera_relative_orientation &&
            state->camera_relative_live_current_valid &&
            _stricmp(physics_environment_cfg.gravity_basis_node, "root") == 0;
        if (gravity_len > 0.000001f) {
            gravity_n[0] = physics_environment_cfg.world_gravity[0] / gravity_len;
            gravity_n[1] = physics_environment_cfg.world_gravity[1] / gravity_len;
            gravity_n[2] = physics_environment_cfg.world_gravity[2] / gravity_len;
        }
        gravity_basis_n[0] = gravity_n[0];
        gravity_basis_n[1] = gravity_n[1];
        gravity_basis_n[2] = gravity_n[2];
        if (physics_environment_cfg.gravity_basis_camera_compensate &&
            camera_world_to_view_direction(gravity_n, gravity_basis_n)) {
            gravity_camera_compensated = 1;
        }
        chain_gravity_h =
            gravity_n[0] * body_chain_physics_cfg.horizontal_source_vector[0] +
            gravity_n[1] * body_chain_physics_cfg.horizontal_source_vector[1] +
            gravity_n[2] * body_chain_physics_cfg.horizontal_source_vector[2];
        chain_gravity_v =
            gravity_n[0] * body_chain_physics_cfg.vertical_source_vector[0] +
            gravity_n[1] * body_chain_physics_cfg.vertical_source_vector[1] +
            gravity_n[2] * body_chain_physics_cfg.vertical_source_vector[2];
        gravity_h =
            gravity_n[0] * physics_environment_cfg.gravity_horizontal_source_vector[0] +
            gravity_n[1] * physics_environment_cfg.gravity_horizontal_source_vector[1] +
            gravity_n[2] * physics_environment_cfg.gravity_horizontal_source_vector[2];
        gravity_v =
            gravity_n[0] * physics_environment_cfg.gravity_vertical_source_vector[0] +
            gravity_n[1] * physics_environment_cfg.gravity_vertical_source_vector[1] +
            gravity_n[2] * physics_environment_cfg.gravity_vertical_source_vector[2];
        if (physics_environment_cfg.gravity_dynamic_body_basis) {
            int h_off = physics_environment_cfg.gravity_horizontal_basis_offset;
            int v_off = physics_environment_cfg.gravity_vertical_basis_offset;
            int hs_off = physics_environment_cfg.gravity_horizontal_secondary_basis_offset;
            if (_stricmp(physics_environment_cfg.gravity_basis_node,
                         "root") != 0) {
                char basis_name[256];
                make_body_runtime_name(basis_name, sizeof(basis_name),
                                       person,
                                       physics_environment_cfg.gravity_basis_node);
                basis_raw = resolve_axis_map_raw(basis_name);
            }
            /* Prefer the same true-world projection used by room wind:
               world direction -> current view -> live model-view basis.
               The old root * inverse(TRS_group) path removed pose-location
               rotation, so gravity and wind occupied different frames. */
            if (gravity_camera_compensated &&
                read_normalized_basis_vector(basis_raw, h_off, basis_h_raw) &&
                read_normalized_basis_vector(basis_raw, v_off, basis_v_raw)) {
                basis_h[0] = basis_h_raw[0];
                basis_h[1] = basis_h_raw[1];
                basis_h[2] = basis_h_raw[2];
                basis_v[0] = basis_v_raw[0];
                basis_v[1] = basis_v_raw[1];
                basis_v[2] = basis_v_raw[2];
                gravity_h = vec3_dot(gravity_basis_n, basis_h) *
                            physics_environment_cfg.gravity_horizontal_basis_sign;
                gravity_v = vec3_dot(gravity_basis_n, basis_v) *
                            physics_environment_cfg.gravity_vertical_basis_sign;
                if (read_normalized_basis_vector(
                        basis_raw, hs_off, basis_h_secondary_raw)) {
                    basis_h_secondary[0] = basis_h_secondary_raw[0];
                    basis_h_secondary[1] = basis_h_secondary_raw[1];
                    basis_h_secondary[2] = basis_h_secondary_raw[2];
                    gravity_h_secondary =
                        vec3_dot(gravity_basis_n, basis_h_secondary) *
                        physics_environment_cfg.gravity_horizontal_secondary_basis_sign;
                }
                dynamic_basis_used = 1;
            } else if (camera_free_basis_available &&
                body_chain_relative_basis_vector(state, h_off, basis_h_raw) &&
                body_chain_relative_basis_vector(state, v_off, basis_v_raw)) {
                basis_h[0] = basis_h_raw[0];
                basis_h[1] = basis_h_raw[1];
                basis_h[2] = basis_h_raw[2];
                basis_v[0] = basis_v_raw[0];
                basis_v[1] = basis_v_raw[1];
                basis_v[2] = basis_v_raw[2];
                gravity_h = vec3_dot(gravity_n, basis_h) *
                            physics_environment_cfg.gravity_horizontal_basis_sign;
                gravity_v = vec3_dot(gravity_n, basis_v) *
                            physics_environment_cfg.gravity_vertical_basis_sign;
                if (body_chain_relative_basis_vector(state, hs_off, basis_h_secondary_raw)) {
                    basis_h_secondary[0] = basis_h_secondary_raw[0];
                    basis_h_secondary[1] = basis_h_secondary_raw[1];
                    basis_h_secondary[2] = basis_h_secondary_raw[2];
                    gravity_h_secondary =
                        vec3_dot(gravity_n, basis_h_secondary) *
                        physics_environment_cfg.gravity_horizontal_secondary_basis_sign;
                }
                dynamic_basis_used = 2;
                gravity_camera_compensated = 0;
            } else {
            if (read_normalized_basis_vector(basis_raw, h_off, basis_h_raw) &&
                read_normalized_basis_vector(basis_raw, v_off, basis_v_raw)) {
                basis_h[0] = basis_h_raw[0];
                basis_h[1] = basis_h_raw[1];
                basis_h[2] = basis_h_raw[2];
                basis_v[0] = basis_v_raw[0];
                basis_v[1] = basis_v_raw[1];
                basis_v[2] = basis_v_raw[2];
                gravity_h = vec3_dot(gravity_basis_n, basis_h) *
                            physics_environment_cfg.gravity_horizontal_basis_sign;
                gravity_v = vec3_dot(gravity_basis_n, basis_v) *
                            physics_environment_cfg.gravity_vertical_basis_sign;
                if (read_normalized_basis_vector(basis_raw, hs_off, basis_h_secondary_raw)) {
                    basis_h_secondary[0] = basis_h_secondary_raw[0];
                    basis_h_secondary[1] = basis_h_secondary_raw[1];
                    basis_h_secondary[2] = basis_h_secondary_raw[2];
                    gravity_h_secondary =
                        vec3_dot(gravity_basis_n, basis_h_secondary) *
                        physics_environment_cfg.gravity_horizontal_secondary_basis_sign;
                }
                dynamic_basis_used = 1;
            }
            }
        }
        {
            float candidate[3] = {gravity_h, gravity_v, gravity_h_secondary};
            float trusted[3];
            int valid = !physics_environment_cfg.gravity_dynamic_body_basis ||
                        (dynamic_basis_used == 1 && gravity_camera_compensated) ||
                        (dynamic_basis_used != 0 &&
                         !physics_environment_cfg.gravity_basis_camera_compensate);
            int available = gravity_sample_live(&state->gravity_sample, basis_raw,
                candidate, valid, now, trusted);
            state->gravity_camera_hold_active = !state->gravity_sample.accepted;
            if (!available || state->gravity_camera_hold_active) return;
            gravity_h = trusted[0]; gravity_v = trusted[1]; gravity_h_secondary = trusted[2];
        }
        if (physics_environment_cfg.gravity_apply_to_body_chain &&
            physics_environment_cfg.gravity_zero_at_start &&
            state->gravity_probe_promoted &&
            !state->gravity_drive_ref_valid) {
            state->gravity_drive_ref[0] = gravity_h;
            state->gravity_drive_ref[1] = gravity_v;
            state->gravity_drive_ref[2] = gravity_h_secondary;
            state->gravity_drive_ref_valid = 1;
            log_line("physics-environment gravity-reference captured person=\"%s\" gravity_ref=(h=%.6f,v=%.6f,hs=%.6f) basis_node=\"%s\" basis_raw=%p basis_offsets=(h=0x%03x,v=0x%03x,hs=0x%03x) atomic_with_promotion=%d note=\"initial stable pose permanently becomes neutral; live gravity uses current-minus-reference\"",
                     person,
                     state->gravity_drive_ref[0],
                     state->gravity_drive_ref[1],
                     state->gravity_drive_ref[2],
                     physics_environment_cfg.gravity_basis_node,
                     basis_raw,
                     physics_environment_cfg.gravity_horizontal_basis_offset,
                     physics_environment_cfg.gravity_vertical_basis_offset,
                     physics_environment_cfg.gravity_horizontal_secondary_basis_offset,
                     capture_initial_gravity_reference);
            body_chain_cache_room_gravity_snapshot(
                room_cache, state, root_raw, person, system_name);
        }
        if (physics_environment_cfg.gravity_apply_to_body_chain &&
            physics_environment_cfg.gravity_zero_at_start &&
            state->gravity_probe_promoted &&
            state->gravity_drive_ref_valid) {
            float gravity_raw_direction[3];
            float gravity_direction_drive[3];
            gravity_raw_direction[0] = gravity_h;
            gravity_raw_direction[1] = gravity_v;
            gravity_raw_direction[2] = gravity_h_secondary;
            body_gravity_direction_drive(gravity_raw_direction,
                                         state->gravity_drive_ref,
                                         state->gravity_drive_filtered_valid ?
                                             state->gravity_drive_filtered :
                                             state->gravity_drive,
                                         gravity_direction_drive);
            state->gravity_drive[0] = gravity_direction_drive[0];
            state->gravity_drive[1] = gravity_direction_drive[1];
            state->gravity_drive[2] = gravity_direction_drive[2];
        } else {
            if (!physics_environment_cfg.gravity_zero_at_start) {
                if (state->gravity_drive_ref_valid)
                    reset_body_chain_gravity_basis_gate(state);
                state->gravity_drive_ref_valid = 0;
                state->gravity_drive_ref[0] = 0.0f;
                state->gravity_drive_ref[1] = 0.0f;
                state->gravity_drive_ref[2] = 0.0f;
            }
            state->gravity_drive[0] = gravity_h;
            state->gravity_drive[1] = gravity_v;
            state->gravity_drive[2] = gravity_h_secondary;
        }
        state->camera_seen_version = captured_camera_version;
        state->gravity_probe_last_root[0] = root[0];
        state->gravity_probe_last_root[1] = root[1];
        state->gravity_probe_last_root[2] = root[2];
        if (root_world_valid) {
            body_chain_copy_vec3(state->gravity_probe_last_root_world, root_world);
            state->gravity_probe_world_valid = 1;
        } else {
            state->gravity_probe_world_valid = 0;
        }
        if ((int)(now - state->gravity_probe_log_tick) <
            physics_environment_cfg.gravity_probe_log_ms) {
            return;
        }
        state->gravity_probe_log_tick = now;
        root_ref_h =
            root_delta[0] * body_chain_physics_cfg.horizontal_source_vector[0] +
            root_delta[1] * body_chain_physics_cfg.horizontal_source_vector[1] +
            root_delta[2] * body_chain_physics_cfg.horizontal_source_vector[2];
        root_ref_v =
            root_delta[0] * body_chain_physics_cfg.vertical_source_vector[0] +
            root_delta[1] * body_chain_physics_cfg.vertical_source_vector[1] +
            root_delta[2] * body_chain_physics_cfg.vertical_source_vector[2];
        if (defaults_cfg.debug) {
            log_line("physics-environment gravity-math person=\"%s\" root=(%.5f,%.5f,%.5f) root_ref=(%.5f,%.5f,%.5f) root_delta=(%.5f,%.5f,%.5f) root_delta_len=%.6f mapped_root_delta=(h=%.6f,v=%.6f) world_gravity=(%.5f,%.5f,%.5f) gravity_normalized=(%.5f,%.5f,%.5f) basis_gravity=(%.5f,%.5f,%.5f) basis_camera_compensated=%d axis_projection=(x=%.6f,y=%.6f,z=%.6f) chain_projected_gravity=(h=%.6f,v=%.6f) gravity_raw=(h=%.6f,v=%.6f,hs=%.6f) gravity_ref=(h=%.6f,v=%.6f,hs=%.6f,valid=%d) gravity_drive=(h=%.6f,v=%.6f,hs=%.6f) gravity_zero_at_start=%d dynamic_basis=%d basis_node=\"%s\" basis_raw=%p basis_offsets=(h=0x%03x,v=0x%03x,hs=0x%03x) basis_h_raw=(%.5f,%.5f,%.5f) basis_v_raw=(%.5f,%.5f,%.5f) basis_h=(%.5f,%.5f,%.5f) basis_v=(%.5f,%.5f,%.5f) gravity_horizontal_source=(%.3f,%.3f,%.3f) gravity_vertical_source=(%.3f,%.3f,%.3f) chain_horizontal_source=(%.3f,%.3f,%.3f) chain_vertical_source=(%.3f,%.3f,%.3f) note=\"log throttled\"",
                     person,
                     root[0], root[1], root[2],
                     state->gravity_probe_root[0],
                     state->gravity_probe_root[1],
                     state->gravity_probe_root[2],
                     root_delta[0], root_delta[1], root_delta[2],
                     move_len,
                     root_ref_h, root_ref_v,
                     physics_environment_cfg.world_gravity[0],
                     physics_environment_cfg.world_gravity[1],
                     physics_environment_cfg.world_gravity[2],
                     gravity_n[0], gravity_n[1], gravity_n[2],
                     gravity_basis_n[0], gravity_basis_n[1], gravity_basis_n[2],
                     gravity_camera_compensated,
                     gravity_n[0], gravity_n[1], gravity_n[2],
                     chain_gravity_h, chain_gravity_v,
                     gravity_h, gravity_v, gravity_h_secondary,
                     state->gravity_drive_ref[0],
                     state->gravity_drive_ref[1],
                     state->gravity_drive_ref[2],
                     state->gravity_drive_ref_valid,
                     state->gravity_drive[0],
                     state->gravity_drive[1],
                     state->gravity_drive[2],
                     physics_environment_cfg.gravity_zero_at_start,
                     dynamic_basis_used,
                     physics_environment_cfg.gravity_basis_node,
                     basis_raw,
                     physics_environment_cfg.gravity_horizontal_basis_offset,
                     physics_environment_cfg.gravity_vertical_basis_offset,
                     physics_environment_cfg.gravity_horizontal_secondary_basis_offset,
                     dynamic_basis_used ? basis_h_raw[0] : 0.0f,
                     dynamic_basis_used ? basis_h_raw[1] : 0.0f,
                     dynamic_basis_used ? basis_h_raw[2] : 0.0f,
                     dynamic_basis_used ? basis_v_raw[0] : 0.0f,
                     dynamic_basis_used ? basis_v_raw[1] : 0.0f,
                     dynamic_basis_used ? basis_v_raw[2] : 0.0f,
                     dynamic_basis_used ? basis_h[0] : 0.0f,
                     dynamic_basis_used ? basis_h[1] : 0.0f,
                     dynamic_basis_used ? basis_h[2] : 0.0f,
                     dynamic_basis_used ? basis_v[0] : 0.0f,
                     dynamic_basis_used ? basis_v[1] : 0.0f,
                     dynamic_basis_used ? basis_v[2] : 0.0f,
                     physics_environment_cfg.gravity_horizontal_source_vector[0],
                     physics_environment_cfg.gravity_horizontal_source_vector[1],
                     physics_environment_cfg.gravity_horizontal_source_vector[2],
                     physics_environment_cfg.gravity_vertical_source_vector[0],
                     physics_environment_cfg.gravity_vertical_source_vector[1],
                     physics_environment_cfg.gravity_vertical_source_vector[2],
                     body_chain_physics_cfg.horizontal_source_vector[0],
                     body_chain_physics_cfg.horizontal_source_vector[1],
                     body_chain_physics_cfg.horizontal_source_vector[2],
                     body_chain_physics_cfg.vertical_source_vector[0],
                     body_chain_physics_cfg.vertical_source_vector[1],
                     body_chain_physics_cfg.vertical_source_vector[2]);
        }
    }
}

#define BODY_CHAIN_REACTIVATION_COLLISION_GRACE_MS 1400u

static DWORD body_chain_reactivation_collision_until[4];
static DWORD body_chain_reactivation_collision_log_tick[4];

static void clear_body_chain_collision_handoff_state(
    body_chain_person_state_t *state)
{
    if (!state) return;
    clear_body_chain_collision_for_test(state);
    state->collision_prev_chain_points_ready = 0;
    memset(state->collision_prev_chain_points, 0,
           sizeof(state->collision_prev_chain_points));
}

static void begin_body_chain_reactivation_collision_grace(
    int person_index, DWORD now, const char *reason)
{
    body_chain_person_state_t *state;
    if (person_index < 0 || person_index >= 4) return;
    state = body_chain_active_person_state(person_index, 0);
    clear_body_chain_collision_handoff_state(state);
    clear_body_chain_prev_collider_for_person(person_index);
    body_chain_reactivation_collision_until[person_index] =
        now + BODY_CHAIN_REACTIVATION_COLLISION_GRACE_MS;
    body_chain_reactivation_collision_log_tick[person_index] = 0;
    log_line("body-chain-physics activation-reset person=\"%s\" reason=\"%s\" grace_ms=%lu note=\"cleared stale velocity/contact handoff before re-enabled penis collision follows fresh TK17 live bones\"",
             body_chain_person_name(person_index),
             reason ? reason : "reactivation",
             (unsigned long)BODY_CHAIN_REACTIVATION_COLLISION_GRACE_MS);
}

static int body_chain_reactivation_collision_grace_active(
    int person_index, body_chain_person_state_t *state, DWORD now)
{
    DWORD until_tick;
    if (person_index < 0 || person_index >= 4) return 0;
    until_tick = body_chain_reactivation_collision_until[person_index];
    if (!until_tick || now >= until_tick) return 0;
    clear_body_chain_collision_handoff_state(state);
    if (!body_chain_reactivation_collision_log_tick[person_index] ||
        now - body_chain_reactivation_collision_log_tick[person_index] >= 500u) {
        body_chain_reactivation_collision_log_tick[person_index] = now;
        log_line("body-chain-physics collision-warmup person=\"%s\" remaining_ms=%lu note=\"penis simulation is active, but collider response is briefly muted after re-enable so live pivots can settle without a ghost impulse\"",
                 body_chain_person_name(person_index),
                 (unsigned long)(until_tick - now));
    }
    return 1;
}

static void run_body_chain_physics_for_person(int person_index, DWORD now)
{
    const char *person = body_chain_person_name(person_index);
    int runtime_mode = body_chain_runtime_mode_active();
    body_chain_person_state_t *state =
        body_chain_active_person_state(person_index, 0);
    void *root_raw;
    void *joint_raw[3];
    float *root;
    float *out[3];
    float root_step[3];
    float root_drive_position[3];
    float camera_neutral_root_step[3] = { 0.0f, 0.0f, 0.0f };
    float camera_relative_delta[9] = { 0.0f };
    float parent_rotation_step[3] = { 0.0f, 0.0f, 0.0f };
    float pre_physx_output[3][3] = { { 0.0f } };
    float camera_relative_energy = 0.0f;
    float camera_relative_h_step = 0.0f;
    float camera_relative_v_step = 0.0f;
    float camera_relative_d_step = 0.0f;
    float h_step;
    float v_step;
    float d_step = 0.0f;
    float translation_drive_step[3] = { 0.0f, 0.0f, 0.0f };
    float wind_channels[3] = { 0.0f, 0.0f, 0.0f };
    float inverted_gravity_target = 0.0f;
    float dt;
    DWORD elapsed_ms;
    int cache_hot = 0;
    int camera_relative_step_valid = 0;
    int camera_relative_root_drive = 0;
    int camera_relative_prediction_valid = 0;
    int camera_neutral_translation_valid = 0;
    int root_drive_untrusted = 0;
    int face_down_translation = 0;
    int wind_active = 0;
    int chain_target_limited = 0;
    int chain_state_limited = 0;
    int camera_test_isolation = camera_contamination_test_isolating_forces();
    int reactivation_collision_grace = 0;
    int pre_physx_output_captured = 0;
    int runtime_mapping_ready = 1;
    int i, axis;
    LONGLONG perf_section_start;
    LONGLONG perf_setup_start;
    if (!state) return;
    /* Match the other body solvers: a definitely invisible person slot has
       no output to simulate, and an unresolved live slot is retried at a
       short cadence instead of performing four failed named lookups every
       rendered frame. Visibility changes and explicit reactivation still
       bypass the retry delay immediately. */
    if (!state->initialized &&
        poseedit_scene_person_visible(person_index) == 0) {
        return;
    }
    if (!state->initialized && state->resolve_retry_tick &&
        now - state->resolve_retry_tick <
            (DWORD)PENIS_PHYSICS_MISSING_RETRY_MS) {
        return;
    }
    if (state->last_tick &&
        now - state->last_tick < (DWORD)body_chain_physics_cfg.interval_ms) return;
    elapsed_ms = state->last_tick ? now - state->last_tick : 0;
    dt = body_motion_duration(elapsed_ms);
    int motion_steps = body_motion_substeps(dt), motion_step;
    float motion_dt = dt / (float)motion_steps;
    state->last_tick = now;
    perf_setup_start = physx_perf_counter();

    if (state->initialized &&
        state->root_raw &&
        state->joint_raw[0] &&
        state->joint_raw[1] &&
        state->joint_raw[2]) {
        root_raw = state->root_raw;
        for (i = 0; i < 3; i++) {
            joint_raw[i] = state->joint_raw[i];
        }
        cache_hot = 1;
        if (!state->cache_verify_tick ||
            now - state->cache_verify_tick >= 10000) {
            void *verify_root = NULL;
            void *verify_joint[3] = { NULL, NULL, NULL };
            state->cache_verify_tick = now;
            if (!resolve_body_chain_raws(person, &verify_root, verify_joint) ||
                verify_root != root_raw ||
                verify_joint[0] != joint_raw[0] ||
                verify_joint[1] != joint_raw[1] ||
                verify_joint[2] != joint_raw[2]) {
                reset_body_chain_person_state(state);
                return;
            }
        }
    } else {
        if (!resolve_body_chain_raws(person, &root_raw, joint_raw)) {
            reset_body_chain_person_state(state);
            state->resolve_retry_tick = now;
            return;
        }
    }
    state->resolve_retry_tick = 0;

    if (!root_raw || !ptr_readable((BYTE*)root_raw + body_chain_physics_cfg.root_offset, sizeof(float) * 3)) {
        reset_body_chain_person_state(state);
        return;
    }
    root = (float*)((BYTE*)root_raw + body_chain_physics_cfg.root_offset);
    for (axis = 0; axis < 3; axis++) {
        if (!sane_probe_float(root[axis])) {
            reset_body_chain_person_state(state);
            return;
        }
        root_drive_position[axis] = root[axis];
    }
    {
        float pivot_view[3];
        if (body_collider_engine_pivot_view(person, "root", NULL,
                                            pivot_view)) {
            root_drive_position[0] = pivot_view[0];
            root_drive_position[1] = pivot_view[1];
            root_drive_position[2] = pivot_view[2];
        }
    }
    for (i = 0; i < 3; i++) {
        if (!joint_raw[i] ||
            !ptr_readable((BYTE*)joint_raw[i] + body_chain_physics_cfg.output_offset, sizeof(float) * 3)) {
            reset_body_chain_person_state(state);
            return;
        }
        out[i] = (float*)((BYTE*)joint_raw[i] + body_chain_physics_cfg.output_offset);
        for (axis = 0; axis < 3; axis++) {
            if (!sane_probe_float(out[i][axis])) {
                reset_body_chain_person_state(state);
                return;
            }
        }
    }
    if (runtime_mode) {
        runtime_mapping_ready = body_chain_sample_runtime_mapping(
            person_index, 0, joint_raw, now);
        if (!runtime_mapping_ready) return;
    }
    if ((!state->initialized || root_raw != state->root_raw) &&
        physics_environment_cfg.gravity_probe_require_nonzero_root &&
        physx_vec3_len(root) <=
            physics_environment_cfg.gravity_probe_motion_epsilon) {
        if (!state->health_log_tick ||
            now - state->health_log_tick >= 2000) {
            state->health_log_tick = now;
            log_line("body-chain-physics ownership-deferred person=\"%s\" root=(%.5f,%.5f,%.5f) epsilon=%.6f note=\"room load root is still placeholder; no PoseEditor, animation, transform-lock, or PhysX output writes yet\"",
                     person,
                     root[0], root[1], root[2],
                     physics_environment_cfg.gravity_probe_motion_epsilon);
        }
        return;
    }
    if (!state->initialized ||
        root_raw != state->root_raw ||
        joint_raw[0] != state->joint_raw[0] ||
        joint_raw[1] != state->joint_raw[1] ||
        joint_raw[2] != state->joint_raw[2]) {
        for (i = 0; i < 3; i++) {
            for (axis = 0; axis < 3; axis++) {
                pre_physx_output[i][axis] = out[i][axis];
            }
        }
        pre_physx_output_captured = 1;
    }
    if (!runtime_mode && pre_physx_output_captured &&
        !capture_body_chain_pose_compensation(person_index, person, state)) {
        if (!state->health_log_tick ||
            now - state->health_log_tick >= 2000) {
            state->health_log_tick = now;
            log_line("body-chain-physics pose-compensation waiting person=\"%s\" note=\"penis_joint02/03 PoseEditor rotations are not readable yet; PhysX ownership is deferred without writes\"",
                     person);
        }
        return;
    }
    if (!runtime_mode && body_chain_physics_cfg.override_animation) {
        perf_section_start = physx_perf_counter();
        int poseeditor_tracks_ready =
            suppress_poseeditor_joint01_track_for_person(
                person_index, person, state, now);
        if (poseeditor_tracks_ready) {
            poseeditor_tracks_ready =
                suppress_poseeditor_joint02_03_tracks_for_person(
                    person_index, person, state);
        }
        if (!poseeditor_tracks_ready) {
            if (!state->health_log_tick ||
                now - state->health_log_tick >= 2000) {
                state->health_log_tick = now;
                log_line("body-chain-physics poseeditor ownership waiting person=\"%s\" note=\"current pose joint tracks are changing; PhysX output is held until TK17's active tracks are validated, neutralized, and suppressed\"",
                         person);
            }
            physx_perf_add(PHYSX_PERF_PENIS_OWNERSHIP,
                           perf_section_start);
            return;
        }
        physx_perf_add(PHYSX_PERF_PENIS_OWNERSHIP,
                       perf_section_start);
    }
    if (!runtime_mode && body_chain_physics_cfg.override_animation &&
        (perf_section_start = physx_perf_counter(),
         !neutralize_body_chain_animation(person, state))) {
        physx_perf_add(PHYSX_PERF_PENIS_OWNERSHIP,
                       perf_section_start);
        if (!state->health_log_tick ||
            now - state->health_log_tick >= 2000) {
            state->health_log_tick = now;
            log_line("body-chain-physics override-animation waiting person=\"%s\" animation_offset=0x%03x note=\"could not resolve/read penis_joint01/02/03 yet\"",
                     person, body_chain_physics_cfg.animation_output_offset);
        }
    } else if (!runtime_mode && body_chain_physics_cfg.override_animation) {
        physx_perf_add(PHYSX_PERF_PENIS_OWNERSHIP,
                       perf_section_start);
    }
    if (!runtime_mode && body_chain_physics_cfg.override_animation &&
        (perf_section_start = physx_perf_counter(),
         !neutralize_body_chain_joint01_pose(person, state, joint_raw[0]))) {
        physx_perf_add(PHYSX_PERF_PENIS_OWNERSHIP,
                       perf_section_start);
        if (!state->health_log_tick ||
            now - state->health_log_tick >= 2000) {
            state->health_log_tick = now;
            log_line("body-chain-physics joint01-pose-override waiting person=\"%s\" note=\"could not read Spenis_joint01 pose override offsets yet\"",
                     person);
        }
    } else if (!runtime_mode && body_chain_physics_cfg.override_animation) {
        physx_perf_add(PHYSX_PERF_PENIS_OWNERSHIP,
                       perf_section_start);
    }
    if (!runtime_mode && body_chain_physics_cfg.override_animation &&
        (perf_section_start = physx_perf_counter(),
         !lock_body_chain_joint01_transform(person, state))) {
        physx_perf_add(PHYSX_PERF_PENIS_OWNERSHIP,
                       perf_section_start);
        if (!state->health_log_tick ||
            now - state->health_log_tick >= 2000) {
            state->health_log_tick = now;
            log_line("body-chain-physics joint01-transform-lock waiting person=\"%s\" note=\"could not resolve/write penis_joint01 local transform rows yet\"",
                     person);
        }
    } else if (!runtime_mode && body_chain_physics_cfg.override_animation) {
        physx_perf_add(PHYSX_PERF_PENIS_OWNERSHIP,
                       perf_section_start);
    }
    if (!state->initialized ||
        root_raw != state->root_raw ||
        joint_raw[0] != state->joint_raw[0] ||
        joint_raw[1] != state->joint_raw[1] ||
        joint_raw[2] != state->joint_raw[2]) {
        body_chain_gravity_snapshot_t gravity_snapshot;
        body_chain_capture_gravity_snapshot(state, &gravity_snapshot,
                                            root_raw, now);
        state->initialized = 1;
        state->active_logged = 0;
        state->camera_seen_version = 0;
        state->root_drive_camera_seen_version = 0;
        state->root_drive_camera_quarantine_tick = 0;
        state->root_drive_camera_last_untrusted_tick = 0;
        state->root_drive_camera_ramp_log_tick = 0;
        state->root_drive_untrusted_log_tick = 0;
        state->root_drive_last_step_valid = 0;
        state->root_drive_last_h_step = 0.0f;
        state->root_drive_last_v_step = 0.0f;
        state->root_drive_last_d_step = 0.0f;
        memset(&state->root_drive_filter, 0, sizeof(state->root_drive_filter));
        state->dynamics_valid=state->dynamics_gravity_valid=0;
        state->camera_relative_trs_raw = NULL;
        state->camera_relative_live_prev_valid = 0;
        state->camera_relative_live_current_valid = 0;
        state->camera_relative_calibration_samples = 0;
        state->camera_relative_h_peak = 0.0f;
        state->camera_relative_v_peak = 0.0f;
        state->camera_relative_d_peak = 0.0f;
        state->camera_relative_live_log_tick = 0;
        for (i = 0; i < 9; i++) {
            state->camera_relative_live_prev[i] = 0.0f;
            state->camera_relative_live_current[i] = 0.0f;
            state->camera_relative_h_coeff[i] = 0.0f;
            state->camera_relative_v_coeff[i] = 0.0f;
            state->camera_relative_d_coeff[i] = 0.0f;
        }
        state->root_raw = root_raw;
        state->init_tick = now;
        state->gravity_probe_log_tick = 0;
        state->gravity_probe_stable_tick = 0;
        state->gravity_probe_candidate_tick = 0;
        state->gravity_probe_captured = 0;
        state->gravity_probe_promoted = 0;
        state->gravity_probe_sampled = 0;
        reset_body_chain_gravity_state(state);
        state->gravity_probe_prev_root[0] = 0.0f;
        state->gravity_probe_prev_root[1] = 0.0f;
        state->gravity_probe_prev_root[2] = 0.0f;
        state->gravity_probe_last_root[0] = 0.0f;
        state->gravity_probe_last_root[1] = 0.0f;
        state->gravity_probe_last_root[2] = 0.0f;
        for (axis = 0; axis < 3; axis++) {
            state->root_prev[axis] = root_drive_position[axis];
        }
        state->root_prev_world[0] = 0.0f;
        state->root_prev_world[1] = 0.0f;
        state->root_prev_world[2] = 0.0f;
        state->root_prev_world_valid = 0;
        for (i = 0; i < 3; i++) {
            state->joint_raw[i] = joint_raw[i];
            for (axis = 0; axis < 3; axis++) {
                /* zero_output_rest controls the simulation origin, but the
                   exact pre-PhysX output must survive for ownership release. */
                state->output_handoff_rest[i][axis] =
                    pre_physx_output_captured ?
                        pre_physx_output[i][axis] : out[i][axis];
                state->rest[i][axis] =
                    body_chain_physics_cfg.zero_output_rest ? 0.0f : out[i][axis];
                out[i][axis] = state->rest[i][axis] -
                    (state->pose_compensation_valid ?
                        state->pose_compensation[i][axis] : 0.0f);
            }
            for (axis = 0; axis < 3; axis++) {
                state->angle[i][axis] = 0.0f;
                state->velocity[i][axis] = 0.0f;
            }
        }
        state->output_handoff_rest_valid = 1;
        log_line("body-chain-physics initialized person=\"%s\" root_raw=%p root_offset=0x%03x output_offset=0x%03x root=(%.5f,%.5f,%.5f) rest01=(%.5f,%.5f,%.5f) rest02=(%.5f,%.5f,%.5f) rest03=(%.5f,%.5f,%.5f) handoff01=(%.5f,%.5f,%.5f) handoff02=(%.5f,%.5f,%.5f) handoff03=(%.5f,%.5f,%.5f)",
                 person, root_raw,
                 body_chain_physics_cfg.root_offset, body_chain_physics_cfg.output_offset,
                 root[0], root[1], root[2],
                 state->rest[0][0], state->rest[0][1], state->rest[0][2],
                 state->rest[1][0], state->rest[1][1], state->rest[1][2],
                 state->rest[2][0], state->rest[2][1], state->rest[2][2],
                 state->output_handoff_rest[0][0],
                 state->output_handoff_rest[0][1],
                 state->output_handoff_rest[0][2],
                 state->output_handoff_rest[1][0],
                 state->output_handoff_rest[1][1],
                 state->output_handoff_rest[1][2],
                 state->output_handoff_rest[2][0],
                 state->output_handoff_rest[2][1],
                 state->output_handoff_rest[2][2]);
        body_chain_camera_relative_orientation_step(
            person, state, root_raw,
            camera_relative_delta, NULL, &camera_relative_energy);
        if (!body_chain_restore_gravity_snapshot(state, &gravity_snapshot,
                                                 root, now, person,
                                                 "penis_physics")) {
            body_chain_restore_room_gravity_snapshot(
                state, body_chain_active_gravity_cache(person_index, 0),
                root_raw, root, now, person, "penis_physics");
        }
        perf_section_start = physx_perf_counter();
        run_body_chain_gravity_probe(
            person, state, root_raw, root, now,
            body_chain_active_gravity_cache(person_index, 0),
            "penis_physics");
        physx_perf_add(PHYSX_PERF_PENIS_GRAVITY, perf_section_start);
        if (runtime_mode) {
            body_chain_publish_runtime_pose(person_index, 0, out);
        }
        return;
    }

    physx_perf_add(PHYSX_PERF_PENIS_SETUP, perf_setup_start);
    perf_section_start = physx_perf_counter();
    camera_relative_step_valid =
        body_chain_camera_relative_orientation_step(
            person, state, root_raw,
            camera_relative_delta, parent_rotation_step,
            &camera_relative_energy);
    physx_perf_add(PHYSX_PERF_PENIS_ORIENTATION, perf_section_start);
    perf_section_start = physx_perf_counter();
    camera_neutral_translation_valid =
        body_chain_camera_neutral_pivot_step(
            person, person_index, state, root_drive_position,
            body_chain_physics_cfg.root_offset,
            NULL, camera_neutral_root_step);
    physx_perf_add(PHYSX_PERF_PENIS_TRANSLATION, perf_section_start);
    perf_section_start = physx_perf_counter();

    {
        DWORD root_camera_quarantine_ms =
            (DWORD)physics_environment_cfg.body_chain_camera_quarantine_ms;
        DWORD camera_age_ms = captured_camera_change_tick ?
            (now - captured_camera_change_tick) : 0xffffffffu;
        DWORD camera_quarantine_age_ms = 0xffffffffu;
        DWORD camera_settle_window_ms = root_camera_quarantine_ms + 900u;
        float camera_settle_threshold =
            physx_clampf(physics_environment_cfg.gravity_probe_invalidate_epsilon *
                         0.5f,
                         0.035f, 0.080f);
        float raw_root_step[3] = { 0.0f, 0.0f, 0.0f };
        float raw_root_step_len = 0.0f;
        int camera_changed =
            state->root_drive_camera_seen_version != captured_camera_version;
        int camera_recent =
            captured_camera_inverse_valid && camera_age_ms < root_camera_quarantine_ms;
        int camera_quarantine_active = 0;
        int camera_settle_quarantine = 0;

        for (axis = 0; axis < 3; axis++) {
            raw_root_step[axis] =
                root_drive_position[axis] - state->root_prev[axis];
            raw_root_step_len += raw_root_step[axis] * raw_root_step[axis];
        }
        raw_root_step_len = (float)sqrt((double)raw_root_step_len);
        if (camera_changed || camera_recent) {
            state->root_drive_camera_quarantine_tick = now;
        }
        if (state->root_drive_camera_quarantine_tick) {
            camera_quarantine_age_ms =
                now - state->root_drive_camera_quarantine_tick;
        }
        camera_quarantine_active =
            state->root_drive_camera_quarantine_tick &&
            root_camera_quarantine_ms > 0 &&
            camera_quarantine_age_ms < root_camera_quarantine_ms;
        if (!camera_changed && !camera_recent &&
            captured_camera_inverse_valid &&
            camera_age_ms < camera_settle_window_ms &&
            state->root_drive_camera_quarantine_tick &&
            root_camera_quarantine_ms > 0 &&
            raw_root_step_len > camera_settle_threshold) {
            camera_settle_quarantine = 1;
            camera_quarantine_active = 1;
            state->root_drive_camera_quarantine_tick = now;
            camera_quarantine_age_ms = 0;
        }
        root_drive_untrusted =
            camera_changed || camera_recent || camera_quarantine_active ||
            camera_settle_quarantine;
        if (root_drive_untrusted) {
            state->root_drive_camera_last_untrusted_tick = now;
        }

        for (axis = 0; axis < 3; axis++) {
            root_step[axis] = camera_neutral_translation_valid ?
                camera_neutral_root_step[axis] : 0.0f;
            state->root_prev[axis] = root_drive_position[axis];
        }
        if (root_drive_untrusted &&
            (!state->last_log_tick ||
             now - state->last_log_tick >= 1000)) {
            state->last_log_tick = now;
            log_line("body-chain-physics root-drive camera-rebased person=\"%s\" camera_version=%ld camera_age_ms=%lu quarantine_age_ms=%lu settle=%d world_step=%d raw_root_step_len=%.5f root=(%.5f,%.5f,%.5f) note=\"root drive fallback is camera-contaminated; discarding root impulse only, spring/gravity simulation continues\"",
                     person,
                     captured_camera_version,
                     (unsigned long)camera_age_ms,
                     (unsigned long)camera_quarantine_age_ms,
                     camera_settle_quarantine,
                     0,
                     raw_root_step_len,
                     root[0], root[1], root[2]);
        }
        state->root_drive_camera_seen_version = captured_camera_version;
    }
    h_step =
        root_step[0] * body_chain_physics_cfg.horizontal_source_vector[0] +
        root_step[1] * body_chain_physics_cfg.horizontal_source_vector[1] +
        root_step[2] * body_chain_physics_cfg.horizontal_source_vector[2];
    v_step =
        root_step[0] * body_chain_physics_cfg.vertical_source_vector[0] +
        root_step[1] * body_chain_physics_cfg.vertical_source_vector[1] +
        root_step[2] * body_chain_physics_cfg.vertical_source_vector[2];
    d_step = root_step[body_chain_physics_cfg.translation_source_axis[2]];
    if (!root_drive_untrusted &&
        camera_relative_step_valid &&
        state->camera_relative_calibration_samples < 256u) {
        body_chain_train_camera_relative_orientation(
            state, camera_relative_delta, camera_relative_energy,
            h_step, v_step, d_step);
    }
    if (camera_relative_step_valid) {
        camera_relative_prediction_valid =
            body_chain_predict_camera_relative_orientation(
                state, camera_relative_delta, camera_relative_energy,
                &camera_relative_h_step, &camera_relative_v_step,
                &camera_relative_d_step);
    }
    if (root_drive_untrusted) {
        float held_h_step = state->root_drive_last_h_step;
        float held_v_step = state->root_drive_last_v_step;
        int held_step_valid = state->root_drive_last_step_valid;
        camera_relative_root_drive =
            camera_neutral_translation_valid ||
            camera_relative_prediction_valid;
        if (camera_neutral_translation_valid) {
            /* h/v/d already came from the camera-neutral world pivot. */
        } else if (camera_relative_prediction_valid) {
            h_step = camera_relative_h_step;
            v_step = camera_relative_v_step;
            d_step = camera_relative_d_step;
        } else {
            h_step = 0.0f;
            v_step = 0.0f;
            d_step = 0.0f;
            state->root_drive_last_step_valid = 0;
            state->root_drive_last_h_step = 0.0f;
            state->root_drive_last_v_step = 0.0f;
            state->root_drive_last_d_step = 0.0f;
        }
        if (defaults_cfg.debug &&
            (!state->root_drive_untrusted_log_tick ||
             now - state->root_drive_untrusted_log_tick >= 250)) {
            state->root_drive_untrusted_log_tick = now;
            log_line("body-chain-physics camera-free-run person=\"%s\" camera_version=%ld relative_orientation=%d calibration_samples=%u relative_energy=%.7f held_step_valid=%d held_source_step=(h=%.5f,v=%.5f) applied_source_step=(h=%.5f,v=%.5f) decay=%.3f gravity_drive=(%.4f,%.4f,%.4f) angle01=(%.3f,%.3f) note=\"camera-contaminated translation ignored; simulation coasts from the last trusted body step\"",
                     person,
                     captured_camera_version,
                     camera_relative_root_drive,
                     state->camera_relative_calibration_samples,
                     camera_relative_energy,
                     held_step_valid,
                     held_h_step,
                     held_v_step,
                     h_step,
                     v_step,
                     0.0f,
                     state->gravity_drive[0],
                     state->gravity_drive[1],
                     state->gravity_drive[2],
                     state->angle[0][0],
                     state->angle[0][1]);
        }
    }
    {
        float scale = body_motion_input_scale(elapsed_ms);
        float input[3] = { h_step * scale, v_step * scale, d_step * scale };
        float filtered[3];
        int drive_sample_valid = !root_drive_untrusted || camera_relative_root_drive;
        if (physx_absf(input[0]) < body_chain_physics_cfg.horizontal_deadzone) input[0] = 0;
        if (physx_absf(input[1]) < body_chain_physics_cfg.vertical_deadzone) input[1] = 0;
        if (physx_absf(input[2]) < body_chain_physics_cfg.translation_deadzone) input[2] = 0;
        body_motion_filter_sample(&state->root_drive_filter, input,
                                 elapsed_ms, drive_sample_valid, filtered);
        h_step = filtered[0]; v_step = filtered[1]; d_step = filtered[2];
        for (axis = 0; axis < 3; axis++) parent_rotation_step[axis] *= scale;
    }
    if (physx_absf(h_step) < body_chain_physics_cfg.horizontal_deadzone) h_step = 0.0f;
    if (physx_absf(v_step) < body_chain_physics_cfg.vertical_deadzone) v_step = 0.0f;
    if (physx_absf(d_step) < body_chain_physics_cfg.translation_deadzone) d_step = 0.0f;
    if (!root_drive_untrusted) {
        state->root_drive_last_h_step = h_step;
        state->root_drive_last_v_step = v_step;
        state->root_drive_last_d_step = d_step;
        state->root_drive_last_step_valid = 1;
    }
    physx_perf_add(PHYSX_PERF_PENIS_TRANSLATION, perf_section_start);
    perf_section_start = physx_perf_counter();
    run_body_chain_gravity_probe(
        person, state, root_raw, root, now,
        body_chain_active_gravity_cache(person_index, 0),
        "penis_physics");
    physx_perf_add(PHYSX_PERF_PENIS_GRAVITY, perf_section_start);
    if (camera_test_isolation) {
        clear_body_chain_collision_for_test(state);
    }
    reactivation_collision_grace =
        body_chain_reactivation_collision_grace_active(
            person_index, state, now);
    if (!camera_test_isolation &&
        physics_environment_cfg.gravity_apply_to_body_chain &&
        physics_environment_cfg.world_gravity_probe &&
        (!state->gravity_probe_promoted ||
         (physics_environment_cfg.gravity_zero_at_start &&
         !state->gravity_drive_ref_valid))) {
        if (!runtime_mode && body_chain_physics_cfg.override_animation) {
            perf_section_start = physx_perf_counter();
            neutralize_body_chain_animation(person, state);
            neutralize_body_chain_joint01_pose(person, state, joint_raw[0]);
            lock_body_chain_joint01_transform(person, state);
            physx_perf_add(PHYSX_PERF_PENIS_OWNERSHIP,
                           perf_section_start);
        }
        for (i = 0; i < 3; i++) {
            for (axis = 0; axis < 3; axis++) {
                out[i][axis] = state->rest[i][axis] -
                    (state->pose_compensation_valid ?
                        state->pose_compensation[i][axis] : 0.0f);
            }
            for (axis = 0; axis < 3; axis++) {
                state->angle[i][axis] = 0.0f;
                state->velocity[i][axis] = 0.0f;
            }
        }
        if (!state->health_log_tick ||
            now - state->health_log_tick >= 2000) {
            state->health_log_tick = now;
            log_line("body-chain-physics gated person=\"%s\" reason=\"waiting for atomic gravity baseline\" root=(%.5f,%.5f,%.5f) gravity_promoted=%d gravity_ref_valid=%d gravity_apply=%d note=\"neutral output; no spring accumulation\"",
                     person,
                     root[0], root[1], root[2],
                     state->gravity_probe_promoted,
                     state->gravity_drive_ref_valid,
                     physics_environment_cfg.gravity_apply_to_body_chain);
        }
        if (runtime_mode) {
            body_chain_publish_runtime_pose(person_index, 0, out);
        }
        return;
    }
    if (!state->active_logged) {
        state->active_logged = 1;
        log_line("body-chain-physics active person=\"%s\" source=\"camera-relative root\" root_offset=0x%03x write=\"Spenis_joint01/02/03 + 0x%03x\" override_animation=%d animation_offset=0x%03x animation_override_offsets=(0x%03x,0x%03x,0x%03x,0x%03x) animation_override_count=%d translation=(h:%d->%d x%.3f,v:%d->%d x%.3f,d:%d->%d x%.3f) rotation=(h:%d->%d x%.3f,v:%d->%d x%.3f,t:%d->%d x%.3f) deadzone=(translation=%.5f,rotation=%.5f) zero_rest=%d gravity_angle=%.2f stiffness=%.2f damping=%.2f max_angle=%.2f joint_max=(j1=%.2f/%.2f/%.2f,j2=%.2f/%.2f/%.2f,j3=%.2f/%.2f/%.2f)",
                 person,
                 body_chain_physics_cfg.root_offset, body_chain_physics_cfg.output_offset,
                 body_chain_physics_cfg.override_animation,
                 body_chain_physics_cfg.animation_output_offset,
                 body_chain_physics_cfg.animation_override_offsets[0],
                 body_chain_physics_cfg.animation_override_offsets[1],
                 body_chain_physics_cfg.animation_override_offsets[2],
                 body_chain_physics_cfg.animation_override_offsets[3],
                 body_chain_physics_cfg.animation_override_offset_count,
                 body_chain_physics_cfg.translation_source_axis[0],
                 body_chain_physics_cfg.translation_tail_axis[0],
                 body_chain_physics_cfg.translation_scale[0],
                 body_chain_physics_cfg.translation_source_axis[1],
                 body_chain_physics_cfg.translation_tail_axis[1],
                 body_chain_physics_cfg.translation_scale[1],
                 body_chain_physics_cfg.translation_source_axis[2],
                 body_chain_physics_cfg.translation_tail_axis[2],
                 body_chain_physics_cfg.translation_scale[2],
                 body_chain_physics_cfg.rotation_source_axis[0],
                 body_chain_physics_cfg.rotation_tail_axis[0],
                 body_chain_physics_cfg.rotation_scale[0],
                 body_chain_physics_cfg.rotation_source_axis[1],
                 body_chain_physics_cfg.rotation_tail_axis[1],
                 body_chain_physics_cfg.rotation_scale[1],
                 body_chain_physics_cfg.rotation_source_axis[2],
                 body_chain_physics_cfg.rotation_tail_axis[2],
                 body_chain_physics_cfg.rotation_scale[2],
                 body_chain_physics_cfg.translation_deadzone,
                 body_chain_physics_cfg.rotation_deadzone,
                 body_chain_physics_cfg.zero_output_rest,
                 body_chain_physics_cfg.gravity_angle,
                 body_chain_physics_cfg.stiffness,
                 body_chain_physics_cfg.damping,
                 body_chain_physics_cfg.max_angle,
                 body_chain_physics_cfg.link_max_angle[0][0],
                 body_chain_physics_cfg.link_max_angle[0][1],
                 body_chain_physics_cfg.link_max_angle[0][2],
                 body_chain_physics_cfg.link_max_angle[1][0],
                 body_chain_physics_cfg.link_max_angle[1][1],
                 body_chain_physics_cfg.link_max_angle[1][2],
                 body_chain_physics_cfg.link_max_angle[2][0],
                 body_chain_physics_cfg.link_max_angle[2][1],
                 body_chain_physics_cfg.link_max_angle[2][2]);
        log_line("body-chain-physics gravity-response person=\"%s\" response_ms=%.1f gravity_curve=(h=%.2f,v=%.2f) max_degrees_per_second=%.1f note=\"direction-weighted curves shape combined partial-tilt strength without skewing direction; near-complete orientations recover full legacy strength\"",
                 person,
                 physics_environment_cfg.gravity_response_ms,
                 body_chain_physics_cfg.gravity_horizontal_curve,
                 body_chain_physics_cfg.gravity_vertical_curve,
                 physics_environment_cfg.gravity_max_degrees_per_second);
    }

    perf_section_start = physx_perf_counter();
    body_contact_begin_step(person_index, state, 0, now, dt);
    face_down_translation = body_chain_face_down_translation_active(state);
    translation_drive_step[0] = h_step;
    translation_drive_step[1] = v_step;
    translation_drive_step[2] = d_step;
    if (face_down_translation) {
        translation_drive_step[
            body_chain_physics_cfg.face_down_translation_channel] *=
            body_chain_physics_cfg.face_down_translation_sign;
    }
    wind_active = body_chain_room_wind_channels(
        &body_chain_physics_cfg, state, state->root_raw, person,
        "penis_physics", now,
        wind_channels);

    for (motion_step = 0; motion_step < motion_steps; motion_step++) {
        if (motion_step > 0) perf_section_start = physx_perf_counter();
        state->collision_step_dt = motion_dt;
        update_body_chain_gravity_filter(state, motion_dt);
        {
            float target[3][3],gravity_target[3][3]={{0}};
            const float *active_gravity_drive = NULL;
            float curved_gravity_drive[3];
            int horizontal_axis = body_chain_physics_cfg.horizontal_output_axis;
            int vertical_axis = body_chain_physics_cfg.vertical_output_axis;

            if (!camera_test_isolation &&
                physics_environment_cfg.gravity_apply_to_body_chain &&
                state->gravity_probe_promoted) {
                active_gravity_drive = state->gravity_drive_filtered_valid ?
                    state->gravity_drive_filtered : state->gravity_drive;
                if (physx_absf(
                        body_chain_physics_cfg.gravity_horizontal_curve - 1.0f) >
                        0.000001f ||
                    physx_absf(
                        body_chain_physics_cfg.gravity_vertical_curve - 1.0f) >
                        0.000001f) {
                    body_chain_apply_gravity_curve(
                        active_gravity_drive,
                        body_chain_physics_cfg.gravity_horizontal_curve,
                        body_chain_physics_cfg.gravity_vertical_curve,
                        curved_gravity_drive);
                    active_gravity_drive = curved_gravity_drive;
                }
            }
            for (i = 0; i < 3; i++) {
                float gain = body_chain_physics_cfg.link_gain[i];
                float gravity_add_h = 0.0f;
                float gravity_add_v = 0.0f;
                float mapped_drive[3],gravity_mapped[3];
                if (active_gravity_drive) {
                    float gravity_secondary_drive = active_gravity_drive[2];
                    gravity_add_h = active_gravity_drive[0] *
                                    physics_environment_cfg.gravity_horizontal_body_chain_scale;
                    gravity_add_h += gravity_secondary_drive *
                                     physics_environment_cfg.gravity_horizontal_secondary_body_chain_scale;
                    gravity_add_v = active_gravity_drive[1] *
                                    physics_environment_cfg.gravity_vertical_body_chain_scale;
                    gravity_add_v += gravity_secondary_drive *
                                     physics_environment_cfg.gravity_vertical_secondary_body_chain_scale;
                }
                body_chain_build_mapped_drive(
                    &body_chain_physics_cfg,
                    translation_drive_step[0], translation_drive_step[1],
                    translation_drive_step[2],
                    parent_rotation_step, gravity_add_h, gravity_add_v,
                    mapped_drive);
                if (active_gravity_drive &&
                    body_chain_physics_cfg.gravity_inverted_strength > 0.0f &&
                    body_chain_physics_cfg.gravity_inverted_tail_axis >= 0 &&
                    body_chain_physics_cfg.gravity_inverted_tail_axis <= 2) {
                    float inverted_amount =
                        body_chain_inverted_gravity_amount(
                            active_gravity_drive);
                    inverted_gravity_target = inverted_amount *
                        body_chain_physics_cfg.gravity_inverted_strength *
                        body_chain_physics_cfg.gravity_inverted_sign;
                    mapped_drive[
                        body_chain_physics_cfg.gravity_inverted_tail_axis] +=
                            inverted_gravity_target;
                }
                body_chain_build_mapped_drive(&body_chain_physics_cfg,
                    0,0,0,NULL,gravity_add_h,gravity_add_v,gravity_mapped);
                if(active_gravity_drive && body_chain_physics_cfg.gravity_inverted_strength>0 &&
                   body_chain_physics_cfg.gravity_inverted_tail_axis>=0 && body_chain_physics_cfg.gravity_inverted_tail_axis<3)
                    gravity_mapped[body_chain_physics_cfg.gravity_inverted_tail_axis]+=inverted_gravity_target;
                if (wind_active) {
                    body_chain_add_room_wind_target(
                        &body_chain_physics_cfg,
                        wind_channels,
                        NULL,
                        mapped_drive);
                }
                for (axis = 0; axis < 3; axis++) {
                    gravity_target[i][axis]=body_chain_clamp_link_axis_angle(
                        &body_chain_physics_cfg,i,axis,gravity_mapped[axis]*gain);
                    target[i][axis] = body_chain_clamp_link_axis_angle(
                        &body_chain_physics_cfg, i, axis,
                        mapped_drive[axis] * gain);
                }
            }
            chain_target_limited = body_chain_limit_total_rotation(
                &body_chain_physics_cfg, target, NULL, 3);

            body_chain_limit_total_rotation(&body_chain_physics_cfg,gravity_target,NULL,3);
            body_chain_shape_gravity(person_index,state,&body_chain_physics_cfg,
                now,motion_dt,gravity_target,target);
            for (i = 0; i < 3; i++) {
                for (axis = 0; axis < 3; axis++) {
                    target[i][axis] = body_chain_clamp_link_axis_angle(
                        &body_chain_physics_cfg, i, axis, target[i][axis]);
                }
                for (axis = 0; axis < 3; axis++) {
                    float effective_stiffness = body_chain_physics_cfg.stiffness;
                    float effective_damping = body_chain_physics_cfg.damping;
                    if (root_drive_untrusted && !camera_relative_root_drive) {
                        effective_stiffness *=
                            physics_environment_cfg.body_chain_camera_coast_stiffness_scale;
                        effective_damping *=
                            physics_environment_cfg.body_chain_camera_coast_damping_scale;
                    }
                    body_chain_apply_link_inertia(state,i,axis,&effective_stiffness,&effective_damping);
                    body_motion_limited_spring_step(&state->angle[i][axis], &state->velocity[i][axis],
                        target[i][axis], effective_stiffness, effective_damping, motion_dt,
                        body_chain_link_axis_min_limit(&body_chain_physics_cfg, i, axis),
                        body_chain_link_axis_limit(&body_chain_physics_cfg, i, axis));
                }
            }
            chain_state_limited = body_chain_limit_total_rotation(
                &body_chain_physics_cfg, state->angle, state->velocity, 3);
            body_contact_limit_velocity(&body_chain_physics_cfg,state,NULL,3);

            physx_perf_add(PHYSX_PERF_PENIS_SOLVER, perf_section_start);
            perf_section_start = physx_perf_counter();
            if ((body_chain_collider_cfg.penis_collision_enabled ||
                 body_chain_physics_cfg.room_collision_enabled) &&
                !camera_test_isolation &&
                !reactivation_collision_grace) {
                /* Contact iterations run inside one coherent manifold sample.
                   Re-sampling the same frame here would duplicate corrections. */
                float collider_correction[3][2] = {
                    { 0.0f, 0.0f },
                    { 0.0f, 0.0f },
                    { 0.0f, 0.0f }
                };
                float collider_max_penetration = 0.0f;
                float collider_sweep_radius = 0.0f;
                float collider_motion = 0.0f;
                body_chain_compute_collider_projection(
                    person_index, state, collider_correction, now, 1,
                    &collider_max_penetration,
                    &collider_sweep_radius,
                    &collider_motion,
                    BODY_CHAIN_COLLISION_TARGET_PENIS);
                body_contact_apply(state, &body_chain_physics_cfg,
                    collider_correction, 3, collider_max_penetration);
            }
            physx_perf_add(PHYSX_PERF_PENIS_COLLISION,
                           perf_section_start);

        }
    } /* Substeps complete; publish only the final chain pose. */

    for (i = 0; i < 3; i++) {
        for (axis = 0; axis < 3; axis++) {
            out[i][axis] = state->rest[i][axis] +
                           state->angle[i][axis] -
                           (state->pose_compensation_valid ?
                               state->pose_compensation[i][axis] : 0.0f);
            out[i][axis] =
                body_chain_clamp_link_output_value(
                    &body_chain_physics_cfg, i, axis,
                    state->rest[i][axis] -
                        (state->pose_compensation_valid ?
                            state->pose_compensation[i][axis] : 0.0f),
                    out[i][axis]);
        }
    }

    if (!runtime_mode && body_chain_physics_cfg.override_animation) {
        perf_section_start = physx_perf_counter();
        neutralize_body_chain_animation(person, state);
        neutralize_body_chain_joint01_pose(person, state, joint_raw[0]);
        lock_body_chain_joint01_transform(person, state);
        physx_perf_add(PHYSX_PERF_PENIS_OWNERSHIP,
                       perf_section_start);
    }
    if (runtime_mode) {
        body_chain_publish_runtime_pose(person_index, 0, out);
    }
    if (defaults_cfg.debug &&
        (!state->last_log_tick || now - state->last_log_tick >= 1000)) {
        state->last_log_tick = now;
        log_line("body-chain-physics write person=\"%s\" root_step=(%.5f,%.5f,%.5f) source_step=(h=%.5f,v=%.5f,d=%.5f) applied_step=(h=%.5f,v=%.5f,d=%.5f) face_down_translation=%d face_down_channel=%d face_down_sign=%.3f root_drive_untrusted=%d camera_relative_root_drive=%d camera_relative_samples=%u held_step_valid=%d gravity_drive=(%.4f,%.4f,%.4f) gravity_filtered=(%.4f,%.4f,%.4f) inverted_gravity=%.3f chain_limit=(target=%d,state=%d,max_bend=%.1f,max_twist=%.1f) angle01=(%.3f,%.3f) angle02=(%.3f,%.3f) angle03=(%.3f,%.3f) velocity=(%.3f,%.3f;%.3f,%.3f;%.3f,%.3f) contact=(%.3f,%.3f;%.3f,%.3f;%.3f,%.3f) collision=(penetration=%.5f,rest=%.5f,rest_valid=%d,rest_ticks=%d,grace_ticks=%d,impact_ticks=%d,manifold_contacts=%d,room_contacts=%d,multi_grace=%d) out01=(%.3f,%.3f,%.3f) out02=(%.3f,%.3f,%.3f) out03=(%.3f,%.3f,%.3f)",
                 person,
                 root_step[0], root_step[1], root_step[2],
                 h_step, v_step, d_step,
                 translation_drive_step[0], translation_drive_step[1],
                 translation_drive_step[2], face_down_translation,
                 body_chain_physics_cfg.face_down_translation_channel,
                 body_chain_physics_cfg.face_down_translation_sign,
                 root_drive_untrusted,
                 camera_relative_root_drive,
                 state->camera_relative_calibration_samples,
                 state->root_drive_last_step_valid,
                 state->gravity_drive[0],
                 state->gravity_drive[1],
                 state->gravity_drive[2],
                 state->gravity_drive_filtered[0],
                 state->gravity_drive_filtered[1],
                 state->gravity_drive_filtered[2],
                 inverted_gravity_target,
                 chain_target_limited,
                 chain_state_limited,
                 body_chain_physics_cfg.chain_total_bend_max,
                 body_chain_physics_cfg.chain_total_twist_max,
                 state->angle[0][0], state->angle[0][1],
                 state->angle[1][0], state->angle[1][1],
                 state->angle[2][0], state->angle[2][1],
                 state->velocity[0][0], state->velocity[0][1],
                 state->velocity[1][0], state->velocity[1][1],
                 state->velocity[2][0], state->velocity[2][1],
                 state->collision_contact_direction[0][0],
                 state->collision_contact_direction[0][1],
                 state->collision_contact_direction[1][0],
                 state->collision_contact_direction[1][1],
                 state->collision_contact_direction[2][0],
                 state->collision_contact_direction[2][1],
                 state->collision_prev_max_penetration,
                 state->collision_rest_penetration,
                 state->collision_rest_valid,
                 state->collision_rest_ticks,
                 state->collision_rest_grace_ticks,
                 state->collision_impact_ticks,
                 state->collision_manifold_contacts,
                 state->collision_room_contacts,
                 state->collision_multi_support_grace_ticks,
                 out[0][0], out[0][1], out[0][2],
                 out[1][0], out[1][1], out[1][2],
                 out[2][0], out[2][1], out[2][2]);
    }
    if (defaults_cfg.debug &&
        (!state->health_log_tick ||
         now - state->health_log_tick >= 5000)) {
        state->health_log_tick = now;
        log_line("body-chain-physics health person=\"%s\" active=1 cache=%s interval_ms=%d elapsed_ms=%lu dt=%.4f substeps=%d step_dt=%.4f inertia=%d geometry_gravity=%d root_raw=%p joint_raw=(%p,%p,%p)",
                 person,
                 cache_hot ? "hot" : "resolved",
                 body_chain_physics_cfg.interval_ms,
                 (unsigned long)elapsed_ms,
                 dt, motion_steps, motion_dt,
                 state->dynamics_valid, state->dynamics_gravity_active,
                 root_raw,
                 joint_raw[0], joint_raw[1], joint_raw[2]);
    }
}

static int testicle_physics_candidate_is_stable(
    body_chain_person_state_t *state,
    void *root_raw,
    void *joint_raw[3],
    DWORD now)
{
    int same_candidate;
    if (!state || !root_raw || !joint_raw ||
        !joint_raw[0] || !joint_raw[1] || !joint_raw[2]) {
        return 0;
    }
    if (state->initialized) return 1;
    same_candidate =
        state->ownership_candidate_root_raw == root_raw &&
        state->ownership_candidate_joint_raw[0] == joint_raw[0] &&
        state->ownership_candidate_joint_raw[1] == joint_raw[1] &&
        state->ownership_candidate_joint_raw[2] == joint_raw[2];
    if (!same_candidate) {
        state->ownership_candidate_root_raw = root_raw;
        state->ownership_candidate_joint_raw[0] = joint_raw[0];
        state->ownership_candidate_joint_raw[1] = joint_raw[1];
        state->ownership_candidate_joint_raw[2] = joint_raw[2];
        state->ownership_candidate_tick = now;
        state->ownership_candidate_samples = 1;
        return 0;
    }
    if (state->ownership_candidate_samples < 0xffffffffu) {
        state->ownership_candidate_samples++;
    }
    return state->ownership_candidate_samples >= 3 &&
           now - state->ownership_candidate_tick >= 50u;
}

static int testicle_physics_live_ownership_matches(
    int person_index,
    body_chain_person_state_t *state)
{
    const char *person;
    body_chain_physics_config_t *cfg = &testicle_physics_cfg;
    void *root_raw = NULL;
    void *joint_raw[3] = { NULL, NULL, NULL };
    float *root;
    LONG node_generation;
    int i, axis;
    if (person_index < 0 || person_index >= 4 || !state ||
        !state->initialized || !state->root_raw ||
        !state->joint_raw[0] || !state->joint_raw[1] ||
        !state->joint_raw[2]) {
        return 0;
    }
    node_generation = InterlockedCompareExchange(
        &named_node_generation, 0, 0);
    if (InterlockedCompareExchange(&physx_late_ownership_active, 0, 0) &&
        physx_simulation_serial &&
        state->live_ownership_simulation_serial ==
            physx_simulation_serial &&
        state->live_ownership_node_generation == node_generation) {
        return 1;
    }
    person = body_chain_person_name(person_index);
    if (!resolve_testicle_physics_raws(person, &root_raw, joint_raw) ||
        root_raw != state->root_raw ||
        joint_raw[0] != state->joint_raw[0] ||
        joint_raw[1] != state->joint_raw[1] ||
        joint_raw[2] != state->joint_raw[2] ||
        !ptr_readable((BYTE*)root_raw + cfg->root_offset,
                      sizeof(float) * 3)) {
        return 0;
    }
    root = (float*)((BYTE*)root_raw + cfg->root_offset);
    if (physics_environment_cfg.gravity_probe_require_nonzero_root &&
        physx_vec3_len(root) <=
            physics_environment_cfg.gravity_probe_motion_epsilon) {
        return 0;
    }
    for (axis = 0; axis < 3; axis++) {
        if (!sane_probe_float(root[axis])) return 0;
    }
    for (i = 0; i < 3; i++) {
        float *out;
        if (!ptr_readable((BYTE*)joint_raw[i] + cfg->output_offset,
                          sizeof(float) * 3)) {
            return 0;
        }
        out = (float*)((BYTE*)joint_raw[i] + cfg->output_offset);
        for (axis = 0; axis < 3; axis++) {
            if (!sane_probe_float(out[axis])) return 0;
        }
    }
    state->live_ownership_simulation_serial = physx_simulation_serial;
    state->live_ownership_node_generation = node_generation;
    return 1;
}

static void run_testicle_physics_for_person(int person_index, DWORD now)
{
    const char *person = body_chain_person_name(person_index);
    int runtime_mode = body_chain_runtime_mode_active();
    body_chain_person_state_t *state =
        body_chain_active_person_state(person_index, 1);
    body_chain_physics_config_t *cfg = &testicle_physics_cfg;
    void *root_raw;
    void *joint_raw[3];
    float *root;
    float *out[3];
    float root_step[3];
    float root_drive_position[3];
    float camera_neutral_root_step[3] = { 0.0f, 0.0f, 0.0f };
    float camera_relative_delta[9] = { 0.0f };
    float parent_rotation_step[3] = { 0.0f, 0.0f, 0.0f };
    float camera_relative_energy = 0.0f;
    float camera_relative_h_step = 0.0f;
    float camera_relative_v_step = 0.0f;
    float camera_relative_d_step = 0.0f;
    float h_step;
    float v_step;
    float d_step = 0.0f;
    float translation_drive_step[3] = { 0.0f, 0.0f, 0.0f };
    float wind_channels[3] = { 0.0f, 0.0f, 0.0f };
    float spring_target[3][3] = { { 0.0f }, { 0.0f }, { 0.0f } };
    float inverted_gravity_target = 0.0f;
    float dt;
    DWORD elapsed_ms;
    int cache_hot = 0;
    int camera_relative_step_valid = 0;
    int camera_relative_root_drive = 0;
    int camera_relative_prediction_valid = 0;
    int camera_neutral_translation_valid = 0;
    int root_drive_untrusted = 0;
    int face_down_translation = 0;
    int wind_active = 0;
    int chain_target_limited = 0;
    int chain_state_limited = 0;
    int ownership_ready = 1;
    int poseeditor_suppressed = 1;
    int horizontal_axis = cfg->horizontal_output_axis;
    int vertical_axis = cfg->vertical_output_axis;
    int i, axis;

    if (!state) return;
    /* An unloaded slot has no visible physics output.  Avoid repeatedly
       resolving its missing root/joints, but do not bypass validation for an
       initialized person or for engines where visibility is unavailable. */
    if (!state->initialized &&
        poseedit_scene_person_visible(person_index) == 0) {
        return;
    }
    if (!state->initialized && state->resolve_retry_tick &&
        now - state->resolve_retry_tick <
            (DWORD)TESTICLE_PHYSICS_MISSING_RETRY_MS) {
        return;
    }
    if (state->last_tick &&
        now - state->last_tick < (DWORD)cfg->interval_ms) {
        return;
    }
    elapsed_ms = state->last_tick ? now - state->last_tick : 0;
    dt = body_motion_duration(elapsed_ms);
    int motion_steps = body_motion_substeps(dt), motion_step;
    float motion_dt = dt / (float)motion_steps;
    state->last_tick = now;

    if (state->initialized &&
        state->root_raw &&
        state->joint_raw[0] &&
        state->joint_raw[1] &&
        state->joint_raw[2]) {
        if (!testicle_physics_live_ownership_matches(person_index, state)) {
            reset_body_chain_person_state(state);
            state->resolve_retry_tick = now;
            return;
        }
        root_raw = state->root_raw;
        joint_raw[0] = state->joint_raw[0];
        joint_raw[1] = state->joint_raw[1];
        joint_raw[2] = state->joint_raw[2];
        cache_hot = 1;
        state->cache_verify_tick = now;
    } else {
        if (!resolve_testicle_physics_raws(person, &root_raw, joint_raw)) {
            reset_body_chain_person_state(state);
            state->resolve_retry_tick = now;
            return;
        }
    }
    state->resolve_retry_tick = 0;

    if (!root_raw ||
        !ptr_readable((BYTE*)root_raw + cfg->root_offset,
                      sizeof(float) * 3)) {
        reset_body_chain_person_state(state);
        return;
    }
    root = (float*)((BYTE*)root_raw + cfg->root_offset);
    for (axis = 0; axis < 3; axis++) {
        if (!sane_probe_float(root[axis])) {
            reset_body_chain_person_state(state);
            return;
        }
        root_drive_position[axis] = root[axis];
    }
    {
        float pivot_view[3];
        if (body_collider_engine_pivot_view(person, "root", NULL,
                                            pivot_view)) {
            root_drive_position[0] = pivot_view[0];
            root_drive_position[1] = pivot_view[1];
            root_drive_position[2] = pivot_view[2];
        }
    }
    for (i = 0; i < 3; i++) {
        if (!joint_raw[i] ||
            !ptr_readable((BYTE*)joint_raw[i] + cfg->output_offset,
                          sizeof(float) * 3)) {
            reset_body_chain_person_state(state);
            return;
        }
        out[i] = (float*)((BYTE*)joint_raw[i] + cfg->output_offset);
        for (axis = 0; axis < 3; axis++) {
            if (!sane_probe_float(out[i][axis])) {
                reset_body_chain_person_state(state);
                return;
            }
        }
    }
    if (runtime_mode) {
        body_chain_sample_runtime_mapping(person_index, 1, joint_raw, now);
    }
    if ((!state->initialized || root_raw != state->root_raw) &&
        physics_environment_cfg.gravity_probe_require_nonzero_root &&
        physx_vec3_len(root) <=
            physics_environment_cfg.gravity_probe_motion_epsilon) {
        if (!state->health_log_tick ||
            now - state->health_log_tick >= 2000) {
            state->health_log_tick = now;
            log_line("testicle-physics ownership-deferred person=\"%s\" root=(%.5f,%.5f,%.5f) epsilon=%.6f note=\"room load root is still placeholder; no PoseEditor, animation, or PhysX output writes yet\"",
                     person,
                     root[0], root[1], root[2],
                     physics_environment_cfg.gravity_probe_motion_epsilon);
        }
        return;
    }

    if (!testicle_physics_candidate_is_stable(state, root_raw,
                                               joint_raw, now)) {
        if (defaults_cfg.debug &&
            (!state->health_log_tick ||
             now - state->health_log_tick >= 2000)) {
            state->health_log_tick = now;
            log_line("testicle-physics ownership-deferred person=\"%s\" reason=\"live root/testicle joints are settling\" samples=%u settle_ms=%lu note=\"native TesticleInertia and PhysX outputs remain untouched until the live skeleton is stable\"",
                     person,
                     state->ownership_candidate_samples,
                     (unsigned long)(now -
                         state->ownership_candidate_tick));
        }
        return;
    }

    if (!suppress_tk17_testicle_inertia_for_person(person_index, now)) {
        if (defaults_cfg.debug &&
            (!state->health_log_tick ||
             now - state->health_log_tick >= 2000)) {
            state->health_log_tick = now;
            log_line("testicle-physics ownership-deferred person=\"%s\" reason=\"live TK17 TesticleInertia ownership is not ready\" note=\"no PoseEditor, animation, or PhysX output writes were performed\"",
                     person);
        }
        return;
    }

    if (!runtime_mode && cfg->override_animation) {
        poseeditor_suppressed =
            suppress_poseeditor_testicle_track_for_person(
            person_index, person, state);
        if (!neutralize_testicle_physics_animation(person, state) &&
            (!state->health_log_tick ||
             now - state->health_log_tick >= 2000)) {
            state->health_log_tick = now;
            log_line("testicle-physics override-animation waiting person=\"%s\" animation_offset=0x%03x note=\"could not resolve/read testicles_joint01/02/End yet\"",
                     person, cfg->animation_output_offset);
        }
    }

    ownership_ready = poseeditor_suppressed;

    if (!ownership_ready || !poseeditor_suppressed) {
        if (!state->health_log_tick ||
            now - state->health_log_tick >= 2000) {
            state->health_log_tick = now;
            log_line("testicle-physics ownership-pending person=\"%s\" ownership_ready=%d poseeditor_suppressed=%d note=\"continuing PhysX Stesticles output while retrying PoseEditor track ownership\"",
                     person,
                     ownership_ready,
                     poseeditor_suppressed);
        }
    }

    if (!state->initialized ||
        root_raw != state->root_raw ||
        joint_raw[0] != state->joint_raw[0] ||
        joint_raw[1] != state->joint_raw[1] ||
        joint_raw[2] != state->joint_raw[2]) {
        body_chain_gravity_snapshot_t gravity_snapshot;
        body_chain_capture_gravity_snapshot(state, &gravity_snapshot,
                                            root_raw, now);
        state->initialized = 1;
        state->active_logged = 0;
        state->camera_seen_version = 0;
        state->root_drive_camera_seen_version = 0;
        state->root_drive_camera_quarantine_tick = 0;
        state->root_drive_camera_last_untrusted_tick = 0;
        state->root_drive_camera_ramp_log_tick = 0;
        state->root_drive_untrusted_log_tick = 0;
        state->late_ownership_log_tick = 0;
        state->ownership_candidate_tick = 0;
        state->ownership_candidate_samples = 0;
        state->ownership_candidate_root_raw = NULL;
        state->ownership_candidate_joint_raw[0] = NULL;
        state->ownership_candidate_joint_raw[1] = NULL;
        state->ownership_candidate_joint_raw[2] = NULL;
        state->root_drive_last_step_valid = 0;
        state->root_drive_last_h_step = 0.0f;
        state->root_drive_last_v_step = 0.0f;
        state->root_drive_last_d_step = 0.0f;
        memset(&state->root_drive_filter, 0, sizeof(state->root_drive_filter));
        state->dynamics_valid=state->dynamics_gravity_valid=0;
        state->camera_relative_trs_raw = NULL;
        state->camera_relative_live_prev_valid = 0;
        state->camera_relative_live_current_valid = 0;
        state->camera_relative_calibration_samples = 0;
        state->camera_relative_h_peak = 0.0f;
        state->camera_relative_v_peak = 0.0f;
        state->camera_relative_d_peak = 0.0f;
        state->camera_relative_live_log_tick = 0;
        for (i = 0; i < 9; i++) {
            state->camera_relative_live_prev[i] = 0.0f;
            state->camera_relative_live_current[i] = 0.0f;
            state->camera_relative_h_coeff[i] = 0.0f;
            state->camera_relative_v_coeff[i] = 0.0f;
            state->camera_relative_d_coeff[i] = 0.0f;
        }
        state->root_raw = root_raw;
        state->init_tick = now;
        state->gravity_probe_log_tick = 0;
        state->gravity_probe_stable_tick = 0;
        state->gravity_probe_candidate_tick = 0;
        state->gravity_probe_captured = 0;
        state->gravity_probe_promoted = 0;
        state->gravity_probe_sampled = 0;
        reset_body_chain_gravity_state(state);
        state->gravity_probe_prev_root[0] = 0.0f;
        state->gravity_probe_prev_root[1] = 0.0f;
        state->gravity_probe_prev_root[2] = 0.0f;
        state->gravity_probe_last_root[0] = 0.0f;
        state->gravity_probe_last_root[1] = 0.0f;
        state->gravity_probe_last_root[2] = 0.0f;
        for (axis = 0; axis < 3; axis++) {
            state->root_prev[axis] = root_drive_position[axis];
        }
        state->root_prev_world_valid = 0;
        for (i = 0; i < 3; i++) {
            state->joint_raw[i] = joint_raw[i];
            for (axis = 0; axis < 3; axis++) {
                if (runtime_mode) {
                    state->output_handoff_rest[i][axis] = out[i][axis];
                }
                state->rest[i][axis] =
                    cfg->zero_output_rest ? 0.0f : out[i][axis];
                if (cfg->zero_output_rest) out[i][axis] = 0.0f;
            }
            for (axis = 0; axis < 3; axis++) {
                state->angle[i][axis] = 0.0f;
                state->velocity[i][axis] = 0.0f;
            }
        }
        if (runtime_mode) {
            state->output_handoff_rest_valid = 1;
        }
        log_line("testicle-physics initialized person=\"%s\" root_raw=%p root_offset=0x%03x output_offset=0x%03x root=(%.5f,%.5f,%.5f) rest01=(%.5f,%.5f,%.5f) rest02=(%.5f,%.5f,%.5f) restEnd=(%.5f,%.5f,%.5f)",
                 person, root_raw, cfg->root_offset, cfg->output_offset,
                 root[0], root[1], root[2],
                 state->rest[0][0], state->rest[0][1], state->rest[0][2],
                 state->rest[1][0], state->rest[1][1], state->rest[1][2],
                 state->rest[2][0], state->rest[2][1], state->rest[2][2]);
        body_chain_camera_relative_orientation_step(
            person, state, root_raw,
            camera_relative_delta, NULL, &camera_relative_energy);
        if (!body_chain_restore_gravity_snapshot(state, &gravity_snapshot,
                                                 root, now, person,
                                                 "testicle_physics")) {
            body_chain_restore_room_gravity_snapshot(
                state, body_chain_active_gravity_cache(person_index, 1),
                root_raw, root, now, person, "testicle_physics");
        }
        run_body_chain_gravity_probe(
            person, state, root_raw, root, now,
            body_chain_active_gravity_cache(person_index, 1),
            "testicle_physics");
        if (runtime_mode) {
            body_chain_publish_runtime_pose(person_index, 1, out);
        }
        return;
    }

    camera_relative_step_valid =
        body_chain_camera_relative_orientation_step(
            person, state, root_raw,
            camera_relative_delta, parent_rotation_step,
            &camera_relative_energy);
    camera_neutral_translation_valid =
        body_chain_camera_neutral_pivot_step(
            person, person_index, state, root_drive_position,
            cfg->root_offset,
            NULL, camera_neutral_root_step);

    {
        DWORD root_camera_quarantine_ms =
            (DWORD)physics_environment_cfg.body_chain_camera_quarantine_ms;
        DWORD camera_age_ms = captured_camera_change_tick ?
            (now - captured_camera_change_tick) : 0xffffffffu;
        DWORD camera_quarantine_age_ms = 0xffffffffu;
        DWORD camera_settle_window_ms = root_camera_quarantine_ms + 900u;
        float camera_settle_threshold =
            physx_clampf(physics_environment_cfg.gravity_probe_invalidate_epsilon *
                         0.5f,
                         0.035f, 0.080f);
        float raw_root_step[3] = { 0.0f, 0.0f, 0.0f };
        float raw_root_step_len = 0.0f;
        int camera_changed =
            state->root_drive_camera_seen_version != captured_camera_version;
        int camera_recent =
            captured_camera_inverse_valid &&
            camera_age_ms < root_camera_quarantine_ms;
        int camera_quarantine_active = 0;
        int camera_settle_quarantine = 0;

        for (axis = 0; axis < 3; axis++) {
            raw_root_step[axis] =
                root_drive_position[axis] - state->root_prev[axis];
            raw_root_step_len += raw_root_step[axis] * raw_root_step[axis];
        }
        raw_root_step_len = (float)sqrt((double)raw_root_step_len);
        if (camera_changed || camera_recent) {
            state->root_drive_camera_quarantine_tick = now;
        }
        if (state->root_drive_camera_quarantine_tick) {
            camera_quarantine_age_ms =
                now - state->root_drive_camera_quarantine_tick;
        }
        camera_quarantine_active =
            state->root_drive_camera_quarantine_tick &&
            root_camera_quarantine_ms > 0 &&
            camera_quarantine_age_ms < root_camera_quarantine_ms;
        if (!camera_changed && !camera_recent &&
            captured_camera_inverse_valid &&
            camera_age_ms < camera_settle_window_ms &&
            state->root_drive_camera_quarantine_tick &&
            root_camera_quarantine_ms > 0 &&
            raw_root_step_len > camera_settle_threshold) {
            camera_settle_quarantine = 1;
            camera_quarantine_active = 1;
            state->root_drive_camera_quarantine_tick = now;
            camera_quarantine_age_ms = 0;
        }
        root_drive_untrusted =
            camera_changed || camera_recent || camera_quarantine_active ||
            camera_settle_quarantine;
        if (root_drive_untrusted) {
            state->root_drive_camera_last_untrusted_tick = now;
        }
        for (axis = 0; axis < 3; axis++) {
            root_step[axis] = camera_neutral_translation_valid ?
                camera_neutral_root_step[axis] : 0.0f;
            state->root_prev[axis] = root_drive_position[axis];
        }
        if (root_drive_untrusted &&
            (!state->last_log_tick ||
             now - state->last_log_tick >= 1000)) {
            state->last_log_tick = now;
            log_line("testicle-physics root-drive camera-rebased person=\"%s\" camera_version=%ld camera_age_ms=%lu quarantine_age_ms=%lu settle=%d raw_root_step_len=%.5f root=(%.5f,%.5f,%.5f) note=\"testicle root drive source is camera-contaminated; discarding root impulse only\"",
                     person, captured_camera_version,
                     (unsigned long)camera_age_ms,
                     (unsigned long)camera_quarantine_age_ms,
                     camera_settle_quarantine,
                     raw_root_step_len,
                     root[0], root[1], root[2]);
        }
        state->root_drive_camera_seen_version = captured_camera_version;
    }

    h_step =
        root_step[0] * cfg->horizontal_source_vector[0] +
        root_step[1] * cfg->horizontal_source_vector[1] +
        root_step[2] * cfg->horizontal_source_vector[2];
    v_step =
        root_step[0] * cfg->vertical_source_vector[0] +
        root_step[1] * cfg->vertical_source_vector[1] +
        root_step[2] * cfg->vertical_source_vector[2];
    d_step = root_step[cfg->translation_source_axis[2]];
    if (!root_drive_untrusted &&
        camera_relative_step_valid &&
        state->camera_relative_calibration_samples < 256u) {
        body_chain_train_camera_relative_orientation(
            state, camera_relative_delta, camera_relative_energy,
            h_step, v_step, d_step);
    }
    if (camera_relative_step_valid) {
        camera_relative_prediction_valid =
            body_chain_predict_camera_relative_orientation(
                state, camera_relative_delta, camera_relative_energy,
                &camera_relative_h_step, &camera_relative_v_step,
                &camera_relative_d_step);
    }
    if (root_drive_untrusted) {
        float held_h_step = state->root_drive_last_h_step;
        float held_v_step = state->root_drive_last_v_step;
        int held_step_valid = state->root_drive_last_step_valid;
        camera_relative_root_drive =
            camera_neutral_translation_valid ||
            camera_relative_prediction_valid;
        if (camera_neutral_translation_valid) {
            /* h/v/d already came from the camera-neutral world pivot. */
        } else if (camera_relative_prediction_valid) {
            h_step = camera_relative_h_step;
            v_step = camera_relative_v_step;
            d_step = camera_relative_d_step;
        } else {
            h_step = 0.0f;
            v_step = 0.0f;
            d_step = 0.0f;
            state->root_drive_last_step_valid = 0;
            state->root_drive_last_h_step = 0.0f;
            state->root_drive_last_v_step = 0.0f;
            state->root_drive_last_d_step = 0.0f;
        }
        if (defaults_cfg.debug &&
            (!state->root_drive_untrusted_log_tick ||
             now - state->root_drive_untrusted_log_tick >= 250)) {
            state->root_drive_untrusted_log_tick = now;
            log_line("testicle-physics camera-free-run person=\"%s\" camera_version=%ld relative_orientation=%d calibration_samples=%u relative_energy=%.7f held_step_valid=%d held_source_step=(h=%.5f,v=%.5f) applied_source_step=(h=%.5f,v=%.5f) gravity_drive=(%.4f,%.4f,%.4f) angle01=(%.3f,%.3f) note=\"camera-contaminated translation ignored; simulation uses camera-relative prediction when calibrated\"",
                     person,
                     captured_camera_version,
                     camera_relative_root_drive,
                     state->camera_relative_calibration_samples,
                     camera_relative_energy,
                     held_step_valid,
                     held_h_step,
                     held_v_step,
                     h_step,
                     v_step,
                     state->gravity_drive[0],
                     state->gravity_drive[1],
                     state->gravity_drive[2],
                     state->angle[0][0],
                     state->angle[0][1]);
        }
    }
    {
        float scale = body_motion_input_scale(elapsed_ms);
        float input[3] = { h_step * scale, v_step * scale, d_step * scale };
        float filtered[3];
        int drive_sample_valid = !root_drive_untrusted || camera_relative_root_drive;
        if (physx_absf(input[0]) < cfg->horizontal_deadzone) input[0] = 0;
        if (physx_absf(input[1]) < cfg->vertical_deadzone) input[1] = 0;
        if (physx_absf(input[2]) < cfg->translation_deadzone) input[2] = 0;
        body_motion_filter_sample(&state->root_drive_filter, input,
                                 elapsed_ms, drive_sample_valid, filtered);
        h_step = filtered[0]; v_step = filtered[1]; d_step = filtered[2];
        for (axis = 0; axis < 3; axis++) parent_rotation_step[axis] *= scale;
    }
    if (physx_absf(h_step) < cfg->horizontal_deadzone) h_step = 0.0f;
    if (physx_absf(v_step) < cfg->vertical_deadzone) v_step = 0.0f;
    if (physx_absf(d_step) < cfg->translation_deadzone) d_step = 0.0f;
    if (!root_drive_untrusted || camera_relative_root_drive) {
        state->root_drive_last_h_step = h_step;
        state->root_drive_last_v_step = v_step;
        state->root_drive_last_d_step = d_step;
        state->root_drive_last_step_valid = 1;
    }
    run_body_chain_gravity_probe(
        person, state, root_raw, root, now,
        body_chain_active_gravity_cache(person_index, 1),
        "testicle_physics");
    if (physics_environment_cfg.gravity_apply_to_body_chain &&
        physics_environment_cfg.world_gravity_probe &&
        (!state->gravity_probe_promoted ||
         (physics_environment_cfg.gravity_zero_at_start &&
          !state->gravity_drive_ref_valid))) {
        if (!runtime_mode && cfg->override_animation) {
            neutralize_testicle_physics_animation(person, state);
        }
        for (i = 0; i < 3; i++) {
            for (axis = 0; axis < 3; axis++) {
                out[i][axis] = state->rest[i][axis];
            }
        }
        if (!state->health_log_tick ||
            now - state->health_log_tick >= 2000) {
            state->health_log_tick = now;
            log_line("testicle-physics gated person=\"%s\" reason=\"waiting for atomic gravity baseline\" root=(%.5f,%.5f,%.5f) gravity_promoted=%d gravity_ref_valid=%d gravity_apply=%d note=\"neutral output; no spring accumulation\"",
                     person, root[0], root[1], root[2],
                     state->gravity_probe_promoted,
                     state->gravity_drive_ref_valid,
                     physics_environment_cfg.gravity_apply_to_body_chain);
        }
        if (runtime_mode) {
            body_chain_publish_runtime_pose(person_index, 1, out);
        }
        return;
    }
    if (!state->active_logged) {
        state->active_logged = 1;
        log_line("testicle-physics active person=\"%s\" source=\"camera-relative root\" root_offset=0x%03x write=\"Stesticles_joint01/02 + 0x%03x; Stesticles_jointEnd locked\" override_animation=%d translation_scale=(%.3f,%.3f,%.3f) rotation_scale=(%.3f,%.3f,%.3f) stiffness=%.2f damping=%.2f joint_max=(j1=%.2f/%.2f/%.2f,j2=%.2f/%.2f/%.2f) gains=(%.2f,%.2f)",
                 person, cfg->root_offset, cfg->output_offset,
                 cfg->override_animation,
                 cfg->translation_scale[0],
                 cfg->translation_scale[1],
                 cfg->translation_scale[2],
                 cfg->rotation_scale[0],
                 cfg->rotation_scale[1],
                 cfg->rotation_scale[2],
                 cfg->stiffness, cfg->damping,
                 cfg->link_max_angle[0][0],
                 cfg->link_max_angle[0][1],
                 cfg->link_max_angle[0][2],
                 cfg->link_max_angle[1][0],
                 cfg->link_max_angle[1][1],
                 cfg->link_max_angle[1][2],
                 cfg->link_gain[0],
                 cfg->link_gain[1]);
        log_line("testicle-physics gravity-response person=\"%s\" response_ms=%.1f gravity_curve=(h=%.2f,v=%.2f) inverted=(strength=%.2f,axis=%d,sign=%.2f) chain_limit=(bend=%.1f,twist=%.1f) max_degrees_per_second=%.1f note=\"positive axial gravity drives the inverted-only response; normal standing gravity remains neutral\"",
                 person,
                 physics_environment_cfg.gravity_response_ms,
                 cfg->gravity_horizontal_curve,
                 cfg->gravity_vertical_curve,
                 cfg->gravity_inverted_strength,
                 cfg->gravity_inverted_tail_axis,
                 cfg->gravity_inverted_sign,
                 cfg->chain_total_bend_max,
                 cfg->chain_total_twist_max,
                 physics_environment_cfg.gravity_max_degrees_per_second);
    }

    body_contact_begin_step(person_index, state, 1, now, dt);
    face_down_translation = body_chain_face_down_translation_active(state);
    translation_drive_step[0] = h_step;
    translation_drive_step[1] = v_step;
    translation_drive_step[2] = d_step;
    if (face_down_translation) {
        translation_drive_step[cfg->face_down_translation_channel] *=
            cfg->face_down_translation_sign;
    }
    wind_active = body_chain_room_wind_channels(
        cfg, state, state->root_raw, person, "testicle_physics", now,
        wind_channels);
    for (motion_step = 0; motion_step < motion_steps; motion_step++) {
        state->collision_step_dt = motion_dt;
        update_body_chain_gravity_filter(state, motion_dt);
        {
            float gravity_target[3][3]={{0}};
            const float *active_gravity_drive = NULL;
            float curved_gravity_drive[3];
            if (physics_environment_cfg.gravity_apply_to_body_chain &&
                state->gravity_probe_promoted) {
                active_gravity_drive = state->gravity_drive_filtered_valid ?
                    state->gravity_drive_filtered : state->gravity_drive;
                if (physx_absf(cfg->gravity_horizontal_curve - 1.0f) >
                        0.000001f ||
                    physx_absf(cfg->gravity_vertical_curve - 1.0f) >
                        0.000001f) {
                    body_chain_apply_gravity_curve(
                        active_gravity_drive,
                        cfg->gravity_horizontal_curve,
                        cfg->gravity_vertical_curve,
                        curved_gravity_drive);
                    active_gravity_drive = curved_gravity_drive;
                }
            }
            {
                float mapped_drive[3],gravity_mapped[3];
                float gravity_add_h = 0.0f;
                float gravity_add_v = 0.0f;
                if (active_gravity_drive) {
                    float gravity_secondary_drive = active_gravity_drive[2];
                    gravity_add_h = active_gravity_drive[0] *
                        physics_environment_cfg.gravity_horizontal_body_chain_scale;
                    gravity_add_h += gravity_secondary_drive *
                        physics_environment_cfg.gravity_horizontal_secondary_body_chain_scale;
                    gravity_add_v = active_gravity_drive[1] *
                        physics_environment_cfg.gravity_vertical_body_chain_scale;
                    gravity_add_v += gravity_secondary_drive *
                        physics_environment_cfg.gravity_vertical_secondary_body_chain_scale;
                }
                body_chain_build_mapped_drive(
                    cfg, translation_drive_step[0], translation_drive_step[1],
                    translation_drive_step[2], parent_rotation_step,
                    gravity_add_h, gravity_add_v, mapped_drive);
                if (active_gravity_drive &&
                    cfg->gravity_inverted_strength > 0.0f &&
                    cfg->gravity_inverted_tail_axis >= 0 &&
                    cfg->gravity_inverted_tail_axis <= 2) {
                    inverted_gravity_target =
                        body_chain_inverted_gravity_amount(
                            active_gravity_drive) *
                        cfg->gravity_inverted_strength *
                        cfg->gravity_inverted_sign;
                    mapped_drive[cfg->gravity_inverted_tail_axis] +=
                        inverted_gravity_target;
                }
                body_chain_build_mapped_drive(cfg,0,0,0,NULL,gravity_add_h,gravity_add_v,gravity_mapped);
                if(active_gravity_drive && cfg->gravity_inverted_strength>0 &&
                   cfg->gravity_inverted_tail_axis>=0 && cfg->gravity_inverted_tail_axis<3)
                    gravity_mapped[cfg->gravity_inverted_tail_axis]+=inverted_gravity_target;
                if (wind_active) {
                    body_chain_add_room_wind_target(
                        cfg, wind_channels, NULL, mapped_drive);
                }
                for (i = 0; i < 2; i++) {
                    float gain = cfg->link_gain[i];
                    for (axis = 0; axis < 3; axis++) {
                        gravity_target[i][axis]=body_chain_clamp_link_axis_angle(
                            cfg,i,axis,gravity_mapped[axis]*gain);
                        spring_target[i][axis] =
                            body_chain_clamp_link_axis_angle(
                                cfg, i, axis, mapped_drive[axis] * gain);
                    }
                }
            }
            chain_target_limited = body_chain_limit_total_rotation(
                cfg, spring_target, NULL, 2);
            body_chain_limit_total_rotation(cfg,gravity_target,NULL,2);
            body_chain_shape_gravity(person_index,state,cfg,now,motion_dt,gravity_target,spring_target);
            for (i = 0; i < 2; i++) {
                for (axis = 0; axis < 3; axis++) {
                    float axis_target = spring_target[i][axis];
                    float effective_stiffness = cfg->stiffness;
                    float effective_damping = cfg->damping;
                    if (root_drive_untrusted && !camera_relative_root_drive) {
                        effective_stiffness *=
                            physics_environment_cfg.body_chain_camera_coast_stiffness_scale;
                        effective_damping *=
                            physics_environment_cfg.body_chain_camera_coast_damping_scale;
                    }
                    body_chain_apply_link_inertia(state,i,axis,&effective_stiffness,&effective_damping);
                    body_motion_limited_spring_step(&state->angle[i][axis], &state->velocity[i][axis],
                        axis_target, effective_stiffness, effective_damping, motion_dt,
                        body_chain_link_axis_min_limit(cfg, i, axis),
                        body_chain_link_axis_limit(cfg, i, axis));
                }
            }
            chain_state_limited = body_chain_limit_total_rotation(
                cfg, state->angle, state->velocity, 2);
            body_contact_limit_velocity(cfg,state,NULL,2);
        }

        if (body_chain_collider_cfg.testicle_collision_enabled ||
            cfg->room_collision_enabled) {
            float collider_correction[3][2] = {
                { 0.0f, 0.0f },
                { 0.0f, 0.0f },
                { 0.0f, 0.0f }
            };
            float collider_max_penetration = 0.0f;
            float collider_sweep_radius = 0.0f;
            float collider_motion = 0.0f;
            body_chain_compute_collider_projection(
                person_index, state, collider_correction, now, 1,
                &collider_max_penetration,
                &collider_sweep_radius,
                &collider_motion,
                BODY_CHAIN_COLLISION_TARGET_TESTICLES);

            body_contact_apply(state, cfg, collider_correction, 2, collider_max_penetration);
        }

    } /* Substeps complete; publish only the final chain pose. */

    for (i = 0; i < 2; i++) {
        for (axis = 0; axis < 3; axis++) {
            out[i][axis] = state->rest[i][axis] +
                           state->angle[i][axis];
            out[i][axis] =
                body_chain_clamp_link_output_value(
                    cfg, i, axis, state->rest[i][axis], out[i][axis]);
        }
    }
    for (axis = 0; axis < 3; axis++) {
        out[2][axis] = state->rest[2][axis];
    }
    if (!runtime_mode && cfg->override_animation) {
        neutralize_testicle_physics_animation(person, state);
    }
    if (runtime_mode) {
        body_chain_publish_runtime_pose(person_index, 1, out);
    }
    if (defaults_cfg.debug &&
        (!state->last_log_tick || now - state->last_log_tick >= 1000)) {
        state->last_log_tick = now;
        log_line("testicle-physics write person=\"%s\" root_step=(%.5f,%.5f,%.5f) source_step=(h=%.5f,v=%.5f,d=%.5f) applied_step=(h=%.5f,v=%.5f,d=%.5f) face_down_translation=%d face_down_channel=%d face_down_sign=%.3f root_drive_untrusted=%d camera_relative_root_drive=%d camera_relative_samples=%u gravity_drive=(%.4f,%.4f,%.4f) gravity_filtered=(%.4f,%.4f,%.4f) inverted_gravity=%.3f chain_limit=(target=%d,state=%d) angle01=(%.3f,%.3f) angle02=(%.3f,%.3f) contact=(%.3f,%.3f;%.3f,%.3f) collision=(penetration=%.5f,impact_ticks=%d,manifold_contacts=%d,room_contacts=%d) out01=(%.3f,%.3f,%.3f) out02=(%.3f,%.3f,%.3f)",
                 person,
                 root_step[0], root_step[1], root_step[2],
                 h_step, v_step, d_step,
                 translation_drive_step[0], translation_drive_step[1],
                 translation_drive_step[2], face_down_translation,
                 cfg->face_down_translation_channel,
                 cfg->face_down_translation_sign,
                 root_drive_untrusted,
                 camera_relative_root_drive,
                 state->camera_relative_calibration_samples,
                 state->gravity_drive[0],
                 state->gravity_drive[1],
                 state->gravity_drive[2],
                 state->gravity_drive_filtered[0],
                 state->gravity_drive_filtered[1],
                 state->gravity_drive_filtered[2],
                 inverted_gravity_target,
                 chain_target_limited,
                 chain_state_limited,
                 state->angle[0][0], state->angle[0][1],
                 state->angle[1][0], state->angle[1][1],
                 state->collision_contact_direction[0][0],
                 state->collision_contact_direction[0][1],
                 state->collision_contact_direction[1][0],
                 state->collision_contact_direction[1][1],
                 state->collision_prev_max_penetration,
                 state->collision_impact_ticks,
                 state->collision_manifold_contacts,
                 state->collision_room_contacts,
                 out[0][0], out[0][1], out[0][2],
                 out[1][0], out[1][1], out[1][2]);
    }
    if (defaults_cfg.debug &&
        (!state->health_log_tick ||
         now - state->health_log_tick >= 5000)) {
        state->health_log_tick = now;
        log_line("testicle-physics health person=\"%s\" active=1 cache=%s interval_ms=%d elapsed_ms=%lu dt=%.4f substeps=%d step_dt=%.4f inertia=%d geometry_gravity=%d root_raw=%p joint_raw=(%p,%p,%p)",
                 person,
                 cache_hot ? "hot" : "resolved",
                 cfg->interval_ms,
                 (unsigned long)elapsed_ms,
                 dt, motion_steps, motion_dt,
                 state->dynamics_valid, state->dynamics_gravity_active,
                 root_raw,
                 joint_raw[0], joint_raw[1], joint_raw[2]);
    }
}

static int body_collider_person_physics_active(int person_index)
{
    int active = 0;
    if (person_index < 0 || person_index >= 4) return 0;
    body_profile_set_active_person_config(person_index);
    if (breasts_physics_person_enabled(person_index) &&
        (body_chain_collider_cfg.breasts_collision_enabled ||
         breasts_physics_cfg.room_collision_enabled)) {
        active = 1;
    }
    if (butt_physics_person_enabled(person_index) &&
        (body_chain_collider_cfg.butt_collision_enabled ||
         butt_physics_cfg.room_collision_enabled)) {
        active = 1;
    }
    if (body_chain_physics_cfg.enabled &&
        body_chain_physics_cfg.enabled_person[person_index] &&
        (body_chain_collider_cfg.penis_collision_enabled ||
         body_chain_physics_cfg.room_collision_enabled)) {
        active = 1;
    }
    if (!active &&
        testicle_physics_cfg.enabled &&
        testicle_physics_cfg.enabled_person[person_index] &&
        (body_chain_collider_cfg.testicle_collision_enabled ||
         testicle_physics_cfg.room_collision_enabled)) {
        active = 1;
    }
    body_profile_set_active_person_config(-1);
    return active;
}

static int body_collider_person_physics_due(int person_index, DWORD now)
{
    int due = 0;
    if (person_index < 0 || person_index >= 4) return 0;
    body_profile_set_active_person_config(person_index);
    if (breasts_physics_person_enabled(person_index) &&
        (body_chain_collider_cfg.breasts_collision_enabled ||
         breasts_physics_cfg.room_collision_enabled)) {
        breasts_physics_person_state_t *state =
            &breasts_physics_states[person_index];
        if (!state->last_tick ||
            now - state->last_tick >=
                (DWORD)breasts_physics_cfg.interval_ms) {
            due = 1;
        }
    }
    if (!due && butt_physics_person_enabled(person_index) &&
        (body_chain_collider_cfg.butt_collision_enabled ||
         butt_physics_cfg.room_collision_enabled)) {
        breasts_physics_person_state_t *state =
            &butt_physics_states[person_index];
        if (!state->last_tick ||
            now - state->last_tick >=
                (DWORD)butt_physics_cfg.interval_ms) {
            due = 1;
        }
    }
    if (!due && body_chain_physics_cfg.enabled &&
        body_chain_physics_cfg.enabled_person[person_index] &&
        (body_chain_collider_cfg.penis_collision_enabled ||
         body_chain_physics_cfg.room_collision_enabled)) {
        body_chain_person_state_t *state =
            body_chain_active_person_state(person_index, 0);
        if (!state->last_tick ||
            now - state->last_tick >=
                (DWORD)body_chain_physics_cfg.interval_ms) {
            due = 1;
        }
    }
    if (!due &&
        testicle_physics_cfg.enabled &&
        testicle_physics_cfg.enabled_person[person_index] &&
        (body_chain_collider_cfg.testicle_collision_enabled ||
         testicle_physics_cfg.room_collision_enabled)) {
        body_chain_person_state_t *state =
            body_chain_active_person_state(person_index, 1);
        if (!state->last_tick ||
            now - state->last_tick >=
                (DWORD)testicle_physics_cfg.interval_ms) {
            due = 1;
        }
    }
    body_profile_set_active_person_config(-1);
    return due;
}

static int body_collider_person_physics_uses_all_scope(int person_index)
{
    int uses_all = 0;
    if (person_index < 0 || person_index >= 4) return 0;
    body_profile_set_active_person_config(person_index);
    if (breasts_physics_person_enabled(person_index) &&
        body_chain_collider_cfg.breasts_collision_enabled &&
        body_chain_collision_scope_all_persons(
            breasts_physics_cfg.collision_scope)) {
        uses_all = 1;
    }
    if (!uses_all && butt_physics_person_enabled(person_index) &&
        body_chain_collider_cfg.butt_collision_enabled &&
        body_chain_collision_scope_all_persons(
            butt_physics_cfg.collision_scope)) {
        uses_all = 1;
    }
    if (!uses_all && body_chain_physics_cfg.enabled &&
        body_chain_physics_cfg.enabled_person[person_index] &&
        body_chain_collider_cfg.penis_collision_enabled &&
        body_chain_collision_scope_all_persons(
            body_chain_physics_cfg.collision_scope)) {
        uses_all = 1;
    }
    if (!uses_all &&
        testicle_physics_cfg.enabled &&
        testicle_physics_cfg.enabled_person[person_index] &&
        body_chain_collider_cfg.testicle_collision_enabled &&
        body_chain_collision_scope_all_persons(
            testicle_physics_cfg.collision_scope)) {
        uses_all = 1;
    }
    body_profile_set_active_person_config(-1);
    return uses_all;
}

static void body_collider_add_scope_requirement(
    int owner_index, int scope, int required_scope_mask[4])
{
    int i;
    int mask;
    if (!required_scope_mask || owner_index < 0 || owner_index >= 4) return;
    mask = body_chain_collision_scope_collider_mask(scope);
    if (body_chain_collision_scope_all_persons(scope)) {
        for (i = 0; i < 4; i++) {
            required_scope_mask[i] |= mask;
        }
    } else {
        required_scope_mask[owner_index] |= mask;
    }
}

/* Build the union of collider groups that enabled breast/Butt/penis/testicle
   systems can consume. This changes only which unused engine pivots are read;
   every required group still updates at the original simulation cadence. */
static void body_collider_collect_scope_requirements(
    int required_scope_mask[4])
{
    int owner_index;
    DWORD now = GetTickCount();
    DWORD external_tick = (DWORD)InterlockedCompareExchange(
        &body_chain_external_query_tick, 0, 0);
    int external_active = external_tick && now - external_tick <= 750u;
    if (!required_scope_mask) return;
    for (owner_index = 0; owner_index < 4; owner_index++) {
        required_scope_mask[owner_index] = 0;
    }
    for (owner_index = 0; owner_index < 4; owner_index++) {
        body_profile_set_active_person_config(owner_index);
        if (breasts_physics_person_enabled(owner_index) &&
            (body_chain_collider_cfg.breasts_collision_enabled ||
             breasts_physics_cfg.room_collision_enabled)) {
            body_collider_add_scope_requirement(
                owner_index, breasts_physics_cfg.collision_scope,
                required_scope_mask);
            required_scope_mask[owner_index] |=
                BODY_CHAIN_COLLIDER_GROUP_BREAST_SOURCE;
        }
        if (butt_physics_person_enabled(owner_index) &&
            (body_chain_collider_cfg.butt_collision_enabled ||
             butt_physics_cfg.room_collision_enabled)) {
            body_collider_add_scope_requirement(
                owner_index, butt_physics_cfg.collision_scope,
                required_scope_mask);
            required_scope_mask[owner_index] |=
                BODY_CHAIN_COLLIDER_GROUP_BUTT_SOURCE;
        }
        if (body_chain_physics_cfg.enabled &&
            body_chain_physics_cfg.enabled_person[owner_index] &&
            (body_chain_collider_cfg.penis_collision_enabled ||
             body_chain_physics_cfg.room_collision_enabled)) {
            body_collider_add_scope_requirement(
                owner_index, body_chain_physics_cfg.collision_scope,
                required_scope_mask);
        }
        if (testicle_physics_cfg.enabled &&
            testicle_physics_cfg.enabled_person[owner_index] &&
            (body_chain_collider_cfg.testicle_collision_enabled ||
             testicle_physics_cfg.room_collision_enabled)) {
            body_collider_add_scope_requirement(
                owner_index, testicle_physics_cfg.collision_scope,
                required_scope_mask);
        }
    }
    if (external_active) {
        for (owner_index = 0; owner_index < 4; owner_index++) {
            required_scope_mask[owner_index] |=
                BODY_CHAIN_COLLIDER_GROUP_ALL;
        }
    }
    body_profile_set_active_person_config(-1);
}

static void refresh_body_colliders_for_physics(DWORD now)
{
    int i;
    int any_active = 0;
    int any_due = 0;
    int refresh_all = 0;
    int due_person[4] = { 0, 0, 0, 0 };
    int required_scope_mask[4] = { 0, 0, 0, 0 };
    DWORD external_tick = (DWORD)InterlockedCompareExchange(
        &body_chain_external_query_tick, 0, 0);
    int external_active = external_tick && now - external_tick <= 750u;
    static int previous_active[4] = { 0, 0, 0, 0 };

    if ((!body_chain_collider_cfg.enabled && !room_collision_is_enabled()) ||
        !engine_FindObjC) {
        for (i = 0; i < 4; i++) {
            previous_active[i] = 0;
        }
        if (!body_chain_collider_cfg.enabled && !room_collision_is_enabled()) {
            for (i = 0; i < 4; i++) {
                if (body_chain_collider_states[i].ready ||
                    body_chain_collider_states[i].sample_ready ||
                    body_chain_collider_states[i].basis_valid) {
                    reset_body_chain_collider_state(&body_chain_collider_states[i]);
                }
            }
        }
        return;
    }

    body_collider_collect_scope_requirements(required_scope_mask);

    for (i = 0; i < 4; i++) {
        if (external_active || body_collider_person_physics_active(i)) {
            body_chain_collider_person_state_t *state =
                &body_chain_collider_states[i];
            any_active = 1;
            if (!previous_active[i] &&
                (state->ready || state->sample_ready || state->basis_valid ||
                 state->chain_points_ready || state->limb_points_ready ||
                 state->testicle_points_ready)) {
                reset_body_chain_collider_state_preserve_scene_liveness(state);
                clear_body_chain_prev_collider_for_person(i);
                log_line("body-chain-colliders activation-reset person=\"Person%02d\" note=\"physics collision was re-enabled; clearing stale collider/contact cache before following fresh TK17 live bones\"",
                         i + 1);
            }
            if (external_active || body_collider_person_physics_due(i, now)) {
                any_due = 1;
                due_person[i] = 1;
                if (body_collider_person_physics_uses_all_scope(i)) {
                    refresh_all = 1;
                }
            }
            previous_active[i] = 1;
        } else {
            previous_active[i] = 0;
        }
    }

    if (!body_chain_collider_cfg.debug_draw && !any_active) {
        for (i = 0; i < 4; i++) {
            previous_active[i] = 0;
        }
        for (i = 0; i < 4; i++) {
            if (body_chain_collider_states[i].ready ||
                body_chain_collider_states[i].sample_ready ||
                body_chain_collider_states[i].basis_valid) {
                reset_body_chain_collider_state(&body_chain_collider_states[i]);
            }
        }
        return;
    }

    if (body_chain_collider_cfg.debug_draw || any_due) {
        for (i = 0; i < 4; i++) {
            if (!body_chain_collider_cfg.debug_draw &&
                !refresh_all && !due_person[i]) {
                continue;
            }
            body_profile_set_active_person_config(i);
            update_body_chain_colliders_for_person_scope(
                i, now,
                body_chain_collider_cfg.debug_draw ? 3 :
                required_scope_mask[i] ?
                    required_scope_mask[i] :
                    BODY_CHAIN_COLLIDER_GROUP_ALL_TARGETS);
            body_profile_set_active_person_config(-1);
        }
    }
}

static void body_chain_mark_runtime_transform_dirty(void *object)
{
    unsigned int version;
    if (!object || !engine_TBaseTransformGetMatrixVersion ||
        !engine_TBaseTransformSetMatrixVersion ||
        !ptr_readable(object, sizeof(void*))) {
        return;
    }
    version = engine_TBaseTransformGetMatrixVersion(object);
    engine_TBaseTransformSetMatrixVersion(object, version + 1u);
}

static int body_chain_write_runtime_identity(void *object)
{
    float *rotation;
    float *row0;
    float *row1;
    float *row2;
    if (!body_chain_runtime_tjoint_layout_valid(object) ||
        !ptr_readable((BYTE*)object + 0x06c, sizeof(float) * 3)) {
        return 0;
    }
    rotation = (float*)((BYTE*)object + 0x06c);
    row0 = (float*)((BYTE*)object + 0x078);
    row1 = (float*)((BYTE*)object + 0x088);
    row2 = (float*)((BYTE*)object + 0x098);
    rotation[0] = 0.0f;
    rotation[1] = 0.0f;
    rotation[2] = 0.0f;
    row0[0] = 1.0f; row0[1] = 0.0f; row0[2] = 0.0f;
    row1[0] = 0.0f; row1[1] = 1.0f; row1[2] = 0.0f;
    row2[0] = 0.0f; row2[1] = 0.0f; row2[2] = 1.0f;
    body_chain_mark_runtime_transform_dirty(object);
    return 1;
}

static int body_chain_capture_runtime_animation_rows(
    runtime_body_chain_ownership_state_t *ownership)
{
    int i;
    if (!ownership) return 0;
    for (i = 0; i < 3; i++) {
        void *object = ownership->animation_joint_raw[i];
        if (!body_chain_runtime_tjoint_layout_valid(object) ||
            !ptr_readable((BYTE*)object + 0x06c,
                          sizeof(float) * 3)) {
            return 0;
        }
        memcpy(ownership->animation_rotation[i],
               (BYTE*)object + 0x06c, sizeof(float) * 3);
        memcpy(&ownership->animation_rows[i][0],
               (BYTE*)object + 0x078, sizeof(float) * 3);
        memcpy(&ownership->animation_rows[i][3],
               (BYTE*)object + 0x088, sizeof(float) * 3);
        memcpy(&ownership->animation_rows[i][6],
               (BYTE*)object + 0x098, sizeof(float) * 3);
    }
    ownership->animation_rows_valid = 1;
    return 1;
}

static int body_chain_runtime_penis_person_enabled_now(int person_index,
                                                       DWORD now)
{
    const body_chain_physics_config_t *cfg;
    if (person_index < 0 || person_index >= 4) return 0;
    cfg = &body_chain_physics_person_cfg[person_index];
    if (!cfg->enabled || !cfg->enabled_person[person_index]) return 0;
    if (body_chain_physics_settings_change_tick[person_index] &&
        !body_chain_physics_settings_change_enabled[person_index] &&
        now - body_chain_physics_settings_change_tick[person_index] <=
            3000u) {
        return 0;
    }
    return 1;
}

/* FreeMode pose clips are evaluated through TK17's normal script-member
   setters.  Filtering at that boundary prevents the authored penis channels
   from entering the live transform graph while PhysX owns the chain.  The
   setter hooks are never active for PoseEditor. The latest authored values
   are retained solely for an exact FreeMode handoff when PhysX is disabled. */
static int body_chain_runtime_should_neutralize_penis_animation_write(
    void *object, DWORD member_id, const float *value)
{
    DWORD now;
    int person_index;
    int joint_index;
    if (!object || !value ||
        !ptr_readable(value, sizeof(float) * 3) ||
        !body_chain_vec3_sane_limit(value, 720.0f) ||
        !body_chain_runtime_mode_active() ||
        !InterlockedCompareExchange(&body_chain_runtime_ownership_active,
                                    0, 0) ||
        (member_id != SCRIPT_PROPERTY_SSIMPLE_ROTATION &&
         member_id != SCRIPT_PROPERTY_SJOINT_ROTATION_AXIS)) {
        return 0;
    }
    for (person_index = 0; person_index < 4; person_index++) {
        runtime_body_chain_ownership_state_t *ownership =
            &runtime_body_chain_ownership_states[person_index];
        const body_chain_physics_config_t *cfg =
            &body_chain_physics_person_cfg[person_index];
        for (joint_index = 0; joint_index < 3; joint_index++) {
            if (object != ownership->animation_joint_raw[joint_index]) {
                continue;
            }
            now = GetTickCount();
            if (!cfg->override_animation || !ownership->mapping_ready ||
                !ownership->pose_valid ||
                !body_chain_runtime_penis_person_enabled_now(
                    person_index, now)) {
                return 0;
            }
            memcpy(ownership->animation_rotation[joint_index], value,
                   sizeof(float) * 3);
            ownership->animation_value_valid_mask |= 1u << joint_index;
            if (!ownership->runtime_writer_logged) {
                ownership->runtime_writer_logged = 1;
                log_line("body-chain-physics FreeMode animation writer filtered person=\"%s\" animation_joints=(%p,%p,%p) member=0x%08lx note=\"TK17's native setter receives a neutral value only for this active penis chain; PoseEditor and every unrelated animation channel remain untouched\"",
                         body_chain_person_name(person_index),
                         ownership->animation_joint_raw[0],
                         ownership->animation_joint_raw[1],
                         ownership->animation_joint_raw[2],
                         (unsigned long)member_id);
            }
            return 1;
        }
    }
    return 0;
}

/* FreeMode has a second penis animation channel: the engine's
   penis_raScheduler writes SJoint.RotationAxis directly on the mapped
   Spenis joints.  PhysX writes SSimpleTransform.Rotation, so leaving this
   axis channel live composes the authored animation on top of the solver.
   Capture it for the OFF handoff and neutralize it only for the exact
   runtime chain currently owned by PhysX. */
static int
body_chain_runtime_should_neutralize_penis_source_axis_write(
    void *object, DWORD member_id, const float *value)
{
    int person_index;
    int joint_index;
    if (!object || !value ||
        member_id != SCRIPT_PROPERTY_SJOINT_ROTATION_AXIS ||
        !ptr_readable(value, sizeof(float) * 3) ||
        !body_chain_vec3_sane_limit(value, 720.0f) ||
        !body_chain_runtime_mode_active() ||
        !InterlockedCompareExchange(&body_chain_runtime_ownership_active,
                                    0, 0)) {
        return 0;
    }
    for (person_index = 0; person_index < 4; person_index++) {
        runtime_body_chain_ownership_state_t *ownership =
            &runtime_body_chain_ownership_states[person_index];
        const body_chain_physics_config_t *cfg =
            &body_chain_physics_person_cfg[person_index];
        if (!ownership->ownership_active || !ownership->mapping_ready ||
            !ownership->pose_valid || !cfg->override_animation ||
            !body_chain_runtime_penis_person_enabled_now(
                person_index, GetTickCount())) {
            continue;
        }
        for (joint_index = 0; joint_index < 3; joint_index++) {
            if (object != ownership->source_joint_raw[joint_index]) {
                continue;
            }
            memcpy(
                ownership->native_source_axis_animation_rotation[
                    joint_index],
                value, sizeof(float) * 3);
            ownership->native_source_axis_animation_valid_mask |=
                1u << joint_index;
            if (!(ownership->native_source_axis_writer_logged &
                  (1u << joint_index))) {
                ownership->native_source_axis_writer_logged |=
                    1u << joint_index;
                log_line("body-chain-physics FreeMode source-axis writer filtered person=\"%s\" joint=%d source_joint=%p member=0x%08lx captured_authored=(%.3f,%.3f,%.3f) captured_mask=0x%x note=\"TK17's penis RotationAxis animation is retained for the OFF handoff but neutralized while PhysX owns the chain; PoseEditor is untouched\"",
                         body_chain_person_name(person_index),
                         joint_index + 1,
                         ownership->source_joint_raw[joint_index],
                         (unsigned long)member_id,
                         value[0], value[1], value[2],
                         ownership
                             ->native_source_axis_animation_valid_mask);
            }
            return 1;
        }
    }
    return 0;
}

/* Replay the latest values authored by FreeMode through the same native
   setters that supplied them. This makes the OFF handoff exact even for a
   static pose whose animation channels are not evaluated again afterward. */
static int body_chain_restore_runtime_penis_animation_values(
    runtime_body_chain_ownership_state_t *ownership)
{
    int i;
    int restored = 0;
    if (!ownership) return 0;
    for (i = 0; i < 3; i++) {
        DWORD member_id = i == 0
            ? SCRIPT_PROPERTY_SSIMPLE_ROTATION
            : SCRIPT_PROPERTY_SJOINT_ROTATION_AXIS;
        script_vector3_set_property_t setter = i == 0
            ? real_SSimpleTransform_RotationSet
            : real_SJoint_RotationAxisSet;
        if (!(ownership->animation_value_valid_mask & (1u << i)) ||
            !setter || !ownership->animation_joint_raw[i] ||
            !body_chain_vec3_sane_limit(
                ownership->animation_rotation[i], 720.0f)) {
            continue;
        }
        setter(ownership->animation_joint_raw[i], member_id,
               ownership->animation_rotation[i]);
        restored |= 1u << i;
    }
    return restored;
}

/* Replay the latest values observed at TK17's real FreeMode output-joint
   writer. Ownership must already be released so the lower-level hook passes
   these calls through normally and the native wrapper updates its flags. */
static unsigned int
body_chain_restore_runtime_native_output_animation_values(
    runtime_body_chain_ownership_state_t *ownership,
    void *live_output_joint[3])
{
    unsigned int restored = 0;
    int i;
    if (!ownership || !live_output_joint ||
        !real_SSimpleTransform_RotationSet) {
        return 0;
    }
    for (i = 0; i < 3; i++) {
        if (!(ownership->native_output_animation_valid_mask &
              (1u << i)) ||
            !live_output_joint[i] ||
            live_output_joint[i] != ownership->source_joint_raw[i] ||
            !body_chain_vec3_sane_limit(
                ownership->native_output_animation_rotation[i], 720.0f)) {
            continue;
        }
        real_SSimpleTransform_RotationSet(
            live_output_joint[i], SCRIPT_PROPERTY_SSIMPLE_ROTATION,
            ownership->native_output_animation_rotation[i]);
        restored |= 1u << i;
    }
    return restored;
}

static void body_chain_set_runtime_source_axis_native(
    void *joint, const float value[3])
{
    if (!joint || !value) return;
    if (tramp_RuntimeJointRotationAxisWrite) {
        tramp_RuntimeJointRotationAxisWrite(
            joint, SCRIPT_PROPERTY_SJOINT_ROTATION_AXIS, value);
    } else if (real_SJoint_RotationAxisSet &&
               real_SJoint_RotationAxisSet !=
                   hook_SJoint_RotationAxisSet) {
        real_SJoint_RotationAxisSet(
            joint, SCRIPT_PROPERTY_SJOINT_ROTATION_AXIS, value);
    }
}

/* SJoint.RotationAxis is independent from the Rotation vector written by
   the solver. Keep it neutral while runtime PhysX owns the chain, otherwise
   TK17's penis animation scheduler is composed on top of the simulated
   result. The first live value is retained for an exact OFF handoff. */
static int body_chain_neutralize_runtime_penis_source_axes(
    runtime_body_chain_ownership_state_t *ownership)
{
    static const float neutral[3] = { 0.0f, 0.0f, 0.0f };
    int i;
    if (!ownership || !tramp_RuntimeJointRotationAxisWrite) return 0;
    for (i = 0; i < 3; i++) {
        const float *current;
        void *joint = ownership->source_joint_raw[i];
        if (!joint ||
            !ptr_readable((BYTE*)joint + 0x0d8, sizeof(float) * 3)) {
            return 0;
        }
        current = (const float*)((BYTE*)joint + 0x0d8);
        if (!body_chain_vec3_sane_limit(current, 720.0f)) return 0;
        if (!(ownership->native_source_axis_animation_valid_mask &
              (1u << i))) {
            memcpy(
                ownership->native_source_axis_animation_rotation[i],
                current, sizeof(float) * 3);
            ownership->native_source_axis_animation_valid_mask |= 1u << i;
        }
        if (fabsf(current[0]) > 0.00001f ||
            fabsf(current[1]) > 0.00001f ||
            fabsf(current[2]) > 0.00001f) {
            body_chain_set_runtime_source_axis_native(joint, neutral);
        }
    }
    return 1;
}

static unsigned int
body_chain_restore_runtime_penis_source_axis_values(
    runtime_body_chain_ownership_state_t *ownership,
    void *live_output_joint[3])
{
    unsigned int restored = 0;
    int i;
    if (!ownership || !live_output_joint) return 0;
    for (i = 0; i < 3; i++) {
        if (!(ownership->native_source_axis_animation_valid_mask &
              (1u << i)) ||
            !live_output_joint[i] ||
            live_output_joint[i] != ownership->source_joint_raw[i] ||
            !body_chain_vec3_sane_limit(
                ownership->native_source_axis_animation_rotation[i],
                720.0f)) {
            continue;
        }
        body_chain_set_runtime_source_axis_native(
            live_output_joint[i],
            ownership->native_source_axis_animation_rotation[i]);
        restored |= 1u << i;
    }
    return restored;
}

static void body_chain_runtime_forget_penis_pose_tracks(
    body_chain_person_state_t *state)
{
    int i;
    if (!state) return;
    state->pose_track_suppressed = 0;
    state->pose_track_base = NULL;
    state->pose_track_saved_obj = NULL;
    state->pose_track_saved_track_data = NULL;
    for (i = 0; i < POSEEDIT_EXTRA_PHYSICS_TRACK_COUNT; i++) {
        state->pose_track_extra_suppressed[i] = 0;
        state->pose_track_extra_base[i] = NULL;
        state->pose_track_extra_saved_obj[i] = NULL;
        state->pose_track_extra_saved_track_data[i] = NULL;
    }
}

/* Pose loads may replace PoseEdit::EditPose without invoking InitTracks again.
   The capture hook is therefore only a way to find the owning PoseEdit object;
   runtime ownership must read its current EditPose pointer before touching a
   track. */
static BYTE *body_chain_runtime_live_editpose(void)
{
    BYTE *poseedit = (BYTE*)captured_poseedit_this;
    BYTE *editpose;
    if (!poseedit ||
        !ptr_readable(poseedit + POSEEDIT_EDITPOSE_OFFSET,
                      sizeof(editpose))) {
        return NULL;
    }
    editpose = *(BYTE**)(poseedit + POSEEDIT_EDITPOSE_OFFSET);
    if (!editpose ||
        !ptr_readable(editpose + POSEEDIT_TRACKS_OFFSET,
                      POSEEDIT_TRACK_SIZE)) {
        return NULL;
    }
    return editpose;
}

static BYTE *body_chain_runtime_find_pose_track(
    BYTE *editpose, int person_index, int track_id,
    void *expected_obj, void *expected_raw,
    int neighbor_track_id, void *neighbor_obj, void *neighbor_raw)
{
    BYTE *base;
    BYTE *found;
    int track_index;
    if (!editpose || person_index < 0 || person_index >= 4 ||
        track_id < 0 || !expected_obj) {
        return NULL;
    }

    track_index = person_index *
        body_chain_physics_cfg.poseeditor_total_tracks + track_id;
    base = editpose + captured_poseedit_tracks_offset +
        track_index * POSEEDIT_TRACK_SIZE;
    if (ptr_readable(base, POSEEDIT_TRACK_SIZE) &&
        (validate_poseeditor_track_slot(base, expected_obj) ||
         validate_poseeditor_track_slot(base, expected_raw))) {
        return base;
    }

    found = scan_poseedit_candidate_for_track(
        editpose, person_index, track_id, expected_obj,
        neighbor_obj, neighbor_track_id, POSEEDIT_EDITPOSE_OFFSET);
    if (!found && expected_raw) {
        found = scan_poseedit_candidate_for_track(
            editpose, person_index, track_id, expected_raw,
            neighbor_raw, neighbor_track_id,
            POSEEDIT_EDITPOSE_OFFSET);
    }
    return found;
}

/* FreeMode loads PoseEditor-authored poses into the same EditPose table, but
   evaluates them later through AppBase::ProcessAnimation. Detach only this
   person's validated penis tracks while runtime PhysX owns the chain. This
   performs no native track evaluation and never runs in PoseEditor mode. */
static int body_chain_runtime_suppress_penis_pose_tracks(
    int person_index, const char *person,
    body_chain_person_state_t *state)
{
    static const int track_ids[3] = {
        POSEEDIT_TRACK_PENIS_JOINT01,
        POSEEDIT_TRACK_PENIS_JOINT02,
        POSEEDIT_TRACK_PENIS_JOINT03
    };
    static const char *joint_suffixes[3] = {
        "penis_joint01", "penis_joint02", "penis_joint03"
    };
    runtime_body_chain_ownership_state_t *ownership =
        body_chain_runtime_ownership_state(person_index, 0);
    void *nil_weak = engine_G_NilWeakObjTarget_ptr
        ? *engine_G_NilWeakObjTarget_ptr : NULL;
    void *null_array = engine_G_NullArray_ptr
        ? *engine_G_NullArray_ptr : NULL;
    BYTE *track[3] = { NULL, NULL, NULL };
    BYTE *live_editpose;
    void *joint_raw[3] = { NULL, NULL, NULL };
    void *joint_obj[3] = { NULL, NULL, NULL };
    void *target[3] = { NULL, NULL, NULL };
    void *track_data[3] = { NULL, NULL, NULL };
    int patch_track[3] = { 0, 0, 0 };
    unsigned int suppressed_mask = 0;
    int table_replaced = 0;
    int pose_rebound = 0;
    int i;

    if (!body_chain_runtime_mode_active() ||
        !body_chain_physics_cfg.override_animation) {
        return 1;
    }
    if (person_index < 0 || person_index >= 4 || !person || !state ||
        !ownership || !nil_weak || !null_array) {
        return 1;
    }

    live_editpose = body_chain_runtime_live_editpose();
    if (!live_editpose) {
        return 1;
    }

    if (ownership->pose_editpose &&
        ownership->pose_editpose != live_editpose) {
        /* The old table belongs to the previous pose and may already have
           been freed. Never write saved bindings back into stale memory. */
        body_chain_runtime_forget_penis_pose_tracks(state);
        ownership->ownership_active = 0;
        ownership->animation_rows_valid = 0;
        ownership->ownership_logged = 0;
        table_replaced = 1;
    }
    if (captured_poseedit_editpose != live_editpose) {
        void *old_editpose = captured_poseedit_editpose;
        captured_poseedit_editpose = live_editpose;
        captured_poseedit_tracks_offset = POSEEDIT_TRACKS_OFFSET;
        captured_poseedit_tracks_offset_detected = 0;
        captured_poseedit_tick = GetTickCount();
        log_line("body-chain-physics runtime live pose table changed old_editpose=%p live_editpose=%p poseedit=%p note=\"following PoseEdit's current table after a FreeMode pose replacement; stale memory is not restored or modified\"",
                 old_editpose, live_editpose, captured_poseedit_this);
    }
    ownership->pose_editpose = live_editpose;

    for (i = 0; i < 3; i++) {
        char joint_name[256];
        make_body_runtime_name(joint_name, sizeof(joint_name),
                               person, joint_suffixes[i]);
        joint_obj[i] = resolve_find_obj(joint_name, &joint_raw[i]);
        if ((!joint_obj[i] ||
             is_nil_engine_object(joint_raw[i], joint_obj[i])) &&
            captured_script_engine) {
            joint_obj[i] = resolve_script_engine_obj(
                joint_name, &joint_raw[i]);
        }
        if (!joint_obj[i] ||
            is_nil_engine_object(joint_raw[i], joint_obj[i])) {
            return 1;
        }
    }

    for (i = 0; i < 3; i++) {
        int neighbor = i == 0 ? 1 : i - 1;
        track[i] = body_chain_runtime_find_pose_track(
            live_editpose, person_index, track_ids[i],
            joint_obj[i], joint_raw[i], track_ids[neighbor],
            joint_obj[neighbor], joint_raw[neighbor]);
        if (!track[i] ||
            !ptr_readable(track[i], POSEEDIT_TRACK_SIZE)) {
            if (!state->pose_track_logged) {
                state->pose_track_logged = 1;
                log_line("body-chain-physics runtime track unavailable person=\"%s\" track_id=%d editpose=%p note=\"live track could not be resolved; existing downstream PhysX behavior continues and no track memory is modified\"",
                         person, track_ids[i], live_editpose);
            }
            return 1;
        }
    }

    if ((state->pose_track_suppressed &&
         state->pose_track_base != track[0]) ||
        (state->pose_track_extra_suppressed[0] &&
         state->pose_track_extra_base[0] != track[1]) ||
        (state->pose_track_extra_suppressed[1] &&
         state->pose_track_extra_base[1] != track[2])) {
        restore_poseeditor_joint01_track(state);
        ownership->ownership_active = 0;
        ownership->animation_rows_valid = 0;
        ownership->ownership_logged = 0;
        table_replaced = 1;
    }

    /* A pose loader may repopulate an existing allocation at the same
       address. Detect that separately from a pointer replacement so the OFF
       handoff is captured from the new pose, not the preceding one. */
    for (i = 0; i < 3; i++) {
        int was_suppressed = i == 0
            ? state->pose_track_suppressed
            : state->pose_track_extra_suppressed[i - 1];
        void *saved_base = i == 0
            ? state->pose_track_base
            : state->pose_track_extra_base[i - 1];
        if (was_suppressed && saved_base == track[i] &&
            (*(void**)(track[i] + 0x04) != nil_weak ||
             *(void**)(track[i] + 0x24) != null_array)) {
            pose_rebound = 1;
        }
    }
    if (pose_rebound) {
        ownership->ownership_active = 0;
        ownership->animation_rows_valid = 0;
        ownership->ownership_logged = 0;
    }

    if (table_replaced || pose_rebound) {
        state->output_handoff_rest_valid = state->initialized;
        ownership->pose_valid = 0;
        for (i = 0; i < 3 && state->output_handoff_rest_valid; i++) {
            float *live_output;
            if (!state->joint_raw[i] ||
                state->joint_raw[i] != ownership->source_joint_raw[i] ||
                !ptr_readable((BYTE*)state->joint_raw[i] +
                                  body_chain_physics_cfg.output_offset,
                              sizeof(float) * 3)) {
                state->output_handoff_rest_valid = 0;
                break;
            }
            live_output = (float*)((BYTE*)state->joint_raw[i] +
                                   body_chain_physics_cfg.output_offset);
            if (!body_chain_vec3_sane_limit(live_output, 720.0f)) {
                state->output_handoff_rest_valid = 0;
                break;
            }
            memcpy(state->output_handoff_rest[i], live_output,
                   sizeof(float) * 3);
        }
    }

    for (i = 0; i < 3; i++) {
        int already_suppressed = i == 0
            ? state->pose_track_suppressed
            : state->pose_track_extra_suppressed[i - 1];
        void *saved_base = i == 0
            ? state->pose_track_base
            : state->pose_track_extra_base[i - 1];

        if (already_suppressed && saved_base == track[i] &&
            *(void**)(track[i] + 0x04) == nil_weak &&
            *(void**)(track[i] + 0x24) == null_array) {
            suppressed_mask |= 1u << i;
            continue;
        }

        target[i] = *(void**)(track[i] + 0x04);
        track_data[i] = *(void**)(track[i] + 0x24);
        if (target[i] == nil_weak || track_data[i] == null_array) {
            continue;
        }
        if ((target[i] != joint_obj[i] && target[i] != joint_raw[i]) ||
            !ptr_readable(target[i], sizeof(DWORD)) ||
            !track_data[i] ||
            !ptr_readable((BYTE*)track_data[i] - sizeof(int),
                          sizeof(int))) {
            if (!state->pose_track_logged) {
                state->pose_track_logged = 1;
                log_line("body-chain-physics runtime track mismatch person=\"%s\" track_id=%d track=%p target=%p expected_obj=%p expected_raw=%p data=%p editpose=%p note=\"runtime track was not modified\"",
                         person, track_ids[i], track[i], target[i],
                         joint_obj[i], joint_raw[i], track_data[i],
                         live_editpose);
            }
            return 1;
        }
        patch_track[i] = 1;
    }

    for (i = 0; i < 3; i++) {
        DWORD old;
        if (!patch_track[i]) continue;
        if (!VirtualProtect(track[i], POSEEDIT_TRACK_SIZE,
                            PAGE_READWRITE, &old)) {
            restore_poseeditor_joint01_track(state);
            return 1;
        }
        if (i == 0) {
            state->pose_track_base = track[i];
            state->pose_track_saved_obj = target[i];
            state->pose_track_saved_track_data = track_data[i];
            state->pose_track_suppressed = 1;
        } else {
            state->pose_track_extra_base[i - 1] = track[i];
            state->pose_track_extra_saved_obj[i - 1] = target[i];
            state->pose_track_extra_saved_track_data[i - 1] =
                track_data[i];
            state->pose_track_extra_suppressed[i - 1] = 1;
        }
        *(void**)(track[i] + 0x04) = nil_weak;
        *(void**)(track[i] + 0x24) = null_array;
        VirtualProtect(track[i], POSEEDIT_TRACK_SIZE, old, &old);
        suppressed_mask |= 1u << i;
    }

    if ((suppressed_mask || table_replaced || pose_rebound) &&
        (!state->pose_track_extra_logged || table_replaced ||
         pose_rebound)) {
        state->pose_track_extra_logged = 1;
        log_line("body-chain-physics runtime tracks suppressed person=\"%s\" tracks=(%d,%d,%d) mask=0x%x editpose=%p table_replaced=%d pose_rebound=%d note=\"validated FreeMode pose inputs detached without native calls; PoseEditor mode is untouched\"",
                 person, track_ids[0], track_ids[1], track_ids[2],
                 suppressed_mask, live_editpose,
                 table_replaced, pose_rebound);
    }
    return 1;
}

static int body_chain_release_runtime_ownership_for_person(
    int person_index, int testicle_chain, const char *reason)
{
    runtime_body_chain_ownership_state_t *ownership =
        body_chain_runtime_ownership_state(person_index, testicle_chain);
    body_chain_person_state_t *state = testicle_chain
        ? &runtime_testicle_physics_states[person_index]
        : &runtime_body_chain_person_states[person_index];
    body_chain_physics_config_t *cfg = testicle_chain
        ? &testicle_physics_cfg : &body_chain_physics_cfg;
    void *live_root = NULL;
    void *live_output_joint[3] = { NULL, NULL, NULL };
    void *live_animation_joint[3] = { NULL, NULL, NULL };
    const float (*handoff)[3];
    int output_valid;
    int animation_valid;
    int output_restored = 0;
    int animation_restored = 0;
    int i;
    if (person_index < 0 || person_index >= 4 || !ownership ||
        (!ownership->ownership_active && !ownership->pose_valid &&
         !state->initialized && !state->pose_track_suppressed &&
         !state->pose_track_extra_suppressed[0] &&
         !state->pose_track_extra_suppressed[1])) {
        return 0;
    }
    if (!testicle_chain) {
        unsigned int restored_tracks =
            restore_poseeditor_joint01_track(state);
        unsigned int restored_animation = 0;
        unsigned int restored_native_output = 0;
        unsigned int restored_native_axis = 0;
        output_valid = resolve_body_chain_raws(
            body_chain_person_name(person_index),
            &live_root, live_output_joint);
        if (output_valid && state->initialized &&
            live_root == state->root_raw &&
            state->output_handoff_rest_valid) {
            output_restored = 1;
            for (i = 0; i < 3; i++) {
                if (live_output_joint[i] != state->joint_raw[i] ||
                    live_output_joint[i] !=
                        ownership->source_joint_raw[i] ||
                    !ptr_readable((BYTE*)live_output_joint[i] +
                                      cfg->output_offset,
                                  sizeof(float) * 3) ||
                    !body_chain_vec3_sane_limit(
                        state->output_handoff_rest[i], 720.0f)) {
                    output_restored = 0;
                    break;
                }
            }
            if (output_restored) {
                for (i = 0; i < 3; i++) {
                    memcpy((BYTE*)live_output_joint[i] +
                               cfg->output_offset,
                           state->output_handoff_rest[i],
                           sizeof(float) * 3);
                }
            }
        }
        /* Release first so replaying the captured native values goes through
           TK17's normal writer instead of being held by our ownership hook. */
        ownership->ownership_active = 0;
        body_chain_refresh_runtime_penis_native_filter();
        restored_native_output =
            body_chain_restore_runtime_native_output_animation_values(
                ownership, live_output_joint);
        restored_native_axis =
            body_chain_restore_runtime_penis_source_axis_values(
                ownership, live_output_joint);
        restored_animation =
            body_chain_restore_runtime_penis_animation_values(ownership);
        ownership->pose_valid = 0;
        ownership->animation_rows_valid = 0;
        ownership->animation_value_valid_mask = 0;
        ownership->native_output_animation_valid_mask = 0;
        ownership->native_source_axis_animation_valid_mask = 0;
        ownership->runtime_writer_logged = 0;
        ownership->native_output_writer_logged = 0;
        ownership->native_source_axis_writer_logged = 0;
        ownership->ownership_logged = 0;
        log_line("body-chain-physics runtime-ownership released person=\"%s\" tracks_restored=0x%x output_restored=%d native_output_animation_restored=0x%x native_source_axis_restored=0x%x legacy_animation_restored=0x%x reason=\"%s\" note=\"the latest authored FreeMode rotation and RotationAxis values are replayed through TK17's normal setters\"",
                 body_chain_person_name(person_index),
                 restored_tracks, output_restored,
                 restored_native_output, restored_native_axis,
                 restored_animation,
                 reason ? reason : "runtime-release");
        return restored_tracks || output_restored ||
               restored_native_output || restored_native_axis ||
               restored_animation;
    }
    output_valid = testicle_chain
        ? resolve_testicle_physics_raws(body_chain_person_name(person_index),
                                       &live_root, live_output_joint)
        : resolve_body_chain_raws(body_chain_person_name(person_index),
                                  &live_root, live_output_joint);
    animation_valid = body_chain_resolve_runtime_animation_joints(
        body_chain_person_name(person_index), testicle_chain,
        live_animation_joint);
    handoff = state->output_handoff_rest_valid
        ? state->output_handoff_rest : state->rest;
    if (output_valid && state->initialized && live_root == state->root_raw) {
        output_restored = 1;
        for (i = 0; i < 3; i++) {
            if ((ownership->source_joint_raw[i] &&
                 live_output_joint[i] !=
                     ownership->source_joint_raw[i]) ||
                live_output_joint[i] != state->joint_raw[i] ||
                !ptr_readable((BYTE*)live_output_joint[i] +
                                  cfg->output_offset,
                              sizeof(float) * 3) ||
                !body_chain_vec3_sane_limit(handoff[i], 720.0f)) {
                output_restored = 0;
                break;
            }
        }
        if (output_restored) {
            for (i = 0; i < 3; i++) {
                memcpy((BYTE*)live_output_joint[i] + cfg->output_offset,
                       handoff[i], sizeof(float) * 3);
            }
        }
    }
    if (cfg->override_animation && ownership->animation_rows_valid &&
        animation_valid) {
        animation_restored = 1;
        for (i = 0; i < 3; i++) {
            if (live_animation_joint[i] !=
                    ownership->animation_joint_raw[i] ||
                !body_chain_runtime_tjoint_layout_valid(
                    live_animation_joint[i])) {
                animation_restored = 0;
                break;
            }
        }
        if (animation_restored) {
            for (i = 0; i < 3; i++) {
                memcpy((BYTE*)live_animation_joint[i] + 0x078,
                       &ownership->animation_rows[i][0],
                       sizeof(float) * 3);
                memcpy((BYTE*)live_animation_joint[i] + 0x088,
                       &ownership->animation_rows[i][3],
                       sizeof(float) * 3);
                memcpy((BYTE*)live_animation_joint[i] + 0x098,
                       &ownership->animation_rows[i][6],
                       sizeof(float) * 3);
                body_chain_mark_runtime_transform_dirty(
                    live_animation_joint[i]);
            }
        }
    }
    ownership->ownership_active = 0;
    ownership->pose_valid = 0;
    ownership->animation_rows_valid = 0;
    ownership->runtime_writer_logged = 0;
    ownership->ownership_logged = 0;
    log_line("%s runtime-ownership released person=\"%s\" output_restored=%d animation_restored=%d pose_tracks_untouched=1 reason=\"%s\"",
             testicle_chain ? "testicle-physics" : "body-chain-physics",
             body_chain_person_name(person_index),
             output_restored, animation_restored,
             reason ? reason : "runtime-release");
    return output_restored || animation_restored;
}

static void body_chain_release_all_runtime_ownership(const char *reason)
{
    int i;
    for (i = 0; i < 4; i++) {
        body_profile_set_active_person_config(i);
        body_chain_release_runtime_ownership_for_person(i, 0, reason);
        body_chain_release_runtime_ownership_for_person(i, 1, reason);
    }
    body_profile_set_active_person_config(-1);
    InterlockedExchange(&body_chain_runtime_ownership_active, 0);
    InterlockedExchange(&body_chain_traverse_overlay_active, 0);
    InterlockedExchange(&body_chain_runtime_penis_native_filter_active, 0);
}

static void reset_runtime_body_chain_physics(const char *reason)
{
    int i;
    body_chain_release_all_runtime_ownership(reason);
    for (i = 0; i < 4; i++) {
        reset_body_chain_person_state(
            &runtime_body_chain_person_states[i]);
        reset_body_chain_person_state(
            &runtime_testicle_physics_states[i]);
    }
    memset(runtime_body_chain_ownership_states, 0,
           sizeof(runtime_body_chain_ownership_states));
    memset(runtime_testicle_ownership_states, 0,
           sizeof(runtime_testicle_ownership_states));
    memset(runtime_body_chain_axis_reference_cache, 0,
           sizeof(runtime_body_chain_axis_reference_cache));
    memset(runtime_breasts_axis_reference_cache, 0,
           sizeof(runtime_breasts_axis_reference_cache));
    memset(runtime_body_chain_room_gravity_cache, 0,
           sizeof(runtime_body_chain_room_gravity_cache));
    memset(runtime_testicle_room_gravity_cache, 0,
           sizeof(runtime_testicle_room_gravity_cache));
}

static void body_chain_prepare_runtime_mode_from_poseeditor(void)
{
    int i;
    for (i = 0; i < 4; i++) {
        body_chain_person_state_t *penis_state =
            &body_chain_person_states[i];
        body_chain_person_state_t *testicle_state =
            &testicle_physics_states[i];
        void *live_root = NULL;
        void *live_joint[3] = { NULL, NULL, NULL };
        int j;
        body_profile_set_active_person_config(i);
        if (penis_state->initialized) {
            restore_body_chain_output_rest_for_person(i, penis_state);
        }
        restore_poseeditor_joint01_track(penis_state);
        if (penis_state->root_translation_parent_rest_valid) {
            memcpy(runtime_body_chain_person_states[i]
                       .root_translation_parent_rest,
                   penis_state->root_translation_parent_rest,
                   sizeof(penis_state->root_translation_parent_rest));
            runtime_body_chain_person_states[i]
                .root_translation_parent_rest_valid = 1;
            runtime_body_chain_person_states[i]
                .root_translation_parent_rest_root_raw =
                    penis_state->root_translation_parent_rest_root_raw;
            runtime_body_chain_person_states[i]
                .root_translation_parent_rest_trs_raw =
                    penis_state->root_translation_parent_rest_trs_raw;
        }
        if (testicle_state->initialized &&
            resolve_testicle_physics_raws(body_chain_person_name(i),
                                          &live_root, live_joint) &&
            live_root == testicle_state->root_raw) {
            for (j = 0; j < 3; j++) {
                if (live_joint[j] != testicle_state->joint_raw[j] ||
                    !ptr_readable((BYTE*)live_joint[j] +
                                      testicle_physics_cfg.output_offset,
                                  sizeof(float) * 3)) {
                    break;
                }
            }
            if (j == 3) {
                for (j = 0; j < 3; j++) {
                    memcpy((BYTE*)live_joint[j] +
                               testicle_physics_cfg.output_offset,
                           testicle_state->rest[j],
                           sizeof(float) * 3);
                }
            }
        }
        restore_poseeditor_joint01_track(testicle_state);
        if (testicle_state->root_translation_parent_rest_valid) {
            memcpy(runtime_testicle_physics_states[i]
                       .root_translation_parent_rest,
                   testicle_state->root_translation_parent_rest,
                   sizeof(testicle_state->root_translation_parent_rest));
            runtime_testicle_physics_states[i]
                .root_translation_parent_rest_valid = 1;
            runtime_testicle_physics_states[i]
                .root_translation_parent_rest_root_raw =
                    testicle_state->root_translation_parent_rest_root_raw;
            runtime_testicle_physics_states[i]
                .root_translation_parent_rest_trs_raw =
                    testicle_state->root_translation_parent_rest_trs_raw;
        }
        body_profile_set_active_person_config(-1);
    }
    log_line("body-chain-physics runtime entry prepared note=\"PoseEditor tracks were reconnected without resetting its solver state, gravity cache, or axis reference\"");
}

static int body_chain_apply_runtime_ownership_for_person(
    int person_index, int testicle_chain, int validate_live_mapping)
{
    runtime_body_chain_ownership_state_t *ownership =
        body_chain_runtime_ownership_state(person_index, testicle_chain);
    body_chain_person_state_t *state = testicle_chain
        ? &runtime_testicle_physics_states[person_index]
        : &runtime_body_chain_person_states[person_index];
    body_chain_physics_config_t *cfg = testicle_chain
        ? &testicle_physics_cfg : &body_chain_physics_cfg;
    void *live_root = NULL;
    void *live_output_joint[3] = { NULL, NULL, NULL };
    void *live_animation_joint[3] = { NULL, NULL, NULL };
    float *output[3];
    int output_valid;
    int animation_valid;
    int i;
    if (!body_chain_runtime_mode_active() || !ownership ||
        !cfg->enabled || !cfg->enabled_person[person_index] ||
        (!testicle_chain &&
         !body_chain_runtime_penis_person_enabled_now(
             person_index, GetTickCount())) ||
        !state->initialized || !ownership->mapping_ready ||
        !ownership->pose_valid) {
        return 0;
    }
    if (validate_live_mapping) {
        output_valid = testicle_chain
            ? resolve_testicle_physics_raws(
                  body_chain_person_name(person_index),
                  &live_root, live_output_joint)
            : resolve_body_chain_raws(
                  body_chain_person_name(person_index),
                  &live_root, live_output_joint);
        animation_valid = body_chain_resolve_runtime_animation_joints(
            body_chain_person_name(person_index), testicle_chain,
            live_animation_joint);
        if (!output_valid || !animation_valid ||
            live_root != state->root_raw) {
            body_chain_release_runtime_ownership_for_person(
                person_index, testicle_chain,
                "runtime-mapping-changed");
            return 0;
        }
        for (i = 0; i < 3; i++) {
            if (live_output_joint[i] != ownership->source_joint_raw[i] ||
                live_output_joint[i] != state->joint_raw[i] ||
                live_animation_joint[i] !=
                    ownership->animation_joint_raw[i]) {
                body_chain_release_runtime_ownership_for_person(
                    person_index, testicle_chain,
                    "runtime-mapping-changed");
                return 0;
            }
        }
        if (testicle_chain && cfg->override_animation &&
            (!ownership->ownership_active ||
             !ownership->animation_rows_valid) &&
            !body_chain_capture_runtime_animation_rows(ownership)) {
            return 0;
        }
    }
    for (i = 0; i < 3; i++) {
        if (ownership->source_joint_raw[i] != state->joint_raw[i] ||
            !ptr_readable((BYTE*)state->joint_raw[i] +
                              cfg->output_offset,
                          sizeof(float) * 3) ||
            !body_chain_vec3_sane_limit(ownership->rotation[i],
                                        720.0f) ||
            !body_chain_runtime_tjoint_layout_valid(
                ownership->animation_joint_raw[i])) {
            return 0;
        }
        output[i] = (float*)((BYTE*)state->joint_raw[i] +
                             cfg->output_offset);
    }
    for (i = 0; i < 3; i++) {
        memcpy(output[i], ownership->rotation[i], sizeof(float) * 3);
        if (cfg->override_animation &&
            !body_chain_write_runtime_identity(
                ownership->animation_joint_raw[i])) {
            return 0;
        }
    }
    if (!testicle_chain && cfg->override_animation &&
        !body_chain_neutralize_runtime_penis_source_axes(ownership)) {
        return 0;
    }
    ownership->ownership_active = 1;
    if (!testicle_chain && cfg->override_animation) {
        InterlockedExchange(&body_chain_runtime_penis_native_filter_active,
                            1);
    }
    if (!ownership->ownership_logged) {
        ownership->ownership_logged = 1;
        log_line("%s runtime-ownership active person=\"%s\" output_joints=(%p,%p,%p) animation_joints=(%p,%p,%p) note=\"separate runtime state; PoseEditor tracks and axis cache are untouched\"",
                 testicle_chain ? "testicle-physics" :
                                  "body-chain-physics",
                 body_chain_person_name(person_index),
                 state->joint_raw[0], state->joint_raw[1],
                 state->joint_raw[2],
                 ownership->animation_joint_raw[0],
                 ownership->animation_joint_raw[1],
                 ownership->animation_joint_raw[2]);
    }
    return 1;
}

static int body_chain_apply_all_runtime_ownership(
    int validate_live_mapping)
{
    int applied = 0;
    int i;
    if (!body_chain_runtime_mode_active()) return 0;
    for (i = 0; i < 4; i++) {
        body_profile_set_active_person_config(i);
        applied += body_chain_apply_runtime_ownership_for_person(
            i, 0, validate_live_mapping);
        applied += body_chain_apply_runtime_ownership_for_person(
            i, 1, validate_live_mapping);
    }
    body_profile_set_active_person_config(-1);
    return applied;
}

static void publish_body_chain_runtime_ownership(void)
{
    int active = 0;
    int i;
    if (body_chain_runtime_mode_active()) {
        for (i = 0; i < 4 && !active; i++) {
            body_profile_set_active_person_config(i);
            active =
                (body_chain_physics_cfg.enabled &&
                 body_chain_physics_cfg.enabled_person[i] &&
                 runtime_body_chain_ownership_states[i].mapping_ready &&
                 runtime_body_chain_ownership_states[i].pose_valid) ||
                (testicle_physics_cfg.enabled &&
                 testicle_physics_cfg.enabled_person[i] &&
                 runtime_testicle_ownership_states[i].mapping_ready &&
                 runtime_testicle_ownership_states[i].pose_valid);
        }
    }
    body_profile_set_active_person_config(-1);
    InterlockedExchange(&body_chain_runtime_ownership_active,
                        active ? 1 : 0);
    InterlockedExchange(&body_chain_traverse_overlay_active,
                        active ? 1 : 0);
    body_chain_refresh_runtime_penis_native_filter();
}

static int resolve_breasts_physics_raws(
    const char *person, void **root_raw_out, void *source_raw_out[2],
    void *animation_raw_out[2], void *translation_parent_raw_out[2])
{
    static const char *source_names[2] = {
        "Sbreast_scale_L_joint", "Sbreast_scale_R_joint"
    };
    static const char *animation_names[2] = {
        "breast_scale_L_joint", "breast_scale_R_joint"
    };
    static const char *translation_parent_names[2] = {
        "breast_L_joint", "breast_R_joint"
    };
    char name[256];
    int side;
    if (!person || !root_raw_out || !source_raw_out || !animation_raw_out ||
        !translation_parent_raw_out) {
        return 0;
    }
    /* The upper-spine transform inherits whole-character root motion while
       also carrying torso-only animation. Sampling it lets breast PhysX
       react when the chest moves and the pelvis/root remains stationary. */
    make_body_runtime_name(name, sizeof(name), person, "spine_joint04");
    *root_raw_out = resolve_axis_map_raw(name);
    if (!*root_raw_out) return 0;
    for (side = 0; side < 2; side++) {
        make_body_runtime_name(name, sizeof(name), person,
                               source_names[side]);
        source_raw_out[side] = resolve_axis_map_raw(name);
        make_body_runtime_name(name, sizeof(name), person,
                               animation_names[side]);
        animation_raw_out[side] = resolve_axis_map_raw(name);
        make_body_runtime_name(name, sizeof(name), person,
                               translation_parent_names[side]);
        translation_parent_raw_out[side] = resolve_axis_map_raw(name);
        if (!source_raw_out[side] || !animation_raw_out[side] ||
            !translation_parent_raw_out[side] ||
            !body_chain_runtime_tjoint_layout_valid(
                animation_raw_out[side]) ||
            !body_chain_runtime_tjoint_layout_valid(
                translation_parent_raw_out[side])) {
            return 0;
        }
    }
    return 1;
}

static int breasts_physics_values_sane(
    void *root_raw, void *source_raw[2], void *animation_raw[2],
    void *translation_parent_raw[2])
{
    int side;
    if (!root_raw ||
        !ptr_readable((BYTE*)root_raw +
                          breasts_physics_cfg.root_offset,
                      sizeof(float) * 3)) {
        return 0;
    }
    for (side = 0; side < 2; side++) {
        float *rotation;
        float *translation;
        if (!source_raw[side] || !animation_raw[side] ||
            !translation_parent_raw[side] ||
            !ptr_readable((BYTE*)source_raw[side] +
                              breasts_physics_cfg.output_offset,
                          sizeof(float) * 3) ||
            !ptr_readable((BYTE*)source_raw[side] +
                              breasts_physics_bone_translation_offset,
                          sizeof(float) * 3) ||
            !body_chain_runtime_tjoint_layout_valid(animation_raw[side]) ||
            !body_chain_runtime_tjoint_layout_valid(
                translation_parent_raw[side])) {
            return 0;
        }
        rotation = (float*)((BYTE*)source_raw[side] +
                            breasts_physics_cfg.output_offset);
        translation = (float*)((BYTE*)source_raw[side] +
                               breasts_physics_bone_translation_offset);
        if (!body_chain_vec3_sane_limit(rotation, 720.0f) ||
            !body_chain_vec3_sane_limit(translation, 64.0f)) return 0;
    }
    return 1;
}

static int breasts_physics_candidate_is_stable(
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
    if (state->ownership_candidate_samples < 0xffffffffu) {
        state->ownership_candidate_samples++;
    }
    return state->ownership_candidate_samples >= 3u &&
           now - state->ownership_candidate_tick >= 50u;
}

static int breasts_physics_live_ownership_matches(
    int person_index, breasts_physics_person_state_t *state)
{
    void *root_raw = NULL;
    void *source_raw[2] = { NULL, NULL };
    void *animation_raw[2] = { NULL, NULL };
    void *translation_parent_raw[2] = { NULL, NULL };
    LONG node_generation;
    if (person_index < 0 || person_index >= 4 || !state ||
        !state->initialized) {
        return 0;
    }
    node_generation = InterlockedCompareExchange(
        &named_node_generation, 0, 0);
    if (InterlockedCompareExchange(&physx_late_ownership_active, 0, 0) &&
        physx_simulation_serial &&
        state->live_ownership_simulation_serial ==
            physx_simulation_serial &&
        state->live_ownership_node_generation == node_generation) {
        return 1;
    }
    if (!resolve_breasts_physics_raws(
            body_chain_person_name(person_index), &root_raw, source_raw,
            animation_raw, translation_parent_raw)) {
        return 0;
    }
    if (!(root_raw == state->motion.root_raw &&
        source_raw[0] == state->source_joint_raw[0] &&
        source_raw[1] == state->source_joint_raw[1] &&
        animation_raw[0] == state->animation_joint_raw[0] &&
        animation_raw[1] == state->animation_joint_raw[1] &&
        translation_parent_raw[0] ==
            state->translation_parent_joint_raw[0] &&
        translation_parent_raw[1] ==
            state->translation_parent_joint_raw[1] &&
        breasts_physics_values_sane(root_raw, source_raw, animation_raw,
                                    translation_parent_raw))) {
        return 0;
    }
    state->live_ownership_simulation_serial = physx_simulation_serial;
    state->live_ownership_node_generation = node_generation;
    return 1;
}

static void breasts_physics_capture_animation_rows(
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

static int breasts_physics_apply_output(
    breasts_physics_person_state_t *state, int restore_handoff,
    int capture_animation)
{
    int side, axis;
    int wrote_bone_translation = 0;
    if (!state || !state->initialized) return 0;
    (void)capture_animation;
    for (side = 0; side < 2; side++) {
        float *output;
        float *translation_output;
        int write_translation = restore_handoff
            ? state->bone_translation_applied
            : (breasts_physics_bone_translation_enabled ||
               state->contact_translation_active || state->bone_translation_applied);
        if (!state->source_joint_raw[side] ||
            !ptr_readable((BYTE*)state->source_joint_raw[side] +
                              breasts_physics_cfg.output_offset,
                          sizeof(float) * 3) ||
            (write_translation &&
             !ptr_readable((BYTE*)state->source_joint_raw[side] +
                               breasts_physics_bone_translation_offset,
                           sizeof(float) * 3))) {
            return 0;
        }
        output = (float*)((BYTE*)state->source_joint_raw[side] +
                          breasts_physics_cfg.output_offset);
        for (axis = 0; axis < 3; axis++) {
            output[axis] = restore_handoff
                ? state->source_handoff[side][axis]
                : state->rest_rotation[side][axis] +
                      state->rotation[side][axis];
        }
        if (write_translation) {
            translation_output =
                (float*)((BYTE*)state->source_joint_raw[side] +
                         breasts_physics_bone_translation_offset);
            for (axis = 0; axis < 3; axis++) {
                translation_output[axis] =
                    state->source_translation_handoff[side][axis] +
                    ((!restore_handoff &&
                      (breasts_physics_bone_translation_enabled || state->contact_translation_active))
                         ? state->bone_translation[side][axis]
                         : 0.0f);
            }
            wrote_bone_translation = 1;
        }
        body_chain_mark_runtime_transform_dirty(
            state->source_joint_raw[side]);
        if (breasts_physics_cfg.override_animation) {
            if (!state->animation_rows_valid) {
                if (!restore_handoff) return 0;
            } else {
                memcpy((BYTE*)state->animation_joint_raw[side] + 0x078,
                       &state->animation_rows[side][0],
                       sizeof(float) * 3);
                memcpy((BYTE*)state->animation_joint_raw[side] + 0x088,
                       &state->animation_rows[side][3],
                       sizeof(float) * 3);
                memcpy((BYTE*)state->animation_joint_raw[side] + 0x098,
                       &state->animation_rows[side][6],
                       sizeof(float) * 3);
                /* breast_scale_* is TK17's animation-owned joint. Its
                   ProcessAnimation/UpdateTraverse path already advances the
                   matrix version for this frame. The rows above must still
                   be restored to suppress native breast animation, but an
                   extra SetMatrixVersion call races that traversal and emits
                   "MatrixVersion updatet twice" every frame. Only the
                   Sbreast_scale_* PhysX output joint is dirtied explicitly. */
            }
        }
    }
    state->output_applied = restore_handoff ? 0 : 1;
    if (wrote_bone_translation) {
        state->bone_translation_applied =
            (!restore_handoff &&
             (breasts_physics_bone_translation_enabled || state->contact_translation_active)) ? 1 : 0;
    }
    return 1;
}

static const poseeditor_paired_bone_track_def_t
breasts_physics_poseeditor_tracks[] = {
    { POSEEDIT_TRACK_BREAST_LEFT, "breast_L_joint", "Breast Left", 0 },
    { POSEEDIT_TRACK_BREAST_RIGHT, "breast_R_joint", "Breast Right", 0 },
    { POSEEDIT_TRACK_BREAST_L_HIGH,
      "breast_L_pe_up", "Breast L High", 1 },
    { POSEEDIT_TRACK_BREAST_R_HIGH,
      "breast_R_pe_up", "Breast R High", 1 },
    { POSEEDIT_TRACK_BREAST_L_LOW,
      "breast_L_pe_down", "Breast L Low", 1 },
    { POSEEDIT_TRACK_BREAST_R_LOW,
      "breast_R_pe_down", "Breast R Low", 1 }
};

static void reset_breasts_physics_person_state(
    int person_index, breasts_physics_person_state_t *state,
    int restore_output)
{
    unsigned int restored_tracks;
    if (!state) return;
    if (restore_output && state->output_applied &&
        breasts_physics_live_ownership_matches(person_index, state)) {
        breasts_physics_apply_output(state, 1, 0);
    }
    restored_tracks = restore_poseeditor_paired_bone_tracks(
        &state->pose_tracks,
        (int)(sizeof(breasts_physics_poseeditor_tracks) /
              sizeof(breasts_physics_poseeditor_tracks[0])));
    if (restored_tracks) {
        schedule_poseeditor_track_handoff_refresh(
            person_index, restored_tracks, GetTickCount());
    }
    memset(state, 0, sizeof(*state));
}

static void breasts_physics_build_side_drive(
    const body_chain_physics_config_t *cfg, int side,
    const float translation_step[3], const float rotation_step[3],
    float out[3])
{
    int channel;
    out[0] = 0.0f;
    out[1] = 0.0f;
    out[2] = 0.0f;
    if (!cfg || side < 0 || side > 1) return;
    for (channel = 0; channel < 3; channel++) {
        int tail_axis = cfg->translation_tail_axis[channel];
        if (tail_axis >= 0 && tail_axis <= 2) {
            out[tail_axis] +=
                translation_step[channel] *
                cfg->translation_scale[channel] *
                breasts_physics_translation_sign[side][channel] *
                BODY_CHAIN_TRANSLATION_RESPONSE;
        }
    }
    for (channel = 0; channel < 3; channel++) {
        int source_axis = cfg->rotation_source_axis[channel];
        int tail_axis = cfg->rotation_tail_axis[channel];
        if (rotation_step && source_axis >= 0 && source_axis <= 2 &&
            tail_axis >= 0 && tail_axis <= 2) {
            float value = rotation_step[source_axis];
            if (physx_absf(value) >= cfg->rotation_deadzone) {
                out[tail_axis] +=
                    value * cfg->rotation_scale[channel] *
                    breasts_physics_rotation_sign[side][channel];
            }
        }
    }
}

static int breasts_physics_body_translation_to_parent_local(
    const breasts_physics_person_state_t *state, int side,
    const float body_target[3], float out[3])
{
    float parent_view[9];
    float trs_view[9];
    float trs_inverse[9];
    float parent_relative[9];
    float parent_inverse[9];
    float body_relative[9];
    float target_relative[3];
    if (!state || side < 0 || side > 1 || !body_target || !out ||
        !state->motion.camera_relative_live_current_valid ||
        !state->motion.camera_relative_trs_raw ||
        !state->translation_parent_joint_raw[side] ||
        !body_chain_read_mat3_rows(
            state->translation_parent_joint_raw[side], parent_view) ||
        !body_chain_read_mat3_rows(
            state->motion.camera_relative_trs_raw, trs_view) ||
        !body_chain_mat3_inverse(trs_view, trs_inverse)) {
        return 0;
    }
    memcpy(body_relative, state->motion.camera_relative_live_current,
           sizeof(body_relative));
    if (!body_chain_normalize_basis_rows(body_relative)) return 0;
    body_chain_mat3_multiply(parent_view, trs_inverse, parent_relative);
    if (!body_chain_normalize_basis_rows(parent_relative) ||
        !body_chain_mat3_inverse(parent_relative, parent_inverse)) {
        return 0;
    }
    /* body_target is expressed along the character's authored body axes.
       Move it into camera-neutral TRS space, then into the live parent
       joint's local axes. Only the additive offset is converted; the
       captured SJoint translation remains untouched. */
    body_chain_transform_row_vector3(body_target, body_relative,
                                     target_relative);
    body_chain_transform_row_vector3(target_relative, parent_inverse, out);
    return body_chain_vec3_sane_limit(out, 0.25f);
}

static void breasts_physics_build_bone_translation_target(
    const body_chain_physics_config_t *cfg,
    const breasts_physics_person_state_t *state, int side,
    const float local_step[3], float out[3])
{
    float mapped_target[3] = { 0.0f, 0.0f, 0.0f };
    int channel;
    if (!cfg || side < 0 || side > 1 || !local_step || !out) return;
    out[0] = out[1] = out[2] = 0.0f;
    for (channel = 0; channel < 3; channel++) {
        int source_axis =
            breasts_physics_bone_translation_source_axis[channel];
        int tail_axis =
            breasts_physics_bone_translation_tail_axis[channel];
        float drive = local_step[source_axis];
        if (physx_absf(drive) <= cfg->translation_deadzone) drive = 0.0f;
        mapped_target[tail_axis] += drive *
            breasts_physics_bone_translation_scale[channel] *
            breasts_physics_bone_translation_sign[side][channel];
    }
    if (breasts_physics_bone_translation_space &&
        breasts_physics_body_translation_to_parent_local(
            state, side, mapped_target, out)) {
        return;
    }
    memcpy(out, mapped_target, sizeof(mapped_target));
}

static int breasts_physics_build_gravity_sag_target(
    const breasts_physics_person_state_t *state, int side, float out[3])
{
    float gravity_len;
    float body_relative[9];
    float body_inverse[9];
    float fixed_down[3];
    float body_target[3];
    float parent_local[3];
    float sag_limit;
    float sag_distance;
    int axis;
    if (!state || side < 0 || side > 1 || !out ||
        breasts_physics_bone_translation_gravity_sag <= 0.000001f ||
        !physics_environment_cfg.world_gravity_probe ||
        !physics_environment_cfg.gravity_apply_to_body_chain ||
        !state->motion.camera_relative_live_current_valid ||
        !state->motion.root_translation_parent_rest_valid ||
        state->motion.root_translation_parent_rest_root_raw !=
            state->motion.root_raw ||
        state->motion.root_translation_parent_rest_trs_raw !=
            state->motion.camera_relative_trs_raw) {
        return 0;
    }
    gravity_len = physx_vec3_len(physics_environment_cfg.world_gravity);
    if (gravity_len <= 0.000001f) return 0;
    for (axis = 0; axis < 3; axis++) {
        fixed_down[axis] =
            physics_environment_cfg.world_gravity[axis] / gravity_len;
    }
    memcpy(body_relative, state->motion.camera_relative_live_current,
           sizeof(body_relative));
    if (!body_chain_normalize_basis_rows(body_relative) ||
        !body_chain_mat3_inverse(body_relative, body_inverse)) {
        return 0;
    }
    /* The room observer captures the character's authored standing axes
       before any physics system takes ownership. Define world-down from
       that trusted frame before sag is allowed, then express configured
       world-down in the current live body frame using the actual matrix
       inverse. This prevents a transient or non-orthogonal root/TRS matrix
       at room load from making an upright character sag as if lying
       sideways, while still following world-down after real rotations. */
    body_chain_transform_row_vector3(fixed_down, body_inverse, body_target);
    sag_limit = breasts_physics_bone_translation_max_offset[0];
    for (axis = 1; axis < 3; axis++) {
        if (breasts_physics_bone_translation_max_offset[axis] > sag_limit) {
            sag_limit = breasts_physics_bone_translation_max_offset[axis];
        }
    }
    if (sag_limit <= 0.000001f) return 0;
    /* Apply the configured percentage before converting into the bone
       parent's local space. The shared conversion helper deliberately
       rejects displacements above 0.25 scene units; passing it a normalized
       (one-unit) direction caused every sag target to be discarded. */
    sag_distance =
        sag_limit * breasts_physics_bone_translation_gravity_sag;
    for (axis = 0; axis < 3; axis++) {
        body_target[axis] *= sag_distance;
    }
    if (!breasts_physics_body_translation_to_parent_local(
            state, side, body_target, parent_local)) {
        return 0;
    }
    /* bone_translation_gravity_sag is a fraction of each configured movement
       limit, not a raw scene-unit displacement. Thus 0.05 means five percent
       and cannot instantly pin a 0.03-unit translation limit. */
    for (axis = 0; axis < 3; axis++) {
        float limit = breasts_physics_bone_translation_max_offset[axis];
        out[axis] = physx_clampf(parent_local[axis], -limit, limit);
    }
    return body_chain_vec3_sane_limit(out, 0.25f);
}

/* Inward/outward breast spacing follows spine_joint04 directly. Unlike the
   character root, this joint can legitimately have a zero translation at
   root_offset, so it must not pass through the position-based startup probe. */
static int breasts_physics_spine_gravity_drive(
    void *spine_raw, const body_chain_physics_config_t *cfg,
    gravity_sample_t *sample, DWORD now, float out[3])
{
    float gravity_view[3];
    float gravity_world[3];
    float basis[3];
    float length;
    int offsets[3];
    float signs[3];
    int channel;
    int valid = 0;

    out[0] = out[1] = out[2] = 0.0f;
    if (!spine_raw || !cfg) return 0;
    length = physx_vec3_len(physics_environment_cfg.world_gravity);
    if (!isfinite(length) || length <= 0.000001f) goto sample_gravity;
    for (channel = 0; channel < 3; channel++) {
        gravity_world[channel] =
            physics_environment_cfg.world_gravity[channel] / length;
    }
    if (!camera_world_to_view_direction(gravity_world, gravity_view)) {
        goto sample_gravity;
    }
    offsets[0] = physics_environment_cfg.gravity_horizontal_basis_offset;
    offsets[1] = physics_environment_cfg.gravity_vertical_basis_offset;
    offsets[2] =
        physics_environment_cfg.gravity_horizontal_secondary_basis_offset;
    signs[0] = physics_environment_cfg.gravity_horizontal_basis_sign;
    signs[1] = physics_environment_cfg.gravity_vertical_basis_sign;
    signs[2] =
        physics_environment_cfg.gravity_horizontal_secondary_basis_sign;
    for (channel = 0; channel < 3; channel++) {
        if (!read_normalized_basis_vector(spine_raw, offsets[channel], basis)) {
            out[0] = out[1] = out[2] = 0.0f;
            goto sample_gravity;
        }
        out[channel] = vec3_dot(gravity_view, basis) * signs[channel];
    }
    valid = 1;
sample_gravity:
    {
        float trusted[3];
        if (!gravity_sample_live(sample,spine_raw,out,valid,now,trusted)) {
            out[0] = out[1] = out[2] = 0.0f;
            return 0;
        }
        memcpy(out,trusted,sizeof(trusted));
    }
    body_chain_apply_gravity_curve(
        out, cfg->gravity_horizontal_curve,
        cfg->gravity_vertical_curve, out);
    return body_chain_vec3_sane_limit(out, 4.0f);
}

static void breasts_physics_add_gravity_target(
    const body_chain_physics_config_t *cfg, int side,
    const float gravity_drive[3], const float spacing_gravity_drive[3],
    float out[3])
{
    int channel;
    float inversion_blend;
    float planar_scale;
    if (!cfg || !gravity_drive || !spacing_gravity_drive || !out ||
        side < 0 || side > 1) return;

    /* Fade the planar fall direction as the body becomes fully inverted,
       then replace it with a stable configured upside-down response. */
    inversion_blend = paired_bone_inverted_gravity_amount(gravity_drive);
    planar_scale = 1.0f - inversion_blend;

    /* TK17's legacy gravity channel labels do not match the visible body
       orientations: channel 0 is prone/supine pitch, while channel 1 is
       left/right roll. The independent signs make roll move both mirrored
       breasts toward the same world-space side. */
    for (channel = 0; channel < 2; channel++) {
        int tail_axis = breasts_physics_gravity_tail_axis[channel];
        if (tail_axis >= 0 && tail_axis <= 2) {
            out[tail_axis] +=
                gravity_drive[channel] * cfg->gravity_angle *
                breasts_physics_gravity_sign[side][channel] * planar_scale;
        }
    }

    if (inversion_blend > 0.0f) {
        int tail_axis = breasts_physics_gravity_tail_axis[2];
        if (tail_axis >= 0 && tail_axis <= 2) {
            out[tail_axis] +=
                inversion_blend * cfg->gravity_inverted_strength *
                cfg->gravity_inverted_sign *
                breasts_physics_gravity_sign[side][2];
        }
    }

    /* Preserve the main prone/supine gravity rotation above, then add a much
       smaller mirrored-local convergence layer: inward while face-down and
       outward while face-up. */
    if (breasts_physics_gravity_inward_outward_tail_axis >= 0 &&
        breasts_physics_gravity_inward_outward_tail_axis <= 2) {
        float spacing_planar_scale = 1.0f -
            paired_bone_inverted_gravity_amount(spacing_gravity_drive);
        /* spine_joint04's local basis maps chest-facing gravity to channel 1;
           channel 0 is the corresponding root-bone mapping only. */
        float spacing_drive =
            spacing_gravity_drive[1] *
            breasts_physics_gravity_inward_outward_sign;
        float spacing_strength = spacing_drive >= 0.0f ?
            breasts_physics_gravity_inward_strength :
            breasts_physics_gravity_outward_strength;
        out[breasts_physics_gravity_inward_outward_tail_axis] +=
            spacing_drive * spacing_strength * spacing_planar_scale;
    }
}

static void run_breasts_physics_for_person(int person_index, DWORD now)
{
    const char *person = body_chain_person_name(person_index);
    breasts_physics_person_state_t *state =
        &breasts_physics_states[person_index];
    body_chain_physics_config_t *cfg = &breasts_physics_cfg;
    void *root_raw = NULL;
    void *gravity_root_raw = NULL;
    void *source_raw[2] = { NULL, NULL };
    void *animation_raw[2] = { NULL, NULL };
    void *translation_parent_raw[2] = { NULL, NULL };
    float *root;
    float *gravity_root;
    float root_pivot[3];
    float local_step[3] = { 0.0f, 0.0f, 0.0f };
    float translation_step[3] = { 0.0f, 0.0f, 0.0f };
    float rotation_delta[9] = { 0.0f };
    float parent_rotation_step[3] = { 0.0f, 0.0f, 0.0f };
    float gravity_drive[3] = { 0.0f, 0.0f, 0.0f };
    float spacing_gravity_drive[3] = { 0.0f, 0.0f, 0.0f };
    float wind_channels[2][3] = {
        { 0.0f, 0.0f, 0.0f },
        { 0.0f, 0.0f, 0.0f }
    };
    float target[2][3] = {
        { 0.0f, 0.0f, 0.0f },
        { 0.0f, 0.0f, 0.0f }
    };
    float bone_translation_target[2][3] = {
        { 0.0f, 0.0f, 0.0f },
        { 0.0f, 0.0f, 0.0f }
    };
    float collision_local_offset[2][3] = {
        { 0.0f, 0.0f, 0.0f },
        { 0.0f, 0.0f, 0.0f }
    };
    float gravity_sag_local_offset[2][3] = {
        { 0.0f, 0.0f, 0.0f },
        { 0.0f, 0.0f, 0.0f }
    };
    float dt;
    DWORD elapsed_ms;
    int wind_active[2] = { 0, 0 };
    int side, axis, channel;

    if (!state->initialized && state->resolve_retry_tick &&
        now - state->resolve_retry_tick <
            (DWORD)BREASTS_PHYSICS_MISSING_RETRY_MS) return;
    if (state->last_tick &&
        now - state->last_tick < (DWORD)cfg->interval_ms) return;
    elapsed_ms = state->last_tick ? now - state->last_tick : 0;
    state->last_tick = now;
    dt = elapsed_ms ? (float)elapsed_ms / 1000.0f : 0.016f;
    if (dt <= 0.0f) dt = 0.016f;
    if (dt > 0.025f) dt = 0.025f;

    if (state->initialized && state->cache_verify_tick &&
        now - state->cache_verify_tick <
            (DWORD)BREASTS_PHYSICS_CACHE_VERIFY_MS) {
        root_raw = state->motion.root_raw;
        gravity_root_raw = state->gravity_motion.root_raw;
        source_raw[0] = state->source_joint_raw[0];
        source_raw[1] = state->source_joint_raw[1];
        animation_raw[0] = state->animation_joint_raw[0];
        animation_raw[1] = state->animation_joint_raw[1];
        translation_parent_raw[0] =
            state->translation_parent_joint_raw[0];
        translation_parent_raw[1] =
            state->translation_parent_joint_raw[1];
    } else {
        char gravity_root_name[256];
        if (!resolve_breasts_physics_raws(person, &root_raw, source_raw,
                                          animation_raw,
                                          translation_parent_raw) ||
            !breasts_physics_values_sane(root_raw, source_raw,
                                         animation_raw,
                                         translation_parent_raw)) {
            reset_breasts_physics_person_state(person_index, state, 0);
            state->resolve_retry_tick = now;
            restore_tk17_breasts_inertia_for_person(person_index, now);
            return;
        }
        make_body_runtime_name(gravity_root_name,
                               sizeof(gravity_root_name), person, "root");
        gravity_root_raw = resolve_axis_map_raw(gravity_root_name);
        if (!gravity_root_raw ||
            !ptr_readable((BYTE*)gravity_root_raw + cfg->root_offset,
                          sizeof(float) * 3)) {
            reset_breasts_physics_person_state(person_index, state, 0);
            state->resolve_retry_tick = now;
            restore_tk17_breasts_inertia_for_person(person_index, now);
            return;
        }
        if (state->initialized &&
            (root_raw != state->motion.root_raw ||
             gravity_root_raw != state->gravity_motion.root_raw ||
             source_raw[0] != state->source_joint_raw[0] ||
             source_raw[1] != state->source_joint_raw[1] ||
             animation_raw[0] != state->animation_joint_raw[0] ||
             animation_raw[1] != state->animation_joint_raw[1] ||
             translation_parent_raw[0] !=
                 state->translation_parent_joint_raw[0] ||
             translation_parent_raw[1] !=
                 state->translation_parent_joint_raw[1])) {
            reset_breasts_physics_person_state(person_index, state, 0);
            restore_tk17_breasts_inertia_for_person(person_index, now);
            return;
        }
        state->cache_verify_tick = now;
    }
    state->resolve_retry_tick = 0;
    root = (float*)((BYTE*)root_raw + cfg->root_offset);
    gravity_root = (float*)((BYTE*)gravity_root_raw + cfg->root_offset);
    if (!breasts_physics_candidate_is_stable(
            state, root_raw, source_raw, animation_raw,
            translation_parent_raw, now)) return;
    state->live_ownership_simulation_serial = physx_simulation_serial;
    state->live_ownership_node_generation = InterlockedCompareExchange(
        &named_node_generation, 0, 0);

    if (!state->initialized) {
        /* Capture the complete pose while TK17 still owns both breast
           transforms. PhysX then simulates offsets around this handoff
           instead of snapping the joints to a neutral transform. */
        for (side = 0; side < 2; side++) {
            float *output = (float*)((BYTE*)source_raw[side] +
                                     cfg->output_offset);
            float *translation_output =
                (float*)((BYTE*)source_raw[side] +
                         breasts_physics_bone_translation_offset);
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
        breasts_physics_capture_animation_rows(state);
    }

    if (!body_chain_runtime_mode_active() && cfg->override_animation &&
        cfg->poseeditor_track_override &&
        !suppress_poseeditor_paired_bone_tracks(
            person_index, person, "breasts-physics", &state->pose_tracks,
            breasts_physics_poseeditor_tracks,
            (int)(sizeof(breasts_physics_poseeditor_tracks) /
                  sizeof(breasts_physics_poseeditor_tracks[0])))) {
        return;
    }

    if (!suppress_tk17_breasts_inertia_for_person(person_index, now)) return;

    if (!state->initialized) {
        memset(&state->motion, 0, sizeof(state->motion));
        state->motion.initialized = 1;
        state->motion.root_raw = root_raw;
        state->motion.init_tick = now;
        memset(&state->gravity_motion, 0, sizeof(state->gravity_motion));
        memset(&state->spine_gravity_sample, 0, sizeof(state->spine_gravity_sample));
        state->gravity_motion.initialized = 1;
        state->gravity_motion.root_raw = gravity_root_raw;
        state->gravity_motion.init_tick = now;
        state->initialized = 1;
        state->active_logged = 0;
        body_chain_camera_relative_orientation_step(
            person, &state->motion, root_raw, rotation_delta, NULL, NULL);
        body_chain_camera_relative_orientation_step(
            person, &state->gravity_motion, gravity_root_raw,
            rotation_delta, NULL, NULL);
        breasts_physics_apply_output(state, 0, 0);
        return;
    }

    root_pivot[0] = root[0];
    root_pivot[1] = root[1];
    root_pivot[2] = root[2];
    body_collider_engine_pivot_view(
        person, "spine_joint04", NULL, root_pivot);
    body_chain_camera_relative_orientation_step(
        person, &state->motion, root_raw, rotation_delta,
        parent_rotation_step, NULL);
    body_chain_camera_relative_orientation_step(
        person, &state->gravity_motion, gravity_root_raw,
        rotation_delta, NULL, NULL);
    body_chain_camera_neutral_pivot_step(
        person, person_index, &state->motion, root_pivot,
        cfg->root_offset,
        breasts_physics_active_axis_reference_cache(person_index),
        local_step);
    for (channel = 0; channel < 3; channel++) {
        translation_step[channel] =
            local_step[cfg->translation_source_axis[channel]];
    }
    if ((physx_absf(cfg->gravity_angle) > 0.000001f ||
         cfg->gravity_inverted_strength > 0.000001f) &&
        physics_environment_cfg.world_gravity_probe &&
        physics_environment_cfg.gravity_apply_to_body_chain) {
        run_body_chain_gravity_probe(
            person, &state->gravity_motion, gravity_root_raw, gravity_root, now,
            &breasts_physics_room_gravity_cache[person_index],
            "breasts_physics");
        if (state->gravity_motion.gravity_probe_promoted &&
            state->gravity_motion.gravity_sample.trusted_valid) {
            const float *active_gravity;
            float relative_raw[3];
            update_body_chain_gravity_filter(&state->gravity_motion, dt);
            active_gravity =
                state->gravity_motion.gravity_drive_filtered_valid ?
                    state->gravity_motion.gravity_drive_filtered :
                    state->gravity_motion.gravity_drive;
            if (!state->gravity_reference_valid) {
                for (channel = 0; channel < 3; channel++) {
                    state->gravity_reference[channel] =
                        active_gravity[channel];
                    state->gravity_relative[channel] = 0.0f;
                }
                state->gravity_reference_valid = 1;
                log_line("breasts-physics gravity-reference captured person=\"%s\" reference=(%.5f,%.5f,%.5f) strength=%.2f note=\"captured standing pose remains neutral; gravity is applied only as orientation changes\"",
                         person,
                         state->gravity_reference[0],
                         state->gravity_reference[1],
                         state->gravity_reference[2],
                         cfg->gravity_angle);
            } else {
                body_gravity_direction_drive(
                    active_gravity, state->gravity_reference,
                    state->gravity_relative, relative_raw);
                for (channel = 0; channel < 3; channel++) {
                    state->gravity_relative[channel] =
                        relative_raw[channel];
                }
                body_chain_apply_gravity_curve(
                    relative_raw,
                    cfg->gravity_horizontal_curve,
                    cfg->gravity_vertical_curve,
                    gravity_drive);
            }
        }
    }
    if ((breasts_physics_gravity_inward_strength > 0.000001f ||
         breasts_physics_gravity_outward_strength > 0.000001f) &&
        physics_environment_cfg.world_gravity_probe &&
        physics_environment_cfg.gravity_apply_to_body_chain) {
        breasts_physics_spine_gravity_drive(
            root_raw, cfg, &state->spine_gravity_sample, now, spacing_gravity_drive);
    }
    /* Orientation gravity rotates around the captured rest pose. Optional
       translation sag is added separately to the bone-translation spring. */
    for (side = 0; side < 2; side++) {
        wind_active[side] = body_chain_room_wind_channels(
            cfg, &state->motion,
            state->translation_parent_joint_raw[side],
            person, "breasts_physics", now, wind_channels[side]);
        breasts_physics_build_side_drive(
            cfg, side, translation_step, parent_rotation_step,
            target[side]);
        breasts_physics_add_gravity_target(
            cfg, side, gravity_drive, spacing_gravity_drive,
            target[side]);
        if (wind_active[side]) {
            body_chain_add_room_wind_target(
                cfg,
                wind_channels[side],
                breasts_physics_wind_sign[side],
                target[side]);
        }
        for (axis = 0; axis < 3; axis++) {
            target[side][axis] = body_chain_clamp_link_axis_angle(
                cfg, 0, axis,
                target[side][axis] * cfg->link_gain[0]);
        }
        for (axis = 0; axis < 3; axis++) {
            float acceleration =
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
        if (breasts_physics_bone_translation_enabled) {
            breasts_physics_build_bone_translation_target(
                cfg, state, side, local_step, bone_translation_target[side]);
            if (breasts_physics_build_gravity_sag_target(state, side,
                    gravity_sag_local_offset[side])) {
                for (axis=0;axis<3;axis++)
                    bone_translation_target[side][axis]+=gravity_sag_local_offset[side][axis];
            }
        }
    }
    state->contact_translation_active = breasts_physics_bone_translation_enabled ||
        (body_chain_collider_cfg.enabled && body_chain_collider_cfg.breasts_collision_enabled) ||
        (cfg->room_collision_enabled && room_collision_is_enabled()) || state->bone_translation_applied;
    if (state->contact_translation_active) {
        single_bone_contact_step(person_index, 0, state, cfg,
            now, elapsed_ms, bone_translation_target, breasts_physics_bone_translation_stiffness,
            breasts_physics_bone_translation_damping, breasts_physics_bone_translation_max_offset,
            collision_local_offset);
    } else {
        memset(state->bone_translation,0,sizeof(state->bone_translation));
        memset(state->bone_translation_velocity,0,sizeof(state->bone_translation_velocity));
    }
    breasts_physics_apply_output(state, 0, 0);
    if (!state->active_logged) {
        state->active_logged = 1;
        log_line("breasts-physics active person=\"%s\" motion_source=\"spine_joint04\" gravity_source=\"root\" targets=\"breast_scale_L_joint,breast_scale_R_joint\" translation_axes=(%d->%d,%d->%d,%d->%d) rotation_axes=(%d->%d,%d->%d,%d->%d) gravity=(strength=%.2f,axes=%d,%d,%d,inverted_strength=%.2f,inverted_sign=%.2f,inward_outward_axis=%d,inward_strength=%.2f,outward_strength=%.2f) spring=(%.2f,%.2f) collision=(enabled=%d,scope=%s) bone_translation=(enabled=%d,space=%s,offset=0x%03x,axes=%d->%d/%d->%d/%d->%d,scale=%.3f/%.3f/%.3f,gravity_sag=%.4f,stiffness=%.2f,damping=%.2f,max=%.4f/%.4f/%.4f) note=\"shared settings; upper-spine motion includes inherited whole-body movement and torso-only animation; canonical root gravity preserves established orientation channels; two independent single-bone rotation and translation springs; world-gravity sag and body-space movement are converted into each breast parent's local basis\"",
                 person,
                 cfg->translation_source_axis[0],
                 cfg->translation_tail_axis[0],
                 cfg->translation_source_axis[1],
                 cfg->translation_tail_axis[1],
                 cfg->translation_source_axis[2],
                 cfg->translation_tail_axis[2],
                 cfg->rotation_source_axis[0], cfg->rotation_tail_axis[0],
                 cfg->rotation_source_axis[1], cfg->rotation_tail_axis[1],
                 cfg->rotation_source_axis[2], cfg->rotation_tail_axis[2],
                 cfg->gravity_angle,
                 breasts_physics_gravity_tail_axis[0],
                 breasts_physics_gravity_tail_axis[1],
                 breasts_physics_gravity_tail_axis[2],
                 cfg->gravity_inverted_strength,
                 cfg->gravity_inverted_sign,
                 breasts_physics_gravity_inward_outward_tail_axis,
                 breasts_physics_gravity_inward_strength,
                 breasts_physics_gravity_outward_strength,
                 cfg->stiffness, cfg->damping,
                 body_chain_collider_cfg.breasts_collision_enabled,
                 body_chain_collision_scope_name(cfg->collision_scope),
                 breasts_physics_bone_translation_enabled,
                 breasts_physics_bone_translation_space
                     ? "body" : "local",
                 breasts_physics_bone_translation_offset,
                 breasts_physics_bone_translation_source_axis[0],
                 breasts_physics_bone_translation_tail_axis[0],
                 breasts_physics_bone_translation_source_axis[1],
                 breasts_physics_bone_translation_tail_axis[1],
                 breasts_physics_bone_translation_source_axis[2],
                 breasts_physics_bone_translation_tail_axis[2],
                 breasts_physics_bone_translation_scale[0],
                 breasts_physics_bone_translation_scale[1],
                 breasts_physics_bone_translation_scale[2],
                 breasts_physics_bone_translation_gravity_sag,
                 breasts_physics_bone_translation_stiffness,
                 breasts_physics_bone_translation_damping,
                 breasts_physics_bone_translation_max_offset[0],
                 breasts_physics_bone_translation_max_offset[1],
                 breasts_physics_bone_translation_max_offset[2]);
    }
    if (defaults_cfg.debug &&
        (!state->log_tick || now - state->log_tick >= 1000u)) {
        state->log_tick = now;
        log_line("breasts-physics write person=\"%s\" local_step=(%.5f,%.5f,%.5f) rotation_step=(%.3f,%.3f,%.3f) gravity=(%.4f,%.4f,%.4f) spacing_gravity=(%.4f,%.4f,%.4f) inverted_blend=%.4f left_target=(%.3f,%.3f,%.3f) right_target=(%.3f,%.3f,%.3f) left=(%.3f,%.3f,%.3f) right=(%.3f,%.3f,%.3f) gravity_sag_local=(%.5f,%.5f,%.5f;%.5f,%.5f,%.5f) bone_translation_target=(%.5f,%.5f,%.5f;%.5f,%.5f,%.5f) bone_translation=(%.5f,%.5f,%.5f;%.5f,%.5f,%.5f)",
                 person, local_step[0], local_step[1], local_step[2],
                 parent_rotation_step[0], parent_rotation_step[1],
                 parent_rotation_step[2], gravity_drive[0],
                 gravity_drive[1], gravity_drive[2],
                 spacing_gravity_drive[0], spacing_gravity_drive[1],
                 spacing_gravity_drive[2],
                 paired_bone_inverted_gravity_amount(gravity_drive),
                 target[0][0], target[0][1],
                 target[0][2], target[1][0], target[1][1], target[1][2],
                 state->rotation[0][0], state->rotation[0][1],
                 state->rotation[0][2], state->rotation[1][0],
                 state->rotation[1][1], state->rotation[1][2],
                 gravity_sag_local_offset[0][0],
                 gravity_sag_local_offset[0][1],
                 gravity_sag_local_offset[0][2],
                 gravity_sag_local_offset[1][0],
                 gravity_sag_local_offset[1][1],
                 gravity_sag_local_offset[1][2],
                 bone_translation_target[0][0],
                 bone_translation_target[0][1],
                 bone_translation_target[0][2],
                 bone_translation_target[1][0],
                 bone_translation_target[1][1],
                 bone_translation_target[1][2],
                 state->bone_translation[0][0],
                 state->bone_translation[0][1],
                 state->bone_translation[0][2],
                 state->bone_translation[1][0],
                 state->bone_translation[1][1],
                 state->bone_translation[1][2]);
    }
}

static void run_breasts_physics(DWORD now)
{
    int i;
    if (!breasts_physics_global_cfg.enabled || !engine_FindObjC) {
        for (i = 0; i < 4; i++) {
            body_profile_set_active_person_config(i);
            reset_breasts_physics_person_state(
                i, &breasts_physics_states[i], 1);
            restore_tk17_breasts_inertia_for_person(i, now);
        }
        body_profile_set_active_person_config(-1);
        return;
    }
    for (i = 0; i < 4; i++) {
        body_profile_set_active_person_config(i);
        if (breasts_physics_person_enabled(i)) {
            LONGLONG perf_start = physx_perf_counter();
            run_breasts_physics_for_person(i, now);
            physx_perf_add(PHYSX_PERF_BREASTS_ACTIVE, perf_start);
        } else {
            reset_breasts_physics_person_state(
                i, &breasts_physics_states[i], 1);
            restore_tk17_breasts_inertia_for_person(i, now);
        }
    }
    body_profile_set_active_person_config(-1);
}

static int breasts_physics_apply_all_outputs(int capture_animation)
{
    int applied = 0;
    int i;
    if (!breasts_physics_global_cfg.enabled) return 0;
    for (i = 0; i < 4; i++) {
        breasts_physics_person_state_t *state =
            &breasts_physics_states[i];
        body_profile_set_active_person_config(i);
        if (breasts_physics_person_enabled(i) &&
            state->initialized &&
            breasts_physics_live_ownership_matches(i, state)) {
            applied += breasts_physics_apply_output(
                state, 0, capture_animation);
        }
    }
    body_profile_set_active_person_config(-1);
    return applied;
}

static void run_breasts_physics_late_ownership(DWORD now)
{
    int i;
    if (!breasts_physics_global_cfg.enabled) {
        restore_tk17_breasts_inertia_all(now);
        return;
    }
    for (i = 0; i < 4; i++) {
        breasts_physics_person_state_t *state =
            &breasts_physics_states[i];
        body_profile_set_active_person_config(i);
        if (breasts_physics_person_enabled(i) &&
            state->initialized &&
            breasts_physics_live_ownership_matches(i, state) &&
            suppress_tk17_breasts_inertia_for_person(i, now)) {
            breasts_physics_apply_output(state, 0, 0);
        } else if (!breasts_physics_person_enabled(i)) {
            restore_tk17_breasts_inertia_for_person(i, now);
        }
    }
    body_profile_set_active_person_config(-1);
}

static int physx_body_chain_apply_traverse_overlay(void *object,
                                                   int allow_global)
{
    int applied;
    (void)object;
    if (!body_chain_runtime_mode_active()) {
        InterlockedExchange(&body_chain_traverse_overlay_active, 0);
        return 0;
    }
    if (!allow_global ||
        InterlockedExchange(&body_chain_traverse_overlay_active, 0) == 0) {
        return 0;
    }
    applied = body_chain_apply_all_runtime_ownership(0);
    return applied;
}

static void physx_body_chain_apply_post_animation_ownership(void)
{
    int applied;
    breasts_physics_apply_all_outputs(1);
    butt_physics_apply_all_outputs(1);
    if (!body_chain_runtime_mode_active() ||
        !InterlockedCompareExchange(&body_chain_runtime_ownership_active,
                                    0, 0)) {
        return;
    }
    applied = body_chain_apply_all_runtime_ownership(1);
    if (applied > 0) {
        InterlockedExchange(&body_chain_traverse_overlay_active, 1);
    } else {
        InterlockedExchange(&body_chain_runtime_ownership_active, 0);
        InterlockedExchange(&body_chain_traverse_overlay_active, 0);
    }
}

static void run_testicle_physics(DWORD now)
{
    int i;
    if (!testicle_physics_cfg.enabled) {
        for (i = 0; i < 4; i++) {
            body_chain_person_state_t *state =
                body_chain_active_person_state(i, 1);
            if (body_chain_runtime_mode_active()) {
                body_chain_release_runtime_ownership_for_person(
                    i, 1, "testicle-physics-disabled");
            }
            restore_tk17_testicle_inertia_for_person(i, now);
            if (state && state->initialized) {
                reset_body_chain_person_state_for_reactivation(
                    state);
            }
        }
        return;
    }
    if (!engine_FindObjC) return;
    for (i = 0; i < 4; i++) {
        int active;
        body_profile_set_active_person_config(i);
        active = testicle_physics_cfg.enabled &&
                 testicle_physics_cfg.enabled_person[i];
        if (body_chain_runtime_mode_active() &&
            testicle_physics_settings_change_tick[i] &&
            !testicle_physics_settings_change_enabled[i] &&
            now - testicle_physics_settings_change_tick[i] <= 3000u) {
            active = 0;
        }
        if (active) {
            LONGLONG perf_start = physx_perf_counter();
            run_testicle_physics_for_person(i, now);
            physx_perf_add(PHYSX_PERF_TESTICLE_ACTIVE, perf_start);
        } else {
            body_chain_person_state_t *state =
                body_chain_active_person_state(i, 1);
            if (body_chain_runtime_mode_active()) {
                body_chain_release_runtime_ownership_for_person(
                    i, 1, "testicle-physics-disabled");
            }
            restore_tk17_testicle_inertia_for_person(i, now);
            if (state && state->initialized) {
                reset_body_chain_person_state_for_reactivation(
                    state);
            }
        }
        body_profile_set_active_person_config(-1);
    }
}

static void testicle_physics_late_ownership_for_person(int person_index,
                                                       DWORD now)
{
    const char *person = body_chain_person_name(person_index);
    int runtime_mode = body_chain_runtime_mode_active();
    body_chain_person_state_t *state =
        body_chain_active_person_state(person_index, 1);
    body_chain_physics_config_t *cfg = &testicle_physics_cfg;
    float *out[3];
    float desired[3][3];
    float before[3][3];
    float max_delta = 0.0f;
    int max_joint = -1;
    int max_axis = -1;
    int i, axis;
    if (person_index < 0 || person_index >= 4 || !state) return;
    if (!cfg->enabled || !cfg->enabled_person[person_index]) return;
    if (!state->initialized ||
        !state->joint_raw[0] ||
        !state->joint_raw[1] ||
        !state->joint_raw[2]) {
        return;
    }
    for (i = 0; i < 3; i++) {
        if (!ptr_readable((BYTE*)state->joint_raw[i] + cfg->output_offset,
                          sizeof(float) * 3)) {
            return;
        }
        out[i] = (float*)((BYTE*)state->joint_raw[i] + cfg->output_offset);
        for (axis = 0; axis < 3; axis++) {
            if (!sane_probe_float(out[i][axis])) {
                return;
            }
            before[i][axis] = out[i][axis];
            desired[i][axis] = state->rest[i][axis];
        }
    }
    for (i = 0; i < 2; i++) {
        for (axis = 0; axis < 3; axis++) {
            desired[i][axis] = state->rest[i][axis] +
                               state->angle[i][axis];
            desired[i][axis] =
                body_chain_clamp_link_output_value(
                    cfg, i, axis, state->rest[i][axis], desired[i][axis]);
        }
    }
    for (i = 0; i < 3; i++) {
        for (axis = 0; axis < 3; axis++) {
            float delta = before[i][axis] - desired[i][axis];
            float abs_delta = physx_absf(delta);
            if (abs_delta > max_delta) {
                max_delta = abs_delta;
                max_joint = i;
                max_axis = axis;
            }
            out[i][axis] = desired[i][axis];
        }
    }
    if (!runtime_mode && cfg->override_animation) {
        neutralize_testicle_physics_animation(person, state);
    }
    if (defaults_cfg.debug &&
        max_delta > 0.00050f &&
        (!state->late_ownership_log_tick ||
         now - state->late_ownership_log_tick >= 1000)) {
        const char *joint_name =
            max_joint == 0 ? "Stesticles_joint01" :
            max_joint == 1 ? "Stesticles_joint02" :
            max_joint == 2 ? "Stesticles_jointEnd" : "unknown";
        state->late_ownership_log_tick = now;
        log_line("testicle-physics late-ownership clamp person=\"%s\" joint=\"%s\" axis=%d max_delta=%.5f before=(%.3f,%.3f,%.3f) desired=(%.3f,%.3f,%.3f) note=\"late frame re-applied enabled Testicle PhysX bone ownership after an engine-side update\"",
                 person,
                 joint_name,
                 max_axis,
                 max_delta,
                 max_joint >= 0 ? before[max_joint][0] : 0.0f,
                 max_joint >= 0 ? before[max_joint][1] : 0.0f,
                 max_joint >= 0 ? before[max_joint][2] : 0.0f,
                 max_joint >= 0 ? desired[max_joint][0] : 0.0f,
                 max_joint >= 0 ? desired[max_joint][1] : 0.0f,
                 max_joint >= 0 ? desired[max_joint][2] : 0.0f);
    }
}

static void run_testicle_physics_late_ownership(DWORD now)
{
    int i;
    body_profile_set_active_person_config(-1);
    if (!testicle_physics_cfg.enabled) {
        restore_tk17_testicle_inertia_all(now);
        return;
    }
    for (i = 0; i < 4; i++) {
        body_profile_set_active_person_config(i);
        if (testicle_physics_cfg.enabled &&
            testicle_physics_cfg.enabled_person[i] &&
            testicle_physics_live_ownership_matches(
                i, body_chain_active_person_state(i, 1))) {
            if (suppress_tk17_testicle_inertia_for_person(i, now)) {
                testicle_physics_late_ownership_for_person(i, now);
            }
        } else {
            if (!testicle_physics_cfg.enabled ||
                !testicle_physics_cfg.enabled_person[i]) {
                restore_tk17_testicle_inertia_for_person(i, now);
            }
        }
        body_profile_set_active_person_config(-1);
    }
}

static void physx_late_frame_ownership_tick(void)
{
    DWORD now = GetTickCount();
    LONGLONG perf_start = physx_perf_counter();
    LONGLONG phase_start;
    InterlockedExchange(&physx_late_ownership_active, 1);
    body_profile_set_active_person_config(-1);
    phase_start = physx_perf_counter();
    run_testicle_physics_late_ownership(now);
    physx_perf_add(PHYSX_PERF_LATE_TESTICLES, phase_start);
    phase_start = physx_perf_counter();
    run_breasts_physics_late_ownership(now);
    physx_perf_add(PHYSX_PERF_LATE_BREASTS, phase_start);
    phase_start = physx_perf_counter();
    run_butt_physics_late_ownership(now);
    physx_perf_add(PHYSX_PERF_LATE_BUTT, phase_start);
    body_profile_set_active_person_config(-1);
    InterlockedExchange(&physx_late_ownership_active, 0);
    physx_perf_add(PHYSX_PERF_LATE_OWNERSHIP, perf_start);
}

static void run_body_chain_physics(DWORD now)
{
    int i;
    int runtime_mode = body_chain_runtime_mode_active();
    int active_flags[4] = { 0, 0, 0, 0 };
    static int previous_active[4] = { 0, 0, 0, 0 };
    static DWORD seen_physics_setting_tick[4];
    static DWORD seen_collision_setting_tick[4];
    static DWORD seen_config_reload_tick[4];
    body_profile_set_active_person_config(-1);
    if (!body_chain_physics_cfg.enabled || !engine_FindObjC) {
        for (i = 0; i < 4; i++) {
            int was_active = previous_active[i];
            body_chain_person_state_t *state =
                body_chain_active_person_state(i, 0);
            body_profile_set_active_person_config(i);
            body_chain_reactivation_collision_until[i] = 0;
            body_chain_reactivation_collision_log_tick[i] = 0;
            if (runtime_mode) {
                body_chain_release_runtime_ownership_for_person(
                    i, 0, "penis-physics-disabled");
            }
            if (state && state->initialized) {
                unsigned int restored_output_mask =
                    (was_active && !runtime_mode)
                    ? restore_body_chain_output_rest_for_person(
                          i, state)
                    : 0;
                unsigned int restored_track_mask =
                    reset_body_chain_person_state_for_reactivation(
                        state);
                if (was_active && !runtime_mode) {
                    schedule_poseeditor_track_handoff_refresh(
                        i, restored_output_mask | restored_track_mask, now);
                }
            }
            previous_active[i] = 0;
            body_profile_set_active_person_config(-1);
        }
        return;
    }
    for (i = 0; i < 4; i++) {
        body_profile_set_active_person_config(i);
        active_flags[i] =
            body_chain_physics_cfg.enabled &&
            body_chain_physics_cfg.enabled_person[i];
        if (body_chain_physics_settings_change_tick[i] &&
            !body_chain_physics_settings_change_enabled[i] &&
            now - body_chain_physics_settings_change_tick[i] <= 3000u) {
            active_flags[i] = 0;
        }
        body_profile_set_active_person_config(-1);
    }

    for (i = 0; i < 4; i++) {
        body_profile_set_active_person_config(i);
        if (active_flags[i]) {
            int reset_for_reactivation = !previous_active[i];
            body_chain_person_state_t *state =
                body_chain_active_person_state(i, 0);
            if (body_chain_physics_settings_change_tick[i] &&
                body_chain_physics_settings_change_tick[i] !=
                    seen_physics_setting_tick[i]) {
                seen_physics_setting_tick[i] =
                    body_chain_physics_settings_change_tick[i];
                if (body_chain_physics_settings_change_enabled[i]) {
                    reset_for_reactivation = 1;
                }
            }
            if (body_chain_collision_settings_change_tick[i] &&
                body_chain_collision_settings_change_tick[i] !=
                    seen_collision_setting_tick[i]) {
                seen_collision_setting_tick[i] =
                    body_chain_collision_settings_change_tick[i];
                if (body_chain_collision_settings_change_enabled[i]) {
                    begin_body_chain_reactivation_collision_grace(
                        i, now, "penis-collision-reenabled");
                }
            }
            if (config_hot_reload_tick &&
                config_hot_reload_tick != seen_config_reload_tick[i] &&
                now - config_hot_reload_tick <= 3000u) {
                seen_config_reload_tick[i] = config_hot_reload_tick;
                begin_body_chain_reactivation_collision_grace(
                    i, now, "ini-hot-reload");
            }
            if (reset_for_reactivation) {
                if (runtime_mode) {
                    body_chain_release_runtime_ownership_for_person(
                        i, 0, "penis-physics-reenabled");
                }
                if (state && state->initialized) {
                    reset_body_chain_person_state_for_reactivation(
                        state);
                }
                begin_body_chain_reactivation_collision_grace(
                    i, now, "penis-physics-reenabled");
            }
            previous_active[i] = 1;
            run_body_chain_physics_for_person(i, now);
        } else {
            int was_active = previous_active[i];
            body_chain_person_state_t *state =
                body_chain_active_person_state(i, 0);
            if (body_chain_physics_settings_change_tick[i] &&
                body_chain_physics_settings_change_tick[i] !=
                    seen_physics_setting_tick[i]) {
                seen_physics_setting_tick[i] =
                    body_chain_physics_settings_change_tick[i];
            }
            if (config_hot_reload_tick &&
                config_hot_reload_tick != seen_config_reload_tick[i]) {
                seen_config_reload_tick[i] = config_hot_reload_tick;
            }
            body_chain_reactivation_collision_until[i] = 0;
            body_chain_reactivation_collision_log_tick[i] = 0;
            if (runtime_mode) {
                body_chain_release_runtime_ownership_for_person(
                    i, 0, "penis-physics-disabled");
            }
            if (state && state->initialized) {
                unsigned int restored_output_mask =
                    (was_active && !runtime_mode)
                    ? restore_body_chain_output_rest_for_person(
                          i, state)
                    : 0;
                unsigned int restored_track_mask =
                    reset_body_chain_person_state_for_reactivation(
                        state);
                if (was_active && !runtime_mode) {
                    schedule_poseeditor_track_handoff_refresh(
                        i, restored_output_mask | restored_track_mask, now);
                }
            }
            previous_active[i] = 0;
        }
        body_profile_set_active_person_config(-1);
    }
}

