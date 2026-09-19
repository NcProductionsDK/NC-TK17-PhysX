/* Customizer startup acceleration. Included after the add-on owner helpers. */
static int physx_prepare_customizer_entry(void)
{
    int i, c, t;
    if (!physx_customizer_entry_pending) return 0;
    physx_customizer_entry_pending = 0;
    if (!physx_customizer_active) return 0;
    last_update_tick = 0;
    memset(addon_owner_placement_states, 0, sizeof(addon_owner_placement_states));
    for (i = 0; i < 4; i++) {
        /* Customizer reuses these nodes. Request a native transform refresh
           even if its stationary camera leaves the old traversal cached.
           Change only the engine's version, never the authored pose/matrix. */
        static const char *nodes[] = { "TRS_group", "root", "spine_joint04" };
        for (int n = 0; n < 3; n++) {
            char name[256];
            make_body_runtime_name(name, sizeof(name), body_chain_person_name(i), nodes[n]);
            void *raw = resolve_axis_map_raw(name);
            if (raw && ptr_readable((BYTE*)raw + 0xfc, sizeof(unsigned int)))
                body_chain_mark_runtime_transform_dirty(raw);
        }
        body_chain_person_state_t *gravity_states[] = {
            &runtime_body_chain_person_states[i], &runtime_testicle_physics_states[i],
            &breasts_physics_states[i].gravity_motion, &butt_physics_states[i].motion
        };
        for (int g = 0; g < 4; g++) {
            reset_body_chain_gravity_state(gravity_states[g]);
            gravity_states[g]->gravity_probe_promoted = 0;
            gravity_states[g]->gravity_probe_captured = 0;
            gravity_states[g]->gravity_probe_sampled = 0;
        }
        memset(&runtime_body_chain_room_gravity_cache[i], 0,
            sizeof(runtime_body_chain_room_gravity_cache[i]));
        memset(&runtime_testicle_room_gravity_cache[i], 0,
            sizeof(runtime_testicle_room_gravity_cache[i]));
        memset(&breasts_physics_room_gravity_cache[i], 0,
            sizeof(breasts_physics_room_gravity_cache[i]));
        memset(&butt_physics_room_gravity_cache[i], 0,
            sizeof(butt_physics_room_gravity_cache[i]));
        breasts_physics_states[i].gravity_reference_valid = 0;
        butt_physics_states[i].gravity_reference_valid = 0;
        memset(&breasts_physics_states[i].spine_gravity_sample, 0,
            sizeof(breasts_physics_states[i].spine_gravity_sample));
        memset(&breasts_physics_states[i].spine_gravity_reference_sample, 0,
            sizeof(breasts_physics_states[i].spine_gravity_reference_sample));
        /* Verify live sources immediately, rather than reuse the previous
           mode's cached bindings until the ordinary verification interval. */
        runtime_body_chain_person_states[i].cache_verify_tick = 0;
        runtime_body_chain_person_states[i].resolve_retry_tick = 0;
        runtime_testicle_physics_states[i].cache_verify_tick = 0;
        runtime_testicle_physics_states[i].resolve_retry_tick = 0;
        breasts_physics_states[i].cache_verify_tick = 0;
        breasts_physics_states[i].resolve_retry_tick = 0;
        butt_physics_states[i].cache_verify_tick = 0;
        butt_physics_states[i].resolve_retry_tick = 0;
        reset_body_chain_collider_state(&body_chain_collider_states[i]);
        clear_body_chain_prev_collider_for_person(i);
    }
    log_line("physics Customizer reference refresh note=\"requested native placement/root/chest update; fresh gravity and collision samples required without changing authored transforms\"");
    log_line("physics Customizer collision isolation note=\"body and add-on contacts exclude other room persons; own-body contacts remain enabled\"");
    for (i = 0; i < sidecar_count; i++) {
        for (c = 0; c < sidecars[i].chain_count; c++) {
            physx_chain_t *chain = &sidecars[i].chains[c];
            chain->customizer_ready_samples = 0;
            chain->customizer_sample_tick = 0;
            chain->customizer_owner_raw = NULL;
            for (t = 0; t < chain->target_count; t++)
                chain->targets[t].addon_resolve_retry_tick = 0;
        }
    }
    return 1;
}

/* Only verified live skin traversal matrices can replace the fixed wait.
   Provisional scene transforms, generic name fallbacks and object-transform
   add-ons retain their ordinary startup path. No transform is written here. */
static int addon_customizer_sample_ready(physx_chain_t *chain, void *owner_raw,
                                         DWORD now)
{
    float matrices[32][12];
    void *bases[32] = {0};
    unsigned int samples = chain->customizer_ready_samples;
    int count = 0, changed = 0, t, row, axis;
    DWORD quiet_ms = 160u;
    if (physics_environment_cfg.gravity_probe_camera_quiet_ms > (int)quiet_ms)
        quiet_ms = physics_environment_cfg.gravity_probe_camera_quiet_ms;
    if (physics_environment_cfg.body_chain_camera_quarantine_ms > (int)quiet_ms)
        quiet_ms = physics_environment_cfg.body_chain_camera_quarantine_ms;
    if (!physx_customizer_startup_active(now) || !owner_raw ||
        chain->object_transform_chain || chain->target_count <= 0 ||
        chain->target_count > 32 || !captured_camera_inverse_valid ||
        (captured_camera_change_tick &&
         now - captured_camera_change_tick < quiet_ms)) goto unready;
    if (chain->customizer_owner_raw != owner_raw) samples = 0;
    for (t = 0; t < chain->target_count; t++) {
        physx_target_t *target = &chain->targets[t];
        BYTE *base = (BYTE*)target->s_translation_base;
        if (!target->addon_simulated_target) continue;
        count++;
        if (!target->object || !target->s_object ||
            is_nil_engine_object(target->raw_object, target->object) ||
            is_nil_engine_object(target->s_raw_object, target->s_object) ||
            (target->addon_object_name_fallback && !target->addon_skin_bound) ||
            !base || target->s_rotation_base != base ||
            (base != target->s_raw_object && base != target->s_object) ||
            target->s_rotation_offset != 0x038 ||
            target->s_translation_offset != 0x048 ||
            !ptr_readable(base + 0x018, 0x03c)) goto unready;
        bases[t] = base;
        for (row = 0; row < 4; row++) {
            const float *v = (const float*)(base + 0x018 + row * 0x10);
            if (row < 3 ? !addon_vector_basis_like(v) :
                          !physx_vec3_sane_limit(v, 8.0f)) goto unready;
            for (axis = 0; axis < 3; axis++) {
                int k = row * 3 + axis;
                matrices[t][k] = v[axis];
                if (samples && physx_absf(v[axis] -
                    chain->customizer_target_matrices[t][k]) >
                        (row < 3 ? 0.02f : 0.005f)) changed = 1;
            }
        }
        /* Reject collapsed/skewed provisional bases too. */
        for (row = 0; row < 3; row++) {
            float *a = &matrices[t][row * 3];
            float *b = &matrices[t][((row + 1) % 3) * 3];
            if (physx_absf(vec3_dot(a, b)) > 0.1f) goto unready;
        }
        if (chain->customizer_target_objects[t] != target->object ||
            chain->customizer_target_bases[t] != base) changed = 1;
    }
    if (!count) goto unready;
    if (changed) samples = 0;
    /* Repeated queries in one frame cannot manufacture confirmation. */
    if (samples && chain->customizer_sample_tick == now) return 0;
    if (!samples) chain->customizer_ready_tick = now;
    for (t = 0; t < chain->target_count; t++) {
        chain->customizer_target_objects[t] = chain->targets[t].object;
        chain->customizer_target_bases[t] = bases[t];
        if (bases[t]) memcpy(chain->customizer_target_matrices[t], matrices[t],
                             sizeof(matrices[t]));
    }
    chain->customizer_owner_raw = owner_raw;
    chain->customizer_sample_tick = now;
    chain->customizer_ready_samples = samples + 1;
    return chain->customizer_ready_samples >= 3 &&
           now - chain->customizer_ready_tick >= 32u;
unready:
    chain->customizer_ready_samples = 0;
    return 0;
}

static void addon_customizer_try_finish_settle(physx_sidecar_t *sc,
                                               physx_chain_t *chain, DWORD now)
{
    void *root_raw = NULL;
    float *root = NULL;
    if (!physx_customizer_startup_active(now) || sc->room_scene_sidecar ||
        !chain->addon_root_settle_until_tick ||
        now >= chain->addon_root_settle_until_tick) return;
    /* Unlike the ordinary settle window's visibility shortcut, early release
       must prove that the current owner is loaded on every sampled frame. */
    if (!addon_body_root_pointer_for_person(sc->addon_owner_person, &root_raw, &root) ||
        !root || !physx_vec3_sane_limit(root, 10000.0f) ||
        (physics_environment_cfg.gravity_probe_require_nonzero_root &&
         physx_vec3_len(root) <= 0.0001f) ||
        !addon_chain_owner_body_ready(sc, chain, now)) {
        chain->customizer_ready_samples = 0;
        return;
    }
    if (!addon_customizer_sample_ready(chain, root_raw, now)) return;
    chain->addon_root_settle_until_tick = 0;
    chain->addon_live_layout_retry_tick = 0;
    log_line("addon-chain Customizer ready chain=\"%s\" samples=%u elapsed_ms=%lu note=\"verified live matrices; gravity and collision camera guards remain active\"",
             chain->name, chain->customizer_ready_samples,
             (unsigned long)(now - chain->addon_root_seen_tick));
}
