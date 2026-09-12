/* Frozen pre-optimization functions for differential tests. */
static addon_equipment_definition_t *reference_addon_equipment_definition_find(
    const char *addon_id, int create)
{
    addon_equipment_definition_t *empty = NULL;
    int i;
    if (!addon_id || !addon_id[0]) return NULL;
    for (i = 0; i < ADDON_EQUIPMENT_DEFINITION_COUNT; i++) {
        addon_equipment_definition_t *definition =
            &addon_equipment_definitions[i];
        if (definition->addon_id[0] &&
            _stricmp(definition->addon_id, addon_id) == 0) {
            return definition;
        }
        if (!definition->addon_id[0]) {
            empty = definition;
            break;
        }
    }
    if (!create || !empty) return NULL;
    memset(empty, 0, sizeof(*empty));
    lstrcpynA(empty->addon_id, addon_id, sizeof(empty->addon_id));
    return empty;
}

static addon_equipment_slot_t *reference_addon_equipment_slot_find(
    const char *owner, const char *zone, int create)
{
    addon_equipment_slot_t *empty = NULL;
    addon_equipment_slot_t *oldest = NULL;
    DWORD oldest_age = 0;
    DWORD now = GetTickCount();
    int i;
    if (!owner || !owner[0] || !zone || !zone[0]) return NULL;
    for (i = 0; i < ADDON_EQUIPMENT_SLOT_COUNT; i++) {
        addon_equipment_slot_t *slot = &addon_equipment_slots[i];
        if (slot->owner[0] && slot->zone[0] &&
            _stricmp(slot->owner, owner) == 0 &&
            _stricmp(slot->zone, zone) == 0) {
            return slot;
        }
        if (!slot->owner[0]) {
            empty = slot;
            break;
        }
        if (slot->owner[0] &&
            (!oldest || now - slot->root_tick > oldest_age)) {
            oldest = slot;
            oldest_age = now - slot->root_tick;
        }
    }
    if (!create) return NULL;
    if (!empty) empty = oldest;
    if (!empty) return NULL;
    memset(empty, 0, sizeof(*empty));
    lstrcpynA(empty->owner, owner, sizeof(empty->owner));
    lstrcpynA(empty->zone, zone, sizeof(empty->zone));
    return empty;
}

static addon_active_slot_t *reference_addon_active_slot_find(const char *owner,
                                                    const char *addon_id,
                                                    int create)
{
    addon_active_slot_t *empty = NULL;
    addon_active_slot_t *oldest = NULL;
    DWORD oldest_age = 0;
    DWORD now = GetTickCount();
    int i;
    if (!owner || !owner[0] || !addon_id || !addon_id[0]) return NULL;
    for (i = 0; i < ADDON_ACTIVE_SLOT_COUNT; i++) {
        addon_active_slot_t *entry = &addon_active_slots[i];
        if (entry->owner[0] && entry->addon_id[0] &&
            _stricmp(entry->owner, owner) == 0 &&
            _stricmp(entry->addon_id, addon_id) == 0) {
            return entry;
        }
        if (!entry->owner[0] && !empty) empty = entry;
        if (entry->owner[0] &&
            (!oldest || now - entry->root_tick > oldest_age)) {
            oldest = entry;
            oldest_age = now - entry->root_tick;
        }
    }
    if (!create) return NULL;
    if (!empty) empty = oldest;
    if (!empty) return NULL;
    memset(empty, 0, sizeof(*empty));
    lstrcpynA(empty->owner, owner, sizeof(empty->owner));
    lstrcpynA(empty->addon_id, addon_id, sizeof(empty->addon_id));
    return empty;
}

static int reference_addon_chain_owner_body_ready(physx_sidecar_t *sc,
                                        physx_chain_t *chain,
                                        DWORD now)
{
    void *root_raw = NULL;
    float *root = NULL;
    float root_len = 0.0f;
    float epsilon = physics_environment_cfg.gravity_probe_motion_epsilon;
    const char *owner;
    if (!sc || !chain) return 0;
    owner = sc->addon_owner_person;
    if (!owner[0]) return 0;
    if (epsilon < 0.0001f) epsilon = 0.0001f;
    if (addon_body_root_pointer_for_person(owner, &root_raw, &root) && root) {
        root_len = physx_vec3_len(root);
        if (chain->addon_scene_visible &&
            chain->addon_body_root_raw &&
            chain->addon_body_root_raw != root_raw) {
            if (!chain->addon_body_root_miss_log_tick ||
                now - chain->addon_body_root_miss_log_tick >= 500u) {
                chain->addon_body_root_miss_log_tick = now;
                log_line("addon-chain owner instance changed chain=\"%s\" owner=\"%s\" previous_root_raw=%p current_root_raw=%p reason=\"game mode replaced the live person scene\" sidecar=\"%s\" note=\"the complete chain will reset atomically instead of carrying solver/output bindings into the replacement room\"",
                         chain->name,
                         owner,
                         chain->addon_body_root_raw,
                         root_raw,
                         sc->path);
            }
            return 0;
        }
        /* An established chain may observe a transient zero vector from the
           same live owner during an engine traversal update. That is not a
           scene replacement and must not restart the chain. */
        if (chain->addon_scene_visible &&
            chain->addon_body_root_raw == root_raw) {
            return 1;
        }
        if (!physics_environment_cfg.gravity_probe_require_nonzero_root ||
            root_len > epsilon) {
            /* Add-on readiness must exist even when the wearer's body profile
               intentionally disables penis and testicle physics. */
            if (!addon_owner_placement_ready(owner, root_raw, root, now)) {
                return 0;
            }
            chain->addon_body_root_raw = root_raw;
            chain->addon_body_root_initialized = 0;
            return 1;
        }
    }
    if (!chain->addon_body_root_miss_log_tick ||
        now - chain->addon_body_root_miss_log_tick >= 2000u) {
        chain->addon_body_root_miss_log_tick = now;
        log_line("addon-chain activation waiting chain=\"%s\" owner=\"%s\" root_raw=%p root_len=%.6f epsilon=%.6f reason=\"owner body is not loaded yet\" sidecar=\"%s\" note=\"no add-on PhysX output is written while TK17 still exposes the zero/placeholder person root\"",
                 chain->name,
                 owner,
                 root_raw,
                 root_len,
                 epsilon,
                 sc->path);
    }
    return 0;
}

static int reference_addon_chain_scene_visible(physx_sidecar_t *sc,
                                     physx_chain_t *chain,
                                     DWORD now,
                                     int *runtime_fallback_out,
                                     int *live_targets_out,
                                     int *writable_targets_out)
{
    int poseedit_visible = addon_output_scene_visible();
    int owner_person_index = sc ?
        addon_person_prefix_to_index(sc->addon_owner_person) : -1;
    int live_targets = 0;
    int writable_targets = 0;
    int root_target_live = 0;
    int root_target_seen = 0;
    int t;

    if (runtime_fallback_out) *runtime_fallback_out = 0;
    if (live_targets_out) *live_targets_out = 0;
    if (writable_targets_out) *writable_targets_out = 0;
    if (!sc || !chain || !sc->enabled || !sc->loaded ||
        !sc->addon_scene_active || !chain->addon_chain) {
        return -1;
    }
    if (sc->room_scene_sidecar) {
        /* Room joints are unique scene objects, not PersonXX equipment
           clones. Their active room path plus an exact named SJoint mapping
           is the ownership proof; PoseEditor person visibility and skin-
           palette ownership do not apply. */
        for (t = 0; t < chain->target_count; t++) {
            physx_target_t *target = &chain->targets[t];
            float *translation;
            if (!target->addon_simulated_target ||
                !target->object ||
                is_nil_engine_object(target->raw_object, target->object)) {
                continue;
            }
            live_targets++;
            if (target->s_translation_base &&
                target->s_rotation_base == target->s_translation_base &&
                target->s_rotation_offset == 0x06c &&
                target->s_translation_offset == 0x07c &&
                ptr_readable((BYTE*)target->s_translation_base + 0x06c,
                             0x01c)) {
                translation = (float*)((BYTE*)target->s_translation_base +
                                        target->s_translation_offset);
                if (physx_vec3_sane_limit(translation, 64.0f)) {
                    writable_targets++;
                }
            }
        }
        if (live_targets_out) *live_targets_out = live_targets;
        if (writable_targets_out) *writable_targets_out = writable_targets;
        if (runtime_fallback_out) *runtime_fallback_out = 1;
        return live_targets > 0 && writable_targets > 0 ? 1 : 0;
    }
    /* Non-PoseEditor modes briefly expose a zero owner root while their
       already-equipped add-on is being attached to the live person. Once a
       verified writable add-on has armed its controlled settle window, keep
       that window authoritative; checking the transient owner first caused
       every chain to reset roughly one second into a 1.2 second settle. */
    if (chain->addon_scene_visible &&
        chain->addon_root_settle_until_tick &&
        now < chain->addon_root_settle_until_tick) {
        if (runtime_fallback_out) *runtime_fallback_out = 1;
        return 1;
    }
    if (owner_person_index >= 0 && owner_person_index < 4) {
        /* PoseEdit's person slots can remain allocated in Freemode, Quick
           Mode, Sequencer, and Story Mode and intermittently report hidden.
           They are authoritative only when the PoseEditor scene itself is
           available; other modes use the live owner/target fallback below. */
        if (poseedit_visible >= 0) {
            int owner_visible =
                poseedit_scene_person_visible(owner_person_index);
            if (owner_visible >= 0) poseedit_visible = owner_visible;
        }
        /* The room modes can briefly publish a zero person root after the
           equipped add-on is already live. Use that root to gate first
           activation, but never let the transient placeholder tear down an
           established chain; the live target validation below remains the
           authority for an active add-on. */
        if (!addon_chain_owner_body_ready(sc, chain, now)) {
            return 0;
        }
    } else if (poseedit_visible < 0 && !sc->room_scene_sidecar) {
        return -1;
    }
    /* A positive PoseEditor result is authoritative. A zero is not: TK17
       leaves PoseEdit slots allocated in other game modes, where they report
       hidden even while the person and equipped add-on are live. Confirm a
       zero against the runtime add-on targets below before deactivating. */
    if (poseedit_visible > 0) return 1;

    for (t = 0; t < chain->target_count; t++) {
        physx_target_t *target = &chain->targets[t];
        int exact_live_target;
        int writable_target = 0;
        int root_target;
        float *translation;
        if (!target->addon_simulated_target) continue;
        root_target = !root_target_seen;
        root_target_seen = 1;
        if (!target->object ||
            is_nil_engine_object(target->raw_object, target->object)) {
            continue;
        }
        exact_live_target = target->addon_skin_bound ||
                            !target->addon_object_name_fallback;
        if (!exact_live_target) continue;
        live_targets++;
        if (chain->object_transform_chain) {
            if (real_SSimpleTransform_RotationSet &&
                target->s_raw_object &&
                target->s_object &&
                !is_nil_engine_object(target->s_raw_object,
                                      target->s_object)) {
                writable_target = 1;
                writable_targets++;
            }
        } else if (target->s_translation_base &&
            target->s_translation_offset >= 0 &&
            ptr_readable((BYTE*)target->s_translation_base +
                             target->s_translation_offset,
                         sizeof(float) * 3)) {
            translation = (float*)((BYTE*)target->s_translation_base +
                                    target->s_translation_offset);
            if (physx_vec3_sane_limit(translation, 8.0f)) {
                writable_target = 1;
                writable_targets++;
            }
        }
        if (root_target) {
            root_target_live = writable_target ? 2 : 1;
        }
    }

    if (live_targets_out) *live_targets_out = live_targets;
    if (writable_targets_out) *writable_targets_out = writable_targets;
    if (!root_target_live) return poseedit_visible == 0 ? 0 : -1;
    if (runtime_fallback_out) *runtime_fallback_out = 1;
    return 1;
}
