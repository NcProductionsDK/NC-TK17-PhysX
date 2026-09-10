typedef struct physx_settings_binding_t {
    const char *param_name;
    const char *section;
    const char *key;
} physx_settings_binding_t;

typedef struct body_profile_log_observer_t {
    int initialized;
    DWORD offset;
    char partial[512];
    int partial_len;
    DWORD observed_size;
    FILETIME observed_write_time;
    int metadata_valid;
} body_profile_log_observer_t;

static const physx_settings_binding_t physx_settings_bindings[] = {
    { "NCPhysXBreastsPhysics", BREASTS_PHYSICS_CONFIG_SECTION, "enabled" },
    { "NCPhysXPenisPhysics", PENIS_PHYSICS_CONFIG_SECTION, "enabled" },
    { "NCPhysXTesticlePhysics", TESTICLE_PHYSICS_CONFIG_SECTION, "enabled" },
    { "NCPhysXButtPhysics", BUTT_PHYSICS_CONFIG_SECTION, "enabled" },
    { "NCPhysXBodyColliders", BODY_COLLIDERS_CONFIG_SECTION, "enabled" },
    { "NCPhysXBreastsCollision", BODY_COLLIDERS_CONFIG_SECTION, "breasts_collision_enabled" },
    { "NCPhysXButtCollision", BODY_COLLIDERS_CONFIG_SECTION, "butt_collision_enabled" },
    { "NCPhysXPenisCollision", BODY_COLLIDERS_CONFIG_SECTION, "penis_collision_enabled" },
    { "NCPhysXTesticleCollision", BODY_COLLIDERS_CONFIG_SECTION, "testicle_collision_enabled" },
    { "NCPhysXBreastsCollisionScope", BREASTS_PHYSICS_CONFIG_SECTION, "collision_scope" },
    { "NCPhysXButtCollisionScope", BUTT_PHYSICS_CONFIG_SECTION, "collision_scope" },
    { "NCPhysXPenisCollisionScope", PENIS_PHYSICS_CONFIG_SECTION, "collision_scope" },
    { "NCPhysXTesticleCollisionScope", TESTICLE_PHYSICS_CONFIG_SECTION, "collision_scope" },
    { "NCPhysXColliderVisuals", BODY_COLLIDERS_CONFIG_SECTION, "debug_draw" },
    { "NCPhysXWorldGravity", "physics_environment", "gravity_apply_to_body_chain" },
    { "NCPhysXWorldWind", "physics_environment", "wind_enabled" },
    { "NCPhysXBreastsRoomCollision", BREASTS_PHYSICS_CONFIG_SECTION, "room_collision_enabled" },
    { "NCPhysXPenisRoomCollision", PENIS_PHYSICS_CONFIG_SECTION, "room_collision_enabled" },
    { "NCPhysXTesticleRoomCollision", TESTICLE_PHYSICS_CONFIG_SECTION, "room_collision_enabled" },
    { "NCPhysXButtRoomCollision", BUTT_PHYSICS_CONFIG_SECTION, "room_collision_enabled" }
};

static int physx_settings_bool_value(const char *value, int *enabled)
{
    if (!value || !enabled) return 0;
    if (_stricmp(value, "ON") == 0 || _stricmp(value, "YES") == 0 ||
        _stricmp(value, "TRUE") == 0 || strcmp(value, "1") == 0) {
        *enabled = 1;
        return 1;
    }
    if (_stricmp(value, "OFF") == 0 || _stricmp(value, "NO") == 0 ||
        _stricmp(value, "FALSE") == 0 || strcmp(value, "0") == 0) {
        *enabled = 0;
        return 1;
    }
    return 0;
}

static const char *physx_settings_collision_scope_value(const char *value)
{
    static const char *const values[] = {
        "genitals_only",
        "genitals_only_all",
        "pelvis_and_genitals_only",
        "pelvis_and_genitals_only_all",
        "pelvis_genitals_hands_only",
        "pelvis_genitals_hands_only_all",
        "full_body",
        "full_body_all",
        "hands_only",
        "hands_only_all"
    };
    size_t i;
    if (!value) return NULL;
    if (_stricmp(value, "pelvis_genitals_and_hands_only") == 0) {
        return "pelvis_genitals_hands_only";
    }
    if (_stricmp(value, "pelvis_genitals_and_hands_only_all") == 0 ||
        _stricmp(value, "pelvis_genitals_and_hands_only_") == 0) {
        return "pelvis_genitals_hands_only_all";
    }
    for (i = 0; i < sizeof(values) / sizeof(values[0]); i++) {
        if (_stricmp(value, values[i]) == 0) return values[i];
    }
    return NULL;
}

static int physx_settings_enabled_person_index(const char *key)
{
    if (!key) return -1;
    if (strcmp(key, "enabled_person01") == 0) return 0;
    if (strcmp(key, "enabled_person02") == 0) return 1;
    if (strcmp(key, "enabled_person03") == 0) return 2;
    if (strcmp(key, "enabled_person04") == 0) return 3;
    return -1;
}

static void physx_mark_penis_physics_setting_change(const char *key,
                                                    int enabled)
{
    DWORD now = GetTickCount();
    int person_index = physx_settings_enabled_person_index(key);
    int i;
    if (person_index >= 0) {
        body_chain_physics_settings_change_tick[person_index] = now;
        body_chain_physics_settings_change_enabled[person_index] = enabled;
        return;
    }
    if (key && strcmp(key, "enabled") == 0) {
        for (i = 0; i < 4; i++) {
            body_chain_physics_settings_change_tick[i] = now;
            body_chain_physics_settings_change_enabled[i] = enabled;
        }
    }
}

static void physx_mark_testicle_physics_setting_change(const char *key,
                                                       int enabled)
{
    DWORD now = GetTickCount();
    int person_index = physx_settings_enabled_person_index(key);
    int i;
    if (person_index >= 0) {
        testicle_physics_settings_change_tick[person_index] = now;
        testicle_physics_settings_change_enabled[person_index] = enabled;
        return;
    }
    if (key && strcmp(key, "enabled") == 0) {
        for (i = 0; i < 4; i++) {
            testicle_physics_settings_change_tick[i] = now;
            testicle_physics_settings_change_enabled[i] = enabled;
        }
    }
}

static void physx_mark_penis_collision_setting_change(int enabled)
{
    DWORD now = GetTickCount();
    int i;
    for (i = 0; i < 4; i++) {
        body_chain_collision_settings_change_tick[i] = now;
        body_chain_collision_settings_change_enabled[i] = enabled;
    }
}

static int body_profile_build_sidecar_path_a(const char *body_path,
                                             char *out,
                                             size_t outsz)
{
    char *dot;
    if (!body_path || !out || outsz == 0) return 0;
    lstrcpynA(out, body_path, (int)outsz);
    dot = strrchr(out, '.');
    if (!dot) return 0;
    lstrcpynA(dot, ".physx.ini", (int)(outsz - (size_t)(dot - out)));
    return 1;
}

static int body_profile_build_body_path_from_sidecar_a(const char *sidecar_path,
                                                       char *out,
                                                       size_t outsz)
{
    char *dot;
    if (!sidecar_path || !out || outsz == 0) return 0;
    lstrcpynA(out, sidecar_path, (int)outsz);
    dot = strrchr(out, '.');
    if (!dot) return 0;
    if (dot > out && _stricmp(dot, ".ini") == 0) {
        *dot = 0;
        dot = strrchr(out, '.');
    }
    if (!dot) return 0;
    lstrcpynA(dot, ".bs", (int)(outsz - (size_t)(dot - out)));
    return 1;
}

static void body_profile_tk17_cname_from_sidecar_a(const char *sidecar_path,
                                                   char *out,
                                                   size_t outsz)
{
    const char *addons;
    const char *start;
    const char *end;
    size_t len;
    if (!out || outsz == 0) return;
    out[0] = 0;
    if (!sidecar_path) return;
    addons = strstr(sidecar_path, "\\Addons\\");
    if (!addons) addons = strstr(sidecar_path, "/Addons/");
    if (!addons) return;
    start = addons + 8;
    end = strpbrk(start, "\\/");
    if (!end || end <= start) return;
    len = (size_t)(end - start);
    if (len >= outsz) len = outsz - 1;
    memcpy(out, start, len);
    out[len] = 0;
    len = strlen(out);
    if (len > 2) {
        char *v = out + len - 1;
        while (v >= out && ((*v >= '0' && *v <= '9') || *v == '.')) v--;
        if (v > out && v[-1] == '-' && (*v == 'v' || *v == 'V')) {
            v[-1] = 0;
        }
    }
    while (*out && strchr("!/@=+~#_ $&-", *out)) {
        memmove(out, out + 1, strlen(out));
    }
}

static DWORD body_profile_xxh32_a(const char *s)
{
    const unsigned char *p = (const unsigned char*)s;
    size_t len;
    size_t i = 0;
    DWORD h32;
    const DWORD P1 = 0x9E3779B1u;
    const DWORD P2 = 0x85EBCA77u;
    const DWORD P3 = 0xC2B2AE3Du;
    const DWORD P4 = 0x27D4EB2Fu;
    const DWORD P5 = 0x165667B1u;
#define BODY_PROFILE_ROTL32(x, r) (((x) << (r)) | ((x) >> (32 - (r))))
#define BODY_PROFILE_READ32(q) ((DWORD)(q)[0] | ((DWORD)(q)[1] << 8) | ((DWORD)(q)[2] << 16) | ((DWORD)(q)[3] << 24))
    if (!s) return 0;
    len = strlen(s);
    if (len >= 16) {
        DWORD v1 = P1 + P2;
        DWORD v2 = P2;
        DWORD v3 = 0;
        DWORD v4 = (DWORD)(0u - P1);
        size_t limit = len - 16;
        while (i <= limit) {
            v1 += BODY_PROFILE_READ32(p + i) * P2;
            v1 = BODY_PROFILE_ROTL32(v1, 13) * P1;
            i += 4;
            v2 += BODY_PROFILE_READ32(p + i) * P2;
            v2 = BODY_PROFILE_ROTL32(v2, 13) * P1;
            i += 4;
            v3 += BODY_PROFILE_READ32(p + i) * P2;
            v3 = BODY_PROFILE_ROTL32(v3, 13) * P1;
            i += 4;
            v4 += BODY_PROFILE_READ32(p + i) * P2;
            v4 = BODY_PROFILE_ROTL32(v4, 13) * P1;
            i += 4;
        }
        h32 = BODY_PROFILE_ROTL32(v1, 1) +
              BODY_PROFILE_ROTL32(v2, 7) +
              BODY_PROFILE_ROTL32(v3, 12) +
              BODY_PROFILE_ROTL32(v4, 18);
    } else {
        h32 = P5;
    }
    h32 += (DWORD)len;
    while (i + 4 <= len) {
        h32 += BODY_PROFILE_READ32(p + i) * P3;
        h32 = BODY_PROFILE_ROTL32(h32, 17) * P4;
        i += 4;
    }
    while (i < len) {
        h32 += (DWORD)p[i] * P5;
        h32 = BODY_PROFILE_ROTL32(h32, 11) * P1;
        i++;
    }
    h32 ^= h32 >> 15;
    h32 *= P2;
    h32 ^= h32 >> 13;
    h32 *= P3;
    h32 ^= h32 >> 16;
#undef BODY_PROFILE_ROTL32
#undef BODY_PROFILE_READ32
    return h32;
}

static int body_profile_signature_index(const body_profile_sidecar_entry_t *entry,
                                        const char *name)
{
    int i;
    if (!entry || !name || !name[0]) return -1;
    for (i = 0; i < entry->signature_count; i++) {
        if (_stricmp(entry->signature[i], name) == 0) return i;
    }
    return -1;
}

static int body_profile_signature_name_allowed_a(const char *name,
                                                 const char *line)
{
    size_t len;
    (void)line;
    if (!name || !name[0]) return 0;
    len = strlen(name);
    if (len < 6 || len >= BODY_PROFILE_SIGNATURE_LEN) return 0;
    if (strchr(name, '/') || strchr(name, '\\')) return 0;
    if (_stricmp(name, "TRS_group") == 0 ||
        _stricmp(name, "STRS_group") == 0 ||
        _stricmp(name, "root") == 0 ||
        _stricmp(name, "Sroot") == 0 ||
        _stricmp(name, "body_mesh_group") == 0 ||
        _stricmp(name, "Sbody_mesh_group") == 0 ||
        _stricmp(name, "body_subdiv_cage") == 0 ||
        _stricmp(name, "Sbody_subdiv_cage") == 0 ||
        _stricmp(name, "body_subdiv_cageShape") == 0 ||
        _stricmp(name, "Sbody_subdiv_cageShape") == 0) {
        return 0;
    }
    if (contains_i(name, "_texture") ||
        contains_i(name, "Shader") ||
        contains_i(name, "_joint") ||
        contains_i(name, "_locator") ||
        contains_i(name, "_target") ||
        contains_i(name, "_group")) {
        return 0;
    }

    /*
       Body sidecars must not bind from generic body02/body03 names.  Those
       names exist on several add-on bodies, so accepting body_*_morph or
       every BlendControl lets one sidecar bleed into every loaded person.
       Keep only distinctive morph controls for automatic identity matching;
       direct body-load/body-select binding still works without signatures.
    */
    if (contains_i(name, "body_blends_") ||
        contains_i(name, "_morph") ||
        contains_i(name, "body_subdiv") ||
        contains_i(name, "__body_")) {
        return 0;
    }
    if (contains_i(name, "Dangly") ||
        contains_i(name, "Bulky") ||
        contains_i(name, "Chin") ||
        contains_i(name, "Under")) {
        return 1;
    }
    return 0;
}

static void body_profile_add_signature_a(body_profile_sidecar_entry_t *entry,
                                         const char *name)
{
    if (!entry || !name || !name[0]) return;
    if (entry->signature_count >= BODY_PROFILE_SIGNATURE_COUNT) return;
    if (body_profile_signature_index(entry, name) >= 0) return;
    lstrcpynA(entry->signature[entry->signature_count], name,
              sizeof(entry->signature[entry->signature_count]));
    entry->signature_count++;
}

static int body_profile_extract_object_name_a(const char *line,
                                              char *out,
                                              size_t outsz)
{
    const char *p;
    const char *q;
    size_t len;
    if (!line || !out || outsz == 0) return 0;
    out[0] = 0;
    p = strstr(line, "Object.Name");
    if (!p) return 0;
    p = strchr(p, '"');
    if (!p) return 0;
    p++;
    q = strchr(p, '"');
    if (!q || q <= p) return 0;
    len = (size_t)(q - p);
    if (len >= outsz) len = outsz - 1;
    memcpy(out, p, len);
    out[len] = 0;
    return out[0] != 0;
}

static void body_profile_build_signatures_from_body_a(
    body_profile_sidecar_entry_t *entry)
{
    FILE *f;
    char line[768];
    char name[BODY_PROFILE_SIGNATURE_LEN];
    int pass;
    if (!entry || !entry->body_path[0]) return;
    memset(entry->signature, 0, sizeof(entry->signature));
    entry->signature_count = 0;
    memset(entry->person_signature_mask, 0, sizeof(entry->person_signature_mask));
    f = fopen(entry->body_path, "rb");
    if (!f) return;
    for (pass = 0; pass < 2 && entry->signature_count < BODY_PROFILE_SIGNATURE_COUNT; pass++) {
        fseek(f, 0, SEEK_SET);
        while (fgets(line, sizeof(line), f)) {
            if (!body_profile_extract_object_name_a(line, name, sizeof(name))) {
                continue;
            }
            if (!body_profile_signature_name_allowed_a(name, line)) {
                continue;
            }
            if (pass == 0 &&
                !(contains_i(name, "Dangly") ||
                  contains_i(name, "Bulky") ||
                  contains_i(name, "Chin") ||
                  contains_i(name, "Under"))) {
                continue;
            }
            body_profile_add_signature_a(entry, name);
            if (entry->signature_count >= BODY_PROFILE_SIGNATURE_COUNT) break;
        }
    }
    fclose(f);
}

static int body_profile_parse_person_body_tsnode_a(const char *name,
                                                   int *person_index_out,
                                                   const char **tail_out)
{
    const char *p;
    int person_number;
    if (person_index_out) *person_index_out = -1;
    if (tail_out) *tail_out = NULL;
    if (!name || _strnicmp(name, "Person", 6) != 0) return 0;
    p = name + 6;
    if (p[0] < '0' || p[0] > '9' || p[1] < '0' || p[1] > '9') return 0;
    person_number = (p[0] - '0') * 10 + (p[1] - '0');
    p += 2;
    if (person_number < 1 || person_number > BODY_PROFILE_PERSON_COUNT) return 0;
    if (_strnicmp(p, "Body:", 5) != 0) return 0;
    if (person_index_out) *person_index_out = person_number - 1;
    if (tail_out) *tail_out = p + 5;
    return 1;
}

static int body_profile_popcount32(DWORD v)
{
    int count = 0;
    while (v) {
        count += (int)(v & 1u);
        v >>= 1;
    }
    return count;
}

static void body_profile_activate_sidecar_for_person_a(int person_index,
                                                       int body_slot,
                                                       const char *body_path,
                                                       const char *sidecar_path,
                                                       const char *reason);

static DWORD body_profile_last_runtime_probe_tick;
static DWORD body_profile_runtime_miss_start_tick[BODY_PROFILE_PERSON_COUNT];
static int body_profile_signature_ambiguity_logged[BODY_PROFILE_PERSON_COUNT];
static int body_profile_signature_ambiguity_hits[BODY_PROFILE_PERSON_COUNT];

static int body_profile_signature_is_generic_a(const char *name)
{
    if (!name || !name[0]) return 1;
    return _stricmp(name, "TesticlesDangly") == 0 ||
           _stricmp(name, "TesticlesVanilla") == 0 ||
           _stricmp(name, "BulkyPenis") == 0 ||
           _stricmp(name, "EyeUnderL") == 0 ||
           _stricmp(name, "EyeUnderR") == 0;
}

static int body_profile_runtime_object_exists_a(const char *name)
{
    void *raw = NULL;
    void *obj = NULL;
    if (!name || !name[0]) return 0;
    obj = resolve_find_obj(name, &raw);
    if (!obj || is_nil_engine_object(raw, obj)) {
        obj = resolve_script_engine_obj(name, &raw);
    }
    return obj && !is_nil_engine_object(raw, obj);
}

static int body_profile_runtime_signature_exists_a(int person_index,
                                                   const char *leaf,
                                                   char *matched,
                                                   size_t matched_sz)
{
    char name[192];
    if (matched && matched_sz) matched[0] = 0;
    if (person_index < 0 || person_index >= BODY_PROFILE_PERSON_COUNT ||
        !leaf || !leaf[0]) {
        return 0;
    }
    _snprintf(name, sizeof(name), "Person%02dBody:%s",
              person_index + 1, leaf);
    name[sizeof(name) - 1] = 0;
    if (!body_profile_runtime_object_exists_a(name)) return 0;
    if (matched && matched_sz) {
        lstrcpynA(matched, name, (int)matched_sz);
    }
    return 1;
}

static int body_profile_runtime_person_body_present_a(int person_index)
{
    char name[160];
    static const char *anchors[] = {
        "body_subdiv_cageShape",
        "body_subdiv_cage",
        "TRS_group",
        "STRS_group"
    };
    int i;
    if (person_index < 0 || person_index >= BODY_PROFILE_PERSON_COUNT) {
        return 0;
    }
    for (i = 0; i < (int)(sizeof(anchors) / sizeof(anchors[0])); i++) {
        _snprintf(name, sizeof(name), "Person%02dBody:%s",
                  person_index + 1, anchors[i]);
        name[sizeof(name) - 1] = 0;
        if (body_profile_runtime_object_exists_a(name)) return 1;
    }
    return 0;
}

static int body_profile_bind_strength_from_reason_a(const char *reason)
{
    if (!reason || !reason[0]) return 2;
    if (contains_i(reason, "signature")) return 1;
    return 2;
}

static void body_profile_clear_person_sidecar_a(int person_index,
                                                const char *reason)
{
    if (person_index < 0 || person_index >= BODY_PROFILE_PERSON_COUNT) return;
    if (!body_profile_person_sidecar_active[person_index] &&
        !body_profile_person_sidecar_path[person_index][0]) {
        return;
    }
    log_line("body-profile sidecar fallback person=\"Person%02d\" source=\"%s\" note=\"live PersonXXBody no longer matches an active body sidecar; using global PhysX INI for this person\"",
             person_index + 1, reason ? reason : "runtime-signature-miss");
    body_profile_person_sidecar_active[person_index] = 0;
    body_profile_person_sidecar_path[person_index][0] = 0;
    body_profile_person_body_path[person_index][0] = 0;
    body_profile_person_body_hash[person_index] = 0;
    body_profile_person_bind_strength[person_index] = 0;
    memset(&body_profile_person_sidecar_write_time[person_index], 0,
           sizeof(body_profile_person_sidecar_write_time[person_index]));
    InterlockedExchange(&body_profile_reload_pending, 1);
}

static void body_profile_probe_runtime_bindings(DWORD now)
{
    int person_index;
    int i;
    if (body_profile_last_runtime_probe_tick &&
        now - body_profile_last_runtime_probe_tick < 1000u) {
        return;
    }
    body_profile_last_runtime_probe_tick = now;
    if (!engine_FindObjC && !captured_script_engine) return;

    for (person_index = 0;
         person_index < BODY_PROFILE_PERSON_COUNT;
         person_index++) {
        int body_present = body_profile_runtime_person_body_present_a(person_index);
        int best = -1;
        int best_hits = 0;
        int best_strong_hits = 0;
        int tied = 0;
        int exact_bound = body_profile_person_sidecar_active[person_index] &&
                          body_profile_person_bind_strength[person_index] >= 2;
        int active_hits = 0;
        int active_strong_hits = 0;
        int active_strong_total = 0;
        int active_signature_count = 0;
        if (!exact_bound && !defaults_cfg.debug) {
            body_profile_runtime_miss_start_tick[person_index] = 0;
            continue;
        }

        for (i = 0; i < BODY_PROFILE_SIDECAR_COUNT; i++) {
            body_profile_sidecar_entry_t *entry = &body_profile_sidecars[i];
            int sig;
            int hits = 0;
            int strong_total = 0;
            int strong_hits = 0;
            DWORD mask = 0;
            if (!entry->active || entry->signature_count <= 0) continue;
            for (sig = 0; sig < entry->signature_count &&
                          sig < BODY_PROFILE_SIGNATURE_COUNT && sig < 32;
                 sig++) {
                char matched[192];
                int generic = body_profile_signature_is_generic_a(
                    entry->signature[sig]);
                if (!generic) strong_total++;
                if (!body_profile_runtime_signature_exists_a(
                        person_index, entry->signature[sig],
                        matched, sizeof(matched))) {
                    continue;
                }
                hits++;
                mask |= (1u << sig);
                if (!generic) strong_hits++;
                if (defaults_cfg.debug &&
                    !(entry->person_signature_mask[person_index] &
                      (1u << sig))) {
                    entry->person_signature_mask[person_index] |= (1u << sig);
                    log_line("body-profile runtime-hit person=\"Person%02d\" body_type=body%02d hit=\"%s\" hits=%d/%d sidecar=\"%s\" note=\"matching live PersonXXBody nodes against registered sidecar\"",
                             person_index + 1, entry->body_slot + 1,
                             matched, hits, entry->signature_count,
                             entry->sidecar_path);
                }
            }

            if (exact_bound &&
                _stricmp(entry->sidecar_path,
                         body_profile_person_sidecar_path[person_index]) == 0) {
                active_hits = hits;
                active_strong_hits = strong_hits;
                active_strong_total = strong_total;
                active_signature_count = entry->signature_count;
            }

            if (strong_total > 0) {
                if (strong_hits <= 0) continue;
            } else {
                if (hits < BODY_PROFILE_SIGNATURE_MIN_HITS ||
                    hits < entry->signature_count) {
                    continue;
                }
            }

            if (strong_hits > best_strong_hits ||
                (strong_hits == best_strong_hits && hits > best_hits)) {
                best = i;
                best_hits = hits;
                best_strong_hits = strong_hits;
                tied = 0;
            } else if (strong_hits == best_strong_hits && hits == best_hits) {
                tied = 1;
            }
            (void)mask;
        }

        if (exact_bound) {
            int active_matches = 0;
            if (active_signature_count <= 0) {
                active_matches = 1;
            } else if (active_strong_total > 0) {
                active_matches = active_strong_hits > 0;
            } else {
                active_matches =
                    active_hits >= BODY_PROFILE_SIGNATURE_MIN_HITS &&
                    active_hits >= active_signature_count;
            }
            if (active_matches || !body_present) {
                body_profile_runtime_miss_start_tick[person_index] = 0;
                body_profile_signature_ambiguity_logged[person_index] = 0;
                body_profile_signature_ambiguity_hits[person_index] = 0;
                continue;
            }
            best = -1;
            tied = 0;
        }

        if (best >= 0 && !tied) {
            body_profile_runtime_miss_start_tick[person_index] = 0;
            body_profile_signature_ambiguity_logged[person_index] = 0;
            body_profile_signature_ambiguity_hits[person_index] = 0;
            if (body_profile_person_sidecar_active[person_index] &&
                body_profile_person_bind_strength[person_index] < 2) {
                body_profile_clear_person_sidecar_a(
                    person_index, "signature-activation-disabled");
            }
        } else if (tied) {
            body_profile_runtime_miss_start_tick[person_index] = 0;
            if (defaults_cfg.debug &&
                (!body_profile_signature_ambiguity_logged[person_index] ||
                 body_profile_signature_ambiguity_hits[person_index] !=
                     best_hits)) {
                body_profile_signature_ambiguity_logged[person_index] = 1;
                body_profile_signature_ambiguity_hits[person_index] =
                    best_hits;
                log_line("body-profile signature ambiguous person=\"Person%02d\" hits=%d note=\"more than one body sidecar matched live PersonXXBody signatures equally; keeping current/global config\"",
                         person_index + 1, best_hits);
            }
        } else if (body_present &&
                   body_profile_person_sidecar_active[person_index]) {
            body_profile_signature_ambiguity_logged[person_index] = 0;
            body_profile_signature_ambiguity_hits[person_index] = 0;
            if (!body_profile_runtime_miss_start_tick[person_index]) {
                body_profile_runtime_miss_start_tick[person_index] = now;
            } else if (now - body_profile_runtime_miss_start_tick[person_index] >
                       3000u) {
                body_profile_runtime_miss_start_tick[person_index] = 0;
                body_profile_clear_person_sidecar_a(
                    person_index, "runtime-signature-miss");
            }
        } else {
            body_profile_runtime_miss_start_tick[person_index] = 0;
            body_profile_signature_ambiguity_logged[person_index] = 0;
            body_profile_signature_ambiguity_hits[person_index] = 0;
        }
    }
}

static int body_profile_tail_matches_body_slot_a(const char *tail,
                                                 int body_slot)
{
    char body_name[16];
    if (!tail || body_slot < 0 || body_slot >= BODY_PROFILE_BODY_SLOT_COUNT) {
        return 0;
    }
    wsprintfA(body_name, "body%02d", body_slot + 1);
    return contains_i(tail, body_name);
}

static const char *body_profile_tail_leaf_a(const char *tail)
{
    const char *colon;
    if (!tail) return "";
    colon = strrchr(tail, ':');
    return colon ? colon + 1 : tail;
}

static void body_profile_queue_pending_bind_a(int person_index,
                                              int body_slot,
                                              DWORD now,
                                              const char *source)
{
    int i;
    int target = -1;
    int oldest = -1;
    DWORD oldest_tick = 0xffffffffu;
    if (person_index < 0 || person_index >= BODY_PROFILE_PERSON_COUNT ||
        body_slot < 0 || body_slot >= BODY_PROFILE_BODY_SLOT_COUNT) {
        return;
    }
    for (i = 0; i < BODY_PROFILE_PENDING_COUNT; i++) {
        if (!body_profile_pending_bind[i].active && target < 0) {
            target = i;
        }
        if (body_profile_pending_bind[i].active &&
            body_profile_pending_bind[i].tick < oldest_tick) {
            oldest_tick = body_profile_pending_bind[i].tick;
            oldest = i;
        }
    }
    if (target < 0) target = oldest >= 0 ? oldest : 0;
    body_profile_pending_bind[target].active = 1;
    body_profile_pending_bind[target].person_index = person_index;
    body_profile_pending_bind[target].body_slot = body_slot;
    body_profile_pending_bind[target].tick = now;
    if (defaults_cfg.debug) {
        log_line("body-profile pending-bind person=\"Person%02d\" body_type=body%02d source=\"%s\" note=\"next exact loaded bodyXX.bs path for this body type binds the sidecar to this person\"",
                 person_index + 1, body_slot + 1,
                 source ? source : "bodyselect-marker");
    }
}

static int body_profile_bind_pending_open_slot_a(int person_index,
                                                 int body_slot,
                                                 DWORD now,
                                                 const char *reason);

static int body_profile_parse_virtual_body_scene_a(const char *path,
                                                   int *person_out,
                                                   int *body_slot_out)
{
    const char *p;
    if (person_out) *person_out = -1;
    if (body_slot_out) *body_slot_out = -1;
    if (!path ||
        (!contains_i(path, "Shared/Body/") &&
         !contains_i(path, "Shared\\Body\\"))) {
        return 0;
    }
    for (p = path; *p; p++) {
        int body_slot;
        int person_number;
        if (_strnicmp(p, "body0", 5) != 0) continue;
        if (p[5] < '1' || p[5] > '3') continue;
        if (p[6] != '_' || p[7] != '_' || p[8] != '_') continue;
        body_slot = p[5] - '1';
        person_number = atoi(p + 9);
        if (person_number < 1 || person_number > 4) continue;
        if (person_out) *person_out = person_number - 1;
        if (body_slot_out) *body_slot_out = body_slot;
        return 1;
    }
    return 0;
}

static void body_profile_note_virtual_body_scene_a(const char *path,
                                                   const char *source)
{
    int person_index;
    int body_slot;
    if (!body_profile_parse_virtual_body_scene_a(path, &person_index,
                                                 &body_slot)) {
        return;
    }
    body_profile_signature_ambiguity_logged[person_index] = 0;
    body_profile_signature_ambiguity_hits[person_index] = 0;
    body_profile_queue_pending_bind_a(person_index, body_slot, GetTickCount(),
                                      source ? source : "body-scene");
    body_profile_bind_pending_open_slot_a(person_index, body_slot,
                                          GetTickCount(),
                                          "body-file-open");
    log_line("body-profile body-scene person=\"Person%02d\" body=body%02d source=\"%s\" reason=\"%s\" note=\"TK17 virtual body load observed; waiting for exact add-on body path\"",
             person_index + 1, body_slot + 1, path,
             source ? source : "body-scene");
}

static void body_profile_queue_pending_open_a(int body_slot,
                                              const char *body_path,
                                              const char *sidecar_path,
                                              DWORD now)
{
    int i;
    int target = -1;
    int oldest = -1;
    DWORD oldest_tick = 0xffffffffu;
    if (body_slot < 0 || body_slot >= BODY_PROFILE_BODY_SLOT_COUNT ||
        !body_path || !body_path[0] ||
        !sidecar_path || !sidecar_path[0]) {
        return;
    }
    for (i = 0; i < BODY_PROFILE_PENDING_COUNT; i++) {
        if (!body_profile_pending_open[i].active && target < 0) {
            target = i;
        }
        if (body_profile_pending_open[i].active &&
            body_profile_pending_open[i].tick < oldest_tick) {
            oldest_tick = body_profile_pending_open[i].tick;
            oldest = i;
        }
    }
    if (target < 0) target = oldest >= 0 ? oldest : 0;
    body_profile_pending_open[target].active = 1;
    body_profile_pending_open[target].body_slot = body_slot;
    body_profile_pending_open[target].tick = now;
    lstrcpynA(body_profile_pending_open[target].body_path, body_path,
              sizeof(body_profile_pending_open[target].body_path));
    lstrcpynA(body_profile_pending_open[target].sidecar_path, sidecar_path,
              sizeof(body_profile_pending_open[target].sidecar_path));
    log_line("body-profile sidecar pending-open body=\"%s\" body_type=body%02d sidecar=\"%s\" note=\"waiting for next matching PersonXXBody node; never applied globally\"",
             body_path, body_slot + 1, sidecar_path);
}

static int body_profile_bind_pending_open_a(int person_index,
                                            const char *tail,
                                            DWORD now)
{
    int i;
    int best = -1;
    DWORD best_tick = 0xffffffffu;
    if (person_index < 0 || person_index >= BODY_PROFILE_PERSON_COUNT ||
        !tail || !tail[0]) {
        return 0;
    }
    for (i = 0; i < BODY_PROFILE_PENDING_COUNT; i++) {
        body_profile_pending_open_t *pending = &body_profile_pending_open[i];
        if (!pending->active) continue;
        if (now - pending->tick > 3000u) {
            pending->active = 0;
            continue;
        }
        if (!body_profile_tail_matches_body_slot_a(tail, pending->body_slot)) {
            continue;
        }
        if (pending->tick < best_tick) {
            best_tick = pending->tick;
            best = i;
        }
    }
    if (best < 0) return 0;
    body_profile_person_body_hash[person_index] = 0;
    body_profile_activate_sidecar_for_person_a(
        person_index,
        body_profile_pending_open[best].body_slot,
        body_profile_pending_open[best].body_path,
        body_profile_pending_open[best].sidecar_path,
        "body-file-open-tsnode");
    body_profile_pending_open[best].active = 0;
    return 1;
}

static int body_profile_bind_pending_open_slot_a(int person_index,
                                                 int body_slot,
                                                 DWORD now,
                                                 const char *reason)
{
    int i;
    int best = -1;
    int matches = 0;
    DWORD best_tick = 0xffffffffu;
    if (person_index < 0 || person_index >= BODY_PROFILE_PERSON_COUNT ||
        body_slot < 0 || body_slot >= BODY_PROFILE_BODY_SLOT_COUNT) {
        return 0;
    }
    for (i = 0; i < BODY_PROFILE_PENDING_COUNT; i++) {
        body_profile_pending_open_t *pending = &body_profile_pending_open[i];
        if (!pending->active) continue;
        if (now - pending->tick > 3000u) {
            pending->active = 0;
            continue;
        }
        if (pending->body_slot != body_slot) continue;
        matches++;
        if (pending->tick < best_tick) {
            best_tick = pending->tick;
            best = i;
        }
    }
    if (best < 0 || matches != 1) {
        if (matches > 1) {
            log_line("body-profile sidecar pending-open ambiguous person=\"Person%02d\" body_type=body%02d matches=%d note=\"not binding from bodyselect marker because more than one sidecar candidate is pending\"",
                     person_index + 1, body_slot + 1, matches);
        }
        return 0;
    }
    body_profile_person_body_hash[person_index] = 0;
    body_profile_activate_sidecar_for_person_a(
        person_index,
        body_profile_pending_open[best].body_slot,
        body_profile_pending_open[best].body_path,
        body_profile_pending_open[best].sidecar_path,
        reason ? reason : "bodyselect-pending");
    body_profile_pending_open[best].active = 0;
    return 1;
}

static void body_profile_note_tsnode_name_a(const char *name)
{
    int person_index;
    const char *tail;
    if (!body_profile_parse_person_body_tsnode_a(name, &person_index, &tail)) {
        return;
    }
    body_profile_bind_pending_open_a(person_index, tail, GetTickCount());
}

static body_profile_sidecar_entry_t *body_profile_find_sidecar_by_hash(
    int body_slot, DWORD body_hash)
{
    int i;
    if (body_hash == 0) return NULL;
    for (i = 0; i < BODY_PROFILE_SIDECAR_COUNT; i++) {
        body_profile_sidecar_entry_t *entry = &body_profile_sidecars[i];
        if (entry->active && entry->body_slot == body_slot &&
            entry->body_hash == body_hash) {
            return entry;
        }
    }
    return NULL;
}

static int body_profile_parse_first_u32_in_span_a(const char *first,
                                                  const char *last,
                                                  DWORD *value_out);

static int body_profile_parse_bodyselect_hash_file_a(const char *path,
                                                     DWORD *hash_out)
{
    const char *name;
    const char *p;
    const char *end;
    if (hash_out) *hash_out = 0;
    if (!path) return 0;
    name = body_profile_basename_a(path);
    p = strstr(name, "bodyselect_");
    if (!p) return 0;
    p += strlen("bodyselect_");
    end = p;
    while (*end && *end != '.' && *end != '\\' && *end != '/') {
        end++;
    }
    return body_profile_parse_first_u32_in_span_a(p, end, hash_out);
}

static void body_profile_note_bodyselect_hash_file_a(const char *path)
{
    DWORD body_hash;
    int i;
    body_profile_sidecar_entry_t *entry = NULL;
    if (!body_profile_parse_bodyselect_hash_file_a(path, &body_hash)) return;
    for (i = 0; i < BODY_PROFILE_SIDECAR_COUNT; i++) {
        if (body_profile_sidecars[i].active &&
            body_profile_sidecars[i].body_hash == body_hash) {
            entry = &body_profile_sidecars[i];
            break;
        }
    }
    if (!entry) return;
    body_profile_queue_pending_open_a(entry->body_slot, entry->body_path,
                                      entry->sidecar_path, GetTickCount());
    if (defaults_cfg.debug) {
        log_line("body-profile bodyselect sidecar-candidate body_type=body%02d hash=%lu source=\"%s\" sidecar=\"%s\" note=\"waiting for following VAR_personsel marker to bind this sidecar to one person\"",
                 entry->body_slot + 1, (unsigned long)body_hash, path,
                 entry->sidecar_path);
    }
}

static body_profile_sidecar_entry_t *body_profile_find_or_add_sidecar_entry(
    const char *sidecar_path)
{
    int i;
    int free_index = -1;
    if (!sidecar_path || !sidecar_path[0]) return NULL;
    for (i = 0; i < BODY_PROFILE_SIDECAR_COUNT; i++) {
        if (body_profile_sidecars[i].active &&
            _stricmp(body_profile_sidecars[i].sidecar_path, sidecar_path) == 0) {
            return &body_profile_sidecars[i];
        }
        if (!body_profile_sidecars[i].active && free_index < 0) {
            free_index = i;
        }
    }
    if (free_index < 0) return NULL;
    memset(&body_profile_sidecars[free_index], 0,
           sizeof(body_profile_sidecars[free_index]));
    return &body_profile_sidecars[free_index];
}

static void body_profile_activate_sidecar_for_person_a(int person_index,
                                                       int body_slot,
                                                       const char *body_path,
                                                       const char *sidecar_path,
                                                       const char *reason);

static void body_profile_register_sidecar_a(const char *sidecar_path)
{
    int slot;
    char body_path[MAX_PATH * 4];
    char cname[160];
    DWORD hash;
    DWORD attr;
    body_profile_sidecar_entry_t *entry;
    int changed = 0;
    if (!body_profile_sidecar_slot_from_name_a(sidecar_path, &slot)) return;
    attr = GetFileAttributesA(sidecar_path);
    if (attr == INVALID_FILE_ATTRIBUTES || (attr & FILE_ATTRIBUTE_DIRECTORY)) {
        return;
    }
    if (!body_profile_build_body_path_from_sidecar_a(sidecar_path, body_path,
                                                     sizeof(body_path))) {
        return;
    }
    body_profile_tk17_cname_from_sidecar_a(sidecar_path, cname, sizeof(cname));
    if (!cname[0]) return;
    hash = body_profile_xxh32_a(cname);
    if (!hash) return;
    entry = body_profile_find_or_add_sidecar_entry(sidecar_path);
    if (!entry) return;
    if (!entry->active || entry->body_slot != slot ||
        entry->body_hash != hash ||
        _stricmp(entry->body_path, body_path) != 0 ||
        _stricmp(entry->addon_cname, cname) != 0) {
        changed = 1;
    }
    entry->active = 1;
    entry->body_slot = slot;
    entry->body_hash = hash;
    lstrcpynA(entry->addon_cname, cname, sizeof(entry->addon_cname));
    lstrcpynA(entry->body_path, body_path, sizeof(entry->body_path));
    lstrcpynA(entry->sidecar_path, sidecar_path, sizeof(entry->sidecar_path));
    if (changed || entry->signature_count <= 0) {
        body_profile_build_signatures_from_body_a(entry);
    }
    if (changed) {
        log_line("body-profile registry body_type=body%02d hash=%lu signatures=%d addon=\"%s\" body=\"%s\" sidecar=\"%s\" note=\"registered only; exact body-load binding activates sidecars; signatures are diagnostic only\"",
                 slot + 1, (unsigned long)hash, entry->signature_count,
                 entry->addon_cname, entry->body_path, entry->sidecar_path);
    }
}

static int body_profile_path_has_body_select_marker_a(const char *path)
{
    return path &&
           (strstr(path, "VAR_Body___Person") ||
            strstr(path, "VAR_personsel___") ||
            contains_i(path, "bodyselect_") ||
            (contains_i(path, "Shared/Body/body0") &&
             strstr(path, "___")) ||
            (contains_i(path, "Shared\\Body\\body0") &&
             strstr(path, "___")));
}

static int body_profile_path_has_body_select_marker_w(const WCHAR *path)
{
    char path_a[MAX_PATH * 4];
    if (!path) return 0;
    if (!WideCharToMultiByte(CP_ACP, 0, path, -1,
                             path_a, sizeof(path_a), NULL, NULL)) {
        return 0;
    }
    return body_profile_path_has_body_select_marker_a(path_a);
}

static int body_profile_parse_u32_span_a(const char *first,
                                         const char *last,
                                         DWORD *value_out)
{
    DWORD value = 0;
    int digits = 0;
    if (value_out) *value_out = 0;
    if (!first || !last || first >= last) return 0;
    while (first < last && (*first == ' ' || *first == '\t')) first++;
    while (first < last && *first >= '0' && *first <= '9') {
        value = value * 10u + (DWORD)(*first - '0');
        digits++;
        first++;
    }
    while (first < last && (*first == ' ' || *first == '\t')) first++;
    if (!digits || first != last) return 0;
    if (value_out) *value_out = value;
    return 1;
}

static int body_profile_parse_first_u32_in_span_a(const char *first,
                                                  const char *last,
                                                  DWORD *value_out)
{
    while (first && first < last &&
           (*first < '0' || *first > '9')) {
        first++;
    }
    if (!first || first >= last) {
        if (value_out) *value_out = 0;
        return 0;
    }
    while (last > first && (last[-1] < '0' || last[-1] > '9')) {
        last--;
    }
    return body_profile_parse_u32_span_a(first, last, value_out);
}

static int body_profile_parse_body_select_marker_a(const char *path,
                                                   int *person_out,
                                                   int *body_slot_out,
                                                   DWORD *hash_out)
{
    const char *marker;
    const char *person_text;
    const char *bar;
    const char *hash_text;
    const char *bar2;
    const char *bar3;
    DWORD parsed_hash;
    int person_number;
    int gender_number;
    if (person_out) *person_out = -1;
    if (body_slot_out) *body_slot_out = -1;
    if (hash_out) *hash_out = 0;
    if (!path) return 0;
    marker = strstr(path, "VAR_Body___Person");
    if (marker) {
        person_text = marker + strlen("VAR_Body___Person");
        person_number = atoi(person_text);
        bar = strchr(person_text, '|');
        if (!bar) return 0;
        gender_number = atoi(bar + 1);
        hash_text = strchr(bar + 1, '|');
        if (person_number < 1 || person_number > 4 ||
            gender_number < 1 || gender_number > 3) {
            return 0;
        }
        if (person_out) *person_out = person_number - 1;
        if (body_slot_out) *body_slot_out = gender_number - 1;
        if (hash_out && hash_text && hash_text[1]) {
            *hash_out = (DWORD)strtoul(hash_text + 1, NULL, 10);
        }
        return 1;
    }

    /*
       Normal VX body selection uses LUA/VAR_personsel___<kind>|<gender>|<id>|<person>.
       The file is often virtual, so the CreateFile hook must parse it before the
       open result is known.  For add-on body sidecars we only bind direct numeric
       body hashes; non-hash UI ids are logged and left to the PersonXXBody
       fallback so one sidecar cannot bleed into unrelated bodies.
    */
    marker = strstr(path, "VAR_personsel___");
    if (!marker) return 0;
    person_text = marker + strlen("VAR_personsel___");
    bar = strchr(person_text, '|');
    if (!bar) return 0;
    bar2 = strchr(bar + 1, '|');
    if (!bar2) return 0;
    bar3 = strchr(bar2 + 1, '|');
    if (!bar3) return 0;
    gender_number = atoi(bar + 1);
    person_number = atoi(bar3 + 1);
    if (person_number < 1 || person_number > 4 ||
        gender_number < 1 || gender_number > 3) {
        return 0;
    }
    if (person_out) *person_out = person_number - 1;
    if (body_slot_out) *body_slot_out = gender_number - 1;
    if (hash_out &&
        body_profile_parse_first_u32_in_span_a(bar2 + 1, bar3, &parsed_hash)) {
        *hash_out = parsed_hash;
    }
    return 1;
}

static void body_profile_note_body_select_marker_a(const char *path)
{
    int person_index;
    int body_slot;
    DWORD body_hash = 0;
    body_profile_sidecar_entry_t *entry;
    if (!body_profile_parse_body_select_marker_a(path, &person_index,
                                                 &body_slot, &body_hash)) {
        return;
    }
    if (body_hash) {
        entry = body_profile_find_sidecar_by_hash(body_slot, body_hash);
        if (entry) {
            body_profile_queue_pending_bind_a(person_index, body_slot,
                                              GetTickCount(),
                                              "bodyselect-hash");
            body_profile_person_body_hash[person_index] = body_hash;
            body_profile_activate_sidecar_for_person_a(
                person_index, body_slot, entry->body_path,
                entry->sidecar_path, "bodyselect-hash");
            return;
        }
        body_profile_queue_pending_bind_a(person_index, body_slot,
                                          GetTickCount(),
                                          "bodyselect-hash-unmatched");
        log_line("body-profile bodyselect hash-unmatched person=\"Person%02d\" body=body%02d hash=%lu source=\"%s\" note=\"no registered bodyXX.physx.ini matched this TK17 body id; waiting for body file or PersonXXBody fallback\"",
                 person_index + 1, body_slot + 1,
                 (unsigned long)body_hash, path);
    } else {
        body_profile_queue_pending_bind_a(person_index, body_slot,
                                          GetTickCount(),
                                          "bodyselect-marker");
        if (defaults_cfg.debug) {
            log_line("body-profile bodyselect marker person=\"Person%02d\" body=body%02d hash=0 source=\"%s\" note=\"TK17 marker did not carry a direct numeric body hash; waiting for body file or PersonXXBody fallback\"",
                     person_index + 1, body_slot + 1, path);
        }
    }
}

static int body_profile_person_has_exact_sidecar_a(int body_slot,
                                                   const char *body_path,
                                                   const char *sidecar_path)
{
    int i;
    if (!body_path || !sidecar_path) return 0;
    for (i = 0; i < BODY_PROFILE_PERSON_COUNT; i++) {
        if (body_profile_person_sidecar_active[i] &&
            body_profile_person_bind_strength[i] >= 2 &&
            _stricmp(body_profile_person_body_path[i], body_path) == 0 &&
            _stricmp(body_profile_person_sidecar_path[i], sidecar_path) == 0) {
            return 1;
        }
    }
    (void)body_slot;
    return 0;
}

static void body_profile_activate_sidecar_for_person_a(int person_index,
                                                       int body_slot,
                                                       const char *body_path,
                                                       const char *sidecar_path,
                                                       const char *reason)
{
    FILETIME wt;
    int changed = 0;
    if (person_index < 0 || person_index >= 4 ||
        body_slot < 0 || body_slot >= 3 ||
        !body_path || !sidecar_path) return;
    if (_stricmp(body_profile_person_body_path[person_index], body_path) != 0) {
        lstrcpynA(body_profile_person_body_path[person_index], body_path,
                  sizeof(body_profile_person_body_path[person_index]));
        changed = 1;
    }
    if (_stricmp(body_profile_person_sidecar_path[person_index],
                 sidecar_path) != 0 ||
        !body_profile_person_sidecar_active[person_index]) {
        lstrcpynA(body_profile_person_sidecar_path[person_index], sidecar_path,
                  sizeof(body_profile_person_sidecar_path[person_index]));
        body_profile_person_sidecar_active[person_index] = 1;
        changed = 1;
    }
    {
        int strength = body_profile_bind_strength_from_reason_a(reason);
        if (strength > body_profile_person_bind_strength[person_index]) {
            body_profile_person_bind_strength[person_index] = strength;
            changed = 1;
        } else if (!body_profile_person_bind_strength[person_index]) {
            body_profile_person_bind_strength[person_index] = strength;
        }
    }
    if (get_file_write_time_a(sidecar_path, &wt)) {
        body_profile_person_sidecar_write_time[person_index] = wt;
    }
    if (changed) {
        log_line("body-profile sidecar active person=\"Person%02d\" body=\"%s\" body_type=body%02d sidecar=\"%s\" source=\"%s\" note=\"sidecar overrides only this person's effective PhysX config\"",
                 person_index + 1, body_path, body_slot + 1, sidecar_path,
                 reason ? reason : "body-load");
        InterlockedExchange(&body_profile_reload_pending, 1);
    }
}

static void body_profile_note_body_file_a(const char *body_path)
{
    int slot;
    char sidecar[MAX_PATH * 4];
    DWORD attr;
    int sidecar_exists;
    int changed = 0;
    int person_index = -1;
    DWORD now;
    DWORD oldest_match = 0xffffffffu;
    int pending_index = -1;
    int i;
    body_profile_note_virtual_body_scene_a(body_path, "body-scene-path");
    body_profile_note_body_select_marker_a(body_path);
    body_profile_note_bodyselect_hash_file_a(body_path);
    if (!body_path || !body_profile_body_slot_from_name_a(body_path, &slot)) {
        return;
    }
    if (!body_profile_build_sidecar_path_a(body_path, sidecar, sizeof(sidecar))) {
        return;
    }
    attr = GetFileAttributesA(sidecar);
    sidecar_exists = attr != INVALID_FILE_ATTRIBUTES &&
                     !(attr & FILE_ATTRIBUTE_DIRECTORY);
    if (sidecar_exists) {
        body_profile_register_sidecar_a(sidecar);
    }
    now = GetTickCount();
    for (i = 0; i < BODY_PROFILE_PENDING_COUNT; i++) {
        if (!body_profile_pending_bind[i].active) continue;
        if (now - body_profile_pending_bind[i].tick > 8000u) {
            body_profile_pending_bind[i].active = 0;
            continue;
        }
        if (body_profile_pending_bind[i].body_slot == slot &&
            body_profile_pending_bind[i].tick < oldest_match) {
            oldest_match = body_profile_pending_bind[i].tick;
            pending_index = i;
        }
    }
    if (pending_index >= 0) {
        person_index = body_profile_pending_bind[pending_index].person_index;
        body_profile_pending_bind[pending_index].active = 0;
    }
    if (person_index < 0) {
        if (sidecar_exists &&
            !body_profile_person_has_exact_sidecar_a(slot, body_path,
                                                     sidecar)) {
            body_profile_queue_pending_open_a(slot, body_path, sidecar, now);
        }
        return;
    }
    if (sidecar_exists) {
        body_profile_activate_sidecar_for_person_a(person_index, slot,
                                                   body_path, sidecar,
                                                   "body-file-open");
    } else if (body_profile_person_sidecar_active[person_index] ||
               body_profile_person_sidecar_path[person_index][0]) {
        body_profile_person_sidecar_active[person_index] = 0;
        body_profile_person_sidecar_path[person_index][0] = 0;
        body_profile_person_body_path[person_index][0] = 0;
        body_profile_person_body_hash[person_index] = 0;
        body_profile_person_bind_strength[person_index] = 0;
        memset(&body_profile_person_sidecar_write_time[person_index], 0,
               sizeof(body_profile_person_sidecar_write_time[person_index]));
        changed = 1;
        log_line("body-profile sidecar fallback person=\"Person%02d\" body=\"%s\" body_type=body%02d missing=\"%s\" note=\"using global PhysX INI for this person\"",
                 person_index + 1, body_path, slot + 1, sidecar);
    }
    if (changed) {
        InterlockedExchange(&body_profile_reload_pending, 1);
    }
}

static void body_profile_game_log_path_a(const char *filename,
                                         char *out,
                                         size_t outsz)
{
    if (!out || outsz == 0) return;
    out[0] = 0;
    if (self_module) {
        GetModuleFileNameA(self_module, out, (DWORD)outsz);
        {
            char *slash = strrchr(out, '\\');
            if (slash) slash[1] = 0;
        }
        lstrcatA(out, "..\\Logs\\");
    } else {
        lstrcpynA(out, "Logs\\", (int)outsz);
    }
    lstrcatA(out, filename);
}

static void body_profile_normalize_path_a(char *path)
{
    char *p;
    if (!path) return;
    for (p = path; *p; p++) {
        if (*p == '/') *p = '\\';
    }
}

static void body_profile_build_game_path_a(const char *rel,
                                           char *out,
                                           size_t outsz)
{
    DWORD n;
    char candidate[MAX_PATH * 4];
    if (!out || outsz == 0) return;
    out[0] = 0;
    if (!rel || !rel[0]) return;
    if ((rel[0] && rel[1] == ':') ||
        (rel[0] == '\\' && rel[1] == '\\')) {
        n = GetFullPathNameA(rel, (DWORD)outsz, out, NULL);
        if (n == 0 || n >= outsz) lstrcpynA(out, rel, (int)outsz);
        body_profile_normalize_path_a(out);
        return;
    }
    candidate[0] = 0;
    if (self_module) {
        GetModuleFileNameA(self_module, candidate, sizeof(candidate));
        {
            char *slash = strrchr(candidate, '\\');
            if (slash) slash[1] = 0;
        }
        lstrcatA(candidate, "..\\");
        lstrcatA(candidate, rel);
        n = GetFullPathNameA(candidate, (DWORD)outsz, out, NULL);
        if (n == 0 || n >= outsz) lstrcpynA(out, candidate, (int)outsz);
    } else {
        n = GetFullPathNameA(rel, (DWORD)outsz, out, NULL);
        if (n == 0 || n >= outsz) lstrcpynA(out, rel, (int)outsz);
    }
    body_profile_normalize_path_a(out);
}

static void body_profile_handle_goodbye_log_line_a(const char *line)
{
    const char *start;
    const char *end;
    char path[256];
    size_t len;
    if (!line || !contains_i(line, "Execute3")) return;
    start = strchr(line, '\'');
    if (!start) return;
    start++;
    end = strchr(start, '\'');
    if (!end || end <= start) return;
    len = (size_t)(end - start);
    if (len >= sizeof(path)) len = sizeof(path) - 1;
    memcpy(path, start, len);
    path[len] = 0;
    body_profile_note_virtual_body_scene_a(path, "tk17-execute3-log");
}

static void body_profile_handle_vx_addon_log_line_a(const char *line)
{
    const char *marker;
    const char *path_start;
    char rel[MAX_PATH * 4];
    char full[MAX_PATH * 4];
    DWORD n;
    char *end;
    int slot;
    if (!line) return;
    marker = strstr(line, "addon-bs:");
    if (!marker) marker = strstr(line, "addon-bs-cached:");
    if (!marker) return;
    path_start = strchr(marker, ':');
    if (!path_start) return;
    path_start++;
    while (*path_start == ' ' || *path_start == '\t') path_start++;
    if (!contains_i(path_start, "Addons/") &&
        !contains_i(path_start, "Addons\\")) {
        return;
    }
    if (!contains_i(path_start, "Scenes/Shared/Body/") &&
        !contains_i(path_start, "Scenes\\Shared\\Body\\")) {
        return;
    }
    lstrcpynA(rel, path_start, sizeof(rel));
    end = rel + strlen(rel);
    while (end > rel &&
           (end[-1] == '\r' || end[-1] == '\n' ||
            end[-1] == ' ' || end[-1] == '\t')) {
        *--end = 0;
    }
    body_profile_normalize_path_a(rel);
    if (!body_profile_body_slot_from_name_a(rel, &slot)) return;
    (void)n;
    body_profile_build_game_path_a(rel, full, sizeof(full));
    body_profile_note_body_file_a(full);
}

static void body_profile_observe_log_file_a(body_profile_log_observer_t *obs,
                                            const char *path,
                                            void (*handle_line)(const char*))
{
    HANDLE h;
    WIN32_FILE_ATTRIBUTE_DATA metadata;
    DWORD size;
    DWORD read_bytes;
    DWORD loops = 0;
    if (!obs || !path || !handle_line) return;
    if (!GetFileAttributesExA(path, GetFileExInfoStandard, &metadata)) return;
    if (obs->initialized && obs->metadata_valid &&
        metadata.nFileSizeHigh == 0 &&
        metadata.nFileSizeLow == obs->offset &&
        metadata.nFileSizeLow == obs->observed_size &&
        CompareFileTime(&metadata.ftLastWriteTime,
                        &obs->observed_write_time) == 0) {
        return;
    }
    h = CreateFileA(path, GENERIC_READ,
                    FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE,
                    NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
    if (h == INVALID_HANDLE_VALUE) return;
    size = GetFileSize(h, NULL);
    if (size == INVALID_FILE_SIZE) {
        CloseHandle(h);
        return;
    }
    if (!obs->initialized) {
        obs->initialized = 1;
        obs->offset = 0;
        obs->partial_len = 0;
    } else if (obs->offset > size) {
        obs->initialized = 1;
        obs->offset = size;
        obs->partial_len = 0;
    }
    if (obs->offset < size) {
        SetLastError(NO_ERROR);
        if (SetFilePointer(h, (LONG)obs->offset, NULL, FILE_BEGIN) ==
                INVALID_SET_FILE_POINTER &&
            GetLastError() != NO_ERROR) {
            CloseHandle(h);
            return;
        }
    }
    while (obs->offset < size && loops++ < 512) {
        char chunk[32768];
        char text[sizeof(obs->partial) + sizeof(chunk) + 1];
        char *cursor;
        char *last;
        DWORD want = size - obs->offset;
        if (want > sizeof(chunk)) want = sizeof(chunk);
        if (!ReadFile(h, chunk, want, &read_bytes, NULL) ||
            read_bytes == 0) {
            break;
        }
        obs->offset += read_bytes;
        memcpy(text, obs->partial, obs->partial_len);
        memcpy(text + obs->partial_len, chunk, read_bytes);
        text[obs->partial_len + read_bytes] = 0;
        obs->partial_len = 0;
        cursor = text;
        last = text;
        while (*cursor) {
            if (*cursor == '\n') {
                *cursor = 0;
                handle_line(last);
                last = cursor + 1;
            }
            cursor++;
        }
        if (*last) {
            size_t remaining = strlen(last);
            if (remaining >= sizeof(obs->partial)) {
                remaining = sizeof(obs->partial) - 1;
            }
            memcpy(obs->partial, last, remaining);
            obs->partial[remaining] = 0;
            obs->partial_len = (int)remaining;
        }
    }
    CloseHandle(h);
    obs->observed_size = size;
    obs->observed_write_time = metadata.ftLastWriteTime;
    obs->metadata_valid = 1;
}

static void body_profile_observe_tk17_body_logs(DWORD now)
{
    static DWORD last_observe_tick;
    static body_profile_log_observer_t goodbye_obs;
    static body_profile_log_observer_t addon_obs;
    char path[MAX_PATH * 4];
    if (last_observe_tick && now - last_observe_tick < 250u) return;
    last_observe_tick = now;
    body_profile_game_log_path_a("good-bye-txx.log", path, sizeof(path));
    body_profile_observe_log_file_a(&goodbye_obs, path,
                                    body_profile_handle_goodbye_log_line_a);
    body_profile_game_log_path_a("VX-addon.log", path, sizeof(path));
    body_profile_observe_log_file_a(&addon_obs, path,
                                    body_profile_handle_vx_addon_log_line_a);
}

static void body_profile_note_body_file_w(const WCHAR *body_path)
{
    char path[MAX_PATH * 4];
    if (!body_path) return;
    if (!WideCharToMultiByte(CP_ACP, 0, body_path, -1,
                             path, sizeof(path), NULL, NULL)) {
        return;
    }
    body_profile_note_body_file_a(path);
}

static void migrate_legacy_penis_physics_section(void)
{
    static const char missing[] = "\x1fmissing\x1f";
    static const char *user_keys[] = {
        "enabled",
        "enabled_person01", "enabled_person02",
        "enabled_person03", "enabled_person04",
        "translation_horizontal_scale", "translation_vertical_scale",
        "translation_depth_scale", "rotation_horizontal_scale",
        "rotation_vertical_scale", "rotation_twist_scale",
        "gravity_curve", "gravity_horizontal_curve",
        "gravity_vertical_curve",
        "stiffness", "damping",
        "joint01_max_angle", "joint02_max_angle", "joint03_max_angle",
        "joint01_gain", "joint02_gain", "joint03_gain",
        "interval_ms"
    };
    char value[32];
    char *section_values;
    DWORD count;
    size_t i;
    GetPrivateProfileStringA(PENIS_PHYSICS_CONFIG_SECTION, "enabled", missing,
                             value, sizeof(value), config_path);
    if (strcmp(value, missing) != 0) return;
    GetPrivateProfileStringA(PENIS_PHYSICS_LEGACY_CONFIG_SECTION, "enabled", missing,
                             value, sizeof(value), config_path);
    if (strcmp(value, missing) == 0) return;
    section_values = (char*)malloc(32768);
    if (!section_values) return;
    count = GetPrivateProfileSectionA(PENIS_PHYSICS_LEGACY_CONFIG_SECTION,
                                      section_values, 32768, config_path);
    if (count > 0 && count < 32766 &&
        WritePrivateProfileSectionA(PENIS_PHYSICS_INTERNAL_CONFIG_SECTION,
                                    section_values, config_path)) {
        for (i = 0; i < sizeof(user_keys) / sizeof(user_keys[0]); i++) {
            GetPrivateProfileStringA(PENIS_PHYSICS_LEGACY_CONFIG_SECTION,
                                     user_keys[i], missing,
                                     value, sizeof(value), config_path);
            if (strcmp(value, missing) != 0) {
                WritePrivateProfileStringA(PENIS_PHYSICS_CONFIG_SECTION,
                                           user_keys[i], value, config_path);
            }
        }
        log_line("settings migrated legacy INI section [%s] to [%s] and [%s]",
                 PENIS_PHYSICS_LEGACY_CONFIG_SECTION,
                 PENIS_PHYSICS_CONFIG_SECTION,
                 PENIS_PHYSICS_INTERNAL_CONFIG_SECTION);
    }
    free(section_values);
}

static float profile_float(const char *section, const char *key,
                           float fallback, const char *path);
static int profile_bool(const char *section, const char *key,
                        int fallback, const char *path);
static int parse_axis_name(const char *axis);
static int profile_axis(const char *section, const char *key,
                        int fallback, const char *path);
static int profile_translation_channel(const char *section, const char *key,
                                       int fallback, const char *path);
static int profile_body_chain_collision_scope(const char *section,
                                               int fallback,
                                               const char *path);
static void profile_vec3_or_float(const char *section, const char *key,
                                  float scalar_fallback,
                                  float out[3], const char *path);
static void profile_min_angle_vec3_or_float(const char *section,
                                            const char *key,
                                            const float max_angle[3],
                                            float out[3],
                                            const char *path);
static void clamp_angle_range_vec3(float min_angle[3], float max_angle[3]);
static void parse_vec3(const char *s, float out[3]);

static void body_profile_clamp_current_physics_config(
    body_chain_physics_config_t *cfg, int link_count)
{
    int i, axis;
    if (!cfg) return;
    cfg->wind_scale = physx_clampf(cfg->wind_scale, 0.0f, 10.0f);
    if (cfg->interval_ms < 16) cfg->interval_ms = 16;
    if (cfg->interval_ms > 1000) cfg->interval_ms = 1000;
    cfg->horizontal_drive_scale =
        physx_clampf(cfg->horizontal_drive_scale, -5000.0f, 5000.0f);
    cfg->vertical_drive_scale =
        physx_clampf(cfg->vertical_drive_scale, -5000.0f, 5000.0f);
    cfg->gravity_horizontal_curve =
        physx_clampf(cfg->gravity_horizontal_curve, 0.1f, 8.0f);
    cfg->gravity_vertical_curve =
        physx_clampf(cfg->gravity_vertical_curve, 0.1f, 8.0f);
    cfg->stiffness = physx_clampf(cfg->stiffness, 1.0f, 200.0f);
    cfg->damping = physx_clampf(cfg->damping, 0.0f, 80.0f);
    for (axis = 0; axis < 3; axis++) {
        cfg->translation_scale[axis] =
            physx_clampf(cfg->translation_scale[axis], -10.0f, 10.0f);
        cfg->rotation_scale[axis] =
            physx_clampf(cfg->rotation_scale[axis], -10.0f, 10.0f);
    }
    for (i = 0; i < link_count && i < 3; i++) {
        clamp_angle_range_vec3(cfg->link_min_angle[i],
                               cfg->link_max_angle[i]);
    }
}

static int body_profile_bool_if_present(const char *section,
                                        const char *key,
                                        const char *path,
                                        int *out)
{
    char buf[64];
    if (!out ||
        !profile_string_found(section, key, buf, sizeof(buf), path)) {
        return 0;
    }
    trim_in_place(buf);
    if (_stricmp(buf, "1") == 0 || _stricmp(buf, "true") == 0 ||
        _stricmp(buf, "yes") == 0 || _stricmp(buf, "on") == 0) {
        *out = 1;
    } else if (_stricmp(buf, "0") == 0 || _stricmp(buf, "false") == 0 ||
               _stricmp(buf, "no") == 0 || _stricmp(buf, "off") == 0) {
        *out = 0;
    } else {
        *out = atoi(buf) ? 1 : 0;
    }
    return 1;
}

static void body_profile_overlay_paired_physics_section(
    const char *section, const char *path,
    body_chain_physics_config_t *cfg, int breasts)
{
    static const char *translation_keys[3] = {
        "translation_horizontal_scale",
        "translation_vertical_scale",
        "translation_depth_scale"
    };
    static const char *bone_translation_keys[3] = {
        "bone_translation_horizontal_scale",
        "bone_translation_vertical_scale",
        "bone_translation_depth_scale"
    };
    static const char *rotation_keys[3] = {
        "rotation_horizontal_scale",
        "rotation_vertical_scale",
        "rotation_twist_scale"
    };
    char value[64];
    int channel;
    int enabled;
    int max_angle_present;
    int min_angle_present;
    if (!section || !path || !path[0] || !cfg) return;

    enabled = cfg->enabled;
    if (body_profile_bool_if_present(section, "enabled", path,
                                     &enabled)) {
        cfg->enabled = enabled;
    }
    cfg->wind_enabled = profile_bool(
        section, "wind_enabled", cfg->wind_enabled, path);
    cfg->wind_scale = profile_float(
        section, "wind_scale", cfg->wind_scale, path);
    cfg->collision_scope = profile_body_chain_collision_scope(
        section, cfg->collision_scope, path);
    cfg->room_collision_enabled = profile_bool(
        section, "room_collision_enabled",
        cfg->room_collision_enabled, path);
    for (channel = 0; channel < 3; channel++) {
        cfg->translation_scale[channel] = profile_float(
            section, translation_keys[channel],
            cfg->translation_scale[channel], path);
        cfg->bone_translation_scale[channel] = profile_float(
            section, bone_translation_keys[channel],
            cfg->bone_translation_scale[channel], path);
        cfg->rotation_scale[channel] = profile_float(
            section, rotation_keys[channel],
            cfg->rotation_scale[channel], path);
    }
    cfg->bone_translation_enabled = profile_bool(
        section, "bone_translation_enabled",
        cfg->bone_translation_enabled, path);
    if (profile_string_found(section, "bone_translation_space", value,
                             sizeof(value), path)) {
        trim_in_place(value);
        if (_stricmp(value, "body") == 0) {
            cfg->bone_translation_space = 1;
        } else if (_stricmp(value, "local") == 0) {
            cfg->bone_translation_space = 0;
        }
    }
    cfg->bone_translation_stiffness = profile_float(
        section, "bone_translation_stiffness",
        cfg->bone_translation_stiffness, path);
    cfg->bone_translation_damping = profile_float(
        section, "bone_translation_damping",
        cfg->bone_translation_damping, path);
    if (profile_string_found(section, "bone_translation_max_offset",
                             value, sizeof(value), path)) {
        profile_vec3_or_float(section, "bone_translation_max_offset",
                              cfg->bone_translation_max_offset[0],
                              cfg->bone_translation_max_offset, path);
    }
    if (breasts) {
        cfg->bone_translation_gravity_sag = profile_float(
            section, "bone_translation_gravity_sag",
            cfg->bone_translation_gravity_sag, path);
        cfg->gravity_inward_strength = profile_float(
            section, "gravity_inward_outward_strength",
            cfg->gravity_inward_strength, path);
        cfg->gravity_outward_strength = profile_float(
            section, "gravity_inward_outward_strength",
            cfg->gravity_outward_strength, path);
        cfg->gravity_inward_strength = profile_float(
            section, "gravity_inward_strength",
            cfg->gravity_inward_strength, path);
        cfg->gravity_outward_strength = profile_float(
            section, "gravity_outward_strength",
            cfg->gravity_outward_strength, path);
    }
    cfg->gravity_horizontal_curve = profile_float(
        section, "gravity_horizontal_curve",
        cfg->gravity_horizontal_curve, path);
    cfg->gravity_vertical_curve = profile_float(
        section, "gravity_vertical_curve",
        cfg->gravity_vertical_curve, path);
    cfg->gravity_angle = profile_float(
        section, "gravity_strength", cfg->gravity_angle, path);
    cfg->stiffness = profile_float(
        section, "stiffness", cfg->stiffness, path);
    cfg->damping = profile_float(
        section, "damping", cfg->damping, path);
    max_angle_present = profile_string_found(
        section, "joint01_max_angle", value, sizeof(value), path);
    min_angle_present = profile_string_found(
        section, "joint01_min_angle", value, sizeof(value), path);
    if (max_angle_present) {
        profile_vec3_or_float(section, "joint01_max_angle", cfg->max_angle,
                              cfg->link_max_angle[0], path);
    }
    if (max_angle_present || min_angle_present) {
        profile_min_angle_vec3_or_float(
            section, "joint01_min_angle", cfg->link_max_angle[0],
            cfg->link_min_angle[0], path);
    }
    cfg->link_gain[0] = profile_float(
        section, "joint01_gain", cfg->link_gain[0], path);
    cfg->interval_ms = GetPrivateProfileIntA(
        section, "interval_ms", cfg->interval_ms, path);

    body_profile_clamp_current_physics_config(cfg, 1);
    cfg->bone_translation_stiffness = physx_clampf(
        cfg->bone_translation_stiffness, 1.0f, 200.0f);
    cfg->bone_translation_damping = physx_clampf(
        cfg->bone_translation_damping, 0.0f, 80.0f);
    cfg->bone_translation_gravity_sag = physx_clampf(
        cfg->bone_translation_gravity_sag, 0.0f, 1.0f);
    cfg->gravity_inward_strength = physx_clampf(
        cfg->gravity_inward_strength, 0.0f, 45.0f);
    cfg->gravity_outward_strength = physx_clampf(
        cfg->gravity_outward_strength, 0.0f, 45.0f);
    for (channel = 0; channel < 3; channel++) {
        cfg->bone_translation_scale[channel] = physx_clampf(
            cfg->bone_translation_scale[channel], -10.0f, 10.0f);
        cfg->bone_translation_max_offset[channel] = physx_clampf(
            cfg->bone_translation_max_offset[channel], 0.0f, 0.100f);
    }
}

static void body_profile_overlay_body_physics_sections(int person_index,
                                                       const char *path)
{
    int breasts_enabled;
    int penis_enabled;
    int penis_enabled_present = 0;
    int testicle_enabled;
    int testicle_enabled_present = 0;

    if (!path || !path[0]) return;

    body_profile_overlay_paired_physics_section(
        BREASTS_PHYSICS_CONFIG_SECTION, path,
        &breasts_physics_person_cfg[person_index], 1);
    breasts_enabled = breasts_physics_person_cfg[person_index].enabled;
    if (!breasts_enabled) {
        body_chain_collider_cfg.breasts_collision_enabled = 0;
    }

    penis_enabled = body_chain_physics_cfg.enabled;
    if (body_profile_bool_if_present(PENIS_PHYSICS_LEGACY_CONFIG_SECTION,
                                     "enabled", path, &penis_enabled)) {
        penis_enabled_present = 1;
    }
    if (body_profile_bool_if_present(PENIS_PHYSICS_CONFIG_SECTION,
                                     "enabled", path, &penis_enabled)) {
        penis_enabled_present = 1;
    }
    if (penis_enabled_present) {
        body_chain_physics_cfg.enabled = penis_enabled;
        if (!penis_enabled) {
            if (person_index >= 0 && person_index < 4) {
                body_chain_physics_cfg.enabled_person[person_index] = 0;
            }
            body_chain_collider_cfg.penis_collision_enabled = 0;
        }
    }
    body_chain_physics_cfg.wind_enabled = profile_bool(
        PENIS_PHYSICS_CONFIG_SECTION, "wind_enabled",
        body_chain_physics_cfg.wind_enabled, path);
    body_chain_physics_cfg.wind_scale = profile_float(
        PENIS_PHYSICS_CONFIG_SECTION, "wind_scale",
        body_chain_physics_cfg.wind_scale, path);
    body_chain_physics_cfg.collision_scope =
        profile_body_chain_collision_scope(
            PENIS_PHYSICS_CONFIG_SECTION,
            body_chain_physics_cfg.collision_scope, path);
    body_chain_physics_cfg.room_collision_enabled = profile_bool(
        PENIS_PHYSICS_CONFIG_SECTION, "room_collision_enabled",
        body_chain_physics_cfg.room_collision_enabled, path);

    body_chain_physics_cfg.translation_scale[0] =
        profile_float(PENIS_PHYSICS_CONFIG_SECTION,
                      "translation_horizontal_scale",
                      body_chain_physics_cfg.translation_scale[0], path);
    body_chain_physics_cfg.translation_scale[1] =
        profile_float(PENIS_PHYSICS_CONFIG_SECTION,
                      "translation_vertical_scale",
                      body_chain_physics_cfg.translation_scale[1], path);
    body_chain_physics_cfg.translation_scale[2] =
        profile_float(PENIS_PHYSICS_CONFIG_SECTION,
                      "translation_depth_scale",
                      body_chain_physics_cfg.translation_scale[2], path);
    body_chain_physics_cfg.face_down_translation_channel =
        profile_translation_channel(
            PENIS_PHYSICS_CONFIG_SECTION,
            "face_down_translation_channel",
            body_chain_physics_cfg.face_down_translation_channel, path);
    body_chain_physics_cfg.face_down_translation_sign =
        physx_clampf(
            profile_float(PENIS_PHYSICS_CONFIG_SECTION,
                          "face_down_translation_sign",
                          body_chain_physics_cfg.face_down_translation_sign,
                          path),
            -1.0f, 1.0f);
    body_chain_physics_cfg.rotation_scale[0] =
        profile_float(PENIS_PHYSICS_CONFIG_SECTION,
                      "rotation_horizontal_scale",
                      body_chain_physics_cfg.rotation_scale[0], path);
    body_chain_physics_cfg.rotation_scale[1] =
        profile_float(PENIS_PHYSICS_CONFIG_SECTION,
                      "rotation_vertical_scale",
                      body_chain_physics_cfg.rotation_scale[1], path);
    body_chain_physics_cfg.rotation_scale[2] =
        profile_float(PENIS_PHYSICS_CONFIG_SECTION,
                      "rotation_twist_scale",
                      body_chain_physics_cfg.rotation_scale[2], path);
    body_chain_physics_cfg.stiffness =
        profile_float(PENIS_PHYSICS_CONFIG_SECTION, "stiffness",
                      body_chain_physics_cfg.stiffness, path);
    body_chain_physics_cfg.damping =
        profile_float(PENIS_PHYSICS_CONFIG_SECTION, "damping",
                      body_chain_physics_cfg.damping, path);
    body_chain_physics_cfg.gravity_angle =
        profile_float(PENIS_PHYSICS_CONFIG_SECTION, "gravity_angle",
                      body_chain_physics_cfg.gravity_angle, path);
    body_chain_physics_cfg.gravity_horizontal_curve =
        profile_float(PENIS_PHYSICS_CONFIG_SECTION,
                      "gravity_horizontal_curve",
                      profile_float(PENIS_PHYSICS_CONFIG_SECTION,
                                    "gravity_curve",
                                    body_chain_physics_cfg
                                        .gravity_horizontal_curve,
                                    path),
                      path);
    body_chain_physics_cfg.gravity_vertical_curve =
        profile_float(PENIS_PHYSICS_CONFIG_SECTION,
                      "gravity_vertical_curve",
                      profile_float(PENIS_PHYSICS_CONFIG_SECTION,
                                    "gravity_curve",
                                    body_chain_physics_cfg
                                        .gravity_vertical_curve,
                                    path),
                      path);
    body_chain_physics_cfg.max_angle =
        profile_float(PENIS_PHYSICS_CONFIG_SECTION, "max_angle",
                      body_chain_physics_cfg.max_angle, path);
    profile_vec3_or_float(PENIS_PHYSICS_CONFIG_SECTION, "joint01_max_angle",
                          body_chain_physics_cfg.max_angle,
                          body_chain_physics_cfg.link_max_angle[0], path);
    profile_min_angle_vec3_or_float(PENIS_PHYSICS_CONFIG_SECTION,
                                    "joint01_min_angle",
                                    body_chain_physics_cfg.link_max_angle[0],
                                    body_chain_physics_cfg.link_min_angle[0],
                                    path);
    profile_vec3_or_float(PENIS_PHYSICS_CONFIG_SECTION, "joint02_max_angle",
                          body_chain_physics_cfg.max_angle,
                          body_chain_physics_cfg.link_max_angle[1], path);
    profile_min_angle_vec3_or_float(PENIS_PHYSICS_CONFIG_SECTION,
                                    "joint02_min_angle",
                                    body_chain_physics_cfg.link_max_angle[1],
                                    body_chain_physics_cfg.link_min_angle[1],
                                    path);
    profile_vec3_or_float(PENIS_PHYSICS_CONFIG_SECTION, "joint03_max_angle",
                          body_chain_physics_cfg.max_angle,
                          body_chain_physics_cfg.link_max_angle[2], path);
    profile_min_angle_vec3_or_float(PENIS_PHYSICS_CONFIG_SECTION,
                                    "joint03_min_angle",
                                    body_chain_physics_cfg.link_max_angle[2],
                                    body_chain_physics_cfg.link_min_angle[2],
                                    path);
    body_chain_physics_cfg.link_gain[0] =
        profile_float(PENIS_PHYSICS_CONFIG_SECTION, "joint01_gain",
                      body_chain_physics_cfg.link_gain[0], path);
    body_chain_physics_cfg.link_gain[1] =
        profile_float(PENIS_PHYSICS_CONFIG_SECTION, "joint02_gain",
                      body_chain_physics_cfg.link_gain[1], path);
    body_chain_physics_cfg.link_gain[2] =
        profile_float(PENIS_PHYSICS_CONFIG_SECTION, "joint03_gain",
                      body_chain_physics_cfg.link_gain[2], path);
    body_chain_physics_cfg.interval_ms =
        GetPrivateProfileIntA(PENIS_PHYSICS_CONFIG_SECTION, "interval_ms",
                              body_chain_physics_cfg.interval_ms, path);
    body_profile_clamp_current_physics_config(&body_chain_physics_cfg, 3);

    testicle_enabled = testicle_physics_cfg.enabled;
    if (body_profile_bool_if_present(TESTICLE_PHYSICS_CONFIG_SECTION,
                                     "enabled", path, &testicle_enabled)) {
        testicle_enabled_present = 1;
    }
    if (testicle_enabled_present) {
        testicle_physics_cfg.enabled = testicle_enabled;
        if (!testicle_enabled) {
            if (person_index >= 0 && person_index < 4) {
                testicle_physics_cfg.enabled_person[person_index] = 0;
            }
            body_chain_collider_cfg.testicle_collision_enabled = 0;
            body_chain_collider_cfg.live_testicle_bones = 0;
        }
    }
    testicle_physics_cfg.wind_enabled = profile_bool(
        TESTICLE_PHYSICS_CONFIG_SECTION, "wind_enabled",
        testicle_physics_cfg.wind_enabled, path);
    testicle_physics_cfg.wind_scale = profile_float(
        TESTICLE_PHYSICS_CONFIG_SECTION, "wind_scale",
        testicle_physics_cfg.wind_scale, path);
    testicle_physics_cfg.collision_scope =
        profile_body_chain_collision_scope(
            TESTICLE_PHYSICS_CONFIG_SECTION,
            testicle_physics_cfg.collision_scope, path);
    testicle_physics_cfg.room_collision_enabled = profile_bool(
        TESTICLE_PHYSICS_CONFIG_SECTION, "room_collision_enabled",
        testicle_physics_cfg.room_collision_enabled, path);

    testicle_physics_cfg.translation_scale[0] =
        profile_float(TESTICLE_PHYSICS_CONFIG_SECTION,
                      "translation_horizontal_scale",
                      testicle_physics_cfg.translation_scale[0], path);
    testicle_physics_cfg.translation_scale[1] =
        profile_float(TESTICLE_PHYSICS_CONFIG_SECTION,
                      "translation_vertical_scale",
                      testicle_physics_cfg.translation_scale[1], path);
    testicle_physics_cfg.translation_scale[2] =
        profile_float(TESTICLE_PHYSICS_CONFIG_SECTION,
                      "translation_depth_scale",
                      testicle_physics_cfg.translation_scale[2], path);
    testicle_physics_cfg.face_down_translation_channel =
        profile_translation_channel(
            TESTICLE_PHYSICS_CONFIG_SECTION,
            "face_down_translation_channel",
            testicle_physics_cfg.face_down_translation_channel, path);
    testicle_physics_cfg.face_down_translation_sign =
        physx_clampf(
            profile_float(TESTICLE_PHYSICS_CONFIG_SECTION,
                          "face_down_translation_sign",
                          testicle_physics_cfg.face_down_translation_sign,
                          path),
            -1.0f, 1.0f);
    testicle_physics_cfg.rotation_scale[0] =
        profile_float(TESTICLE_PHYSICS_CONFIG_SECTION,
                      "rotation_horizontal_scale",
                      testicle_physics_cfg.rotation_scale[0], path);
    testicle_physics_cfg.rotation_scale[1] =
        profile_float(TESTICLE_PHYSICS_CONFIG_SECTION,
                      "rotation_vertical_scale",
                      testicle_physics_cfg.rotation_scale[1], path);
    testicle_physics_cfg.rotation_scale[2] =
        profile_float(TESTICLE_PHYSICS_CONFIG_SECTION,
                      "rotation_twist_scale",
                      testicle_physics_cfg.rotation_scale[2], path);
    testicle_physics_cfg.stiffness =
        profile_float(TESTICLE_PHYSICS_CONFIG_SECTION, "stiffness",
                      testicle_physics_cfg.stiffness, path);
    testicle_physics_cfg.damping =
        profile_float(TESTICLE_PHYSICS_CONFIG_SECTION, "damping",
                      testicle_physics_cfg.damping, path);
    testicle_physics_cfg.gravity_angle =
        profile_float(TESTICLE_PHYSICS_CONFIG_SECTION, "gravity_angle",
                      testicle_physics_cfg.gravity_angle, path);
    testicle_physics_cfg.gravity_horizontal_curve =
        profile_float(TESTICLE_PHYSICS_CONFIG_SECTION,
                      "gravity_horizontal_curve",
                      profile_float(TESTICLE_PHYSICS_CONFIG_SECTION,
                                    "gravity_curve",
                                    testicle_physics_cfg
                                        .gravity_horizontal_curve,
                                    path),
                      path);
    testicle_physics_cfg.gravity_vertical_curve =
        profile_float(TESTICLE_PHYSICS_CONFIG_SECTION,
                      "gravity_vertical_curve",
                      profile_float(TESTICLE_PHYSICS_CONFIG_SECTION,
                                    "gravity_curve",
                                    testicle_physics_cfg
                                        .gravity_vertical_curve,
                                    path),
                      path);
    profile_vec3_or_float(TESTICLE_PHYSICS_CONFIG_SECTION,
                          "joint01_max_angle",
                          testicle_physics_cfg.max_angle,
                          testicle_physics_cfg.link_max_angle[0], path);
    profile_min_angle_vec3_or_float(TESTICLE_PHYSICS_CONFIG_SECTION,
                                    "joint01_min_angle",
                                    testicle_physics_cfg.link_max_angle[0],
                                    testicle_physics_cfg.link_min_angle[0],
                                    path);
    profile_vec3_or_float(TESTICLE_PHYSICS_CONFIG_SECTION,
                          "joint02_max_angle",
                          testicle_physics_cfg.max_angle,
                          testicle_physics_cfg.link_max_angle[1], path);
    profile_min_angle_vec3_or_float(TESTICLE_PHYSICS_CONFIG_SECTION,
                                    "joint02_min_angle",
                                    testicle_physics_cfg.link_max_angle[1],
                                    testicle_physics_cfg.link_min_angle[1],
                                    path);
    testicle_physics_cfg.link_gain[0] =
        profile_float(TESTICLE_PHYSICS_CONFIG_SECTION, "joint01_gain",
                      testicle_physics_cfg.link_gain[0], path);
    testicle_physics_cfg.link_gain[1] =
        profile_float(TESTICLE_PHYSICS_CONFIG_SECTION, "joint02_gain",
                      testicle_physics_cfg.link_gain[1], path);
    testicle_physics_cfg.interval_ms =
        GetPrivateProfileIntA(TESTICLE_PHYSICS_CONFIG_SECTION,
                              "interval_ms",
                              testicle_physics_cfg.interval_ms, path);
    body_profile_clamp_current_physics_config(&testicle_physics_cfg, 2);

    body_profile_overlay_paired_physics_section(
        BUTT_PHYSICS_CONFIG_SECTION, path,
        &butt_physics_person_cfg[person_index], 0);
    if (!butt_physics_person_cfg[person_index].enabled) {
        body_chain_collider_cfg.butt_collision_enabled = 0;
    }
}

static void body_profile_overlay_body_collider_section(const char *path)
{
    char buf[512];
    int i;
    if (!path || !path[0]) return;

    body_colliders_profile_string("testicles01_fine_offset", "",
                                  buf, sizeof(buf), path);
    parse_vec3(buf, body_chain_collider_cfg.testicle_fine_offset[0]);
    body_colliders_profile_string("testicles02_fine_offset", "",
                                  buf, sizeof(buf), path);
    parse_vec3(buf, body_chain_collider_cfg.testicle_fine_offset[1]);
    body_colliders_profile_string_alias("spine01_fine_offset",
                                        "stomach01_fine_offset", "",
                                        buf, sizeof(buf), path);
    parse_vec3(buf, body_chain_collider_cfg.stomach_fine_offset[0]);
    body_colliders_profile_string_alias("spine02_fine_offset",
                                        "stomach02_fine_offset", "",
                                        buf, sizeof(buf), path);
    parse_vec3(buf, body_chain_collider_cfg.stomach_fine_offset[1]);
    body_colliders_profile_string_alias("spine03_fine_offset",
                                        "stomach03_fine_offset", "",
                                        buf, sizeof(buf), path);
    parse_vec3(buf, body_chain_collider_cfg.stomach_extra_fine_offset[0]);
    body_colliders_profile_string_alias("spine04_fine_offset",
                                        "stomach04_fine_offset", "",
                                        buf, sizeof(buf), path);
    parse_vec3(buf, body_chain_collider_cfg.stomach_extra_fine_offset[1]);
    body_colliders_profile_string("hip_fine_offset", "",
                                  buf, sizeof(buf), path);
    parse_vec3(buf, body_chain_collider_cfg.hip_fine_offset);
    body_colliders_profile_string("thigh_fine_offset", "",
                                  buf, sizeof(buf), path);
    parse_vec3(buf, body_chain_collider_cfg.thigh_fine_offset);
    body_colliders_profile_string("knee_fine_offset", "",
                                  buf, sizeof(buf), path);
    parse_vec3(buf, body_chain_collider_cfg.knee_fine_offset);
    body_colliders_profile_string("ankle_fine_offset", "",
                                  buf, sizeof(buf), path);
    parse_vec3(buf, body_chain_collider_cfg.ankle_fine_offset);
    body_colliders_profile_string("ball_fine_offset", "",
                                  buf, sizeof(buf), path);
    parse_vec3(buf, body_chain_collider_cfg.ball_fine_offset);
    body_colliders_profile_string("breast_fine_offset", "",
                                  buf, sizeof(buf), path);
    parse_vec3(buf, body_chain_collider_cfg.breast_fine_offset);
    body_colliders_profile_string("butt_fine_offset", "",
                                  buf, sizeof(buf), path);
    parse_vec3(buf, body_chain_collider_cfg.butt_fine_offset);
    body_colliders_profile_string("neck_fine_offset", "",
                                  buf, sizeof(buf), path);
    parse_vec3(buf, body_chain_collider_cfg.neck_fine_offset);
    body_colliders_profile_string("head_fine_offset", "",
                                  buf, sizeof(buf), path);
    parse_vec3(buf, body_chain_collider_cfg.head_fine_offset);
    body_colliders_profile_string("clavicle_fine_offset", "",
                                  buf, sizeof(buf), path);
    parse_vec3(buf, body_chain_collider_cfg.clavicle_fine_offset);
    body_colliders_profile_string("shoulder_fine_offset", "",
                                  buf, sizeof(buf), path);
    parse_vec3(buf, body_chain_collider_cfg.shoulder_fine_offset);
    body_colliders_profile_string("elbow_fine_offset", "",
                                  buf, sizeof(buf), path);
    parse_vec3(buf, body_chain_collider_cfg.elbow_fine_offset);
    body_colliders_profile_string("forearm_fine_offset", "",
                                  buf, sizeof(buf), path);
    parse_vec3(buf, body_chain_collider_cfg.forearm_fine_offset);
    body_colliders_profile_string("wrist_fine_offset", "",
                                  buf, sizeof(buf), path);
    parse_vec3(buf, body_chain_collider_cfg.wrist_fine_offset);
    body_colliders_profile_string("palm_fine_offset", "",
                                  buf, sizeof(buf), path);
    parse_vec3(buf, body_chain_collider_cfg.palm_fine_offset);
    for (i = 0; i < 5; i++) {
        char key[48];
        wsprintfA(key, "finger%02d_fine_offset", i + 1);
        body_colliders_profile_string(key, "", buf, sizeof(buf), path);
        parse_vec3(buf, body_chain_collider_cfg.finger_fine_offset[i]);
    }

    if (body_chain_collider_cfg.testicles_bone_head_mode) {
        body_chain_collider_cfg.local_offset[BODY_COLLIDER_TESTICLES_01][0] =
            body_chain_collider_cfg.testicle_pivot[0][0] +
            body_chain_collider_cfg.testicle_fine_offset[0][0];
        body_chain_collider_cfg.local_offset[BODY_COLLIDER_TESTICLES_01][1] =
            body_chain_collider_cfg.testicle_pivot[0][1] +
            body_chain_collider_cfg.testicle_fine_offset[0][1];
        body_chain_collider_cfg.local_offset[BODY_COLLIDER_TESTICLES_01][2] =
            body_chain_collider_cfg.testicle_pivot[0][2] +
            body_chain_collider_cfg.testicle_fine_offset[0][2];
        body_chain_collider_cfg.local_offset[BODY_COLLIDER_TESTICLES_02][0] =
            body_chain_collider_cfg.testicle_pivot[1][0] +
            body_chain_collider_cfg.testicle_fine_offset[1][0];
        body_chain_collider_cfg.local_offset[BODY_COLLIDER_TESTICLES_02][1] =
            body_chain_collider_cfg.testicle_pivot[1][1] +
            body_chain_collider_cfg.testicle_fine_offset[1][1];
        body_chain_collider_cfg.local_offset[BODY_COLLIDER_TESTICLES_02][2] =
            body_chain_collider_cfg.testicle_pivot[1][2] +
            body_chain_collider_cfg.testicle_fine_offset[1][2];
    }
    body_chain_collider_cfg.local_offset[BODY_COLLIDER_TESTICLES_MID][0] =
        (body_chain_collider_cfg.local_offset[BODY_COLLIDER_TESTICLES_01][0] +
         body_chain_collider_cfg.local_offset[BODY_COLLIDER_TESTICLES_02][0]) * 0.5f;
    body_chain_collider_cfg.local_offset[BODY_COLLIDER_TESTICLES_MID][1] =
        (body_chain_collider_cfg.local_offset[BODY_COLLIDER_TESTICLES_01][1] +
         body_chain_collider_cfg.local_offset[BODY_COLLIDER_TESTICLES_02][1]) * 0.5f;
    body_chain_collider_cfg.local_offset[BODY_COLLIDER_TESTICLES_MID][2] =
        (body_chain_collider_cfg.local_offset[BODY_COLLIDER_TESTICLES_01][2] +
         body_chain_collider_cfg.local_offset[BODY_COLLIDER_TESTICLES_02][2]) * 0.5f;

    body_chain_collider_apply_config_to_nodes();
    body_chain_profile_radius_nodes_alias("spine01_radius", "stomach01_radius",
                                          body_chain_collider_cfg.stomach_radius[0],
                                          BODY_COLLIDER_STOMACH_01, -1, path);
    body_chain_profile_radius_nodes_alias("spine02_radius", "stomach02_radius",
                                          body_chain_collider_cfg.stomach_radius[1],
                                          BODY_COLLIDER_STOMACH_02, -1, path);
    body_chain_profile_radius_nodes_alias(
        "spine03_radius", "stomach03_radius",
        body_chain_collider_cfg.stomach_extra_radius[0],
        BODY_COLLIDER_STOMACH_03, -1, path);
    body_chain_profile_radius_nodes_alias(
        "spine04_radius", "stomach04_radius",
        body_chain_collider_cfg.stomach_extra_radius[1],
        BODY_COLLIDER_STOMACH_04, -1, path);
    body_chain_profile_radius_pair("hip_radius",
                                   &body_chain_collider_cfg.hip_radius,
                                   BODY_COLLIDER_HIP_L,
                                   BODY_COLLIDER_HIP_R, path);
    body_chain_profile_radius_pair("thigh_radius",
                                   &body_chain_collider_cfg.thigh_radius,
                                   BODY_COLLIDER_THIGH_L,
                                   BODY_COLLIDER_THIGH_R, path);
    body_chain_profile_radius_pair("knee_radius",
                                   &body_chain_collider_cfg.knee_radius,
                                   BODY_COLLIDER_KNEE_L,
                                   BODY_COLLIDER_KNEE_R, path);
    body_chain_profile_radius_pair("testicles_radius",
                                   &body_chain_collider_cfg.testicles_radius,
                                   BODY_COLLIDER_TESTICLES_01,
                                   BODY_COLLIDER_TESTICLES_02, path);
    body_chain_set_node_radius(BODY_COLLIDER_TESTICLES_MID,
                               body_chain_collider_cfg.node_radius
                                   [BODY_COLLIDER_TESTICLES_01]);
    body_chain_profile_radius_nodes("ankle_radius",
                                    body_chain_collider_cfg.ankle_radius,
                                    BODY_COLLIDER_ANKLE_L,
                                    BODY_COLLIDER_ANKLE_R, path);
    body_chain_profile_radius_nodes("ball_radius",
                                    body_chain_collider_cfg.ball_radius,
                                    BODY_COLLIDER_BALL_L,
                                    BODY_COLLIDER_BALL_R, path);
    body_chain_profile_radius_nodes("breast_radius",
                                    body_chain_collider_cfg.breast_radius,
                                    BODY_COLLIDER_BREAST_L,
                                    BODY_COLLIDER_BREAST_R, path);
    body_chain_profile_radius_nodes("butt_radius",
                                    body_chain_collider_cfg.butt_radius,
                                    BODY_COLLIDER_BUTT_L,
                                    BODY_COLLIDER_BUTT_R, path);
    body_chain_profile_radius_nodes("neck_radius",
                                    body_chain_collider_cfg.neck_radius,
                                    BODY_COLLIDER_NECK_01, -1, path);
    body_chain_profile_radius_nodes("head_radius",
                                    body_chain_collider_cfg.head_radius,
                                    BODY_COLLIDER_HEAD_02, -1, path);
    body_chain_profile_radius_nodes("clavicle_radius",
                                    body_chain_collider_cfg.clavicle_radius,
                                    BODY_COLLIDER_CLAVICLE_L,
                                    BODY_COLLIDER_CLAVICLE_R, path);
    body_chain_profile_radius_nodes("shoulder_radius",
                                    body_chain_collider_cfg.shoulder_radius,
                                    BODY_COLLIDER_SHOULDER_L,
                                    BODY_COLLIDER_SHOULDER_R, path);
    body_chain_profile_radius_nodes("elbow_radius",
                                    body_chain_collider_cfg.elbow_radius,
                                    BODY_COLLIDER_ELBOW_L,
                                    BODY_COLLIDER_ELBOW_R, path);
    body_chain_profile_radius_nodes("forearm_radius",
                                    body_chain_collider_cfg.forearm_radius,
                                    BODY_COLLIDER_FOREARM_L,
                                    BODY_COLLIDER_FOREARM_R, path);
    body_chain_profile_radius_nodes("wrist_radius",
                                    body_chain_collider_cfg.wrist_radius,
                                    BODY_COLLIDER_WRIST_L,
                                    BODY_COLLIDER_WRIST_R, path);
    body_chain_profile_radius_nodes("palm_radius",
                                    body_chain_collider_cfg.palm_radius,
                                    BODY_COLLIDER_PALM_L,
                                    BODY_COLLIDER_PALM_R, path);
    for (i = 0; i < 5; i++) {
        char key[32];
        int left = BODY_COLLIDER_FINGER01_L_01;
        int right = BODY_COLLIDER_FINGER01_R_01;
        wsprintfA(key, "finger%02d_radius", i + 1);
        if (i == 1) { left = BODY_COLLIDER_FINGER02_L_01; right = BODY_COLLIDER_FINGER02_R_01; }
        if (i == 2) { left = BODY_COLLIDER_FINGER03_L_01; right = BODY_COLLIDER_FINGER03_R_01; }
        if (i == 3) { left = BODY_COLLIDER_FINGER04_L_01; right = BODY_COLLIDER_FINGER04_R_01; }
        if (i == 4) { left = BODY_COLLIDER_FINGER05_L_01; right = BODY_COLLIDER_FINGER05_R_01; }
        body_chain_profile_radius_nodes(key,
                                        body_chain_collider_cfg.finger_radius[i],
                                        left, right, path);
    }
    for (i = 0; i < 4; i++) {
        body_chain_set_node_radius(BODY_COLLIDER_FINGER01_L_01 + i,
                                   body_chain_collider_cfg.finger_radius[0]);
        body_chain_set_node_radius(BODY_COLLIDER_FINGER01_R_01 + i,
                                   body_chain_collider_cfg.finger_radius[0]);
    }
    for (i = 0; i < 5; i++) {
        body_chain_set_node_radius(BODY_COLLIDER_FINGER02_L_01 + i,
                                   body_chain_collider_cfg.finger_radius[1]);
        body_chain_set_node_radius(BODY_COLLIDER_FINGER02_R_01 + i,
                                   body_chain_collider_cfg.finger_radius[1]);
        body_chain_set_node_radius(BODY_COLLIDER_FINGER03_L_01 + i,
                                   body_chain_collider_cfg.finger_radius[2]);
        body_chain_set_node_radius(BODY_COLLIDER_FINGER03_R_01 + i,
                                   body_chain_collider_cfg.finger_radius[2]);
        body_chain_set_node_radius(BODY_COLLIDER_FINGER04_L_01 + i,
                                   body_chain_collider_cfg.finger_radius[3]);
        body_chain_set_node_radius(BODY_COLLIDER_FINGER04_R_01 + i,
                                   body_chain_collider_cfg.finger_radius[3]);
        body_chain_set_node_radius(BODY_COLLIDER_FINGER05_L_01 + i,
                                   body_chain_collider_cfg.finger_radius[4]);
        body_chain_set_node_radius(BODY_COLLIDER_FINGER05_R_01 + i,
                                   body_chain_collider_cfg.finger_radius[4]);
    }
    body_chain_collider_cfg.response_radius_scale =
        body_colliders_profile_float("response_radius_scale",
                      body_chain_collider_cfg.response_radius_scale, path);
    body_chain_collider_cfg.chain_radius =
        body_colliders_profile_float("chain_radius",
                      body_chain_collider_cfg.chain_radius, path);
    body_chain_collider_cfg.link_length[0] =
        body_colliders_profile_float("joint01_link_length",
                      body_chain_collider_cfg.link_length[0], path);
    body_chain_collider_cfg.link_length[1] =
        body_colliders_profile_float("joint02_link_length",
                      body_chain_collider_cfg.link_length[1], path);
    body_chain_collider_cfg.link_length[2] =
        body_colliders_profile_float("joint03_link_length",
                      body_chain_collider_cfg.link_length[2], path);
    body_chain_collider_cfg.response_strength =
        body_colliders_profile_float("collision_strength",
                      body_colliders_profile_float("response_strength",
                                    body_chain_collider_cfg.response_strength,
                                    path), path);
    body_chain_collider_cfg.response_max_degrees_per_tick =
        body_colliders_profile_float("collision_max_degrees_per_tick",
                      body_colliders_profile_float("response_max_degrees_per_tick",
                                    body_chain_collider_cfg.response_max_degrees_per_tick,
                                    path), path);
    body_chain_collider_cfg.collision_iterations =
        body_colliders_profile_int("collision_iterations",
                              body_chain_collider_cfg.collision_iterations,
                              path);
    body_chain_collider_cfg.collision_slop =
        body_colliders_profile_float("collision_slop",
                      body_chain_collider_cfg.collision_slop, path);
    body_chain_collider_cfg.response_radius_scale =
        physx_clampf(body_chain_collider_cfg.response_radius_scale, 0.05f, 4.0f);
    body_chain_collider_cfg.chain_radius =
        physx_clampf(body_chain_collider_cfg.chain_radius, 0.001f, 0.25f);
    for (i = 0; i < 3; i++) {
        body_chain_collider_cfg.link_length[i] =
            physx_clampf(body_chain_collider_cfg.link_length[i], 0.005f, 0.50f);
    }
    body_chain_collider_cfg.response_strength =
        physx_clampf(body_chain_collider_cfg.response_strength, 0.0f, 4.0f);
    body_chain_collider_cfg.response_max_degrees_per_tick =
        physx_clampf(body_chain_collider_cfg.response_max_degrees_per_tick,
                     0.1f, 45.0f);
    if (body_chain_collider_cfg.collision_iterations < 1) {
        body_chain_collider_cfg.collision_iterations = 1;
    }
    if (body_chain_collider_cfg.collision_iterations > 6) {
        body_chain_collider_cfg.collision_iterations = 6;
    }
    body_chain_collider_cfg.collision_slop =
        physx_clampf(body_chain_collider_cfg.collision_slop, 0.0f, 0.02f);

    {
        int enabled = 1;
        int found = 0;
        if (body_profile_bool_if_present(PENIS_PHYSICS_LEGACY_CONFIG_SECTION,
                                         "enabled", path, &enabled)) {
            found = 1;
        }
        if (body_profile_bool_if_present(PENIS_PHYSICS_CONFIG_SECTION,
                                         "enabled", path, &enabled)) {
            found = 1;
        }
        if (found && !enabled) {
            body_chain_collider_cfg.penis_collision_enabled = 0;
        }

        enabled = 1;
        if (body_profile_bool_if_present(TESTICLE_PHYSICS_CONFIG_SECTION,
                                         "enabled", path, &enabled) &&
            !enabled) {
            body_chain_collider_cfg.testicle_collision_enabled = 0;
            body_chain_collider_cfg.live_testicle_bones = 0;
        }

        enabled = 1;
        if (body_profile_bool_if_present(BREASTS_PHYSICS_CONFIG_SECTION,
                                         "enabled", path, &enabled) &&
            !enabled) {
            body_chain_collider_cfg.breasts_collision_enabled = 0;
        }
    }
}

static int physx_toggle_person_ini_setting(const char *section,
                                           const char *kind,
                                           int person_index,
                                           int fallback_enabled)
{
    char key[32];
    char current_value[32];
    int currently_enabled;
    int new_enabled;
    if (!section || !kind || person_index < 0 || person_index >= 4) {
        return 0;
    }
    if (!config_path[0]) config_file_path(config_path, sizeof(config_path));
    migrate_legacy_penis_physics_section();
    _snprintf(key, sizeof(key) - 1, "enabled_person%02d",
              person_index + 1);
    key[sizeof(key) - 1] = 0;
    currently_enabled = fallback_enabled ? 1 : 0;
    current_value[0] = 0;
    GetPrivateProfileStringA(section, key, "", current_value,
                             sizeof(current_value), config_path);
    if (current_value[0]) {
        int ini_enabled;
        if (physx_settings_bool_value(current_value, &ini_enabled)) {
            currently_enabled = ini_enabled;
        }
    }
    new_enabled = currently_enabled ? 0 : 1;
    if (!WritePrivateProfileStringA(section, key,
                                    new_enabled ? "true" : "false",
                                    config_path)) {
        log_line("settings write failed source=person-context action=%s person=Person%02d ini=[%s] %s path=\"%s\"",
                 kind, person_index + 1, section, key, config_path);
        return 0;
    }
    if (strcmp(section, PENIS_PHYSICS_CONFIG_SECTION) == 0) {
        physx_mark_penis_physics_setting_change(key, new_enabled);
    } else if (strcmp(section, TESTICLE_PHYSICS_CONFIG_SECTION) == 0) {
        physx_mark_testicle_physics_setting_change(key, new_enabled);
    }
    log_line("person-context toggle action=%s person=Person%02d old=%s new=%s ini=[%s] %s source=direct-ini note=\"normal INI hot-reload scheduled\"",
             kind, person_index + 1,
             currently_enabled ? "ON" : "OFF",
             new_enabled ? "ON" : "OFF",
             section, key);
    return 1;
}

static void body_profile_rebuild_effective_configs(void)
{
    int person_index;
    body_profile_set_active_person_config(-1);
    for (person_index = 0; person_index < 4; person_index++) {
        body_chain_physics_person_cfg[person_index] =
            body_chain_physics_global_cfg;
        testicle_physics_person_cfg[person_index] =
            testicle_physics_global_cfg;
        breasts_physics_person_cfg[person_index] =
            breasts_physics_global_cfg;
        butt_physics_person_cfg[person_index] =
            butt_physics_global_cfg;
        body_chain_collider_person_cfg[person_index] =
            body_chain_collider_global_cfg;
        if (body_profile_person_sidecar_active[person_index]) {
            body_profile_set_active_person_config(person_index);
            body_profile_overlay_body_physics_sections(
                person_index,
                body_profile_person_sidecar_path[person_index]);
            body_profile_overlay_body_collider_section(
                body_profile_person_sidecar_path[person_index]);
            log_line("body-profile effective person=\"Person%02d\" sidecar=\"%s\" body=\"%s\" breasts_physics=%d breasts_person=%d breasts_collision=%d breasts_scope=%s butt_physics=%d butt_person=%d butt_collision=%d butt_scope=%s penis_physics=%d penis_person=%d penis_collision=%d penis_scope=%s testicle_physics=%d testicle_person=%d testicle_collision=%d testicle_scope=%s body_colliders=%d note=\"global fallback copied first; sidecar overlaid for this person only\"",
                     person_index + 1,
                     body_profile_person_sidecar_path[person_index],
                     body_profile_person_body_path[person_index],
                     breasts_physics_person_cfg[person_index].enabled,
                     breasts_physics_person_cfg[person_index]
                         .enabled_person[person_index],
                     body_chain_collider_cfg.breasts_collision_enabled,
                     body_chain_collision_scope_name(
                         breasts_physics_person_cfg[person_index]
                             .collision_scope),
                     butt_physics_person_cfg[person_index].enabled,
                     butt_physics_person_cfg[person_index]
                         .enabled_person[person_index],
                     body_chain_collider_cfg.butt_collision_enabled,
                     body_chain_collision_scope_name(
                         butt_physics_person_cfg[person_index]
                             .collision_scope),
                     body_chain_physics_cfg.enabled,
                     body_chain_physics_cfg.enabled_person[person_index],
                     body_chain_collider_cfg.penis_collision_enabled,
                     body_chain_collision_scope_name(
                         body_chain_physics_cfg.collision_scope),
                     testicle_physics_cfg.enabled,
                     testicle_physics_cfg.enabled_person[person_index],
                     body_chain_collider_cfg.testicle_collision_enabled,
                     body_chain_collision_scope_name(
                         testicle_physics_cfg.collision_scope),
                     body_chain_collider_cfg.enabled);
        }
    }
    body_profile_set_active_person_config(-1);
}

static void handle_physx_settings_change(const char *param_ref,
                                         const char *value_ref)
{
    const char *param_name = stringref_cstr_a(param_ref);
    const char *string_value = stringref_cstr_a(value_ref);
    const char *collision_scope;
    int enabled;
    size_t i;
    if (!param_name || _strnicmp(param_name, "NCPhysX", 7) != 0) return;
    if (!config_path[0]) config_file_path(config_path, sizeof(config_path));
    migrate_legacy_penis_physics_section();
    for (i = 0; i < sizeof(physx_settings_bindings) / sizeof(physx_settings_bindings[0]); i++) {
        const physx_settings_binding_t *binding = &physx_settings_bindings[i];
        if (strcmp(param_name, binding->param_name) != 0) continue;
        if (strcmp(binding->key, "collision_scope") == 0) {
            collision_scope = physx_settings_collision_scope_value(string_value);
            if (!collision_scope) {
                log_line("settings ignored param=\"%s\" value=\"%s\" reason=\"unsupported collision scope\"",
                         param_name, string_value ? string_value : "");
                return;
            }
            if (WritePrivateProfileStringA(binding->section, binding->key,
                                           collision_scope, config_path)) {
                log_line("settings changed param=\"%s\" value=%s ini=[%s] %s note=\"normal INI hot-reload scheduled\"",
                         param_name, collision_scope,
                         binding->section, binding->key);
            } else {
                log_line("settings write failed param=\"%s\" ini=[%s] %s path=\"%s\"",
                         param_name, binding->section, binding->key, config_path);
            }
            return;
        }
        if (!string_value || !physx_settings_bool_value(string_value, &enabled)) {
            log_line("settings ignored param=\"%s\" value=%p reason=\"unsupported spinbox value\"",
                     param_name, value_ref);
            return;
        }
        if (WritePrivateProfileStringA(binding->section, binding->key,
                                       enabled ? "true" : "false",
                                       config_path)) {
            if (strcmp(binding->section, PENIS_PHYSICS_CONFIG_SECTION) == 0) {
                physx_mark_penis_physics_setting_change(binding->key, enabled);
            } else if (strcmp(binding->section,
                              TESTICLE_PHYSICS_CONFIG_SECTION) == 0) {
                physx_mark_testicle_physics_setting_change(binding->key,
                                                           enabled);
            } else if (strcmp(binding->section, BODY_COLLIDERS_CONFIG_SECTION) == 0 &&
                       strcmp(binding->key, "penis_collision_enabled") == 0) {
                physx_mark_penis_collision_setting_change(enabled);
            }
            log_line("settings changed param=\"%s\" value=%s ini=[%s] %s note=\"normal INI hot-reload scheduled\"",
                     param_name, enabled ? "ON" : "OFF",
                     binding->section, binding->key);
        } else {
            log_line("settings write failed param=\"%s\" ini=[%s] %s path=\"%s\"",
                     param_name, binding->section, binding->key, config_path);
        }
        return;
    }
    log_line("settings ignored unknown PhysX parameter param=\"%s\"", param_name);
}

static int detect_poseeditor_total_tracks_from_file(void)
{
    char path[MAX_PATH * 4];
    char line[2048];
    FILE *f;
    int max_track = -1;
    sibling_file_path("tracks.ini", path, sizeof(path));
    f = fopen(path, "rb");
    if (!f) return 0;
    while (fgets(line, sizeof(line), f)) {
        char *p = line;
        char *comma1;
        char *comma2;
        long track_id;
        while (*p == ' ' || *p == '\t') p++;
        if (!*p || *p == '#' || *p == ';' || *p == '\r' || *p == '\n') continue;
        comma1 = strchr(p, ',');
        if (!comma1) continue;
        comma2 = strchr(comma1 + 1, ',');
        if (!comma2) continue;
        *comma2 = 0;
        track_id = strtol(comma1 + 1, NULL, 10);
        if (track_id > max_track) max_track = (int)track_id;
    }
    fclose(f);
    return max_track >= 0 ? max_track + 1 : 0;
}

static void parse_vec3(const char *s, float out[3])
{
    float x, y, z;
    if (!s || !s[0]) return;
    if (sscanf(s, "%f,%f,%f", &x, &y, &z) == 3 || sscanf(s, "%f %f %f", &x, &y, &z) == 3) {
        out[0] = x; out[1] = y; out[2] = z;
    }
}

static float profile_float(const char *section, const char *key, float fallback, const char *path)
{
    char buf[128];
    GetPrivateProfileStringA(section, key, "", buf, sizeof(buf), path);
    if (!buf[0]) return fallback;
    return (float)atof(buf);
}

static int profile_axis(const char *section, const char *key,
                        int fallback, const char *path)
{
    char buf[64];
    int axis;
    GetPrivateProfileStringA(section, key, "", buf, sizeof(buf), path);
    trim_in_place(buf);
    if (!buf[0]) return fallback;
    axis = parse_axis_name(buf);
    return axis >= 0 && axis <= 2 ? axis : fallback;
}

static void load_body_wind_axis_mapping(
    body_chain_physics_config_t *cfg, const char *section,
    const char *path)
{
    static const char *source_keys[3] = {
        "wind_horizontal_source_axis",
        "wind_vertical_source_axis",
        "wind_depth_source_axis"
    };
    static const char *tail_keys[3] = {
        "wind_horizontal_tail_axis",
        "wind_vertical_tail_axis",
        "wind_depth_tail_axis"
    };
    static const char *scale_keys[3] = {
        "wind_horizontal_scale",
        "wind_vertical_scale",
        "wind_depth_scale"
    };
    int channel;
    if (!cfg || !section || !path) return;
    for (channel = 0; channel < 3; channel++) {
        cfg->wind_source_axis[channel] = profile_axis(
            section, source_keys[channel],
            cfg->wind_source_axis[channel], path);
        cfg->wind_tail_axis[channel] = profile_axis(
            section, tail_keys[channel],
            cfg->wind_tail_axis[channel], path);
        cfg->wind_axis_scale[channel] = physx_clampf(
            profile_float(section, scale_keys[channel],
                          cfg->wind_axis_scale[channel], path),
            -10.0f, 10.0f);
    }
}

static int profile_translation_channel(const char *section, const char *key,
                                       int fallback, const char *path)
{
    char buf[64];
    GetPrivateProfileStringA(section, key, "", buf, sizeof(buf), path);
    trim_in_place(buf);
    if (!buf[0]) return fallback;
    if (_stricmp(buf, "horizontal") == 0) return 0;
    if (_stricmp(buf, "vertical") == 0) return 1;
    if (_stricmp(buf, "depth") == 0) return 2;
    log_line("invalid face-down translation channel section=[%s] value=\"%s\"; keeping %s",
             section, buf,
             fallback == 0 ? "horizontal" :
             fallback == 2 ? "depth" : "vertical");
    return fallback;
}

static int profile_body_chain_collision_scope(const char *section,
                                               int fallback,
                                               const char *path)
{
    char buf[128];
    GetPrivateProfileStringA(section, "collision_scope", "",
                             buf, sizeof(buf), path);
    trim_in_place(buf);
    if (!buf[0]) return fallback;
    if (_stricmp(buf, "genitals_only") == 0) {
        return BODY_CHAIN_COLLISION_SCOPE_GENITALS_ONLY;
    }
    if (_stricmp(buf, "genitals_only_all") == 0) {
        return BODY_CHAIN_COLLISION_SCOPE_GENITALS_ONLY_ALL;
    }
    if (_stricmp(buf, "pelvis_and_genitals_only") == 0) {
        return BODY_CHAIN_COLLISION_SCOPE_PELVIS_AND_GENITALS_ONLY;
    }
    if (_stricmp(buf, "pelvis_and_genitals_only_all") == 0 ||
        _stricmp(buf, "pelvis_and_testicles_only_all") == 0) {
        return BODY_CHAIN_COLLISION_SCOPE_PELVIS_AND_GENITALS_ONLY_ALL;
    }
    if (_stricmp(buf, "pelvis_genitals_hands_only") == 0 ||
        _stricmp(buf, "pelvis_genitals_and_hands_only") == 0) {
        return BODY_CHAIN_COLLISION_SCOPE_PELVIS_GENITALS_AND_HANDS_ONLY;
    }
    if (_stricmp(buf, "pelvis_genitals_hands_only_all") == 0 ||
        _stricmp(buf, "pelvis_genitals_and_hands_only_all") == 0 ||
        _stricmp(buf, "pelvis_genitals_and_hands_only_") == 0) {
        return BODY_CHAIN_COLLISION_SCOPE_PELVIS_GENITALS_AND_HANDS_ONLY_ALL;
    }
    if (_stricmp(buf, "full_body") == 0) {
        return BODY_CHAIN_COLLISION_SCOPE_FULL_BODY;
    }
    if (_stricmp(buf, "full_body_all") == 0 ||
        _stricmp(buf, "full_body_allt") == 0) {
        return BODY_CHAIN_COLLISION_SCOPE_FULL_BODY_ALL;
    }
    if (_stricmp(buf, "hands_only") == 0) {
        return BODY_CHAIN_COLLISION_SCOPE_HANDS_ONLY;
    }
    if (_stricmp(buf, "hands_only_all") == 0) {
        return BODY_CHAIN_COLLISION_SCOPE_HANDS_ONLY_ALL;
    }
    log_line("invalid collision_scope section=[%s] value=\"%s\"; keeping %s",
             section, buf, body_chain_collision_scope_name(fallback));
    return fallback;
}

static void fill_vec3(float out[3], float value)
{
    if (!out) return;
    out[0] = value;
    out[1] = value;
    out[2] = value;
}

static void profile_vec3_or_float(const char *section,
                                  const char *key,
                                  float fallback,
                                  float out[3],
                                  const char *path)
{
    char buf[128];
    float x, y, z;
    if (!out) return;
    fill_vec3(out, fallback);
    GetPrivateProfileStringA(section, key, "", buf, sizeof(buf), path);
    trim_in_place(buf);
    if (!buf[0]) return;
    if (sscanf(buf, "%f,%f,%f", &x, &y, &z) == 3 ||
        sscanf(buf, "%f %f %f", &x, &y, &z) == 3) {
        out[0] = x;
        out[1] = y;
        out[2] = z;
        return;
    }
    x = (float)atof(buf);
    fill_vec3(out, x);
}

static void profile_min_angle_vec3_or_float(const char *section,
                                            const char *key,
                                            const float max_angle[3],
                                            float out[3],
                                            const char *path)
{
    char buf[128];
    float x, y, z;
    int axis;
    if (!out || !max_angle) return;
    for (axis = 0; axis < 3; axis++) {
        out[axis] = -max_angle[axis];
    }
    GetPrivateProfileStringA(section, key, "", buf, sizeof(buf), path);
    trim_in_place(buf);
    if (!buf[0]) return;
    if (sscanf(buf, "%f,%f,%f", &x, &y, &z) == 3 ||
        sscanf(buf, "%f %f %f", &x, &y, &z) == 3) {
        out[0] = x;
        out[1] = y;
        out[2] = z;
        return;
    }
    x = (float)atof(buf);
    fill_vec3(out, x);
}

static void clamp_angle_range_vec3(float min_angle[3], float max_angle[3])
{
    int axis;
    if (!min_angle || !max_angle) return;
    for (axis = 0; axis < 3; axis++) {
        max_angle[axis] = physx_clampf(max_angle[axis], 0.0f, 360.0f);
        min_angle[axis] = physx_clampf(min_angle[axis], -360.0f, 360.0f);
        if (min_angle[axis] > max_angle[axis]) {
            min_angle[axis] = max_angle[axis];
        }
    }
}

static float body_chain_link_axis_limit(const body_chain_physics_config_t *cfg,
                                        int link,
                                        int output_axis)
{
    if (!cfg || link < 0 || link >= 3 ||
        output_axis < 0 || output_axis > 2) {
        return 1.0f;
    }
    return physx_clampf(cfg->link_max_angle[link][output_axis],
                        1.0f, 360.0f);
}

static float body_chain_link_axis_min_limit(
    const body_chain_physics_config_t *cfg,
    int link,
    int output_axis)
{
    float max_angle;
    if (!cfg || link < 0 || link >= 3 ||
        output_axis < 0 || output_axis > 2) {
        return -1.0f;
    }
    max_angle = body_chain_link_axis_limit(cfg, link, output_axis);
    return physx_clampf(cfg->link_min_angle[link][output_axis],
                        -360.0f, max_angle);
}

static float body_chain_clamp_link_axis_angle(
    const body_chain_physics_config_t *cfg,
    int link,
    int output_axis,
    float angle)
{
    return physx_clampf(angle,
                        body_chain_link_axis_min_limit(cfg, link, output_axis),
                        body_chain_link_axis_limit(cfg, link, output_axis));
}

static float body_chain_clamp_link_output_value(
    const body_chain_physics_config_t *cfg,
    int link,
    int output_axis,
    float rest,
    float value)
{
    return physx_clampf(value,
                        rest + body_chain_link_axis_min_limit(cfg, link,
                                                              output_axis),
                        rest + body_chain_link_axis_limit(cfg, link,
                                                          output_axis));
}

static int profile_bool(const char *section, const char *key, int fallback, const char *path)
{
    char buf[64];
    GetPrivateProfileStringA(section, key, "", buf, sizeof(buf), path);
    trim_in_place(buf);
    if (!buf[0]) return fallback ? 1 : 0;
    if (_stricmp(buf, "1") == 0 || _stricmp(buf, "true") == 0 ||
        _stricmp(buf, "yes") == 0 || _stricmp(buf, "on") == 0) return 1;
    if (_stricmp(buf, "0") == 0 || _stricmp(buf, "false") == 0 ||
        _stricmp(buf, "no") == 0 || _stricmp(buf, "off") == 0) return 0;
    return atoi(buf) ? 1 : 0;
}

static int parse_axis_name(const char *axis)
{
    if (!axis || !axis[0]) return 1;
    if (_stricmp(axis, "x") == 0 || _stricmp(axis, "0") == 0) return 0;
    if (_stricmp(axis, "y") == 0 || _stricmp(axis, "1") == 0) return 1;
    if (_stricmp(axis, "z") == 0 || _stricmp(axis, "2") == 0) return 2;
    return 1;
}

static int parse_optional_axis_name(const char *axis, int fallback)
{
    if (!axis || !axis[0]) return fallback;
    if (_stricmp(axis, "none") == 0 || _stricmp(axis, "off") == 0 || _stricmp(axis, "-1") == 0) return -1;
    if (_stricmp(axis, "x") == 0 || _stricmp(axis, "0") == 0) return 0;
    if (_stricmp(axis, "y") == 0 || _stricmp(axis, "1") == 0) return 1;
    if (_stricmp(axis, "z") == 0 || _stricmp(axis, "2") == 0) return 2;
    return fallback;
}

static int parse_offset_value(const char *s, int fallback)
{
    char *end = NULL;
    long v;
    if (!s || !s[0]) return fallback;
    while (*s == ' ' || *s == '\t') s++;
    if (!s[0]) return fallback;
    if (_stricmp(s, "auto") == 0) return -2;
    v = strtol(s, &end, 0);
    if (end == s || v < 0 || v > 0x4000) return fallback;
    return (int)v;
}

static int parse_offset_list(const char *s, int *out, int max_count)
{
    char buf[256];
    char *p;
    int count = 0;
    if (!s || !s[0] || !out || max_count <= 0) return 0;
    lstrcpynA(buf, s, sizeof(buf));
    p = buf;
    while (*p && count < max_count) {
        char *start;
        char *end;
        int value;
        while (*p == ' ' || *p == '\t' || *p == ',') p++;
        if (!*p) break;
        start = p;
        while (*p && *p != ',') p++;
        end = p;
        while (end > start && (end[-1] == ' ' || end[-1] == '\t')) end--;
        if (*p == ',') {
            *p = '\0';
            p++;
        }
        *end = '\0';
        value = parse_offset_value(start, -1);
        if (value >= 0) out[count++] = value;
    }
    return count;
}

static void load_global_config(void)
{
    char buf[256];
    int i;
    int hot_reload = config_loaded;
    restore_collision_auto_test_active();
    config_file_path(config_path, sizeof(config_path));
    defaults_cfg.stiffness = profile_float("defaults", "stiffness", defaults_cfg.stiffness, config_path);
    defaults_cfg.damping = profile_float("defaults", "damping", defaults_cfg.damping, config_path);
    defaults_cfg.limit_angle = profile_float("defaults", "limit_angle", defaults_cfg.limit_angle, config_path);
    defaults_cfg.debug = profile_bool("defaults", "debug", defaults_cfg.debug, config_path);
    defaults_cfg.performance_profile =
        profile_bool("defaults", "performance_profile",
                     defaults_cfg.performance_profile, config_path);
    defaults_cfg.config_reload_poll_ms =
        GetPrivateProfileIntA("defaults", "config_reload_poll_ms",
                              defaults_cfg.config_reload_poll_ms,
                              config_path);
    if (defaults_cfg.config_reload_poll_ms < 100) {
        defaults_cfg.config_reload_poll_ms = 100;
    }
    if (defaults_cfg.config_reload_poll_ms > 5000) {
        defaults_cfg.config_reload_poll_ms = 5000;
    }
    GetPrivateProfileStringA("defaults", "gravity", "0,-1,0", buf, sizeof(buf), config_path);
    parse_vec3(buf, defaults_cfg.gravity);
    addon_physics_enabled =
        profile_bool("addon_physics", "enabled", addon_physics_enabled,
                     config_path);
    addon_physics_probe_enabled =
        profile_bool("addon_physics", "binding_probe",
                     addon_physics_probe_enabled,
                     config_path);
    addon_sidecar_hot_reload_enabled =
        profile_bool("addon_physics", "sidecar_hot_reload",
                     addon_sidecar_hot_reload_enabled,
                     config_path);
    addon_sidecar_hot_reload_interval_ms =
        GetPrivateProfileIntA("addon_physics",
                              "sidecar_hot_reload_interval_ms",
                              addon_sidecar_hot_reload_interval_ms,
                              config_path);
    addon_sidecar_hot_reload_max_checks_per_tick =
        GetPrivateProfileIntA("addon_physics",
                              "sidecar_hot_reload_max_checks_per_tick",
                              addon_sidecar_hot_reload_max_checks_per_tick,
                              config_path);
    if (addon_sidecar_hot_reload_interval_ms < 250) {
        addon_sidecar_hot_reload_interval_ms = 250;
    }
    if (addon_sidecar_hot_reload_interval_ms > 10000) {
        addon_sidecar_hot_reload_interval_ms = 10000;
    }
    if (addon_sidecar_hot_reload_max_checks_per_tick < 1) {
        addon_sidecar_hot_reload_max_checks_per_tick = 1;
    }
    if (addon_sidecar_hot_reload_max_checks_per_tick > 64) {
        addon_sidecar_hot_reload_max_checks_per_tick = 64;
    }

    physics_environment_cfg.world_gravity_probe =
        profile_bool("physics_environment", "world_gravity_probe",
                     physics_environment_cfg.world_gravity_probe,
                     config_path);
    physics_environment_cfg.wind_enabled =
        profile_bool("physics_environment", "wind_enabled",
                     physics_environment_cfg.wind_enabled,
                     config_path);
    physics_environment_cfg.world_gravity[0] = defaults_cfg.gravity[0];
    physics_environment_cfg.world_gravity[1] = defaults_cfg.gravity[1];
    physics_environment_cfg.world_gravity[2] = defaults_cfg.gravity[2];
    GetPrivateProfileStringA("physics_environment", "world_gravity", "", buf, sizeof(buf), config_path);
    trim_in_place(buf);
    parse_vec3(buf, physics_environment_cfg.world_gravity);
    GetPrivateProfileStringA("physics_environment", "gravity_horizontal_source_vector", "", buf, sizeof(buf), config_path);
    trim_in_place(buf);
    parse_vec3(buf, physics_environment_cfg.gravity_horizontal_source_vector);
    GetPrivateProfileStringA("physics_environment", "gravity_vertical_source_vector", "", buf, sizeof(buf), config_path);
    trim_in_place(buf);
    parse_vec3(buf, physics_environment_cfg.gravity_vertical_source_vector);
    physics_environment_cfg.gravity_apply_to_body_chain =
        profile_bool("physics_environment", "gravity_apply_to_body_chain",
                     physics_environment_cfg.gravity_apply_to_body_chain,
                     config_path);
    physics_environment_cfg.gravity_body_chain_scale =
        profile_float("physics_environment", "gravity_body_chain_scale",
                      physics_environment_cfg.gravity_body_chain_scale,
                      config_path);
    physics_environment_cfg.gravity_horizontal_body_chain_scale =
        profile_float("physics_environment", "gravity_horizontal_body_chain_scale",
                      physics_environment_cfg.gravity_body_chain_scale,
                      config_path);
    physics_environment_cfg.gravity_vertical_body_chain_scale =
        profile_float("physics_environment", "gravity_vertical_body_chain_scale",
                      physics_environment_cfg.gravity_body_chain_scale,
                      config_path);
    physics_environment_cfg.gravity_horizontal_secondary_body_chain_scale =
        profile_float("physics_environment", "gravity_horizontal_secondary_body_chain_scale",
                      physics_environment_cfg.gravity_horizontal_secondary_body_chain_scale,
                      config_path);
    physics_environment_cfg.gravity_vertical_secondary_body_chain_scale =
        profile_float("physics_environment", "gravity_vertical_secondary_body_chain_scale",
                      physics_environment_cfg.gravity_vertical_secondary_body_chain_scale,
                      config_path);
    physics_environment_cfg.gravity_horizontal_tail_axis =
        profile_axis("physics_environment", "gravity_horizontal_tail_axis",
                     physics_environment_cfg.gravity_horizontal_tail_axis,
                     config_path);
    physics_environment_cfg.gravity_vertical_tail_axis =
        profile_axis("physics_environment", "gravity_vertical_tail_axis",
                     physics_environment_cfg.gravity_vertical_tail_axis,
                     config_path);
    physics_environment_cfg.body_chain_camera_relative_orientation =
        profile_bool("physics_environment", "body_chain_camera_relative_orientation",
                     physics_environment_cfg.body_chain_camera_relative_orientation,
                     config_path);
    physics_environment_cfg.body_chain_camera_coast_stiffness_scale =
        profile_float("physics_environment", "body_chain_camera_coast_stiffness_scale",
                      physics_environment_cfg.body_chain_camera_coast_stiffness_scale,
                      config_path);
    physics_environment_cfg.body_chain_camera_coast_damping_scale =
        profile_float("physics_environment", "body_chain_camera_coast_damping_scale",
                      physics_environment_cfg.body_chain_camera_coast_damping_scale,
                      config_path);
    physics_environment_cfg.body_chain_camera_quarantine_ms =
        GetPrivateProfileIntA("physics_environment", "body_chain_camera_quarantine_ms",
                              physics_environment_cfg.body_chain_camera_quarantine_ms,
                              config_path);
    physics_environment_cfg.gravity_dynamic_body_basis =
        profile_bool("physics_environment", "gravity_dynamic_body_basis",
                     physics_environment_cfg.gravity_dynamic_body_basis,
                     config_path);
    physics_environment_cfg.gravity_basis_camera_compensate =
        profile_bool("physics_environment", "gravity_basis_camera_compensate",
                     physics_environment_cfg.gravity_basis_camera_compensate,
                     config_path);
    GetPrivateProfileStringA("physics_environment", "gravity_basis_node",
                             physics_environment_cfg.gravity_basis_node,
                             physics_environment_cfg.gravity_basis_node,
                             sizeof(physics_environment_cfg.gravity_basis_node),
                             config_path);
    trim_in_place(physics_environment_cfg.gravity_basis_node);
    if (!physics_environment_cfg.gravity_basis_node[0]) {
        lstrcpynA(physics_environment_cfg.gravity_basis_node, "root",
                  sizeof(physics_environment_cfg.gravity_basis_node));
    }
    GetPrivateProfileStringA("physics_environment", "gravity_horizontal_basis_offset", "", buf, sizeof(buf), config_path);
    trim_in_place(buf);
    physics_environment_cfg.gravity_horizontal_basis_offset =
        parse_offset_value(buf, physics_environment_cfg.gravity_horizontal_basis_offset);
    GetPrivateProfileStringA("physics_environment", "gravity_vertical_basis_offset", "", buf, sizeof(buf), config_path);
    trim_in_place(buf);
    physics_environment_cfg.gravity_vertical_basis_offset =
        parse_offset_value(buf, physics_environment_cfg.gravity_vertical_basis_offset);
    GetPrivateProfileStringA("physics_environment", "gravity_horizontal_secondary_basis_offset", "", buf, sizeof(buf), config_path);
    trim_in_place(buf);
    physics_environment_cfg.gravity_horizontal_secondary_basis_offset =
        parse_offset_value(buf, physics_environment_cfg.gravity_horizontal_secondary_basis_offset);
    physics_environment_cfg.gravity_horizontal_basis_sign =
        profile_float("physics_environment", "gravity_horizontal_basis_sign",
                      physics_environment_cfg.gravity_horizontal_basis_sign,
                      config_path);
    physics_environment_cfg.gravity_vertical_basis_sign =
        profile_float("physics_environment", "gravity_vertical_basis_sign",
                      physics_environment_cfg.gravity_vertical_basis_sign,
                      config_path);
    physics_environment_cfg.gravity_horizontal_secondary_basis_sign =
        profile_float("physics_environment", "gravity_horizontal_secondary_basis_sign",
                      physics_environment_cfg.gravity_horizontal_secondary_basis_sign,
                      config_path);
    physics_environment_cfg.gravity_zero_at_start =
        profile_bool("physics_environment", "gravity_zero_at_start",
                     physics_environment_cfg.gravity_zero_at_start,
                     config_path);
    physics_environment_cfg.gravity_response_ms =
        profile_float("physics_environment", "gravity_response_ms",
                      physics_environment_cfg.gravity_response_ms,
                      config_path);
    physics_environment_cfg.gravity_max_degrees_per_second =
        profile_float("physics_environment", "gravity_max_degrees_per_second",
                      physics_environment_cfg.gravity_max_degrees_per_second,
                      config_path);
    physics_environment_cfg.gravity_probe_settle_ms =
        GetPrivateProfileIntA("physics_environment", "gravity_probe_settle_ms",
                              physics_environment_cfg.gravity_probe_settle_ms,
                              config_path);
    physics_environment_cfg.gravity_probe_confirm_ms =
        GetPrivateProfileIntA("physics_environment", "gravity_probe_confirm_ms",
                              physics_environment_cfg.gravity_probe_confirm_ms,
                              config_path);
    physics_environment_cfg.gravity_probe_camera_quiet_ms =
        GetPrivateProfileIntA("physics_environment", "gravity_probe_camera_quiet_ms",
                              physics_environment_cfg.gravity_probe_camera_quiet_ms,
                              config_path);
    physics_environment_cfg.gravity_probe_log_ms =
        GetPrivateProfileIntA("physics_environment", "gravity_probe_log_ms",
                              physics_environment_cfg.gravity_probe_log_ms,
                              config_path);
    physics_environment_cfg.gravity_probe_motion_epsilon =
        profile_float("physics_environment", "gravity_probe_motion_epsilon",
                      physics_environment_cfg.gravity_probe_motion_epsilon,
                      config_path);
    physics_environment_cfg.gravity_probe_invalidate_epsilon =
        profile_float("physics_environment", "gravity_probe_invalidate_epsilon",
                      physics_environment_cfg.gravity_probe_invalidate_epsilon,
                      config_path);
    physics_environment_cfg.gravity_probe_require_nonzero_root =
        profile_bool("physics_environment", "gravity_probe_require_nonzero_root",
                     physics_environment_cfg.gravity_probe_require_nonzero_root,
                     config_path);
    if (physics_environment_cfg.gravity_probe_settle_ms < 0) physics_environment_cfg.gravity_probe_settle_ms = 0;
    if (physics_environment_cfg.gravity_probe_settle_ms > 30000) physics_environment_cfg.gravity_probe_settle_ms = 30000;
    if (physics_environment_cfg.gravity_probe_confirm_ms < 0) physics_environment_cfg.gravity_probe_confirm_ms = 0;
    if (physics_environment_cfg.gravity_probe_confirm_ms > 30000) physics_environment_cfg.gravity_probe_confirm_ms = 30000;
    if (physics_environment_cfg.gravity_probe_camera_quiet_ms < 0) physics_environment_cfg.gravity_probe_camera_quiet_ms = 0;
    if (physics_environment_cfg.gravity_probe_camera_quiet_ms > 30000) physics_environment_cfg.gravity_probe_camera_quiet_ms = 30000;
    if (physics_environment_cfg.gravity_probe_log_ms < 250) physics_environment_cfg.gravity_probe_log_ms = 250;
    if (physics_environment_cfg.gravity_probe_log_ms > 30000) physics_environment_cfg.gravity_probe_log_ms = 30000;
    physics_environment_cfg.gravity_response_ms =
        physx_clampf(physics_environment_cfg.gravity_response_ms, 0.0f, 5000.0f);
    physics_environment_cfg.gravity_max_degrees_per_second =
        physx_clampf(physics_environment_cfg.gravity_max_degrees_per_second, 0.0f, 5000.0f);
    physics_environment_cfg.body_chain_camera_coast_stiffness_scale =
        physx_clampf(physics_environment_cfg.body_chain_camera_coast_stiffness_scale,
                     0.0f, 1.0f);
    physics_environment_cfg.body_chain_camera_coast_damping_scale =
        physx_clampf(physics_environment_cfg.body_chain_camera_coast_damping_scale,
                     0.0f, 1.0f);
    if (physics_environment_cfg.body_chain_camera_quarantine_ms < 0) {
        physics_environment_cfg.body_chain_camera_quarantine_ms = 0;
    }
    if (physics_environment_cfg.body_chain_camera_quarantine_ms > 5000) {
        physics_environment_cfg.body_chain_camera_quarantine_ms = 5000;
    }
    physics_environment_cfg.gravity_probe_motion_epsilon =
        physx_clampf(physics_environment_cfg.gravity_probe_motion_epsilon,
                     0.0f, 1.0f);
    physics_environment_cfg.gravity_probe_invalidate_epsilon =
        physx_clampf(physics_environment_cfg.gravity_probe_invalidate_epsilon,
                     physics_environment_cfg.gravity_probe_motion_epsilon,
                     100.0f);
    physics_environment_cfg.gravity_body_chain_scale =
        physx_clampf(physics_environment_cfg.gravity_body_chain_scale, -60.0f, 60.0f);
    physics_environment_cfg.gravity_horizontal_body_chain_scale =
        physx_clampf(physics_environment_cfg.gravity_horizontal_body_chain_scale, -60.0f, 60.0f);
    physics_environment_cfg.gravity_vertical_body_chain_scale =
        physx_clampf(physics_environment_cfg.gravity_vertical_body_chain_scale, -60.0f, 60.0f);
    physics_environment_cfg.gravity_horizontal_secondary_body_chain_scale =
        physx_clampf(physics_environment_cfg.gravity_horizontal_secondary_body_chain_scale, -60.0f, 60.0f);
    physics_environment_cfg.gravity_vertical_secondary_body_chain_scale =
        physx_clampf(physics_environment_cfg.gravity_vertical_secondary_body_chain_scale, -60.0f, 60.0f);
    physics_environment_cfg.gravity_horizontal_basis_sign =
        physx_clampf(physics_environment_cfg.gravity_horizontal_basis_sign, -4.0f, 4.0f);
    physics_environment_cfg.gravity_vertical_basis_sign =
        physx_clampf(physics_environment_cfg.gravity_vertical_basis_sign, -4.0f, 4.0f);
    physics_environment_cfg.gravity_horizontal_secondary_basis_sign =
        physx_clampf(physics_environment_cfg.gravity_horizontal_secondary_basis_sign, -4.0f, 4.0f);
    if (physics_environment_cfg.gravity_horizontal_basis_offset < 0) {
        physics_environment_cfg.gravity_horizontal_basis_offset = 0x088;
    }
    if (physics_environment_cfg.gravity_vertical_basis_offset < 0) {
        physics_environment_cfg.gravity_vertical_basis_offset = 0x098;
    }
    if (physics_environment_cfg.gravity_horizontal_secondary_basis_offset < 0) {
        physics_environment_cfg.gravity_horizontal_secondary_basis_offset = 0x078;
    }
    {
        int axis;
        for (axis = 0; axis < 3; axis++) {
            physics_environment_cfg.gravity_horizontal_source_vector[axis] =
                physx_clampf(physics_environment_cfg.gravity_horizontal_source_vector[axis], -4.0f, 4.0f);
            physics_environment_cfg.gravity_vertical_source_vector[axis] =
                physx_clampf(physics_environment_cfg.gravity_vertical_source_vector[axis], -4.0f, 4.0f);
        }
    }

    body_probe_cfg.enabled = profile_bool("body_probe", "enabled", body_probe_cfg.enabled, config_path);
    GetPrivateProfileStringA("body_probe", "person", body_probe_cfg.person, body_probe_cfg.person, sizeof(body_probe_cfg.person), config_path);
    trim_in_place(body_probe_cfg.person);
    GetPrivateProfileStringA("body_probe", "node", body_probe_cfg.node, body_probe_cfg.node, sizeof(body_probe_cfg.node), config_path);
    trim_in_place(body_probe_cfg.node);
    body_probe_cfg.use_object = profile_bool("body_probe", "use_object", body_probe_cfg.use_object, config_path);
    GetPrivateProfileStringA("body_probe", "offset", "", buf, sizeof(buf), config_path);
    trim_in_place(buf);
    body_probe_cfg.offset = parse_offset_value(buf, body_probe_cfg.offset);
    GetPrivateProfileStringA("body_probe", "axis", "", buf, sizeof(buf), config_path);
    trim_in_place(buf);
    if (buf[0]) body_probe_cfg.axis = parse_axis_name(buf);
    body_probe_cfg.amount = profile_float("body_probe", "amount", body_probe_cfg.amount, config_path);
    body_probe_cfg.duration_ms = GetPrivateProfileIntA("body_probe", "duration_ms", body_probe_cfg.duration_ms, config_path);
    if (body_probe_cfg.duration_ms < 100) body_probe_cfg.duration_ms = 100;
    if (body_probe_cfg.duration_ms > 5000) body_probe_cfg.duration_ms = 5000;
    body_probe_cfg.cycle = profile_bool("body_probe", "cycle", body_probe_cfg.cycle, config_path);
    body_probe_cfg.state = 0;
    body_probe_cfg.start_tick = 0;
    body_probe_cfg.raw = NULL;
    body_probe_cfg.runtime[0] = 0;
    body_probe_cfg.cycle_index = 0;

    transform_probe_cfg.enabled = profile_bool("transform_probe", "enabled", transform_probe_cfg.enabled, config_path);
    GetPrivateProfileStringA("transform_probe", "person", transform_probe_cfg.person, transform_probe_cfg.person, sizeof(transform_probe_cfg.person), config_path);
    trim_in_place(transform_probe_cfg.person);
    transform_probe_cfg.interval_ms = GetPrivateProfileIntA("transform_probe", "interval_ms", transform_probe_cfg.interval_ms, config_path);
    if (transform_probe_cfg.interval_ms < 250) transform_probe_cfg.interval_ms = 250;
    if (transform_probe_cfg.interval_ms > 10000) transform_probe_cfg.interval_ms = 10000;
    transform_probe_cfg.scan_floats = GetPrivateProfileIntA("transform_probe", "scan_floats", transform_probe_cfg.scan_floats, config_path);
    if (transform_probe_cfg.scan_floats < 64) transform_probe_cfg.scan_floats = 64;
    if (transform_probe_cfg.scan_floats > 512) transform_probe_cfg.scan_floats = 512;
    transform_probe_cfg.top_count = GetPrivateProfileIntA("transform_probe", "top_count", transform_probe_cfg.top_count, config_path);
    if (transform_probe_cfg.top_count < 1) transform_probe_cfg.top_count = 1;
    if (transform_probe_cfg.top_count > 8) transform_probe_cfg.top_count = 8;
    transform_probe_cfg.include_object = profile_bool("transform_probe", "include_object", transform_probe_cfg.include_object, config_path);
    transform_probe_cfg.threshold = profile_float("transform_probe", "threshold", transform_probe_cfg.threshold, config_path);
    if (transform_probe_cfg.threshold < 0.0001f) transform_probe_cfg.threshold = 0.0001f;
    if (transform_probe_cfg.threshold > 100.0f) transform_probe_cfg.threshold = 100.0f;
    transform_probe_cfg.last_tick = 0;

    axis_map_probe_cfg.enabled = profile_bool("axis_map_probe", "enabled", axis_map_probe_cfg.enabled, config_path);
    GetPrivateProfileStringA("axis_map_probe", "person", axis_map_probe_cfg.person, axis_map_probe_cfg.person, sizeof(axis_map_probe_cfg.person), config_path);
    trim_in_place(axis_map_probe_cfg.person);
    axis_map_probe_cfg.interval_ms = GetPrivateProfileIntA("axis_map_probe", "interval_ms", axis_map_probe_cfg.interval_ms, config_path);
    if (axis_map_probe_cfg.interval_ms < 100) axis_map_probe_cfg.interval_ms = 100;
    if (axis_map_probe_cfg.interval_ms > 5000) axis_map_probe_cfg.interval_ms = 5000;
    axis_map_probe_cfg.threshold = profile_float("axis_map_probe", "threshold", axis_map_probe_cfg.threshold, config_path);
    if (axis_map_probe_cfg.threshold < 0.0f) axis_map_probe_cfg.threshold = 0.0f;
    if (axis_map_probe_cfg.threshold > 10.0f) axis_map_probe_cfg.threshold = 10.0f;
    axis_map_probe_cfg.phase_start_threshold = profile_float("axis_map_probe", "phase_start_threshold", axis_map_probe_cfg.phase_start_threshold, config_path);
    if (axis_map_probe_cfg.phase_start_threshold < axis_map_probe_cfg.threshold) axis_map_probe_cfg.phase_start_threshold = axis_map_probe_cfg.threshold;
    if (axis_map_probe_cfg.phase_start_threshold > 10.0f) axis_map_probe_cfg.phase_start_threshold = 10.0f;
    axis_map_probe_cfg.phase_seconds = GetPrivateProfileIntA("axis_map_probe", "phase_seconds", axis_map_probe_cfg.phase_seconds, config_path);
    if (axis_map_probe_cfg.phase_seconds < 0) axis_map_probe_cfg.phase_seconds = 0;
    if (axis_map_probe_cfg.phase_seconds > 60) axis_map_probe_cfg.phase_seconds = 60;
    axis_map_probe_cfg.phase_start_delay_ms = GetPrivateProfileIntA("axis_map_probe", "phase_start_delay_ms", axis_map_probe_cfg.phase_start_delay_ms, config_path);
    if (axis_map_probe_cfg.phase_start_delay_ms < 0) axis_map_probe_cfg.phase_start_delay_ms = 0;
    if (axis_map_probe_cfg.phase_start_delay_ms > 60000) axis_map_probe_cfg.phase_start_delay_ms = 60000;
    axis_map_probe_cfg.phase_start_tick = 0;
    axis_map_probe_cfg.phase_logged = -1;
    axis_map_probe_cfg.last_tick = 0;

    root_drive_probe_cfg.enabled = profile_bool("root_drive_probe", "enabled", root_drive_probe_cfg.enabled, config_path);
    GetPrivateProfileStringA("root_drive_probe", "person", root_drive_probe_cfg.person, root_drive_probe_cfg.person, sizeof(root_drive_probe_cfg.person), config_path);
    trim_in_place(root_drive_probe_cfg.person);
    GetPrivateProfileStringA("root_drive_probe", "source_node", root_drive_probe_cfg.source_node, root_drive_probe_cfg.source_node, sizeof(root_drive_probe_cfg.source_node), config_path);
    trim_in_place(root_drive_probe_cfg.source_node);
    GetPrivateProfileStringA("root_drive_probe", "target_node", root_drive_probe_cfg.target_node, root_drive_probe_cfg.target_node, sizeof(root_drive_probe_cfg.target_node), config_path);
    trim_in_place(root_drive_probe_cfg.target_node);
    GetPrivateProfileStringA("root_drive_probe", "source_offset", "", buf, sizeof(buf), config_path);
    trim_in_place(buf);
    root_drive_probe_cfg.source_offset = parse_offset_value(buf, root_drive_probe_cfg.source_offset);
    GetPrivateProfileStringA("root_drive_probe", "target_offset", "", buf, sizeof(buf), config_path);
    trim_in_place(buf);
    root_drive_probe_cfg.target_offset = parse_offset_value(buf, root_drive_probe_cfg.target_offset);
    GetPrivateProfileStringA("root_drive_probe", "source_axis", "", buf, sizeof(buf), config_path);
    trim_in_place(buf);
    if (buf[0]) {
        if (_stricmp(buf, "auto") == 0 || _stricmp(buf, "strongest") == 0) {
            root_drive_probe_cfg.source_axis = -1;
        } else {
            root_drive_probe_cfg.source_axis = parse_axis_name(buf);
        }
    }
    GetPrivateProfileStringA("root_drive_probe", "target_axis", "", buf, sizeof(buf), config_path);
    trim_in_place(buf);
    if (buf[0]) root_drive_probe_cfg.target_axis = parse_axis_name(buf);
    root_drive_probe_cfg.scale = profile_float("root_drive_probe", "scale", root_drive_probe_cfg.scale, config_path);
    root_drive_probe_cfg.max_amount = profile_float("root_drive_probe", "max_amount", root_drive_probe_cfg.max_amount, config_path);
    if (root_drive_probe_cfg.max_amount < 0.001f) root_drive_probe_cfg.max_amount = 0.001f;
    if (root_drive_probe_cfg.max_amount > 5.0f) root_drive_probe_cfg.max_amount = 5.0f;
    root_drive_probe_cfg.threshold = profile_float("root_drive_probe", "threshold", root_drive_probe_cfg.threshold, config_path);
    if (root_drive_probe_cfg.threshold < 0.0f) root_drive_probe_cfg.threshold = 0.0f;
    root_drive_probe_cfg.interval_ms = GetPrivateProfileIntA("root_drive_probe", "interval_ms", root_drive_probe_cfg.interval_ms, config_path);
    if (root_drive_probe_cfg.interval_ms < 16) root_drive_probe_cfg.interval_ms = 16;
    if (root_drive_probe_cfg.interval_ms > 5000) root_drive_probe_cfg.interval_ms = 5000;
    root_drive_probe_cfg.last_tick = 0;
    root_drive_probe_cfg.source_raw = NULL;
    root_drive_probe_cfg.target_raw = NULL;
    root_drive_probe_cfg.initialized = 0;
    root_drive_probe_cfg.resolved_logged = 0;

    write_sweep_probe_cfg.enabled = profile_bool("write_sweep_probe", "enabled", write_sweep_probe_cfg.enabled, config_path);
    GetPrivateProfileStringA("write_sweep_probe", "person", write_sweep_probe_cfg.person, write_sweep_probe_cfg.person, sizeof(write_sweep_probe_cfg.person), config_path);
    trim_in_place(write_sweep_probe_cfg.person);
    write_sweep_probe_cfg.duration_ms = GetPrivateProfileIntA("write_sweep_probe", "duration_ms", write_sweep_probe_cfg.duration_ms, config_path);
    if (write_sweep_probe_cfg.duration_ms < 250) write_sweep_probe_cfg.duration_ms = 250;
    if (write_sweep_probe_cfg.duration_ms > 5000) write_sweep_probe_cfg.duration_ms = 5000;
    write_sweep_probe_cfg.start_delay_ms = GetPrivateProfileIntA("write_sweep_probe", "start_delay_ms", write_sweep_probe_cfg.start_delay_ms, config_path);
    if (write_sweep_probe_cfg.start_delay_ms < 0) write_sweep_probe_cfg.start_delay_ms = 0;
    if (write_sweep_probe_cfg.start_delay_ms > 30000) write_sweep_probe_cfg.start_delay_ms = 30000;
    write_sweep_probe_cfg.session_start_tick = 0;
    write_sweep_probe_cfg.delay_logged = 0;
    write_sweep_probe_cfg.start_tick = 0;
    write_sweep_probe_cfg.state = 0;
    write_sweep_probe_cfg.index = 0;
    write_sweep_probe_cfg.base = NULL;

    collision_auto_test_cfg.enabled =
        profile_bool("collision_auto_test", "enabled",
                     collision_auto_test_cfg.enabled, config_path);
    GetPrivateProfileStringA("collision_auto_test", "person",
                             collision_auto_test_cfg.person,
                             collision_auto_test_cfg.person,
                             sizeof(collision_auto_test_cfg.person), config_path);
    trim_in_place(collision_auto_test_cfg.person);
    GetPrivateProfileStringA("collision_auto_test", "mode",
                             collision_auto_test_cfg.mode,
                             collision_auto_test_cfg.mode,
                             sizeof(collision_auto_test_cfg.mode), config_path);
    trim_in_place(collision_auto_test_cfg.mode);
    if (!collision_auto_test_cfg.mode[0]) {
        lstrcpynA(collision_auto_test_cfg.mode, "testicles",
                  sizeof(collision_auto_test_cfg.mode));
    }
    collision_auto_test_cfg.start_delay_ms =
        GetPrivateProfileIntA("collision_auto_test", "start_delay_ms",
                              collision_auto_test_cfg.start_delay_ms, config_path);
    collision_auto_test_cfg.phase_ms =
        GetPrivateProfileIntA("collision_auto_test", "phase_ms",
                              collision_auto_test_cfg.phase_ms, config_path);
    collision_auto_test_cfg.rest_ms =
        GetPrivateProfileIntA("collision_auto_test", "rest_ms",
                              collision_auto_test_cfg.rest_ms, config_path);
    collision_auto_test_cfg.root_amount =
        profile_float("collision_auto_test", "root_amount",
                      collision_auto_test_cfg.root_amount, config_path);
    collision_auto_test_cfg.testicle_amount =
        profile_float("collision_auto_test", "testicle_amount",
                      collision_auto_test_cfg.testicle_amount, config_path);
    collision_auto_test_cfg.hip_amount =
        profile_float("collision_auto_test", "hip_amount",
                      collision_auto_test_cfg.hip_amount, config_path);
    if (collision_auto_test_cfg.start_delay_ms < 1000) {
        collision_auto_test_cfg.start_delay_ms = 1000;
    }
    if (collision_auto_test_cfg.start_delay_ms > 30000) {
        collision_auto_test_cfg.start_delay_ms = 30000;
    }
    if (collision_auto_test_cfg.phase_ms < 500) {
        collision_auto_test_cfg.phase_ms = 500;
    }
    if (collision_auto_test_cfg.phase_ms > 10000) {
        collision_auto_test_cfg.phase_ms = 10000;
    }
    if (collision_auto_test_cfg.rest_ms < 250) {
        collision_auto_test_cfg.rest_ms = 250;
    }
    if (collision_auto_test_cfg.rest_ms > 10000) {
        collision_auto_test_cfg.rest_ms = 10000;
    }
    collision_auto_test_cfg.root_amount =
        physx_clampf(collision_auto_test_cfg.root_amount, 5.0f, 180.0f);
    collision_auto_test_cfg.testicle_amount =
        physx_clampf(collision_auto_test_cfg.testicle_amount, 5.0f, 90.0f);
    collision_auto_test_cfg.hip_amount =
        physx_clampf(collision_auto_test_cfg.hip_amount, 5.0f, 90.0f);
    collision_auto_test_cfg.ready_tick = 0;
    collision_auto_test_cfg.phase_tick = 0;
    collision_auto_test_cfg.phase = 0;
    collision_auto_test_cfg.state = 0;
    collision_auto_test_cfg.completed = 0;
    collision_auto_test_cfg.active_raw = NULL;
    collision_auto_test_cfg.active_offset = -1;
    collision_auto_test_cfg.active_axis = -1;
    collision_auto_test_cfg.active_basis_mode = 0;
    collision_auto_test_cfg.secondary_raw = NULL;
    collision_auto_test_cfg.secondary_offset = -1;

    camera_contamination_test_cfg.active = 0;
    camera_contamination_test_cfg.enabled =
        profile_bool("camera_contamination_test", "enabled",
                     camera_contamination_test_cfg.enabled, config_path);
    GetPrivateProfileStringA("camera_contamination_test", "person",
                             camera_contamination_test_cfg.person,
                             camera_contamination_test_cfg.person,
                             sizeof(camera_contamination_test_cfg.person),
                             config_path);
    trim_in_place(camera_contamination_test_cfg.person);
    camera_contamination_test_cfg.start_delay_ms =
        GetPrivateProfileIntA("camera_contamination_test", "start_delay_ms",
                              camera_contamination_test_cfg.start_delay_ms,
                              config_path);
    camera_contamination_test_cfg.hold_ms =
        GetPrivateProfileIntA("camera_contamination_test", "hold_ms",
                              camera_contamination_test_cfg.hold_ms,
                              config_path);
    camera_contamination_test_cfg.sample_ms =
        GetPrivateProfileIntA("camera_contamination_test", "sample_ms",
                              camera_contamination_test_cfg.sample_ms,
                              config_path);
    camera_contamination_test_cfg.input_drag_ms =
        GetPrivateProfileIntA("camera_contamination_test", "input_drag_ms",
                              camera_contamination_test_cfg.input_drag_ms,
                              config_path);
    camera_contamination_test_cfg.input_yaw_pixels =
        GetPrivateProfileIntA("camera_contamination_test", "input_yaw_pixels",
                              camera_contamination_test_cfg.input_yaw_pixels,
                              config_path);
    camera_contamination_test_cfg.input_pitch_pixels =
        GetPrivateProfileIntA("camera_contamination_test", "input_pitch_pixels",
                              camera_contamination_test_cfg.input_pitch_pixels,
                              config_path);
    camera_contamination_test_cfg.auto_focus =
        profile_bool("camera_contamination_test", "auto_focus",
                     camera_contamination_test_cfg.auto_focus, config_path);
    if (camera_contamination_test_cfg.start_delay_ms < 5000) {
        camera_contamination_test_cfg.start_delay_ms = 5000;
    }
    if (camera_contamination_test_cfg.start_delay_ms > 60000) {
        camera_contamination_test_cfg.start_delay_ms = 60000;
    }
    if (camera_contamination_test_cfg.hold_ms < 1000) {
        camera_contamination_test_cfg.hold_ms = 1000;
    }
    if (camera_contamination_test_cfg.hold_ms > 10000) {
        camera_contamination_test_cfg.hold_ms = 10000;
    }
    if (camera_contamination_test_cfg.sample_ms < 50) {
        camera_contamination_test_cfg.sample_ms = 50;
    }
    if (camera_contamination_test_cfg.sample_ms > 1000) {
        camera_contamination_test_cfg.sample_ms = 1000;
    }
    if (camera_contamination_test_cfg.input_drag_ms < 100) {
        camera_contamination_test_cfg.input_drag_ms = 100;
    }
    if (camera_contamination_test_cfg.input_drag_ms > 3000) {
        camera_contamination_test_cfg.input_drag_ms = 3000;
    }
    if (camera_contamination_test_cfg.input_yaw_pixels < 20) {
        camera_contamination_test_cfg.input_yaw_pixels = 20;
    }
    if (camera_contamination_test_cfg.input_yaw_pixels > 2000) {
        camera_contamination_test_cfg.input_yaw_pixels = 2000;
    }
    if (camera_contamination_test_cfg.input_pitch_pixels < 20) {
        camera_contamination_test_cfg.input_pitch_pixels = 20;
    }
    if (camera_contamination_test_cfg.input_pitch_pixels > 2000) {
        camera_contamination_test_cfg.input_pitch_pixels = 2000;
    }
    camera_contamination_test_cfg.ready_tick = 0;
    camera_contamination_test_cfg.phase_tick = 0;
    camera_contamination_test_cfg.last_sample_tick = 0;
    camera_contamination_test_cfg.input_drag_start_tick = 0;
    camera_contamination_test_cfg.input_drag_last_log_tick = 0;
    camera_contamination_test_cfg.phase = -1;
    camera_contamination_test_cfg.completed = 0;
    camera_contamination_test_cfg.sample_index = 0;
    camera_contamination_test_cfg.input_drag_sent_x = 0;
    camera_contamination_test_cfg.input_drag_sent_y = 0;
    camera_contamination_test_cfg.input_drag_button_down = 0;
    camera_contamination_test_cfg.input_drag_foreground = 0;
    camera_contamination_test_camera_valid = 0;

    body_chain_physics_cfg.enabled = profile_bool(PENIS_PHYSICS_CONFIG_SECTION, "enabled", body_chain_physics_cfg.enabled, config_path);
    body_chain_physics_cfg.wind_enabled =
        profile_bool(PENIS_PHYSICS_CONFIG_SECTION, "wind_enabled",
                     body_chain_physics_cfg.wind_enabled, config_path);
    body_chain_physics_cfg.wind_scale =
        profile_float(PENIS_PHYSICS_CONFIG_SECTION, "wind_scale",
                      body_chain_physics_cfg.wind_scale, config_path);
    body_chain_physics_cfg.enabled_person[0] =
        profile_bool(PENIS_PHYSICS_CONFIG_SECTION, "enabled_person01", body_chain_physics_cfg.enabled_person[0], config_path);
    body_chain_physics_cfg.enabled_person[1] =
        profile_bool(PENIS_PHYSICS_CONFIG_SECTION, "enabled_person02", body_chain_physics_cfg.enabled_person[1], config_path);
    body_chain_physics_cfg.enabled_person[2] =
        profile_bool(PENIS_PHYSICS_CONFIG_SECTION, "enabled_person03", body_chain_physics_cfg.enabled_person[2], config_path);
    body_chain_physics_cfg.enabled_person[3] =
        profile_bool(PENIS_PHYSICS_CONFIG_SECTION, "enabled_person04", body_chain_physics_cfg.enabled_person[3], config_path);
    body_chain_physics_cfg.collision_scope =
        profile_body_chain_collision_scope(
            PENIS_PHYSICS_CONFIG_SECTION,
            body_chain_physics_cfg.collision_scope, config_path);
    body_chain_physics_cfg.room_collision_enabled = profile_bool(
        PENIS_PHYSICS_CONFIG_SECTION, "room_collision_enabled",
        body_chain_physics_cfg.room_collision_enabled, config_path);
    GetPrivateProfileStringA(PENIS_PHYSICS_INTERNAL_CONFIG_SECTION, "root_offset", "", buf, sizeof(buf), config_path);
    trim_in_place(buf);
    body_chain_physics_cfg.root_offset = parse_offset_value(buf, body_chain_physics_cfg.root_offset);
    GetPrivateProfileStringA(PENIS_PHYSICS_INTERNAL_CONFIG_SECTION, "output_offset", "", buf, sizeof(buf), config_path);
    trim_in_place(buf);
    body_chain_physics_cfg.output_offset = parse_offset_value(buf, body_chain_physics_cfg.output_offset);
    GetPrivateProfileStringA(PENIS_PHYSICS_INTERNAL_CONFIG_SECTION, "animation_output_offset", "", buf, sizeof(buf), config_path);
    trim_in_place(buf);
    body_chain_physics_cfg.animation_output_offset = parse_offset_value(buf, body_chain_physics_cfg.animation_output_offset);
    GetPrivateProfileStringA(PENIS_PHYSICS_INTERNAL_CONFIG_SECTION, "animation_override_offsets", "", buf, sizeof(buf), config_path);
    trim_in_place(buf);
    if (buf[0]) {
        int count = parse_offset_list(buf,
                                      body_chain_physics_cfg.animation_override_offsets,
                                      BODY_CHAIN_ANIM_OVERRIDE_MAX_OFFSETS);
        if (count > 0) body_chain_physics_cfg.animation_override_offset_count = count;
    }
    body_chain_physics_cfg.override_animation =
        profile_bool(PENIS_PHYSICS_INTERNAL_CONFIG_SECTION, "override_animation",
                     body_chain_physics_cfg.override_animation, config_path);
    body_chain_physics_cfg.joint01_pose_override =
        profile_bool(PENIS_PHYSICS_INTERNAL_CONFIG_SECTION, "joint01_pose_override",
                     body_chain_physics_cfg.joint01_pose_override, config_path);
    GetPrivateProfileStringA(PENIS_PHYSICS_INTERNAL_CONFIG_SECTION, "joint01_pose_override_offsets", "", buf, sizeof(buf), config_path);
    trim_in_place(buf);
    if (buf[0]) {
        int count = parse_offset_list(buf,
                                      body_chain_physics_cfg.joint01_pose_override_offsets,
                                      BODY_CHAIN_ANIM_OVERRIDE_MAX_OFFSETS);
        if (count > 0) body_chain_physics_cfg.joint01_pose_override_offset_count = count;
    }
    body_chain_physics_cfg.joint01_transform_lock =
        profile_bool(PENIS_PHYSICS_INTERNAL_CONFIG_SECTION, "joint01_transform_lock",
                     body_chain_physics_cfg.joint01_transform_lock, config_path);
    GetPrivateProfileStringA(PENIS_PHYSICS_INTERNAL_CONFIG_SECTION, "joint01_transform_lock_offset", "", buf, sizeof(buf), config_path);
    trim_in_place(buf);
    body_chain_physics_cfg.joint01_transform_lock_offset =
        parse_offset_value(buf, body_chain_physics_cfg.joint01_transform_lock_offset);
    body_chain_physics_cfg.poseeditor_track_override =
        profile_bool(PENIS_PHYSICS_INTERNAL_CONFIG_SECTION, "poseeditor_track_override",
                     body_chain_physics_cfg.poseeditor_track_override, config_path);
    body_chain_physics_cfg.poseeditor_track_diagnostic =
        profile_bool(PENIS_PHYSICS_INTERNAL_CONFIG_SECTION, "poseeditor_track_diagnostic",
                     body_chain_physics_cfg.poseeditor_track_diagnostic, config_path);
    {
        int detected_total_tracks = detect_poseeditor_total_tracks_from_file();
        int configured_total_tracks =
            GetPrivateProfileIntA(PENIS_PHYSICS_INTERNAL_CONFIG_SECTION, "poseeditor_total_tracks",
                                  detected_total_tracks > 0 ? detected_total_tracks : body_chain_physics_cfg.poseeditor_total_tracks,
                                  config_path);
        body_chain_physics_cfg.poseeditor_total_tracks = configured_total_tracks;
        if (body_chain_physics_cfg.poseeditor_total_tracks < POSEEDIT_TRACK_TESTICLES_JOINT02 + 1) {
            body_chain_physics_cfg.poseeditor_total_tracks = detected_total_tracks > 0 ? detected_total_tracks : POSEEDIT_FALLBACK_TOTAL_TRACKS;
        }
        if (body_chain_physics_cfg.poseeditor_total_tracks < POSEEDIT_TRACK_TESTICLES_JOINT02 + 1) {
            body_chain_physics_cfg.poseeditor_total_tracks = POSEEDIT_TRACK_TESTICLES_JOINT02 + 1;
        }
        if (body_chain_physics_cfg.poseeditor_total_tracks > 4096) {
            body_chain_physics_cfg.poseeditor_total_tracks = 4096;
        }
    }
    body_chain_physics_cfg.translation_source_axis[0] =
        profile_axis(PENIS_PHYSICS_INTERNAL_CONFIG_SECTION,
                     "translation_horizontal_source_axis",
                     body_chain_physics_cfg.translation_source_axis[0], config_path);
    body_chain_physics_cfg.translation_tail_axis[0] =
        profile_axis(PENIS_PHYSICS_INTERNAL_CONFIG_SECTION,
                     "translation_horizontal_tail_axis",
                     body_chain_physics_cfg.translation_tail_axis[0], config_path);
    body_chain_physics_cfg.translation_source_axis[1] =
        profile_axis(PENIS_PHYSICS_INTERNAL_CONFIG_SECTION,
                     "translation_vertical_source_axis",
                     body_chain_physics_cfg.translation_source_axis[1], config_path);
    body_chain_physics_cfg.translation_tail_axis[1] =
        profile_axis(PENIS_PHYSICS_INTERNAL_CONFIG_SECTION,
                     "translation_vertical_tail_axis",
                     body_chain_physics_cfg.translation_tail_axis[1], config_path);
    body_chain_physics_cfg.translation_source_axis[2] =
        profile_axis(PENIS_PHYSICS_INTERNAL_CONFIG_SECTION,
                     "translation_depth_source_axis",
                     body_chain_physics_cfg.translation_source_axis[2], config_path);
    body_chain_physics_cfg.translation_tail_axis[2] =
        profile_axis(PENIS_PHYSICS_INTERNAL_CONFIG_SECTION,
                     "translation_depth_tail_axis",
                     body_chain_physics_cfg.translation_tail_axis[2], config_path);
    body_chain_physics_cfg.rotation_source_axis[0] =
        profile_axis(PENIS_PHYSICS_INTERNAL_CONFIG_SECTION,
                     "rotation_horizontal_source_axis",
                     body_chain_physics_cfg.rotation_source_axis[0], config_path);
    body_chain_physics_cfg.rotation_tail_axis[0] =
        profile_axis(PENIS_PHYSICS_INTERNAL_CONFIG_SECTION,
                     "rotation_horizontal_tail_axis",
                     body_chain_physics_cfg.rotation_tail_axis[0], config_path);
    body_chain_physics_cfg.rotation_source_axis[1] =
        profile_axis(PENIS_PHYSICS_INTERNAL_CONFIG_SECTION,
                     "rotation_vertical_source_axis",
                     body_chain_physics_cfg.rotation_source_axis[1], config_path);
    body_chain_physics_cfg.rotation_tail_axis[1] =
        profile_axis(PENIS_PHYSICS_INTERNAL_CONFIG_SECTION,
                     "rotation_vertical_tail_axis",
                     body_chain_physics_cfg.rotation_tail_axis[1], config_path);
    body_chain_physics_cfg.rotation_source_axis[2] =
        profile_axis(PENIS_PHYSICS_INTERNAL_CONFIG_SECTION,
                     "rotation_twist_source_axis",
                     body_chain_physics_cfg.rotation_source_axis[2], config_path);
    body_chain_physics_cfg.rotation_tail_axis[2] =
        profile_axis(PENIS_PHYSICS_INTERNAL_CONFIG_SECTION,
                     "rotation_twist_tail_axis",
                     body_chain_physics_cfg.rotation_tail_axis[2], config_path);
    load_body_wind_axis_mapping(
        &body_chain_physics_cfg,
        PENIS_PHYSICS_INTERNAL_CONFIG_SECTION, config_path);
    body_chain_physics_cfg.gravity_inverted_strength =
        profile_float(PENIS_PHYSICS_INTERNAL_CONFIG_SECTION,
                      "gravity_inverted_strength",
                      body_chain_physics_cfg.gravity_inverted_strength,
                      config_path);
    body_chain_physics_cfg.gravity_inverted_tail_axis =
        profile_axis(PENIS_PHYSICS_INTERNAL_CONFIG_SECTION,
                     "gravity_inverted_tail_axis",
                     body_chain_physics_cfg.gravity_inverted_tail_axis,
                     config_path);
    body_chain_physics_cfg.gravity_inverted_sign =
        profile_float(PENIS_PHYSICS_INTERNAL_CONFIG_SECTION,
                      "gravity_inverted_sign",
                      body_chain_physics_cfg.gravity_inverted_sign,
                      config_path);
    body_chain_physics_cfg.chain_total_bend_max =
        profile_float(PENIS_PHYSICS_INTERNAL_CONFIG_SECTION,
                      "chain_total_bend_max",
                      body_chain_physics_cfg.chain_total_bend_max,
                      config_path);
    body_chain_physics_cfg.chain_total_twist_max =
        profile_float(PENIS_PHYSICS_INTERNAL_CONFIG_SECTION,
                      "chain_total_twist_max",
                      body_chain_physics_cfg.chain_total_twist_max,
                      config_path);
    body_chain_physics_cfg.translation_deadzone =
        profile_float(PENIS_PHYSICS_INTERNAL_CONFIG_SECTION,
                      "translation_deadzone",
                      body_chain_physics_cfg.translation_deadzone, config_path);
    body_chain_physics_cfg.rotation_deadzone =
        profile_float(PENIS_PHYSICS_INTERNAL_CONFIG_SECTION,
                      "rotation_deadzone",
                      body_chain_physics_cfg.rotation_deadzone, config_path);
    body_chain_physics_cfg.translation_scale[0] =
        profile_float(PENIS_PHYSICS_CONFIG_SECTION,
                      "translation_horizontal_scale",
                      body_chain_physics_cfg.translation_scale[0], config_path);
    body_chain_physics_cfg.translation_scale[1] =
        profile_float(PENIS_PHYSICS_CONFIG_SECTION,
                      "translation_vertical_scale",
                      body_chain_physics_cfg.translation_scale[1], config_path);
    body_chain_physics_cfg.translation_scale[2] =
        profile_float(PENIS_PHYSICS_CONFIG_SECTION,
                      "translation_depth_scale",
                      body_chain_physics_cfg.translation_scale[2], config_path);
    body_chain_physics_cfg.face_down_translation_channel =
        profile_translation_channel(
            PENIS_PHYSICS_CONFIG_SECTION,
            "face_down_translation_channel",
            body_chain_physics_cfg.face_down_translation_channel,
            config_path);
    body_chain_physics_cfg.face_down_translation_sign =
        physx_clampf(
            profile_float(PENIS_PHYSICS_CONFIG_SECTION,
                          "face_down_translation_sign",
                          body_chain_physics_cfg.face_down_translation_sign,
                          config_path),
            -1.0f, 1.0f);
    body_chain_physics_cfg.rotation_scale[0] =
        profile_float(PENIS_PHYSICS_CONFIG_SECTION,
                      "rotation_horizontal_scale",
                      body_chain_physics_cfg.rotation_scale[0], config_path);
    body_chain_physics_cfg.rotation_scale[1] =
        profile_float(PENIS_PHYSICS_CONFIG_SECTION,
                      "rotation_vertical_scale",
                      body_chain_physics_cfg.rotation_scale[1], config_path);
    body_chain_physics_cfg.rotation_scale[2] =
        profile_float(PENIS_PHYSICS_CONFIG_SECTION,
                      "rotation_twist_scale",
                      body_chain_physics_cfg.rotation_scale[2], config_path);

    /* Collision geometry needs two independent bend axes. Drive mappings may
       legitimately target the same axis, so do not derive collision from them. */
    body_chain_physics_cfg.horizontal_source_axis =
        body_chain_physics_cfg.translation_source_axis[0];
    body_chain_physics_cfg.vertical_source_axis =
        body_chain_physics_cfg.translation_source_axis[1];
    body_chain_physics_cfg.horizontal_output_axis =
        profile_axis(PENIS_PHYSICS_INTERNAL_CONFIG_SECTION,
                     "collision_horizontal_tail_axis",
                     body_chain_physics_cfg.horizontal_output_axis,
                     config_path);
    body_chain_physics_cfg.vertical_output_axis =
        profile_axis(PENIS_PHYSICS_INTERNAL_CONFIG_SECTION,
                     "collision_vertical_tail_axis",
                     body_chain_physics_cfg.vertical_output_axis,
                     config_path);
    if (body_chain_physics_cfg.horizontal_output_axis ==
        body_chain_physics_cfg.vertical_output_axis) {
        log_line("invalid built-in collision axis mapping horizontal=%d vertical=%d; using calibrated z/y axes",
                 body_chain_physics_cfg.horizontal_output_axis,
                 body_chain_physics_cfg.vertical_output_axis);
        body_chain_physics_cfg.horizontal_output_axis = 2;
        body_chain_physics_cfg.vertical_output_axis = 1;
    }
    {
        int axis;
        for (axis = 0; axis < 3; axis++) {
            body_chain_physics_cfg.horizontal_source_vector[axis] = 0.0f;
            body_chain_physics_cfg.vertical_source_vector[axis] = 0.0f;
        }
        body_chain_physics_cfg.horizontal_source_vector[body_chain_physics_cfg.horizontal_source_axis] = 1.0f;
        body_chain_physics_cfg.vertical_source_vector[body_chain_physics_cfg.vertical_source_axis] = 1.0f;
    }
    body_chain_physics_cfg.horizontal_secondary_output_axis = -1;
    body_chain_physics_cfg.horizontal_secondary_output_scale = 0.0f;
    body_chain_physics_cfg.horizontal_sign = 1.0f;
    body_chain_physics_cfg.vertical_sign = 1.0f;
    body_chain_physics_cfg.horizontal_deadzone = body_chain_physics_cfg.translation_deadzone;
    body_chain_physics_cfg.vertical_deadzone = body_chain_physics_cfg.translation_deadzone;
    body_chain_physics_cfg.gravity_angle = profile_float(PENIS_PHYSICS_CONFIG_SECTION, "gravity_angle", body_chain_physics_cfg.gravity_angle, config_path);
    body_chain_physics_cfg.gravity_horizontal_curve =
        profile_float(PENIS_PHYSICS_CONFIG_SECTION,
                      "gravity_horizontal_curve",
                      profile_float(PENIS_PHYSICS_CONFIG_SECTION,
                                    "gravity_curve",
                                    body_chain_physics_cfg
                                        .gravity_horizontal_curve,
                                    config_path),
                      config_path);
    body_chain_physics_cfg.gravity_vertical_curve =
        profile_float(PENIS_PHYSICS_CONFIG_SECTION,
                      "gravity_vertical_curve",
                      profile_float(PENIS_PHYSICS_CONFIG_SECTION,
                                    "gravity_curve",
                                    body_chain_physics_cfg
                                        .gravity_vertical_curve,
                                    config_path),
                      config_path);
    body_chain_physics_cfg.stiffness = profile_float(PENIS_PHYSICS_CONFIG_SECTION, "stiffness", body_chain_physics_cfg.stiffness, config_path);
    body_chain_physics_cfg.damping = profile_float(PENIS_PHYSICS_CONFIG_SECTION, "damping", body_chain_physics_cfg.damping, config_path);
    body_chain_physics_cfg.max_angle = profile_float(PENIS_PHYSICS_CONFIG_SECTION, "max_angle", body_chain_physics_cfg.max_angle, config_path);
    profile_vec3_or_float(PENIS_PHYSICS_CONFIG_SECTION, "joint01_max_angle",
                          body_chain_physics_cfg.max_angle,
                          body_chain_physics_cfg.link_max_angle[0],
                          config_path);
    profile_min_angle_vec3_or_float(PENIS_PHYSICS_CONFIG_SECTION, "joint01_min_angle",
                                    body_chain_physics_cfg.link_max_angle[0],
                                    body_chain_physics_cfg.link_min_angle[0],
                                    config_path);
    profile_vec3_or_float(PENIS_PHYSICS_CONFIG_SECTION, "joint02_max_angle",
                          body_chain_physics_cfg.max_angle,
                          body_chain_physics_cfg.link_max_angle[1],
                          config_path);
    profile_min_angle_vec3_or_float(PENIS_PHYSICS_CONFIG_SECTION, "joint02_min_angle",
                                    body_chain_physics_cfg.link_max_angle[1],
                                    body_chain_physics_cfg.link_min_angle[1],
                                    config_path);
    profile_vec3_or_float(PENIS_PHYSICS_CONFIG_SECTION, "joint03_max_angle",
                          body_chain_physics_cfg.max_angle,
                          body_chain_physics_cfg.link_max_angle[2],
                          config_path);
    profile_min_angle_vec3_or_float(PENIS_PHYSICS_CONFIG_SECTION, "joint03_min_angle",
                                    body_chain_physics_cfg.link_max_angle[2],
                                    body_chain_physics_cfg.link_min_angle[2],
                                    config_path);
    body_chain_physics_cfg.link_gain[0] = profile_float(PENIS_PHYSICS_CONFIG_SECTION, "joint01_gain", body_chain_physics_cfg.link_gain[0], config_path);
    body_chain_physics_cfg.link_gain[1] = profile_float(PENIS_PHYSICS_CONFIG_SECTION, "joint02_gain", body_chain_physics_cfg.link_gain[1], config_path);
    body_chain_physics_cfg.link_gain[2] = profile_float(PENIS_PHYSICS_CONFIG_SECTION, "joint03_gain", body_chain_physics_cfg.link_gain[2], config_path);
    body_chain_physics_cfg.interval_ms = GetPrivateProfileIntA(PENIS_PHYSICS_CONFIG_SECTION, "interval_ms", body_chain_physics_cfg.interval_ms, config_path);
    body_chain_physics_cfg.zero_output_rest = profile_bool(PENIS_PHYSICS_INTERNAL_CONFIG_SECTION, "zero_output_rest", body_chain_physics_cfg.zero_output_rest, config_path);
    if (body_chain_physics_cfg.interval_ms < 16) body_chain_physics_cfg.interval_ms = 16;
    if (body_chain_physics_cfg.interval_ms > 1000) body_chain_physics_cfg.interval_ms = 1000;
    if (body_chain_physics_cfg.root_offset < 0) body_chain_physics_cfg.root_offset = 0x0e8;
    if (body_chain_physics_cfg.output_offset < 0) body_chain_physics_cfg.output_offset = 0x06c;
    if (body_chain_physics_cfg.animation_output_offset < 0) body_chain_physics_cfg.animation_output_offset = body_chain_physics_cfg.output_offset;
    if (body_chain_physics_cfg.animation_override_offset_count <= 0 ||
        body_chain_physics_cfg.animation_override_offset_count > BODY_CHAIN_ANIM_OVERRIDE_MAX_OFFSETS) {
        body_chain_physics_cfg.animation_override_offsets[0] = body_chain_physics_cfg.animation_output_offset;
        body_chain_physics_cfg.animation_override_offset_count = 1;
    }
    if (body_chain_physics_cfg.joint01_pose_override_offset_count <= 0 ||
        body_chain_physics_cfg.joint01_pose_override_offset_count > BODY_CHAIN_ANIM_OVERRIDE_MAX_OFFSETS) {
        body_chain_physics_cfg.joint01_pose_override_offsets[0] = 0x014;
        body_chain_physics_cfg.joint01_pose_override_offsets[1] = 0x018;
        body_chain_physics_cfg.joint01_pose_override_offsets[2] = 0x024;
        body_chain_physics_cfg.joint01_pose_override_offset_count = 3;
    }
    if (body_chain_physics_cfg.joint01_transform_lock_offset < 0) {
        body_chain_physics_cfg.joint01_transform_lock_offset = 0x078;
    }
    if (body_chain_physics_cfg.horizontal_output_axis < 0 || body_chain_physics_cfg.horizontal_output_axis > 2) body_chain_physics_cfg.horizontal_output_axis = 0;
    if (body_chain_physics_cfg.vertical_output_axis < 0 || body_chain_physics_cfg.vertical_output_axis > 2) body_chain_physics_cfg.vertical_output_axis = 1;
    if (body_chain_physics_cfg.horizontal_secondary_output_axis < -1 ||
        body_chain_physics_cfg.horizontal_secondary_output_axis > 2) {
        body_chain_physics_cfg.horizontal_secondary_output_axis = -1;
    }
    body_chain_physics_cfg.horizontal_secondary_output_scale =
        physx_clampf(body_chain_physics_cfg.horizontal_secondary_output_scale, -4.0f, 4.0f);
    if (body_chain_physics_cfg.horizontal_drive_scale == 0.0f) {
        body_chain_physics_cfg.horizontal_drive_scale = body_chain_physics_cfg.drive_scale;
    }
    if (body_chain_physics_cfg.vertical_drive_scale == 0.0f) {
        body_chain_physics_cfg.vertical_drive_scale = body_chain_physics_cfg.drive_scale;
    }
    body_chain_physics_cfg.horizontal_drive_scale =
        physx_clampf(body_chain_physics_cfg.horizontal_drive_scale, -5000.0f, 5000.0f);
    body_chain_physics_cfg.vertical_drive_scale =
        physx_clampf(body_chain_physics_cfg.vertical_drive_scale, -5000.0f, 5000.0f);
    body_chain_physics_cfg.horizontal_deadzone =
        physx_clampf(body_chain_physics_cfg.horizontal_deadzone, 0.0f, 1.0f);
    body_chain_physics_cfg.vertical_deadzone =
        physx_clampf(body_chain_physics_cfg.vertical_deadzone, 0.0f, 1.0f);
    {
        int axis;
        for (axis = 0; axis < 3; axis++) {
            body_chain_physics_cfg.translation_scale[axis] =
                physx_clampf(body_chain_physics_cfg.translation_scale[axis],
                             -10.0f, 10.0f);
            body_chain_physics_cfg.rotation_scale[axis] =
                physx_clampf(body_chain_physics_cfg.rotation_scale[axis],
                             -10.0f, 10.0f);
            body_chain_physics_cfg.horizontal_source_vector[axis] =
                physx_clampf(body_chain_physics_cfg.horizontal_source_vector[axis], -4.0f, 4.0f);
            body_chain_physics_cfg.vertical_source_vector[axis] =
                physx_clampf(body_chain_physics_cfg.vertical_source_vector[axis], -4.0f, 4.0f);
        }
    }
    body_chain_physics_cfg.translation_deadzone =
        physx_clampf(body_chain_physics_cfg.translation_deadzone, 0.0f, 1.0f);
    body_chain_physics_cfg.rotation_deadzone =
        physx_clampf(body_chain_physics_cfg.rotation_deadzone, 0.0f, 45.0f);
    body_chain_physics_cfg.gravity_horizontal_curve =
        physx_clampf(body_chain_physics_cfg.gravity_horizontal_curve,
                     0.1f, 8.0f);
    body_chain_physics_cfg.gravity_vertical_curve =
        physx_clampf(body_chain_physics_cfg.gravity_vertical_curve,
                     0.1f, 8.0f);
    body_chain_physics_cfg.gravity_inverted_strength =
        physx_clampf(body_chain_physics_cfg.gravity_inverted_strength,
                     0.0f, 120.0f);
    body_chain_physics_cfg.gravity_inverted_sign =
        physx_clampf(body_chain_physics_cfg.gravity_inverted_sign,
                     -1.0f, 1.0f);
    body_chain_physics_cfg.chain_total_bend_max =
        physx_clampf(body_chain_physics_cfg.chain_total_bend_max,
                     0.0f, 360.0f);
    body_chain_physics_cfg.chain_total_twist_max =
        physx_clampf(body_chain_physics_cfg.chain_total_twist_max,
                     0.0f, 360.0f);
    body_chain_physics_cfg.max_angle = physx_clampf(body_chain_physics_cfg.max_angle, 1.0f, 360.0f);
    clamp_angle_range_vec3(body_chain_physics_cfg.link_min_angle[0],
                           body_chain_physics_cfg.link_max_angle[0]);
    clamp_angle_range_vec3(body_chain_physics_cfg.link_min_angle[1],
                           body_chain_physics_cfg.link_max_angle[1]);
    clamp_angle_range_vec3(body_chain_physics_cfg.link_min_angle[2],
                           body_chain_physics_cfg.link_max_angle[2]);
    body_chain_physics_cfg.stiffness = physx_clampf(body_chain_physics_cfg.stiffness, 1.0f, 200.0f);
    body_chain_physics_cfg.damping = physx_clampf(body_chain_physics_cfg.damping, 0.0f, 80.0f);

    testicle_physics_cfg.root_offset = body_chain_physics_cfg.root_offset;
    testicle_physics_cfg.output_offset = body_chain_physics_cfg.output_offset;
    testicle_physics_cfg.animation_output_offset =
        body_chain_physics_cfg.animation_output_offset;
    memcpy(testicle_physics_cfg.animation_override_offsets,
           body_chain_physics_cfg.animation_override_offsets,
           sizeof(testicle_physics_cfg.animation_override_offsets));
    testicle_physics_cfg.animation_override_offset_count =
        body_chain_physics_cfg.animation_override_offset_count;
    testicle_physics_cfg.override_animation =
        body_chain_physics_cfg.override_animation;
    testicle_physics_cfg.poseeditor_track_override =
        body_chain_physics_cfg.poseeditor_track_override;
    testicle_physics_cfg.poseeditor_track_diagnostic =
        body_chain_physics_cfg.poseeditor_track_diagnostic;
    testicle_physics_cfg.poseeditor_total_tracks =
        body_chain_physics_cfg.poseeditor_total_tracks;
    memcpy(testicle_physics_cfg.translation_source_axis,
           body_chain_physics_cfg.translation_source_axis,
           sizeof(testicle_physics_cfg.translation_source_axis));
    memcpy(testicle_physics_cfg.translation_tail_axis,
           body_chain_physics_cfg.translation_tail_axis,
           sizeof(testicle_physics_cfg.translation_tail_axis));
    memcpy(testicle_physics_cfg.rotation_source_axis,
           body_chain_physics_cfg.rotation_source_axis,
           sizeof(testicle_physics_cfg.rotation_source_axis));
    memcpy(testicle_physics_cfg.rotation_tail_axis,
           body_chain_physics_cfg.rotation_tail_axis,
           sizeof(testicle_physics_cfg.rotation_tail_axis));
    memcpy(testicle_physics_cfg.wind_source_axis,
           body_chain_physics_cfg.wind_source_axis,
           sizeof(testicle_physics_cfg.wind_source_axis));
    memcpy(testicle_physics_cfg.wind_tail_axis,
           body_chain_physics_cfg.wind_tail_axis,
           sizeof(testicle_physics_cfg.wind_tail_axis));
    memcpy(testicle_physics_cfg.wind_axis_scale,
           body_chain_physics_cfg.wind_axis_scale,
           sizeof(testicle_physics_cfg.wind_axis_scale));
    load_body_wind_axis_mapping(
        &testicle_physics_cfg,
        TESTICLE_PHYSICS_INTERNAL_CONFIG_SECTION, config_path);
    testicle_physics_cfg.gravity_inverted_strength =
        profile_float(TESTICLE_PHYSICS_INTERNAL_CONFIG_SECTION,
                      "gravity_inverted_strength",
                      testicle_physics_cfg.gravity_inverted_strength,
                      config_path);
    testicle_physics_cfg.gravity_inverted_tail_axis =
        profile_axis(TESTICLE_PHYSICS_INTERNAL_CONFIG_SECTION,
                     "gravity_inverted_tail_axis",
                     testicle_physics_cfg.gravity_inverted_tail_axis,
                     config_path);
    testicle_physics_cfg.gravity_inverted_sign =
        profile_float(TESTICLE_PHYSICS_INTERNAL_CONFIG_SECTION,
                      "gravity_inverted_sign",
                      testicle_physics_cfg.gravity_inverted_sign,
                      config_path);
    testicle_physics_cfg.chain_total_bend_max =
        profile_float(TESTICLE_PHYSICS_INTERNAL_CONFIG_SECTION,
                      "chain_total_bend_max",
                      testicle_physics_cfg.chain_total_bend_max,
                      config_path);
    testicle_physics_cfg.chain_total_twist_max =
        profile_float(TESTICLE_PHYSICS_INTERNAL_CONFIG_SECTION,
                      "chain_total_twist_max",
                      testicle_physics_cfg.chain_total_twist_max,
                      config_path);
    testicle_physics_cfg.translation_deadzone =
        body_chain_physics_cfg.translation_deadzone;
    testicle_physics_cfg.rotation_deadzone =
        body_chain_physics_cfg.rotation_deadzone;
    testicle_physics_cfg.horizontal_source_axis =
        body_chain_physics_cfg.horizontal_source_axis;
    testicle_physics_cfg.vertical_source_axis =
        body_chain_physics_cfg.vertical_source_axis;
    memcpy(testicle_physics_cfg.horizontal_source_vector,
           body_chain_physics_cfg.horizontal_source_vector,
           sizeof(testicle_physics_cfg.horizontal_source_vector));
    memcpy(testicle_physics_cfg.vertical_source_vector,
           body_chain_physics_cfg.vertical_source_vector,
           sizeof(testicle_physics_cfg.vertical_source_vector));
    testicle_physics_cfg.horizontal_output_axis =
        body_chain_physics_cfg.horizontal_output_axis;
    testicle_physics_cfg.vertical_output_axis =
        body_chain_physics_cfg.vertical_output_axis;
    testicle_physics_cfg.horizontal_secondary_output_axis =
        body_chain_physics_cfg.horizontal_secondary_output_axis;
    testicle_physics_cfg.horizontal_secondary_output_scale =
        body_chain_physics_cfg.horizontal_secondary_output_scale;
    testicle_physics_cfg.horizontal_sign =
        body_chain_physics_cfg.horizontal_sign;
    testicle_physics_cfg.vertical_sign =
        body_chain_physics_cfg.vertical_sign;
    testicle_physics_cfg.horizontal_deadzone =
        body_chain_physics_cfg.horizontal_deadzone;
    testicle_physics_cfg.vertical_deadzone =
        body_chain_physics_cfg.vertical_deadzone;
    testicle_physics_cfg.zero_output_rest =
        body_chain_physics_cfg.zero_output_rest;
    testicle_physics_cfg.enabled =
        profile_bool(TESTICLE_PHYSICS_CONFIG_SECTION, "enabled",
                     testicle_physics_cfg.enabled, config_path);
    testicle_physics_cfg.wind_enabled =
        profile_bool(TESTICLE_PHYSICS_CONFIG_SECTION, "wind_enabled",
                     testicle_physics_cfg.wind_enabled, config_path);
    testicle_physics_cfg.wind_scale =
        profile_float(TESTICLE_PHYSICS_CONFIG_SECTION, "wind_scale",
                      testicle_physics_cfg.wind_scale, config_path);
    testicle_physics_cfg.enabled_person[0] =
        profile_bool(TESTICLE_PHYSICS_CONFIG_SECTION, "enabled_person01",
                     testicle_physics_cfg.enabled_person[0], config_path);
    testicle_physics_cfg.enabled_person[1] =
        profile_bool(TESTICLE_PHYSICS_CONFIG_SECTION, "enabled_person02",
                     testicle_physics_cfg.enabled_person[1], config_path);
    testicle_physics_cfg.enabled_person[2] =
        profile_bool(TESTICLE_PHYSICS_CONFIG_SECTION, "enabled_person03",
                     testicle_physics_cfg.enabled_person[2], config_path);
    testicle_physics_cfg.enabled_person[3] =
        profile_bool(TESTICLE_PHYSICS_CONFIG_SECTION, "enabled_person04",
                     testicle_physics_cfg.enabled_person[3], config_path);
    testicle_physics_cfg.collision_scope =
        profile_body_chain_collision_scope(
            TESTICLE_PHYSICS_CONFIG_SECTION,
            testicle_physics_cfg.collision_scope, config_path);
    testicle_physics_cfg.room_collision_enabled = profile_bool(
        TESTICLE_PHYSICS_CONFIG_SECTION, "room_collision_enabled",
        testicle_physics_cfg.room_collision_enabled, config_path);
    testicle_physics_cfg.translation_scale[0] =
        profile_float(TESTICLE_PHYSICS_CONFIG_SECTION,
                      "translation_horizontal_scale",
                      testicle_physics_cfg.translation_scale[0], config_path);
    testicle_physics_cfg.translation_scale[1] =
        profile_float(TESTICLE_PHYSICS_CONFIG_SECTION,
                      "translation_vertical_scale",
                      testicle_physics_cfg.translation_scale[1], config_path);
    testicle_physics_cfg.translation_scale[2] =
        profile_float(TESTICLE_PHYSICS_CONFIG_SECTION,
                      "translation_depth_scale",
                      testicle_physics_cfg.translation_scale[2], config_path);
    testicle_physics_cfg.face_down_translation_channel =
        profile_translation_channel(
            TESTICLE_PHYSICS_CONFIG_SECTION,
            "face_down_translation_channel",
            testicle_physics_cfg.face_down_translation_channel,
            config_path);
    testicle_physics_cfg.face_down_translation_sign =
        physx_clampf(
            profile_float(TESTICLE_PHYSICS_CONFIG_SECTION,
                          "face_down_translation_sign",
                          testicle_physics_cfg.face_down_translation_sign,
                          config_path),
            -1.0f, 1.0f);
    testicle_physics_cfg.rotation_scale[0] =
        profile_float(TESTICLE_PHYSICS_CONFIG_SECTION,
                      "rotation_horizontal_scale",
                      testicle_physics_cfg.rotation_scale[0], config_path);
    testicle_physics_cfg.rotation_scale[1] =
        profile_float(TESTICLE_PHYSICS_CONFIG_SECTION,
                      "rotation_vertical_scale",
                      testicle_physics_cfg.rotation_scale[1], config_path);
    testicle_physics_cfg.rotation_scale[2] =
        profile_float(TESTICLE_PHYSICS_CONFIG_SECTION,
                      "rotation_twist_scale",
                      testicle_physics_cfg.rotation_scale[2], config_path);
    testicle_physics_cfg.stiffness =
        profile_float(TESTICLE_PHYSICS_CONFIG_SECTION, "stiffness",
                      testicle_physics_cfg.stiffness, config_path);
    testicle_physics_cfg.damping =
        profile_float(TESTICLE_PHYSICS_CONFIG_SECTION, "damping",
                      testicle_physics_cfg.damping, config_path);
    testicle_physics_cfg.gravity_angle =
        profile_float(TESTICLE_PHYSICS_CONFIG_SECTION, "gravity_angle",
                      testicle_physics_cfg.gravity_angle, config_path);
    testicle_physics_cfg.gravity_horizontal_curve =
        profile_float(TESTICLE_PHYSICS_CONFIG_SECTION,
                      "gravity_horizontal_curve",
                      profile_float(TESTICLE_PHYSICS_CONFIG_SECTION,
                                    "gravity_curve",
                                    testicle_physics_cfg
                                        .gravity_horizontal_curve,
                                    config_path),
                      config_path);
    testicle_physics_cfg.gravity_vertical_curve =
        profile_float(TESTICLE_PHYSICS_CONFIG_SECTION,
                      "gravity_vertical_curve",
                      profile_float(TESTICLE_PHYSICS_CONFIG_SECTION,
                                    "gravity_curve",
                                    testicle_physics_cfg
                                        .gravity_vertical_curve,
                                    config_path),
                      config_path);
    profile_vec3_or_float(TESTICLE_PHYSICS_CONFIG_SECTION, "joint01_max_angle",
                          testicle_physics_cfg.max_angle,
                          testicle_physics_cfg.link_max_angle[0],
                          config_path);
    profile_min_angle_vec3_or_float(TESTICLE_PHYSICS_CONFIG_SECTION,
                                    "joint01_min_angle",
                                    testicle_physics_cfg.link_max_angle[0],
                                    testicle_physics_cfg.link_min_angle[0],
                                    config_path);
    profile_vec3_or_float(TESTICLE_PHYSICS_CONFIG_SECTION, "joint02_max_angle",
                          testicle_physics_cfg.max_angle,
                          testicle_physics_cfg.link_max_angle[1],
                          config_path);
    profile_min_angle_vec3_or_float(TESTICLE_PHYSICS_CONFIG_SECTION,
                                    "joint02_min_angle",
                                    testicle_physics_cfg.link_max_angle[1],
                                    testicle_physics_cfg.link_min_angle[1],
                                    config_path);
    testicle_physics_cfg.link_gain[0] =
        profile_float(TESTICLE_PHYSICS_CONFIG_SECTION, "joint01_gain",
                      testicle_physics_cfg.link_gain[0], config_path);
    testicle_physics_cfg.link_gain[1] =
        profile_float(TESTICLE_PHYSICS_CONFIG_SECTION, "joint02_gain",
                      testicle_physics_cfg.link_gain[1], config_path);
    testicle_physics_cfg.interval_ms =
        GetPrivateProfileIntA(TESTICLE_PHYSICS_CONFIG_SECTION, "interval_ms",
                              testicle_physics_cfg.interval_ms, config_path);
    if (testicle_physics_cfg.interval_ms < 16) testicle_physics_cfg.interval_ms = 16;
    if (testicle_physics_cfg.interval_ms > 1000) testicle_physics_cfg.interval_ms = 1000;
    testicle_physics_cfg.horizontal_drive_scale =
        physx_clampf(testicle_physics_cfg.horizontal_drive_scale, -5000.0f, 5000.0f);
    testicle_physics_cfg.vertical_drive_scale =
        physx_clampf(testicle_physics_cfg.vertical_drive_scale, -5000.0f, 5000.0f);
    {
        int axis;
        for (axis = 0; axis < 3; axis++) {
            testicle_physics_cfg.translation_scale[axis] =
                physx_clampf(testicle_physics_cfg.translation_scale[axis],
                             -10.0f, 10.0f);
            testicle_physics_cfg.rotation_scale[axis] =
                physx_clampf(testicle_physics_cfg.rotation_scale[axis],
                             -10.0f, 10.0f);
        }
    }
    clamp_angle_range_vec3(testicle_physics_cfg.link_min_angle[0],
                           testicle_physics_cfg.link_max_angle[0]);
    clamp_angle_range_vec3(testicle_physics_cfg.link_min_angle[1],
                           testicle_physics_cfg.link_max_angle[1]);
    clamp_angle_range_vec3(testicle_physics_cfg.link_min_angle[2],
                           testicle_physics_cfg.link_max_angle[2]);
    testicle_physics_cfg.stiffness =
        physx_clampf(testicle_physics_cfg.stiffness, 1.0f, 200.0f);
    testicle_physics_cfg.damping =
        physx_clampf(testicle_physics_cfg.damping, 0.0f, 80.0f);
    testicle_physics_cfg.gravity_horizontal_curve =
        physx_clampf(testicle_physics_cfg.gravity_horizontal_curve,
                     0.1f, 8.0f);
    testicle_physics_cfg.gravity_vertical_curve =
        physx_clampf(testicle_physics_cfg.gravity_vertical_curve,
                     0.1f, 8.0f);
    testicle_physics_cfg.gravity_inverted_strength =
        physx_clampf(testicle_physics_cfg.gravity_inverted_strength,
                     0.0f, 120.0f);
    testicle_physics_cfg.gravity_inverted_sign =
        physx_clampf(testicle_physics_cfg.gravity_inverted_sign,
                     -1.0f, 1.0f);
    testicle_physics_cfg.chain_total_bend_max =
        physx_clampf(testicle_physics_cfg.chain_total_bend_max,
                     0.0f, 360.0f);
    testicle_physics_cfg.chain_total_twist_max =
        physx_clampf(testicle_physics_cfg.chain_total_twist_max,
                     0.0f, 360.0f);

    breasts_physics_global_cfg.enabled =
        profile_bool(BREASTS_PHYSICS_CONFIG_SECTION, "enabled",
                     breasts_physics_global_cfg.enabled, config_path);
    breasts_physics_global_cfg.wind_enabled =
        profile_bool(BREASTS_PHYSICS_CONFIG_SECTION, "wind_enabled",
                     breasts_physics_global_cfg.wind_enabled, config_path);
    breasts_physics_global_cfg.wind_scale =
        profile_float(BREASTS_PHYSICS_CONFIG_SECTION, "wind_scale",
                      breasts_physics_global_cfg.wind_scale, config_path);
    breasts_physics_global_cfg.enabled_person[0] =
        profile_bool(BREASTS_PHYSICS_CONFIG_SECTION, "enabled_person01",
                     breasts_physics_global_cfg.enabled_person[0], config_path);
    breasts_physics_global_cfg.enabled_person[1] =
        profile_bool(BREASTS_PHYSICS_CONFIG_SECTION, "enabled_person02",
                     breasts_physics_global_cfg.enabled_person[1], config_path);
    breasts_physics_global_cfg.enabled_person[2] =
        profile_bool(BREASTS_PHYSICS_CONFIG_SECTION, "enabled_person03",
                     breasts_physics_global_cfg.enabled_person[2], config_path);
    breasts_physics_global_cfg.enabled_person[3] =
        profile_bool(BREASTS_PHYSICS_CONFIG_SECTION, "enabled_person04",
                     breasts_physics_global_cfg.enabled_person[3], config_path);
    breasts_physics_global_cfg.collision_scope =
        profile_body_chain_collision_scope(
            BREASTS_PHYSICS_CONFIG_SECTION,
            breasts_physics_global_cfg.collision_scope, config_path);
    breasts_physics_global_cfg.room_collision_enabled = profile_bool(
        BREASTS_PHYSICS_CONFIG_SECTION, "room_collision_enabled",
        breasts_physics_global_cfg.room_collision_enabled, config_path);
    breasts_physics_bone_translation_enabled =
        profile_bool(BREASTS_PHYSICS_CONFIG_SECTION,
                     "bone_translation_enabled",
                     breasts_physics_bone_translation_enabled, config_path);
    GetPrivateProfileStringA(
        BREASTS_PHYSICS_CONFIG_SECTION, "bone_translation_space",
        breasts_physics_bone_translation_space ? "body" : "local",
        buf, sizeof(buf), config_path);
    trim_in_place(buf);
    if (_stricmp(buf, "body") == 0) {
        breasts_physics_bone_translation_space = 1;
    } else if (_stricmp(buf, "local") == 0) {
        breasts_physics_bone_translation_space = 0;
    }
    breasts_physics_global_cfg.translation_scale[0] =
        profile_float(BREASTS_PHYSICS_CONFIG_SECTION,
                      "translation_horizontal_scale",
                      breasts_physics_global_cfg.translation_scale[0], config_path);
    breasts_physics_global_cfg.translation_scale[1] =
        profile_float(BREASTS_PHYSICS_CONFIG_SECTION,
                      "translation_vertical_scale",
                      breasts_physics_global_cfg.translation_scale[1], config_path);
    breasts_physics_global_cfg.translation_scale[2] =
        profile_float(BREASTS_PHYSICS_CONFIG_SECTION,
                      "translation_depth_scale",
                      breasts_physics_global_cfg.translation_scale[2], config_path);
    breasts_physics_bone_translation_scale[0] =
        profile_float(BREASTS_PHYSICS_CONFIG_SECTION,
                      "bone_translation_horizontal_scale",
                      breasts_physics_bone_translation_scale[0], config_path);
    breasts_physics_bone_translation_scale[1] =
        profile_float(BREASTS_PHYSICS_CONFIG_SECTION,
                      "bone_translation_vertical_scale",
                      breasts_physics_bone_translation_scale[1], config_path);
    breasts_physics_bone_translation_scale[2] =
        profile_float(BREASTS_PHYSICS_CONFIG_SECTION,
                      "bone_translation_depth_scale",
                      breasts_physics_bone_translation_scale[2], config_path);
    breasts_physics_bone_translation_gravity_sag =
        profile_float(BREASTS_PHYSICS_CONFIG_SECTION,
                      "bone_translation_gravity_sag",
                      breasts_physics_bone_translation_gravity_sag,
                      config_path);
    breasts_physics_global_cfg.rotation_scale[0] =
        profile_float(BREASTS_PHYSICS_CONFIG_SECTION,
                      "rotation_horizontal_scale",
                      breasts_physics_global_cfg.rotation_scale[0], config_path);
    breasts_physics_global_cfg.rotation_scale[1] =
        profile_float(BREASTS_PHYSICS_CONFIG_SECTION,
                      "rotation_vertical_scale",
                      breasts_physics_global_cfg.rotation_scale[1], config_path);
    breasts_physics_global_cfg.rotation_scale[2] =
        profile_float(BREASTS_PHYSICS_CONFIG_SECTION,
                      "rotation_twist_scale",
                      breasts_physics_global_cfg.rotation_scale[2], config_path);
    breasts_physics_global_cfg.gravity_horizontal_curve =
        profile_float(BREASTS_PHYSICS_CONFIG_SECTION,
                      "gravity_horizontal_curve",
                      breasts_physics_global_cfg.gravity_horizontal_curve,
                      config_path);
    breasts_physics_global_cfg.gravity_vertical_curve =
        profile_float(BREASTS_PHYSICS_CONFIG_SECTION,
                      "gravity_vertical_curve",
                      breasts_physics_global_cfg.gravity_vertical_curve,
                      config_path);
    breasts_physics_global_cfg.gravity_angle =
        profile_float(BREASTS_PHYSICS_CONFIG_SECTION,
                      "gravity_strength",
                      breasts_physics_global_cfg.gravity_angle,
                      config_path);
    /* Preserve the original combined option as a compatibility fallback.
       Explicit inward/outward values override it independently. */
    breasts_physics_gravity_inward_strength =
        profile_float(BREASTS_PHYSICS_CONFIG_SECTION,
                      "gravity_inward_outward_strength",
                      breasts_physics_gravity_inward_strength,
                      config_path);
    breasts_physics_gravity_outward_strength =
        breasts_physics_gravity_inward_strength;
    breasts_physics_gravity_inward_strength =
        profile_float(BREASTS_PHYSICS_CONFIG_SECTION,
                      "gravity_inward_strength",
                      breasts_physics_gravity_inward_strength,
                      config_path);
    breasts_physics_gravity_outward_strength =
        profile_float(BREASTS_PHYSICS_CONFIG_SECTION,
                      "gravity_outward_strength",
                      breasts_physics_gravity_outward_strength,
                      config_path);
    breasts_physics_global_cfg.stiffness =
        profile_float(BREASTS_PHYSICS_CONFIG_SECTION, "stiffness",
                      breasts_physics_global_cfg.stiffness, config_path);
    breasts_physics_global_cfg.damping =
        profile_float(BREASTS_PHYSICS_CONFIG_SECTION, "damping",
                      breasts_physics_global_cfg.damping, config_path);
    breasts_physics_bone_translation_stiffness =
        profile_float(BREASTS_PHYSICS_CONFIG_SECTION,
                      "bone_translation_stiffness",
                      breasts_physics_bone_translation_stiffness,
                      config_path);
    breasts_physics_bone_translation_damping =
        profile_float(BREASTS_PHYSICS_CONFIG_SECTION,
                      "bone_translation_damping",
                      breasts_physics_bone_translation_damping,
                      config_path);
    profile_vec3_or_float(BREASTS_PHYSICS_CONFIG_SECTION,
                          "bone_translation_max_offset", 0.015f,
                          breasts_physics_bone_translation_max_offset,
                          config_path);
    profile_vec3_or_float(BREASTS_PHYSICS_CONFIG_SECTION,
                          "joint01_max_angle",
                          breasts_physics_global_cfg.max_angle,
                          breasts_physics_global_cfg.link_max_angle[0],
                          config_path);
    profile_min_angle_vec3_or_float(
        BREASTS_PHYSICS_CONFIG_SECTION, "joint01_min_angle",
        breasts_physics_global_cfg.link_max_angle[0],
        breasts_physics_global_cfg.link_min_angle[0], config_path);
    breasts_physics_global_cfg.link_gain[0] =
        profile_float(BREASTS_PHYSICS_CONFIG_SECTION, "joint01_gain",
                      breasts_physics_global_cfg.link_gain[0], config_path);
    breasts_physics_global_cfg.interval_ms =
        GetPrivateProfileIntA(BREASTS_PHYSICS_CONFIG_SECTION, "interval_ms",
                              breasts_physics_global_cfg.interval_ms,
                              config_path);

    {
        char internal_buf[64];
        GetPrivateProfileStringA(BREASTS_PHYSICS_INTERNAL_CONFIG_SECTION,
                                 "root_offset", "", internal_buf,
                                 sizeof(internal_buf), config_path);
        trim_in_place(internal_buf);
        breasts_physics_global_cfg.root_offset = parse_offset_value(
            internal_buf, breasts_physics_global_cfg.root_offset);
        GetPrivateProfileStringA(BREASTS_PHYSICS_INTERNAL_CONFIG_SECTION,
                                 "output_offset", "", internal_buf,
                                 sizeof(internal_buf), config_path);
        trim_in_place(internal_buf);
        breasts_physics_global_cfg.output_offset = parse_offset_value(
            internal_buf, breasts_physics_global_cfg.output_offset);
        GetPrivateProfileStringA(BREASTS_PHYSICS_INTERNAL_CONFIG_SECTION,
                                 "bone_translation_output_offset", "",
                                 internal_buf, sizeof(internal_buf),
                                 config_path);
        trim_in_place(internal_buf);
        breasts_physics_bone_translation_offset = parse_offset_value(
            internal_buf, breasts_physics_bone_translation_offset);
    }
    breasts_physics_global_cfg.override_animation =
        profile_bool(BREASTS_PHYSICS_INTERNAL_CONFIG_SECTION,
                     "override_animation",
                     breasts_physics_global_cfg.override_animation,
                     config_path);
    breasts_physics_global_cfg.poseeditor_track_override =
        profile_bool(BREASTS_PHYSICS_INTERNAL_CONFIG_SECTION,
                     "poseeditor_track_override",
                     breasts_physics_global_cfg.poseeditor_track_override,
                     config_path);
    breasts_physics_global_cfg.translation_source_axis[0] =
        profile_axis(BREASTS_PHYSICS_INTERNAL_CONFIG_SECTION,
                     "translation_horizontal_source_axis",
                     breasts_physics_global_cfg.translation_source_axis[0],
                     config_path);
    breasts_physics_global_cfg.translation_tail_axis[0] =
        profile_axis(BREASTS_PHYSICS_INTERNAL_CONFIG_SECTION,
                     "translation_horizontal_tail_axis",
                     breasts_physics_global_cfg.translation_tail_axis[0],
                     config_path);
    breasts_physics_global_cfg.translation_source_axis[1] =
        profile_axis(BREASTS_PHYSICS_INTERNAL_CONFIG_SECTION,
                     "translation_vertical_source_axis",
                     breasts_physics_global_cfg.translation_source_axis[1],
                     config_path);
    breasts_physics_global_cfg.translation_tail_axis[1] =
        profile_axis(BREASTS_PHYSICS_INTERNAL_CONFIG_SECTION,
                     "translation_vertical_tail_axis",
                     breasts_physics_global_cfg.translation_tail_axis[1],
                     config_path);
    breasts_physics_global_cfg.translation_source_axis[2] =
        profile_axis(BREASTS_PHYSICS_INTERNAL_CONFIG_SECTION,
                     "translation_depth_source_axis",
                     breasts_physics_global_cfg.translation_source_axis[2],
                     config_path);
    breasts_physics_global_cfg.translation_tail_axis[2] =
        profile_axis(BREASTS_PHYSICS_INTERNAL_CONFIG_SECTION,
                     "translation_depth_tail_axis",
                     breasts_physics_global_cfg.translation_tail_axis[2],
                     config_path);
    breasts_physics_bone_translation_source_axis[0] =
        profile_axis(BREASTS_PHYSICS_INTERNAL_CONFIG_SECTION,
                     "bone_translation_horizontal_source_axis",
                     breasts_physics_bone_translation_source_axis[0],
                     config_path);
    breasts_physics_bone_translation_tail_axis[0] =
        profile_axis(BREASTS_PHYSICS_INTERNAL_CONFIG_SECTION,
                     "bone_translation_horizontal_tail_axis",
                     breasts_physics_bone_translation_tail_axis[0],
                     config_path);
    breasts_physics_bone_translation_source_axis[1] =
        profile_axis(BREASTS_PHYSICS_INTERNAL_CONFIG_SECTION,
                     "bone_translation_vertical_source_axis",
                     breasts_physics_bone_translation_source_axis[1],
                     config_path);
    breasts_physics_bone_translation_tail_axis[1] =
        profile_axis(BREASTS_PHYSICS_INTERNAL_CONFIG_SECTION,
                     "bone_translation_vertical_tail_axis",
                     breasts_physics_bone_translation_tail_axis[1],
                     config_path);
    breasts_physics_bone_translation_source_axis[2] =
        profile_axis(BREASTS_PHYSICS_INTERNAL_CONFIG_SECTION,
                     "bone_translation_depth_source_axis",
                     breasts_physics_bone_translation_source_axis[2],
                     config_path);
    breasts_physics_bone_translation_tail_axis[2] =
        profile_axis(BREASTS_PHYSICS_INTERNAL_CONFIG_SECTION,
                     "bone_translation_depth_tail_axis",
                     breasts_physics_bone_translation_tail_axis[2],
                     config_path);
    breasts_physics_global_cfg.rotation_source_axis[0] =
        profile_axis(BREASTS_PHYSICS_INTERNAL_CONFIG_SECTION,
                     "rotation_horizontal_source_axis",
                     breasts_physics_global_cfg.rotation_source_axis[0],
                     config_path);
    breasts_physics_global_cfg.rotation_tail_axis[0] =
        profile_axis(BREASTS_PHYSICS_INTERNAL_CONFIG_SECTION,
                     "rotation_horizontal_tail_axis",
                     breasts_physics_global_cfg.rotation_tail_axis[0],
                     config_path);
    breasts_physics_global_cfg.rotation_source_axis[1] =
        profile_axis(BREASTS_PHYSICS_INTERNAL_CONFIG_SECTION,
                     "rotation_vertical_source_axis",
                     breasts_physics_global_cfg.rotation_source_axis[1],
                     config_path);
    breasts_physics_global_cfg.rotation_tail_axis[1] =
        profile_axis(BREASTS_PHYSICS_INTERNAL_CONFIG_SECTION,
                     "rotation_vertical_tail_axis",
                     breasts_physics_global_cfg.rotation_tail_axis[1],
                     config_path);
    breasts_physics_global_cfg.rotation_source_axis[2] =
        profile_axis(BREASTS_PHYSICS_INTERNAL_CONFIG_SECTION,
                     "rotation_twist_source_axis",
                     breasts_physics_global_cfg.rotation_source_axis[2],
                     config_path);
    breasts_physics_global_cfg.rotation_tail_axis[2] =
        profile_axis(BREASTS_PHYSICS_INTERNAL_CONFIG_SECTION,
                     "rotation_twist_tail_axis",
                     breasts_physics_global_cfg.rotation_tail_axis[2],
                     config_path);
    load_body_wind_axis_mapping(
        &breasts_physics_global_cfg,
        BREASTS_PHYSICS_INTERNAL_CONFIG_SECTION, config_path);
    breasts_physics_global_cfg.gravity_inverted_strength =
        profile_float(BREASTS_PHYSICS_INTERNAL_CONFIG_SECTION,
                      "gravity_inverted_strength",
                      breasts_physics_global_cfg.gravity_inverted_strength,
                      config_path);
    breasts_physics_global_cfg.gravity_inverted_sign =
        profile_float(BREASTS_PHYSICS_INTERNAL_CONFIG_SECTION,
                      "gravity_inverted_sign",
                      breasts_physics_global_cfg.gravity_inverted_sign,
                      config_path);
    breasts_physics_gravity_tail_axis[0] =
        profile_axis(BREASTS_PHYSICS_INTERNAL_CONFIG_SECTION,
                     "gravity_horizontal_tail_axis",
                     breasts_physics_gravity_tail_axis[0], config_path);
    breasts_physics_gravity_tail_axis[1] =
        profile_axis(BREASTS_PHYSICS_INTERNAL_CONFIG_SECTION,
                     "gravity_vertical_tail_axis",
                     breasts_physics_gravity_tail_axis[1], config_path);
    breasts_physics_gravity_tail_axis[2] =
        profile_axis(BREASTS_PHYSICS_INTERNAL_CONFIG_SECTION,
                     "gravity_inverted_tail_axis",
                     breasts_physics_gravity_tail_axis[2], config_path);
    breasts_physics_gravity_inward_outward_tail_axis =
        profile_axis(BREASTS_PHYSICS_INTERNAL_CONFIG_SECTION,
                     "gravity_inward_outward_tail_axis",
                     breasts_physics_gravity_inward_outward_tail_axis,
                     config_path);
    breasts_physics_gravity_inward_outward_sign =
        profile_float(BREASTS_PHYSICS_INTERNAL_CONFIG_SECTION,
                      "gravity_inward_outward_sign",
                      breasts_physics_gravity_inward_outward_sign,
                      config_path);
    {
        char sign_buf[128];
        int side, channel;
        static const char *translation_keys[2] = {
            "left_translation_sign", "right_translation_sign"
        };
        static const char *rotation_keys[2] = {
            "left_rotation_sign", "right_rotation_sign"
        };
        static const char *wind_keys[2] = {
            "left_wind_sign", "right_wind_sign"
        };
        static const char *gravity_keys[2] = {
            "left_gravity_sign", "right_gravity_sign"
        };
        static const char *bone_translation_keys[2] = {
            "left_bone_translation_sign",
            "right_bone_translation_sign"
        };
        static const char *fallbacks[2] = {
            "1,1,1", "-1,1,-1"
        };
        static const char *gravity_fallbacks[2] = {
            "1,1,1", "1,-1,1"
        };
        for (side = 0; side < 2; side++) {
            GetPrivateProfileStringA(
                BREASTS_PHYSICS_INTERNAL_CONFIG_SECTION,
                translation_keys[side], fallbacks[side], sign_buf,
                sizeof(sign_buf), config_path);
            parse_vec3(sign_buf,
                       breasts_physics_translation_sign[side]);
            GetPrivateProfileStringA(
                BREASTS_PHYSICS_INTERNAL_CONFIG_SECTION,
                rotation_keys[side], fallbacks[side], sign_buf,
                sizeof(sign_buf), config_path);
            parse_vec3(sign_buf,
                       breasts_physics_rotation_sign[side]);
            GetPrivateProfileStringA(
                BREASTS_PHYSICS_INTERNAL_CONFIG_SECTION,
                wind_keys[side], "1,1,1", sign_buf,
                sizeof(sign_buf), config_path);
            parse_vec3(sign_buf, breasts_physics_wind_sign[side]);
            GetPrivateProfileStringA(
                BREASTS_PHYSICS_INTERNAL_CONFIG_SECTION,
                gravity_keys[side], gravity_fallbacks[side], sign_buf,
                sizeof(sign_buf), config_path);
            parse_vec3(sign_buf,
                       breasts_physics_gravity_sign[side]);
            GetPrivateProfileStringA(
                BREASTS_PHYSICS_INTERNAL_CONFIG_SECTION,
                bone_translation_keys[side], "1,1,1", sign_buf,
                sizeof(sign_buf), config_path);
            parse_vec3(sign_buf,
                       breasts_physics_bone_translation_sign[side]);
            for (channel = 0; channel < 3; channel++) {
                breasts_physics_translation_sign[side][channel] =
                    physx_clampf(
                        breasts_physics_translation_sign[side][channel],
                        -1.0f, 1.0f);
                breasts_physics_rotation_sign[side][channel] =
                    physx_clampf(
                        breasts_physics_rotation_sign[side][channel],
                        -1.0f, 1.0f);
                breasts_physics_wind_sign[side][channel] =
                    physx_clampf(
                        breasts_physics_wind_sign[side][channel],
                        -1.0f, 1.0f);
                breasts_physics_gravity_sign[side][channel] =
                    physx_clampf(
                        breasts_physics_gravity_sign[side][channel],
                        -1.0f, 1.0f);
                breasts_physics_bone_translation_sign[side][channel] =
                    physx_clampf(
                        breasts_physics_bone_translation_sign[side][channel],
                        -1.0f, 1.0f);
            }
        }
    }
    breasts_physics_global_cfg.translation_deadzone =
        profile_float(BREASTS_PHYSICS_INTERNAL_CONFIG_SECTION,
                      "translation_deadzone",
                      breasts_physics_global_cfg.translation_deadzone,
                      config_path);
    breasts_physics_global_cfg.rotation_deadzone =
        profile_float(BREASTS_PHYSICS_INTERNAL_CONFIG_SECTION,
                      "rotation_deadzone",
                      breasts_physics_global_cfg.rotation_deadzone,
                      config_path);
    breasts_physics_global_cfg.zero_output_rest =
        profile_bool(BREASTS_PHYSICS_INTERNAL_CONFIG_SECTION,
                     "zero_output_rest",
                     breasts_physics_global_cfg.zero_output_rest,
                     config_path);
    body_profile_clamp_current_physics_config(
        &breasts_physics_global_cfg, 1);
    breasts_physics_global_cfg.translation_deadzone =
        physx_clampf(breasts_physics_global_cfg.translation_deadzone,
                     0.0f, 1.0f);
    breasts_physics_global_cfg.rotation_deadzone =
        physx_clampf(breasts_physics_global_cfg.rotation_deadzone,
                     0.0f, 45.0f);
    breasts_physics_global_cfg.gravity_angle =
        physx_clampf(breasts_physics_global_cfg.gravity_angle,
                     -90.0f, 90.0f);
    breasts_physics_global_cfg.gravity_inverted_strength =
        physx_clampf(breasts_physics_global_cfg.gravity_inverted_strength,
                     0.0f, 120.0f);
    breasts_physics_global_cfg.gravity_inverted_sign =
        physx_clampf(breasts_physics_global_cfg.gravity_inverted_sign,
                     -1.0f, 1.0f);
    breasts_physics_gravity_inward_strength =
        physx_clampf(breasts_physics_gravity_inward_strength,
                     0.0f, 45.0f);
    breasts_physics_gravity_outward_strength =
        physx_clampf(breasts_physics_gravity_outward_strength,
                     0.0f, 45.0f);
    breasts_physics_gravity_inward_outward_sign =
        physx_clampf(breasts_physics_gravity_inward_outward_sign,
                     -1.0f, 1.0f);
    {
        int axis;
        breasts_physics_bone_translation_stiffness = physx_clampf(
            breasts_physics_bone_translation_stiffness, 1.0f, 200.0f);
        breasts_physics_bone_translation_damping = physx_clampf(
            breasts_physics_bone_translation_damping, 0.0f, 80.0f);
        breasts_physics_bone_translation_gravity_sag = physx_clampf(
            breasts_physics_bone_translation_gravity_sag, 0.0f, 1.0f);
        for (axis = 0; axis < 3; axis++) {
            breasts_physics_bone_translation_scale[axis] = physx_clampf(
                breasts_physics_bone_translation_scale[axis],
                -10.0f, 10.0f);
            breasts_physics_bone_translation_max_offset[axis] =
                physx_clampf(
                    breasts_physics_bone_translation_max_offset[axis],
                    0.0f, 0.100f);
        }
    }

    /* Butt PhysX mirrors the breast movement controls but remains an
       independent two-joint solver. Constant translation sag is not used. */
    butt_physics_global_cfg.enabled =
        profile_bool(BUTT_PHYSICS_CONFIG_SECTION, "enabled",
                     butt_physics_global_cfg.enabled, config_path);
    butt_physics_global_cfg.wind_enabled =
        profile_bool(BUTT_PHYSICS_CONFIG_SECTION, "wind_enabled",
                     butt_physics_global_cfg.wind_enabled, config_path);
    butt_physics_global_cfg.wind_scale =
        profile_float(BUTT_PHYSICS_CONFIG_SECTION, "wind_scale",
                      butt_physics_global_cfg.wind_scale, config_path);
    butt_physics_global_cfg.collision_scope =
        profile_body_chain_collision_scope(
            BUTT_PHYSICS_CONFIG_SECTION,
            butt_physics_global_cfg.collision_scope, config_path);
    butt_physics_global_cfg.room_collision_enabled = profile_bool(
        BUTT_PHYSICS_CONFIG_SECTION, "room_collision_enabled",
        butt_physics_global_cfg.room_collision_enabled, config_path);
    {
        int person_index;
        char key[32];
        for (person_index = 0; person_index < 4; person_index++) {
            snprintf(key, sizeof(key), "enabled_person%02d",
                     person_index + 1);
            butt_physics_global_cfg.enabled_person[person_index] =
                profile_bool(
                    BUTT_PHYSICS_CONFIG_SECTION, key,
                    butt_physics_global_cfg.enabled_person[person_index],
                    config_path);
        }
    }
    butt_physics_bone_translation_enabled =
        profile_bool(BUTT_PHYSICS_CONFIG_SECTION,
                     "bone_translation_enabled",
                     butt_physics_bone_translation_enabled, config_path);
    GetPrivateProfileStringA(
        BUTT_PHYSICS_CONFIG_SECTION, "bone_translation_space",
        butt_physics_bone_translation_space ? "body" : "local",
        buf, sizeof(buf), config_path);
    trim_in_place(buf);
    if (_stricmp(buf, "body") == 0) {
        butt_physics_bone_translation_space = 1;
    } else if (_stricmp(buf, "local") == 0) {
        butt_physics_bone_translation_space = 0;
    }
    {
        int channel;
        static const char *translation_scale_keys[3] = {
            "translation_horizontal_scale",
            "translation_vertical_scale",
            "translation_depth_scale"
        };
        static const char *bone_translation_scale_keys[3] = {
            "bone_translation_horizontal_scale",
            "bone_translation_vertical_scale",
            "bone_translation_depth_scale"
        };
        static const char *rotation_scale_keys[3] = {
            "rotation_horizontal_scale",
            "rotation_vertical_scale",
            "rotation_twist_scale"
        };
        for (channel = 0; channel < 3; channel++) {
            butt_physics_global_cfg.translation_scale[channel] =
                profile_float(
                    BUTT_PHYSICS_CONFIG_SECTION,
                    translation_scale_keys[channel],
                    butt_physics_global_cfg.translation_scale[channel],
                    config_path);
            butt_physics_bone_translation_scale[channel] =
                profile_float(
                    BUTT_PHYSICS_CONFIG_SECTION,
                    bone_translation_scale_keys[channel],
                    butt_physics_bone_translation_scale[channel],
                    config_path);
            butt_physics_global_cfg.rotation_scale[channel] =
                profile_float(
                    BUTT_PHYSICS_CONFIG_SECTION,
                    rotation_scale_keys[channel],
                    butt_physics_global_cfg.rotation_scale[channel],
                    config_path);
        }
    }
    butt_physics_global_cfg.gravity_horizontal_curve =
        profile_float(BUTT_PHYSICS_CONFIG_SECTION,
                      "gravity_horizontal_curve",
                      butt_physics_global_cfg.gravity_horizontal_curve,
                      config_path);
    butt_physics_global_cfg.gravity_vertical_curve =
        profile_float(BUTT_PHYSICS_CONFIG_SECTION,
                      "gravity_vertical_curve",
                      butt_physics_global_cfg.gravity_vertical_curve,
                      config_path);
    butt_physics_global_cfg.gravity_angle =
        profile_float(BUTT_PHYSICS_CONFIG_SECTION, "gravity_strength",
                      butt_physics_global_cfg.gravity_angle, config_path);
    butt_physics_global_cfg.stiffness =
        profile_float(BUTT_PHYSICS_CONFIG_SECTION, "stiffness",
                      butt_physics_global_cfg.stiffness, config_path);
    butt_physics_global_cfg.damping =
        profile_float(BUTT_PHYSICS_CONFIG_SECTION, "damping",
                      butt_physics_global_cfg.damping, config_path);
    butt_physics_bone_translation_stiffness =
        profile_float(BUTT_PHYSICS_CONFIG_SECTION,
                      "bone_translation_stiffness",
                      butt_physics_bone_translation_stiffness,
                      config_path);
    butt_physics_bone_translation_damping =
        profile_float(BUTT_PHYSICS_CONFIG_SECTION,
                      "bone_translation_damping",
                      butt_physics_bone_translation_damping,
                      config_path);
    profile_vec3_or_float(BUTT_PHYSICS_CONFIG_SECTION,
                          "bone_translation_max_offset", 0.015f,
                          butt_physics_bone_translation_max_offset,
                          config_path);
    profile_vec3_or_float(BUTT_PHYSICS_CONFIG_SECTION,
                          "joint01_max_angle",
                          butt_physics_global_cfg.max_angle,
                          butt_physics_global_cfg.link_max_angle[0],
                          config_path);
    profile_min_angle_vec3_or_float(
        BUTT_PHYSICS_CONFIG_SECTION, "joint01_min_angle",
        butt_physics_global_cfg.link_max_angle[0],
        butt_physics_global_cfg.link_min_angle[0], config_path);
    butt_physics_global_cfg.link_gain[0] =
        profile_float(BUTT_PHYSICS_CONFIG_SECTION, "joint01_gain",
                      butt_physics_global_cfg.link_gain[0], config_path);
    butt_physics_global_cfg.interval_ms =
        GetPrivateProfileIntA(BUTT_PHYSICS_CONFIG_SECTION, "interval_ms",
                              butt_physics_global_cfg.interval_ms,
                              config_path);
    {
        char internal_buf[64];
        GetPrivateProfileStringA(BUTT_PHYSICS_INTERNAL_CONFIG_SECTION,
                                 "root_offset", "", internal_buf,
                                 sizeof(internal_buf), config_path);
        trim_in_place(internal_buf);
        butt_physics_global_cfg.root_offset = parse_offset_value(
            internal_buf, butt_physics_global_cfg.root_offset);
        GetPrivateProfileStringA(BUTT_PHYSICS_INTERNAL_CONFIG_SECTION,
                                 "output_offset", "", internal_buf,
                                 sizeof(internal_buf), config_path);
        trim_in_place(internal_buf);
        butt_physics_global_cfg.output_offset = parse_offset_value(
            internal_buf, butt_physics_global_cfg.output_offset);
        GetPrivateProfileStringA(BUTT_PHYSICS_INTERNAL_CONFIG_SECTION,
                                 "bone_translation_output_offset", "",
                                 internal_buf, sizeof(internal_buf),
                                 config_path);
        trim_in_place(internal_buf);
        butt_physics_bone_translation_offset = parse_offset_value(
            internal_buf, butt_physics_bone_translation_offset);
    }
    butt_physics_global_cfg.override_animation =
        profile_bool(BUTT_PHYSICS_INTERNAL_CONFIG_SECTION,
                     "override_animation",
                     butt_physics_global_cfg.override_animation,
                     config_path);
    butt_physics_global_cfg.poseeditor_track_override =
        profile_bool(BUTT_PHYSICS_INTERNAL_CONFIG_SECTION,
                     "poseeditor_track_override",
                     butt_physics_global_cfg.poseeditor_track_override,
                     config_path);
    {
        int channel;
        static const char *translation_source_keys[3] = {
            "translation_horizontal_source_axis",
            "translation_vertical_source_axis",
            "translation_depth_source_axis"
        };
        static const char *translation_tail_keys[3] = {
            "translation_horizontal_tail_axis",
            "translation_vertical_tail_axis",
            "translation_depth_tail_axis"
        };
        static const char *rotation_source_keys[3] = {
            "rotation_horizontal_source_axis",
            "rotation_vertical_source_axis",
            "rotation_twist_source_axis"
        };
        static const char *rotation_tail_keys[3] = {
            "rotation_horizontal_tail_axis",
            "rotation_vertical_tail_axis",
            "rotation_twist_tail_axis"
        };
        static const char *bone_source_keys[3] = {
            "bone_translation_horizontal_source_axis",
            "bone_translation_vertical_source_axis",
            "bone_translation_depth_source_axis"
        };
        static const char *bone_tail_keys[3] = {
            "bone_translation_horizontal_tail_axis",
            "bone_translation_vertical_tail_axis",
            "bone_translation_depth_tail_axis"
        };
        static const char *gravity_tail_keys[3] = {
            "gravity_horizontal_tail_axis",
            "gravity_vertical_tail_axis",
            "gravity_inverted_tail_axis"
        };
        for (channel = 0; channel < 3; channel++) {
            butt_physics_global_cfg.translation_source_axis[channel] =
                profile_axis(
                    BUTT_PHYSICS_INTERNAL_CONFIG_SECTION,
                    translation_source_keys[channel],
                    butt_physics_global_cfg.translation_source_axis[channel],
                    config_path);
            butt_physics_global_cfg.translation_tail_axis[channel] =
                profile_axis(
                    BUTT_PHYSICS_INTERNAL_CONFIG_SECTION,
                    translation_tail_keys[channel],
                    butt_physics_global_cfg.translation_tail_axis[channel],
                    config_path);
            butt_physics_global_cfg.rotation_source_axis[channel] =
                profile_axis(
                    BUTT_PHYSICS_INTERNAL_CONFIG_SECTION,
                    rotation_source_keys[channel],
                    butt_physics_global_cfg.rotation_source_axis[channel],
                    config_path);
            butt_physics_global_cfg.rotation_tail_axis[channel] =
                profile_axis(
                    BUTT_PHYSICS_INTERNAL_CONFIG_SECTION,
                    rotation_tail_keys[channel],
                    butt_physics_global_cfg.rotation_tail_axis[channel],
                    config_path);
            butt_physics_bone_translation_source_axis[channel] =
                profile_axis(
                    BUTT_PHYSICS_INTERNAL_CONFIG_SECTION,
                    bone_source_keys[channel],
                    butt_physics_bone_translation_source_axis[channel],
                    config_path);
            butt_physics_bone_translation_tail_axis[channel] =
                profile_axis(
                    BUTT_PHYSICS_INTERNAL_CONFIG_SECTION,
                    bone_tail_keys[channel],
                    butt_physics_bone_translation_tail_axis[channel],
                    config_path);
            butt_physics_gravity_tail_axis[channel] =
                profile_axis(
                    BUTT_PHYSICS_INTERNAL_CONFIG_SECTION,
                    gravity_tail_keys[channel],
                    butt_physics_gravity_tail_axis[channel], config_path);
        }
    }
    load_body_wind_axis_mapping(
        &butt_physics_global_cfg,
        BUTT_PHYSICS_INTERNAL_CONFIG_SECTION, config_path);
    butt_physics_global_cfg.gravity_inverted_strength =
        profile_float(BUTT_PHYSICS_INTERNAL_CONFIG_SECTION,
                      "gravity_inverted_strength",
                      butt_physics_global_cfg.gravity_inverted_strength,
                      config_path);
    butt_physics_global_cfg.gravity_inverted_sign =
        profile_float(BUTT_PHYSICS_INTERNAL_CONFIG_SECTION,
                      "gravity_inverted_sign",
                      butt_physics_global_cfg.gravity_inverted_sign,
                      config_path);
    {
        char sign_buf[128];
        int side, channel;
        static const char *translation_keys[2] = {
            "left_translation_sign", "right_translation_sign"
        };
        static const char *rotation_keys[2] = {
            "left_rotation_sign", "right_rotation_sign"
        };
        static const char *wind_keys[2] = {
            "left_wind_sign", "right_wind_sign"
        };
        static const char *gravity_keys[2] = {
            "left_gravity_sign", "right_gravity_sign"
        };
        static const char *bone_keys[2] = {
            "left_bone_translation_sign",
            "right_bone_translation_sign"
        };
        static const char *translation_fallbacks[2] = {
            "1,1,1", "1,1,-1"
        };
        static const char *rotation_fallbacks[2] = {
            "1,-1,1", "-1,1,1"
        };
        static const char *gravity_fallbacks[2] = {
            "1,1,1", "1,-1,1"
        };
        for (side = 0; side < 2; side++) {
            GetPrivateProfileStringA(
                BUTT_PHYSICS_INTERNAL_CONFIG_SECTION,
                translation_keys[side], translation_fallbacks[side],
                sign_buf, sizeof(sign_buf), config_path);
            parse_vec3(sign_buf, butt_physics_translation_sign[side]);
            GetPrivateProfileStringA(
                BUTT_PHYSICS_INTERNAL_CONFIG_SECTION,
                rotation_keys[side], rotation_fallbacks[side], sign_buf,
                sizeof(sign_buf), config_path);
            parse_vec3(sign_buf, butt_physics_rotation_sign[side]);
            GetPrivateProfileStringA(
                BUTT_PHYSICS_INTERNAL_CONFIG_SECTION,
                wind_keys[side], "1,1,1", sign_buf,
                sizeof(sign_buf), config_path);
            parse_vec3(sign_buf, butt_physics_wind_sign[side]);
            GetPrivateProfileStringA(
                BUTT_PHYSICS_INTERNAL_CONFIG_SECTION,
                gravity_keys[side], gravity_fallbacks[side], sign_buf,
                sizeof(sign_buf), config_path);
            parse_vec3(sign_buf, butt_physics_gravity_sign[side]);
            GetPrivateProfileStringA(
                BUTT_PHYSICS_INTERNAL_CONFIG_SECTION, bone_keys[side],
                "1,1,1", sign_buf, sizeof(sign_buf), config_path);
            parse_vec3(sign_buf,
                       butt_physics_bone_translation_sign[side]);
            for (channel = 0; channel < 3; channel++) {
                butt_physics_translation_sign[side][channel] =
                    physx_clampf(
                        butt_physics_translation_sign[side][channel],
                        -1.0f, 1.0f);
                butt_physics_rotation_sign[side][channel] =
                    physx_clampf(
                        butt_physics_rotation_sign[side][channel],
                        -1.0f, 1.0f);
                butt_physics_wind_sign[side][channel] =
                    physx_clampf(
                        butt_physics_wind_sign[side][channel],
                        -1.0f, 1.0f);
                butt_physics_gravity_sign[side][channel] =
                    physx_clampf(butt_physics_gravity_sign[side][channel],
                                 -1.0f, 1.0f);
                butt_physics_bone_translation_sign[side][channel] =
                    physx_clampf(
                        butt_physics_bone_translation_sign[side][channel],
                        -1.0f, 1.0f);
            }
        }
    }
    butt_physics_global_cfg.translation_deadzone =
        profile_float(BUTT_PHYSICS_INTERNAL_CONFIG_SECTION,
                      "translation_deadzone",
                      butt_physics_global_cfg.translation_deadzone,
                      config_path);
    butt_physics_global_cfg.rotation_deadzone =
        profile_float(BUTT_PHYSICS_INTERNAL_CONFIG_SECTION,
                      "rotation_deadzone",
                      butt_physics_global_cfg.rotation_deadzone,
                      config_path);
    butt_physics_global_cfg.zero_output_rest =
        profile_bool(BUTT_PHYSICS_INTERNAL_CONFIG_SECTION,
                     "zero_output_rest",
                     butt_physics_global_cfg.zero_output_rest,
                     config_path);
    body_profile_clamp_current_physics_config(&butt_physics_global_cfg, 1);
    butt_physics_global_cfg.translation_deadzone = physx_clampf(
        butt_physics_global_cfg.translation_deadzone, 0.0f, 1.0f);
    butt_physics_global_cfg.rotation_deadzone = physx_clampf(
        butt_physics_global_cfg.rotation_deadzone, 0.0f, 45.0f);
    butt_physics_global_cfg.gravity_angle = physx_clampf(
        butt_physics_global_cfg.gravity_angle, -90.0f, 90.0f);
    butt_physics_global_cfg.gravity_inverted_strength = physx_clampf(
        butt_physics_global_cfg.gravity_inverted_strength, 0.0f, 120.0f);
    butt_physics_global_cfg.gravity_inverted_sign = physx_clampf(
        butt_physics_global_cfg.gravity_inverted_sign, -1.0f, 1.0f);
    {
        int axis;
        butt_physics_bone_translation_stiffness = physx_clampf(
            butt_physics_bone_translation_stiffness, 1.0f, 200.0f);
        butt_physics_bone_translation_damping = physx_clampf(
            butt_physics_bone_translation_damping, 0.0f, 80.0f);
        for (axis = 0; axis < 3; axis++) {
            butt_physics_bone_translation_scale[axis] = physx_clampf(
                butt_physics_bone_translation_scale[axis],
                -10.0f, 10.0f);
            butt_physics_bone_translation_max_offset[axis] = physx_clampf(
                butt_physics_bone_translation_max_offset[axis],
                0.0f, 0.100f);
        }
    }

    body_chain_collider_cfg.enabled =
        body_colliders_profile_bool("enabled",
                     body_chain_collider_cfg.enabled, config_path);
    body_chain_collider_cfg.debug_draw =
        body_colliders_profile_bool("debug_draw",
                     body_chain_collider_cfg.debug_draw, config_path);
    body_chain_collider_cfg.response_enabled =
        body_colliders_profile_bool("response_enabled",
                     body_chain_collider_cfg.response_enabled, config_path);
    body_chain_collider_cfg.breasts_collision_enabled =
        body_colliders_profile_bool("breasts_collision_enabled",
                     body_chain_collider_cfg.breasts_collision_enabled,
                     config_path);
    body_chain_collider_cfg.butt_collision_enabled =
        body_colliders_profile_bool("butt_collision_enabled",
                     body_chain_collider_cfg.butt_collision_enabled,
                     config_path);
    body_chain_collider_cfg.penis_collision_enabled =
        body_colliders_profile_bool("penis_collision_enabled",
                     body_chain_collider_cfg.response_enabled, config_path);
    body_chain_collider_cfg.testicle_collision_enabled =
        body_colliders_profile_bool("testicle_collision_enabled",
                     body_chain_collider_cfg.testicle_collision_enabled, config_path);
    body_chain_collider_cfg.diagnostic =
        body_colliders_profile_bool("diagnostic",
                     body_chain_collider_cfg.diagnostic, config_path);
    body_chain_collider_cfg.root_local_offsets =
        body_colliders_profile_bool("root_local_offsets",
                     body_chain_collider_cfg.root_local_offsets, config_path);
    body_chain_collider_cfg.live_testicle_bones =
        body_colliders_profile_bool("live_testicle_bones",
                     body_chain_collider_cfg.live_testicle_bones, config_path);
    body_chain_collider_cfg.testicles_bone_head_mode =
        body_colliders_profile_bool("testicles_bone_head_mode",
                     body_chain_collider_cfg.testicles_bone_head_mode, config_path);
    body_colliders_profile_string("testicle_rotation_offset", "",
                             buf, sizeof(buf), config_path);
    trim_in_place(buf);
    body_chain_collider_cfg.testicle_rotation_offset =
        parse_offset_value(buf, body_chain_collider_cfg.testicle_rotation_offset);
    body_colliders_profile_string("position_offset", "",
                             buf, sizeof(buf), config_path);
    trim_in_place(buf);
    body_chain_collider_cfg.position_offset =
        parse_offset_value(buf, body_chain_collider_cfg.position_offset);
    body_colliders_profile_string("root_local_pelvis_offset", "",
                             buf, sizeof(buf), config_path);
    parse_vec3(buf, body_chain_collider_cfg.local_offset[BODY_COLLIDER_ROOT]);
    body_colliders_profile_string("root_local_hip_L_offset", "",
                             buf, sizeof(buf), config_path);
    parse_vec3(buf, body_chain_collider_cfg.local_offset[BODY_COLLIDER_HIP_L]);
    body_colliders_profile_string("root_local_hip_R_offset", "",
                             buf, sizeof(buf), config_path);
    parse_vec3(buf, body_chain_collider_cfg.local_offset[BODY_COLLIDER_HIP_R]);
    body_colliders_profile_string("root_local_body_chain_base_offset", "",
                             buf, sizeof(buf), config_path);
    parse_vec3(buf, body_chain_collider_cfg.body_chain_base_offset);
    body_colliders_profile_string("root_local_testicles01_offset", "",
                             buf, sizeof(buf), config_path);
    parse_vec3(buf, body_chain_collider_cfg.local_offset[BODY_COLLIDER_TESTICLES_01]);
    body_colliders_profile_string("root_local_testicles02_offset", "",
                             buf, sizeof(buf), config_path);
    parse_vec3(buf, body_chain_collider_cfg.local_offset[BODY_COLLIDER_TESTICLES_02]);
    body_colliders_profile_string("root_local_testicles01_pivot", "",
                             buf, sizeof(buf), config_path);
    parse_vec3(buf, body_chain_collider_cfg.testicle_head[0]);
    body_colliders_profile_string("root_local_testicles02_pivot", "",
                             buf, sizeof(buf), config_path);
    parse_vec3(buf, body_chain_collider_cfg.testicle_head[1]);
    body_chain_collider_cfg.testicle_pivot[0][0] =
        body_chain_collider_cfg.testicle_head[0][0] -
        body_chain_collider_cfg.body_chain_base_offset[0];
    body_chain_collider_cfg.testicle_pivot[0][1] =
        body_chain_collider_cfg.testicle_head[0][1] -
        body_chain_collider_cfg.body_chain_base_offset[1];
    body_chain_collider_cfg.testicle_pivot[0][2] =
        body_chain_collider_cfg.testicle_head[0][2] -
        body_chain_collider_cfg.body_chain_base_offset[2];
    body_chain_collider_cfg.testicle_pivot[1][0] =
        body_chain_collider_cfg.testicle_head[1][0] -
        body_chain_collider_cfg.body_chain_base_offset[0];
    body_chain_collider_cfg.testicle_pivot[1][1] =
        body_chain_collider_cfg.testicle_head[1][1] -
        body_chain_collider_cfg.body_chain_base_offset[1];
    body_chain_collider_cfg.testicle_pivot[1][2] =
        body_chain_collider_cfg.testicle_head[1][2] -
        body_chain_collider_cfg.body_chain_base_offset[2];
    body_colliders_profile_string("testicles01_fine_offset", "",
                             buf, sizeof(buf), config_path);
    parse_vec3(buf, body_chain_collider_cfg.testicle_fine_offset[0]);
    body_colliders_profile_string("testicles02_fine_offset", "",
                             buf, sizeof(buf), config_path);
    parse_vec3(buf, body_chain_collider_cfg.testicle_fine_offset[1]);
    body_colliders_profile_string_alias("spine01_fine_offset",
                                        "stomach01_fine_offset", "",
                                        buf, sizeof(buf), config_path);
    parse_vec3(buf, body_chain_collider_cfg.stomach_fine_offset[0]);
    body_colliders_profile_string_alias("spine02_fine_offset",
                                        "stomach02_fine_offset", "",
                                        buf, sizeof(buf), config_path);
    parse_vec3(buf, body_chain_collider_cfg.stomach_fine_offset[1]);
    body_colliders_profile_string("hip_fine_offset", "",
                             buf, sizeof(buf), config_path);
    parse_vec3(buf, body_chain_collider_cfg.hip_fine_offset);
    body_colliders_profile_string("knee_fine_offset", "",
                             buf, sizeof(buf), config_path);
    parse_vec3(buf, body_chain_collider_cfg.knee_fine_offset);
    body_colliders_profile_string("thigh_fine_offset", "",
                             buf, sizeof(buf), config_path);
    parse_vec3(buf, body_chain_collider_cfg.thigh_fine_offset);
    body_colliders_profile_string_alias("spine03_fine_offset",
                                        "stomach03_fine_offset", "",
                                        buf, sizeof(buf), config_path);
    parse_vec3(buf, body_chain_collider_cfg.stomach_extra_fine_offset[0]);
    body_colliders_profile_string_alias("spine04_fine_offset",
                                        "stomach04_fine_offset", "",
                                        buf, sizeof(buf), config_path);
    parse_vec3(buf, body_chain_collider_cfg.stomach_extra_fine_offset[1]);
    body_colliders_profile_string("ankle_fine_offset", "",
                             buf, sizeof(buf), config_path);
    parse_vec3(buf, body_chain_collider_cfg.ankle_fine_offset);
    body_colliders_profile_string("ball_fine_offset", "",
                             buf, sizeof(buf), config_path);
    parse_vec3(buf, body_chain_collider_cfg.ball_fine_offset);
    body_colliders_profile_string("breast_fine_offset", "",
                             buf, sizeof(buf), config_path);
    parse_vec3(buf, body_chain_collider_cfg.breast_fine_offset);
    body_colliders_profile_string("butt_fine_offset", "",
                             buf, sizeof(buf), config_path);
    parse_vec3(buf, body_chain_collider_cfg.butt_fine_offset);
    body_colliders_profile_string("neck_fine_offset", "",
                             buf, sizeof(buf), config_path);
    parse_vec3(buf, body_chain_collider_cfg.neck_fine_offset);
    body_colliders_profile_string("head_fine_offset", "",
                             buf, sizeof(buf), config_path);
    parse_vec3(buf, body_chain_collider_cfg.head_fine_offset);
    body_colliders_profile_string("clavicle_fine_offset", "",
                             buf, sizeof(buf), config_path);
    parse_vec3(buf, body_chain_collider_cfg.clavicle_fine_offset);
    body_colliders_profile_string("shoulder_fine_offset", "",
                             buf, sizeof(buf), config_path);
    parse_vec3(buf, body_chain_collider_cfg.shoulder_fine_offset);
    body_colliders_profile_string("elbow_fine_offset", "",
                             buf, sizeof(buf), config_path);
    parse_vec3(buf, body_chain_collider_cfg.elbow_fine_offset);
    body_colliders_profile_string("forearm_fine_offset", "",
                             buf, sizeof(buf), config_path);
    parse_vec3(buf, body_chain_collider_cfg.forearm_fine_offset);
    body_colliders_profile_string("wrist_fine_offset", "",
                             buf, sizeof(buf), config_path);
    parse_vec3(buf, body_chain_collider_cfg.wrist_fine_offset);
    body_colliders_profile_string("palm_fine_offset", "",
                             buf, sizeof(buf), config_path);
    parse_vec3(buf, body_chain_collider_cfg.palm_fine_offset);
    body_colliders_profile_string("finger01_fine_offset", "",
                             buf, sizeof(buf), config_path);
    parse_vec3(buf, body_chain_collider_cfg.finger_fine_offset[0]);
    body_colliders_profile_string("finger02_fine_offset", "",
                             buf, sizeof(buf), config_path);
    parse_vec3(buf, body_chain_collider_cfg.finger_fine_offset[1]);
    body_colliders_profile_string("finger03_fine_offset", "",
                             buf, sizeof(buf), config_path);
    parse_vec3(buf, body_chain_collider_cfg.finger_fine_offset[2]);
    body_colliders_profile_string("finger04_fine_offset", "",
                             buf, sizeof(buf), config_path);
    parse_vec3(buf, body_chain_collider_cfg.finger_fine_offset[3]);
    body_colliders_profile_string("finger05_fine_offset", "",
                             buf, sizeof(buf), config_path);
    parse_vec3(buf, body_chain_collider_cfg.finger_fine_offset[4]);
    if (body_chain_collider_cfg.testicles_bone_head_mode) {
        body_chain_collider_cfg.local_offset[BODY_COLLIDER_TESTICLES_01][0] =
            body_chain_collider_cfg.testicle_pivot[0][0] +
            body_chain_collider_cfg.testicle_fine_offset[0][0];
        body_chain_collider_cfg.local_offset[BODY_COLLIDER_TESTICLES_01][1] =
            body_chain_collider_cfg.testicle_pivot[0][1] +
            body_chain_collider_cfg.testicle_fine_offset[0][1];
        body_chain_collider_cfg.local_offset[BODY_COLLIDER_TESTICLES_01][2] =
            body_chain_collider_cfg.testicle_pivot[0][2] +
            body_chain_collider_cfg.testicle_fine_offset[0][2];
        body_chain_collider_cfg.local_offset[BODY_COLLIDER_TESTICLES_02][0] =
            body_chain_collider_cfg.testicle_pivot[1][0] +
            body_chain_collider_cfg.testicle_fine_offset[1][0];
        body_chain_collider_cfg.local_offset[BODY_COLLIDER_TESTICLES_02][1] =
            body_chain_collider_cfg.testicle_pivot[1][1] +
            body_chain_collider_cfg.testicle_fine_offset[1][1];
        body_chain_collider_cfg.local_offset[BODY_COLLIDER_TESTICLES_02][2] =
            body_chain_collider_cfg.testicle_pivot[1][2] +
            body_chain_collider_cfg.testicle_fine_offset[1][2];
    }
    body_chain_collider_cfg.local_offset[BODY_COLLIDER_TESTICLES_MID][0] =
        (body_chain_collider_cfg.local_offset[BODY_COLLIDER_TESTICLES_01][0] +
         body_chain_collider_cfg.local_offset[BODY_COLLIDER_TESTICLES_02][0]) * 0.5f;
    body_chain_collider_cfg.local_offset[BODY_COLLIDER_TESTICLES_MID][1] =
        (body_chain_collider_cfg.local_offset[BODY_COLLIDER_TESTICLES_01][1] +
         body_chain_collider_cfg.local_offset[BODY_COLLIDER_TESTICLES_02][1]) * 0.5f;
    body_chain_collider_cfg.local_offset[BODY_COLLIDER_TESTICLES_MID][2] =
        (body_chain_collider_cfg.local_offset[BODY_COLLIDER_TESTICLES_01][2] +
         body_chain_collider_cfg.local_offset[BODY_COLLIDER_TESTICLES_02][2]) * 0.5f;
    body_chain_collider_cfg.pelvis_radius =
        body_colliders_profile_float("pelvis_radius",
                      body_chain_collider_cfg.pelvis_radius, config_path);
    body_chain_collider_apply_config_to_nodes();
    body_chain_profile_radius_nodes_alias(
        "spine01_radius", "stomach01_radius",
        body_chain_collider_cfg.stomach_radius[0],
        BODY_COLLIDER_STOMACH_01, -1, config_path);
    body_chain_profile_radius_nodes_alias(
        "spine02_radius", "stomach02_radius",
        body_chain_collider_cfg.stomach_radius[1],
        BODY_COLLIDER_STOMACH_02, -1, config_path);
    body_chain_profile_radius_nodes_alias(
        "spine03_radius", "stomach03_radius",
        body_chain_collider_cfg.stomach_extra_radius[0],
        BODY_COLLIDER_STOMACH_03, -1, config_path);
    body_chain_profile_radius_nodes_alias(
        "spine04_radius", "stomach04_radius",
        body_chain_collider_cfg.stomach_extra_radius[1],
        BODY_COLLIDER_STOMACH_04, -1, config_path);
    body_chain_profile_radius_pair("hip_radius",
                                   &body_chain_collider_cfg.hip_radius,
                                   BODY_COLLIDER_HIP_L,
                                   BODY_COLLIDER_HIP_R,
                                   config_path);
    body_chain_profile_radius_pair("knee_radius",
                                   &body_chain_collider_cfg.knee_radius,
                                   BODY_COLLIDER_KNEE_L,
                                   BODY_COLLIDER_KNEE_R,
                                   config_path);
    if (body_chain_collider_cfg.thigh_radius < 0.0f) {
        body_chain_collider_cfg.thigh_radius =
            (body_chain_collider_cfg.hip_radius +
             body_chain_collider_cfg.knee_radius) * 0.5f;
    }
    body_chain_profile_radius_pair("thigh_radius",
                                   &body_chain_collider_cfg.thigh_radius,
                                   BODY_COLLIDER_THIGH_L,
                                   BODY_COLLIDER_THIGH_R,
                                   config_path);
    body_chain_profile_radius_pair("testicles_radius",
                                   &body_chain_collider_cfg.testicles_radius,
                                   BODY_COLLIDER_TESTICLES_01,
                                   BODY_COLLIDER_TESTICLES_02,
                                   config_path);
    body_chain_set_node_radius(BODY_COLLIDER_TESTICLES_MID,
                               body_chain_collider_cfg.node_radius
                                   [BODY_COLLIDER_TESTICLES_01]);
    body_chain_profile_radius_nodes("ankle_radius",
                                    body_chain_collider_cfg.ankle_radius,
                                    BODY_COLLIDER_ANKLE_L,
                                    BODY_COLLIDER_ANKLE_R,
                                    config_path);
    body_chain_profile_radius_nodes("ball_radius",
                                    body_chain_collider_cfg.ball_radius,
                                    BODY_COLLIDER_BALL_L,
                                    BODY_COLLIDER_BALL_R,
                                    config_path);
    body_chain_profile_radius_nodes("breast_radius",
                                    body_chain_collider_cfg.breast_radius,
                                    BODY_COLLIDER_BREAST_L,
                                    BODY_COLLIDER_BREAST_R,
                                    config_path);
    body_chain_profile_radius_nodes("butt_radius",
                                    body_chain_collider_cfg.butt_radius,
                                    BODY_COLLIDER_BUTT_L,
                                    BODY_COLLIDER_BUTT_R,
                                    config_path);
    body_chain_profile_radius_nodes("neck_radius",
                                    body_chain_collider_cfg.neck_radius,
                                    BODY_COLLIDER_NECK_01, -1,
                                    config_path);
    body_chain_profile_radius_nodes("head_radius",
                                    body_chain_collider_cfg.head_radius,
                                    BODY_COLLIDER_HEAD_02, -1,
                                    config_path);
    body_chain_profile_radius_nodes("clavicle_radius",
                                    body_chain_collider_cfg.clavicle_radius,
                                    BODY_COLLIDER_CLAVICLE_L,
                                    BODY_COLLIDER_CLAVICLE_R,
                                    config_path);
    body_chain_profile_radius_nodes("shoulder_radius",
                                    body_chain_collider_cfg.shoulder_radius,
                                    BODY_COLLIDER_SHOULDER_L,
                                    BODY_COLLIDER_SHOULDER_R,
                                    config_path);
    body_chain_profile_radius_nodes("elbow_radius",
                                    body_chain_collider_cfg.elbow_radius,
                                    BODY_COLLIDER_ELBOW_L,
                                    BODY_COLLIDER_ELBOW_R,
                                    config_path);
    body_chain_profile_radius_nodes("forearm_radius",
                                    body_chain_collider_cfg.forearm_radius,
                                    BODY_COLLIDER_FOREARM_L,
                                    BODY_COLLIDER_FOREARM_R,
                                    config_path);
    body_chain_profile_radius_nodes("wrist_radius",
                                    body_chain_collider_cfg.wrist_radius,
                                    BODY_COLLIDER_WRIST_L,
                                    BODY_COLLIDER_WRIST_R,
                                    config_path);
    body_chain_profile_radius_nodes("palm_radius",
                                    body_chain_collider_cfg.palm_radius,
                                    BODY_COLLIDER_PALM_L,
                                    BODY_COLLIDER_PALM_R,
                                    config_path);
    body_chain_profile_radius_nodes("finger01_radius",
                                    body_chain_collider_cfg.finger_radius[0],
                                    BODY_COLLIDER_FINGER01_L_01,
                                    BODY_COLLIDER_FINGER01_R_01,
                                    config_path);
    body_chain_profile_radius_nodes("finger02_radius",
                                    body_chain_collider_cfg.finger_radius[1],
                                    BODY_COLLIDER_FINGER02_L_01,
                                    BODY_COLLIDER_FINGER02_R_01,
                                    config_path);
    body_chain_profile_radius_nodes("finger03_radius",
                                    body_chain_collider_cfg.finger_radius[2],
                                    BODY_COLLIDER_FINGER03_L_01,
                                    BODY_COLLIDER_FINGER03_R_01,
                                    config_path);
    body_chain_profile_radius_nodes("finger04_radius",
                                    body_chain_collider_cfg.finger_radius[3],
                                    BODY_COLLIDER_FINGER04_L_01,
                                    BODY_COLLIDER_FINGER04_R_01,
                                    config_path);
    body_chain_profile_radius_nodes("finger05_radius",
                                    body_chain_collider_cfg.finger_radius[4],
                                    BODY_COLLIDER_FINGER05_L_01,
                                    BODY_COLLIDER_FINGER05_R_01,
                                    config_path);
    for (i = 0; i < 4; i++) {
        body_chain_set_node_radius(BODY_COLLIDER_FINGER01_L_01 + i,
                                   body_chain_collider_cfg.finger_radius[0]);
        body_chain_set_node_radius(BODY_COLLIDER_FINGER01_R_01 + i,
                                   body_chain_collider_cfg.finger_radius[0]);
    }
    for (i = 0; i < 5; i++) {
        body_chain_set_node_radius(BODY_COLLIDER_FINGER02_L_01 + i,
                                   body_chain_collider_cfg.finger_radius[1]);
        body_chain_set_node_radius(BODY_COLLIDER_FINGER02_R_01 + i,
                                   body_chain_collider_cfg.finger_radius[1]);
        body_chain_set_node_radius(BODY_COLLIDER_FINGER03_L_01 + i,
                                   body_chain_collider_cfg.finger_radius[2]);
        body_chain_set_node_radius(BODY_COLLIDER_FINGER03_R_01 + i,
                                   body_chain_collider_cfg.finger_radius[2]);
        body_chain_set_node_radius(BODY_COLLIDER_FINGER04_L_01 + i,
                                   body_chain_collider_cfg.finger_radius[3]);
        body_chain_set_node_radius(BODY_COLLIDER_FINGER04_R_01 + i,
                                   body_chain_collider_cfg.finger_radius[3]);
        body_chain_set_node_radius(BODY_COLLIDER_FINGER05_L_01 + i,
                                   body_chain_collider_cfg.finger_radius[4]);
        body_chain_set_node_radius(BODY_COLLIDER_FINGER05_R_01 + i,
                                   body_chain_collider_cfg.finger_radius[4]);
    }
    body_chain_collider_cfg.pelvis_capsule_radius =
        body_colliders_profile_float("pelvis_capsule_radius",
                      body_chain_collider_cfg.pelvis_capsule_radius, config_path);
    body_chain_collider_cfg.response_radius_scale =
        body_colliders_profile_float("response_radius_scale",
                      body_chain_collider_cfg.response_radius_scale, config_path);
    body_chain_collider_cfg.chain_radius =
        body_colliders_profile_float("chain_radius",
                      body_chain_collider_cfg.chain_radius, config_path);
    body_chain_collider_cfg.link_length[0] =
        body_colliders_profile_float("joint01_link_length",
                      body_chain_collider_cfg.link_length[0], config_path);
    body_chain_collider_cfg.link_length[1] =
        body_colliders_profile_float("joint02_link_length",
                      body_chain_collider_cfg.link_length[1], config_path);
    body_chain_collider_cfg.link_length[2] =
        body_colliders_profile_float("joint03_link_length",
                      body_chain_collider_cfg.link_length[2], config_path);
    body_chain_collider_cfg.response_strength =
        body_colliders_profile_float("collision_strength",
                      body_colliders_profile_float("response_strength",
                                    body_chain_collider_cfg.response_strength,
                                    config_path),
                      config_path);
    body_chain_collider_cfg.response_max_degrees_per_tick =
        body_colliders_profile_float("collision_max_degrees_per_tick",
                      body_colliders_profile_float("response_max_degrees_per_tick",
                                    body_chain_collider_cfg.response_max_degrees_per_tick,
                                    config_path),
                      config_path);
    body_chain_collider_cfg.collision_iterations =
        body_colliders_profile_int("collision_iterations",
                              body_chain_collider_cfg.collision_iterations,
                              config_path);
    body_chain_collider_cfg.collision_slop =
        body_colliders_profile_float("collision_slop",
                      body_chain_collider_cfg.collision_slop,
                      config_path);
    body_chain_collider_cfg.health_log_ms =
        body_colliders_profile_int("health_log_ms",
                              body_chain_collider_cfg.health_log_ms, config_path);
    body_chain_collider_cfg.response_log_ms =
        body_colliders_profile_int("response_log_ms",
                              body_chain_collider_cfg.response_log_ms, config_path);
    if (body_chain_collider_cfg.position_offset < 0 &&
        body_chain_collider_cfg.position_offset != -2) {
        body_chain_collider_cfg.position_offset = 0x0e8;
    }
    if (body_chain_collider_cfg.testicle_rotation_offset < 0) {
        body_chain_collider_cfg.testicle_rotation_offset = 0x06c;
    }
    body_chain_collider_cfg.pelvis_radius =
        physx_clampf(body_chain_collider_cfg.pelvis_radius, 0.005f, 2.0f);
    for (i = 0; i < 2; i++) {
        int axis;
        for (axis = 0; axis < 3; axis++) {
            body_chain_collider_cfg.stomach_radius[i][axis] =
                physx_clampf(body_chain_collider_cfg.stomach_radius[i][axis],
                             0.005f, 2.0f);
        }
    }
    body_chain_collider_cfg.hip_radius =
        physx_clampf(body_chain_collider_cfg.hip_radius, 0.005f, 2.0f);
    body_chain_collider_cfg.knee_radius =
        physx_clampf(body_chain_collider_cfg.knee_radius, 0.005f, 2.0f);
    if (body_chain_collider_cfg.thigh_radius < 0.0f) {
        body_chain_collider_cfg.thigh_radius =
            (body_chain_collider_cfg.hip_radius +
             body_chain_collider_cfg.knee_radius) * 0.5f;
    }
    body_chain_collider_cfg.thigh_radius =
        physx_clampf(body_chain_collider_cfg.thigh_radius, 0.005f, 2.0f);
    body_chain_collider_cfg.testicles_radius =
        physx_clampf(body_chain_collider_cfg.testicles_radius, 0.005f, 2.0f);
    {
        float pelvis_radius_vec[3];
        int node;
        body_chain_set_radius_scalar(pelvis_radius_vec,
                                     body_chain_collider_cfg.pelvis_radius);
        body_chain_set_node_radius(BODY_COLLIDER_ROOT, pelvis_radius_vec);
        for (node = 0; node < BODY_COLLIDER_NODE_COUNT; node++) {
            body_chain_clamp_radius_vec3(
                body_chain_collider_cfg.node_radius[node]);
        }
    }
    body_chain_collider_cfg.pelvis_capsule_radius =
        physx_clampf(body_chain_collider_cfg.pelvis_capsule_radius, 0.005f, 2.0f);
    body_chain_collider_cfg.response_radius_scale =
        physx_clampf(body_chain_collider_cfg.response_radius_scale, 0.05f, 4.0f);
    body_chain_collider_cfg.chain_radius =
        physx_clampf(body_chain_collider_cfg.chain_radius, 0.001f, 0.25f);
    body_chain_collider_cfg.link_length[0] =
        physx_clampf(body_chain_collider_cfg.link_length[0], 0.005f, 0.50f);
    body_chain_collider_cfg.link_length[1] =
        physx_clampf(body_chain_collider_cfg.link_length[1], 0.005f, 0.50f);
    body_chain_collider_cfg.link_length[2] =
        physx_clampf(body_chain_collider_cfg.link_length[2], 0.005f, 0.50f);
    body_chain_collider_cfg.response_strength =
        physx_clampf(body_chain_collider_cfg.response_strength, 0.0f, 4.0f);
    body_chain_collider_cfg.response_max_degrees_per_tick =
        physx_clampf(body_chain_collider_cfg.response_max_degrees_per_tick, 0.1f, 45.0f);
    if (body_chain_collider_cfg.collision_iterations < 1) {
        body_chain_collider_cfg.collision_iterations = 1;
    }
    if (body_chain_collider_cfg.collision_iterations > 6) {
        body_chain_collider_cfg.collision_iterations = 6;
    }
    body_chain_collider_cfg.collision_slop =
        physx_clampf(body_chain_collider_cfg.collision_slop, 0.0f, 0.02f);
    if (body_chain_collider_cfg.health_log_ms < 250) {
        body_chain_collider_cfg.health_log_ms = 250;
    }
    if (body_chain_collider_cfg.health_log_ms > 30000) {
        body_chain_collider_cfg.health_log_ms = 30000;
    }
    if (body_chain_collider_cfg.response_log_ms < 250) {
        body_chain_collider_cfg.response_log_ms = 250;
    }
    if (body_chain_collider_cfg.response_log_ms > 30000) {
        body_chain_collider_cfg.response_log_ms = 30000;
    }
    body_chain_physics_cfg.last_tick = 0;
    body_profile_rebuild_effective_configs();
    if (hot_reload) {
        config_hot_reload_tick = GetTickCount();
        /*
           Keep live collider samples across INI reloads.  A reload can happen
           while TK17 briefly exposes a camera-space body pivot; preserving the
           previous centerline lets the collider reject that one bad sample
           instead of accepting it as the new chain.
        */
    } else {
        config_hot_reload_tick = 0;
        reset_body_chain_physics();
    }
    {
        int i;
        for (i = 0; i < (int)(sizeof(axis_map_probe_samples) / sizeof(axis_map_probe_samples[0])); i++) {
            axis_map_probe_samples[i].base = NULL;
            axis_map_probe_samples[i].initialized = 0;
        }
        memset(&axis_root_scan, 0, sizeof(axis_root_scan));
    }

    get_file_write_time_a(config_path, &config_write_time);
    config_loaded = 1;
    config_reload_pending = 0;
    log_line("addon-physics config enabled=%d binding_probe=%d sidecar_hot_reload=%d interval_ms=%d max_checks_per_tick=%d note=\"hot reload checks only already-active sidecar timestamps, not the whole Addons tree\"",
             addon_physics_enabled,
             addon_physics_probe_enabled,
             addon_sidecar_hot_reload_enabled,
             addon_sidecar_hot_reload_interval_ms,
             addon_sidecar_hot_reload_max_checks_per_tick);
    log_line("config stiffness=%.3f damping=%.3f gravity=(%.3f %.3f %.3f) limit_angle=%.1f debug=%d config_reload_poll_ms=%d addon_physics_enabled=%d addon_binding_probe=%d body_probe enabled=%d person=\"%s\" node=\"%s\" use_object=%d offset=0x%03x axis=%d amount=%.3f duration_ms=%d cycle=%d transform_probe enabled=%d person=\"%s\" interval_ms=%d scan_floats=%d top_count=%d include_object=%d threshold=%.5f axis_map_probe enabled=%d person=\"%s\" interval_ms=%d threshold=%.5f root_drive_probe enabled=%d person=\"%s\" source=\"%s\" target=\"%s\" source_offset=0x%03x source_axis=%d target_offset=0x%03x target_axis=%d scale=%.3f max=%.3f threshold=%.5f interval_ms=%d write_sweep_probe enabled=%d person=\"%s\" duration_ms=%d start_delay_ms=%d candidates=%d penis_physics enabled=%d enabled_person=(%d,%d,%d,%d) root_offset=0x%03x output_offset=0x%03x translation=(source:%d/%d/%d,tail:%d/%d/%d,scale:%.3f/%.3f/%.3f) rotation=(source:%d/%d/%d,tail:%d/%d/%d,scale:%.3f/%.3f/%.3f) deadzone=(%.5f,%.5f) gravity_angle=%.2f stiffness=%.2f damping=%.2f max_angle=%.2f joint_max=(j1=%.2f/%.2f/%.2f,j2=%.2f/%.2f/%.2f,j3=%.2f/%.2f/%.2f) zero_rest=%d gains=(%.2f,%.2f,%.2f)",
             defaults_cfg.stiffness, defaults_cfg.damping, defaults_cfg.gravity[0],
             defaults_cfg.gravity[1], defaults_cfg.gravity[2], defaults_cfg.limit_angle,
             defaults_cfg.debug, defaults_cfg.config_reload_poll_ms,
             addon_physics_enabled, addon_physics_probe_enabled,
             body_probe_cfg.enabled, body_probe_cfg.person, body_probe_cfg.node,
             body_probe_cfg.use_object, body_probe_cfg.offset, body_probe_cfg.axis,
             body_probe_cfg.amount, body_probe_cfg.duration_ms, body_probe_cfg.cycle,
             transform_probe_cfg.enabled, transform_probe_cfg.person,
             transform_probe_cfg.interval_ms, transform_probe_cfg.scan_floats,
             transform_probe_cfg.top_count, transform_probe_cfg.include_object,
             transform_probe_cfg.threshold,
             axis_map_probe_cfg.enabled, axis_map_probe_cfg.person,
             axis_map_probe_cfg.interval_ms, axis_map_probe_cfg.threshold,
             root_drive_probe_cfg.enabled, root_drive_probe_cfg.person,
             root_drive_probe_cfg.source_node, root_drive_probe_cfg.target_node,
             root_drive_probe_cfg.source_offset, root_drive_probe_cfg.source_axis,
             root_drive_probe_cfg.target_offset, root_drive_probe_cfg.target_axis,
             root_drive_probe_cfg.scale, root_drive_probe_cfg.max_amount,
             root_drive_probe_cfg.threshold, root_drive_probe_cfg.interval_ms,
             write_sweep_probe_cfg.enabled, write_sweep_probe_cfg.person,
             write_sweep_probe_cfg.duration_ms,
             write_sweep_probe_cfg.start_delay_ms,
             (int)(sizeof(write_sweep_candidates) / sizeof(write_sweep_candidates[0])),
             body_chain_physics_cfg.enabled,
             body_chain_physics_cfg.enabled_person[0],
             body_chain_physics_cfg.enabled_person[1],
             body_chain_physics_cfg.enabled_person[2],
             body_chain_physics_cfg.enabled_person[3],
             body_chain_physics_cfg.root_offset, body_chain_physics_cfg.output_offset,
             body_chain_physics_cfg.translation_source_axis[0],
             body_chain_physics_cfg.translation_source_axis[1],
             body_chain_physics_cfg.translation_source_axis[2],
             body_chain_physics_cfg.translation_tail_axis[0],
             body_chain_physics_cfg.translation_tail_axis[1],
             body_chain_physics_cfg.translation_tail_axis[2],
             body_chain_physics_cfg.translation_scale[0],
             body_chain_physics_cfg.translation_scale[1],
             body_chain_physics_cfg.translation_scale[2],
             body_chain_physics_cfg.rotation_source_axis[0],
             body_chain_physics_cfg.rotation_source_axis[1],
             body_chain_physics_cfg.rotation_source_axis[2],
             body_chain_physics_cfg.rotation_tail_axis[0],
             body_chain_physics_cfg.rotation_tail_axis[1],
             body_chain_physics_cfg.rotation_tail_axis[2],
             body_chain_physics_cfg.rotation_scale[0],
             body_chain_physics_cfg.rotation_scale[1],
             body_chain_physics_cfg.rotation_scale[2],
             body_chain_physics_cfg.translation_deadzone,
             body_chain_physics_cfg.rotation_deadzone,
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
             body_chain_physics_cfg.link_max_angle[2][2],
             body_chain_physics_cfg.zero_output_rest,
             body_chain_physics_cfg.link_gain[0],
             body_chain_physics_cfg.link_gain[1],
             body_chain_physics_cfg.link_gain[2]);
    log_line("testicle-physics config enabled=%d enabled_person=(%d,%d,%d,%d) root_offset=0x%03x output_offset=0x%03x translation_scale=(%.3f,%.3f,%.3f) rotation_scale=(%.3f,%.3f,%.3f) stiffness=%.2f damping=%.2f joint_max=(j1=%.2f/%.2f/%.2f,j2=%.2f/%.2f/%.2f) gains=(%.2f,%.2f) interval_ms=%d note=\"uses penis_physics_internal mappings; owns Stesticles_joint01/02 and passively locks Stesticles_jointEnd only when enabled\"",
             testicle_physics_cfg.enabled,
             testicle_physics_cfg.enabled_person[0],
             testicle_physics_cfg.enabled_person[1],
             testicle_physics_cfg.enabled_person[2],
             testicle_physics_cfg.enabled_person[3],
             testicle_physics_cfg.root_offset,
             testicle_physics_cfg.output_offset,
             testicle_physics_cfg.translation_scale[0],
             testicle_physics_cfg.translation_scale[1],
             testicle_physics_cfg.translation_scale[2],
             testicle_physics_cfg.rotation_scale[0],
             testicle_physics_cfg.rotation_scale[1],
             testicle_physics_cfg.rotation_scale[2],
             testicle_physics_cfg.stiffness,
             testicle_physics_cfg.damping,
             testicle_physics_cfg.link_max_angle[0][0],
             testicle_physics_cfg.link_max_angle[0][1],
             testicle_physics_cfg.link_max_angle[0][2],
             testicle_physics_cfg.link_max_angle[1][0],
             testicle_physics_cfg.link_max_angle[1][1],
             testicle_physics_cfg.link_max_angle[1][2],
             testicle_physics_cfg.link_gain[0],
             testicle_physics_cfg.link_gain[1],
             testicle_physics_cfg.interval_ms);
    log_line("body-chain gravity-curve config penis=(h=%.2f,v=%.2f) testicle=(h=%.2f,v=%.2f) response_ms=%.1f note=\"direction-weighted curves preserve gravity direction; 1.0 is legacy and near-complete orientations recover full strength\"",
             body_chain_physics_cfg.gravity_horizontal_curve,
             body_chain_physics_cfg.gravity_vertical_curve,
             testicle_physics_cfg.gravity_horizontal_curve,
             testicle_physics_cfg.gravity_vertical_curve,
             physics_environment_cfg.gravity_response_ms);
    log_line("body-chain collision-scope breasts=%s butt=%s penis=%s testicle=%s note=\"full_body_all preserves legacy behavior; scopes without _all only refresh and test the wearer\"",
             body_chain_collision_scope_name(
                 breasts_physics_global_cfg.collision_scope),
             body_chain_collision_scope_name(
                 butt_physics_global_cfg.collision_scope),
             body_chain_collision_scope_name(
                 body_chain_physics_cfg.collision_scope),
             body_chain_collision_scope_name(
                 testicle_physics_cfg.collision_scope));
    log_line("physics-environment config world_gravity_probe=%d wind_enabled=%d world_gravity=(%.3f %.3f %.3f) gravity_horizontal_source=(%.3f %.3f %.3f) gravity_vertical_source=(%.3f %.3f %.3f) gravity_apply_to_body_chain=%d gravity_body_chain_scale=%.3f gravity_axis_scale=(h=%.3f,v=%.3f,hs_h=%.3f,hs_v=%.3f) gravity_tail_axis=(h=%d,v=%d) body_chain_camera_relative_orientation=%d camera_coast=(stiffness=%.3f,damping=%.3f,quarantine_ms=%d) dynamic_basis=%d basis_camera_compensate=%d basis_node=\"%s\" basis_offsets=(h=0x%03x,v=0x%03x,hs=0x%03x) basis_sign=(h=%.3f,v=%.3f,hs=%.3f) gravity_mapping=calibrated_axis_angle gravity_zero_at_start=%d response_ms=%.1f max_degrees_per_second=%.1f probe_settle_ms=%d probe_confirm_ms=%d probe_camera_quiet_ms=%d probe_log_ms=%d probe_motion_epsilon=%.6f probe_invalidate_epsilon=%.6f require_nonzero_root=%d debug_basis_candidate_logs=%d note=\"gravity and wind master switches are guarded by INI\"",
             physics_environment_cfg.world_gravity_probe,
             physics_environment_cfg.wind_enabled,
             physics_environment_cfg.world_gravity[0],
             physics_environment_cfg.world_gravity[1],
             physics_environment_cfg.world_gravity[2],
             physics_environment_cfg.gravity_horizontal_source_vector[0],
             physics_environment_cfg.gravity_horizontal_source_vector[1],
             physics_environment_cfg.gravity_horizontal_source_vector[2],
             physics_environment_cfg.gravity_vertical_source_vector[0],
             physics_environment_cfg.gravity_vertical_source_vector[1],
             physics_environment_cfg.gravity_vertical_source_vector[2],
             physics_environment_cfg.gravity_apply_to_body_chain,
             physics_environment_cfg.gravity_body_chain_scale,
             physics_environment_cfg.gravity_horizontal_body_chain_scale,
             physics_environment_cfg.gravity_vertical_body_chain_scale,
             physics_environment_cfg.gravity_horizontal_secondary_body_chain_scale,
             physics_environment_cfg.gravity_vertical_secondary_body_chain_scale,
             physics_environment_cfg.gravity_horizontal_tail_axis,
             physics_environment_cfg.gravity_vertical_tail_axis,
             physics_environment_cfg.body_chain_camera_relative_orientation,
             physics_environment_cfg.body_chain_camera_coast_stiffness_scale,
             physics_environment_cfg.body_chain_camera_coast_damping_scale,
             physics_environment_cfg.body_chain_camera_quarantine_ms,
             physics_environment_cfg.gravity_dynamic_body_basis,
             physics_environment_cfg.gravity_basis_camera_compensate,
             physics_environment_cfg.gravity_basis_node,
             physics_environment_cfg.gravity_horizontal_basis_offset,
             physics_environment_cfg.gravity_vertical_basis_offset,
             physics_environment_cfg.gravity_horizontal_secondary_basis_offset,
             physics_environment_cfg.gravity_horizontal_basis_sign,
             physics_environment_cfg.gravity_vertical_basis_sign,
             physics_environment_cfg.gravity_horizontal_secondary_basis_sign,
             physics_environment_cfg.gravity_zero_at_start,
             physics_environment_cfg.gravity_response_ms,
             physics_environment_cfg.gravity_max_degrees_per_second,
             physics_environment_cfg.gravity_probe_settle_ms,
             physics_environment_cfg.gravity_probe_confirm_ms,
             physics_environment_cfg.gravity_probe_camera_quiet_ms,
             physics_environment_cfg.gravity_probe_log_ms,
             physics_environment_cfg.gravity_probe_motion_epsilon,
             physics_environment_cfg.gravity_probe_invalidate_epsilon,
             physics_environment_cfg.gravity_probe_require_nonzero_root,
             defaults_cfg.debug);
    log_line("body-chain-colliders config enabled=%d debug_draw=%d response_enabled=%d breasts_collision=%d butt_collision=%d penis_collision=%d testicle_collision=%d diagnostic=%d live_bones=%d nodes=%d direct_nodes=%d edges=%d position_offset=0x%03x body_chain_base=(%.4f,%.4f,%.4f) testicle_fine_offsets=(%.4f,%.4f,%.4f;%.4f,%.4f,%.4f) spine_fine_offsets=(%.4f,%.4f,%.4f;%.4f,%.4f,%.4f) hip_fine_offset=(%.4f,%.4f,%.4f) thigh_fine_offset=(%.4f,%.4f,%.4f) knee_fine_offset=(%.4f,%.4f,%.4f) response_radius_scale=%.3f chain_radius=%.4f link_length=(%.4f,%.4f,%.4f) collision_strength=%.3f collision_max_degrees_per_tick=%.3f collision_iterations=%d collision_slop=%.5f collision_point_hold_ms=%d mode=\"unified visible-shape solver expanded-body\"",
             body_chain_collider_cfg.enabled,
             body_chain_collider_cfg.debug_draw,
             body_chain_collider_cfg.response_enabled,
             body_chain_collider_cfg.breasts_collision_enabled,
             body_chain_collider_cfg.butt_collision_enabled,
             body_chain_collider_cfg.penis_collision_enabled,
             body_chain_collider_cfg.testicle_collision_enabled,
             body_chain_collider_cfg.diagnostic,
             body_chain_collider_cfg.live_testicle_bones,
             BODY_COLLIDER_NODE_COUNT,
             BODY_COLLIDER_DIRECT_NODE_COUNT,
             BODY_COLLIDER_EXTRA_EDGE_COUNT,
             body_chain_collider_cfg.position_offset,
             body_chain_collider_cfg.body_chain_base_offset[0],
             body_chain_collider_cfg.body_chain_base_offset[1],
             body_chain_collider_cfg.body_chain_base_offset[2],
             body_chain_collider_cfg.testicle_fine_offset[0][0],
             body_chain_collider_cfg.testicle_fine_offset[0][1],
             body_chain_collider_cfg.testicle_fine_offset[0][2],
             body_chain_collider_cfg.testicle_fine_offset[1][0],
             body_chain_collider_cfg.testicle_fine_offset[1][1],
             body_chain_collider_cfg.testicle_fine_offset[1][2],
             body_chain_collider_cfg.stomach_fine_offset[0][0],
             body_chain_collider_cfg.stomach_fine_offset[0][1],
             body_chain_collider_cfg.stomach_fine_offset[0][2],
             body_chain_collider_cfg.stomach_fine_offset[1][0],
             body_chain_collider_cfg.stomach_fine_offset[1][1],
             body_chain_collider_cfg.stomach_fine_offset[1][2],
             body_chain_collider_cfg.hip_fine_offset[0],
             body_chain_collider_cfg.hip_fine_offset[1],
             body_chain_collider_cfg.hip_fine_offset[2],
             body_chain_collider_cfg.thigh_fine_offset[0],
             body_chain_collider_cfg.thigh_fine_offset[1],
             body_chain_collider_cfg.thigh_fine_offset[2],
             body_chain_collider_cfg.knee_fine_offset[0],
             body_chain_collider_cfg.knee_fine_offset[1],
             body_chain_collider_cfg.knee_fine_offset[2],
             body_chain_collider_cfg.response_radius_scale,
             body_chain_collider_cfg.chain_radius,
             body_chain_collider_cfg.link_length[0],
             body_chain_collider_cfg.link_length[1],
             body_chain_collider_cfg.link_length[2],
             body_chain_collider_cfg.response_strength,
             body_chain_collider_cfg.response_max_degrees_per_tick,
             body_chain_collider_cfg.collision_iterations,
             body_chain_collider_cfg.collision_slop,
             BODY_CHAIN_COLLISION_POINT_HOLD_MS);
    log_line("body-chain-colliders radius-vectors spine01=(%.4f,%.4f,%.4f) spine02=(%.4f,%.4f,%.4f) spine03=(%.4f,%.4f,%.4f) spine04=(%.4f,%.4f,%.4f) hip=(%.4f,%.4f,%.4f) thigh=(%.4f,%.4f,%.4f) knee=(%.4f,%.4f,%.4f) ankle=(%.4f,%.4f,%.4f) ball=(%.4f,%.4f,%.4f) testicles=(%.4f,%.4f,%.4f) breast=(%.4f,%.4f,%.4f) neck=(%.4f,%.4f,%.4f) head=(%.4f,%.4f,%.4f) note=\"all radius keys accept either scalar or three axes\"",
             body_chain_collider_cfg.node_radius[BODY_COLLIDER_STOMACH_01][0],
             body_chain_collider_cfg.node_radius[BODY_COLLIDER_STOMACH_01][1],
             body_chain_collider_cfg.node_radius[BODY_COLLIDER_STOMACH_01][2],
             body_chain_collider_cfg.node_radius[BODY_COLLIDER_STOMACH_02][0],
             body_chain_collider_cfg.node_radius[BODY_COLLIDER_STOMACH_02][1],
             body_chain_collider_cfg.node_radius[BODY_COLLIDER_STOMACH_02][2],
             body_chain_collider_cfg.node_radius[BODY_COLLIDER_STOMACH_03][0],
             body_chain_collider_cfg.node_radius[BODY_COLLIDER_STOMACH_03][1],
             body_chain_collider_cfg.node_radius[BODY_COLLIDER_STOMACH_03][2],
             body_chain_collider_cfg.node_radius[BODY_COLLIDER_STOMACH_04][0],
             body_chain_collider_cfg.node_radius[BODY_COLLIDER_STOMACH_04][1],
             body_chain_collider_cfg.node_radius[BODY_COLLIDER_STOMACH_04][2],
             body_chain_collider_cfg.node_radius[BODY_COLLIDER_HIP_L][0],
             body_chain_collider_cfg.node_radius[BODY_COLLIDER_HIP_L][1],
             body_chain_collider_cfg.node_radius[BODY_COLLIDER_HIP_L][2],
             body_chain_collider_cfg.node_radius[BODY_COLLIDER_THIGH_L][0],
             body_chain_collider_cfg.node_radius[BODY_COLLIDER_THIGH_L][1],
             body_chain_collider_cfg.node_radius[BODY_COLLIDER_THIGH_L][2],
             body_chain_collider_cfg.node_radius[BODY_COLLIDER_KNEE_L][0],
             body_chain_collider_cfg.node_radius[BODY_COLLIDER_KNEE_L][1],
             body_chain_collider_cfg.node_radius[BODY_COLLIDER_KNEE_L][2],
             body_chain_collider_cfg.node_radius[BODY_COLLIDER_ANKLE_L][0],
             body_chain_collider_cfg.node_radius[BODY_COLLIDER_ANKLE_L][1],
             body_chain_collider_cfg.node_radius[BODY_COLLIDER_ANKLE_L][2],
             body_chain_collider_cfg.node_radius[BODY_COLLIDER_BALL_L][0],
             body_chain_collider_cfg.node_radius[BODY_COLLIDER_BALL_L][1],
             body_chain_collider_cfg.node_radius[BODY_COLLIDER_BALL_L][2],
             body_chain_collider_cfg.node_radius[BODY_COLLIDER_TESTICLES_01][0],
             body_chain_collider_cfg.node_radius[BODY_COLLIDER_TESTICLES_01][1],
             body_chain_collider_cfg.node_radius[BODY_COLLIDER_TESTICLES_01][2],
             body_chain_collider_cfg.node_radius[BODY_COLLIDER_BREAST_L][0],
             body_chain_collider_cfg.node_radius[BODY_COLLIDER_BREAST_L][1],
             body_chain_collider_cfg.node_radius[BODY_COLLIDER_BREAST_L][2],
             body_chain_collider_cfg.node_radius[BODY_COLLIDER_NECK_01][0],
             body_chain_collider_cfg.node_radius[BODY_COLLIDER_NECK_01][1],
             body_chain_collider_cfg.node_radius[BODY_COLLIDER_NECK_01][2],
             body_chain_collider_cfg.node_radius[BODY_COLLIDER_HEAD_02][0],
             body_chain_collider_cfg.node_radius[BODY_COLLIDER_HEAD_02][1],
             body_chain_collider_cfg.node_radius[BODY_COLLIDER_HEAD_02][2]);
    {
        int collision_auto_step_count = 0;
        collision_auto_test_active_steps(&collision_auto_step_count);
        log_line("collision-auto-test config enabled=%d person=\"%s\" mode=\"%s\" start_delay_ms=%d phase_ms=%d rest_ms=%d root_amount=%.1f testicle_amount=%.1f hip_amount=%.1f steps=%d note=\"one automatic pass per game launch; every written rotation is restored before the next step\"",
                 collision_auto_test_cfg.enabled,
                 collision_auto_test_cfg.person,
                 collision_auto_test_cfg.mode,
                 collision_auto_test_cfg.start_delay_ms,
                 collision_auto_test_cfg.phase_ms,
                 collision_auto_test_cfg.rest_ms,
                 collision_auto_test_cfg.root_amount,
                 collision_auto_test_cfg.testicle_amount,
                 collision_auto_test_cfg.hip_amount,
                 collision_auto_step_count);
    }
    log_line("camera-contamination-test config enabled=%d person=\"%s\" start_delay_ms=%d hold_ms=%d sample_ms=%d input_drag_ms=%d input_yaw_pixels=%d input_pitch_pixels=%d auto_focus=%d steps=%d total_capture_ms=%d isolation=(gravity=1,colliders=1) camera_driver=\"sendinput-right-drag\" note=\"diagnostic suppresses gravity/collider forces and drives TK17's real camera input; no matrix spoofing\"",
             camera_contamination_test_cfg.enabled,
             camera_contamination_test_cfg.person,
             camera_contamination_test_cfg.start_delay_ms,
             camera_contamination_test_cfg.hold_ms,
             camera_contamination_test_cfg.sample_ms,
             camera_contamination_test_cfg.input_drag_ms,
             camera_contamination_test_cfg.input_yaw_pixels,
             camera_contamination_test_cfg.input_pitch_pixels,
             camera_contamination_test_cfg.auto_focus,
             CAMERA_CONTAMINATION_TEST_STEP_COUNT,
             CAMERA_CONTAMINATION_TEST_STEP_COUNT *
                 camera_contamination_test_cfg.hold_ms);
}

static void refresh_global_config(DWORD now)
{
    static DWORD last_poll_tick;
    FILETIME wt;
    int slot;
    if (!config_loaded) {
        load_global_config();
        return;
    }
    if (InterlockedExchange(&body_profile_reload_pending, 0)) {
        log_line("body-profile reload note=\"person-bound body sidecar changed; rebuilding global fallback plus per-person effective configs\"");
        load_global_config();
        return;
    }
    if (last_poll_tick &&
        now - last_poll_tick <
            (DWORD)defaults_cfg.config_reload_poll_ms) {
        return;
    }
    last_poll_tick = now;
    for (slot = 0; slot < 4; slot++) {
        FILETIME sidecar_wt;
        if (!body_profile_person_sidecar_active[slot]) continue;
        if (!get_file_write_time_a(body_profile_person_sidecar_path[slot],
                                   &sidecar_wt)) {
            body_profile_person_sidecar_active[slot] = 0;
            log_line("body-profile sidecar removed person=\"Person%02d\" path=\"%s\" note=\"falling back to global INI for this person\"",
                     slot + 1, body_profile_person_sidecar_path[slot]);
            body_profile_person_sidecar_path[slot][0] = 0;
            body_profile_person_body_path[slot][0] = 0;
            body_profile_person_body_hash[slot] = 0;
            body_profile_person_bind_strength[slot] = 0;
            load_global_config();
            return;
        }
        if (filetime_differs(&sidecar_wt,
                             &body_profile_person_sidecar_write_time[slot])) {
            body_profile_person_sidecar_write_time[slot] = sidecar_wt;
            log_line("body-profile sidecar reload person=\"Person%02d\" path=\"%s\" note=\"sidecar INI write detected\"",
                     slot + 1, body_profile_person_sidecar_path[slot]);
            load_global_config();
            return;
        }
    }
    if (!get_file_write_time_a(config_path, &wt)) return;
    if (!filetime_differs(&wt, &config_write_time)) return;

    if (!config_reload_pending ||
        filetime_differs(&wt, &config_reload_pending_write_time)) {
        config_reload_pending = 1;
        config_reload_pending_write_time = wt;
        config_reload_pending_tick = now;
        if (!config_reload_pending_log_tick ||
            now - config_reload_pending_log_tick >= 1000) {
            config_reload_pending_log_tick = now;
            log_line("config reload pending debounce_ms=%d note=\"INI write detected; waiting for editor save to settle before applying collider hot-reload\"",
                     1500);
        }
        return;
    }

    if (now - config_reload_pending_tick < 1500) {
        return;
    }

    config_reload_pending = 0;
    load_global_config();
}

