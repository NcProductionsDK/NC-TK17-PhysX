static int anchor_vector_offset_for_name(const char *name)
{
    if (!name || !name[0]) return -1;
    return -1;
}

static int physx_vec3_sane_limit(const float *v, float limit);

#define ADDON_BODY_PARENT_SCAN_SLOTS 16
#define ADDON_BODY_PARENT_SCAN_FLOATS 512
#define ADDON_BODY_PARENT_STEP_THRESHOLD 0.003f
#define ADDON_TJOINT_ROTATION_UNAVAILABLE (-2)

static float addon_vec3_len_exact(const float v[3])
{
    double length_sq;
    if (!v) return 0.0f;
    length_sq = (double)v[0] * (double)v[0] +
                (double)v[1] * (double)v[1] +
                (double)v[2] * (double)v[2];
    return length_sq > 0.0 ? (float)sqrt(length_sq) : 0.0f;
}

typedef struct addon_body_parent_scan_slot_t {
    void *base;
    int initialized;
    DWORD last_update_tick;
    DWORD log_tick;
    int last_step_offset;
    float last_step_len;
    float last_step[3];
    float previous[ADDON_BODY_PARENT_SCAN_FLOATS];
    char label[192];
} addon_body_parent_scan_slot_t;

static addon_body_parent_scan_slot_t addon_body_parent_scan_slots[
    ADDON_BODY_PARENT_SCAN_SLOTS];
static addon_body_parent_scan_slot_t addon_parent_translation_scan_slots[
    ADDON_BODY_PARENT_SCAN_SLOTS];

static const int addon_body_parent_transform_offsets[] = {
    0x06c, 0x078, 0x088, 0x098, 0x0a8, 0x0b8,
    0x0c8, 0x0d8, 0x0e8, 0x118, 0x128, 0x138,
    0x148, 0x338, 0x348, 0x358, 0x368, 0x378,
    0x388
};

static const int addon_parent_translation_offsets[] = {
    /* Keep translation drive on the matrix translation row only.  The nearby
       basis rows are camera/view-space sensitive and become distance-scaled
       when the camera is pushed close to the model. */
    0x0a8
};

static const int addon_body_parent_basis_triads[][3] = {
    { 0x078, 0x088, 0x098 },
    { 0x0c8, 0x0d8, 0x0e8 },
    { 0x118, 0x128, 0x138 },
    { 0x338, 0x348, 0x358 },
    { 0x368, 0x378, 0x388 }
};

static float *resolve_chain_anchor_vector(physx_chain_t *chain)
{
    void *raw = NULL;
    void *obj;
    char matched[384];
    const char *name;
    int off;
    if (!chain) return NULL;
    name = chain->attach_name[0] ? chain->attach_name : chain->anchor_name;
    if (!name || !name[0]) return NULL;
    off = anchor_vector_offset_for_name(name);
    if (off < 0) return NULL;
    obj = resolve_runtime_exact_target(name, &raw, matched, sizeof(matched));
    if (raw && ptr_readable((BYTE*)raw + off, sizeof(float) * 3)) {
        float *v = (float*)((BYTE*)raw + off);
        if (sane_probe_float(v[0]) && sane_probe_float(v[1]) && sane_probe_float(v[2])) {
            chain->anchor_raw_object = raw;
            chain->anchor_object = obj;
            chain->anchor_vector = v;
            chain->anchor_offset = off;
            if (!chain->anchor_logged) {
                chain->anchor_logged = 1;
                log_line("chain anchor resolved chain=\"%s\" anchor=\"%s\" runtime=\"%s\" raw=%p object=%p offset=0x%03x current=(%.5f,%.5f,%.5f)",
                         chain->name, name, matched, raw, obj, off, v[0], v[1], v[2]);
            }
            return v;
        }
    }
    return NULL;
}

static int inertial_local_drive_offset_for_name(const char *name)
{
    if (!name || !name[0]) return -1;
    return -1;
}

static float *resolve_inertial_auto_drive_vector(physx_chain_t *chain, const char *name)
{
    static const int offsets[] = {
        0x098, 0x0a8, 0x0b8, 0x0d8, 0x0e8,
        0x1d8, 0x1e8, 0x1f8, 0x208, 0x218, 0x228,
        0x2f8, 0x318, 0x328, 0x338, 0x348, 0x350,
        0x358, 0x368, 0x378, 0x380, 0x384, 0x388, 0x398
    };
    void *raw = NULL;
    void *obj;
    char matched[384];
    int i;
    int best_i = -1;
    float best_delta = -1.0f;
    float *best_v = NULL;
    if (!chain || !name || !name[0]) return NULL;
    obj = resolve_runtime_exact_target(name, &raw, matched, sizeof(matched));
    (void)obj;
    if (!raw) return NULL;
    for (i = 0; i < (int)(sizeof(offsets) / sizeof(offsets[0])); i++) {
        float *v;
        float len;
        float delta = 0.0f;
        if (!ptr_readable((BYTE*)raw + offsets[i], sizeof(float) * 3)) continue;
        v = (float*)((BYTE*)raw + offsets[i]);
        if (!sane_probe_float(v[0]) || !sane_probe_float(v[1]) || !sane_probe_float(v[2])) continue;
        len = (float)sqrt((double)(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]));
        if (len > 12.0f) continue;
        if (chain->drive_auto_initialized) {
            float dx = v[0] - chain->drive_auto_prev[i][0];
            float dy = v[1] - chain->drive_auto_prev[i][1];
            float dz = v[2] - chain->drive_auto_prev[i][2];
            delta = (float)sqrt((double)(dx * dx + dy * dy + dz * dz));
        }
        if (!best_v || delta > best_delta) {
            best_i = i;
            best_delta = delta;
            best_v = v;
        }
    }
    if (!best_v) {
        if (!chain->inertial_local_drive_logged) {
            chain->inertial_local_drive_logged = 1;
            log_line("inertial-chain auto drive unresolved chain=\"%s\" attach=\"%s\" drive=\"%s\" raw=%p reason=\"no sane raw transform row candidates\"",
                     chain->name, chain->attach_name[0] ? chain->attach_name : chain->anchor_name, name, raw);
        }
        return NULL;
    }
    if (!chain->drive_auto_initialized) {
        for (i = 0; i < (int)(sizeof(offsets) / sizeof(offsets[0])); i++) {
            if (ptr_readable((BYTE*)raw + offsets[i], sizeof(float) * 3)) {
                float *v = (float*)((BYTE*)raw + offsets[i]);
                if (sane_probe_float(v[0]) && sane_probe_float(v[1]) && sane_probe_float(v[2])) {
                    chain->drive_auto_prev[i][0] = v[0];
                    chain->drive_auto_prev[i][1] = v[1];
                    chain->drive_auto_prev[i][2] = v[2];
                }
            }
        }
        chain->drive_auto_initialized = 1;
        chain->drive_auto_offset = offsets[best_i];
        chain->anchor_raw_object = raw;
        chain->anchor_object = NULL;
        chain->anchor_vector = best_v;
        chain->anchor_offset = chain->drive_auto_offset;
        log_line("inertial-chain auto drive initialized chain=\"%s\" attach=\"%s\" drive=\"%s\" runtime=\"%s\" source=raw raw=%p offset=0x%03x current=(%.5f,%.5f,%.5f)",
                 chain->name, chain->attach_name[0] ? chain->attach_name : chain->anchor_name,
                 name, matched, raw, chain->drive_auto_offset, best_v[0], best_v[1], best_v[2]);
        return best_v;
    }
    if (best_delta > 0.0005f) {
        chain->drive_auto_offset = offsets[best_i];
        best_v = (float*)((BYTE*)raw + chain->drive_auto_offset);
        if (!chain->drive_auto_logged) {
            chain->drive_auto_logged = 1;
            log_line("inertial-chain auto drive selected chain=\"%s\" attach=\"%s\" drive=\"%s\" runtime=\"%s\" source=raw raw=%p offset=0x%03x delta=%.5f current=(%.5f,%.5f,%.5f) reason=\"selected changing raw local row\"",
                     chain->name, chain->attach_name[0] ? chain->attach_name : chain->anchor_name,
                     name, matched, raw, chain->drive_auto_offset, best_delta, best_v[0], best_v[1], best_v[2]);
        }
    } else if (chain->drive_auto_offset >= 0 && ptr_readable((BYTE*)raw + chain->drive_auto_offset, sizeof(float) * 3)) {
        best_v = (float*)((BYTE*)raw + chain->drive_auto_offset);
    }
    for (i = 0; i < (int)(sizeof(offsets) / sizeof(offsets[0])); i++) {
        if (ptr_readable((BYTE*)raw + offsets[i], sizeof(float) * 3)) {
            float *v = (float*)((BYTE*)raw + offsets[i]);
            if (sane_probe_float(v[0]) && sane_probe_float(v[1]) && sane_probe_float(v[2])) {
                chain->drive_auto_prev[i][0] = v[0];
                chain->drive_auto_prev[i][1] = v[1];
                chain->drive_auto_prev[i][2] = v[2];
            }
        }
    }
    chain->anchor_raw_object = raw;
    chain->anchor_object = NULL;
    chain->anchor_vector = best_v;
    chain->anchor_offset = chain->drive_auto_offset;
    return best_v;
}

static float *resolve_inertial_local_drive_vector(physx_chain_t *chain)
{
    void *raw = NULL;
    void *obj;
    char matched[384];
    const char *name;
    int off;
    if (!chain) return NULL;
    name = chain->drive_name[0] ? chain->drive_name : (chain->attach_name[0] ? chain->attach_name : chain->anchor_name);
    if (!name || !name[0]) return NULL;
    if (chain->drive_offset_override == -2) return resolve_inertial_auto_drive_vector(chain, name);
    off = chain->drive_offset_override >= 0 ? chain->drive_offset_override : inertial_local_drive_offset_for_name(name);
    if (off < 0) {
        if (!chain->inertial_local_drive_logged) {
            chain->inertial_local_drive_logged = 1;
            log_line("inertial-chain local drive unavailable chain=\"%s\" attach=\"%s\" drive=\"%s\" reason=\"no verified camera-safe raw local row for this drive node\"",
                     chain->name, chain->attach_name[0] ? chain->attach_name : chain->anchor_name, name);
        }
        return NULL;
    }
    obj = resolve_runtime_exact_target(name, &raw, matched, sizeof(matched));
    (void)obj;
    if (raw && ptr_readable((BYTE*)raw + off, sizeof(float) * 3)) {
        float *v = (float*)((BYTE*)raw + off);
        if (sane_probe_float(v[0]) && sane_probe_float(v[1]) && sane_probe_float(v[2])) {
            chain->anchor_raw_object = raw;
            chain->anchor_object = NULL;
            chain->anchor_vector = v;
            chain->anchor_offset = off;
            if (!chain->inertial_local_drive_logged) {
                chain->inertial_local_drive_logged = 1;
                log_line("inertial-chain local drive resolved chain=\"%s\" attach=\"%s\" drive=\"%s\" runtime=\"%s\" source=raw raw=%p offset=0x%03x current=(%.5f,%.5f,%.5f) reason=\"drive raw row changed on Pose Editor head movement while object rows were camera-contaminated\"",
                         chain->name, chain->attach_name[0] ? chain->attach_name : chain->anchor_name,
                         name, matched, raw, off, v[0], v[1], v[2]);
            }
            return v;
        }
    }
    if (!chain->inertial_local_drive_logged) {
        chain->inertial_local_drive_logged = 1;
        log_line("inertial-chain local drive unresolved chain=\"%s\" attach=\"%s\" drive=\"%s\" offset=0x%03x raw=%p reason=\"raw row missing or invalid\"",
                 chain->name, chain->attach_name[0] ? chain->attach_name : chain->anchor_name, name, off, raw);
    }
    return NULL;
}

static int resolve_addon_chain_parent_pivot(physx_chain_t *chain,
                                            physx_target_t *parent,
                                            float out[3])
{
    if (!chain || !parent || !out || !parent->name[0]) return 0;
    if (!chain->addon_chain || parent->addon_simulated_target) return 0;
    if (parent->s_translation_base &&
        parent->s_translation_offset >= 0 &&
        ptr_readable((BYTE*)parent->s_translation_base + parent->s_translation_offset,
                     sizeof(float) * 3)) {
        float *v = (float*)((BYTE*)parent->s_translation_base +
                            parent->s_translation_offset);
        if (sane_probe_float(v[0]) && sane_probe_float(v[1]) &&
            sane_probe_float(v[2])) {
            out[0] = v[0];
            out[1] = v[1];
            out[2] = v[2];
            chain->anchor_raw_object = parent->s_translation_base;
            chain->anchor_object = parent->s_object;
            chain->anchor_vector = v;
            chain->anchor_offset = parent->s_translation_offset;
            if (!chain->anchor_logged) {
                chain->anchor_logged = 1;
                log_line("addon-chain root anchor resolved chain=\"%s\" parent=\"%s\" source=%s base=%p offset=0x%03x current=(%.5f,%.5f,%.5f) note=\"using parent SJoint translation as root drive for the first custom add-on PhysX link\"",
                         chain->name, parent->name,
                         parent->s_translation_source ? parent->s_translation_source : "s",
                         parent->s_translation_base,
                         parent->s_translation_offset,
                         out[0], out[1], out[2]);
            }
            return 1;
        }
    }
    if (!chain->anchor_logged) {
        chain->anchor_logged = 1;
        log_line("addon-chain root anchor unresolved chain=\"%s\" parent=\"%s\" parent_object=%p parent_raw=%p s_object=%p note=\"strict parent-local mode requires the defined parent bone SJoint translation; first custom add-on PhysX link will stay rest-anchored instead of using body/root fallback rows\"",
                 chain->name,
                 parent->name,
                 parent->object,
                 parent->raw_object,
                 parent->s_object);
    }
    return 0;
}

static float addon_angle_delta_degrees(float current, float previous)
{
    float d = current - previous;
    while (d > 180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}

static int addon_vector_basis_like(const float v[3])
{
    float len;
    if (!v) return 0;
    if (physx_absf(v[0]) > 1.35f ||
        physx_absf(v[1]) > 1.35f ||
        physx_absf(v[2]) > 1.35f) {
        return 0;
    }
    len = physx_vec3_len(v);
    return len >= 0.45f && len <= 1.35f;
}

static void addon_parent_vector_step_to_degrees(const float current[3],
                                                const float previous[3],
                                                float out[3],
                                                int *basis_like_out)
{
    int basis_like =
        addon_vector_basis_like(current) &&
        addon_vector_basis_like(previous);
    if (basis_like) {
        out[0] = (current[0] - previous[0]) * 57.29577951308232f;
        out[1] = (current[1] - previous[1]) * 57.29577951308232f;
        out[2] = (current[2] - previous[2]) * 57.29577951308232f;
    } else {
        out[0] = addon_angle_delta_degrees(current[0], previous[0]);
        out[1] = addon_angle_delta_degrees(current[1], previous[1]);
        out[2] = addon_angle_delta_degrees(current[2], previous[2]);
    }
    if (basis_like_out) *basis_like_out = basis_like;
}

static int addon_read_normalized_basis_from_snapshot(
    const float previous[ADDON_BODY_PARENT_SCAN_FLOATS],
    const int offsets[3],
    float out[9])
{
    int row;
    if (!previous || !offsets || !out) return 0;
    for (row = 0; row < 3; row++) {
        int idx = offsets[row] / (int)sizeof(float);
        float len;
        if (idx < 0 || idx > ADDON_BODY_PARENT_SCAN_FLOATS - 3) return 0;
        if (!sane_probe_float(previous[idx]) ||
            !sane_probe_float(previous[idx + 1]) ||
            !sane_probe_float(previous[idx + 2])) {
            return 0;
        }
        len = (float)sqrt((double)(previous[idx] * previous[idx] +
                                   previous[idx + 1] * previous[idx + 1] +
                                   previous[idx + 2] * previous[idx + 2]));
        if (len < 0.000001f || len > 1000.0f) return 0;
        out[row * 3 + 0] = previous[idx] / len;
        out[row * 3 + 1] = previous[idx + 1] / len;
        out[row * 3 + 2] = previous[idx + 2] / len;
    }
    return 1;
}

static int addon_read_normalized_basis_triad(void *base,
                                             const int offsets[3],
                                             float out[9])
{
    int row;
    if (!base || !offsets || !out) return 0;
    for (row = 0; row < 3; row++) {
        if (!read_normalized_basis_vector(base, offsets[row],
                                          &out[row * 3])) {
            return 0;
        }
    }
    return 1;
}

static int addon_basis_rows_orthonormal_enough(const float rows[9])
{
    float d01, d02, d12;
    if (!rows) return 0;
    d01 = physx_absf(vec3_dot(&rows[0], &rows[3]));
    d02 = physx_absf(vec3_dot(&rows[0], &rows[6]));
    d12 = physx_absf(vec3_dot(&rows[3], &rows[6]));
    return d01 < 0.28f && d02 < 0.28f && d12 < 0.28f;
}

static int addon_normalize_basis_rows(float rows[9])
{
    int row;
    if (!rows) return 0;
    for (row = 0; row < 3; row++) {
        float *r = &rows[row * 3];
        float len;
        if (!sane_probe_float(r[0]) ||
            !sane_probe_float(r[1]) ||
            !sane_probe_float(r[2])) {
            return 0;
        }
        len = physx_vec3_len(r);
        if (len < 0.000001f || len > 8.0f) return 0;
        r[0] /= len;
        r[1] /= len;
        r[2] /= len;
    }
    return addon_basis_rows_orthonormal_enough(rows);
}

static void addon_basis_rows_to_euler_degrees(const float rows[9],
                                              float out[3])
{
    float sy;
    float cy;
    if (!rows || !out) return;
    sy = physx_clampf(-rows[2], -1.0f, 1.0f);
    out[1] = (float)(asin((double)sy) * 57.29577951308232);
    cy = (float)cos((double)(out[1] * 0.017453292519943295f));
    if (physx_absf(cy) > 0.0001f) {
        out[0] = (float)(atan2((double)rows[5],
                               (double)rows[8]) *
                         57.29577951308232);
        out[2] = (float)(atan2((double)rows[1],
                               (double)rows[0]) *
                         57.29577951308232);
    } else {
        out[0] = 0.0f;
        out[2] = (float)(atan2((double)(-rows[3]),
                               (double)rows[4]) *
                         57.29577951308232);
    }
}

static void addon_basis_triad_step_to_degrees(const float current[9],
                                              const float previous[9],
                                              float out[3])
{
    float current_euler[3];
    float previous_euler[3];
    if (!current || !previous || !out) return;
    addon_basis_rows_to_euler_degrees(current, current_euler);
    addon_basis_rows_to_euler_degrees(previous, previous_euler);
    out[0] = addon_angle_delta_degrees(current_euler[0],
                                       previous_euler[0]);
    out[1] = addon_angle_delta_degrees(current_euler[1],
                                       previous_euler[1]);
    out[2] = addon_angle_delta_degrees(current_euler[2],
                                       previous_euler[2]);
}

static int addon_basis_triad_relative_step_to_degrees(
    const float current[9],
    const float previous[9],
    float out[3])
{
    const float rad_to_deg = 57.29577951308232f;
    float previous_inverse[9];
    float relative_rotation[9];
    if (!current || !previous || !out) return 0;
    out[0] = 0.0f;
    out[1] = 0.0f;
    out[2] = 0.0f;
    if (!body_chain_mat3_inverse(previous, previous_inverse)) return 0;

    /* Measure the incremental rotation directly. Absolute Euler subtraction
       becomes unstable when the owner's body approaches a 90-degree pose. */
    body_chain_mat3_multiply(current, previous_inverse,
                             relative_rotation);
    out[0] = physx_clampf(
        (relative_rotation[7] - relative_rotation[5]) *
            0.5f * rad_to_deg,
        -45.0f, 45.0f);
    out[1] = physx_clampf(
        (relative_rotation[2] - relative_rotation[6]) *
            0.5f * rad_to_deg,
        -45.0f, 45.0f);
    out[2] = physx_clampf(
        (relative_rotation[3] - relative_rotation[1]) *
            0.5f * rad_to_deg,
        -45.0f, 45.0f);
    return 1;
}

static int addon_parent_name_can_use_body_drive(const char *name)
{
    if (!name || !name[0]) return 0;
    if (contains_i(name, "_joint")) return 1;
    if (_stricmp(name, "root") == 0 || _stricmp(name, "Sroot") == 0) return 1;
    if (contains_i(name, "head") || contains_i(name, "neck")) return 1;
    return 0;
}

static int addon_extract_person_prefix(const char *name, char *out,
                                       size_t outsz)
{
    if (out && outsz) out[0] = 0;
    if (!name || !out || outsz < 9) return 0;
    if (_strnicmp(name, "Person", 6) != 0) return 0;
    if (name[6] < '0' || name[6] > '9' ||
        name[7] < '0' || name[7] > '9') {
        return 0;
    }
    memcpy(out, name, 8);
    out[8] = 0;
    return 1;
}

static int addon_person_prefix_to_index(const char *person)
{
    if (!person) return -1;
    if (_stricmp(person, "Person01") == 0) return 0;
    if (_stricmp(person, "Person02") == 0) return 1;
    if (_stricmp(person, "Person03") == 0) return 2;
    if (_stricmp(person, "Person04") == 0) return 3;
    return -1;
}

static void addon_chain_note_body_root_person(physx_chain_t *chain,
                                              const char *name,
                                              DWORD now,
                                              const char *source)
{
    char person[16];
    if (!chain || !addon_extract_person_prefix(name, person, sizeof(person))) {
        return;
    }
    if (_stricmp(chain->addon_body_root_person, person) != 0) {
        lstrcpynA(chain->addon_body_root_person, person,
                  sizeof(chain->addon_body_root_person));
        chain->addon_body_root_raw = NULL;
        chain->addon_body_root_initialized = 0;
        chain->addon_body_root_log_tick = 0;
        chain->addon_body_root_miss_log_tick = 0;
        log_line("addon-chain body owner inferred chain=\"%s\" person=\"%s\" source=%s name=\"%s\" note=\"custom sidecar root translation drive will sample this body's root\"",
                 chain->name,
                 chain->addon_body_root_person,
                 source ? source : "unknown",
                 name);
    } else {
        chain->addon_body_root_miss_log_tick = now;
    }
}

static void sidecar_note_live_addon_owner_person(physx_sidecar_t *sc,
                                                 const char *owner_prefix,
                                                 const char *root_name,
                                                 DWORD now)
{
    char person[16];
    const char *source = owner_prefix && owner_prefix[0] ?
                         owner_prefix : root_name;
    if (!sc || !addon_extract_person_prefix(source, person, sizeof(person))) {
        return;
    }
    sc->addon_scene_active = 1;
    sc->addon_scene_active_tick = now;
    sc->addon_owner_seen_tick = now;
    if (_stricmp(sc->addon_owner_person, person) != 0) {
        lstrcpynA(sc->addon_owner_person, person,
                  sizeof(sc->addon_owner_person));
        sc->addon_owner_logged = 0;
    }
    if (!sc->addon_owner_logged) {
        sc->addon_owner_logged = 1;
        log_line("addon sidecar owner inferred person=\"%s\" root=\"%s\" owner_prefix=\"%s\" sidecar=\"%s\" note=\"add-on PhysX body translation drive will use this person root when possible\"",
                 sc->addon_owner_person,
                 root_name ? root_name : "",
                 owner_prefix ? owner_prefix : "",
                 sc->path);
    }
}

static addon_body_parent_scan_slot_t *addon_body_parent_scan_slot_for_base(
    void *base, DWORD now)
{
    addon_body_parent_scan_slot_t *oldest = NULL;
    DWORD oldest_age = 0;
    int i;
    if (!base) return NULL;
    for (i = 0; i < ADDON_BODY_PARENT_SCAN_SLOTS; i++) {
        addon_body_parent_scan_slot_t *slot = &addon_body_parent_scan_slots[i];
        if (slot->base == base) return slot;
        if (!slot->base) return slot;
        if (!oldest || now - slot->last_update_tick > oldest_age) {
            oldest = slot;
            oldest_age = now - slot->last_update_tick;
        }
    }
    if (oldest) {
        memset(oldest, 0, sizeof(*oldest));
        return oldest;
    }
    return NULL;
}

static int addon_body_parent_capture_snapshot(addon_body_parent_scan_slot_t *slot,
                                              void *base)
{
    BYTE *b = (BYTE*)base;
    int captured = 0;
    int i;
    if (!slot || !base) return 0;
    for (i = 0; i < (int)(sizeof(addon_body_parent_transform_offsets) /
                           sizeof(addon_body_parent_transform_offsets[0])); i++) {
        int off = addon_body_parent_transform_offsets[i];
        int idx = off / (int)sizeof(float);
        float *v;
        if (idx < 0 || idx > ADDON_BODY_PARENT_SCAN_FLOATS - 3) continue;
        slot->previous[idx] = 20000.0f;
        slot->previous[idx + 1] = 20000.0f;
        slot->previous[idx + 2] = 20000.0f;
        if (!ptr_readable(b + off, sizeof(float) * 3)) continue;
        v = (float*)(b + off);
        if (!physx_vec3_sane_limit(v, 720.0f)) continue;
        slot->previous[idx] = v[0];
        slot->previous[idx + 1] = v[1];
        slot->previous[idx + 2] = v[2];
        captured++;
    }
    return captured;
}

static addon_body_parent_scan_slot_t *addon_parent_translation_scan_slot_for_base(
    void *base, DWORD now)
{
    addon_body_parent_scan_slot_t *oldest = NULL;
    DWORD oldest_age = 0;
    int i;
    if (!base) return NULL;
    for (i = 0; i < ADDON_BODY_PARENT_SCAN_SLOTS; i++) {
        addon_body_parent_scan_slot_t *slot =
            &addon_parent_translation_scan_slots[i];
        if (slot->base == base) return slot;
        if (!slot->base) return slot;
        if (!oldest || now - slot->last_update_tick > oldest_age) {
            oldest = slot;
            oldest_age = now - slot->last_update_tick;
        }
    }
    if (oldest) {
        memset(oldest, 0, sizeof(*oldest));
        return oldest;
    }
    return NULL;
}

static int addon_parent_translation_capture_snapshot(
    addon_body_parent_scan_slot_t *slot,
    void *base)
{
    BYTE *b = (BYTE*)base;
    int captured = 0;
    int i;
    if (!slot || !base) return 0;
    for (i = 0;
         i < (int)(sizeof(addon_parent_translation_offsets) /
                   sizeof(addon_parent_translation_offsets[0]));
         i++) {
        int off = addon_parent_translation_offsets[i];
        int idx = off / (int)sizeof(float);
        float *v;
        if (idx < 0 || idx > ADDON_BODY_PARENT_SCAN_FLOATS - 3) continue;
        slot->previous[idx] = 20000.0f;
        slot->previous[idx + 1] = 20000.0f;
        slot->previous[idx + 2] = 20000.0f;
        if (!ptr_readable(b + off, sizeof(float) * 3)) continue;
        v = (float*)(b + off);
        if (!physx_vec3_sane_limit(v, 64.0f)) continue;
        slot->previous[idx] = v[0];
        slot->previous[idx + 1] = v[1];
        slot->previous[idx + 2] = v[2];
        captured++;
    }
    return captured;
}

static int addon_effective_parent_translation_scan_base(
    physx_chain_t *chain,
    physx_target_t *parent,
    void *base,
    const char *source,
    float out[3],
    int *offset_out,
    DWORD now)
{
    addon_body_parent_scan_slot_t *slot;
    BYTE *b;
    int i;
    int best_offset = -1;
    int best_basis_like = 0;
    float best_step[3] = { 0.0f, 0.0f, 0.0f };
    float best_current[3] = { 0.0f, 0.0f, 0.0f };
    float best_len = 0.0f;
    int best_non_basis_offset = -1;
    float best_non_basis_step[3] = { 0.0f, 0.0f, 0.0f };
    float best_non_basis_current[3] = { 0.0f, 0.0f, 0.0f };
    float best_non_basis_len = 0.0f;
    if (!chain || !parent || !base || !out) return 0;
    if (!ptr_readable(base, sizeof(float))) return 0;
    slot = addon_parent_translation_scan_slot_for_base(base, now);
    if (!slot) return 0;
    b = (BYTE*)base;
    if (slot->base != base) {
        memset(slot, 0, sizeof(*slot));
        slot->base = base;
    }
    if (slot->last_update_tick == now) {
        if (slot->last_step_len >= ADDON_BODY_PARENT_STEP_THRESHOLD) {
            out[0] = slot->last_step[0];
            out[1] = slot->last_step[1];
            out[2] = slot->last_step[2];
            if (offset_out) *offset_out = slot->last_step_offset;
            return 1;
        }
        return 0;
    }
    if (!slot->initialized) {
        int captured = addon_parent_translation_capture_snapshot(slot, base);
        slot->initialized = captured > 0;
        slot->last_update_tick = now;
        slot->last_step_offset = -1;
        slot->last_step_len = 0.0f;
        slot->last_step[0] = slot->last_step[1] =
            slot->last_step[2] = 0.0f;
        if (captured > 0 && defaults_cfg.debug) {
            _snprintf(slot->label, sizeof(slot->label), "%s:%s",
                      source ? source : "unknown",
                      parent->name);
            slot->label[sizeof(slot->label) - 1] = 0;
            log_line("addon-chain effective parent translation baseline chain=\"%s\" parent=\"%s\" source=%s base=%p captured_offsets=%d note=\"direct parent target only; ancestor/root motion can drive the add-on chain only through this parent's final transform\"",
                     chain->name,
                     parent->name,
                     source ? source : "unknown",
                     base,
                     captured);
        }
        return 0;
    }

    for (i = 0;
         i < (int)(sizeof(addon_parent_translation_offsets) /
                   sizeof(addon_parent_translation_offsets[0]));
         i++) {
        int off = addon_parent_translation_offsets[i];
        int idx = off / (int)sizeof(float);
        float *v;
        float previous[3];
        float current[3];
        float step[3];
        float len;
        int basis_like;
        if (idx < 0 || idx > ADDON_BODY_PARENT_SCAN_FLOATS - 3) continue;
        if (!ptr_readable(b + off, sizeof(float) * 3)) continue;
        v = (float*)(b + off);
        if (!physx_vec3_sane_limit(v, 64.0f)) continue;
        if (!sane_probe_float(slot->previous[idx]) ||
            !sane_probe_float(slot->previous[idx + 1]) ||
            !sane_probe_float(slot->previous[idx + 2])) {
            continue;
        }
        current[0] = v[0];
        current[1] = v[1];
        current[2] = v[2];
        previous[0] = slot->previous[idx];
        previous[1] = slot->previous[idx + 1];
        previous[2] = slot->previous[idx + 2];
        if (!physx_vec3_sane_limit(previous, 64.0f)) continue;
        step[0] = current[0] - previous[0];
        step[1] = current[1] - previous[1];
        step[2] = current[2] - previous[2];
        len = addon_vec3_len_exact(step);
        if (len < ADDON_BODY_PARENT_STEP_THRESHOLD || len > 4.0f) continue;
        basis_like =
            addon_vector_basis_like(current) &&
            addon_vector_basis_like(previous);
        if (len > best_len) {
            best_len = len;
            best_offset = off;
            best_basis_like = basis_like;
            best_step[0] = step[0];
            best_step[1] = step[1];
            best_step[2] = step[2];
            best_current[0] = current[0];
            best_current[1] = current[1];
            best_current[2] = current[2];
        }
        if (!basis_like && len > best_non_basis_len) {
            best_non_basis_len = len;
            best_non_basis_offset = off;
            best_non_basis_step[0] = step[0];
            best_non_basis_step[1] = step[1];
            best_non_basis_step[2] = step[2];
            best_non_basis_current[0] = current[0];
            best_non_basis_current[1] = current[1];
            best_non_basis_current[2] = current[2];
        }
    }

    if (best_non_basis_offset >= 0) {
        best_offset = best_non_basis_offset;
        best_len = best_non_basis_len;
        best_basis_like = 0;
        best_step[0] = best_non_basis_step[0];
        best_step[1] = best_non_basis_step[1];
        best_step[2] = best_non_basis_step[2];
        best_current[0] = best_non_basis_current[0];
        best_current[1] = best_non_basis_current[1];
        best_current[2] = best_non_basis_current[2];
    }

    addon_parent_translation_capture_snapshot(slot, base);
    slot->last_update_tick = now;
    slot->last_step_offset = best_offset;
    slot->last_step_len = best_len;
    slot->last_step[0] = best_step[0];
    slot->last_step[1] = best_step[1];
    slot->last_step[2] = best_step[2];
    if (best_offset >= 0 && best_len >= ADDON_BODY_PARENT_STEP_THRESHOLD) {
        out[0] = best_step[0];
        out[1] = best_step[1];
        out[2] = best_step[2];
        if (offset_out) *offset_out = best_offset;
        if (defaults_cfg.debug &&
            (!slot->log_tick || now - slot->log_tick >= 500u)) {
            slot->log_tick = now;
            log_line("addon-chain effective parent translation step chain=\"%s\" parent=\"%s\" source=%s base=%p offset=0x%03x current=(%.5f,%.5f,%.5f) step=(%.5f,%.5f,%.5f) step_len=%.5f basis_like=%d note=\"configured parent final transform moved; no separate neck/root/body drive was sampled\"",
                     chain->name,
                     parent->name,
                     source ? source : "unknown",
                     base,
                     best_offset,
                     best_current[0], best_current[1], best_current[2],
                     best_step[0], best_step[1], best_step[2],
                     best_len,
                     best_basis_like);
        }
        return 1;
    }
    return 0;
}

static int addon_body_parent_scan_base(physx_chain_t *chain,
                                       physx_target_t *parent,
                                       void *base,
                                       const char *runtime,
                                       const char *source,
                                       float out[3],
                                       int *offset_out,
                                       int basis_only,
                                       DWORD now)
{
    addon_body_parent_scan_slot_t *slot;
    BYTE *b;
    int i;
    int best_offset = -1;
    int best_basis_like = 0;
    float best_step[3] = { 0.0f, 0.0f, 0.0f };
    float best_current[3] = { 0.0f, 0.0f, 0.0f };
    float best_len = 0.0f;
    if (!chain || !parent || !base || !out) return 0;
    if (!ptr_readable(base, sizeof(float))) return 0;
    slot = addon_body_parent_scan_slot_for_base(base, now);
    if (!slot) return 0;
    b = (BYTE*)base;
    if (slot->base != base) {
        memset(slot, 0, sizeof(*slot));
        slot->base = base;
    }
    if (slot->last_update_tick == now) {
        if (slot->last_step_len >= ADDON_BODY_PARENT_STEP_THRESHOLD) {
            out[0] = slot->last_step[0];
            out[1] = slot->last_step[1];
            out[2] = slot->last_step[2];
            if (offset_out) *offset_out = slot->last_step_offset;
            return 1;
        }
        return 0;
    }
    if (!slot->initialized) {
        int captured = addon_body_parent_capture_snapshot(slot, base);
        slot->initialized = captured > 0;
        slot->last_update_tick = now;
        slot->last_step_offset = -1;
        slot->last_step_len = 0.0f;
        slot->last_step[0] = slot->last_step[1] = slot->last_step[2] = 0.0f;
        if (captured > 0) {
            _snprintf(slot->label, sizeof(slot->label), "%s:%s",
                      source ? source : "unknown",
                      runtime ? runtime : "");
            slot->label[sizeof(slot->label) - 1] = 0;
            log_line("addon-chain effective parent rotation baseline chain=\"%s\" parent=\"%s\" source=%s runtime=\"%s\" base=%p captured_floats=%d note=\"watching this parent-owned transform candidate for configured-parent movement\"",
                     chain->name,
                     parent->name,
                     source ? source : "unknown",
                     runtime ? runtime : "",
                     base,
                     captured);
        }
        return 0;
    }

    {
        int triad_i;
        int best_triad_i = -1;
        float best_basis_step[3] = { 0.0f, 0.0f, 0.0f };
        float best_basis_len = 0.0f;
        float best_basis_current[9];
        for (triad_i = 0;
             triad_i < (int)(sizeof(addon_body_parent_basis_triads) /
                              sizeof(addon_body_parent_basis_triads[0]));
             triad_i++) {
            const int *triad = addon_body_parent_basis_triads[triad_i];
            float current_basis[9];
            float previous_basis[9];
            float step[3];
            float len;
            if (!addon_read_normalized_basis_triad(base, triad,
                                                   current_basis)) {
                continue;
            }
            if (!addon_read_normalized_basis_from_snapshot(slot->previous,
                                                           triad,
                                                           previous_basis)) {
                continue;
            }
            if (!addon_basis_rows_orthonormal_enough(current_basis) ||
                !addon_basis_rows_orthonormal_enough(previous_basis)) {
                continue;
            }
            addon_basis_triad_step_to_degrees(current_basis,
                                              previous_basis,
                                              step);
            len = addon_vec3_len_exact(step);
            if (len > best_basis_len && len < 95.0f) {
                best_triad_i = triad_i;
                best_basis_len = len;
                best_basis_step[0] = step[0];
                best_basis_step[1] = step[1];
                best_basis_step[2] = step[2];
                memcpy(best_basis_current, current_basis,
                       sizeof(best_basis_current));
            }
        }
        if (best_triad_i >= 0 &&
            best_basis_len >= ADDON_BODY_PARENT_STEP_THRESHOLD) {
            const int *triad = addon_body_parent_basis_triads[best_triad_i];
            addon_body_parent_capture_snapshot(slot, base);
            slot->last_update_tick = now;
            slot->last_step_offset = triad[0];
            slot->last_step_len = best_basis_len;
            slot->last_step[0] = best_basis_step[0];
            slot->last_step[1] = best_basis_step[1];
            slot->last_step[2] = best_basis_step[2];
            out[0] = best_basis_step[0];
            out[1] = best_basis_step[1];
            out[2] = best_basis_step[2];
            if (offset_out) *offset_out = triad[0];
            if (defaults_cfg.debug &&
                (!slot->log_tick || now - slot->log_tick >= 500u)) {
                slot->log_tick = now;
                log_line("addon-chain effective parent rotation basis-step chain=\"%s\" parent=\"%s\" source=%s runtime=\"%s\" base=%p offsets=(0x%03x,0x%03x,0x%03x) step=(%.5f,%.5f,%.5f) step_len=%.5f row0=(%.5f,%.5f,%.5f) row1=(%.5f,%.5f,%.5f) row2=(%.5f,%.5f,%.5f) note=\"configured parent final basis changed; ancestor/root motion contributes only through this parent transform\"",
                         chain->name,
                         parent->name,
                         source ? source : "unknown",
                         runtime ? runtime : "",
                         base,
                         triad[0], triad[1], triad[2],
                         best_basis_step[0],
                         best_basis_step[1],
                         best_basis_step[2],
                         best_basis_len,
                         best_basis_current[0],
                         best_basis_current[1],
                         best_basis_current[2],
                         best_basis_current[3],
                         best_basis_current[4],
                         best_basis_current[5],
                         best_basis_current[6],
                         best_basis_current[7],
                         best_basis_current[8]);
            }
            return 1;
        }
    }

    if (basis_only) {
        addon_body_parent_capture_snapshot(slot, base);
        slot->last_update_tick = now;
        slot->last_step_offset = -1;
        slot->last_step_len = 0.0f;
        slot->last_step[0] = slot->last_step[1] =
            slot->last_step[2] = 0.0f;
        return 0;
    }

    for (i = 0; i < (int)(sizeof(addon_body_parent_transform_offsets) /
                           sizeof(addon_body_parent_transform_offsets[0])); i++) {
        int off = addon_body_parent_transform_offsets[i];
        int idx = off / (int)sizeof(float);
        float *v;
        float previous[3];
        float current[3];
        float step[3];
        float len;
        int basis_like = 0;
        if (idx < 0 || idx > ADDON_BODY_PARENT_SCAN_FLOATS - 3) continue;
        if (!ptr_readable(b + off, sizeof(float) * 3)) continue;
        v = (float*)(b + off);
        if (!physx_vec3_sane_limit(v, 720.0f)) continue;
        if (!sane_probe_float(slot->previous[idx]) ||
            !sane_probe_float(slot->previous[idx + 1]) ||
            !sane_probe_float(slot->previous[idx + 2])) {
            continue;
        }
        current[0] = v[0];
        current[1] = v[1];
        current[2] = v[2];
        previous[0] = slot->previous[idx];
        previous[1] = slot->previous[idx + 1];
        previous[2] = slot->previous[idx + 2];
        if (!physx_vec3_sane_limit(previous, 720.0f)) continue;
        addon_parent_vector_step_to_degrees(current, previous, step,
                                            &basis_like);
        len = addon_vec3_len_exact(step);
        if (len > best_len && len < 95.0f) {
            best_len = len;
            best_offset = off;
            best_basis_like = basis_like;
            best_step[0] = step[0];
            best_step[1] = step[1];
            best_step[2] = step[2];
            best_current[0] = current[0];
            best_current[1] = current[1];
            best_current[2] = current[2];
        }
    }

    addon_body_parent_capture_snapshot(slot, base);
    slot->last_update_tick = now;
    slot->last_step_offset = best_offset;
    slot->last_step_len = best_len;
    slot->last_step[0] = best_step[0];
    slot->last_step[1] = best_step[1];
    slot->last_step[2] = best_step[2];
    if (best_offset >= 0 && best_len >= ADDON_BODY_PARENT_STEP_THRESHOLD) {
        out[0] = best_step[0];
        out[1] = best_step[1];
        out[2] = best_step[2];
        if (offset_out) *offset_out = best_offset;
        if (defaults_cfg.debug &&
            (!slot->log_tick || now - slot->log_tick >= 500u)) {
            slot->log_tick = now;
            log_line("addon-chain effective parent rotation vector-step chain=\"%s\" parent=\"%s\" source=%s runtime=\"%s\" base=%p offset=0x%03x current=(%.5f,%.5f,%.5f) step=(%.5f,%.5f,%.5f) step_len=%.5f basis_like=%d note=\"fallback parent-owned vector changed; no separate neck/root/body drive was sampled\"",
                     chain->name,
                     parent->name,
                     source ? source : "unknown",
                     runtime ? runtime : "",
                     base,
                     best_offset,
                     best_current[0], best_current[1], best_current[2],
                     best_step[0], best_step[1], best_step[2],
                     best_len,
                     best_basis_like);
        }
        return 1;
    }
    return 0;
}

static int addon_effective_parent_runtime_name_exists(char names[][256],
                                                      int count,
                                                      const char *name)
{
    int i;
    if (!name || !name[0]) return 1;
    for (i = 0; i < count; i++) {
        if (_stricmp(names[i], name) == 0) return 1;
    }
    return 0;
}

static int addon_effective_parent_add_runtime_name(char names[][256],
                                                   int count,
                                                   int max_count,
                                                   const char *name)
{
    if (count >= max_count || !name || !name[0]) return count;
    if (addon_effective_parent_runtime_name_exists(names, count, name)) {
        return count;
    }
    lstrcpynA(names[count], name, 256);
    names[count][255] = 0;
    return count + 1;
}

static int addon_build_effective_parent_runtime_name(const char *person,
                                                     const char *parent_name,
                                                     int variant,
                                                     char *out,
                                                     size_t outsz)
{
    char s_name[128];
    if (!out || outsz == 0) return 0;
    out[0] = 0;
    if (!person || !person[0] || !parent_name || !parent_name[0]) return 0;
    if (parent_name[0] == 'S' || parent_name[0] == 's') {
        lstrcpynA(s_name, parent_name, sizeof(s_name));
    } else {
        _snprintf(s_name, sizeof(s_name), "S%s", parent_name);
        s_name[sizeof(s_name) - 1] = 0;
    }
    switch (variant) {
    case 0:
        make_body_runtime_name(out, outsz, person, parent_name);
        break;
    case 1:
        make_body_runtime_name(out, outsz, person, s_name);
        break;
    case 2:
        _snprintf(out, outsz, "%s:Model01:%s", person, parent_name);
        break;
    case 3:
        _snprintf(out, outsz, "%s:Model01:%s", person, s_name);
        break;
    case 4:
        _snprintf(out, outsz, "%sBody:%s", person, parent_name);
        break;
    case 5:
        _snprintf(out, outsz, "%sBody:%s", person, s_name);
        break;
    default:
        return 0;
    }
    out[outsz - 1] = 0;
    return out[0] != 0;
}

static int addon_effective_parent_add_person_runtimes(char names[][256],
                                                      int count,
                                                      int max_count,
                                                      const char *person,
                                                      const char *parent_name)
{
    int variant;
    if (!person || !person[0] || !parent_name || !parent_name[0]) {
        return count;
    }
    for (variant = 0; variant < 6 && count < max_count; variant++) {
        char runtime[256];
        if (!addon_build_effective_parent_runtime_name(
                person, parent_name, variant,
                runtime, sizeof(runtime))) {
            continue;
        }
        count = addon_effective_parent_add_runtime_name(
            names, count, max_count, runtime);
    }
    return count;
}

static int addon_collect_effective_parent_runtime_names(
    physx_sidecar_t *sc,
    physx_chain_t *chain,
    physx_target_t *parent,
    char names[][256],
    int max_count)
{
    static const char *persons[] = {
        "Person01", "Person02", "Person03", "Person04"
    };
    int count = 0;
    int i;
    if (!parent || !parent->name[0] || max_count <= 0) return 0;
    if (sc && sc->addon_owner_person[0]) {
        count = addon_effective_parent_add_person_runtimes(
            names, count, max_count, sc->addon_owner_person, parent->name);
    }
    if (chain && chain->addon_body_root_person[0]) {
        count = addon_effective_parent_add_person_runtimes(
            names, count, max_count,
            chain->addon_body_root_person, parent->name);
    }
    for (i = 0; i < (int)(sizeof(persons) / sizeof(persons[0])); i++) {
        count = addon_effective_parent_add_person_runtimes(
            names, count, max_count, persons[i], parent->name);
    }
    return count;
}

static void addon_effective_parent_runtime_owner_key(physx_sidecar_t *sc,
                                                     physx_chain_t *chain,
                                                     char *out,
                                                     size_t outsz)
{
    if (out && outsz) out[0] = 0;
    if (!out || outsz == 0) return;
    if (sc && sc->addon_owner_person[0]) {
        lstrcpynA(out, sc->addon_owner_person, (int)outsz);
    } else if (chain && chain->addon_body_root_person[0]) {
        lstrcpynA(out, chain->addon_body_root_person, (int)outsz);
    }
    out[outsz - 1] = 0;
}

static void addon_effective_parent_runtime_cache_clear(physx_chain_t *chain)
{
    if (!chain) return;
    chain->addon_parent_runtime_raw = NULL;
    chain->addon_parent_runtime_name[0] = 0;
    chain->addon_parent_runtime_parent[0] = 0;
    chain->addon_parent_runtime_owner[0] = 0;
    chain->addon_parent_runtime_resolve_tick = 0;
    chain->addon_parent_camera_relative_trs_raw = NULL;
    chain->addon_parent_camera_relative_parent_raw = NULL;
    chain->addon_parent_camera_relative_initialized = 0;
    chain->addon_parent_camera_relative_log_tick = 0;
    chain->addon_parent_camera_relative_miss_log_tick = 0;
    chain->addon_parent_translation_camera_relative = 0;
    chain->addon_parent_rotation_camera_relative = 0;
    chain->addon_parent_rotation_camera_relative_available = 0;
    chain->addon_parent_camera_relative_translation_raw = NULL;
    chain->addon_parent_camera_relative_translation_offset = -1;
    chain->addon_parent_camera_relative_translation_mode = 0;
    chain->addon_parent_camera_relative_translation_initialized = 0;
    chain->addon_parent_camera_relative_translation_prev[0] = 0.0f;
    chain->addon_parent_camera_relative_translation_prev[1] = 0.0f;
    chain->addon_parent_camera_relative_translation_prev[2] = 0.0f;
    chain->addon_parent_camera_relative_translation_basis_initialized = 0;
    memset(chain->addon_parent_camera_relative_translation_parent_rest, 0,
           sizeof(chain->addon_parent_camera_relative_translation_parent_rest));
    memset(chain->addon_parent_camera_relative_translation_model_rest, 0,
           sizeof(chain->addon_parent_camera_relative_translation_model_rest));
    chain->addon_parent_camera_relative_translation_global_initialized = 0;
    memset(chain->addon_parent_camera_relative_translation_global_prev, 0,
           sizeof(chain->addon_parent_camera_relative_translation_global_prev));
    chain->addon_parent_camera_relative_translation_global_pending_valid = 0;
    memset(chain->addon_parent_camera_relative_translation_global_pending, 0,
           sizeof(chain->addon_parent_camera_relative_translation_global_pending));
    chain->addon_parent_camera_relative_translation_global_pending_version = 0;
    chain->addon_parent_camera_relative_translation_camera_hold_active = 0;
    chain->addon_parent_camera_relative_translation_log_tick = 0;
    chain->addon_parent_camera_relative_rotation_pending_valid = 0;
    memset(chain->addon_parent_camera_relative_rotation_pending, 0,
           sizeof(chain->addon_parent_camera_relative_rotation_pending));
    chain->addon_parent_camera_relative_rotation_pending_version = 0;
    chain->addon_gravity_trusted_drive[0] = 0.0f;
    chain->addon_gravity_trusted_drive[1] = 0.0f;
    chain->addon_gravity_trusted_drive[2] = 0.0f;
    chain->addon_gravity_trusted_valid = 0;
    memset(&chain->addon_gravity_sample,0,sizeof(chain->addon_gravity_sample));
    chain->addon_gravity_camera_hold_active = 0;
    chain->addon_gravity_camera_release_active = 0;
    chain->addon_gravity_camera_log_tick = 0;
    chain->addon_stationary_camera_hold_active = 0;
    chain->addon_stationary_parent_motion_tick = 0;
    chain->addon_room_contact_normal_valid = 0;
    chain->addon_room_contact_normal_generation = 0;
    chain->addon_room_contact_normal_tick = 0;
    chain->addon_room_contact_normal[0] = 0.0f;
    chain->addon_room_contact_normal[1] = 0.0f;
    chain->addon_room_contact_normal[2] = 0.0f;
    chain->addon_room_rest_sleeping = 0;
    chain->addon_room_rest_frames = 0;
    chain->addon_room_rest_generation = 0;
    chain->addon_room_rest_tick = 0;
    memset(chain->addon_wind_trusted_drive, 0,
           sizeof(chain->addon_wind_trusted_drive));
    chain->addon_wind_trusted_valid = 0;
    chain->addon_wind_camera_seen_version = 0;
    chain->addon_wind_camera_quarantine_tick = 0;
    chain->addon_wind_update_tick = 0;
    chain->addon_wind_generation = 0;
    chain->addon_wind_logged = 0;
}

static void *addon_effective_parent_cached_runtime_raw(
    physx_sidecar_t *sc,
    physx_chain_t *chain,
    physx_target_t *parent,
    DWORD now,
    const char **runtime_out)
{
    enum { MAX_PARENT_RUNTIMES = 32 };
    char runtimes[MAX_PARENT_RUNTIMES][256];
    char owner[16];
    int count;
    int i;
    if (runtime_out) *runtime_out = "";
    if (!chain || !parent || !parent->name[0]) return NULL;
    addon_effective_parent_runtime_owner_key(sc, chain, owner, sizeof(owner));
    if (_stricmp(chain->addon_parent_runtime_parent, parent->name) != 0 ||
        _stricmp(chain->addon_parent_runtime_owner, owner) != 0) {
        addon_effective_parent_runtime_cache_clear(chain);
        lstrcpynA(chain->addon_parent_runtime_parent, parent->name,
                  sizeof(chain->addon_parent_runtime_parent));
        lstrcpynA(chain->addon_parent_runtime_owner, owner,
                  sizeof(chain->addon_parent_runtime_owner));
    }
    if (chain->addon_parent_runtime_raw &&
        ptr_readable(chain->addon_parent_runtime_raw, sizeof(float))) {
        if (runtime_out) *runtime_out = chain->addon_parent_runtime_name;
        return chain->addon_parent_runtime_raw;
    }
    if (chain->addon_parent_runtime_resolve_tick &&
        now - chain->addon_parent_runtime_resolve_tick < 1000u) {
        return NULL;
    }
    chain->addon_parent_runtime_resolve_tick = now;
    count = addon_collect_effective_parent_runtime_names(
        sc, chain, parent, runtimes, MAX_PARENT_RUNTIMES);
    for (i = 0; i < count; i++) {
        void *raw = resolve_axis_map_raw(runtimes[i]);
        if (!raw || !ptr_readable(raw, sizeof(float))) continue;
        chain->addon_parent_runtime_raw = raw;
        lstrcpynA(chain->addon_parent_runtime_name, runtimes[i],
                  sizeof(chain->addon_parent_runtime_name));
        if (runtime_out) *runtime_out = chain->addon_parent_runtime_name;
        if (defaults_cfg.debug &&
            (!chain->addon_parent_runtime_log_tick ||
             now - chain->addon_parent_runtime_log_tick >= 2000u)) {
            chain->addon_parent_runtime_log_tick = now;
            log_line("addon-chain effective parent runtime cached chain=\"%s\" parent=\"%s\" runtime=\"%s\" raw=%p owner=\"%s\" note=\"cached live parent pointer; per-frame sidecar drive no longer resolves all PersonXX candidates\"",
                     chain->name,
                     parent->name,
                     chain->addon_parent_runtime_name,
                     chain->addon_parent_runtime_raw,
                     chain->addon_parent_runtime_owner);
        }
        return chain->addon_parent_runtime_raw;
    }
    if (defaults_cfg.debug &&
        (!chain->addon_parent_runtime_miss_log_tick ||
         now - chain->addon_parent_runtime_miss_log_tick >= 3000u)) {
        chain->addon_parent_runtime_miss_log_tick = now;
        log_line("addon-chain effective parent runtime unresolved chain=\"%s\" parent=\"%s\" owner=\"%s\" candidates=%d note=\"will retry on a throttle; falling back to local parent pointers until live body parent appears\"",
                 chain->name,
                 parent->name,
                 owner,
                 count);
    }
    return NULL;
}

static int addon_parent_camera_relative_person(
    physx_sidecar_t *sc,
    physx_chain_t *chain,
    const char *runtime,
    char *out,
    size_t outsz)
{
    if (out && outsz) out[0] = 0;
    if (!out || outsz == 0) return 0;
    if (sc && sc->addon_owner_person[0]) {
        lstrcpynA(out, sc->addon_owner_person, (int)outsz);
        out[outsz - 1] = 0;
        return 1;
    }
    if (chain && chain->addon_body_root_person[0]) {
        lstrcpynA(out, chain->addon_body_root_person, (int)outsz);
        out[outsz - 1] = 0;
        return 1;
    }
    if (runtime && addon_extract_person_prefix(runtime, out, outsz)) {
        return 1;
    }
    return 0;
}

static void *addon_parent_camera_relative_trs_raw(
    physx_chain_t *chain,
    const char *person,
    DWORD now)
{
    char name[256];
    char matched[384];
    void *raw = NULL;
    void *obj = NULL;
    float tmp[9];
    if (!chain || !person || !person[0]) return NULL;
    raw = chain->addon_parent_camera_relative_trs_raw;
    if (body_chain_read_mat3_rows(raw, tmp)) return raw;

    make_body_runtime_name(name, sizeof(name), person, "TRS_group");
    matched[0] = 0;
    obj = resolve_find_obj(name, &raw);
    if ((!obj || is_nil_engine_object(raw, obj)) && captured_script_engine) {
        obj = resolve_script_engine_obj(name, &raw);
    }
    if ((!obj || is_nil_engine_object(raw, obj))) {
        obj = resolve_runtime_exact_target(name, &raw,
                                           matched, sizeof(matched));
    }
    if (!obj || !raw || is_nil_engine_object(raw, obj) ||
        !body_chain_read_mat3_rows(raw, tmp)) {
        chain->addon_parent_camera_relative_trs_raw = NULL;
        if (!chain->addon_parent_camera_relative_miss_log_tick ||
            now - chain->addon_parent_camera_relative_miss_log_tick >=
                2000u) {
            chain->addon_parent_camera_relative_miss_log_tick = now;
            log_line("addon-chain camera-relative parent unresolved chain=\"%s\" person=\"%s\" runtime=\"%s\" note=\"could not resolve TRS_group; falling back only if no camera-safe parent basis is available\"",
                     chain->name,
                     person,
                     name);
        }
        return NULL;
    }
    chain->addon_parent_camera_relative_trs_raw = raw;
    return raw;
}

static int addon_effective_parent_runtime_camera_relative_rotation_step(
    physx_sidecar_t *sc,
    physx_chain_t *chain,
    physx_target_t *parent,
    void *parent_raw,
    const char *runtime,
    float out[3],
    DWORD now,
    int *handled_out)
{
    char person[16];
    void *trs_raw;
    float parent_matrix[9];
    float trs_matrix[9];
    float trs_inverse[9];
    float current[9];
    float step[3];
    float sampled_step[3];
    float step_sq;
    float step_threshold;
    float step_len;
    int pending_camera_changed;
    if (handled_out) *handled_out = 0;
    if (out) out[0] = out[1] = out[2] = 0.0f;
    if (!physics_environment_cfg.body_chain_camera_relative_orientation ||
        !chain || !parent || !parent_raw || !out) {
        return 0;
    }
    if (!addon_parent_camera_relative_person(sc, chain, runtime,
                                             person, sizeof(person))) {
        return 0;
    }
    trs_raw = addon_parent_camera_relative_trs_raw(chain, person, now);
    if (!trs_raw) return 0;
    if (!body_chain_read_mat3_rows(parent_raw, parent_matrix) ||
        !body_chain_read_mat3_rows(trs_raw, trs_matrix) ||
        !body_chain_mat3_inverse(trs_matrix, trs_inverse)) {
        return 0;
    }
    body_chain_mat3_multiply(parent_matrix, trs_inverse, current);
    if (!addon_normalize_basis_rows(current)) return 0;
    if (handled_out) *handled_out = 1;

    if (!chain->addon_parent_camera_relative_initialized ||
        chain->addon_parent_camera_relative_parent_raw != parent_raw) {
        chain->addon_parent_camera_relative_initialized = 1;
        chain->addon_parent_camera_relative_parent_raw = parent_raw;
        memcpy(chain->addon_parent_camera_relative_prev, current,
               sizeof(current));
        chain->addon_parent_camera_relative_rotation_pending_valid = 0;
        if (!chain->addon_parent_camera_relative_log_tick ||
            now - chain->addon_parent_camera_relative_log_tick >= 1000u) {
            chain->addon_parent_camera_relative_log_tick = now;
            log_line("addon-chain camera-relative parent baseline chain=\"%s\" parent=\"%s\" person=\"%s\" runtime=\"%s\" parent_raw=%p trs_raw=%p note=\"using parent * inverse(TRS_group), matching penis/testicle camera-safe body basis\"",
                     chain->name,
                     parent->name,
                     person,
                     runtime ? runtime : "",
                     parent_raw,
                     trs_raw);
        }
        return 0;
    }

    if (!addon_basis_triad_relative_step_to_degrees(
            current, chain->addon_parent_camera_relative_prev, step)) {
        memcpy(chain->addon_parent_camera_relative_prev, current,
               sizeof(current));
        return 0;
    }
    memcpy(chain->addon_parent_camera_relative_prev, current,
           sizeof(current));
    sampled_step[0] = step[0];
    sampled_step[1] = step[1];
    sampled_step[2] = step[2];
    if (!chain->addon_parent_camera_relative_rotation_pending_valid) {
        memcpy(chain->addon_parent_camera_relative_rotation_pending,
               sampled_step, sizeof(sampled_step));
        chain->addon_parent_camera_relative_rotation_pending_version =
            captured_camera_version;
        chain->addon_parent_camera_relative_rotation_pending_valid = 1;
        return 0;
    }
    pending_camera_changed =
        chain->addon_parent_camera_relative_rotation_pending_version !=
        captured_camera_version;
    memcpy(step, chain->addon_parent_camera_relative_rotation_pending,
           sizeof(step));
    memcpy(chain->addon_parent_camera_relative_rotation_pending,
           sampled_step, sizeof(sampled_step));
    chain->addon_parent_camera_relative_rotation_pending_version =
        captured_camera_version;
    step_sq = step[0] * step[0] +
              step[1] * step[1] +
              step[2] * step[2];
    step_threshold = ADDON_BODY_PARENT_STEP_THRESHOLD;
    if (pending_camera_changed ||
        (captured_camera_inverse_valid &&
         captured_camera_change_tick &&
         now - captured_camera_change_tick < 300u)) {
        /* Parent and TRS_group are sampled at adjacent traversal points.
           Validate one frame later, after TK17 publishes the camera version,
           so the leading look-around residual is filtered as well. */
        step_threshold = 0.05f;
    }
    if (step_sq < step_threshold * step_threshold ||
        step_sq > 95.0f * 95.0f) {
        return 0;
    }
    step_len = addon_vec3_len_exact(step);
    out[0] = step[0];
    out[1] = step[1];
    out[2] = step[2];
    addon_chain_note_body_root_person(chain, person, now,
                                      "camera-relative-parent");
    if (defaults_cfg.debug &&
        (!chain->addon_parent_camera_relative_log_tick ||
         now - chain->addon_parent_camera_relative_log_tick >= 500u)) {
        chain->addon_parent_camera_relative_log_tick = now;
        log_line("addon-chain camera-relative parent rotation selected chain=\"%s\" parent=\"%s\" person=\"%s\" runtime=\"%s\" step=(%.5f,%.5f,%.5f) step_len=%.5f note=\"camera-independent parent drive feeds the custom add-on chain\"",
                 chain->name,
                 parent->name,
                 person,
                 runtime ? runtime : "",
                 out[0], out[1], out[2],
                 step_len);
    }
    return 1;
}

static void addon_transform_row_vector3(const float v[3],
                                        const float m[9],
                                        float out[3])
{
    if (!v || !m || !out) return;
    out[0] = v[0] * m[0] + v[1] * m[3] + v[2] * m[6];
    out[1] = v[0] * m[1] + v[1] * m[4] + v[2] * m[7];
    out[2] = v[0] * m[2] + v[1] * m[5] + v[2] * m[8];
}

static int addon_person_index_from_name(const char *person)
{
    int value;
    if (!person || _strnicmp(person, "Person", 6) != 0) return -1;
    if (person[6] < '0' || person[6] > '9' ||
        person[7] < '0' || person[7] > '9') {
        return -1;
    }
    value = (person[6] - '0') * 10 + (person[7] - '0');
    if (value < 1 || value > 4) return -1;
    return value - 1;
}

static int addon_gravity_basis_row_from_offset(int offset, int fallback_row)
{
    if (offset == 0x078) return 0;
    if (offset == 0x088) return 1;
    if (offset == 0x098) return 2;
    return fallback_row;
}

static void addon_chain_project_direction_basis(const float direction[3],
                                                const float basis[9],
                                                float out[3])
{
    int h_row;
    int v_row;
    int hs_row;
    if (!direction || !basis || !out) return;
    h_row = addon_gravity_basis_row_from_offset(
        physics_environment_cfg.gravity_horizontal_basis_offset, 1);
    v_row = addon_gravity_basis_row_from_offset(
        physics_environment_cfg.gravity_vertical_basis_offset, 2);
    hs_row = addon_gravity_basis_row_from_offset(
        physics_environment_cfg.gravity_horizontal_secondary_basis_offset, 0);
    out[0] = vec3_dot(direction, &basis[h_row * 3]) *
             physics_environment_cfg.gravity_horizontal_basis_sign;
    out[1] = vec3_dot(direction, &basis[v_row * 3]) *
             physics_environment_cfg.gravity_vertical_basis_sign;
    out[2] = vec3_dot(direction, &basis[hs_row * 3]) *
             physics_environment_cfg.gravity_horizontal_secondary_basis_sign;
}

/* Consume only a deferred camera-validated parent gravity sample. Parent
   motion impulses do not authorize a gravity update. */
static int addon_chain_camera_safe_gravity_drive(physx_chain_t *chain,
    DWORD now, const float candidate[3], int candidate_valid,
    const void *source, float out[3])
{
    float trusted[3] = {0};
    int available = gravity_sample_live(&chain->addon_gravity_sample,source,
        candidate,candidate_valid,now,trusted);
    chain->addon_gravity_camera_hold_active = !chain->addon_gravity_sample.accepted;
    chain->addon_gravity_camera_release_active = 0;
    chain->addon_gravity_trusted_valid = available;
    if (available) memcpy(chain->addon_gravity_trusted_drive,trusted,sizeof(trusted));
    memcpy(out,trusted,sizeof(trusted));
    /* A warming/held parent source is handled, never replaced with gravity
       from a different body frame or the local fallback vector. */
    return 1;
}

static int addon_chain_parent_gravity_drive(physx_sidecar_t *sc,
                                            physx_chain_t *chain,
                                            DWORD now,
                                            int camera_safe_parent_rotation,
                                            float out[3])
{
    physx_target_t *parent;
    const char *runtime = "";
    char person[16];
    void *parent_raw;
    float gravity_len;
    float gravity_world[3] = { 0.0f, -1.0f, 0.0f };
    float gravity_view[3];
    float parent_matrix[9];
    (void)camera_safe_parent_rotation;
    if (!out) return 0;
    out[0] = 0.0f;
    out[1] = 0.0f;
    out[2] = 0.0f;
    if (!chain || !chain->gravity_enabled ||
        !physics_environment_cfg.gravity_apply_to_body_chain ||
        !physics_environment_cfg.gravity_dynamic_body_basis ||
        !physics_environment_cfg.body_chain_camera_relative_orientation ||
        chain->target_count <= 0) {
        return 0;
    }
    parent = &chain->targets[0];
    if (!parent->name[0] || parent->addon_simulated_target) {
        return 0;
    }
    /* Custom add-on parents already carry their live, add-on-local object.
       Do not repeatedly search every PersonXX runtime for a body copy that
       cannot exist.  Known body-parent names keep the runtime-first path,
       and an unresolved custom parent still gets the legacy lookup so this
       optimization cannot hide a late body binding. */
    parent_raw = parent->raw_object;
    if (!parent_raw) parent_raw = parent->object;
    if (!parent_raw) parent_raw = parent->s_raw_object;
    if (!parent_raw) parent_raw = parent->s_object;
    if (!parent_raw || addon_parent_name_can_use_body_drive(parent->name)) {
        void *runtime_raw = addon_effective_parent_cached_runtime_raw(
            sc, chain, parent, now, &runtime);
        if (runtime_raw) parent_raw = runtime_raw;
    }
    if (!addon_parent_camera_relative_person(sc, chain, runtime,
                                             person, sizeof(person))) {
        return 0;
    }
    gravity_len = physx_vec3_len(physics_environment_cfg.world_gravity);
    if (gravity_len > 0.000001f) {
        gravity_world[0] = physics_environment_cfg.world_gravity[0] /
                           gravity_len;
        gravity_world[1] = physics_environment_cfg.world_gravity[1] /
                           gravity_len;
        gravity_world[2] = physics_environment_cfg.world_gravity[2] /
                           gravity_len;
    }
    if (!body_chain_read_mat3_rows(parent_raw, parent_matrix) ||
        !addon_normalize_basis_rows(parent_matrix) ||
        !camera_world_to_view_direction(gravity_world, gravity_view)) {
        return addon_chain_camera_safe_gravity_drive(chain,now,NULL,0,parent_raw,out);
    }
    /* Gravity and wind must be expressed through the same live model-view
       parent basis. Removing TRS_group here made gravity body-relative while
       wind remained room-relative, so their sum changed with pose placement. */
    addon_chain_project_direction_basis(gravity_view, parent_matrix, out);
    if (!addon_chain_camera_safe_gravity_drive(
            chain, now, out, physx_vec3_sane_limit(out, 4.0f), parent_raw, out)) {
        out[0] = 0.0f;
        out[1] = 0.0f;
        out[2] = 0.0f;
        return 0;
    }
    addon_chain_note_body_root_person(chain, person, now,
                                      "parent-gravity-basis");
    return 1;
}

static int addon_chain_body_gravity_drive(physx_sidecar_t *sc,
                                          physx_chain_t *chain,
                                          DWORD now,
                                          int camera_safe_parent_rotation,
                                          float out[3])
{
    body_chain_person_state_t *state = NULL;
    const float *drive;
    char person[16];
    int person_index;
    if (!out) return 0;
    out[0] = 0.0f;
    out[1] = 0.0f;
    out[2] = 0.0f;
    if (!chain || !chain->gravity_enabled ||
        !physics_environment_cfg.gravity_apply_to_body_chain) {
        return 0;
    }
    if (addon_chain_parent_gravity_drive(
            sc, chain, now, camera_safe_parent_rotation, out)) {
        return 1;
    }
    if (!addon_parent_camera_relative_person(sc, chain, NULL,
                                             person, sizeof(person))) {
        return 0;
    }
    person_index = addon_person_index_from_name(person);
    if (person_index < 0 || person_index >= 4) return 0;

    state = body_chain_active_person_state(person_index, 0);
    if (!state || !state->gravity_probe_promoted) {
        state = body_chain_active_person_state(person_index, 1);
        if (state && !state->gravity_probe_promoted) state = NULL;
    }
    if (!state) return 0;
    drive = state->gravity_drive_filtered_valid ?
            state->gravity_drive_filtered : state->gravity_drive;
    if (!physx_vec3_sane_limit(drive, 4.0f)) return 0;
    out[0] = drive[0];
    out[1] = drive[1];
    out[2] = drive[2];
    return 1;
}

static int addon_chain_camera_safe_wind_drive(physx_chain_t *chain,
                                              DWORD now,
                                              const float candidate[3],
                                              float out[3])
{
    unsigned int generation;
    if (!chain || !candidate || !out ||
        !physx_vec3_sane_limit(candidate, 4.0f)) {
        return 0;
    }
    generation = room_wind_generation();
    if (chain->addon_wind_generation != generation) {
        chain->addon_wind_generation = generation;
        chain->addon_wind_trusted_valid = 0;
        chain->addon_wind_camera_seen_version = captured_camera_version;
        chain->addon_wind_camera_quarantine_tick = 0;
        chain->addon_wind_update_tick = 0;
        chain->addon_wind_logged = 0;
    }
    /* World-to-view projection already removes camera motion. Feed the live
       candidate directly to the chain and let its physical spring provide
       the visible response. The old 1.6-second camera quarantine plus an
       extra 120 ms low-pass made add-ons lag behind body PhysX after poses. */
    chain->addon_wind_camera_seen_version = captured_camera_version;
    chain->addon_wind_camera_quarantine_tick = 0;
    chain->addon_wind_update_tick = now;
    chain->addon_wind_trusted_valid = 1;
    memcpy(chain->addon_wind_trusted_drive, candidate,
           sizeof(chain->addon_wind_trusted_drive));
    memcpy(out, candidate, sizeof(chain->addon_wind_trusted_drive));
    return 1;
}

static int addon_chain_parent_wind_drive(physx_sidecar_t *sc,
                                         physx_chain_t *chain,
                                         DWORD now,
                                         float out[3])
{
    physx_target_t *parent;
    const char *runtime = "";
    char person[16];
    void *parent_raw;
    float world_direction[3];
    float view_direction[3];
    float parent_matrix[9];
    if (!out) return 0;
    out[0] = 0.0f;
    out[1] = 0.0f;
    out[2] = 0.0f;
    if (!sc || !chain || !chain->wind_enabled ||
        chain->wind_scale <= 0.000001f ||
        chain->target_count <= 0 ||
        !room_wind_direction(world_direction) ||
        !camera_world_to_view_direction(world_direction, view_direction)) {
        return 0;
    }
    parent = &chain->targets[0];
    if (!parent->name[0] ||
        (parent->addon_simulated_target && !sc->room_scene_sidecar)) {
        return 0;
    }
    parent_raw = parent->raw_object;
    if (!parent_raw) parent_raw = parent->object;
    if (!parent_raw) parent_raw = parent->s_raw_object;
    if (!parent_raw) parent_raw = parent->s_object;
    if (!parent_raw || addon_parent_name_can_use_body_drive(parent->name)) {
        void *runtime_raw = addon_effective_parent_cached_runtime_raw(
            sc, chain, parent, now, &runtime);
        if (runtime_raw) parent_raw = runtime_raw;
    }
    if (!addon_parent_camera_relative_person(sc, chain, runtime,
                                             person, sizeof(person)) &&
        !sc->room_scene_sidecar) {
        return 0;
    }
    if (!parent_raw ||
        !body_chain_read_mat3_rows(parent_raw, parent_matrix) ||
        !addon_normalize_basis_rows(parent_matrix)) {
        return 0;
    }
    /* Use one world-to-parent projection for wearable and room chains. The
       shared bend solver already converts this drive into the appropriate
       output rotation. Reversing room input here applied a second polarity
       correction and made static room bones lean against the world wind. */
    addon_chain_project_direction_basis(view_direction, parent_matrix, out);
    if (!physx_vec3_sane_limit(out, 4.0f) ||
        !addon_chain_camera_safe_wind_drive(chain, now, out, out)) {
        out[0] = 0.0f;
        out[1] = 0.0f;
        out[2] = 0.0f;
        return 0;
    }
    if (defaults_cfg.debug && !chain->addon_wind_logged) {
        chain->addon_wind_logged = 1;
        log_line("addon room wind active owner=\"%s\" chain=\"%s\" wind_scale=%.3f room_direction=(%.4f,%.4f,%.4f) view_direction=(%.4f,%.4f,%.4f) local_direction=(%.4f,%.4f,%.4f) room_sidecar=\"%s\" addon_sidecar=\"%s\"",
                 chain->addon_owner_person,
                 chain->name,
                 chain->wind_scale,
                 world_direction[0], world_direction[1], world_direction[2],
                 view_direction[0], view_direction[1], view_direction[2],
                 out[0], out[1], out[2],
                 room_wind_cfg.path,
                 sc->path);
    }
    return 1;
}

static int addon_chain_gravity_axis_for_target(physx_target_t *target,
                                               int requested_axis,
                                               int channel)
{
    float unit[3];
    float len;
    int fallback_axis;
    if (requested_axis < 0 || requested_axis > 2) {
        requested_axis = channel == 1 ? 2 : 0;
    }
    if (!target || target->sim_length <= 0.0001f) return requested_axis;
    len = target->sim_length;
    unit[0] = target->sim_rest[0] / len;
    unit[1] = target->sim_rest[1] / len;
    unit[2] = target->sim_rest[2] / len;
    if (physx_absf(unit[requested_axis]) < 0.82f) return requested_axis;

    fallback_axis = channel == 1 ? 2 : 0;
    if (physx_absf(unit[fallback_axis]) < 0.65f) return fallback_axis;
    fallback_axis = fallback_axis == 0 ? 2 : 0;
    if (physx_absf(unit[fallback_axis]) < 0.65f) return fallback_axis;
    return requested_axis;
}

static int addon_chain_target_gravity_tail_axis(physx_chain_t *chain,
                                                physx_target_t *target,
                                                int channel)
{
    int axis;
    int explicit_axis = 0;
    if (!chain || !target || !target->gravity_settings_initialized) {
        axis = channel == 1 ?
               (chain ? chain->rotation_drive_vertical_tail_axis : 2) :
               (chain ? chain->rotation_drive_horizontal_tail_axis : 0);
        return addon_chain_gravity_axis_for_target(target, axis, channel);
    }
    if (channel == 1) {
        axis = target->gravity_vertical_tail_axis;
        explicit_axis = target->gravity_vertical_tail_axis_explicit;
    } else {
        axis = target->gravity_horizontal_tail_axis;
        explicit_axis = target->gravity_horizontal_tail_axis_explicit;
    }
    if (explicit_axis) {
        if (axis < 0 || axis > 2) return channel == 1 ? 2 : 0;
        return axis;
    }
    return addon_chain_gravity_axis_for_target(target, axis, channel);
}

static float addon_chain_gravity_drive_component(const float drive[3],
                                                 int source_axis)
{
    float value;
    float h_primary_scale;
    float v_primary_scale;
    if (!drive) return 0.0f;
    if (source_axis < 0 || source_axis > 2) source_axis = 0;
    value = drive[source_axis];
    h_primary_scale = physics_environment_cfg.gravity_horizontal_body_chain_scale;
    v_primary_scale = physics_environment_cfg.gravity_vertical_body_chain_scale;
    if (source_axis == 0 &&
        physx_absf(h_primary_scale) > 0.000001f) {
        value += drive[2] *
                 (physics_environment_cfg.gravity_horizontal_secondary_body_chain_scale /
                  h_primary_scale);
    } else if (source_axis == 1) {
        if (physx_absf(v_primary_scale) > 0.000001f) {
            value += drive[2] *
                     (physics_environment_cfg.gravity_vertical_secondary_body_chain_scale /
                      v_primary_scale);
        }
        if (physx_absf(physics_environment_cfg.gravity_horizontal_secondary_body_chain_scale) <= 0.000001f &&
            physx_absf(physics_environment_cfg.gravity_vertical_secondary_body_chain_scale) <= 0.000001f &&
            drive[2] > 0.0f) {
            value += drive[2];
        }
    }
    return value;
}

static float addon_axis_component(const float v[3], int source_axis)
{
    if (!v) return 0.0f;
    if (source_axis < 0 || source_axis > 2) return 0.0f;
    return v[source_axis];
}

static void addon_translation_drive_add_tangent(
    float out[3],
    physx_chain_t *chain,
    physx_target_t *target,
    const float source_step[3],
    int source_axis,
    int tail_axis,
    int fallback_axis,
    float scale)
{
    float value;
    float segment[3];
    float rest_segment[3];
    float base[3] = { 0.0f, 0.0f, 0.0f };
    float rotation_axis[3];
    float transported[3];
    float segment_len;
    float rest_len;
    float base_len;
    float transported_len;
    float dot;
    float cosine;
    float sine_sq;
    int axis;
    if (!out || !chain || !target || !source_step) return;
    if (source_axis < 0 || source_axis > 2) return;
    if (tail_axis < 0 || tail_axis > 2) return;
    if (!sane_probe_float(scale)) return;
    value = source_step[source_axis] * scale;
    if (physx_absf(value) <= 0.0000001f) return;

    memcpy(segment, target->sim_offset, sizeof(segment));
    segment_len = addon_vec3_len_exact(segment);
    if (segment_len <= 0.0001f) {
        memcpy(segment, target->sim_rest, sizeof(segment));
        segment_len = addon_vec3_len_exact(segment);
    }
    if (segment_len <= 0.0001f) {
        out[tail_axis] += value;
        return;
    }
    for (axis = 0; axis < 3; axis++) segment[axis] /= segment_len;

    memcpy(rest_segment, target->sim_rest, sizeof(rest_segment));
    rest_len = addon_vec3_len_exact(rest_segment);
    if (rest_len <= 0.0001f) {
        memcpy(rest_segment, segment, sizeof(rest_segment));
        rest_len = 1.0f;
    }
    for (axis = 0; axis < 3; axis++) rest_segment[axis] /= rest_len;

    base[tail_axis] = 1.0f;
    dot = base[0] * rest_segment[0] +
          base[1] * rest_segment[1] +
          base[2] * rest_segment[2];
    for (axis = 0; axis < 3; axis++) {
        base[axis] -= rest_segment[axis] * dot;
    }
    base_len = addon_vec3_len_exact(base);

    if (base_len <= 0.05f &&
        fallback_axis >= 0 && fallback_axis <= 2 &&
        fallback_axis != tail_axis) {
        base[0] = 0.0f;
        base[1] = 0.0f;
        base[2] = 0.0f;
        base[fallback_axis] = 1.0f;
        dot = base[0] * rest_segment[0] +
              base[1] * rest_segment[1] +
              base[2] * rest_segment[2];
        for (axis = 0; axis < 3; axis++) {
            base[axis] -= rest_segment[axis] * dot;
        }
        base_len = addon_vec3_len_exact(base);
    }
    if (base_len <= 0.05f) {
        int least_axis = 0;
        if (physx_absf(rest_segment[1]) <
            physx_absf(rest_segment[least_axis])) {
            least_axis = 1;
        }
        if (physx_absf(rest_segment[2]) <
            physx_absf(rest_segment[least_axis])) {
            least_axis = 2;
        }
        base[0] = 0.0f;
        base[1] = 0.0f;
        base[2] = 0.0f;
        base[least_axis] = 1.0f;
        dot = base[least_axis] * rest_segment[least_axis];
        for (axis = 0; axis < 3; axis++) {
            base[axis] -= rest_segment[axis] * dot;
        }
        base_len = addon_vec3_len_exact(base);
    }
    if (base_len <= 0.0001f) return;
    for (axis = 0; axis < 3; axis++) base[axis] /= base_len;

    /* Move the configured rest-pose bend direction with the segment's
       gravity rotation. This preserves the user's horizontal/vertical intent
       when the chain points in a different direction instead of selecting an
       arbitrary perpendicular axis at that pose. */
    rotation_axis[0] = rest_segment[1] * segment[2] -
                       rest_segment[2] * segment[1];
    rotation_axis[1] = rest_segment[2] * segment[0] -
                       rest_segment[0] * segment[2];
    rotation_axis[2] = rest_segment[0] * segment[1] -
                       rest_segment[1] * segment[0];
    cosine = physx_clampf(
        rest_segment[0] * segment[0] +
        rest_segment[1] * segment[1] +
        rest_segment[2] * segment[2], -1.0f, 1.0f);
    sine_sq = rotation_axis[0] * rotation_axis[0] +
              rotation_axis[1] * rotation_axis[1] +
              rotation_axis[2] * rotation_axis[2];
    if (sine_sq > 0.000001f) {
        float axis_dot_base =
            rotation_axis[0] * base[0] +
            rotation_axis[1] * base[1] +
            rotation_axis[2] * base[2];
        float axis_scale = axis_dot_base * (1.0f - cosine) / sine_sq;
        transported[0] = base[0] * cosine +
            (rotation_axis[1] * base[2] - rotation_axis[2] * base[1]) +
            rotation_axis[0] * axis_scale;
        transported[1] = base[1] * cosine +
            (rotation_axis[2] * base[0] - rotation_axis[0] * base[2]) +
            rotation_axis[1] * axis_scale;
        transported[2] = base[2] * cosine +
            (rotation_axis[0] * base[1] - rotation_axis[1] * base[0]) +
            rotation_axis[2] * axis_scale;
    } else {
        memcpy(transported, base, sizeof(transported));
    }
    dot = transported[0] * segment[0] +
          transported[1] * segment[1] +
          transported[2] * segment[2];
    for (axis = 0; axis < 3; axis++) {
        transported[axis] -= segment[axis] * dot;
    }
    transported_len = addon_vec3_len_exact(transported);
    if (transported_len <= 0.0001f) return;
    for (axis = 0; axis < 3; axis++) {
        out[axis] += transported[axis] * (value / transported_len);
    }
}

static int addon_chain_world_gravity_bend_vector(
    physx_chain_t *chain,
    physx_target_t *target,
    const float drive[3],
    float out[3])
{
    int h_axis;
    int v_axis;
    float h_drive;
    float v_drive;
    float v_axis_sign;
    float rest_len_sq;
    float rest_dot;
    float len;
    if (!chain || !target || !drive || !out) return 0;
    out[0] = 0.0f;
    out[1] = 0.0f;
    out[2] = 0.0f;

    h_axis = addon_chain_target_gravity_tail_axis(chain, target, 0);
    v_axis = addon_chain_target_gravity_tail_axis(chain, target, 1);
    if (!target->gravity_settings_initialized) {
        target->gravity_horizontal_source_axis = 0;
        target->gravity_horizontal_source_sign = 1.0f;
        target->gravity_horizontal_scale =
            chain->rotation_drive_horizontal_scale < 0.0f ? -1.0f : 1.0f;
        target->gravity_vertical_source_axis = 1;
        target->gravity_vertical_source_sign = 1.0f;
        target->gravity_vertical_scale =
            chain->rotation_drive_vertical_scale < 0.0f ? -1.0f : 1.0f;
    }
    h_drive = addon_chain_gravity_drive_component(
        drive, target->gravity_horizontal_source_axis) *
        target->gravity_horizontal_source_sign;
    v_drive = addon_chain_gravity_drive_component(
        drive, target->gravity_vertical_source_axis) *
        target->gravity_vertical_source_sign;
    v_axis_sign = (v_axis == 1) ? -1.0f : 1.0f;
    out[h_axis] += h_drive * target->gravity_horizontal_scale;
    out[v_axis] += v_drive * target->gravity_vertical_scale * v_axis_sign;

    if (target->sim_length > 0.0001f) {
        rest_len_sq = target->sim_length * target->sim_length;
        rest_dot = (out[0] * target->sim_rest[0] +
                    out[1] * target->sim_rest[1] +
                    out[2] * target->sim_rest[2]) / rest_len_sq;
        out[0] -= target->sim_rest[0] * rest_dot;
        out[1] -= target->sim_rest[1] * rest_dot;
        out[2] -= target->sim_rest[2] * rest_dot;
    }

    len = physx_vec3_len(out);
    if (len < 0.0001f) return 0;
    if (len > 1.5f) {
        float scale = 1.5f / len;
        out[0] *= scale;
        out[1] *= scale;
        out[2] *= scale;
    }
    return 1;
}

static int addon_chain_apply_inverted_gravity_bend(
    physx_chain_t *chain,
    physx_target_t *target,
    const float gravity_drive[3],
    float gravity_scale,
    float bend[3])
{
    float inverted_amount;
    float rest_len_sq;
    float rest_dot;
    float len;
    int axis;
    if (!chain || !target || !gravity_drive || !bend ||
        !target->gravity_inverted_configured ||
        target->gravity_inverted_strength <= 0.000001f ||
        target->gravity_inverted_tail_axis < 0 ||
        target->gravity_inverted_tail_axis > 2 ||
        physx_absf(target->gravity_inverted_sign) <= 0.000001f ||
        physx_absf(gravity_scale) <= 0.000001f) {
        return physx_vec3_len(bend) >= 0.0001f;
    }

    /* The secondary gravity channel is positive only while the chain's
       parent is inverted. Keep this contribution gravity-only: the room
       wind channels must never activate the inverted response. */
    inverted_amount = physx_clampf(gravity_drive[2], 0.0f, 1.0f);
    inverted_amount = inverted_amount * inverted_amount *
                      (3.0f - 2.0f * inverted_amount);
    axis = target->gravity_inverted_tail_axis;
    bend[axis] += inverted_amount *
                  target->gravity_inverted_strength *
                  target->gravity_inverted_sign * gravity_scale;

    /* Apply the same link-axis constraint and safety cap used by the normal
       add-on gravity mapping. */
    if (target->sim_length > 0.0001f) {
        rest_len_sq = target->sim_length * target->sim_length;
        rest_dot = (bend[0] * target->sim_rest[0] +
                    bend[1] * target->sim_rest[1] +
                    bend[2] * target->sim_rest[2]) / rest_len_sq;
        bend[0] -= target->sim_rest[0] * rest_dot;
        bend[1] -= target->sim_rest[1] * rest_dot;
        bend[2] -= target->sim_rest[2] * rest_dot;
    }
    len = physx_vec3_len(bend);
    if (len < 0.0001f) return 0;
    if (len > 1.5f) {
        float clamp_scale = 1.5f / len;
        bend[0] *= clamp_scale;
        bend[1] *= clamp_scale;
        bend[2] *= clamp_scale;
    }
    return 1;
}

static void addon_chain_bend_to_output_angles(physx_chain_t *chain,
                                              physx_target_t *target,
                                              const float bend[3],
                                              float rest_len,
                                              float *pitch_out,
                                              float *roll_out)
{
    float pitch = 0.0f;
    float roll = 0.0f;
    if (!bend || rest_len <= 0.0001f) {
        if (pitch_out) *pitch_out = 0.0f;
        if (roll_out) *roll_out = 0.0f;
        return;
    }
    if (chain && target && chain->addon_chain) {
        int h_axis = addon_chain_target_gravity_tail_axis(chain, target, 0);
        int v_axis = addon_chain_target_gravity_tail_axis(chain, target, 1);
        float pitch_delta = bend[v_axis];
        float roll_delta = bend[h_axis];
        if (v_axis == 1) pitch_delta = -pitch_delta;
        pitch = (float)(atan2((double)pitch_delta,
                              (double)rest_len) * 57.29577951308232);
        roll = (float)(atan2((double)-roll_delta,
                             (double)rest_len) * 57.29577951308232);
    } else {
        pitch = (float)(atan2((double)bend[2],
                              (double)rest_len) * 57.29577951308232);
        roll = (float)(atan2((double)bend[0],
                             (double)rest_len) * -57.29577951308232);
    }
    if (pitch_out) *pitch_out = pitch;
    if (roll_out) *roll_out = roll;
}

static int addon_chain_full_angle_output_delta_at(physx_chain_t *chain,
                                               physx_target_t *target,
                                               const float offset[3],
                                               float out_delta[3])
{
    float rest[3];
    float current[3];
    float tangent[3];
    float dot;
    float rest_len;
    float current_len;
    float angle;
    float pitch_component;
    float roll_component;
    float component_len;
    int h_axis;
    int v_axis;
    int k;
    if (!chain || !target || !out_delta) return 0;
    out_delta[0] = 0.0f;
    out_delta[1] = 0.0f;
    out_delta[2] = 0.0f;

    rest_len = addon_vec3_len_exact(target->sim_rest);
    current_len = addon_vec3_len_exact(offset);
    if (rest_len <= 0.0001f || current_len <= 0.0001f) return 0;
    for (k = 0; k < 3; k++) {
        rest[k] = target->sim_rest[k] / rest_len;
        current[k] = offset[k] / current_len;
    }
    dot = physx_clampf(rest[0] * current[0] +
                       rest[1] * current[1] +
                       rest[2] * current[2], -1.0f, 1.0f);
    angle = (float)(acos((double)dot) * 57.29577951308232);
    if (angle <= 0.0001f) return 1;

    /* Keep the legacy sidecar channel meaning: the configured vertical tail
       axis produces X bend and the configured horizontal tail axis produces
       Z bend.  Only the magnitude changes from the legacy atan2 projection
       to the complete rest-to-current direction angle. */
    h_axis = addon_chain_target_gravity_tail_axis(chain, target, 0);
    v_axis = addon_chain_target_gravity_tail_axis(chain, target, 1);
    for (k = 0; k < 3; k++) {
        tangent[k] = current[k] - rest[k] * dot;
    }
    pitch_component = tangent[v_axis];
    if (v_axis == 1) pitch_component = -pitch_component;
    roll_component = -tangent[h_axis];
    component_len = (float)sqrt((double)(pitch_component * pitch_component +
                                         roll_component * roll_component));
    if (component_len <= 0.0001f && dot < 0.0f) {
        /* At the exact antipode the direction alone cannot identify which
           way the link travelled.  Its tangential velocity normally retains
           that information; fall back to the vertical channel at rest. */
        pitch_component = target->sim_velocity[v_axis];
        if (v_axis == 1) pitch_component = -pitch_component;
        roll_component = -target->sim_velocity[h_axis];
        component_len =
            (float)sqrt((double)(pitch_component * pitch_component +
                                 roll_component * roll_component));
        if (component_len <= 0.0001f) {
            pitch_component = 1.0f;
            roll_component = 0.0f;
            component_len = 1.0f;
        }
    }
    if (component_len <= 0.0001f) return 1;
    out_delta[0] = angle * pitch_component / component_len;
    out_delta[1] = 0.0f;
    out_delta[2] = angle * roll_component / component_len;
    return 1;
}

static int addon_chain_full_angle_output_delta(physx_chain_t *chain,
    physx_target_t *target, float out_delta[3])
{
    if (!target) return 0;
    return addon_chain_full_angle_output_delta_at(chain,target,target->sim_offset,out_delta);
}

static int addon_chain_root_visual_roll_inverted(physx_chain_t *chain,
                                                 physx_target_t *target)
{
    float rx;
    if (!chain || !target || !chain->addon_chain) return 0;
    if (chain->target_count <= 2) return 0;
    if (target != &chain->targets[1]) return 0;
    if (!target->addon_simulated_target) return 0;
    rx = target->sim_rotation_rest[0];
    while (rx > 180.0f) rx -= 360.0f;
    while (rx < -180.0f) rx += 360.0f;
    return physx_absf(rx) > 90.0f;
}

static int addon_chain_root_physx_target(physx_chain_t *chain,
                                         physx_target_t *target)
{
    int root_index = 0;
    if (!chain || !target || !chain->addon_chain) return 0;
    if (!target->addon_simulated_target) return 0;
    if (chain->target_count <= 0) return 0;
    if (chain->parent_name[0] &&
        chain->target_count > 1 &&
        _stricmp(chain->targets[0].name, chain->parent_name) == 0 &&
        _stricmp(chain->targets[1].name, chain->name) == 0) {
        root_index = 1;
    }
    return target == &chain->targets[root_index];
}

static int addon_gravity_diag_enabled(physx_chain_t *chain)
{
    const int addon_gravity_diag_auto_test = 0;
    if (!addon_gravity_diag_auto_test) return 0;
    return chain &&
           chain->addon_chain &&
           chain->target_count >= 4 &&
           contains_i(chain->name, "physx_tail01");
}

static void addon_gravity_diag_reset_target(physx_target_t *target)
{
    if (!target) return;
    target->addon_gravity_diag_initialized = 0;
    target->addon_gravity_diag_samples = 0;
    target->addon_gravity_diag_start_offset[0] = 0.0f;
    target->addon_gravity_diag_start_offset[1] = 0.0f;
    target->addon_gravity_diag_start_offset[2] = 0.0f;
    target->addon_gravity_diag_max_delta = 0.0f;
    target->addon_gravity_diag_max_bend_len = 0.0f;
    target->addon_gravity_diag_last_delta[0] = 0.0f;
    target->addon_gravity_diag_last_delta[1] = 0.0f;
    target->addon_gravity_diag_last_delta[2] = 0.0f;
    target->addon_gravity_diag_last_bend[0] = 0.0f;
    target->addon_gravity_diag_last_bend[1] = 0.0f;
    target->addon_gravity_diag_last_bend[2] = 0.0f;
    target->addon_gravity_diag_last_rotation[0] = 0.0f;
    target->addon_gravity_diag_last_rotation[1] = 0.0f;
    target->addon_gravity_diag_last_rotation[2] = 0.0f;
}

static void addon_gravity_diag_log_pose_probe(physx_chain_t *chain,
                                              physx_target_t *target,
                                              const char *label,
                                              const float drive[3])
{
    float bend[3] = { 0.0f, 0.0f, 0.0f };
    float pitch = 0.0f;
    float roll = 0.0f;
    float bend_len;
    int valid;
    int secondary_only;
    int secondary_disabled;
    if (!chain || !target || !label || !drive) return;
    valid = addon_chain_world_gravity_bend_vector(chain, target, drive, bend);
    bend_len = physx_vec3_len(bend);
    if (valid) {
        addon_chain_bend_to_output_angles(
            chain, target, bend, 1.0f, &pitch, &roll);
    }
    secondary_only = physx_absf(drive[0]) < 0.0001f &&
                     physx_absf(drive[1]) < 0.0001f &&
                     physx_absf(drive[2]) > 0.0001f;
    secondary_disabled =
        physx_absf(physics_environment_cfg.gravity_horizontal_secondary_body_chain_scale) <= 0.000001f &&
        physx_absf(physics_environment_cfg.gravity_vertical_secondary_body_chain_scale) <= 0.000001f;
    log_line("ADDON GRAVITY TEST POSE chain=\"%s\" target=\"%s\" label=\"%s\" drive=(%.5f,%.5f,%.5f) bend_valid=%d bend=(%.5f,%.5f,%.5f) bend_len=%.5f approx_rotation_delta=(pitch=%.3f,roll=%.3f) secondary_only=%d secondary_disabled=%d verdict=\"%s\"",
             chain->name,
             target->name,
             label,
             drive[0], drive[1], drive[2],
             valid,
             bend[0], bend[1], bend[2],
             bend_len,
             pitch, roll,
             secondary_only,
             secondary_disabled,
             (secondary_only && secondary_disabled &&
              drive[2] > 0.0f && bend_len > 0.0001f) ?
                "ANTIPODAL_FALL_BEND" :
             (secondary_only && secondary_disabled &&
              drive[2] <= 0.0f && bend_len <= 0.0001f) ?
                "EXPECTED_ZERO" :
                (secondary_only && secondary_disabled) ?
                    "UNEXPECTED_SECONDARY_BEND" :
                    (valid ? "BEND_NONZERO" : "BEND_ZERO"));
}

static int addon_gravity_diag_expected_angle_ok(float value,
                                                int expected_sign,
                                                float min_abs)
{
    if (expected_sign > 0) return value >= min_abs;
    if (expected_sign < 0) return value <= -min_abs;
    return physx_absf(value) <= min_abs;
}

static void addon_gravity_diag_log_chain_pose(physx_chain_t *chain,
                                              const char *label,
                                              const float drive[3],
                                              int expected_pitch_sign,
                                              int expected_roll_sign)
{
    const char *name[3] = { "", "", "" };
    int valid[3] = { 0, 0, 0 };
    float pitch[3] = { 0.0f, 0.0f, 0.0f };
    float roll[3] = { 0.0f, 0.0f, 0.0f };
    float bend_len[3] = { 0.0f, 0.0f, 0.0f };
    int t;
    int slot = 0;
    int pass = 1;
    if (!chain || !label || !drive) return;
    for (t = 1; t < chain->target_count && slot < 3; t++) {
        physx_target_t *target = &chain->targets[t];
        float bend[3] = { 0.0f, 0.0f, 0.0f };
        name[slot] = target->name;
        valid[slot] = addon_chain_world_gravity_bend_vector(
            chain, target, drive, bend);
        bend_len[slot] = physx_vec3_len(bend);
        if (valid[slot]) {
            addon_chain_bend_to_output_angles(
                chain, target, bend, 1.0f, &pitch[slot], &roll[slot]);
        }
        slot++;
    }
    if (slot < 3) {
        pass = 0;
    }
    for (t = 0; t < slot; t++) {
        if (expected_pitch_sign != 0 &&
            !addon_gravity_diag_expected_angle_ok(
                pitch[t], expected_pitch_sign, 5.0f)) {
            pass = 0;
        }
        if (expected_roll_sign != 0 &&
            !addon_gravity_diag_expected_angle_ok(
                roll[t], expected_roll_sign, 5.0f)) {
            pass = 0;
        }
        if (expected_pitch_sign == 0 && expected_roll_sign == 0) {
            if (valid[t] || bend_len[t] > 0.05f ||
                physx_absf(pitch[t]) > 1.0f ||
                physx_absf(roll[t]) > 1.0f) {
                pass = 0;
            }
        }
    }
    log_line("ADDON GRAVITY TEST CHAIN chain=\"%s\" label=\"%s\" drive=(%.5f,%.5f,%.5f) targets=(\"%s\",\"%s\",\"%s\") valid=(%d,%d,%d) pitch=(%.3f,%.3f,%.3f) roll=(%.3f,%.3f,%.3f) bend_len=(%.5f,%.5f,%.5f) expected=(pitch_sign=%d,roll_sign=%d) verdict=\"%s\" note=\"chain-level world-gravity shape check; all three tail bones should agree on the driven angle sign\"",
             chain->name,
             label,
             drive[0], drive[1], drive[2],
             name[0], name[1], name[2],
             valid[0], valid[1], valid[2],
             pitch[0], pitch[1], pitch[2],
             roll[0], roll[1], roll[2],
             bend_len[0], bend_len[1], bend_len[2],
             expected_pitch_sign,
             expected_roll_sign,
             pass ? "PASS" : "FAIL");
}

static void addon_gravity_diag_log_axis_probe(physx_chain_t *chain,
                                              physx_target_t *target,
                                              const float live_drive[3])
{
    float h_pos_drive[3] = { 1.0f, 0.0f, 0.0f };
    float h_neg_drive[3] = { -1.0f, 0.0f, 0.0f };
    float v_pos_drive[3] = { 0.0f, 1.0f, 0.0f };
    float v_neg_drive[3] = { 0.0f, -1.0f, 0.0f };
    float secondary_pos_drive[3] = { 0.0f, 0.0f, 1.0f };
    float secondary_neg_drive[3] = { 0.0f, 0.0f, -1.0f };
    float h_pos[3] = { 0.0f, 0.0f, 0.0f };
    float h_neg[3] = { 0.0f, 0.0f, 0.0f };
    float v_pos[3] = { 0.0f, 0.0f, 0.0f };
    float v_neg[3] = { 0.0f, 0.0f, 0.0f };
    int h_pos_valid;
    int h_neg_valid;
    int v_pos_valid;
    int v_neg_valid;
    if (!chain || !target) return;
    h_pos_valid = addon_chain_world_gravity_bend_vector(
        chain, target, h_pos_drive, h_pos);
    h_neg_valid = addon_chain_world_gravity_bend_vector(
        chain, target, h_neg_drive, h_neg);
    v_pos_valid = addon_chain_world_gravity_bend_vector(
        chain, target, v_pos_drive, v_pos);
    v_neg_valid = addon_chain_world_gravity_bend_vector(
        chain, target, v_neg_drive, v_neg);
    log_line("ADDON GRAVITY TEST AXIS chain=\"%s\" target=\"%s\" rest=(%.5f,%.5f,%.5f) length=%.5f configured_axes=(h_tail=%d,v_tail=%d) configured_scales=(h=%.3f,v=%.3f) synthetic_h_pos=(valid=%d,bend=(%.5f,%.5f,%.5f),len=%.5f) synthetic_h_neg=(valid=%d,bend=(%.5f,%.5f,%.5f),len=%.5f) synthetic_v_pos=(valid=%d,bend=(%.5f,%.5f,%.5f),len=%.5f) synthetic_v_neg=(valid=%d,bend=(%.5f,%.5f,%.5f),len=%.5f)",
             chain->name,
             target->name,
             target->sim_rest[0],
             target->sim_rest[1],
             target->sim_rest[2],
             target->sim_length,
             chain->rotation_drive_horizontal_tail_axis,
             chain->rotation_drive_vertical_tail_axis,
             chain->rotation_drive_horizontal_scale,
             chain->rotation_drive_vertical_scale,
             h_pos_valid,
             h_pos[0], h_pos[1], h_pos[2],
             physx_vec3_len(h_pos),
             h_neg_valid,
             h_neg[0], h_neg[1], h_neg[2],
             physx_vec3_len(h_neg),
             v_pos_valid,
             v_pos[0], v_pos[1], v_pos[2],
             physx_vec3_len(v_pos),
             v_neg_valid,
             v_neg[0], v_neg[1], v_neg[2],
             physx_vec3_len(v_neg));
    if (live_drive) {
        addon_gravity_diag_log_pose_probe(chain, target, "live_drive", live_drive);
    }
    addon_gravity_diag_log_pose_probe(chain, target, "primary_h_positive", h_pos_drive);
    addon_gravity_diag_log_pose_probe(chain, target, "primary_h_negative", h_neg_drive);
    addon_gravity_diag_log_pose_probe(chain, target, "primary_v_positive", v_pos_drive);
    addon_gravity_diag_log_pose_probe(chain, target, "primary_v_negative", v_neg_drive);
    addon_gravity_diag_log_pose_probe(chain, target, "secondary_positive", secondary_pos_drive);
    addon_gravity_diag_log_pose_probe(chain, target, "secondary_negative", secondary_neg_drive);
}

static void addon_gravity_diag_start(physx_sidecar_t *sc,
                                     physx_chain_t *chain,
                                     DWORD now,
                                     int gravity_valid,
                                     const float drive[3])
{
    int t;
    if (!addon_gravity_diag_enabled(chain) ||
        chain->addon_gravity_diag_state != 0) {
        return;
    }
    chain->addon_gravity_diag_state = 1;
    chain->addon_gravity_diag_start_tick = now;
    chain->addon_gravity_diag_status_tick = now;
    chain->addon_gravity_diag_sample_count = 0;
    chain->addon_gravity_diag_done_logged = 0;
    chain->addon_gravity_diag_any_valid = gravity_valid ? 1 : 0;
    chain->addon_gravity_diag_drive[0] = drive ? drive[0] : 0.0f;
    chain->addon_gravity_diag_drive[1] = drive ? drive[1] : 0.0f;
    chain->addon_gravity_diag_drive[2] = drive ? drive[2] : 0.0f;
    for (t = 0; t < chain->target_count; t++) {
        addon_gravity_diag_reset_target(&chain->targets[t]);
    }
    log_line("============================================================");
    log_line("============== ADDON GRAVITY TEST STARTED ==================");
    log_line("ADDON GRAVITY TEST STARTED chain=\"%s\" target_count=%d parent=\"%s\" sidecar=\"%s\" gravity_valid=%d gravity_drive=(%.5f,%.5f,%.5f) duration_ms=7000 note=\"launch the room, wait for DONE, close game, then write done\"",
             chain->name,
             chain->target_count,
             chain->parent_name,
             sc ? sc->path : "",
             gravity_valid,
             drive ? drive[0] : 0.0f,
             drive ? drive[1] : 0.0f,
             drive ? drive[2] : 0.0f);
    log_line("============================================================");
}

static void addon_gravity_diag_finish(physx_sidecar_t *sc,
                                      physx_chain_t *chain,
                                      DWORD now)
{
    int t;
    int checked = 0;
    int reacted = 1;
    int bend_nonzero = 1;
    float upright_drive[3] = { 0.0f, 0.0f, -1.0f };
    float upside_drive[3] = { 0.0f, 0.0f, 1.0f };
    float face_down_drive[3] = { 0.0f, 1.0f, 0.0f };
    float face_up_drive[3] = { 0.0f, -1.0f, 0.0f };
    float side_pos_drive[3] = { 1.0f, 0.0f, 0.0f };
    float side_neg_drive[3] = { -1.0f, 0.0f, 0.0f };
    const float delta_threshold = 0.00010f;
    const float bend_threshold = 0.00010f;
    if (!addon_gravity_diag_enabled(chain) ||
        chain->addon_gravity_diag_state != 1 ||
        chain->addon_gravity_diag_done_logged) {
        return;
    }
    for (t = 1; t < chain->target_count; t++) {
        physx_target_t *target = &chain->targets[t];
        if (!contains_i(target->name, "physx_tail")) continue;
        checked++;
        if (target->addon_gravity_diag_max_delta <= delta_threshold) {
            reacted = 0;
        }
        if (target->addon_gravity_diag_max_bend_len <= bend_threshold) {
            bend_nonzero = 0;
        }
    }
    if (checked <= 0) {
        reacted = 0;
        bend_nonzero = 0;
    }
    chain->addon_gravity_diag_done_logged = 1;
    chain->addon_gravity_diag_state = 2;
    log_line("============================================================");
    log_line("============= ADDON GRAVITY TEST DONE!!!!! ================");
    log_line("ADDON GRAVITY TEST DONE chain=\"%s\" elapsed_ms=%lu samples=%d checked_tail_bones=%d result=\"%s\" gravity_any_valid=%d last_gravity_drive=(%.5f,%.5f,%.5f) sidecar=\"%s\" note=\"safe to close the game now and write done\"",
             chain->name,
             (unsigned long)(now - chain->addon_gravity_diag_start_tick),
             chain->addon_gravity_diag_sample_count,
             checked,
             (reacted && bend_nonzero && chain->addon_gravity_diag_any_valid) ?
                "PASS" : "FAIL",
             chain->addon_gravity_diag_any_valid,
             chain->addon_gravity_diag_drive[0],
             chain->addon_gravity_diag_drive[1],
             chain->addon_gravity_diag_drive[2],
             sc ? sc->path : "");
    for (t = 1; t < chain->target_count; t++) {
        physx_target_t *target = &chain->targets[t];
        if (!contains_i(target->name, "physx_tail")) continue;
        log_line("ADDON GRAVITY TEST TARGET chain=\"%s\" index=%d target=\"%s\" samples=%d rest=(%.5f,%.5f,%.5f) sim_offset=(%.5f,%.5f,%.5f) start_offset=(%.5f,%.5f,%.5f) last_delta=(%.5f,%.5f,%.5f) max_delta=%.6f max_bend_len=%.6f last_bend=(%.5f,%.5f,%.5f) last_rotation=(%.5f,%.5f,%.5f) velocity=(%.5f,%.5f,%.5f) verdict_delta=\"%s\" verdict_bend=\"%s\"",
                 chain->name,
                 t,
                 target->name,
                 target->addon_gravity_diag_samples,
                 target->sim_rest[0],
                 target->sim_rest[1],
                 target->sim_rest[2],
                 target->sim_offset[0],
                 target->sim_offset[1],
                 target->sim_offset[2],
                 target->addon_gravity_diag_start_offset[0],
                 target->addon_gravity_diag_start_offset[1],
                 target->addon_gravity_diag_start_offset[2],
                 target->addon_gravity_diag_last_delta[0],
                 target->addon_gravity_diag_last_delta[1],
                 target->addon_gravity_diag_last_delta[2],
                 target->addon_gravity_diag_max_delta,
                 target->addon_gravity_diag_max_bend_len,
                 target->addon_gravity_diag_last_bend[0],
                 target->addon_gravity_diag_last_bend[1],
                 target->addon_gravity_diag_last_bend[2],
                 target->addon_gravity_diag_last_rotation[0],
                 target->addon_gravity_diag_last_rotation[1],
                 target->addon_gravity_diag_last_rotation[2],
                 target->sim_velocity[0],
                 target->sim_velocity[1],
                 target->sim_velocity[2],
                 target->addon_gravity_diag_max_delta > delta_threshold ?
                    "MOVED" : "STATIC",
                 target->addon_gravity_diag_max_bend_len > bend_threshold ?
                    "BEND_NONZERO" : "BEND_ZERO");
        addon_gravity_diag_log_axis_probe(
            chain, target, chain->addon_gravity_diag_drive);
    }
    addon_gravity_diag_log_chain_pose(
        chain, "upright_rest", upright_drive, 0, 0);
    addon_gravity_diag_log_chain_pose(
        chain, "upside_down_antipodal", upside_drive, 1, 0);
    addon_gravity_diag_log_chain_pose(
        chain, "face_down_vertical_positive", face_down_drive, 1, 0);
    addon_gravity_diag_log_chain_pose(
        chain, "face_up_vertical_negative", face_up_drive, -1, 0);
    addon_gravity_diag_log_chain_pose(
        chain, "side_positive_horizontal", side_pos_drive, 0, -1);
    addon_gravity_diag_log_chain_pose(
        chain, "side_negative_horizontal", side_neg_drive, 0, 1);
    log_line("============================================================");
}

static void addon_gravity_diag_sample(physx_sidecar_t *sc,
                                      physx_chain_t *chain,
                                      physx_target_t *target,
                                      DWORD now,
                                      int gravity_valid,
                                      const float drive[3],
                                      int bend_valid,
                                      const float bend[3],
                                      const float rotation[3])
{
    float delta[3];
    float delta_len;
    float bend_len = 0.0f;
    if (!addon_gravity_diag_enabled(chain) || !target) return;
    if (chain->addon_gravity_diag_state == 2) return;
    if (chain->addon_gravity_diag_state == 0) {
        addon_gravity_diag_start(sc, chain, now, gravity_valid, drive);
    }
    if (chain->addon_gravity_diag_state != 1) return;
    if (gravity_valid && drive) {
        chain->addon_gravity_diag_any_valid = 1;
        chain->addon_gravity_diag_drive[0] = drive[0];
        chain->addon_gravity_diag_drive[1] = drive[1];
        chain->addon_gravity_diag_drive[2] = drive[2];
    }
    if (!target->addon_gravity_diag_initialized) {
        target->addon_gravity_diag_initialized = 1;
        target->addon_gravity_diag_start_offset[0] = target->sim_rest[0];
        target->addon_gravity_diag_start_offset[1] = target->sim_rest[1];
        target->addon_gravity_diag_start_offset[2] = target->sim_rest[2];
    }
    delta[0] = target->sim_offset[0] -
               target->addon_gravity_diag_start_offset[0];
    delta[1] = target->sim_offset[1] -
               target->addon_gravity_diag_start_offset[1];
    delta[2] = target->sim_offset[2] -
               target->addon_gravity_diag_start_offset[2];
    delta_len = physx_vec3_len(delta);
    if (bend_valid && bend) {
        bend_len = physx_vec3_len(bend);
        target->addon_gravity_diag_last_bend[0] = bend[0];
        target->addon_gravity_diag_last_bend[1] = bend[1];
        target->addon_gravity_diag_last_bend[2] = bend[2];
    }
    if (rotation) {
        target->addon_gravity_diag_last_rotation[0] = rotation[0];
        target->addon_gravity_diag_last_rotation[1] = rotation[1];
        target->addon_gravity_diag_last_rotation[2] = rotation[2];
    }
    target->addon_gravity_diag_last_delta[0] = delta[0];
    target->addon_gravity_diag_last_delta[1] = delta[1];
    target->addon_gravity_diag_last_delta[2] = delta[2];
    if (delta_len > target->addon_gravity_diag_max_delta) {
        target->addon_gravity_diag_max_delta = delta_len;
    }
    if (bend_len > target->addon_gravity_diag_max_bend_len) {
        target->addon_gravity_diag_max_bend_len = bend_len;
    }
    target->addon_gravity_diag_samples++;
    chain->addon_gravity_diag_sample_count++;
    if (now - chain->addon_gravity_diag_status_tick >= 1500u) {
        chain->addon_gravity_diag_status_tick = now;
        log_line("ADDON GRAVITY TEST RUNNING chain=\"%s\" elapsed_ms=%lu samples=%d gravity_valid=%d last_drive=(%.5f,%.5f,%.5f) note=\"wait for ADDON GRAVITY TEST DONE\"",
                 chain->name,
                 (unsigned long)(now - chain->addon_gravity_diag_start_tick),
                 chain->addon_gravity_diag_sample_count,
                 gravity_valid,
                 drive ? drive[0] : 0.0f,
                 drive ? drive[1] : 0.0f,
                 drive ? drive[2] : 0.0f);
    }
    if (now - chain->addon_gravity_diag_start_tick >= 7000u) {
        addon_gravity_diag_finish(sc, chain, now);
    }
}

static int addon_parent_camera_relative_translation_step(
    physx_sidecar_t *sc,
    physx_chain_t *chain,
    const char *runtime,
    void *parent_raw,
    int parent_offset,
    const float raw_step[3],
    float out[3],
    DWORD now)
{
    char person[16];
    void *trs_raw;
    float parent_matrix[9];
    float trs_matrix[9];
    float trs_inverse[9];
    float parent_relative_basis[9];
    float model_view_basis[9];
    float model_world_basis[9];
    float parent_rest_inverse[9];
    float parent_delta[9];
    float source_local_basis[9];
    float *parent_pos;
    float *trs_pos;
    float relative_view[3];
    float relative_local[3];
    float relative_step[3];
    float global_world[3];
    float global_world_step[3];
    float global_local_step[3];
    float sampled_global_local_step[3];
    float combined_step[3];
    float local_step[3];
    const float *camera_inverse = captured_camera_inverse;
    float step_sq;
    float step_len;
    int row;
    const int mode = 2;
    const char *mode_label = "camera-neutral-relative-plus-trusted-root";
    DWORD camera_quarantine_ms =
        (DWORD)physics_environment_cfg.body_chain_camera_quarantine_ms;
    DWORD camera_age_ms = captured_camera_change_tick ?
        (now - captured_camera_change_tick) : 0xffffffffu;
    int root_camera_untrusted_now =
        chain && chain->addon_root_drive_camera_last_untrusted_tick == now;
    int camera_motion_active;
    const float camera_relative_deadzone = 0.0010f;
    /* Parent and TRS_group are updated at adjacent points in TK17's render
       pass. Keep only the world/root component quarantined long enough for
       both snapshots to settle; the camera-neutral relative sampler below
       continues advancing throughout this window. */
    if (camera_quarantine_ms < 48u) camera_quarantine_ms = 48u;
    camera_motion_active =
        captured_camera_inverse_valid &&
        (root_camera_untrusted_now ||
         camera_age_ms < camera_quarantine_ms);
    person[0] = '\0';
    if (out) out[0] = out[1] = out[2] = 0.0f;
    if (!physics_environment_cfg.body_chain_camera_relative_orientation ||
        !chain || !raw_step || !out || !parent_raw || parent_offset < 0) {
        return 0;
    }
    if (!ptr_readable((BYTE*)parent_raw + parent_offset,
                      sizeof(float) * 3)) {
        return 0;
    }
    parent_pos = (float*)((BYTE*)parent_raw + parent_offset);
    if (!physx_vec3_sane_limit(parent_pos, 64.0f)) {
        return 0;
    }
    if (!addon_parent_camera_relative_person(sc, chain, runtime,
                                             person, sizeof(person))) {
        return 0;
    }
    trs_raw = addon_parent_camera_relative_trs_raw(chain, person, now);
    if (!trs_raw ||
        !ptr_readable((BYTE*)trs_raw + parent_offset, sizeof(float) * 3) ||
        !body_chain_read_mat3_rows(parent_raw, parent_matrix) ||
        !body_chain_read_mat3_rows(trs_raw, trs_matrix) ||
        !body_chain_mat3_inverse(trs_matrix, trs_inverse)) {
        return 0;
    }
    trs_pos = (float*)((BYTE*)trs_raw + parent_offset);
    if (!physx_vec3_sane_limit(trs_pos, 64.0f)) return 0;
    body_chain_mat3_multiply(parent_matrix, trs_inverse,
                             parent_relative_basis);
    if (!addon_normalize_basis_rows(parent_relative_basis)) {
        return 0;
    }
    relative_view[0] = parent_pos[0] - trs_pos[0];
    relative_view[1] = parent_pos[1] - trs_pos[1];
    relative_view[2] = parent_pos[2] - trs_pos[2];
    addon_transform_row_vector3(relative_view, trs_inverse, relative_local);
    if (!physx_vec3_sane_limit(relative_local, 64.0f)) return 0;

    memcpy(model_view_basis, trs_matrix, sizeof(model_view_basis));
    if (!addon_normalize_basis_rows(model_view_basis)) return 0;

    if (!chain->addon_parent_camera_relative_translation_initialized ||
        chain->addon_parent_camera_relative_translation_raw != parent_raw ||
        chain->addon_parent_camera_relative_translation_offset !=
            parent_offset ||
        chain->addon_parent_camera_relative_translation_mode != mode ||
        !chain->addon_parent_camera_relative_translation_basis_initialized) {
        chain->addon_parent_camera_relative_translation_initialized = 1;
        chain->addon_parent_camera_relative_translation_basis_initialized = 1;
        chain->addon_parent_camera_relative_translation_raw = parent_raw;
        chain->addon_parent_camera_relative_translation_offset =
            parent_offset;
        chain->addon_parent_camera_relative_translation_mode = mode;
        chain->addon_parent_camera_relative_translation_prev[0] =
            relative_local[0];
        chain->addon_parent_camera_relative_translation_prev[1] =
            relative_local[1];
        chain->addon_parent_camera_relative_translation_prev[2] =
            relative_local[2];
        memcpy(chain->addon_parent_camera_relative_translation_parent_rest,
               parent_relative_basis, sizeof(parent_relative_basis));
        chain->addon_parent_camera_relative_translation_global_initialized = 0;
        chain->addon_parent_camera_relative_translation_global_pending_valid =
            0;
        chain->addon_parent_camera_relative_translation_camera_hold_active =
            camera_motion_active ? 1 : 0;
        if (!camera_motion_active && captured_camera_inverse_valid &&
            camera_view_to_world_point(trs_pos, global_world) &&
            physx_vec3_sane_limit(global_world, 4096.0f)) {
            for (row = 0; row < 3; row++) {
                const float *view_axis = &model_view_basis[row * 3];
                float *world_axis = &model_world_basis[row * 3];
                world_axis[0] = view_axis[0] * camera_inverse[0] +
                                view_axis[1] * camera_inverse[4] +
                                view_axis[2] * camera_inverse[8];
                world_axis[1] = view_axis[0] * camera_inverse[1] +
                                view_axis[1] * camera_inverse[5] +
                                view_axis[2] * camera_inverse[9];
                world_axis[2] = view_axis[0] * camera_inverse[2] +
                                view_axis[1] * camera_inverse[6] +
                                view_axis[2] * camera_inverse[10];
            }
            if (addon_normalize_basis_rows(model_world_basis)) {
                memcpy(
                    chain->addon_parent_camera_relative_translation_model_rest,
                    model_world_basis, sizeof(model_world_basis));
                memcpy(
                    chain->addon_parent_camera_relative_translation_global_prev,
                    global_world, sizeof(global_world));
                chain->addon_parent_camera_relative_translation_global_initialized =
                    1;
            }
        }
        out[0] = 0.0f;
        out[1] = 0.0f;
        out[2] = 0.0f;
        if (!chain->addon_parent_camera_relative_translation_log_tick ||
            now - chain->addon_parent_camera_relative_translation_log_tick >=
                1000u) {
            chain->addon_parent_camera_relative_translation_log_tick = now;
            log_line("addon-chain camera-safe parent translation baseline chain=\"%s\" person=\"%s\" runtime=\"%s\" mode=\"%s\" camera_active=%d parent_raw=%p trs_raw=%p offset=0x%03x relative=(%.5f,%.5f,%.5f) global_initialized=%d note=\"dance motion uses parent minus TRS_group in the live body basis; root translation is sampled only on camera-trusted frames\"",
                     chain->name,
                     person,
                     runtime ? runtime : "",
                     mode_label,
                     camera_motion_active,
                     parent_raw,
                     trs_raw,
                     parent_offset,
                     relative_local[0],
                     relative_local[1],
                     relative_local[2],
                     chain->addon_parent_camera_relative_translation_global_initialized);
        }
        return 1;
    }
    relative_step[0] = relative_local[0] -
        chain->addon_parent_camera_relative_translation_prev[0];
    relative_step[1] = relative_local[1] -
        chain->addon_parent_camera_relative_translation_prev[1];
    relative_step[2] = relative_local[2] -
        chain->addon_parent_camera_relative_translation_prev[2];
    chain->addon_parent_camera_relative_translation_prev[0] =
        relative_local[0];
    chain->addon_parent_camera_relative_translation_prev[1] =
        relative_local[1];
    chain->addon_parent_camera_relative_translation_prev[2] =
        relative_local[2];
    if (!physx_vec3_sane_limit(relative_step, 8.0f)) return 0;
    step_sq = relative_step[0] * relative_step[0] +
              relative_step[1] * relative_step[1] +
              relative_step[2] * relative_step[2];
    if (step_sq > 16.0f) return 0;
    step_len = step_sq > 0.0f ?
        (float)sqrt((double)step_sq) : 0.0f;
    if (camera_motion_active && step_len > 0.0f) {
        if (step_len <= camera_relative_deadzone) {
            relative_step[0] = 0.0f;
            relative_step[1] = 0.0f;
            relative_step[2] = 0.0f;
        } else {
            float keep =
                (step_len - camera_relative_deadzone) / step_len;
            relative_step[0] *= keep;
            relative_step[1] *= keep;
            relative_step[2] *= keep;
        }
    }

    global_local_step[0] = 0.0f;
    global_local_step[1] = 0.0f;
    global_local_step[2] = 0.0f;
    sampled_global_local_step[0] = 0.0f;
    sampled_global_local_step[1] = 0.0f;
    sampled_global_local_step[2] = 0.0f;
    if (camera_motion_active) {
        chain->addon_parent_camera_relative_translation_camera_hold_active = 1;
        chain->addon_parent_camera_relative_translation_global_pending_valid =
            0;
    } else if (captured_camera_inverse_valid &&
               camera_view_to_world_point(trs_pos, global_world) &&
               physx_vec3_sane_limit(global_world, 4096.0f)) {
        for (row = 0; row < 3; row++) {
            const float *view_axis = &model_view_basis[row * 3];
            float *world_axis = &model_world_basis[row * 3];
            world_axis[0] = view_axis[0] * camera_inverse[0] +
                            view_axis[1] * camera_inverse[4] +
                            view_axis[2] * camera_inverse[8];
            world_axis[1] = view_axis[0] * camera_inverse[1] +
                            view_axis[1] * camera_inverse[5] +
                            view_axis[2] * camera_inverse[9];
            world_axis[2] = view_axis[0] * camera_inverse[2] +
                            view_axis[1] * camera_inverse[6] +
                            view_axis[2] * camera_inverse[10];
        }
        if (addon_normalize_basis_rows(model_world_basis)) {
            if (!chain->addon_parent_camera_relative_translation_global_initialized ||
                chain->addon_parent_camera_relative_translation_camera_hold_active) {
                memcpy(
                    chain->addon_parent_camera_relative_translation_global_prev,
                    global_world, sizeof(global_world));
                chain->addon_parent_camera_relative_translation_global_initialized =
                    1;
                chain->addon_parent_camera_relative_translation_camera_hold_active =
                    0;
                chain->addon_parent_camera_relative_translation_global_pending_valid =
                    0;
            } else {
                global_world_step[0] = global_world[0] -
                    chain->addon_parent_camera_relative_translation_global_prev[0];
                global_world_step[1] = global_world[1] -
                    chain->addon_parent_camera_relative_translation_global_prev[1];
                global_world_step[2] = global_world[2] -
                    chain->addon_parent_camera_relative_translation_global_prev[2];
                memcpy(
                    chain->addon_parent_camera_relative_translation_global_prev,
                    global_world, sizeof(global_world));
                step_sq = global_world_step[0] * global_world_step[0] +
                          global_world_step[1] * global_world_step[1] +
                          global_world_step[2] * global_world_step[2];
                if (physx_vec3_sane_limit(global_world_step, 8.0f) &&
                    step_sq <= 16.0f) {
                    sampled_global_local_step[0] =
                        vec3_dot(global_world_step, &model_world_basis[0]);
                    sampled_global_local_step[1] =
                        vec3_dot(global_world_step, &model_world_basis[3]);
                    sampled_global_local_step[2] =
                        vec3_dot(global_world_step, &model_world_basis[6]);
                    if (chain->addon_parent_camera_relative_translation_global_pending_valid &&
                        chain->addon_parent_camera_relative_translation_global_pending_version ==
                            captured_camera_version) {
                        memcpy(global_local_step,
                               chain->addon_parent_camera_relative_translation_global_pending,
                               sizeof(global_local_step));
                    }
                    memcpy(
                        chain->addon_parent_camera_relative_translation_global_pending,
                        sampled_global_local_step,
                        sizeof(sampled_global_local_step));
                    chain->addon_parent_camera_relative_translation_global_pending_version =
                        captured_camera_version;
                    chain->addon_parent_camera_relative_translation_global_pending_valid =
                        1;
                } else {
                    chain->addon_parent_camera_relative_translation_global_pending_valid =
                        0;
                }
            }
        }
    }
    if (!body_chain_mat3_inverse(
            chain->addon_parent_camera_relative_translation_parent_rest,
            parent_rest_inverse)) {
        return 0;
    }
    body_chain_mat3_multiply(parent_rest_inverse, parent_relative_basis,
                             parent_delta);
    memcpy(source_local_basis, parent_delta, sizeof(parent_delta));
    if (!addon_normalize_basis_rows(source_local_basis)) return 0;
    combined_step[0] = relative_step[0] + global_local_step[0];
    combined_step[1] = relative_step[1] + global_local_step[1];
    combined_step[2] = relative_step[2] + global_local_step[2];
    local_step[0] = vec3_dot(combined_step, &source_local_basis[0]);
    local_step[1] = vec3_dot(combined_step, &source_local_basis[3]);
    local_step[2] = vec3_dot(combined_step, &source_local_basis[6]);
    if (!physx_vec3_sane_limit(local_step, 8.0f)) return 0;
    out[0] = local_step[0];
    out[1] = local_step[1];
    out[2] = local_step[2];
    return 1;
}

static int addon_effective_parent_runtime_translation_step(
    physx_sidecar_t *sc,
    physx_chain_t *chain,
    physx_target_t *parent,
    float out[3],
    DWORD now,
    int *runtime_available)
{
    const char *runtime = "";
    void *raw;
    int offset = -1;
    if (runtime_available) *runtime_available = 0;
    if (!chain || !parent || !out) return 0;
    chain->addon_parent_translation_camera_relative = 0;
    raw = addon_effective_parent_cached_runtime_raw(
        sc, chain, parent, now, &runtime);
    if (!raw) return 0;
    if (runtime_available) *runtime_available = 1;
    if (addon_effective_parent_translation_scan_base(
            chain, parent, raw, runtime, out, &offset, now)) {
        float raw_step[3];
        int camera_relative_translation;
        raw_step[0] = out[0];
        raw_step[1] = out[1];
        raw_step[2] = out[2];
        camera_relative_translation =
            addon_parent_camera_relative_translation_step(
                sc, chain, runtime, raw, offset, raw_step, out, now);
        chain->addon_parent_translation_camera_relative =
            camera_relative_translation ? 1 : 0;
        if (defaults_cfg.debug &&
            (!chain->addon_parent_rotation_auto_log_tick ||
             now - chain->addon_parent_rotation_auto_log_tick >= 500u)) {
            chain->addon_parent_rotation_auto_log_tick = now;
            log_line("addon-chain effective parent runtime translation selected chain=\"%s\" parent=\"%s\" runtime=\"%s\" base=%p offset=0x%03x raw_step=(%.5f,%.5f,%.5f) parent_local_step=(%.5f,%.5f,%.5f) step_len=%.5f camera_relative=%d note=\"configured-parent displacement is projected onto that parent's current live orientation; source-axis and tail-axis mapping remain independent\"",
                     chain->name,
                     parent->name,
                     runtime,
                     raw,
                     offset,
                     raw_step[0], raw_step[1], raw_step[2],
                     out[0], out[1], out[2],
                     addon_vec3_len_exact(out),
                     camera_relative_translation);
        }
        return 1;
    }
    return 0;
}

static int addon_effective_parent_runtime_rotation_step(
    physx_sidecar_t *sc,
    physx_chain_t *chain,
    physx_target_t *parent,
    float out[3],
    DWORD now,
    int *runtime_available)
{
    const char *runtime = "";
    void *raw;
    int offset = -1;
    int camera_relative_handled = 0;
    int parent_can_use_body_runtime;
    if (runtime_available) *runtime_available = 0;
    if (!chain || !parent || !out) return 0;
    parent_can_use_body_runtime =
        addon_parent_name_can_use_body_drive(parent->name);
    if (!parent_can_use_body_runtime && parent->raw_object) {
        chain->addon_parent_rotation_camera_relative = 0;
        chain->addon_parent_rotation_camera_relative_available = 0;
        if (addon_effective_parent_runtime_camera_relative_rotation_step(
                sc, chain, parent, parent->raw_object,
                "addon-local-parent", out, now,
                &camera_relative_handled)) {
            chain->addon_parent_rotation_camera_relative = 1;
            chain->addon_parent_rotation_camera_relative_available = 1;
            return 1;
        }
        if (camera_relative_handled) {
            chain->addon_parent_rotation_camera_relative_available = 1;
            if (runtime_available) *runtime_available = 1;
            return 0;
        }
    }
    if (!parent_can_use_body_runtime) return 0;
    raw = addon_effective_parent_cached_runtime_raw(
        sc, chain, parent, now, &runtime);
    if (!raw) return 0;
    if (runtime_available) *runtime_available = 1;
    chain->addon_parent_rotation_camera_relative = 0;
    chain->addon_parent_rotation_camera_relative_available = 0;
    if (addon_effective_parent_runtime_camera_relative_rotation_step(
            sc, chain, parent, raw, runtime, out, now,
            &camera_relative_handled)) {
        chain->addon_parent_rotation_camera_relative = 1;
        chain->addon_parent_rotation_camera_relative_available = 1;
        return 1;
    }
    if (camera_relative_handled) {
        chain->addon_parent_rotation_camera_relative_available = 1;
        return 0;
    }
    if (addon_body_parent_scan_base(chain, parent, raw,
                                    runtime,
                                    "parent-runtime",
                                    out, &offset, 1, now)) {
        if (defaults_cfg.debug &&
            (!chain->addon_parent_rotation_auto_log_tick ||
             now - chain->addon_parent_rotation_auto_log_tick >= 500u)) {
            chain->addon_parent_rotation_auto_log_tick = now;
            log_line("addon-chain effective parent runtime rotation selected chain=\"%s\" parent=\"%s\" runtime=\"%s\" base=%p offset=0x%03x step=(%.5f,%.5f,%.5f) step_len=%.5f note=\"live body runtime for configured parent only; ancestor/root motion enters only through this parent\"",
                     chain->name,
                     parent->name,
                     runtime,
                     raw,
                     offset,
                     out[0], out[1], out[2],
                     addon_vec3_len_exact(out));
        }
        return 1;
    }
    return 0;
}

static int addon_parent_translation_camera_safe_for_root(
    const physx_chain_t *chain)
{
    if (!chain || !chain->addon_parent_translation_camera_relative) {
        return 0;
    }
    return chain->addon_parent_camera_relative_translation_initialized &&
           chain->addon_parent_camera_relative_translation_mode == 2;
}

static int resolve_addon_chain_effective_parent_translation_step(
    physx_sidecar_t *sc,
    physx_chain_t *chain,
    physx_target_t *parent,
    float out[3],
    DWORD now)
{
    void *direct_bases[4];
    const char *direct_labels[4];
    int i;
    int runtime_available = 0;
    int best_offset = -1;
    float best_step[3] = { 0.0f, 0.0f, 0.0f };
    float best_len = 0.0f;
    const char *best_source = "";
    void *best_base = NULL;
    if (out) {
        out[0] = 0.0f;
        out[1] = 0.0f;
        out[2] = 0.0f;
    }
    if (!chain || !parent || !out || !chain->addon_chain ||
        !parent->name[0] || parent->addon_simulated_target) {
        return 0;
    }
    chain->addon_parent_translation_camera_relative = 0;

    if (addon_parent_name_can_use_body_drive(parent->name)) {
        if (addon_effective_parent_runtime_translation_step(
                sc, chain, parent, out, now, &runtime_available)) {
            return 1;
        }
        if (runtime_available) return 0;
    }

    direct_bases[0] = parent->raw_object;
    direct_bases[1] = parent->object;
    direct_bases[2] = parent->s_raw_object;
    direct_bases[3] = parent->s_object;
    direct_labels[0] = "parent-raw";
    direct_labels[1] = "parent-object";
    direct_labels[2] = "parent-s-raw";
    direct_labels[3] = "parent-s-object";

    for (i = 0; i < 4; i++) {
        float step[3] = { 0.0f, 0.0f, 0.0f };
        int offset = -1;
        float len;
        if (!direct_bases[i]) continue;
        if (!addon_effective_parent_translation_scan_base(
                chain, parent, direct_bases[i], direct_labels[i],
                step, &offset, now)) {
            continue;
        }
        len = addon_vec3_len_exact(step);
        if (len > best_len) {
            best_len = len;
            best_offset = offset;
            best_step[0] = step[0];
            best_step[1] = step[1];
            best_step[2] = step[2];
            best_source = direct_labels[i];
            best_base = direct_bases[i];
        }
    }

    if (best_len >= ADDON_BODY_PARENT_STEP_THRESHOLD) {
        float raw_step[3];
        int camera_relative_translation;
        raw_step[0] = best_step[0];
        raw_step[1] = best_step[1];
        raw_step[2] = best_step[2];
        out[0] = best_step[0];
        out[1] = best_step[1];
        out[2] = best_step[2];
        camera_relative_translation =
            addon_parent_camera_relative_translation_step(
                sc, chain, NULL, best_base, best_offset, raw_step, out,
                now);
        chain->addon_parent_translation_camera_relative =
            camera_relative_translation ? 1 : 0;
        if (defaults_cfg.debug &&
            (!chain->addon_parent_rotation_auto_log_tick ||
             now - chain->addon_parent_rotation_auto_log_tick >= 500u)) {
            chain->addon_parent_rotation_auto_log_tick = now;
            log_line("addon-chain effective parent translation selected chain=\"%s\" parent=\"%s\" source=%s base=%p offset=0x%03x raw_step=(%.5f,%.5f,%.5f) parent_local_step=(%.5f,%.5f,%.5f) step_len=%.5f camera_relative=%d note=\"single configured-parent displacement projected onto the current live parent orientation; source-axis and tail-axis mapping remain independent\"",
                     chain->name,
                     parent->name,
                     best_source,
                     best_base,
                     best_offset,
                     raw_step[0], raw_step[1], raw_step[2],
                     out[0], out[1], out[2],
                     addon_vec3_len_exact(out),
                     camera_relative_translation);
        }
        return 1;
    }
    return 0;
}

static int resolve_addon_chain_effective_parent_rotation_step(
    physx_sidecar_t *sc,
    physx_chain_t *chain,
    physx_target_t *parent,
    float out[3],
    DWORD now)
{
    void *direct_bases[4];
    const char *direct_labels[4];
    int i;
    int runtime_available = 0;
    int best_offset = -1;
    float best_step[3] = { 0.0f, 0.0f, 0.0f };
    float best_len = 0.0f;
    const char *best_source = "";
    void *best_base = NULL;
    if (out) {
        out[0] = 0.0f;
        out[1] = 0.0f;
        out[2] = 0.0f;
    }
    if (!chain || !parent || !out || !chain->addon_chain ||
        !parent->name[0] || parent->addon_simulated_target) {
        return 0;
    }
    chain->addon_parent_rotation_camera_relative = 0;
    chain->addon_parent_rotation_camera_relative_available = 0;

    if (addon_effective_parent_runtime_rotation_step(
            sc, chain, parent, out, now, &runtime_available)) {
        return 1;
    }
    if (runtime_available) return 0;

    direct_bases[0] = parent->raw_object;
    direct_bases[1] = parent->object;
    direct_bases[2] = parent->s_raw_object;
    direct_bases[3] = parent->s_object;
    direct_labels[0] = "parent-raw";
    direct_labels[1] = "parent-object";
    direct_labels[2] = "parent-s-raw";
    direct_labels[3] = "parent-s-object";

    for (i = 0; i < 4; i++) {
        float step[3] = { 0.0f, 0.0f, 0.0f };
        int offset = -1;
        float len;
        if (!direct_bases[i]) continue;
        if (!addon_body_parent_scan_base(chain, parent, direct_bases[i],
                                         parent->name, direct_labels[i],
                                         step, &offset, 1, now)) {
            continue;
        }
        len = addon_vec3_len_exact(step);
        if (len > best_len) {
            best_len = len;
            best_offset = offset;
            best_step[0] = step[0];
            best_step[1] = step[1];
            best_step[2] = step[2];
            best_source = direct_labels[i];
            best_base = direct_bases[i];
        }
    }

    if (best_len >= ADDON_BODY_PARENT_STEP_THRESHOLD) {
        out[0] = best_step[0];
        out[1] = best_step[1];
        out[2] = best_step[2];
        if (defaults_cfg.debug &&
            (!chain->addon_parent_rotation_auto_log_tick ||
             now - chain->addon_parent_rotation_auto_log_tick >= 500u)) {
            chain->addon_parent_rotation_auto_log_tick = now;
            log_line("addon-chain effective parent rotation selected chain=\"%s\" parent=\"%s\" source=%s base=%p offset=0x%03x step=(%.5f,%.5f,%.5f) step_len=%.5f note=\"single configured-parent basis drive; neck/root are not sampled separately\"",
                     chain->name,
                     parent->name,
                     best_source,
                     best_base,
                     best_offset,
                     out[0], out[1], out[2],
                     best_len);
        }
        return 1;
    }
    return 0;
}

static int addon_build_body_parent_runtime_name(int index,
                                                const char *parent_name,
                                                char *out,
                                                size_t outsz)
{
    static const char *persons[] = {
        "Person01", "Person02", "Person03", "Person04"
    };
    enum { VARIANT_COUNT = 6 };
    const char *person;
    int person_index;
    int variant;
    char s_name[128];
    if (!out || outsz == 0) return 0;
    out[0] = 0;
    if (!parent_name || !parent_name[0]) return 0;
    person_index = index / VARIANT_COUNT;
    variant = index % VARIANT_COUNT;
    if (person_index < 0 ||
        person_index >= (int)(sizeof(persons) / sizeof(persons[0]))) {
        return 0;
    }
    person = persons[person_index];
    if (parent_name[0] == 'S' || parent_name[0] == 's') {
        lstrcpynA(s_name, parent_name, sizeof(s_name));
    } else {
        _snprintf(s_name, sizeof(s_name), "S%s", parent_name);
        s_name[sizeof(s_name) - 1] = 0;
    }
    switch (variant) {
    case 0:
        make_body_runtime_name(out, outsz, person, parent_name);
        break;
    case 1:
        make_body_runtime_name(out, outsz, person, s_name);
        break;
    case 2:
        _snprintf(out, outsz, "%s:Model01:%s", person, parent_name);
        break;
    case 3:
        _snprintf(out, outsz, "%s:Model01:%s", person, s_name);
        break;
    case 4:
        _snprintf(out, outsz, "%sBody:%s", person, parent_name);
        break;
    case 5:
        _snprintf(out, outsz, "%sBody:%s", person, s_name);
        break;
    }
    out[outsz - 1] = 0;
    return out[0] != 0;
}

static int resolve_addon_chain_live_body_parent_rotation_step(
    physx_chain_t *chain,
    physx_target_t *parent,
    float out[3],
    DWORD now)
{
    enum { RUNTIME_CANDIDATES = 24 };
    enum { RUNTIME_CACHE_SLOTS = 12 };
    static const int candidate_order[RUNTIME_CANDIDATES] = {
        6, 7, 8, 9, 10, 11,
        0, 1, 2, 3, 4, 5,
        12, 13, 14, 15, 16, 17,
        18, 19, 20, 21, 22, 23
    };
    static void *cached_base[RUNTIME_CACHE_SLOTS];
    static char cached_runtime[RUNTIME_CACHE_SLOTS][256];
    static DWORD next_resolve_tick;
    static int next_candidate_index;
    static int cache_replace_index;
    void *direct_bases[4];
    const char *direct_labels[4];
    int i;
    int best_offset = -1;
    float best_step[3] = { 0.0f, 0.0f, 0.0f };
    float best_len = 0.0f;
    char best_runtime[256] = "";
    char best_source[64] = "";
    void *best_base = NULL;
    if (!chain || !parent || !out || parent->addon_simulated_target ||
        !addon_parent_name_can_use_body_drive(parent->name)) {
        return 0;
    }

    direct_bases[0] = parent->raw_object;
    direct_bases[1] = parent->s_raw_object;
    direct_bases[2] = parent->object;
    direct_bases[3] = parent->s_object;
    direct_labels[0] = "addon-target-raw";
    direct_labels[1] = "addon-target-s-raw";
    direct_labels[2] = "addon-target-object";
    direct_labels[3] = "addon-target-s-object";
    for (i = 0; i < 4; i++) {
        float step[3] = { 0.0f, 0.0f, 0.0f };
        int offset = -1;
        float len;
        if (!direct_bases[i]) continue;
        if (!addon_body_parent_scan_base(chain, parent, direct_bases[i],
                                         parent->name, direct_labels[i],
                                         step, &offset, 0, now)) {
            continue;
        }
        len = addon_vec3_len_exact(step);
        if (len > best_len) {
            best_len = len;
            best_offset = offset;
            best_step[0] = step[0];
            best_step[1] = step[1];
            best_step[2] = step[2];
            lstrcpynA(best_runtime, parent->name, sizeof(best_runtime));
            lstrcpynA(best_source, direct_labels[i], sizeof(best_source));
            best_base = direct_bases[i];
        }
    }

    for (i = 0; i < RUNTIME_CACHE_SLOTS; i++) {
        float step[3] = { 0.0f, 0.0f, 0.0f };
        int offset = -1;
        float len;
        if (!cached_base[i]) continue;
        if (!addon_body_parent_scan_base(chain, parent, cached_base[i],
                                         cached_runtime[i],
                                         "body-runtime-cached",
                                         step, &offset, 0, now)) {
            continue;
        }
        len = addon_vec3_len_exact(step);
        if (len > best_len) {
            best_len = len;
            best_offset = offset;
            best_step[0] = step[0];
            best_step[1] = step[1];
            best_step[2] = step[2];
            lstrcpynA(best_runtime, cached_runtime[i], sizeof(best_runtime));
            lstrcpynA(best_source, "body-runtime-cached",
                      sizeof(best_source));
            best_base = cached_base[i];
        }
    }

    if (!next_resolve_tick || now - next_resolve_tick >= 180u) {
        char runtime[256];
        void *raw = NULL;
        int candidate = candidate_order[next_candidate_index];
        next_candidate_index++;
        if (next_candidate_index >= RUNTIME_CANDIDATES) next_candidate_index = 0;
        next_resolve_tick = now;
        if (addon_build_body_parent_runtime_name(candidate, parent->name,
                                                  runtime, sizeof(runtime))) {
            raw = resolve_axis_map_raw(runtime);
            if (raw && ptr_readable(raw, sizeof(float))) {
                int slot = -1;
                for (i = 0; i < RUNTIME_CACHE_SLOTS; i++) {
                    if (cached_base[i] == raw) {
                        slot = i;
                        break;
                    }
                    if (!cached_base[i] && slot < 0) slot = i;
                }
                if (slot < 0) {
                    slot = cache_replace_index;
                    cache_replace_index++;
                    if (cache_replace_index >= RUNTIME_CACHE_SLOTS) {
                        cache_replace_index = 0;
                    }
                }
                cached_base[slot] = raw;
                lstrcpynA(cached_runtime[slot], runtime,
                          sizeof(cached_runtime[slot]));
                addon_body_parent_scan_base(chain, parent, raw, runtime,
                                            "body-runtime-resolved",
                                            best_step, &best_offset, 0, now);
            }
        }
    }

    if (best_len >= 0.020f) {
        out[0] = best_step[0];
        out[1] = best_step[1];
        out[2] = best_step[2];
        addon_chain_note_body_root_person(chain, best_runtime, now,
                                          "live-parent-runtime");
        if (!chain->addon_parent_rotation_auto_log_tick ||
            now - chain->addon_parent_rotation_auto_log_tick >= 500u) {
            chain->addon_parent_rotation_auto_log_tick = now;
            log_line("addon-chain live body parent selected chain=\"%s\" parent=\"%s\" source=%s runtime=\"%s\" base=%p offset=0x%03x step=(%.5f,%.5f,%.5f) step_len=%.5f note=\"live body parent beat the add-on-local fallback and is feeding the root anchor\"",
                     chain->name,
                     parent->name,
                     best_source,
                     best_runtime,
                     best_base,
                     best_offset,
                     out[0], out[1], out[2],
                     best_len);
        }
        return 1;
    }
    return 0;
}

static int addon_parent_rotation_step_from_vector(physx_chain_t *chain,
                                                  physx_target_t *parent,
                                                  void *base,
                                                  int offset,
                                                  const char *source,
                                                  float out[3],
                                                  DWORD now)
{
    float *v;
    float step_len;
    if (!chain || !parent || !base || offset < 0 || !out) return 0;
    if (!ptr_readable((BYTE*)base + offset, sizeof(float) * 3)) return 0;
    v = (float*)((BYTE*)base + offset);
    if (!physx_vec3_sane_limit(v, 720.0f)) return 0;

    if (!chain->addon_parent_rotation_initialized ||
        chain->addon_parent_rotation_base != base ||
        chain->addon_parent_rotation_offset != offset) {
        chain->addon_parent_rotation_initialized = 1;
        chain->addon_parent_rotation_base = base;
        chain->addon_parent_rotation_offset = offset;
        chain->addon_parent_rotation_prev[0] = v[0];
        chain->addon_parent_rotation_prev[1] = v[1];
        chain->addon_parent_rotation_prev[2] = v[2];
        if (!chain->addon_parent_rotation_logged) {
            chain->addon_parent_rotation_logged = 1;
            log_line("addon-chain parent rotation baseline chain=\"%s\" parent=\"%s\" source=%s base=%p offset=0x%03x rotation=(%.5f,%.5f,%.5f) note=\"root anchor drive now watches parent rotation, so Pose Editor head_joint02 rotations can excite the custom PhysX chain\"",
                     chain->name,
                     parent->name,
                     source ? source : "unknown",
                     base,
                     offset,
                     v[0], v[1], v[2]);
        }
        return 0;
    }

    {
        float previous[3];
        int basis_like = 0;
        previous[0] = chain->addon_parent_rotation_prev[0];
        previous[1] = chain->addon_parent_rotation_prev[1];
        previous[2] = chain->addon_parent_rotation_prev[2];
        addon_parent_vector_step_to_degrees(v, previous, out, &basis_like);
        (void)basis_like;
    }
    chain->addon_parent_rotation_prev[0] = v[0];
    chain->addon_parent_rotation_prev[1] = v[1];
    chain->addon_parent_rotation_prev[2] = v[2];
    step_len = physx_vec3_len(out);
    if (step_len < 0.0005f) return 0;
    if (defaults_cfg.debug &&
        (!chain->addon_parent_rotation_log_tick ||
         now - chain->addon_parent_rotation_log_tick >= 1000u)) {
        chain->addon_parent_rotation_log_tick = now;
        log_line("addon-chain parent rotation step chain=\"%s\" parent=\"%s\" source=%s base=%p offset=0x%03x step=(%.5f,%.5f,%.5f) step_len=%.5f note=\"this is the root-anchor impulse feeding physx_tail01 from head_joint02\"",
                 chain->name,
                 parent->name,
                 source ? source : "unknown",
                 base,
                 offset,
                 out[0], out[1], out[2],
                 step_len);
    }
    return 1;
}

static int resolve_addon_chain_parent_rotation_step(physx_sidecar_t *sc,
                                                    physx_chain_t *chain,
                                                    physx_target_t *parent,
                                                    float out[3],
                                                    DWORD now)
{
    if (out) {
        out[0] = 0.0f;
        out[1] = 0.0f;
        out[2] = 0.0f;
    }
    if (!chain || !parent || !out || !chain->addon_chain ||
        !parent->name[0] || parent->addon_simulated_target) {
        return 0;
    }

    if (resolve_addon_chain_effective_parent_rotation_step(
            sc, chain, parent, out, now)) {
        return 1;
    }

    if (parent->s_rotation_base && parent->s_rotation_offset >= 0) {
        if (addon_parent_rotation_step_from_vector(
                chain, parent, parent->s_rotation_base,
                parent->s_rotation_offset,
                parent->s_rotation_source ? parent->s_rotation_source : "s-rotation",
                out, now)) {
            return 1;
        }
    }

    if (!parent->s_rotation_base &&
        !chain->addon_parent_rotation_camera_relative_available &&
        !chain->addon_parent_rotation_auto_logged) {
        chain->addon_parent_rotation_auto_logged = 1;
        log_line("addon-chain parent rotation unresolved chain=\"%s\" parent=\"%s\" note=\"strict parent-local mode will not scan body/root fallback rows; define/resolve this parent bone rotation slot to drive the add-on chain\"",
                 chain->name,
                 parent->name);
    }
    return 0;
}

static int addon_chain_root_drive_camera_untrusted(physx_chain_t *chain,
                                                   DWORD now)
{
    DWORD quarantine_ms =
        (DWORD)physics_environment_cfg.body_chain_camera_quarantine_ms;
    DWORD camera_age_ms = captured_camera_change_tick ?
        (now - captured_camera_change_tick) : 0xffffffffu;
    DWORD quarantine_age_ms = 0xffffffffu;
    int camera_changed;
    int camera_recent;
    int quarantine_active;
    int untrusted;
    if (!chain || !chain->addon_chain) return 0;
    camera_changed =
        chain->addon_root_drive_camera_seen_version != captured_camera_version;
    camera_recent =
        captured_camera_inverse_valid && camera_age_ms < quarantine_ms;
    if (camera_changed || camera_recent) {
        chain->addon_root_drive_camera_quarantine_tick = now;
    }
    if (chain->addon_root_drive_camera_quarantine_tick) {
        quarantine_age_ms =
            now - chain->addon_root_drive_camera_quarantine_tick;
    }
    quarantine_active =
        chain->addon_root_drive_camera_quarantine_tick &&
        quarantine_ms > 0 &&
        quarantine_age_ms < quarantine_ms;
    untrusted = camera_changed || camera_recent || quarantine_active;
    if (untrusted) {
        chain->addon_root_drive_camera_last_untrusted_tick = now;
        if (defaults_cfg.debug &&
            (!chain->addon_root_drive_camera_log_tick ||
             now - chain->addon_root_drive_camera_log_tick >= 1000u)) {
            chain->addon_root_drive_camera_log_tick = now;
            log_line("addon-chain root-drive camera-rebased chain=\"%s\" camera_version=%ld camera_age_ms=%lu quarantine_age_ms=%lu note=\"discarding camera-tainted add-on parent/root impulses only; custom chain spring/gravity simulation continues\"",
                     chain->name,
                     captured_camera_version,
                     (unsigned long)camera_age_ms,
                     (unsigned long)quarantine_age_ms);
        }
    }
    chain->addon_root_drive_camera_seen_version = captured_camera_version;
    return untrusted;
}

static int addon_body_root_pointer_for_person(const char *person,
                                              void **raw_out,
                                              float **root_out)
{
    char name[256];
    void *raw;
    float *root;
    if (raw_out) *raw_out = NULL;
    if (root_out) *root_out = NULL;
    if (!person || !person[0]) return 0;
    make_body_runtime_name(name, sizeof(name), person, "root");
    raw = resolve_axis_map_raw(name);
    if (!raw ||
        body_chain_physics_cfg.root_offset < 0 ||
        !ptr_readable((BYTE*)raw + body_chain_physics_cfg.root_offset,
                      sizeof(float) * 3)) {
        return 0;
    }
    root = (float*)((BYTE*)raw + body_chain_physics_cfg.root_offset);
    if (!physx_vec3_sane_limit(root, 64.0f)) return 0;
    if (raw_out) *raw_out = raw;
    if (root_out) *root_out = root;
    return 1;
}

static int addon_owner_person_live_for_binding(const char *person)
{
    static DWORD cache_tick;
    static int cache_valid[4];
    static int cache_live[4];
    void *root_raw = NULL;
    float *root = NULL;
    float epsilon = physics_environment_cfg.gravity_probe_motion_epsilon;
    DWORD now = GetTickCount();
    int person_index = addon_person_prefix_to_index(person);
    if (!person || !person[0]) return 0;
    if (!cache_tick || now - cache_tick >= 16u) {
        memset(cache_valid, 0, sizeof(cache_valid));
        cache_tick = now;
    }
    if (person_index >= 0 && person_index < 4 &&
        cache_valid[person_index]) {
        return cache_live[person_index];
    }
    if (epsilon < 0.0001f) epsilon = 0.0001f;
    if (!addon_body_root_pointer_for_person(person, &root_raw, &root) ||
        !root || !root_raw) {
        if (person_index >= 0 && person_index < 4) {
            cache_valid[person_index] = 1;
            cache_live[person_index] = 0;
        }
        return 0;
    }
    if (person_index >= 0 && person_index < 4) {
        cache_valid[person_index] = 1;
        cache_live[person_index] = physx_vec3_len(root) > epsilon;
        return cache_live[person_index];
    }
    return physx_vec3_len(root) > epsilon;
}

static int addon_sample_body_root_step_for_person(
    physx_chain_t *chain,
    const char *person,
    int root_drive_untrusted,
    float out[3],
    int *found_out,
    DWORD now)
{
    void *root_raw = NULL;
    float *root = NULL;
    float raw_step[3];
    float step_len;
    float step_limit;
    int axis;
    if (found_out) *found_out = 0;
    if (out) out[0] = out[1] = out[2] = 0.0f;
    if (!chain || !person || !person[0] || !out) return 0;
    if (!addon_body_root_pointer_for_person(person, &root_raw, &root)) {
        return 0;
    }
    if (found_out) *found_out = 1;
    if (!chain->addon_body_root_initialized ||
        chain->addon_body_root_raw != root_raw ||
        _stricmp(chain->addon_body_root_person, person) != 0) {
        lstrcpynA(chain->addon_body_root_person, person,
                  sizeof(chain->addon_body_root_person));
        chain->addon_body_root_raw = root_raw;
        chain->addon_body_root_initialized = 1;
        chain->addon_body_root_prev[0] = root[0];
        chain->addon_body_root_prev[1] = root[1];
        chain->addon_body_root_prev[2] = root[2];
        chain->addon_body_root_log_tick = now;
        log_line("addon-chain body root translation baseline chain=\"%s\" person=\"%s\" root_raw=%p root_offset=0x%03x root=(%.5f,%.5f,%.5f) note=\"whole-body movement can now feed the custom add-on chain through the same camera gate as penis/testicle root drive\"",
                 chain->name,
                 chain->addon_body_root_person,
                 root_raw,
                 body_chain_physics_cfg.root_offset,
                 root[0], root[1], root[2]);
        return 0;
    }

    for (axis = 0; axis < 3; axis++) {
        raw_step[axis] = root[axis] - chain->addon_body_root_prev[axis];
        chain->addon_body_root_prev[axis] = root[axis];
    }
    if (root_drive_untrusted) {
        return 0;
    }
    step_len = physx_vec3_len(raw_step);
    if (step_len < 0.0007f) {
        return 0;
    }
    step_limit = chain->max_offset > 0.001f ?
                 chain->max_offset * 0.65f : 0.08f;
    for (axis = 0; axis < 3; axis++) {
        out[axis] = physx_clampf(raw_step[axis], -step_limit, step_limit);
    }
    if (defaults_cfg.debug &&
        (!chain->addon_body_root_log_tick ||
         now - chain->addon_body_root_log_tick >= 500u)) {
        chain->addon_body_root_log_tick = now;
        log_line("addon-chain body root translation step chain=\"%s\" person=\"%s\" root_step=(%.5f,%.5f,%.5f) clamped=(%.5f,%.5f,%.5f) step_len=%.5f note=\"trusted whole-body translation is feeding physx_tail01 inertia\"",
                 chain->name,
                 chain->addon_body_root_person,
                 raw_step[0], raw_step[1], raw_step[2],
                 out[0], out[1], out[2],
                 step_len);
    }
    return 1;
}

static int resolve_addon_chain_body_root_step(physx_sidecar_t *sc,
                                              physx_chain_t *chain,
                                              int root_drive_untrusted,
                                              float out[3],
                                              DWORD now)
{
    int found = 0;
    int i;
    if (out) out[0] = out[1] = out[2] = 0.0f;
    if (!chain || !out) return 0;

    if (sc && sc->addon_owner_person[0]) {
        if (addon_sample_body_root_step_for_person(
                chain, sc->addon_owner_person, root_drive_untrusted,
                out, &found, now)) {
            return 1;
        }
        if (found) return 0;
    }
    if (chain->addon_body_root_person[0]) {
        if (addon_sample_body_root_step_for_person(
                chain, chain->addon_body_root_person, root_drive_untrusted,
                out, &found, now)) {
            return 1;
        }
        if (found) return 0;
    }
    for (i = 0; i < 4; i++) {
        if (!body_chain_physics_cfg.enabled_person[i] &&
            !testicle_physics_cfg.enabled_person[i]) {
            continue;
        }
        if (addon_sample_body_root_step_for_person(
                chain, body_chain_person_name(i), root_drive_untrusted,
                out, &found, now)) {
            return 1;
        }
        if (found) return 0;
    }
    for (i = 0; i < 4; i++) {
        if (!poseedit_scene_person_visible(i)) continue;
        if (addon_sample_body_root_step_for_person(
                chain, body_chain_person_name(i), root_drive_untrusted,
                out, &found, now)) {
            return 1;
        }
        if (found) return 0;
    }
    if (!chain->addon_body_root_miss_log_tick ||
        now - chain->addon_body_root_miss_log_tick >= 2000u) {
        chain->addon_body_root_miss_log_tick = now;
        log_line("addon-chain body root translation unresolved chain=\"%s\" owner=\"%s\" inferred=\"%s\" note=\"could not resolve a readable PersonXX root row yet; rotation drive still works while translation waits\"",
                 chain->name,
                 sc ? sc->addon_owner_person : "",
                 chain->addon_body_root_person);
    }
    return 0;
}

static void addon_rotation_drive_add_impulse(float impulse[3],
                                             const float source_step[3],
                                             int source_axis,
                                             int tail_axis,
                                             float scale,
                                             float rot_to_local)
{
    if (!impulse || !source_step) return;
    if (source_axis < 0 || source_axis > 2) return;
    if (tail_axis < 0 || tail_axis > 2) return;
    if (!sane_probe_float(scale)) return;
    scale = physx_clampf(scale, -10.0f, 10.0f);
    impulse[tail_axis] += source_step[source_axis] * rot_to_local * scale;
}

static void diagnose_chain_attachment(physx_chain_t *chain)
{
    void *raw = NULL;
    void *obj;
    char matched[384];
    const char *name;
    if (!chain) return;
    name = chain->attach_name[0] ? chain->attach_name : chain->anchor_name;
    if (!name || !name[0] || chain->attach_logged) return;
    obj = resolve_runtime_exact_target(name, &raw, matched, sizeof(matched));
    if (raw || obj) {
        chain->attach_logged = 1;
        log_line("chain attach diagnostic chain=\"%s\" attach=\"%s\" runtime=\"%s\" raw=%p object=%p note=\"attach is resolved for diagnostics only; world-to-local drive is disabled until conversion is verified\"",
                 chain->name, name, matched, raw, obj);
    }
}

static int known_initial_translation(const char *name, float out[3])
{
    (void)name;
    (void)out;
    return 0;
}

static int known_rotation_limits(const char *name, float min_out[3], float max_out[3])
{
    (void)name;
    (void)min_out;
    (void)max_out;
    return 0;
}

static int known_rotation_transform(const char *name, float translation_out[3], float rotation_out[3])
{
    (void)name;
    (void)translation_out;
    (void)rotation_out;
    return 0;
}

static int find_vector3_offset(BYTE *base, const float expected[3], float epsilon,
                               int *best_off_out, float best_out[3], float *best_err_out)
{
    int off;
    int exact_off = -1;
    float best_err = 999999.0f;
    int best_off = -1;
    float best_v[3] = { 0.0f, 0.0f, 0.0f };
    if (!base || !expected || !ptr_readable(base, 0x1000)) return -1;
    for (off = 0; off <= 0x1000 - (int)(sizeof(float) * 3); off += 4) {
        float *v = (float*)(base + off);
        float err = physx_absf(v[0] - expected[0]) +
                    physx_absf(v[1] - expected[1]) +
                    physx_absf(v[2] - expected[2]);
        if (err < best_err) {
            best_err = err;
            best_off = off;
            best_v[0] = v[0];
            best_v[1] = v[1];
            best_v[2] = v[2];
        }
        if (physx_absf(v[0] - expected[0]) <= epsilon &&
            physx_absf(v[1] - expected[1]) <= epsilon &&
            physx_absf(v[2] - expected[2]) <= epsilon) {
            exact_off = off;
            break;
        }
    }
    if (best_off_out) *best_off_out = best_off;
    if (best_out) {
        best_out[0] = best_v[0];
        best_out[1] = best_v[1];
        best_out[2] = best_v[2];
    }
    if (best_err_out) *best_err_out = best_err;
    return exact_off;
}

static void probe_known_rotation_transform_layout(const char *name)
{
    BYTE *bases[2];
    const char *labels[2];
    float expected_t[3];
    float expected_r[3];
    char matched[384];
    void *raw = NULL;
    void *obj;
    int bi;
    if (!known_rotation_transform(name, expected_t, expected_r)) return;
    obj = resolve_runtime_exact_target(name, &raw, matched, sizeof(matched));
    log_line("known rotation transform resolve name=\"%s\" runtime=\"%s\" raw=%p object=%p nil=%d expected_t=(%.5f,%.5f,%.5f) expected_r=(%.5f,%.5f,%.5f)",
             name, matched, raw, obj, is_nil_engine_object(raw, obj),
             expected_t[0], expected_t[1], expected_t[2],
             expected_r[0], expected_r[1], expected_r[2]);
    if (!obj || is_nil_engine_object(raw, obj)) return;
    bases[0] = (BYTE*)obj;
    bases[1] = (BYTE*)raw;
    labels[0] = "object";
    labels[1] = "raw";
    for (bi = 0; bi < 2; bi++) {
        int t_best = -1;
        int r_best = -1;
        int t_off;
        int r_off;
        float t_near[3];
        float r_near[3];
        float t_err = 0.0f;
        float r_err = 0.0f;
        BYTE *base = bases[bi];
        if (!base || !ptr_readable(base, 0x1000)) continue;
        t_off = find_vector3_offset(base, expected_t, 0.0002f, &t_best, t_near, &t_err);
        r_off = find_vector3_offset(base, expected_r, 0.0002f, &r_best, r_near, &r_err);
        log_line("known rotation transform probe name=\"%s\" source=%s base=%p t_off=0x%03x r_off=0x%03x t_best=0x%03x t_err=%.5f t_near=(%.5f,%.5f,%.5f) r_best=0x%03x r_err=%.5f r_near=(%.5f,%.5f,%.5f)",
                 name, labels[bi], base,
                 t_off, r_off,
                 t_best, t_err, t_near[0], t_near[1], t_near[2],
                 r_best, r_err, r_near[0], r_near[1], r_near[2]);
        if (t_off >= 0 && r_off >= 0) {
            log_line("known rotation transform layout exact name=\"%s\" source=%s base=%p translation_offset=0x%03x rotation_offset=0x%03x delta=0x%03x",
                     name, labels[bi], base, t_off, r_off, r_off - t_off);
        }
    }
}

static int probe_s_transform_translation_layout(physx_target_t *target)
{
    BYTE *bases[2];
    const char *labels[2];
    float expected[3];
    int bi, off;
    if (!target || !target->s_object || !known_initial_translation(target->name, expected)) return -1;
    bases[0] = (BYTE*)target->s_object;
    bases[1] = (BYTE*)target->s_raw_object;
    labels[0] = "object";
    labels[1] = "raw";
    for (bi = 0; bi < 2; bi++) {
        BYTE *base = bases[bi];
        float best_err = 999999.0f;
        int best_off = -1;
        float best_v[3] = { 0.0f, 0.0f, 0.0f };
        if (!base || !ptr_readable(base, 0x1000)) continue;
        for (off = 0; off <= 0x1000 - (int)(sizeof(float) * 3); off += 4) {
            float *v = (float*)(base + off);
            float err = physx_absf(v[0] - expected[0]) +
                        physx_absf(v[1] - expected[1]) +
                        physx_absf(v[2] - expected[2]);
            if (err < best_err) {
                best_err = err;
                best_off = off;
                best_v[0] = v[0];
                best_v[1] = v[1];
                best_v[2] = v[2];
            }
            if (physx_absf(v[0] - expected[0]) < 0.0002f &&
                physx_absf(v[1] - expected[1]) < 0.0002f &&
                physx_absf(v[2] - expected[2]) < 0.0002f) {
                target->s_translation_offset = off;
                target->s_translation_base = base;
                target->s_translation_source = labels[bi];
                log_line("target s-translation probe exact target=\"%s\" source=%s base=%p offset=0x%03x",
                         target->name, labels[bi], base, off);
                return off;
            }
        }
        if (best_off >= 0) {
            log_line("target s-translation probe nearest target=\"%s\" source=%s base=%p offset=0x%03x err=%.5f nearest=(%.5f,%.5f,%.5f) expected=(%.5f,%.5f,%.5f)",
                     target->name, labels[bi], base, best_off, best_err,
                     best_v[0], best_v[1], best_v[2],
                     expected[0], expected[1], expected[2]);
        }
    }
    return -1;
}

static void probe_s_transform_rotation_layout(physx_target_t *target)
{
    BYTE *bases[2];
    const char *labels[2];
    float min_expected[3];
    float max_expected[3];
    int bi, off;
    if (!target || !target->s_object || !known_rotation_limits(target->name, min_expected, max_expected)) return;
    bases[0] = (BYTE*)target->s_object;
    bases[1] = (BYTE*)target->s_raw_object;
    labels[0] = "object";
    labels[1] = "raw";
    target->s_rotation_min_offset = -1;
    target->s_rotation_max_offset = -1;
    target->s_rotation_offset = -1;
    for (bi = 0; bi < 2; bi++) {
        BYTE *base = bases[bi];
        if (!base || !ptr_readable(base, 0x1000)) continue;
        for (off = 0; off <= 0x1000 - (int)(sizeof(float) * 3); off += 4) {
            float *v = (float*)(base + off);
            if (physx_absf(v[0] - min_expected[0]) < 0.0002f &&
                physx_absf(v[1] - min_expected[1]) < 0.0002f &&
                physx_absf(v[2] - min_expected[2]) < 0.0002f) {
                target->s_rotation_min_offset = off;
                log_line("target s-rotation-min probe exact target=\"%s\" source=%s base=%p offset=0x%03x",
                         target->name, labels[bi], base, off);
            }
            if (physx_absf(v[0] - max_expected[0]) < 0.0002f &&
                physx_absf(v[1] - max_expected[1]) < 0.0002f &&
                physx_absf(v[2] - max_expected[2]) < 0.0002f) {
                target->s_rotation_max_offset = off;
                log_line("target s-rotation-max probe exact target=\"%s\" source=%s base=%p offset=0x%03x",
                         target->name, labels[bi], base, off);
            }
        }
    }
    if (target->s_translation_base && target->s_translation_offset >= 0 &&
        ptr_readable((BYTE*)target->s_translation_base + target->s_translation_offset, 0x40)) {
        float *near0 = (float*)((BYTE*)target->s_translation_base + target->s_translation_offset + 0x0c);
        float *near1 = (float*)((BYTE*)target->s_translation_base + target->s_translation_offset + 0x10);
        float *near2 = (float*)((BYTE*)target->s_translation_base + target->s_translation_offset + 0x14);
        log_line("target s-rotation candidates target=\"%s\" source=%s base=%p t_offset=0x%03x plus0c=(%.5f,%.5f,%.5f) plus10=(%.5f,%.5f,%.5f) plus14=(%.5f,%.5f,%.5f) min_off=0x%03x max_off=0x%03x",
                 target->name,
                 target->s_translation_source ? target->s_translation_source : "unknown",
                 target->s_translation_base, target->s_translation_offset,
                 near0[0], near0[1], near0[2],
                 near1[0], near1[1], near1[2],
                 near2[0], near2[1], near2[2],
                 target->s_rotation_min_offset, target->s_rotation_max_offset);
    }
    if (target->s_raw_object && target->s_translation_offset == 0x07c &&
        ptr_readable((BYTE*)target->s_raw_object + 0x06c, sizeof(float) * 3)) {
        float *r = (float*)((BYTE*)target->s_raw_object + 0x06c);
        target->s_rotation_base = target->s_raw_object;
        target->s_rotation_source = "raw";
        target->s_rotation_offset = 0x06c;
        log_line("target s-rotation layout assumed target=\"%s\" source=raw base=%p offset=0x06c current=(%.5f,%.5f,%.5f) reason=\"known STransform rotation probe\"",
                 target->name, target->s_rotation_base, r[0], r[1], r[2]);
    }
}

static void *search_one_named_node_tree(void *root, const char *root_name, const char *name)
{
    const void *ref;
    void *found;
    if (!engine_SearchTree || !name || !name[0]) return NULL;
    ref = stringref_from_cstr_a(name);
    if (!ref) return NULL;
    found = engine_SearchTree(root, ref, NULL);
    if (found && !is_nil_engine_object(NULL, found)) {
        log_line("tree search found root_name=\"%s\" root=%p target=\"%s\" object=%p",
                 root_name, root, name, found);
        return found;
    }
    return NULL;
}

static void *search_named_node_trees(const char *name)
{
    int i;
    char s_name[192];
    char suffix_name[192];
    char suffix_s_name[192];
    char mesh_name[192];
    char mesh_s_name[192];
    char colon_name[256];
    char colon_s_name[256];
    char colon_mesh_name[256];
    char colon_mesh_s_name[256];
    const char *suffix_part = NULL;
    if (!engine_SearchTree || !name || !name[0]) return NULL;
    _snprintf(s_name, sizeof(s_name), "S%s", name);
    _snprintf(mesh_name, sizeof(mesh_name), "%s_mesh", name);
    _snprintf(mesh_s_name, sizeof(mesh_s_name), "S%s_mesh", name);
    suffix_name[0] = 0;
    suffix_s_name[0] = 0;
    if (contains_i(name, "_Part")) {
        suffix_part = strstr(name, "_Part");
        if (suffix_part && suffix_part[0] == '_') suffix_part++;
        if (suffix_part && suffix_part[0]) {
            _snprintf(suffix_name, sizeof(suffix_name), "%s", suffix_part);
            _snprintf(suffix_s_name, sizeof(suffix_s_name), "S%s", suffix_part);
        }
    }
    for (i = 0; i < named_node_count; i++) {
        void *root = named_nodes[i].object;
        const char *root_name = named_nodes[i].name;
        void *found;
        if (!root || is_nil_engine_object(NULL, root)) continue;
        if (defaults_cfg.debug && tree_candidate_log_budget > 0) {
            tree_candidate_log_budget--;
            log_line("tree search candidates root_name=\"%s\" target=\"%s\" raw=\"%s\" s=\"%s\" mesh=\"%s\" mesh_s=\"%s\" suffix=\"%s\" suffix_s=\"%s\"",
                     root_name, name, name, s_name, mesh_name, mesh_s_name, suffix_name, suffix_s_name);
        }
        found = search_one_named_node_tree(root, root_name, name);
        if (found) return found;
        found = search_one_named_node_tree(root, root_name, s_name);
        if (found) return found;
        found = search_one_named_node_tree(root, root_name, mesh_name);
        if (found) return found;
        found = search_one_named_node_tree(root, root_name, mesh_s_name);
        if (found) return found;
        if (suffix_name[0]) {
            found = search_one_named_node_tree(root, root_name, suffix_name);
            if (found) return found;
            found = search_one_named_node_tree(root, root_name, suffix_s_name);
            if (found) return found;
            _snprintf(colon_name, sizeof(colon_name), "%s:%s", root_name, suffix_name);
            _snprintf(colon_s_name, sizeof(colon_s_name), "%s:S%s", root_name, suffix_name);
            _snprintf(colon_mesh_name, sizeof(colon_mesh_name), "%s:%s_mesh", root_name, name);
            _snprintf(colon_mesh_s_name, sizeof(colon_mesh_s_name), "%s:S%s_mesh", root_name, name);
            found = search_one_named_node_tree(root, root_name, colon_name);
            if (found) return found;
            found = search_one_named_node_tree(root, root_name, colon_s_name);
            if (found) return found;
            found = search_one_named_node_tree(root, root_name, colon_mesh_name);
            if (found) return found;
            found = search_one_named_node_tree(root, root_name, colon_mesh_s_name);
            if (found) return found;
        }
    }
    return NULL;
}

static void sidecar_scene_object_prefix(const char *sidecar, char *out, size_t outsz)
{
    const char *scene;
    const char *base_sidecar = NULL;
    char tmp[MAX_PATH * 4];
    size_t len;
    out[0] = 0;
    if (!sidecar || !outsz) return;
    scene = strstr(sidecar, "\\Scenes\\");
    if (!scene) scene = strstr(sidecar, "/Scenes/");
    if (!scene) {
        char addon_id[256];
        const char *name = strrchr(sidecar, '\\');
        const char *slash = strrchr(sidecar, '/');
        const char *dot;
        int i;
        if (!name || (slash && slash > name)) name = slash;
        name = name ? name + 1 : sidecar;
        lstrcpynA(addon_id, name, sizeof(addon_id));
        dot = strstr(addon_id, ".physx.ini");
        if (!dot) dot = strstr(addon_id, ".PHYSX.INI");
        if (dot) addon_id[dot - addon_id] = 0;
        for (i = 0; addon_id[0] && i < sidecar_count; i++) {
            const char *candidate = sidecars[i].path;
            const char *candidate_name;
            const char *candidate_slash;
            char candidate_id[256];
            char *candidate_dot;
            if (!candidate[0] || _stricmp(candidate, sidecar) == 0 ||
                contains_i(candidate, "\\ActiveMod\\") ||
                contains_i(candidate, "/ActiveMod/")) {
                continue;
            }
            if (!strstr(candidate, "\\Scenes\\") &&
                !strstr(candidate, "/Scenes/")) {
                continue;
            }
            candidate_name = strrchr(candidate, '\\');
            candidate_slash = strrchr(candidate, '/');
            if (!candidate_name ||
                (candidate_slash && candidate_slash > candidate_name)) {
                candidate_name = candidate_slash;
            }
            candidate_name = candidate_name ? candidate_name + 1 : candidate;
            lstrcpynA(candidate_id, candidate_name, sizeof(candidate_id));
            candidate_dot = candidate_id;
            while (*candidate_dot) {
                if (_strnicmp(candidate_dot, ".physx.ini", 10) == 0) {
                    *candidate_dot = 0;
                    break;
                }
                candidate_dot++;
            }
            if (_stricmp(candidate_id, addon_id) == 0) {
                base_sidecar = candidate;
                break;
            }
        }
        if (base_sidecar) {
            sidecar_scene_object_prefix(base_sidecar, out, outsz);
        }
        return;
    }
    scene += 8;
    lstrcpynA(tmp, scene, sizeof(tmp));
    len = strlen(tmp);
    if (len > 10 && _stricmp(tmp + len - 10, ".physx.ini") == 0) {
        tmp[len - 10] = 0;
    } else {
        char *dot = strrchr(tmp, '.');
        if (dot) *dot = 0;
    }
    {
        char *p;
        for (p = tmp; *p; p++) {
            if (*p == '\\') *p = '/';
        }
    }
    _snprintf(out, outsz, "%s.ma", tmp);
}

static void sidecar_scene_object_prefix_no_ext(const char *sidecar, char *out, size_t outsz)
{
    char with_ext[512];
    char *dot;
    if (!out || outsz == 0) return;
    out[0] = 0;
    sidecar_scene_object_prefix(sidecar, with_ext, sizeof(with_ext));
    if (!with_ext[0]) return;
    lstrcpynA(out, with_ext, outsz);
    dot = strrchr(out, '.');
    if (dot) *dot = 0;
}

static void sidecar_scene_basename_no_ext(const char *sidecar, char *out, size_t outsz)
{
    char no_ext[512];
    char *slash;
    if (!out || outsz == 0) return;
    out[0] = 0;
    sidecar_scene_object_prefix_no_ext(sidecar, no_ext, sizeof(no_ext));
    if (!no_ext[0]) {
        const char *name;
        char *dot;
        name = strrchr(sidecar ? sidecar : "", '\\');
        slash = strrchr(sidecar ? sidecar : "", '/');
        if (!name || (slash && slash > name)) name = slash;
        name = name ? name + 1 : sidecar;
        if (!name || !name[0]) return;
        lstrcpynA(out, name, outsz);
        dot = out;
        while (*dot) {
            if (_strnicmp(dot, ".physx.ini", 10) == 0) {
                *dot = 0;
                break;
            }
            dot++;
        }
        return;
    }
    slash = strrchr(no_ext, '/');
    if (!slash) slash = strrchr(no_ext, '\\');
    lstrcpynA(out, slash ? slash + 1 : no_ext, outsz);
}

static void *resolve_sidecar_named_target(physx_sidecar_t *sc,
                                          const char *target_name,
                                          void **raw_out,
                                          char *matched,
                                          size_t matched_sz)
{
    char prefix_ma[512];
    char prefix_no_ext[512];
    char basename[256];
    const char *prefixes[4];
    int pi;
    if (raw_out) *raw_out = NULL;
    if (matched && matched_sz) matched[0] = 0;
    if (!sc || !target_name || !target_name[0]) return NULL;
    sidecar_scene_object_prefix(sc->path, prefix_ma, sizeof(prefix_ma));
    sidecar_scene_object_prefix_no_ext(sc->path, prefix_no_ext, sizeof(prefix_no_ext));
    sidecar_scene_basename_no_ext(sc->path, basename, sizeof(basename));
    prefixes[0] = prefix_no_ext;
    prefixes[1] = prefix_ma;
    prefixes[2] = basename;
    prefixes[3] = "";
    for (pi = 0; pi < 4; pi++) {
        char full_name[768];
        char local_name[768];
        const char *candidates[2];
        int ci;
        void *raw = NULL;
        void *obj = NULL;
        if (!prefixes[pi] || !prefixes[pi][0]) continue;
        _snprintf(full_name, sizeof(full_name), "%s:%s", prefixes[pi], target_name);
        _snprintf(local_name, sizeof(local_name), "%s:local_%s", prefixes[pi], target_name);
        candidates[0] = full_name;
        candidates[1] = local_name;
        for (ci = 0; ci < 2; ci++) {
            raw = NULL;
            obj = resolve_find_obj(candidates[ci], &raw);
            if (obj && !is_nil_engine_object(raw, obj)) {
                if (raw_out) *raw_out = raw;
                if (matched && matched_sz) lstrcpynA(matched, candidates[ci], (int)matched_sz);
                return obj;
            }
            if (captured_script_engine) {
                raw = NULL;
                obj = resolve_script_engine_obj(candidates[ci], &raw);
                if (obj && !is_nil_engine_object(raw, obj)) {
                    if (raw_out) *raw_out = raw;
                    if (matched && matched_sz) lstrcpynA(matched, candidates[ci], (int)matched_sz);
                    return obj;
                }
            }
        }
    }
    return NULL;
}

static int sidecar_named_node_matches_addon_root(physx_sidecar_t *sc, const char *root_name)
{
    const char *basename;
    size_t root_len;
    size_t basename_len;
    if (!sc || !root_name || !root_name[0]) return 0;
    if (_strnicmp(root_name, "Person", 6) != 0) return 0;
    if (!sc->addon_id_cache[0]) {
        sidecar_scene_basename_no_ext(sc->path, sc->addon_id_cache,
                                      sizeof(sc->addon_id_cache));
    }
    basename = sc->addon_id_cache;
    if (!basename[0]) return 0;
    root_len = strlen(root_name);
    basename_len = strlen(basename);
    if (root_len < basename_len) return 0;
    return _stricmp(root_name + root_len - basename_len, basename) == 0;
}

/* The sidecar table holds 32 add-ons, each with one live instance per person. */
#define ADDON_ACTIVE_SLOT_COUNT (32 * 4)

typedef struct addon_active_slot_t {
    char owner[16];
    char addon_id[256];
    physx_sidecar_t *sidecar;
    void *root_object;
    DWORD root_tick;
} addon_active_slot_t;

static addon_active_slot_t addon_active_slots[ADDON_ACTIVE_SLOT_COUNT];

#define ADDON_EQUIPMENT_DEFINITION_COUNT 512
#define ADDON_EQUIPMENT_ZONE_COUNT 4
#define ADDON_EQUIPMENT_SLOT_COUNT (64 * 4)

typedef struct addon_equipment_definition_t {
    char addon_id[256];
    char zones[ADDON_EQUIPMENT_ZONE_COUNT][64];
    int zone_count;
} addon_equipment_definition_t;

typedef struct addon_equipment_slot_t {
    char owner[16];
    char zone[64];
    char addon_id[256];
    void *root_object;
    DWORD root_tick;
} addon_equipment_slot_t;

/* TK17 replacement ownership is keyed by the DressDescription PrimaryZone,
   not by a generic "cloth" or "hair" code path.  Every add-on sidecar uses
   this same table; the zone merely describes which engine equipment slot can
   replace which other add-on. */
static addon_equipment_definition_t
    addon_equipment_definitions[ADDON_EQUIPMENT_DEFINITION_COUNT];
static addon_equipment_slot_t
    addon_equipment_slots[ADDON_EQUIPMENT_SLOT_COUNT];

#define ADDON_SELECTION_HINT_COUNT 32
#define ADDON_SELECTION_HINT_MAX_AGE_MS 15000u

typedef struct addon_selection_hint_t {
    char addon_id[256];
    char activemod_folder[MAX_PATH * 4];
    int use_activemod;
    DWORD tick;
} addon_selection_hint_t;

static addon_selection_hint_t
    addon_selection_hints[ADDON_SELECTION_HINT_COUNT];

static int sidecar_path_is_activemod_a(const char *path);
static void addon_equipment_scan_dress_script(const char *path);
static int addon_equipment_sidecar_current(physx_sidecar_t *sc,
                                           const char *owner);
static int addon_equipment_sidecar_root_allowed(physx_sidecar_t *sc,
                                                const char *owner,
                                                void *root_object);
static void addon_chain_reset_runtime_state(physx_chain_t *chain);
static void addon_selection_note_base_scene(const char *addon_id, DWORD now);
static void physx_note_activemod_file_a(const char *file_path);
static void physx_note_activemod_file_w(const WCHAR *file_path);

static int sidecar_addon_identifier(physx_sidecar_t *sc,
                                    char *out,
                                    size_t outsz)
{
    /*
       Several concurrently equipped add-ons can live under Scenes\Shared\Cloth.
       Their scene basename, which also suffix-matches the PersonXX live root,
       is the actual TK17 add-on identifier.
    */
    if (!out || outsz == 0) return 0;
    out[0] = 0;
    if (!sc || !sc->path[0]) return 0;
    if (!sc->addon_id_cache[0]) {
        sidecar_scene_basename_no_ext(sc->path, sc->addon_id_cache,
                                      sizeof(sc->addon_id_cache));
    }
    lstrcpynA(out, sc->addon_id_cache, (int)outsz);
    return out[0] != 0;
}

static addon_equipment_definition_t *addon_equipment_definition_find(
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

static void addon_equipment_register_zone(const char *addon_id,
                                          const char *zone)
{
    addon_equipment_definition_t *definition;
    int i;
    if (!addon_id || !addon_id[0] || !zone || !zone[0]) return;
    definition = addon_equipment_definition_find(addon_id, 1);
    if (!definition) return;
    for (i = 0; i < definition->zone_count; i++) {
        if (_stricmp(definition->zones[i], zone) == 0) return;
    }
    if (definition->zone_count >= ADDON_EQUIPMENT_ZONE_COUNT) return;
    lstrcpynA(definition->zones[definition->zone_count], zone,
              sizeof(definition->zones[definition->zone_count]));
    definition->zone_count++;
}

static addon_equipment_definition_t *addon_equipment_definition_for_id(
    const char *addon_id)
{
    addon_equipment_definition_t *definition;
    if (!addon_id || !addon_id[0]) return NULL;
    definition = addon_equipment_definition_find(addon_id, 0);
    if (definition && definition->zone_count > 0) return definition;

    /* The normal source of truth is the DressDescription scan.  These common
       names are a startup-order fallback for a live hair root observed before
       that one-time scan reaches its dress script. */
    if (_strnicmp(addon_id, "NcHair", 6) == 0 ||
        _strnicmp(addon_id, "R9Hair", 6) == 0 ||
        _strnicmp(addon_id, "DriverHair", 10) == 0) {
        addon_equipment_register_zone(addon_id, "DZ_Hair");
    }
    definition = addon_equipment_definition_find(addon_id, 0);
    return definition && definition->zone_count > 0 ? definition : NULL;
}

static addon_equipment_definition_t *addon_equipment_definition_for_sidecar(
    physx_sidecar_t *sc)
{
    char addon_id[256];
    addon_equipment_definition_t *definition;
    if (sc && sc->addon_equipment_definition_cache) {
        return (addon_equipment_definition_t *)
            sc->addon_equipment_definition_cache;
    }
    if (!sidecar_addon_identifier(sc, addon_id, sizeof(addon_id))) return NULL;
    definition = addon_equipment_definition_for_id(addon_id);
    if (!definition && sc &&
        (contains_i(sc->path, "\\Scenes\\Shared\\Hair\\") ||
         contains_i(sc->path, "/Scenes/Shared/Hair/"))) {
        addon_equipment_register_zone(addon_id, "DZ_Hair");
        definition = addon_equipment_definition_for_id(addon_id);
    }
    if (sc && definition) {
        sc->addon_equipment_definition_cache = definition;
    }
    return definition;
}

static int addon_equipment_definitions_overlap(
    const addon_equipment_definition_t *a,
    const addon_equipment_definition_t *b)
{
    int ai, bi;
    if (!a || !b) return 0;
    for (ai = 0; ai < a->zone_count; ai++) {
        for (bi = 0; bi < b->zone_count; bi++) {
            if (_stricmp(a->zones[ai], b->zones[bi]) == 0) return 1;
        }
    }
    return 0;
}

static addon_equipment_slot_t *addon_equipment_slot_find(
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

static int addon_equipment_definition_current(
    const addon_equipment_definition_t *definition,
    const char *addon_id,
    const char *owner,
    void *root_object)
{
    int i;
    if (!definition || !addon_id || !owner || !owner[0]) return 1;
    for (i = 0; i < definition->zone_count; i++) {
        addon_equipment_slot_t *slot = addon_equipment_slot_find(
            owner, definition->zones[i], 0);
        if (!slot || !slot->addon_id[0]) continue;
        if (_stricmp(slot->addon_id, addon_id) != 0) return 0;
        if (root_object && slot->root_object &&
            slot->root_object != root_object) {
            return 0;
        }
    }
    return 1;
}

static int addon_equipment_sidecar_current(physx_sidecar_t *sc,
                                           const char *owner)
{
    char addon_id[256];
    addon_equipment_definition_t *definition;
    if (!sc || !owner || !owner[0] ||
        !sidecar_addon_identifier(sc, addon_id, sizeof(addon_id))) {
        return 1;
    }
    definition = addon_equipment_definition_for_sidecar(sc);
    return addon_equipment_definition_current(definition, addon_id, owner,
                                               NULL);
}

static int addon_equipment_sidecar_root_allowed(physx_sidecar_t *sc,
                                                const char *owner,
                                                void *root_object)
{
    char addon_id[256];
    addon_equipment_definition_t *definition;
    if (!sc || !owner || !owner[0] ||
        !sidecar_addon_identifier(sc, addon_id, sizeof(addon_id))) {
        return 0;
    }
    definition = addon_equipment_definition_for_sidecar(sc);
    return addon_equipment_definition_current(definition, addon_id, owner,
                                               root_object);
}

static void addon_equipment_scan_dress_script(const char *path)
{
    FILE *f;
    char line[2048];
    char addon_id[256];
    char zone[64];
    if (!path || !path[0]) return;
    f = fopen(path, "rb");
    if (!f) return;
    addon_id[0] = 0;
    zone[0] = 0;
    while (fgets(line, sizeof(line), f)) {
        char *p;
        if (strstr(line, "DressDescription")) {
            addon_id[0] = 0;
            zone[0] = 0;
        }
        p = strstr(line, "\"Ac");
        if (p && strstr(line, ".SceneScriptFile")) {
            char *end;
            p += 3;
            end = strchr(p, '\"');
            if (end && end > p) {
                size_t len = (size_t)(end - p);
                if (len >= sizeof(addon_id)) len = sizeof(addon_id) - 1;
                memcpy(addon_id, p, len);
                addon_id[len] = 0;
            }
        }
        if (strstr(line, ".PrimaryZone")) {
            char *end;
            p = strstr(line, ":DZ_");
            if (p) {
                p++;
                end = p;
                while (*end && *end != ';' && *end != ']' &&
                       *end != ' ' && *end != '\t' &&
                       *end != '\r' && *end != '\n') {
                    end++;
                }
            } else {
                p = strstr(line, "I32(");
                end = p ? strchr(p, ')') : NULL;
                if (end) end++;
            }
            if (p && end && end > p) {
                size_t len = (size_t)(end - p);
                if (len >= sizeof(zone)) len = sizeof(zone) - 1;
                memcpy(zone, p, len);
                zone[len] = 0;
            }
        }
        if (addon_id[0] && zone[0]) {
            addon_equipment_register_zone(addon_id, zone);
        }
    }
    fclose(f);
}

static addon_active_slot_t *addon_active_slot_find(const char *owner,
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

static int sidecar_selected_for_owner(physx_sidecar_t *sc,
                                      const char *owner)
{
    addon_active_slot_t *entry;
    char addon_id[256];
    if (!sc || !owner || !owner[0] ||
        !sidecar_addon_identifier(sc, addon_id, sizeof(addon_id))) {
        return 1;
    }
    if (!addon_equipment_sidecar_current(sc, owner)) return 0;
    entry = addon_active_slot_find(owner, addon_id, 0);
    if (!entry || !entry->sidecar) {
        /* An override is never the implicit/default configuration.  It only
           becomes eligible after an observed ActiveMod selection assigns it
           to this exact person/add-on slot. */
        return sidecar_path_is_activemod_a(sc->path) ? 0 : 1;
    }
    return entry->sidecar == sc;
}

static int sidecar_owner_has_live_addon_root(physx_sidecar_t *sc,
                                             const char *person)
{
    DWORD now = GetTickCount();
    LONG generation =
        InterlockedCompareExchange(&named_node_generation, 0, 0);
    addon_active_slot_t *active_entry;
    char addon_id[256];
    int person_index = addon_person_prefix_to_index(person);
    int i;
    int live = 0;

    if (!sc || !person || !person[0] || person_index < 0 || person_index >= 4) {
        return 0;
    }
    if (!sidecar_selected_for_owner(sc, person)) return 0;
    if (sc->addon_live_root_cache_valid[person_index] &&
        sc->addon_live_root_cache_generation[person_index] == generation &&
        now - sc->addon_live_root_cache_tick[person_index] < 1000u) {
        return sc->addon_live_root_cache_value[person_index] ? 1 : 0;
    }

    /* Normal TK17 root and ActiveMod events already maintain this exact
       Person/add-on slot and invalidate it on equipment replacement.  Use
       that authoritative lifecycle record instead of periodically walking
       every captured Object.Name entry for every installed sidecar. */
    addon_id[0] = 0;
    if (!sidecar_addon_identifier(sc, addon_id, sizeof(addon_id))) {
        goto cache_result;
    }
    active_entry = addon_active_slot_find(person, addon_id, 0);
    if (active_entry) {
        void *root = active_entry->root_object;
        live = root &&
               (!active_entry->sidecar || active_entry->sidecar == sc) &&
               !is_nil_engine_object(NULL, root) &&
               addon_equipment_sidecar_root_allowed(sc, person, root);
        goto cache_result;
    }

    /* Retain one recovery scan for an actual .bs scene activation whose root
       naming event was missed (for example unusual startup ordering).  The
       attempt is keyed to that activation, so absent or unequipped sidecars
       cannot rescan the 16K node table once per second forever. */
    if (!sc->addon_scene_active || !sc->addon_scene_active_tick ||
        sc->addon_live_root_fallback_scene_tick[person_index] ==
            sc->addon_scene_active_tick) {
        goto cache_result;
    }
    if (!addon_owner_person_live_for_binding(person)) {
        goto cache_result;
    }
    sc->addon_live_root_fallback_scene_tick[person_index] =
        sc->addon_scene_active_tick;
    for (i = named_node_count - 1; i >= 0; i--) {
        char root_owner[16];
        void *root = named_nodes[i].object;
        if (!root ||
            !sidecar_named_node_matches_addon_root(sc,
                                                   named_nodes[i].name) ||
            is_nil_engine_object(NULL, root)) {
            continue;
        }
        root_owner[0] = 0;
        if (addon_extract_person_prefix(named_nodes[i].name,
                                        root_owner,
                                        sizeof(root_owner)) &&
            _stricmp(root_owner, person) == 0 &&
            addon_equipment_sidecar_root_allowed(sc, person, root)) {
            live = 1;
            active_entry = addon_active_slot_find(person, addon_id, 1);
            if (active_entry) {
                active_entry->sidecar = sc;
                active_entry->root_object = root;
                active_entry->root_tick = now;
            }
            break;
        }
    }

cache_result:
    sc->addon_live_root_cache_generation[person_index] = generation;
    sc->addon_live_root_cache_tick[person_index] = now;
    sc->addon_live_root_cache_valid[person_index] = 1;
    sc->addon_live_root_cache_value[person_index] = live ? 1 : 0;
    return live;
}

static int sidecar_named_node_matches_addon_armature_root(const char *root_name)
{
    (void)root_name;
    /*
       Object.Name captures for Blender armature roots are not guaranteed to be
       live TSNodes. Calling SearchTree on one during room load can crash TK17.
       Keep live add-on resolution limited to named scene roots until we have a
       verified owner-bound armature pointer.
    */
    return 0;
}

static int sidecar_live_addon_owner_body_prefix(physx_sidecar_t *sc,
                                                const char *root_name,
                                                char *out,
                                                size_t outsz)
{
    char basename[256];
    const char *p;
    if (!out || outsz == 0) return 0;
    out[0] = 0;
    if (!sc || !root_name || !root_name[0]) return 0;
    sidecar_scene_basename_no_ext(sc->path, basename, sizeof(basename));
    if (!basename[0]) return 0;
    for (p = root_name; *p; p++) {
        if (_strnicmp(p, basename, strlen(basename)) == 0) {
            size_t prefix_len = (size_t)(p - root_name);
            if (prefix_len == 0 || prefix_len + 5 >= outsz) return 0;
            lstrcpynA(out, root_name, (int)min(prefix_len + 1, outsz));
            out[prefix_len] = 0;
            lstrcpynA(out + prefix_len, "Body", outsz - prefix_len);
            return 1;
        }
    }
    return 0;
}

static int sidecar_live_addon_owner_prefix(physx_sidecar_t *sc,
                                           const char *root_name,
                                           char *out,
                                           size_t outsz)
{
    char basename[256];
    const char *p;
    if (!out || outsz == 0) return 0;
    out[0] = 0;
    if (!sc || !root_name || !root_name[0]) return 0;
    sidecar_scene_basename_no_ext(sc->path, basename, sizeof(basename));
    if (!basename[0]) return 0;
    for (p = root_name; *p; p++) {
        if (_strnicmp(p, basename, strlen(basename)) == 0) {
            size_t prefix_len = (size_t)(p - root_name);
            if (prefix_len == 0 || prefix_len >= outsz) return 0;
            lstrcpynA(out, root_name, (int)min(prefix_len + 1, outsz));
            out[prefix_len] = 0;
            return 1;
        }
    }
    return 0;
}

static void *resolve_exact_named_candidate(const char *candidate,
                                           void **raw_out,
                                           char *matched,
                                           size_t matched_sz)
{
    void *raw = NULL;
    void *obj = NULL;
    if (raw_out) *raw_out = NULL;
    if (!candidate || !candidate[0]) return NULL;
    obj = resolve_find_obj(candidate, &raw);
    if (obj && !is_nil_engine_object(raw, obj)) {
        if (raw_out) *raw_out = raw;
        if (matched && matched_sz) lstrcpynA(matched, candidate, (int)matched_sz);
        return obj;
    }
    if (captured_script_engine) {
        raw = NULL;
        obj = resolve_script_engine_obj(candidate, &raw);
        if (obj && !is_nil_engine_object(raw, obj)) {
            if (raw_out) *raw_out = raw;
            if (matched && matched_sz) lstrcpynA(matched, candidate, (int)matched_sz);
            return obj;
        }
    }
    return NULL;
}

static void *resolve_addon_object_name_target(const char *target_name,
                                              void **raw_out,
                                              char *matched,
                                              size_t matched_sz)
{
    char local_name[192];
    const char *variants[2];
    int vi;
    if (raw_out) *raw_out = NULL;
    if (matched && matched_sz) matched[0] = 0;
    if (!target_name || !target_name[0]) return NULL;
    _snprintf(local_name, sizeof(local_name), "local_%s", target_name);
    variants[0] = target_name;
    variants[1] = local_name;
    for (vi = 0; vi < 2; vi++) {
        void *obj = find_named_node(variants[vi]);
        if (obj && !is_nil_engine_object(NULL, obj)) {
            if (matched && matched_sz) {
                char label[256];
                _snprintf(label, sizeof(label), "Object.Name:%s", variants[vi]);
                lstrcpynA(matched, label, (int)matched_sz);
            }
            return obj;
        }
    }
    return NULL;
}

static void *resolve_prefixed_candidate(const char *prefix,
                                        const char *target_name,
                                        void **raw_out,
                                        char *matched,
                                        size_t matched_sz)
{
    char plain_name[768];
    char local_name[768];
    const char *candidates[2];
    int ci;
    if (raw_out) *raw_out = NULL;
    if (!prefix || !prefix[0] || !target_name || !target_name[0]) return NULL;
    _snprintf(plain_name, sizeof(plain_name), "%s:%s", prefix, target_name);
    _snprintf(local_name, sizeof(local_name), "%s:local_%s", prefix, target_name);
    candidates[0] = plain_name;
    candidates[1] = local_name;
    for (ci = 0; ci < 2; ci++) {
        void *raw = NULL;
        void *obj = resolve_find_obj(candidates[ci], &raw);
        if (obj && !is_nil_engine_object(raw, obj)) {
            if (raw_out) *raw_out = raw;
            if (matched && matched_sz) lstrcpynA(matched, candidates[ci], (int)matched_sz);
            return obj;
        }
        if (captured_script_engine) {
            raw = NULL;
            obj = resolve_script_engine_obj(candidates[ci], &raw);
            if (obj && !is_nil_engine_object(raw, obj)) {
                if (raw_out) *raw_out = raw;
                if (matched && matched_sz) lstrcpynA(matched, candidates[ci], (int)matched_sz);
                return obj;
            }
        }
    }
    return NULL;
}

static DWORD tick_delta_abs(DWORD a, DWORD b)
{
    return (a >= b) ? (a - b) : (b - a);
}

static void *resolve_recent_addon_object_name(const char *root_name,
                                              DWORD root_tick,
                                              const char *target_name,
                                              void **raw_out,
                                              char *matched,
                                              size_t matched_sz)
{
    char local_name[192];
    char s_name[192];
    char local_s_name[192];
    const char *variants[4];
    int vi, ni;
    static int recent_object_log_count;
    if (raw_out) *raw_out = NULL;
    if (!root_name || !root_name[0] || !root_tick ||
        !target_name || !target_name[0]) {
        return NULL;
    }
    _snprintf(local_name, sizeof(local_name), "local_%s", target_name);
    variants[0] = target_name;
    variants[1] = local_name;
    if (_strnicmp(target_name, "S", 1) == 0) {
        variants[2] = "";
        variants[3] = "";
    } else {
        _snprintf(s_name, sizeof(s_name), "S%s", target_name);
        _snprintf(local_s_name, sizeof(local_s_name), "local_S%s", target_name);
        variants[2] = s_name;
        variants[3] = local_s_name;
    }
    for (vi = 0; vi < 4; vi++) {
        void *best_obj = NULL;
        DWORD best_delta = 0xffffffffu;
        if (!variants[vi] || !variants[vi][0]) continue;
        for (ni = named_node_count - 1; ni >= 0; ni--) {
            void *obj;
            DWORD delta;
            if (_stricmp(named_nodes[ni].name, variants[vi]) != 0) continue;
            if (!named_nodes[ni].first_seen_tick) continue;
            obj = named_nodes[ni].object;
            if (!obj || is_nil_engine_object(NULL, obj)) continue;
            delta = tick_delta_abs(named_nodes[ni].first_seen_tick, root_tick);
            if (delta > 5000) continue;
            if (delta >= best_delta) continue;
            best_delta = delta;
            best_obj = obj;
        }
        if (best_obj) {
            if (matched && matched_sz) {
                char label[256];
                _snprintf(label, sizeof(label), "Object.Name:%s", variants[vi]);
                lstrcpynA(matched, label, (int)matched_sz);
            }
            if (defaults_cfg.debug && recent_object_log_count < 120) {
                recent_object_log_count++;
                log_line("target found via recent addon Object.Name root=\"%s\" target=\"%s\" runtime=\"%s\" object=%p root_age_delta_ms=%lu named_nodes=%d note=\"custom add-on joint was named immediately around live add-on root activation; accepting as live import object\"",
                         root_name, target_name, variants[vi], best_obj,
                         best_delta,
                         named_node_count);
            }
            return best_obj;
        }
    }
    return NULL;
}

static void *resolve_live_addon_named_target(physx_sidecar_t *sc,
                                             const char *required_owner,
                                             const char *target_name,
                                             int simulated_target,
                                             void **raw_out,
                                             char *matched,
                                             size_t matched_sz)
{
    static int live_miss_log_count;
    static int captured_name_log_count;
    int i;
    if (raw_out) *raw_out = NULL;
    if (matched && matched_sz) matched[0] = 0;
    if (!sc || !target_name || !target_name[0]) return NULL;
    if (sc->room_scene_sidecar && sc->addon_scene_active) {
        void *room_obj = resolve_addon_object_name_target(
            target_name, raw_out, matched, matched_sz);
        if (room_obj) {
            return room_obj;
        }
    }
    if (required_owner && required_owner[0] &&
        !sidecar_selected_for_owner(sc, required_owner)) {
        return NULL;
    }
    for (i = named_node_count - 1; i >= 0; i--) {
        void *root = named_nodes[i].object;
        const char *root_name = named_nodes[i].name;
        void *obj = NULL;
        void *raw = NULL;
        char owner_prefix[128];
        char owner_body[128];
        char owner_anim[128];
        char root_model[256];
        char root_armature[256];
        char root_local_armature[256];
        char root_scene_no_ext[768];
        char root_scene_ma[768];
        char scene_no_ext[512];
        char scene_ma[512];
        char local_name[192];
        char s_name[192];
        char local_s_name[192];
        const char *target_variants[4];
        const char *prefixes[10];
        int pi, vi;
        int is_scene_root;
        int is_armature_root;
        if (!root || is_nil_engine_object(NULL, root)) continue;
        is_scene_root = sidecar_named_node_matches_addon_root(sc, root_name);
        is_armature_root = sidecar_named_node_matches_addon_armature_root(root_name);
        if (!is_scene_root && !is_armature_root) continue;
        if (captured_script_engine && !named_nodes[i].logged) {
            named_nodes[i].logged = 1;
            import_script_object_tree_names(root);
            log_line("live addon root import attempted root_name=\"%s\" root=%p sidecar=\"%s\" note=\"importing live add-on tree before resolving custom sidecar bones\"",
                     root_name, root, sc->path);
        }
        owner_prefix[0] = 0;
        owner_body[0] = 0;
        owner_anim[0] = 0;
        root_model[0] = 0;
        root_armature[0] = 0;
        root_local_armature[0] = 0;
        root_scene_no_ext[0] = 0;
        root_scene_ma[0] = 0;
        scene_no_ext[0] = 0;
        scene_ma[0] = 0;
        sidecar_live_addon_owner_prefix(sc, root_name, owner_prefix, sizeof(owner_prefix));
        sidecar_live_addon_owner_body_prefix(sc, root_name, owner_body, sizeof(owner_body));
        if (required_owner && required_owner[0] &&
            _stricmp(owner_prefix, required_owner) != 0) {
            continue;
        }
        if (!addon_equipment_sidecar_root_allowed(sc, owner_prefix, root)) {
            continue;
        }
        /* TK17 preloads configured add-ons for person slots that have not
           entered the room. Those roots are valid objects, but their matching
           Body:root remains a zero placeholder. Never let such a preload take
           ownership from the same add-on worn by a live person. */
        if (!addon_owner_person_live_for_binding(owner_prefix)) {
            continue;
        }
        sidecar_note_live_addon_owner_person(sc, owner_prefix, root_name,
                                             GetTickCount());
        if (owner_prefix[0]) _snprintf(owner_anim, sizeof(owner_anim), "%sAnim", owner_prefix);
        _snprintf(root_model, sizeof(root_model), "%s:Model01", root_name);
        _snprintf(root_armature, sizeof(root_armature), "%s:root_rotation_group", root_name);
        _snprintf(root_local_armature, sizeof(root_local_armature), "%s:local_root_rotation_group", root_name);
        sidecar_scene_object_prefix_no_ext(sc->path, scene_no_ext, sizeof(scene_no_ext));
        sidecar_scene_object_prefix(sc->path, scene_ma, sizeof(scene_ma));
        if (scene_no_ext[0]) _snprintf(root_scene_no_ext, sizeof(root_scene_no_ext), "%s:%s", root_name, scene_no_ext);
        if (scene_ma[0]) _snprintf(root_scene_ma, sizeof(root_scene_ma), "%s:%s", root_name, scene_ma);
        _snprintf(local_name, sizeof(local_name), "local_%s", target_name);
        target_variants[0] = target_name;
        target_variants[1] = local_name;
        if (_strnicmp(target_name, "S", 1) == 0) {
            target_variants[2] = "";
            target_variants[3] = "";
        } else {
            _snprintf(s_name, sizeof(s_name), "S%s", target_name);
            _snprintf(local_s_name, sizeof(local_s_name), "local_S%s", target_name);
            target_variants[2] = s_name;
            target_variants[3] = local_s_name;
        }
        prefixes[0] = root_name;
        prefixes[1] = root_model;
        prefixes[2] = root_armature;
        prefixes[3] = root_local_armature;
        prefixes[4] = root_scene_no_ext;
        prefixes[5] = root_scene_ma;
        prefixes[6] = owner_body;
        prefixes[7] = owner_anim;
        prefixes[8] = owner_anim[0] ? "Model01" : "";
        prefixes[9] = "";
        if (!simulated_target && owner_body[0]) {
            obj = resolve_prefixed_candidate(owner_body, target_name, &raw, matched, matched_sz);
            if (obj) {
                if (raw_out) *raw_out = raw;
                return obj;
            }
        }
        for (pi = 0; pi < 8; pi++) {
            if (!prefixes[pi] || !prefixes[pi][0]) continue;
            for (vi = 0; vi < 4; vi++) {
                char candidate[768];
                if (!target_variants[vi] || !target_variants[vi][0]) continue;
                _snprintf(candidate, sizeof(candidate), "%s:%s", prefixes[pi], target_variants[vi]);
                obj = resolve_exact_named_candidate(candidate, &raw, matched, matched_sz);
                if (obj) {
                    if (raw_out) *raw_out = raw;
                    return obj;
                }
            }
        }
        if (owner_anim[0]) {
            for (vi = 0; vi < 4; vi++) {
                char candidate[768];
                if (!target_variants[vi] || !target_variants[vi][0]) continue;
                _snprintf(candidate, sizeof(candidate), "%s:Model01:%s", owner_anim, target_variants[vi]);
                obj = resolve_exact_named_candidate(candidate, &raw, matched, matched_sz);
                if (obj) {
                    if (raw_out) *raw_out = raw;
                    return obj;
                }
            }
        }
        obj = resolve_recent_addon_object_name(root_name,
                                              named_nodes[i].first_seen_tick,
                                              target_name,
                                              &raw,
                                              matched,
                                              matched_sz);
        if (obj) {
            if (raw_out) *raw_out = raw;
            return obj;
        }
        if (captured_script_engine && engine_SearchTree) {
            int si;
            for (si = 0; si < 4; si++) {
                if (!target_variants[si] || !target_variants[si][0]) continue;
                obj = search_one_named_node_tree(root, root_name, target_variants[si]);
                if (obj) {
                    if (raw_out) *raw_out = NULL;
                    if (matched && matched_sz) lstrcpynA(matched, target_variants[si], (int)matched_sz);
                    if (defaults_cfg.debug && captured_name_log_count < 80) {
                        captured_name_log_count++;
                        log_line("target found via live addon search tree root=\"%s\" target=\"%s\" runtime=\"%s\" object=%p sidecar=\"%s\" note=\"validated custom add-on joint as a child of the active add-on root\"",
                                 root_name, target_name, target_variants[si], obj, sc->path);
                    }
                    return obj;
                }
            }
        }
        /*
           ComponentArray traversal is intentionally disabled here. Some live
           add-on roots expose unsafe component slots during room load, and
           probing them can crash TK17 before the room finishes initializing.
        */
        if (defaults_cfg.debug && live_miss_log_count < 80) {
            live_miss_log_count++;
            log_line("live addon target unresolved root=\"%s\" owner=\"%s\" body=\"%s\" anim=\"%s\" target=\"%s\" tried_root_model=\"%s\" scene=\"%s\" sidecar=\"%s\"",
                     root_name, owner_prefix, owner_body, owner_anim, target_name,
                     root_model, scene_no_ext, sc->path);
        }
    }
    if (simulated_target && defaults_cfg.debug &&
        captured_name_log_count < 80) {
        void *captured_obj = NULL;
        DWORD captured_age = 0;
        for (i = 0; i < named_node_count; i++) {
            if (_stricmp(named_nodes[i].name, target_name) == 0) {
                captured_obj = named_nodes[i].object;
                if (named_nodes[i].first_seen_tick) {
                    captured_age = GetTickCount() - named_nodes[i].first_seen_tick;
                }
                break;
            }
        }
        captured_name_log_count++;
        log_line("live addon target not-bound target=\"%s\" captured_object=%p captured_age_ms=%lu named_nodes=%d sidecar=\"%s\" note=\"bare Object.Name captures are diagnostics only until a clone or live add-on root validates the custom bone\"",
                 target_name, captured_obj, captured_age, named_node_count,
                 sc->path);
    }
    return NULL;
}

static physx_sidecar_t *find_or_add_sidecar(const char *path)
{
    int i;
    for (i = 0; i < sidecar_count; i++) {
        if (_stricmp(sidecars[i].path, path) == 0) return &sidecars[i];
    }
    if (sidecar_count >= (int)(sizeof(sidecars) / sizeof(sidecars[0]))) return NULL;
    memset(&sidecars[sidecar_count], 0, sizeof(sidecars[sidecar_count]));
    lstrcpynA(sidecars[sidecar_count].path, path, sizeof(sidecars[sidecar_count].path));
    return &sidecars[sidecar_count++];
}

static int physx_game_root_path_a(char *out, size_t outsz)
{
    char *slash;
    char *last_dir;
    if (!out || outsz == 0) return 0;
    out[0] = 0;
    if (self_module) {
        GetModuleFileNameA(self_module, out, (DWORD)outsz);
    } else {
        GetModuleFileNameA(NULL, out, (DWORD)outsz);
    }
    out[outsz - 1] = 0;
    slash = strrchr(out, '\\');
    if (!slash) slash = strrchr(out, '/');
    if (!slash) return 0;
    *slash = 0;
    last_dir = strrchr(out, '\\');
    if (!last_dir) last_dir = strrchr(out, '/');
    if (last_dir && _stricmp(last_dir + 1, "Binaries") == 0) {
        *last_dir = 0;
    }
    return out[0] != 0;
}

static int physx_build_game_path_a(const char *relative, char *out, size_t outsz)
{
    size_t len;
    if (!relative || !out || outsz == 0) return 0;
    if (!physx_game_root_path_a(out, outsz)) return 0;
    len = strlen(out);
    if (len + 1 >= outsz) return 0;
    if (out[len - 1] != '\\' && out[len - 1] != '/') {
        lstrcpynA(out + len, "\\", outsz - len);
        len++;
    }
    lstrcpynA(out + len, relative, outsz - len);
    return 1;
}

static int build_adjacent_physx_sidecar_path_a(const char *scene_path,
                                               char *out, size_t outsz)
{
    char *dot;
    if (!scene_path || !out || outsz == 0 || !ends_with_i(scene_path, ".bs")) return 0;
    lstrcpynA(out, scene_path, outsz);
    dot = strrchr(out, '.');
    if (!dot) return 0;
    lstrcpynA(dot, ".physx.ini", outsz - (size_t)(dot - out));
    return 1;
}

static int sidecar_path_is_activemod_a(const char *path)
{
    if (!path || !path[0]) return 0;
    return contains_i(path, "\\ActiveMod\\") ||
           contains_i(path, "/ActiveMod/");
}

static physx_sidecar_t *find_base_addon_sidecar_by_id(const char *addon_id)
{
    int i;
    if (!addon_id || !addon_id[0]) return NULL;
    for (i = 0; i < sidecar_count; i++) {
        physx_sidecar_t *candidate = &sidecars[i];
        char candidate_id[256];
        if (!candidate->path[0] ||
            sidecar_path_is_activemod_a(candidate->path) ||
            (!strstr(candidate->path, "\\Scenes\\") &&
             !strstr(candidate->path, "/Scenes/"))) {
            continue;
        }
        sidecar_scene_basename_no_ext(candidate->path,
                                      candidate_id,
                                      sizeof(candidate_id));
        if (_stricmp(candidate_id, addon_id) == 0) return candidate;
    }
    return NULL;
}

static int build_adjacent_scene_path_from_sidecar_a(const char *sidecar_path,
                                                    char *out, size_t outsz)
{
    char *dot;
    const char *source_path = sidecar_path;
    char addon_id[256];
    physx_sidecar_t *base_sc;
    if (!sidecar_path || !out || outsz == 0 || !ends_with_i(sidecar_path, ".physx.ini")) return 0;
    if (sidecar_path_is_activemod_a(sidecar_path)) {
        sidecar_scene_basename_no_ext(sidecar_path,
                                      addon_id,
                                      sizeof(addon_id));
        base_sc = find_base_addon_sidecar_by_id(addon_id);
        if (!base_sc) return 0;
        source_path = base_sc->path;
    }
    lstrcpynA(out, source_path, outsz);
    dot = strrchr(out, '.');
    if (!dot) return 0;
    while (dot > out && _strnicmp(dot, ".physx.ini", 10) != 0) {
        dot--;
    }
    if (_strnicmp(dot, ".physx.ini", 10) != 0) return 0;
    lstrcpynA(dot, ".bs", outsz - (size_t)(dot - out));
    return 1;
}

static int sidecar_has_addon_physx_sections_a(const char *path)
{
    char sections[8192];
    char *s;
    if (!path) return 0;
    GetPrivateProfileSectionNamesA(sections, sizeof(sections), path);
    for (s = sections; *s; s += strlen(s) + 1) {
        if (_strnicmp(s, "NC-TK17-PhysX:", 14) == 0) return 1;
    }
    return 0;
}

static int infer_addon_parent_from_scene_a(const char *sidecar_path,
                                           const char *target_name,
                                           char *out, size_t outsz)
{
    char scene_path[MAX_PATH * 4];
    char target_decl[256];
    FILE *f;
    char line[1024];
    int in_target = 0;
    if (!sidecar_path || !target_name || !target_name[0] || !out || outsz == 0) return 0;
    out[0] = 0;
    if (!build_adjacent_scene_path_from_sidecar_a(sidecar_path, scene_path, sizeof(scene_path))) {
        log_line("addon parent infer failed target=\"%s\" sidecar=\"%s\" reason=\"scene-path-build-failed\"",
                 target_name, sidecar_path ? sidecar_path : "");
        return 0;
    }
    f = fopen(scene_path, "rb");
    if (!f) {
        log_line("addon parent infer failed target=\"%s\" scene=\"%s\" reason=\"scene-open-failed\"",
                 target_name, scene_path);
        return 0;
    }
    _snprintf(target_decl, sizeof(target_decl), "TJoint :local_%s", target_name);
    while (fgets(line, sizeof(line), f)) {
        char *p;
        trim_in_place(line);
        if (!in_target) {
            if (strncmp(line, target_decl, strlen(target_decl)) == 0) {
                in_target = 1;
            }
            continue;
        }
        if (strncmp(line, "};", 2) == 0) break;
        p = strstr(line, "TNode.Parent TJoint :local_");
        if (p) {
            char *start = p + strlen("TNode.Parent TJoint :local_");
            char *end = start;
            while (*end && *end != ';' && *end != ' ' && *end != '\t' && *end != '\r' && *end != '\n') end++;
            *end = 0;
            if (start[0]) {
                lstrcpynA(out, start, outsz);
                fclose(f);
                log_line("addon parent inferred target=\"%s\" parent=\"%s\" scene=\"%s\" source=\"scene-block\"",
                         target_name, out, scene_path);
                return 1;
            }
        }
    }
    fclose(f);
    log_line("addon parent infer missing target=\"%s\" scene=\"%s\" decl=\"%s\"",
             target_name, scene_path, target_decl);
    return 0;
}

static unsigned int parse_collision_scope(const char *scope, int collision_enabled)
{
    char buf[256];
    char *p;
    unsigned int flags = 0;
    if (!collision_enabled) return 0;
    if (!scope || !scope[0]) return PHYSX_COLLISION_SCOPE_SELF;
    lstrcpynA(buf, scope, sizeof(buf));
    p = strtok(buf, ",");
    while (p) {
        trim_in_place(p);
        if (_stricmp(p, "self") == 0) flags |= PHYSX_COLLISION_SCOPE_SELF;
        else if (_stricmp(p, "body") == 0) flags |= PHYSX_COLLISION_SCOPE_BODY;
        else if (_stricmp(p, "body_all") == 0 || _stricmp(p, "bodies") == 0) flags |= PHYSX_COLLISION_SCOPE_BODY_ALL;
        else if (_stricmp(p, "addon") == 0 || _stricmp(p, "addons") == 0) flags |= PHYSX_COLLISION_SCOPE_ADDONS;
        else if (_stricmp(p, "custom") == 0) flags |= PHYSX_COLLISION_SCOPE_CUSTOM;
        else if (_stricmp(p, "room") == 0) flags |= PHYSX_COLLISION_SCOPE_ROOM;
        else if (_stricmp(p, "all") == 0) {
            flags |= PHYSX_COLLISION_SCOPE_SELF | PHYSX_COLLISION_SCOPE_BODY | PHYSX_COLLISION_SCOPE_ADDONS;
        }
        p = strtok(NULL, ",");
    }
    return flags ? flags : PHYSX_COLLISION_SCOPE_SELF;
}

typedef struct addon_custom_collision_target_def_t {
    const char *name;
    int node;
} addon_custom_collision_target_def_t;

static const addon_custom_collision_target_def_t
    addon_custom_collision_targets[] = {
    { "pelvis", BODY_COLLIDER_ROOT },
    { "spine01", BODY_COLLIDER_STOMACH_01 },
    { "spine02", BODY_COLLIDER_STOMACH_02 },
    { "spine03", BODY_COLLIDER_STOMACH_03 },
    { "spine04", BODY_COLLIDER_STOMACH_04 },
    /* Legacy sidecars may still use the old stomach target names. */
    { "stomach01", BODY_COLLIDER_STOMACH_01 },
    { "stomach02", BODY_COLLIDER_STOMACH_02 },
    { "stomach03", BODY_COLLIDER_STOMACH_03 },
    { "stomach04", BODY_COLLIDER_STOMACH_04 },
    { "neck01", BODY_COLLIDER_NECK_01 },
    { "head02", BODY_COLLIDER_HEAD_02 },
    { "breast_l", BODY_COLLIDER_BREAST_L },
    { "breast_r", BODY_COLLIDER_BREAST_R },
    { "hip_l", BODY_COLLIDER_HIP_L },
    { "hip_r", BODY_COLLIDER_HIP_R },
    { "thigh_l", BODY_COLLIDER_THIGH_L },
    { "thigh_r", BODY_COLLIDER_THIGH_R },
    { "knee_l", BODY_COLLIDER_KNEE_L },
    { "knee_r", BODY_COLLIDER_KNEE_R },
    { "ankle_l", BODY_COLLIDER_ANKLE_L },
    { "ankle_r", BODY_COLLIDER_ANKLE_R },
    { "ball_l", BODY_COLLIDER_BALL_L },
    { "ball_r", BODY_COLLIDER_BALL_R },
    { "clavicle_l", BODY_COLLIDER_CLAVICLE_L },
    { "clavicle_r", BODY_COLLIDER_CLAVICLE_R },
    { "shoulder_l", BODY_COLLIDER_SHOULDER_L },
    { "shoulder_r", BODY_COLLIDER_SHOULDER_R },
    { "elbow_l", BODY_COLLIDER_ELBOW_L },
    { "elbow_r", BODY_COLLIDER_ELBOW_R },
    { "forearm_l", BODY_COLLIDER_FOREARM_L },
    { "forearm_r", BODY_COLLIDER_FOREARM_R },
    { "wrist_l", BODY_COLLIDER_WRIST_L },
    { "wrist_r", BODY_COLLIDER_WRIST_R },
    { "palm_l", BODY_COLLIDER_PALM_L },
    { "palm_r", BODY_COLLIDER_PALM_R },
    { "testicles01", BODY_COLLIDER_TESTICLES_01 },
    { "testicles02", BODY_COLLIDER_TESTICLES_02 },
    { "testicles_mid", BODY_COLLIDER_TESTICLES_MID }
};

static void addon_custom_collision_set_node(physx_chain_t *chain, int node)
{
    if (!chain || node < 0 || node >= BODY_COLLIDER_NODE_COUNT) return;
    chain->collision_custom_target_mask[node] = 1;
}

static void addon_custom_collision_set_pair(physx_chain_t *chain,
                                            int left,
                                            int right)
{
    addon_custom_collision_set_node(chain, left);
    addon_custom_collision_set_node(chain, right);
}

static void addon_custom_collision_set_finger(physx_chain_t *chain,
                                              int finger,
                                              int side)
{
    static const int left_base[5] = {
        BODY_COLLIDER_FINGER01_L_01,
        BODY_COLLIDER_FINGER02_L_01,
        BODY_COLLIDER_FINGER03_L_01,
        BODY_COLLIDER_FINGER04_L_01,
        BODY_COLLIDER_FINGER05_L_01
    };
    static const int right_base[5] = {
        BODY_COLLIDER_FINGER01_R_01,
        BODY_COLLIDER_FINGER02_R_01,
        BODY_COLLIDER_FINGER03_R_01,
        BODY_COLLIDER_FINGER04_R_01,
        BODY_COLLIDER_FINGER05_R_01
    };
    int joint_count;
    int i;
    if (!chain || finger < 0 || finger >= 5) return;
    joint_count = finger == 0 ? 3 : 4;
    if (side <= 0) {
        for (i = 0; i <= joint_count; i++) {
            addon_custom_collision_set_node(chain, left_base[finger] + i);
        }
    }
    if (side < 0 || side == 1) {
        for (i = 0; i <= joint_count; i++) {
            addon_custom_collision_set_node(chain, right_base[finger] + i);
        }
    }
}

static int addon_custom_collision_set_finger_token(physx_chain_t *chain,
                                                    const char *token)
{
    static const int left_base[5] = {
        BODY_COLLIDER_FINGER01_L_01,
        BODY_COLLIDER_FINGER02_L_01,
        BODY_COLLIDER_FINGER03_L_01,
        BODY_COLLIDER_FINGER04_L_01,
        BODY_COLLIDER_FINGER05_L_01
    };
    static const int right_base[5] = {
        BODY_COLLIDER_FINGER01_R_01,
        BODY_COLLIDER_FINGER02_R_01,
        BODY_COLLIDER_FINGER03_R_01,
        BODY_COLLIDER_FINGER04_R_01,
        BODY_COLLIDER_FINGER05_R_01
    };
    const char *suffix;
    const char *segment;
    int finger;
    int side;
    int joint_count;
    int joint;
    int base;
    if (!chain || !token || _strnicmp(token, "finger", 6) != 0 ||
        token[6] != '0' || token[7] < '1' || token[7] > '5') {
        return 0;
    }
    finger = token[7] - '1';
    suffix = token + 8;
    if (!suffix[0]) {
        addon_custom_collision_set_finger(chain, finger, -1);
        return 1;
    }
    if (_stricmp(suffix, "_l") == 0) {
        addon_custom_collision_set_finger(chain, finger, 0);
        return 1;
    }
    if (_stricmp(suffix, "_r") == 0) {
        addon_custom_collision_set_finger(chain, finger, 1);
        return 1;
    }
    if (_strnicmp(suffix, "_l_", 3) == 0) {
        side = 0;
    } else if (_strnicmp(suffix, "_r_", 3) == 0) {
        side = 1;
    } else {
        return 0;
    }
    segment = suffix + 3;
    joint_count = finger == 0 ? 3 : 4;
    base = side == 0 ? left_base[finger] : right_base[finger];
    if (_stricmp(segment, "end") == 0) {
        addon_custom_collision_set_node(chain, base + joint_count);
        return 1;
    }
    if (segment[0] != '0' || segment[1] < '1' ||
        segment[1] > '0' + joint_count || segment[2] != 0) {
        return 0;
    }
    joint = segment[1] - '1';
    addon_custom_collision_set_node(chain, base + joint);
    return 1;
}

static void addon_custom_collision_set_hand(physx_chain_t *chain, int side)
{
    int finger;
    if (!chain) return;
    if (side <= 0) {
        addon_custom_collision_set_node(chain, BODY_COLLIDER_WRIST_L);
        addon_custom_collision_set_node(chain, BODY_COLLIDER_PALM_L);
    }
    if (side < 0 || side == 1) {
        addon_custom_collision_set_node(chain, BODY_COLLIDER_WRIST_R);
        addon_custom_collision_set_node(chain, BODY_COLLIDER_PALM_R);
    }
    for (finger = 0; finger < 5; finger++) {
        addon_custom_collision_set_finger(chain, finger, side);
    }
}

static void addon_custom_collision_set_arm(physx_chain_t *chain, int side)
{
    if (!chain) return;
    if (side <= 0) {
        addon_custom_collision_set_node(chain, BODY_COLLIDER_CLAVICLE_L);
        addon_custom_collision_set_node(chain, BODY_COLLIDER_SHOULDER_L);
        addon_custom_collision_set_node(chain, BODY_COLLIDER_ELBOW_L);
        addon_custom_collision_set_node(chain, BODY_COLLIDER_FOREARM_L);
    }
    if (side < 0 || side == 1) {
        addon_custom_collision_set_node(chain, BODY_COLLIDER_CLAVICLE_R);
        addon_custom_collision_set_node(chain, BODY_COLLIDER_SHOULDER_R);
        addon_custom_collision_set_node(chain, BODY_COLLIDER_ELBOW_R);
        addon_custom_collision_set_node(chain, BODY_COLLIDER_FOREARM_R);
    }
    addon_custom_collision_set_hand(chain, side);
}

static void addon_custom_collision_set_leg(physx_chain_t *chain, int side)
{
    if (!chain) return;
    if (side <= 0) {
        addon_custom_collision_set_node(chain, BODY_COLLIDER_HIP_L);
        addon_custom_collision_set_node(chain, BODY_COLLIDER_THIGH_L);
        addon_custom_collision_set_node(chain, BODY_COLLIDER_KNEE_L);
        addon_custom_collision_set_node(chain, BODY_COLLIDER_ANKLE_L);
        addon_custom_collision_set_node(chain, BODY_COLLIDER_BALL_L);
    }
    if (side < 0 || side == 1) {
        addon_custom_collision_set_node(chain, BODY_COLLIDER_HIP_R);
        addon_custom_collision_set_node(chain, BODY_COLLIDER_THIGH_R);
        addon_custom_collision_set_node(chain, BODY_COLLIDER_KNEE_R);
        addon_custom_collision_set_node(chain, BODY_COLLIDER_ANKLE_R);
        addon_custom_collision_set_node(chain, BODY_COLLIDER_BALL_R);
    }
}

static int addon_custom_collision_set_bilateral_alias(
    physx_chain_t *chain,
    const char *token)
{
    typedef struct bilateral_alias_t {
        const char *singular;
        const char *plural;
        int left;
        int right;
    } bilateral_alias_t;
    static const bilateral_alias_t aliases[] = {
        { "hip", "hips", BODY_COLLIDER_HIP_L, BODY_COLLIDER_HIP_R },
        { "thigh", "thighs", BODY_COLLIDER_THIGH_L, BODY_COLLIDER_THIGH_R },
        { "knee", "knees", BODY_COLLIDER_KNEE_L, BODY_COLLIDER_KNEE_R },
        { "ankle", "ankles", BODY_COLLIDER_ANKLE_L, BODY_COLLIDER_ANKLE_R },
        { "ball", "balls", BODY_COLLIDER_BALL_L, BODY_COLLIDER_BALL_R },
        { "breast", "breasts", BODY_COLLIDER_BREAST_L, BODY_COLLIDER_BREAST_R },
        { "clavicle", "clavicles", BODY_COLLIDER_CLAVICLE_L, BODY_COLLIDER_CLAVICLE_R },
        { "shoulder", "shoulders", BODY_COLLIDER_SHOULDER_L, BODY_COLLIDER_SHOULDER_R },
        { "elbow", "elbows", BODY_COLLIDER_ELBOW_L, BODY_COLLIDER_ELBOW_R },
        { "forearm", "forearms", BODY_COLLIDER_FOREARM_L, BODY_COLLIDER_FOREARM_R },
        { "wrist", "wrists", BODY_COLLIDER_WRIST_L, BODY_COLLIDER_WRIST_R },
        { "palm", "palms", BODY_COLLIDER_PALM_L, BODY_COLLIDER_PALM_R }
    };
    int i;
    if (!chain || !token) return 0;
    for (i = 0; i < (int)(sizeof(aliases) / sizeof(aliases[0])); i++) {
        if (_stricmp(token, aliases[i].singular) == 0 ||
            _stricmp(token, aliases[i].plural) == 0) {
            addon_custom_collision_set_pair(chain,
                                            aliases[i].left,
                                            aliases[i].right);
            return 1;
        }
    }
    return 0;
}

static int addon_custom_collision_apply_token(physx_chain_t *chain,
                                              const char *token)
{
    int i;
    if (!chain || !token || !token[0]) return 0;
    for (i = 0;
         i < (int)(sizeof(addon_custom_collision_targets) /
                   sizeof(addon_custom_collision_targets[0]));
         i++) {
        if (_stricmp(token, addon_custom_collision_targets[i].name) == 0) {
            addon_custom_collision_set_node(
                chain, addon_custom_collision_targets[i].node);
            return 1;
        }
    }
    if (addon_custom_collision_set_finger_token(chain, token) ||
        addon_custom_collision_set_bilateral_alias(chain, token)) {
        return 1;
    }
    if (_stricmp(token, "neck") == 0) {
        addon_custom_collision_set_node(chain, BODY_COLLIDER_NECK_01);
    } else if (_stricmp(token, "head") == 0) {
        addon_custom_collision_set_node(chain, BODY_COLLIDER_HEAD_02);
    } else if (_stricmp(token, "spine") == 0) {
        addon_custom_collision_set_node(chain, BODY_COLLIDER_STOMACH_01);
        addon_custom_collision_set_node(chain, BODY_COLLIDER_STOMACH_02);
        addon_custom_collision_set_node(chain, BODY_COLLIDER_STOMACH_03);
        addon_custom_collision_set_node(chain, BODY_COLLIDER_STOMACH_04);
    } else if (_stricmp(token, "pelvis_area") == 0) {
        addon_custom_collision_set_node(chain, BODY_COLLIDER_ROOT);
        addon_custom_collision_set_node(chain, BODY_COLLIDER_STOMACH_01);
        addon_custom_collision_set_pair(chain, BODY_COLLIDER_HIP_L,
                                        BODY_COLLIDER_HIP_R);
    } else if (_stricmp(token, "torso") == 0) {
        addon_custom_collision_set_node(chain, BODY_COLLIDER_ROOT);
        addon_custom_collision_set_node(chain, BODY_COLLIDER_STOMACH_01);
        addon_custom_collision_set_node(chain, BODY_COLLIDER_STOMACH_02);
        addon_custom_collision_set_node(chain, BODY_COLLIDER_STOMACH_03);
        addon_custom_collision_set_node(chain, BODY_COLLIDER_STOMACH_04);
        addon_custom_collision_set_pair(chain, BODY_COLLIDER_BREAST_L,
                                        BODY_COLLIDER_BREAST_R);
    } else if (_stricmp(token, "genitals") == 0) {
        addon_custom_collision_set_node(chain, BODY_COLLIDER_TESTICLES_01);
        addon_custom_collision_set_node(chain, BODY_COLLIDER_TESTICLES_02);
        addon_custom_collision_set_node(chain, BODY_COLLIDER_TESTICLES_MID);
    } else if (_stricmp(token, "fingers_l") == 0) {
        for (i = 0; i < 5; i++) addon_custom_collision_set_finger(chain, i, 0);
    } else if (_stricmp(token, "fingers_r") == 0) {
        for (i = 0; i < 5; i++) addon_custom_collision_set_finger(chain, i, 1);
    } else if (_stricmp(token, "fingers") == 0) {
        for (i = 0; i < 5; i++) addon_custom_collision_set_finger(chain, i, -1);
    } else if (_stricmp(token, "hand_l") == 0) {
        addon_custom_collision_set_hand(chain, 0);
    } else if (_stricmp(token, "hand_r") == 0) {
        addon_custom_collision_set_hand(chain, 1);
    } else if (_stricmp(token, "hands") == 0) {
        addon_custom_collision_set_hand(chain, -1);
    } else if (_stricmp(token, "arm_l") == 0) {
        addon_custom_collision_set_arm(chain, 0);
    } else if (_stricmp(token, "arm_r") == 0) {
        addon_custom_collision_set_arm(chain, 1);
    } else if (_stricmp(token, "arms") == 0) {
        addon_custom_collision_set_arm(chain, -1);
    } else if (_stricmp(token, "leg_l") == 0) {
        addon_custom_collision_set_leg(chain, 0);
    } else if (_stricmp(token, "leg_r") == 0) {
        addon_custom_collision_set_leg(chain, 1);
    } else if (_stricmp(token, "legs") == 0) {
        addon_custom_collision_set_leg(chain, -1);
    } else if (_stricmp(token, "all") == 0) {
        memset(chain->collision_custom_target_mask, 1,
               sizeof(chain->collision_custom_target_mask));
    } else {
        return 0;
    }
    return 1;
}

static int parse_collision_custom_targets(physx_chain_t *chain,
                                          const char *targets,
                                          const char *section,
                                          const char *path)
{
    char buf[4096];
    char *p;
    int node;
    int count = 0;
    if (!chain) return 0;
    memset(chain->collision_custom_target_mask, 0,
           sizeof(chain->collision_custom_target_mask));
    if (!targets || !targets[0]) return 0;
    lstrcpynA(buf, targets, sizeof(buf));
    p = strtok(buf, ",");
    while (p) {
        trim_in_place(p);
        if (p[0] && !addon_custom_collision_apply_token(chain, p)) {
            log_line("addon sidecar custom collision target ignored section=\"%s\" target=\"%s\" sidecar=\"%s\" reason=\"unknown collision_scope_custom_targets value\"",
                     section ? section : "", p, path ? path : "");
        }
        p = strtok(NULL, ",");
    }
    for (node = 0; node < BODY_COLLIDER_NODE_COUNT; node++) {
        if (chain->collision_custom_target_mask[node]) count++;
    }
    return count;
}

static int parse_collision_custom_persons(const char *value,
                                          int *all_persons_out)
{
    if (!value || !value[0] || !all_persons_out) return 0;
    if (_stricmp(value, "wearer") == 0 ||
        _stricmp(value, "owner") == 0 ||
        _stricmp(value, "self") == 0 ||
        _stricmp(value, "body") == 0) {
        *all_persons_out = 0;
        return 1;
    }
    if (_stricmp(value, "all") == 0 ||
        _stricmp(value, "body_all") == 0) {
        *all_persons_out = 1;
        return 1;
    }
    return 0;
}

static int addon_chain_custom_body_node_enabled(const physx_chain_t *chain,
                                                int node)
{
    if (!chain || node < 0 || node >= BODY_COLLIDER_NODE_COUNT) return 0;
    return !chain->collision_custom_enabled ||
           chain->collision_custom_target_mask[node] != 0;
}

static int addon_chain_custom_body_edge_enabled(const physx_chain_t *chain,
                                                int start_node,
                                                int end_node)
{
    return addon_chain_custom_body_node_enabled(chain, start_node) &&
           addon_chain_custom_body_node_enabled(chain, end_node);
}

static int sidecar_hex_digit(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

static DWORD parse_sidecar_debug_color(const char *value, DWORD fallback)
{
    const char *p = value;
    unsigned int r, g, b;
    int len = 0;
    if (!p) return fallback;
    while (*p == ' ' || *p == '\t') p++;
    if (*p == '#') p++;
    while (sidecar_hex_digit(p[len]) >= 0) len++;
    if (len == 3) {
        int r0 = sidecar_hex_digit(p[0]);
        int g0 = sidecar_hex_digit(p[1]);
        int b0 = sidecar_hex_digit(p[2]);
        if (r0 < 0 || g0 < 0 || b0 < 0) return fallback;
        r = (unsigned int)(r0 * 17);
        g = (unsigned int)(g0 * 17);
        b = (unsigned int)(b0 * 17);
    } else if (len == 6) {
        int r0 = sidecar_hex_digit(p[0]);
        int r1 = sidecar_hex_digit(p[1]);
        int g0 = sidecar_hex_digit(p[2]);
        int g1 = sidecar_hex_digit(p[3]);
        int b0 = sidecar_hex_digit(p[4]);
        int b1 = sidecar_hex_digit(p[5]);
        if (r0 < 0 || r1 < 0 || g0 < 0 ||
            g1 < 0 || b0 < 0 || b1 < 0) {
            return fallback;
        }
        r = (unsigned int)((r0 << 4) | r1);
        g = (unsigned int)((g0 << 4) | g1);
        b = (unsigned int)((b0 << 4) | b1);
    } else {
        return fallback;
    }
    return 0xff000000u | (r << 16) | (g << 8) | b;
}

static void parse_angle_vec3_or_scalar(const char *section, const char *key,
                                       float fallback, float out[3],
                                       const char *path)
{
    char buf[128];
    out[0] = fallback;
    out[1] = fallback;
    out[2] = fallback;
    GetPrivateProfileStringA(section, key, "", buf, sizeof(buf), path);
    trim_in_place(buf);
    if (!buf[0]) return;
    if (strchr(buf, ',')) {
        parse_vec3(buf, out);
    } else {
        out[0] = out[1] = out[2] = (float)atof(buf);
    }
}

static int sidecar_profile_key_exists(const char *section, const char *key,
                                      const char *path)
{
    char buf[16];
    if (!section || !key || !path) return 0;
    GetPrivateProfileStringA(section, key, "\x01", buf, sizeof(buf), path);
    return strcmp(buf, "\x01") != 0;
}

static int parse_angle_alias_vec3_or_scalar(const char *section,
                                            const char *friendly_key,
                                            const char *legacy_key,
                                            float fallback,
                                            float out[3],
                                            const char *path)
{
    if (friendly_key &&
        sidecar_profile_key_exists(section, friendly_key, path)) {
        parse_angle_vec3_or_scalar(section, friendly_key, fallback, out, path);
        return 1;
    }
    if (legacy_key && sidecar_profile_key_exists(section, legacy_key, path)) {
        parse_angle_vec3_or_scalar(section, legacy_key, fallback, out, path);
        return 1;
    }
    return 0;
}

static int sidecar_profile_axis(const char *section, const char *key,
                                int fallback, const char *path)
{
    char buf[32];
    if (!section || !key || !path) return fallback;
    GetPrivateProfileStringA(section, key, "", buf, sizeof(buf), path);
    trim_in_place(buf);
    if (!buf[0]) return fallback;
    return parse_axis_name(buf);
}

static int sidecar_profile_signed_axis(const char *section,
                                       const char *key,
                                       int fallback,
                                       float *sign_out,
                                       const char *path)
{
    char buf[32];
    char *p;
    float sign = 1.0f;
    if (sign_out) *sign_out = 1.0f;
    if (!section || !key || !path) return fallback;
    GetPrivateProfileStringA(section, key, "", buf, sizeof(buf), path);
    trim_in_place(buf);
    if (!buf[0]) return fallback;
    p = buf;
    if (*p == '-') {
        sign = -1.0f;
        p++;
    } else if (*p == '+') {
        p++;
    }
    trim_in_place(p);
    if (sign_out) *sign_out = sign;
    return parse_axis_name(p);
}

static int sidecar_profile_axis_alias(const char *section,
                                      const char *key,
                                      const char *legacy_key,
                                      int fallback,
                                      const char *path)
{
    if (key && sidecar_profile_key_exists(section, key, path)) {
        return sidecar_profile_axis(section, key, fallback, path);
    }
    if (legacy_key && sidecar_profile_key_exists(section, legacy_key, path)) {
        return sidecar_profile_axis(section, legacy_key, fallback, path);
    }
    return fallback;
}

static float sidecar_profile_float_alias(const char *section,
                                         const char *key,
                                         const char *legacy_key,
                                         float fallback,
                                         const char *path)
{
    if (key && sidecar_profile_key_exists(section, key, path)) {
        return profile_float(section, key, fallback, path);
    }
    if (legacy_key && sidecar_profile_key_exists(section, legacy_key, path)) {
        return profile_float(section, legacy_key, fallback, path);
    }
    return fallback;
}

static float max_abs_vec3_value(const float v[3])
{
    float m = physx_absf(v[0]);
    if (physx_absf(v[1]) > m) m = physx_absf(v[1]);
    if (physx_absf(v[2]) > m) m = physx_absf(v[2]);
    return m;
}

static int parse_scene_vector3_line(const char *line, float out[3])
{
    const char *p;
    if (!line || !out) return 0;
    p = strstr(line, "Vector3f(");
    if (!p) return 0;
    p += strlen("Vector3f(");
    return sscanf(p, " %f , %f , %f", &out[0], &out[1], &out[2]) == 3;
}

static int read_addon_sjoint_scene_pose(const char *sidecar_path,
                                        const char *target_name,
                                        float translation_out[3],
                                        int *has_translation,
                                        float orientation_out[3],
                                        int *has_orientation)
{
    char scene_path[MAX_PATH * 4];
    char sjoint_decl[256];
    FILE *f;
    char line[1024];
    int in_sjoint = 0;
    int found = 0;
    if (has_translation) *has_translation = 0;
    if (has_orientation) *has_orientation = 0;
    if (!sidecar_path || !target_name || !target_name[0]) return 0;
    if (!build_adjacent_scene_path_from_sidecar_a(sidecar_path, scene_path, sizeof(scene_path))) return 0;
    f = fopen(scene_path, "rb");
    if (!f) return 0;
    _snprintf(sjoint_decl, sizeof(sjoint_decl), "SJoint :local_S%s", target_name);
    while (fgets(line, sizeof(line), f)) {
        trim_in_place(line);
        if (!in_sjoint) {
            if (strncmp(line, sjoint_decl, strlen(sjoint_decl)) == 0) {
                in_sjoint = 1;
                found = 1;
            }
            continue;
        }
        if (strncmp(line, "};", 2) == 0) break;
        if (strstr(line, "SSimpleTransform.Translation") &&
            translation_out && parse_scene_vector3_line(line, translation_out)) {
            if (has_translation) *has_translation = 1;
        } else if (strstr(line, "SJoint.JointOrientation") &&
                   orientation_out && parse_scene_vector3_line(line, orientation_out)) {
            if (has_orientation) *has_orientation = 1;
        }
    }
    fclose(f);
    return found;
}

/* Read authored STransform data for type=object chains.  Object sidecars use
   this metadata instead of probing SJoint matrix layouts; the scene remains
   the authority for the object's pivot, rest rotation, and native limits. */
static int read_addon_stransform_scene_pose(
    const char *sidecar_path,
    const char *target_name,
    float translation_out[3],
    int *has_translation,
    float rotation_out[3],
    int *has_rotation,
    float rotation_min_out[3],
    int *has_rotation_min,
    float rotation_max_out[3],
    int *has_rotation_max)
{
    char scene_path[MAX_PATH * 4];
    char transform_decl[256];
    FILE *f;
    char line[1024];
    int in_transform = 0;
    int found = 0;
    if (has_translation) *has_translation = 0;
    if (has_rotation) *has_rotation = 0;
    if (has_rotation_min) *has_rotation_min = 0;
    if (has_rotation_max) *has_rotation_max = 0;
    if (!sidecar_path || !target_name || !target_name[0]) return 0;
    if (!build_adjacent_scene_path_from_sidecar_a(
            sidecar_path, scene_path, sizeof(scene_path))) {
        return 0;
    }
    f = fopen(scene_path, "rb");
    if (!f) return 0;
    _snprintf(transform_decl, sizeof(transform_decl),
              "STransform :local_S%s", target_name);
    while (fgets(line, sizeof(line), f)) {
        trim_in_place(line);
        if (!in_transform) {
            if (strncmp(line, transform_decl, strlen(transform_decl)) == 0) {
                in_transform = 1;
                found = 1;
            }
            continue;
        }
        if (strncmp(line, "};", 2) == 0) break;
        if (strstr(line, "SSimpleTransform.Translation") &&
            translation_out &&
            parse_scene_vector3_line(line, translation_out)) {
            if (has_translation) *has_translation = 1;
        } else if (strstr(line, "SSimpleTransform.RotationMin") &&
                   rotation_min_out &&
                   parse_scene_vector3_line(line, rotation_min_out)) {
            if (has_rotation_min) *has_rotation_min = 1;
        } else if (strstr(line, "SSimpleTransform.RotationMax") &&
                   rotation_max_out &&
                   parse_scene_vector3_line(line, rotation_max_out)) {
            if (has_rotation_max) *has_rotation_max = 1;
        } else if (strstr(line, "SSimpleTransform.Rotation") &&
                   !strstr(line, "RotationLimitEnableMask") &&
                   rotation_out &&
                   parse_scene_vector3_line(line, rotation_out)) {
            if (has_rotation) *has_rotation = 1;
        }
    }
    fclose(f);
    return found;
}

static void addon_object_chain_init_scene_metadata(physx_sidecar_t *sc,
                                                   physx_chain_t *chain)
{
    int t;
    if (!sc || !chain || !chain->object_transform_chain) return;
    for (t = 0; t < chain->target_count; t++) {
        physx_target_t *target = &chain->targets[t];
        int has_min = 0;
        int has_max = 0;
        target->object_transform_target = 1;
        read_addon_stransform_scene_pose(
            sc->path, target->name,
            target->object_scene_translation,
            &target->object_scene_translation_valid,
            target->object_scene_rotation,
            &target->object_scene_rotation_valid,
            target->object_scene_rotation_min, &has_min,
            target->object_scene_rotation_max, &has_max);
        target->object_scene_limits_valid = has_min && has_max;
        if (!target->object_scene_rotation_valid) {
            target->object_scene_rotation[0] = 0.0f;
            target->object_scene_rotation[1] = 0.0f;
            target->object_scene_rotation[2] = 0.0f;
        }
        memcpy(target->object_output_rotation,
               target->object_scene_rotation,
               sizeof(target->object_output_rotation));
    }
    for (t = 0; t < chain->target_count; t++) {
        physx_target_t *target = &chain->targets[t];
        const float *segment = NULL;
        float segment_len = 0.0f;
        if (!target->addon_simulated_target) {
            memset(target->object_proxy_rest, 0,
                   sizeof(target->object_proxy_rest));
            memset(target->object_proxy_translation, 0,
                   sizeof(target->object_proxy_translation));
            continue;
        }
        /* A transform's visible link extends toward its child's pivot.  For
           the terminal transform, its own authored local translation is the
           best available length/direction and avoids inventing a bone head. */
        if (t + 1 < chain->target_count &&
            chain->targets[t + 1].object_scene_translation_valid) {
            segment = chain->targets[t + 1].object_scene_translation;
            segment_len = addon_vec3_len_exact(segment);
        }
        if ((!segment || segment_len < 0.0005f || segment_len > 1.5f) &&
            target->object_scene_translation_valid) {
            segment = target->object_scene_translation;
            segment_len = addon_vec3_len_exact(segment);
        }
        if (!segment || segment_len < 0.0005f || segment_len > 1.5f) {
            target->object_proxy_rest[0] = 0.0f;
            target->object_proxy_rest[1] = -0.03f;
            target->object_proxy_rest[2] = 0.0f;
        } else {
            memcpy(target->object_proxy_rest, segment,
                   sizeof(target->object_proxy_rest));
        }
        memcpy(target->object_proxy_translation,
               target->object_proxy_rest,
               sizeof(target->object_proxy_translation));
        log_line("addon object scene metadata chain=\"%s\" target=\"%s\" simulated=%d translation_valid=%d translation=(%.5f,%.5f,%.5f) segment_rest=(%.5f,%.5f,%.5f) rotation_valid=%d rotation=(%.3f,%.3f,%.3f) scene_limits=%d min=(%.1f,%.1f,%.1f) max=(%.1f,%.1f,%.1f) sidecar=\"%s\"",
                 chain->name, target->name,
                 target->addon_simulated_target,
                 target->object_scene_translation_valid,
                 target->object_scene_translation[0],
                 target->object_scene_translation[1],
                 target->object_scene_translation[2],
                 target->object_proxy_rest[0],
                 target->object_proxy_rest[1],
                 target->object_proxy_rest[2],
                 target->object_scene_rotation_valid,
                 target->object_scene_rotation[0],
                 target->object_scene_rotation[1],
                 target->object_scene_rotation[2],
                 target->object_scene_limits_valid,
                 target->object_scene_rotation_min[0],
                 target->object_scene_rotation_min[1],
                 target->object_scene_rotation_min[2],
                 target->object_scene_rotation_max[0],
                 target->object_scene_rotation_max[1],
                 target->object_scene_rotation_max[2],
                 sc->path);
    }
}

static void parse_target_list(physx_chain_t *chain, const char *list)
{
    char buf[2048];
    char *p;
    if (!chain || !list) return;
    lstrcpynA(buf, list, sizeof(buf));
    p = strtok(buf, ",");
    while (p && chain->target_count < (int)(sizeof(chain->targets) / sizeof(chain->targets[0]))) {
        trim_in_place(p);
        if (p[0]) {
            physx_target_t *target = &chain->targets[chain->target_count];
            lstrcpynA(target->name, p, sizeof(target->name));
            target->s_translation_offset = -1;
            target->s_rotation_offset = -1;
            target->addon_rotation_offset = -1;
            target->s_rotation_min_offset = -1;
            target->s_rotation_max_offset = -1;
            chain->target_count++;
        }
        p = strtok(NULL, ",");
    }
}

static void addon_chain_init_target_joint_settings(physx_chain_t *chain)
{
    int t, axis;
    if (!chain) return;
    for (t = 0; t < chain->target_count; t++) {
        physx_target_t *target = &chain->targets[t];
        target->joint_settings_initialized = 1;
        target->joint_gain = chain->joint_gain;
        target->limit_angle = chain->limit_angle;
        for (axis = 0; axis < 3; axis++) {
            target->joint_min_angle[axis] = chain->joint_min_angle[axis];
            target->joint_max_angle[axis] = chain->joint_max_angle[axis];
        }
    }
}

static void addon_chain_init_target_gravity_settings(physx_chain_t *chain)
{
    int t;
    if (!chain) return;
    for (t = 0; t < chain->target_count; t++) {
        physx_target_t *target = &chain->targets[t];
        target->gravity_settings_initialized = 1;
        target->gravity_horizontal_source_axis =
            chain->gravity_horizontal_source_axis;
        target->gravity_horizontal_source_sign =
            chain->gravity_horizontal_source_sign;
        target->gravity_horizontal_tail_axis =
            chain->gravity_horizontal_tail_axis;
        target->gravity_horizontal_tail_axis_explicit =
            chain->gravity_horizontal_tail_axis_explicit;
        target->gravity_horizontal_scale =
            chain->gravity_horizontal_scale;
        target->gravity_vertical_source_axis =
            chain->gravity_vertical_source_axis;
        target->gravity_vertical_source_sign =
            chain->gravity_vertical_source_sign;
        target->gravity_vertical_tail_axis =
            chain->gravity_vertical_tail_axis;
        target->gravity_vertical_tail_axis_explicit =
            chain->gravity_vertical_tail_axis_explicit;
        target->gravity_vertical_scale =
            chain->gravity_vertical_scale;
        target->gravity_inverted_configured =
            chain->gravity_inverted_configured;
        target->gravity_inverted_strength =
            chain->gravity_inverted_strength;
        target->gravity_inverted_tail_axis =
            chain->gravity_inverted_tail_axis;
        target->gravity_inverted_sign =
            chain->gravity_inverted_sign;
    }
}

static int addon_sidecar_section_has_chain_keys(const char *section,
                                                const char *path)
{
    static const char *keys[] = {
        "parent", "children", "type", "enabled", "enable",
        "gravity_enabled", "collision_enabled", "collision_scope",
        "collision_scope_custom_targets",
        "collision_scope_custom_persons",
        "collision_scope_custom_scope",
        "drive_scale", "drive_strength", "stiffness", "damping",
        "translation_horizontal_source_axis",
        "translation_horizontal_tail_axis",
        "translation_horizontal_scale",
        "translation_vertical_source_axis",
        "translation_vertical_tail_axis",
        "translation_vertical_scale",
        "translation_depth_source_axis",
        "translation_depth_tail_axis",
        "translation_depth_scale",
        "gravity", "gravity_scale", "max_offset", "startup_impulse",
        "rotation_horizontal_source_axis",
        "rotation_horizontal_tail_axis",
        "rotation_horizontal_scale",
        "rotation_vertical_source_axis",
        "rotation_vertical_tail_axis",
        "rotation_vertical_scale",
        "rotation_twist_source_axis",
        "rotation_twist_tail_axis",
        "rotation_twist_scale",
        "rotation_solver",
        "gravity_horizontal_source_axis",
        "gravity_horizontal_tail_axis",
        "gravity_horizontal_scale",
        "gravity_vertical_source_axis",
        "gravity_vertical_tail_axis",
        "gravity_vertical_scale",
        "gravity_inverted_strength",
        "gravity_inverted_tail_axis",
        "gravity_inverted_sign"
    };
    int i;
    if (!section || !path) return 0;
    for (i = 0; i < (int)(sizeof(keys) / sizeof(keys[0])); i++) {
        if (sidecar_profile_key_exists(section, keys[i], path)) return 1;
    }
    return 0;
}

static int addon_sidecar_section_has_chain_root_keys(const char *section,
                                                     const char *path)
{
    if (!section || !path) return 0;
    return sidecar_profile_key_exists(section, "parent", path) ||
           sidecar_profile_key_exists(section, "children", path);
}

static int comma_list_contains_token_i(const char *list, const char *needle)
{
    char buf[2048];
    char *p;
    if (!list || !needle || !needle[0]) return 0;
    lstrcpynA(buf, list, sizeof(buf));
    p = buf;
    while (*p) {
        char *start = p;
        char *end;
        while (*start == ' ' || *start == '\t') start++;
        end = start;
        while (*end && *end != ',') end++;
        if (*end == ',') {
            *end = 0;
            p = end + 1;
        } else {
            p = end;
        }
        trim_in_place(start);
        if (_stricmp(start, needle) == 0) return 1;
    }
    return 0;
}

static int addon_sidecar_section_name_is_declared_child(
    const char *sections,
    const char *target_name,
    const char *path)
{
    const char *s;
    char children[2048];
    if (!sections || !target_name || !target_name[0] || !path) return 0;
    for (s = sections; *s; s += strlen(s) + 1) {
        if (_strnicmp(s, "NC-TK17-PhysX:", 14) != 0) continue;
        GetPrivateProfileStringA(s, "children", "", children,
                                 sizeof(children), path);
        trim_in_place(children);
        if (!children[0]) continue;
        if (comma_list_contains_token_i(children, target_name)) return 1;
    }
    return 0;
}

static physx_target_t *addon_sidecar_find_chain_target(
    physx_sidecar_t *sc,
    const char *target_name,
    physx_chain_t **chain_out,
    int *target_index_out)
{
    int c, t;
    if (chain_out) *chain_out = NULL;
    if (target_index_out) *target_index_out = -1;
    if (!sc || !target_name || !target_name[0]) return NULL;
    for (c = 0; c < sc->chain_count; c++) {
        physx_chain_t *chain = &sc->chains[c];
        if (!chain->addon_chain) continue;
        for (t = 0; t < chain->target_count; t++) {
            physx_target_t *target = &chain->targets[t];
            if (!target->addon_simulated_target) continue;
            if (_stricmp(target->name, target_name) != 0) continue;
            if (chain_out) *chain_out = chain;
            if (target_index_out) *target_index_out = t;
            return target;
        }
    }
    return NULL;
}

static int addon_sidecar_section_is_target_override(physx_sidecar_t *sc,
                                                    const char *section,
                                                    const char *target_name)
{
    if (!sc || !section || !target_name || !target_name[0]) return 0;
    if (!addon_sidecar_find_chain_target(sc, target_name, NULL, NULL)) {
        return 0;
    }
    return !addon_sidecar_section_has_chain_root_keys(section, sc->path);
}

static void addon_sidecar_apply_target_joint_override(
    physx_sidecar_t *sc,
    const char *section,
    const char *target_name)
{
    physx_chain_t *chain = NULL;
    physx_target_t *target;
    float axis_sign;
    float gravity_h_tail_sign = 1.0f;
    float gravity_v_tail_sign = 1.0f;
    int target_index = -1;
    int axis;
    int gain_overridden = 0;
    int min_overridden = 0;
    int max_overridden = 0;
    int scalar_limit_overridden = 0;
    int gravity_overridden = 0;
    int gravity_h_tail_overridden = 0;
    int gravity_v_tail_overridden = 0;
    if (!sc || !section || !target_name || !target_name[0]) return;
    target = addon_sidecar_find_chain_target(sc, target_name, &chain,
                                             &target_index);
    if (!target || !chain) return;
    if (!target->joint_settings_initialized) {
        target->joint_settings_initialized = 1;
        target->joint_gain = chain->joint_gain;
        target->limit_angle = chain->limit_angle;
        for (axis = 0; axis < 3; axis++) {
            target->joint_min_angle[axis] = chain->joint_min_angle[axis];
            target->joint_max_angle[axis] = chain->joint_max_angle[axis];
        }
    }
    if (!target->gravity_settings_initialized) {
        target->gravity_settings_initialized = 1;
        target->gravity_horizontal_source_axis =
            chain->gravity_horizontal_source_axis;
        target->gravity_horizontal_source_sign =
            chain->gravity_horizontal_source_sign;
        target->gravity_horizontal_tail_axis =
            chain->gravity_horizontal_tail_axis;
        target->gravity_horizontal_tail_axis_explicit =
            chain->gravity_horizontal_tail_axis_explicit;
        target->gravity_horizontal_scale =
            chain->gravity_horizontal_scale;
        target->gravity_vertical_source_axis =
            chain->gravity_vertical_source_axis;
        target->gravity_vertical_source_sign =
            chain->gravity_vertical_source_sign;
        target->gravity_vertical_tail_axis =
            chain->gravity_vertical_tail_axis;
        target->gravity_vertical_tail_axis_explicit =
            chain->gravity_vertical_tail_axis_explicit;
        target->gravity_vertical_scale =
            chain->gravity_vertical_scale;
        target->gravity_inverted_configured =
            chain->gravity_inverted_configured;
        target->gravity_inverted_strength =
            chain->gravity_inverted_strength;
        target->gravity_inverted_tail_axis =
            chain->gravity_inverted_tail_axis;
        target->gravity_inverted_sign =
            chain->gravity_inverted_sign;
    }
    if (sidecar_profile_key_exists(section, "joint_gain", sc->path)) {
        target->joint_gain = profile_float(section, "joint_gain",
                                           target->joint_gain, sc->path);
        gain_overridden = 1;
    }
    if (sidecar_profile_key_exists(section, "limit_angle", sc->path)) {
        target->limit_angle = profile_float(section, "limit_angle",
                                            target->limit_angle, sc->path);
        scalar_limit_overridden = 1;
    }
    min_overridden =
        parse_angle_alias_vec3_or_scalar(section,
                                         "limit_min_angle",
                                         "joint_min_angle",
                                         -target->limit_angle,
                                         target->joint_min_angle,
                                         sc->path);
    max_overridden =
        parse_angle_alias_vec3_or_scalar(section,
                                         "limit_max_angle",
                                         "joint_max_angle",
                                         target->limit_angle,
                                         target->joint_max_angle,
                                         sc->path);
    if (scalar_limit_overridden && !min_overridden) {
        target->joint_min_angle[0] = -target->limit_angle;
        target->joint_min_angle[1] = -target->limit_angle;
        target->joint_min_angle[2] = -target->limit_angle;
    }
    if (scalar_limit_overridden && !max_overridden) {
        target->joint_max_angle[0] = target->limit_angle;
        target->joint_max_angle[1] = target->limit_angle;
        target->joint_max_angle[2] = target->limit_angle;
    }
    if (min_overridden || max_overridden || scalar_limit_overridden) {
        float limit_from_min = max_abs_vec3_value(target->joint_min_angle);
        float limit_from_max = max_abs_vec3_value(target->joint_max_angle);
        target->limit_angle = limit_from_min;
        if (limit_from_max > target->limit_angle) {
            target->limit_angle = limit_from_max;
        }
    }
    if (sidecar_profile_key_exists(section, "gravity_horizontal_source_axis", sc->path)) {
        target->gravity_horizontal_source_axis =
            sidecar_profile_signed_axis(
                section, "gravity_horizontal_source_axis",
                target->gravity_horizontal_source_axis,
                &target->gravity_horizontal_source_sign,
                sc->path);
        gravity_overridden = 1;
    }
    if (sidecar_profile_key_exists(section, "gravity_horizontal_tail_axis", sc->path)) {
        target->gravity_horizontal_tail_axis =
            sidecar_profile_signed_axis(
                section, "gravity_horizontal_tail_axis",
                target->gravity_horizontal_tail_axis,
                &axis_sign,
                sc->path);
        target->gravity_horizontal_tail_axis_explicit = 1;
        gravity_h_tail_sign = axis_sign;
        gravity_h_tail_overridden = 1;
        gravity_overridden = 1;
    }
    if (sidecar_profile_key_exists(section, "gravity_horizontal_scale", sc->path)) {
        target->gravity_horizontal_scale =
            profile_float(section, "gravity_horizontal_scale",
                          target->gravity_horizontal_scale, sc->path);
        gravity_overridden = 1;
    }
    if (sidecar_profile_key_exists(section, "gravity_vertical_source_axis", sc->path)) {
        target->gravity_vertical_source_axis =
            sidecar_profile_signed_axis(
                section, "gravity_vertical_source_axis",
                target->gravity_vertical_source_axis,
                &target->gravity_vertical_source_sign,
                sc->path);
        gravity_overridden = 1;
    }
    if (sidecar_profile_key_exists(section, "gravity_vertical_tail_axis", sc->path)) {
        target->gravity_vertical_tail_axis =
            sidecar_profile_signed_axis(
                section, "gravity_vertical_tail_axis",
                target->gravity_vertical_tail_axis,
                &axis_sign,
                sc->path);
        target->gravity_vertical_tail_axis_explicit = 1;
        gravity_v_tail_sign = axis_sign;
        gravity_v_tail_overridden = 1;
        gravity_overridden = 1;
    }
    if (sidecar_profile_key_exists(section, "gravity_vertical_scale", sc->path)) {
        target->gravity_vertical_scale =
            profile_float(section, "gravity_vertical_scale",
                          target->gravity_vertical_scale, sc->path);
        gravity_overridden = 1;
    }
    if (sidecar_profile_key_exists(section, "gravity_inverted_strength",
                                   sc->path) ||
        sidecar_profile_key_exists(section, "gravity_inverted_tail_axis",
                                   sc->path) ||
        sidecar_profile_key_exists(section, "gravity_inverted_sign",
                                   sc->path)) {
        target->gravity_inverted_configured = 1;
        gravity_overridden = 1;
    }
    if (sidecar_profile_key_exists(section, "gravity_inverted_strength",
                                   sc->path)) {
        target->gravity_inverted_strength = physx_clampf(
            profile_float(section, "gravity_inverted_strength",
                          target->gravity_inverted_strength, sc->path),
            0.0f, 120.0f);
    }
    if (sidecar_profile_key_exists(section, "gravity_inverted_tail_axis",
                                   sc->path)) {
        target->gravity_inverted_tail_axis = sidecar_profile_axis(
            section, "gravity_inverted_tail_axis",
            target->gravity_inverted_tail_axis, sc->path);
    }
    if (sidecar_profile_key_exists(section, "gravity_inverted_sign",
                                   sc->path)) {
        target->gravity_inverted_sign = physx_clampf(
            profile_float(section, "gravity_inverted_sign",
                          target->gravity_inverted_sign, sc->path),
            -1.0f, 1.0f);
    }
    if (gravity_h_tail_overridden) {
        target->gravity_horizontal_scale *= gravity_h_tail_sign;
    }
    if (gravity_v_tail_overridden) {
        target->gravity_vertical_scale *= gravity_v_tail_sign;
    }
    if (target->gravity_inverted_configured) {
        log_line("addon sidecar target inverted gravity override chain=\"%s\" target=\"%s\" strength=%.3f tail_axis=%d sign=%.3f section=\"%s\" sidecar=\"%s\"",
                 chain->name,
                 target->name,
                 target->gravity_inverted_strength,
                 target->gravity_inverted_tail_axis,
                 target->gravity_inverted_sign,
                 section,
                 sc->path);
    }
    log_line("addon sidecar target override applied chain=\"%s\" target=\"%s\" target_index=%d gain=%.3f limits_min=(%.1f,%.1f,%.1f) limits_max=(%.1f,%.1f,%.1f) gravity_h=(source_axis=%d source_sign=%.1f tail_axis=%d tail_explicit=%d scale=%.3f) gravity_v=(source_axis=%d source_sign=%.1f tail_axis=%d tail_explicit=%d scale=%.3f) section=\"%s\" sidecar=\"%s\" note=\"section matched an existing child/root target, so it was used as per-bone tuning instead of creating a new chain\"",
             chain->name,
             target->name,
             target_index,
             target->joint_gain,
             target->joint_min_angle[0],
             target->joint_min_angle[1],
             target->joint_min_angle[2],
             target->joint_max_angle[0],
             target->joint_max_angle[1],
             target->joint_max_angle[2],
             target->gravity_horizontal_source_axis,
             target->gravity_horizontal_source_sign,
             target->gravity_horizontal_tail_axis,
             target->gravity_horizontal_tail_axis_explicit,
             target->gravity_horizontal_scale,
             target->gravity_vertical_source_axis,
             target->gravity_vertical_source_sign,
             target->gravity_vertical_tail_axis,
             target->gravity_vertical_tail_axis_explicit,
             target->gravity_vertical_scale,
             section,
             sc->path);
    (void)gain_overridden;
    (void)gravity_overridden;
}

static void addon_sidecar_apply_target_joint_overrides(physx_sidecar_t *sc,
                                                       const char *sections)
{
    const char *s;
    if (!sc || !sections) return;
    for (s = sections; *s; s += strlen(s) + 1) {
        const char *target_name;
        if (_strnicmp(s, "NC-TK17-PhysX:", 14) != 0) continue;
        target_name = strchr(s, ':');
        if (!target_name || !target_name[1]) continue;
        target_name++;
        if (addon_sidecar_section_has_chain_root_keys(s, sc->path)) continue;
        if (addon_sidecar_find_chain_target(sc, target_name, NULL, NULL)) {
            addon_sidecar_apply_target_joint_override(sc, s, target_name);
        } else if (!addon_sidecar_section_has_chain_keys(s, sc->path)) {
            log_line("addon sidecar target override ignored section=\"%s\" target=\"%s\" sidecar=\"%s\" reason=\"not-found-in-any-chain\" note=\"tuning-only sections must name a bone already listed by a parent/children chain\"",
                     s,
                     target_name,
                     sc->path);
        }
    }
}

static void reset_addon_tjoint_rotation_rest(physx_target_t *target)
{
    if (!target) return;
    target->sim_addon_rotation_rest_valid = 0;
    target->sim_addon_rotation_rest[0] = 0.0f;
    target->sim_addon_rotation_rest[1] = 0.0f;
    target->sim_addon_rotation_rest[2] = 0.0f;
}

static void reset_addon_sjoint_orientation_rest(physx_target_t *target)
{
    if (!target) return;
    target->addon_joint_orientation_valid = 0;
    target->addon_joint_orientation[0] = 0.0f;
    target->addon_joint_orientation[1] = 0.0f;
    target->addon_joint_orientation[2] = 0.0f;
}

static float *addon_tjoint_rotation_ptr(physx_target_t *target)
{
    float *rv;
    if (!target ||
        !target->addon_rotation_base ||
        target->addon_rotation_offset < 0) {
        return NULL;
    }
    rv = (float*)((BYTE*)target->addon_rotation_base +
                  target->addon_rotation_offset);
    if (!ptr_readable(rv, sizeof(float) * 3)) return NULL;
    if (!physx_vec3_sane_limit(rv, 720.0f)) return NULL;
    return rv;
}

static void capture_addon_tjoint_rotation_rest(physx_target_t *target,
                                               float *rv)
{
    if (!target || !rv || !ptr_readable(rv, sizeof(float) * 3)) return;
    target->sim_addon_rotation_rest[0] = rv[0];
    target->sim_addon_rotation_rest[1] = rv[1];
    target->sim_addon_rotation_rest[2] = rv[2];
    target->sim_addon_rotation_rest_valid = 1;
}

static void addon_tjoint_rotation_from_visual(physx_target_t *target,
                                              const float visual_rotation[3],
                                              float out[3])
{
    if (!target || !visual_rotation || !out) return;
    if (!target->sim_addon_rotation_rest_valid) {
        float *rv = addon_tjoint_rotation_ptr(target);
        if (rv) capture_addon_tjoint_rotation_rest(target, rv);
    }
    if (target->sim_addon_rotation_rest_valid) {
        out[0] = target->sim_addon_rotation_rest[0] +
                 (visual_rotation[0] - target->sim_rotation_rest[0]);
        out[1] = target->sim_addon_rotation_rest[1] +
                 (visual_rotation[1] - target->sim_rotation_rest[1]);
        out[2] = target->sim_addon_rotation_rest[2] +
                 (visual_rotation[2] - target->sim_rotation_rest[2]);
    } else {
        out[0] = visual_rotation[0];
        out[1] = visual_rotation[1];
        out[2] = visual_rotation[2];
    }
}

static int assume_addon_t_joint_rotation_layout(physx_target_t *target,
                                                const char *sidecar_path)
{
    BYTE *bases[2];
    const char *labels[2];
    float expected_t[3];
    float expected_r[3];
    int has_t = 0;
    int has_r = 0;
    int bi;
    if (!target || (!target->object && !target->raw_object)) return 0;
    if (target->addon_rotation_base && target->addon_rotation_offset >= 0) return 1;
    if (target->addon_rotation_offset == ADDON_TJOINT_ROTATION_UNAVAILABLE) return 0;
    read_addon_sjoint_scene_pose(sidecar_path, target->name,
                                 expected_t, &has_t,
                                 expected_r, &has_r);
    if (!has_r) {
        target->addon_rotation_offset = ADDON_TJOINT_ROTATION_UNAVAILABLE;
        return 0;
    }
    bases[0] = (BYTE*)target->raw_object;
    bases[1] = (BYTE*)target->object;
    labels[0] = "raw";
    labels[1] = "object";
    for (bi = 0; bi < 2; bi++) {
        BYTE *base = bases[bi];
        int best_off = -1;
        int r_off;
        float best_v[3] = { 0.0f, 0.0f, 0.0f };
        float best_err = 0.0f;
        if (!base || !ptr_readable(base, 0x1000)) continue;
        r_off = find_vector3_offset(base, expected_r, 0.01f,
                                    &best_off, best_v, &best_err);
        if (r_off >= 0 && ptr_readable(base + r_off, sizeof(float) * 3)) {
            float *r = (float*)(base + r_off);
            target->addon_rotation_base = base;
            target->addon_rotation_source = labels[bi];
            target->addon_rotation_offset = r_off;
            reset_addon_tjoint_rotation_rest(target);
            log_line("target addon t-joint rotation layout target=\"%s\" source=%s base=%p rotation_offset=0x%03x rotation=(%.5f,%.5f,%.5f) reason=\"custom addon TJoint also contains the scene JointOrientation; driving both possible live-bone rotation channels\"",
                     target->name, labels[bi], base, r_off,
                     r[0], r[1], r[2]);
            return 1;
        }
        if (best_off >= 0) {
            log_line("target addon t-joint rotation probe nearest target=\"%s\" source=%s base=%p best_offset=0x%03x err=%.5f nearest=(%.5f,%.5f,%.5f) expected=(%.5f,%.5f,%.5f)",
                     target->name, labels[bi], base, best_off, best_err,
                     best_v[0], best_v[1], best_v[2],
                     expected_r[0], expected_r[1], expected_r[2]);
        }
    }
    target->addon_rotation_offset = ADDON_TJOINT_ROTATION_UNAVAILABLE;
    log_line("target addon t-joint rotation unavailable target=\"%s\" sidecar=\"%s\" note=\"no exact TJoint JointOrientation slot found; caching this miss so the live solver does not re-scan and spam logs every frame\"",
             target->name,
             sidecar_path ? sidecar_path : "");
    return 0;
}

static void mark_addon_transform_dirty(void *object,
                                       const char *target_name,
                                       const char *source)
{
    enum { SLOT_COUNT = 96 };
    enum { DIRTY_THROTTLE_MS = 16 };
    static struct {
        void *object;
        DWORD tick;
        unsigned int version;
    } slots[SLOT_COUNT];
    static int dirty_log_count;
    DWORD now;
    unsigned int version;
    int i;
    int slot = -1;
    if (!object ||
        !engine_TBaseTransformGetMatrixVersion ||
        !engine_TBaseTransformSetMatrixVersion ||
        !ptr_readable(object, sizeof(void*))) {
        return;
    }
    now = GetTickCount();
    for (i = 0; i < SLOT_COUNT; i++) {
        if (slots[i].object == object) {
            slot = i;
            break;
        }
        if (!slots[i].object && slot < 0) slot = i;
    }
    if (slot < 0) slot = (int)(now % SLOT_COUNT);
    if (slots[slot].object == object &&
        now - slots[slot].tick < (DWORD)DIRTY_THROTTLE_MS) {
        return;
    }
    version = engine_TBaseTransformGetMatrixVersion(object);
    engine_TBaseTransformSetMatrixVersion(object, version + 1u);
    slots[slot].object = object;
    slots[slot].tick = now;
    slots[slot].version = version + 1u;
    if (dirty_log_count < 80) {
        dirty_log_count++;
        log_line("addon transform dirty target=\"%s\" source=\"%s\" object=%p old_version=%u new_version=%u throttle_ms=%d note=\"invalidating TK17 cached transform after add-on PhysX write\"",
                 target_name ? target_name : "",
                 source ? source : "",
                 object,
                 version,
                 version + 1u,
                 DIRTY_THROTTLE_MS);
    }
}

static void addon_target_reset_write_guard(physx_target_t *target)
{
    if (!target) return;
    target->addon_write_guard_object = NULL;
    target->addon_write_guard_raw_object = NULL;
    target->addon_write_guard_s_object = NULL;
    target->addon_write_guard_s_rotation_base = NULL;
    target->addon_write_guard_s_translation_base = NULL;
    target->addon_write_guard_s_rotation_offset = -1;
    target->addon_write_guard_s_translation_offset = -1;
    target->addon_write_guard_samples = 0;
    target->addon_write_guard_first_tick = 0;
    target->addon_write_guard_last_sample_tick = 0;
    target->addon_write_guard_ready = 0;
}

static int addon_target_write_mapping_matches_guard(
    const physx_target_t *target)
{
    if (!target || !target->addon_write_guard_ready) return 0;
    return target->object == target->addon_write_guard_object &&
           target->raw_object == target->addon_write_guard_raw_object &&
           target->s_object == target->addon_write_guard_s_object &&
           target->s_rotation_base ==
               target->addon_write_guard_s_rotation_base &&
           target->s_translation_base ==
               target->addon_write_guard_s_translation_base &&
           target->s_rotation_offset ==
               target->addon_write_guard_s_rotation_offset &&
           target->s_translation_offset ==
               target->addon_write_guard_s_translation_offset;
}

static int assume_addon_s_transform_layout(physx_target_t *target,
                                           const char *sidecar_path);

static int addon_target_sample_write_guard(physx_target_t *target,
                                           DWORD now,
                                           int room_scene_target,
                                           const char *sidecar_path)
{
    int mapping_changed;
    int was_ready;
    if (!target || !target->object ||
        is_nil_engine_object(target->raw_object, target->object) ||
        !target->addon_simulated_target) {
        if (target) {
            target->addon_visual_pose_valid = 0;
            addon_target_reset_write_guard(target);
        }
        return 0;
    }
    if (target->s_rotation_base &&
        (((target->s_rotation_offset != 0x038 ||
           target->s_translation_offset != 0x048) &&
          (!room_scene_target ||
           target->s_rotation_offset != 0x06c ||
           target->s_translation_offset != 0x07c)) ||
         target->s_translation_base != target->s_rotation_base)) {
        target->addon_visual_pose_valid = 0;
        addon_target_reset_write_guard(target);
        /* A rejected cached layout must not disable this joint forever.
           Retry detection, never relax the accepted write offsets. Keep the
           retry clock outside reset_write_guard so rejection cannot turn a
           scene-pose scan into per-frame work. */
        if (sidecar_path && (!target->addon_layout_retry_pending ||
            now - target->addon_layout_retry_last_tick >= 1000u)) {
            target->addon_layout_retry_pending = 1;
            target->addon_layout_retry_last_tick = now;
            target->sim_initialized = 0;
            log_line("addon safety layout retry target=\"%s\" rotation_offset=0x%03x translation_offset=0x%03x retry_ms=1000 note=\"rechecking rejected mapping; writes remain quarantined\"",
                     target->name, target->s_rotation_offset,
                     target->s_translation_offset);
            assume_addon_s_transform_layout(target, sidecar_path);
        }
        return 0;
    }

    mapping_changed =
        target->object != target->addon_write_guard_object ||
        target->raw_object != target->addon_write_guard_raw_object ||
        target->s_object != target->addon_write_guard_s_object ||
        target->s_rotation_base != target->addon_write_guard_s_rotation_base ||
        target->s_translation_base !=
            target->addon_write_guard_s_translation_base ||
        target->s_rotation_offset !=
            target->addon_write_guard_s_rotation_offset ||
        target->s_translation_offset !=
            target->addon_write_guard_s_translation_offset;
    if (mapping_changed) {
        was_ready = target->addon_write_guard_ready;
        if (was_ready) {
            log_line("addon safety write quarantined target=\"%s\" reason=\"runtime transform mapping changed\" old_object=%p new_object=%p old_s_object=%p new_s_object=%p note=\"visual output cleared before any rotation write; binding must stabilize again\"",
                     target->name,
                     target->addon_write_guard_object, target->object,
                     target->addon_write_guard_s_object, target->s_object);
        }
        target->addon_write_guard_object = target->object;
        target->addon_write_guard_raw_object = target->raw_object;
        target->addon_write_guard_s_object = target->s_object;
        target->addon_write_guard_s_rotation_base = target->s_rotation_base;
        target->addon_write_guard_s_translation_base =
            target->s_translation_base;
        target->addon_write_guard_s_rotation_offset =
            target->s_rotation_offset;
        target->addon_write_guard_s_translation_offset =
            target->s_translation_offset;
        target->addon_write_guard_samples = 1;
        target->addon_write_guard_first_tick = now;
        target->addon_write_guard_last_sample_tick = now;
        target->addon_write_guard_ready = 0;
        target->addon_visual_pose_valid = 0;
        target->sim_initialized = 0;
        return 0;
    }
    if (target->addon_write_guard_last_sample_tick != now) {
        target->addon_write_guard_last_sample_tick = now;
        if (target->addon_write_guard_samples < 3) {
            target->addon_write_guard_samples++;
        }
    }
    if (target->addon_write_guard_samples >= 3 &&
        now - target->addon_write_guard_first_tick >= 32u) {
        target->addon_write_guard_ready = 1;
        if (target->addon_layout_retry_pending) {
            log_line("addon safety layout recovered target=\"%s\" rotation_offset=0x%03x translation_offset=0x%03x note=\"accepted mapping stabilized before simulation resumed\"",
                     target->name, target->s_rotation_offset,
                     target->s_translation_offset);
            target->addon_layout_retry_pending = 0;
        }
    }
    return target->addon_write_guard_ready;
}

static int addon_object_target_sample_write_guard(physx_target_t *target,
                                                  DWORD now)
{
    int mapping_changed;
    if (!target || !target->object_transform_target ||
        !target->addon_simulated_target ||
        !target->object ||
        is_nil_engine_object(target->raw_object, target->object) ||
        !target->s_raw_object ||
        !target->s_object ||
        is_nil_engine_object(target->s_raw_object, target->s_object) ||
        !real_SSimpleTransform_RotationSet) {
        if (target) addon_target_reset_write_guard(target);
        return 0;
    }
    mapping_changed =
        target->object != target->addon_write_guard_object ||
        target->raw_object != target->addon_write_guard_raw_object ||
        target->s_object != target->addon_write_guard_s_object ||
        target->s_raw_object != target->addon_write_guard_s_rotation_base;
    if (mapping_changed) {
        if (target->addon_write_guard_ready) {
            log_line("addon object safety write quarantined target=\"%s\" reason=\"runtime TTransform/STransform mapping changed\" old_object=%p new_object=%p old_s_object=%p new_s_object=%p note=\"native rotation output waits for the replacement scene objects to stabilize\"",
                     target->name,
                     target->addon_write_guard_object, target->object,
                     target->addon_write_guard_s_object, target->s_object);
        }
        addon_target_reset_write_guard(target);
        target->addon_write_guard_object = target->object;
        target->addon_write_guard_raw_object = target->raw_object;
        target->addon_write_guard_s_object = target->s_object;
        target->addon_write_guard_s_rotation_base = target->s_raw_object;
        target->addon_write_guard_samples = 1;
        target->addon_write_guard_first_tick = now;
        target->addon_write_guard_last_sample_tick = now;
        target->sim_initialized = 0;
        target->object_output_applied = 0;
        memcpy(target->object_proxy_translation,
               target->object_proxy_rest,
               sizeof(target->object_proxy_translation));
        memcpy(target->object_output_rotation,
               target->object_scene_rotation,
               sizeof(target->object_output_rotation));
        return 0;
    }
    if (target->addon_write_guard_last_sample_tick != now) {
        target->addon_write_guard_last_sample_tick = now;
        if (target->addon_write_guard_samples < 3) {
            target->addon_write_guard_samples++;
        }
    }
    if (target->addon_write_guard_samples >= 3 &&
        now - target->addon_write_guard_first_tick >= 32u) {
        target->addon_write_guard_ready = 1;
    }
    return target->addon_write_guard_ready;
}

static void addon_object_write_bone_solver_rotation(
    physx_target_t *target,
    const float rotation[3])
{
    void *receiver;
    if (!target || !rotation ||
        !real_SSimpleTransform_RotationSet) {
        return;
    }
    receiver = target->s_raw_object ? target->s_raw_object : target->s_object;
    if (!receiver || is_nil_engine_object(target->s_raw_object,
                                          target->s_object)) {
        return;
    }
    addon_object_bone_limit_write_thread = GetCurrentThreadId();
    InterlockedExchange(&addon_object_bone_limit_write_active, 1);
    real_SSimpleTransform_RotationSet(
        receiver,
        SCRIPT_PROPERTY_SSIMPLE_ROTATION,
        rotation);
    InterlockedExchange(&addon_object_bone_limit_write_active, 0);
    addon_object_bone_limit_write_thread = 0;
}

static int write_addon_sjoint_matrix_rows(physx_target_t *target,
                                          const float rotation_deg[3],
                                          const float translation[3],
                                          int write_translation)
{
    BYTE *base;
    float *row0;
    float *row1;
    float *row2;
    float *row3 = NULL;
    float rotation_rows[9];
    int row2_off;
    int row1_off;
    int row0_off;
    int row3_off;
    static int matrix_log_count;
    if (!target || !rotation_deg || !target->s_rotation_base ||
        !addon_target_write_mapping_matches_guard(target)) return 0;
    if (!target->addon_simulated_target) return 0;
    if (target->s_rotation_offset < 0 ||
        target->s_translation_offset < 0 ||
        target->s_translation_offset != target->s_rotation_offset + 0x10) {
        return 0;
    }
    /* The add-on SJoint contains two different transform representations.
       After traversal, 0x018/0x028/0x038/0x048 are the live matrix rows used
       successfully by PoseEditor. During initial loads in other modes the
       scene-definition Rotation/Translation at 0x06c/0x07c can be found
       first. Treating the bytes preceding that pair as matrix rows overwrites
       unrelated transform/scale state and stretches the skinned hair. Only
       write the confirmed live matrix layout here; otherwise the caller falls
       back to the TJoint matrix rows used by the skin palette. */
    if (target->s_rotation_offset != 0x038 ||
        target->s_translation_offset != 0x048) {
        return 0;
    }
    row2_off = target->s_rotation_offset;
    row1_off = row2_off - 0x10;
    row0_off = row2_off - 0x20;
    row3_off = target->s_translation_offset;
    if (row0_off < 0 || row1_off < 0) return 0;
    base = (BYTE*)target->s_rotation_base;
    if (!ptr_readable(base + row0_off, 0x40)) return 0;

    physx_contact_rotation_rows(rotation_deg, rotation_rows);

    row0 = (float*)(base + row0_off);
    row1 = (float*)(base + row1_off);
    row2 = (float*)(base + row2_off);
    if (write_translation &&
        translation &&
        ptr_readable(base + row3_off, sizeof(float) * 3)) {
        row3 = (float*)(base + row3_off);
    }

    memcpy(row0, rotation_rows, sizeof(float)*3);
    memcpy(row1, rotation_rows+3, sizeof(float)*3);
    memcpy(row2, rotation_rows+6, sizeof(float)*3);

    if (row3) {
        row3[0] = translation[0];
        row3[1] = translation[1];
        row3[2] = translation[2];
    }
    /* Matrix rows are applied before/during UpdateTraverse.  Bumping the
       engine matrix version here races TK17's traversal and spams
       "MatrixVersion updated twice" for custom add-on bones. */

    if (defaults_cfg.debug && matrix_log_count < 80) {
        matrix_log_count++;
        log_line("target addon sjoint matrix write target=\"%s\" base=%p rotation_rows=(0x%03x,0x%03x,0x%03x) translation_row=0x%03x translation_written=%d rotation=(%.5f,%.5f,%.5f) translation=(%.5f,%.5f,%.5f) row0=(%.5f,%.5f,%.5f) row1=(%.5f,%.5f,%.5f) row2=(%.5f,%.5f,%.5f) row3=(%.5f,%.5f,%.5f) note=\"writing the live SJoint matrix basis rows; row2 is the probed SSimpleTransform rotation vector, so this keeps scale at unit length\"",
                 target->name, base,
                 row0_off, row1_off, row2_off, row3_off,
                 row3 ? 1 : 0,
                 rotation_deg[0], rotation_deg[1], rotation_deg[2],
                 translation ? translation[0] : 0.0f,
                 translation ? translation[1] : 0.0f,
                 translation ? translation[2] : 0.0f,
                 row0[0], row0[1], row0[2],
                 row1[0], row1[1], row1[2],
                 row2[0], row2[1], row2[2],
                 row3 ? row3[0] : 0.0f,
                 row3 ? row3[1] : 0.0f,
                 row3 ? row3[2] : 0.0f);
    }
    return 1;
}

static int write_addon_tjoint_matrix_rows(physx_target_t *target,
                                          const float rotation_deg[3],
                                          const float translation[3],
                                          int write_translation)
{
    BYTE *base;
    float *row0;
    float *row1;
    float *row2;
    float *row3 = NULL;
    float rotation_rows[9];
    static int matrix_log_count;
    if (!target || !rotation_deg || !target->object ||
        !addon_target_write_mapping_matches_guard(target)) return 0;
    if (!target->addon_simulated_target) return 0;
    base = (BYTE*)target->object;
    if (!ptr_readable(base + 0x078, 0x30)) return 0;

    physx_contact_rotation_rows(rotation_deg, rotation_rows);

    row0 = (float*)(base + 0x078);
    row1 = (float*)(base + 0x088);
    row2 = (float*)(base + 0x098);
    if (write_translation &&
        translation &&
        ptr_readable(base + 0x0a8, sizeof(float) * 3)) {
        row3 = (float*)(base + 0x0a8);
    }

    memcpy(row0, rotation_rows, sizeof(float)*3);
    memcpy(row1, rotation_rows+3, sizeof(float)*3);
    memcpy(row2, rotation_rows+6, sizeof(float)*3);

    if (row3) {
        row3[0] = translation[0];
        row3[1] = translation[1];
        row3[2] = translation[2];
    }
    /* See SJoint matrix writer: row writes are consumed by traversal without
       forcing a second matrix-version update on the same frame. */

    if (defaults_cfg.debug && matrix_log_count < 80) {
        matrix_log_count++;
        log_line("target addon tjoint matrix write target=\"%s\" base=%p rotation_rows=(0x078,0x088,0x098) translation_row=0x0a8 translation_written=%d rotation=(%.5f,%.5f,%.5f) row0=(%.5f,%.5f,%.5f) row1=(%.5f,%.5f,%.5f) row2=(%.5f,%.5f,%.5f) note=\"skinned add-on mesh is weighted to TJoint entries; visual row write mirrors the built-in body-chain local bone row layout\"",
                 target->name, base,
                 row3 ? 1 : 0,
                 rotation_deg[0], rotation_deg[1], rotation_deg[2],
                 row0[0], row0[1], row0[2],
                 row1[0], row1[1], row1[2],
                 row2[0], row2[1], row2[2]);
    }
    return 1;
}

static int addon_apply_target_visual_pose_in_traverse;

static int physx_addon_apply_target_visual_pose(physx_target_t *target)
{
    int wrote_smatrix = 0;
    int wrote_tmatrix = 0;
    if (!target || !target->addon_visual_pose_valid ||
        !addon_target_write_mapping_matches_guard(target)) return 0;
    if (!target->object || !target->addon_simulated_target) return 0;
    wrote_smatrix = write_addon_sjoint_matrix_rows(
        target,
        target->addon_visual_rotation,
        target->addon_visual_translation,
        target->addon_visual_write_translation);
    if (wrote_smatrix) {
        return 1;
    }
    wrote_tmatrix = write_addon_tjoint_matrix_rows(
        target,
        target->addon_visual_rotation,
        target->addon_visual_translation,
        target->addon_visual_write_translation);
    if (wrote_tmatrix) {
        return 1;
    }
    return 0;
}

static int physx_addon_publish_visual_pose(physx_target_t *target,
                                           const float rotation_deg[3],
                                           int write_translation)
{
    int wrote_immediate;
    if (!target || !rotation_deg) return 0;
    target->addon_visual_pose_valid = 1;
    target->addon_visual_write_translation = write_translation;
    target->addon_visual_tick = GetTickCount();
    target->addon_visual_rotation[0] = rotation_deg[0];
    target->addon_visual_rotation[1] = rotation_deg[1];
    target->addon_visual_rotation[2] = rotation_deg[2];
    if (target->s_translation_base && target->s_translation_offset >= 0 &&
        ptr_readable((BYTE*)target->s_translation_base + target->s_translation_offset,
                     sizeof(float) * 3)) {
        float *tv = (float*)((BYTE*)target->s_translation_base +
                             target->s_translation_offset);
        target->addon_visual_translation[0] = tv[0];
        target->addon_visual_translation[1] = tv[1];
        target->addon_visual_translation[2] = tv[2];
    } else {
        target->addon_visual_translation[0] = target->sim_rest[0];
        target->addon_visual_translation[1] = target->sim_rest[1];
        target->addon_visual_translation[2] = target->sim_rest[2];
    }
    InterlockedExchange(&addon_traverse_overlay_active, 1);
    wrote_immediate = physx_addon_apply_target_visual_pose(target);
    return wrote_immediate;
}

static int physx_addon_apply_traverse_overlay(void *object, int allow_global)
{
    int i, c, t;
    int applied = 0;
    static int log_count;
    static DWORD last_global_tick;
    DWORD now = GetTickCount();
    if (!InterlockedCompareExchange(&addon_traverse_overlay_active, 0, 0)) return 0;
    if (!object || is_nil_engine_object(NULL, object)) {
        object = NULL;
    }
    if (allow_global) {
        if (last_global_tick == now) {
            InterlockedExchange(&addon_traverse_overlay_active, 0);
            return 0;
        }
        last_global_tick = now;
    }
    for (i = 0; i < sidecar_count; i++) {
        physx_sidecar_t *sc = &sidecars[i];
        if (!sc->enabled || !sc->loaded) continue;
        for (c = 0; c < sc->chain_count; c++) {
            physx_chain_t *chain = &sc->chains[c];
            if (!chain->addon_chain || !chain->skinned_matrix_enabled) continue;
            if (!chain->addon_scene_visible) continue;
            for (t = 0; t < chain->target_count; t++) {
                physx_target_t *target = &chain->targets[t];
                if (!target->addon_visual_pose_valid) continue;
                if (!allow_global &&
                    target->object != object &&
                    target->raw_object != object) {
                    continue;
                }
                int pose_applied;
                addon_apply_target_visual_pose_in_traverse = 1;
                pose_applied = physx_addon_apply_target_visual_pose(target);
                addon_apply_target_visual_pose_in_traverse = 0;
                if (pose_applied) {
                    applied++;
                    if (log_count < 80) {
                        log_count++;
                        log_line("addon traverse-overlay applied mode=\"%s\" chain=\"%s\" target=\"%s\" traverse_object=%p target_object=%p rotation=(%.5f,%.5f,%.5f) translation=(%.5f,%.5f,%.5f) write_translation=%d note=\"PhysX pose was overlaid during Bionic::UpdateTraverse; global mode is used when TK17 traverses a parent/root instead of the leaf custom bone pointer\"",
                                 allow_global ? "global" : "exact",
                                 chain->name, target->name, object,
                                 target->object,
                                 target->addon_visual_rotation[0],
                                 target->addon_visual_rotation[1],
                                 target->addon_visual_rotation[2],
                                 target->addon_visual_translation[0],
                                 target->addon_visual_translation[1],
                                 target->addon_visual_translation[2],
                                 target->addon_visual_write_translation);
                    }
                }
            }
        }
    }
    if (allow_global) {
        InterlockedExchange(&addon_traverse_overlay_active, 0);
    }
    return applied;
}

static int addon_chain_target_name_in_line(physx_chain_t *chain,
                                           const char *line)
{
    int i;
    if (!chain || !line) return 0;
    for (i = 0; i < chain->target_count; i++) {
        char local_name[192];
        physx_target_t *target = &chain->targets[i];
        if (!target->name[0] || !target->addon_simulated_target) continue;
        if (contains_i(line, target->name)) return 1;
        _snprintf(local_name, sizeof(local_name), "local_%s", target->name);
        if (contains_i(line, local_name)) return 1;
    }
    return 0;
}

static int collect_addon_skin_mesh_names(const char *sidecar_path,
                                         physx_chain_t *chain,
                                         char names[][128],
                                         int max_names)
{
    char scene_path[MAX_PATH * 4];
    FILE *f;
    char line[2048];
    int count = 0;
    int in_skin = 0;
    int relevant = 0;
    char object_name[128];
    if (!sidecar_path || !chain || !names || max_names <= 0) return 0;
    if (!build_adjacent_scene_path_from_sidecar_a(sidecar_path, scene_path, sizeof(scene_path))) return 0;
    f = fopen(scene_path, "rb");
    if (!f) return 0;
    object_name[0] = 0;
    while (fgets(line, sizeof(line), f)) {
        char *p;
        trim_in_place(line);
        if (!in_skin) {
            if (strncmp(line, "TSkinPolygonGeometry ", 21) == 0 ||
                strncmp(line, "TSkinPolygonGeometry:", 21) == 0) {
                in_skin = 1;
                relevant = 0;
                object_name[0] = 0;
            }
            continue;
        }
        if (strstr(line, "TSkinPolygonGeometry.Joint") &&
            addon_chain_target_name_in_line(chain, line)) {
            relevant = 1;
        }
        p = strstr(line, "Object.Name \"");
        if (p) {
            char *q;
            p += strlen("Object.Name \"");
            q = strchr(p, '"');
            if (q) {
                size_t n = (size_t)(q - p);
                if (n >= sizeof(object_name)) n = sizeof(object_name) - 1;
                memcpy(object_name, p, n);
                object_name[n] = 0;
            }
        }
        if (strncmp(line, "};", 2) == 0) {
            if (relevant && object_name[0] && count < max_names) {
                int dup = 0;
                int i;
                for (i = 0; i < count; i++) {
                    if (_stricmp(names[i], object_name) == 0) {
                        dup = 1;
                        break;
                    }
                }
                if (!dup) {
                    lstrcpynA(names[count], object_name, 128);
                    count++;
                }
            }
            in_skin = 0;
            relevant = 0;
            object_name[0] = 0;
        }
    }
    fclose(f);
    return count;
}

static int addon_name_matches_alias(const char *name, const char *plain);

static void normalize_addon_joint_ref(const char *token,
                                      char *out,
                                      size_t outsz)
{
    char tmp[256];
    char *p;
    char *end;
    if (!out || outsz == 0) return;
    out[0] = 0;
    if (!token || !token[0]) return;
    lstrcpynA(tmp, token, sizeof(tmp));
    trim_in_place(tmp);
    p = strchr(tmp, ':');
    p = p ? p + 1 : tmp;
    trim_in_place(p);
    if (_strnicmp(p, "local_", 6) == 0) p += 6;
    end = p;
    while (*end && *end != ' ' && *end != '\t' &&
           *end != ',' && *end != ']' && *end != '[' &&
           *end != ';' && *end != '\r' && *end != '\n') {
        end++;
    }
    *end = 0;
    if (p[0]) lstrcpynA(out, p, outsz);
}

static int parse_addon_skin_joint_line(const char *line,
                                       char joints[][128],
                                       int max_joints)
{
    char buf[2048];
    char *p;
    char *token;
    int count = 0;
    if (!line || !joints || max_joints <= 0) return 0;
    lstrcpynA(buf, line, sizeof(buf));
    p = strchr(buf, '[');
    if (!p) return 0;
    p++;
    token = strtok(p, ",]");
    while (token && count < max_joints) {
        char name[128];
        normalize_addon_joint_ref(token, name, sizeof(name));
        if (name[0]) {
            lstrcpynA(joints[count], name, 128);
            count++;
        }
        token = strtok(NULL, ",]");
    }
    return count;
}

static int collect_addon_skin_joint_order(const char *sidecar_path,
                                          const char *mesh_name,
                                          char joints[][128],
                                          int max_joints)
{
    char scene_path[MAX_PATH * 4];
    FILE *f;
    char line[2048];
    char object_name[128];
    char joint_line[2048];
    int in_skin = 0;
    static int order_log_count;
    if (!sidecar_path || !mesh_name || !mesh_name[0] ||
        !joints || max_joints <= 0) {
        return 0;
    }
    if (!build_adjacent_scene_path_from_sidecar_a(sidecar_path,
                                                  scene_path,
                                                  sizeof(scene_path))) {
        return 0;
    }
    f = fopen(scene_path, "rb");
    if (!f) return 0;
    object_name[0] = 0;
    joint_line[0] = 0;
    while (fgets(line, sizeof(line), f)) {
        char *p;
        trim_in_place(line);
        if (!in_skin) {
            if (strncmp(line, "TSkinPolygonGeometry ", 21) == 0 ||
                strncmp(line, "TSkinPolygonGeometry:", 21) == 0) {
                in_skin = 1;
                object_name[0] = 0;
                joint_line[0] = 0;
            }
            continue;
        }
        if (strstr(line, "TSkinPolygonGeometry.Joint")) {
            lstrcpynA(joint_line, line, sizeof(joint_line));
        }
        p = strstr(line, "Object.Name \"");
        if (p) {
            char *q;
            p += strlen("Object.Name \"");
            q = strchr(p, '"');
            if (q) {
                size_t n = (size_t)(q - p);
                if (n >= sizeof(object_name)) n = sizeof(object_name) - 1;
                memcpy(object_name, p, n);
                object_name[n] = 0;
            }
        }
        if (strncmp(line, "};", 2) == 0) {
            if (object_name[0] && _stricmp(object_name, mesh_name) == 0 &&
                joint_line[0]) {
                int count = parse_addon_skin_joint_line(joint_line,
                                                        joints,
                                                        max_joints);
                fclose(f);
                if (count > 0 && order_log_count < 32) {
                    int i;
                    char list[768];
                    list[0] = 0;
                    for (i = 0; i < count; i++) {
                        size_t used = strlen(list);
                        if (used + strlen(joints[i]) + 3 >= sizeof(list)) break;
                        if (used) strcat(list, ",");
                        strcat(list, joints[i]);
                    }
                    order_log_count++;
                    log_line("addon skin-joint order mesh=\"%s\" joints=\"%s\" count=%d scene=\"%s\" note=\"using sidecar .bs TSkinPolygonGeometry.Joint order to infer unnamed live palette slots\"",
                             mesh_name, list, count, scene_path);
                }
                return count;
            }
            in_skin = 0;
            object_name[0] = 0;
            joint_line[0] = 0;
        }
    }
    fclose(f);
    return 0;
}

static int addon_skin_joint_order_index(char joints[][128],
                                        int joint_count,
                                        const char *name)
{
    int i;
    if (!joints || joint_count <= 0 || !name || !name[0]) return -1;
    for (i = 0; i < joint_count; i++) {
        if (addon_name_matches_alias(name, joints[i])) return i;
    }
    return -1;
}

static void append_probe_offset(char *buf, size_t buf_sz, int *count, int off)
{
    char tmp[32];
    size_t used;
    if (!buf || !buf_sz || !count) return;
    if (*count >= 12) return;
    _snprintf(tmp, sizeof(tmp), "%s0x%03x", (*count > 0) ? "," : "", off);
    used = strlen(buf);
    if (used + strlen(tmp) + 1 < buf_sz) {
        strcat(buf, tmp);
        (*count)++;
    }
}

static void append_probe_indirect(char *buf, size_t buf_sz, int *count,
                                  int outer_off, int inner_off)
{
    char tmp[48];
    size_t used;
    if (!buf || !buf_sz || !count) return;
    if (*count >= 12) return;
    _snprintf(tmp, sizeof(tmp), "%s0x%03x->0x%03x",
              (*count > 0) ? "," : "", outer_off, inner_off);
    used = strlen(buf);
    if (used + strlen(tmp) + 1 < buf_sz) {
        strcat(buf, tmp);
        (*count)++;
    }
}

static int scan_pointer_block_for_target(BYTE *base,
                                         size_t scan_bytes,
                                         void *needle,
                                         char *direct_buf,
                                         size_t direct_buf_sz,
                                         int *direct_count)
{
    size_t off;
    int found = 0;
    if (!base || !needle) return 0;
    for (off = 0; off + sizeof(void*) <= scan_bytes; off += sizeof(void*)) {
        void *candidate;
        if (!ptr_readable(base + off, sizeof(void*))) continue;
        candidate = *(void**)(base + off);
        if (candidate == needle) {
            append_probe_offset(direct_buf, direct_buf_sz, direct_count, (int)off);
            found++;
        }
    }
    return found;
}

static int scan_pointer_block_indirect_for_target(BYTE *base,
                                                  size_t scan_bytes,
                                                  void *needle,
                                                  char *indirect_buf,
                                                  size_t indirect_buf_sz,
                                                  int *indirect_count)
{
    size_t off;
    int found = 0;
    if (!base || !needle) return 0;
    for (off = 0; off + sizeof(void*) <= scan_bytes; off += sizeof(void*)) {
        void *candidate;
        size_t inner_scan = 0x400;
        size_t inner;
        if (!ptr_readable(base + off, sizeof(void*))) continue;
        candidate = *(void**)(base + off);
        if (!candidate || candidate == base) continue;
        if (!ptr_readable(candidate, inner_scan)) continue;
        for (inner = 0; inner + sizeof(void*) <= inner_scan; inner += sizeof(void*)) {
            void *inner_candidate;
            if (!ptr_readable((BYTE*)candidate + inner, sizeof(void*))) continue;
            inner_candidate = *(void**)((BYTE*)candidate + inner);
            if (inner_candidate == needle) {
                append_probe_indirect(indirect_buf, indirect_buf_sz,
                                      indirect_count, (int)off, (int)inner);
                found++;
                break;
            }
        }
    }
    return found;
}

static int assume_addon_s_transform_layout(physx_target_t *target,
                                           const char *sidecar_path);
static int addon_output_scene_visible(void);
static int addon_chain_scene_visible(physx_sidecar_t *sc,
                                     physx_chain_t *chain,
                                     DWORD now,
                                     int *runtime_fallback_out,
                                     int *live_targets_out,
                                     int *writable_targets_out);

static int addon_name_matches_alias(const char *name, const char *plain)
{
    char local_name[192];
    if (!name || !plain || !name[0] || !plain[0]) return 0;
    if (_stricmp(name, plain) == 0) return 1;
    _snprintf(local_name, sizeof(local_name), "local_%s", plain);
    return _stricmp(name, local_name) == 0;
}

static const char *addon_named_node_name_for_object(void *object)
{
    int i;
    if (!object) return NULL;
    for (i = named_node_count - 1; i >= 0; i--) {
        if (named_nodes[i].object == object &&
            named_nodes[i].name[0]) {
            return named_nodes[i].name;
        }
    }
    return NULL;
}

static physx_target_t *addon_chain_target_for_named_object(physx_chain_t *chain,
                                                           const char *name)
{
    int t;
    if (!chain || !name || !name[0]) return NULL;
    for (t = 0; t < chain->target_count; t++) {
        physx_target_t *target = &chain->targets[t];
        if (!target->addon_simulated_target) continue;
        if (addon_name_matches_alias(name, target->name)) return target;
    }
    return NULL;
}

static int addon_chain_count_physx_targets(physx_chain_t *chain,
                                           int *bound_out)
{
    int t;
    int total = 0;
    int bound = 0;
    if (bound_out) *bound_out = 0;
    if (!chain) return 0;
    for (t = 0; t < chain->target_count; t++) {
        physx_target_t *target = &chain->targets[t];
        if (!target->addon_simulated_target) continue;
        total++;
        if (target->addon_skin_bound &&
            target->object &&
            !is_nil_engine_object(target->raw_object, target->object)) {
            bound++;
        }
    }
    if (bound_out) *bound_out = bound;
    return total;
}

static int addon_chain_all_physx_targets_skin_bound(physx_chain_t *chain)
{
    int bound = 0;
    int total = addon_chain_count_physx_targets(chain, &bound);
    return total > 0 && bound >= total;
}

static int addon_chain_all_physx_targets_live_sjoint(physx_chain_t *chain)
{
    int t;
    int total = 0;
    if (!chain) return 0;
    for (t = 0; t < chain->target_count; t++) {
        physx_target_t *target = &chain->targets[t];
        if (!target->addon_simulated_target) continue;
        total++;
        if (!target->s_raw_object ||
            target->s_rotation_base != target->s_raw_object ||
            target->s_translation_base != target->s_raw_object ||
            target->s_rotation_offset != 0x038 ||
            target->s_translation_offset != 0x048) {
            return 0;
        }
    }
    return total > 0;
}

static int addon_chain_log_automated_test_done(physx_sidecar_t *sc,
                                               physx_chain_t *chain,
                                               DWORD now,
                                               const char *result)
{
    int bound = 0;
    int total;
    DWORD elapsed = 0;
    if (!sc || !chain || chain->addon_test_done_logged) return 0;
    total = addon_chain_count_physx_targets(chain, &bound);
    if (chain->addon_test_start_tick) elapsed = now - chain->addon_test_start_tick;
    chain->addon_test_done_logged = 1;
    log_line("============================================================");
    log_line("================ ADDON AUTOMATED TEST DONE!!!!! =============");
    log_line("DONE!!!!! ADDON AUTOMATED TEST DONE!!!!! chain=\"%s\" result=\"%s\" bound=%d/%d probes=%d elapsed_ms=%lu",
             chain->name,
             result ? result : "",
             bound,
             total,
             chain->skin_probe_count,
             (unsigned long)elapsed);
    if (result && strcmp(result, "live-sjoint-output") == 0) {
        log_line("addon automated-test done chain=\"%s\" result=\"live-sjoint-output\" bound=%d/%d probes=%d elapsed_ms=%lu sidecar=\"%s\" note=\"all simulated targets use confirmed raw SJoint matrix rows; the legacy skin-palette audit was skipped because it is not the active output path\"",
                 chain->name,
                 bound,
                 total,
                 chain->skin_probe_count,
                 (unsigned long)elapsed,
                 sc->path);
    } else {
        log_line("addon automated-test done chain=\"%s\" result=\"%s\" bound=%d/%d probes=%d elapsed_ms=%lu sidecar=\"%s\" note=\"skin-palette audit is done; regular add-on PhysX simulation continues separately while the room is visible\"",
                 chain->name,
                 result ? result : "",
                 bound,
                 total,
                 chain->skin_probe_count,
                 (unsigned long)elapsed,
                 sc->path);
    }
    log_line("============================================================");
    return 1;
}

static void *find_named_pointer_alias_in_block(void *base,
                                               size_t scan_bytes,
                                               const char *plain_name,
                                               int *offset_out,
                                               const char **matched_name_out)
{
    size_t off;
    if (offset_out) *offset_out = -1;
    if (matched_name_out) *matched_name_out = NULL;
    if (!base || !plain_name || !plain_name[0]) return NULL;
    for (off = 0; off + sizeof(void*) <= scan_bytes; off += sizeof(void*)) {
        void *candidate;
        const char *name;
        if (!ptr_readable((BYTE*)base + off, sizeof(void*))) continue;
        candidate = *(void**)((BYTE*)base + off);
        if (!candidate || candidate == base) continue;
        name = addon_named_node_name_for_object(candidate);
        if (name && addon_name_matches_alias(name, plain_name)) {
            if (offset_out) *offset_out = (int)off;
            if (matched_name_out) *matched_name_out = name;
            return candidate;
        }
    }
    return NULL;
}

static void *find_addon_sjoint_for_mesh_joint(physx_target_t *target,
                                              int *direct_offset_out,
                                              const char **matched_name_out)
{
    char s_name[192];
    void *s_obj;
    if (direct_offset_out) *direct_offset_out = -1;
    if (matched_name_out) *matched_name_out = NULL;
    if (!target || !target->object) return NULL;
    _snprintf(s_name, sizeof(s_name), "S%s", target->name);
    s_obj = find_named_pointer_alias_in_block(target->object, 0x1200,
                                              s_name,
                                              direct_offset_out,
                                              matched_name_out);
    if (s_obj) return s_obj;
    s_obj = find_named_node(s_name);
    if (s_obj && !is_nil_engine_object(NULL, s_obj)) {
        if (matched_name_out) *matched_name_out = s_name;
        return s_obj;
    }
    {
        char local_s_name[192];
        _snprintf(local_s_name, sizeof(local_s_name), "local_%s", s_name);
        s_obj = find_named_node(local_s_name);
        if (s_obj && !is_nil_engine_object(NULL, s_obj)) {
            if (matched_name_out) *matched_name_out = local_s_name;
            return s_obj;
        }
    }
    return NULL;
}

static void bind_addon_target_from_skin_joint(physx_sidecar_t *sc,
                                              physx_chain_t *chain,
                                              physx_target_t *target,
                                              void *joint_obj,
                                              const char *joint_name,
                                              const char *mesh_name,
                                              const char *scan_source,
                                              int outer_off,
                                              int inner_off)
{
    int changed;
    int identity_changed;
    int s_direct_off = -1;
    const char *s_match = NULL;
    void *s_obj;
    static int bind_log_count;
    if (!sc || !chain || !target || !joint_obj) return;
    if (!ptr_readable(joint_obj, sizeof(void*))) return;
    identity_changed = target->object != joint_obj;
    changed = identity_changed || !target->addon_skin_bound;
    if (identity_changed) {
        target->object = joint_obj;
        target->raw_object = NULL;
        target->addon_object_name_fallback = 0;
        target->s_object = NULL;
        target->s_raw_object = NULL;
        target->s_translation_base = NULL;
        target->s_rotation_base = NULL;
        target->addon_rotation_base = NULL;
        reset_addon_tjoint_rotation_rest(target);
        reset_addon_sjoint_orientation_rest(target);
        target->s_translation_offset = -1;
        target->s_rotation_offset = -1;
        target->addon_rotation_offset = -1;
        target->s_found_logged = 0;
        target->s_missing_logged = 0;
        target->s_translation_probe_logged = 0;
        target->s_rotation_probe_logged = 0;
        target->sim_initialized = 0;
        target->addon_visual_pose_valid = 0;
    }
    /* A loose Object.Name binding and the later skin-palette binding often
       resolve to the exact same live joint. Promotion changes provenance, not
       identity, so restarting the solver here creates a delayed rest/gravity
       release and a visible kick several seconds after room load. */
    target->object = joint_obj;
    target->addon_skin_bound = 0;
    target->addon_skin_bound_tick = 0;
    s_obj = find_addon_sjoint_for_mesh_joint(target, &s_direct_off, &s_match);
    if (s_obj) {
        target->s_object = s_obj;
        target->s_raw_object = NULL;
        assume_addon_s_transform_layout(target, sc->path);
        assume_addon_t_joint_rotation_layout(target, sc->path);
        if ((target->s_translation_base && target->s_translation_offset >= 0) ||
            (target->s_rotation_base && target->s_rotation_offset >= 0)) {
            target->addon_skin_bound = 1;
            target->addon_skin_bound_tick = GetTickCount();
        }
    }
    if (changed || bind_log_count < 80) {
        bind_log_count++;
        log_line("addon skin-joint bind chain=\"%s\" target=\"%s\" mesh=\"%s\" source=\"%s\" outer_offset=0x%03x inner_offset=0x%03x joint_name=\"%s\" joint_object=%p s_name=\"%s\" s_object=%p skin_bound=%d s_direct_offset=0x%03x s_translation_offset=0x%03x s_rotation_offset=0x%03x sidecar=\"%s\" note=\"target rebound from the live TSkinPolygonGeometry joint palette instead of loose Object.Name cache\"",
                 chain->name,
                 target->name,
                 mesh_name ? mesh_name : "",
                 scan_source ? scan_source : "",
                 outer_off,
                 inner_off,
                 joint_name ? joint_name : "",
                 joint_obj,
                 s_match ? s_match : "",
                 s_obj,
                 target->addon_skin_bound,
                 s_direct_off,
                 target->s_translation_offset,
                 target->s_rotation_offset,
                 sc->path);
    }
}

static int bind_addon_ordered_skin_joint_neighbors(physx_sidecar_t *sc,
                                                   physx_chain_t *chain,
                                                   const char *mesh_name,
                                                   const char *scan_source,
                                                   void *block,
                                                   int outer_off,
                                                   int matched_inner,
                                                   const char *matched_name,
                                                   char joints[][128],
                                                   int joint_count)
{
    static const int strides[] = {
        0x020, 0x040, 0x060, 0x080, 0x0a0, 0x0c0,
        0x0e0, 0x100, 0x120, 0x140, 0x160, 0x180,
        0x010, 0x030, 0x050, 0x070
    };
    int matched_index;
    int si;
    int hits = 0;
    static int ordered_log_count;
    if (!sc || !chain || !block || !joints || joint_count <= 0 ||
        !matched_name || !matched_name[0] ||
        !ptr_readable(block, 0x400)) {
        return 0;
    }
    matched_index = addon_skin_joint_order_index(joints, joint_count,
                                                 matched_name);
    if (matched_index < 0) return 0;
    for (si = 0; si < (int)(sizeof(strides) / sizeof(strides[0])); si++) {
        int stride = strides[si];
        int ji;
        for (ji = 0; ji < joint_count; ji++) {
            int inferred_inner;
            void *candidate;
            const char *candidate_name;
            physx_target_t *target;
            char s_name[192];
            int s_direct_off = -1;
            const char *s_match = NULL;
            void *s_obj;
            if (ji == matched_index) continue;
            target = addon_chain_target_for_named_object(chain, joints[ji]);
            if (!target || target->addon_skin_bound) continue;
            inferred_inner = matched_inner + ((ji - matched_index) * stride);
            if (inferred_inner < 0 ||
                inferred_inner + (int)sizeof(void*) > 0x400) {
                continue;
            }
            if (!ptr_readable((BYTE*)block + inferred_inner,
                              sizeof(void*))) {
                continue;
            }
            candidate = *(void**)((BYTE*)block + inferred_inner);
            if (!candidate || candidate == block ||
                !ptr_readable(candidate, sizeof(void*))) {
                continue;
            }
            candidate_name = addon_named_node_name_for_object(candidate);
            if (candidate_name && candidate_name[0] &&
                !addon_name_matches_alias(candidate_name, joints[ji])) {
                continue;
            }
            _snprintf(s_name, sizeof(s_name), "S%s", target->name);
            s_obj = find_named_pointer_alias_in_block(candidate, 0x1200,
                                                      s_name,
                                                      &s_direct_off,
                                                      &s_match);
            if (!s_obj) continue;
            bind_addon_target_from_skin_joint(sc, chain, target,
                                              candidate,
                                              candidate_name ?
                                                  candidate_name :
                                                  joints[ji],
                                              mesh_name, scan_source,
                                              outer_off, inferred_inner);
            if (target->addon_skin_bound) {
                hits++;
                if (ordered_log_count < 120) {
                    ordered_log_count++;
                    log_line("addon skin-joint ordered bind mesh=\"%s\" source=\"%s\" target=\"%s\" joint_index=%d matched_name=\"%s\" matched_index=%d outer_offset=0x%03x matched_inner=0x%03x inferred_inner=0x%03x stride=0x%03x pointer=%p pointer_name=\"%s\" s_name=\"%s\" s_object=%p s_direct_offset=0x%03x sidecar=\"%s\" note=\"inferred unnamed TSkin joint palette slot from .bs joint order and validated local SJoint pointer\"",
                             mesh_name ? mesh_name : "",
                             scan_source ? scan_source : "",
                             target->name,
                             ji,
                             matched_name,
                             matched_index,
                             outer_off,
                             matched_inner,
                             inferred_inner,
                             stride,
                             candidate,
                             candidate_name ? candidate_name : "",
                             s_match ? s_match : s_name,
                             s_obj,
                             s_direct_off,
                             sc->path);
                }
            }
        }
    }
    return hits;
}

static int scan_addon_skin_memory_for_target_refs(physx_sidecar_t *sc,
                                                  physx_chain_t *chain,
                                                  const char *mesh_name,
                                                  const char *scan_source,
                                                  void *container,
                                                  void *scan_base,
                                                  int outer_off,
                                                  size_t scan_bytes)
{
    size_t off;
    int hits = 0;
    static int target_ref_log_count;
    if (!sc || !chain || !mesh_name || !scan_base || scan_bytes < sizeof(void*) ||
        !ptr_readable(scan_base, sizeof(void*))) {
        return 0;
    }
    for (off = 0; off + sizeof(void*) <= scan_bytes; off += sizeof(void*)) {
        void *candidate;
        int ti;
        if (!ptr_readable((BYTE*)scan_base + off, sizeof(void*))) continue;
        candidate = *(void**)((BYTE*)scan_base + off);
        if (!candidate || candidate == scan_base || candidate == container) continue;
        for (ti = 0; ti < chain->target_count; ti++) {
            physx_target_t *target = &chain->targets[ti];
            if (!target->addon_simulated_target) continue;
            if (!target->object || candidate != target->object) continue;
            hits++;
            if (target_ref_log_count < 160) {
                target_ref_log_count++;
                log_line("addon skin-joint target-ref mesh=\"%s\" source=\"%s\" container=%p scan_base=%p outer_offset=0x%03x inner_offset=0x%03x pointer=%p target=\"%s\" target_skin_bound=%d sidecar=\"%s\" note=\"live Object.Name PhysX target pointer was found inside the mesh/S-mesh memory; attempting to promote it to the skin-bound target\"",
                         mesh_name,
                         scan_source ? scan_source : "",
                         container,
                         scan_base,
                         outer_off,
                         (int)off,
                         candidate,
                         target->name,
                         target->addon_skin_bound,
                         sc->path);
            }
            bind_addon_target_from_skin_joint(sc, chain, target,
                                              candidate, target->name,
                                              mesh_name, scan_source,
                                              outer_off, (int)off);
        }
    }
    return hits;
}

static int scan_addon_skin_memory_for_joints(physx_sidecar_t *sc,
                                             physx_chain_t *chain,
                                             const char *mesh_name,
                                             void *container,
                                             const char *scan_source)
{
    size_t off;
    size_t bi;
    int hits = 0;
    char joint_order[32][128];
    int joint_order_count = 0;
    static const int joint_block_offsets[] = {
        0x034, 0x030, 0x038, 0x03c, 0x040, 0x044, 0x048
    };
    static int pointer_log_count;
    static int block_log_count;
    if (!sc || !chain || !mesh_name || !container ||
        !ptr_readable(container, sizeof(void*))) {
        return 0;
    }
    joint_order_count = collect_addon_skin_joint_order(sc->path,
                                                       mesh_name,
                                                       joint_order,
                                                       32);
    hits += scan_addon_skin_memory_for_target_refs(sc, chain, mesh_name,
                                                   scan_source, container,
                                                   container, -1, 0x800);
    for (off = 0; off + sizeof(void*) <= 0x800; off += sizeof(void*)) {
        void *candidate;
        const char *name;
        physx_target_t *target;
        if (!ptr_readable((BYTE*)container + off, sizeof(void*))) continue;
        candidate = *(void**)((BYTE*)container + off);
        if (!candidate || candidate == container) continue;
        name = addon_named_node_name_for_object(candidate);
        target = addon_chain_target_for_named_object(chain, name);
        if (target) {
            hits++;
            if (pointer_log_count < 160) {
                pointer_log_count++;
                log_line("addon skin-joint pointer direct mesh=\"%s\" source=\"%s\" container=%p offset=0x%03x pointer=%p pointer_name=\"%s\" target=\"%s\" current_target_object=%p sidecar=\"%s\"",
                         mesh_name, scan_source ? scan_source : "",
                         container, (int)off, candidate,
                         name ? name : "", target->name,
                         target->object, sc->path);
            }
            bind_addon_target_from_skin_joint(sc, chain, target,
                                              candidate, name,
                                              mesh_name, scan_source,
                                              (int)off, -1);
        }
    }
    for (bi = 0; bi < sizeof(joint_block_offsets) / sizeof(joint_block_offsets[0]); bi++) {
        int outer = joint_block_offsets[bi];
        void *block;
        size_t inner;
        if (!ptr_readable((BYTE*)container + outer, sizeof(void*))) continue;
        block = *(void**)((BYTE*)container + outer);
        if (!block || block == container) continue;
        if (block_log_count < 160) {
            block_log_count++;
            log_line("addon skin-joint block mesh=\"%s\" source=\"%s\" container=%p outer_offset=0x%03x block=%p readable=%d scan_bytes=0x800 sidecar=\"%s\" note=\"census of likely TSkinPolygonGeometry joint-list pointer blocks\"",
                     mesh_name, scan_source ? scan_source : "",
                     container, outer, block,
                     ptr_readable(block, sizeof(void*)) ? 1 : 0,
                     sc->path);
        }
        if (!ptr_readable(block, sizeof(void*))) continue;
        hits += scan_addon_skin_memory_for_target_refs(sc, chain, mesh_name,
                                                       scan_source, container,
                                                       block, outer, 0x800);
        for (inner = 0; inner + sizeof(void*) <= 0x800; inner += sizeof(void*)) {
            void *inner_candidate;
            const char *inner_name;
            physx_target_t *inner_target;
            if (!ptr_readable((BYTE*)block + inner, sizeof(void*))) continue;
            inner_candidate = *(void**)((BYTE*)block + inner);
            if (!inner_candidate || inner_candidate == block ||
                inner_candidate == container) {
                continue;
            }
            inner_name = addon_named_node_name_for_object(inner_candidate);
            inner_target = addon_chain_target_for_named_object(chain, inner_name);
            if (!inner_target) continue;
            hits++;
            if (pointer_log_count < 160) {
                pointer_log_count++;
                log_line("addon skin-joint pointer indirect mesh=\"%s\" source=\"%s\" container=%p outer_offset=0x%03x block=%p inner_offset=0x%03x pointer=%p pointer_name=\"%s\" target=\"%s\" current_target_object=%p sidecar=\"%s\" note=\"known mesh joint block only\"",
                         mesh_name, scan_source ? scan_source : "",
                         container, outer, block,
                         (int)inner, inner_candidate,
                         inner_name ? inner_name : "",
                         inner_target->name,
                         inner_target->object, sc->path);
            }
            bind_addon_target_from_skin_joint(sc, chain, inner_target,
                                              inner_candidate, inner_name,
                                              mesh_name, scan_source,
                                              outer, (int)inner);
            if (joint_order_count > 0) {
                hits += bind_addon_ordered_skin_joint_neighbors(sc,
                                                                chain,
                                                                mesh_name,
                                                                scan_source,
                                                                block,
                                                                outer,
                                                                (int)inner,
                                                                inner_name,
                                                                joint_order,
                                                                joint_order_count);
            }
        }
    }
    return hits;
}

static int scan_addon_skin_named_instances(physx_sidecar_t *sc,
                                           physx_chain_t *chain,
                                           const char *mesh_name,
                                           const char *scan_source,
                                           int scan_memory)
{
    void *seen[12];
    int seen_count = 0;
    int i, s;
    int hits = 0;
    int scan_hits = 0;
    static int instance_log_count;
    if (!sc || !chain || !mesh_name || !mesh_name[0]) return 0;
    memset(seen, 0, sizeof(seen));
    for (i = named_node_count - 1; i >= 0 && seen_count < (int)(sizeof(seen) / sizeof(seen[0])); i--) {
        void *obj;
        int duplicate = 0;
        if (_stricmp(named_nodes[i].name, mesh_name) != 0) continue;
        obj = named_nodes[i].object;
        if (!obj || !ptr_readable(obj, sizeof(void*))) continue;
        for (s = 0; s < seen_count; s++) {
            if (seen[s] == obj) {
                duplicate = 1;
                break;
            }
        }
        if (duplicate) continue;
        seen[seen_count++] = obj;
        if (instance_log_count < 80) {
            instance_log_count++;
            log_line("addon skin-probe mesh-instance mesh=\"%s\" source=\"%s\" object=%p seen_index=%d named_nodes=%d first_seen_tick=%lu sidecar=\"%s\" note=\"automated no-touch test recorded the rendered mesh instance; mesh memory scan disabled because it crashes during TK17 room load\"",
                     mesh_name,
                     scan_source ? scan_source : "",
                     obj,
                     seen_count,
                     named_node_count,
                     (unsigned long)named_nodes[i].first_seen_tick,
                     sc->path);
        }
        hits++;
        if (scan_memory) {
            scan_hits += scan_addon_skin_memory_for_joints(sc, chain,
                                                           mesh_name, obj,
                                                           scan_source);
        }
    }
    return hits + scan_hits;
}

static void probe_addon_skin_geometry(physx_sidecar_t *sc,
                                      physx_chain_t *chain,
                                      DWORD now)
{
    char mesh_names[16][128];
    int mesh_count;
    int mi;
    int found_any = 0;
    int scan_memory = 0;
    int scene_visible = -1;
    const DWORD skin_memory_scan_delay_ms = 8000u;
    const DWORD automated_timeout_ms = 30000u;
    if (!sc || !chain || !chain->addon_chain) return;
    if (!addon_physics_probe_enabled && !defaults_cfg.debug) return;
    if (!chain->addon_test_done_logged &&
        addon_chain_all_physx_targets_live_sjoint(chain)) {
        if (!chain->addon_test_start_tick) {
            chain->addon_test_start_tick = now;
        }
        addon_chain_log_automated_test_done(
            sc, chain, now, "live-sjoint-output");
        return;
    }
    if (!chain->addon_test_start_tick) {
        chain->addon_test_start_tick = now;
        chain->addon_test_status_tick = now;
        log_line("addon automated-test started chain=\"%s\" timeout_ms=%lu sidecar=\"%s\" note=\"wait for addon automated-test done before closing the game\"",
                 chain->name,
                 (unsigned long)automated_timeout_ms,
                 sc->path);
    }
    if (chain->addon_test_done_logged) return;
    if (addon_chain_all_physx_targets_skin_bound(chain)) {
        addon_chain_log_automated_test_done(sc, chain, now, "skin-bound");
        return;
    }
    if (now - chain->addon_test_start_tick >= automated_timeout_ms) {
        addon_chain_log_automated_test_done(sc, chain, now, "timeout");
        return;
    }
    if (now - chain->addon_test_status_tick >= 10000u) {
        int bound = 0;
        int total = addon_chain_count_physx_targets(chain, &bound);
        chain->addon_test_status_tick = now;
        log_line("addon automated-test status chain=\"%s\" bound=%d/%d probes=%d elapsed_ms=%lu sidecar=\"%s\" note=\"test still running; wait for addon automated-test done\"",
                 chain->name, bound, total, chain->skin_probe_count,
                 (unsigned long)(now - chain->addon_test_start_tick),
                 sc->path);
    }
    if (chain->skin_probe_count >= 120) {
        addon_chain_log_automated_test_done(sc, chain, now, "probe-limit");
        return;
    }
    if (chain->skin_probe_tick && now - chain->skin_probe_tick < 2500) return;
    chain->skin_probe_tick = now;
    chain->skin_probe_count++;
    scene_visible = addon_chain_scene_visible(sc, chain, now,
                                              NULL, NULL, NULL);
    if (chain->skin_probe_found_tick &&
        scene_visible > 0 &&
        now - chain->skin_probe_found_tick >= skin_memory_scan_delay_ms) {
        scan_memory = 1;
        if (!chain->skin_memory_scan_armed_logged) {
            chain->skin_memory_scan_armed_logged = 1;
            log_line("addon skin-joint scan armed chain=\"%s\" delay_ms=%lu scene_visible=%d probes=%d sidecar=\"%s\" note=\"mesh exists and room is visible; scanning known TSkinPolygonGeometry joint-list offsets and exact live PhysX target pointer refs\"",
                     chain->name,
                     (unsigned long)skin_memory_scan_delay_ms,
                     scene_visible,
                     chain->skin_probe_count,
                     sc->path);
        }
    }
    mesh_count = collect_addon_skin_mesh_names(sc->path, chain, mesh_names, 16);
    if (mesh_count <= 0) {
        if (chain->skin_probe_count <= 6) {
            log_line("addon skin-probe no mesh names chain=\"%s\" sidecar=\"%s\" note=\"adjacent .bs did not expose a TSkinPolygonGeometry.Joint list containing this physx chain\"",
                     chain->name, sc->path);
        }
        return;
    }
    for (mi = 0; mi < mesh_count; mi++) {
        char s_mesh_name[192];
        void *mesh_obj;
        void *s_mesh_obj;
        int instance_hits = 0;
        mesh_obj = find_named_node(mesh_names[mi]);
        _snprintf(s_mesh_name, sizeof(s_mesh_name), "S%s", mesh_names[mi]);
        s_mesh_obj = find_named_node(s_mesh_name);
        if (mesh_obj || s_mesh_obj) found_any = 1;
        if (chain->skin_probe_count <= 6 || mesh_obj || s_mesh_obj) {
            log_line("addon skin-probe mesh-name mesh=\"%s\" mesh_obj=%p smesh=\"%s\" smesh_obj=%p chain=\"%s\" sidecar=\"%s\" note=\"safe Object.Name lookup only; no mesh memory scan\"",
                     mesh_names[mi], mesh_obj, s_mesh_name, s_mesh_obj,
                     chain->name, sc->path);
        }
        instance_hits += scan_addon_skin_named_instances(sc, chain, mesh_names[mi], "TSkinPolygonGeometry", scan_memory);
        instance_hits += scan_addon_skin_named_instances(sc, chain, s_mesh_name, "SSkinPolygonGeometry", scan_memory);
        if (instance_hits > 0) found_any = 1;
    }
    if (found_any) {
        chain->skin_probe_found = 1;
        if (!chain->skin_probe_found_tick) {
            chain->skin_probe_found_tick = now;
            log_line("addon skin-probe mesh-found chain=\"%s\" scene_visible=%d delay_before_joint_scan_ms=%lu sidecar=\"%s\" note=\"mesh instance found; output test will wait for skin-bound joint palette, not loose Object.Name bones\"",
                     chain->name,
                     scene_visible,
                     (unsigned long)skin_memory_scan_delay_ms,
                     sc->path);
        }
    }
    if (addon_chain_all_physx_targets_skin_bound(chain)) {
        addon_chain_log_automated_test_done(sc, chain, now, "skin-bound");
    }
}

/* Object.Name may expose the matrix through s_object without a raw pointer.
   Verify that small, known matrix window rather than requiring 4 KB after
   the object. Scene translation plus a valid basis avoids mistaking the
   native 0x06c/0x07c transform fields for this layout. */
static int addon_object_live_matrix_matches_scene(BYTE *base,
                                                  const float expected_t[3])
{
    float rows[9];
    const float *translation;
    int row, axis;
    if (!base || !expected_t ||
        !physx_vec3_sane_limit(expected_t, 64.0f) ||
        !ptr_readable(base + 0x018, 0x03c)) return 0;
    for (row = 0; row < 3; row++) {
        const float *v = (const float*)(base + 0x018 + row * 0x10);
        if (!addon_vector_basis_like(v)) return 0;
        memcpy(&rows[row * 3], v, sizeof(float) * 3);
    }
    if (!addon_normalize_basis_rows(rows)) return 0;
    translation = (const float*)(base + 0x048);
    if (!physx_vec3_sane_limit(translation, 64.0f)) return 0;
    for (axis = 0; axis < 3; axis++) {
        if (physx_absf(translation[axis] - expected_t[axis]) > 0.0005f)
            return 0;
    }
    return 1;
}

static int assume_addon_s_transform_layout(physx_target_t *target,
                                           const char *sidecar_path)
{
    BYTE *bases[2];
    const char *labels[2];
    float expected_t[3];
    float expected_r[3];
    int has_t = 0;
    int has_r = 0;
    int bi;
    if (!target || !target->s_object) return 0;
    read_addon_sjoint_scene_pose(sidecar_path, target->name,
                                 expected_t, &has_t,
                                 expected_r, &has_r);
    reset_addon_sjoint_orientation_rest(target);
    if (has_r) {
        target->addon_joint_orientation[0] = expected_r[0];
        target->addon_joint_orientation[1] = expected_r[1];
        target->addon_joint_orientation[2] = expected_r[2];
        target->addon_joint_orientation_valid = 1;
    }
    /*
       The live SJoint matrix occupies only 0x018..0x053. Do this bounded
       check before the legacy 0x1000 scene-pose scan: a valid raw SJoint can
       sit less than 4 KB from the end of its committed allocation, in which
       case requiring the whole scan window incorrectly rejects it and falls
       through to the wrapper Object.
    */
    if (target->s_raw_object &&
        ptr_readable((BYTE*)target->s_raw_object + 0x018, 0x03c)) {
        BYTE *live_base = (BYTE*)target->s_raw_object;
        float *row0 = (float*)(live_base + 0x018);
        float *row1 = (float*)(live_base + 0x028);
        float *row2 = (float*)(live_base + 0x038);
        float *translation = (float*)(live_base + 0x048);
        if (addon_vector_basis_like(row0) &&
            addon_vector_basis_like(row1) &&
            addon_vector_basis_like(row2) &&
            physx_vec3_sane_limit(translation, 64.0f)) {
            target->s_translation_base = live_base;
            target->s_translation_source = "raw-live-matrix";
            target->s_translation_offset = 0x048;
            target->s_rotation_base = live_base;
            target->s_rotation_source = "raw-live-matrix";
            target->s_rotation_offset = 0x038;
            if (defaults_cfg.debug) {
                log_line("target addon live s-transform layout confirmed target=\"%s\" source=raw base=%p rotation_offset=0x038 translation_offset=0x048 row_lengths=(%.5f,%.5f,%.5f) translation=(%.5f,%.5f,%.5f) note=\"bounded live-matrix validation avoids rejecting valid SJoints near an allocation boundary\"",
                         target->name,
                         live_base,
                         physx_vec3_len(row0),
                         physx_vec3_len(row1),
                         physx_vec3_len(row2),
                         translation[0],
                         translation[1],
                         translation[2]);
            }
            return 1;
        }
    }
    if (has_t && addon_object_live_matrix_matches_scene(
            (BYTE*)target->s_object, expected_t)) {
        target->s_translation_base = (BYTE*)target->s_object;
        target->s_rotation_base = (BYTE*)target->s_object;
        target->s_translation_source = "object-live-matrix";
        target->s_rotation_source = "object-live-matrix";
        target->s_translation_offset = 0x048;
        target->s_rotation_offset = 0x038;
        if (defaults_cfg.debug) {
            log_line("target addon live s-transform layout confirmed target=\"%s\" source=object base=%p rotation_offset=0x038 translation_offset=0x048 note=\"bounded matrix and scene-translation validation\"",
                     target->name, target->s_object);
        }
        return 1;
    }
    bases[0] = (BYTE*)target->s_raw_object;
    bases[1] = (BYTE*)target->s_object;
    labels[0] = "raw";
    labels[1] = "object";
    for (bi = 0; bi < 2; bi++) {
        BYTE *base = bases[bi];
        float *t;
        float *r = NULL;
        int t_off = -1;
        int r_off = -1;
        int orientation_off = -1;
        int best_off = -1;
        float best_v[3] = { 0.0f, 0.0f, 0.0f };
        float best_err = 0.0f;
        /* The native SSimpleTransform pose fields only require the bounded
           0x06c..0x087 window.  Requiring a complete 4 KB scan here rejects
           valid room SJoints that happen to sit near the end of a committed
           allocation (for example palm02_trunk3/4).  The optional scene-pose
           scan below can still fail independently; the canonical live pose
           slots remain safe and sufficient. */
        if (!base ||
            !ptr_readable(base + 0x06c, 0x01c)) {
            continue;
        }
        if (has_t) {
            t_off = find_vector3_offset(base, expected_t, 0.0005f,
                                        &best_off, best_v, &best_err);
            if (t_off < 0) {
                log_line("target addon s-translation scene probe nearest target=\"%s\" source=%s base=%p best_offset=0x%03x err=%.5f nearest=(%.5f,%.5f,%.5f) expected=(%.5f,%.5f,%.5f)",
                         target->name, labels[bi], base, best_off, best_err,
                         best_v[0], best_v[1], best_v[2],
                         expected_t[0], expected_t[1], expected_t[2]);
            }
        }
        if (t_off < 0 && ptr_readable(base + 0x07c, sizeof(float) * 3)) {
            t_off = 0x07c;
        }
        if (has_r) {
            best_off = -1;
            best_v[0] = best_v[1] = best_v[2] = 0.0f;
            best_err = 0.0f;
            orientation_off = find_vector3_offset(base, expected_r, 0.01f,
                                                 &best_off, best_v, &best_err);
            if (orientation_off < 0) {
                log_line("target addon s-orientation scene probe nearest target=\"%s\" source=%s base=%p best_offset=0x%03x err=%.5f nearest=(%.5f,%.5f,%.5f) expected=(%.5f,%.5f,%.5f)",
                         target->name, labels[bi], base, best_off, best_err,
                         best_v[0], best_v[1], best_v[2],
                         expected_r[0], expected_r[1], expected_r[2]);
            }
        }
        if (t_off < 0 || !ptr_readable(base + t_off, sizeof(float) * 3)) {
            continue;
        }
        t = (float*)(base + t_off);
        target->s_translation_base = base;
        target->s_translation_source = labels[bi];
        target->s_translation_offset = t_off;
        if (t_off >= 0x10 &&
            ptr_readable(base + t_off - 0x10, sizeof(float) * 3)) {
            float *pose_r = (float*)(base + t_off - 0x10);
            if (sane_probe_float(pose_r[0]) &&
                sane_probe_float(pose_r[1]) &&
                sane_probe_float(pose_r[2])) {
                r_off = t_off - 0x10;
            }
        }
        if (r_off < 0 &&
            ptr_readable(base + 0x06c, sizeof(float) * 3)) {
            float *pose_r = (float*)(base + 0x06c);
            if (sane_probe_float(pose_r[0]) &&
                sane_probe_float(pose_r[1]) &&
                sane_probe_float(pose_r[2])) {
                r_off = 0x06c;
            }
        }
        if (r_off < 0 &&
            orientation_off >= 0 &&
            ptr_readable(base + orientation_off, sizeof(float) * 3)) {
            r_off = orientation_off;
        }
        if (r_off >= 0 && ptr_readable(base + r_off, sizeof(float) * 3)) {
            r = (float*)(base + r_off);
            if (r_off != orientation_off &&
                orientation_off >= 0 &&
                ptr_readable(base + orientation_off, sizeof(float) * 3)) {
                float *orientation = (float*)(base + orientation_off);
                log_line("target addon s-transform live-pose preferred target=\"%s\" source=%s base=%p pose_rotation_offset=0x%03x orientation_offset=0x%03x pose_rotation=(%.5f,%.5f,%.5f) joint_orientation=(%.5f,%.5f,%.5f) reason=\"writing live SSimpleTransform rotation instead of rest JointOrientation\"",
                         target->name, labels[bi], base,
                         r_off, orientation_off,
                         r[0], r[1], r[2],
                         orientation[0], orientation[1], orientation[2]);
            }
        }
        if (r_off >= 0 && ptr_readable(base + r_off, sizeof(float) * 3)) {
            r = (float*)(base + r_off);
            target->s_rotation_base = base;
            target->s_rotation_source = labels[bi];
            target->s_rotation_offset = r_off;
        }
        if (defaults_cfg.debug) {
            log_line("target addon s-transform layout assumed target=\"%s\" source=%s base=%p translation_offset=0x%03x rotation_offset=0x%03x orientation_offset=0x%03x translation=(%.5f,%.5f,%.5f) rotation=(%.5f,%.5f,%.5f) scene_translation=%d scene_orientation=%d reason=\"custom addon SJoint translation matched sidecar .bs; runtime rotation uses %s slot\"",
                 target->name, labels[bi], base, t_off,
                 r_off, orientation_off,
                 t[0], t[1], t[2],
                 r ? r[0] : 0.0f,
                 r ? r[1] : 0.0f,
                 r ? r[2] : 0.0f,
                 has_t, has_r,
                 (r_off >= 0 && r_off == t_off - 0x10) ? "SSimpleTransform.Rotation" :
                 (r_off == 0x06c) ? "STransformOutput" :
                     ((r_off == orientation_off && orientation_off >= 0) ? "JointOrientation" : "STransform"));
        }
        return 1;
    }
    return 0;
}

static void addon_object_restore_outputs_before_reload(physx_sidecar_t *sc)
{
    int c, t;
    if (!sc || !sc->loaded || !real_SSimpleTransform_RotationSet) return;
    for (c = 0; c < sc->chain_count; c++) {
        physx_chain_t *chain = &sc->chains[c];
        if (!chain->object_transform_chain) continue;
        for (t = 0; t < chain->target_count; t++) {
            physx_target_t *target = &chain->targets[t];
            if (!target->addon_simulated_target ||
                !target->object_output_applied ||
                !target->addon_write_guard_ready ||
                target->s_object != target->addon_write_guard_s_object ||
                target->s_raw_object !=
                    target->addon_write_guard_s_rotation_base ||
                !target->s_raw_object ||
                !target->s_object ||
                is_nil_engine_object(target->s_raw_object,
                                     target->s_object)) {
                continue;
            }
            real_SSimpleTransform_RotationSet(
                target->s_raw_object, SCRIPT_PROPERTY_SSIMPLE_ROTATION,
                target->object_scene_rotation);
            target->object_output_applied = 0;
        }
    }
}

static int addon_sidecar_is_room_scene_path(const char *path)
{
    if (!path || !path[0]) return 0;
    return contains_i(path, "\\Scenes\\") &&
           contains_i(path, "\\Room\\");
}

static void load_sidecar(physx_sidecar_t *sc)
{
    char sections[8192];
    char *s;
    FILETIME wt;
    if (!sc || !get_file_write_time_a(sc->path, &wt)) return;
    if (sc->loaded && !filetime_differs(&wt, &sc->write_time)) return;
    addon_object_restore_outputs_before_reload(sc);
    sc->write_time = wt;
    sc->loaded = 1;
    sc->room_scene_sidecar = addon_sidecar_is_room_scene_path(sc->path);
    /* A room sidecar is already selected explicitly by the live room path.
       Do not require a redundant [physics] header merely to activate its
       room-bone sections. Ordinary add-on sidecars retain their old default. */
    sc->enabled = profile_bool("physics", "enabled",
                               sc->room_scene_sidecar ? 1 : 0,
                               sc->path);
    sc->write_test = profile_bool("physics", "write_test", 0, sc->path);
    GetPrivateProfileStringA("physics", "write_test_target", "",
                             sc->write_test_target, sizeof(sc->write_test_target), sc->path);
    GetPrivateProfileStringA("physics", "write_test_axis", "y", sections, sizeof(sections), sc->path);
    sc->write_test_axis = parse_axis_name(sections);
    sc->write_test_amount = profile_float("physics", "write_test_amount", -0.05f, sc->path);
    sc->write_test_duration_ms = GetPrivateProfileIntA("physics", "write_test_duration_ms", 750, sc->path);
    if (sc->write_test_duration_ms < 50) sc->write_test_duration_ms = 50;
    if (sc->write_test_duration_ms > 5000) sc->write_test_duration_ms = 5000;
    sc->write_test_state = 0;
    sc->write_test_start_tick = 0;
    sc->write_test_original[0] = 0.0f;
    sc->write_test_original[1] = 0.0f;
    sc->write_test_original[2] = 0.0f;
    sc->chain_count = 0;
    sc->root_logged = 0;
    sc->root_import_attempted = 0;
    sc->root_raw_object = NULL;
    sc->root_object = NULL;
    sc->script_root_raw_object = NULL;
    sc->script_root_object = NULL;
    sc->addon_owner_person[0] = 0;
    sc->addon_owner_seen_tick = 0;
    sc->addon_owner_logged = 0;
    memset(sc->chains, 0, sizeof(sc->chains));
    GetPrivateProfileSectionNamesA(sections, sizeof(sections), sc->path);
    for (s = sections; *s; s += strlen(s) + 1) {
        physx_chain_t *chain;
        char list[2048];
        char gravity[128];
        int addon_section = (_strnicmp(s, "NC-TK17-PhysX:", 14) == 0);
        int legacy_chain_section = (_strnicmp(s, "chain:", 6) == 0);
        int legacy_object_section = (_strnicmp(s, "object:", 7) == 0);
        if (!addon_section && !legacy_chain_section && !legacy_object_section) continue;
        if (addon_section && !addon_physics_enabled && !addon_physics_probe_enabled) {
            log_line("addon sidecar chain skipped section=\"%s\" sidecar=\"%s\" reason=\"[addon_physics] enabled=false\" note=\"regular add-on physics is disabled; body sidecars and built-in penis/testicle chains are unaffected\"",
                     s, sc->path);
            continue;
        }
        if (addon_section) {
            const char *target_name = strchr(s, ':');
            if (target_name && target_name[1] &&
                (addon_sidecar_section_is_target_override(sc, s,
                                                          target_name + 1) ||
                 (!addon_sidecar_section_has_chain_root_keys(s, sc->path) &&
                  addon_sidecar_section_name_is_declared_child(sections,
                                                               target_name + 1,
                                                               sc->path)))) {
                continue;
            }
        }
        if (addon_section && !addon_sidecar_section_has_chain_keys(s, sc->path)) {
            continue;
        }
        /* Person-owned add-ons need four runtime copies and therefore retain
           the 16-definition limit (16 * 4 == 64). Room joints exist only once,
           so a room sidecar may use the complete chain array. */
        if (sc->chain_count >=
            (sc->room_scene_sidecar ?
             (int)(sizeof(sc->chains) / sizeof(sc->chains[0])) :
             (int)(sizeof(sc->chains) / sizeof(sc->chains[0]) / 4))) {
            break;
        }
        chain = &sc->chains[sc->chain_count++];
        chain->drive_offset_override = -1;
        chain->drive_auto_offset = -1;
        lstrcpynA(chain->name, strchr(s, ':') + 1, sizeof(chain->name));
        if (addon_section) {
            char requested_type[64];
            char rotation_solver[64];
            char children[2048];
            char scope[256];
            char collision_custom_targets[4096];
            char collision_custom_persons[64];
            char collision_color[64];
            char explicit_parent[128];
            float limit_from_min;
            float limit_from_max;
            float translation_h_sign;
            float translation_v_sign;
            float translation_d_sign;
            float rotation_h_source_sign;
            float rotation_v_source_sign;
            float rotation_twist_source_sign;
            float rotation_h_tail_sign = 1.0f;
            float rotation_v_tail_sign = 1.0f;
            float rotation_twist_tail_sign = 1.0f;
            float gravity_h_tail_sign;
            float gravity_v_tail_sign;
            int collision_custom_persons_valid;
            chain->addon_chain = 1;
            GetPrivateProfileStringA(s, "type", "bone", requested_type, sizeof(requested_type), sc->path);
            trim_in_place(requested_type);
            if (!requested_type[0] ||
                _stricmp(requested_type, "bone") == 0 ||
                _stricmp(requested_type, "transform") == 0 ||
                _stricmp(requested_type, "transform_chain") == 0) {
                lstrcpynA(chain->type, "rigid_chain", sizeof(chain->type));
            } else {
                lstrcpynA(chain->type, requested_type, sizeof(chain->type));
            }
            chain->object_transform_chain =
                _stricmp(chain->type, "object") == 0;
            GetPrivateProfileStringA(s, "rotation_solver", "legacy",
                                     rotation_solver,
                                     sizeof(rotation_solver), sc->path);
            trim_in_place(rotation_solver);
            chain->rotation_solver_full_angle =
                _stricmp(rotation_solver, "full_angle") == 0;
            if (rotation_solver[0] &&
                _stricmp(rotation_solver, "legacy") != 0 &&
                !chain->rotation_solver_full_angle) {
                log_line("addon sidecar rotation solver fallback section=\"%s\" requested=\"%s\" selected=\"legacy\" sidecar=\"%s\" reason=\"recognized values are legacy and full_angle\"",
                         s, rotation_solver, sc->path);
            }
            chain->simulate = profile_bool(s, "enabled",
                                           profile_bool(s, "enable", 1, sc->path),
                                           sc->path);
            if (!addon_physics_enabled) {
                chain->simulate = 0;
            }
            chain->gravity_enabled = profile_bool(s, "gravity_enabled", 1, sc->path);
            /* Room wind is inherited automatically. These optional per-chain
               keys only opt out or tune response; existing add-on sidecars do
               not need to be edited. */
            chain->wind_enabled = profile_bool(s, "wind_enabled", 1,
                                                sc->path);
            chain->wind_scale = physx_clampf(
                profile_float(s, "wind_scale", 1.0f, sc->path),
                0.0f, 10.0f);
            chain->wind_sway_strength =
                sidecar_profile_key_exists(s, "sway_strength", sc->path) ?
                physx_clampf(profile_float(s, "sway_strength", 0.0f,
                                           sc->path), 0.0f, 10.0f) :
                -1.0f;
            chain->wind_sway_frequency =
                sidecar_profile_key_exists(s, "sway_frequency", sc->path) ?
                physx_clampf(profile_float(s, "sway_frequency", 0.08f,
                                           sc->path), 0.001f, 10.0f) :
                -1.0f;
            chain->collision_enabled = profile_bool(s, "collision_enabled", 0, sc->path);
            GetPrivateProfileStringA(s, "collision_scope", "", scope, sizeof(scope), sc->path);
            trim_in_place(scope);
            chain->collision_scope = parse_collision_scope(scope, chain->collision_enabled);
            if (chain->collision_scope & PHYSX_COLLISION_SCOPE_CUSTOM) {
                GetPrivateProfileStringA(
                    s, "collision_scope_custom_targets", "",
                    collision_custom_targets,
                    sizeof(collision_custom_targets), sc->path);
                trim_in_place(collision_custom_targets);
                GetPrivateProfileStringA(
                    s, "collision_scope_custom_persons", "",
                    collision_custom_persons,
                    sizeof(collision_custom_persons), sc->path);
                trim_in_place(collision_custom_persons);
                if (!collision_custom_persons[0]) {
                    GetPrivateProfileStringA(
                        s, "collision_scope_custom_scope", "",
                        collision_custom_persons,
                        sizeof(collision_custom_persons), sc->path);
                    trim_in_place(collision_custom_persons);
                }
                chain->collision_custom_target_count =
                    parse_collision_custom_targets(
                        chain, collision_custom_targets, s, sc->path);
                collision_custom_persons_valid =
                    parse_collision_custom_persons(
                        collision_custom_persons,
                        &chain->collision_custom_all_persons);
                if (chain->collision_custom_target_count > 0 &&
                    collision_custom_persons_valid) {
                    chain->collision_custom_enabled = 1;
                    chain->collision_scope &=
                        ~(PHYSX_COLLISION_SCOPE_BODY |
                          PHYSX_COLLISION_SCOPE_BODY_ALL);
                    chain->collision_scope |=
                        chain->collision_custom_all_persons ?
                            PHYSX_COLLISION_SCOPE_BODY_ALL :
                            PHYSX_COLLISION_SCOPE_BODY;
                } else {
                    chain->collision_scope &=
                        ~PHYSX_COLLISION_SCOPE_CUSTOM;
                    chain->collision_custom_enabled = 0;
                    chain->collision_custom_all_persons = 0;
                    chain->collision_custom_target_count = 0;
                    memset(chain->collision_custom_target_mask, 0,
                           sizeof(chain->collision_custom_target_mask));
                    log_line("addon sidecar custom collision ignored section=\"%s\" targets_present=%d persons=\"%s\" persons_valid=%d sidecar=\"%s\" reason=\"custom requires at least one recognized collision_scope_custom_targets value and collision_scope_custom_persons=wearer or all\"",
                             s, collision_custom_targets[0] ? 1 : 0,
                             collision_custom_persons,
                             collision_custom_persons_valid,
                             sc->path);
                }
            }
            chain->collision_radius =
                physx_clampf(profile_float(s, "collision_radius",
                                           body_chain_collider_global_cfg.chain_radius,
                                           sc->path),
                             0.001f, 0.200f);
            chain->collision_terminal_scale =
                physx_clampf(profile_float(s, "collision_terminal_scale",
                                           1.0f, sc->path),
                             0.0f, 4.0f);
            chain->collision_debug_draw =
                profile_bool(s, "collision_debug_draw", 0, sc->path);
            GetPrivateProfileStringA(s, "collision_debug_color", "",
                                     collision_color, sizeof(collision_color),
                                     sc->path);
            trim_in_place(collision_color);
            chain->collision_debug_color =
                parse_sidecar_debug_color(collision_color, 0xffff4040u);
            chain->stiffness = profile_float(s, "stiffness", defaults_cfg.stiffness, sc->path);
            chain->damping = profile_float(s, "damping", defaults_cfg.damping, sc->path);
            chain->limit_angle = profile_float(s, "limit_angle", defaults_cfg.limit_angle, sc->path);
            if (!parse_angle_alias_vec3_or_scalar(s, "limit_min_angle",
                                                  "joint_min_angle",
                                                  -chain->limit_angle,
                                                  chain->joint_min_angle,
                                                  sc->path)) {
                chain->joint_min_angle[0] = -chain->limit_angle;
                chain->joint_min_angle[1] = -chain->limit_angle;
                chain->joint_min_angle[2] = -chain->limit_angle;
            }
            if (!parse_angle_alias_vec3_or_scalar(s, "limit_max_angle",
                                                  "joint_max_angle",
                                                  chain->limit_angle,
                                                  chain->joint_max_angle,
                                                  sc->path)) {
                chain->joint_max_angle[0] = chain->limit_angle;
                chain->joint_max_angle[1] = chain->limit_angle;
                chain->joint_max_angle[2] = chain->limit_angle;
            }
            limit_from_min = max_abs_vec3_value(chain->joint_min_angle);
            limit_from_max = max_abs_vec3_value(chain->joint_max_angle);
            if (limit_from_min > chain->limit_angle) chain->limit_angle = limit_from_min;
            if (limit_from_max > chain->limit_angle) chain->limit_angle = limit_from_max;
            chain->drive_scale = profile_float(s, "drive_scale", 1.0f, sc->path);
            chain->drive_strength = profile_float(s, "drive_strength", 1.0f, sc->path);
            chain->translation_horizontal_source_axis =
                sidecar_profile_signed_axis(
                    s, "translation_horizontal_source_axis",
                    0, &translation_h_sign, sc->path);
            if (sidecar_profile_key_exists(s, "translation_horizontal_tail_axis",
                                           sc->path)) {
                chain->translation_horizontal_tail_axis =
                    sidecar_profile_axis(
                        s, "translation_horizontal_tail_axis",
                        chain->translation_horizontal_source_axis,
                        sc->path);
            } else {
                chain->translation_horizontal_tail_axis =
                    chain->translation_horizontal_source_axis;
            }
            chain->translation_horizontal_scale =
                profile_float(s, "translation_horizontal_scale",
                              1.0f, sc->path) * translation_h_sign;
            chain->translation_vertical_source_axis =
                sidecar_profile_signed_axis(
                    s, "translation_vertical_source_axis",
                    1, &translation_v_sign, sc->path);
            if (sidecar_profile_key_exists(s, "translation_vertical_tail_axis",
                                           sc->path)) {
                chain->translation_vertical_tail_axis =
                    sidecar_profile_axis(
                        s, "translation_vertical_tail_axis",
                        chain->translation_vertical_source_axis,
                        sc->path);
            } else {
                chain->translation_vertical_tail_axis =
                    chain->translation_vertical_source_axis;
            }
            chain->translation_vertical_scale =
                profile_float(s, "translation_vertical_scale",
                              1.0f, sc->path) * translation_v_sign;
            chain->translation_depth_source_axis =
                sidecar_profile_signed_axis(
                    s, "translation_depth_source_axis",
                    2, &translation_d_sign, sc->path);
            if (sidecar_profile_key_exists(s, "translation_depth_tail_axis",
                                           sc->path)) {
                chain->translation_depth_tail_axis =
                    sidecar_profile_axis(
                        s, "translation_depth_tail_axis",
                        chain->translation_depth_source_axis,
                        sc->path);
            } else {
                chain->translation_depth_tail_axis =
                    chain->translation_depth_source_axis;
            }
            chain->translation_depth_scale =
                profile_float(s, "translation_depth_scale",
                              1.0f, sc->path) * translation_d_sign;
            chain->rotation_drive_horizontal_source_axis =
                sidecar_profile_signed_axis(
                    s, "rotation_horizontal_source_axis", 0,
                    &rotation_h_source_sign, sc->path);
            if (sidecar_profile_key_exists(s, "rotation_horizontal_tail_axis",
                                           sc->path)) {
                chain->rotation_drive_horizontal_tail_axis =
                    sidecar_profile_signed_axis(
                        s, "rotation_horizontal_tail_axis",
                        chain->rotation_drive_horizontal_source_axis,
                        &rotation_h_tail_sign, sc->path);
            } else {
                chain->rotation_drive_horizontal_tail_axis =
                    chain->rotation_drive_horizontal_source_axis;
            }
            chain->rotation_drive_horizontal_scale =
                profile_float(s, "rotation_horizontal_scale",
                              -1.0f, sc->path) *
                rotation_h_source_sign * rotation_h_tail_sign;
            chain->rotation_drive_vertical_source_axis =
                sidecar_profile_signed_axis(
                    s, "rotation_vertical_source_axis", 1,
                    &rotation_v_source_sign, sc->path);
            if (sidecar_profile_key_exists(s, "rotation_vertical_tail_axis",
                                           sc->path)) {
                chain->rotation_drive_vertical_tail_axis =
                    sidecar_profile_signed_axis(
                        s, "rotation_vertical_tail_axis",
                        chain->rotation_drive_vertical_source_axis,
                        &rotation_v_tail_sign, sc->path);
            } else {
                chain->rotation_drive_vertical_tail_axis =
                    chain->rotation_drive_vertical_source_axis;
            }
            chain->rotation_drive_vertical_scale =
                profile_float(s, "rotation_vertical_scale",
                              1.0f, sc->path) *
                rotation_v_source_sign * rotation_v_tail_sign;
            chain->rotation_drive_twist_source_axis =
                sidecar_profile_signed_axis(
                    s, "rotation_twist_source_axis", 2,
                    &rotation_twist_source_sign, sc->path);
            if (sidecar_profile_key_exists(s, "rotation_twist_tail_axis",
                                           sc->path)) {
                chain->rotation_drive_twist_tail_axis =
                    sidecar_profile_signed_axis(
                        s, "rotation_twist_tail_axis",
                        chain->rotation_drive_twist_source_axis,
                        &rotation_twist_tail_sign, sc->path);
            } else {
                chain->rotation_drive_twist_tail_axis =
                    chain->rotation_drive_twist_source_axis;
            }
            chain->rotation_drive_twist_scale =
                profile_float(s, "rotation_twist_scale",
                              0.35f, sc->path) *
                rotation_twist_source_sign * rotation_twist_tail_sign;
            chain->gravity_horizontal_source_axis =
                sidecar_profile_signed_axis(
                    s,
                    "gravity_horizontal_source_axis",
                    0,
                    &chain->gravity_horizontal_source_sign,
                    sc->path);
            chain->gravity_horizontal_tail_axis_explicit =
                sidecar_profile_key_exists(s, "gravity_horizontal_tail_axis",
                                           sc->path);
            chain->gravity_horizontal_tail_axis =
                sidecar_profile_signed_axis(
                    s,
                    "gravity_horizontal_tail_axis",
                    chain->rotation_drive_horizontal_tail_axis,
                    &gravity_h_tail_sign,
                    sc->path);
            chain->gravity_horizontal_scale =
                profile_float(
                    s,
                    "gravity_horizontal_scale",
                    chain->rotation_drive_horizontal_scale < 0.0f ?
                        -1.0f : 1.0f,
                    sc->path) * gravity_h_tail_sign;
            chain->gravity_vertical_source_axis =
                sidecar_profile_signed_axis(
                    s,
                    "gravity_vertical_source_axis",
                    1,
                    &chain->gravity_vertical_source_sign,
                    sc->path);
            chain->gravity_vertical_tail_axis_explicit =
                sidecar_profile_key_exists(s, "gravity_vertical_tail_axis",
                                           sc->path);
            chain->gravity_vertical_tail_axis =
                sidecar_profile_signed_axis(
                    s,
                    "gravity_vertical_tail_axis",
                    chain->rotation_drive_vertical_tail_axis,
                    &gravity_v_tail_sign,
                    sc->path);
            chain->gravity_vertical_scale =
                profile_float(
                    s,
                    "gravity_vertical_scale",
                    chain->rotation_drive_vertical_scale < 0.0f ?
                        -1.0f : 1.0f,
                    sc->path) * gravity_v_tail_sign;
            chain->gravity_inverted_configured =
                sidecar_profile_key_exists(
                    s, "gravity_inverted_strength", sc->path) ||
                sidecar_profile_key_exists(
                    s, "gravity_inverted_tail_axis", sc->path) ||
                sidecar_profile_key_exists(
                    s, "gravity_inverted_sign", sc->path);
            chain->gravity_inverted_strength = physx_clampf(
                profile_float(s, "gravity_inverted_strength", 0.0f,
                              sc->path),
                0.0f, 120.0f);
            chain->gravity_inverted_tail_axis = sidecar_profile_axis(
                s, "gravity_inverted_tail_axis",
                chain->gravity_vertical_tail_axis, sc->path);
            chain->gravity_inverted_sign = physx_clampf(
                profile_float(s, "gravity_inverted_sign", 1.0f,
                              sc->path),
                -1.0f, 1.0f);
            chain->joint_gain = profile_float(s, "joint_gain", 1.5f, sc->path);
            chain->skinned_matrix_enabled = profile_bool(s, "skinned_matrix_enabled", 1, sc->path);
            chain->skinned_matrix_translation_enabled =
                profile_bool(s, "skinned_matrix_translation_enabled", 0, sc->path);
            chain->skinned_matrix_scale =
                physx_clampf(profile_float(s, "skinned_matrix_scale", 1.0f, sc->path),
                             0.0f, 1.0f);
            chain->root_bend_scale =
                physx_clampf(profile_float(s, "root_bend_scale", 1.0f, sc->path),
                             0.0f, 4.0f);
            chain->gravity_scale = profile_float(s, "gravity_scale", 1.0f, sc->path);
            if (!chain->gravity_enabled) chain->gravity_scale = 0.0f;
            chain->max_offset = profile_float(s, "max_offset", 0.12f, sc->path);
            chain->startup_impulse = profile_float(s, "startup_impulse", 0.0f, sc->path);
            chain->gravity[0] = defaults_cfg.gravity[0];
            chain->gravity[1] = defaults_cfg.gravity[1];
            chain->gravity[2] = defaults_cfg.gravity[2];
            GetPrivateProfileStringA(s, "gravity", "", gravity, sizeof(gravity), sc->path);
            parse_vec3(gravity, chain->gravity);
            GetPrivateProfileStringA(s, "parent", "", explicit_parent, sizeof(explicit_parent), sc->path);
            trim_in_place(explicit_parent);
            if (explicit_parent[0]) {
                lstrcpynA(chain->parent_name, explicit_parent, sizeof(chain->parent_name));
                lstrcpynA(chain->parent_source, "ini", sizeof(chain->parent_source));
            } else if (infer_addon_parent_from_scene_a(sc->path, chain->name,
                                                       chain->parent_name,
                                                       sizeof(chain->parent_name))) {
                lstrcpynA(chain->parent_source, "scene", sizeof(chain->parent_source));
            }
            GetPrivateProfileStringA(s, "children", "", children, sizeof(children), sc->path);
            trim_in_place(children);
            if (chain->parent_name[0] && _stricmp(chain->parent_name, chain->name) != 0) {
                if (children[0]) {
                    _snprintf(list, sizeof(list), "%s,%s,%s", chain->parent_name, chain->name, children);
                } else {
                    _snprintf(list, sizeof(list), "%s,%s", chain->parent_name, chain->name);
                }
                lstrcpynA(chain->anchor_name, chain->parent_name, sizeof(chain->anchor_name));
                lstrcpynA(chain->attach_name, chain->parent_name, sizeof(chain->attach_name));
            } else if (children[0]) {
                _snprintf(list, sizeof(list), "%s,%s", chain->name, children);
            } else {
                lstrcpynA(list, chain->name, sizeof(list));
            }
            parse_target_list(chain, list);
            {
                int target_index;
                int simulated_start =
                    chain->parent_name[0] &&
                    _stricmp(chain->parent_name, chain->name) != 0 ? 1 : 0;
                int collision_segment_count =
                    chain->target_count > 1 ? chain->target_count - 1 : 0;
                for (target_index = 0;
                     target_index < chain->target_count;
                     target_index++) {
                    chain->targets[target_index].addon_simulated_target =
                        target_index >= simulated_start;
                }
                /* Adjacent links intentionally do not collide with each other.
                   Self collision therefore needs at least three simulated
                   segments before a non-adjacent pair can exist. */
                if ((chain->collision_scope & PHYSX_COLLISION_SCOPE_SELF) &&
                    collision_segment_count < 3) {
                    chain->collision_scope &= ~PHYSX_COLLISION_SCOPE_SELF;
                    log_line("addon sidecar self collision ignored section=\"%s\" collision_segments=%d sidecar=\"%s\" reason=\"self collision requires at least three segments so a non-adjacent pair exists\"",
                             s, collision_segment_count, sc->path);
                    if (!chain->collision_scope) {
                        chain->collision_enabled = 0;
                    }
                }
            }
            addon_object_chain_init_scene_metadata(sc, chain);
            addon_chain_init_target_joint_settings(chain);
            addon_chain_init_target_gravity_settings(chain);
            if (chain->simulate) sc->enabled = 1;
            if (chain->gravity_inverted_configured) {
                log_line("addon sidecar inverted gravity configured chain=\"%s\" strength=%.3f tail_axis=%d sign=%.3f section=\"%s\" sidecar=\"%s\"",
                         chain->name,
                         chain->gravity_inverted_strength,
                         chain->gravity_inverted_tail_axis,
                         chain->gravity_inverted_sign,
                         s,
                         sc->path);
            }
            log_line("addon sidecar chain loaded section=\"%s\" chain=\"%s\" parent=\"%s\" parent_source=\"%s\" internal_type=\"%s\" rotation_solver=\"%s\" enabled=%d probe_only=%d gravity_enabled=%d collision_enabled=%d collision_scope=0x%x collision_custom_enabled=%d collision_custom_targets=%d collision_custom_persons=\"%s\" collision_radius=%.4f collision_debug_draw=%d collision_debug_color=0x%08lx targets=%d stiffness=%.3f damping=%.3f limit=%.1f limit_min=(%.1f,%.1f,%.1f) limit_max=(%.1f,%.1f,%.1f) drive_scale=%.3f drive_strength=%.3f translation_h=(source_axis=%d tail_axis=%d scale=%.3f) translation_v=(source_axis=%d tail_axis=%d scale=%.3f) translation_depth=(source_axis=%d tail_axis=%d scale=%.3f) rotation_h=(source_axis=%d tail_axis=%d scale=%.3f) rotation_v=(source_axis=%d tail_axis=%d scale=%.3f) rotation_twist=(source_axis=%d tail_axis=%d scale=%.3f) gravity_h=(source_axis=%d source_sign=%.1f tail_axis=%d tail_explicit=%d scale=%.3f) gravity_v=(source_axis=%d source_sign=%.1f tail_axis=%d tail_explicit=%d scale=%.3f) joint_gain=%.3f skinned_matrix=%d skinned_matrix_translation=%d skinned_matrix_scale=%.3f root_bend_scale=%.3f sidecar=\"%s\"",
                     s, chain->name, chain->parent_name, chain->parent_source,
                     chain->type,
                     chain->rotation_solver_full_angle ? "full_angle" : "legacy",
                     chain->simulate,
                     addon_physics_probe_enabled && !addon_physics_enabled,
                     chain->gravity_enabled, chain->collision_enabled,
                     chain->collision_scope,
                     chain->collision_custom_enabled,
                     chain->collision_custom_target_count,
                     chain->collision_custom_all_persons ? "all" : "wearer",
                     chain->collision_radius,
                     chain->collision_debug_draw,
                     (unsigned long)chain->collision_debug_color,
                     chain->target_count,
                     chain->stiffness, chain->damping, chain->limit_angle,
                     chain->joint_min_angle[0],
                     chain->joint_min_angle[1],
                     chain->joint_min_angle[2],
                     chain->joint_max_angle[0],
                     chain->joint_max_angle[1],
                     chain->joint_max_angle[2],
                     chain->drive_scale, chain->drive_strength,
                     chain->translation_horizontal_source_axis,
                     chain->translation_horizontal_tail_axis,
                     chain->translation_horizontal_scale,
                     chain->translation_vertical_source_axis,
                     chain->translation_vertical_tail_axis,
                     chain->translation_vertical_scale,
                     chain->translation_depth_source_axis,
                     chain->translation_depth_tail_axis,
                     chain->translation_depth_scale,
                     chain->rotation_drive_horizontal_source_axis,
                     chain->rotation_drive_horizontal_tail_axis,
                     chain->rotation_drive_horizontal_scale,
                     chain->rotation_drive_vertical_source_axis,
                     chain->rotation_drive_vertical_tail_axis,
                     chain->rotation_drive_vertical_scale,
                     chain->rotation_drive_twist_source_axis,
                     chain->rotation_drive_twist_tail_axis,
                     chain->rotation_drive_twist_scale,
                     chain->gravity_horizontal_source_axis,
                     chain->gravity_horizontal_source_sign,
                     chain->gravity_horizontal_tail_axis,
                     chain->gravity_horizontal_tail_axis_explicit,
                     chain->gravity_horizontal_scale,
                     chain->gravity_vertical_source_axis,
                     chain->gravity_vertical_source_sign,
                     chain->gravity_vertical_tail_axis,
                     chain->gravity_vertical_tail_axis_explicit,
                     chain->gravity_vertical_scale,
                     chain->joint_gain, chain->skinned_matrix_enabled,
                     chain->skinned_matrix_translation_enabled,
                     chain->skinned_matrix_scale,
                     chain->root_bend_scale,
                     sc->path);
            continue;
        }
        GetPrivateProfileStringA(s, "type", legacy_object_section ? "object" : "transform_chain", chain->type, sizeof(chain->type), sc->path);
        GetPrivateProfileStringA(s, "attach", "", chain->attach_name, sizeof(chain->attach_name), sc->path);
        trim_in_place(chain->attach_name);
        GetPrivateProfileStringA(s, "anchor", "", chain->anchor_name, sizeof(chain->anchor_name), sc->path);
        trim_in_place(chain->anchor_name);
        GetPrivateProfileStringA(s, "drive", "", chain->drive_name, sizeof(chain->drive_name), sc->path);
        trim_in_place(chain->drive_name);
        {
            char drive_offset[64];
            GetPrivateProfileStringA(s, "drive_offset", "", drive_offset, sizeof(drive_offset), sc->path);
            trim_in_place(drive_offset);
            chain->drive_offset_override = parse_offset_value(drive_offset, -1);
        }
        if (!chain->attach_name[0] && chain->anchor_name[0]) {
            lstrcpynA(chain->attach_name, chain->anchor_name, sizeof(chain->attach_name));
            if (!chain->anchor_alias_logged) {
                chain->anchor_alias_logged = 1;
                log_line("sidecar chain=\"%s\" uses legacy anchor=\"%s\"; treating it as attach diagnostic only",
                         chain->name, chain->anchor_name);
            }
        }
        chain->stiffness = profile_float(s, "stiffness", defaults_cfg.stiffness, sc->path);
        chain->damping = profile_float(s, "damping", defaults_cfg.damping, sc->path);
        chain->limit_angle = profile_float(s, "limit_angle", defaults_cfg.limit_angle, sc->path);
        chain->simulate = profile_bool(s, "simulate", 0, sc->path);
        chain->gravity_scale = profile_float(s, "gravity_scale", 0.25f, sc->path);
        chain->max_offset = profile_float(s, "max_offset", 0.12f, sc->path);
        chain->startup_impulse = profile_float(s, "startup_impulse", 0.18f, sc->path);
        chain->gravity[0] = defaults_cfg.gravity[0];
        chain->gravity[1] = defaults_cfg.gravity[1];
        chain->gravity[2] = defaults_cfg.gravity[2];
        GetPrivateProfileStringA(s, "gravity", "", gravity, sizeof(gravity), sc->path);
        parse_vec3(gravity, chain->gravity);
        GetPrivateProfileStringA(s, "nodes", "", list, sizeof(list), sc->path);
        if (!list[0]) GetPrivateProfileStringA(s, "bones", "", list, sizeof(list), sc->path);
        if (!list[0]) GetPrivateProfileStringA(s, "target", "", list, sizeof(list), sc->path);
        parse_target_list(chain, list);
    }
    addon_sidecar_apply_target_joint_overrides(sc, sections);
    {
        int definition_count = sc->chain_count;
        int person, definition;
        for (definition = 0; definition < definition_count; definition++) {
            physx_chain_t *chain = &sc->chains[definition];
            chain->rigid_simulation =
                chain->object_transform_chain ||
                _stricmp(chain->type, "rigid_chain") == 0 ||
                _stricmp(chain->type, "rigid_pendulum") == 0;
            chain->inertial_simulation =
                _stricmp(chain->type, "inertial_chain") == 0 ||
                _stricmp(chain->type, "inertial_pendulum") == 0;
        }
        if (sc->room_scene_sidecar) {
            for (definition = 0; definition < definition_count; definition++) {
                sc->chains[definition].addon_owner_person[0] = 0;
            }
            sc->chain_count = definition_count;
            log_line("room sidecar runtime instances created definitions=%d runtime_chains=%d sidecar=\"%s\" note=\"room joints exist once and are not cloned for Person01-04\"",
                     definition_count, sc->chain_count, sc->path);
        } else {
            for (person = 3; person >= 0; person--) {
                char owner[16];
                _snprintf(owner, sizeof(owner), "Person%02d", person + 1);
                for (definition = 0; definition < definition_count; definition++) {
                    physx_chain_t *source = &sc->chains[definition];
                    physx_chain_t *instance =
                        &sc->chains[person * definition_count + definition];
                    if (instance != source) {
                        memcpy(instance, source, sizeof(*instance));
                    }
                    if (instance->addon_chain) {
                        lstrcpynA(instance->addon_owner_person, owner,
                                  sizeof(instance->addon_owner_person));
                    }
                }
            }
            if (definition_count > 0) {
                sc->chain_count = definition_count * 4;
            }
            log_line("addon sidecar runtime instances created definitions=%d persons=4 runtime_chains=%d sidecar=\"%s\" note=\"each PersonXX owns independent bindings, solver state, collision state, and visual output\"",
                     definition_count, sc->chain_count, sc->path);
        }
    }
    if (defaults_cfg.debug) {
        log_line("sidecar loaded enabled=%d chains=%d write_test=%d write_test_target=\"%s\" axis=%d amount=%.5f duration_ms=%d path=\"%s\"",
                 sc->enabled, sc->chain_count, sc->write_test,
                 sc->write_test_target, sc->write_test_axis,
                 sc->write_test_amount, sc->write_test_duration_ms, sc->path);
    } else {
        log_line("sidecar loaded enabled=%d chains=%d path=\"%s\"",
                 sc->enabled, sc->chain_count, sc->path);
    }
}

static void scan_sidecar_dir(const char *dir, int depth)
{
    char pattern[MAX_PATH * 4];
    WIN32_FIND_DATAA data;
    HANDLE h;
    if (!dir || depth > 7) return;
    _snprintf(pattern, sizeof(pattern), "%s\\*", dir);
    h = FindFirstFileA(pattern, &data);
    if (h == INVALID_HANDLE_VALUE) return;
    do {
        char path[MAX_PATH * 4];
        if (strcmp(data.cFileName, ".") == 0 || strcmp(data.cFileName, "..") == 0) continue;
        _snprintf(path, sizeof(path), "%s\\%s", dir, data.cFileName);
        if (data.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) {
            scan_sidecar_dir(path, depth + 1);
        } else if (ends_with_i(data.cFileName, ".bs") &&
                   contains_i(data.cFileName, "DcDress") &&
                   (contains_i(path, "\\Scripts\\Dress\\") ||
                    contains_i(path, "/Scripts/Dress/"))) {
            /* This is a one-time metadata pass over small dress scripts.  It
               gives every add-on the engine PrimaryZone that governs mutual
               replacement, including non-PhysX choices that must retire a
               previously active PhysX add-on safely. */
            addon_equipment_scan_dress_script(path);
        } else if (ends_with_i(data.cFileName, ".physx.ini")) {
            /* Room [wind]/[collision] sidecars are environment profiles, not
               wearable add-ons. Register them in their own room-keyed
               lifecycle before the Person01-04 parser considers this file. */
            room_wind_register_sidecar_a(path);
            room_collision_register_sidecar_a(path);
            if (body_profile_is_body_sidecar_name(data.cFileName)) {
                /* Body profile sidecars are registered here, then applied only
                   after the matching bodyXX.bs load is paired to a PersonXXBody
                   node.  Never apply them from the Addons scanner itself. */
                body_profile_register_sidecar_a(path);
                continue;
            }
            if (sidecar_has_addon_physx_sections_a(path)) {
                physx_sidecar_t *sc = find_or_add_sidecar(path);
                if (sc) {
                    if (!sc->addon_registered_logged) {
                        sc->addon_registered_logged = 1;
                        log_line("addon sidecar registered sidecar=\"%s\" source=\"opt-in-scan\" note=\"loaded only because the INI contains an [NC-TK17-PhysX:*] section\"",
                                 path);
                    }
                    load_sidecar(sc);
                }
            }
        }
    } while (FindNextFileA(h, &data));
    FindClose(h);
}

static void scan_sidecars(DWORD now)
{
    static int initial_addon_scan_done;
    static int addon_scan_disabled_logged;
    char addons_path[MAX_PATH * 4];
    DWORD attr;
    (void)now;
    if (!addon_physics_enabled && !addon_physics_probe_enabled) {
        if (!addon_scan_disabled_logged) {
            addon_scan_disabled_logged = 1;
            log_line("addon sidecar opt-in scan skipped reason=\"[addon_physics] enabled=false\" note=\"regular add-on physics scanner is disabled; body sidecars still load through body profile binding\"");
        }
        return;
    }
    if (initial_addon_scan_done) return;
    initial_addon_scan_done = 1;
    if (!physx_build_game_path_a("Addons", addons_path, sizeof(addons_path))) {
        log_line("addon sidecar opt-in scan skipped reason=\"game-root-unresolved\" note=\"could not build absolute Addons path from loaded PhysX DLL\"");
        return;
    }
    attr = GetFileAttributesA(addons_path);
    if (attr == INVALID_FILE_ATTRIBUTES || !(attr & FILE_ATTRIBUTE_DIRECTORY)) {
        char cwd[MAX_PATH * 4];
        cwd[0] = 0;
        GetCurrentDirectoryA(sizeof(cwd), cwd);
        log_line("addon sidecar opt-in scan skipped root=\"%s\" cwd=\"%s\" reason=\"Addons directory missing\" note=\"scan uses absolute path from the loaded DLL, not TK17 current directory\"",
                 addons_path, cwd);
        return;
    }
    log_line("addon sidecar opt-in scan started root=\"%s\" note=\"one-time scan; non-body add-on sidecars are loaded only when they contain [NC-TK17-PhysX:*]\"",
             addons_path);
    scan_sidecar_dir(addons_path, 0);
    {
        int definition_count = 0;
        int zone_count = 0;
        int i;
        for (i = 0; i < ADDON_EQUIPMENT_DEFINITION_COUNT; i++) {
            if (!addon_equipment_definitions[i].addon_id[0]) continue;
            definition_count++;
            zone_count += addon_equipment_definitions[i].zone_count;
        }
        log_line("addon equipment metadata ready definitions=%d zones=%d note=\"one-time DressDescription PrimaryZone map gives PhysX and non-PhysX add-ons the same replacement lifecycle without adding per-frame file work\"",
                 definition_count, zone_count);
    }
}

static void physx_note_addon_scene_file_a(const char *scene_path)
{
    char sidecar_path[MAX_PATH * 4];
    DWORD attr;
    physx_sidecar_t *sc;
    static int addon_scene_disabled_logged;
    if (!scene_path || !ends_with_i(scene_path, ".bs")) return;
    if (body_profile_body_slot_from_name_a(scene_path, NULL)) return;
    if (!addon_physics_enabled && !addon_physics_probe_enabled) {
        if (!addon_scene_disabled_logged) {
            addon_scene_disabled_logged = 1;
            log_line("addon sidecar scene activation skipped scene=\"%s\" reason=\"[addon_physics] enabled=false\" note=\"regular add-on adjacent sidecars are disabled until the add-on live binding path is proven\"",
                     scene_path);
        }
        return;
    }
    if (!build_adjacent_physx_sidecar_path_a(scene_path, sidecar_path, sizeof(sidecar_path))) return;
    attr = GetFileAttributesA(sidecar_path);
    if (attr == INVALID_FILE_ATTRIBUTES || (attr & FILE_ATTRIBUTE_DIRECTORY)) return;
    if (body_profile_is_body_sidecar_name(sidecar_path)) return;
    sc = find_or_add_sidecar(sidecar_path);
    if (!sc) {
        log_line("addon sidecar register failed scene=\"%s\" sidecar=\"%s\" reason=\"sidecar table full\"",
                 scene_path, sidecar_path);
        return;
    }
    if (!sc->addon_registered_logged) {
        sc->addon_registered_logged = 1;
        log_line("addon sidecar registered scene=\"%s\" sidecar=\"%s\" note=\"exact .bs load activated adjacent .physx.ini without recursive Addons scan\"",
                 scene_path, sidecar_path);
    }
    sc->addon_scene_active = 1;
    sc->addon_scene_active_tick = GetTickCount();
    load_sidecar(sc);
    {
        char addon_id[256];
        if (sidecar_addon_identifier(sc, addon_id, sizeof(addon_id))) {
            addon_selection_note_base_scene(addon_id,
                                            sc->addon_scene_active_tick);
        }
    }
}

static void run_sidecar_hot_reload(DWORD now)
{
    static DWORD last_hot_reload_tick;
    static int next_hot_reload_index;
    int checked = 0;
    int visited = 0;
    int max_checks = addon_sidecar_hot_reload_max_checks_per_tick;
    if (!addon_sidecar_hot_reload_enabled) return;
    if (!addon_physics_enabled && !addon_physics_probe_enabled) return;
    if (sidecar_count <= 0) return;
    if (addon_sidecar_hot_reload_interval_ms > 0 &&
        last_hot_reload_tick &&
        now - last_hot_reload_tick <
            (DWORD)addon_sidecar_hot_reload_interval_ms) {
        return;
    }
    last_hot_reload_tick = now;
    if (max_checks < 1) max_checks = 1;
    if (next_hot_reload_index < 0 ||
        next_hot_reload_index >= sidecar_count) {
        next_hot_reload_index = 0;
    }
    while (checked < max_checks && visited < sidecar_count) {
        int index = next_hot_reload_index++;
        physx_sidecar_t *sc;
        FILETIME previous_write_time;
        if (next_hot_reload_index >= sidecar_count) {
            next_hot_reload_index = 0;
        }
        visited++;
        sc = &sidecars[index];
        if (!sc->loaded || !sc->addon_scene_active) continue;
        checked++;
        previous_write_time = sc->write_time;
        load_sidecar(sc);
        if (sc->loaded &&
            filetime_differs(&previous_write_time, &sc->write_time)) {
            log_line("addon sidecar hot-reloaded sidecar=\"%s\" index=%d active_tick=%lu interval_ms=%d max_checks_per_tick=%d note=\"only this active sidecar was reparsed; other add-ons were left untouched\"",
                     sc->path,
                     index,
                     (unsigned long)sc->addon_scene_active_tick,
                     addon_sidecar_hot_reload_interval_ms,
                     addon_sidecar_hot_reload_max_checks_per_tick);
        }
    }
}

static void sync_room_sidecar_scene_activity(DWORD now)
{
    int i;
    for (i = 0; i < sidecar_count; i++) {
        physx_sidecar_t *sc = &sidecars[i];
        int active;
        if (!sc->loaded || !sc->room_scene_sidecar) continue;
        active = room_wind_cfg.active && room_wind_cfg.path[0] &&
                 _stricmp(room_wind_cfg.path, sc->path) == 0;
        if (active && !sc->addon_scene_active) {
            sc->addon_scene_active = 1;
            sc->addon_scene_active_tick = now;
            log_line("room PhysX sidecar activated sidecar=\"%s\" wind_enabled=%d note=\"room bone chains now follow the same room-keyed lifecycle as [wind]\"",
                     sc->path, room_wind_cfg.enabled);
        } else if (!active && sc->addon_scene_active) {
            sc->addon_scene_active = 0;
            sc->addon_scene_active_tick = 0;
            log_line("room PhysX sidecar deactivated sidecar=\"%s\" note=\"current room no longer owns this sidecar; animation ownership and cached outputs will be released\"",
                     sc->path);
        }
    }
}

static void run_addon_binding_probe(DWORD now)
{
    int i, c, t;
    if (!addon_physics_probe_enabled) return;
    for (i = 0; i < sidecar_count; i++) {
        physx_sidecar_t *sc = &sidecars[i];
        char base_name[256];
        if (!sc->loaded) continue;
        sidecar_scene_basename_no_ext(sc->path, base_name, sizeof(base_name));
        for (c = 0; c < sc->chain_count; c++) {
            physx_chain_t *chain = &sc->chains[c];
            void *parent_obj = NULL;
            void *armature_obj = NULL;
            void *pivot_armature_obj = NULL;
            void *main_armature_obj = NULL;
            void *local_armature_obj = NULL;
            int found_pairs = 0;
            if (!chain->addon_chain) continue;
            if (chain->binding_probe_count >= 80) continue;
            if (chain->binding_probe_tick &&
                now - chain->binding_probe_tick < 2000) {
                continue;
            }
            chain->binding_probe_tick = now;
            chain->binding_probe_count++;
            {
                char local_parent[192];
                _snprintf(local_parent, sizeof(local_parent), "local_%s", chain->parent_name);
                parent_obj = find_named_node(chain->parent_name);
                if (!parent_obj) parent_obj = find_named_node(local_parent);
            }
            armature_obj = find_named_node("root_rotation_group");
            pivot_armature_obj = find_named_node("root_pivot_rotation_group");
            main_armature_obj = find_named_node("root_main_group");
            local_armature_obj = find_named_node("local_root_rotation_group");
            if (defaults_cfg.debug) {
                log_line("addon binding-probe chain=\"%s\" sidecar_base=\"%s\" parent=\"%s\" parent_object=%p armature=%p pivot_armature=%p main_armature=%p local_armature=%p target_count=%d probe=%d note=\"read-only; no SearchTree, no ComponentArray traversal, no transform writes\"",
                         chain->name,
                         base_name,
                         chain->parent_name,
                         parent_obj,
                         armature_obj,
                         pivot_armature_obj,
                         main_armature_obj,
                         local_armature_obj,
                         chain->target_count,
                         chain->binding_probe_count);
            }
            for (t = 0; t < chain->target_count; t++) {
                physx_target_t *target = &chain->targets[t];
                char s_name[192];
                char local_name[192];
                char local_s_name[192];
                void *obj;
                void *s_obj;
                _snprintf(s_name, sizeof(s_name), "S%s", target->name);
                _snprintf(local_name, sizeof(local_name), "local_%s", target->name);
                _snprintf(local_s_name, sizeof(local_s_name), "local_S%s", target->name);
                obj = find_named_node(target->name);
                if (!obj) obj = find_named_node(local_name);
                s_obj = find_named_node(s_name);
                if (!s_obj) s_obj = find_named_node(local_s_name);
                if (obj && s_obj) found_pairs++;
                if (defaults_cfg.debug) {
                    log_line("addon binding-probe target chain=\"%s\" target=\"%s\" object=%p local=\"%s\" s_target=\"%s\" s_object=%p local_s=\"%s\" pair=%d sidecar=\"%s\"",
                             chain->name, target->name, obj, local_name,
                             s_name, s_obj, local_s_name, (obj && s_obj) ? 1 : 0, sc->path);
                }
            }
            if (parent_obj && found_pairs >= chain->target_count) {
                log_line("addon binding-probe ready chain=\"%s\" parent=\"%s\" found_pairs=%d target_count=%d sidecar=\"%s\" note=\"live Object.Name candidates exist after delayed room load; next step can bind targets after a settle window, still without matrix writes\"",
                         chain->name, chain->parent_name, found_pairs,
                         chain->target_count, sc->path);
                chain->binding_probe_count = 80;
            }
        }
    }
}

static void physx_note_addon_scene_file_w(const WCHAR *scene_path)
{
    char path[MAX_PATH * 4];
    int ok;
    if (!scene_path) return;
    path[0] = 0;
    ok = WideCharToMultiByte(CP_ACP, 0, scene_path, -1, path, sizeof(path), NULL, NULL);
    if (!ok) return;
    path[sizeof(path) - 1] = 0;
    physx_note_addon_scene_file_a(path);
}

static int addon_target_retry_deferred(physx_target_t *target, DWORD now)
{
    if (!target || !target->addon_simulated_target) return 0;
    return target->addon_resolve_retry_tick &&
           now < target->addon_resolve_retry_tick;
}

static void addon_target_note_resolve_success(physx_target_t *target)
{
    if (!target) return;
    target->addon_resolve_miss_count = 0;
    target->addon_resolve_retry_tick = 0;
    target->addon_resolve_backoff_logged = 0;
}

static void addon_target_note_resolve_miss(physx_target_t *target,
                                           const char *chain_name,
                                           const char *sidecar_path,
                                           DWORD now)
{
    DWORD delay_ms;
    if (!target || !target->addon_simulated_target) return;
    target->addon_resolve_miss_count++;
    if (target->addon_resolve_miss_count < 4) {
        delay_ms = 1000u;
    } else if (target->addon_resolve_miss_count < 8) {
        delay_ms = 5000u;
    } else {
        delay_ms = 15000u;
    }
    target->addon_resolve_retry_tick = now + delay_ms;
    if (!target->addon_resolve_backoff_logged &&
        target->addon_resolve_miss_count >= 4) {
        target->addon_resolve_backoff_logged = 1;
        log_line("addon target resolve backoff chain=\"%s\" target=\"%s\" miss_count=%d retry_ms=%lu sidecar=\"%s\" note=\"custom add-on bone is not exposed as a live TK17 runtime name; throttling retries to protect FPS\"",
                 chain_name ? chain_name : "",
                 target->name,
                 target->addon_resolve_miss_count,
                 (unsigned long)delay_ms,
                 sidecar_path ? sidecar_path : "");
    }
}

static void invalidate_addon_target_binding(physx_target_t *target);

static int addon_chain_settling(physx_chain_t *chain, DWORD now)
{
    if (!chain || !chain->addon_root_settle_until_tick) return 0;
    if (now < chain->addon_root_settle_until_tick) return 1;
    chain->addon_root_settle_until_tick = 0;
    return 0;
}

static void addon_chain_reset_runtime_state(physx_chain_t *chain)
{
    int t;
    if (!chain) return;
    chain->anchor_raw_object = NULL;
    chain->anchor_object = NULL;
    chain->anchor_vector = NULL;
    chain->anchor_offset = -1;
    chain->anchor_initialized = 0;
    chain->drive_auto_initialized = 0;
    chain->drive_auto_logged = 0;
    chain->drive_auto_offset = -1;
    chain->addon_parent_rotation_base = NULL;
    chain->addon_parent_rotation_offset = -1;
    chain->addon_parent_rotation_initialized = 0;
    chain->addon_parent_rotation_logged = 0;
    chain->addon_parent_rotation_log_tick = 0;
    chain->addon_parent_rotation_auto_base = NULL;
    chain->addon_parent_rotation_auto_logged = 0;
    chain->addon_parent_rotation_auto_log_tick = 0;
    chain->addon_body_root_person[0] = 0;
    chain->addon_body_root_raw = NULL;
    chain->addon_body_root_initialized = 0;
    chain->addon_body_root_prev[0] = 0.0f;
    chain->addon_body_root_prev[1] = 0.0f;
    chain->addon_body_root_prev[2] = 0.0f;
    chain->addon_body_root_log_tick = 0;
    chain->addon_body_root_miss_log_tick = 0;
    chain->addon_parent_runtime_raw = NULL;
    chain->addon_parent_runtime_name[0] = 0;
    chain->addon_parent_runtime_parent[0] = 0;
    chain->addon_parent_runtime_owner[0] = 0;
    chain->addon_parent_runtime_resolve_tick = 0;
    chain->addon_parent_runtime_log_tick = 0;
    chain->addon_parent_runtime_miss_log_tick = 0;
    chain->addon_parent_camera_relative_trs_raw = NULL;
    chain->addon_parent_camera_relative_parent_raw = NULL;
    chain->addon_parent_camera_relative_initialized = 0;
    memset(chain->addon_parent_camera_relative_prev, 0,
           sizeof(chain->addon_parent_camera_relative_prev));
    chain->addon_parent_camera_relative_log_tick = 0;
    chain->addon_parent_camera_relative_miss_log_tick = 0;
    chain->addon_parent_translation_camera_relative = 0;
    chain->addon_parent_rotation_camera_relative = 0;
    chain->addon_parent_rotation_camera_relative_available = 0;
    chain->addon_parent_camera_relative_translation_raw = NULL;
    chain->addon_parent_camera_relative_translation_offset = -1;
    chain->addon_parent_camera_relative_translation_mode = 0;
    chain->addon_parent_camera_relative_translation_initialized = 0;
    chain->addon_parent_camera_relative_translation_prev[0] = 0.0f;
    chain->addon_parent_camera_relative_translation_prev[1] = 0.0f;
    chain->addon_parent_camera_relative_translation_prev[2] = 0.0f;
    chain->addon_parent_camera_relative_translation_basis_initialized = 0;
    memset(chain->addon_parent_camera_relative_translation_parent_rest, 0,
           sizeof(chain->addon_parent_camera_relative_translation_parent_rest));
    memset(chain->addon_parent_camera_relative_translation_model_rest, 0,
           sizeof(chain->addon_parent_camera_relative_translation_model_rest));
    chain->addon_parent_camera_relative_translation_global_initialized = 0;
    memset(chain->addon_parent_camera_relative_translation_global_prev, 0,
           sizeof(chain->addon_parent_camera_relative_translation_global_prev));
    chain->addon_parent_camera_relative_translation_camera_hold_active = 0;
    chain->addon_parent_camera_relative_translation_log_tick = 0;
    chain->addon_root_drive_camera_seen_version = 0;
    chain->addon_root_drive_camera_quarantine_tick = 0;
    chain->addon_root_drive_camera_last_untrusted_tick = 0;
    chain->addon_root_drive_camera_log_tick = 0;
    chain->addon_gravity_trusted_drive[0] = 0.0f;
    chain->addon_gravity_trusted_drive[1] = 0.0f;
    chain->addon_gravity_trusted_drive[2] = 0.0f;
    chain->addon_gravity_trusted_valid = 0;
    memset(&chain->addon_gravity_sample,0,sizeof(chain->addon_gravity_sample));
    chain->addon_gravity_camera_hold_active = 0;
    chain->addon_gravity_camera_release_active = 0;
    chain->addon_gravity_camera_log_tick = 0;
    memset(chain->addon_wind_trusted_drive, 0,
           sizeof(chain->addon_wind_trusted_drive));
    chain->addon_wind_trusted_valid = 0;
    chain->addon_wind_camera_seen_version = 0;
    chain->addon_wind_camera_quarantine_tick = 0;
    chain->addon_wind_update_tick = 0;
    chain->addon_wind_generation = 0;
    chain->addon_wind_logged = 0;
    chain->addon_stationary_camera_hold_active = 0;
    chain->addon_stationary_parent_motion_tick = 0;
    chain->addon_gravity_diag_state = 0;
    chain->addon_gravity_diag_start_tick = 0;
    chain->addon_gravity_diag_status_tick = 0;
    chain->addon_gravity_diag_sample_count = 0;
    chain->addon_gravity_diag_done_logged = 0;
    chain->addon_gravity_diag_any_valid = 0;
    chain->addon_gravity_diag_drive[0] = 0.0f;
    chain->addon_gravity_diag_drive[1] = 0.0f;
    chain->addon_gravity_diag_drive[2] = 0.0f;
    chain->binding_probe_tick = 0;
    chain->binding_probe_count = 0;
    chain->skin_probe_tick = 0;
    chain->skin_probe_count = 0;
    chain->skin_probe_found = 0;
    chain->skin_probe_found_tick = 0;
    chain->skin_memory_scan_armed_logged = 0;
    chain->addon_live_layout_pending = 0;
    chain->addon_live_layout_retry_tick = 0;
    for (t = 0; t < chain->target_count; t++) {
        physx_target_t *target = &chain->targets[t];
        invalidate_addon_target_binding(target);
        target->missing_logged = 0;
        target->nil_logged = 0;
        target->addon_resolve_miss_count = 0;
        target->addon_resolve_backoff_logged = 0;
        target->addon_resolve_retry_tick = 0;
        target->sim_started_logged = 0;
        target->sim_motion_probe_tick = 0;
        target->sim_rotation_probe_tick = 0;
        target->sim_world_anchor_initialized = 0;
        target->sim_velocity[0] = 0.0f;
        target->sim_velocity[1] = 0.0f;
        target->sim_velocity[2] = 0.0f;
        target->addon_gravity_diag_initialized = 0;
        target->addon_gravity_diag_samples = 0;
        target->addon_gravity_diag_start_offset[0] = 0.0f;
        target->addon_gravity_diag_start_offset[1] = 0.0f;
        target->addon_gravity_diag_start_offset[2] = 0.0f;
        target->addon_gravity_diag_max_delta = 0.0f;
        target->addon_gravity_diag_max_bend_len = 0.0f;
        target->addon_gravity_diag_last_delta[0] = 0.0f;
        target->addon_gravity_diag_last_delta[1] = 0.0f;
        target->addon_gravity_diag_last_delta[2] = 0.0f;
        target->addon_gravity_diag_last_bend[0] = 0.0f;
        target->addon_gravity_diag_last_bend[1] = 0.0f;
        target->addon_gravity_diag_last_bend[2] = 0.0f;
        target->addon_gravity_diag_last_rotation[0] = 0.0f;
        target->addon_gravity_diag_last_rotation[1] = 0.0f;
        target->addon_gravity_diag_last_rotation[2] = 0.0f;
        target->sim_offset[0] = 0.0f;
        target->sim_offset[1] = 0.0f;
        target->sim_offset[2] = 0.0f;
    }
}

static int addon_equipment_commit_live_root(const char *owner,
                                            const char *addon_id,
                                            void *root_object,
                                            DWORD now)
{
    addon_equipment_definition_t *definition;
    char previous_id[256];
    char zone_list[280];
    int person_index;
    int changed = 0;
    int i, c;
    if (!owner || !owner[0] || !addon_id || !addon_id[0] ||
        !root_object) {
        return 0;
    }
    definition = addon_equipment_definition_for_id(addon_id);
    if (!definition) return 0;
    previous_id[0] = 0;
    zone_list[0] = 0;
    for (i = 0; i < definition->zone_count; i++) {
        addon_equipment_slot_t *slot = addon_equipment_slot_find(
            owner, definition->zones[i], 1);
        if (!slot) continue;
        if (zone_list[0]) lstrcatA(zone_list, ",");
        if (strlen(zone_list) + strlen(definition->zones[i]) + 1 <
            sizeof(zone_list)) {
            lstrcatA(zone_list, definition->zones[i]);
        }
        if (_stricmp(slot->addon_id, addon_id) != 0 ||
            slot->root_object != root_object) {
            if (!previous_id[0] && slot->addon_id[0]) {
                lstrcpynA(previous_id, slot->addon_id,
                          sizeof(previous_id));
            }
            changed = 1;
        }
        lstrcpynA(slot->addon_id, addon_id, sizeof(slot->addon_id));
        slot->root_object = root_object;
        slot->root_tick = now;
    }
    if (!changed) return 1;

    /* Stop traversal output first, then invalidate every sidecar that belongs
       to an overlapping engine zone for this owner.  No stale target pointer
       survives long enough to reach ModelViewMatrix or ParentTransform after
       TK17 replaces the old scene tree. */
    InterlockedExchange(&addon_traverse_overlay_active, 0);
    person_index = addon_person_prefix_to_index(owner);
    for (i = 0; i < sidecar_count; i++) {
        physx_sidecar_t *candidate = &sidecars[i];
        addon_equipment_definition_t *candidate_definition;
        if (!candidate->loaded) continue;
        candidate_definition = addon_equipment_definition_for_sidecar(
            candidate);
        if (!addon_equipment_definitions_overlap(candidate_definition,
                                                 definition)) {
            continue;
        }
        if (person_index >= 0 && person_index < 4) {
            candidate->addon_live_root_cache_valid[person_index] = 0;
            candidate->addon_live_root_cache_value[person_index] = 0;
            candidate->addon_live_root_cache_tick[person_index] = 0;
        }
        for (c = 0; c < candidate->chain_count; c++) {
            physx_chain_t *chain = &candidate->chains[c];
            if (!chain->addon_chain ||
                _stricmp(chain->addon_owner_person, owner) != 0) {
                continue;
            }
            addon_chain_reset_runtime_state(chain);
            chain->addon_scene_visible = 0;
            chain->addon_root_seen_tick = 0;
            chain->addon_root_settle_until_tick = 0;
        }
    }
    for (i = 0; i < ADDON_ACTIVE_SLOT_COUNT; i++) {
        addon_active_slot_t *entry = &addon_active_slots[i];
        addon_equipment_definition_t *entry_definition;
        if (!entry->owner[0] ||
            _stricmp(entry->owner, owner) != 0 ||
            !entry->addon_id[0]) {
            continue;
        }
        entry_definition = addon_equipment_definition_for_id(entry->addon_id);
        if (addon_equipment_definitions_overlap(entry_definition,
                                                definition)) {
            entry->root_object = NULL;
        }
    }
    log_line("addon equipment root changed owner=\"%s\" zones=\"%s\" previous_addon_id=\"%s\" addon_id=\"%s\" root=%p note=\"uniform PrimaryZone lifecycle retired overlapping sidecar bindings before the old TK17 scene tree can be reused\"",
             owner, zone_list, previous_id, addon_id, root_object);
    return 1;
}

static int addon_chain_promote_live_sjoint_layout(physx_sidecar_t *sc,
                                                  physx_chain_t *chain,
                                                  DWORD now)
{
    int t;
    int ready = 1;
    if (!sc || !chain || !chain->addon_live_layout_pending) return 1;
    if (chain->addon_live_layout_retry_tick &&
        now < chain->addon_live_layout_retry_tick) {
        return 0;
    }
    chain->addon_live_layout_retry_tick = now + 200u;
    for (t = 0; t < chain->target_count; t++) {
        physx_target_t *target = &chain->targets[t];
        BYTE *live_base;
        float *row0;
        float *row1;
        float *row2;
        float *translation;
        if (!target->addon_simulated_target) continue;
        if (!target->s_object ||
            is_nil_engine_object(target->s_raw_object, target->s_object)) {
            ready = 0;
            continue;
        }
        target->s_translation_base = NULL;
        target->s_rotation_base = NULL;
        target->s_translation_offset = -1;
        target->s_rotation_offset = -1;
        target->s_translation_probe_logged = 0;
        target->s_rotation_probe_logged = 0;
        target->sim_initialized = 0;
        target->addon_visual_pose_valid = 0;
        assume_addon_s_transform_layout(target, sc->path);
        live_base = (BYTE*)target->s_translation_base;
        row0 = live_base ? (float*)(live_base + 0x018) : NULL;
        row1 = live_base ? (float*)(live_base + 0x028) : NULL;
        row2 = live_base ? (float*)(live_base + 0x038) : NULL;
        translation = live_base ? (float*)(live_base + 0x048) : NULL;
        if (target->s_rotation_offset != 0x038 ||
            target->s_translation_offset != 0x048 ||
            target->s_rotation_base != target->s_translation_base ||
            (target->s_translation_base != target->s_raw_object &&
             target->s_translation_base != target->s_object) ||
            !live_base ||
            !ptr_readable(live_base + 0x018, 0x03c) ||
            !addon_vector_basis_like(row0) ||
            !addon_vector_basis_like(row1) ||
            !addon_vector_basis_like(row2) ||
            !physx_vec3_sane_limit(translation, 64.0f)) {
            ready = 0;
        }
    }
    if (!ready) return 0;
    chain->addon_live_layout_pending = 0;
    chain->addon_live_layout_retry_tick = 0;
    log_line("addon-chain live SJoint layout promoted chain=\"%s\" rotation_offset=0x038 translation_offset=0x048 sidecar=\"%s\" note=\"all declared PhysX targets now expose traversal matrix rows; simulation may start without touching provisional scene transform/scale fields\"",
             chain->name,
             sc->path);
    return 1;
}

static void addon_chain_schedule_runtime_rebind(physx_sidecar_t *sc,
                                                physx_chain_t *chain,
                                                DWORD now,
                                                const char *reason,
                                                DWORD settle_ms)
{
    if (!sc || !chain || !chain->addon_chain) return;
    addon_chain_reset_runtime_state(chain);
    chain->addon_root_seen_tick = now;
    chain->addon_root_settle_until_tick = now + settle_ms;
    log_line("addon-chain runtime rebind scheduled chain=\"%s\" reason=\"%s\" root=\"%s\" settle_ms=%lu sidecar=\"%s\" note=\"cleared early cached add-on pointers so initial room load follows the same fresh-bind path as re-equip\"",
             chain->name,
             reason ? reason : "",
             addon_tsnode_window_root,
             (unsigned long)settle_ms,
             sc->path);
}

static addon_selection_hint_t *addon_selection_hint_find(
    const char *addon_id, int create)
{
    addon_selection_hint_t *empty = NULL;
    addon_selection_hint_t *oldest = NULL;
    DWORD oldest_age = 0;
    DWORD now = GetTickCount();
    int i;
    if (!addon_id || !addon_id[0]) return NULL;
    for (i = 0; i < ADDON_SELECTION_HINT_COUNT; i++) {
        addon_selection_hint_t *hint = &addon_selection_hints[i];
        if (hint->addon_id[0] &&
            _stricmp(hint->addon_id, addon_id) == 0) {
            return hint;
        }
        if (!hint->addon_id[0] && !empty) empty = hint;
        if (hint->addon_id[0] &&
            (!oldest || now - hint->tick > oldest_age)) {
            oldest = hint;
            oldest_age = now - hint->tick;
        }
    }
    if (!create) return NULL;
    if (!empty) empty = oldest;
    if (!empty) return NULL;
    memset(empty, 0, sizeof(*empty));
    lstrcpynA(empty->addon_id, addon_id, sizeof(empty->addon_id));
    return empty;
}

static void addon_selection_note_base_scene(const char *addon_id, DWORD now)
{
    addon_selection_hint_t *hint = addon_selection_hint_find(addon_id, 1);
    if (!hint) return;
    hint->use_activemod = 0;
    hint->activemod_folder[0] = 0;
    hint->tick = now;
}

static void addon_selection_note_activemod(const char *addon_id,
                                           const char *folder,
                                           DWORD now)
{
    addon_selection_hint_t *hint;
    if (!addon_id || !addon_id[0] || !folder || !folder[0]) return;
    hint = addon_selection_hint_find(addon_id, 1);
    if (!hint) return;
    hint->use_activemod = 1;
    lstrcpynA(hint->activemod_folder, folder,
              sizeof(hint->activemod_folder));
    hint->tick = now;
}

static int addon_sidecar_scene_references_texture(
    physx_sidecar_t *base_sc, const char *texture_name)
{
    char scene_path[MAX_PATH * 4];
    FILE *f;
    char line[2048];
    if (!base_sc || !texture_name || !texture_name[0] ||
        !build_adjacent_scene_path_from_sidecar_a(base_sc->path,
                                                   scene_path,
                                                   sizeof(scene_path))) {
        return 0;
    }
    f = fopen(scene_path, "rb");
    if (!f) return 0;
    while (fgets(line, sizeof(line), f)) {
        char *p;
        char *end;
        char *name;
        char *slash;
        char *dot;
        if (!strstr(line, "FileObject.FileName")) continue;
        p = strchr(line, '"');
        if (!p) continue;
        p++;
        end = strchr(p, '"');
        if (!end) continue;
        *end = 0;
        name = strrchr(p, '\\');
        slash = strrchr(p, '/');
        if (!name || (slash && slash > name)) name = slash;
        name = name ? name + 1 : p;
        dot = strrchr(name, '.');
        if (dot) *dot = 0;
        if (_stricmp(name, texture_name) == 0) {
            fclose(f);
            return 1;
        }
    }
    fclose(f);
    return 0;
}

static physx_sidecar_t *addon_activemod_sidecar_for(
    const char *folder, const char *addon_id, DWORD now)
{
    char path[MAX_PATH * 4];
    DWORD attr;
    physx_sidecar_t *sc;
    if (!folder || !folder[0] || !addon_id || !addon_id[0]) return NULL;
    _snprintf(path, sizeof(path), "%s\\%s.physx.ini", folder, addon_id);
    path[sizeof(path) - 1] = 0;
    attr = GetFileAttributesA(path);
    if (attr == INVALID_FILE_ATTRIBUTES || (attr & FILE_ATTRIBUTE_DIRECTORY) ||
        !sidecar_has_addon_physx_sections_a(path)) {
        return NULL;
    }
    sc = find_or_add_sidecar(path);
    if (!sc) {
        log_line("ActiveMod sidecar register failed addon_id=\"%s\" sidecar=\"%s\" reason=\"sidecar table full\"",
                 addon_id, path);
        return NULL;
    }
    if (!sc->addon_registered_logged) {
        sc->addon_registered_logged = 1;
        log_line("ActiveMod sidecar registered addon_id=\"%s\" folder=\"%s\" sidecar=\"%s\" note=\"variant is scoped to the selected live add-on instance; main add-on sidecar remains registered as fallback\"",
                 addon_id, folder, path);
    }
    sc->addon_scene_active = 1;
    sc->addon_scene_active_tick = now;
    load_sidecar(sc);
    return sc;
}

static void addon_switch_active_slot_sidecar(addon_active_slot_t *entry,
                                              physx_sidecar_t *active_sc,
                                              const char *root_name,
                                              void *root_object,
                                              DWORD now,
                                              const char *reason)
{
    physx_sidecar_t *old_sc;
    int person_index;
    int i;
    int c;
    if (!entry || !active_sc || !entry->owner[0] ||
        !entry->addon_id[0]) {
        return;
    }
    old_sc = entry->sidecar;
    if (old_sc == active_sc && entry->root_object == root_object) {
        entry->root_tick = now;
        return;
    }
    entry->sidecar = active_sc;
    entry->root_object = root_object;
    entry->root_tick = now;
    sidecar_note_live_addon_owner_person(active_sc, entry->owner,
                                         root_name, now);
    person_index = addon_person_prefix_to_index(entry->owner);

    addon_tsnode_window_until_tick = now + 3500u;
    addon_tsnode_window_count = 0;
    addon_tsnode_window_serial++;
    if (addon_tsnode_window_serial <= 0) addon_tsnode_window_serial = 1;
    lstrcpynA(addon_tsnode_window_root, root_name ? root_name : "",
              sizeof(addon_tsnode_window_root));

    for (i = 0; i < sidecar_count; i++) {
        physx_sidecar_t *candidate = &sidecars[i];
        char candidate_addon_id[256];
        if (!candidate->loaded ||
            !sidecar_addon_identifier(candidate, candidate_addon_id,
                                      sizeof(candidate_addon_id)) ||
            _stricmp(candidate_addon_id, entry->addon_id) != 0) {
            continue;
        }
        if (person_index >= 0 && person_index < 4) {
            candidate->addon_live_root_cache_valid[person_index] = 0;
            candidate->addon_live_root_cache_value[person_index] = 0;
            candidate->addon_live_root_cache_tick[person_index] = 0;
        }
        for (c = 0; c < candidate->chain_count; c++) {
            physx_chain_t *chain = &candidate->chains[c];
            if (!chain->addon_chain ||
                _stricmp(chain->addon_owner_person, entry->owner) != 0) {
                continue;
            }
            addon_chain_reset_runtime_state(chain);
            chain->addon_scene_visible = 0;
            chain->addon_root_window_serial = addon_tsnode_window_serial;
            if (candidate == active_sc) {
                chain->addon_root_seen_tick = now;
                chain->addon_root_settle_until_tick = now + 900u;
                chain->addon_live_layout_pending = 1;
                chain->addon_live_layout_retry_tick = now + 900u;
            } else {
                chain->addon_root_seen_tick = 0;
                chain->addon_root_settle_until_tick = 0;
            }
        }
    }
    log_line("addon sidecar selection changed owner=\"%s\" addon_id=\"%s\" root=\"%s\" object=%p previous_sidecar=\"%s\" active_sidecar=\"%s\" enabled=%d reason=\"%s\" settle_ms=900 note=\"main and ActiveMod configurations remain independently loaded; only this person/add-on instance changed selection\"",
             entry->owner, entry->addon_id,
             root_name ? root_name : "", root_object,
             old_sc ? old_sc->path : "", active_sc->path,
             active_sc->enabled, reason ? reason : "");
}

static physx_sidecar_t *addon_selected_sidecar_for_root(
    const char *addon_id, addon_active_slot_t *entry, DWORD now)
{
    addon_selection_hint_t *hint;
    physx_sidecar_t *base_sc;
    (void)entry;
    (void)now;
    base_sc = find_base_addon_sidecar_by_id(addon_id);
    if (!base_sc) return NULL;

    /* TK17 rebuilds the PersonXX<addon> root for main and ActiveMod choices,
       but cached scene execution does not necessarily reopen the physical .bs
       file when the user returns to the main texture.  The root rebuild is
       therefore the reliable main/default boundary.  ActiveMod loads its
       selected image immediately after this event and the file hook below
       promotes the matching variant for the same live slot. */
    hint = addon_selection_hint_find(addon_id, 0);
    if (hint) hint->tick = 0;
    return base_sc;
}

static void physx_note_activemod_file_a(const char *file_path)
{
    char folder[MAX_PATH * 4];
    char texture_name[256];
    char *slash;
    char *slash2;
    char *dot;
    physx_sidecar_t *matched_base = NULL;
    physx_sidecar_t *selected;
    addon_active_slot_t *newest_entry = NULL;
    DWORD newest_age = 0;
    DWORD now = GetTickCount();
    int i;
    if (!file_path || !sidecar_path_is_activemod_a(file_path) ||
        (!ends_with_i(file_path, ".png") &&
         !ends_with_i(file_path, ".jp2"))) {
        return;
    }
    lstrcpynA(folder, file_path, sizeof(folder));
    slash = strrchr(folder, '\\');
    slash2 = strrchr(folder, '/');
    if (!slash || (slash2 && slash2 > slash)) slash = slash2;
    if (!slash || !slash[1]) return;
    lstrcpynA(texture_name, slash + 1, sizeof(texture_name));
    *slash = 0;
    dot = strrchr(texture_name, '.');
    if (dot) *dot = 0;

    for (i = 0; i < sidecar_count; i++) {
        physx_sidecar_t *candidate = &sidecars[i];
        char addon_id[256];
        if (!candidate->loaded ||
            sidecar_path_is_activemod_a(candidate->path) ||
            !sidecar_addon_identifier(candidate, addon_id,
                                      sizeof(addon_id))) {
            continue;
        }
        if (_stricmp(addon_id, texture_name) == 0 ||
            addon_sidecar_scene_references_texture(candidate,
                                                    texture_name)) {
            matched_base = candidate;
            addon_selection_note_activemod(addon_id, folder, now);
            selected = addon_activemod_sidecar_for(folder, addon_id, now);
            if (!selected) selected = candidate;
            newest_entry = NULL;
            newest_age = 0;
            {
                int si;
                for (si = 0; si < ADDON_ACTIVE_SLOT_COUNT; si++) {
                    addon_active_slot_t *entry = &addon_active_slots[si];
                    DWORD age;
                    if (!entry->owner[0] || !entry->root_object ||
                        _stricmp(entry->addon_id, addon_id) != 0) {
                        continue;
                    }
                    age = now - entry->root_tick;
                    if (!newest_entry || age < newest_age) {
                        newest_entry = entry;
                        newest_age = age;
                    }
                }
            }
            if (newest_entry) {
                char root_name[320];
                _snprintf(root_name, sizeof(root_name), "%s%s",
                          newest_entry->owner, newest_entry->addon_id);
                addon_switch_active_slot_sidecar(
                    newest_entry, selected, root_name,
                    newest_entry->root_object, now,
                    selected == candidate ?
                        "ActiveMod-selected-main-inheritance" :
                        "ActiveMod-selected-variant");
            }
            log_line("ActiveMod selection observed texture=\"%s\" folder=\"%s\" addon_id=\"%s\" selected_sidecar=\"%s\" inherited_main=%d note=\"texture name was mapped through the add-on scene; sidecar filename remains the scene/add-on identifier\"",
                     texture_name, folder, addon_id, selected->path,
                     selected == candidate ? 1 : 0);
        }
    }
    if (!matched_base && defaults_cfg.debug) {
        log_line("ActiveMod texture ignored texture=\"%s\" folder=\"%s\" reason=\"no registered PhysX add-on scene references this texture\"",
                 texture_name, folder);
    }
}

static void physx_note_activemod_file_w(const WCHAR *file_path)
{
    char path[MAX_PATH * 4];
    int ok;
    if (!file_path) return;
    path[0] = 0;
    ok = WideCharToMultiByte(CP_ACP, 0, file_path, -1,
                             path, sizeof(path), NULL, NULL);
    if (!ok) return;
    path[sizeof(path) - 1] = 0;
    physx_note_activemod_file_a(path);
}

static int addon_sidecar_note_live_root_name(const char *root_name,
                                             void *root_object,
                                             DWORD now)
{
    physx_sidecar_t *active_sc = NULL;
    addon_active_slot_t *entry;
    char owner[16];
    char addon_id[256];
    size_t owner_len;
    int equipment_root_event;
    if (!root_name || !root_name[0] || !root_object ||
        _strnicmp(root_name, "Person", 6) != 0 ||
        strchr(root_name, ':') || strchr(root_name, '/') ||
        strchr(root_name, '\\')) {
        return 0;
    }
    if (!addon_extract_person_prefix(root_name, owner, sizeof(owner))) {
        return 0;
    }
    owner_len = strlen(owner);
    if (strlen(root_name) <= owner_len) return 0;
    lstrcpynA(addon_id, root_name + owner_len, sizeof(addon_id));
    equipment_root_event = addon_equipment_commit_live_root(
        owner, addon_id, root_object, now);
    if (!find_base_addon_sidecar_by_id(addon_id)) {
        return equipment_root_event;
    }
    entry = addon_active_slot_find(owner, addon_id, 1);
    if (!entry) return 0;
    active_sc = addon_selected_sidecar_for_root(addon_id, entry, now);
    if (!active_sc) return 0;
    if (entry->sidecar == active_sc && entry->root_object == root_object) {
        entry->root_tick = now;
        return equipment_root_event;
    }
    addon_switch_active_slot_sidecar(entry, active_sc, root_name,
                                     root_object, now,
                                     sidecar_path_is_activemod_a(active_sc->path) ?
                                         "live-root-ActiveMod" :
                                         "live-root-main");
    return 1;
}

static void addon_chain_note_live_root_event(physx_sidecar_t *sc,
                                             physx_chain_t *chain,
                                             DWORD now)
{
    char owner[16];
    if (!sc || !chain || !chain->addon_chain) return;
    if (sc->room_scene_sidecar) return;
    if (addon_tsnode_window_serial <= 0) return;
    if (chain->addon_root_window_serial == addon_tsnode_window_serial) return;
    if (!sidecar_named_node_matches_addon_root(sc, addon_tsnode_window_root)) return;
    owner[0] = 0;
    if (!addon_extract_person_prefix(addon_tsnode_window_root, owner,
                                     sizeof(owner)) ||
        !addon_owner_person_live_for_binding(owner)) {
        return;
    }
    if (chain->addon_owner_person[0] &&
        _stricmp(chain->addon_owner_person, owner) != 0) {
        return;
    }
    chain->addon_root_window_serial = addon_tsnode_window_serial;
    addon_chain_schedule_runtime_rebind(sc, chain, now,
                                        "live-add-on-root",
                                        900u);
}

static void update_targets(DWORD now)
{
    int i, c, t;
    if (last_update_tick && now - last_update_tick < 1000) return;
    last_update_tick = now;
    resolve_engine_symbols();
    if (!engine_FindObjC) return;
    for (i = 0; i < sidecar_count; i++) {
        physx_sidecar_t *sc = &sidecars[i];
        if (!sc->enabled) continue;
        if (!sc->root_logged) {
            char prefix[512];
            sidecar_scene_object_prefix(sc->path, prefix, sizeof(prefix));
            sc->root_logged = 1;
            if (prefix[0]) {
                sc->root_object = resolve_find_obj(prefix, &sc->root_raw_object);
                if (captured_script_engine) {
                    sc->script_root_object = resolve_script_engine_obj(prefix, &sc->script_root_raw_object);
                }
                log_line("sidecar root variants root=\"%s\" raw=%p obj=%p nil=%d script_raw=%p script_obj=%p script_nil=%d sidecar=\"%s\"",
                         prefix,
                         sc->root_raw_object, sc->root_object, is_nil_engine_object(sc->root_raw_object, sc->root_object),
                         sc->script_root_raw_object, sc->script_root_object,
                         is_nil_engine_object(sc->script_root_raw_object, sc->script_root_object),
                         sc->path);
            }
        }
        if (captured_script_engine && !sc->root_import_attempted) {
            void *root = NULL;
            sc->root_import_attempted = 1;
            if (sc->script_root_object && !is_nil_engine_object(sc->script_root_raw_object, sc->script_root_object)) {
                root = sc->script_root_object;
            } else if (sc->root_object && !is_nil_engine_object(sc->root_raw_object, sc->root_object)) {
                root = sc->root_object;
            }
            if (root) {
                import_script_object_tree_names(root);
                log_line("sidecar root import attempted root=%p sidecar=\"%s\"", root, sc->path);
            } else {
                log_line("sidecar root import skipped unresolved sidecar=\"%s\"", sc->path);
            }
        }
        for (c = 0; c < sc->chain_count; c++) {
            physx_chain_t *chain = &sc->chains[c];
            if (chain->addon_chain && chain->addon_owner_person[0]) {
                lstrcpynA(sc->addon_owner_person,
                          chain->addon_owner_person,
                          sizeof(sc->addon_owner_person));
                if (!sidecar_owner_has_live_addon_root(
                        sc, chain->addon_owner_person)) {
                    continue;
                }
            }
            if (chain->addon_chain && !sc->room_scene_sidecar) {
                addon_chain_note_live_root_event(sc, chain, now);
                if (addon_chain_settling(chain, now)) continue;
            }
            for (t = 0; t < chain->target_count; t++) {
                physx_target_t *target = &chain->targets[t];
                if (chain->addon_chain) {
                    char matched_live_name[256];
                    void *live_raw = NULL;
                    void *live_obj = NULL;
                    if (target->addon_skin_bound &&
                        target->object &&
                        !is_nil_engine_object(target->raw_object, target->object)) {
                        live_obj = target->object;
                    } else {
                        live_obj = resolve_live_addon_named_target(sc,
                                                                   chain->addon_owner_person,
                                                                   target->name,
                                                                   target->addon_simulated_target,
                                                                   &live_raw,
                                                                   matched_live_name,
                                                                   sizeof(matched_live_name));
                        if (live_obj && live_obj != target->object) {
                            if (chain->addon_scene_visible &&
                                target->object &&
                                _stricmp(target->name, chain->name) == 0) {
                                addon_chain_schedule_runtime_rebind(
                                    sc, chain, now,
                                    "live-root-output-rebind",
                                    1200u);
                            }
                            target->object = live_obj;
                            target->raw_object = live_raw;
                            target->addon_object_name_fallback = 0;
                            target->addon_skin_bound = 0;
                            target->addon_skin_bound_tick = 0;
                            target->s_object = NULL;
                            target->s_raw_object = NULL;
                            target->s_translation_base = NULL;
                            target->s_rotation_base = NULL;
                            target->addon_rotation_base = NULL;
                            reset_addon_tjoint_rotation_rest(target);
                            reset_addon_sjoint_orientation_rest(target);
                            target->s_translation_offset = -1;
                            target->s_rotation_offset = -1;
                            target->addon_rotation_offset = -1;
                            target->s_found_logged = 0;
                            target->s_missing_logged = 0;
                            target->s_translation_probe_logged = 0;
                            target->s_rotation_probe_logged = 0;
                            target->sim_initialized = 0;
                            target->addon_visual_pose_valid = 0;
                            target->found_logged = 0;
                            if (!target->found_logged) {
                                target->found_logged = 1;
                                log_line("target found via live addon root chain=\"%s\" type=\"%s\" target=\"%s\" runtime=\"%s\" raw=%p object=%p stiffness=%.3f damping=%.3f limit=%.1f sidecar=\"%s\" note=\"target was validated against the newest matching live add-on root instead of a generic Object.Name capture\"",
                                         chain->name, chain->type, target->name, matched_live_name,
                                         target->raw_object, target->object,
                                         chain->stiffness, chain->damping, chain->limit_angle, sc->path);
                            }
                        } else if (!target->object) {
                            target->object = resolve_addon_object_name_target(target->name,
                                                                              &live_raw,
                                                                              matched_live_name,
                                                                              sizeof(matched_live_name));
                            if (target->object) {
                                target->raw_object = live_raw;
                                target->addon_object_name_fallback = 1;
                                if (!target->found_logged) {
                                    target->found_logged = 1;
                                    log_line("target found via delayed addon Object.Name chain=\"%s\" type=\"%s\" target=\"%s\" runtime=\"%s\" raw=%p object=%p stiffness=%.3f damping=%.3f limit=%.1f sidecar=\"%s\" note=\"temporary generic add-on binding; will rebind when a matching live add-on root exposes this custom bone\"",
                                             chain->name, chain->type, target->name, matched_live_name,
                                             target->raw_object, target->object,
                                             chain->stiffness, chain->damping, chain->limit_angle, sc->path);
                                }
                            }
                        }
                    }
                }
                if (!target->object && chain->addon_chain &&
                    addon_target_retry_deferred(target, now)) {
                    continue;
                }
                if (!target->object && !chain->addon_chain) target->object = resolve_find_obj(target->name, &target->raw_object);
                if (target->object && is_nil_engine_object(target->raw_object, target->object)) {
                    if (!target->nil_logged) {
                        target->nil_logged = 1;
                        log_line("target unresolved-nil chain=\"%s\" type=\"%s\" target=\"%s\" raw=%p object=%p sidecar=\"%s\"",
                                 chain->name, chain->type, target->name, target->raw_object, target->object, sc->path);
                    }
                    target->object = NULL;
                } else if (target->object && !target->found_logged) {
                    target->found_logged = 1;
                    log_line("target found chain=\"%s\" type=\"%s\" target=\"%s\" raw=%p object=%p stiffness=%.3f damping=%.3f limit=%.1f sidecar=\"%s\"",
                             chain->name, chain->type, target->name, target->raw_object, target->object,
                             chain->stiffness, chain->damping, chain->limit_angle, sc->path);
                }
                if (!target->object && !chain->addon_chain) {
                    target->object = find_named_node(target->name);
                    if (target->object && !target->found_logged) {
                        target->found_logged = 1;
                        log_line("target found via tsnode chain=\"%s\" type=\"%s\" target=\"%s\" object=%p stiffness=%.3f damping=%.3f limit=%.1f sidecar=\"%s\"",
                                 chain->name, chain->type, target->name, target->object,
                                 chain->stiffness, chain->damping, chain->limit_angle, sc->path);
                    }
                }
                if (!target->object && !chain->addon_chain) {
                    char local_target_name[192];
                    _snprintf(local_target_name, sizeof(local_target_name), "local_%s", target->name);
                    target->object = resolve_find_obj(local_target_name, &target->raw_object);
                    if (target->object && is_nil_engine_object(target->raw_object, target->object)) {
                        target->object = NULL;
                    } else if (target->object && !target->found_logged) {
                        target->found_logged = 1;
                        log_line("target found via local name chain=\"%s\" type=\"%s\" target=\"%s\" runtime=\"%s\" raw=%p object=%p stiffness=%.3f damping=%.3f limit=%.1f sidecar=\"%s\"",
                                 chain->name, chain->type, target->name, local_target_name,
                                 target->raw_object, target->object,
                                 chain->stiffness, chain->damping, chain->limit_angle, sc->path);
                    }
                }
                if (!target->object && !chain->addon_chain && named_node_count > 0) {
                    char matched_name[384];
                    void *runtime_raw = NULL;
                    target->object = resolve_runtime_named_target(target->name, &runtime_raw, matched_name, sizeof(matched_name));
                    if (target->object) {
                        target->raw_object = runtime_raw;
                        if (!target->found_logged) {
                            target->found_logged = 1;
                            log_line("target found via runtime name chain=\"%s\" type=\"%s\" target=\"%s\" runtime=\"%s\" raw=%p object=%p stiffness=%.3f damping=%.3f limit=%.1f sidecar=\"%s\"",
                                     chain->name, chain->type, target->name, matched_name,
                                     target->raw_object, target->object,
                                     chain->stiffness, chain->damping, chain->limit_angle, sc->path);
                        }
                    }
                }
                if (!target->object && !chain->addon_chain) {
                    target->object = search_named_node_trees(target->name);
                    if (target->object && !target->found_logged) {
                        target->found_logged = 1;
                        log_line("target found via tree search chain=\"%s\" type=\"%s\" target=\"%s\" object=%p stiffness=%.3f damping=%.3f limit=%.1f sidecar=\"%s\"",
                                 chain->name, chain->type, target->name, target->object,
                                 chain->stiffness, chain->damping, chain->limit_angle, sc->path);
                    } else if (!target->tree_search_logged) {
                        target->tree_search_logged = 1;
                        target->tree_search_named_roots_logged = named_node_count;
                        log_line("target tree search missing chain=\"%s\" target=\"%s\" named_roots=%d sidecar=\"%s\"",
                                 chain->name, target->name, named_node_count, sc->path);
                    } else if (target->tree_search_named_roots_logged != named_node_count) {
                        target->tree_search_named_roots_logged = named_node_count;
                        log_line("target tree search retry missing chain=\"%s\" target=\"%s\" named_roots=%d sidecar=\"%s\"",
                                 chain->name, target->name, named_node_count, sc->path);
                    }
                }
                if (target->object && chain->addon_chain) {
                    addon_target_note_resolve_success(target);
                }
                if (!target->object && !target->missing_logged) {
                    target->missing_logged = 1;
                    log_line("target missing chain=\"%s\" target=\"%s\" sidecar=\"%s\"",
                             chain->name, target->name, sc->path);
                }
                if (!target->object && chain->addon_chain) {
                    addon_target_note_resolve_miss(target, chain->name, sc->path, now);
                }
                if (target->object &&
                    (!target->s_found_logged ||
                     (chain->addon_chain && !target->s_object))) {
                    char s_name[192];
                    char matched_s_name[384];
                    _snprintf(s_name, sizeof(s_name), "S%s", target->name);
                    if (chain->addon_chain) {
                        target->s_object = resolve_live_addon_named_target(sc,
                                                                           chain->addon_owner_person,
                                                                           s_name,
                                                                           target->addon_simulated_target,
                                                                           &target->s_raw_object,
                                                                           matched_s_name,
                                                                           sizeof(matched_s_name));
                        if (!target->s_object) {
                            target->s_object = resolve_addon_object_name_target(s_name, &target->s_raw_object,
                                                                                matched_s_name, sizeof(matched_s_name));
                        }
                    }
                    if (!target->s_object && !chain->addon_chain) {
                        target->s_object = resolve_runtime_exact_target(s_name, &target->s_raw_object,
                                                                        matched_s_name, sizeof(matched_s_name));
                    }
                    if (target->s_object) {
                        if (!target->s_found_logged) {
                            target->s_found_logged = 1;
                            target->s_missing_logged = 0;
                            log_line("target s-transform found chain=\"%s\" target=\"%s\" s_target=\"%s\" runtime=\"%s\" t_raw=%p t_object=%p s_raw=%p s_object=%p sidecar=\"%s\"",
                                     chain->name, target->name, s_name, matched_s_name,
                                     target->raw_object, target->object,
                                     target->s_raw_object, target->s_object, sc->path);
                        }
                        if (chain->object_transform_chain) {
                            /* type=object writes through the native
                               SSimpleTransform setter.  Never interpret an
                               STransform as an SJoint matrix or scan it for
                               joint-orientation fields. */
                            target->s_translation_probe_logged = 1;
                            target->s_rotation_probe_logged = 1;
                        } else if (!target->s_translation_probe_logged) {
                            int off;
                            float expected[3];
                            target->s_translation_probe_logged = 1;
                            off = probe_s_transform_translation_layout(target);
                            if (off < 0 && chain->addon_chain &&
                                target->addon_simulated_target) {
                                off = assume_addon_s_transform_layout(target, sc->path) ? target->s_translation_offset : -1;
                            }
                            if (chain->addon_chain &&
                                target->addon_simulated_target) {
                                assume_addon_t_joint_rotation_layout(target, sc->path);
                            }
                            if (off >= 0 && target->s_translation_base && known_initial_translation(target->name, expected)) {
                                float *v = (float*)((BYTE*)target->s_translation_base + off);
                                log_line("target s-translation layout chain=\"%s\" target=\"%s\" source=%s base=%p s_object=%p s_raw=%p offset=0x%03x expected=(%.5f,%.5f,%.5f) current=(%.5f,%.5f,%.5f) sidecar=\"%s\"",
                                         chain->name, target->name,
                                         target->s_translation_source ? target->s_translation_source : "unknown",
                                         target->s_translation_base, target->s_object, target->s_raw_object, off,
                                         expected[0], expected[1], expected[2],
                                         v[0], v[1], v[2], sc->path);
                            } else if (off >= 0 && target->s_translation_base) {
                                float *v = (float*)((BYTE*)target->s_translation_base + off);
                                log_line("target s-translation layout chain=\"%s\" target=\"%s\" source=%s base=%p s_object=%p s_raw=%p offset=0x%03x current=(%.5f,%.5f,%.5f) sidecar=\"%s\"",
                                         chain->name, target->name,
                                         target->s_translation_source ? target->s_translation_source : "unknown",
                                         target->s_translation_base, target->s_object, target->s_raw_object, off,
                                         v[0], v[1], v[2], sc->path);
                            } else {
                                log_line("target s-translation layout missing chain=\"%s\" target=\"%s\" s_object=%p sidecar=\"%s\"",
                                         chain->name, target->name, target->s_object, sc->path);
                            }
                            if (!target->s_rotation_probe_logged) {
                                target->s_rotation_probe_logged = 1;
                                if (!(chain->addon_chain && target->s_rotation_base)) {
                                    probe_s_transform_rotation_layout(target);
                                }
                            }
                        }
                    } else if (!target->s_missing_logged) {
                        target->s_missing_logged = 1;
                        log_line("target s-transform missing chain=\"%s\" target=\"%s\" s_target=\"%s\" t_raw=%p t_object=%p sidecar=\"%s\"",
                                 chain->name, target->name, s_name,
                                 target->raw_object, target->object, sc->path);
                    }
                }
                if (!target->variants_logged) {
                    char s_name[192];
                    char mesh_name[192];
                    char s_mesh_name[192];
                    char prefix[512];
                    char path_name[768];
                    char path_s_name[768];
                    void *s_obj;
                    void *mesh_obj;
                    void *s_mesh_obj;
                    void *path_raw = NULL;
                    void *path_obj = NULL;
                    void *path_s_raw = NULL;
                    void *path_s_obj = NULL;
                    target->variants_logged = 1;
                    _snprintf(s_name, sizeof(s_name), "S%s", target->name);
                    _snprintf(mesh_name, sizeof(mesh_name), "%s_mesh", target->name);
                    _snprintf(s_mesh_name, sizeof(s_mesh_name), "S%s_mesh", target->name);
                    s_obj = resolve_find_obj(s_name, NULL);
                    mesh_obj = resolve_find_obj(mesh_name, NULL);
                    s_mesh_obj = resolve_find_obj(s_mesh_name, NULL);
                    sidecar_scene_object_prefix(sc->path, prefix, sizeof(prefix));
                    if (prefix[0]) {
                        _snprintf(path_name, sizeof(path_name), "%s:%s", prefix, target->name);
                        _snprintf(path_s_name, sizeof(path_s_name), "%s:S%s", prefix, target->name);
                        path_obj = resolve_find_obj(path_name, &path_raw);
                        path_s_obj = resolve_find_obj(path_s_name, &path_s_raw);
                    } else {
                        path_name[0] = 0;
                        path_s_name[0] = 0;
                    }
                    log_line("target variants target=\"%s\" raw=%p base=%p nil=%d s_name=\"%s\" s=%p s_nil=%d mesh_name=\"%s\" mesh=%p mesh_nil=%d s_mesh_name=\"%s\" s_mesh=%p s_mesh_nil=%d path_name=\"%s\" path_raw=%p path_obj=%p path_nil=%d path_s_name=\"%s\" path_s_raw=%p path_s_obj=%p path_s_nil=%d sidecar=\"%s\"",
                             target->name, target->raw_object, target->object,
                             is_nil_engine_object(target->raw_object, target->object), s_name, s_obj,
                             is_nil_engine_object(NULL, s_obj),
                             mesh_name, mesh_obj, is_nil_engine_object(NULL, mesh_obj),
                             s_mesh_name, s_mesh_obj, is_nil_engine_object(NULL, s_mesh_obj),
                             path_name, path_raw, path_obj, is_nil_engine_object(path_raw, path_obj),
                             path_s_name, path_s_raw, path_s_obj, is_nil_engine_object(path_s_raw, path_s_obj),
                             sc->path);
                }
                if (captured_script_engine && !target->script_variants_logged) {
                    char s_name[192];
                    char prefix[512];
                    char path_name[768];
                    char path_s_name[768];
                    void *script_raw = NULL;
                    void *script_obj = NULL;
                    void *script_s_raw = NULL;
                    void *script_s_obj = NULL;
                    void *script_path_raw = NULL;
                    void *script_path_obj = NULL;
                    void *script_path_s_raw = NULL;
                    void *script_path_s_obj = NULL;
                    target->script_variants_logged = 1;
                    _snprintf(s_name, sizeof(s_name), "S%s", target->name);
                    script_obj = resolve_script_engine_obj(target->name, &script_raw);
                    script_s_obj = resolve_script_engine_obj(s_name, &script_s_raw);
                    sidecar_scene_object_prefix(sc->path, prefix, sizeof(prefix));
                    if (prefix[0]) {
                        _snprintf(path_name, sizeof(path_name), "%s:%s", prefix, target->name);
                        _snprintf(path_s_name, sizeof(path_s_name), "%s:S%s", prefix, target->name);
                        script_path_obj = resolve_script_engine_obj(path_name, &script_path_raw);
                        script_path_s_obj = resolve_script_engine_obj(path_s_name, &script_path_s_raw);
                    } else {
                        path_name[0] = 0;
                        path_s_name[0] = 0;
                    }
                    log_line("script target variants target=\"%s\" se=%p raw=%p obj=%p nil=%d s_name=\"%s\" s_raw=%p s_obj=%p s_nil=%d path_name=\"%s\" path_raw=%p path_obj=%p path_nil=%d path_s_name=\"%s\" path_s_raw=%p path_s_obj=%p path_s_nil=%d sidecar=\"%s\"",
                             target->name, captured_script_engine,
                             script_raw, script_obj, is_nil_engine_object(script_raw, script_obj),
                             s_name, script_s_raw, script_s_obj, is_nil_engine_object(script_s_raw, script_s_obj),
                             path_name, script_path_raw, script_path_obj, is_nil_engine_object(script_path_raw, script_path_obj),
                             path_s_name, script_path_s_raw, script_path_s_obj, is_nil_engine_object(script_path_s_raw, script_path_s_obj),
                             sc->path);
                }
            }
        }
    }
}

static void run_write_tests(DWORD now)
{
    int i, c, t;
    for (i = 0; i < sidecar_count; i++) {
        physx_sidecar_t *sc = &sidecars[i];
        physx_target_t *test_target = NULL;
        physx_chain_t *test_chain = NULL;
        float *v;
        if (!sc->enabled || !sc->write_test) continue;
        for (c = 0; c < sc->chain_count && !test_target; c++) {
            physx_chain_t *chain = &sc->chains[c];
            if (chain->addon_chain &&
                (!chain->addon_scene_visible ||
                 !sidecar_selected_for_owner(sc,
                                             chain->addon_owner_person))) {
                continue;
            }
            for (t = 0; t < chain->target_count; t++) {
                physx_target_t *target = &chain->targets[t];
                if (!target->s_translation_base) continue;
                if (sc->write_test_target[0] && _stricmp(sc->write_test_target, target->name) != 0) continue;
                test_target = target;
                test_chain = chain;
                break;
            }
        }
        if (!test_target || !test_chain ||
            sc->write_test_axis < 0 || sc->write_test_axis > 2) continue;
        v = (float*)((BYTE*)test_target->s_translation_base + test_target->s_translation_offset);
        if (!ptr_readable(v, sizeof(float) * 3)) continue;
        if (sc->write_test_state == 0) {
            sc->write_test_original[0] = v[0];
            sc->write_test_original[1] = v[1];
            sc->write_test_original[2] = v[2];
            v[sc->write_test_axis] = sc->write_test_original[sc->write_test_axis] + sc->write_test_amount;
            sc->write_test_start_tick = now;
            sc->write_test_state = 1;
            log_line("write-test applied target=\"%s\" source=%s base=%p offset=0x%03x axis=%d amount=%.5f before=(%.5f,%.5f,%.5f) after=(%.5f,%.5f,%.5f) sidecar=\"%s\"",
                     test_target->name,
                     test_target->s_translation_source ? test_target->s_translation_source : "unknown",
                     test_target->s_translation_base, test_target->s_translation_offset,
                     sc->write_test_axis, sc->write_test_amount,
                     sc->write_test_original[0], sc->write_test_original[1], sc->write_test_original[2],
                     v[0], v[1], v[2], sc->path);
        } else if (sc->write_test_state == 1 &&
                   now - sc->write_test_start_tick >= (DWORD)sc->write_test_duration_ms) {
            v[0] = sc->write_test_original[0];
            v[1] = sc->write_test_original[1];
            v[2] = sc->write_test_original[2];
            sc->write_test_state = 2;
            log_line("write-test restored target=\"%s\" source=%s base=%p offset=0x%03x current=(%.5f,%.5f,%.5f) sidecar=\"%s\"",
                     test_target->name,
                     test_target->s_translation_source ? test_target->s_translation_source : "unknown",
                     test_target->s_translation_base, test_target->s_translation_offset,
                     v[0], v[1], v[2], sc->path);
        }
    }
}

static float physx_clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static int physx_vec3_sane_limit(const float *v, float limit)
{
    if (!v) return 0;
    if (!sane_probe_float(v[0]) || !sane_probe_float(v[1]) || !sane_probe_float(v[2])) return 0;
    if (v[0] < -limit || v[0] > limit) return 0;
    if (v[1] < -limit || v[1] > limit) return 0;
    if (v[2] < -limit || v[2] > limit) return 0;
    return 1;
}

static void invalidate_addon_target_binding(physx_target_t *target)
{
    if (!target) return;
    target->object = NULL;
    target->raw_object = NULL;
    target->s_object = NULL;
    target->s_raw_object = NULL;
    target->s_translation_base = NULL;
    target->s_rotation_base = NULL;
    target->addon_rotation_base = NULL;
    reset_addon_tjoint_rotation_rest(target);
    reset_addon_sjoint_orientation_rest(target);
    target->addon_skin_bound = 0;
    target->addon_skin_bound_tick = 0;
    target->s_translation_offset = -1;
    target->s_rotation_offset = -1;
    target->addon_rotation_offset = -1;
    target->s_found_logged = 0;
    target->s_missing_logged = 0;
    target->s_translation_probe_logged = 0;
    target->s_rotation_probe_logged = 0;
    target->found_logged = 0;
    target->missing_logged = 0;
    target->nil_logged = 0;
    target->addon_resolve_miss_count = 0;
    target->addon_resolve_backoff_logged = 0;
    target->addon_resolve_retry_tick = 0;
    target->addon_validation_parent_ptr = NULL;
    target->addon_validation_target_ptr = NULL;
    target->addon_validation_generation = 0;
    target->addon_validation_tick = 0;
    target->addon_validation_ready = 0;
    target->sim_initialized = 0;
    target->sim_started_logged = 0;
    target->sim_world_anchor_initialized = 0;
    target->addon_gravity_diag_initialized = 0;
    target->addon_gravity_diag_samples = 0;
    target->addon_gravity_diag_max_delta = 0.0f;
    target->addon_gravity_diag_max_bend_len = 0.0f;
    target->addon_gravity_diag_start_offset[0] = 0.0f;
    target->addon_gravity_diag_start_offset[1] = 0.0f;
    target->addon_gravity_diag_start_offset[2] = 0.0f;
    target->addon_gravity_diag_last_delta[0] = 0.0f;
    target->addon_gravity_diag_last_delta[1] = 0.0f;
    target->addon_gravity_diag_last_delta[2] = 0.0f;
    target->addon_gravity_diag_last_bend[0] = 0.0f;
    target->addon_gravity_diag_last_bend[1] = 0.0f;
    target->addon_gravity_diag_last_bend[2] = 0.0f;
    target->addon_gravity_diag_last_rotation[0] = 0.0f;
    target->addon_gravity_diag_last_rotation[1] = 0.0f;
    target->addon_gravity_diag_last_rotation[2] = 0.0f;
    target->room_collision_track_valid = 0;
    target->room_collision_track_generation = 0;
    target->room_collision_track_tick = 0;
    target->room_collision_track_world[0] = 0.0f;
    target->room_collision_track_world[1] = 0.0f;
    target->room_collision_track_world[2] = 0.0f;
    target->room_collision_contact_valid = 0;
    target->room_collision_contact_tick = 0;
    target->room_collision_rest_frames = 0;
    target->room_collision_contact_direction[0] = 0.0f;
    target->room_collision_contact_direction[1] = 0.0f;
    target->room_collision_contact_direction[2] = 0.0f;
    target->room_collision_world_contact_valid = 0;
    target->room_collision_world_contact_tick = 0;
    target->room_collision_world_contact_direction[0] = 0.0f;
    target->room_collision_world_contact_direction[1] = 0.0f;
    target->room_collision_world_contact_direction[2] = 0.0f;
    target->room_collision_world_contact_mesh_index = -1;
    target->room_collision_terminal_world_contact_valid = 0;
    target->room_collision_terminal_world_contact_tick = 0;
    target->room_collision_terminal_world_contact_direction[0] = 0.0f;
    target->room_collision_terminal_world_contact_direction[1] = 0.0f;
    target->room_collision_terminal_world_contact_direction[2] = 0.0f;
    target->room_collision_terminal_world_contact_mesh_index = -1;
    target->room_collision_terminal_track_valid = 0;
    target->room_collision_terminal_track_generation = 0;
    target->room_collision_terminal_track_tick = 0;
    target->room_collision_terminal_track_world[0] = 0.0f;
    target->room_collision_terminal_track_world[1] = 0.0f;
    target->room_collision_terminal_track_world[2] = 0.0f;
    target->room_collision_terminal_axis_valid = 0;
    target->room_collision_terminal_axis_generation = 0;
    target->room_collision_terminal_axis_base = -1;
    target->room_collision_terminal_axis_index = -1;
    target->room_collision_terminal_axis_sign = 1.0f;
    target->room_collision_rest_pose_valid = 0;
    target->room_collision_rest_pose_sleeping = 0;
    target->room_collision_rest_pose_frames = 0;
    target->room_collision_rest_pose_generation = 0;
    target->room_collision_rest_pose_tick = 0;
    target->room_collision_rest_pose_normal[0] = 0.0f;
    target->room_collision_rest_pose_normal[1] = 0.0f;
    target->room_collision_rest_pose_normal[2] = 0.0f;
    target->room_collision_rest_pose_offset[0] = 0.0f;
    target->room_collision_rest_pose_offset[1] = 0.0f;
    target->room_collision_rest_pose_offset[2] = 0.0f;
    target->room_collision_terminal_response_log_tick = 0;
    target->addon_collision_correction_valid = 0;
    target->addon_collision_correction_tick = 0;
    target->addon_collision_correction_prev[0] = 0.0f;
    target->addon_collision_correction_prev[1] = 0.0f;
    target->addon_collision_correction_prev[2] = 0.0f;
    target->addon_collision_response_stable = 0;
    target->addon_collision_direct_contact_tick = 0;
    target->addon_collision_direct_correction[0] = 0.0f;
    target->addon_collision_direct_correction[1] = 0.0f;
    target->addon_collision_direct_correction[2] = 0.0f;
    target->addon_visual_pose_valid = 0;
    target->object_output_applied = 0;
    target->object_output_logged = 0;
    target->constraint_suppression_logged = 0;
    target->animation_suppression_seen = 0;
    target->animation_suppression_logged = 0;
    memcpy(target->object_proxy_translation,
           target->object_proxy_rest,
           sizeof(target->object_proxy_translation));
    memcpy(target->object_output_rotation,
           target->object_scene_rotation,
           sizeof(target->object_output_rotation));
    addon_target_reset_write_guard(target);
}

/* TBaseTransform::Update asks the class-member table for the number of
   constraints attached to every transform. Keep a compact pointer set of
   exact add-on targets currently owned by PhysX so that getter hook can
   answer in constant time without scanning all sidecars during traversal.
   The actual ConstraintArray is never edited. */
#define ADDON_CONSTRAINT_CACHE_CAPACITY 2048

typedef struct addon_constraint_suppression_entry_t {
    void *key;
    physx_sidecar_t *sidecar;
    physx_chain_t *chain;
    physx_target_t *target;
    unsigned int generation;
} addon_constraint_suppression_entry_t;

typedef struct addon_constraint_suppression_cache_t {
    addon_constraint_suppression_entry_t
        entries[ADDON_CONSTRAINT_CACHE_CAPACITY];
    unsigned int generation;
} addon_constraint_suppression_cache_t;

static addon_constraint_suppression_cache_t
    addon_constraint_suppression_caches[2];
static volatile LONG addon_constraint_suppression_cache_index;
static int addon_constraint_suppression_cache_overflow_logged;

static unsigned int addon_constraint_suppression_hash(void *key)
{
    ULONG_PTR value = (ULONG_PTR)key;
    value >>= 4;
    value ^= value >> 11;
    return (unsigned int)value &
           (ADDON_CONSTRAINT_CACHE_CAPACITY - 1u);
}

static int addon_constraint_target_owned(
    const physx_sidecar_t *sc,
    const physx_chain_t *chain,
    const physx_target_t *target)
{
    if (!sc || !chain || !target ||
        !sc->loaded || !sc->enabled || sc->write_test ||
        !chain->addon_chain || !chain->simulate ||
        !chain->addon_scene_visible ||
        !target->addon_simulated_target ||
        !target->sim_initialized ||
        !target->addon_write_guard_ready ||
        !target->object ||
        is_nil_engine_object(target->raw_object, target->object)) {
        return 0;
    }
    return 1;
}

static int addon_constraint_suppression_cache_insert(
    addon_constraint_suppression_cache_t *cache,
    void *key,
    physx_sidecar_t *sc,
    physx_chain_t *chain,
    physx_target_t *target)
{
    unsigned int slot;
    unsigned int probe;
    if (!cache || !key) return 1;
    slot = addon_constraint_suppression_hash(key);
    for (probe = 0; probe < ADDON_CONSTRAINT_CACHE_CAPACITY; probe++) {
        addon_constraint_suppression_entry_t *entry =
            &cache->entries[(slot + probe) &
                            (ADDON_CONSTRAINT_CACHE_CAPACITY - 1u)];
        if (entry->generation != cache->generation || entry->key == key) {
            entry->key = key;
            entry->sidecar = sc;
            entry->chain = chain;
            entry->target = target;
            entry->generation = cache->generation;
            return 1;
        }
    }
    return 0;
}

static void rebuild_addon_constraint_suppression_cache(void)
{
    LONG active = InterlockedCompareExchange(
        &addon_constraint_suppression_cache_index, 0, 0) & 1;
    LONG next = active ^ 1;
    addon_constraint_suppression_cache_t *cache =
        &addon_constraint_suppression_caches[next];
    int i, c, t;
    int overflow = 0;
    /* The cache is rebuilt every rendered frame.  Generations make stale
       hash slots logically empty without clearing the entire 40 KB table. */
    cache->generation++;
    if (!cache->generation) {
        memset(cache->entries, 0, sizeof(cache->entries));
        cache->generation = 1;
    }
    for (i = 0; i < sidecar_count; i++) {
        physx_sidecar_t *sc = &sidecars[i];
        if (!sc->loaded || !sc->enabled || sc->write_test) continue;
        for (c = 0; c < sc->chain_count; c++) {
            physx_chain_t *chain = &sc->chains[c];
            if (!chain->addon_chain || !chain->simulate ||
                !chain->addon_scene_visible) {
                continue;
            }
            for (t = 0; t < chain->target_count; t++) {
                physx_target_t *target = &chain->targets[t];
                if (!addon_constraint_target_owned(sc, chain, target)) {
                    continue;
                }
                if (!addon_constraint_suppression_cache_insert(
                        cache, target->object, sc, chain, target) ||
                    (target->raw_object != target->object &&
                     !addon_constraint_suppression_cache_insert(
                         cache, target->raw_object, sc, chain, target))) {
                    overflow = 1;
                }
            }
        }
    }
    InterlockedExchange(&addon_constraint_suppression_cache_index, next);
    if (overflow && !addon_constraint_suppression_cache_overflow_logged) {
        addon_constraint_suppression_cache_overflow_logged = 1;
        log_line("addon constraint suppression cache full capacity=%d note=\"some unusually numerous active targets will retain their native constraints\"",
                 ADDON_CONSTRAINT_CACHE_CAPACITY);
    }
}

static int addon_should_suppress_constraint_array(
    void *object,
    physx_sidecar_t **sidecar_out,
    physx_chain_t **chain_out,
    physx_target_t **target_out)
{
    LONG active;
    addon_constraint_suppression_cache_t *cache;
    unsigned int slot;
    unsigned int probe;
    if (sidecar_out) *sidecar_out = NULL;
    if (chain_out) *chain_out = NULL;
    if (target_out) *target_out = NULL;
    if (!object) return 0;
    active = InterlockedCompareExchange(
        &addon_constraint_suppression_cache_index, 0, 0) & 1;
    cache = &addon_constraint_suppression_caches[active];
    if (!cache->generation) return 0;
    slot = addon_constraint_suppression_hash(object);
    for (probe = 0; probe < ADDON_CONSTRAINT_CACHE_CAPACITY; probe++) {
        addon_constraint_suppression_entry_t *entry =
            &cache->entries[(slot + probe) &
                            (ADDON_CONSTRAINT_CACHE_CAPACITY - 1u)];
        if (entry->generation != cache->generation) return 0;
        if (entry->key != object) continue;
        if (!addon_constraint_target_owned(entry->sidecar,
                                           entry->chain,
                                           entry->target)) {
            return 0;
        }
        if (entry->target->object != object &&
            entry->target->raw_object != object) {
            return 0;
        }
        if (sidecar_out) *sidecar_out = entry->sidecar;
        if (chain_out) *chain_out = entry->chain;
        if (target_out) *target_out = entry->target;
        return 1;
    }
    return 0;
}

/* AnimationScheduler writes SSimpleTransform.Rotation/SJoint.RotationAxis
   directly to animated room bones.  Room-native PhysX chains must own those
   exact targets, but shared schedulers cannot be disabled wholesale because
   one scheduler may animate several trees and unrelated room objects.  Keep
   a separate exact-pointer cache so the setter hooks can neutralize only the
   bones currently owned by an active sidecar. */
#define ADDON_ANIMATION_CACHE_CAPACITY 2048

typedef struct addon_animation_suppression_entry_t {
    void *key;
    physx_sidecar_t *sidecar;
    physx_chain_t *chain;
    physx_target_t *target;
    unsigned int generation;
} addon_animation_suppression_entry_t;

typedef struct addon_animation_suppression_cache_t {
    addon_animation_suppression_entry_t
        entries[ADDON_ANIMATION_CACHE_CAPACITY];
    unsigned int generation;
} addon_animation_suppression_cache_t;

static addon_animation_suppression_cache_t
    addon_animation_suppression_caches[2];
static volatile LONG addon_animation_suppression_cache_index;
static int addon_animation_suppression_cache_overflow_logged;

static unsigned int addon_animation_suppression_hash(void *key)
{
    ULONG_PTR value = (ULONG_PTR)key;
    value >>= 4;
    value ^= value >> 11;
    return (unsigned int)value &
           (ADDON_ANIMATION_CACHE_CAPACITY - 1u);
}

static int addon_animation_target_owned(
    const physx_sidecar_t *sc,
    const physx_chain_t *chain,
    const physx_target_t *target)
{
    if (!sc || !chain || !target ||
        !sc->loaded || !sc->enabled || sc->write_test ||
        !sc->room_scene_sidecar ||
        !sc->addon_scene_active ||
        !chain->addon_chain || !chain->simulate ||
        !chain->addon_scene_visible ||
        !target->addon_simulated_target ||
        !target->object ||
        is_nil_engine_object(target->raw_object, target->object)) {
        return 0;
    }
    return 1;
}

static int addon_animation_suppression_cache_insert(
    addon_animation_suppression_cache_t *cache,
    void *key,
    physx_sidecar_t *sc,
    physx_chain_t *chain,
    physx_target_t *target)
{
    unsigned int slot;
    unsigned int probe;
    if (!cache || !key) return 1;
    slot = addon_animation_suppression_hash(key);
    for (probe = 0; probe < ADDON_ANIMATION_CACHE_CAPACITY; probe++) {
        addon_animation_suppression_entry_t *entry =
            &cache->entries[(slot + probe) &
                            (ADDON_ANIMATION_CACHE_CAPACITY - 1u)];
        if (entry->generation != cache->generation || entry->key == key) {
            entry->key = key;
            entry->sidecar = sc;
            entry->chain = chain;
            entry->target = target;
            entry->generation = cache->generation;
            return 1;
        }
    }
    return 0;
}

static int addon_animation_target_key_matches(
    const physx_target_t *target, void *key)
{
    if (!target || !key) return 0;
    return key == target->object ||
           key == target->raw_object ||
           key == target->s_object ||
           key == target->s_raw_object ||
           key == target->s_rotation_base ||
           key == target->addon_rotation_base ||
           (target->object && key == (BYTE*)target->object + 0x08) ||
           (target->raw_object && key == (BYTE*)target->raw_object + 0x08) ||
           (target->s_object && key == (BYTE*)target->s_object + 0x08) ||
           (target->s_raw_object && key == (BYTE*)target->s_raw_object + 0x08);
}

static void rebuild_addon_animation_suppression_cache(void)
{
    LONG active = InterlockedCompareExchange(
        &addon_animation_suppression_cache_index, 0, 0) & 1;
    LONG next = active ^ 1;
    addon_animation_suppression_cache_t *cache =
        &addon_animation_suppression_caches[next];
    int i, c, t;
    int overflow = 0;
    cache->generation++;
    if (!cache->generation) {
        memset(cache->entries, 0, sizeof(cache->entries));
        cache->generation = 1;
    }
    for (i = 0; i < sidecar_count; i++) {
        physx_sidecar_t *sc = &sidecars[i];
        for (c = 0; c < sc->chain_count; c++) {
            physx_chain_t *chain = &sc->chains[c];
            for (t = 0; t < chain->target_count; t++) {
                physx_target_t *target = &chain->targets[t];
                void *keys[10];
                int k;
                if (!addon_animation_target_owned(sc, chain, target)) {
                    target->animation_suppression_seen = 0;
                    target->animation_suppression_logged = 0;
                    continue;
                }
                keys[0] = target->object;
                keys[1] = target->raw_object;
                keys[2] = target->s_object;
                keys[3] = target->s_raw_object;
                keys[4] = target->s_rotation_base;
                keys[5] = target->addon_rotation_base;
                keys[6] = target->object ? (BYTE*)target->object + 0x08 : NULL;
                keys[7] = target->raw_object ? (BYTE*)target->raw_object + 0x08 : NULL;
                keys[8] = target->s_object ? (BYTE*)target->s_object + 0x08 : NULL;
                keys[9] = target->s_raw_object ? (BYTE*)target->s_raw_object + 0x08 : NULL;
                for (k = 0; k < 10; k++) {
                    if (!addon_animation_suppression_cache_insert(
                            cache, keys[k], sc, chain, target)) {
                        overflow = 1;
                    }
                }
            }
        }
    }
    InterlockedExchange(&addon_animation_suppression_cache_index, next);
    if (overflow && !addon_animation_suppression_cache_overflow_logged) {
        addon_animation_suppression_cache_overflow_logged = 1;
        log_line("addon animation suppression cache full capacity=%d note=\"some unusually numerous active targets may retain authored animation\"",
                 ADDON_ANIMATION_CACHE_CAPACITY);
    }
}

static int addon_should_suppress_animation_write(
    void *object,
    DWORD member_id,
    physx_sidecar_t **sidecar_out,
    physx_chain_t **chain_out,
    physx_target_t **target_out)
{
    LONG active;
    addon_animation_suppression_cache_t *cache;
    unsigned int slot;
    unsigned int probe;
    if (sidecar_out) *sidecar_out = NULL;
    if (chain_out) *chain_out = NULL;
    if (target_out) *target_out = NULL;
    if (!object ||
        (member_id != SCRIPT_PROPERTY_SSIMPLE_ROTATION &&
         member_id != SCRIPT_PROPERTY_SJOINT_ROTATION_AXIS)) {
        return 0;
    }
    if (InterlockedCompareExchange(
            &addon_object_bone_limit_write_active, 0, 0) &&
        addon_object_bone_limit_write_thread == GetCurrentThreadId()) {
        return 0;
    }
    active = InterlockedCompareExchange(
        &addon_animation_suppression_cache_index, 0, 0) & 1;
    cache = &addon_animation_suppression_caches[active];
    if (!cache->generation) return 0;
    slot = addon_animation_suppression_hash(object);
    for (probe = 0; probe < ADDON_ANIMATION_CACHE_CAPACITY; probe++) {
        addon_animation_suppression_entry_t *entry =
            &cache->entries[(slot + probe) &
                            (ADDON_ANIMATION_CACHE_CAPACITY - 1u)];
        if (entry->generation != cache->generation) return 0;
        if (entry->key != object) continue;
        if (!addon_animation_target_owned(entry->sidecar,
                                          entry->chain,
                                          entry->target) ||
            !addon_animation_target_key_matches(entry->target, object)) {
            return 0;
        }
        if (sidecar_out) *sidecar_out = entry->sidecar;
        if (chain_out) *chain_out = entry->chain;
        if (target_out) *target_out = entry->target;
        return 1;
    }
    return 0;
}

static void addon_note_animation_write_suppressed(
    physx_sidecar_t *sc,
    physx_chain_t *chain,
    physx_target_t *target,
    DWORD member_id,
    void *object)
{
    if (!target) return;
    if (!target->animation_suppression_seen) {
        /* The first authored write proves that the native animation is live.
           Re-capture the neutralized pose on the next simulation pass instead
           of retaining a rest rotation sampled while animation still owned it. */
        target->animation_suppression_seen = 1;
        /* A wearable add-on can first bind while its authored animation is
           already posing the bone, so it must re-capture neutral rest after
           suppression. Room bones have already initialized from their static
           scene pose and are written through SSimpleTransform.Rotation. A
           late room animation tick must not reset that running spring or
           recapture the current PhysX rotation as a new rest pose. */
        if (!sc || !sc->room_scene_sidecar) {
            target->sim_initialized = 0;
            target->sim_started_logged = 0;
            target->sim_world_anchor_initialized = 0;
            target->sim_output_velocity_initialized = 0;
            target->addon_visual_pose_valid = 0;
            memset(target->sim_offset, 0, sizeof(target->sim_offset));
            memset(target->sim_velocity, 0, sizeof(target->sim_velocity));
            memset(target->sim_output_offset_prev, 0,
                   sizeof(target->sim_output_offset_prev));
            memset(target->sim_output_velocity, 0,
                   sizeof(target->sim_output_velocity));
        }
    }
    if (!target->animation_suppression_logged) {
        target->animation_suppression_logged = 1;
        log_line("addon native animation suppressed owner=\"%s\" chain=\"%s\" target=\"%s\" member=0x%08lx object=%p sidecar=\"%s\" note=\"only this active PhysX bone is neutralized; animation is restored automatically when ownership ends\"",
                 chain && chain->addon_owner_person[0]
                     ? chain->addon_owner_person
                     : (sc && sc->room_scene_sidecar ? "room" : "unknown"),
                 chain ? chain->name : "",
                 target->name,
                 (unsigned long)member_id,
                 object,
                 sc ? sc->path : "");
    }
}

static float physx_sqrtf(float v)
{
    if (v <= 0.0f) return 0.0f;
    /* Eight Newton steps starting at 1 left tiny positive inputs near
       1/256 instead of their true root. At resting contact this made unit
       normals much shorter than one and prevented velocity projection from
       cancelling inward motion. Use the runtime's correctly scaled sqrt. */
    return sqrtf(v);
}

static float physx_vec3_len(const float v[3])
{
    return physx_sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

static int addon_output_scene_visible(void)
{
    int person;
    int known = 0;
    for (person = 0; person < 4; person++) {
        int visible = poseedit_scene_person_visible(person);
        if (visible > 0) return 1;
        if (visible >= 0) known = 1;
    }
    return known ? 0 : -1;
}

typedef struct addon_owner_placement_state_t {
    void *root_raw;
    float previous_root[3];
    DWORD stable_tick;
    DWORD last_sample_tick;
    DWORD log_tick;
    int sampled;
    int ready;
} addon_owner_placement_state_t;

static addon_owner_placement_state_t addon_owner_placement_states[4];

static int addon_owner_placement_ready(const char *owner,
                                       void *root_raw,
                                       const float root[3],
                                       DWORD now)
{
    addon_owner_placement_state_t *state;
    float delta[3];
    float move_len;
    float epsilon = physics_environment_cfg.gravity_probe_motion_epsilon;
    int settle_ms = physics_environment_cfg.gravity_probe_settle_ms;
    int owner_index = addon_person_prefix_to_index(owner);
    if (!owner || !root_raw || !root ||
        owner_index < 0 || owner_index >= 4) {
        return 0;
    }
    if (epsilon < 0.0001f) epsilon = 0.0001f;
    if (settle_ms < 0) settle_ms = 0;
    state = &addon_owner_placement_states[owner_index];
    if (state->root_raw != root_raw) {
        memset(state, 0, sizeof(*state));
        state->root_raw = root_raw;
    }
    if (state->ready) return 1;
    if (!state->sampled) {
        state->sampled = 1;
        state->stable_tick = now;
        state->last_sample_tick = now;
        state->previous_root[0] = root[0];
        state->previous_root[1] = root[1];
        state->previous_root[2] = root[2];
        if (defaults_cfg.debug) {
            state->log_tick = now;
            log_line("addon owner placement sampled owner=\"%s\" root_raw=%p root=(%.5f,%.5f,%.5f) settle_ms=%d epsilon=%.6f note=\"add-on-owned readiness is independent of penis and testicle physics\"",
                     owner, root_raw,
                     root[0], root[1], root[2],
                     settle_ms, epsilon);
        }
        return 0;
    }
    /* Multiple chains can query the same owner in one PhysX tick. Only one
       temporal sample per tick may advance the shared readiness state. */
    if (state->last_sample_tick == now) return 0;
    state->last_sample_tick = now;
    delta[0] = root[0] - state->previous_root[0];
    delta[1] = root[1] - state->previous_root[1];
    delta[2] = root[2] - state->previous_root[2];
    state->previous_root[0] = root[0];
    state->previous_root[1] = root[1];
    state->previous_root[2] = root[2];
    move_len = physx_vec3_len(delta);
    if (move_len > epsilon) {
        state->stable_tick = now;
        if (defaults_cfg.debug &&
            (!state->log_tick || now - state->log_tick >= 2000u)) {
            state->log_tick = now;
            log_line("addon owner placement waiting owner=\"%s\" root_raw=%p root=(%.5f,%.5f,%.5f) move_len=%.6f settle_ms=%d epsilon=%.6f reason=\"owner root is still changing\" note=\"no add-on rest pose or output write yet\"",
                     owner, root_raw,
                     root[0], root[1], root[2],
                     move_len, settle_ms, epsilon);
        }
        return 0;
    }
    if ((int)(now - state->stable_tick) < settle_ms) return 0;
    state->ready = 1;
    state->log_tick = now;
    log_line("addon owner placement ready owner=\"%s\" root_raw=%p root=(%.5f,%.5f,%.5f) stable_ms=%lu settle_ms=%d note=\"shared add-on readiness promoted without depending on penis or testicle physics\"",
             owner, root_raw,
             root[0], root[1], root[2],
             (unsigned long)(now - state->stable_tick),
             settle_ms);
    return 1;
}

static int addon_chain_owner_body_ready(physx_sidecar_t *sc,
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

static int addon_chain_scene_visible(physx_sidecar_t *sc,
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

static void run_addon_visual_swing_test(physx_sidecar_t *sc,
                                        physx_chain_t *chain,
                                        DWORD now)
{
    physx_target_t *target;
    physx_target_t *requested;
    float *rv;
    DWORD elapsed;
    DWORD ready_elapsed;
    const DWORD settle_ms = 2000u;
    const DWORD active_ms = 9000u;
    const DWORD room_ready_delay_ms = 6000u;
    const float swing_deg = 70.0f;
    int scene_visible;
    int ti;
    int retargeted = 0;
    int loose_fallback = 0;
    if (!sc || !chain || chain->target_count <= 1) return;
    requested = &chain->targets[1];
    target = requested;
    if (!contains_i(requested->name, "physx_tail01")) return;
    chain->skinned_matrix_enabled = 1;
    chain->skinned_matrix_translation_enabled = 0;
    requested->addon_visual_pose_valid = 0;
    if (!target->addon_skin_bound) {
        for (ti = 1; ti < chain->target_count; ti++) {
            physx_target_t *candidate = &chain->targets[ti];
            if (!candidate->addon_simulated_target) continue;
            if (!candidate->addon_skin_bound) continue;
            if (!candidate->s_rotation_base || candidate->s_rotation_offset < 0) continue;
            target = candidate;
            retargeted = (target != requested);
            break;
        }
    }
    if (!target->addon_skin_bound &&
        requested->object &&
        requested->s_rotation_base &&
        requested->s_rotation_offset >= 0) {
        target = requested;
        retargeted = 0;
        loose_fallback = 1;
    }
    target->addon_visual_pose_valid = 0;
    if (!target->addon_skin_bound && !loose_fallback) {
        if (!chain->addon_output_started_logged ||
            now - chain->addon_output_status_tick >= 2000u) {
            chain->addon_output_started_logged = 1;
            chain->addon_output_status_tick = now;
            log_line("addon output-test waiting chain=\"%s\" requested=\"%s\" target=\"%s\" loose_fallback=%d reason=\"no writable skin-bound PhysX target or live SJoint fallback yet\" requested_object=%p requested_skin_bound=%d target_object=%p target_skin_bound=%d sidecar=\"%s\" note=\"test has not started because there is no writable addon output channel\"",
                     chain->name, requested->name, target->name,
                     loose_fallback,
                     requested->object,
                     requested->addon_skin_bound,
                     target->object,
                     target->addon_skin_bound,
                     sc->path);
        }
        return;
    }
    if (!target->s_rotation_base || target->s_rotation_offset < 0) {
        if (!chain->addon_output_started_logged ||
            now - chain->addon_output_status_tick >= 2000u) {
            chain->addon_output_started_logged = 1;
            chain->addon_output_status_tick = now;
            log_line("addon visual-swing-test waiting chain=\"%s\" requested=\"%s\" target=\"%s\" retargeted=%d loose_fallback=%d reason=\"missing SJoint rotation layout\" s_object=%p s_base=%p s_offset=0x%03x sidecar=\"%s\"",
                     chain->name, requested->name, target->name,
                     retargeted,
                     loose_fallback,
                     target->s_object,
                     target->s_rotation_base, target->s_rotation_offset,
                     sc->path);
        }
        return;
    }
    scene_visible = addon_chain_scene_visible(sc, chain, now,
                                              NULL, NULL, NULL);
    if (!chain->addon_output_start_tick && scene_visible <= 0) {
        chain->addon_output_room_ready_tick = 0;
        chain->addon_output_room_ready_logged = 0;
        if (!chain->addon_output_started_logged ||
            now - chain->addon_output_status_tick >= 1000u) {
            chain->addon_output_started_logged = 1;
            chain->addon_output_status_tick = now;
            log_line("addon output-test waiting chain=\"%s\" requested=\"%s\" target=\"%s\" retargeted=%d loose_fallback=%d reason=\"room/person not visible yet\" scene_visible=%d output=\"forced visual swing TJoint-preferred Euler traversal overlay\" note=\"test has NOT started; loading time will not count\" sidecar=\"%s\"",
                     chain->name, requested->name, target->name,
                     retargeted,
                     loose_fallback,
                     scene_visible, sc->path);
        }
        return;
    }
    if (!chain->addon_output_start_tick && !chain->addon_output_room_ready_tick) {
        chain->addon_output_room_ready_tick = now;
        chain->addon_output_status_tick = now;
        chain->addon_output_room_ready_logged = 1;
        log_line("============================================================");
        log_line("============ ADDON OUTPUT TEST ARMED AFTER LOAD ============");
        log_line("addon output-test armed chain=\"%s\" requested=\"%s\" target=\"%s\" retargeted=%d loose_fallback=%d room_ready_delay_ms=%lu scene_visible=%d output=\"forced visual swing TJoint-preferred Euler traversal overlay\" note=\"test has NOT started; wait for ADDON OUTPUT TEST STARTED\" sidecar=\"%s\"",
                 chain->name, requested->name, target->name,
                 retargeted,
                 loose_fallback,
                 (unsigned long)room_ready_delay_ms,
                 scene_visible,
                 sc->path);
        log_line("============================================================");
        return;
    }
    if (!chain->addon_output_start_tick) {
        ready_elapsed = now - chain->addon_output_room_ready_tick;
        if (ready_elapsed < room_ready_delay_ms) {
            if (now - chain->addon_output_status_tick >= 1000u) {
                DWORD remaining = room_ready_delay_ms - ready_elapsed;
                chain->addon_output_status_tick = now;
                log_line("addon output-test countdown chain=\"%s\" requested=\"%s\" target=\"%s\" retargeted=%d loose_fallback=%d starts_in_ms=%lu room_ready_elapsed_ms=%lu note=\"test has NOT started yet\" sidecar=\"%s\"",
                         chain->name, requested->name, target->name,
                         retargeted,
                         loose_fallback,
                         (unsigned long)remaining,
                         (unsigned long)ready_elapsed,
                         sc->path);
            }
            return;
        }
    }
    rv = (float*)((BYTE*)target->s_rotation_base + target->s_rotation_offset);
    if (!ptr_readable(rv, sizeof(float) * 3)) {
        if (!chain->addon_output_started_logged ||
            now - chain->addon_output_status_tick >= 2000u) {
            chain->addon_output_started_logged = 1;
            chain->addon_output_status_tick = now;
            log_line("addon visual-swing-test waiting chain=\"%s\" requested=\"%s\" target=\"%s\" retargeted=%d loose_fallback=%d reason=\"SJoint rotation pointer unreadable\" s_base=%p s_offset=0x%03x sidecar=\"%s\"",
                     chain->name, requested->name, target->name,
                     retargeted,
                     loose_fallback,
                     target->s_rotation_base, target->s_rotation_offset,
                     sc->path);
        }
        return;
    }
    if (!chain->addon_output_start_tick) {
        chain->addon_output_start_tick = now;
        chain->addon_output_status_tick = now;
        chain->addon_output_started_logged = 1;
        target->sim_initialized = 1;
        target->sim_rotation_rest[0] = rv[0];
        target->sim_rotation_rest[1] = rv[1];
        target->sim_rotation_rest[2] = rv[2];
        assume_addon_t_joint_rotation_layout(target, sc->path);
        physx_addon_publish_visual_pose(target,
                                        target->sim_rotation_rest,
                                        0);
        log_line("============================================================");
        log_line("================ ADDON OUTPUT TEST STARTED =================");
        log_line("addon output-test started chain=\"%s\" requested=\"%s\" target=\"%s\" retargeted=%d loose_fallback=%d duration_ms=%lu settle_ms=%lu output=\"forced visual swing TJoint-preferred Euler traversal overlay\" amplitude_deg=%.1f sjoint_available=%d tjoint_euler=%d matrix_rows=0 euler_overlay=1 tjoint_base=%p tjoint_offset=0x%03x sidecar=\"%s\" note=\"watch the hair; wait for addon output-test done before closing the game\"",
                 chain->name, requested->name, target->name,
                 retargeted,
                 loose_fallback,
                 (unsigned long)(settle_ms + active_ms),
                 (unsigned long)settle_ms,
                 swing_deg,
                 (target->s_rotation_base && target->s_rotation_offset >= 0) ? 1 : 0,
                 (target->addon_rotation_base && target->addon_rotation_offset >= 0) ? 1 : 0,
                 target->addon_rotation_base,
                 target->addon_rotation_offset,
                 sc->path);
        log_line("============================================================");
        return;
    }
    if (!target->sim_initialized) {
        target->sim_initialized = 1;
        target->sim_rotation_rest[0] = rv[0];
        target->sim_rotation_rest[1] = rv[1];
        target->sim_rotation_rest[2] = rv[2];
    }
    elapsed = now - chain->addon_output_start_tick;
    if (elapsed < settle_ms) return;
    if (elapsed >= settle_ms + active_ms) {
        physx_addon_publish_visual_pose(target,
                                        target->sim_rotation_rest,
                                        0);
        if (!chain->addon_output_done_logged) {
            chain->addon_output_done_logged = 1;
            log_line("============================================================");
            log_line("================= ADDON OUTPUT TEST DONE!!!!! ==============");
            log_line("DONE!!!!! ADDON OUTPUT TEST DONE!!!!! chain=\"%s\" requested=\"%s\" target=\"%s\" retargeted=%d loose_fallback=%d elapsed_ms=%lu output=\"forced visual swing TJoint-preferred Euler traversal overlay\"",
                     chain->name, requested->name, target->name,
                     retargeted,
                     loose_fallback,
                     (unsigned long)elapsed);
            log_line("addon output-test done chain=\"%s\" requested=\"%s\" target=\"%s\" retargeted=%d loose_fallback=%d elapsed_ms=%lu output=\"forced visual swing TJoint-preferred Euler traversal overlay\" restored_rotation=(%.5f,%.5f,%.5f) sjoint_available=%d tjoint_euler=%d matrix_rows=0 euler_overlay=1 tjoint_base=%p tjoint_offset=0x%03x sidecar=\"%s\" note=\"safe to close the game now and write done\"",
                     chain->name, requested->name, target->name,
                     retargeted,
                     loose_fallback,
                     (unsigned long)elapsed,
                     target->sim_rotation_rest[0],
                     target->sim_rotation_rest[1],
                     target->sim_rotation_rest[2],
                     (target->s_rotation_base && target->s_rotation_offset >= 0) ? 1 : 0,
                     (target->addon_rotation_base && target->addon_rotation_offset >= 0) ? 1 : 0,
                     target->addon_rotation_base,
                     target->addon_rotation_offset,
                     sc->path);
            log_line("============================================================");
        }
        return;
    }
    {
        DWORD active_elapsed = elapsed - settle_ms;
        float sign = ((active_elapsed / 750u) & 1u) ? -1.0f : 1.0f;
        float swing = sign * swing_deg;
        float visual_r[3];
        visual_r[0] = target->sim_rotation_rest[0] + swing;
        visual_r[1] = target->sim_rotation_rest[1];
        visual_r[2] = target->sim_rotation_rest[2] + (swing * 0.75f);
        physx_addon_publish_visual_pose(target, visual_r, 0);
        if (now - chain->addon_output_status_tick >= 1000u) {
            chain->addon_output_status_tick = now;
            log_line("addon visual-swing-test write chain=\"%s\" requested=\"%s\" target=\"%s\" retargeted=%d loose_fallback=%d elapsed_ms=%lu swing_deg=%.1f rotation=(%.5f,%.5f,%.5f) rest_rotation=(%.5f,%.5f,%.5f) sjoint_available=%d wrote_tjoint_euler=%d matrix_rows=0 euler_overlay=1 tjoint_base=%p tjoint_offset=0x%03x sidecar=\"%s\"",
                     chain->name, requested->name, target->name,
                     retargeted,
                     loose_fallback,
                     (unsigned long)elapsed,
                     swing,
                     visual_r[0], visual_r[1], visual_r[2],
                     target->sim_rotation_rest[0],
                     target->sim_rotation_rest[1],
                     target->sim_rotation_rest[2],
                     (target->s_rotation_base && target->s_rotation_offset >= 0) ? 1 : 0,
                     (target->addon_rotation_base && target->addon_rotation_offset >= 0) ? 1 : 0,
                     target->addon_rotation_base,
                     target->addon_rotation_offset,
                     sc->path);
        }
    }
}

static int addon_chain_collision_person_index(physx_sidecar_t *sc,
                                              physx_chain_t *chain)
{
    char person[16];
    if (!addon_parent_camera_relative_person(sc, chain, NULL,
                                             person, sizeof(person))) {
        return -1;
    }
    return addon_person_index_from_name(person);
}

static int addon_chain_body_collider_person_ready(physx_chain_t *chain,
                                                  int person_index,
                                                  DWORD now)
{
    body_chain_collider_person_state_t *state;
    static int unsettled_collision_logged[4];
    if (!chain || !chain->collision_enabled ||
        person_index < 0 || person_index >= 4) {
        return 0;
    }
    body_profile_set_active_person_config(person_index);
    if (!body_chain_collider_cfg.enabled &&
        !(chain->collision_scope & PHYSX_COLLISION_SCOPE_ROOM)) {
        body_profile_set_active_person_config(-1);
        return 0;
    }
    update_body_chain_colliders_for_person(person_index, now);
    state = &body_chain_collider_states[person_index];
    body_profile_set_active_person_config(-1);
    if (!state->basis_valid ||
        !state->valid[BODY_COLLIDER_ROOT] ||
        state->scene_liveness_engine_invisible ||
        state->scene_liveness_quarantined) {
        return 0;
    }
    if (!state->ready && !unsettled_collision_logged[person_index]) {
        unsettled_collision_logged[person_index] = 1;
        log_line("addon sidecar collision using unsettled body-local frame person_index=%d sample_ready=%d chain_points_ready=%d limb_points_ready=%d note=\"sidecar collision uses populated body-local colliders while native penis/testicle response keeps its stricter ready gate\"",
                 person_index + 1,
                 state->sample_ready,
                 state->chain_points_ready,
                 state->limb_points_ready);
    }
    return 1;
}

static int addon_chain_body_collider_ready(physx_sidecar_t *sc,
                                           physx_chain_t *chain,
                                           DWORD now,
                                           int *person_index_out)
{
    int person_index = addon_chain_collision_person_index(sc, chain);
    if (person_index_out) *person_index_out = person_index;
    return addon_chain_body_collider_person_ready(chain, person_index, now);
}

static int addon_chain_view_to_body_local(
    const body_chain_collider_person_state_t *state,
    const float view[3],
    float out[3])
{
    float delta[3];
    if (!state || !view || !out ||
        !state->basis_valid ||
        !state->valid[BODY_COLLIDER_ROOT]) {
        return 0;
    }
    delta[0] = view[0] - state->view_position[BODY_COLLIDER_ROOT][0];
    delta[1] = view[1] - state->view_position[BODY_COLLIDER_ROOT][1];
    delta[2] = view[2] - state->view_position[BODY_COLLIDER_ROOT][2];
    return body_collider_view_delta_to_local(delta,
                                             state->basis_h,
                                             state->basis_v,
                                             state->basis_s,
                                             out);
}

static int addon_chain_target_body_local(physx_sidecar_t *sc,
                                         physx_chain_t *chain,
                                         physx_target_t *target,
                                         int person_index,
                                         const body_chain_collider_person_state_t *state,
                                         float out[3])
{
    char person[16];
    char fallback[192];
    float view[3];
    if (!target || !out || !state ||
        person_index < 0 || person_index >= 4) {
        return 0;
    }
    _snprintf(person, sizeof(person), "Person%02d", person_index + 1);
    fallback[0] = 0;
    if (!target->addon_simulated_target) {
        _snprintf(fallback, sizeof(fallback), "S%s", target->name);
        if (body_collider_engine_pivot_view(person, target->name,
                                            fallback, view) &&
            addon_chain_view_to_body_local(state, view, out)) {
            addon_chain_note_body_root_person(chain, person, GetTickCount(),
                                              "collision-pivot");
            (void)sc;
            return 1;
        }
    }
    if (sc && chain && chain->addon_owner_person[0] &&
        !sidecar_selected_for_owner(sc, chain->addon_owner_person)) {
        return 0;
    }
    resolve_engine_symbols();
    if (engine_GetModelViewRotationPivot &&
        target->object &&
        !is_nil_engine_object(target->raw_object, target->object)) {
        engine_GetModelViewRotationPivot(target->object, view);
        if (sane_probe_float(view[0]) &&
            sane_probe_float(view[1]) &&
            sane_probe_float(view[2]) &&
            addon_chain_view_to_body_local(state, view, out)) {
            return 1;
        }
    }
    return 0;
}

static int addon_chain_collision_first_target_index(physx_chain_t *chain)
{
    if (chain &&
        chain->addon_chain &&
        chain->parent_name[0] &&
        chain->target_count > 1 &&
        _stricmp(chain->targets[0].name, chain->parent_name) == 0 &&
        _stricmp(chain->targets[1].name, chain->name) == 0) {
        return 1;
    }
    return 0;
}

static int addon_chain_terminal_endpoint_body_local(
    physx_chain_t *chain, physx_target_t *target,
    const body_chain_collider_person_state_t *state,
    const float previous_local[3], const float start_local[3],
    float end_local[3]);

static int addon_chain_collision_points_body_local(
    physx_sidecar_t *sc,
    physx_chain_t *chain,
    float points[32][3],
    int *count_out,
    int *person_index_out,
    DWORD now)
{
    int person_index;
    body_chain_collider_person_state_t *state;
    int t;
    int count = 0;
    int source_start;
    if (count_out) *count_out = 0;
    if (person_index_out) *person_index_out = -1;
    if (!sc || !chain || !points || chain->target_count <= 0) return 0;
    if (!chain->addon_scene_visible ||
        !sidecar_selected_for_owner(sc, chain->addon_owner_person)) {
        return 0;
    }
    if (!addon_chain_body_collider_ready(sc, chain, now, &person_index)) {
        return 0;
    }
    if (person_index_out) *person_index_out = person_index;
    state = &body_chain_collider_states[person_index];
    source_start = addon_chain_collision_first_target_index(chain);
    count = chain->target_count - source_start;
    if (count > 32) count = 32;
    for (t = 0; t < count; t++) {
        int source_index = source_start + t;
        physx_target_t *target = &chain->targets[source_index];
        if (addon_chain_target_body_local(sc, chain, target,
                                          person_index, state,
                                          points[t])) {
            continue;
        }
        if (t == 0) {
            if (source_start > 0) {
                float parent_point[3];
                physx_target_t *source_parent =
                    &chain->targets[source_start - 1];
                if (addon_chain_target_body_local(sc, chain, source_parent,
                                                  person_index, state,
                                                  parent_point)) {
                    points[t][0] = parent_point[0] + target->sim_offset[0];
                    points[t][1] = parent_point[1] + target->sim_offset[1];
                    points[t][2] = parent_point[2] + target->sim_offset[2];
                    continue;
                }
            }
            return 0;
        }
        points[t][0] = points[t - 1][0] + target->sim_offset[0];
        points[t][1] = points[t - 1][1] + target->sim_offset[1];
        points[t][2] = points[t - 1][2] + target->sim_offset[2];
    }
    if (count > 0 && count < 32) {
        physx_target_t *terminal =
            &chain->targets[chain->target_count - 1];
        if (addon_chain_terminal_endpoint_body_local(
                chain, terminal, state,
                count >= 2 ? points[count - 2] : NULL,
                points[count - 1], points[count])) {
            count++;
        }
    }
    if (count_out) *count_out = count;
    return count > 1;
}

static int addon_sidecar_collision_debug_any(void)
{
    int i, c;
    for (i = 0; i < sidecar_count; i++) {
        physx_sidecar_t *sc = &sidecars[i];
        if (!sc->loaded || !sc->enabled) continue;
        for (c = 0; c < sc->chain_count; c++) {
            physx_chain_t *chain = &sc->chains[c];
            if (!chain->addon_chain ||
                !chain->collision_enabled ||
                !chain->collision_debug_draw ||
                chain->target_count <= 1) {
                continue;
            }
            return 1;
        }
    }
    return 0;
}

static void addon_chain_collision_normalize_target(physx_target_t *target)
{
    float len;
    float inv_len;
    float dot;
    int axis;
    if (!target || target->sim_length <= 0.0001f) return;
    len = physx_vec3_len(target->sim_offset);
    if (len <= 0.0001f) return;
    inv_len = 1.0f / len;
    for (axis = 0; axis < 3; axis++) {
        target->sim_offset[axis] *= target->sim_length * inv_len;
    }
    dot = target->sim_velocity[0] * target->sim_offset[0] +
          target->sim_velocity[1] * target->sim_offset[1] +
          target->sim_velocity[2] * target->sim_offset[2];
    dot /= (target->sim_length * target->sim_length);
    for (axis = 0; axis < 3; axis++) {
        target->sim_velocity[axis] -= dot * target->sim_offset[axis];
    }
}

static float addon_chain_collision_proximal_deadzone(int segment_index)
{
    return segment_index <= 1 ? 0.220f : 0.080f;
}

static int addon_chain_collision_previous_segment(
    physx_chain_t *chain,
    physx_target_t *target,
    const float current_start[3],
    const float current_end[3],
    float previous_start[3],
    float previous_end[3],
    float *sweep_radius_out)
{
    int target_index;
    float start_move;
    float end_move;
    float move;
    if (sweep_radius_out) *sweep_radius_out = 0.0f;
    if (!chain || !target || !current_start || !current_end ||
        !previous_start || !previous_end ||
        !chain->addon_chain ||
        !chain->addon_collision_prev_ready) {
        return 0;
    }
    target_index = (int)(target - chain->targets);
    if (target_index <= 0 || target_index >= 32 ||
        target_index >= chain->target_count ||
        !chain->addon_collision_prev_valid[target_index - 1] ||
        !chain->addon_collision_prev_valid[target_index]) {
        return 0;
    }
    previous_start[0] = chain->addon_collision_prev_points[target_index - 1][0];
    previous_start[1] = chain->addon_collision_prev_points[target_index - 1][1];
    previous_start[2] = chain->addon_collision_prev_points[target_index - 1][2];
    previous_end[0] = chain->addon_collision_prev_points[target_index][0];
    previous_end[1] = chain->addon_collision_prev_points[target_index][1];
    previous_end[2] = chain->addon_collision_prev_points[target_index][2];
    if (!physx_vec3_sane_limit(previous_start, 25.0f) ||
        !physx_vec3_sane_limit(previous_end, 25.0f)) {
        return 0;
    }
    start_move = body_collider_distance(previous_start, current_start);
    end_move = body_collider_distance(previous_end, current_end);
    move = start_move > end_move ? start_move : end_move;
    if (sweep_radius_out &&
        sane_probe_float(move) &&
        move > 0.0180f) {
        *sweep_radius_out = physx_clampf((move - 0.0180f) * 0.85f,
                                         0.0f, 0.065f);
    }
    return 1;
}

static void addon_chain_collision_apply_previous_side(
    float normal[3],
    const float previous_start[3],
    const float previous_end[3],
    float segment_t,
    const float closest_body[3],
    const float body_a[3],
    const float body_b[3])
{
    float previous_contact[3];
    float previous_side[3];
    float body_axis[3];
    float body_axis_len;
    float previous_side_len;
    float dot;
    int axis;
    if (!normal || !previous_start || !previous_end ||
        !closest_body || !body_a || !body_b) {
        return;
    }
    segment_t = physx_clampf(segment_t, 0.0f, 1.0f);
    for (axis = 0; axis < 3; axis++) {
        previous_contact[axis] =
            previous_start[axis] +
            (previous_end[axis] - previous_start[axis]) * segment_t;
        previous_side[axis] = previous_contact[axis] - closest_body[axis];
        body_axis[axis] = body_b[axis] - body_a[axis];
    }
    body_axis_len = physx_vec3_len(body_axis);
    if (body_axis_len > 0.0001f) {
        float along;
        for (axis = 0; axis < 3; axis++) body_axis[axis] /= body_axis_len;
        along = previous_side[0] * body_axis[0] +
                previous_side[1] * body_axis[1] +
                previous_side[2] * body_axis[2];
        for (axis = 0; axis < 3; axis++) {
            previous_side[axis] -= body_axis[axis] * along;
        }
    }
    previous_side_len = physx_vec3_len(previous_side);
    if (previous_side_len <= 0.0001f) return;
    dot = normal[0] * previous_side[0] +
          normal[1] * previous_side[1] +
          normal[2] * previous_side[2];
    if (dot >= 0.0f) return;
    normal[0] = -normal[0];
    normal[1] = -normal[1];
    normal[2] = -normal[2];
}

static int addon_chain_stable_collision_normal(physx_chain_t *chain,
                                               physx_target_t *target,
                                               DWORD now,
                                               const float correction[3],
                                               float normal[3])
{
    float correction_len;
    float previous_len;
    float direction_dot;
    float speed;
    float rest_limit;
    if (!chain || !chain->addon_chain || !target || !correction || !normal) {
        return 0;
    }
    correction_len = physx_vec3_len(correction);
    if (correction_len <= 0.000001f) return 0;
    normal[0] = correction[0] / correction_len;
    normal[1] = correction[1] / correction_len;
    normal[2] = correction[2] / correction_len;
    if (!target->addon_collision_correction_valid ||
        target->addon_collision_correction_tick == now ||
        now - target->addon_collision_correction_tick > 96u) {
        return 0;
    }
    previous_len = physx_vec3_len(target->addon_collision_correction_prev);
    if (previous_len <= 0.000001f) return 0;
    direction_dot =
        (target->addon_collision_correction_prev[0] * normal[0] +
         target->addon_collision_correction_prev[1] * normal[1] +
         target->addon_collision_correction_prev[2] * normal[2]) /
        previous_len;
    if (direction_dot < 0.90f) return 0;
    rest_limit = physx_clampf(chain->collision_radius * 1.35f,
                              0.0060f, 0.0400f);
    if (correction_len > rest_limit || previous_len > rest_limit) return 0;
    speed = physx_vec3_len(target->sim_velocity);
    return speed <= 0.30f;
}

static int addon_chain_previous_contact_stable(physx_chain_t *chain,
                                               physx_target_t *target,
                                               DWORD now,
                                               float sweep_radius)
{
    float previous_len;
    float speed;
    float rest_limit;
    float enter_sweep_limit;
    float exit_sweep_limit;
    if (!chain || !chain->addon_chain || !target ||
        !target->addon_collision_direct_contact_tick ||
        now - target->addon_collision_direct_contact_tick > 96u) {
        if (target) target->addon_collision_response_stable = 0;
        return 0;
    }
    previous_len =
        physx_vec3_len(target->addon_collision_direct_correction);
    rest_limit = physx_clampf(chain->collision_radius * 1.35f,
                              0.0060f, 0.0400f);
    enter_sweep_limit = physx_clampf(chain->collision_radius * 0.55f,
                                     0.0040f, 0.0150f);
    exit_sweep_limit = physx_clampf(chain->collision_radius * 0.90f,
                                    0.0080f, 0.0250f);
    speed = physx_vec3_len(target->sim_velocity);
    if (target->addon_collision_response_stable) {
        if (previous_len <= 0.000001f ||
            previous_len > rest_limit * 1.50f ||
            speed > 0.55f ||
            sweep_radius > exit_sweep_limit) {
            target->addon_collision_response_stable = 0;
            return 0;
        }
        return 1;
    }
    if (target->addon_collision_direct_contact_tick == now ||
        previous_len <= 0.000001f || previous_len > rest_limit ||
        speed > 0.30f || sweep_radius > enter_sweep_limit) {
        return 0;
    }
    target->addon_collision_response_stable = 1;
    return 1;
}

static void addon_chain_note_direct_collision_contact(
    physx_chain_t *chain,
    physx_target_t *target,
    DWORD now,
    const float correction[3])
{
    if (!chain || !chain->addon_chain || !target || !correction) return;
    target->addon_collision_direct_contact_tick = now;
    target->addon_collision_direct_correction[0] = correction[0];
    target->addon_collision_direct_correction[1] = correction[1];
    target->addon_collision_direct_correction[2] = correction[2];
}

static void addon_chain_damp_stable_collision_velocity(
    physx_chain_t *chain,
    physx_target_t *contact_target,
    const float normal[3])
{
    int target_index;
    int source_start;
    int i;
    if (!chain || !chain->addon_chain || !contact_target || !normal) return;
    target_index = (int)(contact_target - chain->targets);
    if (target_index < 0 || target_index >= chain->target_count) return;
    source_start = addon_chain_collision_first_target_index(chain);
    if (source_start < 0) source_start = 0;
    if (source_start > target_index) source_start = target_index;
    for (i = source_start; i <= target_index; i++) {
        physx_target_t *target = &chain->targets[i];
        physx_contact_link_velocity(target->sim_velocity, target->sim_offset,
                                    normal);
    }
}

static void addon_chain_apply_collision_correction(physx_chain_t *chain,
                                                   physx_target_t *target,
                                                   float correction[3])
{
    float correction_len;
    float max_push;
    float normal[3];
    int axis;
    if (!target || !correction) return;
    correction_len = physx_vec3_len(correction);
    if (correction_len <= 0.000001f) return;
    if (chain && chain->addon_chain) {
        max_push = chain->collision_radius * 1.45f;
        if (target->sim_length > 0.0001f) {
            float length_push = target->sim_length * 0.18f;
            if (length_push < max_push) max_push = length_push;
        }
        max_push = physx_clampf(max_push, 0.0030f, 0.0240f);
    } else {
        max_push = chain ? chain->collision_radius * 5.0f : 0.050f;
        if (target->sim_length > 0.0001f) {
            float length_push = target->sim_length * 0.35f;
            if (length_push > max_push) max_push = length_push;
        }
        max_push = physx_clampf(max_push, 0.010f, 0.100f);
    }
    if (correction_len > max_push) {
        float scale = max_push / correction_len;
        correction[0] *= scale;
        correction[1] *= scale;
        correction[2] *= scale;
        correction_len = max_push;
    }
    normal[0] = correction[0] / correction_len;
    normal[1] = correction[1] / correction_len;
    normal[2] = correction[2] / correction_len;
    target->sim_contact_corrected = 1;
    if (target->sim_length > 0.0001f) {
        physx_contact_link_position(target->sim_offset, target->sim_length,
                                    correction);
    } else {
        float inward = vec3_dot(target->sim_velocity, normal);
        for (axis = 0; axis < 3; axis++) {
            target->sim_offset[axis] += correction[axis];
            if (inward < 0.0f) target->sim_velocity[axis] -= normal[axis]*inward;
        }
    }
    addon_chain_collision_normalize_target(target);
    physx_contact_link_velocity(target->sim_velocity, target->sim_offset,
                                normal);

}

/* A static room surface is a positional constraint, not a soft secondary
   body contact. Apply enough of the swept correction to keep the link on its
   entry side, then restore the authored link length. Ordinary body/add-on
   collision uses the length-constrained response above. */
static void addon_chain_apply_room_collision_correction(
    physx_chain_t *chain, physx_target_t *target,
    const float correction[3])
{
    float correction_len;
    float max_push;
    float applied[3];
    float normal[3];
    float previous_len;
    float direction_dot = -1.0f;
    DWORD now;
    int axis;
    if (!chain || !target || !correction) return;
    correction_len = physx_vec3_len(correction);
    if (correction_len <= 0.000001f) return;
    memcpy(applied, correction, sizeof(applied));
    max_push = chain->collision_radius * 4.0f;
    if (target->sim_length > 0.0001f) {
        float length_push = target->sim_length * 0.75f;
        if (length_push > max_push) max_push = length_push;
    }
    max_push = physx_clampf(max_push, 0.012f, 0.120f);
    if (correction_len > max_push) {
        float scale = max_push / correction_len;
        applied[0] *= scale;
        applied[1] *= scale;
        applied[2] *= scale;
        correction_len = max_push;
    }
    normal[0] = applied[0] / correction_len;
    normal[1] = applied[1] / correction_len;
    normal[2] = applied[2] / correction_len;
    now = GetTickCount();
    previous_len = physx_vec3_len(target->room_collision_contact_direction);
    if (target->room_collision_contact_valid &&
        now - target->room_collision_contact_tick <= 120u &&
        previous_len > 0.000001f) {
        direction_dot =
            normal[0] * target->room_collision_contact_direction[0] /
                previous_len +
            normal[1] * target->room_collision_contact_direction[1] /
                previous_len +
            normal[2] * target->room_collision_contact_direction[2] /
                previous_len;
    }
    if (direction_dot >= 0.85f) {
        if (target->room_collision_rest_frames < 12) {
            target->room_collision_rest_frames++;
        }
    } else {
        target->room_collision_rest_frames = 1;
    }
    target->room_collision_contact_valid = 1;
    target->room_collision_contact_tick = now;
    target->room_collision_contact_direction[0] = normal[0];
    target->room_collision_contact_direction[1] = normal[1];
    target->room_collision_contact_direction[2] = normal[2];
    for (axis = 0; axis < 3; axis++) {
        target->sim_offset[axis] += applied[axis];
    }
    target->addon_collision_correction_valid = 0;
    target->sim_contact_corrected = 1;
    addon_chain_collision_normalize_target(target);
    physx_contact_link_velocity(target->sim_velocity, target->sim_offset, normal);

}

static void addon_chain_update_room_rest_pose(
    physx_chain_t *chain, physx_target_t *target,
    const float world_correction[3], DWORD now)
{
    float correction_len;
    float normal[3];
    float previous_len;
    float normal_dot = -1.0f;
    float settle_penetration;
    float update_alpha;
    LONG generation;
    int continuous;
    int shallow;
    int axis;
    if (!chain || !target || !world_correction) return;
    correction_len = physx_vec3_len(world_correction);
    if (!sane_probe_float(correction_len) || correction_len <= 0.000001f) {
        return;
    }
    for (axis = 0; axis < 3; axis++) {
        normal[axis] = world_correction[axis] / correction_len;
    }
    generation = InterlockedCompareExchange(&named_node_generation, 0, 0);
    previous_len = physx_vec3_len(
        target->room_collision_rest_pose_normal);
    if (previous_len > 0.000001f) {
        normal_dot =
            normal[0] * target->room_collision_rest_pose_normal[0] /
                previous_len +
            normal[1] * target->room_collision_rest_pose_normal[1] /
                previous_len +
            normal[2] * target->room_collision_rest_pose_normal[2] /
                previous_len;
    }
    continuous =
        target->room_collision_rest_pose_valid &&
        target->room_collision_rest_pose_generation == generation &&
        now - target->room_collision_rest_pose_tick <= 180u &&
        normal_dot >= 0.90f;
    settle_penetration = physx_clampf(
        chain->collision_radius * 0.90f, 0.010f, 0.030f);
    shallow = correction_len <= settle_penetration;
    if (!continuous || !shallow) {
        target->room_collision_rest_pose_frames = 1;
        target->room_collision_rest_pose_sleeping = 0;
        update_alpha = 1.0f;
    } else {
        if (target->room_collision_rest_pose_frames < 30) {
            target->room_collision_rest_pose_frames++;
        }
        target->room_collision_rest_pose_sleeping = 0;
        update_alpha =
            target->room_collision_rest_pose_sleeping ? 0.05f : 0.35f;
    }
    for (axis = 0; axis < 3; axis++) {
        if (update_alpha >= 0.999f) {
            target->room_collision_rest_pose_offset[axis] =
                target->sim_offset[axis];
        } else {
            target->room_collision_rest_pose_offset[axis] +=
                (target->sim_offset[axis] -
                 target->room_collision_rest_pose_offset[axis]) *
                update_alpha;
        }
        target->room_collision_rest_pose_normal[axis] = normal[axis];
    }
    target->room_collision_rest_pose_valid = 1;
    target->room_collision_rest_pose_generation = generation;
    target->room_collision_rest_pose_tick = now;
    if (target->room_collision_rest_pose_sleeping) {
        target->sim_velocity[0] = 0.0f;
        target->sim_velocity[1] = 0.0f;
        target->sim_velocity[2] = 0.0f;
    }
}

static void addon_chain_wake_room_rest(physx_chain_t *chain)
{
    int first;
    int target_index;
    if (!chain) return;
    chain->addon_room_rest_sleeping = 0;
    chain->addon_room_rest_frames = 0;
    chain->addon_room_rest_generation = 0;
    chain->addon_room_rest_tick = 0;
    first = addon_chain_collision_first_target_index(chain);
    if (first < 1) first = 1;
    for (target_index = first;
         target_index < chain->target_count && target_index < 32;
         target_index++) {
        chain->targets[target_index].room_collision_rest_pose_sleeping = 0;
        chain->targets[target_index].room_collision_rest_pose_frames = 0;
    }
}

static int addon_chain_world_room_correction_to_bend(
    physx_sidecar_t *sc, physx_chain_t *chain, physx_target_t *target,
    DWORD now, const float world_correction[3], float bend[3])
{
    physx_target_t *root_parent;
    const char *runtime = "";
    void *parent_raw;
    float correction_len;
    float world_direction[3];
    float view_direction[3];
    float parent_matrix[9];
    float parent_drive[3];
    float bend_len;
    if (!sc || !chain || !target || !world_correction || !bend ||
        chain->target_count <= 0) {
        return 0;
    }
    correction_len = physx_vec3_len(world_correction);
    if (correction_len <= 0.000001f) return 0;
    world_direction[0] = world_correction[0] / correction_len;
    world_direction[1] = world_correction[1] / correction_len;
    world_direction[2] = world_correction[2] / correction_len;
    if (!camera_world_to_view_direction(world_direction, view_direction)) {
        return 0;
    }
    root_parent = &chain->targets[0];
    if (!root_parent->name[0] || root_parent->addon_simulated_target) {
        return 0;
    }
    parent_raw = root_parent->raw_object;
    if (!parent_raw) parent_raw = root_parent->object;
    if (!parent_raw) parent_raw = root_parent->s_raw_object;
    if (!parent_raw) parent_raw = root_parent->s_object;
    if (!parent_raw ||
        addon_parent_name_can_use_body_drive(root_parent->name)) {
        void *runtime_raw = addon_effective_parent_cached_runtime_raw(
            sc, chain, root_parent, now, &runtime);
        if (runtime_raw) parent_raw = runtime_raw;
    }
    if (!parent_raw ||
        !body_chain_read_mat3_rows(parent_raw, parent_matrix) ||
        !addon_normalize_basis_rows(parent_matrix)) {
        return 0;
    }
    addon_chain_project_direction_basis(view_direction, parent_matrix,
                                        parent_drive);
    if (!physx_vec3_sane_limit(parent_drive, 4.0f) ||
        !addon_chain_world_gravity_bend_vector(
            chain, target, parent_drive, bend)) {
        return 0;
    }

    /* The gravity mapper returns a direction whose length depends on the
       configured source/secondary channel blend. That length is meaningful
       for gravity strength, but not for a positional collision constraint.
       Keeping it here made the same room contact strong in one orientation
       and almost zero in the opposite orientation. Preserve only the mapped
       bend direction and restore the actual world-space penetration depth. */
    bend_len = physx_vec3_len(bend);
    if (bend_len <= 0.000001f) return 0;
    bend[0] *= correction_len / bend_len;
    bend[1] *= correction_len / bend_len;
    bend[2] *= correction_len / bend_len;
    return physx_vec3_sane_limit(bend, 0.25f);
}

/* Convert a positional correction at the visible child joint into the
   equivalent change of the solver's fixed-length link.  The add-on solver
   vector and the evaluated TK17 joint segment do not share the same numeric
   axes once parent rotations have been applied.  Reusing gravity's channel
   mapper can therefore place a floor push along the current link length;
   normalization then erases it.  Build the minimal rotation that currently
   maps the solver link onto the visible segment, move the visible endpoint,
   and carry that new direction back through the inverse rotation. */
static int addon_chain_visible_room_correction_to_sim_delta(
    physx_target_t *target,
    const float world_start[3], const float world_end[3],
    const float world_correction[3], float delta_out[3])
{
    float sim[3];
    float visible[3];
    float desired[3];
    float cross_sw[3];
    float mapped[3];
    float sim_len;
    float visible_len;
    float desired_len;
    float sine;
    float cosine;
    int axis;
    if (!target || !world_start || !world_end || !world_correction ||
        !delta_out) {
        return 0;
    }
    sim_len = physx_vec3_len(target->sim_offset);
    visible[0] = world_end[0] - world_start[0];
    visible[1] = world_end[1] - world_start[1];
    visible[2] = world_end[2] - world_start[2];
    visible_len = physx_vec3_len(visible);
    memcpy(desired, visible, sizeof(desired));
    physx_contact_link_position(desired, visible_len, world_correction);
    desired_len = physx_vec3_len(desired);
    if (sim_len <= 0.0001f || visible_len <= 0.0005f ||
        desired_len <= 0.0005f ||
        !sane_probe_float(sim_len) || !sane_probe_float(visible_len) ||
        !sane_probe_float(desired_len)) {
        return 0;
    }
    for (axis = 0; axis < 3; axis++) {
        sim[axis] = target->sim_offset[axis] / sim_len;
        visible[axis] /= visible_len;
        desired[axis] /= desired_len;
    }
    cosine = physx_clampf(vec3_dot(sim, visible), -1.0f, 1.0f);
    cross_sw[0] = sim[1] * visible[2] - sim[2] * visible[1];
    cross_sw[1] = sim[2] * visible[0] - sim[0] * visible[2];
    cross_sw[2] = sim[0] * visible[1] - sim[1] * visible[0];
    sine = physx_vec3_len(cross_sw);
    if (sine > 0.00001f) {
        float rotation_axis[3];
        float axis_cross_desired[3];
        float axis_dot_desired;
        for (axis = 0; axis < 3; axis++) {
            rotation_axis[axis] = cross_sw[axis] / sine;
        }
        axis_cross_desired[0] =
            rotation_axis[1] * desired[2] -
            rotation_axis[2] * desired[1];
        axis_cross_desired[1] =
            rotation_axis[2] * desired[0] -
            rotation_axis[0] * desired[2];
        axis_cross_desired[2] =
            rotation_axis[0] * desired[1] -
            rotation_axis[1] * desired[0];
        axis_dot_desired = vec3_dot(rotation_axis, desired);
        /* Inverse Rodrigues rotation: visible-space back to solver-space. */
        for (axis = 0; axis < 3; axis++) {
            mapped[axis] = desired[axis] * cosine -
                axis_cross_desired[axis] * sine +
                rotation_axis[axis] * axis_dot_desired *
                    (1.0f - cosine);
        }
    } else if (cosine >= 0.0f) {
        memcpy(mapped, desired, sizeof(mapped));
    } else {
        float rotation_axis[3] = { 0.0f, 0.0f, 0.0f };
        float axis_dot_desired;
        int smallest_axis = 0;
        if (physx_absf(sim[1]) < physx_absf(sim[smallest_axis])) {
            smallest_axis = 1;
        }
        if (physx_absf(sim[2]) < physx_absf(sim[smallest_axis])) {
            smallest_axis = 2;
        }
        rotation_axis[smallest_axis] = 1.0f;
        {
            float projection = vec3_dot(rotation_axis, sim);
            float axis_len;
            for (axis = 0; axis < 3; axis++) {
                rotation_axis[axis] -= sim[axis] * projection;
            }
            axis_len = physx_vec3_len(rotation_axis);
            if (axis_len <= 0.00001f) return 0;
            for (axis = 0; axis < 3; axis++) {
                rotation_axis[axis] /= axis_len;
            }
        }
        axis_dot_desired = vec3_dot(rotation_axis, desired);
        for (axis = 0; axis < 3; axis++) {
            mapped[axis] = -desired[axis] +
                2.0f * rotation_axis[axis] * axis_dot_desired;
        }
    }
    for (axis = 0; axis < 3; axis++) {
        delta_out[axis] = mapped[axis] * target->sim_length -
                          target->sim_offset[axis];
    }
    return physx_vec3_sane_limit(delta_out, 0.30f);
}

/* A terminal contact is a constraint on the complete chain, not only on its
   final joint.  In the important hanging-hair case the floor correction is
   nearly parallel to the last link; adding it to that one fixed-length link
   is erased by normalization.  Bend all upstream simulated joints toward a
   shared safe terminal point instead.  When the chain is almost normal to
   the surface, add a deterministic tangent escape so the chain can fold and
   shorten its normal projection rather than buzzing in an impossible
   straight-through-floor configuration. */
static int addon_chain_solve_body_contact(physx_chain_t *chain,
    physx_target_t *target,const float pivot[3],const float end[3],
    const float request[3],float delta[3],float achieved[3],float gradient[3]);

/* World-space room contact expressed in the same body frame as the frozen
   evaluated basis. Translation cancels: only the pivot-to-contact vector and
   requested displacement are transformed. */
static int addon_chain_solve_room_contact(physx_chain_t *chain,
    physx_target_t *target,const body_chain_collider_person_state_t *state,
    const float pivot[3],const float end[3],const float request[3],
    float delta[3],float gradient[3])
{
    float relative[3],body_end[3],body_request[3],achieved[3];
    const float origin[3]={0,0,0};
    int axis;
    if(!target->contact_basis_tick) return 0;
    for(axis=0;axis<3;axis++) relative[axis]=end[axis]-pivot[axis];
    if(!room_collision_world_vector_to_body_local(state,relative,body_end) ||
       !room_collision_world_vector_to_body_local(state,request,body_request)) return 0;
    return addon_chain_solve_body_contact(chain,target,origin,body_end,
                                          body_request,delta,achieved,gradient);
}

static void addon_chain_room_contact_request(const float surface[3],
    const float legacy_escape[3],float gain,int output_aware,float request[3])
{
    int axis;
    /* The numerical solve already computes a separating bend. Adding the
       old axial escape tilts its contact plane and drives lateral motion. */
    const float *source=output_aware ? surface : legacy_escape;
    for(axis=0;axis<3;axis++) request[axis]=source[axis]*gain;
}

static int addon_chain_apply_terminal_room_manifold(
    physx_sidecar_t *sc, physx_chain_t *chain, int person_index,
    const float world_end[3], const float world_correction[3], DWORD now)
{
    body_chain_collider_person_state_t *state;
    float pivot_local[32][3];
    float pivot_world[32][3];
    int pivot_valid[32];
    float correction[3];
    float normal[3];
    float root_to_end[3];
    float tangent[3];
    float correction_len;
    float reach;
    float root_len;
    float axial;
    float tangent_len;
    float escape = 0.0f;
    float chain_settle_penetration;
    LONG generation;
    int first;
    int last;
    int target_index;
    int applied = 0;
    int output_aware_applied = 0;
    int axis;
    if (!sc || !chain || !world_end || !world_correction ||
        person_index < 0 || person_index >= 4) {
        return 0;
    }
    state = &body_chain_collider_states[person_index];
    if (!state->ready || !state->basis_valid) return 0;
    generation = InterlockedCompareExchange(&named_node_generation, 0, 0);
    first = addon_chain_collision_first_target_index(chain);
    if (first < 1) first = 1;
    last = chain->target_count - 1;
    if (last < first || last >= 32) return 0;
    memset(pivot_valid, 0, sizeof(pivot_valid));
    for (target_index = first; target_index <= last; target_index++) {
        if (addon_chain_target_body_local(
                sc, chain, &chain->targets[target_index], person_index,
                state, pivot_local[target_index]) &&
            room_collision_body_local_point_to_world(
                state, pivot_local[target_index],
                pivot_world[target_index])) {
            pivot_valid[target_index] = 1;
        }
    }
    if (!pivot_valid[first]) return 0;
    correction_len = physx_vec3_len(world_correction);
    if (!sane_probe_float(correction_len) || correction_len <= 0.000001f) {
        return 0;
    }
    for (axis = 0; axis < 3; axis++) {
        normal[axis] = world_correction[axis] / correction_len;
        root_to_end[axis] = world_end[axis] - pivot_world[first][axis];
        correction[axis] = world_correction[axis];
    }
    root_len = physx_vec3_len(root_to_end);
    if (!sane_probe_float(root_len) || root_len <= 0.001f) return 0;
    axial = physx_absf(vec3_dot(root_to_end, normal)) / root_len;
    tangent[0] = root_to_end[0] -
        normal[0] * vec3_dot(root_to_end, normal);
    tangent[1] = root_to_end[1] -
        normal[1] * vec3_dot(root_to_end, normal);
    tangent[2] = root_to_end[2] -
        normal[2] * vec3_dot(root_to_end, normal);
    tangent_len = physx_vec3_len(tangent);

    if (axial >= 0.65f) {
        physx_target_t *terminal = &chain->targets[last];
        if (tangent_len <= 0.002f &&
            terminal->room_collision_terminal_track_valid) {
            float tracked_delta[3];
            float tracked_normal;
            tracked_delta[0] =
                terminal->room_collision_terminal_track_world[0] -
                world_end[0];
            tracked_delta[1] =
                terminal->room_collision_terminal_track_world[1] -
                world_end[1];
            tracked_delta[2] =
                terminal->room_collision_terminal_track_world[2] -
                world_end[2];
            tracked_normal = vec3_dot(tracked_delta, normal);
            tangent[0] = tracked_delta[0] - normal[0] * tracked_normal;
            tangent[1] = tracked_delta[1] - normal[1] * tracked_normal;
            tangent[2] = tracked_delta[2] - normal[2] * tracked_normal;
            tangent_len = physx_vec3_len(tangent);
        }
        if (tangent_len <= 0.002f) {
            float candidate[3] = { 1.0f, 0.0f, 0.0f };
            float projection;
            if (physx_absf(normal[0]) > 0.80f) {
                candidate[0] = 0.0f;
                candidate[2] = 1.0f;
            }
            projection = vec3_dot(candidate, normal);
            tangent[0] = candidate[0] - normal[0] * projection;
            tangent[1] = candidate[1] - normal[1] * projection;
            tangent[2] = candidate[2] - normal[2] * projection;
            tangent_len = physx_vec3_len(tangent);
        }
        if (tangent_len > 0.0001f) {
            float max_escape = physx_clampf(
                chain->collision_radius * 6.0f, 0.040f, 0.160f);
            float geometry_escape = (float)sqrt(
                (double)physx_clampf(
                    2.0f * root_len * correction_len,
                    0.0f, max_escape * max_escape));
            escape = geometry_escape *
                physx_clampf((axial - 0.65f) / 0.35f, 0.0f, 1.0f);
            if (escape > root_len * 0.40f) escape = root_len * 0.40f;
            for (axis = 0; axis < 3; axis++) {
                tangent[axis] /= tangent_len;
                correction[axis] += tangent[axis] * escape;
            }
        }
    }

    reach = root_len;
    float total_gain = 0.0f;
    for (target_index = first; target_index <= last; target_index++) {
        physx_target_t *joint = &chain->targets[target_index];
        if (pivot_valid[target_index] && joint->addon_simulated_target &&
            joint->sim_length > 0.0001f) {
            float relative = reach > 0.001f ? physx_clampf(
                body_collider_distance(pivot_world[target_index], world_end)/reach,
                0.0f, 1.0f) : 0.0f;
            total_gain += 0.38f + 0.34f*relative;
        }
    }
    if (total_gain <= 0.000001f) return 0;
    for (target_index = first; target_index <= last; target_index++) {
        physx_target_t *joint = &chain->targets[target_index];
        float delta[3];
        float gain;
        float relative;
        float pivot_reach;
        float desired_correction[3];
        float contact_gradient[3];
        int output_aware;
        float rest_normal_len;
        float rest_normal_dot = -1.0f;
        float rest_update_alpha;
        float settle_penetration;
        int rest_pose_continuous;
        int settled_contact;
        if (!pivot_valid[target_index] ||
            !joint->addon_simulated_target ||
            joint->sim_length <= 0.0001f) {
            continue;
        }
        pivot_reach = body_collider_distance(
            pivot_world[target_index], world_end);
        relative = reach > 0.001f ?
            physx_clampf(pivot_reach / reach, 0.0f, 1.0f) : 0.0f;
        gain = (0.38f + 0.34f * relative) / total_gain;
        rest_normal_len =
            physx_vec3_len(joint->room_collision_rest_pose_normal);
        if (rest_normal_len > 0.000001f) {
            rest_normal_dot =
                normal[0] * joint->room_collision_rest_pose_normal[0] /
                    rest_normal_len +
                normal[1] * joint->room_collision_rest_pose_normal[1] /
                    rest_normal_len +
                normal[2] * joint->room_collision_rest_pose_normal[2] /
                    rest_normal_len;
        }
        rest_pose_continuous =
            joint->room_collision_rest_pose_valid &&
            joint->room_collision_rest_pose_generation == generation &&
            now - joint->room_collision_rest_pose_tick <= 180u &&
            rest_normal_dot >= 0.90f;
        settle_penetration = physx_clampf(
            chain->collision_radius * 0.90f, 0.010f, 0.030f);
        settled_contact = 0;
        output_aware=joint->contact_basis_tick!=0;
        addon_chain_room_contact_request(world_correction,correction,gain,
                                         output_aware,desired_correction);
        if(output_aware) {
            if(!addon_chain_solve_room_contact(chain,joint,state,
                    pivot_world[target_index],world_end,desired_correction,
                    delta,contact_gradient)) continue;
        } else if (!addon_chain_visible_room_correction_to_sim_delta(
                joint, pivot_world[target_index], world_end,
                desired_correction, delta)) {
            continue;
        }
        for (axis = 0; axis < 3; axis++) {
            joint->sim_offset[axis] += delta[axis];
        }
        joint->sim_contact_corrected = 1;
        if(output_aware) output_aware_applied++;
        addon_chain_collision_normalize_target(joint);
        {
            float delta_len = physx_vec3_len(delta);
            float contact_normal[3];
            if (delta_len > 0.000001f) {
                for (axis = 0; axis < 3; axis++) contact_normal[axis] =
                    output_aware ? contact_gradient[axis] : delta[axis]/delta_len;
                physx_contact_link_velocity(joint->sim_velocity, joint->sim_offset,
                                            contact_normal);
            }
        }
        rest_update_alpha = rest_pose_continuous ?
            (settled_contact ? 0.06f : 0.35f) : 1.0f;
        for (axis = 0; axis < 3; axis++) {
            if (!rest_pose_continuous) {
                joint->room_collision_rest_pose_offset[axis] =
                    joint->sim_offset[axis];
            } else {
                joint->room_collision_rest_pose_offset[axis] +=
                    (joint->sim_offset[axis] -
                     joint->room_collision_rest_pose_offset[axis]) *
                    rest_update_alpha;
            }
            joint->room_collision_rest_pose_normal[axis] = normal[axis];
        }
        joint->room_collision_rest_pose_valid = 1;
        joint->room_collision_rest_pose_sleeping = settled_contact;
        if (rest_pose_continuous &&
            correction_len <= settle_penetration) {
            if (joint->room_collision_rest_pose_frames < 30) {
                joint->room_collision_rest_pose_frames++;
            }
        } else {
            joint->room_collision_rest_pose_frames = 1;
            joint->room_collision_rest_pose_sleeping = 0;
        }
        joint->room_collision_rest_pose_generation = generation;
        joint->room_collision_rest_pose_tick = now;
        joint->room_collision_contact_valid = 1;
        joint->room_collision_contact_tick = now;
        joint->room_collision_contact_direction[0] = normal[0];
        joint->room_collision_contact_direction[1] = normal[1];
        joint->room_collision_contact_direction[2] = normal[2];
        if (joint->room_collision_rest_frames < 12) {
            joint->room_collision_rest_frames++;
        }
        applied++;
    }
    chain_settle_penetration = physx_clampf(
        chain->collision_radius * 0.90f, 0.010f, 0.030f);
    if (applied > 0 && correction_len <= chain_settle_penetration) {
        if (chain->addon_room_rest_frames < 30) {
            chain->addon_room_rest_frames++;
        }
        chain->addon_room_rest_sleeping = 0;

    } else {
        chain->addon_room_rest_frames = 0;
    }
    if (defaults_cfg.debug && applied > 0 &&
        (!chain->collision_response_log_tick ||
         now - chain->collision_response_log_tick >= 500u)) {
        chain->collision_response_log_tick = now;
        log_line("addon room collision chain-manifold chain=\"%s\" joints=%d axial=%.4f penetration=%.5f tangent_escape=%.5f root_reach=%.5f resting=%d note=\"terminal correction normalized across upstream joints; axial contacts fold tangentially; unilateral response without pose pinning\"",
                 chain->name, applied, axial, correction_len,
                 output_aware_applied==applied ? 0.0f : escape,
                 root_len,
                 chain->addon_room_rest_sleeping);
    }
    return applied > 0;
}

static void addon_chain_apply_collision_correction_scaled(physx_chain_t *chain,
                                                          physx_target_t *target,
                                                          const float correction[3],
                                                          float scale)
{
    float scaled[3];
    if (!chain || !target || !correction || scale <= 0.0f) return;
    scaled[0] = correction[0] * scale;
    scaled[1] = correction[1] * scale;
    scaled[2] = correction[2] * scale;
    addon_chain_apply_collision_correction(chain, target, scaled);
}

static void addon_chain_apply_body_collision_chain_correction(physx_chain_t *chain,
                                                              physx_target_t *target,
                                                              const float correction[3])
{
    int target_index;
    int source_start;
    int i;
    float total_weight = 0.0f;
    float weights[32];
    if (!chain || !target || !correction || !chain->addon_chain) return;
    target_index = (int)(target - chain->targets);
    if (target_index < 0 || target_index >= chain->target_count ||
        target_index >= 32) {
        return;
    }
    source_start = addon_chain_collision_first_target_index(chain);
    if (source_start < 0) source_start = 0;
    if (source_start > target_index) source_start = target_index;
    memset(weights, 0, sizeof(weights));
    for (i = source_start; i <= target_index && i < 32; i++) {
        int span = target_index - source_start;
        float chain_t = span > 0 ?
            (float)(i - source_start) / (float)span :
            1.0f;
        float weight = 0.45f + chain_t * 0.70f;
        if (i == target_index) weight += 0.20f;
        weights[i] = weight;
        total_weight += weight;
    }
    if (total_weight <= 0.000001f) return;
    for (i = source_start; i <= target_index && i < 32; i++) {
        float scale = (weights[i] / total_weight) * 0.55f;
        addon_chain_apply_collision_correction_scaled(
            chain, &chain->targets[i], correction, scale);
    }
}

static float addon_chain_relaxed_collision_penetration(
    physx_chain_t *chain,
    float margin,
    float penetration)
{
    float depth;
    float rest_band;
    float scale;
    if (!chain || !chain->addon_chain || penetration <= 0.0f) {
        return penetration;
    }
    depth = margin < 0.0f ? -margin : penetration;
    rest_band = physx_clampf(chain->collision_radius * 1.10f,
                             0.0060f, 0.0180f);
    if (depth <= rest_band) {
        float t = rest_band > 0.000001f ? depth / rest_band : 1.0f;
        scale = 0.18f + 0.42f * physx_clampf(t, 0.0f, 1.0f);
    } else {
        float t = physx_clampf((depth - rest_band) / rest_band,
                               0.0f, 1.0f);
        scale = 0.60f + 0.25f * t;
    }
    return penetration * scale;
}

static void addon_chain_contact_predict(physx_chain_t *chain,physx_target_t *target,
    const float offset[3],const float local[3],float point[3]);

#include "physx_chain_contact.c"

typedef struct {
    physx_contact_set_t contacts;
    float point[PHYSX_CONTACT_CAPACITY][3];
    int angular;
} addon_chain_body_manifold_t;

static void addon_chain_body_manifold_store(
    addon_chain_body_manifold_t *manifold,
    const float normal[3], float penetration, const float point[3])
{
    int slot;
    if(manifold->angular) {
        slot=physx_chain_contact_store(&manifold->contacts,manifold->point,
            normal,penetration,point);
    } else slot=physx_contact_store(&manifold->contacts, normal, penetration);
    if(slot>=0) memcpy(manifold->point[slot],point,sizeof(float)*3);
}

static void addon_chain_body_manifold_resolve(
    const addon_chain_body_manifold_t *manifold, float correction[3])
{
    physx_contact_resolve(&manifold->contacts, correction);
}

static void addon_chain_body_manifold_point_requests(
    const addon_chain_body_manifold_t *manifold,
    float weights[PHYSX_CONTACT_CAPACITY])
{
    float resolved[3],total=0.0f,budget;
    int i;
    physx_contact_resolve_weights(&manifold->contacts,resolved,weights);
    for(i=0;i<manifold->contacts.count;i++) total+=weights[i];
    /* Opposing supports have cancelling dual multipliers. Distributing them
       across distinct points must not amplify the old displacement budget. */
    budget=physx_vec3_len(resolved);
    if(total>budget && total>0.000001f)
        for(i=0;i<manifold->contacts.count;i++) weights[i]*=budget/total;
}

static float addon_chain_attachment_contact_weight(int segment_index,
    float segment_t,int proximal_ignore,int addon_chain)
{
    float boundary,t;
    if(!proximal_ignore) return 1.0f;
    if(segment_index<=1) return 0.0f;
    boundary=addon_chain_collision_proximal_deadzone(segment_index);
    if(segment_t<boundary) return 0.0f;
    if(!addon_chain) return 1.0f;
    /* Preserve the attachment exclusion, but do not introduce a full
       penetration push as the nearest point crosses its boundary. */
    t=physx_clampf((segment_t-boundary)/0.04f,0.0f,1.0f);
    return t*t*(3.0f-2.0f*t);
}

static int addon_chain_accumulate_collision_contact(
    physx_chain_t *chain,
    physx_target_t *target,
    int segment_index,
    float segment_t,
    float margin,
    int proximal_ignore,
    const float closest_chain[3],
    const float closest_body[3],
    const float previous_start[3],
    const float previous_end[3],
    const float body_a[3],
    const float body_b[3],
    addon_chain_body_manifold_t *manifold,
    float correction[3],
    float *max_penetration,
    float *applied_penetration_out)
{
    float normal[3];
    float normal_len;
    float penetration;
    float soft_margin = 0.0020f;
    float collision_slop;
    float attachment_weight;
    float response_strength;
    float weight;
    int axis;
    if (!chain || !target || !closest_chain || !closest_body ||
        !correction || !max_penetration) {
        return 0;
    }
    if (applied_penetration_out) *applied_penetration_out = 0.0f;
    if (margin >= soft_margin) return 0;
    attachment_weight=addon_chain_attachment_contact_weight(segment_index,
        segment_t,proximal_ignore,chain->addon_chain);
    if(attachment_weight<=0.0f) return 0;
    collision_slop = body_chain_collider_cfg.collision_slop;
    if (collision_slop < 0.0f) collision_slop = 0.0f;
    penetration = physx_contact_depth(margin, collision_slop, soft_margin);
    penetration = physx_clampf(
        penetration, 0.0f, chain->addon_chain ? 0.024f : 0.100f);
    penetration =
        addon_chain_relaxed_collision_penetration(chain, margin, penetration);
    if (penetration <= 0.000001f) return 0;
    normal[0] = closest_chain[0] - closest_body[0];
    normal[1] = closest_chain[1] - closest_body[1];
    normal[2] = closest_chain[2] - closest_body[2];
    normal_len = physx_vec3_len(normal);
    if (normal_len <= 0.000001f) {
        normal[0] = target->sim_offset[0];
        normal[1] = target->sim_offset[1];
        normal[2] = target->sim_offset[2];
        normal_len = physx_vec3_len(normal);
    }
    if (normal_len <= 0.000001f) return 0;
    normal[0] /= normal_len;
    normal[1] /= normal_len;
    normal[2] /= normal_len;
    if (previous_start && previous_end && body_a && body_b) {
        addon_chain_collision_apply_previous_side(
            normal, previous_start, previous_end, segment_t,
            closest_body, body_a, body_b);
    }
    weight = physx_clampf(0.75f + segment_t * 0.25f, 0.0f, 1.0f);
    response_strength =
        physx_clampf(body_chain_collider_cfg.response_strength, 0.0f, 4.0f);
    if (chain->addon_chain && response_strength > 0.82f) {
        response_strength = 0.82f;
    }
    penetration *= weight * response_strength * attachment_weight;
    if (penetration <= 0.000001f) return 0;
    if (applied_penetration_out) *applied_penetration_out = penetration;
    if (manifold) {
        addon_chain_body_manifold_store(manifold, normal, penetration, closest_chain);
    } else {
        for (axis = 0; axis < 3; axis++) {
            correction[axis] += normal[axis] * penetration;
        }
    }
    if (penetration > *max_penetration) {
        *max_penetration = penetration;
    }
    return 1;
}

static float addon_chain_body_collision_radius_scale(void)
{
    float scale = body_chain_collider_cfg.response_radius_scale;
    return physx_clampf(scale, 0.05f, 4.0f);
}

static float addon_chain_body_collision_response_sweep_radius(
    physx_chain_t *chain,
    float sweep_radius)
{
    float cap;
    if (!chain || !chain->addon_chain || sweep_radius <= 0.0f) {
        return sweep_radius;
    }
    cap = chain->collision_radius * 0.40f;
    cap = physx_clampf(cap, 0.0030f, 0.0080f);
    return sweep_radius < cap ? sweep_radius : cap;
}

static float addon_chain_body_collision_response_margin(
    physx_chain_t *chain,
    float margin,
    float sweep_radius,
    float response_sweep_radius)
{
    float response_margin;
    if (!chain || !chain->addon_chain || sweep_radius <= response_sweep_radius) {
        response_margin = margin;
    } else {
        response_margin = margin + (sweep_radius - response_sweep_radius);
    }
    /* Keep the shallow contact band continuous. The contact-depth function
       already handles slop; snapping this margin to 0.002 turned the whole
       response off immediately above -slop. */
    return response_margin;
}

typedef struct addon_chain_collision_trace_t {
    int has_candidate;
    int hit;
    char collider[128];
    float margin;
    float dist;
    float radius;
    float chain_t;
    float body_t;
    float applied_penetration;
} addon_chain_collision_trace_t;

static int addon_chain_body_contact_parent_attachment(int node)
{
    return node == BODY_COLLIDER_HEAD_02 ||
           node == BODY_COLLIDER_NECK_01;
}

static int addon_chain_point_capsule_margin(
    const float point[3],
    const float pair_a[3],
    const float pair_b[3],
    float radius,
    float *pair_t,
    float closest_body[3],
    float *dist,
    float *margin)
{
    float ab[3];
    float ap[3];
    float len_sq;
    float t = 0.0f;
    float delta[3];
    if (!point || !pair_a || !pair_b || !closest_body ||
        !dist || !margin) {
        return 0;
    }
    ab[0] = pair_b[0] - pair_a[0];
    ab[1] = pair_b[1] - pair_a[1];
    ab[2] = pair_b[2] - pair_a[2];
    ap[0] = point[0] - pair_a[0];
    ap[1] = point[1] - pair_a[1];
    ap[2] = point[2] - pair_a[2];
    len_sq = ab[0] * ab[0] + ab[1] * ab[1] + ab[2] * ab[2];
    if (len_sq > 0.000001f) {
        t = physx_clampf(
            (ap[0] * ab[0] + ap[1] * ab[1] + ap[2] * ab[2]) / len_sq,
            0.0f, 1.0f);
    }
    closest_body[0] = pair_a[0] + ab[0] * t;
    closest_body[1] = pair_a[1] + ab[1] * t;
    closest_body[2] = pair_a[2] + ab[2] * t;
    delta[0] = point[0] - closest_body[0];
    delta[1] = point[1] - closest_body[1];
    delta[2] = point[2] - closest_body[2];
    *dist = physx_vec3_len(delta);
    *margin = *dist - radius;
    if (pair_t) *pair_t = t;
    return 1;
}

static void addon_chain_collision_trace_consider(
    addon_chain_collision_trace_t *trace,
    const char *collider,
    float margin,
    float dist,
    float radius,
    float chain_t,
    float body_t,
    int hit,
    float applied_penetration)
{
    int replace = 0;
    if (!trace || !collider || !collider[0]) return;
    if (!trace->has_candidate) {
        replace = 1;
    } else if (hit) {
        replace = !trace->hit ||
            applied_penetration > trace->applied_penetration;
    } else if (!trace->hit && margin < trace->margin) {
        replace = 1;
    }
    if (!replace) return;
    trace->has_candidate = 1;
    trace->hit = hit ? 1 : 0;
    lstrcpynA(trace->collider, collider, sizeof(trace->collider));
    trace->margin = margin;
    trace->dist = dist;
    trace->radius = radius;
    trace->chain_t = chain_t;
    trace->body_t = body_t;
    trace->applied_penetration = applied_penetration;
}

typedef struct addon_body_node_frame_cache_t {
    DWORD tick;
    int ready;
    unsigned char valid[BODY_COLLIDER_NODE_COUNT];
    float point[BODY_COLLIDER_NODE_COUNT][3];
} addon_body_node_frame_cache_t;

static addon_body_node_frame_cache_t
    addon_body_node_frame_cache[4][4];

static int addon_chain_body_node_in_frame_cached(
    int chain_frame_person_index,
    int collider_person_index,
    int node,
    DWORD now,
    float out[3])
{
    addon_body_node_frame_cache_t *cache;
    body_chain_collider_person_state_t *chain_frame;
    body_chain_collider_person_state_t *state;
    int i;
    if (!out ||
        chain_frame_person_index < 0 || chain_frame_person_index >= 4 ||
        collider_person_index < 0 || collider_person_index >= 4 ||
        node < 0 || node >= BODY_COLLIDER_NODE_COUNT) {
        return 0;
    }
    cache =
        &addon_body_node_frame_cache
            [chain_frame_person_index][collider_person_index];
    chain_frame = &body_chain_collider_states[chain_frame_person_index];
    state = &body_chain_collider_states[collider_person_index];
    if (!cache->ready || cache->tick != now) {
        cache->tick = now;
        cache->ready = 1;
        memset(cache->valid, 0, sizeof(cache->valid));
        for (i = 0; i < BODY_COLLIDER_NODE_COUNT; i++) {
            if (!state->valid[i]) continue;
            if (body_chain_collider_node_in_chain_space(
                    chain_frame, state, i, cache->point[i])) {
                cache->valid[i] = 1;
            }
        }
    }
    if (!cache->valid[node]) return 0;
    out[0] = cache->point[node][0];
    out[1] = cache->point[node][1];
    out[2] = cache->point[node][2];
    return 1;
}

/* The visible endpoint is a joint pivot. Only preceding joints can move it.
   Map its body-space contact displacement into each upstream solver frame;
   adding body-space vectors directly to those offsets mixes coordinate frames. */
static int addon_chain_solve_body_contact(physx_chain_t *chain,
    physx_target_t *target,const float pivot[3],const float end[3],
    const float request[3],float delta[3],float achieved[3],float gradient[3]);

static void addon_chain_contact_point_delta(physx_chain_t *chain,
    physx_target_t *target,const float pivot[3],const float point[3],
    const float delta[3],float movement[3]);

static int addon_chain_apply_visible_body_correction(
    physx_sidecar_t *sc, physx_chain_t *chain, physx_target_t *target,
    int person, const float end[3], const float correction[3], float accepted[3],
    const float sample_point[3], float sample_movement[3], int *sample_joints,
    const float response_point[3])
{
    float pivots[32][3], weights[32], total = 0.0f;
    int i, axis, first, last, applied = 0;
    body_chain_collider_person_state_t *state;
    memset(accepted, 0, sizeof(float)*3);
    if(sample_movement) memset(sample_movement,0,sizeof(float)*3);
    if(sample_joints) *sample_joints=0;
    if (!sc || !chain || !target || person < 0 || person >= 4) return 0;
    first = addon_chain_collision_first_target_index(chain);
    if (first < 1) first = 1;
    last = (int)(target-chain->targets)-1;
    if (last < first || last >= 32) return 0;
    state = &body_chain_collider_states[person];
    memset(weights,0,sizeof(weights));
    for (i=first;i<=last;i++) {
        physx_target_t *joint=&chain->targets[i];
        if (!joint->addon_simulated_target || joint->sim_length<=0.0001f ||
            !addon_chain_target_body_local(sc,chain,joint,person,state,pivots[i])) continue;
        weights[i]=1.0f;
        total+=weights[i];
    }
    if (total<=0.0f) return 0;
    for (i=first;i<=last;i++) {
        float delta[3], normal[3], achieved[3], request[3], length;
        physx_target_t *joint=&chain->targets[i];
        if (!weights[i]) continue;
        for(axis=0;axis<3;axis++) request[axis]=correction[axis]*0.55f/total;
        if (joint->contact_basis_tick) {
            if (!addon_chain_solve_body_contact(chain,joint,pivots[i],
                    response_point ? response_point : end,request,
                    delta,achieved,normal)) continue;
            /* accepted describes endpoint movement for subsequent queries,
               even when the constrained material point is inside the link. */
            if(response_point)
                addon_chain_contact_point_delta(chain,joint,pivots[i],end,delta,achieved);
        } else {
            /* Binding/startup may not yet expose a valid evaluated basis.
               Retain the previous mapping rather than dropping collisions. */
            if (!addon_chain_visible_room_correction_to_sim_delta(
                    joint,pivots[i],end,request,delta)) continue;
            length=physx_vec3_len(delta);
            if(length<=0.000001f) continue;
            for(axis=0;axis<3;axis++) { achieved[axis]=request[axis]; normal[axis]=delta[axis]/length; }
        }
        length=physx_vec3_len(delta);
        if (length<=0.000001f) continue;
        if(sample_point && sample_movement && joint->contact_basis_tick) {
            float movement[3];
            addon_chain_contact_point_delta(chain,joint,pivots[i],sample_point,delta,movement);
            for(axis=0;axis<3;axis++) sample_movement[axis]+=movement[axis];
            if(sample_joints) (*sample_joints)++;
        }
        for(axis=0;axis<3;axis++) {
            joint->sim_offset[axis]+=delta[axis];
            accepted[axis]+=achieved[axis];
        }
        joint->sim_contact_corrected=1;
        addon_chain_collision_normalize_target(joint);
        physx_contact_link_velocity(joint->sim_velocity,joint->sim_offset,normal);
        applied++;
    }
    /* Prediction uses the actual output-angle model, not the requested push. */
    return applied>0;
}

static int addon_chain_apply_body_collision(
    physx_sidecar_t *sc,
    physx_chain_t *chain,
    physx_target_t *target,
    int chain_frame_person_index,
    int collider_person_index,
    const float start[3],
    float end_override[3],
    int segment_index,
    DWORD now,
    float dt,
    int terminal_segment)
{
    body_chain_collider_person_state_t *chain_frame;
    body_chain_collider_person_state_t *state;
    float end[3];
    float previous_start[3];
    float previous_end[3];
    float correction[3] = { 0.0f, 0.0f, 0.0f };
    float max_penetration = 0.0f;
    float body_radius_scale;
    float sweep_radius = 0.0f;
    float response_sweep_radius = 0.0f;
    int previous_segment_valid = 0;
    int end_override_valid = 0;
    int coherent = 0;
    physx_chain_contact_pose_t contact_pose;
    float predicted_start[3];
    int stable_response = 0;
    int normal_hysteresis = 0;
    addon_chain_body_manifold_t manifold;
    addon_chain_collision_trace_t trace;
    int hit_count = 0;
    int trace_enabled;
    int node;
    int j;
    if (!chain || !target || !start ||
        chain_frame_person_index < 0 || chain_frame_person_index >= 4 ||
        collider_person_index < 0 || collider_person_index >= 4 ||
        !(chain->collision_scope &
          (PHYSX_COLLISION_SCOPE_BODY | PHYSX_COLLISION_SCOPE_BODY_ALL)) ||
        chain->collision_radius <= 0.0f) {
        return 0;
    }
    chain_frame = &body_chain_collider_states[chain_frame_person_index];
    state = &body_chain_collider_states[collider_person_index];
    if (!chain_frame->basis_valid ||
        !chain_frame->valid[BODY_COLLIDER_ROOT]) {
        return 0;
    }
    if (!state->basis_valid ||
        !state->valid[BODY_COLLIDER_ROOT] ||
        state->scene_liveness_engine_invisible ||
        state->scene_liveness_quarantined) {
        return 0;
    }
    body_profile_set_active_person_config(collider_person_index);
    body_radius_scale = addon_chain_body_collision_radius_scale();
    memset(&manifold, 0, sizeof(manifold));
    trace_enabled = chain->collision_debug_draw || defaults_cfg.debug;
    if (trace_enabled) memset(&trace, 0, sizeof(trace));
    if (end_override &&
        physx_vec3_sane_limit(end_override, 25.0f)) {
        end[0] = end_override[0];
        end[1] = end_override[1];
        end[2] = end_override[2];
        end_override_valid = 1;
    } else {
        end[0] = start[0] + target->sim_offset[0];
        end[1] = start[1] + target->sim_offset[1];
        end[2] = start[2] + target->sim_offset[2];
    }
    coherent=chain->addon_chain && end_override_valid &&
        physx_chain_contact_pose_init(chain,(int)(target-chain->targets)-(terminal_segment?0:1),now,&contact_pose);
    /* The virtual tip has no child pivot or safe legacy endpoint mapping. */
    if(terminal_segment && !coherent) {
        body_profile_set_active_person_config(-1);
        return 0;
    }
    if(coherent) {
        physx_chain_contact_point(chain,&contact_pose,0.0f,predicted_start);
        physx_chain_contact_point(chain,&contact_pose,1.0f,end);
        start=predicted_start;
        manifold.angular=1;
    }
    previous_segment_valid = !terminal_segment &&
        addon_chain_collision_previous_segment(
            chain, target, start, end,
            previous_start, previous_end, &sweep_radius);
    stable_response =
        addon_chain_previous_contact_stable(
            chain, target, now, sweep_radius);
    if (!stable_response) {
        response_sweep_radius =
            addon_chain_body_collision_response_sweep_radius(
                chain, sweep_radius);
    }

    for (node = 0; node < BODY_COLLIDER_NODE_COUNT; node++) {
        float node_chain_t;
        float node_pair_t;
        float closest_chain[3];
        float closest_body[3];
        float dist;
        float margin;
        float response_margin;
        float chain_distance;
        float radius;
        float applied_penetration = 0.0f;
        float node_pos[3];
        int added_hits;
        if (!state->valid[node] ||
            !addon_chain_custom_body_node_enabled(chain, node)) {
            continue;
        }
        if (!addon_chain_body_node_in_frame_cached(
                chain_frame_person_index, collider_person_index,
                node, now, node_pos)) {
            continue;
        }
        radius =
            body_chain_collider_radius_for_node(node) * body_radius_scale +
            chain->collision_radius + sweep_radius;
        if (!body_chain_collider_pair_margin(
                start, end, 0.0f, 0.0f,
                node_pos,
                node_pos,
                radius, &node_chain_t, &node_pair_t,
                closest_chain, closest_body, &dist, &margin,
                &chain_distance)) {
            continue;
        }
        response_margin =
            addon_chain_body_collision_response_margin(
                chain, margin, sweep_radius, response_sweep_radius);
        added_hits = addon_chain_accumulate_collision_contact(
            chain, target, segment_index, node_chain_t, response_margin,
            addon_chain_body_contact_parent_attachment(node),
            closest_chain, closest_body,
            previous_segment_valid ? previous_start : NULL,
            previous_segment_valid ? previous_end : NULL,
            node_pos,
            node_pos,
            &manifold,
            correction, &max_penetration,
            &applied_penetration);
        hit_count += added_hits;
        if (trace_enabled) {
            addon_chain_collision_trace_consider(
                &trace, body_chain_collider_node_label(node), margin,
                dist, radius, node_chain_t, node_pair_t, added_hits > 0,
                applied_penetration);
        }
    }

    if (addon_chain_custom_body_edge_enabled(
            chain, BODY_COLLIDER_STOMACH_01,
            BODY_COLLIDER_STOMACH_02) &&
        state->stomach_points_ready &&
        state->valid[BODY_COLLIDER_STOMACH_01] &&
        state->valid[BODY_COLLIDER_STOMACH_02]) {
        float stomach_chain_t;
        float stomach_t;
        float closest_chain[3];
        float closest_body[3];
        float dist;
        float radius;
        float margin;
        float response_margin;
        float chain_distance;
        float stomach_a[3];
        float stomach_b[3];
        float applied_penetration = 0.0f;
        int added_hits;
        if (body_chain_collider_stomach_pair_margin_in_frame(
                start, end, 0.0f, 0.0f, chain_frame, state,
                &stomach_chain_t, &stomach_t,
                closest_chain, closest_body, &dist, &radius, &margin,
                &chain_distance, stomach_a, stomach_b)) {
            radius = (radius - body_chain_collider_cfg.chain_radius) *
                     (body_radius_scale /
                      physx_clampf(body_chain_collider_cfg.response_radius_scale,
                                   0.05f, 4.0f)) +
                     chain->collision_radius + sweep_radius;
            margin = dist - radius;
            response_margin =
                addon_chain_body_collision_response_margin(
                    chain, margin, sweep_radius, response_sweep_radius);
            added_hits = addon_chain_accumulate_collision_contact(
                chain, target, segment_index, stomach_chain_t,
                response_margin, 0,
                closest_chain, closest_body,
                previous_segment_valid ? previous_start : NULL,
                previous_segment_valid ? previous_end : NULL,
                stomach_a, stomach_b,
                &manifold,
                correction, &max_penetration,
                &applied_penetration);
            hit_count += added_hits;
            if (trace_enabled) {
                addon_chain_collision_trace_consider(
                    &trace, "spine_pair", margin, dist, radius,
                    stomach_chain_t, stomach_t, added_hits > 0,
                    applied_penetration);
            }
        }
    }

    for (j = 0; j < BODY_COLLIDER_LIMB_PAIR_COUNT; j++) {
        float limb_chain_t;
        float limb_t;
        float closest_chain[3];
        float closest_body[3];
        float dist;
        float radius;
        float margin;
        float response_margin;
        float chain_distance;
        float limb_a[3];
        float limb_b[3];
        float applied_penetration = 0.0f;
        int added_hits;
        int limb_start_node = -1;
        int limb_end_node = -1;
        body_chain_collider_limb_pair_nodes(
            j, &limb_start_node, &limb_end_node);
        if (!addon_chain_custom_body_edge_enabled(
                chain, limb_start_node, limb_end_node)) {
            continue;
        }
        if (!body_chain_collider_limb_pair_margin_in_frame(
                start, end, 0.0f, 0.0f, chain_frame, state, j,
                &limb_chain_t, &limb_t,
                closest_chain, closest_body, &dist,
                &radius, &margin, &chain_distance,
                limb_a, limb_b)) {
            continue;
        }
        radius = (radius - body_chain_collider_cfg.chain_radius) *
                 (body_radius_scale /
                  physx_clampf(body_chain_collider_cfg.response_radius_scale,
                               0.05f, 4.0f)) +
                 chain->collision_radius + sweep_radius;
        margin = dist - radius;
        response_margin =
            addon_chain_body_collision_response_margin(
                chain, margin, sweep_radius, response_sweep_radius);
        added_hits = addon_chain_accumulate_collision_contact(
            chain, target, segment_index, limb_chain_t, response_margin, 0,
            closest_chain, closest_body,
            previous_segment_valid ? previous_start : NULL,
            previous_segment_valid ? previous_end : NULL,
            limb_a, limb_b,
            &manifold,
            correction, &max_penetration,
            &applied_penetration);
        hit_count += added_hits;
        if (trace_enabled) {
            addon_chain_collision_trace_consider(
                &trace, "limb_pair", margin, dist, radius,
                limb_chain_t, limb_t, added_hits > 0,
                applied_penetration);
        }
    }

    for (j = 0; j < BODY_COLLIDER_EXTRA_EDGE_COUNT; j++) {
        const body_collider_extra_edge_def_t *edge = &body_collider_extra_edges[j];
        float edge_chain_t;
        float edge_t;
        float closest_chain[3];
        float closest_body[3];
        float dist;
        float margin;
        float response_margin;
        float chain_distance;
        float r0;
        float r1;
        float radius;
        float applied_penetration = 0.0f;
        float edge_a[3];
        float edge_b[3];
        int added_hits;
        if (!edge ||
            edge->start_node < 0 ||
            edge->start_node >= BODY_COLLIDER_NODE_COUNT ||
            edge->end_node < 0 ||
            edge->end_node >= BODY_COLLIDER_NODE_COUNT ||
            !state->valid[edge->start_node] ||
            !state->valid[edge->end_node]) {
            continue;
        }
        if (!addon_chain_custom_body_edge_enabled(
                chain, edge->start_node, edge->end_node)) {
            continue;
        }
        if (!addon_chain_body_node_in_frame_cached(
                chain_frame_person_index, collider_person_index,
                edge->start_node, now, edge_a) ||
            !addon_chain_body_node_in_frame_cached(
                chain_frame_person_index, collider_person_index,
                edge->end_node, now, edge_b)) {
            continue;
        }
        if (!body_chain_collider_pair_margin(
                start, end, 0.0f, 0.0f,
                edge_a,
                edge_b,
                0.0f, &edge_chain_t, &edge_t,
                closest_chain, closest_body, &dist, &margin,
                &chain_distance)) {
            continue;
        }
        r0 = body_chain_collider_radius_for_node(edge->start_node) *
             body_radius_scale;
        r1 = body_chain_collider_radius_for_node(edge->end_node) *
             body_radius_scale;
        radius = r0 + (r1 - r0) * physx_clampf(edge_t, 0.0f, 1.0f);
        radius += chain->collision_radius + sweep_radius;
        margin = dist - radius;
        response_margin =
            addon_chain_body_collision_response_margin(
                chain, margin, sweep_radius, response_sweep_radius);
        added_hits = addon_chain_accumulate_collision_contact(
            chain, target, segment_index, edge_chain_t, response_margin,
            addon_chain_body_contact_parent_attachment(edge->start_node) &&
            addon_chain_body_contact_parent_attachment(edge->end_node),
            closest_chain, closest_body,
            previous_segment_valid ? previous_start : NULL,
            previous_segment_valid ? previous_end : NULL,
            edge_a,
            edge_b,
            &manifold,
            correction, &max_penetration,
            &applied_penetration);
        hit_count += added_hits;
        if (trace_enabled) {
            addon_chain_collision_trace_consider(
                &trace, edge->name, margin, dist, radius,
                edge_chain_t, edge_t, added_hits > 0,
                applied_penetration);
        }
    }

    /* Compare the FIRST live query this frame with last frame's request.
       Later iterations use a predicted end_override and must not be treated
       as observations. Remove sweep inflation from the reported gap. */
    if (!coherent && trace_enabled && trace.has_candidate && end_override_valid &&
        chain_frame_person_index == collider_person_index &&
        target->contact_response_tick != now) {
        LONG generation = InterlockedCompareExchange(&named_node_generation,0,0);
        float margin = trace.margin + sweep_radius;
        float requested = physx_vec3_len(target->contact_response_request);
        if (target->contact_response_tick &&
            now-target->contact_response_tick<=120u &&
            target->contact_response_generation==generation && requested>0.000001f &&
            (!target->contact_response_log_tick || now-target->contact_response_log_tick>=250u)) {
            float motion[3], normal[3], outward, lateral[3];
            int axis;
            for(axis=0;axis<3;axis++) {
                motion[axis]=end[axis]-target->contact_response_pivot[axis];
                normal[axis]=target->contact_response_request[axis]/requested;
            }
            outward=vec3_dot(motion,normal);
            for(axis=0;axis<3;axis++) lateral[axis]=motion[axis]-outward*normal[axis];
            log_line("addon contact effectiveness chain=\"%s\" target=\"%s\" dt_ms=%lu requested=%.7f observed_outward=%.7f observed_lateral=%.7f previous_gap=%.7f current_gap=%.7f gap_change=%.7f same_feature=%d previous_feature=\"%s\" current_feature=\"%s\" note=\"first live query; gap excludes sweep inflation; animation and changing closest points also affect observation\"",
                chain->name,target->name,(unsigned long)(now-target->contact_response_tick),
                requested,outward,physx_vec3_len(lateral),target->contact_response_margin,
                margin,margin-target->contact_response_margin,
                strcmp(target->contact_response_feature,trace.collider)==0,
                target->contact_response_feature,trace.collider);
            target->contact_response_log_tick=now;
        }
        target->contact_response_tick=now;
        target->contact_response_generation=generation;
        target->contact_response_margin=margin;
        memcpy(target->contact_response_pivot,end,sizeof(end));
        memset(target->contact_response_request,0,sizeof(target->contact_response_request));
        lstrcpynA(target->contact_response_feature,trace.collider,sizeof(target->contact_response_feature));
    }
    addon_chain_body_manifold_resolve(&manifold, correction);
    if (hit_count > 0 && manifold.contacts.count > 0) {
        float stable_normal[3];
        float accepted[3];
        int visible_response = chain->addon_chain && end_override_valid;
        int stable_contact = addon_chain_stable_collision_normal(
            chain, target, now, correction, stable_normal);
        addon_chain_note_direct_collision_contact(
            chain, target, now, correction);
        (void)dt;
        memcpy(accepted,correction,sizeof(accepted));
        if (coherent) {
            int moved=physx_chain_contact_solve(chain,&contact_pose,&manifold.contacts,
                manifold.point,start,end);
            physx_chain_contact_point(chain,&contact_pose,1.0f,accepted);
            for(j=0;j<3;j++) accepted[j]-=end[j];
            if(!terminal_segment && trace_enabled && (!target->collision_response_log_tick ||
                now-target->collision_response_log_tick>=250u))
                log_line("addon coherent body contact chain=\"%s\" target=\"%s\" supports=%d moved_joints=%d endpoint_change=(%.7f,%.7f,%.7f)",
                    chain->name,target->name,manifold.contacts.count,moved,
                    accepted[0],accepted[1],accepted[2]);
        } else if (visible_response) {
            float sample[3], movement[3];
            int axis, sample_joints=0;
            int probe=trace_enabled && trace.hit &&
                (!target->collision_response_log_tick ||
                 now-target->collision_response_log_tick>=250u);
            for(axis=0;axis<3;axis++)
                sample[axis]=start[axis]+trace.chain_t*(end[axis]-start[axis]);
            {
                float lambda[PHYSX_CONTACT_CAPACITY];
                int support;
                memset(accepted,0,sizeof(accepted));
                memset(movement,0,sizeof(movement));
                addon_chain_body_manifold_point_requests(&manifold,lambda);
                for(support=0;support<manifold.contacts.count;support++) {
                    float request[3],endpoint_delta[3],sample_delta[3];
                    int joints=0;
                    if(lambda[support]<=0.000001f) continue;
                    for(axis=0;axis<3;axis++)
                        request[axis]=manifold.contacts.normal[support][axis]*lambda[support];
                    addon_chain_apply_visible_body_correction(sc,chain,target,
                        chain_frame_person_index,end,request,endpoint_delta,
                        probe ? sample : NULL,probe ? sample_delta : NULL,&joints,
                        manifold.point[support]);
                    for(axis=0;axis<3;axis++) {
                        accepted[axis]+=endpoint_delta[axis];
                        if(probe) movement[axis]+=sample_delta[axis];
                    }
                    sample_joints+=joints;
                }
            }
            if(probe) log_line("addon contact point probe chain=\"%s\" target=\"%s\" nearest=\"%s\" chain_t=%.4f supports=%d sampled_joints=%d applied=%d endpoint_prediction=(%.7f,%.7f,%.7f) contact_prediction=(%.7f,%.7f,%.7f) note=\"independent-joint prediction; nearest active contact; not measured engine motion\"",
                chain->name,target->name,trace.collider,trace.chain_t,manifold.contacts.count,
                sample_joints,target->sim_contact_corrected,
                accepted[0],accepted[1],accepted[2],movement[0],movement[1],movement[2]);
        } else if (chain->addon_chain) {
            addon_chain_apply_body_collision_chain_correction(
                chain, target, correction);
        } else {
            addon_chain_apply_collision_correction(chain, target, correction);
        }
        if (stable_contact && !visible_response) {
            addon_chain_damp_stable_collision_velocity(
                chain, target, stable_normal);
        }
        if (end_override_valid) {
            if (trace_enabled && chain_frame_person_index==collider_person_index &&
                target->contact_response_tick==now) {
                int axis;
                for(axis=0;axis<3;axis++) target->contact_response_request[axis]+=accepted[axis];
            }
            end_override[0] = end[0] + accepted[0];
            end_override[1] = end[1] + accepted[1];
            end_override[2] = end[2] + accepted[2];
        }
    }
    if(coherent && end_override_valid && hit_count==0)
        memcpy(end_override,end,sizeof(end));
    if(terminal_segment && trace_enabled && (!target->contact_terminal_log_tick ||
        now-target->contact_terminal_log_tick>=500u)) {
        target->contact_terminal_log_tick=now;
        log_line("addon terminal body contact chain=\"%s\" target=\"%s\" person_index=%d hits=%d supports=%d nearest=\"%s\" margin=%.7f chain_t=%.4f start=(%.6f,%.6f,%.6f) end=(%.6f,%.6f,%.6f)",
            chain->name,target->name,collider_person_index+1,hit_count,manifold.contacts.count,
            trace.has_candidate ? trace.collider : "none",trace.has_candidate ? trace.margin : 0.0f,
            trace.has_candidate ? trace.chain_t : 0.0f,start[0],start[1],start[2],end[0],end[1],end[2]);
    }
    if (!terminal_segment && trace_enabled &&
        trace.has_candidate &&
        (!target->collision_response_log_tick ||
         now - target->collision_response_log_tick >=
             (hit_count > 0 ? 250u : 1000u))) {
        float correction_len = physx_vec3_len(correction);
        float raw_penetration = trace.margin < 0.0f ? -trace.margin : 0.0f;
        float log_deadzone =
            addon_chain_collision_proximal_deadzone(segment_index);
        int base_contact_ignored =
            !trace.hit && trace.margin < 0.0f &&
            trace.chain_t < log_deadzone;
        target->collision_response_log_tick = now;
        log_line("addon sidecar collision contact chain=\"%s\" target=\"%s\" person_index=%d segment=%d hits=%d supports=%d nearest=\"%s\" hit=%d base_contact_ignored=%d margin=%.5f raw_penetration=%.5f applied_penetration=%.5f correction_len=%.5f correction=(%.5f,%.5f,%.5f) dist=%.5f radius=%.5f chain_t=%.3f body_t=%.3f body_radius_scale=%.3f sidecar_radius=%.4f sweep_radius=%.5f response_sweep=%.5f stable_response=%d normal_hysteresis=%d previous_side=%d end_override=%d scope=0x%x note=\"enable collision_debug_draw on this sidecar chain to trace real body contacts without global debug spam\"",
                 chain->name,
                 target->name,
                 collider_person_index + 1,
                 segment_index,
                 hit_count,
                 manifold.contacts.count,
                 trace.collider,
                 trace.hit,
                 base_contact_ignored,
                 trace.margin,
                 raw_penetration,
                 trace.applied_penetration,
                 correction_len,
                 correction[0], correction[1], correction[2],
                 trace.dist,
                 trace.radius,
                 trace.chain_t,
                 trace.body_t,
                 body_radius_scale,
                 chain->collision_radius,
                 sweep_radius,
                 response_sweep_radius,
                 stable_response,
                 normal_hysteresis,
                 previous_segment_valid,
                 end_override_valid,
                 chain->collision_scope);
    }
    body_profile_set_active_person_config(-1);
    return hit_count > 0;
}

static int addon_chain_apply_body_point_collision(
    physx_chain_t *chain,
    physx_target_t *target,
    int chain_frame_person_index,
    int collider_person_index,
    float point[3],
    DWORD now,
    float dt)
{
    body_chain_collider_person_state_t *chain_frame;
    body_chain_collider_person_state_t *state;
    float correction[3] = { 0.0f, 0.0f, 0.0f };
    float max_penetration = 0.0f;
    float body_radius_scale;
    addon_chain_body_manifold_t manifold;
    addon_chain_collision_trace_t trace;
    int hit_count = 0;
    int trace_enabled;
    int node;
    int j;
    if (!chain || !target || !point ||
        chain_frame_person_index < 0 || chain_frame_person_index >= 4 ||
        collider_person_index < 0 || collider_person_index >= 4 ||
        !(chain->collision_scope &
          (PHYSX_COLLISION_SCOPE_BODY | PHYSX_COLLISION_SCOPE_BODY_ALL)) ||
        chain->collision_radius <= 0.0f) {
        return 0;
    }
    chain_frame = &body_chain_collider_states[chain_frame_person_index];
    state = &body_chain_collider_states[collider_person_index];
    if (!chain_frame->basis_valid ||
        !chain_frame->valid[BODY_COLLIDER_ROOT]) {
        return 0;
    }
    if (!state->basis_valid ||
        !state->valid[BODY_COLLIDER_ROOT] ||
        state->scene_liveness_engine_invisible ||
        state->scene_liveness_quarantined) {
        return 0;
    }
    body_profile_set_active_person_config(collider_person_index);
    body_radius_scale = addon_chain_body_collision_radius_scale();
    memset(&manifold, 0, sizeof(manifold));
    trace_enabled = chain->collision_debug_draw || defaults_cfg.debug;
    if (trace_enabled) memset(&trace, 0, sizeof(trace));

    for (node = 0; node < BODY_COLLIDER_NODE_COUNT; node++) {
        float closest_body[3];
        float dist;
        float margin;
        float radius;
        float applied_penetration = 0.0f;
        int added_hits;
        float node_pos[3];
        if (!state->valid[node] ||
            !addon_chain_custom_body_node_enabled(chain, node) ||
            addon_chain_body_contact_parent_attachment(node)) {
            continue;
        }
        if (!addon_chain_body_node_in_frame_cached(
                chain_frame_person_index, collider_person_index,
                node, now, node_pos)) {
            continue;
        }
        radius =
            body_chain_collider_radius_for_node(node) * body_radius_scale +
            chain->collision_radius;
        closest_body[0] = node_pos[0];
        closest_body[1] = node_pos[1];
        closest_body[2] = node_pos[2];
        dist = body_collider_distance(point, closest_body);
        margin = dist - radius;
        added_hits = addon_chain_accumulate_collision_contact(
            chain, target, 1, 1.0f, margin, 0,
            point, closest_body, NULL, NULL, NULL, NULL,
            &manifold,
            correction, &max_penetration,
            &applied_penetration);
        hit_count += added_hits;
        if (trace_enabled) {
            addon_chain_collision_trace_consider(
                &trace, body_chain_collider_node_label(node), margin,
                dist, radius, 1.0f, 0.0f, added_hits > 0,
                applied_penetration);
        }
    }

    if (addon_chain_custom_body_edge_enabled(
            chain, BODY_COLLIDER_STOMACH_01,
            BODY_COLLIDER_STOMACH_02) &&
        state->stomach_points_ready &&
        state->valid[BODY_COLLIDER_STOMACH_01] &&
        state->valid[BODY_COLLIDER_STOMACH_02]) {
        float stomach_t;
        float closest_body[3];
        float dist;
        float radius;
        float margin;
        float radius0[3];
        float radius1[3];
        float radius_at[3];
        float normal[3];
        float normal_len;
        float applied_penetration = 0.0f;
        float stomach_a[3];
        float stomach_b[3];
        int added_hits;
        int axis;
        if (addon_chain_body_node_in_frame_cached(
                chain_frame_person_index, collider_person_index,
                BODY_COLLIDER_STOMACH_01, now, stomach_a) &&
            addon_chain_body_node_in_frame_cached(
                chain_frame_person_index, collider_person_index,
                BODY_COLLIDER_STOMACH_02, now, stomach_b)) {
        if (addon_chain_point_capsule_margin(
                point,
                stomach_a,
                stomach_b,
                0.0f, &stomach_t, closest_body, &dist, &margin)) {
            body_chain_collider_effective_stomach_radius_axes(0, radius0);
            body_chain_collider_effective_stomach_radius_axes(1, radius1);
            stomach_t = physx_clampf(stomach_t, 0.0f, 1.0f);
            for (axis = 0; axis < 3; axis++) {
                radius_at[axis] =
                    radius0[axis] + (radius1[axis] - radius0[axis]) *
                    stomach_t;
                normal[axis] = point[axis] - closest_body[axis];
            }
            normal_len = physx_vec3_len(normal);
            if (normal_len > 0.000001f) {
                float nh = normal[0] / normal_len;
                float nv = normal[1] / normal_len;
                float ns = normal[2] / normal_len;
                radius = (float)sqrt((double)(
                    radius_at[0] * radius_at[0] * nh * nh +
                    radius_at[1] * radius_at[1] * nv * nv +
                    radius_at[2] * radius_at[2] * ns * ns));
            } else {
                radius = radius_at[0];
                if (radius_at[1] > radius) radius = radius_at[1];
                if (radius_at[2] > radius) radius = radius_at[2];
            }
            radius = (radius - body_chain_collider_cfg.chain_radius) *
                     (body_radius_scale /
                      physx_clampf(body_chain_collider_cfg.response_radius_scale,
                                   0.05f, 4.0f)) +
                     chain->collision_radius;
            margin = dist - radius;
            added_hits = addon_chain_accumulate_collision_contact(
                chain, target, 1, 1.0f, margin, 0,
                point, closest_body, NULL, NULL, NULL, NULL,
                &manifold,
                correction, &max_penetration,
                &applied_penetration);
            hit_count += added_hits;
            if (trace_enabled) {
                addon_chain_collision_trace_consider(
                    &trace, "spine_pair", margin, dist, radius,
                    1.0f, stomach_t, added_hits > 0,
                    applied_penetration);
            }
        }
        }
    }

    for (j = 0; j < BODY_COLLIDER_LIMB_PAIR_COUNT; j++) {
        int start_node;
        int end_node;
        float pair_t;
        float closest_body[3];
        float dist;
        float radius;
        float margin;
        float applied_penetration = 0.0f;
        float limb_a[3];
        float limb_b[3];
        int added_hits;
        body_chain_collider_limb_pair_nodes(j, &start_node, &end_node);
        if (!addon_chain_custom_body_edge_enabled(
                chain, start_node, end_node)) {
            continue;
        }
        if (!state->valid[start_node] || !state->valid[end_node]) {
            continue;
        }
        if (!addon_chain_body_node_in_frame_cached(
                chain_frame_person_index, collider_person_index,
                start_node, now, limb_a) ||
            !addon_chain_body_node_in_frame_cached(
                chain_frame_person_index, collider_person_index,
                end_node, now, limb_b)) {
            continue;
        }
        if (!addon_chain_point_capsule_margin(
                point,
                limb_a,
                limb_b,
                0.0f, &pair_t, closest_body, &dist, &margin)) {
            continue;
        }
        radius = body_chain_collider_limb_pair_radius(j, pair_t);
        radius = (radius - body_chain_collider_cfg.chain_radius) *
                 (body_radius_scale /
                  physx_clampf(body_chain_collider_cfg.response_radius_scale,
                               0.05f, 4.0f)) +
                 chain->collision_radius;
        margin = dist - radius;
        added_hits = addon_chain_accumulate_collision_contact(
            chain, target, 1, 1.0f, margin, 0,
            point, closest_body, NULL, NULL, NULL, NULL,
            &manifold,
            correction, &max_penetration,
            &applied_penetration);
        hit_count += added_hits;
        if (trace_enabled) {
            addon_chain_collision_trace_consider(
                &trace, body_chain_collider_limb_pair_name(j), margin,
                dist, radius, 1.0f, pair_t, added_hits > 0,
                applied_penetration);
        }
    }

    for (j = 0; j < BODY_COLLIDER_EXTRA_EDGE_COUNT; j++) {
        const body_collider_extra_edge_def_t *edge = &body_collider_extra_edges[j];
        float edge_t;
        float closest_body[3];
        float dist;
        float margin;
        float r0;
        float r1;
        float radius;
        float applied_penetration = 0.0f;
        float edge_a[3];
        float edge_b[3];
        int added_hits;
        if (!edge ||
            edge->start_node < 0 ||
            edge->start_node >= BODY_COLLIDER_NODE_COUNT ||
            edge->end_node < 0 ||
            edge->end_node >= BODY_COLLIDER_NODE_COUNT ||
            !state->valid[edge->start_node] ||
            !state->valid[edge->end_node] ||
            (addon_chain_body_contact_parent_attachment(edge->start_node) &&
             addon_chain_body_contact_parent_attachment(edge->end_node))) {
            continue;
        }
        if (!addon_chain_custom_body_edge_enabled(
                chain, edge->start_node, edge->end_node)) {
            continue;
        }
        if (!addon_chain_body_node_in_frame_cached(
                chain_frame_person_index, collider_person_index,
                edge->start_node, now, edge_a) ||
            !addon_chain_body_node_in_frame_cached(
                chain_frame_person_index, collider_person_index,
                edge->end_node, now, edge_b)) {
            continue;
        }
        if (!addon_chain_point_capsule_margin(
                point,
                edge_a,
                edge_b,
                0.0f, &edge_t, closest_body, &dist, &margin)) {
            continue;
        }
        r0 = body_chain_collider_radius_for_node(edge->start_node) *
             body_radius_scale;
        r1 = body_chain_collider_radius_for_node(edge->end_node) *
             body_radius_scale;
        radius = r0 + (r1 - r0) * physx_clampf(edge_t, 0.0f, 1.0f);
        radius += chain->collision_radius;
        margin = dist - radius;
        added_hits = addon_chain_accumulate_collision_contact(
            chain, target, 1, 1.0f, margin, 0,
            point, closest_body, NULL, NULL, NULL, NULL,
            &manifold,
            correction, &max_penetration,
            &applied_penetration);
        hit_count += added_hits;
        if (trace_enabled) {
            addon_chain_collision_trace_consider(
                &trace, edge->name, margin, dist, radius,
                1.0f, edge_t, added_hits > 0,
                applied_penetration);
        }
    }

    addon_chain_body_manifold_resolve(&manifold, correction);
    if (hit_count > 0 && manifold.contacts.count > 0) {
        int axis;
        float stable_normal[3];
        int stable_contact = addon_chain_stable_collision_normal(
            chain, target, now, correction, stable_normal);
        (void)dt;
        addon_chain_apply_collision_correction(chain, target, correction);
        if (stable_contact) {
            addon_chain_damp_stable_collision_velocity(
                chain, target, stable_normal);
        }
        for (axis = 0; axis < 3; axis++) {
            point[axis] += correction[axis];
        }
    }
    if (trace_enabled &&
        trace.has_candidate &&
        (!target->collision_response_log_tick ||
         now - target->collision_response_log_tick >=
             (hit_count > 0 ? 250u : 1000u))) {
        float correction_len = physx_vec3_len(correction);
        float raw_penetration = trace.margin < 0.0f ? -trace.margin : 0.0f;
        target->collision_response_log_tick = now;
        log_line("addon sidecar root-anchor collision contact chain=\"%s\" target=\"%s\" person_index=%d hits=%d supports=%d nearest=\"%s\" hit=%d margin=%.5f raw_penetration=%.5f applied_penetration=%.5f correction_len=%.5f correction=(%.5f,%.5f,%.5f) dist=%.5f radius=%.5f body_t=%.3f body_radius_scale=%.3f sidecar_radius=%.4f scope=0x%x note=\"root anchor is tested as a point; no parent-to-root collision capsule is created\"",
                 chain->name,
                 target->name,
                 collider_person_index + 1,
                 hit_count,
                 manifold.contacts.count,
                 trace.collider,
                 trace.hit,
                 trace.margin,
                 raw_penetration,
                 trace.applied_penetration,
                 correction_len,
                 correction[0], correction[1], correction[2],
                 trace.dist,
                 trace.radius,
                 trace.body_t,
                 body_radius_scale,
                 chain->collision_radius,
                 chain->collision_scope);
    }
    body_profile_set_active_person_config(-1);
    return hit_count > 0;
}

static int addon_chain_apply_self_collision(
    physx_chain_t *chain,
    physx_target_t *target,
    const float points[32][3],
    const int valid[32],
    const float start[3],
    int segment_index,
    float dt)
{
    float end[3];
    float correction[3] = { 0.0f, 0.0f, 0.0f };
    float max_penetration = 0.0f;
    int hit_count = 0;
    int previous_segment;
    if (!chain || !target || !points || !valid || !start ||
        !(chain->collision_scope & PHYSX_COLLISION_SCOPE_SELF) ||
        chain->collision_radius <= 0.0f) {
        return 0;
    }
    end[0] = start[0] + target->sim_offset[0];
    end[1] = start[1] + target->sim_offset[1];
    end[2] = start[2] + target->sim_offset[2];
    for (previous_segment = 0;
         previous_segment + 1 < segment_index - 1;
         previous_segment++) {
        float chain_t;
        float pair_t;
        float closest_chain[3];
        float closest_self[3];
        float dist;
        float margin;
        float chain_distance;
        float radius = chain->collision_radius * 2.0f;
        if (!valid[previous_segment] || !valid[previous_segment + 1]) {
            continue;
        }
        if (!body_chain_collider_pair_margin(
                start, end, 0.0f, 0.0f,
                points[previous_segment],
                points[previous_segment + 1],
                radius, &chain_t, &pair_t,
                closest_chain, closest_self, &dist, &margin,
                &chain_distance)) {
            continue;
        }
        hit_count += addon_chain_accumulate_collision_contact(
            chain, target, segment_index, chain_t, margin, 0,
            closest_chain, closest_self, NULL, NULL, NULL, NULL,
            NULL,
            correction, &max_penetration,
            NULL);
    }
    if (hit_count <= 0) return 0;
    (void)dt;
    addon_chain_apply_collision_correction(chain, target, correction);
    return 1;
}

static int addon_chain_apply_addons_collision(
    physx_sidecar_t *owner_sc,
    physx_chain_t *chain,
    physx_target_t *target,
    int person_index,
    const float start[3],
    int segment_index,
    DWORD now,
    float dt)
{
    float end[3];
    float correction[3] = { 0.0f, 0.0f, 0.0f };
    float max_penetration = 0.0f;
    int hit_count = 0;
    int i, c, j;
    if (!owner_sc || !chain || !target || !start ||
        !(chain->collision_scope & PHYSX_COLLISION_SCOPE_ADDONS) ||
        chain->collision_radius <= 0.0f ||
        person_index < 0 || person_index >= 4) {
        return 0;
    }
    end[0] = start[0] + target->sim_offset[0];
    end[1] = start[1] + target->sim_offset[1];
    end[2] = start[2] + target->sim_offset[2];
    for (i = 0; i < sidecar_count; i++) {
        physx_sidecar_t *other_sc = &sidecars[i];
        if (!other_sc->loaded || !other_sc->enabled) continue;
        for (c = 0; c < other_sc->chain_count; c++) {
            physx_chain_t *other = &other_sc->chains[c];
            float other_points[32][3];
            int other_count = 0;
            int other_person_index = -1;
            if (other == chain ||
                !other->addon_chain ||
                !other->collision_enabled ||
                other->target_count <= 1 ||
                other->collision_radius <= 0.0f) {
                continue;
            }
            if (!addon_chain_collision_points_body_local(
                    other_sc, other, other_points, &other_count,
                    &other_person_index, now) ||
                other_person_index != person_index) {
                continue;
            }
            body_profile_set_active_person_config(person_index);
            for (j = 0; j + 1 < other_count; j++) {
                float chain_t;
                float pair_t;
                float closest_chain[3];
                float closest_addon[3];
                float dist;
                float margin;
                float chain_distance;
                float radius = chain->collision_radius + other->collision_radius;
                if (!body_chain_collider_pair_margin(
                        start, end, 0.0f, 0.0f,
                        other_points[j],
                        other_points[j + 1],
                        radius, &chain_t, &pair_t,
                        closest_chain, closest_addon, &dist, &margin,
                        &chain_distance)) {
                    continue;
                }
                hit_count += addon_chain_accumulate_collision_contact(
                    chain, target, segment_index, chain_t, margin, 0,
                    closest_chain, closest_addon, NULL, NULL, NULL, NULL,
                    NULL,
                    correction,
                    &max_penetration, NULL);
            }
        }
    }
    body_profile_set_active_person_config(-1);
    if (hit_count <= 0) return 0;
    (void)dt;
    addon_chain_apply_collision_correction(chain, target, correction);
    return 1;
}

static void run_body_probe(DWORD now)
{
    static const int offsets[] = {
        0x064, 0x068, 0x06c
    };
    char full_name[256];
    char matched[384];
    void *raw = NULL;
    void *obj = NULL;
    BYTE *base;
    float *v;
    int offset;
    int axis;
    int slot_count = (int)(sizeof(offsets) / sizeof(offsets[0])) * 3;
    int offset_index;
    if (!body_probe_cfg.enabled) return;
    if (!body_probe_cfg.person[0] || !body_probe_cfg.node[0]) return;
    if (body_probe_cfg.cycle) {
        offset_index = body_probe_cfg.cycle_index / 3;
        axis = body_probe_cfg.cycle_index % 3;
        if (offset_index < 0 || offset_index >= (int)(sizeof(offsets) / sizeof(offsets[0]))) offset_index = 0;
        offset = offsets[offset_index];
    } else {
        offset = body_probe_cfg.offset;
        axis = body_probe_cfg.axis;
    }
    if (axis < 0 || axis > 2 || offset < 0) return;
    _snprintf(full_name, sizeof(full_name), "%sAnim:Model01:%s", body_probe_cfg.person, body_probe_cfg.node);
    full_name[sizeof(full_name) - 1] = 0;
    obj = resolve_runtime_exact_target(full_name, &raw, matched, sizeof(matched));
    if (!obj || is_nil_engine_object(raw, obj)) {
        obj = resolve_find_obj(full_name, &raw);
        lstrcpynA(matched, full_name, sizeof(matched));
    }
    if (!raw && !obj) return;
    base = (BYTE*)(body_probe_cfg.use_object ? obj : raw);
    if (!base || !ptr_readable(base + offset, sizeof(float) * 3)) return;
    v = (float*)(base + offset);
    if (!sane_probe_float(v[0]) || !sane_probe_float(v[1]) || !sane_probe_float(v[2])) return;
    if (body_probe_cfg.state == 0) {
        body_probe_cfg.raw = base;
        body_probe_cfg.original[0] = v[0];
        body_probe_cfg.original[1] = v[1];
        body_probe_cfg.original[2] = v[2];
        v[axis] = body_probe_cfg.original[axis] + body_probe_cfg.amount;
        body_probe_cfg.start_tick = now;
        body_probe_cfg.state = 1;
        log_line("body-probe applied person=\"%s\" node=\"%s\" runtime=\"%s\" source=%s base=%p offset=0x%03x axis=%d amount=%.5f before=(%.5f,%.5f,%.5f) after=(%.5f,%.5f,%.5f) cycle_index=%d",
                 body_probe_cfg.person, body_probe_cfg.node, matched,
                 body_probe_cfg.use_object ? "object" : "raw",
                 base, offset, axis, body_probe_cfg.amount,
                 body_probe_cfg.original[0], body_probe_cfg.original[1], body_probe_cfg.original[2],
                 v[0], v[1], v[2], body_probe_cfg.cycle_index);
    } else if (body_probe_cfg.state == 1 &&
               now - body_probe_cfg.start_tick >= (DWORD)body_probe_cfg.duration_ms) {
        v[0] = body_probe_cfg.original[0];
        v[1] = body_probe_cfg.original[1];
        v[2] = body_probe_cfg.original[2];
        log_line("body-probe restored person=\"%s\" node=\"%s\" source=%s base=%p offset=0x%03x axis=%d restored=(%.5f,%.5f,%.5f) cycle_index=%d",
                 body_probe_cfg.person, body_probe_cfg.node,
                 body_probe_cfg.use_object ? "object" : "raw",
                 base, offset, axis, v[0], v[1], v[2], body_probe_cfg.cycle_index);
        body_probe_cfg.state = 0;
        body_probe_cfg.raw = NULL;
        if (body_probe_cfg.cycle) {
            body_probe_cfg.cycle_index++;
            if (body_probe_cfg.cycle_index >= slot_count) body_probe_cfg.cycle_index = 0;
        } else {
            body_probe_cfg.state = 2;
        }
    }
}

#define ADDON_TRANSFORM_VALIDATION_INTERVAL_MS 100u

static int addon_target_transform_pointers_readable(
    physx_target_t *target,
    const float *parent_translation,
    const float *target_translation,
    DWORD now)
{
    LONG generation =
        InterlockedCompareExchange(&named_node_generation, 0, 0);
    int readable;
    if (!target || !target_translation) return 0;
    if (target->addon_validation_ready &&
        target->addon_validation_parent_ptr == parent_translation &&
        target->addon_validation_target_ptr == target_translation &&
        target->addon_validation_generation == generation &&
        now - target->addon_validation_tick <
            ADDON_TRANSFORM_VALIDATION_INTERVAL_MS) {
        return 1;
    }
    readable =
        (!parent_translation ||
         ptr_readable(parent_translation, sizeof(float) * 3)) &&
        ptr_readable(target_translation, sizeof(float) * 3);
    target->addon_validation_parent_ptr = (void*)parent_translation;
    target->addon_validation_target_ptr = (void*)target_translation;
    target->addon_validation_generation = generation;
    target->addon_validation_tick = now;
    target->addon_validation_ready = readable ? 1 : 0;
    return readable;
}

/* A selected room mesh is intentionally two-sided.  When two nearly
   coincident surfaces overlap (a common room-authoring pattern), an endpoint
   can otherwise alternate between their opposite normals every frame.  Keep
   the entry-side normal while contact is continuous; accept perpendicular
   normals so real floor-to-wall corner transitions still work. */
static int addon_chain_stabilize_room_world_correction(
    physx_target_t *target, DWORD now, float correction[3],
    int room_mesh_index, int terminal_segment, int *normal_locked_out)
{
    int *contact_valid;
    DWORD *contact_tick;
    float *contact_direction;
    int *contact_mesh_index;
    float correction_len;
    float current[3];
    float previous_len;
    float direction_dot = 1.0f;
    int axis;
    if (normal_locked_out) *normal_locked_out = 0;
    if (!target || !correction) return 0;
    contact_valid = terminal_segment ?
        &target->room_collision_terminal_world_contact_valid :
        &target->room_collision_world_contact_valid;
    contact_tick = terminal_segment ?
        &target->room_collision_terminal_world_contact_tick :
        &target->room_collision_world_contact_tick;
    contact_direction = terminal_segment ?
        target->room_collision_terminal_world_contact_direction :
        target->room_collision_world_contact_direction;
    contact_mesh_index = terminal_segment ?
        &target->room_collision_terminal_world_contact_mesh_index :
        &target->room_collision_world_contact_mesh_index;
    correction_len = physx_vec3_len(correction);
    if (!sane_probe_float(correction_len) || correction_len <= 0.000001f) {
        return 0;
    }
    for (axis = 0; axis < 3; axis++) {
        current[axis] = correction[axis] / correction_len;
    }
    previous_len = physx_vec3_len(contact_direction);
    if (*contact_valid && now - *contact_tick <= 1500u &&
        previous_len > 0.000001f) {
        direction_dot =
            current[0] * contact_direction[0] /
                previous_len +
            current[1] * contact_direction[1] /
                previous_len +
            current[2] * contact_direction[2] /
                previous_len;
        if (direction_dot <= -0.35f) {
            for (axis = 0; axis < 3; axis++) {
                correction[axis] = contact_direction[axis] /
                    previous_len * correction_len;
                current[axis] = contact_direction[axis] / previous_len;
            }
            if (normal_locked_out) *normal_locked_out = 1;
        }
    }
    *contact_valid = 1;
    *contact_tick = now;
    for (axis = 0; axis < 3; axis++) {
        contact_direction[axis] = current[axis];
    }
    if (!normal_locked_out || !*normal_locked_out) {
        *contact_mesh_index = room_mesh_index;
    }
    return 1;
}

/* Room meshes are intentionally two-sided and adjacent authored floor
   objects can overlap.  A per-link entry-side cache lets different links of
   one hair chain choose opposite normals for that same floor, making the
   chain push itself both up and down. Keep the first recent surface family
   as the chain's entry side for nearly opposite duplicate surfaces only.
   Same-side slopes and combined floor/wall corrections keep their actual
   normals so the contact solve is not flattened onto an old floor plane. */
static int addon_chain_stabilize_shared_room_normal(
    physx_chain_t *chain, DWORD now, float correction[3],
    int *normal_locked_out)
{
    float correction_len;
    float current[3];
    float previous_len;
    float direction_dot = 1.0f;
    LONG generation;
    int axis;
    if (!chain || !correction) return 0;
    correction_len = physx_vec3_len(correction);
    if (!sane_probe_float(correction_len) || correction_len <= 0.000001f) {
        return 0;
    }
    for (axis = 0; axis < 3; axis++) {
        current[axis] = correction[axis] / correction_len;
    }
    generation = InterlockedCompareExchange(&named_node_generation, 0, 0);
    previous_len = physx_vec3_len(chain->addon_room_contact_normal);
    if (chain->addon_room_contact_normal_valid &&
        chain->addon_room_contact_normal_generation == generation &&
        now - chain->addon_room_contact_normal_tick <= 2500u &&
        previous_len > 0.000001f) {
        direction_dot =
            current[0] * chain->addon_room_contact_normal[0] /
                previous_len +
            current[1] * chain->addon_room_contact_normal[1] /
                previous_len +
            current[2] * chain->addon_room_contact_normal[2] /
                previous_len;
        if (direction_dot <= -0.95f) {
            for (axis = 0; axis < 3; axis++) {
                current[axis] =
                    chain->addon_room_contact_normal[axis] / previous_len;
                correction[axis] = current[axis] * correction_len;
            }
            chain->addon_room_contact_normal_tick = now;
            if (direction_dot < 0.95f && normal_locked_out) {
                *normal_locked_out = 1;
            }
            return 1;
        }
        /* Do not let a valid perpendicular contact replace the primary
           floor normal. If floor contact ends, the cache expires and the new
           surface naturally becomes authoritative. */
        return 1;
    }
    chain->addon_room_contact_normal_valid = 1;
    chain->addon_room_contact_normal_generation = generation;
    chain->addon_room_contact_normal_tick = now;
    for (axis = 0; axis < 3; axis++) {
        chain->addon_room_contact_normal[axis] = current[axis];
    }
    return 1;
}

/* Most exported add-on chains have no explicit end joint.  The final bone
   still deforms geometry beyond its pivot, so stopping collision at that
   pivot leaves the visible tail/hair tip unprotected.  Extrapolate one final
   link from the last evaluated segment (with the terminal basis as a
   one-pivot fallback), matching the explicit terminal point used by the
   dedicated penis/testicle chains. */
static int addon_chain_terminal_endpoint_body_local(
    physx_chain_t *chain, physx_target_t *target,
    const body_chain_collider_person_state_t *state,
    const float previous_local[3], const float start_local[3],
    float end_local[3])
{
    void *bases[4];
    float matrix[9];
    float previous_direction[3];
    float previous_view[3];
    float view_direction[3];
    float local_direction[3];
    float direction_len;
    float terminal_length;
    float best_score = -1.0f;
    LONG generation;
    int selected_base = -1;
    int selected_axis = -1;
    float selected_sign = 1.0f;
    int bi;
    int axis;
    if (!chain || !target || !state || !start_local || !end_local ||
        target->sim_length <= 0.0001f ||
        chain->collision_terminal_scale <= 0.0001f) {
        return 0;
    }
    terminal_length = physx_clampf(
        target->sim_length * chain->collision_terminal_scale,
        0.005f, 0.750f);

    /* The last TJoint has no child pivot, but its evaluated matrix does carry
       the final bone's current rotation.  Select the matrix row that aligns
       with the preceding visible link once, then keep that row locked for
       the lifetime of the binding.  This makes the synthetic collision tip
       follow the bone that actually deforms the terminal hair instead of
       rebuilding it from tail02->tail03 (a segment controlled by tail02). */
    generation = InterlockedCompareExchange(&named_node_generation, 0, 0);
    bases[0] = target->raw_object;
    bases[1] = target->object;
    bases[2] = target->s_raw_object;
    bases[3] = target->s_object;
    if (previous_local) {
        previous_direction[0] = start_local[0] - previous_local[0];
        previous_direction[1] = start_local[1] - previous_local[1];
        previous_direction[2] = start_local[2] - previous_local[2];
        direction_len = physx_vec3_len(previous_direction);
        if (sane_probe_float(direction_len) && direction_len > 0.0005f) {
            for (axis = 0; axis < 3; axis++) {
                previous_direction[axis] /= direction_len;
            }
            previous_view[0] =
                previous_direction[0] * state->basis_h[0] +
                previous_direction[1] * state->basis_v[0] +
                previous_direction[2] * state->basis_s[0];
            previous_view[1] =
                previous_direction[0] * state->basis_h[1] +
                previous_direction[1] * state->basis_v[1] +
                previous_direction[2] * state->basis_s[1];
            previous_view[2] =
                previous_direction[0] * state->basis_h[2] +
                previous_direction[1] * state->basis_v[2] +
                previous_direction[2] * state->basis_s[2];
            direction_len = physx_vec3_len(previous_view);
            if (direction_len > 0.0001f) {
                for (axis = 0; axis < 3; axis++) {
                    previous_view[axis] /= direction_len;
                }
                if (target->room_collision_terminal_axis_valid &&
                    target->room_collision_terminal_axis_generation ==
                        generation &&
                    target->room_collision_terminal_axis_base >= 0 &&
                    target->room_collision_terminal_axis_base < 4 &&
                    target->room_collision_terminal_axis_index >= 0 &&
                    target->room_collision_terminal_axis_index < 3) {
                    selected_base =
                        target->room_collision_terminal_axis_base;
                    selected_axis =
                        target->room_collision_terminal_axis_index;
                    selected_sign =
                        target->room_collision_terminal_axis_sign;
                } else {
                    for (bi = 0; bi < 4; bi++) {
                        if (!bases[bi] ||
                            !body_chain_read_mat3_rows(bases[bi], matrix) ||
                            !addon_normalize_basis_rows(matrix)) {
                            continue;
                        }
                        for (axis = 0; axis < 3; axis++) {
                            float score =
                                matrix[axis * 3 + 0] * previous_view[0] +
                                matrix[axis * 3 + 1] * previous_view[1] +
                                matrix[axis * 3 + 2] * previous_view[2];
                            float absolute_score = physx_absf(score);
                            if (absolute_score > best_score) {
                                best_score = absolute_score;
                                selected_base = bi;
                                selected_axis = axis;
                                selected_sign = score < 0.0f ? -1.0f : 1.0f;
                            }
                        }
                    }
                    if (selected_base >= 0 && selected_axis >= 0 &&
                        best_score >= 0.45f) {
                        target->room_collision_terminal_axis_valid = 1;
                        target->room_collision_terminal_axis_generation =
                            generation;
                        target->room_collision_terminal_axis_base =
                            selected_base;
                        target->room_collision_terminal_axis_index =
                            selected_axis;
                        target->room_collision_terminal_axis_sign =
                            selected_sign;
                        if (defaults_cfg.debug) {
                            log_line("addon room collision terminal-axis chain=\"%s\" target=\"%s\" matrix_source=%d axis=%d sign=%.0f alignment=%.4f generation=%ld note=\"locked final-bone evaluated axis; terminal capsule now follows the bone that deforms the visible tip\"",
                                     chain->name, target->name,
                                     selected_base, selected_axis,
                                     selected_sign, best_score, generation);
                        }
                    }
                }
                if (selected_base >= 0 && selected_axis >= 0 &&
                    bases[selected_base] &&
                    body_chain_read_mat3_rows(bases[selected_base], matrix) &&
                    addon_normalize_basis_rows(matrix)) {
                    for (axis = 0; axis < 3; axis++) {
                        view_direction[axis] =
                            matrix[selected_axis * 3 + axis] * selected_sign;
                    }
                    if (body_collider_view_delta_to_local(
                            view_direction, state->basis_h, state->basis_v,
                            state->basis_s, local_direction)) {
                        direction_len = physx_vec3_len(local_direction);
                        if (sane_probe_float(direction_len) &&
                            direction_len > 0.0001f) {
                            for (axis = 0; axis < 3; axis++) {
                                end_local[axis] = start_local[axis] +
                                    local_direction[axis] / direction_len *
                                    terminal_length;
                            }
                            return physx_vec3_sane_limit(end_local, 20.0f);
                        }
                    }
                }
            }

            /* Safe fallback for unusual add-ons whose runtime matrix cannot
               be sampled.  It is deliberately secondary to the locked live
               terminal axis. */
            for (axis = 0; axis < 3; axis++) {
                end_local[axis] = start_local[axis] +
                    previous_direction[axis] * terminal_length;
            }
            return physx_vec3_sane_limit(end_local, 20.0f);
        }
    }
    return 0;
}

static int addon_chain_apply_room_collision(
    physx_sidecar_t *sc, physx_chain_t *chain, physx_target_t *target,
    int person_index, const float start_local[3],
    const float end_local[3], physx_target_t *response_override,
    int terminal_segment)
{
    body_chain_collider_person_state_t *state;
    physx_target_t *response_target;
    float world_start[3];
    float world_end[3];
    float world_correction[3];
    float local_correction[3];
    float world_correction_len;
    int bend_mapping = 0;
    int temporal_contact = 0;
    int inherited_terminal_contact = 0;
    int normal_locked = 0;
    int room_mesh_index = -1;
    int target_index;
    int *track_valid;
    LONG *track_generation;
    DWORD *track_tick;
    float *track_world;
    DWORD *response_log_tick;
    LONG generation;
    DWORD now;
    if (!chain || !target || !start_local || !end_local ||
        !(chain->collision_scope & PHYSX_COLLISION_SCOPE_ROOM) ||
        !room_collision_is_enabled() || person_index < 0 ||
        person_index >= 4 || chain->collision_radius <= 0.0f) {
        return 0;
    }
    target_index = (int)(target - chain->targets);
    /* A joint pivot is positioned by the preceding bone. Rotating the
       contacted bone only changes geometry after that pivot, which is why
       the old response logged large rotations while the probed hair joint
       remained below the floor. The first simulated pivot is anchored to
       the body and cannot be displaced by an add-on link rotation. */
    if ((!response_override && target_index <= 1) || target_index < 1 ||
        target_index >= chain->target_count) {
        target->room_collision_track_valid = 0;
        target->room_collision_terminal_track_valid = 0;
        target->room_collision_contact_valid = 0;
        target->room_collision_rest_frames = 0;
        return 0;
    }
    response_target = response_override ? response_override :
        &chain->targets[target_index - 1];
    if (!response_target->addon_simulated_target ||
        response_target->sim_length <= 0.0001f) {
        return 0;
    }
    state = &body_chain_collider_states[person_index];
    if (!state->ready || !state->basis_valid ||
        !room_collision_body_local_point_to_world(
            state, start_local, world_start) ||
        !room_collision_body_local_point_to_world(
            state, end_local, world_end)) {
        return 0;
    }
    now = GetTickCount();
    generation = InterlockedCompareExchange(&named_node_generation, 0, 0);
    track_valid = terminal_segment ?
        &target->room_collision_terminal_track_valid :
        &target->room_collision_track_valid;
    track_generation = terminal_segment ?
        &target->room_collision_terminal_track_generation :
        &target->room_collision_track_generation;
    track_tick = terminal_segment ?
        &target->room_collision_terminal_track_tick :
        &target->room_collision_track_tick;
    track_world = terminal_segment ?
        target->room_collision_terminal_track_world :
        target->room_collision_track_world;
    response_log_tick = terminal_segment ?
        &target->room_collision_terminal_response_log_tick :
        &target->room_collision_response_log_tick;
    if (*track_valid && *track_generation == generation &&
        now - *track_tick <= 160u) {
        float move[3];
        float move_len;
        move[0] = world_end[0] - track_world[0];
        move[1] = world_end[1] - track_world[1];
        move[2] = world_end[2] - track_world[2];
        move_len = physx_vec3_len(move);
        if (sane_probe_float(move_len) && move_len <= 0.750f) {
            temporal_contact = room_collision_resolve_swept_sphere(
                track_world, world_end,
                chain->collision_radius, world_correction,
                &room_mesh_index);
        }
    }
    if (!temporal_contact &&
        !room_collision_resolve_swept_sphere(
            world_start, world_end, chain->collision_radius,
            world_correction, &room_mesh_index)) {
        /* A terminal segment can be initialized wholly behind a thin room
           surface: the preceding live joint crosses the surface and obtains
           a valid contact, while the extrapolated tip starts and ends beyond
           the sphere penetration band.  With no crossing of its own, the
           ordinary two-sided closest-point query cannot know which side to
           recover toward.  Inherit the immediately preceding joint's recent
           contact plane and keep the tip on that same entry side. */
        if (terminal_segment &&
            target->room_collision_world_contact_valid &&
            now - target->room_collision_world_contact_tick <= 1500u &&
            target->room_collision_track_valid &&
            target->room_collision_track_generation == generation) {
            float normal[3];
            float normal_len = physx_vec3_len(
                target->room_collision_world_contact_direction);
            float signed_push;
            if (normal_len > 0.000001f) {
                normal[0] =
                    target->room_collision_world_contact_direction[0] /
                    normal_len;
                normal[1] =
                    target->room_collision_world_contact_direction[1] /
                    normal_len;
                normal[2] =
                    target->room_collision_world_contact_direction[2] /
                    normal_len;
                signed_push =
                    (target->room_collision_track_world[0] - world_end[0]) *
                        normal[0] +
                    (target->room_collision_track_world[1] - world_end[1]) *
                        normal[1] +
                    (target->room_collision_track_world[2] - world_end[2]) *
                        normal[2];
                if (sane_probe_float(signed_push) &&
                    signed_push > 0.0001f && signed_push <= 0.3500f) {
                    world_correction[0] = normal[0] * signed_push;
                    world_correction[1] = normal[1] * signed_push;
                    world_correction[2] = normal[2] * signed_push;
                    room_mesh_index =
                        target->room_collision_world_contact_mesh_index;
                    inherited_terminal_contact = 1;
                }
            }
        }
        if (!inherited_terminal_contact) {
            *track_valid = 1;
            *track_generation = generation;
            *track_tick = now;
            memcpy(track_world, world_end, sizeof(float) * 3);
            if (target->room_collision_contact_valid &&
                now - target->room_collision_contact_tick > 120u) {
                target->room_collision_contact_valid = 0;
                target->room_collision_rest_frames = 0;
            }
            if (terminal_segment) {
                if (target->room_collision_terminal_world_contact_valid &&
                    now -
                        target->room_collision_terminal_world_contact_tick >
                        1500u) {
                    target->room_collision_terminal_world_contact_valid = 0;
                    target->room_collision_terminal_world_contact_mesh_index =
                        -1;
                }
            } else if (target->room_collision_world_contact_valid &&
                       now - target->room_collision_world_contact_tick >
                           1500u) {
                target->room_collision_world_contact_valid = 0;
                target->room_collision_world_contact_mesh_index = -1;
            }
            if (terminal_segment && chain->addon_room_rest_sleeping &&
                now - chain->addon_room_rest_tick > 90u) {
                addon_chain_wake_room_rest(chain);
            }
            return 0;
        }
    }
    if (!addon_chain_stabilize_room_world_correction(
            target, now, world_correction, room_mesh_index,
            terminal_segment, &normal_locked) ||
        !addon_chain_stabilize_shared_room_normal(
            chain, now, world_correction, &normal_locked)) {
        return 0;
    }
    {
        float final_normal_len = physx_vec3_len(world_correction);
        float *target_normal = terminal_segment ?
            target->room_collision_terminal_world_contact_direction :
            target->room_collision_world_contact_direction;
        int *target_contact_valid = terminal_segment ?
            &target->room_collision_terminal_world_contact_valid :
            &target->room_collision_world_contact_valid;
        DWORD *target_contact_tick = terminal_segment ?
            &target->room_collision_terminal_world_contact_tick :
            &target->room_collision_world_contact_tick;
        int axis;
        if (final_normal_len > 0.000001f) {
            for (axis = 0; axis < 3; axis++) {
                target_normal[axis] =
                    world_correction[axis] / final_normal_len;
            }
            *target_contact_valid = 1;
            *target_contact_tick = now;
        }
    }
    {
        float correction_len = physx_vec3_len(world_correction);
        float separation_slop = physx_clampf(
            chain->collision_radius * 0.03f, 0.00050f, 0.00100f);
        int axis;
        for (axis = 0; axis < 3; axis++) {
            float normal = correction_len > 0.000001f ?
                world_correction[axis] / correction_len : 0.0f;
            track_world[axis] =
                world_end[axis] + world_correction[axis] +
                normal * separation_slop;
        }
        *track_valid = 1;
        *track_generation = generation;
        *track_tick = now;
    }
    world_correction_len = physx_vec3_len(world_correction);
    if (terminal_segment &&
        addon_chain_apply_terminal_room_manifold(
            sc, chain, person_index, world_end, world_correction, now)) {
        local_correction[0] = 0.0f;
        local_correction[1] = 0.0f;
        local_correction[2] = 0.0f;
        bend_mapping = 4;
    } else {
        if(response_target->contact_basis_tick) {
            float gradient[3];
            int axis;
            if(!addon_chain_solve_room_contact(chain,response_target,state,
                    world_start,world_end,world_correction,local_correction,gradient)) return 0;
            for(axis=0;axis<3;axis++) response_target->sim_offset[axis]+=local_correction[axis];
            response_target->sim_contact_corrected=1;
            addon_chain_collision_normalize_target(response_target);
            physx_contact_link_velocity(response_target->sim_velocity,response_target->sim_offset,gradient);
            bend_mapping=6;
        } else {
        bend_mapping = addon_chain_visible_room_correction_to_sim_delta(
            response_target, world_start, world_end, world_correction,
            local_correction) ? 2 : 0;
        if (!bend_mapping) {
            bend_mapping = addon_chain_world_room_correction_to_bend(
                sc, chain, response_target, now, world_correction,
                local_correction) ? 1 : 0;
        }
        if (!bend_mapping &&
            !room_collision_world_vector_to_body_local(
                state, world_correction, local_correction)) {
            return 0;
        }
        addon_chain_apply_room_collision_correction(
            chain, response_target, local_correction);
        }
        addon_chain_update_room_rest_pose(
            chain, response_target, world_correction, now);
    }
    if (defaults_cfg.debug &&
        (!*response_log_tick || now - *response_log_tick >= 500u)) {
        *response_log_tick = now;
        log_line("addon room collision contact chain=\"%s\" target=\"%s\" response_target=\"%s\" segment=%s person_index=%d mesh=\"%s\" radius=%.5f world_start=(%.5f,%.5f,%.5f) world_end=(%.5f,%.5f,%.5f) world_correction=(%.5f,%.5f,%.5f) solver_correction=(%.5f,%.5f,%.5f) mapping=%s temporal=%d normal_locked=%d rest_frames=%d",
                 chain->name, target->name, response_target->name,
                 terminal_segment ? "terminal" : "joint",
                 person_index + 1,
                 room_mesh_index >= 0 &&
                 room_mesh_index < room_collision_mesh_count ?
                    room_collision_meshes[room_mesh_index].name : "unknown",
                 chain->collision_radius,
                 world_start[0], world_start[1], world_start[2],
                 world_end[0], world_end[1], world_end[2],
                 world_correction[0], world_correction[1],
                 world_correction[2],
                 local_correction[0], local_correction[1],
                 local_correction[2],
                 bend_mapping == 6 ? "output-aware" : bend_mapping == 4 ? "chain-manifold" :
                    (bend_mapping == 2 ? "visible-segment-ik" :
                     (bend_mapping == 1 ? "live-parent-bend" :
                      "body-local-fallback")),
                 temporal_contact ? temporal_contact :
                    (inherited_terminal_contact ? 2 : 0),
                 normal_locked,
                 response_target->room_collision_rest_frames);
    }
    return 1;
}

/* Shared by normal output and the final publication of upstream contacts. */
static void addon_chain_contact_output_angles_at(
    physx_chain_t *chain, physx_target_t *target, float out_r[3],
    float *pitch_out, float *roll_out, const float offset[3])
{
    float rest_len = target->sim_length;
    float bend_vec[3];
    float pitch;
    float roll;
    float full_angle_delta[3] = { 0.0f, 0.0f, 0.0f };
    int full_angle_valid = 0;
    float pitch_min = -chain->limit_angle;
    float pitch_max = chain->limit_angle;
    float roll_min = -chain->limit_angle;
    float roll_max = chain->limit_angle;
    bend_vec[0] = offset[0] - target->sim_rest[0];
    bend_vec[1] = offset[1] - target->sim_rest[1];
    bend_vec[2] = offset[2] - target->sim_rest[2];
    if (chain->addon_chain &&
        chain->rotation_solver_full_angle) {
        full_angle_valid =
            addon_chain_full_angle_output_delta_at(
                chain, target, offset, full_angle_delta);
    }
    if (full_angle_valid) {
        pitch = full_angle_delta[0];
        roll = full_angle_delta[2];
        if (addon_chain_root_visual_roll_inverted(chain,
                                                  target)) {
            roll = -roll;
            full_angle_delta[2] = roll;
        }
    } else {
        addon_chain_bend_to_output_angles(
            chain, target, bend_vec, rest_len,
            &pitch, &roll);
        if (addon_chain_root_visual_roll_inverted(chain,
                                                  target)) {
            roll = -roll;
        }
    }
    if (addon_chain_root_physx_target(chain, target) &&
        physx_absf(chain->root_bend_scale - 1.0f) > 0.0001f) {
        pitch *= chain->root_bend_scale;
        roll *= chain->root_bend_scale;
        if (full_angle_valid) {
            full_angle_delta[0] *= chain->root_bend_scale;
            full_angle_delta[1] *= chain->root_bend_scale;
            full_angle_delta[2] *= chain->root_bend_scale;
        }
    }
    if (target->joint_settings_initialized) {
        pitch_min = target->joint_min_angle[0];
        pitch_max = target->joint_max_angle[0];
        roll_min = target->joint_min_angle[2];
        roll_max = target->joint_max_angle[2];
    }
    if (pitch_min > pitch_max) {
        float tmp = pitch_min;
        pitch_min = pitch_max;
        pitch_max = tmp;
    }
    if (roll_min > roll_max) {
        float tmp = roll_min;
        roll_min = roll_max;
        roll_max = tmp;
    }
    if (full_angle_valid) {
        int rotation_axis;
        for (rotation_axis = 0;
             rotation_axis < 3;
             rotation_axis++) {
            float axis_min = -chain->limit_angle;
            float axis_max = chain->limit_angle;
            if (target->joint_settings_initialized) {
                axis_min =
                    target->joint_min_angle[rotation_axis];
                axis_max =
                    target->joint_max_angle[rotation_axis];
            }
            if (axis_min > axis_max) {
                float tmp = axis_min;
                axis_min = axis_max;
                axis_max = tmp;
            }
            full_angle_delta[rotation_axis] =
                physx_clampf(
                    full_angle_delta[rotation_axis],
                    axis_min, axis_max);
            out_r[rotation_axis] =
                target->sim_rotation_rest[rotation_axis] +
                full_angle_delta[rotation_axis];
        }
        pitch = full_angle_delta[0];
        roll = full_angle_delta[2];
    } else {
        pitch = physx_clampf(pitch, pitch_min, pitch_max);
        roll = physx_clampf(roll, roll_min, roll_max);
        out_r[0] = target->sim_rotation_rest[0] + pitch;
        out_r[1] = target->sim_rotation_rest[1];
        out_r[2] = target->sim_rotation_rest[2] + roll;
    }
    if (pitch_out) *pitch_out = pitch;
    if (roll_out) *roll_out = roll;
}

static void addon_chain_contact_output_angles(physx_chain_t *chain,
    physx_target_t *target,float out_r[3],float *pitch,float *roll)
{
    addon_chain_contact_output_angles_at(chain,target,out_r,pitch,roll,target->sim_offset);
}

/* Pure forward evaluation: same angle channels, signs, limits and matrix
   scale as publication. No target mutation or engine writes while probing. */
static void addon_chain_contact_predict(physx_chain_t *chain,physx_target_t *target,
    const float offset[3],const float local[3],float point[3])
{
    float angles[3],rows[9],parent_point[3];
    int i,j;
    addon_chain_contact_output_angles_at(chain,target,angles,NULL,NULL,offset);
    for(i=0;i<3;i++) angles[i]=target->sim_rotation_rest[i]+
        (angles[i]-target->sim_rotation_rest[i])*chain->skinned_matrix_scale;
    physx_contact_rotation_rows(angles,rows);
    for(i=0;i<3;i++) {
        parent_point[i]=0;
        for(j=0;j<3;j++) parent_point[i]+=local[j]*rows[j*3+i];
    }
    for(i=0;i<3;i++) {
        point[i]=0;
        for(j=0;j<3;j++) point[i]+=parent_point[j]*target->contact_parent_body_basis[j*3+i];
    }
}

/* Diagnostic only: evaluate motion of an arbitrary material point under one
   joint's proposed correction. This must not modify the joint or its output. */
static void addon_chain_contact_point_delta(physx_chain_t *chain,
    physx_target_t *target,const float pivot[3],const float point[3],
    const float delta[3],float movement[3])
{
    float local[3],before[3],after[3],offset[3];
    int i,j;
    for(i=0;i<3;i++) {
        local[i]=0;
        for(j=0;j<3;j++) local[i]+=(point[j]-pivot[j])*target->contact_live_body_basis[i*3+j];
        offset[i]=target->sim_offset[i]+delta[i];
    }
    addon_chain_contact_predict(chain,target,target->sim_offset,local,before);
    addon_chain_contact_predict(chain,target,offset,local,after);
    for(i=0;i<3;i++) movement[i]=after[i]-before[i];
}

static int addon_chain_solve_body_contact(physx_chain_t *chain,
    physx_target_t *target,const float pivot[3],const float end[3],
    const float request[3],float delta[3],float achieved[3],float gradient[3])
{
    float local[3],normal[3],offset[3],base[3],point[3];
    float depth=physx_vec3_len(request),goal,epsilon;
    int i,j,iteration;
    if (!target->contact_basis_tick || depth<=0.000001f || target->sim_length<=0.0001f) return 0;
    epsilon=target->sim_length*0.001f;
    for(i=0;i<3;i++) {
        local[i]=0;
        for(j=0;j<3;j++) local[i]+=(end[j]-pivot[j])*target->contact_live_body_basis[i*3+j];
        normal[i]=request[i]/depth;
    }
    memcpy(offset,target->sim_offset,sizeof(offset));
    addon_chain_contact_predict(chain,target,offset,local,base);
    goal=vec3_dot(base,normal)+depth;
    memset(gradient,0,sizeof(float)*3);
    for(iteration=0;iteration<4;iteration++) {
        float residual,norm2=0,radial,step[3];
        addon_chain_contact_predict(chain,target,offset,local,point);
        residual=goal-vec3_dot(point,normal);
        if (residual<=0.000001f) break;
        for(i=0;i<3;i++) {
            float plus[3],minus[3],p[3],m[3],len;
            memcpy(plus,offset,sizeof(plus)); memcpy(minus,offset,sizeof(minus));
            plus[i]+=epsilon; minus[i]-=epsilon;
            len=physx_vec3_len(plus); for(j=0;j<3;j++) plus[j]*=target->sim_length/len;
            len=physx_vec3_len(minus); for(j=0;j<3;j++) minus[j]*=target->sim_length/len;
            addon_chain_contact_predict(chain,target,plus,local,p);
            addon_chain_contact_predict(chain,target,minus,local,m);
            gradient[i]=((p[0]-m[0])*normal[0]+(p[1]-m[1])*normal[1]+(p[2]-m[2])*normal[2])/(2*epsilon);
        }
        radial=vec3_dot(gradient,offset)/(target->sim_length*target->sim_length);
        for(i=0;i<3;i++) { gradient[i]-=radial*offset[i]; norm2+=gradient[i]*gradient[i]; }
        if (norm2<0.000001f) break;
        for(i=0;i<3;i++) step[i]=gradient[i]*residual/(norm2+0.0001f);
        {
            float length=physx_vec3_len(step),cap=target->sim_length*0.08f;
            int attempt,improved=0;
            if(length>cap) for(i=0;i<3;i++) step[i]*=cap/length;
            for(attempt=0;attempt<5;attempt++) {
                float trial[3],candidate[3],len,next;
                for(i=0;i<3;i++) trial[i]=offset[i]+step[i];
                len=physx_vec3_len(trial);
                for(i=0;i<3;i++) trial[i]*=target->sim_length/len;
                addon_chain_contact_predict(chain,target,trial,local,candidate);
                next=goal-vec3_dot(candidate,normal);
                if(fabsf(next)<residual && next>=-depth*0.05f) {
                    memcpy(offset,trial,sizeof(offset)); improved=1; break;
                }
                for(i=0;i<3;i++) step[i]*=0.5f;
            }
            if(!improved) break;
        }
    }
    addon_chain_contact_predict(chain,target,offset,local,point);
    for(i=0;i<3;i++) { delta[i]=offset[i]-target->sim_offset[i]; achieved[i]=point[i]-base[i]; }
    depth=physx_vec3_len(gradient);
    if(depth>0.000001f) for(i=0;i<3;i++) gradient[i]/=depth;
    return vec3_dot(achieved,normal)>0.000001f && physx_vec3_sane_limit(delta,0.1f);
}

/* Freeze the evaluated orientation before publishing any new chain pose.
   R_live = R_published * R_parent; therefore parent = transpose(R_published)
   * R_live. Preserve all three axes, including twist around the visible link. */
static void addon_chain_capture_contact_basis(physx_sidecar_t *sc,
    physx_chain_t *chain,DWORD now)
{
    int person,i,row,col,k;
    body_chain_collider_person_state_t *state;
    if(!chain->addon_chain || !chain->collision_enabled || !chain->skinned_matrix_enabled) return;
    person=addon_chain_collision_person_index(sc,chain);
    for(i=0;i<chain->target_count;i++) {
        chain->targets[i].contact_basis_tick=0;
        chain->targets[i].contact_pivot_tick=0;
        chain->targets[i].contact_terminal_tick=0;
    }
    if(person<0 || person>=4) return;
    state=&body_chain_collider_states[person];
    if(!state->ready || !state->basis_valid) return;
    for(i=1;i<chain->target_count;i++) {
        physx_target_t *target=&chain->targets[i];
        float live[9],rotation[9];
        if(addon_chain_target_body_local(sc,chain,target,person,state,target->contact_pivot_body))
            target->contact_pivot_tick=now;
        if(!target->addon_simulated_target || !target->addon_visual_pose_valid ||
            now-target->addon_visual_tick>120u || !target->object ||
            !body_chain_read_mat3_rows(target->object,live) || !addon_normalize_basis_rows(live)) continue;
        for(row=0;row<3;row++) {
            target->contact_live_body_basis[row*3]=vec3_dot(&live[row*3],state->basis_h);
            target->contact_live_body_basis[row*3+1]=vec3_dot(&live[row*3],state->basis_v);
            target->contact_live_body_basis[row*3+2]=vec3_dot(&live[row*3],state->basis_s);
        }
        if(!addon_normalize_basis_rows(target->contact_live_body_basis)) continue;
        physx_contact_rotation_rows(target->addon_visual_rotation,rotation);
        for(row=0;row<3;row++) for(col=0;col<3;col++) {
            float sum=0;
            for(k=0;k<3;k++) sum+=rotation[k*3+row]*target->contact_live_body_basis[k*3+col];
            target->contact_parent_body_basis[row*3+col]=sum;
        }
        target->contact_basis_tick=now;
    }
    if(chain->target_count>2 && chain->target_count<=32) {
        physx_target_t *tip=&chain->targets[chain->target_count-1];
        physx_target_t *previous=&chain->targets[chain->target_count-2];
        if(tip->contact_pivot_tick==now && previous->contact_pivot_tick==now &&
            addon_chain_terminal_endpoint_body_local(chain,tip,state,
                previous->contact_pivot_body,tip->contact_pivot_body,tip->contact_terminal_body))
            tip->contact_terminal_tick=now;
    }
}

/* A later segment may correct an upstream joint already published above.
   Publish its final solver pose now, without integrating it a second time or
   turning depenetration into inherited velocity on the next frame. */
static void addon_chain_publish_final_contacts(physx_chain_t *chain, DWORD now)
{
    int i, axis;
    if (!chain->skinned_matrix_enabled) return;
    for (i = 0; i < chain->target_count; i++) {
        physx_target_t *target = &chain->targets[i];
        float delta[3], out_r[3];
        if (!target->addon_simulated_target || !target->addon_visual_pose_valid ||
            target->addon_visual_tick != now || target->sim_length <= 0.0001f)
            continue;
        for (axis = 0; axis < 3; axis++)
            delta[axis] = target->sim_offset[axis] - target->sim_output_offset_prev[axis];
        if (physx_vec3_len(delta) <= 0.0000001f) continue;
        addon_chain_contact_output_angles(chain, target, out_r, NULL, NULL);
        for (axis = 0; axis < 3; axis++) {
            target->addon_visual_rotation[axis] = target->sim_rotation_rest[axis] +
                (out_r[axis]-target->sim_rotation_rest[axis])*chain->skinned_matrix_scale;
            target->sim_output_offset_prev[axis] = target->sim_offset[axis];
            target->sim_output_velocity[axis] = target->sim_velocity[axis];
        }
        physx_addon_apply_target_visual_pose(target);
    }
}

/* Integrate all free-motion forces together on the fixed-length sphere.
   Project force as well as velocity: a radial spring load must not shift
   the resting direction when frame duration changes. Small substeps bound
   explicit spring integration without delaying motion or contact output. */
static void addon_chain_integrate_free_motion(
    const physx_chain_t *chain, physx_target_t *target,
    const physx_target_t *parent, int target_index, int inherit,
    const float force_acceleration[3], float k, float damping, float dt)
{
    float drive[3], parent_velocity[3] = {0.0f, 0.0f, 0.0f};
    float length = target->sim_length;
    float follow_rate = 0.0f;
    float h;
    int axis, step, steps;
    if (length <= 0.0001f || dt <= 0.0f) return;
    for (axis = 0; axis < 3; axis++)
        drive[axis] = force_acceleration[axis] + k * target->sim_rest[axis];
    if (inherit && parent && parent->sim_initialized && target_index > 1) {
        float parent_bend[3], rest_dot;
        /* Transfer angular bend/speed, expressed at this link's radius.
           Raw parent displacement overdrives a short child of a long link. */
        float parent_scale = parent->sim_length > 0.0001f &&
                             isfinite(parent->sim_length) ?
                             length / parent->sim_length : 1.0f;
        float gain = physx_clampf(target->joint_settings_initialized ?
                         target->joint_gain : chain->joint_gain, 0.05f, 3.0f) *
                     chain->stiffness;
        float taper = 1.0f - (float)(target_index - 2) * 0.18f;
        if (taper < 0.45f) taper = 0.45f;
        follow_rate = 0.16f * 60.0f * taper *
                      physx_clampf(chain->drive_strength, 0.0f, 1.0f);
        for (axis = 0; axis < 3; axis++)
            parent_bend[axis] = (parent->sim_offset[axis] - parent->sim_rest[axis]) *
                                parent_scale;
        rest_dot = vec3_dot(parent_bend, target->sim_rest) / (length * length);
        for (axis = 0; axis < 3; axis++) {
            float inherited = parent_bend[axis] - target->sim_rest[axis] * rest_dot;
            drive[axis] += gain * (target->sim_rest[axis] + inherited * 0.95f * taper);
            if (parent->sim_output_velocity_initialized)
                parent_velocity[axis] = physx_clampf(parent->sim_output_velocity[axis] * parent_scale,
                                                    -3.0f, 3.0f);
        }
    }
    /* The omitted -k*offset and -inherit_gain*offset terms are radial and
       vanish under the tangent projection below. Both spring targets are
       sampled before any position integration, including on child joints. */
    dt = physx_clampf(dt, 0.0f, 0.05f);
    steps = (int)ceilf(dt * 240.0f);
    if (steps < 1) steps = 1;
    h = dt / (float)steps;
    for (step = 0; step < steps; step++) {
        float normal[3], acceleration[3];
        float len = physx_vec3_len(target->sim_offset);
        float radial_force, radial_velocity;
        if (len <= 0.000001f) return;
        for (axis = 0; axis < 3; axis++) {
            normal[axis] = target->sim_offset[axis] / len;
            acceleration[axis] = drive[axis] + follow_rate * parent_velocity[axis];
        }
        radial_force = vec3_dot(acceleration, normal);
        radial_velocity = vec3_dot(target->sim_velocity, normal);
        for (axis = 0; axis < 3; axis++) {
            float tangent_force = acceleration[axis] - radial_force * normal[axis];
            float tangent_velocity = target->sim_velocity[axis] - radial_velocity * normal[axis];
            /* The old damping had a second 0.25*d pass. Combine it with
               velocity following in one stable drag term. */
            target->sim_velocity[axis] = (tangent_velocity + tangent_force * h) /
                (1.0f + (1.25f * damping + follow_rate) * h);
            target->sim_offset[axis] = normal[axis] * length + target->sim_velocity[axis] * h;
        }
        len = physx_vec3_len(target->sim_offset);
        if (len <= 0.000001f) return;
        for (axis = 0; axis < 3; axis++) {
            normal[axis] = target->sim_offset[axis] / len;
            target->sim_offset[axis] = normal[axis] * length;
        }
        radial_velocity = vec3_dot(target->sim_velocity, normal);
        for (axis = 0; axis < 3; axis++)
            target->sim_velocity[axis] -= radial_velocity * normal[axis];
    }
}

/* Fixed-length integration already supplies the solved tangent velocity.
   Use it through contact/release and small free motion alike, without a
   frame-displacement deadzone. Older rigid paths retain their displacement
   estimate, except that depenetration must never become inherited momentum. */
static void addon_chain_update_output_velocity(physx_target_t *target, float dt,
    int solver_velocity)
{
    float delta[3];
    int axis;
    for (axis = 0; axis < 3; axis++)
        delta[axis] = target->sim_offset[axis] - target->sim_output_offset_prev[axis];
    if (target->sim_contact_corrected || solver_velocity) {
        for (axis = 0; axis < 3; axis++)
            target->sim_output_velocity[axis] =
                physx_clampf(target->sim_velocity[axis], -3.0f, 3.0f);
    } else if (!target->sim_output_velocity_initialized ||
               addon_vec3_len_exact(delta) < 0.00001f || dt <= 0.0f) {
        memset(target->sim_output_velocity, 0, sizeof(target->sim_output_velocity));
    } else {
        for (axis = 0; axis < 3; axis++)
            target->sim_output_velocity[axis] = physx_clampf(delta[axis]/dt, -3.0f, 3.0f);
    }
    memcpy(target->sim_output_offset_prev, target->sim_offset,
           sizeof(target->sim_output_offset_prev));
    target->sim_output_velocity_initialized = 1;
}

/* Observe the real engine pivot before integration, then compare it with the
   previous final solver/visual output. A live pivot read is not assumed to
   update synchronously after a matrix write. This is diagnostic only. */
static void addon_chain_trace_contact_motion(
    physx_sidecar_t *sc, physx_chain_t *chain, DWORD now, int after_solve)
{
    int person, i, axis;
    body_chain_collider_person_state_t *state;
    if (!defaults_cfg.debug || !chain->addon_chain || !chain->collision_enabled ||
        !chain->skinned_matrix_enabled) return;
    person = addon_chain_collision_person_index(sc, chain);
    if (person < 0 || person >= 4) return;
    state = &body_chain_collider_states[person];
    if (!state->ready || !state->basis_valid) return;
    for (i = 1; i < chain->target_count; i++) {
        physx_target_t *target = &chain->targets[i];
        if (!target->addon_simulated_target) continue;
        if (after_solve) {
            if (target->addon_visual_tick != now) {
                target->contact_trace_valid = 0;
                continue;
            }
            if (target->contact_trace_valid &&
                now - target->contact_trace_tick <= 120u &&
                target->contact_trace_log_tick == now) {
                float change[3];
                for (axis = 0; axis < 3; axis++)
                    change[axis] = target->sim_offset[axis] - target->contact_trace_solver[axis];
                log_line("addon contact motion final chain=\"%s\" target=\"%s\" corrected=%d solver_delta=(%.7f,%.7f,%.7f) velocity=(%.7f,%.7f,%.7f) published_rotation=(%.5f,%.5f,%.5f)",
                    chain->name, target->name, target->sim_contact_corrected,
                    change[0],change[1],change[2],
                    target->sim_velocity[0],target->sim_velocity[1],target->sim_velocity[2],
                    target->addon_visual_rotation[0],target->addon_visual_rotation[1],target->addon_visual_rotation[2]);
            }
            memcpy(target->contact_trace_solver,target->sim_offset,sizeof(target->sim_offset));
            memcpy(target->contact_trace_rotation,target->addon_visual_rotation,sizeof(target->addon_visual_rotation));
            target->contact_trace_tick = now;
        } else {
            float pivot[3], motion[3];
            if (!addon_chain_target_body_local(sc,chain,target,person,state,pivot)) {
                target->contact_trace_valid = 0;
                continue;
            }
            if (target->contact_trace_valid && now-target->contact_trace_tick<=120u &&
                (!target->contact_trace_log_tick || now-target->contact_trace_log_tick>=250u)) {
                for(axis=0;axis<3;axis++) motion[axis]=pivot[axis]-target->contact_trace_pivot[axis];
                target->contact_trace_log_tick=now;
                log_line("addon contact motion begin chain=\"%s\" target=\"%s\" person=%d dt_ms=%lu pivot=(%.7f,%.7f,%.7f) live_step=(%.7f,%.7f,%.7f) previous_rotation=(%.5f,%.5f,%.5f)",
                    chain->name,target->name,person+1,(unsigned long)(now-target->contact_trace_tick),
                    pivot[0],pivot[1],pivot[2],motion[0],motion[1],motion[2],
                    target->contact_trace_rotation[0],target->contact_trace_rotation[1],target->contact_trace_rotation[2]);
            }
            memcpy(target->contact_trace_pivot,pivot,sizeof(pivot));
            target->contact_trace_valid=1;
        }
    }
}

static void run_chain_simulations(DWORD now)
{
    int i, c, t, axis;
    float dt;
    if (!last_sim_tick) {
        last_sim_tick = now;
        return;
    }
    dt = (float)(now - last_sim_tick) / 1000.0f;
    last_sim_tick = now;
    if (dt <= 0.0f) return;
    if (dt > 0.05f) dt = 0.05f;
    for (i = 0; i < sidecar_count; i++) {
        physx_sidecar_t *sc = &sidecars[i];
        int live_owner_known[4] = { 0, 0, 0, 0 };
        int live_owner_value[4] = { 0, 0, 0, 0 };
        if (!sc->enabled || sc->write_test) continue;
        for (c = 0; c < sc->chain_count; c++) {
            physx_chain_t *chain = &sc->chains[c];
            LONGLONG addon_activation_start = physx_perf_counter();
            if (chain->addon_chain && chain->addon_owner_person[0]) {
                int owner_index = addon_person_prefix_to_index(
                    chain->addon_owner_person);
                int owner_live;
                lstrcpynA(sc->addon_owner_person,
                          chain->addon_owner_person,
                          sizeof(sc->addon_owner_person));
                if (owner_index >= 0 && owner_index < 4) {
                    if (!live_owner_known[owner_index]) {
                        live_owner_value[owner_index] =
                            sidecar_owner_has_live_addon_root(
                                sc, chain->addon_owner_person);
                        live_owner_known[owner_index] = 1;
                    }
                    owner_live = live_owner_value[owner_index];
                } else {
                    owner_live = sidecar_owner_has_live_addon_root(
                        sc, chain->addon_owner_person);
                }
                if (!owner_live) {
                    if (chain->addon_scene_visible) {
                        addon_chain_reset_runtime_state(chain);
                        chain->addon_root_settle_until_tick = 0;
                        chain->addon_scene_visible = 0;
                    }
                    physx_perf_add(PHYSX_PERF_ADDON_ACTIVATION,
                                   addon_activation_start);
                    continue;
                }
            }
            float k, d;
            int object_transform = chain->object_transform_chain;
            int rigid = chain->rigid_simulation;
            int inertial = chain->inertial_simulation;
            int has_anchor = 0;
            int start_index = 1;
            int end_index = chain->target_count;
            int addon_hidden_root_driver = 0;
            int addon_root_drive_untrusted = 0;
            int addon_scene_visible_now = 0;
            int addon_runtime_scene_fallback = 0;
            int addon_runtime_live_targets = 0;
            int addon_runtime_writable_targets = 0;
            float addon_world_gravity_drive[3] = { 0.0f, 0.0f, 0.0f };
            int addon_world_gravity_valid = 0;
            int addon_world_gravity_sampled = 0;
            float addon_world_wind_drive[3] = { 0.0f, 0.0f, 0.0f };
            int addon_world_wind_valid = 0;
            int addon_world_wind_sampled = 0;
            int addon_gravity_parent_rotation_camera_safe = 0;
            float addon_collision_points[32][3];
            int addon_collision_point_valid[32];
            int addon_collision_body_person_ready[4];
            int addon_collision_body_ready = 0;
            int addon_collision_person_index = -1;
            int addon_collision_iterations = 1;
            int addon_stationary_camera_hold = 0;
            if (chain->addon_chain) {
                if (!sc->room_scene_sidecar) {
                    addon_chain_note_live_root_event(sc, chain, now);
                }
                addon_scene_visible_now = addon_chain_scene_visible(
                    sc, chain, now,
                    &addon_runtime_scene_fallback,
                    &addon_runtime_live_targets,
                    &addon_runtime_writable_targets);
                if (addon_scene_visible_now <= 0) {
                    if (chain->addon_scene_visible) {
                        addon_chain_reset_runtime_state(chain);
                        chain->addon_root_settle_until_tick = 0;
                        log_line("addon-chain deactivated for owner load transition chain=\"%s\" owner=\"%s\" sidecar=\"%s\" note=\"cleared cached visual output and early bindings before TK17 clones/rebuilds the room add-on\"",
                                 chain->name,
                                 sc->addon_owner_person,
                                 sc->path);
                    }
                    chain->addon_scene_visible = 0;
                    physx_perf_add(PHYSX_PERF_ADDON_ACTIVATION,
                                   addon_activation_start);
                    continue;
                }
                if (!chain->addon_scene_visible) {
                    chain->addon_scene_visible = 1;
                    if (!addon_chain_settling(chain, now)) {
                        if (addon_runtime_scene_fallback) {
                            /* The fallback has just proved these exact live
                               targets writable. Clearing them here creates a
                               circular wait in modes that do not emit another
                               dress/clone event after initial room load. Keep
                               the bindings and only give TK17's transforms a
                               controlled settle period before simulation. */
                            chain->addon_root_seen_tick = now;
                            chain->addon_root_settle_until_tick = now + 1200u;
                            chain->addon_live_layout_pending = 1;
                            chain->addon_live_layout_retry_tick = now + 1200u;
                            log_line("addon-chain verified-target settle scheduled chain=\"%s\" reason=\"mode-independent-owner-ready\" live_targets=%d writable_targets=%d settle_ms=1200 sidecar=\"%s\" note=\"preserved verified live bindings; non-PoseEditor initial loads do not necessarily emit a second dress event\"",
                                     chain->name,
                                     addon_runtime_live_targets,
                                     addon_runtime_writable_targets,
                                     sc->path);
                        } else {
                            addon_chain_schedule_runtime_rebind(
                                sc, chain, now, "room-visible", 1200u);
                        }
                    }
                    if (addon_runtime_scene_fallback) {
                        log_line("addon-chain mode-independent activation chain=\"%s\" poseedit_visibility=-1 live_targets=%d writable_targets=%d owner=\"%s\" owner_body_ready=1 settle_ms=1200 sidecar=\"%s\" note=\"the live add-on and its non-placeholder owner body are ready; verified target bindings are retained through settle\"",
                                 chain->name,
                                 addon_runtime_live_targets,
                                 addon_runtime_writable_targets,
                                 sc->addon_owner_person,
                                 sc->path);
                    }
                }
                if (addon_chain_settling(chain, now)) {
                    physx_perf_add(PHYSX_PERF_ADDON_ACTIVATION,
                                   addon_activation_start);
                    continue;
                }
                if (object_transform) {
                    /* Native object rotation has no SJoint traversal layout
                       to promote and no skin palette to discover. */
                    chain->addon_live_layout_pending = 0;
                    chain->addon_live_layout_retry_tick = 0;
                } else if (sc->room_scene_sidecar) {
                    /* Room bones expose their stable scene Rotation and
                       Translation slots at 0x06c/0x07c. They are not skinned
                       PersonXX add-ons and therefore have no 0x038/0x048
                       live skin-matrix layout to promote. */
                    chain->addon_live_layout_pending = 0;
                    chain->addon_live_layout_retry_tick = 0;
                } else {
                    if (!addon_chain_promote_live_sjoint_layout(sc, chain,
                                                                now)) {
                        physx_perf_add(PHYSX_PERF_ADDON_ACTIVATION,
                                       addon_activation_start);
                        continue;
                    }
                    probe_addon_skin_geometry(sc, chain, now);
                }
            } else {
                diagnose_chain_attachment(chain);
                probe_addon_skin_geometry(sc, chain, now);
            }
            if (chain->addon_chain && !sc->room_scene_sidecar &&
                chain->parent_name[0] &&
                chain->target_count > 1 &&
                _stricmp(chain->targets[0].name, chain->parent_name) == 0 &&
                _stricmp(chain->targets[1].name, chain->name) == 0) {
                addon_hidden_root_driver = 1;
                addon_root_drive_untrusted =
                    addon_chain_root_drive_camera_untrusted(chain, now);
            }
            if (!chain->simulate || chain->target_count <= start_index) {
                physx_perf_add(PHYSX_PERF_ADDON_ACTIVATION,
                               addon_activation_start);
                continue;
            }
            if (end_index <= start_index) {
                physx_perf_add(PHYSX_PERF_ADDON_ACTIVATION,
                               addon_activation_start);
                continue;
            }
            memset(addon_collision_point_valid, 0,
                   sizeof(addon_collision_point_valid));
            memset(addon_collision_body_person_ready, 0,
                   sizeof(addon_collision_body_person_ready));
            k = chain->stiffness * 18.0f;
            d = chain->damping * 8.0f;
            if (chain->addon_chain &&
                (chain->skinned_matrix_enabled || object_transform)) {
                k = chain->stiffness;
                d = chain->damping;
            }
            if (chain->addon_chain) {
                if (chain->collision_enabled &&
                    (chain->collision_scope &
                     (PHYSX_COLLISION_SCOPE_BODY |
                      PHYSX_COLLISION_SCOPE_BODY_ALL |
                      PHYSX_COLLISION_SCOPE_SELF |
                      PHYSX_COLLISION_SCOPE_ADDONS |
                      PHYSX_COLLISION_SCOPE_ROOM))) {
                    int collider_person_index;
                    addon_collision_person_index =
                        addon_chain_collision_person_index(sc, chain);
                    addon_collision_body_ready =
                        addon_chain_body_collider_person_ready(
                            chain, addon_collision_person_index, now);
                    if (addon_collision_body_ready &&
                        addon_collision_person_index >= 0) {
                        addon_collision_body_person_ready
                            [addon_collision_person_index] = 1;
                        if (chain->collision_scope &
                            PHYSX_COLLISION_SCOPE_BODY_ALL) {
                            for (collider_person_index = 0;
                                 collider_person_index < 4;
                                 collider_person_index++) {
                                if (collider_person_index ==
                                    addon_collision_person_index) {
                                    continue;
                                }
                                if (addon_chain_body_collider_person_ready(
                                        chain, collider_person_index, now)) {
                                    addon_collision_body_person_ready
                                        [collider_person_index] = 1;
                                }
                            }
                        }
                        body_profile_set_active_person_config(
                            addon_collision_person_index);
                        addon_collision_iterations =
                            body_chain_collider_cfg.collision_iterations;
                        body_profile_set_active_person_config(-1);
                        if (addon_collision_iterations < 1) {
                            addon_collision_iterations = 1;
                        }
                        if (addon_collision_iterations > 3) {
                            addon_collision_iterations = 3;
                        }
                    }
                }
            }
            physx_perf_add(PHYSX_PERF_ADDON_ACTIVATION,
                           addon_activation_start);
            addon_chain_trace_contact_motion(sc, chain, now, 0);
            addon_chain_capture_contact_basis(sc,chain,now);
            for (t = start_index; t < end_index; t++) {
                physx_target_t *parent = t > 0 ? &chain->targets[t - 1] : NULL;
                physx_target_t *target = &chain->targets[t];
                LONGLONG addon_drive_start = physx_perf_counter();
                float *pv;
                float *v;
                float *rv = NULL;
                float *addon_rv = NULL;
                float *wv = NULL;
                float anchor_view[3] = { 0.0f, 0.0f, 0.0f };
                float anchor_step[3] = { 0.0f, 0.0f, 0.0f };
                float parent_translation_step[3] = { 0.0f, 0.0f, 0.0f };
                float parent_rotation_step[3] = { 0.0f, 0.0f, 0.0f };
                target->sim_contact_corrected = 0;
                if (chain->addon_chain) {
                    LONGLONG addon_guard_start = physx_perf_counter();
                    int addon_guard_ready = object_transform ?
                        addon_object_target_sample_write_guard(target, now) :
                        addon_target_sample_write_guard(
                            target, now, sc->room_scene_sidecar, sc->path);
                    physx_perf_add(PHYSX_PERF_ADDON_GUARD,
                                   addon_guard_start);
                    if (!addon_guard_ready) {
                        physx_perf_add(PHYSX_PERF_ADDON_DRIVE,
                                       addon_drive_start);
                        continue;
                    }
                }
                float parent_delta[3];
                int addon_root_parent_missing = 0;
                int addon_hidden_root_target = addon_hidden_root_driver && t == 1;
                int addon_root_anchor_drive = 0;
                int addon_parent_translation_drive = 0;
                int addon_parent_rotation_drive = 0;
                int addon_parent_translation_camera_safe = 0;
                if (!object_transform &&
                    parent && chain->addon_chain && t == start_index &&
                    !parent->addon_simulated_target &&
                    (!parent->s_translation_base || parent->s_translation_offset < 0)) {
                    addon_root_parent_missing = 1;
                }
                if (!object_transform &&
                    parent && !addon_root_parent_missing &&
                    (!parent->s_translation_base || parent->s_translation_offset < 0)) {
                    physx_perf_add(PHYSX_PERF_ADDON_DRIVE,
                                   addon_drive_start);
                    continue;
                }
                if (!object_transform &&
                    (!target->s_translation_base ||
                     target->s_translation_offset < 0)) {
                    physx_perf_add(PHYSX_PERF_ADDON_DRIVE,
                                   addon_drive_start);
                    continue;
                }
                if (object_transform) {
                    pv = parent ? parent->object_proxy_translation : NULL;
                    v = target->object_proxy_translation;
                } else {
                    pv = (parent && !addon_root_parent_missing) ?
                         (float*)((BYTE*)parent->s_translation_base +
                                  parent->s_translation_offset) : NULL;
                    v = (float*)((BYTE*)target->s_translation_base +
                                 target->s_translation_offset);
                }
                if (rigid && addon_hidden_root_target && parent) {
                    LONGLONG addon_parent_pivot_start = physx_perf_counter();
                    int addon_parent_pivot_ready =
                        resolve_addon_chain_parent_pivot(chain, parent,
                                                         anchor_view);
                    physx_perf_add(PHYSX_PERF_ADDON_PARENT_PIVOT,
                                   addon_parent_pivot_start);
                    if (addon_parent_pivot_ready) {
                        if (addon_root_drive_untrusted) {
                            if (chain->anchor_initialized) {
                                chain->anchor_rest[0] = anchor_view[0];
                                chain->anchor_rest[1] = anchor_view[1];
                                chain->anchor_rest[2] = anchor_view[2];
                            }
                            if (target->sim_world_anchor_initialized) {
                                target->sim_world_parent_rest[0] =
                                    anchor_view[0];
                                target->sim_world_parent_rest[1] =
                                    anchor_view[1];
                                target->sim_world_parent_rest[2] =
                                    anchor_view[2];
                            }
                        } else {
                            wv = anchor_view;
                            addon_root_anchor_drive = 1;
                        }
                    }
                } else if (rigid && has_anchor && t == 0) {
                    wv = resolve_chain_anchor_vector(chain);
                }
                if (rigid && addon_hidden_root_target && parent) {
                    /* Object and bone chains use the same parent-motion
                       inputs. Only their final output writer differs. */
                    {
                        LONGLONG addon_parent_translation_start =
                            physx_perf_counter();
                        addon_parent_translation_drive =
                        resolve_addon_chain_effective_parent_translation_step(
                            sc, chain, parent, parent_translation_step, now);
                        physx_perf_add(PHYSX_PERF_ADDON_PARENT_TRANSLATION,
                                       addon_parent_translation_start);
                    }
                    addon_parent_translation_camera_safe =
                        addon_parent_translation_camera_safe_for_root(chain);
                    {
                        LONGLONG addon_parent_rotation_start =
                            physx_perf_counter();
                        addon_parent_rotation_drive =
                        resolve_addon_chain_parent_rotation_step(
                            sc, chain, parent, parent_rotation_step, now);
                        physx_perf_add(PHYSX_PERF_ADDON_PARENT_ROTATION,
                                       addon_parent_rotation_start);
                    }
                    addon_gravity_parent_rotation_camera_safe =
                        addon_parent_rotation_drive &&
                        chain->addon_parent_rotation_camera_relative;
                    if (addon_root_drive_untrusted) {
                        if (addon_parent_translation_drive &&
                            !addon_parent_translation_camera_safe) {
                            addon_parent_translation_drive = 0;
                            parent_translation_step[0] = 0.0f;
                            parent_translation_step[1] = 0.0f;
                            parent_translation_step[2] = 0.0f;
                        }
                        if (addon_parent_rotation_drive &&
                            !chain->addon_parent_rotation_camera_relative) {
                            addon_parent_rotation_drive = 0;
                            parent_rotation_step[0] = 0.0f;
                            parent_rotation_step[1] = 0.0f;
                            parent_rotation_step[2] = 0.0f;
                        }
                    }
                    {
                        float translation_len =
                            addon_parent_translation_drive ?
                            addon_vec3_len_exact(parent_translation_step) :
                            0.0f;
                        float rotation_len =
                            addon_parent_rotation_drive ?
                            addon_vec3_len_exact(parent_rotation_step) :
                            0.0f;
                        int meaningful_parent_motion =
                            translation_len > 0.00020f ||
                            rotation_len > 0.03000f;
                        int camera_guard_active =
                            addon_root_drive_untrusted ||
                            chain->addon_gravity_camera_hold_active ||
                            chain->addon_gravity_camera_release_active;

                        if (meaningful_parent_motion) {
                            chain->addon_stationary_parent_motion_tick = now;
                        }
                        if (camera_guard_active) {
                            DWORD motion_age =
                                chain->addon_stationary_parent_motion_tick ?
                                now - chain->addon_stationary_parent_motion_tick :
                                0xffffffffu;
                            chain->addon_stationary_camera_hold_active =
                                !meaningful_parent_motion && motion_age > 120u;
                        } else {
                            chain->addon_stationary_camera_hold_active = 0;
                        }
                        addon_stationary_camera_hold =
                            chain->addon_stationary_camera_hold_active;
                        if (addon_stationary_camera_hold) {
                            addon_parent_translation_drive = 0;
                            addon_parent_rotation_drive = 0;
                            parent_translation_step[0] = 0.0f;
                            parent_translation_step[1] = 0.0f;
                            parent_translation_step[2] = 0.0f;
                            parent_rotation_step[0] = 0.0f;
                            parent_rotation_step[1] = 0.0f;
                            parent_rotation_step[2] = 0.0f;
                        }
                    }
                }
                if (chain->addon_chain &&
                    !addon_world_gravity_sampled) {
                    LONGLONG addon_gravity_start = physx_perf_counter();
                    addon_world_gravity_valid =
                        addon_chain_body_gravity_drive(
                            sc, chain, now,
                            addon_gravity_parent_rotation_camera_safe,
                            addon_world_gravity_drive);
                    physx_perf_add(PHYSX_PERF_ADDON_GRAVITY,
                                   addon_gravity_start);
                    addon_world_gravity_sampled = 1;
                    if (chain->addon_owner_person[0] && chain->gravity_enabled &&
                        gravity_response_trace_due(now, &chain->addon_gravity_camera_log_tick)) {
                        log_line("gravity responsiveness sidecar chain=%s owner=%s valid=%d parent=%s parent_raw=%p camera=%ld rotation=%ld held=%d release=%d trusted=%d drive=(%.5f,%.5f,%.5f) sample_reason=%d sample_counts=(accepted=%lu,camera=%lu,waiting=%lu)",
                            chain->name, chain->addon_owner_person, addon_world_gravity_valid,
                            chain->target_count ? chain->targets[0].name : "",
                            chain->target_count ? chain->targets[0].raw_object : NULL,
                            captured_camera_version,captured_camera_rotation_version,
                            chain->addon_gravity_camera_hold_active,chain->addon_gravity_camera_release_active,
                            chain->addon_gravity_trusted_valid,
                            addon_world_gravity_drive[0],addon_world_gravity_drive[1],addon_world_gravity_drive[2],
                            chain->addon_gravity_sample.reason,(unsigned long)chain->addon_gravity_sample.accepted_count,
                            (unsigned long)chain->addon_gravity_sample.camera_count,(unsigned long)chain->addon_gravity_sample.waiting_count);
                    }
                }
                if (chain->addon_chain &&
                    !addon_world_wind_sampled) {
                    addon_world_wind_valid = addon_chain_parent_wind_drive(
                        sc, chain, now, addon_world_wind_drive);
                    addon_world_wind_sampled = 1;
                }
                if (inertial) wv = resolve_inertial_local_drive_vector(chain);
                {
                    LONGLONG addon_rotation_binding_start =
                        physx_perf_counter();
                if (object_transform) {
                    rv = target->object_output_rotation;
                } else if (target->s_rotation_base &&
                           target->s_rotation_offset >= 0) {
                    rv = (float*)((BYTE*)target->s_rotation_base + target->s_rotation_offset);
                    if (!ptr_readable(rv, sizeof(float) * 3)) rv = NULL;
                }
                if (!object_transform && chain->addon_chain &&
                    (!target->addon_rotation_base ||
                     target->addon_rotation_offset == -1) &&
                    target->addon_simulated_target) {
                    assume_addon_t_joint_rotation_layout(target, sc->path);
                }
                addon_rv = object_transform ? NULL :
                           addon_tjoint_rotation_ptr(target);
                if (chain->addon_chain && target->addon_joint_orientation_valid) {
                    addon_rv = NULL;
                }
                if (!object_transform && chain->addon_chain &&
                    target->addon_simulated_target &&
                    !target->addon_joint_orientation_valid) {
                    rv = NULL;
                }
                    physx_perf_add(PHYSX_PERF_ADDON_ROTATION_BINDING,
                                   addon_rotation_binding_start);
                }
                {
                    LONGLONG addon_validation_start = physx_perf_counter();
                if (!addon_target_transform_pointers_readable(
                        target, pv, v, now)) {
                    physx_perf_add(PHYSX_PERF_ADDON_VALIDATION,
                                   addon_validation_start);
                    physx_perf_add(PHYSX_PERF_ADDON_DRIVE,
                                   addon_drive_start);
                    continue;
                }
                if (chain->addon_chain) {
                    /* A wearable add-on chain root is a local bone offset,
                       but a room chain root stores its absolute room position.
                       Do not reject legitimate room coordinates such as the
                       NcRoom6 palm roots merely because they exceed the local
                       eight-unit bone sanity limit. Child offsets keep the
                       stricter validation below. */
                    if (pv && !physx_vec3_sane_limit(
                            pv,
                            (sc->room_scene_sidecar &&
                             parent == &chain->targets[0]) ?
                                4096.0f : 8.0f)) {
                        log_line("addon-chain stale parent transform rejected chain=\"%s\" parent=\"%s\" target=\"%s\" parent_base=%p parent_offset=0x%03x parent_current=(%.5f,%.5f,%.5f) sidecar=\"%s\" note=\"mapped pointer contained impossible bone translation after add-on scene changed; clearing cached parent/target bindings before any write\"",
                                 chain->name,
                                 parent ? parent->name : "",
                                 target->name,
                                 parent ? parent->s_translation_base : NULL,
                                 parent ? parent->s_translation_offset : -1,
                                 pv[0], pv[1], pv[2], sc->path);
                        invalidate_addon_target_binding(parent);
                        invalidate_addon_target_binding(target);
                        physx_perf_add(PHYSX_PERF_ADDON_VALIDATION,
                                       addon_validation_start);
                        physx_perf_add(PHYSX_PERF_ADDON_DRIVE,
                                       addon_drive_start);
                        continue;
                    }
                    if (!physx_vec3_sane_limit(v, 8.0f)) {
                        log_line("addon-chain stale target transform rejected chain=\"%s\" target=\"%s\" target_base=%p target_offset=0x%03x target_current=(%.5f,%.5f,%.5f) sidecar=\"%s\" note=\"mapped pointer contained impossible bone translation after add-on scene changed; clearing cached target binding before any write\"",
                                 chain->name, target->name,
                                 target->s_translation_base, target->s_translation_offset,
                                 v[0], v[1], v[2], sc->path);
                        invalidate_addon_target_binding(target);
                        physx_perf_add(PHYSX_PERF_ADDON_VALIDATION,
                                       addon_validation_start);
                        physx_perf_add(PHYSX_PERF_ADDON_DRIVE,
                                       addon_drive_start);
                        continue;
                    }
                    if (rv && !physx_vec3_sane_limit(rv, 720.0f)) {
                        rv = NULL;
                    }
                    if (addon_rv && !physx_vec3_sane_limit(addon_rv, 720.0f)) {
                        addon_rv = NULL;
                    }
                }
                    physx_perf_add(PHYSX_PERF_ADDON_VALIDATION,
                                   addon_validation_start);
                }
                physx_perf_add(PHYSX_PERF_ADDON_DRIVE,
                               addon_drive_start);
                {
                    LONGLONG addon_solver_start = physx_perf_counter();
                    if (!target->sim_initialized) {
                        target->sim_initialized = 1;
                        target->sim_start_tick = now;
                        target->sim_parent_rest[0] = pv ? pv[0] : 0.0f;
                    target->sim_parent_rest[1] = pv ? pv[1] : 0.0f;
                    target->sim_parent_rest[2] = pv ? pv[2] : 0.0f;
                    if (wv) {
                        target->sim_world_anchor_initialized = 1;
                        target->sim_world_parent_rest[0] = wv[0];
                        target->sim_world_parent_rest[1] = wv[1];
                        target->sim_world_parent_rest[2] = wv[2];
                    }
                    target->sim_rest[0] = v[0];
                    target->sim_rest[1] = v[1];
                    target->sim_rest[2] = v[2];
                    if (addon_rv) {
                        capture_addon_tjoint_rotation_rest(target, addon_rv);
                    }
                    if (chain->addon_chain &&
                        target->addon_joint_orientation_valid) {
                        target->sim_rotation_rest[0] = target->addon_joint_orientation[0];
                        target->sim_rotation_rest[1] = target->addon_joint_orientation[1];
                        target->sim_rotation_rest[2] = target->addon_joint_orientation[2];
                    } else if (rv || addon_rv) {
                        float *rest_rv = rv ? rv : addon_rv;
                        target->sim_rotation_rest[0] = rest_rv[0];
                        target->sim_rotation_rest[1] = rest_rv[1];
                        target->sim_rotation_rest[2] = rest_rv[2];
                    }
                    target->sim_length = physx_vec3_len(target->sim_rest);
                    target->sim_offset[0] = rigid ? target->sim_rest[0] : 0.0f;
                    target->sim_offset[1] = rigid ? target->sim_rest[1] : 0.0f;
                    target->sim_offset[2] = rigid ? target->sim_rest[2] : 0.0f;
                    target->sim_output_velocity_initialized = 1;
                    memcpy(target->sim_output_offset_prev,
                           target->sim_offset,
                           sizeof(target->sim_output_offset_prev));
                    target->sim_output_velocity[0] = 0.0f;
                    target->sim_output_velocity[1] = 0.0f;
                    target->sim_output_velocity[2] = 0.0f;
                    if (inertial) {
                        target->sim_velocity[0] = 0.0f;
                        target->sim_velocity[1] = 0.0f;
                        target->sim_velocity[2] = 0.0f;
                    } else if (rigid) {
                        float impulse = chain->startup_impulse;
                        target->sim_velocity[0] = impulse;
                        target->sim_velocity[1] = 0.0f;
                        target->sim_velocity[2] = impulse * 0.35f;
                    } else {
                        target->sim_velocity[0] = chain->gravity[0] * chain->startup_impulse;
                        target->sim_velocity[1] = chain->gravity[1] * chain->startup_impulse;
                        target->sim_velocity[2] = chain->gravity[2] * chain->startup_impulse;
                    }
                    if (!target->sim_started_logged) {
                        target->sim_started_logged = 1;
                        if (chain->addon_chain && t == start_index) {
                            if (defaults_cfg.debug) {
                                log_line("============================================================");
                                log_line("================= ADDON PHYSX SIM ACTIVE ===================");
                                log_line("ADDON PHYSX SIM ACTIVE chain=\"%s\" target=\"%s\" output=\"SJoint matrix basis rows\" startup_impulse=%.3f gravity=(%.3f,%.3f,%.3f) world_gravity_valid=%d world_gravity_drive=(%.3f,%.3f,%.3f) note=\"this is the real add-on chain solver, not the forced output swing test\"",
                                         chain->name,
                                         target->name,
                                         chain->startup_impulse,
                                         chain->gravity[0],
                                         chain->gravity[1],
                                         chain->gravity[2],
                                         addon_world_gravity_valid,
                                         addon_world_gravity_drive[0],
                                         addon_world_gravity_drive[1],
                                         addon_world_gravity_drive[2]);
                                log_line("============================================================");
                            } else {
                                log_line("addon physics active owner=\"%s\" chain=\"%s\" targets=%d sidecar=\"%s\"",
                                         chain->addon_owner_person[0] ?
                                             chain->addon_owner_person : "unknown",
                                         chain->name,
                                         chain->target_count,
                                         sc->path);
                            }
                        }
                        log_line("%s sim started chain=\"%s\" parent=\"%s\" target=\"%s\" source=%s base=%p offset=0x%03x parent_rest=(%.5f,%.5f,%.5f) rest=(%.5f,%.5f,%.5f) length=%.5f stiffness=%.3f damping=%.3f gravity_scale=%.3f max_offset=%.3f impulse=%.3f sidecar=\"%s\"",
                                 inertial ? "inertial-chain" : (rigid ? "rigid-chain" : "spring"),
                                 chain->name, parent ? parent->name : chain->anchor_name, target->name,
                                 target->s_translation_source ? target->s_translation_source : "unknown",
                                 target->s_translation_base, target->s_translation_offset,
                                 target->sim_parent_rest[0], target->sim_parent_rest[1], target->sim_parent_rest[2],
                                 target->sim_rest[0], target->sim_rest[1], target->sim_rest[2],
                                 target->sim_length,
                                 chain->stiffness, chain->damping, chain->gravity_scale,
                                 chain->max_offset, chain->startup_impulse, sc->path);
                        if (rigid && (rv || addon_rv)) {
                            log_line("rigid-chain rotation enabled chain=\"%s\" target=\"%s\" source=%s base=%p offset=0x%03x addon_source=%s addon_base=%p addon_offset=0x%03x rest_rotation=(%.5f,%.5f,%.5f) addon_rest_valid=%d addon_rest=(%.5f,%.5f,%.5f)",
                                     chain->name, target->name,
                                     target->s_rotation_source ? target->s_rotation_source : "unknown",
                                     target->s_rotation_base, target->s_rotation_offset,
                                     target->addon_rotation_source ? target->addon_rotation_source : "",
                                     target->addon_rotation_base, target->addon_rotation_offset,
                                     target->sim_rotation_rest[0], target->sim_rotation_rest[1], target->sim_rotation_rest[2],
                                     target->sim_addon_rotation_rest_valid,
                                     target->sim_addon_rotation_rest[0],
                                     target->sim_addon_rotation_rest[1],
                                     target->sim_addon_rotation_rest[2]);
                        }
                        if (rigid && target->sim_world_anchor_initialized) {
                            log_line("rigid-chain world anchor enabled chain=\"%s\" target=\"%s\" anchor=\"%s\" source=raw offset=0x%03x rest=(%.5f,%.5f,%.5f)",
                                     chain->name, target->name, chain->anchor_name, chain->anchor_offset,
                                     target->sim_world_parent_rest[0],
                                     target->sim_world_parent_rest[1],
                                     target->sim_world_parent_rest[2]);
                        }
                    }
                }
                if (inertial && wv && !chain->anchor_initialized) {
                    chain->anchor_initialized = 1;
                    chain->anchor_rest[0] = wv[0];
                    chain->anchor_rest[1] = wv[1];
                    chain->anchor_rest[2] = wv[2];
                    log_line("inertial-chain attach baseline chain=\"%s\" attach=\"%s\" current=(%.5f,%.5f,%.5f)",
                             chain->name, chain->attach_name[0] ? chain->attach_name : chain->anchor_name,
                             wv[0], wv[1], wv[2]);
                }
                parent_delta[0] = pv ? pv[0] - target->sim_parent_rest[0] : 0.0f;
                parent_delta[1] = pv ? pv[1] - target->sim_parent_rest[1] : 0.0f;
                parent_delta[2] = pv ? pv[2] - target->sim_parent_rest[2] : 0.0f;
                if (rigid && wv && !target->sim_world_anchor_initialized) {
                    target->sim_world_anchor_initialized = 1;
                    target->sim_world_parent_rest[0] = wv[0];
                    target->sim_world_parent_rest[1] = wv[1];
                    target->sim_world_parent_rest[2] = wv[2];
                    log_line("rigid-chain world anchor late-enabled chain=\"%s\" target=\"%s\" anchor=\"%s\" source=raw offset=0x%03x rest=(%.5f,%.5f,%.5f)",
                             chain->name, target->name, chain->anchor_name, chain->anchor_offset, wv[0], wv[1], wv[2]);
                }
                if (rigid && wv && target->sim_world_anchor_initialized) {
                    parent_delta[0] = wv[0] - target->sim_world_parent_rest[0];
                    parent_delta[1] = wv[1] - target->sim_world_parent_rest[1];
                    parent_delta[2] = wv[2] - target->sim_world_parent_rest[2];
                }
                if (addon_root_anchor_drive && wv) {
                    if (!chain->anchor_initialized) {
                        chain->anchor_initialized = 1;
                        chain->anchor_rest[0] = wv[0];
                        chain->anchor_rest[1] = wv[1];
                        chain->anchor_rest[2] = wv[2];
                        log_line("addon-chain root anchor baseline chain=\"%s\" parent=\"%s\" target=\"%s\" pivot_view=(%.5f,%.5f,%.5f) sidecar=\"%s\"",
                                 chain->name,
                                 parent ? parent->name : chain->anchor_name,
                                 target->name,
                                 wv[0], wv[1], wv[2],
                                 sc->path);
                    } else {
                        float step_limit = chain->max_offset > 0.001f ?
                                           chain->max_offset * 0.50f : 0.06f;
                        float raw_anchor_step[3];
                        float local_anchor_step[3];
                        raw_anchor_step[0] = wv[0] - chain->anchor_rest[0];
                        raw_anchor_step[1] = wv[1] - chain->anchor_rest[1];
                        raw_anchor_step[2] = wv[2] - chain->anchor_rest[2];
                        local_anchor_step[0] = raw_anchor_step[0];
                        local_anchor_step[1] = raw_anchor_step[1];
                        local_anchor_step[2] = raw_anchor_step[2];
                        anchor_step[0] = physx_clampf(local_anchor_step[0],
                                                       -step_limit, step_limit);
                        anchor_step[1] = physx_clampf(local_anchor_step[1],
                                                       -step_limit, step_limit);
                        anchor_step[2] = physx_clampf(local_anchor_step[2],
                                                       -step_limit, step_limit);
                        chain->anchor_rest[0] = wv[0];
                        chain->anchor_rest[1] = wv[1];
                        chain->anchor_rest[2] = wv[2];
                    }
                }
                if (defaults_cfg.debug &&
                    rigid &&
                    now - target->sim_motion_probe_tick >= 1000) {
                    target->sim_motion_probe_tick = now;
                    log_line("rigid-chain motion probe chain=\"%s\" parent=\"%s\" target=\"%s\" parent_current=(%.5f,%.5f,%.5f) world_anchor=(%.5f,%.5f,%.5f) parent_delta=(%.5f,%.5f,%.5f) anchor_step=(%.5f,%.5f,%.5f) parent_translation_step=(%.5f,%.5f,%.5f) parent_rotation_step=(%.5f,%.5f,%.5f) target_current=(%.5f,%.5f,%.5f)",
                             chain->name, parent ? parent->name : chain->anchor_name, target->name,
                             pv ? pv[0] : 0.0f, pv ? pv[1] : 0.0f, pv ? pv[2] : 0.0f,
                             wv ? wv[0] : 0.0f, wv ? wv[1] : 0.0f, wv ? wv[2] : 0.0f,
                             parent_delta[0], parent_delta[1], parent_delta[2],
                             anchor_step[0], anchor_step[1], anchor_step[2],
                             parent_translation_step[0], parent_translation_step[1], parent_translation_step[2],
                             parent_rotation_step[0], parent_rotation_step[1], parent_rotation_step[2],
                             v[0], v[1], v[2]);
                }
                if (rigid && has_anchor && t == 0) {
                    float follow_limit = chain->max_offset > 0.001f ? chain->max_offset : 0.12f;
                    for (axis = 0; axis < 3; axis++) {
                        float d_axis = physx_clampf(parent_delta[axis], -follow_limit, follow_limit);
                        v[axis] = target->sim_rest[axis] + d_axis;
                        target->sim_offset[axis] = d_axis;
                        target->sim_velocity[axis] = 0.0f;
                    }
                    physx_perf_add(PHYSX_PERF_ADDON_SOLVER,
                                   addon_solver_start);
                    continue;
                }
                if (inertial) {
                    float limit = chain->max_offset > 0.001f ? chain->max_offset : 0.01f;
                    float impulse_limit = chain->startup_impulse > 0.001f ? chain->startup_impulse * 0.06f : 0.02f;
                    float attach_delta[3] = { 0.0f, 0.0f, 0.0f };
                    if (wv && chain->anchor_initialized) {
                        attach_delta[0] = wv[0] - chain->anchor_rest[0];
                        attach_delta[1] = wv[1] - chain->anchor_rest[1];
                        attach_delta[2] = wv[2] - chain->anchor_rest[2];
                        chain->anchor_rest[0] = wv[0];
                        chain->anchor_rest[1] = wv[1];
                        chain->anchor_rest[2] = wv[2];
                    }
                    for (axis = 0; axis < 3; axis++) {
                        attach_delta[axis] = physx_clampf(attach_delta[axis], -impulse_limit, impulse_limit);
                    }
                    for (axis = 0; axis < 3; axis++) {
                        float accel = (-attach_delta[axis] * 10.0f)
                                      + chain->gravity[axis] * chain->gravity_scale
                                      - (k * target->sim_offset[axis])
                                      - (d * target->sim_velocity[axis]);
                        target->sim_velocity[axis] += accel * dt;
                        target->sim_offset[axis] += target->sim_velocity[axis] * dt;
                        target->sim_offset[axis] = physx_clampf(target->sim_offset[axis], -limit, limit);
                        if (target->sim_offset[axis] <= -limit || target->sim_offset[axis] >= limit) {
                            target->sim_velocity[axis] *= 0.25f;
                        }
                        v[axis] = target->sim_rest[axis] + target->sim_offset[axis];
                    }
                    if (defaults_cfg.debug &&
                        now - target->sim_motion_probe_tick >= 1000) {
                        target->sim_motion_probe_tick = now;
                        log_line("inertial-chain motion probe chain=\"%s\" attach=\"%s\" target=\"%s\" attach_delta=(%.5f,%.5f,%.5f) offset=(%.5f,%.5f,%.5f) target_current=(%.5f,%.5f,%.5f)",
                                 chain->name, chain->attach_name[0] ? chain->attach_name : chain->anchor_name,
                                 target->name,
                                 attach_delta[0], attach_delta[1], attach_delta[2],
                                 target->sim_offset[0], target->sim_offset[1], target->sim_offset[2],
                                 v[0], v[1], v[2]);
                    }
                } else if (rigid) {
                    float len;
                    float dot;
                    float inv_len;
                    int addon_local_bend =
                        chain->addon_chain &&
                        (chain->skinned_matrix_enabled || object_transform);
                    float addon_force_acceleration[3] = {0.0f, 0.0f, 0.0f};
                    float addon_force_drive[3] = { 0.0f, 0.0f, 0.0f };
                    float addon_force_bend[3] = { 0.0f, 0.0f, 0.0f };
                    int addon_force_bend_valid = 0;
                    float addon_gravity_bend[3] = { 0.0f, 0.0f, 0.0f };
                    int addon_gravity_bend_valid = 0;
                    float addon_wind_strength = 0.0f;
                    float effective_k = k;
                    float effective_d = d;
                    /* Camera quarantine filters suspect motion inputs above.
                       It must not change spring equilibrium or freeze/release
                       child poses while gravity and contacts keep running. */
                    if (chain->addon_chain && target->sim_start_tick) {
                        DWORD settle_elapsed = now - target->sim_start_tick;
                        if (settle_elapsed < 450u) {
                            for (axis = 0; axis < 3; axis++) {
                                target->sim_offset[axis] = target->sim_rest[axis];
                                v[axis] = addon_local_bend ?
                                          target->sim_rest[axis] :
                                          parent_delta[axis] + target->sim_rest[axis];
                            }
                            physx_perf_add(PHYSX_PERF_ADDON_SOLVER,
                                           addon_solver_start);
                            continue;
                        }
                        log_line("ADDON PHYSX STARTUP HOLD RELEASED chain=\"%s\" target=\"%s\" elapsed_ms=%lu velocity=(%.5f,%.5f,%.5f) output=\"SJoint matrix basis rows\" note=\"rest-pose hold ended; addon chain is now allowed to respond to parent motion, gravity, and collision\"",
                                 chain->name,
                                 target->name,
                                 (unsigned long)settle_elapsed,
                                 target->sim_velocity[0],
                                 target->sim_velocity[1],
                                 target->sim_velocity[2]);
                        target->sim_start_tick = 0;
                    }
                    if (addon_local_bend &&
                        addon_parent_rotation_drive &&
                        target->sim_length > 0.0001f) {
                        float gain = physx_clampf(chain->drive_scale / 300.0f,
                                                  0.0f, 4.0f) *
                                     physx_clampf(chain->drive_strength,
                                                  0.0f, 1.0f);
                        float rot_to_local = target->sim_length *
                                             0.017453292519943295f *
                                             gain;
                        float impulse[3] = { 0.0f, 0.0f, 0.0f };
                        addon_rotation_drive_add_impulse(
                            impulse, parent_rotation_step,
                            chain->rotation_drive_horizontal_source_axis,
                            chain->rotation_drive_horizontal_tail_axis,
                            chain->rotation_drive_horizontal_scale,
                            rot_to_local);
                        addon_rotation_drive_add_impulse(
                            impulse, parent_rotation_step,
                            chain->rotation_drive_vertical_source_axis,
                            chain->rotation_drive_vertical_tail_axis,
                            chain->rotation_drive_vertical_scale,
                            rot_to_local);
                        addon_rotation_drive_add_impulse(
                            impulse, parent_rotation_step,
                            chain->rotation_drive_twist_source_axis,
                            chain->rotation_drive_twist_tail_axis,
                            chain->rotation_drive_twist_scale,
                            rot_to_local);
                        for (axis = 0; axis < 3; axis++) {
                            target->sim_velocity[axis] += impulse[axis] / dt;
                        }
                    }
                    if (addon_local_bend) {
                        if (addon_world_gravity_valid) {
                            /* Retain the gravity-only sample for the existing
                               diagnostic report; simulation uses the combined
                               resultant below. */
                            addon_gravity_bend_valid =
                                addon_chain_world_gravity_bend_vector(
                                    chain, target,
                                    addon_world_gravity_drive,
                                    addon_gravity_bend);
                            if (target->gravity_inverted_configured) {
                                addon_gravity_bend_valid =
                                    addon_chain_apply_inverted_gravity_bend(
                                        chain, target,
                                        addon_world_gravity_drive,
                                        1.0f,
                                        addon_gravity_bend);
                            }
                            for (axis = 0; axis < 3; axis++) {
                                addon_force_drive[axis] +=
                                    addon_world_gravity_drive[axis] *
                                    chain->gravity_scale;
                            }
                        }
                        if (addon_world_wind_valid) {
                            addon_wind_strength = room_wind_target_strength(
                                chain, target, now);
                            for (axis = 0; axis < 3; axis++) {
                                addon_force_drive[axis] +=
                                    addon_world_wind_drive[axis] *
                                    addon_wind_strength;
                            }
                        }
                        /* Map and constrain the physical resultant once.
                           Mapping gravity and wind independently and adding
                           the two constrained bends produced incorrect
                           sideways/prone responses. */
                        if (addon_world_gravity_valid ||
                            (addon_world_wind_valid &&
                             physx_absf(addon_wind_strength) > 0.000001f)) {
                            addon_force_bend_valid =
                                addon_chain_world_gravity_bend_vector(
                                    chain, target, addon_force_drive,
                                    addon_force_bend);
                            if (addon_world_gravity_valid &&
                                target->gravity_inverted_configured) {
                                addon_force_bend_valid =
                                    addon_chain_apply_inverted_gravity_bend(
                                        chain, target,
                                        addon_world_gravity_drive,
                                        chain->gravity_scale,
                                        addon_force_bend);
                            }
                        }
                    }
                    for (axis = 0; axis < 3; axis++) {
                        if (addon_local_bend) {
                            float force_accel = 0.0f;
                            if (addon_force_bend_valid &&
                                target->sim_length > 0.0001f) {
                                force_accel = addon_force_bend[axis] *
                                    effective_k *
                                    target->sim_length;
                            }
                            if (!addon_world_gravity_valid) {
                                force_accel += chain->gravity[axis] *
                                               chain->gravity_scale;
                            }
                            addon_force_acceleration[axis] = force_accel;
                        } else {
                            target->sim_velocity[axis] += chain->gravity[axis] * chain->gravity_scale * dt;
                        }
                        if (addon_parent_translation_drive &&
                            (parent_translation_step[0] != 0.0f ||
                             parent_translation_step[1] != 0.0f ||
                             parent_translation_step[2] != 0.0f)) {
                            float gain = physx_clampf(chain->drive_scale / 300.0f,
                                                      0.0f, 4.0f) *
                                         physx_clampf(chain->drive_strength,
                                                      0.0f, 1.0f);
                            float step_limit = chain->max_offset > 0.001f ?
                                               chain->max_offset * 0.50f :
                                               0.06f;
                            float translation_step[3] = { 0.0f, 0.0f, 0.0f };
                            addon_translation_drive_add_tangent(
                                translation_step, chain, target,
                                parent_translation_step,
                                chain->translation_horizontal_source_axis,
                                chain->translation_horizontal_tail_axis,
                                addon_chain_target_gravity_tail_axis(
                                    chain, target, 0),
                                chain->translation_horizontal_scale);
                            addon_translation_drive_add_tangent(
                                translation_step, chain, target,
                                parent_translation_step,
                                chain->translation_vertical_source_axis,
                                chain->translation_vertical_tail_axis,
                                addon_chain_target_gravity_tail_axis(
                                    chain, target, 1),
                                chain->translation_vertical_scale);
                            addon_translation_drive_add_tangent(
                                translation_step, chain, target,
                                parent_translation_step,
                                chain->translation_depth_source_axis,
                                chain->translation_depth_tail_axis,
                                -1,
                                chain->translation_depth_scale);
                            float step_axis =
                                physx_clampf(translation_step[axis],
                                             -step_limit, step_limit);
                            target->sim_velocity[axis] -=
                                (step_axis * gain) / dt;
                        } else if (addon_root_anchor_drive &&
                            (anchor_step[0] != 0.0f ||
                             anchor_step[1] != 0.0f ||
                             anchor_step[2] != 0.0f)) {
                            float gain = physx_clampf(chain->drive_scale / 300.0f,
                                                      0.0f, 4.0f) *
                                         physx_clampf(chain->drive_strength,
                                                      0.0f, 1.0f);
                            target->sim_velocity[axis] -=
                                (anchor_step[axis] * gain) / dt;
                        }
                        if (!addon_local_bend) {
                            target->sim_velocity[axis] *= physx_clampf(1.0f - (effective_d * dt * 0.25f), 0.0f, 1.0f);
                            target->sim_offset[axis] += target->sim_velocity[axis] * dt;
                        }
                    }
                    if (addon_local_bend) {
                        addon_chain_integrate_free_motion(chain, target, parent,
                            t, addon_hidden_root_driver, addon_force_acceleration,
                            effective_k, effective_d, dt);
                    }
                    len = physx_vec3_len(target->sim_offset);
                    if (len > 0.0001f && target->sim_length > 0.0001f) {
                        inv_len = 1.0f / len;
                        for (axis = 0; axis < 3; axis++) {
                            target->sim_offset[axis] *= target->sim_length * inv_len;
                        }
                        dot = target->sim_velocity[0] * target->sim_offset[0] +
                              target->sim_velocity[1] * target->sim_offset[1] +
                              target->sim_velocity[2] * target->sim_offset[2];
                        dot /= (target->sim_length * target->sim_length);
                        for (axis = 0; axis < 3; axis++) {
                            target->sim_velocity[axis] -= dot * target->sim_offset[axis];
                        }
                    }
                    if (chain->addon_chain &&
                        target->room_collision_rest_pose_sleeping) {
                        LONG room_generation =
                            InterlockedCompareExchange(
                                &named_node_generation, 0, 0);
                        if (target->room_collision_rest_pose_valid &&
                            target->room_collision_rest_pose_generation ==
                                room_generation &&
                            now - target->room_collision_rest_pose_tick <=
                                180u) {
                            /* Apply the solved static-contact pose before
                               this joint is written. The terminal manifold
                               runs after upstream joints in the chain loop;
                               without this pre-output sleep, one frame of
                               spring rebound remains visible even though the
                               manifold restores the pose later that frame. */
                            memcpy(target->sim_offset,
                                   target->room_collision_rest_pose_offset,
                                   sizeof(target->sim_offset));
                            target->sim_velocity[0] = 0.0f;
                            target->sim_velocity[1] = 0.0f;
                            target->sim_velocity[2] = 0.0f;
                            addon_chain_collision_normalize_target(target);
                        } else {
                            target->room_collision_rest_pose_sleeping = 0;
                            target->room_collision_rest_pose_frames = 0;
                        }
                    }
                    if (chain->addon_chain &&
                        chain->collision_enabled &&
                        addon_local_bend &&
                        t < 32 &&
                        (chain->collision_scope &
                         (PHYSX_COLLISION_SCOPE_BODY |
                          PHYSX_COLLISION_SCOPE_BODY_ALL |
                          PHYSX_COLLISION_SCOPE_SELF |
                          PHYSX_COLLISION_SCOPE_ADDONS |
                          PHYSX_COLLISION_SCOPE_ROOM))) {
                        int collision_source_start =
                            addon_chain_collision_first_target_index(chain);
                        int collision_segment_index =
                            t - collision_source_start;
                        int hidden_parent_collision_segment =
                            addon_hidden_root_driver && t == 1;
                        if (collision_segment_index < 1) {
                            collision_segment_index = 1;
                        }
                        if (hidden_parent_collision_segment) {
                            if (addon_collision_body_ready &&
                                addon_collision_person_index >= 0) {
                                body_chain_collider_person_state_t *collision_state =
                                    &body_chain_collider_states
                                        [addon_collision_person_index];
                                if (chain->target_count == 2) {
                                    /* A one-link add-on already has a verified
                                       body parent. Derive its collision point
                                       from that parent instead of asking TK17
                                       to evaluate the simulated add-on joint's
                                       model-view pivot during traversal. */
                                    if (parent &&
                                        addon_chain_target_body_local(
                                            sc, chain, parent,
                                            addon_collision_person_index,
                                            collision_state,
                                            addon_collision_points[t - 1])) {
                                        addon_collision_points[t][0] =
                                            addon_collision_points[t - 1][0] +
                                            target->sim_offset[0];
                                        addon_collision_points[t][1] =
                                            addon_collision_points[t - 1][1] +
                                            target->sim_offset[1];
                                        addon_collision_points[t][2] =
                                            addon_collision_points[t - 1][2] +
                                            target->sim_offset[2];
                                        addon_collision_point_valid[t] = 1;
                                        addon_collision_point_valid[t - 1] = 0;
                                    }
                                } else if (addon_chain_target_body_local(
                                        sc, chain, target,
                                        addon_collision_person_index,
                                        collision_state,
                                        addon_collision_points[t])) {
                                    addon_collision_point_valid[t] = 1;
                                } else if (parent &&
                                           addon_chain_target_body_local(
                                               sc, chain, parent,
                                               addon_collision_person_index,
                                               collision_state,
                                               addon_collision_points[t - 1])) {
                                    addon_collision_points[t][0] =
                                        addon_collision_points[t - 1][0] +
                                        target->sim_offset[0];
                                    addon_collision_points[t][1] =
                                        addon_collision_points[t - 1][1] +
                                        target->sim_offset[1];
                                    addon_collision_points[t][2] =
                                        addon_collision_points[t - 1][2] +
                                        target->sim_offset[2];
                                    addon_collision_point_valid[t] = 1;
                                    addon_collision_point_valid[t - 1] = 0;
                                }
                                if (addon_collision_point_valid[t]) {
                                    int collider_person_index;
                                    int collision_iteration;
                                    for (collider_person_index = 0;
                                         collider_person_index < 4;
                                         collider_person_index++) {
                                        if (!addon_collision_body_person_ready
                                                [collider_person_index]) {
                                            continue;
                                        }
                                        for (collision_iteration = 0;
                                             collision_iteration <
                                                 addon_collision_iterations;
                                             collision_iteration++) {
                                            if (!addon_chain_apply_body_point_collision(
                                                    chain, target,
                                                    addon_collision_person_index,
                                                    collider_person_index,
                                                    addon_collision_points[t],
                                                    now, dt)) {
                                                break;
                                            }
                                        }
                                    }
                                    if (chain->collision_scope &
                                        PHYSX_COLLISION_SCOPE_ROOM) {
                                        int room_hit =
                                            addon_chain_apply_room_collision(
                                                sc, chain, target,
                                                addon_collision_person_index,
                                                (t > 0 &&
                                                 addon_collision_point_valid[t - 1]) ?
                                                    addon_collision_points[t - 1] :
                                                    addon_collision_points[t],
                                                addon_collision_points[t],
                                                NULL, 0);
                                        if (room_hit && t > 0 &&
                                            addon_collision_point_valid[t - 1]) {
                                            addon_collision_points[t][0] =
                                                addon_collision_points[t - 1][0] +
                                                target->sim_offset[0];
                                            addon_collision_points[t][1] =
                                                addon_collision_points[t - 1][1] +
                                                target->sim_offset[1];
                                            addon_collision_points[t][2] =
                                                addon_collision_points[t - 1][2] +
                                                target->sim_offset[2];
                                        }
                                    }
                                }
                            }
                        } else {
                            if (t > 0 &&
                                !addon_collision_point_valid[t - 1] &&
                                parent &&
                                addon_collision_body_ready &&
                                addon_collision_person_index >= 0) {
                                body_chain_collider_person_state_t *collision_state =
                                    &body_chain_collider_states
                                        [addon_collision_person_index];
                                if (addon_chain_target_body_local(
                                        sc, chain, parent,
                                        addon_collision_person_index,
                                        collision_state,
                                        addon_collision_points[t - 1])) {
                                    addon_collision_point_valid[t - 1] = 1;
                                }
                            }
                            if (t > 0 && addon_collision_point_valid[t - 1]) {
                                int collision_iteration;
                                int room_collision_applied = 0;
                                float body_start_override[3];
                                float body_end_override[3];
                                int body_start_override_valid = 0;
                                int body_end_override_valid = 0;
                                if (addon_collision_body_ready &&
                                    addon_collision_person_index >= 0) {
                                    body_chain_collider_person_state_t *collision_state =
                                        &body_chain_collider_states
                                            [addon_collision_person_index];
                                    if (parent &&
                                        addon_chain_target_body_local(
                                            sc, chain, parent,
                                            addon_collision_person_index,
                                            collision_state,
                                            body_start_override)) {
                                        body_start_override_valid = 1;
                                    }
                                    if (addon_chain_target_body_local(
                                            sc, chain, target,
                                            addon_collision_person_index,
                                            collision_state,
                                            body_end_override)) {
                                        float body_delta[3];
                                        float body_len;
                                        body_delta[0] = body_end_override[0] -
                                            addon_collision_points[t - 1][0];
                                        body_delta[1] = body_end_override[1] -
                                            addon_collision_points[t - 1][1];
                                        body_delta[2] = body_end_override[2] -
                                            addon_collision_points[t - 1][2];
                                        body_len = physx_vec3_len(body_delta);
                                        if (body_len >= 0.0005f &&
                                            body_len <= 1.5000f) {
                                            body_end_override_valid = 1;
                                        }
                                    }
                                }
                                for (collision_iteration = 0;
                                     collision_iteration <
                                         addon_collision_iterations;
                                     collision_iteration++) {
                                    int collision_hit = 0;
                                    if (addon_collision_body_ready &&
                                        addon_collision_person_index >= 0) {
                                        int collider_person_index;
                                        for (collider_person_index = 0;
                                             collider_person_index < 4;
                                             collider_person_index++) {
                                            if (!addon_collision_body_person_ready
                                                    [collider_person_index]) {
                                                continue;
                                            }
                                            {
                                                LONGLONG perf_start =
                                                    physx_perf_counter();
                                                int body_collision_hit =
                                                    addon_chain_apply_body_collision(
                                                    sc, chain, target,
                                                    addon_collision_person_index,
                                                    collider_person_index,
                                                    addon_collision_points[t - 1],
                                                    body_end_override_valid ?
                                                        body_end_override : NULL,
                                                    collision_segment_index,
                                                    now, dt, 0);
                                                if(t==chain->target_count-1 && target->contact_terminal_tick==now) {
                                                    float tip_end[3];
                                                    memcpy(tip_end,target->contact_terminal_body,sizeof(tip_end));
                                                    body_collision_hit |= addon_chain_apply_body_collision(
                                                        sc,chain,target,addon_collision_person_index,
                                                        collider_person_index,target->contact_pivot_body,
                                                        tip_end,collision_segment_index+1,now,dt,1);
                                                    /* Tip response can rotate an ancestor too. Refresh the
                                                       ordinary endpoint before other collision consumers. */
                                                    if(body_end_override_valid) {
                                                        physx_chain_contact_pose_t tip_pose;
                                                        if(physx_chain_contact_pose_init(chain,t-1,now,&tip_pose))
                                                            physx_chain_contact_point(chain,&tip_pose,1,body_end_override);
                                                    }
                                                }
                                                physx_perf_add(
                                                    PHYSX_PERF_ADDON_BODY_COLLISION,
                                                    perf_start);
                                                collision_hit |= body_collision_hit;
                                            }
                                        }
                                    }
                                    {
                                        LONGLONG perf_start =
                                            physx_perf_counter();
                                        int self_collision_hit =
                                            addon_chain_apply_self_collision(
                                            chain, target,
                                            addon_collision_points,
                                            addon_collision_point_valid,
                                            addon_collision_points[t - 1],
                                            t, dt);
                                        physx_perf_add(
                                            PHYSX_PERF_ADDON_SELF_COLLISION,
                                            perf_start);
                                        collision_hit |= self_collision_hit;
                                    }
                                    {
                                        LONGLONG perf_start =
                                            physx_perf_counter();
                                        int addons_collision_hit =
                                            addon_chain_apply_addons_collision(
                                            sc, chain, target,
                                            addon_collision_person_index,
                                            addon_collision_points[t - 1],
                                            collision_segment_index,
                                            now, dt);
                                        physx_perf_add(
                                            PHYSX_PERF_ADDON_ADDONS_COLLISION,
                                            perf_start);
                                        collision_hit |= addons_collision_hit;
                                    }
                                    if (!room_collision_applied &&
                                        (chain->collision_scope &
                                         PHYSX_COLLISION_SCOPE_ROOM) &&
                                        addon_collision_person_index >= 0) {
                                        float room_point[3];
                                        const float *room_end;
                                        int room_hit;
                                        room_point[0] =
                                            addon_collision_points[t - 1][0] +
                                            target->sim_offset[0];
                                        room_point[1] =
                                            addon_collision_points[t - 1][1] +
                                            target->sim_offset[1];
                                        room_point[2] =
                                            addon_collision_points[t - 1][2] +
                                            target->sim_offset[2];
                                        /* Test the room against the visible
                                           joint endpoint whenever TK17 has
                                           supplied it. The former predicted
                                           endpoint could place only part of
                                           a rotated hair capsule where the
                                           user actually saw the hair. */
                                        if(body_end_override_valid) {
                                            physx_chain_contact_pose_t room_pose;
                                            if(physx_chain_contact_pose_init(chain,t-1,now,&room_pose)) {
                                                physx_chain_contact_point(chain,&room_pose,0,body_start_override);
                                                physx_chain_contact_point(chain,&room_pose,1,body_end_override);
                                                body_start_override_valid=1;
                                            }
                                        }
                                        room_end = body_end_override_valid ?
                                            body_end_override : room_point;
                                        room_hit =
                                            addon_chain_apply_room_collision(
                                                sc, chain, target,
                                                addon_collision_person_index,
                                                body_start_override_valid ?
                                                    body_start_override :
                                                    addon_collision_points[t - 1],
                                                room_end, NULL, 0);
                                        collision_hit |= room_hit;
                                        room_collision_applied |= room_hit;
                                        if (t == chain->target_count - 1) {
                                            body_chain_collider_person_state_t *
                                                terminal_state =
                                                &body_chain_collider_states
                                                    [addon_collision_person_index];
                                            float terminal_start[3];
                                            float terminal_end[3];
                                            int terminal_hit;
                                            memcpy(terminal_start, room_end,
                                                   sizeof(terminal_start));
                                            if (addon_chain_terminal_endpoint_body_local(
                                                    chain, target,
                                                    terminal_state,
                                                    body_start_override_valid ?
                                                        body_start_override :
                                                        addon_collision_points[t - 1],
                                                    terminal_start,
                                                    terminal_end)) {
                                                terminal_hit =
                                                    addon_chain_apply_room_collision(
                                                        sc, chain, target,
                                                        addon_collision_person_index,
                                                        terminal_start,
                                                        terminal_end,
                                                        target, 1);
                                                collision_hit |= terminal_hit;
                                                room_collision_applied |=
                                                    terminal_hit;
                                            }
                                        }
                                    }
                                    if (!collision_hit) {
                                        break;
                                    }
                                }
                                if (body_end_override_valid &&
                                    !room_collision_applied) {
                                    addon_collision_points[t][0] =
                                        body_end_override[0];
                                    addon_collision_points[t][1] =
                                        body_end_override[1];
                                    addon_collision_points[t][2] =
                                        body_end_override[2];
                                } else {
                                    addon_collision_points[t][0] =
                                        addon_collision_points[t - 1][0] +
                                        target->sim_offset[0];
                                    addon_collision_points[t][1] =
                                        addon_collision_points[t - 1][1] +
                                        target->sim_offset[1];
                                    addon_collision_points[t][2] =
                                        addon_collision_points[t - 1][2] +
                                        target->sim_offset[2];
                                }
                                addon_collision_point_valid[t] = 1;
                            }
                        }
                    }
                    {
                        LONGLONG addon_output_start = physx_perf_counter();
                    for (axis = 0; axis < 3; axis++) {
                        if (addon_local_bend) {
                            v[axis] = target->sim_rest[axis];
                        } else {
                            v[axis] = parent_delta[axis] + target->sim_offset[axis];
                        }
                    }
                    /* Room-scene bones intentionally use their native
                       SSimpleTransform slots instead of the live skinned
                       add-on rotation pointers.  They still need to enter
                       the shared bend/output calculation so the native
                       room rotation write below receives the solver result. */
                    if ((rv || addon_rv || sc->room_scene_sidecar) &&
                        target->sim_length > 0.0001f) {
                        float out_r[3], pitch, roll;
                        addon_chain_contact_output_angles(chain, target, out_r,
                                                           &pitch, &roll);
                        if (addon_local_bend && chain->addon_chain) {
                            addon_gravity_diag_sample(
                                sc,
                                chain,
                                target,
                                now,
                                addon_world_gravity_valid,
                                addon_world_gravity_drive,
                                addon_gravity_bend_valid,
                                addon_gravity_bend,
                                out_r);
                        }
                        if (object_transform || sc->room_scene_sidecar ||
                            !(chain->addon_chain &&
                              chain->skinned_matrix_enabled)) {
                            if (rv && !(chain->addon_chain &&
                                         target->addon_joint_orientation_valid)) {
                                rv[0] = out_r[0];
                                rv[1] = out_r[1];
                                rv[2] = out_r[2];
                            }
                            if (addon_rv) {
                                if (chain->addon_chain) {
                                    float tjoint_r[3] = { 0.0f, 0.0f, 0.0f };
                                    addon_tjoint_rotation_from_visual(target,
                                                                      out_r,
                                                                      tjoint_r);
                                    addon_rv[0] = tjoint_r[0];
                                    addon_rv[1] = tjoint_r[1];
                                    addon_rv[2] = tjoint_r[2];
                                } else {
                                    addon_rv[0] = out_r[0];
                                    addon_rv[1] = out_r[1];
                                    addon_rv[2] = out_r[2];
                                }
                            }
                        }
                        {
                            int wrote_matrix = 0;
                            int wrote_smatrix = 0;
                            int wrote_tmatrix = 0;
                            if (chain->addon_chain &&
                                chain->skinned_matrix_enabled &&
                                !object_transform &&
                                !sc->room_scene_sidecar) {
                                float visual_r[3];
                                float visual_t[3];
                                visual_r[0] = target->sim_rotation_rest[0] +
                                              ((out_r[0] - target->sim_rotation_rest[0]) *
                                               chain->skinned_matrix_scale);
                                visual_r[1] = target->sim_rotation_rest[1] +
                                              ((out_r[1] - target->sim_rotation_rest[1]) *
                                               chain->skinned_matrix_scale);
                                visual_r[2] = target->sim_rotation_rest[2] +
                                              ((out_r[2] - target->sim_rotation_rest[2]) *
                                               chain->skinned_matrix_scale);
                                visual_t[0] = target->sim_rest[0];
                                visual_t[1] = target->sim_rest[1];
                                visual_t[2] = target->sim_rest[2];
                                target->addon_visual_pose_valid = 1;
                                target->addon_visual_write_translation =
                                    chain->skinned_matrix_translation_enabled;
                                target->addon_visual_tick = now;
                                target->addon_visual_rotation[0] = visual_r[0];
                                target->addon_visual_rotation[1] = visual_r[1];
                                target->addon_visual_rotation[2] = visual_r[2];
                                target->addon_visual_translation[0] = visual_t[0];
                                target->addon_visual_translation[1] = visual_t[1];
                                target->addon_visual_translation[2] = visual_t[2];
                                InterlockedExchange(&addon_traverse_overlay_active, 1);
                                if (physx_addon_apply_target_visual_pose(target)) {
                                    wrote_smatrix =
                                        target->addon_joint_orientation_valid ? 1 : 0;
                                }
                                wrote_tmatrix = 0;
                                wrote_matrix = wrote_smatrix || wrote_tmatrix;
                            }
                        if (sc->room_scene_sidecar &&
                            target->addon_write_guard_ready &&
                            target->s_rotation_offset == 0x06c &&
                            target->s_translation_offset == 0x07c &&
                            target->s_rotation_base ==
                                target->addon_write_guard_s_rotation_base &&
                            physx_vec3_sane_limit(out_r, 720.0f)) {
                            /* Publish the current room result before calling
                               the native setter. If TK17's room animation runs
                               later in this frame, the setter hook re-applies
                               this exact PhysX pose instead of zeroing it. */
                            InterlockedExchange(
                                (volatile LONG*)&target->object_output_applied,
                                0);
                            memcpy(target->object_output_rotation,
                                   out_r,
                                   sizeof(target->object_output_rotation));
                            InterlockedExchange(
                                (volatile LONG*)&target->object_output_applied,
                                1);
                            addon_object_write_bone_solver_rotation(target,
                                                                    out_r);
                            wrote_smatrix = 1;
                            wrote_matrix = 1;
                            if (!target->object_output_logged) {
                                target->object_output_logged = 1;
                                log_line("room bone physics active chain=\"%s\" target=\"%s\" output=\"native SSimpleTransform.Rotation\" receiver=%p rotation_slot=0x06c translation_slot=0x07c wind_scale=%.3f sidecar=\"%s\" note=\"TK17 animation is neutralized before the room-wind solver writes this bone\"",
                                         chain->name,
                                         target->name,
                                         target->s_raw_object ?
                                             target->s_raw_object :
                                             target->s_object,
                                         chain->wind_scale,
                                         sc->path);
                            }
                        }
                        if (defaults_cfg.debug &&
                            now - target->sim_rotation_probe_tick >= 1000) {
                            target->sim_rotation_probe_tick = now;
                            log_line("rigid-chain rotation write chain=\"%s\" target=\"%s\" rotation=(%.5f,%.5f,%.5f) pitch=%.5f roll=%.5f offset_vec=(%.5f,%.5f,%.5f) world_gravity_valid=%d gravity_bend=(%.5f,%.5f,%.5f) wrote_sjoint=%d wrote_tjoint=%d wrote_matrix=%d matrix_s=%d matrix_t=%d matrix_scale=%.3f matrix_translation=%d translation_locked=%d",
                                     chain->name, target->name,
                                     out_r[0], out_r[1], out_r[2], pitch, roll,
                                     target->sim_offset[0], target->sim_offset[1], target->sim_offset[2],
                                     addon_gravity_bend_valid,
                                     addon_gravity_bend[0],
                                     addon_gravity_bend[1],
                                     addon_gravity_bend[2],
                                     (rv && !(chain->addon_chain &&
                                              target->addon_joint_orientation_valid)) ? 1 : 0,
                                     addon_rv ? 1 : 0, wrote_matrix,
                                     wrote_smatrix, wrote_tmatrix,
                                     chain->skinned_matrix_enabled ? chain->skinned_matrix_scale : 0.0f,
                                     chain->skinned_matrix_translation_enabled,
                                     addon_local_bend ? 1 : 0);
                        }
                        if (object_transform && rv &&
                            target->addon_write_guard_ready &&
                            target->s_object ==
                                target->addon_write_guard_s_object &&
                            target->s_raw_object ==
                                target->addon_write_guard_s_rotation_base &&
                            !is_nil_engine_object(target->s_raw_object,
                                                  target->s_object) &&
                            physx_vec3_sane_limit(
                                target->object_output_rotation, 720.0f)) {
                            addon_object_write_bone_solver_rotation(
                                target,
                                target->object_output_rotation);
                            target->object_output_applied = 1;
                            if (!target->object_output_logged) {
                                float native_after[3] = {
                                    0.0f, 0.0f, 0.0f
                                };
                                int native_after_valid =
                                    ptr_readable(
                                        (BYTE*)target->s_raw_object + 0x06c,
                                        sizeof(native_after)) &&
                                    physx_vec3_sane_limit(
                                        (float*)((BYTE*)target->s_raw_object +
                                                 0x06c),
                                        720.0f);
                                if (native_after_valid) {
                                    memcpy(native_after,
                                           (BYTE*)target->s_raw_object + 0x06c,
                                           sizeof(native_after));
                                }
                                target->object_output_logged = 1;
                                log_line("addon object physics active owner=\"%s\" chain=\"%s\" target=\"%s\" output=\"native raw STransform rotation\" raw=%p weak=%p requested=(%.5f,%.5f,%.5f) native_after_valid=%d native_after=(%.5f,%.5f,%.5f) rest=(%.5f,%.5f,%.5f) scene_limits=%d scene_min=(%.1f,%.1f,%.1f) scene_max=(%.1f,%.1f,%.1f) sidecar=\"%s\" note=\"object pivot and authored translation remain owned by TK17\"",
                                         chain->addon_owner_person[0] ?
                                             chain->addon_owner_person :
                                             "unknown",
                                         chain->name, target->name,
                                         target->s_raw_object,
                                         target->s_object,
                                         target->object_output_rotation[0],
                                         target->object_output_rotation[1],
                                         target->object_output_rotation[2],
                                         native_after_valid,
                                         native_after[0],
                                         native_after[1],
                                         native_after[2],
                                         target->object_scene_rotation[0],
                                         target->object_scene_rotation[1],
                                         target->object_scene_rotation[2],
                                         target->object_scene_limits_valid,
                                         target->object_scene_rotation_min[0],
                                         target->object_scene_rotation_min[1],
                                         target->object_scene_rotation_min[2],
                                         target->object_scene_rotation_max[0],
                                         target->object_scene_rotation_max[1],
                                         target->object_scene_rotation_max[2],
                                         sc->path);
                            }
                        }
                        }
                    }
                        physx_perf_add(PHYSX_PERF_ADDON_OUTPUT,
                                       addon_output_start);
                    }
                } else {
                    for (axis = 0; axis < 3; axis++) {
                        float accel = chain->gravity[axis] * chain->gravity_scale
                                      - (k * target->sim_offset[axis])
                                      - (d * target->sim_velocity[axis]);
                        target->sim_velocity[axis] += accel * dt;
                        target->sim_offset[axis] += target->sim_velocity[axis] * dt;
                        target->sim_offset[axis] = physx_clampf(target->sim_offset[axis],
                                                                -chain->max_offset,
                                                                chain->max_offset);
                        if (target->sim_offset[axis] <= -chain->max_offset ||
                            target->sim_offset[axis] >= chain->max_offset) {
                            target->sim_velocity[axis] *= 0.35f;
                        }
                        v[axis] = target->sim_rest[axis] + parent_delta[axis] + target->sim_offset[axis];
                    }
                }
                if (chain->addon_chain && rigid) {
                    addon_chain_update_output_velocity(target, dt,
                        chain->skinned_matrix_enabled || object_transform);
                }
                    physx_perf_add(PHYSX_PERF_ADDON_SOLVER,
                                   addon_solver_start);
                }
            }
            if (chain->addon_chain) {
                if (chain->collision_enabled) {
                    addon_chain_publish_final_contacts(chain, now);
                    addon_chain_trace_contact_motion(sc, chain, now, 1);
                    int p;
                    int any_valid = 0;
                    int copy_count = chain->target_count;
                    if (copy_count > 32) copy_count = 32;
                    for (p = 0; p < copy_count; p++) {
                        chain->addon_collision_prev_valid[p] =
                            addon_collision_point_valid[p] ? 1 : 0;
                        if (addon_collision_point_valid[p]) {
                            chain->addon_collision_prev_points[p][0] =
                                addon_collision_points[p][0];
                            chain->addon_collision_prev_points[p][1] =
                                addon_collision_points[p][1];
                            chain->addon_collision_prev_points[p][2] =
                                addon_collision_points[p][2];
                            any_valid = 1;
                        }
                    }
                    for (; p < 32; p++) {
                        chain->addon_collision_prev_valid[p] = 0;
                    }
                    chain->addon_collision_prev_ready = any_valid;
                    chain->addon_collision_prev_tick = any_valid ? now : 0;
                } else {
                    chain->addon_collision_prev_ready = 0;
                    chain->addon_collision_prev_tick = 0;
                }
            }
        }
    }
}

