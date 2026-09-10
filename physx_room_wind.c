#define ROOM_WIND_SIDECAR_COUNT 64

typedef struct room_wind_sidecar_t {
    char path[MAX_PATH * 4];
    char scene_key[MAX_PATH * 2];
    char package_prefix[MAX_PATH * 2];
    FILETIME write_time;
    int loaded;
} room_wind_sidecar_t;

typedef struct room_wind_config_t {
    int active;
    int enabled;
    int sidecar_index;
    unsigned int generation;
    char path[MAX_PATH * 4];
    char scene_key[MAX_PATH * 2];
    float direction[3];
    float strength;
    float turbulence;
    float gust_strength;
    float gust_frequency;
    float variation;
    float sway_strength;
    float sway_frequency;
} room_wind_config_t;

static room_wind_sidecar_t room_wind_sidecars[ROOM_WIND_SIDECAR_COUNT];
static int room_wind_sidecar_count;
static room_wind_config_t room_wind_cfg;
static DWORD room_wind_hot_reload_tick;
static char room_wind_observed_scene_key[MAX_PATH * 2];
static char room_wind_observed_package_prefix[MAX_PATH * 2];
static unsigned int room_wind_observation_generation = 1u;
static unsigned int room_wind_applied_observation_generation;

static char *room_wind_find_i(char *text, const char *needle)
{
    size_t needle_len;
    char *p;
    if (!text || !needle || !needle[0]) return NULL;
    needle_len = strlen(needle);
    for (p = text; *p; p++) {
        if (_strnicmp(p, needle, needle_len) == 0) return p;
    }
    return NULL;
}

static void room_wind_normalize_path(char *path)
{
    char *p;
    if (!path) return;
    for (p = path; *p; p++) {
        if (*p == '/') *p = '\\';
    }
}

static int room_wind_scene_key_from_path(const char *path,
                                         char *out,
                                         size_t outsz)
{
    char normalized[MAX_PATH * 4];
    char *scenes;
    char *leaf;
    char *slash;
    if (!out || outsz == 0) return 0;
    out[0] = 0;
    if (!path || !path[0]) return 0;
    lstrcpynA(normalized, path, sizeof(normalized));
    room_wind_normalize_path(normalized);
    scenes = room_wind_find_i(normalized, "\\Scenes\\");
    if (scenes) {
        leaf = scenes + 8;
    } else if (_strnicmp(normalized, "Scenes\\", 7) == 0) {
        leaf = normalized + 7;
    } else {
        return 0;
    }
    if (!room_wind_find_i(leaf, "\\Room\\")) return 0;
    lstrcpynA(out, leaf, (int)outsz);
    slash = strrchr(out, '\\');
    if (!slash) return 0;
    *slash = 0;
    return out[0] != 0;
}

static int room_wind_package_prefix_from_path(const char *path,
                                              char *out,
                                              size_t outsz)
{
    char normalized[MAX_PATH * 4];
    char *scenes;
    size_t len;
    if (!out || outsz == 0) return 0;
    out[0] = 0;
    if (!path || !path[0]) return 0;
    lstrcpynA(normalized, path, sizeof(normalized));
    room_wind_normalize_path(normalized);
    scenes = room_wind_find_i(normalized, "\\Scenes\\");
    if (!scenes) return 0;
    len = (size_t)(scenes - normalized);
    if (len >= outsz) len = outsz - 1;
    memcpy(out, normalized, len);
    out[len] = 0;
    return out[0] != 0;
}

static int room_wind_scene_key_from_object_name(const char *object_name,
                                                char *out,
                                                size_t outsz)
{
    char normalized[MAX_PATH * 4];
    char stem[256];
    char folder[256];
    char *file;
    char *slash;
    char *dot;
    int i;
    int strip_s_prefix = 0;
    if (!out || outsz == 0) return 0;
    out[0] = 0;
    if (!object_name || !object_name[0]) return 0;
    lstrcpynA(normalized, object_name, sizeof(normalized));
    room_wind_normalize_path(normalized);
    if (!room_wind_find_i(normalized, "\\Room\\")) return 0;
    file = strrchr(normalized, '\\');
    if (!file || !file[1]) return 0;
    lstrcpynA(stem, file + 1, sizeof(stem));
    dot = strrchr(stem, '.');
    if (!dot || _stricmp(dot, ".ma") != 0) return 0;
    *dot = 0;
    *file = 0;
    slash = strrchr(normalized, '\\');
    lstrcpynA(folder, slash ? slash + 1 : normalized, sizeof(folder));
    if (_stricmp(stem, folder) != 0) return 0;
    /* TK17 gives the paired STransform the owning TTransform's room path
       with an extra leading S (for example SLuder instead of Luder).  It is
       the same live room, not a transition.  Strip that prefix only when the
       remaining key is one we already observed, activated, or registered so
       legitimate room namespaces beginning with S remain untouched. */
    if (normalized[0] == 'S' || normalized[0] == 's') {
        if ((room_wind_observed_scene_key[0] &&
             _stricmp(normalized + 1,
                      room_wind_observed_scene_key) == 0) ||
            (room_wind_cfg.scene_key[0] &&
             _stricmp(normalized + 1, room_wind_cfg.scene_key) == 0)) {
            strip_s_prefix = 1;
        }
        for (i = 0; !strip_s_prefix && i < room_wind_sidecar_count; i++) {
            if (_stricmp(normalized + 1,
                         room_wind_sidecars[i].scene_key) == 0) {
                strip_s_prefix = 1;
            }
        }
        if (strip_s_prefix) {
            memmove(normalized, normalized + 1, strlen(normalized));
        }
    }
    lstrcpynA(out, normalized, (int)outsz);
    return out[0] != 0;
}

static void room_wind_observe_scene(const char *scene_key,
                                    const char *package_prefix,
                                    const char *source)
{
    int scene_changed;
    int package_changed;
    if (!scene_key || !scene_key[0]) return;
    scene_changed = _stricmp(room_wind_observed_scene_key, scene_key) != 0;
    package_changed = package_prefix && package_prefix[0] &&
        _stricmp(room_wind_observed_package_prefix, package_prefix) != 0;
    if (!scene_changed && !package_changed) return;
    if (scene_changed) {
        lstrcpynA(room_wind_observed_scene_key, scene_key,
                  sizeof(room_wind_observed_scene_key));
        room_wind_observed_package_prefix[0] = 0;
    }
    if (package_prefix && package_prefix[0]) {
        lstrcpynA(room_wind_observed_package_prefix, package_prefix,
                  sizeof(room_wind_observed_package_prefix));
    }
    room_wind_observation_generation++;
    if (!room_wind_observation_generation) {
        room_wind_observation_generation = 1u;
    }
    log_line("room wind runtime room observed scene_key=\"%s\" source=\"%s\" note=\"the matching room-local [wind] sidecar will be selected on the next physics tick\"",
             room_wind_observed_scene_key,
             source ? source : "runtime");
}

static int room_wind_has_section(const char *path)
{
    char section[2048];
    if (!path || !path[0]) return 0;
    return GetPrivateProfileSectionA("wind", section, sizeof(section), path) > 0;
}

static int room_wind_register_sidecar_a(const char *path)
{
    char scene_key[MAX_PATH * 2];
    int i;
    room_wind_sidecar_t *entry;
    if (!path || !ends_with_i(path, ".physx.ini") ||
        !room_wind_has_section(path) ||
        !room_wind_scene_key_from_path(path, scene_key,
                                       sizeof(scene_key))) {
        return 0;
    }
    for (i = 0; i < room_wind_sidecar_count; i++) {
        if (_stricmp(room_wind_sidecars[i].path, path) == 0) return 1;
    }
    if (room_wind_sidecar_count >= ROOM_WIND_SIDECAR_COUNT) {
        log_line("room wind sidecar ignored sidecar=\"%s\" reason=\"room wind sidecar registry is full\"",
                 path);
        return 0;
    }
    entry = &room_wind_sidecars[room_wind_sidecar_count++];
    memset(entry, 0, sizeof(*entry));
    lstrcpynA(entry->path, path, sizeof(entry->path));
    lstrcpynA(entry->scene_key, scene_key, sizeof(entry->scene_key));
    room_wind_package_prefix_from_path(path, entry->package_prefix,
                                       sizeof(entry->package_prefix));
    log_line("room wind sidecar registered scene_key=\"%s\" sidecar=\"%s\" note=\"[wind] is room-local and does not create Person01-04 physics instances\"",
             entry->scene_key, entry->path);
    if (room_wind_observed_scene_key[0] &&
        _stricmp(room_wind_observed_scene_key, entry->scene_key) == 0) {
        room_wind_observation_generation++;
        if (!room_wind_observation_generation) {
            room_wind_observation_generation = 1u;
        }
    }
    return 1;
}

static void room_wind_clear_active(const char *reason)
{
    if (!room_wind_cfg.active && !room_wind_cfg.enabled) return;
    log_line("room wind deactivated scene_key=\"%s\" sidecar=\"%s\" reason=\"%s\"",
             room_wind_cfg.scene_key,
             room_wind_cfg.path,
             reason ? reason : "room-transition");
    room_wind_cfg.generation++;
    if (!room_wind_cfg.generation) room_wind_cfg.generation = 1;
    room_wind_cfg.active = 0;
    room_wind_cfg.enabled = 0;
    room_wind_cfg.sidecar_index = -1;
    room_wind_cfg.path[0] = 0;
    room_wind_cfg.scene_key[0] = 0;
}

static void room_wind_load_entry(int index, const char *reason)
{
    room_wind_sidecar_t *entry;
    char direction[128];
    float len;
    FILETIME wt;
    unsigned int next_generation;
    if (index < 0 || index >= room_wind_sidecar_count) return;
    entry = &room_wind_sidecars[index];
    if (!get_file_write_time_a(entry->path, &wt) ||
        !room_wind_has_section(entry->path)) {
        if (room_wind_cfg.sidecar_index == index) {
            room_wind_clear_active("active sidecar missing or [wind] removed");
        }
        return;
    }
    entry->write_time = wt;
    entry->loaded = 1;

    next_generation = room_wind_cfg.generation + 1u;
    if (!next_generation) next_generation = 1u;
    memset(&room_wind_cfg, 0, sizeof(room_wind_cfg));
    room_wind_cfg.active = 1;
    room_wind_cfg.sidecar_index = index;
    room_wind_cfg.generation = next_generation;
    lstrcpynA(room_wind_cfg.path, entry->path,
              sizeof(room_wind_cfg.path));
    lstrcpynA(room_wind_cfg.scene_key, entry->scene_key,
              sizeof(room_wind_cfg.scene_key));
    room_wind_cfg.enabled = profile_bool("wind", "enabled", 1,
                                         entry->path);
    room_wind_cfg.direction[0] = 1.0f;
    room_wind_cfg.direction[1] = 0.0f;
    room_wind_cfg.direction[2] = 0.0f;
    GetPrivateProfileStringA("wind", "direction", "1,0,0",
                             direction, sizeof(direction), entry->path);
    parse_vec3(direction, room_wind_cfg.direction);
    len = physx_vec3_len(room_wind_cfg.direction);
    if (len <= 0.0001f ||
        !body_chain_vec3_sane_limit(room_wind_cfg.direction, 1000.0f)) {
        room_wind_cfg.direction[0] = 1.0f;
        room_wind_cfg.direction[1] = 0.0f;
        room_wind_cfg.direction[2] = 0.0f;
        log_line("room wind direction fallback requested=\"%s\" selected=(1,0,0) sidecar=\"%s\" reason=\"direction must be a finite non-zero Vector3\"",
                 direction, entry->path);
    } else {
        room_wind_cfg.direction[0] /= len;
        room_wind_cfg.direction[1] /= len;
        room_wind_cfg.direction[2] /= len;
    }
    room_wind_cfg.strength = physx_clampf(
        profile_float("wind", "strength", 0.25f, entry->path),
        0.0f, 10.0f);
    room_wind_cfg.turbulence = physx_clampf(
        profile_float("wind", "turbulence", 0.20f, entry->path),
        0.0f, 2.0f);
    room_wind_cfg.gust_strength = physx_clampf(
        profile_float("wind", "gust_strength", 0.35f, entry->path),
        0.0f, 4.0f);
    room_wind_cfg.gust_frequency = physx_clampf(
        profile_float("wind", "gust_frequency", 0.15f, entry->path),
        0.0f, 10.0f);
    room_wind_cfg.variation = physx_clampf(
        profile_float("wind", "variation", 0.20f, entry->path),
        0.0f, 1.0f);
    /* Zero strength preserves the pre-sway wind exactly. */
    room_wind_cfg.sway_strength = physx_clampf(
        profile_float("wind", "sway_strength", 0.0f, entry->path),
        0.0f, 10.0f);
    room_wind_cfg.sway_frequency = physx_clampf(
        profile_float("wind", "sway_frequency", 0.08f, entry->path),
        0.001f, 10.0f);
    log_line("room wind %s enabled=%d scene_key=\"%s\" direction=(%.4f,%.4f,%.4f) strength=%.4f turbulence=%.4f gust_strength=%.4f gust_frequency=%.4f variation=%.4f sway_strength=%.4f sway_frequency=%.4f sidecar=\"%s\" note=\"existing add-on chains inherit wind_enabled=true, wind_scale=1.0, and room sway defaults unless overridden\"",
             reason ? reason : "loaded",
             room_wind_cfg.enabled,
             room_wind_cfg.scene_key,
             room_wind_cfg.direction[0],
             room_wind_cfg.direction[1],
             room_wind_cfg.direction[2],
             room_wind_cfg.strength,
             room_wind_cfg.turbulence,
             room_wind_cfg.gust_strength,
             room_wind_cfg.gust_frequency,
             room_wind_cfg.variation,
             room_wind_cfg.sway_strength,
             room_wind_cfg.sway_frequency,
             room_wind_cfg.path);
}

static int room_wind_scene_entry_file(const char *path,
                                      const char *scene_key)
{
    char normalized[MAX_PATH * 4];
    char stem[256];
    char folder[256];
    char *file;
    char *slash;
    char *dot;
    size_t folder_len;
    if (!path || !scene_key) return 0;
    lstrcpynA(normalized, path, sizeof(normalized));
    room_wind_normalize_path(normalized);
    file = strrchr(normalized, '\\');
    file = file ? file + 1 : normalized;
    lstrcpynA(stem, file, sizeof(stem));
    dot = strrchr(stem, '.');
    if (dot) *dot = 0;
    slash = strrchr(scene_key, '\\');
    lstrcpynA(folder, slash ? slash + 1 : scene_key, sizeof(folder));
    if (_stricmp(stem, folder) == 0) return 1;
    folder_len = strlen(folder);
    return _strnicmp(stem, folder, folder_len) == 0 &&
           _stricmp(stem + folder_len, "_core") == 0;
}

static void physx_note_room_scene_file_a(const char *scene_path)
{
    char scene_key[MAX_PATH * 2];
    char package_prefix[MAX_PATH * 2];
    if (!scene_path ||
        (!ends_with_i(scene_path, ".bs") &&
         !ends_with_i(scene_path, ".lua")) ||
        !room_wind_scene_key_from_path(scene_path, scene_key,
                                       sizeof(scene_key))) {
        return;
    }
    if (!room_wind_scene_entry_file(scene_path, scene_key)) return;
    package_prefix[0] = 0;
    room_wind_package_prefix_from_path(scene_path, package_prefix,
                                       sizeof(package_prefix));
    room_wind_observe_scene(scene_key, package_prefix, "scene-file");
}

static void physx_note_room_scene_file_w(const WCHAR *scene_path)
{
    char path[MAX_PATH * 4];
    int ok;
    if (!scene_path) return;
    path[0] = 0;
    ok = WideCharToMultiByte(CP_ACP, 0, scene_path, -1,
                             path, sizeof(path), NULL, NULL);
    if (!ok) return;
    path[sizeof(path) - 1] = 0;
    physx_note_room_scene_file_a(path);
}

static void physx_note_room_object_name_a(const char *object_name)
{
    char scene_key[MAX_PATH * 2];
    if (!room_wind_scene_key_from_object_name(object_name, scene_key,
                                               sizeof(scene_key))) {
        return;
    }
    room_wind_observe_scene(scene_key, NULL, "room-object-name");
}

static void room_wind_apply_observed_scene(void)
{
    int best = -1;
    int package_match = 0;
    int i;
    if (room_wind_applied_observation_generation ==
        room_wind_observation_generation) {
        return;
    }
    room_wind_applied_observation_generation =
        room_wind_observation_generation;
    if (!room_wind_observed_scene_key[0]) return;
    for (i = 0; i < room_wind_sidecar_count; i++) {
        room_wind_sidecar_t *entry = &room_wind_sidecars[i];
        int exact_package = room_wind_observed_package_prefix[0] &&
            entry->package_prefix[0] &&
            _stricmp(room_wind_observed_package_prefix,
                     entry->package_prefix) == 0;
        if (_stricmp(room_wind_observed_scene_key,
                     entry->scene_key) != 0) {
            continue;
        }
        if (exact_package || best < 0) {
            best = i;
            package_match = exact_package;
        }
        if (exact_package) break;
    }
    if (best >= 0) {
        if (!room_wind_cfg.active ||
            room_wind_cfg.sidecar_index != best ||
            (package_match &&
             _stricmp(room_wind_cfg.path,
                      room_wind_sidecars[best].path) != 0)) {
            room_wind_load_entry(best, "activated");
        }
    } else if (room_wind_cfg.active) {
        room_wind_clear_active("new room has no [wind] sidecar");
    }
}

static void run_room_wind_hot_reload(DWORD now)
{
    room_wind_sidecar_t *entry;
    FILETIME wt;
    room_wind_apply_observed_scene();
    if (!room_wind_cfg.active || room_wind_cfg.sidecar_index < 0 ||
        room_wind_cfg.sidecar_index >= room_wind_sidecar_count) {
        return;
    }
    if (room_wind_hot_reload_tick &&
        now - room_wind_hot_reload_tick < 750u) {
        return;
    }
    room_wind_hot_reload_tick = now;
    entry = &room_wind_sidecars[room_wind_cfg.sidecar_index];
    if (!get_file_write_time_a(entry->path, &wt)) {
        room_wind_clear_active("active sidecar removed");
        return;
    }
    if (!entry->loaded || filetime_differs(&entry->write_time, &wt)) {
        room_wind_load_entry(room_wind_cfg.sidecar_index, "hot-reloaded");
    }
}

static int room_wind_is_enabled(void)
{
    return physics_environment_cfg.wind_enabled &&
           room_wind_cfg.active && room_wind_cfg.enabled &&
           room_wind_cfg.strength > 0.000001f;
}

static unsigned int room_wind_generation(void)
{
    return room_wind_cfg.generation;
}

static int room_wind_direction(float out[3])
{
    if (!out || !room_wind_is_enabled()) return 0;
    memcpy(out, room_wind_cfg.direction, sizeof(room_wind_cfg.direction));
    return 1;
}

static unsigned int room_wind_hash_add(unsigned int hash, const char *text)
{
    const unsigned char *p = (const unsigned char *)(text ? text : "");
    while (*p) {
        hash ^= (unsigned int)*p++;
        hash *= 16777619u;
    }
    return hash;
}

static float room_wind_hashed_strength(const char *owner_name,
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

static float room_wind_body_strength(const char *person,
                                     const char *system_name,
                                     float wind_scale,
                                     DWORD now)
{
    return room_wind_hashed_strength(person, system_name, "body",
                                     wind_scale,
                                     room_wind_cfg.sway_strength,
                                     room_wind_cfg.sway_frequency,
                                     now);
}

static float room_wind_target_strength(const physx_chain_t *chain,
                                       const physx_target_t *target,
                                       DWORD now)
{
    float sway_strength;
    float sway_frequency;
    if (!chain || !target || !chain->wind_enabled) return 0.0f;
    sway_strength = chain->wind_sway_strength >= 0.0f ?
        chain->wind_sway_strength : room_wind_cfg.sway_strength;
    sway_frequency = chain->wind_sway_frequency >= 0.0f ?
        chain->wind_sway_frequency : room_wind_cfg.sway_frequency;
    return room_wind_hashed_strength(chain->addon_owner_person,
                                     chain->name, target->name,
                                     chain->wind_scale,
                                     sway_strength,
                                     sway_frequency,
                                     now);
}
