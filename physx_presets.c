/* Preset discovery and selection. All disk writes are explicit settings edits;
   loading a preset never copies its physics values into Config.ini. */
typedef struct physx_preset_list_t {
    char (*names)[MAX_PATH]; /* filenames including .ini; Default is implicit */
    size_t count;
} physx_preset_list_t;

static int physx_preset_compare(const void *a, const void *b)
{
    return _stricmp((const char*)a, (const char*)b);
}

static physx_preset_list_t physx_preset_scan(void)
{
    physx_preset_list_t list = {0};
    WIN32_FIND_DATAA data;
    HANDLE search;
    char pattern[MAX_PATH * 4];
    const char *base = body_profile_basename_a(config_path);
    int length = _snprintf(pattern, sizeof(pattern), "%.*sPresets\\*.ini",
        (int)(base-config_path), config_path);
    if (length < 0 || length >= (int)sizeof(pattern)) return list;
    search = FindFirstFileA(pattern, &data);
    if (search == INVALID_HANDLE_VALUE) return list;
    do {
        char path[MAX_PATH * 4];
        void *grown;
        if (data.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY ||
            !physx_preset_make_path(data.cFileName, path, sizeof(path))) continue;
        grown = realloc(list.names, (list.count + 1) * sizeof(*list.names));
        if (!grown) {
            log_line("preset discovery incomplete reason=out-of-memory");
            break;
        }
        list.names = grown;
        strcpy(list.names[list.count++], data.cFileName);
    } while (FindNextFileA(search, &data));
    FindClose(search);
    if (list.count > 1) qsort(list.names, list.count, sizeof(*list.names), physx_preset_compare);
    return list;
}

static void physx_preset_load_selection(void)
{
    char selected[MAX_PATH], path[MAX_PATH * 4];
    WIN32_FILE_ATTRIBUTE_DATA data;
    GetPrivateProfileStringA("presets", "selected", "DEFAULT", selected, sizeof(selected), config_path);
    physx_preset_path[0] = 0;
    physx_preset_pending = 0;
    if (!_stricmp(selected, "DEFAULT") || !selected[0]) {
        strcpy(physx_preset_name, "DEFAULT");
        return;
    }
    if (!physx_preset_make_path(selected, path, sizeof(path)) ||
        !GetFileAttributesExA(path, GetFileExInfoStandard, &data) ||
        data.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) {
        log_line("preset unavailable name=\"%s\" fallback=DEFAULT reason=\"missing file or invalid filename\"", selected);
        strcpy(physx_preset_name, "DEFAULT");
        WritePrivateProfileStringA("presets", "selected", "DEFAULT", config_path);
        return;
    }
    strcpy(physx_preset_name, selected);
    strcpy(physx_preset_path, path);
    physx_preset_write_time = data.ftLastWriteTime;
    log_line("preset loaded name=\"%s\" priority=\"body sidecar > preset > Config.ini\"", selected);
}

static int physx_preset_select(const char *name)
{
    char path[MAX_PATH * 4];
    DWORD attributes;
    if (!name) return 0;
    if (!_stricmp(name, "DEFAULT")) name = "DEFAULT";
    else {
        if (!physx_preset_make_path(name, path, sizeof(path))) return 0;
        attributes = GetFileAttributesA(path);
        if (attributes == INVALID_FILE_ATTRIBUTES || attributes & FILE_ATTRIBUTE_DIRECTORY) {
            log_line("preset selection unavailable name=\"%s\" fallback=DEFAULT", name);
            name = "DEFAULT";
        }
    }
    if (!WritePrivateProfileStringA("presets", "selected", name, config_path)) {
        log_line("preset selection save failed name=\"%s\" error=%lu", name, GetLastError());
        return 0;
    }
    /* Physics reloads on its normal update thread, at the next update. UI
       readers can already use the new selection without touching live solvers. */
    physx_preset_load_selection();
    InterlockedExchange(&body_profile_reload_pending, 1);
    return 1;
}

static int physx_preset_reload_due(DWORD now)
{
    WIN32_FILE_ATTRIBUTE_DATA data;
    if (!physx_preset_path[0]) return 0;
    if (!GetFileAttributesExA(physx_preset_path, GetFileExInfoStandard, &data) ||
        data.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) return 1;
    if (!CompareFileTime(&data.ftLastWriteTime, &physx_preset_write_time)) {
        physx_preset_pending = 0;
        return 0;
    }
    if (!physx_preset_pending || CompareFileTime(&data.ftLastWriteTime, &physx_preset_pending_time)) {
        physx_preset_pending_time = data.ftLastWriteTime;
        physx_preset_pending_tick = now;
        physx_preset_pending = 1;
        return 0;
    }
    return now - physx_preset_pending_tick >= 1500;
}

static void physx_preset_reset_physics_defaults(void)
{
    static int captured;
    static body_chain_physics_config_t penis, testicle, breasts, butt;
    body_profile_set_active_person_config(-1);
    if (!captured) {
        penis = body_chain_physics_global_cfg;
        testicle = testicle_physics_global_cfg;
        breasts = breasts_physics_global_cfg;
        butt = butt_physics_global_cfg;
        captured = 1;
    }
    body_chain_physics_global_cfg = penis;
    testicle_physics_global_cfg = testicle;
    breasts_physics_global_cfg = breasts;
    butt_physics_global_cfg = butt;
}
