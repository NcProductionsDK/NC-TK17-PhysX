/* Included before the Win32 INI wrappers: reads here deliberately use the
   real API. Presets only override the four body physics sections. */
static char physx_preset_name[MAX_PATH] = "DEFAULT";
static char physx_preset_path[MAX_PATH * 4];
static FILETIME physx_preset_write_time;
static FILETIME physx_preset_pending_time;
static DWORD physx_preset_pending_tick;
static int physx_preset_pending;

static int physx_preset_section_allowed(const char *section)
{
    return section && (!_stricmp(section, "breasts_physics") ||
        !_stricmp(section, "penis_physics") ||
        !_stricmp(section, "testicle_physics") ||
        !_stricmp(section, "butt_physics"));
}

static int physx_preset_make_path(const char *name, char *out, size_t size)
{
    const char *base = body_profile_basename_a(config_path);
    size_t prefix = (size_t)(base - config_path);
    int length;
    if (!name || !name[0] || strlen(name) >= MAX_PATH ||
        strpbrk(name, "\\/:*?\"<>|\r\n") || !strcmp(name, ".") ||
        !strcmp(name, "..") || name[strlen(name)-1] == ' ' ||
        name[strlen(name)-1] == '.') return 0;
    if (strlen(name) <= 4 || _stricmp(name + strlen(name) - 4, ".ini")) return 0;
    length = _snprintf(out, size, "%.*sPresets\\%s", (int)prefix, config_path, name);
    return length > 0 && (size_t)length < size;
}

/* Treat the paired-body aliases as one option, with the higher layer winning
   even when Config.ini spells it max_angle and the preset spells it joint01_max_angle. */
static const char *physx_preset_key(const char *section, const char *key)
{
    static const char *const paired[][2] = {
        {"max_angle", "joint01_max_angle"},
        {"min_angle", "joint01_min_angle"}, {"gain", "joint01_gain"}
    };
    size_t i;
    if (!key || !key[0] || !physx_preset_path[0] ||
        !physx_preset_section_allowed(section)) return NULL;
    if (!_stricmp(section, "breasts_physics") || !_stricmp(section, "butt_physics")) {
        for (i = 0; i < sizeof(paired)/sizeof(paired[0]); i++) {
            if (_stricmp(key, paired[i][0]) && _stricmp(key, paired[i][1])) continue;
            if (raw_profile_key_exists_a(section, paired[i][0], physx_preset_path)) return paired[i][0];
            if (raw_profile_key_exists_a(section, paired[i][1], physx_preset_path)) return paired[i][1];
            return NULL;
        }
    }
    return raw_profile_key_exists_a(section, key, physx_preset_path) ? key : NULL;
}

static const char *physx_settings_write_path(const char *section)
{
    return physx_preset_path[0] && physx_preset_section_allowed(section)
        ? physx_preset_path : config_path;
}
