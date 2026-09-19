/* TK17-158.001 uses these same indexed string setters in ConfigEditor at
   0x4fd223 / 0x4fd2c0 and trims old entries with slot+0x30 at 0x4fd27a.
   Use engine-managed strings and arrays, never substitute raw C arrays. */
typedef void (THISCALL *physx_index_string_set_t)(void *, DWORD, int, const char *);
typedef int (THISCALL *physx_index_count_t)(void *, DWORD);
typedef void (THISCALL *physx_index_remove_t)(void *, DWORD, int, int);

static void *physx_settings_customizer;
static void physx_sync_controls(void *self);

/* Native BoxStringValue setter updates the selected item and the customizer's
   saved value together. record+0x18 is a box; +0x24 is a plain text field. */
static int physx_sync_box_from_ini(void *self, int index, void *record,
    const physx_settings_binding_t *binding)
{
    typedef int (THISCALL *set_box_t)(void *, int, const char *, int);
    static const BYTE expected[] = {0x55,0x8b,0xec,0x83,0xec,0x08,0x53,0x8b,0xd9};
    BYTE *base = (BYTE*)GetModuleHandleA(NULL);
    char requested[MAX_PATH], current[MAX_PATH], *value = NULL, *saved;
    engine_string_construct_cstr_t construct;
    engine_string_release_t release;
    set_box_t set;
    if (!record || !ptr_readable(record, 0xaa) || !*((BYTE*)record+0x9e) ||
        !base || !ptr_readable(base+0x001cd500, sizeof(expected)) ||
        memcmp(base+0x001cd500, expected, sizeof(expected)) ||
        !physx_setting_ini_spinbox_value(binding, requested, sizeof(requested))) return 0;
    memcpy(&saved, (BYTE*)record+0x44, sizeof(saved));
    if (physx_copy_engine_string(saved, current, sizeof(current)) && !strcmp(current, requested)) return 1;
    memcpy(&construct, base+PHYSX_ENGINE_STRING_CSTR_CONSTRUCT_RVA, sizeof(construct));
    memcpy(&release, base+PHYSX_ENGINE_STRING_RELEASE_RVA, sizeof(release));
    set = (set_box_t)(base+0x001cd500);
    if (!ptr_executable((void*)construct) || !ptr_executable((void*)release) ||
        !ptr_executable((void*)set)) return 0;
    construct(&value, requested);
    if (!value) return 0;
    physx_settings_sync_depth++;
    set(self, index, value, 0);
    physx_settings_sync_depth--;
    release(&value);
    return 1;
}

static void *physx_parameter_dispatch(void *parameter, DWORD id, size_t offset)
{
    BYTE *metadata, *table;
    void *method;
    if (!parameter || ((ULONG_PTR)parameter & 8u) ||
        !ptr_readable((BYTE*)parameter - SCRIPT_OBJECT_META_BACK_OFFSET, sizeof(metadata))) return NULL;
    memcpy(&metadata, (BYTE*)parameter - SCRIPT_OBJECT_META_BACK_OFFSET, sizeof(metadata));
    if (!metadata || !ptr_readable(metadata + (id & 0xfffu)*sizeof(void*), sizeof(table))) return NULL;
    memcpy(&table, metadata + (id & 0xfffu)*sizeof(void*), sizeof(table));
    offset += (size_t)(id >> 24) << 6;
    if (!table || !ptr_readable(table + offset, sizeof(method))) return NULL;
    memcpy(&method, table + offset, sizeof(method));
    return ptr_executable(method) ? method : NULL;
}

static int physx_preset_fill_parameter_at_base(void *parameter, const physx_preset_list_t *list, BYTE *base)
{
    DWORD ids[2];
    physx_index_string_set_t set[2];
    physx_index_remove_t remove[2];
    physx_index_count_t count[2];
    engine_string_construct_cstr_t construct;
    engine_string_release_t release;
    size_t index;
    int column;
    if (!base || !ptr_readable(base + 0x002b044c, sizeof(DWORD)) ||
        !ptr_readable(base + PHYSX_ENGINE_STRING_CSTR_CONSTRUCT_RVA, sizeof(construct)) ||
        !ptr_readable(base + PHYSX_ENGINE_STRING_RELEASE_RVA, sizeof(release))) return 0;
    memcpy(&ids[0], base + 0x002b044c, sizeof(DWORD)); /* BoxDescriptionArray */
    memcpy(&ids[1], base + 0x002b0444, sizeof(DWORD)); /* BoxStringValueArray */
    memcpy(&construct, base + PHYSX_ENGINE_STRING_CSTR_CONSTRUCT_RVA, sizeof(construct));
    memcpy(&release, base + PHYSX_ENGINE_STRING_RELEASE_RVA, sizeof(release));
    if (!ptr_executable((void*)construct) || !ptr_executable((void*)release)) return 0;
    for (column = 0; column < 2; column++) {
        set[column] = (physx_index_string_set_t)physx_parameter_dispatch(parameter, ids[column], 4);
        count[column] = (physx_index_count_t)physx_parameter_dispatch(parameter, ids[column], 8);
        remove[column] = (physx_index_remove_t)physx_parameter_dispatch(parameter, ids[column], 0x30);
        if (!set[column] || !count[column] || !remove[column]) return 0;
    }
    for (column = 0; column < 2; column++) {
        for (index = 0; index <= list->count; index++) {
            char label[MAX_PATH];
            char *value = NULL;
            if (!index) strcpy(label, column ? "DEFAULT" : "Default");
            else {
                strcpy(label, list->names[index-1]);
                if (!column) label[strlen(label)-4] = 0;
            }
            construct(&value, label);
            if (!value) return 0;
            set[column](parameter, ids[column], (int)index, value);
            release(&value);
        }
        if (count[column](parameter, ids[column]) > (int)list->count + 1)
            remove[column](parameter, ids[column], (int)list->count + 1, -1);
    }
    return 1;
}

static int physx_preset_fill_parameter(void *parameter, const physx_preset_list_t *list)
{
    return physx_preset_fill_parameter_at_base(parameter, list, (BYTE*)GetModuleHandleA(NULL));
}

static void physx_preset_prepare_controls(void *self)
{
    void **parameters;
    int count, i;
    if (!self || !ptr_readable((BYTE*)self+0x14, sizeof(parameters))) return;
    memcpy(&parameters, (BYTE*)self+0x14, sizeof(parameters));
    if (!parameters || !ptr_readable(parameters-1, sizeof(count))) return;
    memcpy(&count, parameters-1, sizeof(count));
    if (count <= 0 || count > 4096 || !ptr_readable(parameters, count*sizeof(void*))) return;
    for (i = 0; i < count; i++) {
        char name[128];
        if (physx_custom_parameter_name(parameters[i], name, sizeof(name)) &&
            !strcmp(name, "NCPhysXPresets")) {
            physx_preset_list_t list;
            if (!config_path[0]) config_file_path(config_path, sizeof(config_path));
            list = physx_preset_scan();
            if (!physx_preset_fill_parameter(parameters[i], &list))
                log_line("preset selector unavailable reason=\"unsupported parameter methods\"");
            free(list.names);
            return;
        }
    }
}
