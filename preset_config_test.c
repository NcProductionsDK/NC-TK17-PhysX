#include "NC-TK17-PhysX.c"
#include <assert.h>

#undef GetPrivateProfileStringA
#undef GetPrivateProfileIntA

static void near_value(float actual, float expected)
{
    if (fabsf(actual-expected) > .0001f) {
        fprintf(stderr, "expected %g, got %g\n", expected, actual);
        abort();
    }
}

static void write_fixture(const char *path, const char *contents)
{
    FILE *f = fopen(path, "wb");
    assert(f);
    assert(fwrite(contents, 1, strlen(contents), f) == strlen(contents));
    fclose(f);
    WritePrivateProfileStringA(NULL, NULL, NULL, path);
}

static void check_stiffness(float global, float person0)
{
    near_value(breasts_physics_global_cfg.stiffness, global);
    near_value(body_chain_physics_global_cfg.stiffness, global);
    near_value(testicle_physics_global_cfg.stiffness, global);
    near_value(butt_physics_global_cfg.stiffness, global);
    near_value(breasts_physics_person_cfg[0].stiffness, person0);
    near_value(body_chain_physics_person_cfg[0].stiffness, person0);
    near_value(testicle_physics_person_cfg[0].stiffness, person0);
    near_value(butt_physics_person_cfg[0].stiffness, person0);
    near_value(body_chain_physics_person_cfg[1].stiffness, global);
}

static char ui_values[2][8][MAX_PATH];
static int ui_count[2], ui_constructed, ui_released;
static void THISCALL ui_construct(char **out, const char *text)
{
    ui_constructed++;
    *out = _strdup(text);
}
static void THISCALL ui_release(char **value) {ui_released++;free(*value);}
static void THISCALL ui_set(void *object,DWORD id,int index,const char *value)
{
    int col=(id>>24)-1;(void)object;
    assert(index>=0 && index<8);
    strcpy(ui_values[col][index],value);
    if (ui_count[col]<=index) ui_count[col]=index+1;
}
static int THISCALL ui_get_count(void *object,DWORD id) {(void)object;return ui_count[(id>>24)-1];}
static void THISCALL ui_remove(void *object,DWORD id,int start,int count)
{
    (void)object;assert(count==-1);ui_count[(id>>24)-1]=start;
}
static void check_ui_arrays(const physx_preset_list_t *list)
{
    BYTE *base=calloc(1,0x2c0000);
    BYTE *table=calloc(1,0x100);
    BYTE *metadata=calloc(1,0x10);
    BYTE *object=calloc(1,0x40);
    DWORD ids[2]={0x01000001,0x02000001};
    engine_string_construct_cstr_t construct=ui_construct;
    engine_string_release_t release=ui_release;
    physx_index_string_set_t set=ui_set;
    physx_index_count_t count=ui_get_count;
    physx_index_remove_t remove=ui_remove;
    assert(base&&table&&metadata&&object);
    memcpy(base+0x2b044c,&ids[0],4);memcpy(base+0x2b0444,&ids[1],4);
    memcpy(base+PHYSX_ENGINE_STRING_CSTR_CONSTRUCT_RVA,&construct,4);
    memcpy(base+PHYSX_ENGINE_STRING_RELEASE_RVA,&release,4);
    memcpy(metadata+4,&table,4);memcpy(object+8,&metadata,4);
    for(int col=0;col<2;col++) {
        int offset=(col+1)*0x40;
        memcpy(table+offset+4,&set,4);memcpy(table+offset+8,&count,4);memcpy(table+offset+0x30,&remove,4);
        ui_count[col]=7;
    }
    assert(physx_preset_fill_parameter_at_base(object+0x20,list,base));
    assert(ui_count[0]==(int)list->count+1 && ui_count[1]==ui_count[0]);
    assert(!strcmp(ui_values[0][0],"Default")&&!strcmp(ui_values[1][0],"DEFAULT"));
    assert(!strcmp(ui_values[0][1],"01_Test")&&!strcmp(ui_values[1][1],"01_Test.ini"));
    assert(ui_constructed==ui_released);
    physx_preset_list_t empty={0};
    assert(physx_preset_fill_parameter_at_base(object+0x20,&empty,base));
    assert(ui_count[0]==1&&ui_count[1]==1&&ui_constructed==ui_released);
    free(object);free(metadata);free(table);free(base);
    puts("PASS: native indexed string dispatch, Default first, filename labels/values, removal and string lifetimes");
}

int main(void)
{
    char preset[MAX_PATH*4], second[MAX_PATH*4], sidecar[MAX_PATH*4], directory[MAX_PATH*4], value[64];
    physx_preset_list_t list;
    DWORD tick = 100000;
    float default_wind;
    self_module = GetModuleHandleA(NULL);
    config_file_path(config_path, sizeof(config_path));
    strcpy(directory, config_path);
    *strrchr(directory, '\\') = 0;
    strcat(directory, "\\Presets");
    assert(CreateDirectoryA(directory, NULL));
    assert(physx_preset_make_path("01_Test.ini", preset, sizeof(preset)));
    assert(physx_preset_make_path("02_Second.ini", second, sizeof(second)));
    snprintf(sidecar, sizeof(sidecar), "%s\\sidecar.ini", directory);
    write_fixture(config_path,
        "[presets]\nselected=DEFAULT\n"
        "[defaults]\nconfig_reload_poll_ms=100\n"
        "[breasts_physics]\nstiffness=90\ndamping=8\ncollision_strength=0.8\nmax_angle=45,40,35\nmin_angle=-15,-20,-25\ngain=0.9\n"
        "[penis_physics]\nstiffness=90\ndamping=8\nenabled=true\njoint01_max_angle=45,40,35\njoint01_min_angle=-15,-20,-25\n"
        "[testicle_physics]\nstiffness=90\ndamping=8\njoint01_max_angle=45,40,35\njoint01_min_angle=-15,-20,-25\n"
        "[butt_physics]\nstiffness=90\ndamping=8\n"
        "[body_colliders]\nenabled=true\n");
    write_fixture(preset,
        "[breasts_physics]\nstiffness=120\nwind_scale=0.7\njoint01_max_angle=30,31,32\njoint01_gain=0.5\n"
        "[penis_physics]\nstiffness=120\njoint01_max_angle=30,31,32\n"
        "[testicle_physics]\nstiffness=120\njoint01_max_angle=30,31,32\n"
        "[butt_physics]\nstiffness=120\n"
        "[body_colliders]\nenabled=false\n[defaults]\nconfig_reload_poll_ms=5000\n");
    write_fixture(second, "[breasts_physics]\ndamping=12\n");
    load_global_config();
    check_stiffness(90,90);
    default_wind = breasts_physics_global_cfg.wind_scale;
    assert(physx_preset_select("01_Test.ini"));
    refresh_global_config(tick++);
    check_stiffness(120,120);
    near_value(breasts_physics_global_cfg.damping,8);
    near_value(breasts_physics_global_cfg.link_max_angle[0][1],31);
    near_value(breasts_physics_global_cfg.link_min_angle[0][1],-20);
    near_value(breasts_physics_global_cfg.link_gain[0],.5f);
    near_value(breasts_physics_global_cfg.wind_scale,.7f);
    assert(defaults_cfg.config_reload_poll_ms==100);
    assert(body_chain_collider_global_cfg.enabled);
    puts("PASS: all four sections, missing-key fallback, aliases, ignored extra sections");

    write_fixture(sidecar,
        "[breasts_physics]\nstiffness=150\ncollision_strength=0.3\n"
        "[penis_physics]\nstiffness=150\n"
        "[testicle_physics]\nstiffness=150\n"
        "[butt_physics]\nstiffness=150\n");
    body_profile_person_sidecar_active[0]=1;
    strcpy(body_profile_person_sidecar_path[0],sidecar);
    get_file_write_time_a(sidecar,&body_profile_person_sidecar_write_time[0]);
    load_global_config();
    check_stiffness(120,150);
    near_value(breasts_physics_person_cfg[0].link_max_angle[0][1],31);
    near_value(body_chain_physics_person_cfg[0].link_max_angle[0][1],31);
    near_value(testicle_physics_person_cfg[0].link_max_angle[0][1],31);
    near_value(body_chain_physics_person_cfg[0].link_min_angle[0][1],-20);
    puts("PASS: per-person sidecars win and omitted joint angles retain lower layers");

    physx_write_slider_value(physx_settings_binding_by_name("NCPhysXBreastsCollisionStrength"),.6f);
    handle_physx_settings_change("NCPhysXPenisPhysics","OFF");
    handle_physx_settings_change("NCPhysXButtCollisionScope","hands_only");
    assert(physx_toggle_person_ini_setting("testicle_physics","test",1,1));
    GetPrivateProfileStringA("breasts_physics","collision_strength","",value,sizeof(value),preset);
    near_value((float)atof(value),.6f);
    GetPrivateProfileStringA("breasts_physics","collision_strength","",value,sizeof(value),config_path);
    near_value((float)atof(value),.8f);
    assert(raw_profile_key_exists_a("penis_physics","enabled",preset));
    assert(raw_profile_key_exists_a("butt_physics","collision_scope",preset));
    assert(raw_profile_key_exists_a("testicle_physics","enabled_person02",preset));
    assert(!strcmp(physx_settings_write_path("body_colliders"),config_path));
    load_global_config();
    near_value(breasts_physics_person_cfg[0].collision_strength,.3f);
    near_value(breasts_physics_person_cfg[1].collision_strength,.6f);
    puts("PASS: settings save to selected preset; global values and sidecars stay intact");

    assert(physx_preset_select("02_Second.ini"));
    refresh_global_config(tick++);
    check_stiffness(90,150);
    near_value(breasts_physics_global_cfg.damping,12);
    near_value(breasts_physics_global_cfg.wind_scale,default_wind);
    near_value(breasts_physics_global_cfg.link_max_angle[0][1],40);
    assert(physx_preset_select("DEFAULT"));
    refresh_global_config(tick++);
    check_stiffness(90,150);
    near_value(breasts_physics_global_cfg.damping,8);
    assert(!strcmp(physx_settings_write_path("breasts_physics"),config_path));
    puts("PASS: switching and Default discard previous overrides and retain sidecars");

    assert(physx_preset_select("02_Second.ini"));
    load_global_config();
    assert(WritePrivateProfileStringA("breasts_physics","stiffness","110",second));
    /* Explicit timestamp change avoids filesystem timestamp granularity in tests. */
    {
        HANDLE f=CreateFileA(second,FILE_WRITE_ATTRIBUTES,FILE_SHARE_READ|FILE_SHARE_WRITE,NULL,OPEN_EXISTING,0,NULL);
        FILETIME t=physx_preset_write_time;
        t.dwLowDateTime+=10000000;
        assert(f!=INVALID_HANDLE_VALUE && SetFileTime(f,NULL,NULL,&t));CloseHandle(f);
    }
    assert(!physx_preset_reload_due(200000));
    assert(!physx_preset_reload_due(200500));
    assert(physx_preset_reload_due(201501));
    load_global_config();
    near_value(breasts_physics_global_cfg.stiffness,110);
    assert(WritePrivateProfileStringA("breasts_physics","stiffness",NULL,second));
    load_global_config();
    near_value(breasts_physics_global_cfg.stiffness,90);
    assert(DeleteFileA(second));
    assert(physx_preset_reload_due(202000));
    load_global_config();
    assert(!strcmp(physx_preset_name,"DEFAULT"));
    GetPrivateProfileStringA("presets","selected","",value,sizeof(value),config_path);
    assert(!strcmp(value,"DEFAULT"));
    near_value(breasts_physics_global_cfg.damping,8);
    puts("PASS: live edits, deleted keys, removed file fallback, persisted selection");

    assert(!physx_preset_select("..\\outside.ini"));
    assert(!physx_preset_select("C:\\outside.ini"));
    assert(!physx_preset_select("bad.txt"));
    write_fixture(second,"[breasts_physics]\ndamping=10\n");
    list=physx_preset_scan();
    assert(list.count==3 && !strcmp(list.names[0],"01_Test.ini") && !strcmp(list.names[1],"02_Second.ini"));
    check_ui_arrays(&list);
    free(list.names);
    puts("PASS: sorted discovery and filename validation");

    /* A migration must not mistake preset values for old global settings. */
    write_fixture(config_path,"[body_chain_physics]\nenabled=true\nstiffness=88\n");
    assert(physx_preset_select("01_Test.ini"));
    migrate_legacy_penis_physics_section();
    GetPrivateProfileStringA("penis_physics","stiffness","",value,sizeof(value),config_path);
    near_value((float)atof(value),88);
    puts("PASS: legacy migration never copies preset values into the global file");
    return 0;
}
