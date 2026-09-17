
#include <windows.h>
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#define THISCALL __attribute__((thiscall))
#define SCRIPT_OBJECT_META_BACK_OFFSET 0x18
static char config_path[MAX_PATH*4];
static int ini_writes;
static BOOL counted_ini_write(const char *section,const char *key,const char *value,const char *path){
 ini_writes++;
 return WritePrivateProfileStringA(section,key,value,path);
}
#define WritePrivateProfileStringA counted_ini_write
static int physx_settings_sync_depth;
static struct {int debug;} defaults_cfg;
static float physx_clampf(float v,float lo,float hi){return fmaxf(lo,fminf(hi,v));}
static int ptr_readable(const void *p,size_t n){(void)n;return p!=NULL;}
static int ptr_executable(const void *p){return p!=NULL;}
static void log_line(const char *fmt,...){(void)fmt;}
static void config_file_path(char *p,size_t n){(void)p;(void)n;assert(0);}
static const char *stringref_cstr_a(const char *p){return p;}
static void migrate_legacy_penis_physics_section(void){}
static void physx_mark_penis_physics_setting_change(const char *k,int v){(void)k;(void)v;}
static void physx_mark_testicle_physics_setting_change(const char *k,int v){(void)k;(void)v;}
static void physx_mark_penis_collision_setting_change(int v){(void)v;}
static int physx_custom_parameter_name(void *p,char *out,size_t n){lstrcpynA(out,p,(int)n);return 1;}
#define BREASTS_PHYSICS_CONFIG_SECTION "breasts_physics"
#define PENIS_PHYSICS_CONFIG_SECTION "penis_physics"
#define TESTICLE_PHYSICS_CONFIG_SECTION "testicle_physics"
#define BUTT_PHYSICS_CONFIG_SECTION "butt_physics"
#define BODY_COLLIDERS_CONFIG_SECTION "body_colliders"
typedef struct physx_settings_binding_t {
    const char *param_name;
    const char *section;
    const char *key;
    void *slider_widget;
} physx_settings_binding_t;static physx_settings_binding_t physx_settings_bindings[] = {
    { "NCPhysXBreastsPhysics", BREASTS_PHYSICS_CONFIG_SECTION, "enabled", NULL },
    { "NCPhysXPenisPhysics", PENIS_PHYSICS_CONFIG_SECTION, "enabled", NULL },
    { "NCPhysXTesticlePhysics", TESTICLE_PHYSICS_CONFIG_SECTION, "enabled", NULL },
    { "NCPhysXButtPhysics", BUTT_PHYSICS_CONFIG_SECTION, "enabled", NULL },
    { "NCPhysXBodyColliders", BODY_COLLIDERS_CONFIG_SECTION, "enabled", NULL },
    { "NCPhysXBreastsCollision", BODY_COLLIDERS_CONFIG_SECTION, "breasts_collision_enabled", NULL },
    { "NCPhysXButtCollision", BODY_COLLIDERS_CONFIG_SECTION, "butt_collision_enabled", NULL },
    { "NCPhysXPenisCollision", BODY_COLLIDERS_CONFIG_SECTION, "penis_collision_enabled", NULL },
    { "NCPhysXTesticleCollision", BODY_COLLIDERS_CONFIG_SECTION, "testicle_collision_enabled", NULL },
    { "NCPhysXBreastsCollisionScope", BREASTS_PHYSICS_CONFIG_SECTION, "collision_scope", NULL },
    { "NCPhysXButtCollisionScope", BUTT_PHYSICS_CONFIG_SECTION, "collision_scope", NULL },
    { "NCPhysXPenisCollisionScope", PENIS_PHYSICS_CONFIG_SECTION, "collision_scope", NULL },
    { "NCPhysXTesticleCollisionScope", TESTICLE_PHYSICS_CONFIG_SECTION, "collision_scope", NULL },
    { "NCPhysXColliderVisuals", BODY_COLLIDERS_CONFIG_SECTION, "debug_draw", NULL },
    { "NCPhysXPauseHiddenGenitals", "defaults", "pause_hidden_genitals", NULL },
    { "NCPhysXWorldGravity", "physics_environment", "gravity_apply_to_body_chain", NULL },
    { "NCPhysXWorldWind", "physics_environment", "wind_enabled", NULL },
    { "NCPhysXBreastsRoomCollision", BREASTS_PHYSICS_CONFIG_SECTION, "room_collision_enabled", NULL },
    { "NCPhysXPenisRoomCollision", PENIS_PHYSICS_CONFIG_SECTION, "room_collision_enabled", NULL },
    { "NCPhysXTesticleRoomCollision", TESTICLE_PHYSICS_CONFIG_SECTION, "room_collision_enabled", NULL },
    { "NCPhysXButtRoomCollision", BUTT_PHYSICS_CONFIG_SECTION, "room_collision_enabled", NULL },
    { "NCPhysXBreastsCollisionStrength", BREASTS_PHYSICS_CONFIG_SECTION, "collision_strength", NULL },
    { "NCPhysXPenisCollisionStrength", PENIS_PHYSICS_CONFIG_SECTION, "collision_strength", NULL },
    { "NCPhysXTesticleCollisionStrength", TESTICLE_PHYSICS_CONFIG_SECTION, "collision_strength", NULL },
    { "NCPhysXButtCollisionStrength", BUTT_PHYSICS_CONFIG_SECTION, "collision_strength", NULL }
};static physx_settings_binding_t *physx_settings_binding_by_name(
    const char *name)
{
    size_t i;
    if (!name) return NULL;
    for (i = 0; i < sizeof(physx_settings_bindings) /
                        sizeof(physx_settings_bindings[0]); i++) {
        if (strcmp(name, physx_settings_bindings[i].param_name) == 0)
            return &physx_settings_bindings[i];
    }
    return NULL;
}
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
static float profile_collision_strength(const char *section,float fallback,const char *path)
{
    char buf[128],*end;float value;
    GetPrivateProfileStringA(section,"collision_strength","",buf,sizeof(buf),path);
    if(!buf[0]) return fallback;
    value=strtof(buf,&end);
    if(end==buf) return fallback;
    while(*end==' ' || *end=='\t') end++;
    if(*end) return fallback;
    return isfinite(value)?physx_clampf(value,0.1f,1.0f):fallback;
}
static int physx_slider_widget_value(void *slider,float *out_value)
{
    typedef float (THISCALL *get_float_t)(void *,DWORD);
    BYTE *metadata,*dispatch_table;get_float_t getter;float value;
    if(!slider || !out_value ||
       !ptr_readable((BYTE*)slider-SCRIPT_OBJECT_META_BACK_OFFSET,sizeof(metadata))) return 0;
    memcpy(&metadata,(BYTE*)slider-SCRIPT_OBJECT_META_BACK_OFFSET,sizeof(metadata));
    if(!metadata || !ptr_readable(metadata+0x3b4,sizeof(dispatch_table))) return 0;
    memcpy(&dispatch_table,metadata+0x3b4,sizeof(dispatch_table));
    if(!dispatch_table || !ptr_readable(dispatch_table+0x80,sizeof(getter))) return 0;
    memcpy(&getter,dispatch_table+0x80,sizeof(getter));
    if(!ptr_executable((const void*)getter)) return 0;
    value=getter(slider,0x02fff0ed);
    if(!isfinite(value)) return 0;
    *out_value=value;
    return 1;
}
static int physx_slider_widget_set_value(void *slider,float value)
{
    typedef void (THISCALL *set_float_t)(void *,DWORD,float);
    BYTE *metadata,*dispatch_table;set_float_t setter;
    if(!slider || !isfinite(value) ||
       !ptr_readable((BYTE*)slider-SCRIPT_OBJECT_META_BACK_OFFSET,sizeof(metadata))) return 0;
    memcpy(&metadata,(BYTE*)slider-SCRIPT_OBJECT_META_BACK_OFFSET,sizeof(metadata));
    if(!metadata || !ptr_readable(metadata+0x3b4,sizeof(dispatch_table))) return 0;
    memcpy(&dispatch_table,metadata+0x3b4,sizeof(dispatch_table));
    if(!dispatch_table || !ptr_readable(dispatch_table+0x84,sizeof(setter))) return 0;
    memcpy(&setter,dispatch_table+0x84,sizeof(setter));
    if(!ptr_executable((const void*)setter)) return 0;
    setter(slider,0x02fff0ed,value);
    return 1;
}
static void physx_sync_slider_from_ini(const physx_settings_binding_t *binding,void *widget)
{
    float requested,current;int synced;
    if(!binding || !widget) return;
    if(!config_path[0]) config_file_path(config_path,sizeof(config_path));
    requested=profile_collision_strength(binding->section,1.0f,config_path);
    if(physx_slider_widget_value(widget,&current) && fabsf(current-requested)<=.00001f) return;
    physx_settings_sync_depth++;
    synced=physx_slider_widget_set_value(widget,requested);
    physx_settings_sync_depth--;
    if(!synced || defaults_cfg.debug)
        log_line("settings slider sync param=\"%s\" ini=%.6g success=%d",binding->param_name,requested,synced);
}
static void physx_write_slider_value(const physx_settings_binding_t *binding, float value)
{
    char normalized[32], saved[32];
    if (physx_settings_sync_depth || !binding || !isfinite(value)) return;
    if (!config_path[0]) config_file_path(config_path, sizeof(config_path));
    value = physx_clampf(value, .1f, 1.0f);
    _snprintf(normalized, sizeof(normalized), "%.6g", value);
    normalized[sizeof(normalized) - 1] = 0;
    /* TK17 can emit the same change more than once. Compare the persisted
       text instead of a cached value so external INI edits still take effect. */
    GetPrivateProfileStringA(binding->section, binding->key, "", saved,
                             sizeof(saved), config_path);
    if (strcmp(saved, normalized) == 0) {
        physx_sync_slider_from_ini(binding, binding->slider_widget);
        return;
    }
    if (WritePrivateProfileStringA(binding->section, binding->key, normalized, config_path)) {
        log_line("settings slider saved param=\"%s\" value=%s ini=[%s] %s note=\"normal INI hot-reload scheduled\"",
                 binding->param_name, normalized, binding->section, binding->key);
        physx_sync_slider_from_ini(binding, binding->slider_widget);
    } else {
        log_line("settings write failed param=\"%s\" ini=[%s] %s path=\"%s\"",
                 binding->param_name, binding->section, binding->key, config_path);
    }
}
static void physx_write_slider_setting(const physx_settings_binding_t *binding,const char *text)
{
    float value;char *end;
    if(physx_settings_sync_depth || !binding) return;
    if(!physx_slider_widget_value(binding->slider_widget,&value)) {
        if(!text || !text[0]) return;
        value=strtof(text,&end);
        if(end==text || !isfinite(value)) return;
        while(*end==' ' || *end=='\t') end++;
        if(*end) return;
    }
    physx_write_slider_value(binding,value);
}
static void handle_physx_settings_change(const char *param_ref,
                                         const char *value_ref)
{
    const char *param_name = stringref_cstr_a(param_ref);
    const char *string_value = stringref_cstr_a(value_ref);
    const char *collision_scope;
    int enabled;
    size_t i;
    if (physx_settings_sync_depth || !param_name || _strnicmp(param_name, "NCPhysX", 7) != 0) return;
    if (!config_path[0]) config_file_path(config_path, sizeof(config_path));
    migrate_legacy_penis_physics_section();
    for (i = 0; i < sizeof(physx_settings_bindings) / sizeof(physx_settings_bindings[0]); i++) {
        const physx_settings_binding_t *binding = &physx_settings_bindings[i];
        if (strcmp(param_name, binding->param_name) != 0) continue;
        if (strcmp(binding->key, "collision_strength") == 0) {
            physx_write_slider_setting(binding,string_value);
            return;
        }
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

static int spinbox_syncs;
static void physx_sync_spinbox_from_ini(const physx_settings_binding_t *b,void *w){
 assert(!strcmp(b->key,"enabled"));assert(w==(void*)0x1234);spinbox_syncs++;
}
static int original_build(void *self,void *a,void *b,void *c){
 (void)self;(void)a;(void)b;(void)c;
 assert(physx_settings_sync_depth>0);
 handle_physx_settings_change("NCPhysXBreastsCollisionStrength","0.99");
 return 42;
}
static int (*real_Customizer_BuildControls)(void *,void *,void *,void *)=original_build;
static int physx_build_controls_and_sync(
    void *self, void *arg1, void *arg2, void *arg3)
{
    int result;
    void **parameters = NULL;
    void **records = NULL;
    int parameter_count = 0;
    int record_count = 0;
    int count;
    int index;
    result = real_Customizer_BuildControls ?
        real_Customizer_BuildControls(self, arg1, arg2, arg3) : 0;
    if (!self ||
        !ptr_readable((BYTE*)self + 0x14, sizeof(parameters)) ||
        !ptr_readable((BYTE*)self + 0x18, sizeof(records))) return result;
    memcpy(&parameters, (BYTE*)self + 0x14, sizeof(parameters));
    memcpy(&records, (BYTE*)self + 0x18, sizeof(records));
    if (!parameters || !records ||
        !ptr_readable((BYTE*)parameters - sizeof(parameter_count),
                      sizeof(parameter_count)) ||
        !ptr_readable((BYTE*)records - sizeof(record_count),
                      sizeof(record_count))) return result;
    memcpy(&parameter_count,
           (BYTE*)parameters - sizeof(parameter_count),
           sizeof(parameter_count));
    memcpy(&record_count, (BYTE*)records - sizeof(record_count),
           sizeof(record_count));
    if (parameter_count <= 0 || record_count <= 0 ||
        parameter_count > 4096 || record_count > 4096) return result;
    count = parameter_count < record_count ? parameter_count : record_count;
    if (!ptr_readable(parameters, (size_t)count * sizeof(*parameters)) ||
        !ptr_readable(records, (size_t)count * sizeof(*records))) return result;
    for (index = 0; index < count; index++) {
        char parameter_name[128];
        void *record = records[index];
        void *widget = NULL;
        physx_settings_binding_t *binding;
        if (!parameters[index] || !record ||
            !physx_custom_parameter_name(parameters[index], parameter_name,
                                         sizeof(parameter_name))) continue;
        binding = physx_settings_binding_by_name(parameter_name);
        if (binding && strcmp(binding->key, "collision_strength") == 0) {
            /* Main slider is record+0x04; preset slots start at +0x08 and
               must retain their own preset values. Spinboxes use +0x24. */
            if (ptr_readable((BYTE*)record + 0x04, sizeof(widget))) {
                memcpy(&widget, (BYTE*)record + 0x04, sizeof(widget));
                binding->slider_widget = widget;
                if (widget) physx_sync_slider_from_ini(binding, widget);
            }
            continue;
        }
        if (!binding ||
            !ptr_readable((BYTE*)record + 0x24, sizeof(widget))) continue;
        memcpy(&widget, (BYTE*)record + 0x24, sizeof(widget));
        if (widget) physx_sync_spinbox_from_ini(binding, widget);
    }
    return result;
}
static int THISCALL hook_Customizer_BuildControls_PhysX(
    void *self,void *arg1,void *arg2,void *arg3)
{
    int result;size_t i;
    for(i=0;i<sizeof(physx_settings_bindings)/sizeof(physx_settings_bindings[0]);i++)
        physx_settings_bindings[i].slider_widget=NULL;
    /* Original BuildControls can emit ParamChange while restoring saved UI
       values. Keep Config.ini authoritative throughout creation and sync. */
    physx_settings_sync_depth++;
    result=physx_build_controls_and_sync(self,arg1,arg2,arg3);
    physx_settings_sync_depth--;
    return result;
}

static void fixture_param_change(void *,const char *,const char *,DWORD,DWORD);
static void (*real_ConfigEditor_ParamChange)(void *,const char *,const char *,DWORD,DWORD)=fixture_param_change;
static int create_calls;
static int fixture_create(void *s,void *p,void *r,void *parent,float y,int preset,int labels){
 (void)s;(void)p;(void)r;(void)parent;(void)y;(void)preset;(void)labels;create_calls++;return 17;
}
static int (*real_PhysX_CreateSlider)(void *,void *,void *,void *,float,int,int)=fixture_create;
static int THISCALL hook_PhysX_CreateSlider(void *self,void *parameter,void *record,
    void *parent,float y,int preset_index,int has_labels)
{
    char name[128];void *widget=NULL;physx_settings_binding_t *binding;
    int result=real_PhysX_CreateSlider ? real_PhysX_CreateSlider(self,parameter,record,parent,y,preset_index,has_labels):0;
    if(preset_index>=0 || !record || !physx_custom_parameter_name(parameter,name,sizeof(name))) return result;
    binding=physx_settings_binding_by_name(name);
    if(!binding || strcmp(binding->key,"collision_strength") ||
       !ptr_readable((BYTE*)record+4,sizeof(widget))) return result;
    memcpy(&widget,(BYTE*)record+4,sizeof(widget));
    binding->slider_widget=widget;
    if(widget) physx_sync_slider_from_ini(binding,widget);
    return result;
}
static void THISCALL hook_ConfigEditor_ParamChange(void *self,
                                                   const char *param_name,
                                                   const char *string_value,
                                                   DWORD value_arg,
                                                   DWORD event_arg)
{
    char param_copy[96];
    char value_copy[32];
    const char *param_cstr = stringref_cstr_a(param_name);
    const char *value_cstr = stringref_cstr_a(string_value);
    int is_physx = param_cstr &&
                   _strnicmp(param_cstr, "NCPhysX", 7) == 0;
    const physx_settings_binding_t *slider_binding = NULL;
    float slider_value = 0;
    int slider_captured = 0;
    int was_syncing = physx_settings_sync_depth != 0;
    param_copy[0] = 0;
    value_copy[0] = 0;
    if (is_physx) {
        lstrcpynA(param_copy, param_cstr, sizeof(param_copy));
        if (value_cstr) lstrcpynA(value_copy, value_cstr, sizeof(value_copy));
        slider_binding = physx_settings_binding_by_name(param_copy);
        if (!was_syncing && slider_binding &&
            strcmp(slider_binding->key, "collision_strength") == 0) {
            /* TK17's handler can rebuild controls. Capture the user's live
               value before it can be replaced with the old INI/UI value. */
            slider_captured = physx_slider_widget_value(
                slider_binding->slider_widget, &slider_value);
        }
    }
    if (real_ConfigEditor_ParamChange) {
        real_ConfigEditor_ParamChange(self, param_name, string_value,
                                      value_arg, event_arg);
    }
    if (is_physx && !was_syncing && !physx_settings_sync_depth) {
        if (slider_captured) {
            physx_write_slider_value(slider_binding, slider_value);
        } else {
            handle_physx_settings_change(param_copy,
                                         value_copy[0] ? value_copy : NULL);
        }
    }
}

static void (__cdecl *real_PhysX_ConfigEditorCallback)(void *,const char *,const char *,DWORD,const float *);
static void __cdecl hook_PhysX_ConfigEditorCallback(
    void *self, const char *parameter, const char *text, DWORD value_arg,
    const float *numeric_value)
{
    const char *name = stringref_cstr_a(parameter);
    const physx_settings_binding_t *binding = physx_settings_binding_by_name(name);
    int is_strength = binding && strcmp(binding->key, "collision_strength") == 0;
    int was_syncing = physx_settings_sync_depth != 0;
    int captured = 0;
    float value = 0;
    if (is_strength && !was_syncing &&
        ptr_readable(numeric_value, sizeof(value))) {
        memcpy(&value, numeric_value, sizeof(value));
        captured = isfinite(value);
    }
    /* Prevent the downstream text handler from saving an old widget value,
       and prevent any native rebuild from recursively persisting defaults. */
    if (is_strength) physx_settings_sync_depth++;
    if (real_PhysX_ConfigEditorCallback)
        real_PhysX_ConfigEditorCallback(self, parameter, text, value_arg, numeric_value);
    if (is_strength) physx_settings_sync_depth--;
    if (captured) {
        physx_write_slider_value(binding, value);
    } else if (is_strength && !was_syncing) {
        log_line("settings slider event rejected param=\"%s\" reason=\"missing or non-finite numeric payload\"",
                 binding->param_name);
    }
}

static const char *names[]={"NCPhysXBreastsCollisionStrength","NCPhysXPenisCollisionStrength",
 "NCPhysXTesticleCollisionStrength","NCPhysXButtCollisionStrength"};
typedef struct {BYTE *meta;BYTE pad[20];float value;} slider_t;
static slider_t sliders[4],presets[4];
static int setter_calls;
static void fixture_param_change(void *s,const char *p,const char *v,DWORD a,DWORD b){
 (void)s;(void)p;(void)v;(void)a;(void)b;
 /* Reproduce a native handler restoring/replacing the live value. */
 sliders[0].value=1;
}
static float THISCALL slider_get(void *self,DWORD member){assert(member==0x02fff0ed);return *(float*)self;}
static void THISCALL slider_set(void *self,DWORD member,float value){
 assert(member==0x02fff0ed);*(float*)self=value;setter_calls++;
 for(int i=0;i<4;i++)if(self==&sliders[i].value)handle_physx_settings_change(names[i],"stale");
}
static void assert_close(float a,float b){assert(fabsf(a-b)<1e-6f);}
int main(int argc,char **argv){
 assert(argc==2);lstrcpynA(config_path,argv[1],sizeof(config_path));
 BYTE metadata[0x3b8]={0},dispatch[0x88]={0},self[0x1c]={0},record[5][0x30]={{0}};
 BYTE *d=dispatch;memcpy(metadata+0x3b4,&d,sizeof(d));
 float (THISCALL *getter)(void *,DWORD)=slider_get;
 void (THISCALL *setter)(void *,DWORD,float)=slider_set;
 memcpy(dispatch+0x80,&getter,sizeof(getter));memcpy(dispatch+0x84,&setter,sizeof(setter));
 struct {int count;void *items[5];} parameters={5,{0}},records={5,{0}};
 void **p=parameters.items,**r=records.items;memcpy(self+0x14,&p,sizeof(p));memcpy(self+0x18,&r,sizeof(r));
 const char *values[]={"0.2","0.35","0.6","0.9"};
 for(int i=0;i<4;i++){
  physx_settings_binding_t *binding=physx_settings_binding_by_name(names[i]);assert(binding);
  assert(!strcmp(binding->key,"collision_strength"));
  sliders[i].meta=presets[i].meta=metadata;sliders[i].value=1;presets[i].value=.75f;
  void *widget=&sliders[i].value,*preset=&presets[i].value;
  memcpy(record[i]+4,&widget,sizeof(widget));memcpy(record[i]+8,&preset,sizeof(preset));
  parameters.items[i]=(void*)names[i];records.items[i]=record[i];
  WritePrivateProfileStringA(binding->section,binding->key,values[i],config_path);
 }
 parameters.items[4]="NCPhysXBreastsPhysics";records.items[4]=record[4];
 void *spin=(void*)0x1234;memcpy(record[4]+0x24,&spin,sizeof(spin));
 assert(hook_Customizer_BuildControls_PhysX(self,0,0,0)==42);
 assert(physx_settings_sync_depth==0 && setter_calls==4 && spinbox_syncs==1);
 for(int i=0;i<4;i++){
  physx_settings_binding_t *binding=physx_settings_binding_by_name(names[i]);
  assert_close(sliders[i].value,(float)atof(values[i]));assert_close(presets[i].value,.75f);
  assert_close(profile_collision_strength(binding->section,1,config_path),(float)atof(values[i]));
  assert(binding->slider_widget==&sliders[i].value);
  sliders[i].value=.125f+.1f*i;handle_physx_settings_change(names[i],"Weak");
  assert_close(profile_collision_strength(binding->section,1,config_path),sliders[i].value);
 }
 puts("PASS: all four live sliders load INI values and write native floats to the correct sections; initialization callbacks cannot overwrite INI; presets and spinboxes remain separate");
 physx_settings_binding_t *binding=physx_settings_binding_by_name(names[0]);
 sliders[0].value=0;handle_physx_settings_change(names[0],NULL);assert_close(profile_collision_strength(binding->section,1,config_path),.1f);
 sliders[0].value=2;handle_physx_settings_change(names[0],NULL);assert_close(profile_collision_strength(binding->section,1,config_path),1);
 sliders[0].value=NAN;handle_physx_settings_change(names[0],"invalid");assert_close(profile_collision_strength(binding->section,1,config_path),1);
 binding->slider_widget=NULL;handle_physx_settings_change(names[0],"0.375");assert_close(profile_collision_strength(binding->section,1,config_path),.375f);
 handle_physx_settings_change(names[0],"0.5oops");assert_close(profile_collision_strength(binding->section,1,config_path),.375f);
 WritePrivateProfileStringA(binding->section,binding->key,"0.45",config_path);
 hook_Customizer_BuildControls_PhysX(self,0,0,0);assert_close(sliders[0].value,.45f);
 WritePrivateProfileStringA(binding->section,binding->key,NULL,config_path);
 hook_Customizer_BuildControls_PhysX(self,0,0,0);assert_close(sliders[0].value,1);
 char text[32];GetPrivateProfileStringA(binding->section,binding->key,"missing",text,sizeof(text),config_path);assert(!strcmp(text,"missing"));
 assert(hook_Customizer_BuildControls_PhysX(NULL,0,0,0)==42 && physx_settings_sync_depth==0);
 for(int i=0;i<4;i++)assert(physx_settings_binding_by_name(names[i])->slider_widget==NULL);
 WritePrivateProfileStringA(binding->section,binding->key,"0.6",config_path);
 assert(hook_PhysX_CreateSlider(NULL,(void*)names[0],record[0],NULL,0,-1,0)==17);
 assert(binding->slider_widget==&sliders[0].value);assert_close(sliders[0].value,.6f);
 assert(hook_PhysX_CreateSlider(NULL,(void*)names[0],record[0],NULL,0,0,1)==17);
 assert(binding->slider_widget==&sliders[0].value);assert_close(presets[0].value,.75f);
 sliders[0].value=.1f;
 hook_ConfigEditor_ParamChange(NULL,names[0],NULL,0,0);
 assert_close(profile_collision_strength(binding->section,1,config_path),.1f);
 assert_close(sliders[0].value,.1f);
 assert(create_calls==2);
 puts("PASS: creation hook registers the main slider without capturing presets; actual ParamChange hook preserves the incoming 0.1 even when TK17 resets the widget to 1 during its callback");
 /* Execute TK17's actual callback adapter, including its discarded numeric
    payload and stack padding. Only relocate its call to the production hook. */
 const BYTE adapter_bytes[]={0x55,0x8b,0xec,0x8b,0x4d,0x08,0x83,0xec,0x08,
  0xff,0x75,0x10,0xff,0x75,0x0c,0xe8,0x5c,0xed,0xff,0xff,0x5d,0xc3};
 BYTE *adapter=VirtualAlloc(NULL,sizeof(adapter_bytes),MEM_COMMIT|MEM_RESERVE,PAGE_EXECUTE_READWRITE);
 assert(adapter);memcpy(adapter,adapter_bytes,sizeof(adapter_bytes));
 DWORD relative=(DWORD)((BYTE*)hook_ConfigEditor_ParamChange-(adapter+20));
 memcpy(adapter+16,&relative,sizeof(relative));
 FlushInstructionCache(GetCurrentProcess(),adapter,sizeof(adapter_bytes));
 real_PhysX_ConfigEditorCallback=(void*)adapter;
 for(int i=0;i<4;i++){
  binding=physx_settings_binding_by_name(names[i]);binding->slider_widget=NULL;
  WritePrivateProfileStringA(binding->section,binding->key,"0.1",config_path);
  float payload[4]={1,0,0,0};
  /* Old path reproduces the bug: the adapter loses the float, INI stays .1. */
  real_PhysX_ConfigEditorCallback(NULL,names[i],"",0,payload);
  assert_close(profile_collision_strength(binding->section,1,config_path),.1f);
  hook_PhysX_ConfigEditorCallback(NULL,names[i],"",0,payload);
  assert_close(profile_collision_strength(binding->section,1,config_path),1);
  payload[0]=.1f;hook_PhysX_ConfigEditorCallback(NULL,names[i],NULL,0,payload);
  assert_close(profile_collision_strength(binding->section,1,config_path),.1f);
  physx_settings_sync_depth++;payload[0]=1;
  hook_PhysX_ConfigEditorCallback(NULL,names[i],NULL,0,payload);
  assert(physx_settings_sync_depth==1);physx_settings_sync_depth--;
  assert_close(profile_collision_strength(binding->section,1,config_path),.1f);
  payload[0]=NAN;hook_PhysX_ConfigEditorCallback(NULL,names[i],NULL,0,payload);
  hook_PhysX_ConfigEditorCallback(NULL,names[i],NULL,0,NULL);
  assert_close(profile_collision_strength(binding->section,1,config_path),.1f);
 }
 assert(physx_settings_sync_depth==0);
 binding=physx_settings_binding_by_name(names[0]);binding->slider_widget=&sliders[0].value;
 sliders[0].value=1;
 float incoming[4]={.25f,0,0,0};
 hook_PhysX_ConfigEditorCallback(NULL,names[0],"",0,incoming);
 assert_close(profile_collision_strength(binding->section,1,config_path),.25f);
 assert_close(sliders[0].value,.25f);
 int writes_before=ini_writes;
 /* Repeated notifications must still restore the widget after native reset,
    without rewriting the INI or scheduling another reload. */
 hook_PhysX_ConfigEditorCallback(NULL,names[0],"",0,incoming);
 assert(ini_writes==writes_before);
 assert_close(sliders[0].value,.25f);
 WritePrivateProfileStringA(binding->section,binding->key,"0.7",config_path);
 writes_before=ini_writes;
 hook_PhysX_ConfigEditorCallback(NULL,names[0],"",0,incoming);
 assert(ini_writes==writes_before+1);
 assert_close(profile_collision_strength(binding->section,1,config_path),.25f);
 puts("PASS: duplicate native notifications skip INI writes while restoring the widget; external INI edits are not hidden by a value cache");
 hook_PhysX_ConfigEditorCallback(NULL,"NCPhysXBreastsPhysics","OFF",0,incoming);
 GetPrivateProfileStringA(binding->section,"enabled","missing",text,sizeof(text),config_path);
 assert(!strcmp(text,"false"));
 hook_PhysX_ConfigEditorCallback(NULL,"AnotherPluginSlider",NULL,0,incoming);
 assert_close(profile_collision_strength(binding->section,1,config_path),.25f);
 VirtualFree(adapter,0,MEM_RELEASE);
 puts("PASS: actual x86 callback adapter reproduces the missing numeric value; entry hook saves 0.1 -> 1 -> 0.1 for all four sections without widgets, rejects invalid payloads and suppresses initialization events");
 puts("PASS: range clamps, invalid event rejection, numeric fallback, INI edits on reopen, missing-key default, stale-widget reset and balanced sync guard");
 return 0;
}
