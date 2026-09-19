/* Defer the actual Customizer entry command until the native GUI effect is
   black. Retain command arguments with NameHash's own reference-counted copy;
   never keep a borrowed callback stack pointer or stop presenting frames. */
typedef void *(THISCALL *physx_hash_copy_t)(void *, const void *);
typedef void (THISCALL *physx_hash_destroy_t)(void *);
typedef float (THISCALL *physx_effect_float_get_t)(void *, DWORD);
static physx_hash_copy_t physx_customizer_hash_copy;
static physx_hash_destroy_t physx_customizer_hash_destroy;
static struct {
    void *args, *image, *effect;
    DWORD start;
    unsigned int restore_visible;
    unsigned int inactive_person_mask;
    int phase, executing, ready_frames;
} physx_customizer_fade;

static void *physx_effect_dispatch(void *effect, unsigned int offset)
{
    BYTE *object=(BYTE*)effect, *metadata, *table;
    if (!object || !ptr_readable(object-0x18,sizeof(void*))) return NULL;
    metadata=*(BYTE**)(object-0x18);
    if (!metadata || !ptr_readable(metadata+0x310,sizeof(void*))) return NULL;
    table=*(BYTE**)(metadata+0x310);
    if (!table || !ptr_readable(table+offset,sizeof(void*))) return NULL;
    void *method=*(void**)(table+offset);
    return ptr_executable(method) ? method : NULL;
}

static int physx_customizer_effect_read(void *effect, int *target, float *current)
{
    script_u32_property_t get_target=(script_u32_property_t)physx_effect_dispatch(effect,0x40);
    physx_effect_float_get_t get_current=(physx_effect_float_get_t)physx_effect_dispatch(effect,0x100);
    if (!get_target || !get_current || !physx_effect_dispatch(effect,0x44)) return 0;
    *target=(int)get_target(effect,0x01fff0c4); /* WEffect.BlendTo */
    *current=get_current(effect,0x04fff0c4);  /* WEffect.BlendCurrent */
    return isfinite(*current) && *current>=0.f && *current<=1.f;
}

static void physx_customizer_effect_target(void *effect, int target)
{
    script_u32_set_property_t set=(script_u32_set_property_t)physx_effect_dispatch(effect,0x44);
    if (set) set(effect,0x01fff0c4,(unsigned int)target);
}

static int physx_customizer_entry_command(const char *name, void *args)
{
    char sub[64];
    if (!strcmp(name,"Customizer_Enter")) return 1;
    return !strcmp(name,"PersonContext_Cmd") &&
        app_main_command_string_arg(args,"SubCmd",sub,sizeof(sub)) &&
        !strcmp(sub,"Customizer_Enter");
}

static int physx_customizer_defer_entry(const char *name, void *args, DWORD now)
{
    void *image, *effect;
    unsigned int visible=0;
    int target=-1;
    float current=-1;
    if (physx_customizer_fade.executing || !real_AppMain_Command ||
        !physx_customizer_entry_command(name,args)) return 0;
    if (physx_customizer_fade.phase) return 1; /* double click: exactly once */
    if (physx_customizer_active) return 0;
    image=person_context_find_widget("GUI:SituationFade_Image");
    effect=person_context_find_widget("GUI:SituationFade_FX");
    if (!person_context_widget_visibility_get(image,&visible) ||
        !physx_customizer_effect_read(effect,&target,&current) || target!=0 || current>.001f) {
        log_line("physics Customizer fade unavailable visible=%u target=%d current=%.5f note=\"native command passes through; missing or already-owned fade\"",visible,target,current);
        return 0;
    }
    if (!physx_customizer_hash_copy || !physx_customizer_hash_destroy) {
        HMODULE sys=GetModuleHandleA("ThriXXX010278-SYS.dll");
        if (!sys) return 0;
        physx_customizer_hash_copy=(physx_hash_copy_t)GetProcAddress(sys,"??0NameHash@Bionic@@QAE@ABV01@@Z");
        physx_customizer_hash_destroy=(physx_hash_destroy_t)GetProcAddress(sys,"??1NameHash@Bionic@@QAE@XZ");
        if (!physx_customizer_hash_copy || !physx_customizer_hash_destroy) return 0;
    }
    if (!ptr_readable(args,sizeof(void*))) return 0;
    physx_customizer_hash_copy(&physx_customizer_fade.args,args);
    if (!person_context_widget_visibility_set(image,1)) {
        physx_customizer_hash_destroy(&physx_customizer_fade.args);
        physx_customizer_fade.args=NULL;
        return 0;
    }
    physx_customizer_fade.image=image;
    physx_customizer_fade.effect=effect;
    physx_customizer_fade.start=now;
    physx_customizer_fade.restore_visible=visible;
    physx_customizer_fade.phase=1;
    physx_customizer_fade.ready_frames=0;
    physx_customizer_effect_target(effect,1);
    log_line("physics Customizer fade begin command=\"%s\" note=\"native entry deferred until black; outgoing physics remains active\"",name);
    return 1;
}

static void physx_customizer_dispatch_entry(void)
{
    /* Detach the retained hash before native callbacks can re-enter us. */
    void *args=physx_customizer_fade.args;
    physx_customizer_fade.args=NULL;
    physx_customizer_fade.executing=1;
    DWORD result=real_AppMain_Command(&args);
    physx_customizer_fade.executing=0;
    physx_customizer_hash_destroy(&args);
    log_line("physics Customizer fade dispatched result=0x%08lx",(unsigned long)result);
}

/* In Customizer the other room actors retain initialized solver objects,
   but their live roots become placeholders. They cannot satisfy the gravity
   gate and must not hold the selected model behind the timeout. Unknown
   bindings remain conservative; never reveal while every known root is still
   a placeholder. This only scopes the fade, not simulation or camera guards. */
static int physx_customizer_person_present(int person_index)
{
    char name[256];
    int offset=body_chain_physics_person_cfg[person_index].root_offset;
    make_body_runtime_name(name,sizeof(name),body_chain_person_name(person_index),"root");
    void *root=resolve_axis_map_raw(name);
    if (!root || offset<0 || !ptr_readable((BYTE*)root+offset,sizeof(float)*3)) return -1;
    const float *position=(const float*)((BYTE*)root+offset);
    if (!physx_vec3_sane_limit(position,10000.f)) return -1;
    return !physics_environment_cfg.gravity_probe_require_nonzero_root ||
        physx_vec3_len(position)>physics_environment_cfg.gravity_probe_motion_epsilon;
}

static int physx_customizer_body_ready(void)
{
    int present[4], live_count=0;
    if (!physx_customizer_active || physx_customizer_entry_pending ||
        InterlockedCompareExchange(&body_chain_runtime_mode_transition_pending,0,0)) return 0;
    physx_customizer_fade.inactive_person_mask=0;
    for (int p=0;p<4;p++) {
        present[p]=physx_customizer_person_present(p);
        if (present[p]>0) live_count++;
        if (!present[p]) physx_customizer_fade.inactive_person_mask|=1u<<p;
    }
    if (physx_customizer_fade.inactive_person_mask && !live_count) return 0;
    for (int p=0;p<4;p++) {
        if (!present[p]) continue;
        body_chain_person_state_t *chains[2]={&runtime_body_chain_person_states[p],&runtime_testicle_physics_states[p]};
        body_chain_physics_config_t *configs[2]={&body_chain_physics_person_cfg[p],&testicle_physics_person_cfg[p]};
        for (int j=0;j<2;j++) if (configs[j]->enabled && configs[j]->enabled_person[p] && chains[j]->initialized) {
            if (physics_environment_cfg.world_gravity_probe && physics_environment_cfg.gravity_apply_to_body_chain &&
                (!chains[j]->gravity_probe_promoted ||
                 (physics_environment_cfg.gravity_dynamic_body_basis &&
                  !chains[j]->gravity_sample.accepted))) return 0;
        }
        breasts_physics_person_state_t *paired[2]={&breasts_physics_states[p],&butt_physics_states[p]};
        body_chain_physics_config_t *paired_configs[2]={&breasts_physics_person_cfg[p],&butt_physics_person_cfg[p]};
        for (int j=0;j<2;j++) {
            body_chain_physics_config_t *cfg=paired_configs[j];
            body_chain_person_state_t *gravity=j ? &paired[j]->motion : &paired[j]->gravity_motion;
            if (cfg->enabled && cfg->enabled_person[p] && paired[j]->initialized &&
                (fabsf(cfg->gravity_angle)>.000001f || cfg->gravity_inverted_strength>.000001f) &&
                physics_environment_cfg.world_gravity_probe && physics_environment_cfg.gravity_apply_to_body_chain &&
                (!gravity->gravity_probe_promoted ||
                 (physics_environment_cfg.gravity_dynamic_body_basis && !gravity->gravity_sample.accepted))) return 0;
        }
    }
    /* No active body solver is also ready (physics may all be disabled). */
    return 1;
}

static void physx_customizer_transition_tick(DWORD now)
{
    int target;
    float current;
    if (!physx_customizer_fade.phase || physx_customizer_fade.executing) return;
    int same=person_context_find_widget("GUI:SituationFade_Image")==physx_customizer_fade.image &&
        person_context_find_widget("GUI:SituationFade_FX")==physx_customizer_fade.effect;
    int readable=same && physx_customizer_effect_read(physx_customizer_fade.effect,&target,&current);
    DWORD elapsed=now-physx_customizer_fade.start;
    if (!readable || elapsed>=4000u) {
        if (physx_customizer_fade.phase==1) physx_customizer_dispatch_entry();
        if (readable) {
            physx_customizer_effect_target(physx_customizer_fade.effect,0);
            person_context_widget_visibility_set(physx_customizer_fade.image,physx_customizer_fade.restore_visible);
        }
        physx_customizer_fade.phase=0;
        log_line("physics Customizer fade fallback reason=%s",readable?"timeout":"GUI replaced");
        return;
    }
    if (physx_customizer_fade.phase==1) {
        if (target!=1) { /* A different native transition took ownership. */
            physx_customizer_hash_destroy(&physx_customizer_fade.args);
            physx_customizer_fade.args=NULL;
            physx_customizer_fade.phase=0;
            log_line("physics Customizer fade cancelled reason=\"native fade ownership changed before entry\"");
            return;
        }
        if (current<.9999f) return;
        physx_customizer_fade.phase=2;
        physx_customizer_fade.ready_frames=0;
        log_line("physics Customizer fade black note=\"dispatching native entry under black\"");
        physx_customizer_dispatch_entry();
        if (person_context_find_widget("GUI:SituationFade_FX")==physx_customizer_fade.effect &&
            person_context_find_widget("GUI:SituationFade_Image")==physx_customizer_fade.image) {
            person_context_widget_visibility_set(physx_customizer_fade.image,1);
            physx_customizer_effect_target(physx_customizer_fade.effect,1);
        }
        return;
    }
    if (physx_customizer_fade.phase==2) {
        /* Keep the existing GUI fade black while native entry and the first
           physics updates run. Do not wait for the camera to stop. */
        physx_customizer_effect_target(physx_customizer_fade.effect,1);
        if (physx_customizer_body_ready()) physx_customizer_fade.ready_frames++;
        else physx_customizer_fade.ready_frames=0;
        if (physx_customizer_fade.ready_frames<2 && elapsed<3000u) return;
        physx_customizer_effect_target(physx_customizer_fade.effect,0);
        physx_customizer_fade.phase=3;
        log_line("physics Customizer fade reveal ready_frames=%d elapsed_ms=%lu inactive_person_mask=0x%x reason=%s",
            physx_customizer_fade.ready_frames,(unsigned long)elapsed,
            physx_customizer_fade.inactive_person_mask,
            physx_customizer_fade.ready_frames>=2 ? "ready" : "timeout");
        return;
    }
    if (target!=0) { physx_customizer_fade.phase=0; return; }
    if (current<=.0001f) {
        person_context_widget_visibility_set(physx_customizer_fade.image,physx_customizer_fade.restore_visible);
        physx_customizer_fade.phase=0;
    }
}
