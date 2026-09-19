/* Opt-in pause: body02/body03 only, unknown visibility keeps physics active.
   Geometry/materials and the user's per-person enable flags are never changed. */
static volatile LONG physx_genital_pause_mask;

static int physx_genitals_paused(int person_index)
{
    if(!physx_pause_hidden_genitals || person_index<0 || person_index>=4) return 0;
    return (InterlockedCompareExchange(&physx_genital_pause_mask,0,0) &
        (1L<<person_index))!=0;
}

static int physx_genital_pause_candidate(int enabled, int body_slot, int known,
    DWORD visibility)
{
    return enabled && (body_slot==1 || body_slot==2) && known && visibility==0;
}

static void physx_genital_visibility_publish_mask(LONG pause_mask)
{
    LONG old_mask=InterlockedExchange(&physx_genital_pause_mask,pause_mask);
    int person;
    for(person=0;person<4;++person) {
        LONG bit=1L<<person;
        if((old_mask^pause_mask)&bit)
            log_line("genital physics state person=%s physics_paused=%d reason=%s",
                body_chain_person_name(person),(pause_mask&bit)!=0,
                (pause_mask&bit)?"clothing-hidden":"visible-unknown-or-option-disabled");
    }
}
/* Installed SYS getters: TNode.Visibility (0x06FFF043) delegates to
   SNode.Visibility (0x01FFF042). Neither signature contains relocations. */
static const BYTE physx_genital_tnode_getter[]={
    0x8b,0x49,0x10,0x68,0x42,0xf0,0xff,0x01,0x8b,0x41,0xe8,
    0x8b,0x80,0x08,0x01,0x00,0x00,0x8b,0x40,0x40,0xff,0xd0,0xc2,0x04,0x00};
static const BYTE physx_genital_snode_getter[]={0x8b,0x41,0x10,0xc2,0x04,0x00};

static int physx_genital_visibility_dispatch_matches(BYTE *obj, size_t class_offset,
    size_t getter_offset, const BYTE *signature, size_t signature_size)
{
    BYTE *metadata, *property, *getter;
    if(!obj || !ptr_readable(obj-0x18,sizeof(void *))) return 0;
    metadata=*(BYTE **)(obj-0x18);
    if(!metadata || !ptr_readable(metadata+class_offset,sizeof(void *))) return 0;
    property=*(BYTE **)(metadata+class_offset);
    if(!property || !ptr_readable(property-0x1c,sizeof(DWORD)) ||
        !*(DWORD *)(property-0x1c) || !ptr_readable(property+getter_offset,sizeof(void *))) return 0;
    getter=*(BYTE **)(property+getter_offset);
    return ptr_executable(getter) && ptr_readable(getter,signature_size) &&
        !memcmp(getter,signature,signature_size);
}

static int physx_genital_visibility_read(void *object, DWORD *value, const char **reason)
{
    BYTE *obj=(BYTE *)object, *snode;
    if(reason) *reason="invalid-input";
    if(!obj || !value) return 0;
    if(reason) *reason="unsupported-tnode-getter";
    if(!physx_genital_visibility_dispatch_matches(obj,0x10c,0x180,
        physx_genital_tnode_getter,sizeof(physx_genital_tnode_getter))) return 0;
    if(reason) *reason="missing-snode";
    if(!ptr_readable(obj+0x10,sizeof(void *))) return 0;
    snode=*(BYTE **)(obj+0x10);
    if(!snode) return 0;
    if(reason) *reason="unsupported-snode-getter";
    if(!physx_genital_visibility_dispatch_matches(snode,0x108,0x40,
        physx_genital_snode_getter,sizeof(physx_genital_snode_getter))) return 0;
    if(reason) *reason="unreadable-visibility";
    if(!ptr_readable(snode+0x10,sizeof(DWORD))) return 0;
    /* Follow only the validated reads, without invoking engine functions. */
    *value=*(DWORD *)(snode+0x10);
    if(reason) *reason="ok";
    return 1;
}

static void *physx_genital_visibility_object(int person)
{
    char name[192];
    void *raw=NULL, *object;
    if(person<0 || person>=BODY_PROFILE_PERSON_COUNT) return NULL;
    /* Live body nodes are scoped to their owning person's namespace. */
    _snprintf(name,sizeof(name),"Person%02dBody:body_subdiv_cageShape__body_genital01_SG",
        person+1);
    name[sizeof(name)-1]=0;
    object=resolve_find_obj(name,&raw);
    if(!object || is_nil_engine_object(raw,object))
        object=resolve_script_engine_obj(name,&raw);
    if(!object || is_nil_engine_object(raw,object)) return NULL;
    /* Property dispatch belongs to the ScriptObject returned by FindObjC,
       not to a potentially unwrapped native target. */
    return raw ? raw : object;
}

static int physx_genital_body_slot(int person, const char **source)
{
    int slot;
    *source="unknown";
    if(person<0 || person>=BODY_PROFILE_PERSON_COUNT) return -1;
    /* TK17 reports explicit loads such as Shared/Body/body03___02.
       Keep that type independently of optional custom PhysX profiles.
       Runtime Object.Name root strings were not resolvable through FindObjC. */
    slot=body_profile_loaded_body_slot[person];
    if(slot>=0 && slot<3) { *source="body-scene-load"; return slot; }
    if(body_profile_body_slot_from_name_a(body_profile_person_body_path[person],&slot)) {
        *source="body-profile";
        return slot;
    }
    return -1;
}

static void physx_update_genital_pause_state(DWORD now)
{
    static DWORD last_tick;
    static LONG last_generation;
    static int started;
    static struct { void *object; DWORD value; int known, sampled, body_slot;
        const char *reason, *body_source; } previous[4];
    int person, full_sample;
    LONG pause_mask=InterlockedCompareExchange(&physx_genital_pause_mask,0,0);
    LONG generation=InterlockedCompareExchange(&named_node_generation,0,0);
    if(!physx_pause_hidden_genitals || !engine_FindObjC) {
        physx_genital_visibility_publish_mask(0);
        started=0;
        return;
    }
    full_sample=!started || now-last_tick>=250 || generation!=last_generation;
    if(!full_sample && !pause_mask) return;
    if(full_sample) {
        last_generation=generation;
        started=1; last_tick=now;
    }
    for(person=0;person<4;++person) {
        LONG bit=1L<<person;
        void *object=NULL; DWORD value=0;
        int known=0,body_slot=-1;
        const char *reason="person-not-visible";
        const char *body_source="unknown";
        /* Newly covered people may wait for the slow poll; newly exposed
           people must not remain paused for up to 250 ms in plain view.
           Re-resolve paused people each physics tick before the solvers run.
           Never dereference the object retained for change-only logging. */
        if(!full_sample && !(pause_mask&bit)) continue;
        pause_mask&=~bit;
        if(poseedit_scene_person_visible(person)!=0) {
            object=physx_genital_visibility_object(person);
            reason="object-not-found";
            if(object) known=physx_genital_visibility_read(object,&value,&reason);
            if(known) body_slot=physx_genital_body_slot(person,&body_source);
        }
        if(physx_genital_pause_candidate(physx_pause_hidden_genitals,body_slot,known,value))
            pause_mask|=bit;
        if(defaults_cfg.debug && (!previous[person].sampled || previous[person].object!=object ||
            previous[person].known!=known || previous[person].body_slot!=body_slot ||
            strcmp(previous[person].body_source,body_source) ||
            strcmp(previous[person].reason,reason) || (known && previous[person].value!=value))) {
            log_line("genital visibility probe person=%s body_slot=%d body_source=%s object=%p known=%d value=%lu reason=%s",
                body_chain_person_name(person),body_slot,body_source,object,known,(unsigned long)value,reason);
            previous[person].object=object; previous[person].known=known;
            previous[person].value=value; previous[person].sampled=1;
            previous[person].body_slot=body_slot; previous[person].reason=reason;
            previous[person].body_source=body_source;
        }
    }
    physx_genital_visibility_publish_mask(pause_mask);
}

static unsigned int physx_genital_resume_candidates(int chain)
{
    unsigned int persons = 0;
    int i;
    for (i = 0; i < 4; ++i) {
        body_chain_person_state_t *state = body_chain_active_person_state(i, chain);
        if (state->clothing_resume_pending && !physx_genitals_paused(i))
            persons |= 1u << i;
    }
    return persons & ~physx_genital_early_attempted[chain];
}

/* Existing animation boundaries provide an earlier opportunity than EndScene.
   Only pending clothing resumes enter the solvers; normal frames do no extra
   simulation. Present resets the per-chain attempt mask so a blocked startup
   can retry next frame, with the original validation and update-rate gates. */
static void physx_prepare_genital_reveal(const char *phase)
{
    unsigned int persons[2], all;
    DWORD now;
    int i, chain, scope[4] = {0};
    if (physx_poseedit_transition_busy() ||
        !physx_pause_hidden_genitals || !engine_FindObjC || !physx_simulation_serial ||
        physx_genital_early_sample_done ||
        InterlockedCompareExchange(&body_chain_runtime_mode_transition_pending, 0, 0)) return;
    if (InterlockedCompareExchange(&physx_physics_phase_busy, 1, 0)) return;
    physx_genital_early_sample_done = 1;
    now = GetTickCount();
    physx_update_genital_pause_state(now);
    persons[0] = physx_genital_resume_candidates(0);
    persons[1] = physx_genital_resume_candidates(1);
    all = persons[0] | persons[1];
    if (all) {
        body_update_prepare_frame(now);
        /* Refresh only the body groups consumed by these resumed chains. */
        for (i = 0; i < 4; ++i) {
            body_profile_set_active_person_config(i);
            if ((persons[0] & (1u << i)) && body_chain_physics_cfg.enabled &&
                body_chain_physics_cfg.enabled_person[i])
                body_collider_add_scope_requirement(i, body_chain_physics_cfg.collision_scope, scope);
            if ((persons[1] & (1u << i)) && testicle_physics_cfg.enabled &&
                testicle_physics_cfg.enabled_person[i])
                body_collider_add_scope_requirement(i, testicle_physics_cfg.collision_scope, scope);
        }
        for (i = 0; i < 4; ++i) {
            body_profile_set_active_person_config(i);
            if (scope[i] || (all & (1u << i)))
                update_body_chain_colliders_for_person_scope(i, now, scope[i]);
            if (all & (1u << i)) body_chain_prime_axis_reference_for_person(i, now);
        }
        body_profile_set_active_person_config(-1);
        run_body_chain_physics_selected(now, persons[0]);
        run_testicle_physics_selected(now, persons[1]);
        for (chain = 0; chain < 2; ++chain) {
            physx_genital_early_attempted[chain] |= persons[chain];
            if (body_chain_runtime_mode_active()) for (i = 0; i < 4; ++i) {
                if (persons[chain] & (1u << i)) {
                    body_profile_set_active_person_config(i);
                    body_chain_apply_runtime_ownership_for_person(i, chain, 1);
                }
            }
        }
        body_profile_set_active_person_config(-1);
        publish_body_chain_runtime_ownership();
        if (defaults_cfg.debug)
            log_line("genital clothing resume phase=%s penis_mask=0x%x testicle_mask=0x%x",
                phase, persons[0], persons[1]);
    }
    InterlockedExchange(&physx_physics_phase_busy, 0);
}
