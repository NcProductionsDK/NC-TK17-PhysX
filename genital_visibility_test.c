#include "NC-TK17-PhysX.c"
#include <assert.h>
static DWORD read_field;
static void *test_object;
static int lookup_calls;
static unsigned int cadence_seen;
static int cadence_missing=-1;
static int test_person_values_enabled;
static DWORD test_person_values[4];
static void *__cdecl test_find_cadence(const char *name)
{
    int person=-1;
    const char *tail=NULL;
    assert(body_profile_parse_person_body_tsnode_a(name,&person,&tail));
    assert(!strcmp(tail,"body_subdiv_cageShape__body_genital01_SG"));
    cadence_seen|=1u<<person;
    if(test_person_values_enabled) {
        BYTE *snode=*(BYTE **)((BYTE *)test_object+0x10);
        *(DWORD *)(snode+0x10)=test_person_values[person];
    }
    return person==cadence_missing ? NULL : test_object;
}

static void test_all_person_body_detection(void)
{
    const char *source;
    engine_FindObjC=test_find_cadence;
    test_person_values_enabled=1;
    memset(body_profile_person_body_path,0,sizeof(body_profile_person_body_path));
    /* Every slot must support both body02 and body03 without any sidecar. */
    for(int type=1;type<=2;++type) {
        for(int person=0;person<4;++person) {
            char line[100];
            sprintf(line,"Execute3 'Shared/Body/body%02d___%02d'",type+1,person+1);
            body_profile_handle_goodbye_log_line_a(line);
            assert(physx_genital_body_slot(person,&source)==type);
            assert(!strcmp(source,"body-scene-load"));
            assert(!body_profile_person_sidecar_active[person]);
            assert(!body_profile_person_body_path[person][0]);
        }
        for(int hidden=0;hidden<4;++hidden) {
            for(int p=0;p<4;++p) test_person_values[p]=(p!=hidden);
            physx_pause_hidden_genitals=0;
            physx_update_genital_pause_state(2000);
            physx_pause_hidden_genitals=1;
            physx_update_genital_pause_state(2001);
            assert(InterlockedCompareExchange(&physx_genital_pause_mask,0,0)==(1L<<hidden));
            test_person_values[hidden]=1;
            physx_update_genital_pause_state(2002);
            assert(!physx_genitals_paused(hidden));
        }
    }
    /* User's current mix: body02/body03/body02 with Person04 absent. */
    body_profile_handle_goodbye_log_line_a("Execute3 'Shared/Body/body02___01'");
    body_profile_handle_goodbye_log_line_a("Execute3 'Shared/Body/body03___02'");
    body_profile_handle_goodbye_log_line_a("Execute3 'Shared/Body/body02___03'");
    cadence_missing=3;
    memset(test_person_values,0,sizeof(test_person_values));
    physx_pause_hidden_genitals=0;
    physx_update_genital_pause_state(3000);
    physx_pause_hidden_genitals=1;
    physx_update_genital_pause_state(3001);
    assert(InterlockedCompareExchange(&physx_genital_pause_mask,0,0)==7);
    test_person_values[1]=1;
    physx_update_genital_pause_state(3002);
    assert(InterlockedCompareExchange(&physx_genital_pause_mask,0,0)==5);
    /* A real replacement load overrides stale profile paths, but collision
       scenes, malformed events and UI selection intent cannot change type. */
    strcpy(body_profile_person_body_path[1],"body02.bs");
    body_profile_handle_goodbye_log_line_a("Execute3 'Shared/Body/body01___02'");
    assert(physx_genital_body_slot(1,&source)==0);
    body_profile_handle_goodbye_log_line_a("Execute3 'Shared/Body/body03_collision'");
    body_profile_handle_goodbye_log_line_a("Execute3 'Shared/Body/body03___05'");
    body_profile_handle_goodbye_log_line_a("Execute3 'Shared/Body/body03___00'");
    body_profile_note_body_select_marker_a("LUA/VAR_personsel___body|3|0|2");
    assert(physx_genital_body_slot(1,&source)==0);
    test_person_values[1]=0;
    physx_update_genital_pause_state(3300);
    assert(!physx_genitals_paused(1));
    body_profile_loaded_body_slot[1]=-1;
    assert(physx_genital_body_slot(1,&source)==1);
    assert(!strcmp(source,"body-profile"));
    physx_pause_hidden_genitals=0;
    physx_update_genital_pause_state(3301);
    for(int p=0;p<4;++p) body_profile_loaded_body_slot[p]=-1;
    memset(body_profile_person_body_path,0,sizeof(body_profile_person_body_path));
    cadence_missing=-1;
    test_person_values_enabled=0;
    engine_FindObjC=NULL;
}

static void test_resume_cadence(BYTE *snode)
{
    physx_pause_hidden_genitals=0;
    physx_update_genital_pause_state(0); /* Reset polling schedule. */
    engine_FindObjC=test_find_cadence;
    physx_pause_hidden_genitals=1;
    strcpy(body_profile_person_body_path[0],"body02.bs");
    strcpy(body_profile_person_body_path[2],"body03.bs");
    *(DWORD *)(snode+0x10)=0;
    cadence_seen=0;
    physx_update_genital_pause_state(1000);
    assert(cadence_seen==15 && physx_genitals_paused(0) && physx_genitals_paused(2));
    assert(!physx_genitals_paused(1) && !physx_genitals_paused(3));

    /* A removed/replaced object must fail open at the next tick, without
       revisiting active people or dereferencing the old resolved object. */
    cadence_missing=0;
    cadence_seen=0;
    physx_update_genital_pause_state(1008);
    assert(cadence_seen==5 && !physx_genitals_paused(0) && physx_genitals_paused(2));

    /* Uncover before the 250 ms interval expires: resume on this tick. */
    *(DWORD *)(snode+0x10)=1;
    cadence_seen=0;
    physx_update_genital_pause_state(1016);
    assert(cadence_seen==4 && !physx_genitals_paused(2));
    cadence_missing=-1;
    cadence_seen=0;
    physx_update_genital_pause_state(1024);
    assert(cadence_seen==0); /* Nobody paused: no per-frame lookup. */

    /* Newly covered people retain the slow schedule. */
    *(DWORD *)(snode+0x10)=0;
    physx_update_genital_pause_state(1249);
    assert(cadence_seen==0 && !physx_genitals_paused(0));
    physx_update_genital_pause_state(1250);
    assert(cadence_seen==15 && physx_genitals_paused(0) && physx_genitals_paused(2));
    physx_pause_hidden_genitals=0;
    cadence_seen=0;
    physx_update_genital_pause_state(1251);
    assert(cadence_seen==0 && !physx_genitals_paused(0));
    memset(body_profile_person_body_path,0,sizeof(body_profile_person_body_path));
    engine_FindObjC=NULL;
}
static void *__cdecl test_find_nothing(const char *name)
{
    (void)name;
    return NULL;
}

static BYTE resume_skeleton[4][1024];
static void *__cdecl test_find_resume_skeleton(const char *name)
{
    if (strstr(name, ":root")) return resume_skeleton[0]+32;
    if (strstr(name, ":Spenis_joint01")) return resume_skeleton[1]+32;
    if (strstr(name, ":Spenis_joint02")) return resume_skeleton[2]+32;
    if (strstr(name, ":Spenis_joint03")) return resume_skeleton[3]+32;
    return NULL;
}

static void test_resume_initial_output(void)
{
    body_chain_person_state_t *state;
    body_profile_set_active_person_config(-1);
    memset(&body_chain_physics_cfg,0,sizeof(body_chain_physics_cfg));
    memset(&body_chain_collider_cfg,0,sizeof(body_chain_collider_cfg));
    memset(&physics_environment_cfg,0,sizeof(physics_environment_cfg));
    body_chain_physics_cfg.root_offset=0x40;
    body_chain_physics_cfg.output_offset=0x40;
    body_chain_physics_cfg.zero_output_rest=1;
    body_chain_physics_cfg.enabled=1;
    body_chain_physics_cfg.enabled_person[0]=1;
    body_chain_physics_cfg.gravity_angle=12;
    body_chain_physics_cfg.stiffness=90;
    body_chain_physics_cfg.damping=18;
    body_chain_physics_cfg.gravity_horizontal_strength=1;
    body_chain_physics_cfg.gravity_vertical_strength=1;
    body_chain_physics_cfg.horizontal_output_axis=0;
    body_chain_physics_cfg.vertical_output_axis=1;
    physics_environment_cfg.gravity_horizontal_tail_axis=0;
    physics_environment_cfg.gravity_vertical_tail_axis=1;
    for (int j=0;j<3;++j) {
        body_chain_physics_cfg.link_gain[j]=1;
        for (int a=0;a<3;++a) {
            body_chain_physics_cfg.link_min_angle[j][a]=-30;
            body_chain_physics_cfg.link_max_angle[j][a]=30;
        }
    }
    InterlockedExchange(&body_chain_poseeditor_mode_active,1);
    state=body_chain_active_person_state(0,0);
    engine_FindObjC=test_find_resume_skeleton;
    /* Exercise the actual initialization and publication branch with a
       deterministic 12-degree force target and no native ownership writes. */
    for (int resume=0;resume<4;++resume) {
        DWORD start=5000+resume*100;
        float initial=resume==3 ? -70.0f : -8.0f;
        memset(state,0,sizeof(*state));
        memset(resume_skeleton,0,sizeof(resume_skeleton));
        *(float *)(resume_skeleton[0]+32+0x40)=1;
        for (int j=0;j<3;++j)
            *(float *)(resume_skeleton[j+1]+32+0x40+sizeof(float))=initial;
        state->clothing_resume_pending=resume==1;
        poseedit_penis_resume_mask=resume>=2 ? 1u : 0u;
        poseedit_penis_resume_editor=captured_poseedit_this;
        run_body_chain_physics_for_person(0,start);
        assert(state->initialized);
        for (int j=0;j<3;++j) {
            float output=*(float *)(resume_skeleton[j+1]+32+0x40+sizeof(float));
            assert(fabsf(output-initial)<0.0001f);
            assert(state->output_handoff_rest[j][1]==initial);
            assert(state->velocity[j][1]==0);
        }
        assert(state->clothing_resume_pending==(resume==1));
        assert(state->pose_load_resume_pending==(resume>=2));
        assert(!poseedit_penis_resume_mask);
        {
            /* A pending gravity baseline must preserve the visible pose. */
            physics_environment_cfg.gravity_apply_to_body_chain=1;
            physics_environment_cfg.world_gravity_probe=1;
            physics_environment_cfg.gravity_probe_settle_ms=10000;
            run_body_chain_physics_for_person(0,start+16);
            assert(state->activation_resume_pending && state->clothing_resume_pose_valid);
            for (int j=0;j<3;++j)
                assert(fabsf(*(float *)(resume_skeleton[j+1]+32+0x44)-initial)<0.0001f);
            physics_environment_cfg.gravity_apply_to_body_chain=0;
            physics_environment_cfg.world_gravity_probe=0;
            run_body_chain_physics_for_person(0,start+32);
            float output=*(float *)(resume_skeleton[1]+32+0x44);
            assert(output>initial && output<0); /* Moves naturally, not straight to +12. */
            assert(!state->clothing_resume_pending);
            assert(!state->pose_load_resume_pending);
            assert(!state->activation_resume_pending);
            assert(state->output_handoff_rest[0][1]==initial);
        }
    }
    engine_FindObjC=NULL;
    puts("PASS: activation/clothing/pose-load resume preserves incoming output and OFF snapshot, including outside-limit poses, holds through gravity initialization, then springs toward the force pose");
}

static void test_clothing_resume(void)
{
    body_chain_person_state_t state = {0};
    body_chain_physics_config_t cfg = {0};
    BYTE skeleton[4][512] = {{0}};
    void *joints[3] = {skeleton[1], skeleton[2], skeleton[3]};
    float target[3][3] = {{70,-70,70},{70,-70,70},{70,-70,70}};
    LONG generation = InterlockedCompareExchange(&named_node_generation,0,0);
    cfg.horizontal_output_axis = 0;
    cfg.vertical_output_axis = 1;
    cfg.chain_total_bend_max = 30;
    cfg.chain_total_twist_max = 12;
    for (int j=0;j<3;++j) for (int a=0;a<3;++a) {
        cfg.link_min_angle[j][a] = -20;
        cfg.link_max_angle[j][a] = 25;
    }
    for (int count=2;count<=3;++count) {
        state.initialized = 1;
        state.root_raw = skeleton[0];
        memcpy(state.joint_raw,joints,sizeof(joints));
        state.angle[0][0]=16;
        state.velocity[0][0]=42;
        state.collision_step_valid=1;
        body_chain_pause_for_clothing(&state);
        assert(!state.initialized && !state.root_raw && !state.joint_raw[0]);
        assert(!state.angle[0][0] && !state.velocity[0][0] && !state.collision_step_valid);
        assert(state.clothing_resume_pending && state.clothing_resume_skeleton_valid);
        assert(testicle_physics_candidate_is_stable(&state,skeleton[0],joints,1000));
        assert(body_chain_seed_clothing_resume(&state,&cfg,target,count));
        assert(state.clothing_resume_pending && state.clothing_resume_pose_valid && !state.clothing_resume_skeleton_valid);
        float values[3][3], *out[3]={values[0],values[1],values[2]};
        body_chain_write_clothing_resume_pose(&state,out,count);
        for (int j=0;j<3;++j) for (int a=0;a<3;++a)
            assert(fabsf(values[j][a]-target[j][a])<0.0001f);
        float h=0,v=0,t=0;
        for (int j=0;j<3;++j) for (int a=0;a<3;++a) {
            assert(state.velocity[j][a]==0);
            assert(state.angle[j][a]>=-20 && state.angle[j][a]<=25);
            if (j>=count) assert(state.angle[j][a]==0);
        }
        for (int j=0;j<count;++j) { h+=state.angle[j][0]; v+=state.angle[j][1]; t+=state.angle[j][2]; }
        assert(sqrtf(h*h+v*v)<=30.0001f && fabsf(t)<=12.0001f);
        state.angle[0][0]=5;
        state.velocity[0][0]=7;
        assert(!body_chain_seed_clothing_resume(&state,&cfg,target,count));
        assert(state.angle[0][0]==5 && state.velocity[0][0]==7);
    }
    /* A generation change must not trust reused addresses. */
    state.initialized=1; state.root_raw=skeleton[0];
    memcpy(state.joint_raw,joints,sizeof(joints));
    body_chain_pause_for_clothing(&state);
    InterlockedIncrement(&named_node_generation);
    assert(!testicle_physics_candidate_is_stable(&state,skeleton[0],joints,2000));
    assert(!testicle_physics_candidate_is_stable(&state,skeleton[0],joints,2049));
    assert(testicle_physics_candidate_is_stable(&state,skeleton[0],joints,2050));
    /* Even with the same generation, a different joint invalidates the ticket. */
    state.initialized=1; state.root_raw=skeleton[0];
    memcpy(state.joint_raw,joints,sizeof(joints));
    body_chain_pause_for_clothing(&state);
    joints[1]=skeleton[2]+16;
    assert(!testicle_physics_candidate_is_stable(&state,skeleton[0],joints,3000));
    assert(!testicle_physics_candidate_is_stable(&state,skeleton[0],joints,3020));
    assert(testicle_physics_candidate_is_stable(&state,skeleton[0],joints,3050));
    state.clothing_resume_pending=1;
    state.angle[0][0]=5;
    target[1][2]=NAN;
    assert(!body_chain_seed_clothing_resume(&state,&cfg,target,3));
    assert(state.clothing_resume_pending && state.angle[0][0]==5);
    reset_body_chain_person_state(&state);
    assert(!state.clothing_resume_pending && !state.clothing_resume_skeleton_valid && !state.clothing_resume_pose_valid);
    /* Compensation and nonzero rest are included in the output-to-angle mapping. */
    memset(&state,0,sizeof(state));
    state.clothing_resume_pending=1;
    state.pose_compensation_valid=1;
    for (int j=0;j<3;++j) for (int a=0;a<3;++a) {
        target[j][a]=j*10+a;
        state.rest[j][a]=3;
        state.pose_compensation[j][a]=4;
    }
    assert(body_chain_seed_clothing_resume(&state,&cfg,target,3));
    float values[3][3], *out[3]={values[0],values[1],values[2]};
    body_chain_write_clothing_resume_pose(&state,out,3);
    for (int j=0;j<3;++j) for (int a=0;a<3;++a)
        assert(fabsf(values[j][a]-target[j][a])<0.0001f);
    state.collision_step_valid=1;
    memcpy(state.collision_step_angle,state.angle,sizeof(state.angle));
    body_chain_person_state_t split=state;
    body_chain_advance_clothing_resume_rest(&state,0.032f);
    for (int j=0;j<3;++j) for (int a=0;a<3;++a) {
        float predicted=state.angle[j][a]-state.collision_step_angle[j][a];
        assert(fabsf(predicted-(state.rest[j][a]-split.rest[j][a]))<0.0001f);
    }
    body_chain_advance_clothing_resume_rest(&split,0.016f);
    body_chain_advance_clothing_resume_rest(&split,0.016f);
    for (int j=0;j<3;++j) for (int a=0;a<3;++a)
        assert(fabsf(state.rest[j][a]-split.rest[j][a])<0.0001f);
    for (int step=0;step<100;++step) body_chain_advance_clothing_resume_rest(&state,0.016f);
    assert(!state.clothing_resume_pose_valid);
    for (int j=0;j<3;++j) for (int a=0;a<3;++a) assert(state.rest[j][a]==3);

    InterlockedExchange(&named_node_generation,generation);

    /* Actor, chain and mode isolation; frame-end must not touch an already
       attempted chain, even when a second attempt would otherwise reset it. */
    physx_pause_hidden_genitals=1;
    InterlockedExchange(&physx_genital_pause_mask,4);
    for (int mode=0;mode<2;++mode) {
        InterlockedExchange(&body_chain_poseeditor_mode_active,mode);
        for (int p=0;p<4;++p) for (int chain=0;chain<2;++chain) {
            body_chain_person_state_t *s=body_chain_active_person_state(p,chain);
            memset(s,0,sizeof(*s));
            s->clothing_resume_pending=(chain==0 || p==1);
        }
        assert(physx_genital_resume_candidates(0)==11);
        assert(physx_genital_resume_candidates(1)==2);
        physx_genital_early_attempted[0]=1;
        assert(physx_genital_resume_candidates(0)==10);
        assert(physx_genital_resume_candidates(1)==2);
        body_chain_physics_global_cfg.enabled=0;
        engine_FindObjC=test_find_nothing;
        run_body_chain_physics(4000);
        assert(body_chain_active_person_state(0,0)->clothing_resume_pending);
        assert(!body_chain_active_person_state(1,0)->clothing_resume_pending);
        physx_genital_early_attempted[0]=0;
        run_body_chain_physics(4016);
        assert(!body_chain_active_person_state(0,0)->clothing_resume_pending);
    }
    physx_pause_hidden_genitals=0;
    engine_FindObjC=NULL;
    puts("PASS: clothing resume clears stale motion/contact state; checked skeleton tickets, generation/address replacement fallback, continuous initial output including out-of-limit poses, fixed testicle end, compensation, rest-offset convergence and one-time seeding");
    puts("PASS: early resume selection isolates all four actors, both modes and chains; frame-end dispatch excludes earlier attempts; ordinary disable cancels resume");
}

static void test_pause_gates(void)
{
    assert(!physx_genital_pause_candidate(0,1,1,0));
    assert(!physx_genital_pause_candidate(1,-1,1,0));
    assert(!physx_genital_pause_candidate(1,0,1,0));
    assert(!physx_genital_pause_candidate(1,1,0,0));
    assert(!physx_genital_pause_candidate(1,1,1,1));
    assert(!physx_genital_pause_candidate(1,1,1,2));
    assert(physx_genital_pause_candidate(1,1,1,0));
    assert(physx_genital_pause_candidate(1,2,1,0));
    physx_pause_hidden_genitals=1;
    InterlockedExchange(&physx_genital_pause_mask,5);
    assert(physx_genitals_paused(0) && physx_genitals_paused(2));
    assert(!physx_genitals_paused(1) && !physx_genitals_paused(3));
    assert(!physx_genitals_paused(-1) && !physx_genitals_paused(4));
    physx_pause_hidden_genitals=0;
    assert(!physx_genitals_paused(0));
    physx_pause_hidden_genitals=1;
    InterlockedExchange(&physx_genital_pause_mask,0);
    assert(!physx_genitals_paused(0));

    /* Exercise the real solver dispatch in both modes: hidden state resets
       motion without changing the user's enabled settings or touching outputs. */
    body_chain_physics_global_cfg.enabled=1;
    testicle_physics_global_cfg.enabled=1;
    memset(body_chain_physics_person_cfg,0,sizeof(body_chain_physics_person_cfg));
    memset(testicle_physics_person_cfg,0,sizeof(testicle_physics_person_cfg));
    body_chain_physics_person_cfg[0].enabled=1;
    body_chain_physics_person_cfg[0].enabled_person[0]=1;
    testicle_physics_person_cfg[0].enabled=1;
    testicle_physics_person_cfg[0].enabled_person[0]=1;
    engine_FindObjC=test_find_nothing;
    InterlockedExchange(&physx_genital_pause_mask,1);
    for(int editor=0;editor<2;++editor) {
        body_chain_person_state_t *penis,*testicle;
        InterlockedExchange(&body_chain_poseeditor_mode_active,editor);
        penis=body_chain_active_person_state(0,0);
        testicle=body_chain_active_person_state(0,1);
        memset(penis,0,sizeof(*penis));
        memset(testicle,0,sizeof(*testicle));
        penis->initialized=testicle->initialized=1;
        penis->last_tick=testicle->last_tick=123;
        penis->velocity[0][0]=testicle->velocity[0][0]=42;
        run_body_chain_physics(500);
        run_testicle_physics(500);
        assert(!penis->initialized && !testicle->initialized);
        assert(!penis->last_tick && !testicle->last_tick);
        assert(!penis->velocity[0][0] && !testicle->velocity[0][0]);
        assert(body_chain_physics_person_cfg[0].enabled_person[0]);
        assert(testicle_physics_person_cfg[0].enabled_person[0]);
        assert(!body_chain_runtime_penis_person_enabled_now(0,500));
        assert(!body_chain_apply_runtime_ownership_for_person(0,0,0));
        assert(!body_chain_apply_runtime_ownership_for_person(0,1,0));
        run_testicle_physics_late_ownership(500);
        assert(!tk17_testicle_inertia_gate_states[0].active);
    }
    engine_FindObjC=NULL;
    physx_update_genital_pause_state(1000);
    assert(!physx_genitals_paused(0));
    physx_pause_hidden_genitals=0;
}
/* Read the installed DLL as a PE file; do not load or execute engine code. */
static void assert_engine_bytes(DWORD rva, const BYTE *expected, size_t size)
{
    FILE *file=fopen("../../The Klub 17/Binaries/ThriXXX010278-SYS.dll","rb");
    IMAGE_DOS_HEADER dos;
    IMAGE_NT_HEADERS32 nt;
    IMAGE_SECTION_HEADER section;
    BYTE actual[64];
    int found=0;
    assert(file && size<=sizeof(actual));
    assert(fread(&dos,sizeof(dos),1,file)==1 && dos.e_magic==IMAGE_DOS_SIGNATURE);
    assert(fseek(file,dos.e_lfanew,SEEK_SET)==0);
    assert(fread(&nt,sizeof(nt),1,file)==1 && nt.Signature==IMAGE_NT_SIGNATURE);
    assert(nt.FileHeader.Machine==IMAGE_FILE_MACHINE_I386);
    assert(fseek(file,dos.e_lfanew+sizeof(DWORD)+sizeof(IMAGE_FILE_HEADER)+
        nt.FileHeader.SizeOfOptionalHeader,SEEK_SET)==0);
    for(unsigned i=0;i<nt.FileHeader.NumberOfSections;++i) {
        assert(fread(&section,sizeof(section),1,file)==1);
        if(rva>=section.VirtualAddress &&
            rva-section.VirtualAddress+size<=section.SizeOfRawData) {
            assert(fseek(file,section.PointerToRawData+rva-section.VirtualAddress,SEEK_SET)==0);
            assert(fread(actual,size,1,file)==1 && !memcmp(actual,expected,size));
            found=1; break;
        }
    }
    fclose(file);
    assert(found);
}

static void test_engine_signatures(void)
{
    /* Registration pushes getter, name "Visibility", then property ID.
       This catches selecting a different class's equally named property. */
    const BYTE tnode_registration[]={0x68,0xf0,0xd8,0x0e,0x10,
        0x68,0xac,0x04,0x17,0x10,0x68,0x43,0xf0,0xff,0x06};
    const BYTE snode_registration[]={0x68,0x40,0x0c,0x0e,0x10,
        0x68,0xac,0x04,0x17,0x10,0x68,0x42,0xf0,0xff,0x01};
    assert_engine_bytes(0xed8f0,physx_genital_tnode_getter,sizeof(physx_genital_tnode_getter));
    assert_engine_bytes(0xe0c40,physx_genital_snode_getter,sizeof(physx_genital_snode_getter));
    assert_engine_bytes(0x120af6,tnode_registration,sizeof(tnode_registration));
    assert_engine_bytes(0x120933,snode_registration,sizeof(snode_registration));
}
static void *__cdecl test_find_genital(const char *name)
{
    int person=-1;
    const char *tail=NULL;
    assert(body_profile_parse_person_body_tsnode_a(name,&person,&tail));
    assert(person==lookup_calls);
    assert(strcmp(tail,"body_subdiv_cageShape__body_genital01_SG")==0);
    ++lookup_calls;
    return test_object;
}
int main(int argc, char **argv)
{
    BYTE object_storage[128]={0},metadata[1024]={0},property_storage[512]={0};
    BYTE snode_storage[128]={0},snode_metadata[1024]={0},snode_property_storage[256]={0};
    BYTE *object=object_storage+32,*property=property_storage+32;
    BYTE *snode=snode_storage+32,*snode_property=snode_property_storage+32;
    const char *reason=NULL;
    BYTE *code=VirtualAlloc(NULL,4096,MEM_COMMIT|MEM_RESERVE,PAGE_EXECUTE_READWRITE);
    defaults_cfg.debug=0;
    defaults_cfg.performance_profile=0;
    assert(normal_log_line_allowed("genital physics state person=%s"));
    assert(!normal_log_line_allowed("genital visibility probe person=%s"));
    assert(!normal_log_line_allowed("unrelated verbose diagnostic"));
    defaults_cfg.performance_profile=1;
    assert(!normal_log_line_allowed("genital visibility probe person=%s"));
    defaults_cfg.performance_profile=0;
    defaults_cfg.debug=1;
    assert(normal_log_line_allowed("genital visibility probe person=%s"));
    defaults_cfg.debug=0;
    test_engine_signatures();
    assert(code);
    memcpy(code,physx_genital_tnode_getter,sizeof(physx_genital_tnode_getter));
    memcpy(code+64,physx_genital_snode_getter,sizeof(physx_genital_snode_getter));
    *(BYTE **)(object-0x18)=metadata;
    *(BYTE **)(metadata+0x10c)=property;
    *(DWORD *)(property-0x1c)=1;
    *(BYTE **)(property+0x180)=code;
    *(BYTE **)(object+0x10)=snode;
    *(BYTE **)(snode-0x18)=snode_metadata;
    *(BYTE **)(snode_metadata+0x108)=snode_property;
    *(DWORD *)(snode_property-0x1c)=1;
    *(BYTE **)(snode_property+0x40)=code+64;
    test_object=object;
    engine_FindObjC=test_find_genital;
    assert(!physx_genital_visibility_object(-1));
    assert(!physx_genital_visibility_object(4));
    for(int person=0;person<4;++person)
        assert(physx_genital_visibility_object(person)==object);
    assert(lookup_calls==4);
    engine_FindObjC=NULL;
    for(DWORD value=0;value<3;++value) {
        *(DWORD *)(snode+0x10)=value;
        assert(physx_genital_visibility_read(object,&read_field,&reason) && read_field==value);
        assert(!strcmp(reason,"ok"));
    }
    code[2]=0x24;
    assert(!physx_genital_visibility_read(object,&read_field,&reason));
    assert(!strcmp(reason,"unsupported-tnode-getter"));
    code[2]=0x10;
    code[66]=0x20; /* The old ImageLayer field layout must be rejected. */
    assert(!physx_genital_visibility_read(object,&read_field,&reason));
    assert(!strcmp(reason,"unsupported-snode-getter"));
    code[66]=0x10;
    *(DWORD *)(snode_property-0x1c)=0;
    assert(!physx_genital_visibility_read(object,&read_field,&reason));
    *(DWORD *)(snode_property-0x1c)=1;
    *(BYTE **)(object+0x10)=NULL;
    assert(!physx_genital_visibility_read(object,&read_field,&reason));
    assert(!strcmp(reason,"missing-snode"));
    *(BYTE **)(object+0x10)=snode;
    *(DWORD *)(property-0x1c)=0;
    assert(!physx_genital_visibility_read(object,&read_field,&reason));
    assert(!physx_genital_visibility_read(NULL,&read_field,NULL));
    assert(!physx_genital_visibility_read(object,NULL,NULL));
    *(DWORD *)(property-0x1c)=1;
    test_resume_cadence(snode);
    test_all_person_body_detection();
    VirtualFree(code,0,MEM_RELEASE);
    test_pause_gates();
    test_clothing_resume();
    test_resume_initial_output();
    puts("PASS: installed SYS getter registrations and code verified; per-person names, logging, linked SNode visibility, raw values, null links and unsupported getters/properties checked");
    puts("PASS: opt-in/body/visibility gates, person isolation, both solver reset paths, ownership gates, settings preservation and missing-engine fallback");
    puts("PASS: paused people resume or fail open on the next tick; active people retain 250 ms polling; disabling performs no lookups");
    puts("PASS: actual Execute3 body-load parser feeds body02/body03 pause/resume in all four slots without sidecars; mixed bodies, inactive slot, independent clothing, type replacement and unrelated-event rejection");
    if(argc==2) {
        char line[4096];
        FILE *log=fopen(argv[1],"rb");
        assert(log);
        for(int p=0;p<4;++p) body_profile_loaded_body_slot[p]=-1;
        while(fgets(line,sizeof(line),log)) body_profile_handle_goodbye_log_line_a(line);
        fclose(log);
        for(int p=0;p<4;++p)
            printf("Log replay Person%02d body_slot=%d\n",p+1,body_profile_loaded_body_slot[p]);
    }
    return 0;
}
