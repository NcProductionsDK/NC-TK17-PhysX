#include "NC-TK17-PhysX.c"
#include <assert.h>

static BYTE editor[0x400], tracks[POSEEDIT_TRACKS_OFFSET + 4*256*POSEEDIT_TRACK_SIZE];
static BYTE roots[4][0x200], spines[4][0x200];
static BYTE nodes[2][4][2][3][0x200];
static BYTE testicle_nodes[3][0x200];
static float authored[2][4][2][2][3];
static BYTE fade_ready, fade_object[0x30], person_module[0x40];
static BYTE THISCALL fake_fade_ready(void *self, void *arg0, void *arg1)
{
    assert(self == fade_object && arg0 == (void*)1 && arg1 == (void*)2);
    return fade_ready;
}
static const char *names[2][2][3] = {
    {{"Sbreast_scale_L_joint", "breast_scale_L_joint", "breast_L_joint"},
     {"Sbreast_scale_R_joint", "breast_scale_R_joint", "breast_R_joint"}},
    {{"Sbutt_L_joint01", "butt_L_joint01", "unused_L"},
     {"Sbutt_R_joint01", "butt_R_joint01", "unused_R"}}
};

static void *__cdecl find_fixture(const char *name)
{
    char expected[256];
    if (strstr(name, ":Stesticles_joint01")) return testicle_nodes[0];
    if (strstr(name, ":Stesticles_joint02")) return testicle_nodes[1];
    if (strstr(name, ":Stesticles_jointEnd")) return testicle_nodes[2];
    for (int p=0; p<4; p++) {
        make_body_runtime_name(expected, sizeof(expected), body_chain_person_name(p), "root");
        if (!strcmp(name,expected)) return roots[p];
        make_body_runtime_name(expected, sizeof(expected), body_chain_person_name(p), "spine_joint04");
        if (!strcmp(name,expected)) return spines[p];
        for (int g=0;g<2;g++) for(int s=0;s<2;s++) for(int n=0;n<3;n++) {
            make_body_runtime_name(expected,sizeof(expected),body_chain_person_name(p),names[g][s][n]);
            if (!strcmp(name,expected)) return nodes[g][p][s][n];
        }
    }
    return NULL;
}

static breasts_physics_person_state_t *state_for(int g, int p)
{
    return g ? &butt_physics_states[p] : &breasts_physics_states[p];
}

static float *output_for(int g, int p, int s, int translation)
{
    body_profile_set_active_person_config(p);
    int offset=translation ? (g ? butt_physics_bone_translation_offset : breasts_physics_bone_translation_offset)
                           : (g ? butt_physics_cfg.output_offset : breasts_physics_cfg.output_offset);
    return (float*)(nodes[g][p][s][0]+offset);
}

static void native_pose_values(int pose)
{
    for(int g=0;g<2;g++) for(int p=0;p<4;p++) for(int s=0;s<2;s++) for(int t=0;t<2;t++) {
        float *out=output_for(g,p,s,t);
        for(int a=0;a<3;a++)
            out[a]=authored[g][p][s][t][a]=t ? .1f*(p+1)+.01f*(s+a+pose) : 3.0f*(g+p+s+a+pose);
    }
    body_profile_set_active_person_config(-1);
}

static void simulate_offsets(void)
{
    for(int g=0;g<2;g++) for(int p=0;p<4;p++) {
        body_profile_set_active_person_config(p);
        breasts_physics_person_state_t *state=state_for(g,p);
        memset(state,0,sizeof(*state));
        state->initialized=1;
        state->motion.root_raw=g ? roots[p] : spines[p];
        state->contact_translation_active=1;
        for(int s=0;s<2;s++) {
            state->source_joint_raw[s]=nodes[g][p][s][0];
            state->animation_joint_raw[s]=nodes[g][p][s][1];
            state->translation_parent_joint_raw[s]=g ? roots[p] : nodes[g][p][s][2];
            memcpy(state->source_handoff[s],output_for(g,p,s,0),sizeof(float)*3);
            memcpy(state->source_translation_handoff[s],output_for(g,p,s,1),sizeof(float)*3);
            memcpy(state->rest_rotation[s],state->source_handoff[s],sizeof(float)*3);
            for(int a=0;a<3;a++) {
                state->rotation[s][a]=20.0f+s+a;
                state->bone_translation[s][a]=.025f*(s ? -1 : 1);
            }
        }
        assert(g ? butt_physics_live_ownership_matches(p,state) : breasts_physics_live_ownership_matches(p,state));
        assert(g ? butt_physics_apply_output(state,0,0) : breasts_physics_apply_output(state,0,0));
        assert(state->output_applied && state->bone_translation_applied);
    }
    body_profile_set_active_person_config(-1);
}

static void expect_authored(void)
{
    for(int g=0;g<2;g++) for(int p=0;p<4;p++) for(int s=0;s<2;s++) for(int t=0;t<2;t++) {
        if(memcmp(output_for(g,p,s,t),authored[g][p][s][t],sizeof(float)*3)) {
            fprintf(stderr,"FAIL: %s person %d side %d retained physics %s across pose replacement\n",
                    g?"butt":"breasts",p+1,s,t?"displacement":"rotation");
            exit(1);
        }
    }
    body_profile_set_active_person_config(-1);
}

static void activation_regression(void)
{
    static BYTE module[0x400], person[0x400], inertia[0x1000], app;
    static struct { int count; void *slots[5]; } people;
    people.count=5; people.slots[1]=person;
    *(void**)(editor+POSEEDIT_PERSON_MODULE_OFFSET)=module;
    *(void**)(module+sizeof(void*))=&app;
    *(void***)(module+PERSON_MODULE_PERSON_STATE_PTR_OFFSET)=people.slots;
    *(void**)(person+PERSON_STATE_INERTIA_OFFSET)=inertia;
    *(void**)inertia=&app;
    body_profile_set_active_person_config(0);
    memset(&physics_environment_cfg,0,sizeof(physics_environment_cfg));
    memset(&body_chain_collider_cfg,0,sizeof(body_chain_collider_cfg));
    for(int g=0;g<2;g++) for(int zero=0;zero<2;zero++) for(int pass=0;pass<3;pass++) {
        breasts_physics_person_state_t *st=state_for(g,0);
        body_chain_physics_config_t *cfg=g?&butt_physics_cfg:&breasts_physics_cfg;
        memset(st,0,sizeof(*st)); memset(cfg,0,sizeof(*cfg));
        cfg->output_offset=0x100; cfg->root_offset=0xe8;
        cfg->zero_output_rest=zero; cfg->interval_ms=16;
        cfg->stiffness=90; cfg->damping=18;
        for(int a=0;a<3;a++) {
            cfg->link_min_angle[0][a]=-20; cfg->link_max_angle[0][a]=20;
        }
        float start[2][3], position[2][3];
        for(int s=0;s<2;s++) for(int a=0;a<3;a++) {
            /* New live pose for every re-enable, including outside limits. */
            start[s][a]=(s?-1:1)*(8+40*pass+a);
            position[s][a]=.1f*(s+a+1);
            output_for(g,0,s,0)[a]=start[s][a];
            output_for(g,0,s,1)[a]=position[s][a];
        }
        DWORD tick=20000+10000*(g*6+zero*3+pass);
        for(int frame=0;frame<10 && !st->initialized;frame++) {
            if(g) run_butt_physics_for_person(0,tick+frame*32);
            else run_breasts_physics_for_person(0,tick+frame*32);
        }
        assert(st->initialized && st->output_applied);
        for(int s=0;s<2;s++) for(int a=0;a<3;a++) {
            assert(fabsf(output_for(g,0,s,0)[a]-start[s][a])<.0001f);
            assert(fabsf(output_for(g,0,s,1)[a]-position[s][a])<.0001f);
            assert(st->angular_velocity[s][a]==0);
        }
        DWORD first=st->last_tick;
        for(int frame=1;frame<=250;frame++) {
            if(g) run_butt_physics_for_person(0,first+frame*16);
            else run_breasts_physics_for_person(0,first+frame*16);
            for(int s=0;s<2;s++) for(int a=0;a<3;a++) {
                float out=output_for(g,0,s,0)[a];
                assert(isfinite(out));
                assert(st->rotation[s][a]>=-20.001f && st->rotation[s][a]<=20.001f);
                if(frame==1) assert(fabsf(out-start[s][a])<fabsf(start[s][a])*.3f);
                if(frame==250) assert(fabsf(out-(zero?0:start[s][a]))<.001f);
                assert(st->source_handoff[s][a]==start[s][a]);
            }
        }
        if(g) reset_butt_physics_person_state(0,st,1);
        else reset_breasts_physics_person_state(0,st,1);
        for(int s=0;s<2;s++) for(int a=0;a<3;a++) {
            assert(fabsf(output_for(g,0,s,0)[a]-start[s][a])<.0001f);
            assert(fabsf(output_for(g,0,s,1)[a]-position[s][a])<.0001f);
        }
    }
    body_profile_set_active_person_config(-1);
    puts("PASS: actual breast/butt activation starts at live pose, moves continuously, converges within limits and restores exact OFF rotation/position across repeated enables and both rest modes");
    body_profile_set_active_person_config(0);
    body_chain_physics_config_t *cfg=&testicle_physics_cfg;
    memset(cfg,0,sizeof(*cfg));
    cfg->output_offset=0x100; cfg->root_offset=0xe8; cfg->interval_ms=16;
    cfg->zero_output_rest=1; cfg->stiffness=90; cfg->damping=18;
    for(int j=0;j<3;j++) for(int a=0;a<3;a++) {
        cfg->link_min_angle[j][a]=-20; cfg->link_max_angle[j][a]=20;
    }
    for(int pass=0;pass<3;pass++) {
        body_chain_person_state_t *st=body_chain_active_person_state(0,1);
        memset(st,0,sizeof(*st));
        float start=8+40*pass;
        for(int j=0;j<3;j++) for(int a=0;a<3;a++)
            ((float*)(testicle_nodes[j]+cfg->output_offset))[a]=start;
        DWORD tick=200000+10000*pass;
        for(int frame=0;frame<10 && !st->initialized;frame++)
            run_testicle_physics_for_person(0,tick+frame*32);
        assert(st->initialized && st->activation_resume_pending);
        for(int j=0;j<3;j++) for(int a=0;a<3;a++) {
            assert(fabsf(((float*)(testicle_nodes[j]+cfg->output_offset))[a]-start)<.0001f);
            assert(st->velocity[j][a]==0);
        }
        DWORD first=st->last_tick;
        for(int frame=1;frame<=250;frame++) {
            run_testicle_physics_for_person(0,first+frame*16);
            for(int j=0;j<3;j++) for(int a=0;a<3;a++) {
                float out=((float*)(testicle_nodes[j]+cfg->output_offset))[a];
                assert(isfinite(out));
                if(frame==1) assert(fabsf(out-start)<start*.3f);
                if(frame==250) assert(fabsf(out)<.001f);
                if(j==2) assert(st->angle[j][a]==0); /* End stays unsimulated. */
            }
        }
        assert(!st->activation_resume_pending);
    }
    body_profile_set_active_person_config(-1);
    puts("PASS: actual testicle activation preserves all three incoming joints, then converges smoothly; end joint remains unsimulated");
}

int main(void)
{
    captured_poseedit_this=editor;
    captured_poseedit_editpose=tracks;
    *(void**)(editor+POSEEDIT_EDITPOSE_OFFSET)=tracks;
    captured_poseedit_tracks_offset=POSEEDIT_TRACKS_OFFSET;
    engine_FindObjC=find_fixture;
    body_chain_poseeditor_mode_active=1;
    for(int p=0;p<4;p++) {
        body_profile_set_active_person_config(p);
        breasts_physics_cfg.output_offset=0x100+p*16;
        butt_physics_cfg.output_offset=0x100+p*16;
        breasts_physics_cfg.root_offset=butt_physics_cfg.root_offset=0xe8;
        breasts_physics_cfg.override_animation=butt_physics_cfg.override_animation=0;
        /* Include contact-only translations with the ordinary translation
           feature disabled: they still need to be removed on handoff. */
        butt_physics_bone_translation_enabled=0;
    }
    native_pose_values(0);
    for(int pass=0;pass<12;pass++) {
        simulate_offsets();
        assert(physx_poseedit_prepare_replacement(pass%2 ? "native-queue" : "File_New"));
        expect_authored();
        /* Incoming pose may author entirely different values, and the later
           deferred reset must never replay a snapshot from the previous pose. */
        native_pose_values(pass+1);
        assert(physx_poseedit_prepare_replacement("native-reset"));
        expect_authored();
        for(int p=0;p<4;p++) assert(!breasts_physics_states[p].initialized && !butt_physics_states[p].initialized);
    }
    puts("PASS: repeated New/Load/deferred reset removes breast/butt rotation and displacement in all persons; incoming authored values survive");

    /* Replaced ownership must cause an all-or-nothing skip, not a write into
       a new model or a partial restoration of one side. */
    simulate_offsets();
    breasts_physics_states[0].source_joint_raw[1]=nodes[0][1][1][0];
    butt_physics_states[0].motion.root_raw=roots[1];
    float displaced[2][2][2][3];
    for(int g=0;g<2;g++) for(int s=0;s<2;s++) for(int t=0;t<2;t++)
        memcpy(displaced[g][s][t],output_for(g,0,s,t),sizeof(float)*3);
    body_profile_set_active_person_config(-1);
    physx_poseedit_prepare_replacement("native-queue");
    for(int g=0;g<2;g++) for(int s=0;s<2;s++) for(int t=0;t<2;t++)
        assert(!memcmp(displaced[g][s][t],output_for(g,0,s,t),sizeof(float)*3));
    puts("PASS: replaced breast joints or butt root reject stale output restoration for both sides");

    native_pose_values(30);
    simulate_offsets();
    *(void**)(editor+0x28)=person_module;
    *(void**)(fade_object+8)=person_module;
    tramp_PoseFade_Ready=fake_fade_ready;
    assert(physx_poseedit_begin_replacement("native-queue"));
    assert(poseedit_fade_cleanup_editor==editor && physx_poseedit_transition_busy());
    unsigned int serial=physx_simulation_serial;
    for(int frame=0;frame<10;frame++) {
        physx_tick();
        assert(!hook_PoseFade_Ready(fade_object,(void*)1,(void*)2));
        for(int g=0;g<2;g++) for(int p=0;p<4;p++) {
            assert(state_for(g,p)->initialized && state_for(g,p)->output_applied);
            for(int s=0;s<2;s++) {
                assert(fabsf(output_for(g,p,s,0)[0]-(authored[g][p][s][0][0]+20+s))<0.0001f);
                assert(fabsf(output_for(g,p,s,1)[0]-(authored[g][p][s][1][0]+.025f*(s?-1:1)))<0.0001f);
            }
        }
    }
    assert(serial==physx_simulation_serial);
    fade_ready=1;
    *(void**)(fade_object+8)=roots; /* An unrelated fade task cannot release our pose. */
    assert(hook_PoseFade_Ready(fade_object,(void*)1,(void*)2));
    assert(poseedit_fade_cleanup_editor==editor);
    *(void**)(fade_object+8)=person_module;
    assert(hook_PoseFade_Ready(fade_object,(void*)1,(void*)2));
    assert(!poseedit_fade_cleanup_editor && !poseedit_file_command_depth);
    expect_authored();
    native_pose_values(31);
    hook_PoseFade_Ready(fade_object,(void*)1,(void*)2);
    physx_poseedit_prepare_replacement("native-reset");
    expect_authored();
    puts("PASS: native fade retains outgoing breast/butt physics until ready, then restores clean output exactly once before pose replacement; unrelated fade ignored");

    simulate_offsets();
    physx_poseedit_begin_replacement("File_Load");
    physx_poseedit_finish_file_command("File_Load",0x8000000au);
    assert(!poseedit_fade_cleanup_editor && !physx_poseedit_transition_busy());
    for(int p=0;p<4;p++) assert(breasts_physics_states[p].initialized && butt_physics_states[p].initialized);
    puts("PASS: cancellation/failure before a pose is queued releases the fade wait without resetting the outgoing physical pose");
    activation_regression();
    return 0;
}
