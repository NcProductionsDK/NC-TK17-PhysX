static int __attribute__((thiscall)) evaluate(void *, float *, double);
static void __attribute__((thiscall)) update(void *, double);
#define POSEEDIT_TRACK_EVALUATE_ADDR ((BYTE*)evaluate)
#define POSEEDIT_TRACK_UPDATE_ADDR ((BYTE*)update)
#include "NC-TK17-PhysX.c"
#include <assert.h>

static BYTE editor[0x400], table[POSEEDIT_TRACKS_OFFSET+384*POSEEDIT_TRACK_SIZE];
static BYTE joints[2][0x400];
static struct { int count; BYTE key[0x30]; } keys[2];
static float live[2][3];
static int nil, empty;
static int testicles, native_writes;
static void *nil_ptr=&nil, *empty_ptr=&empty;
static const float bend[2][3]={{12,-35,65},{-20,45,-70}};
static void *__cdecl find_joint(const char *name)
{
    if(strstr(name,testicles?":testicles_joint01":":penis_joint02")) return joints[0];
    if(strstr(name,testicles?":testicles_joint02":":penis_joint03")) return joints[1];
    return NULL;
}
static int __attribute__((thiscall)) evaluate(void *self,float *out,double frame)
{
    BYTE *key=*(BYTE**)((BYTE*)self+0x24);
    assert(*(int*)(key-4)==1);
    memcpy(out,key,12);
    return 1;
}
static void __attribute__((thiscall)) update(void *self,double frame)
{
    BYTE *track=self;
    native_writes++;
    void *obj=*(void**)(track+4);
    int j=obj==joints[0]?0:1;
    assert(obj==joints[j]);
    evaluate(track,live[j],frame);
}
static void setup(body_chain_person_state_t *state)
{
    memset(state,0,sizeof(*state));
    memset(table,0,sizeof(table));
    engine_FindObjC=find_joint;
    engine_G_NilWeakObjTarget_ptr=&nil_ptr;
    engine_G_NullArray_ptr=&empty_ptr;
    captured_poseedit_this=editor;
    captured_poseedit_editpose=table;
    captured_poseedit_tracks_offset=POSEEDIT_TRACKS_OFFSET;
    body_chain_physics_cfg.poseeditor_total_tracks=384;
    body_chain_physics_cfg.override_animation=1;
    body_chain_physics_cfg.poseeditor_track_override=1;
    for(int j=0;j<2;j++) {
        keys[j].count=1;
        memcpy(keys[j].key,bend[j],12);
        memcpy(live[j],bend[j],12);
        BYTE *track=poseedit_track_slot(0,j?POSEEDIT_TRACK_PENIS_JOINT03:POSEEDIT_TRACK_PENIS_JOINT02);
        *(void**)(track+4)=joints[j];
        *(void**)(track+0x24)=keys[j].key;
    }
}
static void test_testicle_source_handoff(void)
{
    body_chain_person_state_t state;
    setup(&state);
    testicles=1;
    testicle_physics_cfg=body_chain_physics_cfg;
    testicle_physics_cfg.max_angle=180;
    for(int j=0;j<3;j++) for(int a=0;a<3;a++) {
        testicle_physics_cfg.link_min_angle[j][a]=-180;
        testicle_physics_cfg.link_max_angle[j][a]=180;
    }
    const int ids[]={POSEEDIT_TRACK_TESTICLES_JOINT01,POSEEDIT_TRACK_TESTICLES_JOINT02};
    memset(table,0,sizeof(table));
    for(int j=0;j<2;j++) {
        BYTE *track=poseedit_track_slot(0,ids[j]);
        *(void**)(track+4)=joints[j];
        *(void**)(track+0x24)=keys[j].key;
    }
    float source[3][3]={{0},{0},{3,4,5}};
    float *output[3]={source[0],source[1],source[2]};
    native_writes=0;
    assert(capture_testicle_pose_start(0,"Person01",&state,source));
    assert(!memcmp(source,bend,sizeof(bend)) && source[2][2]==5);
    assert(native_writes==0); /* SSimpleTransform uses the solver's own channel. */
    assert(suppress_poseeditor_testicle_track_for_person(0,"Person01",&state));
    state.activation_resume_pending=1;
    assert(body_chain_seed_clothing_resume(&state,&testicle_physics_cfg,source,2));
    body_chain_write_clothing_resume_pose(&state,output,2);
    assert(!memcmp(source,bend,sizeof(bend)) && source[2][2]==5);
    assert(neutralize_testicle_physics_animation("Person01",&state));
    assert(native_writes==0); /* A native tween would overwrite the spring output. */
    source[0][1]=-13; source[1][2]=8;
    assert(neutralize_testicle_physics_animation("Person01",&state));
    assert(source[0][1]==-13 && source[1][2]==8 && native_writes==0);
    restore_poseeditor_joint01_track(&state);
    for(int j=0;j<2;j++) {
        BYTE *track=poseedit_track_slot(0,ids[j]);
        assert(*(void**)(track+0x24)==keys[j].key);
        assert(!memcmp(keys[j].key,bend[j],12));
        update(track,0);
    }
    assert(!memcmp(live,bend,sizeof(bend)));
    reset_body_chain_person_state(&state);
    assert(!state.pose_compensation_valid);
    *(void**)(poseedit_track_slot(0,ids[1])+0x24)=empty_ptr;
    float untouched[3][3];
    memcpy(untouched,source,sizeof(source));
    assert(!capture_testicle_pose_start(0,"Person01",&state,source));
    assert(!memcmp(source,untouched,sizeof(source))); /* Empty tip falls back atomically. */
    puts("PASS: testicle native tracks seed both source joints exactly, preserve the end joint and OFF keys, and never overwrite simulated output with a second tween");
}

int main(void)
{
    body_chain_person_state_t state;
    const int rates[]={30,60,144};
    for(int rate=0;rate<3;rate++) {
        int hz=rates[rate];
        setup(&state);
        assert(capture_body_chain_pose_compensation(0,"Person01",&state));
        assert(suppress_poseeditor_joint02_03_tracks_for_person(0,"Person01",&state));
        body_chain_apply_pose_start("Person01",&state);
        assert(!memcmp(live,bend,sizeof(bend)));
        /* No advance during startup: repeated ownership writes retain both bends. */
        body_chain_apply_pose_start("Person01",&state);
        assert(!memcmp(live,bend,sizeof(bend)));
        for(int f=0;f<hz;f++) {
            float previous=state.pose_start_weight;
            body_chain_advance_pose_start(&state,1.f/hz);
            assert(state.pose_start_weight<=previous);
            /* The normal neutralizer restores zero before applying the residual. */
            memset(live,0,sizeof(live));
            body_chain_apply_pose_start("Person01",&state);
        }
        assert(state.pose_start_weight==0);
        for(int j=0;j<2;j++) {
            assert(!memcmp(keys[j].key,bend[j],12));
            for(int a=0;a<3;a++) assert(live[j][a]==0);
        }
        restore_poseeditor_joint01_track(&state);
        for(int j=0;j<2;j++) {
            BYTE *track=poseedit_track_slot(0,j?POSEEDIT_TRACK_PENIS_JOINT03:POSEEDIT_TRACK_PENIS_JOINT02);
            assert(*(void**)(track+4)==joints[j]);
            assert(*(void**)(track+0x24)==keys[j].key);
            update(track,0);
        }
        assert(!memcmp(live,bend,sizeof(bend)));
    }
    setup(&state);
    assert(capture_body_chain_pose_compensation(0,"Person01",&state));
    assert(suppress_poseeditor_joint02_03_tracks_for_person(0,"Person01",&state));
    BYTE *slot=state.pose_track_extra_base[0];
    *(void**)(slot+0x24)=keys[0].key; /* Incoming native pose takes ownership. */
    memset(live,0,sizeof(live));
    body_chain_apply_pose_start("Person01",&state);
    assert(state.pose_start_weight==0 && live[0][0]==0);
    state.pose_start_weight=1;
    reset_body_chain_person_state(&state);
    assert(state.pose_start_weight==0);
    puts("PASS: native joint02/03 bends survive ownership transfer, hold during startup, ease to zero, preserve keys and reject replaced ownership");
    test_testicle_source_handoff();
    return 0;
}
