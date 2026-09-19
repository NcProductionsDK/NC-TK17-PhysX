#include "NC-TK17-PhysX.c"
#include <assert.h>

static BYTE image_storage[128],effect_storage[128],image_meta[0x380],effect_meta[0x380];
static BYTE image_dispatch[0x180],effect_dispatch[0x180];
static void *image_object=image_storage+32, *effect_object=effect_storage+32;
static int visible,target,dispatches,copies,destroys,original_value,gui_missing;
static float current;
static BYTE body_roots[4][0x200];
static int body_lookup, missing_body=-1;
static void *__cdecl find_gui(const char *name)
{
    if(gui_missing) return NULL;
    if(!strcmp(name,"GUI:SituationFade_Image")) return image_object;
    if(!strcmp(name,"GUI:SituationFade_FX")) return effect_object;
    if(body_lookup) for(int p=0;p<4;p++) {
        char expected[256];
        make_body_runtime_name(expected,sizeof(expected),body_chain_person_name(p),"root");
        if(!strcmp(name,expected)) return p==missing_body ? NULL : body_roots[p];
    }
    return NULL;
}
static unsigned int THISCALL visibility_get(void *self,DWORD member)
{ assert(self==image_object && member==SCRIPT_PROPERTY_WIDGET_VISIBILITY); return visible; }
static void THISCALL visibility_set(void *self,DWORD member,unsigned int value)
{ assert(self==image_object && member==SCRIPT_PROPERTY_WIDGET_VISIBILITY); visible=value; }
static unsigned int THISCALL target_get(void *self,DWORD member)
{ assert(self==effect_object && member==0x01fff0c4); return target; }
static float THISCALL current_get(void *self,DWORD member)
{ assert(self==effect_object && member==0x04fff0c4); return current; }
static void THISCALL target_set(void *self,DWORD member,unsigned int value)
{ assert(self==effect_object && member==0x01fff0c4); target=value; }
static void *THISCALL copy_hash(void *self,const void *other)
{ *(void**)self=*(void*const*)other; copies++; return self; }
static void THISCALL destroy_hash(void *self)
{ assert(*(void**)self==&original_value); *(void**)self=NULL; destroys++; }
static DWORD __cdecl dispatch(void *args)
{
    assert(*(void**)args==&original_value);
    assert(physx_customizer_fade.executing);
    dispatches++;
    /* Native entry's nested command must not enqueue a second transition. */
    assert(!physx_customizer_defer_entry("Customizer_Enter",args,200));
    physx_customizer_active=1;
    physx_customizer_entry_pending=1;
    return 0;
}
static void setup(void)
{
    memset(&physx_customizer_fade,0,sizeof(physx_customizer_fade));
    *(void**)(image_storage+32-0x18)=image_meta;
    *(void**)(effect_storage+32-0x18)=effect_meta;
    *(void**)(image_meta+0x360)=image_dispatch;
    *(void**)(effect_meta+0x310)=effect_dispatch;
    *(void**)(image_dispatch+0x140)=visibility_get;
    *(void**)(image_dispatch+0x144)=visibility_set;
    *(void**)(effect_dispatch+0x40)=target_get;
    *(void**)(effect_dispatch+0x44)=target_set;
    *(void**)(effect_dispatch+0x100)=current_get;
    engine_FindObjC=find_gui;
    real_AppMain_Command=dispatch;
    physx_customizer_hash_copy=copy_hash;
    physx_customizer_hash_destroy=destroy_hash;
    visible=target=dispatches=copies=destroys=gui_missing=0;
    current=0;
    physx_customizer_active=physx_customizer_entry_pending=0;
    body_chain_runtime_mode_transition_pending=0;
    body_lookup=0;
}
int main(void)
{
    void *args=&original_value;
    setup();
    assert(!physx_customizer_defer_entry("PhotoMode",&args,100));
    assert(physx_customizer_defer_entry("Customizer_Enter",&args,100));
    assert(visible && target==1 && copies==1 && !dispatches);
    args=NULL; /* The callback's original NameHash no longer exists. */
    assert(physx_customizer_defer_entry("Customizer_Enter",&args,110));
    current=.5f;
    physx_customizer_transition_tick(160);
    assert(!dispatches && copies==1);
    current=1.f;
    physx_customizer_transition_tick(240);
    assert(dispatches==1 && destroys==1 && target==1);
    physx_customizer_transition_tick(260);
    assert(target==1); /* Binding preparation is still pending. */
    physx_customizer_entry_pending=0;
    body_chain_physics_person_cfg[0].enabled=1;
    body_chain_physics_person_cfg[0].enabled_person[0]=1;
    runtime_body_chain_person_states[0].initialized=1;
    physics_environment_cfg.gravity_apply_to_body_chain=1;
    physics_environment_cfg.world_gravity_probe=1;
    physics_environment_cfg.gravity_dynamic_body_basis=1;
    physx_customizer_transition_tick(280);
    assert(target==1);
    physx_customizer_transition_tick(300);
    assert(target==1); /* Body is bound, but its gravity is not ready yet. */
    runtime_body_chain_person_states[0].gravity_probe_promoted=1;
    physx_customizer_transition_tick(320);
    assert(target==1);
    physx_customizer_transition_tick(330);
    assert(target==1); /* Old readiness alone cannot reveal an unconfirmed direction. */
    runtime_body_chain_person_states[0].gravity_sample.accepted=1;
    physx_customizer_transition_tick(335);
    assert(target==1);
    physx_customizer_transition_tick(340);
    assert(target==0 && visible);
    current=.5f; physx_customizer_transition_tick(360); assert(visible);
    current=0; physx_customizer_transition_tick(420); assert(!visible && !physx_customizer_fade.phase);
    puts("PASS: retained command executes exactly once under black; native preparation completes before smooth reveal; repeated/nested requests are safe");

    breasts_physics_person_state_t *paired[2]={&breasts_physics_states[0],&butt_physics_states[0]};
    body_chain_physics_config_t *cfg[2]={&breasts_physics_person_cfg[0],&butt_physics_person_cfg[0]};
    for(int j=0;j<2;j++) {
        body_chain_person_state_t *gravity=j?&paired[j]->motion:&paired[j]->gravity_motion;
        cfg[j]->enabled=cfg[j]->enabled_person[0]=paired[j]->initialized=1;
        cfg[j]->gravity_angle=10;
        gravity->gravity_probe_promoted=1;
        gravity->gravity_sample.accepted=0;
        assert(!physx_customizer_body_ready());
        gravity->gravity_sample.accepted=1;
        assert(physx_customizer_body_ready());
        cfg[j]->enabled=0;
    }
    puts("PASS: breast and butt readiness also requires a fresh direction before reveal");

    /* Reproduce the log: Person02 is ready, Person01/03 retain initialized
       paired solvers but have zero placeholder roots throughout Customizer. */
    body_lookup=1;
    physics_environment_cfg.gravity_probe_require_nonzero_root=1;
    physics_environment_cfg.gravity_probe_motion_epsilon=.005f;
    for(int p=0;p<4;p++) {
        body_chain_physics_person_cfg[p].root_offset=0xe8;
        breasts_physics_person_cfg[p].enabled=breasts_physics_person_cfg[p].enabled_person[p]=1;
        breasts_physics_person_cfg[p].gravity_angle=10;
        breasts_physics_states[p].initialized=1;
        breasts_physics_states[p].gravity_motion.gravity_probe_promoted=0;
        breasts_physics_states[p].gravity_motion.gravity_sample.accepted=0;
    }
    *(float*)(body_roots[1]+0xe8)=3;
    assert(!physx_customizer_body_ready()); /* Selected actor still warming. */
    breasts_physics_states[1].gravity_motion.gravity_probe_promoted=1;
    breasts_physics_states[1].gravity_motion.gravity_sample.accepted=1;
    assert(physx_customizer_body_ready());
    assert(physx_customizer_fade.inactive_person_mask==13);
    /* The complete fade must release promptly, not through the 3s timeout. */
    physx_customizer_fade.phase=2;
    physx_customizer_fade.start=500;
    physx_customizer_fade.ready_frames=0;
    target=1; current=1;
    physx_customizer_transition_tick(800);
    assert(target==1);
    physx_customizer_transition_tick(816);
    assert(target==0);
    *(float*)(body_roots[1]+0xe8)=0;
    assert(!physx_customizer_body_ready()); /* All placeholders is not ready. */
    *(float*)(body_roots[1]+0xe8)=3;
    missing_body=0;
    assert(!physx_customizer_body_ready()); /* Unknown binding cannot be ignored. */
    missing_body=-1;
    physics_environment_cfg.gravity_probe_require_nonzero_root=0;
    assert(!physx_customizer_body_ready()); /* Respect custom origin policy. */
    puts("PASS: ready Customizer actor releases fade in two frames despite inactive room actors; all-placeholder and unknown bindings remain guarded");

    args=&original_value;
    setup(); visible=1; target=1;
    assert(!physx_customizer_defer_entry("Customizer_Enter",&args,100));
    assert(!copies && !dispatches);
    setup(); assert(physx_customizer_defer_entry("Customizer_Enter",&args,100));
    target=0; physx_customizer_transition_tick(120);
    assert(!dispatches && destroys==1 && !physx_customizer_fade.phase);
    setup(); assert(physx_customizer_defer_entry("Customizer_Enter",&args,100));
    gui_missing=1; physx_customizer_transition_tick(120);
    assert(dispatches==1 && destroys==1 && !physx_customizer_fade.phase);
    setup(); assert(physx_customizer_defer_entry("Customizer_Enter",&args,100));
    physx_customizer_transition_tick(4100);
    assert(dispatches==1 && destroys==1 && !visible && !physx_customizer_fade.phase);
    puts("PASS: native fade ownership, GUI replacement and timeout paths release retained arguments without duplicate commands or a stuck black screen");
    return 0;
}
