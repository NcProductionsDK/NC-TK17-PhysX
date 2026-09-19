#include "NC-TK17-PhysX.c"

static physx_chain_t test_chain;
static float matrices[3][32];
static int owners[2], objects[3];
static BYTE refresh_nodes[3][0x200];
static int refresh_writes;
static void *__cdecl find_refresh_node(const char *name)
{
    if (!strstr(name, "Person01Anim:")) return NULL;
    if (strstr(name, ":TRS_group")) return refresh_nodes[0];
    if (strstr(name, ":root")) return refresh_nodes[1];
    if (strstr(name, ":spine_joint04")) return refresh_nodes[2];
    return NULL;
}
static unsigned int THISCALL get_refresh_version(void *self)
{ return *(unsigned int*)((BYTE*)self + 0xfc); }
static void THISCALL set_refresh_version(void *self, unsigned int value)
{ *(unsigned int*)((BYTE*)self + 0xfc)=value; refresh_writes++; }

static void require(int ok, const char *message)
{
    if (!ok) { fprintf(stderr, "FAIL: %s\n", message); exit(1); }
}

static void setup(int count)
{
    memset(&test_chain, 0, sizeof(test_chain));
    memset(matrices, 0, sizeof(matrices));
    test_chain.addon_chain = 1;
    test_chain.target_count = count;
    test_chain.addon_root_settle_until_tick = 2200;
    physx_customizer_active = 1;
    physx_customizer_entry_tick = 1000;
    captured_camera_inverse_valid = 1;
    captured_camera_change_tick = 800;
    physics_environment_cfg.gravity_probe_camera_quiet_ms = 10;
    physics_environment_cfg.body_chain_camera_quarantine_ms = 0;
    for (int i = 0; i < count; i++) {
        physx_target_t *t = &test_chain.targets[i];
        t->addon_simulated_target = 1;
        t->object = t->raw_object = &objects[i];
        t->s_object = t->s_raw_object = matrices[i];
        t->s_translation_base = t->s_rotation_base = matrices[i];
        t->s_translation_offset = 0x048;
        t->s_rotation_offset = 0x038;
        matrices[i][0x018 / 4] = matrices[i][0x02c / 4] =
            matrices[i][0x040 / 4] = 1.0f;
        matrices[i][0x048 / 4] = i * .1f;
    }
}

static int sample(DWORD tick)
{
    return addon_customizer_sample_ready(&test_chain, &owners[0], tick);
}

static void transition_tests(void)
{
    engine_FindObjC=find_refresh_node;
    engine_TBaseTransformGetMatrixVersion=get_refresh_version;
    engine_TBaseTransformSetMatrixVersion=set_refresh_version;
    for (int editor = 0; editor < 2; editor++) {
        body_chain_poseeditor_mode_active = editor;
        body_chain_runtime_mode_transition_pending = 0;
        physx_customizer_active = physx_customizer_entry_pending = 0;
        physx_observe_customizer_visibility(0, 0, 0, 900);
        require(!physx_customizer_active, "Photo/FreeMode mistaken for Customizer");
        physx_observe_customizer_visibility(1, 1, 1, 950);
        require(!physx_customizer_active, "overlapping old UI accepted");
        physx_observe_customizer_visibility(1, 1, 0, 1000);
        require(physx_customizer_active && physx_customizer_entry_pending &&
                !body_chain_poseeditor_mode_active, "entry not detected");
        require(body_chain_runtime_mode_transition_pending == (editor ? 2 : 0),
                "PoseEditor ownership handoff changed");
        runtime_body_chain_person_states[0].cache_verify_tick = 500;
        runtime_testicle_physics_states[0].resolve_retry_tick = 999;
        breasts_physics_states[0].cache_verify_tick = 500;
        butt_physics_states[0].cache_verify_tick = 500;
        body_chain_collider_states[0].ready = 1;
        last_update_tick = 999;
        memset(refresh_nodes, 0, sizeof(refresh_nodes));
        refresh_writes=0;
        runtime_body_chain_person_states[0].gravity_probe_promoted=1;
        runtime_body_chain_person_states[0].gravity_sample.trusted_valid=1;
        breasts_physics_states[0].gravity_motion.gravity_probe_promoted=1;
        butt_physics_states[0].motion.gravity_probe_promoted=1;
        require(physx_prepare_customizer_entry(), "entry must defer old-pose sampling for one native pass");
        require(refresh_writes==3, "root, placement and chest native caches were not invalidated");
        for(int n=0;n<3;n++) {
            require(get_refresh_version(refresh_nodes[n])==1, "native version was not advanced");
            *(unsigned int*)(refresh_nodes[n]+0xfc)=0;
            for(int b=0;b<sizeof(refresh_nodes[n]);b++)
                require(refresh_nodes[n][b]==0, "entry changed an authored transform");
        }
        require(!runtime_body_chain_person_states[0].gravity_probe_promoted &&
                !runtime_body_chain_person_states[0].gravity_sample.trusted_valid &&
                !breasts_physics_states[0].gravity_motion.gravity_probe_promoted &&
                !butt_physics_states[0].motion.gravity_probe_promoted,
                "outgoing gravity readiness survived entry");
        require(!physx_prepare_customizer_entry() && refresh_writes==3,
                "continuous Customizer invalidates native caches repeatedly");
        require(!last_update_tick && !body_chain_collider_states[0].ready &&
                !runtime_body_chain_person_states[0].cache_verify_tick &&
                !runtime_testicle_physics_states[0].resolve_retry_tick &&
                !breasts_physics_states[0].cache_verify_tick &&
                !butt_physics_states[0].cache_verify_tick,
                "entry did not refresh body/add-on binding and contact caches");
        physx_observe_customizer_visibility(1, 1, 0, 1100);
        require(!physx_customizer_entry_pending && physx_customizer_entry_tick == 1000,
                "continuous Customizer reinitializes every poll");
        require(physx_customizer_startup_active(3999) &&
                !physx_customizer_startup_active(4000), "fast polling unbounded");
        physx_observe_customizer_visibility(0, 0, 0, 4100);
        require(!physx_customizer_startup_active(4100), "exit kept fast startup");
        physx_observe_customizer_visibility(1, 1, -1, 4200);
        require(!physx_customizer_active, "unknown mode treated as verified");
        physx_observe_customizer_visibility(1, 1, 0, 4300);
        require(physx_customizer_entry_pending && physx_customizer_entry_tick == 4300,
                "re-entry not recognized");
    }
    engine_FindObjC=NULL;
    engine_TBaseTransformGetMatrixVersion=NULL;
    engine_TBaseTransformSetMatrixVersion=NULL;
    puts("PASS: PoseEditor/FreeMode entry, exit, unknown/overlapping UI and re-entry");
}

static void readiness_tests(void)
{
    for (int count = 1; count <= 3; count += 2) {
        const DWORD steps[] = {33, 16, 6};
        for (int rate = 0; rate < 3; rate++) {
            setup(count);
            DWORD step = steps[rate], tick = 1000;
            while (!sample(tick)) {
                require(tick < 1100, "verified single-bone/chain stayed delayed");
                require(!sample(tick), "duplicate frame advanced confirmation");
                tick += step;
            }
            require(tick >= 1032 && test_chain.customizer_ready_samples >= 3,
                    "readiness skipped temporal confirmation");
        }
    }
    setup(1);
    sample(1000); sample(1016);
    require(!addon_customizer_sample_ready(&test_chain, &owners[1], 1032),
            "replacement owner inherited readiness");
    setup(1); sample(1000); sample(1016);
    test_chain.targets[0].object = &objects[2];
    require(!sample(1032), "replacement target inherited readiness");
    setup(1); sample(1000); sample(1016);
    matrices[0][0x048 / 4] = .2f;
    require(!sample(1032), "jumping transform accepted");
    require(!sample(1048) && sample(1064), "stable replacement did not recover");
    puts("PASS: single-bone/chain readiness at 30/60/144 Hz; replaced owners/targets and jumps restart confirmation");
}

static void rejection_tests(void)
{
    for (int test = 0; test < 10; test++) {
        setup(1);
        sample(1000); sample(1016);
        switch (test) {
        case 0: physx_customizer_active = 0; break;
        case 1: captured_camera_inverse_valid = 0; break;
        case 2: captured_camera_change_tick = 1020; break;
        case 3: test_chain.targets[0].s_translation_offset = 0x07c; break;
        case 4: test_chain.targets[0].addon_object_name_fallback = 1; break;
        case 5: test_chain.targets[0].s_object = NULL; break;
        case 6: matrices[0][0x048 / 4] = NAN; break;
        case 7: matrices[0][0x018 / 4] = 0; break;
        case 8: test_chain.object_transform_chain = 1; break;
        case 9: matrices[0][0x028 / 4] = 1; matrices[0][0x02c / 4] = 0; break;
        }
        require(!sample(1032) && test_chain.customizer_ready_samples == 0,
                "unsafe/unverified data accepted by fast startup");
        require(addon_chain_settling(&test_chain, 1032), "fallback wait bypassed");
        require(!addon_chain_settling(&test_chain, 2200), "ordinary fallback cannot finish");
    }
    setup(1); captured_camera_change_tick = 1000;
    require(!sample(1144), "late camera traversal gap accepted");
    require(!sample(1160) && !sample(1176) && sample(1192),
            "quiet-camera confirmation did not recover");
    addon_chain_reset_runtime_state(&test_chain);
    require(!test_chain.customizer_ready_samples, "rebind retained readiness");
    puts("PASS: camera activity, stale layouts, generic bindings, invalid matrices and rebinds retain safeguards");
}

int main(void)
{
    transition_tests();
    readiness_tests();
    rejection_tests();
    return 0;
}
