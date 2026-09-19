#include "NC-TK17-PhysX.c"
#include <assert.h>

static void key(const char *path, const char *name, const char *value)
{
    assert(WritePrivateProfileStringA("body_colliders", name, value, path));
}

int main(int argc, char **argv)
{
    physx_wire_batch batch = {0};
    body_chain_collider_person_state_t saved[4];
    assert(argc == 3);
    body_profile_set_active_person_config(-1);
    key(argv[1], "debug_draw", "true");
    key(argv[1], "debug_draw_filter", "spine02");
    key(argv[1], "debug_draw_person", "1");
    key(argv[1], "debug_draw_capsules", "false");
    body_collider_debug_profile(argv[1], 0);
    assert(body_collider_debug_person == 1);
    assert(body_collider_debug_node_selected(BODY_COLLIDER_STOMACH_02));
    assert(!body_collider_debug_node_selected(BODY_COLLIDER_STOMACH_01));
    assert(!body_collider_debug_edge_selected(BODY_COLLIDER_STOMACH_01, BODY_COLLIDER_STOMACH_02));
    body_chain_collider_global_cfg.enabled = 1;
    for (int i = 0; i < BODY_COLLIDER_NODE_COUNT; i++)
        body_chain_set_radius_scalar(body_chain_collider_global_cfg.node_radius[i], .04f);
    body_chain_set_radius_scalar(body_chain_collider_global_cfg.stomach_radius[1], .04f);
    for (int p = 0; p < 4; p++) {
        body_chain_collider_person_state_t *state = &body_chain_collider_states[p];
        state->valid[BODY_COLLIDER_ROOT] = state->valid[BODY_COLLIDER_STOMACH_02] = 1;
        state->valid[BODY_COLLIDER_THIGH_L] = state->valid[BODY_COLLIDER_THIGH_R] = 1;
        state->valid[BODY_COLLIDER_KNEE_L] = 1;
        state->basis_valid = state->ready = state->stomach_points_ready = 1;
        state->basis_h[0] = state->basis_v[1] = state->basis_s[2] = 1;
        state->view_position[BODY_COLLIDER_ROOT][2] = .5f;
        state->local_position[BODY_COLLIDER_KNEE_L][1] = .2f;
        body_chain_collider_person_cfg[p] = body_chain_collider_global_cfg;
    }
    memcpy(saved, body_chain_collider_states, sizeof(saved));
    memset(&body_chain_hook5_projection, 0, sizeof(body_chain_hook5_projection));
    body_chain_hook5_projection._11 = body_chain_hook5_projection._22 =
        body_chain_hook5_projection._33 = body_chain_hook5_projection._44 = 1;
    physx_wire_collect_body(&batch);
    assert(batch.count == 192); /* One sphere, despite four live people. */
    body_collider_debug_person = 0;
    batch.count = 0; physx_wire_collect_body(&batch); assert(batch.count == 4 * 192);
    assert(!memcmp(saved, body_chain_collider_states, sizeof(saved)));
    puts("PASS: one sphere/person selection, all persons, drawing preserves collision state");

    body_collider_debug_person = 1;
    body_profile_set_active_person_config(0);
    assert(body_collider_debug_set_filter(" LEFT_THIGH ; solo "));
    assert(body_collider_debug_node_selected(BODY_COLLIDER_THIGH_L));
    assert(!body_collider_debug_node_selected(BODY_COLLIDER_THIGH_R));
    body_profile_set_active_person_config(-1);
    batch.count = 0; physx_wire_collect_body(&batch); assert(batch.count == 192);
    body_chain_collider_person_cfg[0].debug_draw_capsules = 1;
    batch.count = 0; physx_wire_collect_body(&batch); assert(batch.count > 192);
    body_profile_set_active_person_config(0);
    assert(body_collider_debug_set_filter("thigh"));
    body_chain_collider_cfg.debug_draw_capsules = 0;
    body_profile_set_active_person_config(-1);
    batch.count = 0; physx_wire_collect_body(&batch); assert(batch.count == 384);
    puts("PASS: single-side/pair filters and independent capsule toggle");

    body_profile_set_active_person_config(0);
    assert(body_collider_debug_set_filter("left_finger02_03"));
    assert(body_collider_debug_node_selected(BODY_COLLIDER_FINGER02_L_03));
    assert(!body_collider_debug_node_selected(BODY_COLLIDER_FINGER02_L_02));
    assert(body_collider_debug_set_filter("finger02"));
    assert(body_collider_debug_node_selected(BODY_COLLIDER_FINGER02_R_END));
    assert(!body_collider_debug_set_filter("not_a_collider"));
    body_profile_set_active_person_config(-1);
    batch.count = 0; physx_wire_collect_body(&batch); assert(batch.count == 0);
    puts("PASS: finger joints/groups and invalid filter does not draw everything");

    key(argv[2], "debug_draw_filter", "right_thigh");
    key(argv[2], "debug_draw_capsules", "true");
    key(argv[2], "debug_draw_person", "4"); /* Global-only setting. */
    lstrcpynA(body_profile_person_sidecar_path[0], argv[2], MAX_PATH * 4);
    body_profile_person_sidecar_active[0] = 1;
    body_profile_rebuild_effective_configs();
    assert(body_collider_debug_person == 1);
    body_profile_set_active_person_config(0);
    assert(body_collider_debug_node_selected(BODY_COLLIDER_THIGH_R));
    assert(!body_collider_debug_node_selected(BODY_COLLIDER_STOMACH_02));
    assert(body_chain_collider_cfg.debug_draw_capsules);
    body_profile_set_active_person_config(1);
    assert(body_collider_debug_node_selected(BODY_COLLIDER_STOMACH_02));
    assert(!body_chain_collider_cfg.debug_draw_capsules);
    body_profile_set_active_person_config(-1);
    key(argv[2], "debug_draw_filter", NULL);
    key(argv[2], "debug_draw_capsules", NULL);
    body_profile_rebuild_effective_configs();
    body_profile_set_active_person_config(0);
    assert(body_collider_debug_node_selected(BODY_COLLIDER_STOMACH_02));
    assert(!body_chain_collider_cfg.debug_draw_capsules);
    body_profile_set_active_person_config(-1);
    puts("PASS: sidecar precedence, global-only person selector, reload/removal inheritance");

    key(argv[1], "debug_draw_filter", NULL);
    key(argv[1], "debug_draw_person", NULL);
    key(argv[1], "debug_draw_capsules", NULL);
    body_collider_debug_profile(argv[1], 0);
    assert(!body_collider_debug_person && !body_chain_collider_cfg.debug_draw_filtered);
    assert(body_chain_collider_cfg.debug_draw_capsules && body_collider_debug_chain_selected());
    body_chain_collider_global_cfg.debug_draw = 0;
    key(argv[2], "debug_draw", "true");
    body_profile_rebuild_effective_configs();
    assert(body_collider_debug_any());
    assert(!body_chain_collider_person_cfg[1].debug_draw);
    puts("PASS: removed global keys restore defaults; sidecar-only drawing can enable renderer");
    free(batch.vertices);
    return 0;
}
