#include "NC-TK17-PhysX.c"
#include <assert.h>

static void close_value(float a, float b) { assert(fabsf(a-b) < 0.00001f); }
static void key(const char *path, const char *name, const char *value)
{
    assert(WritePrivateProfileStringA("body_colliders", name, value, path));
}

int main(int argc, char **argv)
{
    float raw[4][3]={{0,0,0},{.1f,0,0},{.2f,0,0},{.3f,0,0}};
    float shape[4][3], predicted[4][3], expected[4][3], sampled[4][3];
    float delta[3][2]={{1,2},{2,1},{1,1}};
    body_chain_person_state_t state={0};
    int source;
    assert(argc == 3);
    key(argv[1], "chain_radius", "0.02");
    body_penis_profile(argv[1], 0);
    close_value(body_chain_penis_radius(), .02f);
    close_value(body_chain_collider_cfg.chain_radius, .02f);
    key(argv[1], "penis_radius", "0.03");
    key(argv[1], "penis01_fine_offset", "0.01,-0.02,0.03");
    key(argv[1], "penis02_fine_offset", "0.02,0.01,-0.01");
    key(argv[1], "penis03_fine_offset", "0.03,0.02,0.01");
    body_penis_profile(argv[1], 0);
    close_value(body_chain_penis_radius(), .03f);
    close_value(body_chain_collider_cfg.chain_radius, .02f);
    key(argv[2], "chain_radius", "0.015");
    key(argv[2], "penis02_fine_offset", "0,0.04,0");
    lstrcpynA(body_profile_person_sidecar_path[0], argv[2], MAX_PATH*4);
    body_profile_person_sidecar_active[0]=1;
    body_profile_rebuild_effective_configs();
    body_profile_set_active_person_config(0);
    close_value(body_chain_penis_radius(), .015f);
    close_value(body_chain_collider_cfg.penis_fine_offset[0][0], .01f);
    close_value(body_chain_collider_cfg.penis_fine_offset[1][1], .04f);
    body_profile_set_active_person_config(1);
    close_value(body_chain_penis_radius(), .03f);
    close_value(body_chain_collider_cfg.penis_fine_offset[1][1], .01f);
    body_profile_set_active_person_config(-1);
    puts("PASS: preferred radius, legacy compatibility, independent margin, per-person inheritance");

    memcpy(shape,raw,sizeof(raw));
    body_chain_penis_offset_points(shape,1);
    for(int i=0;i<4;i++) for(int a=0;a<3;a++)
        close_value(shape[i][a],raw[i][a]+body_chain_collider_cfg.penis_fine_offset[i<3?i:2][a]);
    for(int a=0;a<3;a++) close_value(shape[3][a]-shape[2][a], raw[3][a]-raw[2][a]);
    body_chain_collider_states[0].chain_points_ready=1;
    body_chain_collider_states[0].chain_points_fresh=1;
    for(int i=0;i<4;i++) body_chain_collider_states[0].chain_point_valid[i]=1;
    memcpy(body_chain_collider_states[0].chain_local_point,raw,sizeof(raw));
    assert(body_chain_collision_points_local(&body_chain_collider_states[0],&state,sampled,&source,0));
    assert(!memcmp(raw,sampled,sizeof(raw)));
    assert(body_contact_predict(&body_chain_physics_cfg,&state,raw,delta,expected));
    assert(body_contact_predict_geometry(&body_chain_physics_cfg,&state,shape,delta,predicted));
    body_chain_penis_offset_points(expected,1);
    for(int i=0;i<4;i++) for(int a=0;a<3;a++) close_value(predicted[i][a],expected[i][a]);
    assert(!memcmp(raw,body_chain_collider_states[0].chain_local_point,sizeof(raw)));
    puts("PASS: shared endpoints, terminal offset, raw engine samples preserved, contact predictions match geometry");

    /* Translation of collision geometry must not alter the joint-space solve. */
    {
        body_chain_contact_t contact[2];
        body_chain_person_state_t original={0}, shifted={0};
        float first[3][2],second[3][2],up[3]={0,1,0},body[3]={0};
        int count=0;
        body_chain_physics_cfg.horizontal_output_axis=1;
        body_chain_physics_cfg.vertical_output_axis=2;
        for(int j=0;j<3;j++) for(int a=0;a<3;a++) {
            body_chain_physics_cfg.link_min_angle[j][a]=-90;
            body_chain_physics_cfg.link_max_angle[j][a]=90;
        }
        body_chain_collider_cfg.response_strength=1;
        body_chain_collider_cfg.response_max_degrees_per_tick=20;
        body_chain_collider_cfg.collision_iterations=2;
        memset(body_chain_collider_cfg.penis_fine_offset,0,sizeof(body_chain_collider_cfg.penis_fine_offset));
        body_chain_store_contact(contact,&count,2,0,1,.002f,raw[1],body,up,1);
        body_contact_solve(&body_chain_physics_cfg,&original,raw,contact,count,3,first);
        for(int j=0;j<3;j++) body_chain_collider_cfg.penis_fine_offset[j][1]=.04f;
        memcpy(shape,raw,sizeof(raw));body_chain_penis_offset_points(shape,1);
        body[1]=.04f;count=0;
        body_chain_store_contact(contact,&count,2,0,1,.002f,shape[1],body,up,1);
        body_contact_solve(&body_chain_physics_cfg,&shifted,shape,contact,count,3,second);
        assert(fabsf(first[0][0])+fabsf(first[0][1])>0.0001f);
        for(int j=0;j<3;j++) for(int a=0;a<2;a++) close_value(first[j][a],second[j][a]);
    }
    puts("PASS: offset collision solve retains physical joint pivots");

    {
        physx_wire_batch before={0},after={0};
        body_chain_collider_person_state_t *live=&body_chain_collider_states[0];
        body_chain_collider_person_state_t snapshot;
        body_chain_collider_cfg.enabled=1;
        body_chain_collider_cfg.debug_draw=1;
        body_chain_collider_cfg.debug_draw_capsules=1;
        body_chain_collider_cfg.penis_collision_enabled=1;
        body_collider_debug_set_filter("penis");
        memset(body_chain_collider_cfg.penis_fine_offset,0,sizeof(body_chain_collider_cfg.penis_fine_offset));
        body_chain_collider_person_cfg[0]=body_chain_collider_cfg;
        for(int p=1;p<4;p++) body_chain_collider_person_cfg[p].debug_draw=0;
        live->valid[BODY_COLLIDER_ROOT]=live->basis_valid=live->ready=1;
        live->basis_h[0]=live->basis_v[1]=live->basis_s[2]=1;
        live->view_position[BODY_COLLIDER_ROOT][2]=.5f;
        live->chain_points_update_tick=GetTickCount();
        snapshot=*live;
        memset(&body_chain_hook5_projection,0,sizeof(body_chain_hook5_projection));
        body_chain_hook5_projection._11=body_chain_hook5_projection._22=
            body_chain_hook5_projection._33=body_chain_hook5_projection._44=1;
        physx_wire_collect_body(&before);
        assert(before.count>0);
        for(int j=0;j<3;j++) body_chain_collider_person_cfg[0].penis_fine_offset[j][1]=.04f;
        physx_wire_collect_body(&after);
        assert(before.count==after.count);
        for(unsigned v=0;v<before.count;v++) {
            close_value(after.vertices[v].clip[0],before.vertices[v].clip[0]);
            close_value(after.vertices[v].clip[1],before.vertices[v].clip[1]+.04f);
            close_value(after.vertices[v].clip[2],before.vertices[v].clip[2]);
        }
        assert(!memcmp(live,&snapshot,sizeof(snapshot)));
        free(before.vertices);free(after.vertices);
    }
    puts("PASS: production wire drawing applies offsets exactly once and preserves cached pivots");

    key(argv[1], "penis_radius", "nan");
    key(argv[1], "penis01_fine_offset", "0,inf,0");
    body_penis_profile(argv[1],0);
    close_value(body_chain_penis_radius(),.018f);
    close_value(body_chain_collider_cfg.penis_fine_offset[0][0],0);
    key(argv[1], "penis_radius", "0.04,0.05,0.06");
    body_penis_profile(argv[1],0);close_value(body_chain_penis_radius(),.018f);
    key(argv[1], "penis_radius", "5");
    body_penis_profile(argv[1],0);close_value(body_chain_penis_radius(),.25f);
    key(argv[1], NULL, NULL);
    body_penis_profile(argv[1],0);
    close_value(body_chain_penis_radius(),.018f);
    for(int j=0;j<3;j++) for(int a=0;a<3;a++) close_value(body_chain_collider_cfg.penis_fine_offset[j][a],0);
    puts("PASS: invalid input rejected, radius clamped, removed global keys reset on reload");
    return 0;
}
