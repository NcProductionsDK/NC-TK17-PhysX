#include "NC-TK17-PhysX.c"
#include <assert.h>
#undef assert
#define assert(ok) do { if (!(ok)) { fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#ok); exit(1); } } while(0)

/* Person02's face-up geometry from the 2026-10-04 diagnostics. Freeze the
   rendered bones so only the simulation ownership toggle changes. */
static const float penis_points[4][3] = {
    {0,0,0}, {.00723f,.00344f,-.07607f},
    {-.04446f,.00715f,-.14831f}, {-.08766f,.00687f,-.17213f}
};
static const float testicle_points[3][3] = {
    {-.02000f,0,-.03700f}, {-.02804f,-.00438f,-.08774f},
    {-.06178f,-.01033f,-.13410f}
};

static void setup(int enabled, int pose)
{
    body_chain_collider_person_state_t *collider;
    body_chain_person_state_t *other;
    body_profile_set_active_person_config(-1);
    memset(body_chain_collider_states,0,sizeof(body_chain_collider_states));
    memset(body_chain_collider_projection_cache,0,sizeof(body_chain_collider_projection_cache));
    memset(testicle_physics_states,0,sizeof(testicle_physics_states));
    memset(runtime_testicle_physics_states,0,sizeof(runtime_testicle_physics_states));
    physx_simulation_serial++;
    body_chain_poseeditor_mode_active=pose;
    defaults_cfg.debug=0;
    body_chain_collider_cfg.diagnostic=0;
    body_chain_collider_cfg.enabled=1;
    body_chain_collider_cfg.root_local_offsets=1;
    body_chain_collider_cfg.penis_collision_enabled=1;
    body_chain_collider_cfg.testicle_collision_enabled=1;
    body_chain_collider_cfg.response_strength=2;
    body_chain_collider_cfg.response_radius_scale=.75f;
    body_chain_collider_cfg.response_max_degrees_per_tick=45;
    body_chain_collider_cfg.collision_iterations=2;
    body_chain_collider_cfg.collision_slop=.001f;
    body_chain_collider_cfg.chain_radius=.0275f;
    body_chain_collider_cfg.penis_radius=.0275f;
    memset(body_chain_collider_cfg.penis_fine_offset,0,sizeof(body_chain_collider_cfg.penis_fine_offset));
    for(int n=BODY_COLLIDER_TESTICLES_01;n<=BODY_COLLIDER_TESTICLES_MID;n++)
        for(int a=0;a<3;a++) body_chain_collider_cfg.node_radius[n][a]=a==1?.042f:.045f;
    body_chain_physics_cfg.collision_scope=BODY_CHAIN_COLLISION_SCOPE_GENITALS_ONLY;
    body_chain_physics_cfg.collision_strength=1;
    body_chain_physics_cfg.horizontal_output_axis=2;
    body_chain_physics_cfg.vertical_output_axis=1;
    body_chain_physics_cfg.room_collision_enabled=0;
    for(int j=0;j<3;j++) for(int a=0;a<3;a++) {
        body_chain_physics_cfg.link_min_angle[j][a]=-180;
        body_chain_physics_cfg.link_max_angle[j][a]=180;
    }
    testicle_physics_cfg.enabled=1;
    testicle_physics_cfg.enabled_person[1]=enabled;
    for(int p=0;p<4;p++) {
        body_chain_collider_person_cfg[p]=body_chain_collider_cfg;
        body_chain_physics_person_cfg[p]=body_chain_physics_cfg;
        testicle_physics_person_cfg[p]=testicle_physics_cfg;
    }
    collider=&body_chain_collider_states[1];
    collider->ready=collider->basis_valid=1;
    collider->active_scope_mask=BODY_CHAIN_COLLIDER_GROUP_GENITALS;
    collider->valid[BODY_COLLIDER_ROOT]=1;
    collider->testicle_points_ready=1;
    collider->testicle_points_update_tick=1000;
    memcpy(collider->testicle_joint_position,testicle_points,sizeof(testicle_points));
    for(int j=0;j<2;j++) {
        int node=BODY_COLLIDER_TESTICLES_01+j;
        collider->valid[node]=1;
        for(int a=0;a<3;a++)
            collider->local_position[node][a]=(testicle_points[j][a]+testicle_points[j+1][a])*.5f;
    }
    collider->local_position[BODY_COLLIDER_TESTICLES_02][0]-=.015f;
    other=pose?&testicle_physics_states[1]:&runtime_testicle_physics_states[1];
    other->initialized=other->active_logged=enabled;
    body_profile_set_active_person_config(1);
}

static int project_points(const float points[4][3],float correction[3][2], float *penetration)
{
    body_chain_person_state_t state={0};
    state.collision_step_valid=state.collision_step_engine_points=1;
    state.collision_step_tick=1000;
    state.collision_step_dt=.016f;
    memcpy(state.collision_step_points,points,sizeof(state.collision_step_points));
    /* Use the legacy geometric predictor here: these rounded log pivots do
       not include the exact Euler sample needed by the stricter pose fit. */
    memset(correction,0,sizeof(float)*6);
    body_chain_compute_collider_projection(1,&state,correction,1000,0,
        penetration,NULL,NULL,BODY_CHAIN_COLLISION_TARGET_PENIS);
    return state.collision_manifold_contacts;
}

static int project(float correction[3][2],float *penetration)
{
    return project_points(penis_points,correction,penetration);
}

static const float straight_shaft[4][3] = {
    {0,0,0},{0,0,-.1f},{0,0,-.2f},{0,0,-.3f}
};

static void native_geometry(float first_z)
{
    body_chain_collider_person_state_t *collider=&body_chain_collider_states[1];
    for(int j=0;j<3;j++) {
        collider->testicle_joint_position[j][0]=.045f;
        collider->testicle_joint_position[j][1]=0;
        collider->testicle_joint_position[j][2]=first_z-.08f*j;
    }
    for(int j=0;j<2;j++) for(int a=0;a<3;a++)
        collider->local_position[BODY_COLLIDER_TESTICLES_01+j][a]=
            (collider->testicle_joint_position[j][a]+collider->testicle_joint_position[j+1][a])*.5f;
}

static void check_native_distal(int pose)
{
    float correction[3][2],depth;
    setup(1,pose);
    native_geometry(-.08f);
    assert(project_points(straight_shaft,correction,&depth)==0);
    /* Check an actual both-enabled contact too: its original narrow radius
       must stay unchanged, not only its no-contact frozen baseline. */
    for(int j=0;j<3;j++) body_chain_collider_states[1].testicle_joint_position[j][0]=.015f;
    assert(project_points(straight_shaft,correction,&depth)>0);
    assert(depth>.004f && depth<.006f);
    setup(0,pose);
    native_geometry(-.08f);
    assert(project_points(straight_shaft,correction,&depth)>0);
    assert(depth>.015f && depth<.017f);
    /* Global off behaves like per-person off. No native/solver ownership is
       acquired, even if a prior simulated state is still present. */
    testicle_physics_cfg.enabled=0;
    testicle_physics_cfg.enabled_person[1]=1;
    assert(project_points(straight_shaft,correction,&depth)>0);
    testicle_physics_cfg.enabled=1;
    assert(project_points(straight_shaft,correction,&depth)==0);
    testicle_physics_cfg.enabled_person[1]=0;
    /* Configured radius and center offsets are honored by the distal shell. */
    for(int a=0;a<3;a++) body_chain_collider_cfg.node_radius[BODY_COLLIDER_TESTICLES_MID][a]=.01f;
    assert(project_points(straight_shaft,correction,&depth)==0);
    for(int a=0;a<3;a++) body_chain_collider_cfg.node_radius[BODY_COLLIDER_TESTICLES_MID][a]=.045f;
    for(int j=0;j<2;j++) body_chain_collider_states[1].local_position[BODY_COLLIDER_TESTICLES_01+j][0]+=.1f;
    assert(project_points(straight_shaft,correction,&depth)==0);
    native_geometry(-.08f);
    body_chain_collider_cfg.testicle_collision_enabled=0;
    body_chain_clear_testicle_collider_state(&body_chain_collider_states[1]);
    assert(project_points(straight_shaft,correction,&depth)==0);
    /* A broad shell overlapping only the base must not be introduced. */
    for(int enabled=0;enabled<2;enabled++) {
        setup(enabled,pose);
        native_geometry(.10f);
        assert(project_points(straight_shaft,correction,&depth)==0);
    }
    /* Exercise repeated production projection against a fixed native capsule.
       The added contact must actually separate the shaft, not just log hits. */
    setup(0,pose);
    native_geometry(-.08f);
    body_chain_person_state_t state={0};
    state.collision_step_valid=state.collision_step_engine_points=1;
    state.collision_step_tick=1000;
    state.collision_step_dt=.016f;
    memcpy(state.collision_step_points,straight_shaft,sizeof(straight_shaft));
    float initial_depth=0;
    for(int step=0;step<120;step++) {
        memset(correction,0,sizeof(correction));
        body_chain_compute_collider_projection(1,&state,correction,1000,0,
            &depth,NULL,NULL,BODY_CHAIN_COLLISION_TARGET_PENIS);
        if(!step) initial_depth=depth;
        body_contact_apply(&state,&body_chain_physics_cfg,correction,3,depth);
    }
    printf("native distal separation mode=%d penetration=%.6f -> %.6f\n",pose,initial_depth,depth);
    assert(initial_depth>.015f);
    assert(depth<.002f);
    for(int j=0;j<3;j++) for(int a=0;a<3;a++) assert(state.velocity[j][a]==0);
}

static void close_points(const float a[4][3],const float b[4][3])
{
    for(int j=0;j<4;j++) for(int axis=0;axis<3;axis++)
        assert(fabsf(a[j][axis]-b[j][axis])<.000001f);
}

static void check_sampling(int pose)
{
    float live[4][3],sampled[4][3],candidate[4][3];
    body_chain_person_state_t snapshot;
    body_chain_person_state_t *other;
    setup(1,pose);
    other=pose?&testicle_physics_states[1]:&runtime_testicle_physics_states[1];
    assert(body_chain_testicle_cross_points_local(1,live,1000));
    other->active_logged=0;
    assert(!body_chain_testicle_cross_points_local(1,sampled,1000));
    other->active_logged=1;
    other->initialized=0;
    assert(!body_chain_testicle_cross_points_local(1,sampled,1000));
    other->initialized=1;
    other->collision_step_valid=1;
    other->collision_step_tick=1000;
    memcpy(other->collision_step_points,live,sizeof(live));
    for(int j=0;j<4;j++) other->collision_step_points[j][0]+=.02f;
    assert(body_chain_testicle_cross_points_local(1,candidate,1000));
    assert(fabsf(candidate[0][0]-live[0][0]-.02f)<.000001f);
    /* Releasing ownership must ignore even a same-tick initialized candidate,
       without clearing or otherwise writing either native or solver state. */
    snapshot=*other;
    testicle_physics_cfg.enabled_person[1]=0;
    assert(body_chain_testicle_cross_points_local(1,sampled,1000));
    close_points(sampled,live);
    assert(!memcmp(other,&snapshot,sizeof(snapshot)));
    testicle_physics_cfg.enabled_person[1]=1;
    testicle_physics_cfg.enabled=0;
    assert(body_chain_testicle_cross_points_local(1,sampled,1000));
    close_points(sampled,live);
    assert(!memcmp(other,&snapshot,sizeof(snapshot)));
    body_chain_collider_cfg.testicle_collision_enabled=0;
    assert(!body_chain_testicle_cross_points_local(1,sampled,1000));
    body_chain_collider_cfg.testicle_collision_enabled=1;
    assert(!body_chain_testicle_cross_points_local(1,sampled,
        1001+BODY_CHAIN_ENGINE_POINT_STALE_MS));
    body_chain_collider_states[1].testicle_points_ready=0;
    assert(!body_chain_testicle_cross_points_local(1,sampled,1000));
}

int main(void)
{
    for(int pose=0;pose<2;pose++) {
        float on[3][2],off[3][2],on_depth,off_depth;
        int on_contacts,off_contacts;
        setup(1,pose);
        on_contacts=project(on,&on_depth);
        /* The original both-enabled result is the baseline, not merely
           equality between two potentially changed collision responses. */
        assert(on_contacts==0);
        assert(on_depth==0);
        for(int j=0;j<3;j++) for(int a=0;a<2;a++) assert(on[j][a]==0);
        setup(0,pose);
        off_contacts=project(off,&off_depth);
        printf("mode=%s enabled=(contacts:%d depth:%.6f root:%.4f,%.4f) disabled=(contacts:%d depth:%.6f root:%.4f,%.4f)\n",
            pose?"PoseEditor":"runtime",on_contacts,on_depth,on[0][0],on[0][1],
            off_contacts,off_depth,off[0][0],off[0][1]);
        fflush(stdout);
        assert(off_contacts>0);
        assert(off_depth>0);
        /* A real overlap remains collidable with simulation disabled. */
        for(int enabled=0;enabled<2;enabled++) {
            setup(enabled,pose);
            body_chain_collider_person_state_t *collider=&body_chain_collider_states[1];
            for(int j=0;j<3;j++) {
                memcpy(collider->testicle_joint_position[j],penis_points[j+1],sizeof(float)*3);
                collider->testicle_joint_position[j][0]+=.005f;
            }
            int count=project(enabled?on:off,enabled?&on_depth:&off_depth);
            assert(count>0);
        }
        assert(on_depth>0);
        assert(off_depth>0);
        check_sampling(pose);
        check_native_distal(pose);
    }
    puts("PASS: original both-enabled baseline preserved; native shaft/tip contact uses configured volume; first-segment attachment rules unchanged");
    puts("PASS: real overlaps still collide; native samples ignore released solver state; stale samples and collision disable are respected");
    return 0;
}
