#include "NC-TK17-PhysX.c"
#include <assert.h>

static void check_direction(const float *a, const float *b)
{
    for (int i = 0; i < 3; i++) {
        if (fabsf(a[i] - b[i]) > .0002f) {
            fprintf(stderr, "channel %d: got %g expected %g\n", i, a[i], b[i]);
            abort();
        }
    }
}

static void rotation_x(float a, float m[9])
{
    memset(m, 0, 9 * sizeof(float));
    m[0] = 1; m[4] = m[8] = cosf(a); m[5] = sinf(a); m[7] = -sinf(a);
}

static void frame_matrices(BYTE *bone, BYTE *trs, float pose, float placement,
                          float orbit, int capture, float expected[3])
{
    float local[9], placed[9], view[9], model[9], model_view[9], trs_view[9];
    rotation_x(pose, local); rotation_x(placement, placed); rotation_x(orbit, view);
    body_chain_mat3_multiply(local, placed, model);
    body_chain_mat3_multiply(model, view, model_view);
    body_chain_mat3_multiply(placed, view, trs_view);
    for (int i = 0; i < 3; i++) {
        memcpy(bone + 0x078 + i * 16, model_view + i * 3, 12);
        memcpy(trs + 0x078 + i * 16, trs_view + i * 3, 12);
        expected[i] = -model[i * 3 + 1];
    }
    if (capture) {
        memset(captured_camera_inverse, 0, sizeof(captured_camera_inverse));
        for (int r = 0; r < 3; r++) for (int c = 0; c < 3; c++)
            captured_camera_inverse[r * 4 + c] = view[c * 3 + r];
        captured_camera_inverse[15] = 1;
    }
    captured_camera_inverse_valid = 1;
    physx_simulation_serial++;
}

static void configure(void)
{
    defaults_cfg.debug = 0;
    physics_environment_cfg.world_gravity_probe = 1;
    physics_environment_cfg.gravity_apply_to_body_chain = 1;
    physics_environment_cfg.gravity_dynamic_body_basis = 1;
    physics_environment_cfg.body_chain_camera_relative_orientation = 1;
    physics_environment_cfg.gravity_basis_camera_compensate = 1;
    physics_environment_cfg.gravity_zero_at_start = 0;
    physics_environment_cfg.gravity_probe_camera_quiet_ms = 10;
    physics_environment_cfg.body_chain_camera_quarantine_ms = 0;
    physics_environment_cfg.gravity_horizontal_basis_offset = 0x078;
    physics_environment_cfg.gravity_vertical_basis_offset = 0x088;
    physics_environment_cfg.gravity_horizontal_secondary_basis_offset = 0x098;
    physics_environment_cfg.gravity_horizontal_basis_sign = 1;
    physics_environment_cfg.gravity_vertical_basis_sign = 1;
    physics_environment_cfg.gravity_horizontal_secondary_basis_sign = 1;
    physics_environment_cfg.world_gravity[0] = physics_environment_cfg.world_gravity[2] = 0;
    physics_environment_cfg.world_gravity[1] = -9.81f;
    strcpy(physics_environment_cfg.gravity_basis_node, "root");
}

static void body_and_spine(int hz, const char *system_name)
{
    BYTE bone[256] = {0}, trs[256] = {0}, replacement[256] = {0};
    body_chain_person_state_t state = {0};
    body_chain_gravity_snapshot_t room = {0};
    gravity_sample_t spine_sample = {0}, spine_reference = {0};
    body_chain_physics_config_t cfg = {0};
    float root[3] = {0, 0, 3}, expected[3], previous[3], output[3];
    DWORD tick = 10000, step = 1000 / hz;
    cfg.gravity_horizontal_curve = cfg.gravity_vertical_curve = 1;
    state.root_raw = bone; state.camera_relative_trs_raw = trs;
    state.gravity_probe_promoted = state.gravity_probe_captured = 1;
    state.init_tick = 1;
    captured_camera_change_tick = 0;
    for (int i = 0; i < 5; i++, tick += step) {
        frame_matrices(bone, trs, 0, .6f, 0, 1, expected);
        run_body_chain_gravity_probe("Person01", &state, bone, root, tick, &room, system_name);
        breasts_physics_spine_gravity_drive(bone, &cfg, &spine_sample, &spine_reference,
            "Person01", &state.camera_relative_trs_raw, tick, output);
    }
    assert(state.gravity_sample.accepted && spine_sample.accepted);
    check_direction(state.gravity_drive, expected); check_direction(output, expected);
    memcpy(previous, expected, sizeof(previous));
    for (int i = 1; i <= 120; i++, tick += step) {
        /* Deliberately leave the captured camera stale, as can happen across
           adjacent engine traversal points. Both raw matrices share the view. */
        captured_camera_version++; captured_camera_change_tick = tick;
        frame_matrices(bone, trs, i * .012f, .6f, i * .02f, 0, expected);
        run_body_chain_gravity_probe("Person01", &state, bone, root, tick, &room, system_name);
        assert(!state.gravity_camera_hold_active);
        check_direction(state.gravity_drive, previous);
        assert(breasts_physics_spine_gravity_drive(bone, &cfg, &spine_sample, &spine_reference,
            "Person01", &state.camera_relative_trs_raw, tick, output));
        check_direction(output, previous);
        memcpy(previous, expected, sizeof(previous));
    }
    for (int i = 1; i <= 60; i++, tick += step) {
        captured_camera_version++; captured_camera_change_tick = tick;
        frame_matrices(bone, trs, 1.44f, .6f, i * .03f, 0, expected);
        run_body_chain_gravity_probe("Person01", &state, bone, root, tick, &room, system_name);
        check_direction(state.gravity_drive, expected);
    }
    /* A pose replacement during orbit must not inherit the old TRS reference
       merely because the placement object was reused by the engine. */
    frame_matrices(replacement, trs, -.8f, -.3f, 1, 0, expected);
    run_body_chain_gravity_probe("Person01", &state, replacement, root, tick, &room, system_name);
    assert(!state.gravity_reference_sample.trusted_valid && !state.gravity_sample.trusted_valid);
    assert(state.gravity_camera_hold_active);
    tick += 200;
    for (int i = 0; i < 5; i++, tick += step) {
        frame_matrices(replacement, trs, -.8f, -.3f, 1, 1, expected);
        run_body_chain_gravity_probe("Person01", &state, replacement, root, tick, &room, system_name);
    }
    check_direction(state.gravity_drive, expected);
    reset_body_chain_gravity_state(&state);
    assert(!state.gravity_sample.trusted_valid && !state.gravity_reference_sample.trusted_valid);
    printf("PASS: %s and chest gravity follow motion during orbit at %d Hz; camera-only stability and replacement/reset confirmed\n", system_name, hz);
}

static void startup_and_fallback(void)
{
    BYTE bone[256] = {0}, trs[256] = {0};
    gravity_sample_t sample = {0}, reference = {0};
    void *cache = trs;
    float expected[3], out[3], zero[3] = {0}, world[3] = {0,-1,0}, view[3];
    DWORD tick = 40000;
    for (int i = 0; i < 10; i++, tick += 16) {
        captured_camera_version++; captured_camera_change_tick = tick;
        frame_matrices(bone, trs, 0, .5f, i * .1f, 1, expected);
        assert(camera_world_to_view_direction(world, view));
        assert(!body_gravity_sample_live("Person01", &cache, &sample, &reference,
            bone, view, expected, 1, tick, out));
        check_direction(out, zero);
    }
    tick += 100;
    for (int i = 0; i < 5; i++, tick += 16) {
        frame_matrices(bone, trs, 0, .5f, 0, 1, expected);
        assert(camera_world_to_view_direction(world, view));
        body_gravity_sample_live("Person01", &cache, &sample, &reference,
            bone, view, expected, 1, tick, out);
    }
    check_direction(out, expected);
    /* Unsupported/custom row offsets keep the original guarded path. */
    physics_environment_cfg.gravity_horizontal_basis_offset = 0x0c8;
    captured_camera_change_tick = tick; captured_camera_version++; physx_simulation_serial++;
    body_gravity_sample_live("Person01", &cache, &sample, &reference, bone, view, zero, 1, tick, out);
    assert(!sample.accepted && sample.reason == GRAVITY_SAMPLE_CAMERA);
    check_direction(out, expected);
    puts("PASS: startup during camera movement waits for a confirmed reference; custom basis fallback stays guarded");
}

static BYTE cached_bone[512], cached_trs[512], cached_source[512];
static float cached_origin[3];
static void *__cdecl find_cached(const char *name)
{
    if (strstr(name, ":STRS_group")) return cached_source+32;
    if (strstr(name, ":TRS_group")) return cached_trs+32;
    if (strstr(name, ":root")) return cached_bone+32;
    return NULL;
}
static void __cdecl pivot_cached(void *object, float *out)
{
    assert(object == cached_trs+32);
    memcpy(out, cached_origin, sizeof(cached_origin));
}

static void pose_restart_during_camera(void)
{
    body_chain_person_state_t state={0};
    body_chain_collider_person_state_t collider={0};
    body_chain_gravity_snapshot_t room={0};
    float expected[3], root[3]={0,0,3}, point[3], world[3];
    DWORD tick=50000;
    configure();
    body_chain_poseeditor_mode_active=1;
    physics_environment_cfg.gravity_probe_settle_ms=10000;
    physics_environment_cfg.gravity_probe_motion_epsilon=.005f;
    engine_FindObjC=find_cached;
    engine_GetModelViewRotationPivot=pivot_cached;
    captured_camera_change_tick=0;
    memset(body_placement_cache,0,sizeof(body_placement_cache));
    for(int frame=0;frame<8;frame++,tick+=16) {
        frame_matrices(cached_bone+32,cached_trs+32,0,.6f,0,1,expected);
        /* Moving position must not gate a confirmed direction, even cold. */
        root[0]+=.1f;
        run_body_chain_gravity_probe("Person01",&state,cached_bone+32,root,tick,&room,"test");
        body_collision_frame_update(&collider,"Person01",tick);
    }
    assert(state.gravity_probe_promoted && state.gravity_sample.trusted_valid);
    assert(collider.contact_frame_valid);
    assert(body_placement_cache[0][0].gravity.trusted_valid);
    assert(body_placement_cache[0][0].collision.trusted_valid);
    /* Simulate the complete solver/contact reset at a pose change. The
       calibrated placement survives, but bone samples must reconfirm. */
    memset(&state,0,sizeof(state));
    memset(&collider,0,sizeof(collider));
    state.root_raw=cached_bone+32;
    body_contact_pose_t pose={0};
    float zero_angles[3][3]={{0}};
    body_chain_physics_config_t cfg={0};
    pose.segments=3;
    for(int j=0;j<3;j++) {
        pose.length[j]=.1f;
        for(int a=0;a<3;a++) { cfg.link_min_angle[j][a]=-90; cfg.link_max_angle[j][a]=90; }
    }
    assert(body_dynamics_prepare(&pose,zero_angles,2,1,&state.dynamics));
    state.dynamics_valid=1;
    for(int frame=0;frame<90;frame++,tick+=16) {
        captured_camera_version++; captured_camera_change_tick=tick;
        frame_matrices(cached_bone+32,cached_trs+32,.8f,.6f,frame*.03f,0,expected);
        root[0]+=.1f; root[2]+=.03f;
        cached_origin[0]=frame*.2f; /* camera pan, captured inverse stale */
        run_body_chain_gravity_probe("Person01",&state,cached_bone+32,root,tick,&room,"test");
        body_collision_frame_update(&collider,"Person01",tick);
        if(!frame) assert(!state.gravity_probe_promoted);
        else {
            assert(state.gravity_probe_promoted && state.gravity_sample.trusted_valid);
            check_direction(state.gravity_drive,expected);
            float configured[3][3]={{0,10,10},{0,10,10},{0,10,10}}, target[3][3];
            memcpy(target,configured,sizeof(target));
            body_chain_shape_gravity(0,&state,&cfg,tick,.016f,configured,target);
            assert(state.dynamics_gravity_active && state.dynamics_gravity_valid);
            check_direction(state.dynamics_gravity_direction,expected);
            /* Production defaults negate the first two primary channels;
               geometry must still see the original unsigned body direction. */
            physics_environment_cfg.gravity_horizontal_basis_sign=-1;
            physics_environment_cfg.gravity_vertical_basis_sign=-1;
            state.gravity_sample.trusted[0]*=-1;
            state.gravity_sample.trusted[1]*=-1;
            float unsigned_direction[3];
            assert(body_chain_confirmed_geometry_direction(&state,unsigned_direction));
            check_direction(unsigned_direction,expected);
            physics_environment_cfg.gravity_horizontal_basis_sign=1;
            physics_environment_cfg.gravity_vertical_basis_sign=1;
            state.gravity_sample.trusted[0]*=-1;
            state.gravity_sample.trusted[1]*=-1;
        }
        assert(collider.contact_frame_valid);
        /* TRS origin maps back to the same room point despite camera pan. */
        memcpy(point,cached_origin,sizeof(point));
        assert(body_collision_view_to_world(&collider,point,world));
        for(int a=0;a<3;a++) assert(fabsf(world[a])<.0001f);
    }
    /* Actual room placement edits, replaced skeletons, generation and mode
       changes must not borrow a previous reference while the camera moves. */
    for(int change=0;change<7;change++) {
        body_placement_cache_t saved=body_placement_cache[0][0];
        if(change==0) cached_source[32+0x06c] ^= 1;
        if(change==1) named_node_generation++;
        if(change==2) body_placement_cache[0][0].root=cached_trs;
        if(change==3) body_chain_poseeditor_mode_active=0;
        if(change==4) physics_environment_cfg.world_gravity[0]=1;
        if(change==5) cached_source[32+0x060] ^= 1; /* Scale. */
        if(change==6) cached_source[32+0x11c] ^= 1; /* Last pivot property. */
        memset(&state,0,sizeof(state)); memset(&collider,0,sizeof(collider));
        for(int frame=0;frame<4;frame++,tick+=16) {
            captured_camera_version++; captured_camera_change_tick=tick;
            frame_matrices(cached_bone+32,cached_trs+32,.8f,.6f,frame*.03f,0,expected);
            root[0]+=.1f;
            run_body_chain_gravity_probe("Person01",&state,cached_bone+32,root,tick,&room,"test");
            body_collision_frame_update(&collider,"Person01",tick);
            assert(!state.gravity_sample.trusted_valid && !collider.contact_frame_valid);
        }
        if(change==0) cached_source[32+0x06c] ^= 1;
        if(change==1) named_node_generation--;
        if(change==5) cached_source[32+0x060] ^= 1;
        if(change==6) cached_source[32+0x11c] ^= 1;
        body_chain_poseeditor_mode_active=1;
        physics_environment_cfg.world_gravity[0]=0;
        body_placement_cache[0][0]=saved;
    }
    /* Translation-only pose placement changes preserve down, but must NOT
       restore old room contact coordinates. Both checks run during orbit. */
    *(float*)(cached_source+32+0x7c)=12.0f;
    memset(&state,0,sizeof(state)); memset(&collider,0,sizeof(collider));
    for(int frame=0;frame<4;frame++,tick+=16) {
        captured_camera_version++; captured_camera_change_tick=tick;
        frame_matrices(cached_bone+32,cached_trs+32,.8f,.6f,frame*.03f,0,expected);
        root[0]+=.1f;
        run_body_chain_gravity_probe("Person01",&state,cached_bone+32,root,tick,&room,"test");
        body_collision_frame_update(&collider,"Person01",tick);
        if(frame) {
            assert(state.gravity_probe_promoted);
            check_direction(state.gravity_drive,expected);
        }
        assert(!collider.contact_frame_valid);
    }
    engine_FindObjC=NULL; engine_GetModelViewRotationPivot=NULL;
    puts("PASS: geometric penis/testicle gravity runs on the first confirmed primary direction during orbit; translation-only pose changes retain direction while discarding old contact placement");
    puts("PASS: pose reset reacquires live gravity in two frames during stale-camera orbit/pan and restores camera-neutral contact placement; placement edits, skeleton, generation, mode and gravity changes reject reuse");
}

static void source_placement(float angle)
{
    float local[9];
    rotation_x(angle,local);
    *(void**)(cached_source+32)=body_placement_source_vtable;
    *(DWORD*)(cached_source+32+0x5c)=2;
    *(float*)(cached_source+32+0x6c)=angle*57.2957795f;
    for(int r=0;r<3;r++) memcpy(cached_source+32+0x18+r*16,local+r*3,12);
}

static void rotated_pose_after_customizer(void)
{
    static int fake_vtable;
    body_chain_person_state_t state={0};
    body_chain_gravity_snapshot_t room={0};
    float expected[3], root[3]={0,0,3};
    DWORD tick=80000;
    configure();
    memset(cached_source,0,sizeof(cached_source));
    memset(body_placement_cache,0,sizeof(body_placement_cache));
    body_placement_source_vtable=&fake_vtable;
    engine_FindObjC=find_cached;
    body_chain_poseeditor_mode_active=1;
    physics_environment_cfg.gravity_probe_settle_ms=10000;
    captured_camera_change_tick=0;
    source_placement(.6f);
    for(int frame=0;frame<6;frame++,tick+=16) {
        frame_matrices(cached_bone+32,cached_trs+32,.2f,.6f,0,1,expected);
        run_body_chain_gravity_probe("Person01",&state,cached_bone+32,root,tick,&room,"test");
    }
    assert(state.gravity_probe_promoted);
    /* Customizer changes local placement on the same live skeleton. Its
       runtime calibration must not overwrite the retained editor record. */
    body_chain_poseeditor_mode_active=0;
    physx_customizer_active=1;
    memset(&state,0,sizeof(state));
    source_placement(-.4f);
    for(int frame=0;frame<6;frame++,tick+=16) {
        frame_matrices(cached_bone+32,cached_trs+32,0,-.4f,0,1,expected);
        run_body_chain_gravity_probe("Person01",&state,cached_bone+32,root,tick,&room,"test");
    }
    body_chain_poseeditor_mode_active=1;
    physx_customizer_active=0;
    /* New, differently tilted poses during uninterrupted camera orbit. */
    for(int pose=0;pose<3;pose++) {
        memset(&state,0,sizeof(state));
        float tilt=1.1f-pose*.3f;
        source_placement(tilt);
        *(DWORD*)(cached_source+32+0x5c)=0;
        assert(!body_placement_cache_for("Person01",cached_trs+32));
        assert(body_placement_cache[0][0].gravity.trusted_valid);
        *(DWORD*)(cached_source+32+0x5c)=2;
        for(int frame=0;frame<8;frame++,tick+=16) {
            captured_camera_version++; captured_camera_change_tick=tick;
            frame_matrices(cached_bone+32,cached_trs+32,.3f,tilt,frame*.09f,0,expected);
            root[0]+=.1f;
            run_body_chain_gravity_probe("Person01",&state,cached_bone+32,root,tick,&room,"test");
            if(frame) {
                assert(state.gravity_probe_promoted);
                check_direction(state.gravity_drive,expected);
            }
        }
    }
    /* A nonuniform local scale cannot use the rigid-direction shortcut. */
    float basis[9];
    *(float*)(cached_source+32+0x18)=2.f;
    assert(!body_placement_local_basis(cached_source+32,basis));
    body_placement_source_vtable=NULL;
    engine_FindObjC=NULL;
    puts("PASS: Customizer round trip and differently rotated pose placements start in two fresh samples during stale-camera orbit; dirty/nonuniform local matrices remain guarded");
}

static void compound_placement_rotation(void)
{
    body_placement_cache_t cache={0};
    float x[9], z[9]={0}, y[9]={0}, next[9], down[3]={-.2f,-.9f,.3f}, expected[3];
    float length=physx_vec3_len(down);
    for(int a=0;a<3;a++) down[a]/=length;
    rotation_x(.3f,x);
    z[0]=z[4]=cosf(.4f); z[1]=sinf(.4f); z[3]=-sinf(.4f); z[8]=1;
    y[0]=y[8]=cosf(.9f); y[2]=-sinf(.9f); y[6]=sinf(.9f); y[4]=1;
    body_chain_mat3_multiply(x,z,cache.local_basis);
    rotation_x(-.7f,x);
    body_chain_mat3_multiply(y,x,next);
    cache.local_basis_valid=cache.gravity.trusted_valid=1;
    for(int a=0;a<3;a++) {
        cache.gravity.trusted[a]=vec3_dot(cache.local_basis+a*3,down);
        expected[a]=vec3_dot(next+a*3,down);
    }
    assert(body_placement_rebase_gravity(&cache,next));
    check_direction(cache.gravity.trusted,expected);
    puts("PASS: compound placement rotations preserve correct gravity axes with a tilted parent reference");
}

int main(void)
{
    configure();
    const char *systems[] = {"penis_physics", "testicle_physics", "breasts_physics", "butt_physics"};
    for (int i = 0; i < 4; i++) {
        body_and_spine(30, systems[i]);
        body_and_spine(144, systems[i]);
    }
    startup_and_fallback();
    pose_restart_during_camera();
    rotated_pose_after_customizer();
    compound_placement_rotation();
    return 0;
}
