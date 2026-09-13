
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "../physx_body_dynamics.h"
#include "../physx_body_motion.h"
#include "../physx_gravity_sample.h"
static body_contact_pose_t rod(int segments) {
    body_contact_pose_t pose={0};pose.segments=segments;
    for(int j=0;j<segments;j++)pose.length[j]=.1f;
    return pose;
}
static void analytic_rod(void) {
    body_contact_pose_t pose=rod(1);float e[3][3]={{0}},inertia[3][2],lever[3][2][3];int axes[2]={2,1};
    float rad=.01745329252f,expected=.1f*.1f*rad*rad/3;
    assert(body_dynamics_geometry(&pose,e,axes,inertia,lever));
    assert(fabsf(inertia[0][0]/expected-1)<.0001f && fabsf(inertia[0][1]/expected-1)<.0001f);
    assert(fabsf(lever[0][1][1]+.05f*rad)<1e-7f);
    assert(fabsf(lever[0][0][0]+.05f*rad)<1e-7f);
    puts("PASS: one-rod inertia agrees with analytic L^2/3; gravity leverage agrees with L/2 in both bend axes");
}
static void gravity_orientation(void) {
    body_contact_pose_t pose=rod(1);body_dynamics_t model={0};
    float e[3][3]={{0}},configured[3][3]={{0,10,0}},out[3][3],g[3]={0,-1,0};
    assert(body_dynamics_prepare(&pose,e,2,1,&model));
    assert(body_dynamics_gravity(&pose,&model,e,g,configured,out));float hanging_drive=out[0][1]-5;
    assert(hanging_drive>3);
    e[0][1]=90;assert(body_dynamics_gravity(&pose,&model,e,g,configured,out));
    assert(fabsf(out[0][1]-5)<.0001f); /* Lever parallel to force: geometric torque vanishes. */
    e[0][1]=180;assert(body_dynamics_gravity(&pose,&model,e,g,configured,out));
    assert(fabsf((out[0][1]-5)+hanging_drive)<.001f);
    e[0][1]=0;g[1]=0;assert(body_dynamics_gravity(&pose,&model,e,g,configured,out));assert(out[0][1]==5);
    g[1]=.00001f;assert(body_dynamics_gravity(&pose,&model,e,g,configured,out));assert(fabsf(out[0][1]-5)<.0001f);
    memset(configured,0,sizeof(configured));g[1]=-1;
    assert(body_dynamics_gravity(&pose,&model,e,g,configured,out));
    for(int j=0;j<3;j++)for(int a=0;a<3;a++)assert(out[j][a]==0);
    puts("PASS: geometric gravity vanishes when hanging, reverses with leverage, crosses zero continuously and respects zero drive");
}
static void scaling_and_lag(void) {
    for(int n=2;n<=3;n++) {
        body_contact_pose_t pose=rod(n);body_dynamics_t a,b;float e[3][3]={{0}};
        assert(body_dynamics_prepare(&pose,e,2,1,&a));
        for(int j=0;j<n;j++)pose.length[j]*=2;
        assert(body_dynamics_prepare(&pose,e,2,1,&b));
        for(int j=0;j<n;j++)for(int k=0;k<2;k++) {
            assert(fabsf(a.inverse_inertia[j][k]-b.inverse_inertia[j][k])<.0001f);
            assert(a.inverse_inertia[j][k]>0 && a.inverse_inertia[j][k]<=1/.65f);
        }
        assert(a.inverse_inertia[0][1]<a.inverse_inertia[n-1][1]);
        float root=0,root_v=0,tip=0,tip_v=0;
        float ri=a.inverse_inertia[0][1],ti=a.inverse_inertia[n-1][1];
        body_motion_limited_spring_step(&root,&root_v,20,100*ri,8*sqrtf(ri),.016f,-90,90);
        body_motion_limited_spring_step(&tip,&tip_v,20,100*ti,8*sqrtf(ti),.016f,-90,90);
        assert(root>0 && tip>root && tip_v>root_v);
        printf("LINK_INERTIA segments=%d root=%g distal=%g first_motion=%g/%g\n",n,1/ri,1/ti,root,tip);
    }
    puts("PASS: measured downstream mass/leverage gives proximal lag; uniform rig scaling preserves relative inertia");
}
static void free_rest(void) {
    const float rates[]={20,30,60,144};
    for(int n=2;n<=3;n++)for(int r=0;r<4;r++) {
        body_contact_pose_t pose=rod(n);body_dynamics_t model;
        float e[3][3]={{0}},velocity[3][3]={{0}},configured[3][3]={{0}},g[3]={0,-1,0};
        float dt=(1/rates[r])/body_motion_substeps(1/rates[r]);
        assert(body_dynamics_prepare(&pose,e,2,1,&model));
        for(int j=0;j<n;j++)configured[j][1]=20;
        for(int t=0;t<2200;t++) {
            float shaped[3][3];
            assert(body_dynamics_gravity(&pose,&model,e,g,configured,shaped));
            for(int j=0;j<n;j++) for(int k=0;k<2;k++) {
                int axis=model.axis[k];float inv=model.inverse_inertia[j][k];
                body_motion_limited_spring_step(&e[j][axis],&velocity[j][axis],shaped[j][axis],
                    100*inv,8*sqrtf(inv),dt,-80,80);
                assert(isfinite(e[j][axis]) && isfinite(velocity[j][axis]));
                if(t>2100) assert(fabsf(velocity[j][axis])<.001f);
            }
        }
        printf("GRAVITY_REST segments=%d hz=%.0f root=%g distal=%g\n",n,rates[r],e[0][1],e[n-1][1]);
    }
    puts("PASS: gravity and inertia together settle in free motion for both chains at 20/30/60/144 Hz");
}
static void invalid_geometry(void) {
    body_contact_pose_t pose=rod(3);body_dynamics_t model={0},saved;
    float e[3][3]={{0}};
    assert(body_dynamics_prepare(&pose,e,2,1,&model));saved=model;
    pose.length[1]=0;assert(!body_dynamics_prepare(&pose,e,2,1,&model));assert(!memcmp(&model,&saved,sizeof(model)));
    pose.length[1]=NAN;assert(!body_dynamics_prepare(&pose,e,2,1,&model));
    pose.length[1]=.1f;assert(!body_dynamics_prepare(&pose,e,1,1,&model));
    puts("PASS: invalid lengths/axes reject the model without damaging a previous valid reference");
}

typedef unsigned long DWORD;
typedef struct {float gravity_horizontal_strength,gravity_vertical_strength;} body_chain_physics_config_t;
typedef struct {
    body_dynamics_t dynamics;int dynamics_valid,dynamics_gravity_valid,dynamics_gravity_active,gravity_probe_promoted,gravity_camera_hold_active;
    float dynamics_gravity_direction[3],angle[3][3];
    gravity_sample_t geometry_sample;
    void *root_raw;
} body_chain_person_state_t;
static struct {int gravity_apply_to_body_chain;float world_gravity[3],gravity_response_ms;int gravity_horizontal_tail_axis,gravity_vertical_tail_axis;} physics_environment_cfg={1,{0,-1,0},100,2,1};
static struct {int debug;} defaults_cfg;
static void log_line(const char *fmt,...){(void)fmt;}
static float physx_clampf(float v,float a,float b){return fmaxf(a,fminf(b,v));}
typedef struct {int ready,basis_valid;} test_collider;
static test_collider body_chain_collider_states[4]={{1,1}};
static int camera_hold,world_queries;
static int gravity_sample_live(gravity_sample_t *s,const void *src,const float v[3],int valid,DWORD now,float out[3]){
    return gravity_sample_update(s,(uintptr_t)src,v,valid,(uint32_t)(now/16),now,0,1,1000,10,out);
}
static int body_chain_camera_pivot_hold_active(DWORD now){(void)now;return camera_hold;}
static int room_collision_world_vector_to_body_local(const test_collider *state,const float *world,float *out){(void)state;world_queries++;memcpy(out,world,sizeof(float)*3);return 1;}
static float physx_vec3_len(const float *v){return sqrtf(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);}
static float body_chain_clamp_link_axis_angle(const body_chain_physics_config_t *cfg,int j,int a,float value){(void)cfg;(void)j;(void)a;return fminf(80,fmaxf(-80,value));}
static int body_chain_limit_total_rotation(const body_chain_physics_config_t *cfg,float v[3][3],float velocity[3][3],int count){(void)cfg;(void)v;(void)velocity;(void)count;return 0;}
static void body_chain_apply_link_inertia(const body_chain_person_state_t *state,
    int joint,int axis,float *stiffness,float *damping)
{
    int a;
    if(!state->dynamics_valid || joint>=state->dynamics.segments) return;
    for(a=0;a<2;a++) if(axis==state->dynamics.axis[a]) {
        float inverse=state->dynamics.inverse_inertia[joint][a];
        *stiffness*=inverse;
        /* Keep the user's damping ratio while changing the natural period. */
        *damping*=sqrtf(inverse);
    }
}
static void body_chain_shape_gravity(int person,body_chain_person_state_t *state,
    const body_chain_physics_config_t *cfg,DWORD now,float dt,
    const float configured[3][3],float target[3][3])
{
    float euler[3][3],shaped[3][3];int j,a;
    state->dynamics_gravity_active=0;
    if(!state->dynamics_valid || !physics_environment_cfg.gravity_apply_to_body_chain ||
       !state->gravity_probe_promoted) return;
    if(!body_chain_collider_states[person].ready ||
       !body_chain_collider_states[person].basis_valid) return;
    /* Geometric gravity uses the same sample protocol as the primary drive.
       Gather while held too, so both can confirm promptly after the camera stops. */
    {
        float candidate[3], trusted[3];
        int valid = !body_chain_camera_pivot_hold_active(now) &&
            room_collision_world_vector_to_body_local(&body_chain_collider_states[person],
                physics_environment_cfg.world_gravity, candidate);
        if (valid) {
            float length = physx_vec3_len(candidate);
            valid = length > .00001f && isfinite(length);
            if (valid) for (a=0;a<3;a++) candidate[a] /= length;
        }
        if (gravity_sample_live(&state->geometry_sample,state->root_raw,
                candidate,valid,now,trusted) && state->geometry_sample.accepted &&
            !state->gravity_camera_hold_active) {
            float response=physics_environment_cfg.gravity_response_ms*.001f;
            float alpha=response>0?1.0f-expf(-dt/response):1.0f;
            for(a=0;a<3;a++)
                state->dynamics_gravity_direction[a]=state->dynamics_gravity_valid?
                    state->dynamics_gravity_direction[a]+alpha*(trusted[a]-state->dynamics_gravity_direction[a]):trusted[a];
            state->dynamics_gravity_valid=1;
        }
    }
    if(!state->dynamics_gravity_valid) return;
    for(j=0;j<3;j++) for(a=0;a<3;a++)
        euler[j][a]=state->dynamics.reference.euler[j][a]+state->angle[j][a];
    if(!body_dynamics_gravity(&state->dynamics.reference,&state->dynamics,euler,
        state->dynamics_gravity_direction,configured,shaped)) return;
    for(j=0;j<state->dynamics.segments;j++) for(a=0;a<3;a++)
        target[j][a]=body_chain_clamp_link_axis_angle(cfg,j,a,
            target[j][a]+shaped[j][a]-configured[j][a]);
    body_chain_limit_total_rotation(cfg,target,NULL,state->dynamics.segments);
    state->dynamics_gravity_active=1;
}
static void body_chain_shape_penis_gravity(int person,body_chain_person_state_t *state,
    const body_chain_physics_config_t *cfg,DWORD now,float dt,
    const float configured[3][3],float target[3][3])
{
    float gravity[3][3];int j,a;
    static DWORD last_log[4];
    if (cfg->gravity_horizontal_strength == 1.0f && cfg->gravity_vertical_strength == 1.0f) {
        body_chain_shape_gravity(person,state,cfg,now,dt,configured,target);
        return;
    }
    memcpy(gravity,configured,sizeof(gravity));
    body_chain_shape_gravity(person,state,cfg,now,dt,configured,gravity);
    for (j=0;j<3;j++) for (a=0;a<3;a++) {
        float strength=1.0f;
        if (a==physics_environment_cfg.gravity_horizontal_tail_axis)
            strength*=cfg->gravity_horizontal_strength;
        if (a==physics_environment_cfg.gravity_vertical_tail_axis)
            strength*=cfg->gravity_vertical_strength;
        target[j][a]=body_chain_clamp_link_axis_angle(cfg,j,a,
            target[j][a]-configured[j][a]+gravity[j][a]*strength);
    }
    body_chain_limit_total_rotation(cfg,target,NULL,3);
    if (defaults_cfg.debug && (!last_log[person] || now-last_log[person]>=1000)) {
        last_log[person]=now;
        log_line("body-chain gravity-strength person=Person%02d strength=(h=%.3f,v=%.3f) geometry=%d gravity01=(%.4f,%.4f,%.4f) target01=(%.4f,%.4f,%.4f)",
            person+1,cfg->gravity_horizontal_strength,cfg->gravity_vertical_strength,
            state->dynamics_gravity_active,gravity[0][0],gravity[0][1],gravity[0][2],
            target[0][0],target[0][1],target[0][2]);
    }
}
static float profile_float(const char *section, const char *key, float fallback, const char *path)
{
    char buf[128];
    GetPrivateProfileStringA(section, key, "", buf, sizeof(buf), path);
    if (!buf[0]) return fallback;
    return (float)atof(buf);
}
static body_chain_physics_config_t body_chain_physics_cfg;
#define PENIS_PHYSICS_CONFIG_SECTION "penis_physics"
static void read_global(const char *config_path){body_chain_physics_cfg.gravity_horizontal_strength = physx_clampf(
        profile_float(PENIS_PHYSICS_CONFIG_SECTION, "gravity_horizontal_strength", 1.0f, config_path), 0.0f, 4.0f);body_chain_physics_cfg.gravity_vertical_strength = physx_clampf(
        profile_float(PENIS_PHYSICS_CONFIG_SECTION, "gravity_vertical_strength", 1.0f, config_path), 0.0f, 4.0f);}
static void read_sidecar(const char *path){body_chain_physics_cfg.gravity_horizontal_strength = physx_clampf(
        profile_float(PENIS_PHYSICS_CONFIG_SECTION, "gravity_horizontal_strength",
                      body_chain_physics_cfg.gravity_horizontal_strength, path), 0.0f, 4.0f);body_chain_physics_cfg.gravity_vertical_strength = physx_clampf(
        profile_float(PENIS_PHYSICS_CONFIG_SECTION, "gravity_vertical_strength",
                      body_chain_physics_cfg.gravity_vertical_strength, path), 0.0f, 4.0f);}

static void production_wiring(void) {
    body_chain_person_state_t s={0};body_chain_physics_config_t cfg={0};body_contact_pose_t pose=rod(3);
    float zero[3][3]={{0}},configured[3][3]={{0,10,0},{0,10,0},{0,10,0}},target[3][3],baseline[3][3];
    assert(body_dynamics_prepare(&pose,zero,2,1,&s.dynamics));s.dynamics_valid=s.gravity_probe_promoted=1;
    memcpy(target,configured,sizeof(target));
    body_chain_shape_gravity(0,&s,&cfg,0,.016f,configured,target);
    assert(!s.dynamics_gravity_active && world_queries==1);
    body_chain_shape_gravity(0,&s,&cfg,16,.016f,configured,target);
    assert(s.dynamics_gravity_active && world_queries==2);memcpy(baseline,target,sizeof(target));
    memcpy(target,configured,sizeof(target));target[0][1]+=5;
    body_chain_shape_gravity(0,&s,&cfg,32,.016f,configured,target);
    assert(fabsf(target[0][1]-baseline[0][1]-5)<1e-5f); /* Motion/wind term retained. */
    camera_hold=1;memcpy(target,configured,sizeof(target));physics_environment_cfg.world_gravity[1]=1;
    body_chain_shape_gravity(0,&s,&cfg,48,.016f,configured,target);
    assert(world_queries==3 && s.dynamics_gravity_active);
    for(int j=0;j<3;j++)for(int a=0;a<3;a++)assert(target[j][a]==baseline[j][a]);
    physics_environment_cfg.gravity_apply_to_body_chain=0;memcpy(target,configured,sizeof(target));
    body_chain_shape_gravity(0,&s,&cfg,64,.016f,configured,target);
    assert(!s.dynamics_gravity_active && !memcmp(target,configured,sizeof(target)));
    physics_environment_cfg.gravity_apply_to_body_chain=1;body_chain_collider_states[0].basis_valid=0;
    body_chain_shape_gravity(0,&s,&cfg,80,.016f,configured,target);
    assert(s.dynamics_gravity_active && world_queries==3 && !memcmp(target,baseline,sizeof(target)));
    /* Production's global pivot hold is disabled. The per-body gravity hold
       must still protect the geometric contribution from a changed camera basis. */
    body_chain_collider_states[0].basis_valid=1;camera_hold=0;s.gravity_camera_hold_active=1;
    memcpy(target,configured,sizeof(target));
    body_chain_shape_gravity(0,&s,&cfg,96,.016f,configured,target);
    assert(world_queries==4 && s.dynamics_gravity_active && !s.geometry_sample.accepted);
    for(int j=0;j<3;j++)for(int a=0;a<3;a++)assert(target[j][a]==baseline[j][a]);
    memcpy(target,configured,sizeof(target));
    body_chain_shape_gravity(0,&s,&cfg,112,.016f,configured,target);
    assert(world_queries==5 && s.geometry_sample.accepted);
    for(int j=0;j<3;j++)for(int a=0;a<3;a++)assert(target[j][a]==baseline[j][a]);
    s.gravity_camera_hold_active=0;
    memcpy(target,configured,sizeof(target));
    body_chain_shape_gravity(0,&s,&cfg,128,.016f,configured,target);
    assert(world_queries==6 && s.dynamics_gravity_active && s.dynamics_gravity_direction[1]==1);
    assert(!s.geometry_sample.accepted_jump);
    /* Confirmed pose changes replace the old direction without touching the
       chain's angles. Subsequent small motion keeps the usual filter. */
    assert(!memcmp(s.angle,zero,sizeof(zero)));
    physics_environment_cfg.world_gravity[0]=.02f;
    memcpy(target,configured,sizeof(target));
    body_chain_shape_gravity(0,&s,&cfg,144,.016f,configured,target);
    memcpy(target,configured,sizeof(target));
    body_chain_shape_gravity(0,&s,&cfg,160,.016f,configured,target);
    assert(s.dynamics_gravity_direction[0]>0 && s.dynamics_gravity_direction[0]<.02f);
    puts("PASS: geometry confirms across frames, retains force during camera/body holds, gathers while held and resumes when both gates allow");
    {float k=100,c=8;body_chain_apply_link_inertia(&s,0,1,&k,&c);
    assert(k<100 && fabsf(c/sqrtf(k)-.8f)<1e-6f);
    k=100;c=8;body_chain_apply_link_inertia(&s,0,0,&k,&c);assert(k==100 && c==8);}
    puts("PASS: actual integration helpers preserve movement contribution, gravity disable, camera-held direction and damping ratio");
}
static void gravity_during_collision_warmup(void) {
    const float strengths[]={0,1,1.5f};
    physics_environment_cfg.gravity_apply_to_body_chain=1;
    physics_environment_cfg.world_gravity[0]=.98711f;
    physics_environment_cfg.world_gravity[1]=.04085f;
    physics_environment_cfg.world_gravity[2]=.15475f;
    camera_hold=0;body_chain_collider_states[0].basis_valid=1;
    for(int n=2;n<=3;n++)for(int k=0;k<3;k++){
        body_chain_person_state_t pending={0},ready={0};
        body_chain_physics_config_t cfg={strengths[k],1};
        body_contact_pose_t pose=rod(n);float zero[3][3]={{0}};
        float configured[3][3]={{0,-1.838f,-48.286f},{0,-1,-25},{0,-.5f,-12}};
        assert(body_dynamics_prepare(&pose,zero,2,1,&pending.dynamics));
        pending.dynamics_valid=pending.gravity_probe_promoted=1;ready=pending;
        /* Identical motion and gravity through repeated collision warmups.
           The old code returned fallback targets whenever ready was false. */
        for(int frame=0;frame<250;frame++){
            float a[3][3],b[3][3];memcpy(a,configured,sizeof(a));memcpy(b,a,sizeof(b));
            a[0][1]+=3;b[0][1]+=3;
            pending.angle[0][2]=ready.angle[0][2]=-10;
            body_chain_collider_states[0].ready=(frame>=100 && frame<150) || frame>=225;
            if(n==3)body_chain_shape_penis_gravity(0,&pending,&cfg,1000+frame*16,.016f,configured,a);
            else body_chain_shape_gravity(0,&pending,&cfg,1000+frame*16,.016f,configured,a);
            body_chain_collider_states[0].ready=1;
            if(n==3)body_chain_shape_penis_gravity(0,&ready,&cfg,1000+frame*16,.016f,configured,b);
            else body_chain_shape_gravity(0,&ready,&cfg,1000+frame*16,.016f,configured,b);
            assert(!memcmp(a,b,sizeof(a)));
            if(frame)assert(pending.dynamics_gravity_active && ready.dynamics_gravity_active);
        }
    }
    puts("PASS: penis/testicle gravity matches the ready path throughout repeated collision warmups, with zero/unit/1.5 strength and movement retained");
}
static void gravity_strength(void) {
    body_contact_pose_t pose=rod(3);float zero[3][3]={{0}};
    body_chain_physics_config_t cfg={1,1};
    physics_environment_cfg.gravity_apply_to_body_chain=1;
    body_chain_collider_states[0].ready=1;camera_hold=0;
    /* Reproduce full horizontal/vertical gravity, both signs, at multiple rates.
       Geometry starts unavailable, then confirms as it does during pose load. */
    const unsigned int cadence[]={7,11,16,33};
    for(int k=0;k<4;k++)for(int channel=0;channel<2;channel++)for(int sign=-1;sign<=1;sign+=2) {
        body_chain_person_state_t baseline={0},half={0},off={0},twice={0};
        float configured[3][3]={{0}},target[3][3],reduced[3][3],disabled[3][3],boosted[3][3];
        int axis=channel?1:2;
        assert(body_dynamics_prepare(&pose,zero,2,1,&baseline.dynamics));
        baseline.dynamics_valid=baseline.gravity_probe_promoted=1;half=off=twice=baseline;
        physics_environment_cfg.world_gravity[0]=channel?0:sign;
        physics_environment_cfg.world_gravity[1]=channel?sign:0;
        physics_environment_cfg.world_gravity[2]=0;
        for(int j=0;j<3;j++)configured[j][axis]=sign*12;
        for(unsigned int n=0;n<400;n++) {
            float dt=cadence[k]*.001f;
            memcpy(target,configured,sizeof(target));memcpy(reduced,target,sizeof(target));memcpy(disabled,target,sizeof(target));
            for(int j=0;j<3;j++)for(int a=0;a<3;a++)target[j][a]+=3,reduced[j][a]+=3,disabled[j][a]+=3;
            memcpy(boosted,target,sizeof(boosted));
            cfg.gravity_horizontal_strength=cfg.gravity_vertical_strength=1;
            /* Original and unit-strength paths agree bit-for-bit. */
            body_chain_person_state_t reference=baseline;float expected[3][3];memcpy(expected,target,sizeof(expected));
            body_chain_shape_gravity(0,&reference,&cfg,1000+n*cadence[k],dt,configured,expected);
            body_chain_shape_penis_gravity(0,&baseline,&cfg,1000+n*cadence[k],dt,configured,target);
            assert(!memcmp(expected,target,sizeof(expected)));
            if(channel)cfg.gravity_vertical_strength=.5f;else cfg.gravity_horizontal_strength=.5f;
            body_chain_shape_penis_gravity(0,&half,&cfg,1000+n*cadence[k],dt,configured,reduced);
            if(channel)cfg.gravity_vertical_strength=0;else cfg.gravity_horizontal_strength=0;
            body_chain_shape_penis_gravity(0,&off,&cfg,1000+n*cadence[k],dt,configured,disabled);
            if(channel)cfg.gravity_vertical_strength=2;else cfg.gravity_horizontal_strength=2;
            body_chain_shape_penis_gravity(0,&twice,&cfg,1000+n*cadence[k],dt,configured,boosted);
            for(int j=0;j<3;j++) {
                assert(fabsf(reduced[j][axis]-(3+(target[j][axis]-3)*.5f))<1e-5f);
                assert(disabled[j][axis]==3); /* Motion/wind retained, geometry cannot restore gravity. */
                assert(fabsf(boosted[j][axis]-(3+(target[j][axis]-3)*2))<1e-5f);
                for(int a=0;a<3;a++)if(a!=axis)assert(fabsf(reduced[j][a]-target[j][a])<1e-5f);
            }
        }
        assert(baseline.dynamics_gravity_active && half.dynamics_gravity_active && off.dynamics_gravity_active);
    }
    puts("PASS: horizontal/vertical strength remains effective at full tilt through geometry startup and repeated updates; unit values exact, half/double/zero preserve movement");
}
static void strength_config(void) {
    char dir[MAX_PATH],file[MAX_PATH];assert(GetTempPathA(sizeof(dir),dir));assert(GetTempFileNameA(dir,"gst",0,file));
    read_global(file);assert(body_chain_physics_cfg.gravity_horizontal_strength==1 && body_chain_physics_cfg.gravity_vertical_strength==1);
    assert(WritePrivateProfileStringA("penis_physics","gravity_horizontal_strength","0.5",file));
    read_global(file);assert(body_chain_physics_cfg.gravity_horizontal_strength==.5f);
    assert(WritePrivateProfileStringA("penis_physics","gravity_horizontal_strength",NULL,file));
    read_sidecar(file);assert(body_chain_physics_cfg.gravity_horizontal_strength==.5f);
    assert(WritePrivateProfileStringA("penis_physics","gravity_vertical_strength","0",file));
    read_sidecar(file);assert(body_chain_physics_cfg.gravity_vertical_strength==0);
    assert(WritePrivateProfileStringA("penis_physics","gravity_vertical_strength","99",file));
    read_global(file);assert(body_chain_physics_cfg.gravity_horizontal_strength==1 && body_chain_physics_cfg.gravity_vertical_strength==4);
    assert(WritePrivateProfileStringA("penis_physics","gravity_vertical_strength","-1",file));
    read_global(file);assert(body_chain_physics_cfg.gravity_vertical_strength==0);
    assert(DeleteFileA(file));
    puts("PASS: production INI assignments load strengths, inherit sidecars, apply overrides/clamps and restore defaults on removal");
}
int main(void){gravity_during_collision_warmup();return 0;}
