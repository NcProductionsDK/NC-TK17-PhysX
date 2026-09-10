"""Validate production geometry-dependent gravity and effective link inertia."""
from pathlib import Path
import os
import re
import subprocess

ROOT = Path(__file__).resolve().parent

def function(name):
    source=(ROOT/'physx_physics.c').read_text()
    match=re.search(r'^static [^;{}]*\b'+name+r'\([^;{}]*\)\s*\{',source,re.M)
    if not match: raise ValueError(name)
    pos,depth=match.end(),1
    while depth:
        depth+=(source[pos]=='{')-(source[pos]=='}');pos+=1
    return source[match.start():pos]+'\n'

source = r'''
#include <assert.h>
#include <stdio.h>
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
'''
source+=r'''
typedef unsigned long DWORD;
typedef struct {int unused;} body_chain_physics_config_t;
typedef struct {
    body_dynamics_t dynamics;int dynamics_valid,dynamics_gravity_valid,dynamics_gravity_active,gravity_probe_promoted,gravity_camera_hold_active;
    float dynamics_gravity_direction[3],angle[3][3];
    gravity_sample_t geometry_sample;
    void *root_raw;
} body_chain_person_state_t;
static struct {int gravity_apply_to_body_chain;float world_gravity[3],gravity_response_ms;} physics_environment_cfg={1,{0,-1,0},100};
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
'''
source+=function('body_chain_apply_link_inertia')+function('body_chain_shape_gravity')
source+=r'''
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
    physics_environment_cfg.gravity_apply_to_body_chain=1;body_chain_collider_states[0].ready=0;
    body_chain_shape_gravity(0,&s,&cfg,80,.016f,configured,target);
    assert(!s.dynamics_gravity_active && world_queries==3 && !memcmp(target,configured,sizeof(target)));
    /* Production's global pivot hold is disabled. The per-body gravity hold
       must still protect the geometric contribution from a changed camera basis. */
    body_chain_collider_states[0].ready=1;camera_hold=0;s.gravity_camera_hold_active=1;
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
    assert(world_queries==6 && s.dynamics_gravity_active && s.dynamics_gravity_direction[1]>-1);
    puts("PASS: geometry confirms across frames, retains force during camera/body holds, gathers while held and resumes when both gates allow");
    {float k=100,c=8;body_chain_apply_link_inertia(&s,0,1,&k,&c);
    assert(k<100 && fabsf(c/sqrtf(k)-.8f)<1e-6f);
    k=100;c=8;body_chain_apply_link_inertia(&s,0,0,&k,&c);assert(k==100 && c==8);}
    puts("PASS: actual integration helpers preserve movement contribution, gravity disable, camera-held direction and damping ratio");
}
int main(void){analytic_rod();gravity_orientation();scaling_and_lag();free_rest();invalid_geometry();production_wiring();return 0;}
'''
build = ROOT / 'build'
build.mkdir(exist_ok=True)
test = build / 'body_dynamics_test.c'
test.write_text(source)
gcc = Path(r'C:\msys64\mingw32\bin\gcc.exe')
env = dict(os.environ, PATH=str(gcc.parent)+os.pathsep+os.environ['PATH'])
exe = build / 'body_dynamics_test.exe'
subprocess.run([str(gcc), '-m32', '-O2', '-Wall', '-Wextra', '-Werror',
                '-Wno-unused-function', '-static-libgcc', '-o', str(exe), str(test)], env=env, check=True)
subprocess.run([str(exe)], env=env, check=True, timeout=60)
