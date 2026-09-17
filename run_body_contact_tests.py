"""Regression fixtures using the actual built-in chain contact solver."""
import os
from pathlib import Path
import re
import subprocess

root=Path(__file__).resolve().parent
colliders=(root/'physx_colliders.c').read_text()
main=(root/'NC-TK17-PhysX.c').read_text()
contact=(root/'physx_body_contact.c').read_text()
dynamics=(root/'physx_body_dynamics.h').read_text()

def function(name,source):
    match=re.search(r'^static [^;{}]*\b'+name+r'\([^;{}]*\)\s*\{',source,re.M)
    if not match: raise ValueError(name)
    pos,depth=match.end(),1
    while depth:
        depth+=(source[pos]=='{')-(source[pos]=='}');pos+=1
    return source[match.start():pos]+'\n'

fixture=r'''
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../physx_body_pose.h"
#define BODY_CHAIN_MAX_CONTACTS 24
typedef unsigned long DWORD;
static float physx_clampf(float v,float a,float b){return fmaxf(a,fminf(b,v));}
static float physx_absf(float v){return fabsf(v);}
static int sane_probe_float(float v){return isfinite(v);}
static float vec3_dot(const float *a,const float *b){return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];}
static float physx_vec3_len(const float *v){return sqrtf(vec3_dot(v,v));}
typedef struct {int horizontal_output_axis,vertical_output_axis,rotation_tail_axis[3],output_offset,collision_scope;float link_min_angle[3][3],link_max_angle[3][3];} body_chain_physics_config_t;
static body_chain_physics_config_t body_chain_physics_cfg;
static struct {float response_strength,response_max_degrees_per_tick;int collision_iterations;float link_length[3];} body_chain_collider_cfg={1,20,2,{.5f,.5f,.5f}};
typedef struct {
    float angle[3][3],velocity[3][3],collision_step_points[4][3],collision_step_angle[3][3],collision_free_target[3][3];
    DWORD collision_step_tick;
    float collision_step_dt;
    int collision_step_valid;
    int collision_step_engine_points;
    body_contact_pose_t collision_pose;
    int collision_pose_valid;
    body_dynamics_t dynamics;
    int dynamics_valid;
    int gravity_camera_hold_active,initialized,active_logged;
    void *joint_raw[3];
    float collision_contact_direction[3][2],collision_prev_max_penetration;
    int collision_rest_valid,collision_rest_ticks,collision_rest_grace_ticks,collision_impact_ticks,collision_multi_support_grace_ticks;
} body_chain_person_state_t;
typedef struct {
    float testicle_joint_position[3][3];
    int testicle_points_ready;
    DWORD testicle_points_update_tick;
    float chain_local_point[4][3];
    int chain_points_ready,chain_points_fresh,chain_point_valid[4];
    DWORD chain_points_update_tick;
    int ready,basis_valid;
} body_chain_collider_person_state_t;
'''
fixture=fixture.replace('#include "../physx_body_pose.h"', function('physx_contact_rotation_rows',(root/'physx_contact_math.h').read_text()) + (root/'physx_body_pose.h').read_text().replace('#include "physx_contact_math.h"',''))
fixture=fixture.replace('#define BODY_CHAIN_MAX_CONTACTS',dynamics.replace('#include "physx_body_pose.h"','')+'\n#define BODY_CHAIN_MAX_CONTACTS',1)
motion=(root/'physx_body_motion.h').read_text()
fixture+='#define BODY_MOTION_REFERENCE_MS 16u\n#define BODY_MOTION_MAX_ELAPSED_MS 100u\n'
for name in ['body_motion_duration','body_motion_substeps','body_motion_spring_step',
             'body_motion_limit_brake','body_motion_limited_spring_step']:
    fixture+=function(name,motion)
fixture+=re.search(r'^#define BODY_CHAIN_ENGINE_POINT_STALE_MS .*$',main,re.M).group(0)+'\n'
fixture+=re.search(r'^#define BODY_CHAIN_COLLISION_POINT_HOLD_MS .*$',main,re.M).group(0)+'\n'
start=main.index('typedef struct body_chain_contact_t {')
fixture+=main[start:main.index('} body_chain_contact_t;',start)+len('} body_chain_contact_t;')]
for name in ['body_chain_link_axis_limit','body_chain_link_axis_min_limit','body_chain_clamp_link_axis_angle']:
    fixture+=function(name,(root/'physx_config.c').read_text())
fixture+=function('body_chain_store_contact',main)
fixture+=function('body_chain_testicle_collision_points_local',colliders)
for name in ['body_chain_simulated_points_local','body_chain_engine_points_plausible','body_chain_collision_points_local']:
    fixture+=function(name,colliders)
fixture+=function('body_chain_closest_segment_pair',colliders)
for name in ['body_collider_wrap_degrees','body_collider_rotate_local_vector',
             'body_collider_rotate_point_about_pivot','body_chain_live_points_with_delta_cfg']:
    fixture+=function(name,colliders)
for name in ['body_contact_inverse_inertia','body_contact_predict','body_contact_candidate_points','body_contact_point','body_contact_jacobian',
             'body_contact_pose_error',
             'body_contact_bound_correction','body_contact_overlap_error','body_contact_limit_velocity',
             'body_contact_orient_from_history','body_contact_refine_combined',
             'body_contact_solve','body_contact_apply']:
    fixture+=function(name,contact)
fixture+=function('body_chain_penis_cross_sample',colliders)
fixture+=r'''
typedef unsigned char BYTE;
static body_chain_physics_config_t testicle_physics_cfg;
static body_chain_collider_person_state_t body_chain_collider_states[4];
static body_chain_person_state_t body_chain_person_states[4],runtime_body_chain_person_states[4],testicle_physics_states[4],runtime_testicle_physics_states[4];
static long body_chain_poseeditor_mode_active;
static long InterlockedCompareExchange(long *v,long a,long b){long old=*v;if(old==b)*v=a;return old;}
static struct {int gravity_horizontal_basis_offset,gravity_vertical_basis_offset,gravity_horizontal_secondary_basis_offset;} physics_environment_cfg={0x088,0x098,0x078};
static int pivot_hold,frame_updates;
static int body_chain_camera_pivot_hold_active(DWORD now){(void)now;return pivot_hold;}
static int ptr_readable(const void *p,size_t n){(void)n;return p!=NULL;}
static int body_chain_collision_scope_collider_mask(int scope){return scope;}
static void update_body_chain_colliders_for_person_scope(int p,DWORD now,int scope){(void)p;(void)now;(void)scope;frame_updates++;}
static void body_contact_trace_pose(const body_chain_collider_person_state_t *c,body_chain_person_state_t *s,const body_chain_physics_config_t *cfg,int t,DWORD now,const float observed[4][3],int engine){(void)c;(void)s;(void)cfg;(void)t;(void)now;(void)observed;(void)engine;}
'''
fixture+=function('body_contact_capture_step',contact)+function('body_contact_begin_step',contact)
fixture+=r'''
static const float base[4][3]={{0,0,0},{-.1f,0,0},{-.1f,.1f,0},{-.1f,.2f,0}};
static void configure(void){
    int j,a;memset(&body_chain_physics_cfg,0,sizeof(body_chain_physics_cfg));
    body_chain_physics_cfg.horizontal_output_axis=2;body_chain_physics_cfg.vertical_output_axis=1;
    body_chain_physics_cfg.rotation_tail_axis[0]=2;body_chain_physics_cfg.rotation_tail_axis[1]=1;
    for(j=0;j<3;j++)for(a=0;a<3;a++){body_chain_physics_cfg.link_min_angle[j][a]=-90;body_chain_physics_cfg.link_max_angle[j][a]=90;}
}
static void support(body_chain_contact_t *c,int segment,float t,float depth,const float *normal,const float points[4][3]){
    memset(c,0,sizeof(*c));c->segment=segment;c->segment_t=t;c->penetration=depth;c->strength=1.0f;
    memcpy(c->normal,normal,3*sizeof(float));body_contact_point(points,c,c->chain);
    c->target_sep=vec3_dot(c->chain,normal)+depth;
}
static float gap(const float p[4][3],const body_chain_contact_t *c){float x[3];body_contact_point(p,c,x);return vec3_dot(x,c->normal)-c->target_sep;}
static void supports(void){
    body_chain_person_state_t state={0};body_chain_contact_t c[8];int count=0;
    float up[3]={0,1,0},side[3]={0,0,1},a[3]={-.025f,0,0},b[3]={-.09f,0,0},body[3]={0};
    float correction[3][2],out[4][3];
    body_chain_store_contact(c,&count,8,0,.25f,.004f,a,body,up,1.0f);
    body_chain_store_contact(c,&count,8,0,.9f,.003f,b,body,up,1.0f);
    assert(count==2);
    body_chain_store_contact(c,&count,8,0,.9f,.002f,b,body,up,1.0f);assert(count==2);
    support(&c[0],0,1,.001f,up,base);support(&c[1],1,1,.005f,up,base);
    support(&c[2],1,.6f,.002f,side,base);
    body_contact_solve(&body_chain_physics_cfg,&state,base,c,3,3,correction);
    body_chain_live_points_with_delta_cfg(&body_chain_physics_cfg,base,correction,out);
    assert(gap(out,&c[0])>=-2e-6f && gap(out,&c[1])>=-2e-6f && gap(out,&c[2])>=-2e-6f);
    assert(fabsf(correction[0][1])>.01f);
    puts("PASS: distinct lever arms retained; duplicate suppressed; distal support can use upstream joint while preserving proximal/corner contacts");
}
static void candidate(void){
    body_chain_person_state_t s={0};float p[4][3];
    memcpy(s.collision_step_points,base,sizeof(base));s.collision_step_valid=1;s.collision_step_tick=42;
    s.angle[0][1]=20;
    assert(body_contact_candidate_points(&s,&body_chain_physics_cfg,42,p));
    assert(fabsf(p[1][1]-base[1][1])>.01f);
    assert(!body_contact_candidate_points(&s,&body_chain_physics_cfg,43,p));
    assert(!memcmp(s.collision_step_points,base,sizeof(base)));
    puts("PASS: predicted query follows angle changes without mutating sampled engine points; old tick rejected");
}
static void testicle_geometry(void){
    /* Recorded real pivots: the old midpoint/extrapolation path returned an
       extended tip and rotated the chain about the proximal sphere centre. */
    const float joints[3][3]={{-.02f,0,-.037f},{-.00265f,.00403f,-.08538f},
                             {-.00759f,.00670f,-.14275f}};
    body_chain_collider_person_state_t cache={0};body_chain_person_state_t s={0};
    float p[4][3],rotated[4][3],delta[3][2]={{0}};int engine=0;
    memcpy(cache.testicle_joint_position,joints,sizeof(joints));
    cache.testicle_points_ready=1;cache.testicle_points_update_tick=100;
    assert(body_chain_testicle_collision_points_local(&cache,&s,p,&engine,100));
    assert(engine==1 && !memcmp(p,joints,sizeof(joints)) && !memcmp(p[2],p[3],sizeof(p[2])));
    delta[1][1]=10;
    body_chain_live_points_with_delta_cfg(&body_chain_physics_cfg,p,delta,rotated);
    assert(!memcmp(rotated[0],joints[0],sizeof(p[0])) && !memcmp(rotated[1],joints[1],sizeof(p[1])));
    assert(fabsf(rotated[2][0]-p[2][0])+fabsf(rotated[2][1]-p[2][1])>1e-5f);
    assert(body_chain_testicle_collision_points_local(&cache,&s,p,&engine,400));
    assert(!body_chain_testicle_collision_points_local(&cache,&s,p,&engine,601) && engine==0);
    cache.testicle_points_ready=0;
    assert(!body_chain_testicle_collision_points_local(&cache,&s,p,&engine,100));
    cache.testicle_points_ready=1;cache.testicle_joint_position[2][1]=NAN;
    assert(!body_chain_testicle_collision_points_local(&cache,&s,p,&engine,100));
    puts("PASS: active testicle chain uses real pivots/tip; child rotates about real joint; held geometry preserved; expired/invalid geometry rejected");
}
static void asynchronous_cross_sample(void){
    body_chain_collider_person_state_t cache={0};body_chain_person_state_t s={0};
    float p[4][3],fallback[4][3];int j;
    cache.chain_points_ready=cache.chain_points_fresh=1;cache.chain_points_update_tick=100;
    memcpy(cache.chain_local_point,base,sizeof(base));
    for(j=0;j<4;j++)cache.chain_point_valid[j]=1;
    memcpy(s.collision_step_points,base,sizeof(base));
    s.collision_step_valid=s.collision_step_engine_points=1;s.collision_step_tick=100;
    s.angle[0][1]=20;
    assert(body_chain_penis_cross_sample(&cache,&s,p,100));
    assert(fabsf(p[1][1]-base[1][1])>.01f);
    assert(body_chain_penis_cross_sample(&cache,&s,p,116));
    assert(!memcmp(p,base,sizeof(base)));
    body_chain_simulated_points_local(&s,&body_chain_physics_cfg,NULL,fallback);
    assert(physx_vec3_len(fallback[3])>1.4f && physx_vec3_len(p[3])<.3f);
    cache.chain_points_fresh=0;
    assert(body_chain_penis_cross_sample(&cache,&s,p,150));
    assert(!body_chain_penis_cross_sample(&cache,&s,p,181));
    cache.chain_points_ready=0;
    assert(!body_chain_penis_cross_sample(&cache,&s,p,116));
    puts("PASS: staggered chain ticks retain live/held penis geometry; expired samples never become configured 1.5-unit fallback colliders");
}
static void velocity(void){
    body_chain_person_state_t s={0};body_chain_contact_t c;
    float up[3]={0,1,0},corr[3][2];
    support(&c,0,1,0,up,base);
    s.velocity[0][1]=-10;s.velocity[0][2]=3;
    body_contact_solve(&body_chain_physics_cfg,&s,base,&c,1,3,corr);
    assert(fabsf(s.velocity[0][1])<1e-5f);
    assert(fabsf(s.velocity[0][2]-3)<1e-6f);
    s.velocity[0][1]=10;
    body_contact_solve(&body_chain_physics_cfg,&s,base,&c,1,3,corr);
    assert(s.velocity[0][1]==10);
    puts("PASS: inward contact velocity removed; tangential sliding and outward release preserved; no position-correction impulse");
}
static void blocked_velocity(void){
    const float straight[4][3]={{0,0,0},{-.1f,0,0},{-.2f,0,0},{-.3f,0,0}};
    body_chain_person_state_t s={0};body_chain_contact_t c;
    float down[3]={0,-1,0},corr[3][2];
    body_chain_physics_cfg.link_max_angle[0][1]=7.5f;
    s.angle[0][1]=7.5f;s.velocity[0][1]=100;
    support(&c,1,1,0,down,straight);
    body_contact_solve(&body_chain_physics_cfg,&s,straight,&c,1,2,corr);
    if(fabsf(s.velocity[0][1])>1e-5f || fabsf(s.velocity[1][1])>1e-5f){
        fprintf(stderr,"FAIL: blocked root velocity leaked into contact: root=%g child=%g\n",s.velocity[0][1],s.velocity[1][1]);exit(1);
    }
    s.velocity[0][1]=-10;
    body_contact_solve(&body_chain_physics_cfg,&s,base,0,0,2,corr);
    assert(s.velocity[0][1]==-10);
    configure();
    puts("PASS: velocity into a joint limit is removed before contact coupling; movement away from the limit remains responsive");
}
static void limits(void){
    body_chain_person_state_t s={0};body_chain_contact_t c;float up[3]={0,1,0},corr[3][2];int j,a;
    support(&c,1,1,.02f,up,base);
    for(j=0;j<3;j++)for(a=0;a<3;a++){body_chain_physics_cfg.link_min_angle[j][a]=body_chain_physics_cfg.link_max_angle[j][a]=1;s.angle[j][a]=1;}
    body_contact_solve(&body_chain_physics_cfg,&s,base,&c,1,3,corr);
    for(j=0;j<3;j++)for(a=0;a<2;a++)assert(corr[j][a]==0);
    configure();memset(&s,0,sizeof(s));body_chain_collider_cfg.response_strength=0;
    body_contact_solve(&body_chain_physics_cfg,&s,base,&c,1,3,corr);
    for(j=0;j<3;j++)for(a=0;a<2;a++)assert(corr[j][a]==0);
    body_chain_collider_cfg.response_strength=1;
    puts("PASS: unreachable joint limits remain bounded; collision strength zero disables response");
}
static void settling(void){
    const float rates[6]={30,60,144,60,20,60};int r,n,j,a,segments,composed;
    for(composed=0;composed<=1;composed++) for(segments=2;segments<=3;segments++) for(r=0;r<6;r++){
        body_chain_person_state_t s={0};float minimum=100,maximum=-100;
        const float straight[4][3]={{0,0,0},{-.1f,0,0},{-.2f,0,0},{-.3f,0,0}};
        if(composed){
            assert(body_pose_fit(straight,s.angle,segments,&s.collision_pose));
            s.collision_step_valid=s.collision_pose_valid=1;
            assert(body_dynamics_prepare(&s.collision_pose,s.angle,2,1,&s.dynamics));s.dynamics_valid=1;
            /* Sideways bending stays active while all links press on the plane. */
            s.angle[0][2]=20;s.angle[1][2]=-15;s.angle[2][2]=10;
        }
        for(n=0;n<3120;n++){
            float duration=r==3?(n%60==0?.05f:.016f):r==5?body_motion_duration(n%60==0?500:16):1/rates[r];
            int steps=body_motion_substeps(duration),step;
            float dt=duration/steps;
            s.collision_step_dt=dt;
            float delta[3][2]={{0}},points[4][3],corr[3][2],up[3]={0,1,0};
            for(step=0;step<steps;step++) {
            body_chain_contact_t c[3];int count=0;
            float configured[3][3]={{0}},shaped[3][3],direction[3]={0,n<3000?-1:1,0};
            for(j=0;j<segments;j++) configured[j][1]=(composed?20:-20)*(n<3000?1:-1);
            memcpy(shaped,configured,sizeof(shaped));
            if(composed) assert(body_dynamics_gravity(&s.dynamics.reference,&s.dynamics,s.angle,direction,configured,shaped));
            for(j=0;j<segments;j++){
                float inverse=body_contact_inverse_inertia(&s,j,1);
                body_motion_limited_spring_step(&s.angle[j][1],&s.velocity[j][1],shaped[j][1],100*inverse,8*sqrtf(inverse),dt,-90,90);
                delta[j][1]=s.angle[j][1];
            }
            if(composed) body_contact_predict(&body_chain_physics_cfg,&s,straight,NULL,points);
            else body_chain_live_points_with_delta_cfg(&body_chain_physics_cfg,straight,delta,points);
            for(j=0;j<segments;j++) if(points[j+1][1]<0){
                support(&c[count],j,1,-points[j+1][1],up,points);count++;
            }
            body_contact_solve(&body_chain_physics_cfg,&s,points,c,count,segments,corr);
            body_contact_apply(&s,&body_chain_physics_cfg,corr,segments,0);
            for(j=0;j<segments;j++)for(a=0;a<2;a++)delta[j][a]=s.angle[j][a?1:2];
            if(composed) body_contact_predict(&body_chain_physics_cfg,&s,straight,NULL,points);
            else body_chain_live_points_with_delta_cfg(&body_chain_physics_cfg,straight,delta,points);
            for(j=0;j<segments;j++)assert(isfinite(s.angle[j][1]) && points[j+1][1]>-.00002f);
            }
            if(n>2500 && n<3000){minimum=fminf(minimum,points[segments][1]);maximum=fmaxf(maximum,points[segments][1]);}
            if(n==3119) assert(points[segments][1]>.002f);
        }
        printf("BODY_REST composed=%d segments=%d pattern=%d tip_range=%.9f\n",composed,segments,r,maximum-minimum);
        assert(maximum-minimum<.00002f);
    }
    puts("PASS: production substeps, angular springs and contact solve rest and release at 20/30/60/144Hz, uneven frames and capped stalls without contact sleep");
}
static float overlap_error(const float p[4][3],body_chain_contact_t *c,int count){
    float error=0;int i;for(i=0;i<count;i++){float d=fmaxf(0,-gap(p,&c[i]));error+=d*d;}return error;
}
static void competing_supports(void){
    const float straight[4][3]={{0,0,0},{-.1f,0,0},{-.2f,0,0},{-.3f,0,0}};
    body_chain_person_state_t s={0};body_chain_contact_t c[2];
    float up[3]={0,1,0},down[3]={0,-1,0},corr[3][2],out[4][3];
    support(&c[0],0,1,.004f,up,straight);
    support(&c[1],0,1,.006f,down,straight);
    body_contact_solve(&body_chain_physics_cfg,&s,straight,c,2,1,corr);
    body_chain_live_points_with_delta_cfg(&body_chain_physics_cfg,straight,corr,out);
    if(overlap_error(out,c,2)>=overlap_error(straight,c,2)-1e-6f){
        fprintf(stderr,"FAIL: competing supports stall despite available combined descent: error %g -> %g\n",overlap_error(straight,c,2),overlap_error(out,c,2));exit(1);
    }
    puts("PASS: competing contacts make combined progress when individual full corrections would worsen overlap");
}
static void embedded_history(void){
    float normal[3]={0,1,0},inside[3]={0,-.025f,0},outside[3]={0,-.11f,0};
    assert(!body_contact_orient_from_history(normal,inside,.025f,.1025f,.001f));
    assert(normal[1]==1);
    assert(body_contact_orient_from_history(normal,outside,.11f,.1025f,.001f));
    assert(normal[1]==-1);
    normal[1]=1;outside[1]=.11f;
    assert(!body_contact_orient_from_history(normal,outside,.11f,.1025f,.001f));
    puts("PASS: embedded history cannot reverse outward recovery; verified exterior entry side remains protected");
}
static void observed_pose_error(void){
    float before[4][3]={{0}},predicted[4][3]={{0}},observed[4][3]={{0}},error[3];
    predicted[2][0]=.01f;observed[2][0]=-.01f;
    body_contact_pose_error(before,predicted,observed,2,error);
    assert(fabsf(error[0]-.02f)<1e-7f && fabsf(error[1]-.01f)<1e-7f && fabsf(error[2]-.01f)<1e-7f);
    memcpy(observed,predicted,sizeof(predicted));
    body_contact_pose_error(before,predicted,observed,2,error);assert(error[0]==0);
    memset(observed,0,sizeof(observed));
    body_contact_pose_error(before,predicted,observed,2,error);assert(fabsf(error[0]-.01f)<1e-7f && error[2]==0);
    assert(predicted[2][0]==.01f && before[2][0]==0);
    puts("PASS: read-only pose audit distinguishes opposite movement, exact agreement and an unchanged engine sample");
}
static void composed_contact(void){
    const float points[4][3]={{0,0,0},{.0219286f,.0069399f,-.0729516f},
        {-.0250599f,.0207295f,-.1471537f},{-.0692438f,.0281818f,-.1677821f}};
    const float angles[3][3]={{0,-5.20544f,26.02263f},{0,-5.49461f,34.13186f},{0,-3.25490f,23.90110f}};
    body_chain_person_state_t s={0};body_chain_contact_t c[2];float up[3]={0,1,0},corr[3][2],out[4][3],raw[4][3];
    memcpy(s.angle,angles,sizeof(angles));memcpy(s.collision_step_angle,angles,sizeof(angles));
    memcpy(s.collision_step_points,points,sizeof(points));s.collision_step_valid=1;
    assert(body_pose_fit(points,angles,3,&s.collision_pose));s.collision_pose_valid=1;
    support(&c[0],0,1,.001f,up,points);support(&c[1],2,.8f,.002f,up,points);
    body_contact_solve(&body_chain_physics_cfg,&s,points,c,2,3,corr);
    body_contact_predict(&body_chain_physics_cfg,&s,points,corr,out);
    assert(gap(out,&c[0])>=-2e-6f && gap(out,&c[1])>=-2e-6f);
    body_contact_apply(&s,&body_chain_physics_cfg,corr,3,0);
    body_pose_evaluate(&s.collision_pose,s.angle,raw);
    for(int j=0;j<4;j++)for(int a=0;a<3;a++)assert(fabsf(raw[j][a]-out[j][a])<1e-7f);
    puts("PASS: composed output mapping separates proximal/distal supports and published angles reproduce the solved pose");
}
static void contact_near_limit(void){
    const float straight[4][3]={{0,0,0},{-.1f,0,0},{-.2f,0,0},{-.3f,0,0}};
    const float rates[]={20,30,60,144};int r,n;
    for(r=0;r<4;r++) {
        body_chain_person_state_t s={0};body_chain_contact_t c;
        float points[4][3],corr[3][2],up[3]={0,1,0},minimum=100,maximum=-100;
        float dt=(1/rates[r])/body_motion_substeps(1/rates[r]);
        configure();body_chain_physics_cfg.link_min_angle[0][1]=-7.5f;
        body_chain_physics_cfg.link_max_angle[0][1]=7.5f;
        assert(body_pose_fit(straight,s.angle,1,&s.collision_pose));
        s.collision_step_valid=s.collision_pose_valid=1;s.collision_step_dt=dt;
        s.angle[0][1]=6; /* The support sits inside the progressive braking band. */
        body_contact_predict(&body_chain_physics_cfg,&s,straight,NULL,points);
        support(&c,0,1,0,up,points);
        for(n=0;n<1600;n++) {
            body_motion_limited_spring_step(&s.angle[0][1],&s.velocity[0][1],
                n<1500?7.5f:-4,115,4,dt,-7.5f,7.5f);
            body_contact_predict(&body_chain_physics_cfg,&s,straight,NULL,points);
            body_contact_solve(&body_chain_physics_cfg,&s,points,&c,1,1,corr);
            body_contact_apply(&s,&body_chain_physics_cfg,corr,1,0);
            body_contact_predict(&body_chain_physics_cfg,&s,straight,NULL,points);
            assert(gap(points,&c)>-2e-6f);
            if(n>1200 && n<1500) {
                minimum=fminf(minimum,points[1][1]);maximum=fmaxf(maximum,points[1][1]);
            }
            if(n==1500) assert(s.velocity[0][1]<0);
        }
        assert(maximum-minimum<2e-6f && gap(points,&c)>.002f);
        /* A contact can still use angles beyond the beginning of the band. */
        s.angle[0][1]=6;s.velocity[0][1]=0;
        body_contact_predict(&body_chain_physics_cfg,&s,straight,NULL,points);
        {float toward_stop[3]={0,-1,0};
        support(&c,0,1,.0024f,toward_stop,points);}
        body_contact_solve(&body_chain_physics_cfg,&s,points,&c,1,1,corr);
        body_contact_apply(&s,&body_chain_physics_cfg,corr,1,0);
        assert(s.angle[0][1]>7.3f && s.angle[0][1]<=7.5f);
    }
    configure();
    puts("PASS: near-limit contact rests and releases at 20/30/60/144Hz; collision escape retains the full angular range");
}
static void inertia_contact_energy(void){
    const float straight[4][3]={{0,0,0},{-.1f,0,0},{-.2f,0,0},{-.3f,0,0}};
    body_chain_person_state_t s={0};body_chain_contact_t c;
    float points[4][3],normal[3]={0,1,0},correction[3][2]={{0}},jac[3][2],before=0,after=0,vn=0;
    int j,a;
    assert(body_pose_fit(straight,s.angle,3,&s.collision_pose));
    assert(body_dynamics_prepare(&s.collision_pose,s.angle,2,1,&s.dynamics));
    s.collision_step_valid=s.collision_pose_valid=s.dynamics_valid=1;
    body_contact_predict(&body_chain_physics_cfg,&s,straight,NULL,points);
    support(&c,2,1,0,normal,points);
    body_contact_jacobian(&body_chain_physics_cfg,&s,points,correction,&c,normal,jac,0);
    for(j=0;j<3;j++)for(a=0;a<2;a++) {
        float v=-jac[j][a]*10000;s.velocity[j][a?1:2]=v;
        before+=v*v/s.dynamics.inverse_inertia[j][a];
    }
    body_contact_solve(&body_chain_physics_cfg,&s,points,&c,1,3,correction);
    for(j=0;j<3;j++)for(a=0;a<2;a++) {
        float v=s.velocity[j][a?1:2];after+=v*v/s.dynamics.inverse_inertia[j][a];vn+=jac[j][a]*v;
    }
    assert(before>0 && after<=before+1e-5f && vn>=-1e-6f);
    puts("PASS: contact projects with matching inverse inertias, cancels inward speed and does not add kinetic energy");
}
static void thigh_recovery(void){
    float points[4][3]={{0,0,0},{.00379f,.05747f,-.05033f},
        {-.04599f,.12097f,-.08767f},{-.08744f,.14531f,-.09872f}};
    const float hip[2][3]={{-.08788f,.11241f,.03107f},{-.08788f,-.11241f,.03107f}};
    const float thigh[2][3]={{-.07939f,.11466f,-.15985f},{-.07939f,-.11466f,-.15985f}};
    body_chain_person_state_t s={0};int frame,i,b,a;float deepest=0;
    for(frame=0;frame<180;frame++){
        body_chain_contact_t c[6];int count=0;float corr[3][2],out[4][3];
        deepest=0;
        for(i=0;i<3;i++)for(b=0;b<2;b++){
            float ct,bt,p[3],q[3],dist,normal[3],depth;
            body_chain_closest_segment_pair(points[i],points[i+1],hip[b],thigh[b],&ct,&bt,p,q,&dist);
            if(i==0 && ct<.08f)continue;
            depth=.1025f-dist-.001f;deepest=fmaxf(deepest,depth);
            if(depth<=1e-6f || dist<1e-6f)continue;
            for(a=0;a<3;a++)normal[a]=(p[a]-q[a])/dist;
            body_chain_store_contact(c,&count,6,i,ct,fminf(depth,.05f),p,q,normal,1.0f);
        }
        if(!count)break;
        body_contact_solve(&body_chain_physics_cfg,&s,points,c,count,3,corr);
        body_chain_live_points_with_delta_cfg(&body_chain_physics_cfg,points,corr,out);
        body_contact_apply(&s,&body_chain_physics_cfg,corr,3,deepest);
        memcpy(points,out,sizeof(points));
    }
    if(deepest>.00005f){fprintf(stderr,"FAIL: synthetic two-thigh recovery stalled: remaining penetration=%g\n",deepest);exit(1);}
    printf("PASS: synthetic embedded chain escapes two thigh capsules in %d bounded updates; residual=%g\n",frame,deepest);
}
static void deep_overlap(void){
    /* The game's failed candidate used a 45-degree per-joint setting and
       contacts within 0.0002 units of the opposing chain's centerline. */
    const float logged[4][3]={{0,0,0},{-.00251f,-.00008f,-.07645f},
        {-.08025f,-.00017f,-.11959f},{-.12943f,-.00020f,-.11576f}};
    int n,j,a;float largest=0;
    body_chain_collider_cfg.response_max_degrees_per_tick=45;
    body_chain_collider_cfg.response_strength=2;
    for(n=0;n<200;n++){
        body_chain_person_state_t s={0};body_chain_contact_t c[3];
        float correction[3][2],out[4][3],norm=0,normal[3];
        float phase=(float)n*.151f;
        normal[0]=cosf(phase)*.01f;normal[1]=sinf(phase);normal[2]=cosf(phase)*.99995f;
        support(&c[0],1,.369f,.01975f,normal,logged);
        normal[0]=-normal[0];normal[1]=-normal[1];normal[2]=-normal[2];
        support(&c[1],2,.7f,.025f,normal,logged);
        normal[0]=1;normal[1]=normal[2]=0;
        support(&c[2],0,.15f,.03f,normal,logged);
        body_contact_solve(&body_chain_physics_cfg,&s,logged,c,3,3,correction);
        body_chain_live_points_with_delta_cfg(&body_chain_physics_cfg,logged,correction,out);
        for(j=0;j<3;j++)for(a=0;a<2;a++)norm+=correction[j][a]*correction[j][a];
        largest=fmaxf(largest,sqrtf(norm));
        if(norm>36.0001f || overlap_error(out,c,3)>overlap_error(logged,c,3)+1e-9f){
            fprintf(stderr,"FAIL deep overlap case=%d correction_norm=%g error=%g -> %g\n",n,sqrtf(norm),overlap_error(logged,c,3),overlap_error(out,c,3));
            exit(1);
        }
    }
    printf("PASS: 200 deep/conflicting supports with 45-degree configuration reduce overlap; largest chain correction=%g degrees\n",largest);
    body_chain_collider_cfg.response_max_degrees_per_tick=20;
    body_chain_collider_cfg.response_strength=1;
}
static void warmup_geometry(void){
    testicle_physics_cfg=body_chain_physics_cfg;
    for(int testicle=0;testicle<=1;testicle++){
        body_chain_person_state_t s={0};body_chain_collider_person_state_t *c=&body_chain_collider_states[0];
        float published[3][3]={{0}};memset(c,0,sizeof(*c));
        c->basis_valid=1;c->chain_points_ready=c->chain_points_fresh=c->testicle_points_ready=1;
        c->chain_points_update_tick=c->testicle_points_update_tick=1000;
        for(int j=0;j<4;j++){
            c->chain_point_valid[j]=1;c->chain_local_point[j][0]=-.1f*j;
            if(j<3){c->testicle_joint_position[j][0]=-.1f*j;s.joint_raw[j]=published[j];}
        }
        body_contact_begin_step(0,&s,testicle,1000,.016f);
        assert(!c->ready && s.dynamics_valid && s.collision_pose_valid);
        assert(s.dynamics.segments==(testicle?2:3));
        body_dynamics_t before=s.dynamics;
        c->ready=1;c->chain_points_update_tick=c->testicle_points_update_tick=1016;
        body_contact_begin_step(0,&s,testicle,1016,.016f);
        assert(s.dynamics_valid && !memcmp(&before,&s.dynamics,sizeof(before)));
        /* Invalid basis, held/unconfirmed or old pivots cannot initialize a
           chain before the collision gate. Previously prepared dynamics survive. */
        c->ready=0;c->basis_valid=0;
        body_contact_begin_step(0,&s,testicle,1032,.016f);assert(!s.collision_step_valid);
        assert(!memcmp(&before,&s.dynamics,sizeof(before)));
        c->basis_valid=1;c->chain_points_update_tick=c->testicle_points_update_tick=1048;
        s.gravity_camera_hold_active=1;
        body_contact_begin_step(0,&s,testicle,1048,.016f);assert(!s.collision_step_valid);
        s.gravity_camera_hold_active=0;pivot_hold=1;
        c->chain_points_update_tick=c->testicle_points_update_tick=1064;
        body_contact_begin_step(0,&s,testicle,1064,.016f);assert(!s.collision_step_valid);
        pivot_hold=0;body_contact_begin_step(0,&s,testicle,1080,.016f);assert(!s.collision_step_valid);
        c->chain_points_update_tick=c->testicle_points_update_tick=1096;
        body_contact_begin_step(0,&s,testicle,1096,.016f);assert(s.collision_pose_valid && !c->ready);
        if(!testicle){
            c->chain_points_fresh=0;c->chain_points_update_tick=1112;
            body_contact_begin_step(0,&s,0,1112,.016f);assert(!s.collision_step_valid);
            c->chain_points_ready=0;
            body_contact_begin_step(0,&s,0,1128,.016f);assert(!s.collision_step_valid);
        }
    }
    assert(frame_updates>0);
    puts("PASS: production capture/begin initializes both chain models before collision promotion from current engine pivots only; ready toggle preserves model, unsafe/held/synthetic samples rejected");
}
static void incoming_strength_checks(void){
 const float strengths[]={.1f,.25f,1.0f};float up[3]={0,1,0},body[3]={0};
 for(int segments=2;segments<=3;segments++)for(int passes=1;passes<=6;passes+=5)for(int k=0;k<3;k++){
  body_chain_person_state_t s={0};body_chain_contact_t c[4];int count=0;
  float correction[3][2],out[4][3];configure();body_chain_collider_cfg.collision_iterations=passes;
  body_chain_store_contact(c,&count,4,0,1,.002f,base[1],body,up,strengths[k]);
  body_contact_solve(&body_chain_physics_cfg,&s,base,c,count,segments,correction);
  body_contact_predict(&body_chain_physics_cfg,&s,base,correction,out);
  assert(fabsf(out[1][1]-.002f*strengths[k])<2e-6f);
  /* Zero-depth contact isolates velocity response from changing Jacobians. */
  support(c,0,1,0,up,base);c[0].strength=strengths[k];s.velocity[0][1]=-10;
  body_contact_solve(&body_chain_physics_cfg,&s,base,c,1,segments,correction);
  assert(fabsf(s.velocity[0][1]+10*(1-strengths[k]))<2e-5f);
  /* Hard self/room support with the same normal must survive soft merging. */
  count=0;memset(&s,0,sizeof(s));
  body_chain_store_contact(c,&count,4,0,1,.002f,base[1],body,up,strengths[k]);
  body_chain_store_contact(c,&count,4,0,1,.002f,base[1],body,up,1);
  assert(count==(strengths[k]<1?2:1));
  body_contact_solve(&body_chain_physics_cfg,&s,base,c,count,segments,correction);
  body_contact_predict(&body_chain_physics_cfg,&s,base,correction,out);
  assert(fabsf(out[1][1]-.002f)<2e-6f);
  support(c,0,1,0,up,base);support(c+1,0,1,0,up,base);c[0].strength=strengths[k];
  s.velocity[0][1]=-10;body_contact_solve(&body_chain_physics_cfg,&s,base,c,2,segments,correction);
  assert(fabsf(s.velocity[0][1])<2e-5f);
 }
 body_chain_collider_cfg.collision_iterations=2;
 puts("PASS: two/three-link incoming strengths preserve fractional recovery across solver iteration counts, soften inward speed, and retain mixed self/room hard supports");
}
static void sustained_strength_checks(void){
 const float base[4][3]={{0,0,0},{-.1f,0,0},{-.2f,0,0},{-.3f,0,0}};
 const float rates[]={20,30,60,144},strengths[]={1,.1f,.25f,.1f,1};
 for(int segments=2;segments<=3;segments++)for(int r=0;r<4;r++)for(int hard=0;hard<2;hard++){
  body_chain_person_state_t s={0};float up[3]={0,1,0},points[4][3],corr[3][2],free_points[4][3];
  configure();assert(body_pose_fit(base,s.angle,segments,&s.collision_pose));
  s.collision_step_valid=s.collision_pose_valid=1;
  float dt=(1/rates[r])/body_motion_substeps(1/rates[r]);s.collision_step_dt=dt;
  for(int phase=0;phase<5;phase++){
   s.collision_free_target[0][1]=phase==3?.5f:0;
   body_pose_evaluate(&s.collision_pose,s.collision_free_target,free_points);
   float applied=hard?1:strengths[phase];
   float expected=free_points[1][1]+applied*(.004f-free_points[1][1]);
   for(int frame=0;frame<800;frame++){
    for(int j=0;j<segments;j++)for(int a=0;a<3;a++)
     body_motion_spring_step(&s.angle[j][a],&s.velocity[j][a],s.collision_free_target[j][a],200,6,dt);
    body_contact_predict(&body_chain_physics_cfg,&s,base,NULL,points);
    body_chain_contact_t c[2];support(c,0,1,.004f-points[1][1],up,points);c[0].strength=strengths[phase];
    if(hard)support(c+1,0,1,.004f-points[1][1],up,points);
    body_contact_solve(&body_chain_physics_cfg,&s,points,c,hard?2:1,segments,corr);
    body_contact_apply(&s,&body_chain_physics_cfg,corr,segments,0);
    body_contact_predict(&body_chain_physics_cfg,&s,base,NULL,points);
    if(frame>600)assert(fabsf(points[1][1]-expected)<.00002f);
   }
  }
 }
 puts("PASS: sustained two/three-link contacts remain weak across thousands of steps, live strength changes, nonzero free targets and frame rates; mixed hard supports stay full");
}
int main(void){setbuf(stdout,NULL);configure();supports();candidate();testicle_geometry();asynchronous_cross_sample();velocity();blocked_velocity();limits();composed_contact();contact_near_limit();inertia_contact_energy();observed_pose_error();embedded_history();competing_supports();thigh_recovery();deep_overlap();settling();warmup_geometry();incoming_strength_checks();sustained_strength_checks();return 0;}
'''
build=root/'build';build.mkdir(exist_ok=True)
path=build/'body_contact_test.c';path.write_text(fixture)
gcc=Path(r'C:\msys64\mingw32\bin\gcc.exe');env=dict(os.environ)
env['PATH']=str(gcc.parent)+os.pathsep+env['PATH'];exe=build/'body_contact_test.exe'
subprocess.run([str(gcc),'-m32', '-include', str(root / 'physx_collision_profile.h'),'-O2','-Wall','-Wextra','-Werror','-static-libgcc','-o',str(exe),str(path)],env=env,check=True)
subprocess.run([str(exe)],env=env,check=True,timeout=60)
