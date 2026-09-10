"""Compile the production sidecar integrator and camera guard without TK17."""
import os
from pathlib import Path
import re
import subprocess

root = Path(__file__).resolve().parent
source = (root / 'physx_sidecar.c').read_text()


def function(name):
    match = re.search(r'^static [^;{}]*\b' + name + r'\([^;{}]*\)\s*\{', source, re.M)
    if not match:
        raise ValueError(name)
    pos, depth = match.end(), 1
    while depth:
        depth += (source[pos] == '{') - (source[pos] == '}')
        pos += 1
    return source[match.start():pos] + '\n'


fixture = r'''
#include <windows.h>
#include <assert.h>
#include <stdio.h>
#include "physx_contact_math.h"
static float physx_vec3_len(const float *v) { return sqrtf(physx_contact_dot(v,v)); }
static float addon_vec3_len_exact(const float *v) { return physx_vec3_len(v); }
static float vec3_dot(const float *a,const float *b) { return physx_contact_dot(a,b); }
static float physx_clampf(float x,float a,float b) { return fmaxf(a,fminf(b,x)); }
static struct { int debug; } defaults_cfg;
static struct { int body_chain_camera_quarantine_ms; } physics_environment_cfg;
static DWORD captured_camera_change_tick;
static LONG captured_camera_version;
static int captured_camera_inverse_valid=1;
static void log_line(const char *s,...) { (void)s; }
typedef struct {
    float sim_length, sim_offset[3], sim_rest[3], sim_velocity[3];
    float sim_output_velocity[3], sim_output_offset_prev[3], joint_gain;
    int sim_contact_corrected;
    int sim_initialized, joint_settings_initialized, sim_output_velocity_initialized;
} physx_target_t;
typedef struct {
    float stiffness, joint_gain, drive_strength;
    int addon_chain;
    LONG addon_root_drive_camera_seen_version;
    DWORD addon_root_drive_camera_quarantine_tick;
    DWORD addon_root_drive_camera_last_untrusted_tick;
    DWORD addon_root_drive_camera_log_tick;
    const char *name;
} physx_chain_t;
'''
fixture += function('addon_chain_root_drive_camera_untrusted')
fixture += function('addon_chain_integrate_free_motion')
fixture += function('addon_chain_update_output_velocity')
fixture += r'''
static physx_chain_t chain;
static physx_target_t parent;
static void initialize(physx_target_t *t) {
    memset(t,0,sizeof(*t));
    t->sim_length=.08741f;
    t->sim_rest[1]=t->sim_offset[1]=t->sim_length;
    t->sim_initialized=1;
}
static void validate(const physx_target_t *t) {
    assert(isfinite(t->sim_offset[0]) && isfinite(t->sim_velocity[0]));
    assert(fabsf(physx_vec3_len(t->sim_offset)-t->sim_length)<1e-7f);
    assert(fabsf(vec3_dot(t->sim_offset,t->sim_velocity))<2e-6f);
}
static float frame_dt(int pattern,int frame) {
    switch(pattern) {
    case 0:return 1.0f/30.0f;
    case 1:return 1.0f/60.0f;
    case 2:return 1.0f/144.0f;
    case 3:return frame%2 ? .016f : .017f;
    default:return frame%60==0 ? .050f : .016f;
    }
}
static void steady(int inherit) {
    float reference[3]={0},force[3];
    int pattern,frame,i;
    for(i=0;i<3;i++) force[i]=(i==0?.85713f:(i==2?-.11427f:0))*.5f*150*.08741f;
    for(pattern=0;pattern<5;pattern++) {
        physx_target_t t;
        float minimum=100,maximum=-100;
        initialize(&t);
        for(frame=0;frame<12000;frame++) {
            addon_chain_integrate_free_motion(&chain,&t,&parent,2,inherit,force,150,6,frame_dt(pattern,frame));
            validate(&t);
            if(frame>=10000) {
                float angle=atan2f(t.sim_offset[0],t.sim_offset[1])*57.2957795f;
                minimum=fminf(minimum,angle); maximum=fmaxf(maximum,angle);
            }
        }
        printf("FREE_REST inherit=%d pattern=%d angular_range=%.8f\n",inherit,pattern,maximum-minimum);
        assert(maximum-minimum<.0002f);
        if(pattern==0) memcpy(reference,t.sim_offset,sizeof(reference));
        else for(i=0;i<3;i++) assert(fabsf(t.sim_offset[i]-reference[i])<2e-6f);
    }
}
static void camera_and_movement(void) {
    physx_target_t t,reference;
    float force[3]={5,0,-1},settled[3];
    DWORD now=1000;
    int frame,untrusted=0;
    initialize(&t);
    for(frame=0;frame<2000;frame++)
        addon_chain_integrate_free_motion(&chain,&t,&parent,2,1,force,150,6,.016f);
    reference=t;
    memcpy(settled,t.sim_offset,sizeof(settled));
    /* The real guard toggles through camera changes, pauses and release.
       Free motion must keep the same coefficients and advance throughout. */
    t.sim_velocity[2]=reference.sim_velocity[2]=.03f;
    for(frame=0;frame<1000;frame++) {
        float dt=frame_dt(4,frame);
        now+=(DWORD)(dt*1000);
        if(frame%50<25) {
            captured_camera_version++;
            captured_camera_change_tick=now;
        }
        untrusted+=addon_chain_root_drive_camera_untrusted(&chain,now)!=0;
        addon_chain_integrate_free_motion(&chain,&t,&parent,2,1,force,150,6,dt);
        addon_chain_integrate_free_motion(&chain,&reference,&parent,2,1,force,150,6,dt);
        assert(!memcmp(t.sim_offset,reference.sim_offset,sizeof(t.sim_offset)));
        if(frame==0) assert(fabsf(t.sim_offset[2]-settled[2])>1e-5f);
    }
    assert(untrusted>0 && untrusted<1000);
    memcpy(settled,t.sim_offset,sizeof(settled));
    parent.sim_offset[0]+=.001f;
    addon_chain_integrate_free_motion(&chain,&t,&parent,2,1,force,150,6,.016f);
    assert(fabsf(t.sim_velocity[0])>1e-5f);
    assert(fabsf(t.sim_offset[0]-settled[0])>1e-6f);
    force[2]+=1;
    for(frame=0;frame<100;frame++)
        addon_chain_integrate_free_motion(&chain,&t,&parent,2,1,force,150,6,.016f);
    assert(fabsf(t.sim_offset[2]-settled[2])>.001f);
    parent.sim_offset[0]-=.001f;
    puts("PASS: camera quarantine toggles without changing free motion; momentum, small parent motion and changing force remain responsive");
}
static void contact_release(void) {
    int pattern,frame;
    for(pattern=0;pattern<5;pattern++) {
        physx_target_t t;
        float force[3]={0,-30,0},normal[3]={0,1,0};
        memset(&t,0,sizeof(t));
        t.sim_length=.1f; t.sim_rest[0]=.1f;
        t.sim_offset[0]=.06f; t.sim_offset[1]=-.08f;
        for(frame=0;frame<2000;frame++) {
            float correction[3]={0,0,0};
            addon_chain_integrate_free_motion(&chain,&t,NULL,1,0,force,150,6,frame_dt(pattern,frame));
            correction[1]=fmaxf(0,-.08f-t.sim_offset[1]);
            if(correction[1]>0) {
                physx_contact_link_position(t.sim_offset,t.sim_length,correction);
                physx_contact_link_velocity(t.sim_velocity,t.sim_offset,normal);
            }
            validate(&t);
            assert(fabsf(t.sim_offset[1]+.08f)<2e-6f);
        }
        force[1]=30;
        addon_chain_integrate_free_motion(&chain,&t,NULL,1,0,force,150,6,.016f);
        assert(t.sim_offset[1]>-.0799f && t.sim_velocity[1]>0);
    }
    puts("PASS: production free-motion integration with contact math rests on a plane and releases without sleep at 30/60/144Hz and uneven timesteps");
}
static void initialize_length(physx_target_t *t,float length) {
    initialize(t);
    t->sim_length=t->sim_rest[1]=t->sim_offset[1]=length;
}
static void angular_inheritance(void) {
    const float lengths[3]={.025f,.1f,.4f};
    for(int pattern=0;pattern<5;pattern++) {
        physx_target_t parents[3],children[3];
        float force[3]={0},elapsed=0;
        for(int i=0;i<3;i++) {
            initialize_length(&parents[i],lengths[i]);
            initialize_length(&children[i],.1f);
            parents[i].sim_output_velocity_initialized=1;
        }
        for(int frame=0;frame<1000;frame++) {
            float dt=frame_dt(pattern,frame);elapsed+=dt;
            float angle=.2f*sinf(elapsed*2),speed=.4f*cosf(elapsed*2);
            for(int i=0;i<3;i++) {
                parents[i].sim_offset[0]=lengths[i]*sinf(angle);
                parents[i].sim_offset[1]=lengths[i]*cosf(angle);
                parents[i].sim_output_velocity[0]=lengths[i]*cosf(angle)*speed;
                parents[i].sim_output_velocity[1]=-lengths[i]*sinf(angle)*speed;
                addon_chain_integrate_free_motion(&chain,&children[i],&parents[i],2,1,force,150,6,dt);
                validate(&children[i]);
                if(i) for(int a=0;a<3;a++) {
                    assert(fabsf(children[i].sim_offset[a]-children[0].sim_offset[a])<2e-6f);
                    assert(fabsf(children[i].sim_velocity[a]-children[0].sim_velocity[a])<2e-5f);
                }
            }
        }
    }
    puts("PASS: identical parent angular motion drives the same child response across 16:1 parent lengths and five frame schedules");
}
static void continuous_output_velocity(void) {
    for(int pattern=0;pattern<5;pattern++) {
        physx_target_t t;float dt=frame_dt(pattern,0);
        initialize(&t);
        /* A real sub-micrometre movement per frame was previously cut off. */
        t.sim_velocity[0]=.00002f;
        t.sim_offset[0]=t.sim_velocity[0]*dt;
        memcpy(t.sim_output_offset_prev,t.sim_rest,sizeof(t.sim_rest));
        addon_chain_update_output_velocity(&t,dt,1);
        assert(t.sim_output_velocity_initialized && t.sim_output_velocity[0]==t.sim_velocity[0]);
        t.sim_contact_corrected=1;t.sim_offset[0]+=.01f;
        addon_chain_update_output_velocity(&t,dt,1);
        assert(t.sim_output_velocity[0]==t.sim_velocity[0]);
        t.sim_contact_corrected=0; /* Release uses the same velocity convention. */
        addon_chain_update_output_velocity(&t,dt,1);
        assert(t.sim_output_velocity[0]==t.sim_velocity[0]);
        t.sim_velocity[0]=0;
        addon_chain_update_output_velocity(&t,dt,1);
        assert(t.sim_output_velocity[0]==0);
    }
    puts("PASS: tiny physical velocity survives publication, contact correction adds no impulse, release is continuous, and exact rest stays at rest");
}
static void coupled_chain(void) {
    const float lengths[2][3]={{.1f,.1f,.1f},{.14f,.05f,.11f}};
    float reference[3]={0};
    for(int pattern=0;pattern<5;pattern++) {
        physx_target_t t[2][3];float min_angle=100,max_angle=-100;
        for(int rig=0;rig<2;rig++)for(int j=0;j<3;j++)initialize_length(&t[rig][j],lengths[rig][j]);
        for(int frame=0;frame<3500;frame++) {
            float dt=frame_dt(pattern,frame);
            for(int rig=0;rig<2;rig++)for(int j=0;j<3;j++) {
                float length=lengths[rig][j],force[3]={40*length,0,-10*length};
                addon_chain_integrate_free_motion(&chain,&t[rig][j],j?&t[rig][j-1]:NULL,j+1,j>0,force,150,6,dt);
                addon_chain_update_output_velocity(&t[rig][j],dt,1);
                validate(&t[rig][j]);
                if(rig)for(int a=0;a<3;a++)
                    assert(fabsf(t[rig][j].sim_offset[a]/length-t[0][j].sim_offset[a]/.1f)<3e-5f);
            }
            if(frame>=3200) {
                float angle=atan2f(t[1][2].sim_offset[0],t[1][2].sim_offset[1]);
                min_angle=fminf(min_angle,angle);max_angle=fmaxf(max_angle,angle);
            }
        }
        assert(max_angle-min_angle<.00001f);
        if(!pattern)memcpy(reference,t[1][2].sim_offset,sizeof(reference));
        else for(int a=0;a<3;a++)assert(fabsf(t[1][2].sim_offset[a]-reference[a])<3e-6f);
        /* A small root motion must still reach the final joint after settling. */
        float before=t[1][2].sim_offset[0];
        t[1][0].sim_velocity[0]+=.0002f;
        for(int frame=0;frame<12;frame++)for(int j=0;j<3;j++) {
            float force[3]={40*lengths[1][j],0,-10*lengths[1][j]};
            addon_chain_integrate_free_motion(&chain,&t[1][j],j?&t[1][j-1]:NULL,j+1,j>0,force,150,6,.016f);
            addon_chain_update_output_velocity(&t[1][j],.016f,1);
        }
        assert(fabsf(t[1][2].sim_offset[0]-before)>1e-7f);
        printf("COUPLED_REST pattern=%d angular_range=%g small_motion=%g\n",pattern,max_angle-min_angle,t[1][2].sim_offset[0]-before);
    }
    puts("PASS: complete three-link inheritance settles consistently across equal/unequal lengths and frame schedules; small root motion reaches the tip");
}
static void coupled_contact_release(void) {
    const float lengths[3]={.14f,.05f,.11f};
    for(int pattern=0;pattern<5;pattern++) {
        physx_target_t t[3];float min_tip=100,max_tip=-100;
        for(int j=0;j<3;j++)initialize_length(&t[j],lengths[j]);
        for(int frame=0;frame<2500;frame++)for(int j=0;j<3;j++) {
            float length=lengths[j],force[3]={150*length,0,0};
            float dt=frame_dt(pattern,frame),normal[3]={-1,0,0},correction[3]={0};
            t[j].sim_contact_corrected=0;
            addon_chain_integrate_free_motion(&chain,&t[j],j?&t[j-1]:NULL,j+1,j>0,force,150,6,dt);
            /* Each link is supported by a plane at 30% of its radius. */
            correction[0]=fminf(0,.3f*length-t[j].sim_offset[0]);
            if(correction[0]<0) {
                physx_contact_link_position(t[j].sim_offset,length,correction);
                physx_contact_link_velocity(t[j].sim_velocity,t[j].sim_offset,normal);
                t[j].sim_contact_corrected=1;
            }
            addon_chain_update_output_velocity(&t[j],dt,1);
            validate(&t[j]);
            assert(t[j].sim_offset[0]<=.3f*length+2e-6f);
            if(frame>=2300 && j==2) {
                min_tip=fminf(min_tip,t[j].sim_offset[0]);max_tip=fmaxf(max_tip,t[j].sim_offset[0]);
                assert(fabsf(t[j].sim_output_velocity[0])<.00001f);
            }
        }
        assert(max_tip-min_tip<2e-6f);
        for(int j=0;j<3;j++) {
            float force[3]={-150*lengths[j],0,0};
            t[j].sim_contact_corrected=0;
            addon_chain_integrate_free_motion(&chain,&t[j],j?&t[j-1]:NULL,j+1,j>0,force,150,6,.016f);
            addon_chain_update_output_velocity(&t[j],.016f,1);
            assert(t[j].sim_offset[0]<.3f*lengths[j]-.00001f && t[j].sim_velocity[0]<0);
        }
    }
    puts("PASS: unequal three-link chain rests on supports without inherited correction impulses and releases all links when force reverses");
}
int main(void) {
    chain.stiffness=150;chain.joint_gain=1;chain.drive_strength=1;
    chain.addon_chain=1;chain.name="test";
    parent.sim_initialized=parent.sim_output_velocity_initialized=1;
    parent.sim_rest[0]=-.02629f;parent.sim_rest[1]=.17020f;parent.sim_rest[2]=.03665f;
    parent.sim_offset[0]=.04341f;parent.sim_offset[1]=.16849f;parent.sim_offset[2]=.02702f;
    parent.sim_length=physx_vec3_len(parent.sim_rest);
    steady(0);steady(1);
    camera_and_movement();contact_release();
    angular_inheritance();continuous_output_velocity();coupled_chain();coupled_contact_release();
    puts("PASS: production constrained springs settle to the same rest across frame rates, with and without parent inheritance");
    return 0;
}
'''
# The camera regression relies on the caller continuing the integrator with
# the configured coefficients. Catch reintroduction of the old physical holds.
simulation = function('run_chain_simulations')
assert 'body_chain_camera_coast_stiffness_scale' not in simulation
assert 'body_chain_camera_coast_damping_scale' not in simulation
assert 'addon_stationary_child_offset' not in simulation
build = root / 'build'
build.mkdir(exist_ok=True)
test_c = build / 'free_motion_test.c'
test_c.write_text(fixture)
gcc = Path(r'C:\msys64\mingw32\bin\gcc.exe')
env = dict(os.environ)
env['PATH'] = str(gcc.parent) + os.pathsep + env['PATH']
exe = build / 'free_motion_test.exe'
subprocess.run([str(gcc), '-m32', '-O2', '-Wall', '-Wextra', '-Werror',
                '-Wno-unused-function', '-static-libgcc', '-I', str(root),
                '-o', str(exe), str(test_c)], env=env, check=True)
subprocess.run([str(exe)], env=env, check=True)
