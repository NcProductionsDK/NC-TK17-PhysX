"""Characterize existing body-contact limitations; does not alter the DLL."""
import os
from pathlib import Path
import re
import subprocess

root = Path(__file__).resolve().parent
colliders = (root / 'physx_colliders.c').read_text()
main = (root / 'NC-TK17-PhysX.c').read_text()


def function(name, source=colliders):
    match = re.search(r'^static [^;{}]*\b' + name + r'\([^;{}]*\)\s*\{', source, re.M)
    if not match:
        raise ValueError(name)
    pos, depth = match.end(), 1
    while depth:
        depth += (source[pos] == '{') - (source[pos] == '}')
        pos += 1
    return source[match.start():pos] + '\n'


fixture = r'''
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
typedef unsigned long DWORD;
static float physx_clampf(float v,float a,float b) {return fmaxf(a,fminf(b,v));}
static float physx_absf(float v) {return fabsf(v);}
static int sane_probe_float(float v) {return isfinite(v);}
static float vec3_dot(const float *a,const float *b) {return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];}
typedef struct { int horizontal_output_axis,vertical_output_axis,rotation_tail_axis[2]; } body_chain_physics_config_t;
static body_chain_physics_config_t body_chain_physics_cfg={2,1,{2,1}};
static struct {float response_strength,link_length[3];} body_chain_collider_cfg={1,{.1f,.1f,.1f}};
typedef struct {float angle[3][3];} body_chain_person_state_t;
typedef struct {
    int chain_points_ready,chain_points_fresh,chain_point_valid[4];
    DWORD chain_points_update_tick;
    float chain_local_point[4][3];
} body_chain_collider_person_state_t;
#define BODY_CHAIN_ENGINE_POINT_STALE_MS 100
#define BODY_CHAIN_COLLISION_POINT_HOLD_MS 80
/* Plausibility is irrelevant to the fresh-point early return under test. */
static int body_chain_engine_points_plausible(const float p[4][3]) {(void)p;return 1;}
static int body_chain_live_points_with_delta(const float p[4][3],const float d[3][2],float o[4][3]);
'''
start = main.index('typedef struct body_chain_contact_t {')
fixture += main[start:main.index('} body_chain_contact_t;', start) + len('} body_chain_contact_t;')]
fixture += function('body_chain_store_contact', main)
for name in ['body_collider_wrap_degrees', 'body_collider_rotate_local_vector',
             'body_collider_rotate_point_about_pivot', 'body_chain_live_points_with_delta_cfg',
             'body_chain_live_points_with_delta', 'body_chain_live_segment_point',
             'body_chain_apply_contact_constraint_correction', 'body_chain_simulated_points_local',
             'body_chain_collision_points_local']:
    fixture += function(name)
fixture += r'''
int main(void) {
    body_chain_contact_t contacts[8];
    float normal[3]={0,1,0},a[3]={-.025f,0,0},b[3]={-.09f,0,0};
    float body[3]={0,-.01f,0};
    int count=0;
    body_chain_store_contact(contacts,&count,8,0,.25f,.005f,a,body,normal);
    body_chain_store_contact(contacts,&count,8,0,.9f,.004f,b,body,normal);
    assert(count==2 && contacts[0].segment_t==.25f);
    printf("FIXED: contacts at segment_t=0.25 and 0.90 with identical normals retained=%d\n",count);
    {
        const float points[4][3]={{0,0,0},{-.1f,0,0},{-.1f,.1f,0},{-.1f,.2f,0}};
        float point[3]={-.1f,.1f,0},anchor[3]={-.1f,.08f,0};
        float correction[3][2]={{0}},out[4][3];
        int locked=body_chain_apply_contact_constraint_correction(points,1,1,1,point,anchor,normal,.005f,1,correction);
        assert(!locked);
        assert(body_chain_apply_contact_constraint_correction(points,1,0,1,point,anchor,normal,.005f,1,correction));
        assert(body_chain_live_points_with_delta(points,correction,out));
        assert(out[2][1]>point[1]+.0001f);
        printf("OBSERVED: distal correction blocked with first_joint=1; allowing upstream joint gives normal separation %.7f\n",out[2][1]-point[1]);
        /* Moving upstream also lifts a proximal point off the same upward
           support plane, so freezing that joint is unnecessarily restrictive. */
        assert(out[1][1]>points[1][1]);
    }
    {
        body_chain_collider_person_state_t cache={0};
        body_chain_person_state_t state={0};
        float before[4][3],after[4][3],predicted[4][3];
        int i,engine=0;
        cache.chain_points_ready=cache.chain_points_fresh=1;
        cache.chain_points_update_tick=1000;
        for(i=0;i<4;i++) {cache.chain_point_valid[i]=1;cache.chain_local_point[i][0]=-.1f*i;}
        assert(body_chain_collision_points_local(&cache,&state,before,&engine,1000));
        assert(engine==1);
        state.angle[0][1]=20;
        assert(body_chain_collision_points_local(&cache,&state,after,&engine,1000));
        assert(!memcmp(before,after,sizeof(before)));
        assert(body_chain_simulated_points_local(&state,&body_chain_physics_cfg,NULL,predicted));
        assert(fabsf(predicted[3][1]-after[3][1])>.01f);
        printf("OBSERVED: fresh cached points ignore a 20-degree candidate angle change; candidate tip discrepancy %.7f\n",predicted[3][1]-after[3][1]);
    }
    puts("The locked-joint and raw-cache examples above exercise retained legacy helpers; the active body_contact_solve uses candidate points and all influencing joints. See run_body_contact_tests.py.");
    return 0;
}
'''
build = root / 'build'
build.mkdir(exist_ok=True)
path = build / 'body_contact_analysis.c'
path.write_text(fixture)
gcc = Path(r'C:\msys64\mingw32\bin\gcc.exe')
env = dict(os.environ)
env['PATH'] = str(gcc.parent) + os.pathsep + env['PATH']
exe = build / 'body_contact_analysis.exe'
subprocess.run([str(gcc), '-m32', '-O2', '-Wall', '-Wextra', '-Werror',
                '-static-libgcc', '-o', str(exe), str(path)], env=env, check=True)
subprocess.run([str(exe)], env=env, check=True)
