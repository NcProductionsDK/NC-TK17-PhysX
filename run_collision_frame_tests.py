"""Exercise production body collision placement with mismatched camera epochs."""
from pathlib import Path
import os
import re
import subprocess

ROOT = Path(__file__).resolve().parent


def function(file, name):
    text = (ROOT / file).read_text()
    match = re.search(r'^static [^;{}]*\b' + name + r'\([^;{}]*\)\s*\{', text, re.M)
    end, depth = match.end(), 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[match.start():end] + '\n'


source = r'''
#include <assert.h>
#include <stdio.h>
#include "../physx_collision_frame.h"
typedef uint32_t DWORD;
typedef int32_t LONG;
typedef struct {
 collision_frame_sample_t contact_frame_sample;
 float contact_view_to_world[9],contact_world_to_view[9],contact_world_origin[3];
 int contact_frame_valid,basis_valid;DWORD contact_frame_log_tick;
 float basis_h[3],basis_v[3],basis_s[3],root[3];
} body_chain_collider_person_state_t;
static struct {int debug;} defaults_cfg={1};
static LONG named_node_generation=1,captured_camera_version=1;
static DWORD physx_simulation_serial=1,captured_camera_change_tick;
static int captured_camera_inverse_valid=1,raw_valid=1;
static float captured_camera_inverse[16],trs_view[9],trs_origin[3];
static LONG InterlockedCompareExchange(LONG *p,LONG a,LONG b){(void)a;(void)b;return *p;}
static void make_body_runtime_name(char *out,size_t size,const char *p,const char *n){snprintf(out,size,"%s/%s",p,n);}
static void *resolve_axis_map_raw(const char *n){(void)n;return raw_valid?trs_view:NULL;}
static int body_chain_read_mat3_rows(void *raw,float out[9]){if(!raw)return 0;memcpy(out,raw,36);return 1;}
static int body_collider_engine_pivot_view(const char *p,const char *n,void *raw,float out[3]){(void)p;(void)n;(void)raw;memcpy(out,trs_origin,12);return raw_valid;}
static int sane_probe_float(float x){return isfinite(x);}
static float physx_absf(float x){return fabsf(x);}
static float physx_vec3_len(const float v[3]){return sqrtf(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);}
static void log_line(const char *fmt,...){(void)fmt;}
'''
for name in ['body_chain_vec3_sane_limit', 'body_chain_mat3_inverse',
             'body_chain_mat3_multiply', 'body_chain_transform_row_vector3',
             'camera_view_to_world_point']:
    source += function('NC-TK17-PhysX.c', name)
source += (ROOT / 'physx_collision_frame.c').read_text()
source += r'''
static int body_chain_collider_local_to_view(const body_chain_collider_person_state_t *s,const float p[3],float v[3]){
 for(int a=0;a<3;a++)v[a]=s->root[a]+p[0]*s->basis_h[a]+p[1]*s->basis_v[a]+p[2]*s->basis_s[a];return 1;
}
static int body_collider_view_delta_to_local(const float v[3],const float h[3],const float b[3],const float s[3],float out[3]){
 for(int a=0;a<3;a++)out[a]=v[0]*(a==0?h[0]:a==1?b[0]:s[0])+v[1]*(a==0?h[1]:a==1?b[1]:s[1])+v[2]*(a==0?h[2]:a==1?b[2]:s[2]);return 1;
}
'''
for name in ['body_collision_local_point_to_world', 'body_collision_world_vector_to_local']:
    source += function('physx_room_collision.c', name)
source += r'''
static void near3(const float a[3],const float b[3]){for(int i=0;i<3;i++)assert(fabsf(a[i]-b[i])<.00002f);}
static void camera(float angle,float translation){
 float c=cosf(angle),s=sinf(angle);memset(captured_camera_inverse,0,sizeof(captured_camera_inverse));
 captured_camera_inverse[0]=c;captured_camera_inverse[1]=s;
 captured_camera_inverse[4]=-s;captured_camera_inverse[5]=c;
 captured_camera_inverse[10]=captured_camera_inverse[15]=1;
 captured_camera_inverse[12]=translation;
}
static void skeleton(body_chain_collider_person_state_t *s,float placement){
 float c=captured_camera_inverse[0],sn=captured_camera_inverse[1];
 float world[3]={1+placement-captured_camera_inverse[12],2,3};
 memset(trs_view,0,sizeof(trs_view));trs_view[0]=c;trs_view[1]=-sn;trs_view[3]=sn;trs_view[4]=c;trs_view[8]=1;
 body_chain_transform_row_vector3(world,trs_view,trs_origin);
 memcpy(s->basis_h,trs_view,12);memcpy(s->basis_v,trs_view+3,12);memcpy(s->basis_s,trs_view+6,12);memcpy(s->root,trs_origin,12);s->basis_valid=1;
}
static void tick(body_chain_collider_person_state_t *s,DWORD now){physx_simulation_serial++;body_collision_frame_update(s,"Person01",now);}
int main(void){
 body_chain_collider_person_state_t s={0};float local[3]={.3f,.5f,.7f},world[3],expected[3]={1.3f,2.5f,3.7f};
 camera(0,0);skeleton(&s,0);tick(&s,1000);assert(!s.contact_frame_valid);
 tick(&s,1016);assert(s.contact_frame_valid);assert(body_collision_local_point_to_world(&s,local,world));near3(world,expected);
 /* New camera arrives first, then skeleton catches up. Pure camera motion
    must not sweep a stationary body through the room or rotate a contact. */
 float biggest_legacy_error=0;
 for(int frame=0;frame<180;frame++){
  DWORD now=1032+frame*16;camera((frame+1)*.027f,(frame+1)*.006f);
  captured_camera_version++;captured_camera_change_tick=now;
  for(int phase=0;phase<2;phase++){
   if(phase)skeleton(&s,0);
   tick(&s,now+phase);assert(s.contact_frame_valid&&s.contact_frame_sample.held);
   assert(body_collision_local_point_to_world(&s,local,world));near3(world,expected);
   float view[3],old[3],delta[3];body_chain_collider_local_to_view(&s,local,view);camera_view_to_world_point(view,old);
   for(int a=0;a<3;a++)delta[a]=old[a]-expected[a];biggest_legacy_error=fmaxf(biggest_legacy_error,physx_vec3_len(delta));
   float push[3]={0,.02f,0},converted[3];assert(body_collision_world_vector_to_local(&s,push,converted));near3(push,converted);
  }
  /* Actual bone motion still reaches the room during camera movement. */
  float moving[3]={local[0]+.05f,local[1],local[2]},want[3]={expected[0]+.05f,expected[1],expected[2]};
  assert(body_collision_local_point_to_world(&s,moving,world));near3(world,want);
 }
 assert(biggest_legacy_error>.05f);
 puts("PASS: asynchronous orbit/pan produced >5cm error in old mapping; production point/vector conversions stay within 0.02mm and preserve live bone movement");
 /* After aggressive camera movement, two mutually agreeing skeleton samples
    can still carry the old camera transform. The log showed displaced trusted
    placement at camera age 141 ms. Keep that delayed plateau out of contacts. */
 for(DWORD age=48;age<=144;age+=16){
  skeleton(&s,.07f);tick(&s,captured_camera_change_tick+age);
  assert(s.contact_frame_sample.held);
  assert(body_collision_local_point_to_world(&s,local,world));near3(world,expected);
 }
 skeleton(&s,0);tick(&s,captured_camera_change_tick+160);
 assert(s.contact_frame_sample.held);
 tick(&s,captured_camera_change_tick+176);assert(!s.contact_frame_sample.held);
 assert(body_collision_local_point_to_world(&s,local,world));near3(world,expected);
 puts("PASS: late agreeing camera-contaminated placement is held through 144 ms; settled placement confirms without a false room sweep");
 /* A pose location changed while the camera moved: hold placement only,
    then confirm the new full placement after quiet; no permanent pinning. */
 captured_camera_version++;captured_camera_change_tick+=200;
 skeleton(&s,.4f);DWORD now=captured_camera_change_tick+176;
 tick(&s,now);assert(s.contact_frame_sample.held);
 body_collision_frame_update(&s,"Person01",now);assert(s.contact_frame_sample.held);
 tick(&s,now+16);assert(!s.contact_frame_sample.held);
 expected[0]+=.4f;assert(body_collision_local_point_to_world(&s,local,world));near3(world,expected);
 skeleton(&s,.45f);tick(&s,now+32);expected[0]+=.05f;assert(body_collision_local_point_to_world(&s,local,world));near3(world,expected);
 puts("PASS: placement confirms after camera quiet, repeated same-frame reads cannot confirm, subsequent real placement motion is immediate");
 named_node_generation++;tick(&s,now+48);assert(!s.contact_frame_valid);
 tick(&s,now+64);assert(s.contact_frame_valid);
 raw_valid=0;tick(&s,now+80);assert(!s.contact_frame_valid);
 raw_valid=1;captured_camera_inverse_valid=0;tick(&s,now+96);assert(!s.contact_frame_valid);
 captured_camera_inverse_valid=1;tick(&s,now+112);assert(!s.contact_frame_valid);
 tick(&s,now+128);assert(s.contact_frame_valid);
 puts("PASS: scene generation, missing TRS and unavailable initial camera cannot reuse another skeleton's placement");
 return 0;
}
'''
build = ROOT / 'build'
build.mkdir(exist_ok=True)
c = build / 'collision_frame_test.c'
c.write_text(source)
gcc = Path(r'C:\msys64\mingw32\bin\gcc.exe')
env = dict(os.environ, PATH=str(gcc.parent) + os.pathsep + os.environ['PATH'])
exe = build / 'collision_frame_test.exe'
subprocess.run([str(gcc), '-m32', '-O2', '-Wall', '-Wextra', '-Werror',
                '-Wno-unused-function', '-Wno-misleading-indentation', '-static-libgcc',
                '-o', str(exe), str(c)], env=env, check=True)
subprocess.run([str(exe)], env=env, check=True, timeout=30)
