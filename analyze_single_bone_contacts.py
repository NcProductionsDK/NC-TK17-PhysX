"""Historical reproduction of the former breast/butt collision call convention.

Compiles production conversion helpers against synthetic engine matrices.
The runtime now bypasses this incorrect call convention. These movement-input
helpers still expect authored axes; run_single_bone_contact_tests.py exercises
the replacement world-to-parent contact path.
"""
from pathlib import Path
import os
import re
import subprocess

ROOT = Path(__file__).resolve().parent

def function(file, name):
    source = (ROOT/file).read_text()
    match = re.search(r'^static [^;{}]*\b'+name+r'\([^;{}]*\)\s*\{', source, re.M)
    if not match:
        raise ValueError(name)
    pos, depth = match.end(), 1
    while depth:
        depth += (source[pos] == '{') - (source[pos] == '}')
        pos += 1
    return source[match.start():pos]+'\n'

source = r'''
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
typedef unsigned char BYTE;
static int ptr_readable(const void *p,size_t n){(void)n;return p!=NULL;}
static float physx_absf(float x){return fabsf(x);}
static int sane_probe_float(float x){return isfinite(x);}
static float physx_vec3_len(const float v[3]){return sqrtf(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);}
static float vec3_dot(const float a[3],const float b[3]){return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];}
typedef struct {struct {int camera_relative_live_current_valid;void *camera_relative_trs_raw;float camera_relative_live_current[9];} motion;void *translation_parent_joint_raw[2];} breasts_physics_person_state_t;
'''
for name in ['body_chain_mat3_inverse', 'body_chain_mat3_multiply',
             'body_chain_read_mat3_rows', 'body_chain_normalize_basis_rows',
             'body_chain_transform_row_vector3', 'body_chain_vec3_sane_limit']:
    source += function('NC-TK17-PhysX.c', name)
for name in ['body_collider_det3', 'body_collider_view_delta_to_local']:
    source += function('physx_colliders.c', name)
source += function('physx_physics.c', 'breasts_physics_body_translation_to_parent_local')
source += function('physx_butt.c', 'butt_physics_body_translation_to_parent_local')
source += r'''
static void store(float raw[64],const float m[9]){
 for(int i=0;i<3;i++)memcpy((BYTE*)raw+0x078+16*i,m+3*i,12);
}
int main(void){
 float identity[9]={1,0,0,0,1,0,0,0,1},trs[64]={0},parent[64]={0};
 breasts_physics_person_state_t s={0};store(trs,identity);
 s.motion.camera_relative_live_current_valid=1;s.motion.camera_relative_trs_raw=trs;
 memcpy(s.motion.camera_relative_live_current,identity,sizeof(identity));
 s.translation_parent_joint_raw[0]=s.translation_parent_joint_raw[1]=parent;
 for(int rotated=0;rotated<2;rotated++){
  float p[9]={0,1,0,-1,0,0,0,0,1};if(!rotated)memcpy(p,identity,sizeof(p));store(parent,p);
  for(int axis=0;axis<3;axis++){
   float push[3]={0},local[3],out[3],visible[3];push[axis]=.02f;
   /* Collider basis offsets from the default/current profile: h=088,v=098,s=078. */
   assert(body_collider_view_delta_to_local(push,identity+3,identity+6,identity,local));
   for(int system=0;system<2;system++){
    int ok=system?butt_physics_body_translation_to_parent_local(&s,0,local,out):breasts_physics_body_translation_to_parent_local(&s,0,local,out);
    assert(ok);body_chain_transform_row_vector3(out,p,visible);
    float d[3]={visible[0]-push[0],visible[1]-push[1],visible[2]-push[2]};
    printf("MAPPING system=%s parent_rotated=%d axis=%d intended=(%.3f,%.3f,%.3f) actual=(%.3f,%.3f,%.3f) error=%.6f\n",system?"butt":"breast",rotated,axis,push[0],push[1],push[2],visible[0],visible[1],visible[2],physx_vec3_len(d));
   }
  }
 }
 return 0;
}
'''
build = ROOT/'build'
build.mkdir(exist_ok=True)
c = build/'single_bone_contact_audit.c'
c.write_text(source)
gcc = Path(r'C:\msys64\mingw32\bin\gcc.exe')
env = dict(os.environ, PATH=str(gcc.parent)+os.pathsep+os.environ['PATH'])
exe = build/'single_bone_contact_audit.exe'
subprocess.run([str(gcc), '-m32', '-O2', '-Wall', '-Wextra', '-Werror',
                '-Wno-unused-function', '-static-libgcc', '-o', str(exe), str(c)],
               env=env, check=True)
subprocess.run([str(exe)], env=env, check=True, timeout=30)
