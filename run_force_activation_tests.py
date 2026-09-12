"""Compare changed production activation and force assembly with the baseline."""
import os
from pathlib import Path
import re
import subprocess

root = Path(__file__).resolve().parent
source = (root / 'physx_sidecar.c').read_text()
main = (root / 'NC-TK17-PhysX.c').read_text()
reference = (root / 'tests/reference_optimization.h').read_text()


def function(text, name):
    m = re.search(r'^static [^;{}]*\b' + name + r'\([^;{}]*\)\s*\{', text, re.M)
    if not m:
        raise ValueError(name)
    pos, depth = m.end(), 1
    while depth:
        depth += (text[pos] == '{') - (text[pos] == '}')
        pos += 1
    return text[m.start():pos] + '\n'


def fields(text, variable, typename, extra=''):
    declarations = []
    for field in sorted(set(re.findall(variable + r'->(\w+)', text))):
        m = re.search(r'^    (?:int|float|char|void|DWORD|LONG|physx_target_t)\b[^;\n]*\b' + field + r'(?:\[[^\]]+\])*;', main, re.M)
        if not m:
            raise ValueError(field)
        declarations.append(m.group())
    return 'typedef struct {\n' + '\n'.join(declarations) + extra + '\n} ' + typename + ';\n'


common = r'''
#include <windows.h>
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
static float physx_absf(float x){return fabsf(x);}
static float physx_vec3_len(const float *v){return sqrtf(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);}
'''
gravity_names = ['addon_chain_gravity_axis_for_target', 'addon_chain_target_gravity_tail_axis',
                 'addon_chain_gravity_drive_component', 'addon_chain_world_gravity_bend_vector',
                 'addon_chain_apply_inverted_gravity_bend']
gravity = ''.join(function(source, n) for n in gravity_names)
pos = source.index('/* Simulation maps the combined force below.')
start = source.rfind('if (addon_local_bend) {', 0, pos)
pos, depth = source.index('{', start) + 1, 1
while depth:
    depth += (source[pos] == '{') - (source[pos] == '}')
    pos += 1
block = source[start:pos]
force = common + fields(gravity + block, 'target', 'physx_target_t')
force += fields(gravity + block, 'chain', 'physx_chain_t')
force += r'''
static struct {int debug;} defaults_cfg;
static struct {float gravity_horizontal_body_chain_scale,gravity_vertical_body_chain_scale,gravity_horizontal_secondary_body_chain_scale,gravity_vertical_secondary_body_chain_scale;} physics_environment_cfg={1,1,.3f,-.2f};
static int diag_enabled;
static int addon_gravity_diag_enabled(physx_chain_t *c){(void)c;return diag_enabled;}
static float wind_strength;
static float room_wind_target_strength(physx_chain_t *c,physx_target_t *t,DWORD now){(void)c;(void)t;(void)now;return wind_strength;}
static float physx_clampf(float x,float a,float b){return x<a?a:x>b?b:x;}
'''
force += gravity
force_declarations = r'''
    int addon_local_bend=1, axis;
    DWORD now=1000;
    float addon_force_drive[3]={0},addon_force_bend[3]={0},addon_gravity_bend[3]={0};
    int addon_force_bend_valid=0,addon_gravity_bend_valid=0;
    float addon_wind_strength=0;
'''
force_output = r'''
    memcpy(out,addon_force_bend,3*sizeof(float));
    if(defaults_cfg.debug || diag_enabled) memcpy(out+3,addon_gravity_bend,3*sizeof(float));
    return addon_force_bend_valid | ((defaults_cfg.debug || diag_enabled)?addon_gravity_bend_valid<<1:0);
}
'''
for name, body in [('original', '#include "tests/reference_force.h"\n'), ('optimized', block)]:
    force += 'static int ' + name + '(physx_chain_t *chain,physx_target_t *target,int addon_world_gravity_valid,int addon_world_wind_valid,const float *addon_world_gravity_drive,const float *addon_world_wind_drive,float out[6]) {\n'
    force += force_declarations + body + force_output
force += r'''
int main(void){
 SetErrorMode(SEM_NOGPFAULTERRORBOX);_set_error_mode(_OUT_TO_STDERR);setbuf(stdout,NULL);
 float maximum_error=0;
 for(int i=0;i<30000;i++){
  physx_chain_t c={0};physx_target_t a={0},b;
  c.rotation_drive_horizontal_tail_axis=i%5-1;c.rotation_drive_vertical_tail_axis=(i/5)%5-1;
  c.rotation_drive_horizontal_scale=i%2?-1:1;c.rotation_drive_vertical_scale=i%3?-1:1;c.gravity_scale=(i%7-3)*.6f;
  a.sim_length=i%13?(.01f+(i%17)*.03f):0;
  float rest[3]={sinf(i*.03f),cosf(i*.03f),.3f};float length=physx_vec3_len(rest);
  for(int k=0;k<3;k++)a.sim_rest[k]=rest[k]/length*a.sim_length;
  a.gravity_settings_initialized=i%2;
  a.gravity_horizontal_source_axis=(i/7)%5-1;a.gravity_vertical_source_axis=(i/11)%5-1;
  a.gravity_horizontal_source_sign=i%3?-1:1;a.gravity_vertical_source_sign=i%5?-1:1;
  a.gravity_horizontal_scale=.7f;a.gravity_vertical_scale=-1.2f;
  a.gravity_horizontal_tail_axis=(i/13)%5-1;a.gravity_vertical_tail_axis=(i/17)%5-1;
  a.gravity_horizontal_tail_axis_explicit=i%3==0;a.gravity_vertical_tail_axis_explicit=i%5==0;
  a.gravity_inverted_configured=i%2;a.gravity_inverted_strength=.8f;a.gravity_inverted_tail_axis=i%5-1;a.gravity_inverted_sign=i%3?-1:1;
  float gd[3]={sinf(i*.02f),cosf(i*.02f),sinf(i*.01f)},wd[3]={cosf(i*.015f),sinf(i*.015f),-.4f};
  wind_strength=(i%19-9)*.35f;defaults_cfg.debug=i%7==0;diag_enabled=i%11==0;
  b=a;float x[6]={0},y[6]={0};
  int old=original(&c,&a,(i/2)%2,(i/3)%2,gd,wd,x),now=optimized(&c,&b,(i/2)%2,(i/3)%2,gd,wd,y);
  assert(old==now);assert(!memcmp(&a,&b,sizeof(a)));
  for(int k=0;k<6;k++) {float error=fabsf(x[k]-y[k]); maximum_error=fmaxf(maximum_error,error); if(error>1e-6f) printf("Mismatch i=%d k=%d old=%.9g new=%.9g\n",i,k,x[k],y[k]); assert(error<=1e-6f);}
 }
 printf("Maximum force difference: %.9g\n",maximum_error);
 puts("PASS: 30000 original/optimized force assemblies agree within 1e-6, including target initialization, configured/default axes, zero/reversed gravity, wind and inverted gravity; debug/diagnostic output preserved");
 return 0;
}
'''

activation_names = ['addon_chain_owner_body_ready', 'addon_chain_scene_visible']
activation_functions = ''.join(function(source, n) for n in activation_names)
activation = common + fields(activation_functions, 'target', 'physx_target_t')
# In the test fixture use two targets instead of the production fixed array.
chain_fields = fields(activation_functions, 'chain', 'physx_chain_t')
chain_fields = re.sub(r'physx_target_t targets\[[^\]]+\]', 'physx_target_t targets[2]', chain_fields)
activation += chain_fields + fields(activation_functions, 'sc', 'physx_sidecar_t')
activation += r'''
static struct {float gravity_probe_motion_epsilon;int gravity_probe_require_nonzero_root;} physics_environment_cfg={.001f,1};
static int real_SSimpleTransform_RotationSet=1;
static int visibility,owner_visibility,root_valid,placement_ready,readable,vis_queries,sqrt_queries;
static float root_position[3];static void *root_raw;
static int addon_output_scene_visible(void){vis_queries++;return visibility;}
static int poseedit_scene_person_visible(int i){(void)i;return owner_visibility;}
static int addon_person_prefix_to_index(const char *p){return p[0]=='P'?0:-1;}
static int addon_body_root_pointer_for_person(const char *p,void **raw,float **root){(void)p;*raw=root_raw;*root=root_position;return root_valid;}
static int addon_owner_placement_ready(const char *p,void *raw,float *root,DWORD now){(void)p;(void)raw;(void)root;(void)now;return placement_ready;}
static void log_line(const char *fmt,...){(void)fmt;}
static int is_nil_engine_object(void *raw,void *obj){(void)raw;return !obj;}
static int ptr_readable(void *p,size_t size){(void)size;return readable&&p;}
static int physx_vec3_sane_limit(const float *v,float limit){return isfinite(v[0])&&isfinite(v[1])&&isfinite(v[2])&&physx_absf(v[0])<=limit&&physx_absf(v[1])<=limit&&physx_absf(v[2])<=limit;}
static float counted_length(const float *v){sqrt_queries++;return physx_vec3_len(v);}
#define physx_vec3_len counted_length
'''
activation += activation_functions
for name in activation_names:
    old = function(reference, 'reference_' + name)
    if name == 'addon_chain_scene_visible':
        old = old.replace('addon_chain_owner_body_ready(', 'reference_addon_chain_owner_body_ready(')
    activation += old
activation += r'''
int main(void){
 SetErrorMode(SEM_NOGPFAULTERRORBOX);_set_error_mode(_OUT_TO_STDERR);setbuf(stdout,NULL);
 unsigned long old_queries=0,new_queries=0,old_sqrts=0,new_sqrts=0;
 float memory[64]={0};
 for(int i=0;i<50000;i++){
  physx_sidecar_t sc={0};physx_chain_t a={0},b;
  sc.enabled=i%7!=0;sc.loaded=i%11!=0;sc.addon_scene_active=i%13!=0;sc.room_scene_sidecar=i%3==0;
  strcpy(sc.addon_owner_person,i%5?"Person01":"");
  a.addon_chain=i%17!=0;a.addon_scene_visible=i%2;a.target_count=2;a.object_transform_chain=i%3==1;
  a.addon_root_settle_until_tick=i%7==0?1200:0;a.addon_body_root_raw=(void*)(uintptr_t)(i%3+1);
  for(int j=0;j<2;j++){
   physx_target_t *t=&a.targets[j];t->addon_simulated_target=(i+j)%3!=0;
   t->object=(i+j)%5?(void*)1:NULL;t->s_object=t->object;t->s_raw_object=t->object;
   t->addon_skin_bound=i%2;t->addon_object_name_fallback=i%3==0;
   t->s_translation_base=i%11?memory:NULL;t->s_rotation_base=t->s_translation_base;t->s_translation_offset=0x7c;t->s_rotation_offset=0x6c;
  }
  visibility=i%3-1;owner_visibility=(i/3)%3-1;root_valid=i%7!=0;placement_ready=i%5!=0;readable=i%13!=0;
  root_raw=(void*)(uintptr_t)(i%2+1);root_position[0]=i%7?.02f:0;
  b=a;int x[3],y[3];vis_queries=sqrt_queries=0;
  int old=reference_addon_chain_scene_visible(&sc,&a,1000,&x[0],&x[1],&x[2]);old_queries+=vis_queries;old_sqrts+=sqrt_queries;
  vis_queries=sqrt_queries=0;int now=addon_chain_scene_visible(&sc,&b,1000,&y[0],&y[1],&y[2]);new_queries+=vis_queries;new_sqrts+=sqrt_queries;
  assert(old==now&&!memcmp(x,y,sizeof(x))&&!memcmp(&a,&b,sizeof(a)));
 }
 assert(new_queries<old_queries&&new_sqrts<old_sqrts);
 printf("PASS: 50000 activation decisions/state match original across rooms, clothing, visibility, root replacement, placeholders, settling and unwritable targets; unused visibility calls %lu -> %lu, lengths %lu -> %lu\n",old_queries,new_queries,old_sqrts,new_sqrts);
 return 0;
}
'''

build = root / 'build/optimization-tests'
build.mkdir(parents=True, exist_ok=True)
env = dict(os.environ)
env['PATH'] = r'C:\msys64\mingw32\bin;' + env.get('PATH', '')
for name, text in [('force', force), ('activation', activation)]:
    cfile, exe = build / (name + '.c'), build / (name + '.exe')
    cfile.write_text(text)
    subprocess.run([r'C:\msys64\mingw32\bin\gcc.exe', '-m32', '-O2', '-Wall', '-Wextra', '-Werror', '-I', str(root), str(cfile), '-o', str(exe)], env=env, check=True)
    subprocess.run([str(exe)], env=env, check=True)
