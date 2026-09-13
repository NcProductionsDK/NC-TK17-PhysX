"""Compare the current contact solver against its frozen pre-cache version.

Run run_body_contact_tests.py first to generate the production-source fixture.
Timings here are isolated helper workloads, not an in-game FPS prediction.
"""
import os
from pathlib import Path
import re
import subprocess

root = Path(__file__).resolve().parent
build = root / 'build/body-contact-performance'
build.mkdir(parents=True, exist_ok=True)
fixture_path = root / 'build/body_contact_test.c'
if not fixture_path.exists():
    raise SystemExit('Run run_body_contact_tests.py first')
fixture = fixture_path.read_text()
production = (root / 'physx_body_contact.c').read_text()


def function(text, name):
    m = re.search(r'^static [^;{}]*\b' + name + r'\([^;{}]*\)\s*\{', text, re.M)
    if not m:
        raise ValueError(name)
    pos, depth = m.end(), 1
    while depth:
        depth += (text[pos] == '{') - (text[pos] == '}')
        pos += 1
    return text[m.start():pos]


# Refresh the changed solver even when the surrounding generated fixture exists.
fixture = fixture.replace(function(fixture, 'body_contact_solve'),
                          function(production, 'body_contact_solve'), 1)
fixture = fixture.replace('int main(void)', 'static int original_regression_main(void)', 1)
jacobian = function(fixture, 'body_contact_jacobian')
fixture = fixture.replace(jacobian, 'static unsigned long jacobian_evaluations;\n' +
                          jacobian.replace('{', '{\n    jacobian_evaluations++;', 1), 1)
fixture += '\n#include "tests/reference_body_contact_solve.h"\n'
fixture += r'''
#include <windows.h>
static unsigned int rng=0x17315u;
static float unit(void){rng=rng*1664525u+1013904223u;return (float)(rng>>8)/16777216.0f;}
static double timer(void){LARGE_INTEGER t,f;QueryPerformanceCounter(&t);QueryPerformanceFrequency(&f);return (double)t.QuadPart/f.QuadPart;}
static void scenario(int index,int count,body_chain_person_state_t *s,
                     body_chain_contact_t contacts[BODY_CHAIN_MAX_CONTACTS],float points[4][3],int *segments){
 const float straight[4][3]={{0,0,0},{-.07f,0,0},{-.18f,0,0},{-.31f,0,0}};
 configure();memset(s,0,sizeof(*s));*segments=1+index%3;
 s->collision_step_dt=(index%4==0?.05f:index%4==1?1.0f/30: index%4==2?1.0f/60:1.0f/144);
 body_chain_physics_cfg.horizontal_output_axis=index%3;
 body_chain_physics_cfg.vertical_output_axis=(index%3+1)%3;
 body_chain_physics_cfg.rotation_tail_axis[0]=body_chain_physics_cfg.horizontal_output_axis;
 body_chain_physics_cfg.rotation_tail_axis[1]=body_chain_physics_cfg.vertical_output_axis;
 body_chain_collider_cfg.response_strength=index%19==0?0:index%7==0?.35f:1;
 body_chain_collider_cfg.response_max_degrees_per_tick=index%2?20:45;
 body_chain_collider_cfg.collision_iterations=1+index%6;
 if(index%2){
  assert(body_pose_fit(straight,s->angle,*segments,&s->collision_pose));
  s->collision_step_valid=s->collision_pose_valid=1;
  if(index%3){assert(body_dynamics_prepare(&s->collision_pose,s->angle,
    body_chain_physics_cfg.horizontal_output_axis,body_chain_physics_cfg.vertical_output_axis,&s->dynamics));s->dynamics_valid=1;}
 }
 for(int j=0;j<3;j++)for(int a=0;a<3;a++){
  s->angle[j][a]=(unit()-.5f)*60;
  s->velocity[j][a]=(unit()-.5f)*80;
  if(index%11==0)body_chain_physics_cfg.link_min_angle[j][a]=body_chain_physics_cfg.link_max_angle[j][a]=s->angle[j][a];
  else if(index%5==0){body_chain_physics_cfg.link_min_angle[j][a]=s->angle[j][a]-1;body_chain_physics_cfg.link_max_angle[j][a]=s->angle[j][a];}
 }
 assert(body_contact_predict(&body_chain_physics_cfg,s,straight,NULL,points));
 for(int c=0;c<count;c++){
  float normal[3]={unit()-.5f,unit()-.5f,unit()-.5f};float length=physx_vec3_len(normal);
  if(length<.001f){normal[0]=1;normal[1]=normal[2]=0;length=1;}
  for(int a=0;a<3;a++)normal[a]/=length;
  support(&contacts[c],c%*segments,unit(),(unit()-.15f)*.025f,normal,points);
  if(index%13==0&&c>0)contacts[c]=contacts[0];
 }
}
static void compare(void){
 float max_correction=0,max_velocity=0;
 unsigned long saved=0;
 for(int i=0;i<2400;i++){
  body_chain_person_state_t old,current;body_chain_contact_t contacts[BODY_CHAIN_MAX_CONTACTS],unchanged[BODY_CHAIN_MAX_CONTACTS];
  float points[4][3],a[3][2],b[3][2];int count=i%25,segments;
  memset(contacts,0,sizeof(contacts));scenario(i,count,&old,contacts,points,&segments);current=old;
  memcpy(unchanged,contacts,sizeof(contacts));
  jacobian_evaluations=0;reference_body_contact_solve(&body_chain_physics_cfg,&old,points,contacts,count,segments,a);
  unsigned long original_calls=jacobian_evaluations;
  jacobian_evaluations=0;body_contact_solve(&body_chain_physics_cfg,&current,points,contacts,count,segments,b);
  assert(original_calls-jacobian_evaluations==(body_chain_collider_cfg.response_strength>0?(unsigned long)(7*count):0));
  saved+=original_calls-jacobian_evaluations;
  assert(!memcmp(contacts,unchanged,sizeof(contacts)));
  for(int j=0;j<3;j++)for(int k=0;k<2;k++){
   float error=fabsf(a[j][k]-b[j][k]);max_correction=fmaxf(max_correction,error);assert(isfinite(error)&&error<=1e-6f);
  }
  for(int j=0;j<3;j++)for(int k=0;k<3;k++){
   float error=fabsf(old.velocity[j][k]-current.velocity[j][k]);max_velocity=fmaxf(max_velocity,error);
   if(error>1e-5f)fprintf(stderr,"case=%d velocity error=%g\n",i,error);
   assert(isfinite(error)&&error<=1e-5f);
  }
  /* All non-velocity state must remain byte-identical. */
  memcpy(current.velocity,old.velocity,sizeof(old.velocity));assert(!memcmp(&old,&current,sizeof(old)));
 }
 printf("PASS: 2400 differential solves (0..24 contacts, 1..3 links, legacy/composed poses, inertia, limits, response settings); max correction=%g degrees, max velocity=%g degrees/s; %lu redundant Jacobians removed\n",max_correction,max_velocity,saved);
}
static volatile float result_sink;
static void benchmark(void){
 const int counts[]={0,1,3,8,24};
 for(int model=0;model<2;model++)for(int workload=0;workload<5;workload++){
  body_chain_person_state_t initial;body_chain_contact_t contacts[BODY_CHAIN_MAX_CONTACTS];float points[4][3];int segments;
  scenario(model?5:2,counts[workload],&initial,contacts,points,&segments);
  /* Use a free angular range for the measured contact solve. */
  configure();body_chain_collider_cfg.response_strength=1;body_chain_collider_cfg.collision_iterations=2;
  for(int mode=0;mode<2;mode++){
   double start=timer();
   for(int repeat=0;repeat<2000;repeat++){
    body_chain_person_state_t state=initial;float correction[3][2];
    if(mode)body_contact_solve(&body_chain_physics_cfg,&state,points,contacts,counts[workload],segments,correction);
    else reference_body_contact_solve(&body_chain_physics_cfg,&state,points,contacts,counts[workload],segments,correction);
    result_sink=state.velocity[0][0]+correction[0][0];
   }
   printf("BENCH %s contacts=%d %s %.3f ms / 2000 solves\n",model?"composed":"legacy",counts[workload],mode?"optimized":"original",(timer()-start)*1000);
  }
 }
}
int main(void){SetErrorMode(SEM_NOGPFAULTERRORBOX);_set_error_mode(_OUT_TO_STDERR);setbuf(stdout,NULL);compare();benchmark();return 0;}
'''
cfile, exe = build / 'comparison.c', build / 'comparison.exe'
cfile.write_text(fixture)
gcc = Path(r'C:\msys64\mingw32\bin\gcc.exe')
env = dict(os.environ, PATH=str(gcc.parent) + os.pathsep + os.environ['PATH'])
subprocess.run([str(gcc), '-m32', '-O2', '-Wall', '-Wextra', '-Werror', '-Wno-unused-function',
                '-I', str(root), '-include', str(root / 'physx_collision_profile.h'),
                str(cfile), '-o', str(exe)], env=env, check=True)
result = subprocess.run([str(exe)], env=env, check=True, capture_output=True, text=True, timeout=120)
(build / 'results.txt').write_text(result.stdout)
print(result.stdout, end='')
