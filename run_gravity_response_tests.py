"""Exercise production gravity sampling, camera capture, filtering and sidecar routing."""
from pathlib import Path
import os,re,subprocess
root=Path(__file__).resolve().parent
main=(root/'NC-TK17-PhysX.c').read_text()
hooks=(root/'physx_hooks_core.c').read_text()
sidecar=(root/'physx_sidecar.c').read_text()
def function(source,name):
    m=re.search(r'^static [^;{}]*\b'+name+r'\([^;{}]*\)\s*\{',source,re.M)
    if not m: raise ValueError(name)
    pos,depth=m.end(),1
    while depth:
        depth+=(source[pos]=='{')-(source[pos]=='}');pos+=1
    return source[m.start():pos]+'\n'
source=r'''
#include <windows.h>
#include <assert.h>
#include <stdio.h>
#include "../physx_gravity_sample.h"
#define THISCALL
#define CAMERA_CONTAMINATION_TEST_STEP_COUNT 1
static struct {LONG active,phase;} camera_contamination_test_cfg;
static float camera_contamination_test_engine_camera[16],camera_contamination_test_applied_camera[16];
static int camera_contamination_test_camera_valid;
static float captured_camera_inverse[16];
static int captured_camera_inverse_valid;
static LONG captured_camera_version,captured_camera_rotation_version;
static DWORD captured_camera_tick,captured_camera_change_tick,captured_camera_rotation_change_tick,last_camera_gate_log_tick;
static uint32_t physx_simulation_serial;
static void (*tramp_AppTracker_SetWorldMatrixInverse)(void*,const float*);
static void (*real_AppTracker_SetWorldMatrixInverse)(void*,const float*);
static void log_line(const char *fmt,...){(void)fmt;}
static float physx_absf(float v){return fabsf(v);}
static float physx_vec3_len(const float *v){return sqrtf(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);}
static int sane_probe_float(float v){return isfinite(v);}
static float physx_clampf(float x,float a,float b){return fminf(b,fmaxf(a,x));}
static struct {int gravity_probe_camera_quiet_ms,body_chain_camera_quarantine_ms;float gravity_response_ms,gravity_max_degrees_per_second;} physics_environment_cfg={10,0,100,0};
typedef struct {gravity_sample_t gravity_sample;float gravity_drive[3],gravity_drive_filtered[3];int gravity_drive_filtered_valid;} body_chain_person_state_t;
typedef struct {gravity_sample_t addon_gravity_sample;int addon_gravity_camera_hold_active,addon_gravity_camera_release_active,addon_gravity_trusted_valid;float addon_gravity_trusted_drive[3];} physx_chain_t;
'''
source+=function(main,'camera_rotation_delta')+function(main,'camera_world_to_view_direction')
source+=function(hooks,'hook_AppTracker_SetWorldMatrixInverse')+function(main,'gravity_sample_live')
source+=function(main,'update_body_chain_gravity_filter')+function(sidecar,'addon_chain_camera_safe_gravity_drive')
source+=r'''
static void identity(float m[16]){memset(m,0,sizeof(float)*16);m[0]=m[5]=m[10]=m[15]=1;}
static int sample(gravity_sample_t *s,const float *v,uint32_t f,uint32_t t,uint32_t camera,uint32_t age,float *out){return gravity_sample_update(s,1,v,1,f,t,camera,1,age,10,out);}
static void protocol(void){
 gravity_sample_t s={0};float a[3]={0,-1,0},b[3]={0,1,0},out[3]={0};
 assert(!sample(&s,a,1,100,0,100,out));
 assert(!sample(&s,a,1,101,0,101,out));
 assert(sample(&s,a,2,116,0,116,out)&&s.accepted&&out[1]==-1);
 sample(&s,b,3,132,0,132,out);assert(!s.accepted&&out[1]==-1);
 sample(&s,b,4,148,0,148,out);assert(s.accepted&&out[1]==1);
 sample(&s,a,5,164,0,164,out);assert(!s.accepted&&out[1]==1);
 sample(&s,b,6,180,0,180,out);assert(!s.accepted&&out[1]==1);
 sample(&s,b,7,196,0,196,out);assert(s.accepted);
 sample(&s,a,7,197,1,0,out);assert(!s.accepted&&s.reason==GRAVITY_SAMPLE_CAMERA&&out[1]==1);
 sample(&s,a,8,244,1,47,out);assert(!s.accepted);
 sample(&s,a,9,245,1,48,out);assert(!s.accepted);
 sample(&s,b,10,261,1,64,out);assert(!s.accepted&&out[1]==1);
 sample(&s,b,11,277,1,80,out);assert(s.accepted&&out[1]==1);
 sample(&s,a,12,400,1,203,out);assert(!s.accepted);
 sample(&s,a,13,416,1,219,out);assert(s.accepted&&out[1]==-1);
 float bad[3]={NAN,0,0};sample(&s,bad,14,432,1,235,out);assert(!s.accepted&&s.reason==GRAVITY_SAMPLE_INVALID&&out[1]==-1);
 sample(&s,b,15,448,1,251,out);assert(!s.accepted);
 assert(!gravity_sample_update(&s,2,b,1,16,464,1,1,267,10,out)&&!s.trusted_valid);
 memset(&s,0,sizeof(s));sample(&s,a,UINT32_MAX,UINT32_MAX-7,0,100,out);
 assert(sample(&s,a,0,8,0,116,out)&&s.accepted);
 puts("PASS: pose flip in two samples; spikes, late matrix, camera updates, stalls, rebinding and substeps; clock/frame wrap");
}
static void cameras(void){
 float m[16],world[3]={0,-1,0},view[3],a[3]={0,-1,0},out[3];gravity_sample_t s={0};
 identity(m);hook_AppTracker_SetWorldMatrixInverse(NULL,m);
 assert(captured_camera_version==0&&captured_camera_rotation_version==0);
 sample(&s,a,1,100,0,100,out);sample(&s,a,2,116,0,116,out);
 for(int i=1;i<=200;i++){
  m[12]=i*.3f;m[13]=-i*.1f;m[14]=i*.5f;hook_AppTracker_SetWorldMatrixInverse(NULL,m);
  assert(captured_camera_rotation_version==0&&captured_camera_version==i);
  assert(camera_world_to_view_direction(world,view)&&view[1]==-1&&view[0]==0&&view[2]==0);
  physx_simulation_serial=100+i;gravity_sample_live(&s,(void*)1,view,1,captured_camera_change_tick,out);
  assert(!s.accepted&&out[1]==-1);
 }
 m[0]=0;m[1]=1;m[4]=-1;m[5]=0;hook_AppTracker_SetWorldMatrixInverse(NULL,m);
 assert(captured_camera_rotation_version==1&&camera_world_to_view_direction(world,view));
 float y[3]={m[1],m[5],m[9]};assert(fabsf(view[0]*y[0]+view[1]*y[1]+view[2]*y[2]+1)<1e-6f);
 DWORD t=captured_camera_change_tick;physx_simulation_serial++;
 gravity_sample_live(&s,(void*)1,a,1,t+48,out);assert(!s.accepted);
 physx_simulation_serial++;gravity_sample_live(&s,(void*)1,a,1,t+64,out);assert(s.accepted);
 puts("PASS: actual camera hook covers pan/orbit; invariant world projection; wrapper resumes after quiet and confirmation");
}
static void rates_and_filter(void){
 const int rates[]={20,30,60,144};
 for(int r=0;r<4;r++){
  int hz=rates[r];gravity_sample_t s={0};float out[3],v[3];
  for(int i=0;i<=hz;i++){
   float angle=(float)i/hz;v[0]=sinf(angle);v[1]=-cosf(angle);v[2]=0;
   sample(&s,v,i,1000+(uint32_t)(1000.0*i/hz),0,1000,out);
   if(i) assert(s.accepted&&fabsf(out[0]-sinf((float)(i-1)/hz))<1e-6f);
  }
  body_chain_person_state_t b={0};b.gravity_sample.trusted_valid=1;
  update_body_chain_gravity_filter(&b,1.0f/hz);b.gravity_drive[0]=1;
  for(int i=0;i<hz;i++)update_body_chain_gravity_filter(&b,1.0f/hz);
  assert(fabsf(b.gravity_drive_filtered[0]-(1-expf(-10)))<1e-6f);
 }
 body_chain_person_state_t b={0};update_body_chain_gravity_filter(&b,.016f);assert(!b.gravity_drive_filtered_valid);
 b.gravity_sample.trusted_valid=1;update_body_chain_gravity_filter(&b,.016f);
 physics_environment_cfg.gravity_response_ms=0;b.gravity_drive[0]=1;
 update_body_chain_gravity_filter(&b,.016f);assert(b.gravity_drive_filtered[0]==1);
 physics_environment_cfg.gravity_max_degrees_per_second=90;b.gravity_drive[0]=-1;
 update_body_chain_gravity_filter(&b,.016f);assert(fabsf(b.gravity_drive_filtered[0]-.984f)<1e-6f);
 puts("PASS: smooth poses at 20/30/60/144 Hz; time-consistent smoothing; startup and zero-ms speed cap");
}
static void sidecars(void){
 physx_chain_t c={0};float a[3]={0,-1,0},b[3]={0,1,0},out[3];DWORD t=captured_camera_change_tick+100;
 physx_simulation_serial++;assert(addon_chain_camera_safe_gravity_drive(&c,t,a,1,(void*)3,out));
 assert(c.addon_gravity_camera_hold_active&&!c.addon_gravity_trusted_valid&&out[1]==0);
 physx_simulation_serial++;addon_chain_camera_safe_gravity_drive(&c,t+16,a,1,(void*)3,out);assert(c.addon_gravity_trusted_valid&&out[1]==-1);
 physx_simulation_serial++;addon_chain_camera_safe_gravity_drive(&c,t+32,b,1,(void*)3,out);assert(c.addon_gravity_camera_hold_active&&out[1]==-1);
 physx_simulation_serial++;addon_chain_camera_safe_gravity_drive(&c,t+48,NULL,0,(void*)3,out);assert(c.addon_gravity_camera_hold_active&&out[1]==-1);
 physx_simulation_serial++;addon_chain_camera_safe_gravity_drive(&c,t+64,b,1,(void*)3,out);assert(c.addon_gravity_camera_hold_active);
 physx_simulation_serial++;addon_chain_camera_safe_gravity_drive(&c,t+80,b,1,(void*)3,out);assert(!c.addon_gravity_camera_hold_active&&out[1]==1);
 puts("PASS: sidecar warmup cannot switch fallback frames; invalid samples retain trusted force and reconfirm");
}
int main(void){protocol();cameras();rates_and_filter();sidecars();return 0;}
'''
build=root/'build';build.mkdir(exist_ok=True)
c=build/'gravity_response_test.c';c.write_text(source)
gcc=Path(r'C:\msys64\mingw32\bin\gcc.exe');env=dict(os.environ,PATH=str(gcc.parent)+os.pathsep+os.environ['PATH'])
exe=build/'gravity_response_test.exe'
subprocess.run([str(gcc),'-m32','-O2','-Wall','-Wextra','-Werror','-Wno-unused-function','-static-libgcc','-o',str(exe),str(c)],env=env,check=True)
subprocess.run([str(exe)],env=env,check=True,timeout=60)
