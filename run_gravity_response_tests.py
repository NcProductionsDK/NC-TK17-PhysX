"""Exercise production gravity sampling, camera capture, filtering and sidecar routing."""
from pathlib import Path
import os,re,subprocess
root=Path(__file__).resolve().parent
main=(root/'NC-TK17-PhysX.c').read_text()
hooks=(root/'physx_hooks_core.c').read_text()
sidecar=(root/'physx_sidecar.c').read_text()
physics=(root/'physx_physics.c').read_text()
relative=(root/'physx_relative_gravity.c').read_text()
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
static float vec3_dot(const float *a,const float *b){return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];}
static int physx_vec3_sane_limit(const float *v,float limit){return isfinite(v[0])&&isfinite(v[1])&&isfinite(v[2])&&fabsf(v[0])<=limit&&fabsf(v[1])<=limit&&fabsf(v[2])<=limit;}
static struct {int gravity_probe_camera_quiet_ms,body_chain_camera_quarantine_ms;float gravity_response_ms,gravity_max_degrees_per_second;
 int gravity_horizontal_basis_offset,gravity_vertical_basis_offset,gravity_horizontal_secondary_basis_offset;
 float gravity_horizontal_basis_sign,gravity_vertical_basis_sign,gravity_horizontal_secondary_basis_sign;
 int gravity_apply_to_body_chain,gravity_dynamic_body_basis,body_chain_camera_relative_orientation;
 float world_gravity[3];
} physics_environment_cfg={10,0,100,0,0x078,0x088,0x098,1,1,1,1,1,1,{0,-1,0}};
typedef struct {gravity_sample_t gravity_sample;float gravity_drive[3],gravity_drive_filtered[3];int gravity_drive_filtered_valid,gravity_camera_hold_active;} body_chain_person_state_t;
typedef struct {char name[32];int addon_simulated_target;void *raw_object,*object,*s_raw_object,*s_object;} physx_target_t;
typedef struct {int unused;} physx_sidecar_t;
typedef struct {gravity_sample_t addon_gravity_sample,addon_gravity_reference_sample;int addon_gravity_camera_hold_active,addon_gravity_camera_release_active,addon_gravity_trusted_valid;float addon_gravity_trusted_drive[3];
 int gravity_enabled,target_count;physx_target_t targets[1];} physx_chain_t;
'''
source+=function(main,'camera_rotation_delta')+function(main,'camera_world_to_view_direction')
source+=function(hooks,'hook_AppTracker_SetWorldMatrixInverse')+function(main,'gravity_sample_live')
source+=function(main,'update_body_chain_gravity_filter')+function(sidecar,'addon_chain_camera_safe_gravity_drive')
source+=function(physics,'body_chain_rebase_confirmed_gravity')
for name in ['body_chain_mat3_inverse','body_chain_mat3_multiply']:
    source+=function(main,name)
for name in ['gravity_normalize_basis','gravity_sample_relative']:
    source+=function(relative,name)
for name in ['addon_basis_rows_orthonormal_enough','addon_normalize_basis_rows',
             'addon_gravity_basis_row_from_offset','addon_chain_project_direction_basis',
             'addon_chain_relative_gravity_drive']:
    source+=function(sidecar,name)
source+=r'''
/* Engine lookup seam: exercise the real parent routing with supplied live
   matrices, including absent/invalid TRS fallback. */
static void *test_trs;
static int body_chain_read_mat3_rows(void *raw,float *out){if(!raw)return 0;memcpy(out,raw,9*sizeof(float));return 1;}
static int addon_parent_name_can_use_body_drive(const char *name){(void)name;return 0;}
static void *addon_effective_parent_cached_runtime_raw(physx_sidecar_t *sc,physx_chain_t *c,physx_target_t *p,DWORD t,const char **r){(void)sc;(void)c;(void)t;(void)r;return p->raw_object;}
static int addon_parent_camera_relative_person(physx_sidecar_t *sc,physx_chain_t *c,const char *r,char *out,size_t n){(void)sc;(void)c;(void)r;(void)n;strcpy(out,"Person01");return 1;}
static void *addon_parent_camera_relative_trs_raw(physx_chain_t *c,const char *p,DWORD t){(void)c;(void)p;(void)t;return test_trs;}
static void addon_chain_note_body_root_person(physx_chain_t *c,const char *p,DWORD t,const char *reason){(void)c;(void)p;(void)t;(void)reason;}
'''
source+=function(sidecar,'addon_chain_parent_gravity_drive')
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
static void pose_rebase(void){
 const float old[3]={-.05263f,0,-.99861f},pose[3]={-.98711f,-.04085f,.15475f};
 const int rates[]={30,60,90,144};
 physics_environment_cfg.gravity_response_ms=100;
 physics_environment_cfg.gravity_max_degrees_per_second=500;
 for(int r=0;r<4;r++){
  body_chain_person_state_t b={0};float out[3];uint32_t step=1000/rates[r];
  sample(&b.gravity_sample,old,1,1000,0,1000,out);
  sample(&b.gravity_sample,old,2,1000+step,0,1000,out);
  memcpy(b.gravity_drive,out,sizeof(out));
  body_chain_rebase_confirmed_gravity(&b);update_body_chain_gravity_filter(&b,1.0f/rates[r]);
  assert(!memcmp(b.gravity_drive_filtered,old,sizeof(out))&&!b.gravity_sample.accepted_jump);
  sample(&b.gravity_sample,pose,3,1000+2*step,0,1000,out);
  assert(!b.gravity_sample.accepted);
  body_chain_rebase_confirmed_gravity(&b);
  assert(b.gravity_drive_filtered_valid&&!memcmp(b.gravity_drive_filtered,old,sizeof(out)));
  sample(&b.gravity_sample,pose,4,1000+3*step,0,1000,out);
  assert(b.gravity_sample.accepted&&b.gravity_sample.accepted_jump);
  memcpy(b.gravity_drive,out,sizeof(out));
  body_chain_person_state_t legacy=b;update_body_chain_gravity_filter(&legacy,1.0f/rates[r]);
  assert(fabsf(legacy.gravity_drive_filtered[2]-pose[2])>.5f);
  /* Confirmation can arrive while the independent body hold is still active. */
  b.gravity_camera_hold_active=1;body_chain_rebase_confirmed_gravity(&b);
  assert(b.gravity_drive_filtered_valid&&b.gravity_sample.accepted_jump);
  sample(&b.gravity_sample,pose,5,1000+4*step,0,1000,out);
  b.gravity_camera_hold_active=0;body_chain_rebase_confirmed_gravity(&b);
  update_body_chain_gravity_filter(&b,1.0f/rates[r]);
  assert(!memcmp(b.gravity_drive_filtered,pose,sizeof(out))&&!b.gravity_sample.accepted_jump);
  /* A repeated substep must not reseed the same event again. */
  sample(&b.gravity_sample,pose,5,1000+4*step,0,1000,out);
  body_chain_rebase_confirmed_gravity(&b);assert(b.gravity_drive_filtered_valid);
  /* Rebinding confirms a fresh sample before replacing a cached old filter. */
  gravity_sample_update(&b.gravity_sample,2,old,1,6,1000+5*step,0,1,1000,10,out);
  body_chain_rebase_confirmed_gravity(&b);assert(b.gravity_drive_filtered_valid);
  gravity_sample_update(&b.gravity_sample,2,old,1,7,1000+6*step,0,1,1000,10,out);
  memcpy(b.gravity_drive,out,sizeof(out));body_chain_rebase_confirmed_gravity(&b);
  update_body_chain_gravity_filter(&b,1.0f/rates[r]);
  assert(!memcmp(b.gravity_drive_filtered,old,sizeof(out)));
 }
 /* Gradual animation retains smoothing, even across a large total rotation. */
 for(int r=0;r<4;r++){
  gravity_sample_t s={0};float out[3],v[3];int hz=rates[r];
  for(int i=0;i<=2*hz;i++){
   float angle=(float)i/hz;v[0]=sinf(angle);v[1]=-cosf(angle);v[2]=0;
   sample(&s,v,i,1000+(uint32_t)(1000.0*i/hz),0,1000,out);
   if(i==1){assert(s.accepted_jump);s.accepted_jump=0;}
   if(i>1)assert(s.accepted&&!s.accepted_jump);
  }
 }
 puts("PASS: logged pose jump rebases confirmed gravity at 30/60/90/144 Hz; holds, substeps, rebinding and gradual animation preserved");
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
static void rot_x(float a,float *m){
 memset(m,0,9*sizeof(float));m[0]=1;m[4]=m[8]=cosf(a);m[5]=sinf(a);m[7]=-sinf(a);
}
static void rot_y(float a,float *m){
 memset(m,0,9*sizeof(float));m[4]=1;m[0]=m[8]=cosf(a);m[2]=-sinf(a);m[6]=sinf(a);
}
static void close3(const float *a,const float *b,float tolerance){
 for(int i=0;i<3;i++)assert(fabsf(a[i]-b[i])<tolerance);
}
/* Construct parent=local*placement*view and TRS=placement*view. Independently
   compute the correct force from local*placement, without any camera basis. */
static void relative_frame(physx_chain_t *chain,float pose,float placement_angle,
 float camera_angle,float camera_yaw,float camera_scale,int stale_capture,
 DWORD tick,const void *trs_source,float *out,float *expected){
 float local[9],placement[9],camera[9],yaw[9],view[9],trs[9],parent[9],world_parent[9];
 float world[3]={0,-1,0},gravity_view[3];
 rot_x(pose,local);rot_x(placement_angle,placement);
 rot_x(camera_angle,camera);rot_y(camera_yaw,yaw);
 body_chain_mat3_multiply(camera,yaw,view);
 body_chain_mat3_multiply(local,placement,world_parent);
 body_chain_mat3_multiply(placement,view,trs);
 body_chain_mat3_multiply(world_parent,view,parent);
 for(int i=0;i<9;i++){trs[i]*=camera_scale;parent[i]*=camera_scale;}
 if(!stale_capture){
  /* captured camera is the inverse (transpose) of the view rotation. */
  identity(captured_camera_inverse);
  for(int r=0;r<3;r++)for(int c=0;c<3;c++)captured_camera_inverse[r*4+c]=view[c*3+r];
 }
 captured_camera_inverse_valid=1;
 assert(camera_world_to_view_direction(world,gravity_view));
 physx_simulation_serial++;
 assert(addon_chain_relative_gravity_drive(chain,(void*)101,trs_source,
  parent,trs,gravity_view,tick,out));
 addon_chain_project_direction_basis(world,world_parent,expected);
}
static void relative_gravity(void){
 const int rates[]={30,60,90,144};
 for(int r=0;r<4;r++){
  physx_chain_t chain={0};float out[3],expected[3],previous[3]={0},stationary[3];
  DWORD tick=10000,step=1000/rates[r];
  captured_camera_change_tick=0;captured_camera_version=0;
  for(int i=0;i<4;i++,tick+=step)
   relative_frame(&chain,0,.7f,0,0,1,0,tick,(void*)102,out,expected);
  assert(chain.addon_gravity_trusted_valid);close3(out,expected,.0001f);
  memcpy(stationary,out,sizeof(out));
  /* Continuous camera orbit/pan/zoom, including a stale captured camera.
     Stationary model gravity must not move or wait for the camera to stop. */
  for(int i=1;i<=120;i++,tick+=step){
   captured_camera_version++;captured_camera_change_tick=tick;
   relative_frame(&chain,0,.7f,i*.02f,i*.013f,1+i*.004f,1,tick,(void*)102,out,expected);
   assert(!chain.addon_gravity_camera_hold_active);close3(out,stationary,.0001f);
  }
  /* Real head motion while the camera keeps moving; expected one-frame
     confirmation delay, rather than holding the old pose for the whole drag. */
  memcpy(previous,expected,sizeof(previous));
  for(int i=1;i<=120;i++,tick+=step){
   captured_camera_version++;captured_camera_change_tick=tick;
   relative_frame(&chain,i*.015f,.7f,i*.03f,i*.017f,1,1,tick,(void*)102,out,expected);
   assert(!chain.addon_gravity_camera_hold_active);close3(out,previous,.0001f);
   memcpy(previous,expected,sizeof(previous));
  }
  assert(fabsf(out[1]-stationary[1])>.5f);
  /* A new global placement is relearned once the camera is quiet. */
  tick+=200;
  for(int i=0;i<5;i++,tick+=step)
   relative_frame(&chain,0,-.4f,0,0,1,0,tick,(void*)102,out,expected);
  close3(out,expected,.0001f);
  /* One-frame bad parent/TRS pairing cannot become a trusted force. */
  memcpy(stationary,out,sizeof(out));
  captured_camera_version++;captured_camera_change_tick=tick;
  relative_frame(&chain,1.0f,-.4f,1,0,1,1,tick,(void*)102,out,expected);tick+=step;
  close3(out,stationary,.0001f);assert(chain.addon_gravity_camera_hold_active);
  for(int i=0;i<3;i++,tick+=step){
   captured_camera_version++;captured_camera_change_tick=tick;
   relative_frame(&chain,0,-.4f,1,0,1,1,tick,(void*)102,out,expected);
   close3(out,stationary,.0001f);
  }
  /* Rebinding during camera motion must not reuse the old placement. */
  relative_frame(&chain,0,0,1,0,1,1,tick,(void*)103,out,expected);
  assert(!chain.addon_gravity_trusted_valid&&chain.addon_gravity_camera_hold_active);
  float zero[3]={0};close3(out,zero,.0001f);
 }
 puts("PASS: sidecar camera-only invariance, tracking during continuous camera movement, stale camera capture, placement, spike rejection and rebinding at 30/60/90/144 Hz");
}
static void parent_routing(void){
 physx_chain_t chain={0};float parent[9],trs[9],out[3],old[3];DWORD tick=20000;
 chain.gravity_enabled=chain.target_count=1;strcpy(chain.targets[0].name,"head_joint02");
 chain.targets[0].raw_object=parent;test_trs=trs;rot_x(0,trs);rot_x(0,parent);
 identity(captured_camera_inverse);captured_camera_inverse_valid=1;captured_camera_change_tick=0;
 for(int i=0;i<4;i++,tick+=16){physx_simulation_serial++;assert(addon_chain_parent_gravity_drive(NULL,&chain,tick,out));}
 assert(chain.addon_gravity_trusted_valid);memcpy(old,out,sizeof(old));
 rot_x(.5f,parent);
 for(int i=0;i<3;i++,tick+=16){
  physx_simulation_serial++;captured_camera_version++;captured_camera_change_tick=tick;
  assert(addon_chain_parent_gravity_drive(NULL,&chain,tick,out));
 }
 assert(!chain.addon_gravity_camera_hold_active&&fabsf(out[2]-old[2])>.4f);
 /* Same frame/substep cannot advance the force, even if raw rows change. */
 memcpy(old,out,sizeof(old));rot_x(1,parent);
 assert(addon_chain_parent_gravity_drive(NULL,&chain,tick,out));close3(out,old,.0001f);
 /* Without TRS, the old guarded path must remain active while orbiting. */
 test_trs=NULL;physx_simulation_serial++;captured_camera_version++;captured_camera_change_tick=tick;
 assert(addon_chain_parent_gravity_drive(NULL,&chain,tick,out));
 assert(chain.addon_gravity_camera_hold_active);close3(out,old,.0001f);
 /* Invalid TRS likewise takes the guarded fallback, not a zero/NaN force. */
 test_trs=trs;memset(trs,0,sizeof(trs));physx_simulation_serial++;
 assert(addon_chain_parent_gravity_drive(NULL,&chain,tick,out));close3(out,old,.0001f);
 /* Authored channel mapping is applied after reconstructing room gravity. */
 rot_x(0,trs);rot_x(.5f,parent);tick+=200;
 physics_environment_cfg.gravity_horizontal_basis_offset=0x098;
 physics_environment_cfg.gravity_horizontal_basis_sign=-1;
 physics_environment_cfg.gravity_vertical_basis_sign=0;
 for(int i=0;i<5;i++,tick+=16){physx_simulation_serial++;assert(addon_chain_parent_gravity_drive(NULL,&chain,tick,out));}
 assert(fabsf(out[0]+sinf(.5f))<.0001f&&out[1]==0);
 puts("PASS: production parent route, same-frame reuse, missing/singular TRS fallback and signed/disabled authored channels");
}
int main(void){protocol();cameras();rates_and_filter();pose_rebase();sidecars();relative_gravity();parent_routing();return 0;}
'''
build=root/'build';build.mkdir(exist_ok=True)
c=build/'gravity_response_test.c';c.write_text(source)
gcc=Path(r'C:\msys64\mingw32\bin\gcc.exe');env=dict(os.environ,PATH=str(gcc.parent)+os.pathsep+os.environ['PATH'])
exe=build/'gravity_response_test.exe'
subprocess.run([str(gcc),'-m32','-O2','-Wall','-Wextra','-Werror','-Wno-unused-function','-static-libgcc','-o',str(exe),str(c)],env=env,check=True)
subprocess.run([str(exe)],env=env,check=True,timeout=60)
