"""Production single-bone math and translation/contact integration fixtures."""
from pathlib import Path
import os,re,subprocess
ROOT=Path(__file__).resolve().parent
def function(file,name):
    s=(ROOT/file).read_text();m=re.search(r'^static [^;{}]*\b'+name+r'\([^;{}]*\)\s*\{',s,re.M)
    if not m: raise ValueError(name)
    p,d=m.end(),1
    while d:d+=(s[p]=='{')-(s[p]=='}');p+=1
    return s[m.start():p]+'\n'
source=r'''
#include <assert.h>
#include <stdio.h>
#include <stdint.h>
#include "../physx_single_bone_contact.h"
#include "../physx_body_motion.h"
#include "../physx_contact_math.h"
typedef unsigned long DWORD;typedef long LONG;
typedef unsigned char BYTE;
static int ptr_readable(const void *p,size_t n){(void)n;return p!=NULL;}
static void body_chain_mark_runtime_transform_dirty(void *p){(void)p;}
static struct {int debug;} defaults_cfg;
static LONG named_node_generation=1;
static DWORD physx_simulation_serial=1;
static LONG InterlockedCompareExchange(LONG *p,LONG a,LONG b){(void)a;(void)b;return *p;}
static void log_line(const char *fmt,...){(void)fmt;}
static float physx_vec3_len(const float x[3]){return sqrtf(single_bone_dot(x,x));}
static float physx_clampf(float x,float lo,float hi){return fmaxf(lo,fminf(hi,x));}
static int sane_probe_float(float x){return isfinite(x);}
static void body_chain_transform_row_vector3(const float v[3],const float m[9],float x[3]){
 for(int a=0;a<3;a++) x[a]=v[0]*m[a]+v[1]*m[3+a]+v[2]*m[6+a];
}
#define BODY_COLLIDER_NODE_COUNT 5
#define BODY_COLLIDER_BREAST_L 0
#define BODY_COLLIDER_BREAST_R 1
#define BODY_COLLIDER_BUTT_L 2
#define BODY_COLLIDER_BUTT_R 3
#define BODY_CHAIN_COLLISION_SCOPE_FULL_BODY_ALL 1
typedef struct {int ready,basis_valid,valid[5],contact_frame_valid;float view_position[5][3],contact_view_to_world[9],contact_world_origin[3];} body_chain_collider_person_state_t;
static body_chain_collider_person_state_t body_chain_collider_states[4];
static struct {int enabled,breasts_collision_enabled,butt_collision_enabled;float response_strength,collision_slop;} body_chain_collider_cfg={1,1,1,1,0};
typedef struct {int room_collision_enabled,collision_scope,output_offset,override_animation;} body_chain_physics_config_t;
static body_chain_physics_config_t breasts_physics_cfg={0,0,0x06c,0},butt_physics_cfg={0,0,0x06c,0};
static int breasts_physics_bone_translation_enabled,butt_physics_bone_translation_enabled;
static int breasts_physics_bone_translation_offset=0x0e8,butt_physics_bone_translation_offset=0x0e8;
typedef struct {float bone_translation[2][3],bone_translation_velocity[2][3],contact_previous_world[2][3];
 int initialized,output_applied,bone_translation_applied,contact_translation_active,animation_rows_valid;
 void *source_joint_raw[2],*animation_joint_raw[2];
 float source_handoff[2][3],source_translation_handoff[2][3],rest_rotation[2][3],rotation[2][3],animation_rows[2][9];
 DWORD contact_previous_tick[2],contact_log_tick;LONG contact_previous_generation[2];void *contact_previous_parent[2],*translation_parent_joint_raw[2];} breasts_physics_person_state_t;
static float captured_camera_inverse[16];
static int body_chain_read_mat3_rows(void *p,float m[9]){if(!p)return 0;memcpy(m,p,sizeof(float)*9);return 1;}
static int body_chain_collision_scope_valid(int s){return s==0||s==1;}
static int body_chain_collision_scope_all_persons(int s){return s==1;}
static int paired_bone_physics_collider_node_in_scope(int n,int s){(void)s;return n>=0&&n<5;}
static int butt_physics_owner_attachment_node(int n){return n<4;}
static int breasts_physics_owner_attachment_node(int n){return n<4;}
static float body_chain_collider_visual_radius_for_node(int n){(void)n;return .1f;}
static int room_enabled,room_corner,sweep_calls;static float floor_height,sweep_start[3];
static int room_collision_is_enabled(void){return room_enabled;}
static int single_bone_room_contacts(const float center[3],float radius,physx_contact_set_t *out){
 float up[3]={0,1,0},right[3]={1,0,0};memset(out,0,sizeof(*out));
 physx_contact_store(out,up,floor_height+radius-center[1]);
 if(room_corner)physx_contact_store(out,right,radius-center[0]);return out->count>0;
}
static int room_collision_resolve_swept_sphere(const float start[3],const float end[3],float radius,float correction[3],int *mesh){
 (void)mesh;sweep_calls++;memcpy(sweep_start,start,sizeof(float)*3);memset(correction,0,sizeof(float)*3);
 if(start[1]>=floor_height+radius-1e-6f && end[1]<floor_height+radius){correction[1]=floor_height+radius-end[1];return 1;}return 0;
}
'''
source+=function('NC-TK17-PhysX.c','body_chain_mat3_inverse')
source+=function('NC-TK17-PhysX.c','body_chain_mat3_multiply')
source+=r'''
static int body_chain_vec3_sane_limit(const float v[3],float limit){for(int a=0;a<3;a++)if(!isfinite(v[a])||fabsf(v[a])>limit)return 0;return 1;}
'''
source+=function('physx_collision_frame.c','body_collision_view_to_world')
source+=(ROOT/'physx_single_bone_contact.c').read_text()
source+=function('physx_physics.c','breasts_physics_apply_output')
source+=function('physx_butt.c','butt_physics_apply_output')
source+=r'''
static void near(float a,float b){assert(fabsf(a-b)<2e-5f);}
static void math_checks(void){
 float lo[3]={-.05f,-.05f,-.05f},hi[3]={.05f,.05f,.05f},x[3]={0},n[3]={1,0,0};single_bone_contacts_t c={0};
 for(int i=0;i<10;i++)single_bone_store(&c,n,.02f);assert(c.count==1);
 single_bone_project(&c,lo,hi,x);near(x[0],.02f);near(x[1],0);
 n[0]=0;n[1]=1;single_bone_store(&c,n,.03f);single_bone_project(&c,lo,hi,x);near(x[0],.02f);near(x[1],.03f);
 float v[3]={-1,-2,3};single_bone_velocity(&c,x,hi,v);near(v[0],0);near(v[1],0);near(v[2],3);
 v[0]=1;v[1]=2;single_bone_velocity(&c,x,hi,v);near(v[0],1);near(v[1],2);
 n[1]=0;n[0]=-1;single_bone_store(&c,n,.04f);single_bone_project(&c,lo,hi,x);
 for(int a=0;a<3;a++)assert(isfinite(x[a])&&fabsf(x[a])<=.05001f);
 memset(&c,0,sizeof(c));n[0]=1;single_bone_store(&c,n,.2f);single_bone_project(&c,lo,hi,x);near(x[0],.05f);assert(single_bone_violation(&c,x)>.02f);
 float m[9]={0,2,0,-3,0,0,0,0,-.5f},world[3]={1,0,0},zero[3]={0};
 memset(&c,0,sizeof(c));single_bone_world_plane(&c,m,world,.03f,zero);memset(x,0,sizeof(x));
 single_bone_project(&c,lo,hi,x);float actual[3];body_chain_transform_row_vector3(x,m,actual);near(actual[0],.03f);near(actual[1],0);
 puts("PASS: duplicate/corner contacts, inward velocity removal, sliding/release, impossible limits and rotated/scaled/mirrored plane conversion");
}
static void reset(breasts_physics_person_state_t *s,float parent[9],int butt){
 memset(s,0,sizeof(*s));memset(body_chain_collider_states,0,sizeof(body_chain_collider_states));
 memset(&single_bone_published,0,sizeof(single_bone_published));
 for(int p=0;p<4;p++) {body_chain_collider_states[p].contact_frame_valid=1;
  body_chain_collider_states[p].contact_view_to_world[0]=body_chain_collider_states[p].contact_view_to_world[4]=body_chain_collider_states[p].contact_view_to_world[8]=1;}
 memset(captured_camera_inverse,0,sizeof(captured_camera_inverse));captured_camera_inverse[0]=captured_camera_inverse[5]=captured_camera_inverse[10]=captured_camera_inverse[15]=1;
 memset(parent,0,9*sizeof(float));parent[0]=parent[4]=parent[8]=1;
 s->translation_parent_joint_raw[0]=s->translation_parent_joint_raw[1]=parent;
 body_chain_collider_states[0].ready=body_chain_collider_states[0].basis_valid=1;
 body_chain_collider_states[0].valid[butt?2:0]=1;
 room_enabled=room_corner=0;floor_height=0;body_chain_collider_cfg.enabled=1;body_chain_collider_cfg.response_strength=1;
}
static void integration_checks(void){
 unsigned rates[]={50,33,16,7,11};
 for(int butt=0;butt<2;butt++)for(int rate=0;rate<5;rate++){
  breasts_physics_person_state_t s;float parent[9],target[2][3]={{0}},push[2][3],limit[3]={.05f,.05f,.05f};
  body_chain_physics_config_t cfg={1,1,0,0};reset(&s,parent,butt);room_enabled=room_corner=1;
  int node=butt?2:0;DWORD now=1000;
  for(int frame=0;frame<800;frame++){
   float *sample=body_chain_collider_states[0].view_position[node];
   sample[0]=.08f+s.bone_translation[0][0];sample[1]=.08f+s.bone_translation[0][1];sample[2]=s.bone_translation[0][2];
   unsigned elapsed=rate==4?(frame%2?23:11):rates[rate];now+=elapsed;
   single_bone_contact_step(0,butt,&s,&cfg,now,elapsed,target,200,6,limit,push);
   near(s.bone_translation[0][0],.02f);near(s.bone_translation[0][1],.02f);
   near(s.bone_translation_velocity[0][0],0);near(s.bone_translation_velocity[0][1],0);
  }
  /* Moving floor follows geometrically without recovery becoming launch velocity. */
  floor_height=.01f;single_bone_contact_step(0,butt,&s,&cfg,now+16,16,target,200,6,limit,push);
  near(s.bone_translation[0][1],.03f);near(s.bone_translation_velocity[0][1],0);
  room_enabled=0;
  for(int frame=0;frame<600;frame++)single_bone_contact_step(0,butt,&s,&cfg,now+32+frame*16,16,target,200,6,limit,push);
  near(s.bone_translation[0][1],0);
 }
 puts("PASS: production translation/contact loop rests in room corners, follows moving support and releases at 20/30/60/144 Hz and uneven intervals");
}
static void body_and_history(void){
 breasts_physics_person_state_t s;float parent[9],target[2][3]={{0}},push[2][3],limit[3]={.05f,.05f,.05f};
 body_chain_physics_config_t cfg={0,1,0,0};reset(&s,parent,0);
 body_chain_collider_states[1].ready=body_chain_collider_states[1].basis_valid=1;
 body_chain_collider_states[1].valid[4]=1;body_chain_collider_states[1].view_position[4][0]=-.18f;
 single_bone_contact_step(0,0,&s,&cfg,1000,16,target,200,6,limit,push);
 near(s.bone_translation[0][0],.02f);near(s.bone_translation[0][1],0);near(s.bone_translation[0][2],0);
 reset(&s,parent,0);room_enabled=1;cfg.room_collision_enabled=1;
 body_chain_collider_states[0].view_position[0][1]=.08f;
 single_bone_contact_step(0,0,&s,&cfg,1000,16,target,200,6,limit,push);
 near(s.contact_previous_world[0][1],.08f); /* Actual sampled center, not .1. */
 body_chain_collider_states[0].view_position[0][1]=.1f;
 single_bone_contact_step(0,0,&s,&cfg,1016,16,target,200,6,limit,push);near(sweep_start[1],.08f);
 int calls=sweep_calls;single_bone_contact_step(0,0,&s,&cfg,2000,984,target,200,6,limit,push);assert(sweep_calls==calls);
 named_node_generation++;calls=sweep_calls;single_bone_contact_step(0,0,&s,&cfg,2016,16,target,200,6,limit,push);assert(sweep_calls==calls);
 puts("PASS: production body contact moves along correct axis; room history records actual samples and rejects stale/generation-changed history");
}
static void output_checks(void){
 for(int butt=0;butt<2;butt++) {
  breasts_physics_person_state_t s={0};float raw[2][80]={{0}};
  s.initialized=s.contact_translation_active=1;
  for(int side=0;side<2;side++){
   s.source_joint_raw[side]=raw[side];s.source_translation_handoff[side][0]=.3f;
   s.bone_translation[side][0]=.02f;
  }
  assert(butt?butt_physics_apply_output(&s,0,0):breasts_physics_apply_output(&s,0,0));
  near(*(float*)((BYTE*)raw[0]+0x0e8),.32f);assert(s.bone_translation_applied);
  assert(butt?butt_physics_apply_output(&s,1,0):breasts_physics_apply_output(&s,1,0));
  near(*(float*)((BYTE*)raw[0]+0x0e8),.3f);assert(!s.bone_translation_applied);
 }
 puts("PASS: actual breast/butt output publishes contact-only translation with ordinary translation disabled; handoff restoration exact");
}
static void camera_and_body_rest(void){
 for(int butt=0;butt<2;butt++)for(int camera=0;camera<2;camera++){
  breasts_physics_person_state_t s;float parent[9],target[2][3]={{0}},push[2][3],limit[3]={.05f,.05f,.05f};
  body_chain_physics_config_t cfg={0,1,0,0};reset(&s,parent,butt);int node=butt?2:0;
  if(camera){
   captured_camera_inverse[0]=0;captured_camera_inverse[1]=1;captured_camera_inverse[4]=-1;captured_camera_inverse[5]=0;
   for(int p=0;p<4;p++) {
    float *m=body_chain_collider_states[p].contact_view_to_world;
    m[0]=0;m[1]=1;m[3]=-1;m[4]=0;
   }
   /* Same world parent, expressed through the inverse camera rotation. */
   parent[0]=0;parent[1]=-1;parent[3]=1;parent[4]=0;
  }
  body_chain_collider_states[1].ready=body_chain_collider_states[1].basis_valid=1;
  body_chain_collider_states[1].valid[4]=1;body_chain_collider_states[1].view_position[4][camera?1:0]=camera?.18f:-.18f;
  for(int frame=0;frame<600;frame++){
   body_chain_transform_row_vector3(s.bone_translation[0],parent,body_chain_collider_states[0].view_position[node]);
   single_bone_contact_step(0,butt,&s,&cfg,1000+16*frame,16,target,200,6,limit,push);
   near(s.bone_translation[0][0],.02f);near(s.bone_translation[0][1],0);near(s.bone_translation_velocity[0][0],0);
  }
  /* Combined body + room support and disabled-body switch semantics. */
  room_enabled=1;cfg.room_collision_enabled=1;floor_height=-.08f;
  single_bone_contact_step(0,butt,&s,&cfg,11000,16,target,200,6,limit,push);
  near(s.bone_translation[0][0],.02f);near(s.bone_translation[0][1],.02f);
  body_chain_collider_cfg.response_strength=0;room_enabled=0;
  for(int frame=0;frame<500;frame++)single_bone_contact_step(0,butt,&s,&cfg,12000+frame*16,16,target,200,6,limit,push);
  near(s.bone_translation[0][0],0);
 }
 puts("PASS: production body sphere rests without residual overlap, camera-invariant mapping, combined body/room supports, strength-zero release");
}
static void paired_support(void){
 breasts_physics_person_state_t a,b;float parent[9],target[2][3]={{0}},push[2][3],limit[3]={.05f,.05f,.05f};
 body_chain_physics_config_t cfg={0,1,0,0};reset(&a,parent,0);memset(&b,0,sizeof(b));
 b.translation_parent_joint_raw[0]=b.translation_parent_joint_raw[1]=parent;
 body_chain_collider_states[1].ready=body_chain_collider_states[1].basis_valid=1;
 body_chain_collider_states[1].valid[0]=1;body_chain_collider_states[1].view_position[0][0]=-.18f;
 single_bone_contact_step(0,0,&a,&cfg,1000,16,target,200,6,limit,push);
 single_bone_contact_step(1,0,&b,&cfg,1000,16,target,200,6,limit,push);
 near(a.bone_translation[0][0],.02f);near(b.bone_translation[0][0],0);
 near(body_chain_collider_states[0].view_position[0][0],0); /* Chain/global proxies untouched. */
 puts("PASS: paired bodies see prior single-bone output within the frame, without double recovery or changes to global colliders");
}
int main(void){math_checks();integration_checks();body_and_history();output_checks();camera_and_body_rest();paired_support();return 0;}
'''
build=ROOT/'build';build.mkdir(exist_ok=True)
c=build/'single_bone_contact_test.c';c.write_text(source)
gcc=Path(r'C:\msys64\mingw32\bin\gcc.exe');env=dict(os.environ,PATH=str(gcc.parent)+os.pathsep+os.environ['PATH'])
exe=build/'single_bone_contact_test.exe'
subprocess.run([str(gcc),'-m32','-O2','-Wall','-Wextra','-Werror','-Wno-unused-function','-Wno-misleading-indentation','-static-libgcc','-o',str(exe),str(c)],env=env,check=True)
subprocess.run([str(exe)],env=env,check=True,timeout=60)
