"""Production settings bridge with native F32 dispatch and Windows INI fixtures."""
from pathlib import Path
import os
import re
import subprocess

root = Path(__file__).resolve().parent
config = (root / 'physx_config.c').read_text()
main = (root / 'NC-TK17-PhysX.c').read_text()

def function(name, code=config):
    match = re.search(r'^static [^;{}]*\b' + name + r'\([^;{}]*\)\s*\{', code, re.M)
    pos, depth = match.end(), 1
    while depth:
        depth += (code[pos] == '{') - (code[pos] == '}')
        pos += 1
    return code[match.start():pos] + '\n'

source = r'''
#include <windows.h>
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#define THISCALL __attribute__((thiscall))
#define SCRIPT_OBJECT_META_BACK_OFFSET 0x18
static char config_path[MAX_PATH*4];
static int ini_writes;
static BOOL counted_ini_write(const char *section,const char *key,const char *value,const char *path){
 ini_writes++;
 return WritePrivateProfileStringA(section,key,value,path);
}
#define WritePrivateProfileStringA counted_ini_write
static int physx_settings_sync_depth;
static struct {int debug;} defaults_cfg;
static float physx_clampf(float v,float lo,float hi){return fmaxf(lo,fminf(hi,v));}
static int ptr_readable(const void *p,size_t n){(void)n;return p!=NULL;}
static int ptr_executable(const void *p){return p!=NULL;}
static void log_line(const char *fmt,...){(void)fmt;}
static void config_file_path(char *p,size_t n){(void)p;(void)n;assert(0);}
static const char *stringref_cstr_a(const char *p){return p;}
static void migrate_legacy_penis_physics_section(void){}
static void physx_mark_penis_physics_setting_change(const char *k,int v){(void)k;(void)v;}
static void physx_mark_testicle_physics_setting_change(const char *k,int v){(void)k;(void)v;}
static void physx_mark_penis_collision_setting_change(int v){(void)v;}
static int physx_custom_parameter_name(void *p,char *out,size_t n){lstrcpynA(out,p,(int)n);return 1;}
'''
for name in ['BREASTS_PHYSICS_CONFIG_SECTION', 'PENIS_PHYSICS_CONFIG_SECTION',
             'TESTICLE_PHYSICS_CONFIG_SECTION', 'BUTT_PHYSICS_CONFIG_SECTION',
             'BODY_COLLIDERS_CONFIG_SECTION']:
    source += re.search(r'^#define ' + name + r' .*$', main, re.M).group(0) + '\n'
source += config[:config.index('} physx_settings_binding_t;') + len('} physx_settings_binding_t;')]
start = config.index('static physx_settings_binding_t physx_settings_bindings[]')
source += config[start:config.index('\n};', start) + 3]
for name in ['physx_settings_binding_by_name', 'physx_settings_bool_value',
             'physx_settings_collision_scope_value', 'profile_collision_strength',
             'physx_slider_widget_value', 'physx_slider_widget_set_value',
             'physx_sync_slider_from_ini', 'physx_write_slider_value', 'physx_write_slider_setting',
             'handle_physx_settings_change']:
    source += function(name)
source += r'''
static int spinbox_syncs;
static void physx_sync_spinbox_from_ini(const physx_settings_binding_t *b,void *w){
 assert(!strcmp(b->key,"enabled"));assert(w==(void*)0x1234);spinbox_syncs++;
}
static int original_build(void *self,void *a,void *b,void *c){
 (void)self;(void)a;(void)b;(void)c;
 assert(physx_settings_sync_depth>0);
 handle_physx_settings_change("NCPhysXBreastsCollisionStrength","0.99");
 return 42;
}
static int (*real_Customizer_BuildControls)(void *,void *,void *,void *)=original_build;
'''
source += function('physx_build_controls_and_sync') + function('hook_Customizer_BuildControls_PhysX')
source += r'''
static void fixture_param_change(void *,const char *,const char *,DWORD,DWORD);
static void (*real_ConfigEditor_ParamChange)(void *,const char *,const char *,DWORD,DWORD)=fixture_param_change;
static int create_calls;
static int fixture_create(void *s,void *p,void *r,void *parent,float y,int preset,int labels){
 (void)s;(void)p;(void)r;(void)parent;(void)y;(void)preset;(void)labels;create_calls++;return 17;
}
static int (*real_PhysX_CreateSlider)(void *,void *,void *,void *,float,int,int)=fixture_create;
'''
source += function('hook_PhysX_CreateSlider')
source += function('hook_ConfigEditor_ParamChange', (root / 'physx_hooks_core.c').read_text())
source += r'''
static void (__cdecl *real_PhysX_ConfigEditorCallback)(void *,const char *,const char *,DWORD,const float *);
'''
source += function('hook_PhysX_ConfigEditorCallback')
source += r'''
static const char *names[]={"NCPhysXBreastsCollisionStrength","NCPhysXPenisCollisionStrength",
 "NCPhysXTesticleCollisionStrength","NCPhysXButtCollisionStrength"};
typedef struct {BYTE *meta;BYTE pad[20];float value;} slider_t;
static slider_t sliders[4],presets[4];
static int setter_calls;
static void fixture_param_change(void *s,const char *p,const char *v,DWORD a,DWORD b){
 (void)s;(void)p;(void)v;(void)a;(void)b;
 /* Reproduce a native handler restoring/replacing the live value. */
 sliders[0].value=1;
}
static float THISCALL slider_get(void *self,DWORD member){assert(member==0x02fff0ed);return *(float*)self;}
static void THISCALL slider_set(void *self,DWORD member,float value){
 assert(member==0x02fff0ed);*(float*)self=value;setter_calls++;
 for(int i=0;i<4;i++)if(self==&sliders[i].value)handle_physx_settings_change(names[i],"stale");
}
static void assert_close(float a,float b){assert(fabsf(a-b)<1e-6f);}
int main(int argc,char **argv){
 assert(argc==2);lstrcpynA(config_path,argv[1],sizeof(config_path));
 BYTE metadata[0x3b8]={0},dispatch[0x88]={0},self[0x1c]={0},record[5][0x30]={{0}};
 BYTE *d=dispatch;memcpy(metadata+0x3b4,&d,sizeof(d));
 float (THISCALL *getter)(void *,DWORD)=slider_get;
 void (THISCALL *setter)(void *,DWORD,float)=slider_set;
 memcpy(dispatch+0x80,&getter,sizeof(getter));memcpy(dispatch+0x84,&setter,sizeof(setter));
 struct {int count;void *items[5];} parameters={5,{0}},records={5,{0}};
 void **p=parameters.items,**r=records.items;memcpy(self+0x14,&p,sizeof(p));memcpy(self+0x18,&r,sizeof(r));
 const char *values[]={"0.2","0.35","0.6","0.9"};
 for(int i=0;i<4;i++){
  physx_settings_binding_t *binding=physx_settings_binding_by_name(names[i]);assert(binding);
  assert(!strcmp(binding->key,"collision_strength"));
  sliders[i].meta=presets[i].meta=metadata;sliders[i].value=1;presets[i].value=.75f;
  void *widget=&sliders[i].value,*preset=&presets[i].value;
  memcpy(record[i]+4,&widget,sizeof(widget));memcpy(record[i]+8,&preset,sizeof(preset));
  parameters.items[i]=(void*)names[i];records.items[i]=record[i];
  WritePrivateProfileStringA(binding->section,binding->key,values[i],config_path);
 }
 parameters.items[4]="NCPhysXBreastsPhysics";records.items[4]=record[4];
 void *spin=(void*)0x1234;memcpy(record[4]+0x24,&spin,sizeof(spin));
 assert(hook_Customizer_BuildControls_PhysX(self,0,0,0)==42);
 assert(physx_settings_sync_depth==0 && setter_calls==4 && spinbox_syncs==1);
 for(int i=0;i<4;i++){
  physx_settings_binding_t *binding=physx_settings_binding_by_name(names[i]);
  assert_close(sliders[i].value,(float)atof(values[i]));assert_close(presets[i].value,.75f);
  assert_close(profile_collision_strength(binding->section,1,config_path),(float)atof(values[i]));
  assert(binding->slider_widget==&sliders[i].value);
  sliders[i].value=.125f+.1f*i;handle_physx_settings_change(names[i],"Weak");
  assert_close(profile_collision_strength(binding->section,1,config_path),sliders[i].value);
 }
 puts("PASS: all four live sliders load INI values and write native floats to the correct sections; initialization callbacks cannot overwrite INI; presets and spinboxes remain separate");
 physx_settings_binding_t *binding=physx_settings_binding_by_name(names[0]);
 sliders[0].value=0;handle_physx_settings_change(names[0],NULL);assert_close(profile_collision_strength(binding->section,1,config_path),.1f);
 sliders[0].value=2;handle_physx_settings_change(names[0],NULL);assert_close(profile_collision_strength(binding->section,1,config_path),1);
 sliders[0].value=NAN;handle_physx_settings_change(names[0],"invalid");assert_close(profile_collision_strength(binding->section,1,config_path),1);
 binding->slider_widget=NULL;handle_physx_settings_change(names[0],"0.375");assert_close(profile_collision_strength(binding->section,1,config_path),.375f);
 handle_physx_settings_change(names[0],"0.5oops");assert_close(profile_collision_strength(binding->section,1,config_path),.375f);
 WritePrivateProfileStringA(binding->section,binding->key,"0.45",config_path);
 hook_Customizer_BuildControls_PhysX(self,0,0,0);assert_close(sliders[0].value,.45f);
 WritePrivateProfileStringA(binding->section,binding->key,NULL,config_path);
 hook_Customizer_BuildControls_PhysX(self,0,0,0);assert_close(sliders[0].value,1);
 char text[32];GetPrivateProfileStringA(binding->section,binding->key,"missing",text,sizeof(text),config_path);assert(!strcmp(text,"missing"));
 assert(hook_Customizer_BuildControls_PhysX(NULL,0,0,0)==42 && physx_settings_sync_depth==0);
 for(int i=0;i<4;i++)assert(physx_settings_binding_by_name(names[i])->slider_widget==NULL);
 WritePrivateProfileStringA(binding->section,binding->key,"0.6",config_path);
 assert(hook_PhysX_CreateSlider(NULL,(void*)names[0],record[0],NULL,0,-1,0)==17);
 assert(binding->slider_widget==&sliders[0].value);assert_close(sliders[0].value,.6f);
 assert(hook_PhysX_CreateSlider(NULL,(void*)names[0],record[0],NULL,0,0,1)==17);
 assert(binding->slider_widget==&sliders[0].value);assert_close(presets[0].value,.75f);
 sliders[0].value=.1f;
 hook_ConfigEditor_ParamChange(NULL,names[0],NULL,0,0);
 assert_close(profile_collision_strength(binding->section,1,config_path),.1f);
 assert_close(sliders[0].value,.1f);
 assert(create_calls==2);
 puts("PASS: creation hook registers the main slider without capturing presets; actual ParamChange hook preserves the incoming 0.1 even when TK17 resets the widget to 1 during its callback");
 /* Execute TK17's actual callback adapter, including its discarded numeric
    payload and stack padding. Only relocate its call to the production hook. */
 const BYTE adapter_bytes[]={0x55,0x8b,0xec,0x8b,0x4d,0x08,0x83,0xec,0x08,
  0xff,0x75,0x10,0xff,0x75,0x0c,0xe8,0x5c,0xed,0xff,0xff,0x5d,0xc3};
 BYTE *adapter=VirtualAlloc(NULL,sizeof(adapter_bytes),MEM_COMMIT|MEM_RESERVE,PAGE_EXECUTE_READWRITE);
 assert(adapter);memcpy(adapter,adapter_bytes,sizeof(adapter_bytes));
 DWORD relative=(DWORD)((BYTE*)hook_ConfigEditor_ParamChange-(adapter+20));
 memcpy(adapter+16,&relative,sizeof(relative));
 FlushInstructionCache(GetCurrentProcess(),adapter,sizeof(adapter_bytes));
 real_PhysX_ConfigEditorCallback=(void*)adapter;
 for(int i=0;i<4;i++){
  binding=physx_settings_binding_by_name(names[i]);binding->slider_widget=NULL;
  WritePrivateProfileStringA(binding->section,binding->key,"0.1",config_path);
  float payload[4]={1,0,0,0};
  /* Old path reproduces the bug: the adapter loses the float, INI stays .1. */
  real_PhysX_ConfigEditorCallback(NULL,names[i],"",0,payload);
  assert_close(profile_collision_strength(binding->section,1,config_path),.1f);
  hook_PhysX_ConfigEditorCallback(NULL,names[i],"",0,payload);
  assert_close(profile_collision_strength(binding->section,1,config_path),1);
  payload[0]=.1f;hook_PhysX_ConfigEditorCallback(NULL,names[i],NULL,0,payload);
  assert_close(profile_collision_strength(binding->section,1,config_path),.1f);
  physx_settings_sync_depth++;payload[0]=1;
  hook_PhysX_ConfigEditorCallback(NULL,names[i],NULL,0,payload);
  assert(physx_settings_sync_depth==1);physx_settings_sync_depth--;
  assert_close(profile_collision_strength(binding->section,1,config_path),.1f);
  payload[0]=NAN;hook_PhysX_ConfigEditorCallback(NULL,names[i],NULL,0,payload);
  hook_PhysX_ConfigEditorCallback(NULL,names[i],NULL,0,NULL);
  assert_close(profile_collision_strength(binding->section,1,config_path),.1f);
 }
 assert(physx_settings_sync_depth==0);
 binding=physx_settings_binding_by_name(names[0]);binding->slider_widget=&sliders[0].value;
 sliders[0].value=1;
 float incoming[4]={.25f,0,0,0};
 hook_PhysX_ConfigEditorCallback(NULL,names[0],"",0,incoming);
 assert_close(profile_collision_strength(binding->section,1,config_path),.25f);
 assert_close(sliders[0].value,.25f);
 int writes_before=ini_writes;
 /* Repeated notifications must still restore the widget after native reset,
    without rewriting the INI or scheduling another reload. */
 hook_PhysX_ConfigEditorCallback(NULL,names[0],"",0,incoming);
 assert(ini_writes==writes_before);
 assert_close(sliders[0].value,.25f);
 WritePrivateProfileStringA(binding->section,binding->key,"0.7",config_path);
 writes_before=ini_writes;
 hook_PhysX_ConfigEditorCallback(NULL,names[0],"",0,incoming);
 assert(ini_writes==writes_before+1);
 assert_close(profile_collision_strength(binding->section,1,config_path),.25f);
 puts("PASS: duplicate native notifications skip INI writes while restoring the widget; external INI edits are not hidden by a value cache");
 hook_PhysX_ConfigEditorCallback(NULL,"NCPhysXBreastsPhysics","OFF",0,incoming);
 GetPrivateProfileStringA(binding->section,"enabled","missing",text,sizeof(text),config_path);
 assert(!strcmp(text,"false"));
 hook_PhysX_ConfigEditorCallback(NULL,"AnotherPluginSlider",NULL,0,incoming);
 assert_close(profile_collision_strength(binding->section,1,config_path),.25f);
 VirtualFree(adapter,0,MEM_RELEASE);
 puts("PASS: actual x86 callback adapter reproduces the missing numeric value; entry hook saves 0.1 -> 1 -> 0.1 for all four sections without widgets, rejects invalid payloads and suppresses initialization events");
 puts("PASS: range clamps, invalid event rejection, numeric fallback, INI edits on reopen, missing-key default, stale-widget reset and balanced sync guard");
 return 0;
}
'''
build = root / 'build'
build.mkdir(exist_ok=True)
c = build / 'settings_slider_test.c'
c.write_text(source)
gcc = Path(r'C:\msys64\mingw32\bin\gcc.exe')
env = dict(os.environ, PATH=str(gcc.parent) + os.pathsep + os.environ['PATH'])
exe = build / 'settings_slider_test.exe'
subprocess.run([str(gcc), '-m32', '-O2', '-Wall', '-Wextra', '-Werror', '-static-libgcc',
                '-o', str(exe), str(c)], env=env, check=True)
subprocess.run([str(exe), str(build / 'settings_slider_test.ini')], env=env, check=True, timeout=30)
