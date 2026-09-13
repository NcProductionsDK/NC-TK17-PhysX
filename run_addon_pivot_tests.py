"""Check actual addon pivot routing, including the null metadata crash layout."""
from pathlib import Path
import os,re,subprocess
root=Path(__file__).resolve().parent
production=(root/'physx_sidecar.c').read_text()
match=re.search(r'^static int addon_chain_target_body_local\([^;{}]*\)\s*\{',production,re.M)
pos,depth=match.end(),1
while depth:
    depth+=(production[pos]=='{')-(production[pos]=='}');pos+=1
function=production[match.start():pos]
source=r'''
#include <windows.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <assert.h>
static int ptr_readable(const void *p,size_t n){
 MEMORY_BASIC_INFORMATION m;return p && VirtualQuery(p,&m,sizeof(m)) &&
 m.State==MEM_COMMIT && !(m.Protect&(PAGE_NOACCESS|PAGE_GUARD)) &&
 (uintptr_t)p+n<=(uintptr_t)m.BaseAddress+m.RegionSize;
}
static int ptr_executable(const void *p){MEMORY_BASIC_INFORMATION m;return p && VirtualQuery(p,&m,sizeof(m)) && m.State==MEM_COMMIT && (m.Protect&(PAGE_EXECUTE|PAGE_EXECUTE_READ|PAGE_EXECUTE_READWRITE|PAGE_EXECUTE_WRITECOPY));}
static int is_nil_engine_object(void *raw,void *obj){(void)raw;return obj==(void*)0x1234;}
#include "../physx_addon_pivot.h"
typedef struct {int addon_simulated_target;char name[32];void *raw_object,*object;} physx_target_t;
typedef struct {char addon_owner_person[16];} physx_chain_t;
typedef struct {int selected;} physx_sidecar_t;
typedef struct {int unused;} body_chain_collider_person_state_t;
static struct {int debug;} defaults_cfg={1};
static const char *physx_fault_stage;static const void *physx_fault_raw,*physx_fault_object;
static int calls;static void *expected;
static void fake_pivot(void *p,float *v){assert(p==expected);assert(addon_pivot_script_object_valid(p));calls++;v[0]=1;v[1]=2;v[2]=3;}
static void (*engine_GetModelViewRotationPivot)(void*,float*)=fake_pivot;
static void log_line(const char *s,...){(void)s;}
static void resolve_engine_symbols(void){}
static int sane_probe_float(float v){return isfinite(v);}
static int sidecar_selected_for_owner(physx_sidecar_t *s,const char *o){(void)o;return s->selected;}
static int body_collider_engine_pivot_view(const char *p,const char *n,const char *f,float *v){(void)p;(void)n;(void)f;(void)v;return 0;}
static int addon_chain_view_to_body_local(const body_chain_collider_person_state_t *s,const float *v,float *o){(void)s;memcpy(o,v,sizeof(float)*3);return 1;}
static void addon_chain_note_body_root_person(physx_chain_t *c,const char *p,DWORD n,const char *s){(void)c;(void)p;(void)n;(void)s;}
'''+function+r'''
int main(void){
 BYTE raw[64]={0},resolved[64]={0},meta[512]={0},dispatch[256]={0};
 void *r=raw+24,*o=resolved+24,*m=meta,*d=dispatch,*fn=(void*)fake_pivot;
 physx_target_t t={1,"DriverBra1",r,o};physx_chain_t chain={"Person02"};
 physx_sidecar_t sc={1};body_chain_collider_person_state_t body={0};float out[3]={0};
 memcpy(raw,&m,sizeof(m));memcpy(meta+0x10c,&d,sizeof(d));memcpy(dispatch+0xc0,&fn,sizeof(fn));
 /* Recorded crash: resolved[-0x18] is NULL. Route the same bone's raw script. */
 expected=r;assert(addon_chain_target_body_local(&sc,&chain,&t,1,&body,out));
 assert(calls==1 && out[0]==1 && out[1]==2 && out[2]==3 && !physx_fault_stage);
 memcpy(resolved,&m,sizeof(m));expected=o;
 assert(addon_chain_target_body_local(&sc,&chain,&t,1,&body,out));assert(calls==2);
 /* No compatible representation: don't enter the engine at all. */
 memset(raw,0,sizeof(void*));memset(resolved,0,sizeof(void*));
 assert(!addon_chain_target_body_local(&sc,&chain,&t,1,&body,out));assert(calls==2);
 memcpy(raw,&m,sizeof(m));memset(meta+0x10c,0,sizeof(void*));assert(!addon_pivot_script_object(r,o));
 memcpy(meta+0x10c,&d,sizeof(d));memset(dispatch+0xc0,0,sizeof(void*));assert(!addon_pivot_script_object(r,o));
 fn=meta;memcpy(dispatch+0xc0,&fn,sizeof(fn));assert(!addon_pivot_script_object(r,o));
 fn=(void*)fake_pivot;memcpy(dispatch+0xc0,&fn,sizeof(fn));
 void *bad=VirtualAlloc(NULL,4096,MEM_RESERVE|MEM_COMMIT,PAGE_NOACCESS);assert(bad);
 assert(!addon_pivot_script_object((BYTE*)bad+24,NULL));VirtualFree(bad,0,MEM_RELEASE);
 assert(!addon_pivot_script_object((void*)0x1234,NULL));
 sc.selected=0;assert(!addon_chain_target_body_local(&sc,&chain,&t,1,&body,out));assert(calls==2);
 puts("PASS: actual pivot caller routes null-metadata payload to raw script; valid resolved path unchanged; missing/unreadable/invalid dispatch skips engine call; owner selection retained");
 return 0;
}
'''
build=root/'build';path=build/'addon_pivot_test.c';path.write_text(source)
gcc=Path(r'C:\msys64\mingw32\bin\gcc.exe');env=dict(os.environ,PATH=str(gcc.parent)+os.pathsep+os.environ['PATH'])
exe=path.with_suffix('.exe')
subprocess.run([str(gcc),'-m32','-O2','-Wall','-Wextra','-Werror','-static-libgcc','-o',str(exe),str(path)],env=env,check=True)
subprocess.run([str(exe)],env=env,check=True,timeout=20)
