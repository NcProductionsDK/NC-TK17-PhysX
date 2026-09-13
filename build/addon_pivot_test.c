
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
static int addon_chain_target_body_local(physx_sidecar_t *sc,
                                         physx_chain_t *chain,
                                         physx_target_t *target,
                                         int person_index,
                                         const body_chain_collider_person_state_t *state,
                                         float out[3])
{
    char person[16];
    char fallback[192];
    float view[3];
    if (!target || !out || !state ||
        person_index < 0 || person_index >= 4) {
        return 0;
    }
    _snprintf(person, sizeof(person), "Person%02d", person_index + 1);
    fallback[0] = 0;
    if (!target->addon_simulated_target) {
        _snprintf(fallback, sizeof(fallback), "S%s", target->name);
        if (body_collider_engine_pivot_view(person, target->name,
                                            fallback, view) &&
            addon_chain_view_to_body_local(state, view, out)) {
            addon_chain_note_body_root_person(chain, person, GetTickCount(),
                                              "collision-pivot");
            (void)sc;
            return 1;
        }
    }
    if (sc && chain && chain->addon_owner_person[0] &&
        !sidecar_selected_for_owner(sc, chain->addon_owner_person)) {
        return 0;
    }
    resolve_engine_symbols();
    if (engine_GetModelViewRotationPivot &&
        target->object &&
        !is_nil_engine_object(target->raw_object, target->object)) {
        void *pivot_object=addon_pivot_script_object(target->raw_object,target->object);
        static unsigned int pivot_mapping_logs;
        if(pivot_object!=target->object && defaults_cfg.debug && pivot_mapping_logs<32) {
            pivot_mapping_logs++;
            log_line("addon pivot ScriptObject selection target=\"%s\" raw=%p resolved=%p selected=%p result=%s note=\"resolved object lacks the pivot API script dispatch; use the validated raw binding or skip this sample\"",
                target->name,target->raw_object,target->object,pivot_object,
                pivot_object?"raw-script-object":"unavailable");
        }
        if(!pivot_object) return 0;
        const char *previous_stage=physx_fault_stage;
        const void *previous_raw=physx_fault_raw,*previous_object=physx_fault_object;
        physx_fault_stage="addon-body-local-pivot";
        physx_fault_raw=target->raw_object;physx_fault_object=pivot_object;
        engine_GetModelViewRotationPivot(pivot_object, view);
        physx_fault_stage=previous_stage;
        physx_fault_raw=previous_raw;physx_fault_object=previous_object;
        if (sane_probe_float(view[0]) &&
            sane_probe_float(view[1]) &&
            sane_probe_float(view[2]) &&
            addon_chain_view_to_body_local(state, view, out)) {
            return 1;
        }
    }
    return 0;
}
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
