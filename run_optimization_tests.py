"""Differential tests and isolated timings for production lookup and wind code."""
import os
from pathlib import Path
import re
import subprocess

root = Path(__file__).resolve().parent
sidecar = (root / 'physx_sidecar.c').read_text()
wind = (root / 'physx_room_wind.c').read_text()
reference = (root / 'tests/reference_optimization.h').read_text()


def function(source, name):
    match = re.search(r'^static [^;{}]*\b' + name + r'\([^;{}]*\)\s*\{', source, re.M)
    if not match:
        raise ValueError(name)
    pos, depth = match.end(), 1
    while depth:
        depth += (source[pos] == '{') - (source[pos] == '}')
        pos += 1
    return source[match.start():pos] + '\n'


common = r'''
#include <windows.h>
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
static double timer(void) { LARGE_INTEGER t,f; QueryPerformanceCounter(&t); QueryPerformanceFrequency(&f); return (double)t.QuadPart/f.QuadPart; }
'''
registry = common + r'''
typedef struct {int unused;} physx_sidecar_t;
static DWORD fake_tick=100000;
#define GetTickCount() fake_tick
'''
tables = sidecar[sidecar.index('#define ADDON_ACTIVE_SLOT_COUNT'):sidecar.index('#define ADDON_SELECTION_HINT_COUNT')]
registry += tables
for name in ('addon_active_slots', 'addon_equipment_definitions', 'addon_equipment_slots'):
    declaration = re.search(r'static [^;]*\b' + name + r'\[[^;]*;', tables).group()
    registry += declaration.replace(name, 'reference_' + name) + '\n'
registry += function(sidecar, 'addon_lookup_hint_index')
for name in ('addon_active_slot_find', 'addon_equipment_definition_find', 'addon_equipment_slot_find'):
    registry += function(sidecar, name)
    original = function(reference, 'reference_' + name)
    for table in ('addon_active_slots', 'addon_equipment_definitions', 'addon_equipment_slots'):
        original = original.replace(table, 'reference_' + table)
    registry += original
registry += r'''
static int offset(const void *p,const void *base,size_t size){return p?(int)(((const char*)p-(const char*)base)/size):-1;}
static void compare(const char *owner,const char *id,const char *zone,int create) {
 addon_active_slot_t *a=addon_active_slot_find(owner,id,create),*b=reference_addon_active_slot_find(owner,id,create);
 assert(offset(a,addon_active_slots,sizeof(*a))==offset(b,reference_addon_active_slots,sizeof(*b)));
 addon_equipment_slot_t *c=addon_equipment_slot_find(owner,zone,create),*d=reference_addon_equipment_slot_find(owner,zone,create);
 assert(offset(c,addon_equipment_slots,sizeof(*c))==offset(d,reference_addon_equipment_slots,sizeof(*d)));
 addon_equipment_definition_t *e=addon_equipment_definition_find(id,create),*f=reference_addon_equipment_definition_find(id,create);
 assert(offset(e,addon_equipment_definitions,sizeof(*e))==offset(f,reference_addon_equipment_definitions,sizeof(*f)));
 if(create) {
  if(a) {a->root_tick=b->root_tick=fake_tick; a->root_object=b->root_object=(void*)(uintptr_t)(fake_tick|1);}
  if(c) {c->root_tick=d->root_tick=fake_tick; c->root_object=d->root_object=(void*)(uintptr_t)(fake_tick|1);}
 }
 assert(!memcmp(addon_active_slots,reference_addon_active_slots,sizeof(addon_active_slots)));
 assert(!memcmp(addon_equipment_slots,reference_addon_equipment_slots,sizeof(addon_equipment_slots)));
 assert(!memcmp(addon_equipment_definitions,reference_addon_equipment_definitions,sizeof(addon_equipment_definitions)));
}
static volatile uintptr_t sink;
int main(void) {
 SetErrorMode(SEM_NOGPFAULTERRORBOX); _set_error_mode(_OUT_TO_STDERR); setbuf(stdout,NULL);
 char owner[16],id[64],zone[64];
 for(int round=0;round<3;round++) {
  /* Keep TLS hints across table resets to exercise stale hints and slot reuse. */
  memset(addon_active_slots,0,sizeof(addon_active_slots));memset(reference_addon_active_slots,0,sizeof(reference_addon_active_slots));
  memset(addon_equipment_slots,0,sizeof(addon_equipment_slots));memset(reference_addon_equipment_slots,0,sizeof(reference_addon_equipment_slots));
  memset(addon_equipment_definitions,0,sizeof(addon_equipment_definitions));memset(reference_addon_equipment_definitions,0,sizeof(reference_addon_equipment_definitions));
  for(int i=0;i<1600;i++) {
   fake_tick+=97; if(i==800) fake_tick=UINT32_MAX-100;
   snprintf(owner,sizeof(owner),"Person%02d",i%4);snprintf(id,sizeof(id),"Cloth_%d",i%700);snprintf(zone,sizeof(zone),"DZ_%d",i%300);
   compare(owner,id,zone,0);compare(owner,id,zone,1);compare(owner,id,zone,0);
   strlwr(owner);strlwr(id);strlwr(zone);compare(owner,id,zone,0);
  }
 }
 compare(NULL,NULL,NULL,0);compare("","","",1);
 puts("PASS: registry results/state match original across case changes, missing keys, full tables, eviction, clock wrap and reset with stale hints");
 char owners[32][16],ids[32][64];
 for(int i=0;i<32;i++){snprintf(owners[i],16,"Person%02d",i%4);snprintf(ids[i],64,"Bench_%d",i);compare(owners[i],ids[i],"DZ_Bench",1);}
 for(int mode=0;mode<2;mode++){
  double start=timer();
  for(int i=0;i<1000000;i++) {int k=i%32;sink=(uintptr_t)(mode?addon_active_slot_find(owners[k],ids[k],0):reference_addon_active_slot_find(owners[k],ids[k],0));}
  printf("BENCH active-slot %s: %.3f ms / 1000000 queries\n",mode?"optimized":"original",(timer()-start)*1000);
 }
 return 0;
}
'''

wind_fixture = common + '#include "physx_wind.h"\n'
wind_fixture += r'''
static physx_wind_state_t room_wind_state;
static struct {int enabled;float strength,turbulence,gust_strength,gust_frequency,variation,sway_strength,sway_frequency;} room_wind_cfg={1,1.2f,.3f,.6f,.4f,.25f,.5f,.18f};
typedef struct {int wind_enabled;char addon_owner_person[16],name[128];float wind_scale,wind_sway_strength,wind_sway_frequency;} physx_chain_t;
typedef struct {char name[128];} physx_target_t;
static int room_wind_is_enabled(void){return room_wind_cfg.enabled&&room_wind_cfg.strength>0.000001f;}
static float physx_clampf(float x,float a,float b){return x<a?a:x>b?b:x;}
'''
for name in ('room_wind_hash_add', 'room_wind_hashed_strength', 'room_wind_body_strength', 'room_wind_target_strength'):
    wind_fixture += function(wind, name)
wind_fixture += '#include "tests/reference_wind.h"\n'
wind_fixture += r'''
static volatile float sink;
static float sample(DWORD t){return room_wind_hashed_strength("Person01","skirt","hem",1,.5f,.18f,t);}
static void clock_tests(void){
 const DWORD boundaries[]={3600000u,UINT32_MAX};
 for(int k=0;k<2;k++){
  memset(&room_wind_state,0,sizeof(room_wind_state));
  DWORD t=boundaries[k]-50;float previous=sample(t);double start=room_wind_state.seconds;
  for(int i=1;i<=100;i++){float v=sample(t+(DWORD)i);assert(isfinite(v)&&fabsf(v-previous)<.02f);previous=v;}
  assert(fabs(room_wind_state.seconds-start-.1)<1e-8);
 }
 puts("PASS: continuous wind through hourly and DWORD clock boundaries");
}
int main(void){
 SetErrorMode(SEM_NOGPFAULTERRORBOX); _set_error_mode(_OUT_TO_STDERR); setbuf(stdout,NULL);
 float maximum=0;
 memset(&room_wind_state,0,sizeof(room_wind_state));
 for(DWORD t=0;t<3600000;t+=37){
  float a=sample(t),b=reference_wind_strength("Person01","skirt","hem",1,.5f,.18f,t);
  maximum=fmaxf(maximum,fabsf(a-b));assert(isfinite(a)&&fabsf(a)<=20);
  assert(a==sample(t));
 }
 printf("Wind old/new maximum first-hour difference: %.8f force units (higher precision clock)\n",maximum);
 assert(maximum<.03f);
 clock_tests();
 /* Identical elapsed time has identical phase regardless of frame cadence. */
 const int rates[]={20,30,60,144};
 float rate_reference=0;
 for(int k=0;k<4;k++){
  memset(&room_wind_state,0,sizeof(room_wind_state));sample(0);
  for(int frame=1;frame<rates[k]*10;frame++)sample((DWORD)(frame*1000/rates[k]));
  float v=sample(10000);if(k)assert(fabsf(v-rate_reference)<2e-6f);else rate_reference=v;
 }
 puts("PASS: matching wind phase at 20/30/60/144 Hz");
 /* Every parameter must take effect even on a repeated timestamp. */
 physx_wind_parameters_t p={1,.2f,.4f,.3f,.2f,1,.7f,.4f};
 physx_wind_state_t cached={0},fresh;
 for(int k=0;k<8;k++){
  float *fields[]={&p.strength,&p.turbulence,&p.gust_strength,&p.gust_frequency,&p.variation,&p.wind_scale,&p.sway_strength,&p.sway_frequency};
  *fields[k]+=.13f;
  float a=physx_wind_sample(&cached,123456,789123,&p,1000);
  memset(&fresh,0,sizeof(fresh));float b=physx_wind_sample(&fresh,123456,789123,&p,1000);assert(fabsf(a-b)<2e-6f);
 }
 /* Force collisions/identity changes: the cache must be transparent. */
 for(unsigned int i=0;i<10000;i++){
  float a=physx_wind_sample(&cached,i*919u,i*65537u,&p,1000);
  memset(&fresh,0,sizeof(fresh));float b=physx_wind_sample(&fresh,i*919u,i*65537u,&p,1000);assert(fabsf(a-b)<2e-6f);
 }
 memset(&room_wind_state,0,sizeof(room_wind_state));
 physx_chain_t chain={1,"Person01","clothing",1,-1,-1};physx_target_t target={"hem"};
 float a=room_wind_target_strength(&chain,&target,1000);
 assert(a==room_wind_hashed_strength(chain.addon_owner_person,chain.name,target.name,1,room_wind_cfg.sway_strength,room_wind_cfg.sway_frequency,1000));
 chain.wind_enabled=0;assert(room_wind_target_strength(&chain,&target,1000)==0);chain.wind_enabled=1;
 chain.wind_sway_strength=0;chain.wind_sway_frequency=.8f;chain.addon_owner_person[0]=0;strcpy(chain.name,"room-tree");
 assert(room_wind_target_strength(&chain,&target,1000)==room_wind_hashed_strength("","room-tree","hem",1,0,.8f,1000));
 assert(room_wind_body_strength("Person01","breasts",1,1000)==room_wind_hashed_strength("Person01","breasts","body",1,room_wind_cfg.sway_strength,room_wind_cfg.sway_frequency,1000));
 room_wind_cfg.enabled=0;assert(sample(1001)==0);room_wind_cfg.enabled=1;assert(room_wind_hashed_strength("a","b","c",0,1,1,1002)==0);
 room_wind_cfg.strength=100;room_wind_cfg.turbulence=0;room_wind_cfg.gust_strength=0;room_wind_cfg.variation=0;
 assert(room_wind_hashed_strength("a","b","c",1,0,0,1003)==20);
 room_wind_cfg.strength=1.2f;room_wind_cfg.gust_strength=.6f;room_wind_cfg.variation=.25f;
 puts("PASS: live parameter edits, cache collisions, disabled wind, body, clothing overrides and ownerless room routing");
 char names[32][32];for(int i=0;i<32;i++)snprintf(names[i],32,"Branch_%02d",i);
 for(int workload=0;workload<3;workload++)for(int mode=0;mode<2;mode++){
  memset(&room_wind_state,0,sizeof(room_wind_state));room_wind_cfg.turbulence=workload==2?0:.3f;
  double start=timer();
  for(DWORD frame=0;frame<20000;frame++)for(int i=0;i<32;i++){
   const char *name=workload==0?names[i/2]:names[i];DWORD t=frame*16;
   sink=mode?room_wind_hashed_strength("Person01","Tree",name,1,.5f,.18f,t):reference_wind_strength("Person01","Tree",name,1,.5f,.18f,t);
  }
  printf("BENCH wind %s %s: %.3f ms / 640000 samples\n",workload==0?"paired":workload==1?"32-target":"zero-turbulence",mode?"optimized":"original",(timer()-start)*1000);
 }
 return 0;
}
'''

build = root / 'build/optimization-tests'
build.mkdir(parents=True, exist_ok=True)
env = dict(os.environ)
env['PATH'] = r'C:\msys64\mingw32\bin;' + env.get('PATH', '')
for name, source in [('registry', registry), ('wind', wind_fixture)]:
    cfile, exe = build / (name + '.c'), build / (name + '.exe')
    cfile.write_text(source)
    subprocess.run([r'C:\msys64\mingw32\bin\gcc.exe', '-m32', '-O2', '-Wall', '-Wextra', '-Werror', '-I', str(root), str(cfile), '-o', str(exe)], env=env, check=True)
    subprocess.run([str(exe)], env=env, check=True)
