"""Compare production addon-name matching with the pre-fix behavior."""
import os
from pathlib import Path
import re
import subprocess

root = Path(__file__).resolve().parent
source = (root / 'NC-TK17-PhysX.c').read_text()


def function(name):
    match = re.search(r'^static int ' + name + r'\([^;{}]*\)\s*\{', source, re.M)
    if not match:
        raise ValueError(name)
    pos, depth = match.end(), 1
    while depth:
        depth += (source[pos] == '{') - (source[pos] == '}')
        pos += 1
    return source[match.start():pos] + '\n'


fixture = r'''
#include <windows.h>
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
typedef struct { int addon_simulated_target; char name[128]; } physx_target_t;
typedef struct { int addon_chain, target_count; char parent_name[128];
                 physx_target_t targets[4]; } physx_chain_t;
typedef struct { int loaded, enabled, chain_count;
                 physx_chain_t chains[2]; } physx_sidecar_t;
static physx_sidecar_t sidecars[2];
static int sidecar_count=2, addon_physics_enabled=1, addon_physics_probe_enabled;
static int reference_match(const char *runtime_name,const char *declared_name) {
    char alias[192];
    if(!runtime_name || !declared_name || !runtime_name[0] || !declared_name[0]) return 0;
    if(!_stricmp(runtime_name,declared_name)) return 1;
    _snprintf(alias,sizeof(alias),"S%s",declared_name);
    if(!_stricmp(runtime_name,alias)) return 1;
    _snprintf(alias,sizeof(alias),"local_%s",declared_name);
    if(!_stricmp(runtime_name,alias)) return 1;
    _snprintf(alias,sizeof(alias),"local_S%s",declared_name);
    return !_stricmp(runtime_name,alias);
}
static double timer(void) {
    LARGE_INTEGER t,f; QueryPerformanceCounter(&t); QueryPerformanceFrequency(&f);
    return (double)t.QuadPart/f.QuadPart;
}
'''
fixture += function('addon_declared_name_matches_alias')
fixture += function('addon_declared_sidecar_name')
fixture += r'''
int main(void) {
    const char *names[]={NULL,"","S","s","local_","local_S","Branch_01",
        "SBranch_01","local_Branch_01","local_SBranch_01","LOCAL_sBRANCH_01",
        "SSBranch_01","local_local_Branch_01","PoseEdit_KeyframeP01_Image",
        "Branch_01_extra","Person01/Branch_01","Hair/Joint01"};
    const char *prefix[]={"","S","local_","local_S"};
    char name[128], runtime[192];
    uint32_t rng=12345;
    for(unsigned i=0;i<sizeof(names)/sizeof(*names);i++)
        for(unsigned j=0;j<sizeof(names)/sizeof(*names);j++)
            assert(addon_declared_name_matches_alias(names[i],names[j])==reference_match(names[i],names[j]));
    for(int i=0;i<20000;i++) {
        int len=1+i%127;
        for(int k=0;k<len;k++) { rng=rng*1664525u+1013904223u; name[k]="aBsSlLoc_19/"[rng%12]; }
        name[len]=0;
        for(int p=0;p<4;p++) {
            _snprintf(runtime,sizeof(runtime),"%s%s",prefix[p],name);
            assert(addon_declared_name_matches_alias(runtime,name));
            assert(reference_match(runtime,name));
            for(char *c=runtime;*c;c++) if(*c>='a' && *c<='z') *c-=32;
            assert(addon_declared_name_matches_alias(runtime,name)==reference_match(runtime,name));
            strcat(runtime,"!");
            assert(!addon_declared_name_matches_alias(runtime,name));
            assert(!reference_match(runtime,name));
        }
    }
    puts("PASS: differential alias matching, case, exact boundaries, nested prefixes, null/empty and 127-byte names");
    physx_sidecar_t *sc=&sidecars[1];
    physx_chain_t *ch=&sc->chains[0];
    sc->loaded=sc->enabled=sc->chain_count=ch->addon_chain=ch->target_count=1;
    strcpy(ch->parent_name,"Anchor"); strcpy(ch->targets[0].name,"Branch_01");
    ch->targets[0].addon_simulated_target=1;
    assert(addon_declared_sidecar_name("local_SAnchor"));
    assert(addon_declared_sidecar_name("SBranch_01"));
    assert(!addon_declared_sidecar_name("PoseEdit_KeyframeP01_Image"));
    ch->targets[0].addon_simulated_target=0; assert(!addon_declared_sidecar_name("SBranch_01"));
    ch->targets[0].addon_simulated_target=1;
    sc->loaded=0; assert(!addon_declared_sidecar_name("SBranch_01")); sc->loaded=1;
    sc->enabled=0; assert(!addon_declared_sidecar_name("SBranch_01")); sc->enabled=1;
    ch->addon_chain=0; assert(!addon_declared_sidecar_name("SBranch_01")); ch->addon_chain=1;
    addon_physics_enabled=0; assert(!addon_declared_sidecar_name("SBranch_01"));
    addon_physics_probe_enabled=1; assert(addon_declared_sidecar_name("SBranch_01"));
    strcpy(ch->targets[0].name,"NewBranch");
    assert(!addon_declared_sidecar_name("SBranch_01")); assert(addon_declared_sidecar_name("local_SNewBranch"));
    puts("PASS: parent/target discovery, disabled/unloaded/non-addon filtering, probe mode and immediate sidecar edits");
    volatile unsigned sink=0;
    for(int mode=0;mode<2;mode++) {
        int (*match)(const char *,const char *)=mode?addon_declared_name_matches_alias:reference_match;
        double start=timer();
        for(int i=0;i<1000000;i++) sink+=match("PoseEdit_KeyframeP01_Image",names[6+i%5]);
        printf("BENCH unrelated UI names %s: %.3f ms / 1000000 comparisons\n",mode?"fixed":"original",(timer()-start)*1000);
    }
    return sink?1:0;
}
'''
build = root / 'build/name-alias-tests'
build.mkdir(parents=True, exist_ok=True)
cfile, exe = build / 'name_alias.c', build / 'name_alias.exe'
cfile.write_text(fixture)
env = dict(os.environ)
env['PATH'] = r'C:\msys64\mingw32\bin;' + env.get('PATH', '')
subprocess.run([r'C:\msys64\mingw32\bin\gcc.exe', '-m32', '-O2', '-Wall',
                '-Wextra', '-Werror', str(cfile), '-o', str(exe)], env=env, check=True)
subprocess.run([str(exe)], env=env, check=True)
