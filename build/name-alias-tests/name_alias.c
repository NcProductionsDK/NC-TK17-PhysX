
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
static int addon_declared_name_matches_alias(const char *runtime_name,
                                             const char *declared_name)
{
    if (!runtime_name || !declared_name ||
        !runtime_name[0] || !declared_name[0]) {
        return 0;
    }
    if (_stricmp(runtime_name, declared_name) == 0) return 1;
    /* Object::iNameSet also receives UI names during Key Editor scrolling.
       Avoid formatting three aliases for every unrelated name/target pair.
       Strip only a recognized runtime prefix, preserving declared names that
       themselves start with S or local_ and the original case-insensitive
       matching. No cached result can become stale after a sidecar reload. */
    if ((runtime_name[0] == 'S' || runtime_name[0] == 's') &&
        _stricmp(runtime_name + 1, declared_name) == 0) return 1;
    if (_strnicmp(runtime_name, "local_", 6) != 0) return 0;
    runtime_name += 6;
    if (_stricmp(runtime_name, declared_name) == 0) return 1;
    return (runtime_name[0] == 'S' || runtime_name[0] == 's') &&
           _stricmp(runtime_name + 1, declared_name) == 0;
}
static int addon_declared_sidecar_name(const char *name)
{
    int i, c, t;
    if (!name || !name[0] ||
        (!addon_physics_enabled && !addon_physics_probe_enabled)) {
        return 0;
    }
    for (i = 0; i < sidecar_count; i++) {
        physx_sidecar_t *sc = &sidecars[i];
        if (!sc->loaded || !sc->enabled) continue;
        for (c = 0; c < sc->chain_count; c++) {
            physx_chain_t *chain = &sc->chains[c];
            if (!chain->addon_chain) continue;
            if (addon_declared_name_matches_alias(name, chain->parent_name)) {
                return 1;
            }
            for (t = 0; t < chain->target_count; t++) {
                physx_target_t *target = &chain->targets[t];
                if (!target->addon_simulated_target) continue;
                if (addon_declared_name_matches_alias(name, target->name)) {
                    return 1;
                }
            }
        }
    }
    return 0;
}

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
