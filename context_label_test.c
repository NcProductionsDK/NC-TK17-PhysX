#define COBJMACROS
#include <windows.h>
static HMODULE label_test_module(LPCSTR name);
#define GetModuleHandleA label_test_module
#include "NC-TK17-PhysX.c"
#undef GetModuleHandleA
#include <assert.h>

static BYTE *test_base;
static void *test_penis_widget;
static HMODULE label_test_module(LPCSTR name) { (void)name; return (HMODULE)test_base; }
static void *__cdecl test_find_widget(const char *name)
{
    return !strcmp(name,"GUI:PersonContext_PhysX_Penis_Toggle") ? test_penis_widget : NULL;
}

static char label[64];
static int constructed, released, written;
static void THISCALL make_string(char **out, const char *text)
{
    constructed++; *out = _strdup(text);
}
static void THISCALL free_string(char **value)
{
    released++; free(*value); *value = NULL;
}
static void THISCALL set_label(void *widget, DWORD id, const char *text)
{
    assert(widget && id == 0x03fff0ebu);
    strcpy(label, text); written++;
}

int main(void)
{
    SetErrorMode(SEM_NOGPFAULTERRORBOX); _set_error_mode(_OUT_TO_STDERR);
    setbuf(stdout,NULL);
    BYTE *base=calloc(1,0x2c0000), *metadata=calloc(1,0x4000);
    BYTE *allocation=calloc(1,0x60), *table=calloc(1,0x100);
    BYTE *object=(BYTE*)(((uintptr_t)allocation+15u)&~(uintptr_t)15u);
    engine_string_construct_cstr_t construct=make_string;
    engine_string_release_t release=free_string;
    void (THISCALL *set)(void*,DWORD,const char*)=set_label;
    assert(base && metadata && object && table);
    memcpy(base+PHYSX_ENGINE_STRING_CSTR_CONSTRUCT_RVA,&construct,sizeof(construct));
    memcpy(base+PHYSX_ENGINE_STRING_RELEASE_RVA,&release,sizeof(release));
    memcpy(object+8,&metadata,sizeof(metadata));
    memcpy(metadata+0xebu*4,&table,sizeof(table));
    memcpy(table+0xc4,&set,sizeof(set));
    const char *kinds[]={"Breast","Penis","Testicle","Butt"};
    for (int p=0;p<4;p++) for (int system=0;system<4;system++) {
        int enabled=(p+system)%2;
        char expected[64];
        snprintf(expected,sizeof(expected),"%s %s PhysX",enabled?"Disable":"Enable",kinds[system]);
        assert(person_context_physx_label_at_base(object+0x20,enabled,kinds[system],base));
        assert(!strcmp(label,expected));
    }
    assert(constructed==16 && released==16 && written==16);
    test_base=base;test_penis_widget=object+0x20;
    engine_FindObjC=test_find_widget;
    person_context_selected_person=2;
    body_chain_physics_global_cfg.enabled=1;
    body_chain_physics_global_cfg.enabled_person[1]=1;
    body_profile_rebuild_effective_configs();
    assert(!strcmp(label,"Disable Penis PhysX"));
    body_chain_physics_global_cfg.enabled_person[1]=0;
    body_profile_rebuild_effective_configs();
    assert(!strcmp(label,"Enable Penis PhysX"));
    body_chain_physics_global_cfg.enabled_person[0]=1;
    body_profile_rebuild_effective_configs();
    assert(!strcmp(label,"Enable Penis PhysX"));
    body_chain_physics_global_cfg.enabled_person[1]=1;
    body_profile_rebuild_effective_configs();
    assert(!strcmp(label,"Disable Penis PhysX"));
    assert(constructed==20 && released==20 && written==20);
    person_context_selected_person=0;
    body_profile_rebuild_effective_configs();
    assert(written==20);
    puts("PASS: reused menu refreshes ON/OFF/ON on config rebuild, follows the selected person, and ignores absent selection");
    /* Missing widget metadata/setter must not call an invalid native target
       or allocate a string that the engine cannot release. */
    memset(table+0xc4,0,sizeof(set));
    assert(!person_context_physx_label_at_base(object+0x20,1,"Breast",base));
    assert(!person_context_physx_label_at_base(NULL,1,"Breast",base));
    assert(constructed==20 && released==20 && written==20);
    free(table);free(allocation);free(metadata);free(base);
    puts("PASS: native string dispatch, Enable/Disable labels, balanced string ownership and missing-widget/setter fallback");
    return 0;
}
