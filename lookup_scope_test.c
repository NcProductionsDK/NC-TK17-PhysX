#include "NC-TK17-PhysX.c"
#include <assert.h>

static unsigned lookup_calls;
static char available_name[384];
static int object_a, object_b;
static void *available_object;

static void *__cdecl lookup_fixture(const char *name)
{
    lookup_calls++;
    return !strcmp(name, available_name) ? available_object : NULL;
}

static void *lookup(const char *name)
{
    return resolve_axis_map_raw(name);
}

static double test_seconds(void)
{
    LARGE_INTEGER t, f;
    QueryPerformanceCounter(&t); QueryPerformanceFrequency(&f);
    return (double)t.QuadPart / f.QuadPart;
}

int main(void)
{
    const char *missing = "Person02Anim:Model01:MissingGroup";
    engine_FindObjC = lookup_fixture;
    named_node_count = 1024;
    for (int i=0; i<named_node_count; ++i)
        snprintf(named_nodes[i].name, sizeof(named_nodes[i].name), "Root%04d", i);

    /* The first miss scans normally; repeat misses still try the direct
       engine path, but do not repeat the root-prefix search in this update. */
    runtime_exact_lookup_begin(); lookup_calls=0;
    assert(!lookup(missing));
    assert(lookup_calls == 1025);
    for (int i=0; i<39; ++i) assert(!lookup(missing));
    assert(lookup_calls == 1064);
    runtime_exact_lookup_end();
    lookup_calls=0; assert(!lookup(missing)); assert(!lookup(missing));
    assert(lookup_calls == 2050);
    runtime_exact_lookup_begin(); lookup_calls=0;
    assert(!lookup(missing)); assert(lookup_calls == 1025);

    /* A late direct binding appears immediately, even with an unchanged
       generation and a cached fallback miss. Returned objects are not saved. */
    strcpy(available_name, missing); available_object=&object_a;
    assert(lookup(missing) == &object_a);
    available_object=&object_b; assert(lookup(missing) == &object_b);
    available_object=NULL;

    strcpy(available_name, "Root0512:Person02Anim:Model01:MissingGroup");
    available_object=&object_a;
    runtime_exact_lookup_invalidate();
    assert(lookup(missing) == &object_a);
    available_object=&object_b;
    assert(lookup(missing) == &object_b);
    available_object=NULL;
    assert(!lookup(missing));
    available_object=&object_a;
    InterlockedIncrement(&named_node_generation);
    assert(lookup(missing) == &object_a);
    available_object=NULL; assert(!lookup(missing));
    available_object=&object_b;
    runtime_exact_lookup_end(); runtime_exact_lookup_begin();
    assert(lookup(missing) == &object_b);
    available_object=NULL;
    puts("PASS: per-update misses, immediate direct binding/replacement, fresh positive lookup, name mutation, generation and next-update invalidation");

    /* Distinct names sharing a slot must never suppress each other's scan. */
    char collision[192];
    DWORD slot=runtime_exact_name_hash(missing,NULL)&(RUNTIME_EXACT_ROOT_HINT_SLOTS-1);
    for (int i=0;;++i) {
        snprintf(collision,sizeof(collision),"Collision%d",i);
        if ((runtime_exact_name_hash(collision,NULL)&(RUNTIME_EXACT_ROOT_HINT_SLOTS-1))==slot) break;
    }
    assert(!lookup(missing));
    lookup_calls=0; assert(!lookup(collision)); assert(lookup_calls==1025);
    lookup_calls=0; assert(!lookup(missing)); assert(lookup_calls==1025);
    captured_script_engine=(void*)1;
    lookup_calls=0; assert(!lookup(missing)); assert(lookup_calls==1025);
    captured_script_engine=NULL;
    runtime_exact_lookup_scope=~0u; runtime_exact_lookup_begin();
    assert(runtime_exact_lookup_scope==1);
    lookup_calls=0; assert(!lookup(missing)); assert(lookup_calls==1025);
    puts("PASS: hash collisions, script-engine replacement and scope counter wrap");

    /* No generation change is required for the command/clone invalidators. */
    LONG mutation=runtime_exact_lookup_mutation;
    assert(!hook_CloneObject(NULL));
    assert(runtime_exact_lookup_mutation!=mutation);
    mutation=runtime_exact_lookup_mutation;
    assert(!hook_CloneNode(NULL,NULL));
    assert(runtime_exact_lookup_mutation!=mutation);
    mutation=runtime_exact_lookup_mutation;
    hook_AppMain_Command(NULL);
    assert(runtime_exact_lookup_mutation!=mutation);
    assert(!runtime_exact_lookup_scope_active);

    for (int scoped=0; scoped<2; ++scoped) {
        double start=test_seconds(); lookup_calls=0;
        for (int frame=0; frame<400; ++frame) {
            runtime_exact_lookup_end();
            if (scoped) runtime_exact_lookup_begin();
            for (int i=0;i<40;++i) assert(!lookup(missing));
        }
        printf("BENCH %s: %.3f ms, %u engine lookups / 400 updates\n",
            scoped?"scoped misses":"uncached misses",(test_seconds()-start)*1000,lookup_calls);
        assert(lookup_calls == (scoped ? 425600u : 16400000u));
    }
    runtime_exact_lookup_end();
    return 0;
}
