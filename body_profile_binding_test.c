#include "NC-TK17-PhysX.c"
#include <assert.h>

static void reset_bindings(void)
{
    memset(body_profile_pending_bind, 0, sizeof(body_profile_pending_bind));
    memset(body_profile_pending_open, 0, sizeof(body_profile_pending_open));
    for (int p = 0; p < BODY_PROFILE_PERSON_COUNT; p++) {
        body_profile_clear_person_sidecar_a(p, "test-reset");
        body_profile_loaded_body_slot[p] = -1;
    }
}

static void expect_profile(int person, const char *path)
{
    assert(body_profile_person_sidecar_active[person] == (path != NULL));
    if (path) assert(!_stricmp(body_profile_person_sidecar_path[person], path));
}

static int pending_count(int person)
{
    int count = 0;
    for (int i = 0; i < BODY_PROFILE_PENDING_COUNT; i++) {
        if (body_profile_pending_bind[i].active &&
            body_profile_pending_bind[i].person_index == person) count++;
    }
    return count;
}

int main(int argc, char **argv)
{
    char a[MAX_PATH * 4], b[MAX_PATH * 4], no_sidecar[MAX_PATH * 4];
    char ai[MAX_PATH * 4], bi[MAX_PATH * 4], marker[160];
    DWORD hash;
    assert(argc == 2);
    snprintf(a, sizeof(a), "%s/Body.A/Scenes/Shared/Body/body02.bs", argv[1]);
    snprintf(b, sizeof(b), "%s/Body.B/Scenes/Shared/Body/body02.bs", argv[1]);
    snprintf(no_sidecar, sizeof(no_sidecar), "%s/Body.Plain/Scenes/Shared/Body/body02.bs", argv[1]);
    assert(body_profile_build_sidecar_path_a(a, ai, sizeof(ai)));
    assert(body_profile_build_sidecar_path_a(b, bi, sizeof(bi)));

    /* Empty and opaque body files both bind: no scene-content inspection. */
    body_profile_note_virtual_body_scene_a("Shared/Body/body02___01", "test");
    body_profile_note_body_file_a(a);
    expect_profile(0, ai);
    body_profile_note_virtual_body_scene_a("Shared/Body/body02___03", "test");
    body_profile_note_body_file_a(b);
    expect_profile(0, ai);
    expect_profile(2, bi);
    expect_profile(1, NULL);
    breasts_physics_global_cfg.enabled = 1;
    body_profile_rebuild_effective_configs();
    assert(!breasts_physics_person_cfg[0].enabled);
    assert(breasts_physics_person_cfg[1].enabled);
    assert(breasts_physics_person_cfg[2].enabled);
    /* Arbitrary morph/node events cannot invalidate or activate profiles. */
    body_profile_note_tsnode_name_a("Person01Body:EyeUnderL");
    body_profile_note_tsnode_name_a("Person01Body:unrelated_body02_morph");
    expect_profile(0, ai);
    puts("PASS: content-independent binding, per-person overrides, morph independence");

    body_profile_note_virtual_body_scene_a("Shared/Body/body02___01", "test");
    expect_profile(0, NULL);
    body_profile_note_body_file_a(no_sidecar);
    expect_profile(0, NULL);
    expect_profile(2, bi);
    body_profile_rebuild_effective_configs();
    assert(breasts_physics_person_cfg[0].enabled);
    puts("PASS: replacement without sidecar restores global settings for one person");

    reset_bindings();
    body_profile_note_body_file_a(a);
    body_profile_note_tsnode_name_a("Person02Body:body02_blendshape");
    expect_profile(1, NULL);
    body_profile_note_tsnode_name_a("Person02Body:Shared/Body/body02.ma");
    expect_profile(1, ai);
    body_profile_note_body_file_a(a);
    body_profile_note_virtual_body_scene_a("Shared/Body/body02___04", "test");
    expect_profile(1, ai);
    expect_profile(3, ai);
    assert(!pending_count(3));
    body_profile_note_body_file_a(no_sidecar);
    body_profile_note_tsnode_name_a("Person02Body:Shared/Body/body02.ma");
    expect_profile(1, NULL);
    expect_profile(3, ai);
    puts("PASS: file-first events, shared add-on, missing sidecar, exact asset names only");

    reset_bindings();
    body_profile_note_virtual_body_scene_a("Shared/Body/body02___01", "hook");
    body_profile_note_virtual_body_scene_a("Shared/Body/body02___01", "log");
    assert(pending_count(0) == 1);
    body_profile_note_body_file_a(a);
    assert(!pending_count(0));
    body_profile_note_virtual_body_scene_a("Shared/Body/body02___03", "hook");
    body_profile_note_body_file_a(b);
    expect_profile(0, ai);
    expect_profile(2, bi);
    puts("PASS: duplicate load observations do not steal another person's file");

    reset_bindings();
    body_profile_note_body_file_a(a);
    body_profile_note_body_file_a(b);
    body_profile_note_tsnode_name_a("Person01Body:Shared/Body/body02.ma");
    expect_profile(0, NULL);
    puts("PASS: ambiguous file-first candidates do not select an arbitrary add-on");

    reset_bindings();
    hash = body_profile_xxh32_a("Body.A");
    snprintf(marker, sizeof(marker), "LUA/VAR_personsel___body|2|%lu|1", (unsigned long)hash);
    body_profile_note_body_select_marker_a(marker);
    expect_profile(0, ai);
    body_profile_note_virtual_body_scene_a("Shared/Body/body02___01", "test");
    expect_profile(0, ai);
    body_profile_note_body_select_marker_a("LUA/VAR_personsel___hair|2|0|1");
    expect_profile(0, ai);
    body_profile_note_body_select_marker_a("LUA/VAR_personsel___body|2|0|1");
    expect_profile(0, NULL);
    puts("PASS: cached selection ids survive load events; non-body UI events are ignored");

    reset_bindings();
    body_profile_note_body_file_a(a);
    for (int i = 0; i < BODY_PROFILE_PENDING_COUNT; i++) {
        body_profile_pending_open[i].tick = GetTickCount() - 4000u;
    }
    body_profile_note_virtual_body_scene_a("Shared/Body/body02___01", "test");
    expect_profile(0, NULL);
    puts("PASS: expired file candidates cannot bind a later load");
    puts("All body profile binding tests passed.");
    return 0;
}
