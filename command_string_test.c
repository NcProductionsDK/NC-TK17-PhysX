#include "NC-TK17-PhysX.c"

typedef struct test_engine_string_t {
    DWORD hash;
    DWORD unused[3];
    DWORD size;
    char text[64];
} test_engine_string_t;

static void require(int ok, const char *message)
{
    if (!ok) { fprintf(stderr, "FAIL: %s\n", message); exit(1); }
}

static void string_init(test_engine_string_t *s, const char *text)
{
    memset(s, 0, sizeof(*s));
    s->size = strlen(text) + 1;
    strcpy(s->text, text);
}

static BYTE native_test_editor[0x400];
static BYTE native_test_table[POSEEDIT_TRACKS_OFFSET + 4 * 256 * POSEEDIT_TRACK_SIZE];
static int native_nil, native_empty, native_joint, native_keys;
static void *native_nil_ref = &native_nil, *native_empty_ref = &native_empty;
static int native_called;

static BYTE *native_test_track(void)
{
    return native_test_table + POSEEDIT_TRACKS_OFFSET +
        POSEEDIT_TRACK_PENIS_JOINT01 * POSEEDIT_TRACK_SIZE;
}

static DWORD __cdecl native_command_boundary(void *args)
{
    BYTE *track = native_test_track();
    (void)args;
    native_called++;
    require(poseedit_file_command_depth == 1,
            "real command hook failed to enter pose replacement guard");
    require(*(void**)(track+4) == &native_joint &&
            *(void**)(track+0x24) == &native_keys,
            "real command hook did not restore keys before calling native handler");
    *(void**)(track+0x24) = native_empty_ref;
    return 0;
}

static void test_full_command_hook(HMODULE app, void *args, const void *cmd_ref)
{
    /* Cmd's engine StringRef normally comes from APP's startup constructor.
       Initialize that data slot in this isolated code-only mapping. */
    DWORD protection;
    BYTE *cmd_slot = (BYTE*)app + APP_MAIN_COMMAND_NAME_STRINGREF_OFFSET;
    require(VirtualProtect(cmd_slot, sizeof(void*), PAGE_READWRITE, &protection),
            "mapped APP Cmd slot unavailable");
    memcpy(cmd_slot, &cmd_ref, sizeof(cmd_ref));
    VirtualProtect(cmd_slot, sizeof(void*), protection, &protection);
    /* Mark unrelated hooks resolved so this test never patches game/test
       code or invokes unresolved DLL imports. Only the two native string
       functions and the production command hook are exercised. */
#define RESOLVED_POINTER(name) name = (__typeof__(name))(uintptr_t)1
    engine_symbols_attempted = 1;
    poseedit_reset_hook_attempted = 1;
    RESOLVED_POINTER(engine_FindObjC);
    RESOLVED_POINTER(engine_GetModelViewRotationPivot);
    RESOLVED_POINTER(real_AppTracker_SetWorldMatrixInverse);
    RESOLVED_POINTER(tramp_PoseEdit_UpdateObjectsFromTracks);
    RESOLVED_POINTER(tramp_AppBase_ProcessAnimation);
    RESOLVED_POINTER(tramp_RuntimeRotationVectorWrite);
    RESOLVED_POINTER(tramp_RuntimeJointRotationAxisWrite);
#undef RESOLVED_POINTER
    runtime_animation_member_setters_installed = 1;
    runtime_blendcontrol_weight_setter_installed = 1;
    addon_constraint_count_getter_installed = 1;
    captured_poseedit_this = native_test_editor;
    captured_poseedit_editpose = native_test_table;
    *(void**)(native_test_editor + POSEEDIT_EDITPOSE_OFFSET) = native_test_table;
    body_chain_physics_cfg.poseeditor_total_tracks = 256;
    engine_G_NilWeakObjTarget_ptr = &native_nil_ref;
    engine_G_NullArray_ptr = &native_empty_ref;
    body_chain_person_state_t *state = &body_chain_person_states[0];
    state->pose_track_suppressed = 1;
    state->pose_track_base = native_test_track();
    state->pose_track_saved_obj = &native_joint;
    state->pose_track_saved_track_data = &native_keys;
    *(void**)(native_test_track()+4) = native_nil_ref;
    *(void**)(native_test_track()+0x24) = native_empty_ref;
    real_AppMain_Command = native_command_boundary;
    require(hook_AppMain_Command(args) == 0 && native_called == 1,
            "full command dispatch did not reach native handler exactly once");
    restore_poseeditor_joint01_track(state);
    require(*(void**)(native_test_track()+0x24) == native_empty_ref &&
            !poseedit_file_command_depth, "full command path resurrected old keys");
    defaults_cfg.debug = 0;
    require(normal_log_line_allowed("PoseEdit file handoff dispatch") &&
            normal_log_line_allowed("PoseEdit file handoff begin") &&
            normal_log_line_allowed("PoseEdit file handoff end"),
            "handoff evidence hidden by normal logging");
    puts("PASS: actual AppMain hook decodes native Cmd/Exec, releases old tracks before handler, and cannot restore them afterwards");
}

static int lifecycle_calls, lifecycle_stage;
static DWORD __cdecl lifecycle_command_boundary(void *args)
{
    (void)args;
    lifecycle_calls++;
    require(!room_wind_cfg.active && !room_wind_observed_scene_key[0],
        "native disposal began before old room wind was retired");
    require(room_wind_observations_blocked == (lifecycle_stage != 1),
        "new room observation gate wrong at native dispatch");
    require(physx_shutting_down == (lifecycle_stage == 2),
        "shutdown guard not armed before native Exit");
    if (lifecycle_stage == 2) physx_tick(); /* engine pointers are deliberately invalid */
    return 123;
}

int main(void)
{
    char path[MAX_PATH * 4], out[64];
    test_engine_string_t exec, mode, cmd, exec_value, mode_value, cmd_value;
    void *exec_pointer, *mode_pointer, *cmd_pointer;
    struct { int size; int dummy; } empty = {0};
    struct {
        int size;
        struct {
            int count;
            void *overflow;
            struct { void *name; void *value; } entry[4];
        } bucket;
    } buckets = {0};
    void *namehash = &buckets.bucket;
    lstrcpynA(path, __FILE__, sizeof(path));
    char *slash = strrchr(path, '\\');
    char *forward = strrchr(path, '/');
    if (!slash || (forward && forward > slash)) slash = forward;
    require(slash != NULL, "absolute test source path missing");
    *slash = 0;
    lstrcatA(path, "/../../The Klub 17/Binaries/ThriXXX010278-SYS.dll");
    /* Map code only: no game startup, DllMain, registration or hooks. These
       two verified routines use local string/hash data and internal code. */
    HMODULE sys = LoadLibraryExA(path, NULL, DONT_RESOLVE_DLL_REFERENCES);
    require(sys != NULL, "could not map installed SYS library for native test");
    engine_StringRefHash32 = (stringref_hash32_t)GetProcAddress(sys,
        "?Hash32@StringRef@Bionic@@QBEIXZ");
    engine_NameHashFind = (namehash_find_t)GetProcAddress(sys,
        "?Find@NameHash@Bionic@@QBEPAXIABVStringRef@2@@Z");
    require(engine_StringRefHash32 && engine_NameHashFind, "native exports missing");
    string_init(&exec, "Exec"); string_init(&mode, "Mode");
    string_init(&exec_value, "File_New"); string_init(&mode_value, "PoseEdit");
    exec_pointer = exec_value.text; mode_pointer = mode_value.text;
    buckets.size = 1;
    buckets.bucket.count = 2;
    buckets.bucket.overflow = &empty.dummy;
    buckets.bucket.entry[0].name = exec.text;
    buckets.bucket.entry[0].value = &exec_pointer;
    buckets.bucket.entry[1].name = mode.text;
    buckets.bucket.entry[1].value = &mode_pointer;
    require(engine_NameHashFind(&namehash, engine_StringRefHash32(exec.text), exec.text)
            == &exec_pointer, "native fixture invalid");
    for (int i = 0; i < 10; i++) {
        require(app_main_command_string_arg(&namehash, "Exec", out, sizeof(out)) &&
                !strcmp(out, "File_New"), "production Exec lookup failed against native engine");
        require(app_main_command_string_arg(&namehash, "Mode", out, sizeof(out)) &&
                !strcmp(out, "PoseEdit"), "alternating Mode lookup failed against native engine");
    }
    require(!app_main_command_string_arg(&namehash, "Missing", out, sizeof(out)),
            "missing field returned a value");
    puts("PASS: production Exec/Mode command decoding with installed native Hash32 and NameHash::Find");
    string_init(&cmd, "Cmd"); string_init(&cmd_value, "PoseEdit");
    cmd_pointer = cmd_value.text;
    buckets.bucket.entry[2].name = cmd.text;
    buckets.bucket.entry[2].value = &cmd_pointer;
    buckets.bucket.count = 3;
    strcpy(strstr(path, "ThriXXX010278-SYS.dll"), "ThriXXX010278-APP.dll");
    HMODULE app = LoadLibraryExA(path, NULL, DONT_RESOLVE_DLL_REFERENCES);
    require(app != NULL, "could not map APP command metadata");
    test_full_command_hook(app, &namehash, cmd.text);
    real_AppMain_Command=lifecycle_command_boundary;
    const char *lifecycle_commands[]={"Game_Abort","NewGame_Start","Exit"};
    for(lifecycle_stage=0;lifecycle_stage<3;lifecycle_stage++) {
        room_wind_cfg.active=room_wind_cfg.enabled=1;
        lstrcpyA(room_wind_observed_scene_key,"Luder\\Room\\OldRoom");
        string_init(&cmd_value,lifecycle_commands[lifecycle_stage]);
        require(hook_AppMain_Command(&namehash)==123,
            "lifecycle hook changed native return value");
        require(lifecycle_calls==lifecycle_stage+1,"lifecycle native command not called exactly once");
    }
    puts("PASS: native Cmd decoding retires room state before Abort/Start/Exit handlers, preserves return values and dispatches once");
    FreeLibrary(app);
    FreeLibrary(sys);
    return 0;
}
