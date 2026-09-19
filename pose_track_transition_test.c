#include "NC-TK17-PhysX.c"

static BYTE editor[0x400];
static BYTE table[POSEEDIT_TRACKS_OFFSET + 4 * 256 * POSEEDIT_TRACK_SIZE];
static BYTE other_table[sizeof(table)];
static int nil_token, empty_token, joint_tokens[4][3], keys_a, keys_b;
static void *nil_ref = &nil_token, *empty_ref = &empty_token;

static void require(int ok, const char *message)
{
    if (!ok) { fprintf(stderr, "FAIL: %s\n", message); exit(1); }
}

static int prepare_file_command(const char *exec)
{
    return physx_poseedit_prepare_file_command(exec);
}

static BYTE *slot(int person, int joint)
{
    const int ids[3] = {POSEEDIT_TRACK_PENIS_JOINT01,
        POSEEDIT_TRACK_PENIS_JOINT02, POSEEDIT_TRACK_PENIS_JOINT03};
    return table + POSEEDIT_TRACKS_OFFSET +
        (person * 256 + ids[joint]) * POSEEDIT_TRACK_SIZE;
}

static void setup(void)
{
    memset(editor, 0, sizeof(editor));
    memset(table, 0, sizeof(table));
    memset(other_table, 0, sizeof(other_table));
    memset(body_chain_person_states, 0, sizeof(body_chain_person_states));
    memset(testicle_physics_states, 0, sizeof(testicle_physics_states));
    memset(breasts_physics_states, 0, sizeof(breasts_physics_states));
    memset(butt_physics_states, 0, sizeof(butt_physics_states));
    captured_poseedit_this = editor;
    captured_poseedit_editpose = table;
    captured_poseedit_tracks_offset = POSEEDIT_TRACKS_OFFSET;
    *(void**)(editor + POSEEDIT_EDITPOSE_OFFSET) = table;
    body_chain_physics_cfg.poseeditor_total_tracks = 256;
    engine_G_NilWeakObjTarget_ptr = &nil_ref;
    engine_G_NullArray_ptr = &empty_ref;
    for (int p = 0; p < 4; p++) for (int j = 0; j < 3; j++) {
        *(void**)(slot(p, j) + 4) = &joint_tokens[p][j];
        *(void**)(slot(p, j) + 0x24) = &keys_a;
    }
}

/* Fixture for the snapshots produced when the three live tracks are detached.
   Restoration and the command handoff use the actual production functions. */
static void own(int person)
{
    body_chain_person_state_t *s = &body_chain_person_states[person];
    for (int j = 0; j < 3; j++) {
        BYTE *b = slot(person, j);
        if (j == 0) {
            s->pose_track_base = b;
            s->pose_track_saved_obj = *(void**)(b + 4);
            s->pose_track_saved_track_data = *(void**)(b + 0x24);
            s->pose_track_suppressed = 1;
        } else {
            s->pose_track_extra_base[j-1] = b;
            s->pose_track_extra_saved_obj[j-1] = *(void**)(b + 4);
            s->pose_track_extra_saved_track_data[j-1] = *(void**)(b + 0x24);
            s->pose_track_extra_suppressed[j-1] = 1;
        }
        *(void**)(b + 4) = nil_ref;
        *(void**)(b + 0x24) = empty_ref;
    }
}

static void blank_pose_regression(void)
{
    setup();
    for (int p = 0; p < 4; p++) own(p);
    prepare_file_command("File_New");
    /* A blank load clears keys in the existing track table. The empty-array
       value is indistinguishable from the plugin's suppression marker. */
    for (int p = 0; p < 4; p++) for (int j = 0; j < 3; j++)
        *(void**)(slot(p,j) + 0x24) = empty_ref;
    for (int p = 0; p < 4; p++) {
        restore_poseeditor_joint01_track(&body_chain_person_states[p]);
        for (int j = 0; j < 3; j++) {
            require(*(void**)(slot(p,j) + 0x24) == empty_ref,
                    "blank pose inherited the previous pose's saved keyframes");
            require(*(void**)(slot(p,j) + 4) == &joint_tokens[p][j],
                    "blank pose lost its live joint target");
        }
    }
    puts("PASS: blank pose cannot resurrect previous keys for any person or joint");
}

static void command_and_load_tests(void)
{
    const char *replace[] = {"File_New", "File_New_DoubleClicked",
        "File_Load", "File_Load_DoubleClicked"};
    const char *unrelated[] = {"Ask", "LoadFrom", "File_New_Clicked",
        "File_Load_Clicked", "File_Load_RightClicked", "File_Save_Ask", "Play", ""};
    for (int n = 0; n < 4; n++) {
        setup(); own(0);
        require(prepare_file_command(replace[n]), "actual file action not handled");
        for (int j = 0; j < 3; j++) {
            require(*(void**)(slot(0,j) + 4) == &joint_tokens[0][j] &&
                    *(void**)(slot(0,j) + 0x24) == &keys_a,
                    "native loader did not receive original target and keys");
            /* Successful in-place load supplies a different pose. */
            *(void**)(slot(0,j) + 0x24) = &keys_b;
        }
        physx_poseedit_finish_file_command(replace[n], 0);
        own(0);
        restore_poseeditor_joint01_track(&body_chain_person_states[0]);
        for (int j = 0; j < 3; j++)
            require(*(void**)(slot(0,j) + 0x24) == &keys_b,
                    "OFF restored earlier pose instead of latest load");
    }
    setup(); own(0);
    require(prepare_file_command("File_Load"), "failed-load test not prepared");
    physx_poseedit_finish_file_command("File_Load", 0x80000001u);
    own(0); restore_poseeditor_joint01_track(&body_chain_person_states[0]);
    require(*(void**)(slot(0,0) + 0x24) == &keys_a, "failed load lost original keys");
    for (int n = 0; n < (int)(sizeof(unrelated)/sizeof(unrelated[0])); n++) {
        setup(); own(0);
        require(!prepare_file_command(unrelated[n]), "dialog/selection/save/play changed ownership");
        require(body_chain_person_states[0].pose_track_suppressed,
                "cancelled dialog dropped ownership");
        restore_poseeditor_joint01_track(&body_chain_person_states[0]);
        require(*(void**)(slot(0,0) + 0x24) == &keys_a, "cancel lost current pose");
    }
    puts("PASS: OK/double-click New/Load, replacement data, failure, cancellation and ordinary OFF");
}

static void stale_and_partial_tests(void)
{
    setup(); own(0);
    /* A new table without InitTracks must not cause writes to the old table. */
    *(void**)(editor + POSEEDIT_EDITPOSE_OFFSET) = other_table;
    require(prepare_file_command("File_New"), "replacement table not recognized");
    require(*(void**)(slot(0,0) + 4) == nil_ref &&
            *(void**)(slot(0,0) + 0x24) == empty_ref,
            "stale table was written during handoff");
    require(!body_chain_person_states[0].pose_track_suppressed &&
            captured_poseedit_editpose == other_table,
            "stale ownership or table capture survived");
    setup(); own(0);
    *(void**)(slot(0,0) + 0x24) = &keys_b;
    prepare_file_command("File_Load");
    require(*(void**)(slot(0,0) + 0x24) == &keys_b &&
            *(void**)(slot(0,0) + 4) == &joint_tokens[0][0],
            "partial native replacement was overwritten or left disconnected");
    setup(); own(0);
    *(void**)(slot(0,0) + 4) = &joint_tokens[1][0];
    prepare_file_command("File_New");
    require(*(void**)(slot(0,0) + 4) == &joint_tokens[1][0] &&
            *(void**)(slot(0,0) + 0x24) == empty_ref,
            "new native owner inherited old keys");
    setup(); own(0);
    body_chain_person_states[0].pose_track_base = slot(1,0);
    *(void**)(slot(1,0) + 4) = nil_ref;
    *(void**)(slot(1,0) + 0x24) = empty_ref;
    prepare_file_command("File_New");
    require(*(void**)(slot(1,0) + 0x24) == empty_ref, "wrong person's slot was restored");
    puts("PASS: replaced table, newer keys, changed target and person boundaries");
}

static void paired_and_reentry_tests(void)
{
    setup();
    poseeditor_paired_bone_track_state_t *paired[] = {
        &breasts_physics_states[0].pose_tracks, &butt_physics_states[0].pose_tracks};
    for (int i = 0; i < 2; i++) {
        BYTE *b = slot(0,i);
        paired[i]->base[0] = b;
        paired[i]->saved_obj[0] = *(void**)(b+4);
        paired[i]->saved_track_data[0] = &keys_a;
        paired[i]->suppressed[0] = 1;
        *(void**)(b+0x24) = empty_ref;
        if (!i) *(void**)(b+4) = nil_ref;
    }
    prepare_file_command("File_New");
    for (int i = 0; i < 2; i++) {
        require(*(void**)(slot(0,i)+0x24) == &keys_a && !paired[i]->suppressed[0],
                "paired ownership not returned before native clear");
        *(void**)(slot(0,i)+0x24) = empty_ref;
        restore_poseeditor_paired_bone_tracks(paired[i], 1);
        require(*(void**)(slot(0,i)+0x24) == empty_ref, "paired old keys resurrected");
    }
    poseedit_file_command_depth = 1;
    unsigned int serial = physx_simulation_serial;
    physx_tick();
    require(physx_simulation_serial == serial, "nested render tick ran during native load");
    poseedit_file_command_depth = 0;
    puts("PASS: paired-track ownership and re-entrant frame guard");
}

static DWORD __cdecl fake_native_command(void *args)
{
    int *depth = (int*)args;
    require(poseedit_file_command_depth == *depth, "native command guard depth wrong");
    require(!body_chain_person_states[0].pose_track_suppressed &&
            *(void**)(slot(0,0)+0x24) == &keys_a,
            "native handler ran before original tracks were reconnected");
    if (*depth == 1) {
        int nested = 2;
        require(physx_poseedit_execute_file_command(&nested, "File_New") == 123,
                "nested native result changed");
    }
    /* The actual game handler is represented only at this boundary. */
    if (*depth == 1) for (int j = 0; j < 3; j++)
        *(void**)(slot(0,j)+0x24) = empty_ref;
    return 123;
}

static void dispatch_test(void)
{
    setup(); own(0);
    real_AppMain_Command = fake_native_command;
    int depth = 1;
    require(physx_poseedit_execute_file_command(&depth, "File_New_DoubleClicked") == 123,
            "native result was not passed through");
    require(!poseedit_file_command_depth, "command guard leaked after return");
    restore_poseeditor_joint01_track(&body_chain_person_states[0]);
    require(*(void**)(slot(0,0)+0x24) == empty_ref, "dispatch restored stale keys");
    real_AppMain_Command = NULL;
    puts("PASS: production native-command wrapper ordering, nested calls and result passthrough");
}

static DWORD __cdecl queued_native_command(void *args)
{
    (void)args;
    *(int*)(editor + 0x1b8) = 1;
    return 0; /* The command returns before clearing any track. */
}

static void *replacement_keys;
static void THISCALL deferred_native_reset(void *self)
{
    require(self == editor, "deferred reset received wrong editor");
#ifndef TEST_WITHOUT_DEFERRED_HANDOFF
    require(poseedit_file_command_depth == 1, "deferred reset is not guarded");
    for (int p = 0; p < 4; p++) {
        require(!body_chain_person_states[p].pose_track_suppressed,
                "deferred reset retained a saved old track");
        for (int j = 0; j < 3; j++)
            require(*(void**)(slot(p,j)+4) == &joint_tokens[p][j],
                    "deferred reset received a disconnected joint");
    }
#endif
    /* Native 0x4ce550 clears key arrays without replacing joint targets.
       Native 0x4c8090 subsequently evaluates tracks several times while
       importing the new pose. Exercise the production early-frame guards. */
    for (int p = 0; p < 4; p++) for (int j = 0; j < 3; j++)
        *(void**)(slot(p,j)+0x24) = replacement_keys;
#ifndef TEST_WITHOUT_DEFERRED_HANDOFF
    unsigned int serial = physx_simulation_serial;
    physx_tick();
    physx_prepare_genital_reveal("deferred-reset-test");
    require(physx_simulation_serial == serial, "physics ran inside native reset");
#endif
}

static void deferred_reset_regression(void)
{
    for (int loaded = 0; loaded < 2; loaded++) {
        setup();
        for (int p = 0; p < 4; p++) own(p);
        real_AppMain_Command = queued_native_command;
        require(physx_poseedit_execute_file_command(NULL, "File_New_DoubleClicked") == 0,
                "queued command failed");
        require(!poseedit_file_command_depth && *(int*)(editor+0x1b8),
                "fixture did not defer native reset beyond command return");
        /* This is the missing frame in the original regression: the game
           still has the old table, so the next PhysX pass owns it again. */
        for (int p = 0; p < 4; p++) own(p);
        replacement_keys = loaded ? (void*)&keys_b : empty_ref;
        tramp_PoseEdit_ResetPose = deferred_native_reset;
#ifdef TEST_WITHOUT_DEFERRED_HANDOFF
        deferred_native_reset(editor);
#else
        hook_PoseEdit_ResetPose(editor);
#endif
        *(int*)(editor+0x1b8) = 0;
        require(!poseedit_file_command_depth, "deferred reset leaked its guard");
        for (int p = 0; p < 4; p++) {
            restore_poseeditor_joint01_track(&body_chain_person_states[p]);
            for (int j = 0; j < 3; j++)
                require(*(void**)(slot(p,j)+0x24) == replacement_keys,
                        "deferred blank reset resurrected the previous animation");
            own(p);
            restore_poseeditor_joint01_track(&body_chain_person_states[p]);
            require(*(void**)(slot(p,0)+0x24) == replacement_keys,
                    "ON/OFF after deferred reset lost the resulting pose");
        }
    }
    tramp_PoseEdit_ResetPose = NULL;
    real_AppMain_Command = NULL;
    puts("PASS: deferred native blank/load reset after intervening PhysX frame, all persons/joints, and subsequent ON/OFF");
}

static BYTE output_roots[4][0x100], output_joints[4][3][0x100];
static BYTE axis_trs[4][0x100], axis_spines[4][0x100];
static unsigned int axis_fixture_mask;
static float authored_output[4][3][3];

static void *__cdecl find_output_fixture(const char *name)
{
    for (int p = 0; p < 4; p++) {
        char expected[128];
        make_body_runtime_name(expected, sizeof(expected), body_chain_person_name(p), "root");
        if (!strcmp(name, expected)) return output_roots[p];
        make_body_runtime_name(expected, sizeof(expected), body_chain_person_name(p), "TRS_group");
        if (!strcmp(name, expected) && (axis_fixture_mask & (1u << p))) return axis_trs[p];
        make_body_runtime_name(expected, sizeof(expected), body_chain_person_name(p), "spine_joint04");
        if (!strcmp(name, expected) && (axis_fixture_mask & (1u << p))) return axis_spines[p];
        for (int j = 0; j < 3; j++) {
            char node[32];
            snprintf(node, sizeof(node), "Spenis_joint%02d", j+1);
            make_body_runtime_name(expected, sizeof(expected), body_chain_person_name(p), node);
            if (!strcmp(name, expected)) return output_joints[p][j];
        }
    }
    return NULL;
}

static float *fixture_output(int p, int j)
{
    return (float*)(output_joints[p][j] + body_chain_physics_person_cfg[p].output_offset);
}

static void capture_and_animate_output(void)
{
    for (int p = 0; p < 4; p++) {
        body_chain_person_state_t *state = &body_chain_person_states[p];
        state->initialized = 1;
        state->root_raw = output_roots[p];
        state->output_handoff_rest_valid = 1;
        for (int j = 0; j < 3; j++) {
            state->joint_raw[j] = output_joints[p][j];
            memcpy(state->output_handoff_rest[j], fixture_output(p,j), sizeof(float)*3);
            /* Reported bad screenshot: the live simulation output, not rest. */
            fixture_output(p,j)[0] = 0;
            fixture_output(p,j)[1] = 1.785043f;
            fixture_output(p,j)[2] = 3.056113f;
        }
        own(p);
    }
}

static void output_rest_regression(void)
{
    setup();
    engine_FindObjC = find_output_fixture;
    for (int p = 0; p < 4; p++) {
        body_chain_physics_person_cfg[p].output_offset = 0x6c + p*4;
        for (int j = 0; j < 3; j++) {
            const float defaults[3] = {-47.5f, -17.16f, -6.988f};
            authored_output[p][j][0] = p * 2.0f;
            authored_output[p][j][1] = p * -3.0f;
            authored_output[p][j][2] = defaults[j] + p;
            memcpy(fixture_output(p,j), authored_output[p][j], sizeof(float)*3);
        }
    }
    /* Load, New command, deferred New reset: every boundary may be followed
       by another physics frame. Native tracks do not write these separate
       Spenis output fields, so the original rest must survive each boundary. */
    for (int step = 0; step < 3; step++) {
        capture_and_animate_output();
        require(physx_poseedit_prepare_replacement(step == 1 ? "File_New" : "native-reset"),
                "output handoff boundary was not prepared");
        for (int p = 0; p < 4; p++) for (int j = 0; j < 3; j++)
            require(!memcmp(fixture_output(p,j), authored_output[p][j], sizeof(float)*3),
                    "pose transition kept physics output instead of the authored default rotation");
    }
    capture_and_animate_output();
    for (int p = 0; p < 4; p++) {
        body_profile_set_active_person_config(p);
        require(restore_body_chain_output_rest_for_person(p, &body_chain_person_states[p]) == 7,
                "final OFF handoff did not restore the live output");
        for (int j = 0; j < 3; j++)
            require(!memcmp(fixture_output(p,j), authored_output[p][j], sizeof(float)*3),
                    "OFF after pose transitions restored a physics-contaminated snapshot");
    }
    body_profile_set_active_person_config(-1);
    capture_and_animate_output();
    body_chain_person_states[0].root_raw = output_roots[1];
    body_chain_person_states[1].joint_raw[1] = output_joints[2][1];
    physx_poseedit_prepare_replacement("native-reset");
    require(fixture_output(0,0)[2] == 3.056113f && fixture_output(1,0)[2] == 3.056113f,
            "output handoff wrote through changed skeleton ownership");
    engine_FindObjC = NULL;
    puts("PASS: authored output rest survives load/New/deferred reset/OFF, per-person values and offsets, replaced skeletons are untouched");
}

static struct { int count; BYTE key[0x30]; } incoming_keys[4][3];
static int queue_fail;

static DWORD THISCALL accept_pose_fixture(void *self, void *pose, void *animation, void *options)
{
    require(self == editor && pose == &keys_a && animation == &keys_b && options == NULL,
            "native pose queue arguments changed");
    require(poseedit_file_command_depth == 1 && !body_chain_person_states[0].initialized,
            "native queue did not release PhysX before scene loading");
    if (queue_fail) return 0x8000000au;
    /* Incoming scene supplies its native rotations during asynchronous load.
       These differ from the old animated pose's saved output snapshot. */
    for (int p = 0; p < 4; p++) for (int j = 0; j < 3; j++)
        memcpy(fixture_output(p,j), authored_output[p][j], sizeof(float)*3);
    *(int*)(editor + POSEEDIT_RESET_PENDING_OFFSET) = 1;
    return 0;
}

static void THISCALL bake_loaded_pose_fixture(void *self)
{
    require(self == editor && physx_poseedit_transition_busy(), "native reset was unguarded");
    for (int p = 0; p < 4; p++) for (int j = 0; j < 3; j++) {
        incoming_keys[p][j].count = 1;
        memcpy(incoming_keys[p][j].key, fixture_output(p,j), sizeof(float)*3);
        *(void**)(slot(p,j)+0x24) = incoming_keys[p][j].key;
    }
}

static void pending_scene_output_regression(void)
{
    setup();
    engine_FindObjC = find_output_fixture;
    body_chain_poseeditor_mode_active = 1;
    queue_fail = 0;
    poseedit_penis_resume_mask = 0;
    for (int p = 0; p < 4; p++) {
        body_chain_physics_person_cfg[p].enabled = 1;
        body_chain_physics_person_cfg[p].enabled_person[p] = 1;
    }
    for (int p = 0; p < 4; p++) for (int j = 0; j < 3; j++) {
        /* Start ON in an animated pose, not a conveniently blank pose. */
        fixture_output(p,j)[0] = -13.10215f;
        fixture_output(p,j)[1] = -13.67511f;
        fixture_output(p,j)[2] = -2.66455f;
    }
    capture_and_animate_output();
    tramp_PoseEdit_QueuePose = accept_pose_fixture;
    tramp_PoseEdit_ResetPose = bake_loaded_pose_fixture;
    require(hook_PoseEdit_QueuePose(editor, &keys_a, &keys_b, NULL) == 0,
            "accepted native queue result changed");
    require(!poseedit_file_command_depth, "synchronous queue guard leaked");
    require(poseedit_penis_resume_mask == 15u, "pose queue did not request a smooth restart for active persons");
    require(physx_poseedit_transition_busy(), "physics resumed while the incoming scene was waiting to become tracks");
    unsigned int serial = physx_simulation_serial;
    int early_sample = physx_genital_early_sample_done;
    for (int frame = 0; frame < 20; frame++) {
        physx_tick();
        physx_prepare_genital_reveal("pending-scene-test");
    }
    require(physx_simulation_serial == serial && physx_genital_early_sample_done == early_sample,
            "a regular/early physics update ran during asynchronous pose loading");
    body_profile_set_active_person_config(0);
    require(!body_chain_seed_pose_load_resume(0, &body_chain_person_states[0], authored_output[0]) &&
            poseedit_penis_resume_mask == 15u, "smooth restart entered an unfinished native load");
    body_profile_set_active_person_config(-1);
    hook_PoseEdit_ResetPose(editor);
    require(physx_poseedit_transition_busy(), "physics resumed before native reset flag cleared");
    *(int*)(editor + POSEEDIT_RESET_PENDING_OFFSET) = 0;
    require(!physx_poseedit_transition_busy(), "completed pose load left physics paused");
    capture_and_animate_output();
    for (int p = 0; p < 4; p++) {
        body_profile_set_active_person_config(p);
        restore_body_chain_output_rest_for_person(p, &body_chain_person_states[p]);
        restore_poseeditor_joint01_track(&body_chain_person_states[p]);
        for (int j = 0; j < 3; j++) {
            require(!memcmp(incoming_keys[p][j].key, authored_output[p][j], sizeof(float)*3),
                    "new pose baked old animated/physics rotation into its single key");
            require(!memcmp(fixture_output(p,j), authored_output[p][j], sizeof(float)*3),
                    "OFF did not restore the incoming pose's own default");
        }
    }
    body_profile_set_active_person_config(-1);
    queue_fail = 1;
    require(hook_PoseEdit_QueuePose(editor, &keys_a, &keys_b, NULL) == 0x8000000au &&
            !physx_poseedit_transition_busy() && !poseedit_penis_resume_mask,
            "failed pose queue left physics paused or a stale restart request");
    *(int*)(editor + POSEEDIT_RESET_PENDING_OFFSET) = 1;
    body_chain_poseeditor_mode_active = 0;
    require(!physx_poseedit_transition_busy(), "stale PoseEditor flag blocked FreeMode/Customizer");
    body_chain_poseeditor_mode_active = 1;
    *(void**)(editor + POSEEDIT_EDITPOSE_OFFSET) = NULL;
    require(!physx_poseedit_transition_busy(), "missing live editor table kept physics paused");
    engine_FindObjC = NULL;
    tramp_PoseEdit_QueuePose = NULL;
    tramp_PoseEdit_ResetPose = NULL;
    puts("PASS: native queue, incoming defaults, 20 pending frames, baked keys and OFF; failure/completion/mode exit release the pause");
}

static void set_axis_matrix(BYTE *raw, const float matrix[9])
{
    for (int row = 0; row < 3; row++)
        memcpy(raw + 0x78 + row * 0x10, matrix + row * 3, sizeof(float)*3);
}

static void require_neutral_axes(int person)
{
    const float identity[9] = {1,0,0, 0,1,0, 0,0,1};
    body_chain_axis_reference_cache_t *root = &poseeditor_body_chain_axis_reference_cache[person];
    body_chain_axis_reference_cache_t *spine = &poseeditor_breasts_axis_reference_cache[person];
    require(root->valid && spine->valid, "loading skipped the read-only axis observer");
    for (int i = 0; i < 9; i++) {
        require(fabsf(root->basis[i] - identity[i]) < 0.0001f,
                "posed root/camera contaminated the horizontal/vertical reference axes");
        require(fabsf(spine->basis[i] - identity[i]) < 0.0001f,
                "posed spine/camera contaminated the breast reference axes");
    }
}

static void pending_axis_regression(void)
{
    const float camera[9] = {0,1,0, -1,0,0, 0,0,1};
    const float posed[9] = {0,0,1, 1,0,0, 0,1,0};
    setup();
    body_chain_poseeditor_mode_active = 1;
    engine_FindObjC = find_output_fixture;
    tramp_PoseEdit_QueuePose = accept_pose_fixture;
    tramp_PoseEdit_ResetPose = bake_loaded_pose_fixture;
    queue_fail = 0;
    memset(poseeditor_body_chain_axis_reference_cache, 0, sizeof(poseeditor_body_chain_axis_reference_cache));
    memset(poseeditor_breasts_axis_reference_cache, 0, sizeof(poseeditor_breasts_axis_reference_cache));
    for (int p = 0; p < 4; p++) {
        set_axis_matrix(output_roots[p], camera);
        set_axis_matrix(axis_spines[p], camera);
        set_axis_matrix(axis_trs[p], camera);
    }
    /* First skeleton is available before queueing. The others appear during
       loading; both paths must observe before the incoming pose rotates them. */
    axis_fixture_mask = 1;
    hook_PoseEdit_QueuePose(editor, &keys_a, &keys_b, NULL);
    require_neutral_axes(0);
    axis_fixture_mask = 15;
    for (int p = 1; p < 4; p++) {
        poseeditor_body_chain_axis_reference_cache[p].next_validation_tick = 0;
        poseeditor_breasts_axis_reference_cache[p].next_validation_tick = 0;
    }
    unsigned int serial = physx_simulation_serial;
    poseedit_file_command_depth++;
    physx_tick();
    require(!poseeditor_body_chain_axis_reference_cache[1].valid,
            "observer ran inside an unfinished native scene operation");
    poseedit_file_command_depth--;
    physx_tick();
    for (int p = 0; p < 4; p++) {
        require_neutral_axes(p);
        set_axis_matrix(output_roots[p], posed);
        set_axis_matrix(axis_spines[p], posed);
        poseeditor_body_chain_axis_reference_cache[p].next_validation_tick = 0;
        poseeditor_breasts_axis_reference_cache[p].next_validation_tick = 0;
    }
    physx_tick();
    for (int p = 0; p < 4; p++) {
        require_neutral_axes(p);
        for (int j = 0; j < 3; j++)
            require(!memcmp(fixture_output(p,j), authored_output[p][j], sizeof(float)*3),
                    "read-only loading observer changed incoming rotations");
    }
    require(physx_simulation_serial == serial && !physx_physics_phase_busy,
            "loading observer advanced simulation or retained its guard");
    axis_fixture_mask = 0;
    engine_FindObjC = NULL;
    tramp_PoseEdit_QueuePose = NULL;
    tramp_PoseEdit_ResetPose = NULL;
    *(int*)(editor + POSEEDIT_RESET_PENDING_OFFSET) = 0;
    puts("PASS: pre-queue and pending-frame axes stay camera-neutral, survive pose rotation, and never write physics output");
}

static BYTE fade_test_object[0x30], fade_test_module[0x40];
static BYTE fade_test_ready;
static BYTE THISCALL fade_test_predicate(void *self, void *a, void *b)
{
    require(self == fade_test_object && a == &keys_a && b == &keys_b, "native fade arguments changed");
    return fade_test_ready;
}
static DWORD THISCALL queue_fading_pose(void *self, void *p, void *a, void *o)
{
    require(poseedit_file_command_depth && body_chain_person_states[0].initialized &&
            body_chain_person_states[0].pose_track_suppressed,
            "outgoing physics was reset before the fade started");
    *(int*)(editor + POSEEDIT_RESET_PENDING_OFFSET) = 1;
    return 0;
}
static void native_fade_handoff_regression(void)
{
    setup();
    body_chain_poseeditor_mode_active=1;
    engine_FindObjC=find_output_fixture;
    *(void**)(editor+0x28)=fade_test_module;
    *(void**)(fade_test_object+8)=fade_test_module;
    tramp_PoseFade_Ready=fade_test_predicate;
    tramp_PoseEdit_QueuePose=queue_fading_pose;
    tramp_PoseEdit_ResetPose=bake_loaded_pose_fixture;
    for(int p=0;p<4;p++) for(int j=0;j<3;j++)
        memcpy(fixture_output(p,j),authored_output[p][j],sizeof(float)*3);
    capture_and_animate_output();
    require(physx_poseedit_prepare_file_command("File_New"), "New did not arm native fade cleanup");
    hook_PoseEdit_QueuePose(editor,&keys_a,&keys_b,NULL);
    unsigned int serial=physx_simulation_serial;
    for(int frame=0;frame<20;frame++) {
        require(!hook_PoseFade_Ready(fade_test_object,&keys_a,&keys_b), "fade completed too early");
        physx_tick();
        physx_prepare_genital_reveal("native-fade-test");
        for(int p=0;p<4;p++) for(int j=0;j<3;j++)
            require(fixture_output(p,j)[2]==3.056113f && body_chain_person_states[p].pose_track_suppressed,
                    "outgoing physical pose changed during fade to black");
    }
    require(physx_simulation_serial==serial, "physics advanced while the pose was queued");
    fade_test_ready=1;
    require(hook_PoseFade_Ready(fade_test_object,&keys_a,&keys_b), "native fade readiness result changed");
    require(!poseedit_fade_cleanup_editor && physx_poseedit_transition_busy(), "native load resumed physics before reset");
    for(int p=0;p<4;p++) for(int j=0;j<3;j++) {
        require(!memcmp(fixture_output(p,j),authored_output[p][j],sizeof(float)*3), "black-screen cleanup lost the correct OFF rotation");
        require(*(void**)(slot(p,j)+0x24)==&keys_a, "native pose replacement did not receive original tracks");
        /* New native scene values become the next pose's own snapshot. */
        authored_output[p][j][2]-=2.0f;
        memcpy(fixture_output(p,j),authored_output[p][j],sizeof(float)*3);
    }
    hook_PoseEdit_ResetPose(editor);
    *(int*)(editor+POSEEDIT_RESET_PENDING_OFFSET)=0;
    capture_and_animate_output();
    for(int p=0;p<4;p++) {
        body_profile_set_active_person_config(p);
        restore_body_chain_output_rest_for_person(p,&body_chain_person_states[p]);
        restore_poseeditor_joint01_track(&body_chain_person_states[p]);
        for(int j=0;j<3;j++) {
            require(!memcmp(fixture_output(p,j),authored_output[p][j],sizeof(float)*3), "OFF after fade restored wrong rotation");
            require(*(void**)(slot(p,j)+0x24)==incoming_keys[p][j].key, "OFF after fade resurrected previous keys");
        }
    }
    body_profile_set_active_person_config(-1);
    physx_poseedit_begin_replacement("File_Load");
    body_chain_poseeditor_mode_active=0;
    require(!physx_poseedit_transition_busy() && !poseedit_fade_cleanup_editor, "mode exit retained a stale fade wait");
    body_chain_poseeditor_mode_active=1;
    require(!physx_poseedit_transition_busy(), "return to PoseEditor resumed a cancelled fade wait");
    engine_FindObjC=NULL;
    tramp_PoseFade_Ready=NULL;
    tramp_PoseEdit_QueuePose=NULL;
    tramp_PoseEdit_ResetPose=NULL;
    puts("PASS: native fade preserves outgoing penis physics until black; new keys/defaults and later OFF remain correct; mode exit cancels the wait");
}

int main(void)
{
    blank_pose_regression();
    command_and_load_tests();
    stale_and_partial_tests();
    paired_and_reentry_tests();
    dispatch_test();
    deferred_reset_regression();
    output_rest_regression();
    pending_scene_output_regression();
    pending_axis_regression();
    native_fade_handoff_regression();
    return 0;
}
