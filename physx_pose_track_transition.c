/* Native pose replacement is asynchronous. Retain outgoing physics through
   the game's fade, release ownership at its readiness gate, and guard the
   later PoseEditor track reset. Unsupported fade hooks use immediate release. */

/* Read-only transition evidence: a saved output snapshot can itself be an
   animated rotation. Record the native keys separately instead of treating
   that snapshot as proof of the new pose's intended value. No evaluators or
   property setters are invoked, and ordinary frames do not enter this path. */
static void physx_poseedit_trace_rotations(const char *phase, unsigned int persons)
{
    static const int ids[3] = {POSEEDIT_TRACK_PENIS_JOINT01,
        POSEEDIT_TRACK_PENIS_JOINT02, POSEEDIT_TRACK_PENIS_JOINT03};
    if (!engine_FindObjC || !ptr_executable((void*)engine_FindObjC)) return;
    for (int p = 0; p < 4; p++) {
        void *root = NULL, *joints[3] = {0};
        if (!(persons & (1u << p)) ||
            !resolve_body_chain_raws(body_chain_person_name(p), &root, joints)) continue;
        const body_chain_physics_config_t *cfg = &body_chain_physics_person_cfg[p];
        for (int j = 0; j < 3; j++) {
            BYTE *track = poseedit_track_slot(p, ids[j]);
            BYTE *keys = NULL;
            float first[3] = {0}, last[3] = {0}, output[3] = {0};
            int count = -1, keys_read = 0, output_read = 0;
            int first_frame = 0, last_frame = 0;
            if (!track || !ptr_readable(track, POSEEDIT_TRACK_SIZE)) continue;
            keys = *(BYTE**)(track + 0x24);
            if (keys && ptr_readable(keys - sizeof(int), sizeof(int))) {
                count = *(int*)(keys - sizeof(int));
                if (count > 0 && count <= 100000 &&
                    ptr_readable(keys, (size_t)count * 0x30u)) {
                    BYTE *last_key = keys + (size_t)(count - 1) * 0x30u;
                    memcpy(first, keys, sizeof(first));
                    memcpy(last, last_key, sizeof(last));
                    memcpy(&first_frame, keys + 0x24, sizeof(first_frame));
                    memcpy(&last_frame, last_key + 0x24, sizeof(last_frame));
                    keys_read = 1;
                }
            }
            if (joints[j] && cfg->output_offset >= 0 &&
                ptr_readable((BYTE*)joints[j] + cfg->output_offset, sizeof(output))) {
                memcpy(output, (BYTE*)joints[j] + cfg->output_offset, sizeof(output));
                output_read = 1;
            }
            log_line("PoseEdit file handoff rotation-trace phase=%s person=%d joint=%d track=%p target=%p member=0x%08lx keys=%p count=%d keys_read=%d first_frame=%d first=(%.6f,%.6f,%.6f) last_frame=%d last=(%.6f,%.6f,%.6f) output_raw=%p output_offset=0x%x output_read=%d output=(%.6f,%.6f,%.6f)",
                phase, p+1, j+1, track, *(void**)(track+4),
                (unsigned long)*(DWORD*)(track+8), keys, count, keys_read,
                first_frame, first[0], first[1], first[2],
                last_frame, last[0], last[1], last[2], joints[j], cfg->output_offset,
                output_read, output[0], output[1], output[2]);
        }
    }
}

static int physx_poseedit_replaces_pose(const char *exec)
{
    return exec && (!strcmp(exec, "File_New") ||
                    !strcmp(exec, "File_New_DoubleClicked") ||
                    !strcmp(exec, "File_Load") ||
                    !strcmp(exec, "File_Load_DoubleClicked"));
}

static int physx_poseedit_native_reset_pending(void)
{
    BYTE *pe = (BYTE*)captured_poseedit_this;
    /* QueuePose sets this flag after accepting the incoming scene. The game
       clears it only AFTER ResetPose has built/evaluated its new keyframes.
       There must be no PhysX output or snapshot capture in that interval. */
    return tramp_PoseEdit_QueuePose && tramp_PoseEdit_ResetPose &&
        !body_chain_runtime_mode_active() && body_chain_runtime_live_editpose() &&
        ptr_readable(pe + POSEEDIT_RESET_PENDING_OFFSET, sizeof(int)) &&
        *(int*)(pe + POSEEDIT_RESET_PENDING_OFFSET) == 1;
}

static int physx_poseedit_transition_busy(void)
{
    if (poseedit_fade_cleanup_editor &&
        (poseedit_fade_cleanup_editor != captured_poseedit_this ||
         body_chain_runtime_mode_active() || !body_chain_runtime_live_editpose()))
        poseedit_fade_cleanup_editor = NULL;
    return poseedit_file_command_depth || physx_poseedit_native_reset_pending() ||
        (poseedit_fade_cleanup_editor &&
         poseedit_fade_cleanup_editor == captured_poseedit_this &&
         !body_chain_runtime_mode_active() && body_chain_runtime_live_editpose());
}

static int physx_poseedit_track_in_live_person(BYTE *live, void *track,
                                              int person)
{
    ULONG_PTR begin, end, address = (ULONG_PTR)track;
    int count = body_chain_physics_cfg.poseeditor_total_tracks;
    if (!live || !track || person < 0 || person >= 4 ||
        count <= 0 || count > 1024 || captured_poseedit_tracks_offset < 0)
        return 0;
    begin = (ULONG_PTR)live + captured_poseedit_tracks_offset +
            person * count * POSEEDIT_TRACK_SIZE;
    end = begin + count * POSEEDIT_TRACK_SIZE;
    return address >= begin && address < end &&
           (address - begin) % POSEEDIT_TRACK_SIZE == 0 &&
           ptr_readable(track, POSEEDIT_TRACK_SIZE);
}

static int physx_poseedit_reconnect_owned_track(BYTE *live, int person,
    void *track, void *saved_obj, void *saved_data, int suppressed)
{
    BYTE *base = (BYTE*)track;
    void *nil = engine_G_NilWeakObjTarget_ptr ? *engine_G_NilWeakObjTarget_ptr : NULL;
    void *empty = engine_G_NullArray_ptr ? *engine_G_NullArray_ptr : NULL;
    void *current_obj;
    DWORD old;
    if (!suppressed || !nil || !empty || !saved_obj || !saved_data ||
        !physx_poseedit_track_in_live_person(live, track, person)) return 0;
    current_obj = *(void**)(base + 0x04);
    /* An engine replacement belongs to the engine, even in the same slot. */
    if (current_obj != nil && current_obj != saved_obj) return 0;
    if (!VirtualProtect(base, POSEEDIT_TRACK_SIZE, PAGE_READWRITE, &old)) return 0;
    if (current_obj == nil) *(void**)(base + 0x04) = saved_obj;
    if (*(void**)(base + 0x24) == empty)
        *(void**)(base + 0x24) = saved_data;
    VirtualProtect(base, POSEEDIT_TRACK_SIZE, old, &old);
    return 1;
}

static void physx_poseedit_release_chain_tracks(BYTE *live, int person,
                                               body_chain_person_state_t *state)
{
    physx_poseedit_reconnect_owned_track(live, person, state->pose_track_base,
        state->pose_track_saved_obj, state->pose_track_saved_track_data,
        state->pose_track_suppressed);
    for (int j = 0; j < POSEEDIT_EXTRA_PHYSICS_TRACK_COUNT; j++)
        physx_poseedit_reconnect_owned_track(live, person,
            state->pose_track_extra_base[j], state->pose_track_extra_saved_obj[j],
            state->pose_track_extra_saved_track_data[j], state->pose_track_extra_suppressed[j]);
    /* Forget stale allocations without writing into them. */
    body_chain_runtime_forget_penis_pose_tracks(state);
}

static void physx_poseedit_release_paired_tracks(BYTE *live, int person,
    poseeditor_paired_bone_track_state_t *state)
{
    for (int j = 0; j < POSEEDIT_PAIRED_BONE_TRACK_MAX; j++)
        physx_poseedit_reconnect_owned_track(live, person, state->base[j],
            state->saved_obj[j], state->saved_track_data[j], state->suppressed[j]);
    memset(state, 0, sizeof(*state));
}

static int physx_poseedit_prepare_replacement(const char *exec)
{
    BYTE *live;
    live = body_chain_runtime_live_editpose();
    if (!live) return 0;
    poseedit_fade_cleanup_editor = NULL;
    if (poseedit_penis_resume_editor != captured_poseedit_this) {
        poseedit_penis_resume_mask = 0;
        poseedit_penis_resume_editor = captured_poseedit_this;
    }
    /* PoseTracks drive penis_joint*, but PhysX also writes Spenis_joint*.
       Return those separate output channels before discarding the snapshot.
       Otherwise the next initialization saves the last simulated rotation as
       its "pre-PhysX" rest, even when the new pose's tracks are correct.
       The existing OFF handoff validates live root/joint ownership and uses
       the pre-PhysX values. These can be animated values; the new pose's
       native tracks still need to supply its intended rotation. */
    for (int p = 0; p < 4; p++) {
        body_profile_set_active_person_config(p);
        if (body_chain_person_states[p].initialized &&
            body_chain_physics_cfg.enabled && body_chain_physics_cfg.enabled_person[p])
            poseedit_penis_resume_mask |= 1u << p;
        restore_body_chain_output_rest_for_person(p, &body_chain_person_states[p]);
        /* Paired solvers write separate source transforms as well as owning
           PoseTracks. Return their rotation/translation before discarding the
           snapshots, or the next pose captures the last simulated displacement
           as its permanent rest. Only the exact live skeleton may be restored. */
        breasts_physics_person_state_t *paired[2] = {
            &breasts_physics_states[p], &butt_physics_states[p]
        };
        for (int group = 0; group < 2; group++) {
            breasts_physics_person_state_t *state = paired[group];
            if (!(state->output_applied || state->bone_translation_applied) ||
                !(group ? butt_physics_live_ownership_matches(p, state)
                        : breasts_physics_live_ownership_matches(p, state))) continue;
            int restored = group ? butt_physics_apply_output(state, 1, 0)
                                 : breasts_physics_apply_output(state, 1, 0);
            if (restored)
                log_line("PoseEdit file handoff paired-output-restored system=%s person=%d translation=(%.6f,%.6f,%.6f;%.6f,%.6f,%.6f) note=\"removed simulation offsets before releasing the old pose\"",
                    group ? "butt" : "breasts", p + 1,
                    state->source_translation_handoff[0][0], state->source_translation_handoff[0][1], state->source_translation_handoff[0][2],
                    state->source_translation_handoff[1][0], state->source_translation_handoff[1][1], state->source_translation_handoff[1][2]);
        }
    }
    body_profile_set_active_person_config(-1);
    for (int p = 0; p < 4; p++) {
        physx_poseedit_release_chain_tracks(live, p, &body_chain_person_states[p]);
        physx_poseedit_release_chain_tracks(live, p, &testicle_physics_states[p]);
        physx_poseedit_release_paired_tracks(live, p, &breasts_physics_states[p].pose_tracks);
        physx_poseedit_release_paired_tracks(live, p, &butt_physics_states[p].pose_tracks);
    }
    captured_poseedit_editpose = live;
    /* All saved track ownership is now released or discarded. Resetting the
       solvers cannot write old keyframes into a reused/new table afterwards. */
    reset_body_chain_physics();
    poseeditor_track_handoff_pending_mask = 0;
    poseeditor_track_handoff_pending_tick = 0;
    poseeditor_track_handoff_poseedit = NULL;
    poseeditor_track_handoff_editpose = NULL;
    log_line("PoseEdit file handoff begin exec=\"%s\" editpose=%p note=\"returned current tracks to TK17 before native pose replacement; discarded old snapshots\"", exec, live);
    return 1;
}

static int physx_poseedit_begin_replacement(const char *exec)
{
    BYTE *editor = (BYTE*)captured_poseedit_this;
    /* Keep the outgoing physical pose intact while TK17 fades it to black.
       The original readiness callback completes cleanup before native pose
       application. Unknown executables retain the proven immediate handoff. */
    if (tramp_PoseFade_Ready && !body_chain_runtime_mode_active() &&
        body_chain_runtime_live_editpose() &&
        ptr_readable(editor + 0x28, sizeof(void*)) && *(void**)(editor + 0x28)) {
        poseedit_fade_cleanup_editor = editor;
        body_chain_prime_axis_references(GetTickCount());
        log_line("PoseEdit file handoff fade-wait exec=\"%s\" note=\"retain outgoing physics until the native fade readiness gate\"", exec);
        return 1;
    }
    return physx_poseedit_prepare_replacement(exec);
}

static BYTE THISCALL hook_PoseFade_Ready(void *self, void *arg0, void *arg1)
{
    BYTE ready = tramp_PoseFade_Ready ? tramp_PoseFade_Ready(self, arg0, arg1) : 0;
    /* Observe native sequencing without changing the predicate or delaying
       scene application. Several person fades may be interleaved. Log only
       new callbacks/readiness edges, not every waiting frame. A ready result
       alone does not prove black: native disabled-fade paths also return 1. */
    if (defaults_cfg.debug) {
        static struct { void *self, *module; int ready; } observed[8];
        static unsigned int next;
        void *module = ptr_readable((BYTE*)self + 8, sizeof(void*))
            ? *(void**)((BYTE*)self + 8) : NULL;
        int slot;
        for (slot = 0; slot < 8; ++slot)
            if (observed[slot].self == self && observed[slot].module == module) break;
        if (slot == 8) {
            slot = (int)(next++ % 8u);
            observed[slot].self = self;
            observed[slot].module = module;
            observed[slot].ready = -1;
        }
        if (observed[slot].ready != (int)ready) {
            log_line("physics native-fade readiness=%d previous=%d callback=%p module=%p mode=%s customizer=%d mode_handoff_pending=%ld note=\"native result observed unchanged; ready can also mean fade disabled\"",
                (int)ready, observed[slot].ready, self, module,
                body_chain_runtime_mode_active() ? "runtime" : "PoseEditor",
                physx_customizer_active,
                (long)InterlockedCompareExchange(&body_chain_runtime_mode_transition_pending, 0, 0));
            observed[slot].ready = ready;
        }
    }
    BYTE *editor = (BYTE*)poseedit_fade_cleanup_editor;
    if (ready && editor && editor == captured_poseedit_this &&
        !body_chain_runtime_mode_active() &&
        ptr_readable(editor + 0x28, sizeof(void*)) &&
        ptr_readable((BYTE*)self + 8, sizeof(void*)) &&
        *(void**)(editor + 0x28) == *(void**)((BYTE*)self + 8)) {
        poseedit_file_command_depth++;
        physx_poseedit_prepare_replacement("native-fade-ready");
        poseedit_file_command_depth--;
    }
    return ready;
}

static int physx_poseedit_prepare_file_command(const char *exec)
{
    return physx_poseedit_replaces_pose(exec) &&
           physx_poseedit_begin_replacement(exec);
}

static void physx_poseedit_finish_file_command(const char *exec, DWORD result)
{
    BYTE *live = body_chain_runtime_live_editpose();
    captured_poseedit_editpose = live;
    captured_poseedit_tick = GetTickCount();
    /* Cancelled dialogs and failed queues must not affect a later activation.
       On success the native flag remains set through ResetPose's return. */
    if (!physx_poseedit_native_reset_pending()) {
        poseedit_fade_cleanup_editor = NULL;
        poseedit_penis_resume_mask = 0;
    }
    log_line("PoseEdit file handoff end exec=\"%s\" result=0x%08lx editpose=%p pending_reset=%d note=\"physics waits for the native pending-reset flag to clear before binding the resulting pose\"", exec, (unsigned long)result, live, physx_poseedit_transition_busy());
}

static DWORD physx_poseedit_execute_file_command(void *args, const char *exec)
{
    DWORD result;
    int handoff;
    if (!real_AppMain_Command) return 0x80000001u;
    handoff = poseedit_file_command_depth ? 1 :
        physx_poseedit_prepare_file_command(exec);
    if (handoff) poseedit_file_command_depth++;
    result = real_AppMain_Command(args);
    if (handoff && --poseedit_file_command_depth == 0)
        physx_poseedit_finish_file_command(exec, result);
    return result;
}

/* The native command at 0x4e7287 sets PoseEdit+0x1b8. A later update at
   0x4dff2a calls 0x4c8090, which clears track arrays via 0x4ce550 and then
   imports the new pose. Its internal UpdateObjectsFromTracks calls must not
   let an early physics callback reacquire partially initialized tracks. */
static DWORD THISCALL hook_PoseEdit_QueuePose(void *self, void *pose,
                                            void *animation, void *options)
{
    DWORD result;
    int handoff;
    if (!tramp_PoseEdit_QueuePose) return 0x80000001u;
    handoff = self == captured_poseedit_this &&
        (poseedit_file_command_depth ||
         physx_poseedit_begin_replacement("native-queue"));
    if (handoff) poseedit_file_command_depth++;
    /* Observe the existing frame before the queue starts applying the pose.
       This is read-only; physics stays paused through the deferred reset. */
    if (handoff) body_chain_prime_axis_references(GetTickCount());
    result = tramp_PoseEdit_QueuePose(self, pose, animation, options);
    if (handoff && --poseedit_file_command_depth == 0)
        physx_poseedit_finish_file_command("native-queue", result);
    return result;
}

static void THISCALL hook_PoseEdit_ResetPose(void *self)
{
    int handoff;
    if (!tramp_PoseEdit_ResetPose) return;
    handoff = self == captured_poseedit_this &&
        (poseedit_file_command_depth ||
         physx_poseedit_prepare_replacement("native-reset"));
    if (handoff) poseedit_file_command_depth++;
    if (handoff) physx_poseedit_trace_rotations("native-reset-before", 15u);
    tramp_PoseEdit_ResetPose(self);
    if (handoff) physx_poseedit_trace_rotations("native-reset-after", 15u);
    if (handoff && --poseedit_file_command_depth == 0)
        physx_poseedit_finish_file_command("native-reset", 0);
}
