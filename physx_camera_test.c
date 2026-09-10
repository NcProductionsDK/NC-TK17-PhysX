static void camera_contamination_test_mat3_from_mat4(const float m[16],
                                                     float out[9])
{
    out[0] = m[0];  out[1] = m[1];  out[2] = m[2];
    out[3] = m[4];  out[4] = m[5];  out[5] = m[6];
    out[6] = m[8];  out[7] = m[9];  out[8] = m[10];
}

static void camera_contamination_test_apply_offset(
    const float input[16],
    float yaw_degrees,
    float pitch_degrees,
    float out[16])
{
    const float deg_to_rad = 0.01745329251994329577f;
    float yaw = yaw_degrees * deg_to_rad;
    float pitch = pitch_degrees * deg_to_rad;
    float cy = cosf(yaw);
    float sy = sinf(yaw);
    float cp = cosf(pitch);
    float sp = sinf(pitch);
    float right[3] = { input[0], input[1], input[2] };
    float up[3] = { input[4], input[5], input[6] };
    float back[3] = { input[8], input[9], input[10] };
    float yaw_right[3];
    float yaw_back[3];
    float pitch_up[3];
    float pitch_back[3];
    int axis;

    memcpy(out, input, sizeof(float) * 16);
    for (axis = 0; axis < 3; axis++) {
        yaw_right[axis] = cy * right[axis] + sy * back[axis];
        yaw_back[axis] = -sy * right[axis] + cy * back[axis];
        pitch_up[axis] = cp * up[axis] + sp * yaw_back[axis];
        pitch_back[axis] = -sp * up[axis] + cp * yaw_back[axis];
    }
    out[0] = yaw_right[0];
    out[1] = yaw_right[1];
    out[2] = yaw_right[2];
    out[4] = pitch_up[0];
    out[5] = pitch_up[1];
    out[6] = pitch_up[2];
    out[8] = pitch_back[0];
    out[9] = pitch_back[1];
    out[10] = pitch_back[2];
}

static void camera_contamination_test_log_mat3(const char *name,
                                               const float m[9],
                                               unsigned int sample_index,
                                               LONG phase)
{
    log_line("camera-contamination-test matrix sample=%u phase=%ld name=\"%s\" values=(%.7f,%.7f,%.7f;%.7f,%.7f,%.7f;%.7f,%.7f,%.7f)",
             sample_index, phase + 1, name,
             m[0], m[1], m[2],
             m[3], m[4], m[5],
             m[6], m[7], m[8]);
}

static int camera_contamination_test_isolating_forces(void)
{
    camera_contamination_test_config_t *cfg =
        &camera_contamination_test_cfg;
    if (!cfg->enabled || cfg->completed) return 0;
    return cfg->ready_tick || cfg->active;
}

static int camera_contamination_test_current_step(
    const camera_contamination_test_step_t **step_out,
    LONG *phase_out)
{
    LONG active = InterlockedCompareExchange(
        &camera_contamination_test_cfg.active, 0, 0);
    LONG phase = InterlockedCompareExchange(
        &camera_contamination_test_cfg.phase, 0, 0);
    if (!active || phase < 0 ||
        phase >= CAMERA_CONTAMINATION_TEST_STEP_COUNT) {
        return 0;
    }
    if (step_out) {
        *step_out = &camera_contamination_test_steps[phase];
    }
    if (phase_out) {
        *phase_out = phase;
    }
    return 1;
}

typedef struct camera_contamination_test_window_find_t {
    DWORD pid;
    HWND hwnd;
} camera_contamination_test_window_find_t;

static BOOL CALLBACK camera_contamination_test_enum_windows(HWND hwnd,
                                                            LPARAM lparam)
{
    camera_contamination_test_window_find_t *find =
        (camera_contamination_test_window_find_t *)lparam;
    DWORD pid = 0;
    if (!find || !hwnd || !IsWindowVisible(hwnd)) return TRUE;
    if (GetWindow(hwnd, GW_OWNER)) return TRUE;
    GetWindowThreadProcessId(hwnd, &pid);
    if (pid != find->pid) return TRUE;
    find->hwnd = hwnd;
    return FALSE;
}

static HWND camera_contamination_test_find_game_window(void)
{
    camera_contamination_test_window_find_t find;
    memset(&find, 0, sizeof(find));
    find.pid = GetCurrentProcessId();
    EnumWindows(camera_contamination_test_enum_windows, (LPARAM)&find);
    return find.hwnd;
}

static int camera_contamination_test_game_foreground(void)
{
    HWND hwnd = GetForegroundWindow();
    DWORD foreground_pid = 0;
    if (!hwnd) return 0;
    GetWindowThreadProcessId(hwnd, &foreground_pid);
    return foreground_pid == GetCurrentProcessId();
}

static int camera_contamination_test_focus_game_window(HWND hwnd)
{
    DWORD foreground_thread = 0;
    DWORD target_thread = 0;
    DWORD current_thread = GetCurrentThreadId();
    HWND foreground = GetForegroundWindow();
    int attach_foreground = 0;
    int attach_target = 0;

    if (!hwnd) return 0;
    if (foreground) {
        foreground_thread = GetWindowThreadProcessId(foreground, NULL);
    }
    target_thread = GetWindowThreadProcessId(hwnd, NULL);

    if (foreground_thread && foreground_thread != current_thread) {
        attach_foreground =
            AttachThreadInput(current_thread, foreground_thread, TRUE);
    }
    if (target_thread && target_thread != current_thread &&
        target_thread != foreground_thread) {
        attach_target =
            AttachThreadInput(current_thread, target_thread, TRUE);
    }

    if (IsIconic(hwnd)) {
        ShowWindow(hwnd, SW_RESTORE);
    } else {
        ShowWindow(hwnd, SW_SHOW);
    }
    SetWindowPos(hwnd, HWND_TOP, 0, 0, 0, 0,
                 SWP_NOMOVE | SWP_NOSIZE | SWP_SHOWWINDOW);
    BringWindowToTop(hwnd);
    SetForegroundWindow(hwnd);
    SetActiveWindow(hwnd);
    SetFocus(hwnd);

    if (attach_target) {
        AttachThreadInput(current_thread, target_thread, FALSE);
    }
    if (attach_foreground) {
        AttachThreadInput(current_thread, foreground_thread, FALSE);
    }

    return camera_contamination_test_game_foreground();
}

static int camera_contamination_test_ensure_game_foreground(
    DWORD now, LONG phase, const camera_contamination_test_step_t *step,
    int target_x, int target_y)
{
    camera_contamination_test_config_t *cfg =
        &camera_contamination_test_cfg;
    HWND hwnd = NULL;

    if (camera_contamination_test_game_foreground()) return 1;
    if (!cfg->auto_focus) return 0;

    hwnd = camera_contamination_test_find_game_window();
    if (hwnd && camera_contamination_test_focus_game_window(hwnd)) {
        if (!cfg->input_drag_last_log_tick ||
            now - cfg->input_drag_last_log_tick >= 1000) {
            cfg->input_drag_last_log_tick = now;
            log_line("camera-contamination-test input-focus phase=%ld/%d label=\"%s\" hwnd=%p result=foreground target_pixels=(%d,%d)",
                     phase + 1,
                     CAMERA_CONTAMINATION_TEST_STEP_COUNT,
                     step ? step->label : "",
                     hwnd,
                     target_x,
                     target_y);
        }
        return 1;
    }

    if (!cfg->input_drag_last_log_tick ||
        now - cfg->input_drag_last_log_tick >= 1000) {
        cfg->input_drag_last_log_tick = now;
        log_line("camera-contamination-test input-focus-failed phase=%ld/%d label=\"%s\" hwnd=%p auto_focus=%d target_pixels=(%d,%d)",
                 phase + 1,
                 CAMERA_CONTAMINATION_TEST_STEP_COUNT,
                 step ? step->label : "",
                 hwnd,
                 cfg->auto_focus,
                 target_x,
                 target_y);
    }
    return 0;
}

static int camera_contamination_test_send_mouse(DWORD flags, LONG dx, LONG dy)
{
    INPUT input;
    memset(&input, 0, sizeof(input));
    input.type = INPUT_MOUSE;
    input.mi.dx = dx;
    input.mi.dy = dy;
    input.mi.dwFlags = flags;
    return SendInput(1, &input, sizeof(input)) == 1;
}

static void camera_contamination_test_release_mouse(void)
{
    camera_contamination_test_config_t *cfg =
        &camera_contamination_test_cfg;
    if (!cfg->input_drag_button_down) return;
    camera_contamination_test_send_mouse(MOUSEEVENTF_RIGHTUP, 0, 0);
    cfg->input_drag_button_down = 0;
}

static int camera_contamination_test_scaled_input(int value,
                                                  int numerator,
                                                  int denominator)
{
    int sign = value < 0 ? -1 : 1;
    int abs_value = value < 0 ? -value : value;
    if (denominator <= 0) return value;
    return sign * ((abs_value * numerator + denominator / 2) / denominator);
}

static int camera_contamination_test_phase_target_x(
    const camera_contamination_test_step_t *step)
{
    return camera_contamination_test_scaled_input(
        step ? step->input_dx : 0,
        camera_contamination_test_cfg.input_yaw_pixels,
        420);
}

static int camera_contamination_test_phase_target_y(
    const camera_contamination_test_step_t *step)
{
    return camera_contamination_test_scaled_input(
        step ? step->input_dy : 0,
        camera_contamination_test_cfg.input_pitch_pixels,
        260);
}

static int camera_contamination_test_drag_goal(int total,
                                               DWORD elapsed_ms,
                                               DWORD duration_ms)
{
    int sign = total < 0 ? -1 : 1;
    int abs_total = total < 0 ? -total : total;
    if (!abs_total) return 0;
    if (!duration_ms || elapsed_ms >= duration_ms) return total;
    return sign * (int)(((DWORD)abs_total * elapsed_ms +
                         duration_ms / 2) / duration_ms);
}

static void camera_contamination_test_begin_phase_input(DWORD now, LONG phase)
{
    camera_contamination_test_config_t *cfg =
        &camera_contamination_test_cfg;
    const camera_contamination_test_step_t *step = NULL;
    int target_x = 0;
    int target_y = 0;

    cfg->input_drag_start_tick = now;
    cfg->input_drag_sent_x = 0;
    cfg->input_drag_sent_y = 0;
    cfg->input_drag_last_log_tick = 0;
    cfg->input_drag_foreground = camera_contamination_test_game_foreground();
    camera_contamination_test_release_mouse();

    if (phase >= 0 && phase < CAMERA_CONTAMINATION_TEST_STEP_COUNT) {
        step = &camera_contamination_test_steps[phase];
        target_x = camera_contamination_test_phase_target_x(step);
        target_y = camera_contamination_test_phase_target_y(step);
    }

    log_line("camera-contamination-test input phase=%ld/%d label=\"%s\" target_pixels=(%d,%d) drag_ms=%d foreground=%d auto_focus=%d note=\"real right-mouse drag; no AppTracker/D3D matrix spoofing\"",
             phase + 1,
             CAMERA_CONTAMINATION_TEST_STEP_COUNT,
             step ? step->label : "",
             target_x,
             target_y,
             cfg->input_drag_ms,
             cfg->input_drag_foreground,
             cfg->auto_focus);
}

static void camera_contamination_test_drive_input(DWORD now)
{
    camera_contamination_test_config_t *cfg =
        &camera_contamination_test_cfg;
    LONG phase = InterlockedCompareExchange(&cfg->phase, 0, 0);
    const camera_contamination_test_step_t *step;
    DWORD elapsed;
    DWORD duration;
    int target_x;
    int target_y;
    int want_x;
    int want_y;
    int delta_x;
    int delta_y;

    if (!cfg->active || phase < 0 ||
        phase >= CAMERA_CONTAMINATION_TEST_STEP_COUNT) {
        camera_contamination_test_release_mouse();
        return;
    }

    step = &camera_contamination_test_steps[phase];
    target_x = camera_contamination_test_phase_target_x(step);
    target_y = camera_contamination_test_phase_target_y(step);
    if (!target_x && !target_y) {
        camera_contamination_test_release_mouse();
        return;
    }

    cfg->input_drag_foreground =
        camera_contamination_test_ensure_game_foreground(now, phase, step,
                                                         target_x, target_y);
    if (!cfg->input_drag_foreground) {
        camera_contamination_test_release_mouse();
        if (!cfg->input_drag_last_log_tick ||
            now - cfg->input_drag_last_log_tick >= 1000) {
            cfg->input_drag_last_log_tick = now;
            log_line("camera-contamination-test input-skipped phase=%ld/%d label=\"%s\" reason=\"TK17 is not foreground\" target_pixels=(%d,%d) sent_pixels=(%d,%d)",
                     phase + 1,
                     CAMERA_CONTAMINATION_TEST_STEP_COUNT,
                     step->label,
                     target_x,
                     target_y,
                     cfg->input_drag_sent_x,
                     cfg->input_drag_sent_y);
        }
        return;
    }

    elapsed = now - cfg->input_drag_start_tick;
    duration = (DWORD)cfg->input_drag_ms;
    want_x = camera_contamination_test_drag_goal(target_x, elapsed,
                                                 duration);
    want_y = camera_contamination_test_drag_goal(target_y, elapsed,
                                                 duration);
    delta_x = want_x - cfg->input_drag_sent_x;
    delta_y = want_y - cfg->input_drag_sent_y;

    if (!cfg->input_drag_button_down) {
        if (camera_contamination_test_send_mouse(MOUSEEVENTF_RIGHTDOWN,
                                                 0, 0)) {
            cfg->input_drag_button_down = 1;
        }
    }
    if ((delta_x || delta_y) && cfg->input_drag_button_down) {
        if (camera_contamination_test_send_mouse(MOUSEEVENTF_MOVE,
                                                 (LONG)delta_x,
                                                 (LONG)delta_y)) {
            cfg->input_drag_sent_x += delta_x;
            cfg->input_drag_sent_y += delta_y;
        }
    }

    if (elapsed >= duration) {
        if (cfg->input_drag_sent_x != target_x ||
            cfg->input_drag_sent_y != target_y) {
            delta_x = target_x - cfg->input_drag_sent_x;
            delta_y = target_y - cfg->input_drag_sent_y;
            if ((delta_x || delta_y) && cfg->input_drag_button_down &&
                camera_contamination_test_send_mouse(MOUSEEVENTF_MOVE,
                                                     (LONG)delta_x,
                                                     (LONG)delta_y)) {
                cfg->input_drag_sent_x += delta_x;
                cfg->input_drag_sent_y += delta_y;
            }
        }
        camera_contamination_test_release_mouse();
        if (!cfg->input_drag_last_log_tick ||
            now - cfg->input_drag_last_log_tick >= 1000) {
            cfg->input_drag_last_log_tick = now;
            log_line("camera-contamination-test input-complete phase=%ld/%d label=\"%s\" target_pixels=(%d,%d) sent_pixels=(%d,%d)",
                     phase + 1,
                     CAMERA_CONTAMINATION_TEST_STEP_COUNT,
                     step->label,
                     target_x,
                     target_y,
                     cfg->input_drag_sent_x,
                     cfg->input_drag_sent_y);
        }
    }
}

static void clear_body_chain_collision_for_test(body_chain_person_state_t *state)
{
    int i;
    if (!state) return;
    for (i = 0; i < 3; i++) {
        state->collision_contact_direction[i][0] = 0.0f;
        state->collision_contact_direction[i][1] = 0.0f;
    }
    state->collision_prev_max_penetration = 0.0f;
    state->collision_rest_penetration = 0.0f;
    state->collision_rest_ticks = 0;
    state->collision_rest_valid = 0;
    state->collision_rest_grace_ticks = 0;
    state->collision_impact_ticks = 0;
    state->collision_manifold_contacts = 0;
    state->collision_multi_support_grace_ticks = 0;
}

static void run_camera_contamination_test(DWORD now)
{
    camera_contamination_test_config_t *cfg =
        &camera_contamination_test_cfg;
    int person_index;
    body_chain_person_state_t *chain;
    body_chain_collider_person_state_t *collider;
    LONG phase;
    const camera_contamination_test_step_t *step;

    if (!cfg->enabled || cfg->completed) return;
    person_index = collision_auto_test_person_index(cfg->person);
    if (person_index < 0) return;
    chain = &body_chain_person_states[person_index];
    collider = &body_chain_collider_states[person_index];
    if (!chain->initialized || !chain->gravity_probe_promoted ||
        !collider->ready || !captured_camera_inverse_valid) {
        cfg->ready_tick = 0;
        cfg->active = 0;
        cfg->phase = -1;
        camera_contamination_test_release_mouse();
        return;
    }

    if (!cfg->ready_tick) {
        cfg->ready_tick = now;
        log_line("camera-contamination-test ARMED person=\"%s\" start_delay_ms=%d capture_ms=%d instruction=\"load the looping pose now; after the delay do not touch the game until DONE\"",
                 cfg->person,
                 cfg->start_delay_ms,
                 CAMERA_CONTAMINATION_TEST_STEP_COUNT * cfg->hold_ms);
        return;
    }
    if (now - cfg->ready_tick < (DWORD)cfg->start_delay_ms) return;

    phase = cfg->phase;
    if (!cfg->active) {
        phase = 0;
        InterlockedExchange(&cfg->phase, phase);
        InterlockedExchange(&cfg->active, 1);
        cfg->phase_tick = now;
        cfg->last_sample_tick = 0;
        cfg->sample_index = 0;
        camera_contamination_test_camera_valid = 0;
        step = &camera_contamination_test_steps[phase];
        InterlockedExchange(&camera_contamination_test_d3d_view_overrides, 0);
        camera_contamination_test_d3d_view_log_tick = 0;
        camera_contamination_test_begin_phase_input(now, phase);
        log_line("camera-contamination-test START person=\"%s\" steps=%d hold_ms=%d sample_ms=%d isolation=(gravity=1,colliders=1) camera_driver=\"sendinput-right-drag\" note=\"gravity/collider forces are suppressed; camera is driven through TK17 input, not a matrix spoof\"",
                 cfg->person,
                 CAMERA_CONTAMINATION_TEST_STEP_COUNT,
                 cfg->hold_ms,
                 cfg->sample_ms);
        log_line("camera-contamination-test PHASE phase=%ld/%d label=\"%s\" yaw=%.1f pitch=%.1f",
                 phase + 1,
                 CAMERA_CONTAMINATION_TEST_STEP_COUNT,
                 step->label,
                 step->yaw_degrees,
                 step->pitch_degrees);
        return;
    }

    phase = cfg->phase;
    if (phase < 0 || phase >= CAMERA_CONTAMINATION_TEST_STEP_COUNT) {
        InterlockedExchange(&cfg->active, 0);
        cfg->completed = 1;
        return;
    }
    if (now - cfg->phase_tick >= (DWORD)cfg->hold_ms) {
        phase++;
        if (phase >= CAMERA_CONTAMINATION_TEST_STEP_COUNT) {
            InterlockedExchange(&cfg->active, 0);
            InterlockedExchange(&cfg->phase, -1);
            cfg->completed = 1;
            camera_contamination_test_camera_valid = 0;
            camera_contamination_test_release_mouse();
            log_line("============================================================");
            log_line("========== CAMERA CONTAMINATION TEST DONE!!!!! =============");
            log_line("DONE!!!!! person=\"%s\" samples=%u camera_pass_through_restored=1 isolation_restored=1 d3d_view_overrides=%ld input_sent=(%d,%d)",
                     cfg->person, cfg->sample_index,
                     InterlockedCompareExchange(
                         &camera_contamination_test_d3d_view_overrides, 0, 0),
                     cfg->input_drag_sent_x,
                     cfg->input_drag_sent_y);
            log_line("============================================================");
            return;
        }
        InterlockedExchange(&cfg->phase, phase);
        cfg->phase_tick = now;
        cfg->last_sample_tick = 0;
        camera_contamination_test_camera_valid = 0;
        step = &camera_contamination_test_steps[phase];
        camera_contamination_test_begin_phase_input(now, phase);
        log_line("camera-contamination-test PHASE phase=%ld/%d label=\"%s\" yaw=%.1f pitch=%.1f",
                 phase + 1,
                 CAMERA_CONTAMINATION_TEST_STEP_COUNT,
                 step->label,
                 step->yaw_degrees,
                 step->pitch_degrees);
        return;
    }

    camera_contamination_test_drive_input(now);

    if (!camera_contamination_test_camera_valid ||
        (cfg->last_sample_tick &&
         now - cfg->last_sample_tick < (DWORD)cfg->sample_ms)) {
        return;
    }
    cfg->last_sample_tick = now;
    cfg->sample_index++;
    step = &camera_contamination_test_steps[phase];
    {
        char name[256];
        void *trs_raw = chain->camera_relative_trs_raw;
        float root_matrix[9];
        float trs_matrix[9];
        float trs_inverse[9];
        float engine_camera[9];
        float applied_camera[9];
        float applied_inverse[9];
        float root_mul_inv_trs[9];
        float inv_trs_mul_root[9];
        float camera_mul_root[9];
        float root_mul_camera[9];
        float inv_camera_mul_root[9];
        float root_mul_inv_camera[9];
        float root_position[3] = { 0.0f, 0.0f, 0.0f };
        const float *root_pos;

        if (!body_chain_read_mat3_rows(trs_raw, trs_matrix)) {
            make_body_runtime_name(name, sizeof(name), cfg->person,
                                   "TRS_group");
            trs_raw = resolve_axis_map_raw(name);
            if (trs_raw) chain->camera_relative_trs_raw = trs_raw;
        }
        if (!body_chain_read_mat3_rows(chain->root_raw, root_matrix) ||
            !body_chain_read_mat3_rows(trs_raw, trs_matrix) ||
            !body_chain_mat3_inverse(trs_matrix, trs_inverse)) {
            log_line("camera-contamination-test sample-skipped sample=%u phase=%ld reason=\"root/TRS matrix unreadable\" root=%p trs=%p",
                     cfg->sample_index, phase + 1,
                     chain->root_raw, trs_raw);
            return;
        }

        camera_contamination_test_mat3_from_mat4(
            camera_contamination_test_engine_camera, engine_camera);
        camera_contamination_test_mat3_from_mat4(
            camera_contamination_test_applied_camera, applied_camera);
        if (!body_chain_mat3_inverse(applied_camera, applied_inverse)) {
            log_line("camera-contamination-test sample-skipped sample=%u phase=%ld reason=\"applied camera basis singular\"",
                     cfg->sample_index, phase + 1);
            return;
        }
        body_chain_mat3_multiply(root_matrix, trs_inverse,
                                 root_mul_inv_trs);
        body_chain_mat3_multiply(trs_inverse, root_matrix,
                                 inv_trs_mul_root);
        body_chain_mat3_multiply(applied_camera, root_matrix,
                                 camera_mul_root);
        body_chain_mat3_multiply(root_matrix, applied_camera,
                                 root_mul_camera);
        body_chain_mat3_multiply(applied_inverse, root_matrix,
                                 inv_camera_mul_root);
        body_chain_mat3_multiply(root_matrix, applied_inverse,
                                 root_mul_inv_camera);

        if (chain->root_raw &&
            ptr_readable((BYTE*)chain->root_raw +
                         body_chain_physics_cfg.root_offset,
                         sizeof(float) * 3)) {
            root_pos = (const float*)((BYTE*)chain->root_raw +
                                     body_chain_physics_cfg.root_offset);
            if (sane_probe_float(root_pos[0]) &&
                sane_probe_float(root_pos[1]) &&
                sane_probe_float(root_pos[2])) {
                root_position[0] = root_pos[0];
                root_position[1] = root_pos[1];
                root_position[2] = root_pos[2];
            }
        }

        log_line("camera-contamination-test sample sample=%u phase=%ld/%d label=\"%s\" phase_elapsed_ms=%lu yaw=%.1f pitch=%.1f camera_version=%ld d3d_view_overrides=%ld isolation=(gravity=1,colliders=1) input=(target=%d,%d sent=%d,%d foreground=%d) engine_pos=(%.7f,%.7f,%.7f) applied_pos=(%.7f,%.7f,%.7f) root_pos=(%.7f,%.7f,%.7f) source_step=(%.7f,%.7f) chain_angle=(%.5f,%.5f;%.5f,%.5f;%.5f,%.5f) chain_velocity=(%.5f,%.5f;%.5f,%.5f;%.5f,%.5f)",
                 cfg->sample_index,
                 phase + 1,
                 CAMERA_CONTAMINATION_TEST_STEP_COUNT,
                 step->label,
                 (unsigned long)(now - cfg->phase_tick),
                 step->yaw_degrees,
                 step->pitch_degrees,
                 captured_camera_version,
                 InterlockedCompareExchange(
                     &camera_contamination_test_d3d_view_overrides, 0, 0),
                 camera_contamination_test_phase_target_x(step),
                 camera_contamination_test_phase_target_y(step),
                 cfg->input_drag_sent_x,
                 cfg->input_drag_sent_y,
                 cfg->input_drag_foreground,
                 camera_contamination_test_engine_camera[12],
                 camera_contamination_test_engine_camera[13],
                 camera_contamination_test_engine_camera[14],
                 camera_contamination_test_applied_camera[12],
                 camera_contamination_test_applied_camera[13],
                 camera_contamination_test_applied_camera[14],
                 root_position[0], root_position[1], root_position[2],
                 chain->root_drive_last_h_step,
                 chain->root_drive_last_v_step,
                 chain->angle[0][0], chain->angle[0][1],
                 chain->angle[1][0], chain->angle[1][1],
                 chain->angle[2][0], chain->angle[2][1],
                 chain->velocity[0][0], chain->velocity[0][1],
                 chain->velocity[1][0], chain->velocity[1][1],
                 chain->velocity[2][0], chain->velocity[2][1]);
        camera_contamination_test_log_mat3(
            "engine_camera", engine_camera,
            cfg->sample_index, phase);
        camera_contamination_test_log_mat3(
            "applied_camera", applied_camera,
            cfg->sample_index, phase);
        camera_contamination_test_log_mat3(
            "root_raw", root_matrix,
            cfg->sample_index, phase);
        camera_contamination_test_log_mat3(
            "trs_raw", trs_matrix,
            cfg->sample_index, phase);
        camera_contamination_test_log_mat3(
            "root_mul_inv_trs", root_mul_inv_trs,
            cfg->sample_index, phase);
        camera_contamination_test_log_mat3(
            "inv_trs_mul_root", inv_trs_mul_root,
            cfg->sample_index, phase);
        camera_contamination_test_log_mat3(
            "camera_mul_root", camera_mul_root,
            cfg->sample_index, phase);
        camera_contamination_test_log_mat3(
            "root_mul_camera", root_mul_camera,
            cfg->sample_index, phase);
        camera_contamination_test_log_mat3(
            "inv_camera_mul_root", inv_camera_mul_root,
            cfg->sample_index, phase);
        camera_contamination_test_log_mat3(
            "root_mul_inv_camera", root_mul_inv_camera,
            cfg->sample_index, phase);
    }
}

