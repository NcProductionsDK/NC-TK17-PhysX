static int write_far_jump(BYTE *address, void *target)
{
    BYTE bytes[7] = { 0xea, 0, 0, 0, 0, 0, 0 };
    DWORD old;
    unsigned short cs;
    if (!address || !target || !ptr_readable(address, sizeof(bytes))) return 0;
    *(DWORD*)(bytes + 1) = (DWORD)target;
    __asm__ volatile ("movw %%cs, %0" : "=r"(cs));
    *(unsigned short*)(bytes + 5) = cs;
    if (!VirtualProtect(address, sizeof(bytes), PAGE_EXECUTE_READWRITE, &old)) return 0;
    memcpy(address, bytes, sizeof(bytes));
    VirtualProtect(address, sizeof(bytes), old, &old);
    FlushInstructionCache(GetCurrentProcess(), address, sizeof(bytes));
    return 1;
}

static void restore_poseedit_inittracks_hook(void)
{
    if (!poseedit_inittracks_hook_installed || !real_PoseEdit_InitTracks) return;
    write_far_jump(POSEEDIT_INITTRACKS_PATCH_ADDR, (void*)real_PoseEdit_InitTracks);
    poseedit_inittracks_hook_installed = 0;
}

static void patch_poseedit_inittracks_hook(void)
{
    BYTE *addr = POSEEDIT_INITTRACKS_PATCH_ADDR;
    void *target;
    if (poseedit_inittracks_hook_installed) return;
    if (!body_chain_physics_cfg.poseeditor_track_override &&
        !body_chain_physics_cfg.poseeditor_track_diagnostic &&
        !testicle_physics_cfg.poseeditor_track_override &&
        !testicle_physics_cfg.poseeditor_track_diagnostic &&
        !breasts_physics_global_cfg.poseeditor_track_override &&
        !butt_physics_global_cfg.poseeditor_track_override) {
        return;
    }
    if (!ptr_readable(addr, 7)) return;
    if (addr[0] != 0xea) {
        if (!poseedit_inittracks_hook_logged) {
            poseedit_inittracks_hook_logged = 1;
            log_line("PoseEdit InitTracks hook waiting addr=%p bytes=%02x %02x %02x %02x %02x %02x %02x reason=\"expected PE mod far jump at 0x004CE710\"",
                     addr, addr[0], addr[1], addr[2], addr[3], addr[4], addr[5], addr[6]);
        }
        return;
    }
    target = *(void**)(addr + 1);
    if (target == (void*)hook_PoseEdit_InitTracks) {
        poseedit_inittracks_hook_installed = 1;
        return;
    }
    if (!target || !ptr_readable(target, 8)) {
        if (!poseedit_inittracks_hook_logged) {
            poseedit_inittracks_hook_logged = 1;
            log_line("PoseEdit InitTracks hook waiting addr=%p target=%p reason=\"PE mod far-jump target is not readable yet\"",
                     addr, target);
        }
        return;
    }
    real_PoseEdit_InitTracks = (poseedit_init_tracks_t)target;
    if (write_far_jump(addr, (void*)hook_PoseEdit_InitTracks)) {
        poseedit_inittracks_hook_installed = 1;
        log_line("PoseEdit InitTracks hook installed addr=%p original=%p hook=%p note=\"captures EditPose table for per-person track override; tracks.ini untouched\"",
                 addr, target, (void*)hook_PoseEdit_InitTracks);
    }
}

static void __stdcall hook_PoseEdit_InitTracks(void)
{
    void *pe = NULL;
    void *editpose = NULL;
    __asm__ volatile ("movl %%ecx, %0" : "=m"(pe));
    if (real_PoseEdit_InitTracks) {
        __asm__ volatile ("movl %0, %%ecx" : : "r"(pe) : "ecx");
        real_PoseEdit_InitTracks();
    }
    log_poseedit_fixed_offsets(pe);
    if (pe && ptr_readable((BYTE*)pe + POSEEDIT_EDITPOSE_OFFSET, sizeof(void*))) {
        editpose = *(void**)((BYTE*)pe + POSEEDIT_EDITPOSE_OFFSET);
    }
    if (editpose && ptr_readable((BYTE*)editpose + POSEEDIT_TRACKS_OFFSET,
                                  POSEEDIT_TRACK_SIZE * 2)) {
        if (captured_poseedit_editpose != editpose) {
            captured_poseedit_this = pe;
            captured_poseedit_editpose = editpose;
            captured_poseedit_tick = GetTickCount();
            log_line("PoseEdit InitTracks captured pe=%p editpose=%p tracks=%p total_tracks=%d track_size=0x%x person02_track144=%p note=\"next body-chain tick can target exact per-person PoseTrack slot\"",
                     pe,
                     editpose,
                     (BYTE*)editpose + POSEEDIT_TRACKS_OFFSET,
                     body_chain_physics_cfg.poseeditor_total_tracks,
                     POSEEDIT_TRACK_SIZE,
                     poseedit_track_slot(1, POSEEDIT_TRACK_PENIS_JOINT01));
            reset_body_chain_physics();
        }
    } else {
        log_line("PoseEdit InitTracks captured pe=%p editpose=%p reason=\"EditPose pointer not readable after original init\"",
                 pe, editpose);
    }
}

static int install_inline_hook(void *target, void *hook, size_t stolen_len, void **trampoline)
{
    BYTE *tramp;
    DWORD old;
    DWORD rel;
    size_t i;
    if (!target || !hook || !trampoline || *trampoline || stolen_len < 5) return 0;

    if (*(BYTE*)target == 0xe9) {
        DWORD old_rel;
        memcpy(&old_rel, (BYTE*)target + 1, sizeof(old_rel));
        *trampoline = (void*)((BYTE*)target + 5 + (LONG)old_rel);
        if (!VirtualProtect(target, 5, PAGE_EXECUTE_READWRITE, &old)) {
            *trampoline = NULL;
            return 0;
        }
        rel = (DWORD)((BYTE*)hook - ((BYTE*)target + 5));
        memcpy((BYTE*)target + 1, &rel, sizeof(rel));
        VirtualProtect(target, 5, old, &old);
        FlushInstructionCache(GetCurrentProcess(), target, 5);
        return 1;
    }

    tramp = (BYTE*)VirtualAlloc(NULL, stolen_len + 5, MEM_COMMIT | MEM_RESERVE, PAGE_EXECUTE_READWRITE);
    if (!tramp) return 0;
    memcpy(tramp, target, stolen_len);
    tramp[stolen_len] = 0xe9;
    rel = (DWORD)(((BYTE*)target + stolen_len) - (tramp + stolen_len + 5));
    memcpy(tramp + stolen_len + 1, &rel, sizeof(rel));

    if (!VirtualProtect(target, stolen_len, PAGE_EXECUTE_READWRITE, &old)) {
        VirtualFree(tramp, 0, MEM_RELEASE);
        return 0;
    }
    ((BYTE*)target)[0] = 0xe9;
    rel = (DWORD)((BYTE*)hook - ((BYTE*)target + 5));
    memcpy((BYTE*)target + 1, &rel, sizeof(rel));
    for (i = 5; i < stolen_len; i++) ((BYTE*)target)[i] = 0x90;
    VirtualProtect(target, stolen_len, old, &old);
    FlushInstructionCache(GetCurrentProcess(), target, stolen_len);
    *trampoline = tramp;
    return 1;
}

static void **runtime_animation_member_slot(DWORD member_id,
                                            unsigned int slot_offset)
{
    void **master;
    void *member_table;
    unsigned int class_index = member_id & 0x0fffu;
    unsigned int member_index = member_id >> 24;
    BYTE *slot;
    if (!engine_G_MasterIsMVTBL_ptr || member_index == 0 ||
        slot_offset >= 0x40u || (slot_offset & 3u) != 0 ||
        !ptr_readable(engine_G_MasterIsMVTBL_ptr, sizeof(void*))) {
        return NULL;
    }
    master = *engine_G_MasterIsMVTBL_ptr;
    if (!master ||
        !ptr_readable(master + class_index, sizeof(void*))) {
        return NULL;
    }
    member_table = master[class_index];
    if (!member_table) {
        return NULL;
    }
    slot = (BYTE*)member_table + member_index * 0x40u + slot_offset;
    if (!ptr_readable(slot, sizeof(void*))) {
        return NULL;
    }
    return (void**)slot;
}

static void **runtime_animation_member_setter_slot(DWORD member_id)
{
    return runtime_animation_member_slot(member_id, 0x04u);
}

static int patch_runtime_animation_member_setter_slot(
    void **slot, void *hook, script_vector3_set_property_t *original)
{
    void *current;
    DWORD old;
    if (!slot || !hook || !original ||
        !ptr_readable(slot, sizeof(void*))) {
        return 0;
    }
    current = *slot;
    if (current == hook) return *original != NULL;
    if (!ptr_executable(current)) return 0;
    if (!*original) {
        *original = (script_vector3_set_property_t)current;
    } else if ((void*)*original != current) {
        return 0;
    }
    if (!VirtualProtect(slot, sizeof(void*), PAGE_READWRITE, &old)) {
        return 0;
    }
    *slot = hook;
    VirtualProtect(slot, sizeof(void*), old, &old);
    FlushInstructionCache(GetCurrentProcess(), slot, sizeof(void*));
    return 1;
}

static void patch_runtime_animation_member_setters(void)
{
    int ssimple_ok;
    int sjoint_ok;
    if (runtime_animation_member_setters_installed) return;
    runtime_ssimple_rotation_set_slot =
        runtime_animation_member_setter_slot(
            SCRIPT_PROPERTY_SSIMPLE_ROTATION);
    runtime_sjoint_rotation_axis_set_slot =
        runtime_animation_member_setter_slot(
            SCRIPT_PROPERTY_SJOINT_ROTATION_AXIS);
    ssimple_ok = patch_runtime_animation_member_setter_slot(
        runtime_ssimple_rotation_set_slot,
        (void*)hook_SSimpleTransform_RotationSet,
        &real_SSimpleTransform_RotationSet);
    sjoint_ok = patch_runtime_animation_member_setter_slot(
        runtime_sjoint_rotation_axis_set_slot,
        (void*)hook_SJoint_RotationAxisSet,
        &real_SJoint_RotationAxisSet);
    runtime_animation_member_setters_installed =
        ssimple_ok && sjoint_ok;
    if (runtime_animation_member_setters_installed &&
        !runtime_animation_member_setters_logged) {
        runtime_animation_member_setters_logged = 1;
        log_line("runtime FreeMode penis animation-setter filters installed ssimple_slot=%p ssimple_original=%p sjoint_slot=%p sjoint_original=%p note=\"only behavior-critical runtime ownership filters are installed\"",
                 runtime_ssimple_rotation_set_slot,
                 (void*)real_SSimpleTransform_RotationSet,
                 runtime_sjoint_rotation_axis_set_slot,
                 (void*)real_SJoint_RotationAxisSet);
    }
}

static void restore_runtime_animation_member_setter_slot(
    void **slot, void *hook, script_vector3_set_property_t original)
{
    DWORD old;
    if (!slot || !original || !ptr_readable(slot, sizeof(void*)) ||
        *slot != hook) {
        return;
    }
    if (!VirtualProtect(slot, sizeof(void*), PAGE_READWRITE, &old)) {
        return;
    }
    *slot = (void*)original;
    VirtualProtect(slot, sizeof(void*), old, &old);
    FlushInstructionCache(GetCurrentProcess(), slot, sizeof(void*));
}

static void restore_runtime_animation_member_setters(void)
{
    restore_runtime_animation_member_setter_slot(
        runtime_ssimple_rotation_set_slot,
        (void*)hook_SSimpleTransform_RotationSet,
        real_SSimpleTransform_RotationSet);
    restore_runtime_animation_member_setter_slot(
        runtime_sjoint_rotation_axis_set_slot,
        (void*)hook_SJoint_RotationAxisSet,
        real_SJoint_RotationAxisSet);
    runtime_animation_member_setters_installed = 0;
}

static void patch_addon_constraint_count_getter(void)
{
    void *current;
    DWORD old;
    if (addon_constraint_count_getter_installed) return;
    addon_constraint_count_getter_slot = runtime_animation_member_slot(
        SCRIPT_PROPERTY_TBASE_CONSTRAINT_ARRAY, 0x08u);
    if (!addon_constraint_count_getter_slot ||
        !ptr_readable(addon_constraint_count_getter_slot, sizeof(void*))) {
        return;
    }
    current = *addon_constraint_count_getter_slot;
    if (current == (void*)hook_TBaseTransform_ConstraintArrayCount) {
        addon_constraint_count_getter_installed =
            real_AddonConstraintArrayCount != NULL;
        return;
    }
    if (!ptr_executable(current)) return;
    if (!real_AddonConstraintArrayCount) {
        real_AddonConstraintArrayCount =
            (script_index_count_property_t)current;
    } else if ((void*)real_AddonConstraintArrayCount != current) {
        return;
    }
    if (!VirtualProtect(addon_constraint_count_getter_slot,
                        sizeof(void*), PAGE_READWRITE, &old)) {
        return;
    }
    *addon_constraint_count_getter_slot =
        (void*)hook_TBaseTransform_ConstraintArrayCount;
    VirtualProtect(addon_constraint_count_getter_slot,
                   sizeof(void*), old, &old);
    FlushInstructionCache(GetCurrentProcess(),
                          addon_constraint_count_getter_slot,
                          sizeof(void*));
    addon_constraint_count_getter_installed = 1;
    if (!addon_constraint_count_getter_logged) {
        addon_constraint_count_getter_logged = 1;
        log_line("addon ConstraintArray count filter installed slot=%p original=%p property=0x%08lx note=\"active sidecar targets report zero native constraints; original scene arrays remain untouched\"",
                 addon_constraint_count_getter_slot,
                 (void*)real_AddonConstraintArrayCount,
                 (unsigned long)SCRIPT_PROPERTY_TBASE_CONSTRAINT_ARRAY);
    }
}

static void restore_addon_constraint_count_getter(void)
{
    DWORD old;
    if (!addon_constraint_count_getter_slot ||
        !real_AddonConstraintArrayCount ||
        !ptr_readable(addon_constraint_count_getter_slot, sizeof(void*)) ||
        *addon_constraint_count_getter_slot !=
            (void*)hook_TBaseTransform_ConstraintArrayCount) {
        addon_constraint_count_getter_installed = 0;
        return;
    }
    if (!VirtualProtect(addon_constraint_count_getter_slot,
                        sizeof(void*), PAGE_READWRITE, &old)) {
        return;
    }
    *addon_constraint_count_getter_slot =
        (void*)real_AddonConstraintArrayCount;
    VirtualProtect(addon_constraint_count_getter_slot,
                   sizeof(void*), old, &old);
    FlushInstructionCache(GetCurrentProcess(),
                          addon_constraint_count_getter_slot,
                          sizeof(void*));
    addon_constraint_count_getter_installed = 0;
}

static int THISCALL hook_TBaseTransform_ConstraintArrayCount(
    void *self, DWORD member_id)
{
    int count = 0;
    physx_sidecar_t *sc = NULL;
    physx_chain_t *chain = NULL;
    physx_target_t *target = NULL;
    if (real_AddonConstraintArrayCount) {
        count = real_AddonConstraintArrayCount(self, member_id);
    }
    if (member_id != SCRIPT_PROPERTY_TBASE_CONSTRAINT_ARRAY || count <= 0 ||
        !addon_should_suppress_constraint_array(self, &sc, &chain,
                                                &target)) {
        return count;
    }
    if (target && !target->constraint_suppression_logged) {
        target->constraint_suppression_logged = 1;
        log_line("addon native constraints suppressed owner=\"%s\" chain=\"%s\" target=\"%s\" constraints=%d object=%p sidecar=\"%s\" note=\"runtime equivalent of commenting out this target's TBaseTransform.ConstraintArray; the original array is automatically visible again when PhysX releases ownership\"",
                 chain && chain->addon_owner_person[0] ?
                     chain->addon_owner_person : "unknown",
                 chain ? chain->name : "",
                 target->name,
                 count,
                 self,
                 sc ? sc->path : "");
    }
    return 0;
}

static void THISCALL hook_SSimpleTransform_RotationSet(
    void *self, DWORD member_id, const float *value)
{
    static const float neutral[3] = { 0.0f, 0.0f, 0.0f };
    const float *applied = value;
    physx_sidecar_t *sc = NULL;
    physx_chain_t *chain = NULL;
    physx_target_t *target = NULL;
    if (body_chain_runtime_should_neutralize_penis_animation_write(
            self, member_id, value)) {
        applied = neutral;
    }
    if (addon_should_suppress_animation_write(
            self, member_id, &sc, &chain, &target)) {
        /* Room animation may execute after the PhysX pass. Preserve the
           latest native room-bone solver rotation in that ordering instead
           of letting the animation suppression write zero over the leaves.
           Non-room add-ons retain the established neutralization behavior. */
        if (sc && sc->room_scene_sidecar && target &&
            InterlockedCompareExchange(
                (volatile LONG*)&target->object_output_applied, 0, 0) &&
            physx_vec3_sane_limit(target->object_output_rotation, 720.0f)) {
            applied = target->object_output_rotation;
        } else {
            applied = neutral;
        }
        addon_note_animation_write_suppressed(
            sc, chain, target, member_id, self);
    }
    if (real_SSimpleTransform_RotationSet) {
        real_SSimpleTransform_RotationSet(self, member_id, applied);
    }
}

static void THISCALL hook_SJoint_RotationAxisSet(
    void *self, DWORD member_id, const float *value)
{
    static const float neutral[3] = { 0.0f, 0.0f, 0.0f };
    const float *applied = value;
    physx_sidecar_t *sc = NULL;
    physx_chain_t *chain = NULL;
    physx_target_t *target = NULL;
    if (body_chain_runtime_should_neutralize_penis_source_axis_write(
            self, member_id, value) ||
        body_chain_runtime_should_neutralize_penis_animation_write(
            self, member_id, value)) {
        applied = neutral;
    }
    if (addon_should_suppress_animation_write(
            self, member_id, &sc, &chain, &target)) {
        applied = neutral;
        addon_note_animation_write_suppressed(
            sc, chain, target, member_id, self);
    }
    if (tramp_RuntimeJointRotationAxisWrite) {
        tramp_RuntimeJointRotationAxisWrite(self, member_id, applied);
    } else if (real_SJoint_RotationAxisSet &&
               real_SJoint_RotationAxisSet !=
                   hook_SJoint_RotationAxisSet) {
        real_SJoint_RotationAxisSet(self, member_id, applied);
    }
}

/* The animation scheduler calls the concrete SJoint setter directly instead
   of always going through the class-member slot. Route that native entry
   through the same runtime-only filter used by script-property writes. */
static void THISCALL hook_RuntimeJointRotationAxisWrite(
    void *self, DWORD member_id, const float *value)
{
    hook_SJoint_RotationAxisSet(self, member_id, value);
}

static void patch_config_editor_param_change(void)
{
    static const BYTE expected[CONFIG_EDITOR_PARAM_CHANGE_STOLEN_LEN] = {
        0x55, 0x8b, 0xec, 0x83, 0xe4, 0xc0
    };
    HMODULE exe;
    BYTE *target;
    if (config_editor_param_change_hook_installed) return;
    exe = GetModuleHandleA(NULL);
    if (!exe) return;
    target = (BYTE*)exe + CONFIG_EDITOR_PARAM_CHANGE_OFFSET;
    if (!ptr_executable(target)) return;
    if (target[0] != 0xe9 && memcmp(target, expected, sizeof(expected)) != 0) {
        if (!config_editor_param_change_hook_logged) {
            config_editor_param_change_hook_logged = 1;
            log_line("settings ConfigEditor hook unavailable target=%p bytes=%02x %02x %02x %02x %02x %02x reason=\"unsupported TK17 executable build\"",
                     target, target[0], target[1], target[2],
                     target[3], target[4], target[5]);
        }
        return;
    }
    if (install_inline_hook(target, (void*)hook_ConfigEditor_ParamChange,
                            CONFIG_EDITOR_PARAM_CHANGE_STOLEN_LEN,
                            (void**)&real_ConfigEditor_ParamChange)) {
        config_editor_param_change_hook_installed = 1;
        log_line("settings ConfigEditor hook installed target=%p trampoline=%p params=15 note=\"only the remaining NCPhysX-prefixed global controls write NC-TK17-PhysX.ini; per-person toggles are handled directly by the context menu\"",
                 target, (void*)real_ConfigEditor_ParamChange);
    }
}

static void THISCALL hook_ConfigEditor_ParamChange(void *self,
                                                   const char *param_name,
                                                   const char *string_value,
                                                   DWORD value_arg,
                                                   DWORD event_arg)
{
    char param_copy[96];
    char value_copy[32];
    const char *param_cstr = stringref_cstr_a(param_name);
    const char *value_cstr = stringref_cstr_a(string_value);
    int is_physx = param_cstr &&
                   _strnicmp(param_cstr, "NCPhysX", 7) == 0;
    param_copy[0] = 0;
    value_copy[0] = 0;
    if (is_physx) {
        lstrcpynA(param_copy, param_cstr, sizeof(param_copy));
        if (value_cstr) lstrcpynA(value_copy, value_cstr, sizeof(value_copy));
    }
    if (real_ConfigEditor_ParamChange) {
        real_ConfigEditor_ParamChange(self, param_name, string_value,
                                      value_arg, event_arg);
    }
    if (is_physx) {
        handle_physx_settings_change(param_copy,
                                     value_copy[0] ? value_copy : NULL);
    }
}

static int person_context_widget_visibility_get(void *widget,
                                                unsigned int *value_out)
{
    BYTE *script_object = (BYTE*)widget;
    BYTE *metadata;
    BYTE *dispatch_table;
    script_u32_property_t getter;
    if (value_out) *value_out = 0;
    if (!script_object || is_nil_engine_object(NULL, script_object) ||
        !ptr_readable(script_object - SCRIPT_OBJECT_META_BACK_OFFSET,
                      sizeof(void*))) {
        return 0;
    }
    metadata = *(BYTE**)(script_object - SCRIPT_OBJECT_META_BACK_OFFSET);
    if (!metadata ||
        !ptr_readable(metadata + SCRIPT_OBJECT_U32_DISPATCH_TABLE_OFFSET,
                      sizeof(void*))) {
        return 0;
    }
    dispatch_table = *(BYTE**)(metadata +
                               SCRIPT_OBJECT_U32_DISPATCH_TABLE_OFFSET);
    if (!dispatch_table ||
        !ptr_readable(dispatch_table + SCRIPT_OBJECT_U32_GET_DISPATCH_OFFSET,
                      sizeof(void*))) {
        return 0;
    }
    getter = *(script_u32_property_t*)(dispatch_table +
                                       SCRIPT_OBJECT_U32_GET_DISPATCH_OFFSET);
    if (!ptr_executable((const void*)getter)) return 0;
    if (value_out) {
        *value_out = getter(script_object,
                            SCRIPT_PROPERTY_WIDGET_VISIBILITY);
    }
    return 1;
}

static int person_context_widget_visibility_set(void *widget,
                                                unsigned int value)
{
    BYTE *script_object = (BYTE*)widget;
    BYTE *metadata;
    BYTE *dispatch_table;
    script_u32_set_property_t setter;
    if (!script_object || is_nil_engine_object(NULL, script_object) ||
        !ptr_readable(script_object - SCRIPT_OBJECT_META_BACK_OFFSET,
                      sizeof(void*))) {
        return 0;
    }
    metadata = *(BYTE**)(script_object - SCRIPT_OBJECT_META_BACK_OFFSET);
    if (!metadata ||
        !ptr_readable(metadata + SCRIPT_OBJECT_U32_DISPATCH_TABLE_OFFSET,
                      sizeof(void*))) {
        return 0;
    }
    dispatch_table = *(BYTE**)(metadata +
                               SCRIPT_OBJECT_U32_DISPATCH_TABLE_OFFSET);
    if (!dispatch_table ||
        !ptr_readable(dispatch_table + SCRIPT_OBJECT_U32_SET_DISPATCH_OFFSET,
                      sizeof(void*))) {
        return 0;
    }
    setter = *(script_u32_set_property_t*)(dispatch_table +
                                           SCRIPT_OBJECT_U32_SET_DISPATCH_OFFSET);
    if (!ptr_executable((const void*)setter)) return 0;
    setter(script_object, SCRIPT_PROPERTY_WIDGET_VISIBILITY, value ? 1u : 0u);
    return 1;
}

static void *person_context_find_widget(const char *name)
{
    void *raw = NULL;
    void *resolved = NULL;

    resolved = resolve_find_obj(name, &raw);
    if (raw && !is_nil_engine_object(raw, raw) &&
        ptr_readable((BYTE*)raw - SCRIPT_OBJECT_META_BACK_OFFSET,
                     sizeof(void*))) {
        /*
         * GUI FindObjC results are already the ScriptObject used by TK17's
         * property dispatcher.  Unlike scene-node references, unwrapping the
         * result can yield a different object that has no Widget properties.
         */
        return raw;
    }
    if (resolved && !is_nil_engine_object(raw, resolved) &&
        ptr_readable((BYTE*)resolved - SCRIPT_OBJECT_META_BACK_OFFSET,
                     sizeof(void*))) {
        return resolved;
    }

    raw = NULL;
    resolved = resolve_script_engine_obj(name, &raw);
    if (raw && !is_nil_engine_object(raw, raw) &&
        ptr_readable((BYTE*)raw - SCRIPT_OBJECT_META_BACK_OFFSET,
                     sizeof(void*))) {
        return raw;
    }
    if (resolved && !is_nil_engine_object(raw, resolved) &&
        ptr_readable((BYTE*)resolved - SCRIPT_OBJECT_META_BACK_OFFSET,
                     sizeof(void*))) {
        return resolved;
    }
    return NULL;
}

static void body_chain_set_poseeditor_mode(int poseeditor,
                                           const char *source,
                                           const char *mode_name)
{
    LONG previous;
    if (poseeditor) {
        body_chain_poseeditor_exit_candidate_tick = 0;
    }
    previous = InterlockedExchange(
        &body_chain_poseeditor_mode_active, poseeditor ? 1 : 0);
    if (previous == (poseeditor ? 1 : 0)) return;

    InterlockedExchange(&body_chain_runtime_mode_transition_pending,
                        poseeditor ? 1 : 2);
    log_line("body-chain-physics mode boundary previous=%s current=%s detected_mode=\"%s\" source=\"%s\" note=\"only the separate runtime state will be reset; PoseEditor state is untouched\"",
             previous ? "PoseEditor" : "runtime",
             poseeditor ? "PoseEditor" : "runtime",
             mode_name ? mode_name :
                 (poseeditor ? "PoseEdit" : "non-PoseEditor"),
             source ? source : "unknown");
}

static int body_chain_read_poseeditor_visibility(int *poseeditor_out)
{
    static void *poseeditor_root;
    unsigned int visibility = 0;

    if (poseeditor_out) *poseeditor_out = 0;
    if (!poseeditor_root) {
        poseeditor_root = person_context_find_widget(
            "GUI:PoseEdit_RootGroup");
    }
    if (!poseeditor_root ||
        !person_context_widget_visibility_get(poseeditor_root,
                                              &visibility)) {
        poseeditor_root = NULL;
        return 0;
    }
    if (poseeditor_out) *poseeditor_out = visibility != 0;
    return 1;
}

/* PersonContext_Mode does not consistently include its Mode field through
   every TK17 route. PoseEdit_RootGroup provides an engine-owned fallback.
   Leaving PoseEditor is debounced because TK17 temporarily hides this widget
   during some PoseEditor operations. */
static void body_chain_poll_poseeditor_mode(DWORD now)
{
    int poseeditor = 0;
    int current_poseeditor;

    if (body_chain_poseeditor_mode_probe_tick &&
        now - body_chain_poseeditor_mode_probe_tick < 100u) {
        return;
    }
    body_chain_poseeditor_mode_probe_tick = now;

    if (!body_chain_read_poseeditor_visibility(&poseeditor)) {
        if (defaults_cfg.debug &&
            (!body_chain_poseeditor_mode_probe_unavailable_log_tick ||
             now - body_chain_poseeditor_mode_probe_unavailable_log_tick >=
                 5000u)) {
            body_chain_poseeditor_mode_probe_unavailable_log_tick = now;
            log_line("body-chain-physics mode probe waiting source=\"GUI:PoseEdit_RootGroup.Visibility\" reason=\"PoseEditor root widget is not available yet\"");
        }
        return;
    }

    body_chain_poseeditor_mode_probe_unavailable_log_tick = 0;
    if (!body_chain_poseeditor_mode_probe_ready) {
        body_chain_poseeditor_mode_probe_ready = 1;
        log_line("body-chain-physics mode detector ready current=%s visibility=%d source=\"GUI:PoseEdit_RootGroup.Visibility\" exit_debounce_ms=%u note=\"temporary PoseEditor UI hiding does not immediately select runtime ownership\"",
                 poseeditor ? "PoseEditor" : "runtime", poseeditor,
                 POSEEDITOR_VISIBILITY_EXIT_DEBOUNCE_MS);
    }
    current_poseeditor =
        InterlockedCompareExchange(&body_chain_poseeditor_mode_active,
                                   0, 0) != 0;
    if (poseeditor) {
        if (body_chain_poseeditor_exit_candidate_tick &&
            defaults_cfg.debug) {
            log_line("body-chain-physics mode exit suppressed current=PoseEditor hidden_ms=%u source=\"GUI:PoseEdit_RootGroup.Visibility\" reason=\"PoseEditor UI became visible again before the stable-exit window\"",
                     now - body_chain_poseeditor_exit_candidate_tick);
        }
        body_chain_poseeditor_exit_candidate_tick = 0;
        body_chain_set_poseeditor_mode(
            1, "GUI:PoseEdit_RootGroup.Visibility", "PoseEdit");
    } else if (current_poseeditor) {
        if (!body_chain_poseeditor_exit_candidate_tick) {
            body_chain_poseeditor_exit_candidate_tick = now;
            if (defaults_cfg.debug) {
                log_line("body-chain-physics mode exit candidate current=PoseEditor source=\"GUI:PoseEdit_RootGroup.Visibility\" debounce_ms=%u note=\"waiting to distinguish a real mode exit from a temporary pose operation\"",
                         POSEEDITOR_VISIBILITY_EXIT_DEBOUNCE_MS);
            }
        } else if (now - body_chain_poseeditor_exit_candidate_tick >=
                       POSEEDITOR_VISIBILITY_EXIT_DEBOUNCE_MS) {
            body_chain_poseeditor_exit_candidate_tick = 0;
            body_chain_set_poseeditor_mode(
                0, "stable GUI:PoseEdit_RootGroup.Visibility",
                "non-PoseEditor");
        }
    } else {
        body_chain_poseeditor_exit_candidate_tick = 0;
    }
}

static void person_context_sync_physx_menu(void *context)
{
    static int lookup_failure_logged;
    void *customize_widget;
    void *separator_widget;
    void *menu_widget;
    void *breasts_widget;
    void *penis_widget;
    void *testicle_widget;
    void *butt_widget;
    unsigned int customize_visibility = 0;
    unsigned int separator_visibility = 0;
    unsigned int menu_visibility = 0;
    unsigned int breasts_visibility = 0;
    unsigned int penis_visibility = 0;
    unsigned int testicle_visibility = 0;
    unsigned int butt_visibility = 0;
    unsigned int context_visibility;
    unsigned int desired_separator_visibility;
    unsigned int desired_menu_visibility;
    unsigned int desired_breasts_visibility;
    unsigned int desired_penis_visibility;
    unsigned int desired_testicle_visibility;
    unsigned int desired_butt_visibility;
    int effective_breasts_enabled = 0;
    int effective_penis_enabled = 0;
    int effective_testicle_enabled = 0;
    int effective_butt_enabled = 0;
    int person_index = -1;
    int person = 0;
    int old_person;

    customize_widget = person_context_find_widget(
        "GUI:PersonContext_Customize_Popup");
    separator_widget = person_context_find_widget(
        "GUI:PersonContext_PhysX_Separator");
    menu_widget = person_context_find_widget(
        "GUI:PersonContext_PhysX_Menu_Item");
    breasts_widget = person_context_find_widget(
        "GUI:PersonContext_PhysX_Breasts_Toggle");
    penis_widget = person_context_find_widget(
        "GUI:PersonContext_PhysX_Penis_Toggle");
    testicle_widget = person_context_find_widget(
        "GUI:PersonContext_PhysX_Testicle_Toggle");
    butt_widget = person_context_find_widget(
        "GUI:PersonContext_PhysX_Butt_Toggle");

    if (!person_context_widget_visibility_get(customize_widget,
                                              &customize_visibility)) {
        InterlockedExchange(&person_context_selected_person, 0);
        if (separator_widget) person_context_widget_visibility_set(separator_widget, 0);
        if (menu_widget) person_context_widget_visibility_set(menu_widget, 0);
        if (breasts_widget) person_context_widget_visibility_set(breasts_widget, 0);
        if (penis_widget) person_context_widget_visibility_set(penis_widget, 0);
        if (testicle_widget) person_context_widget_visibility_set(testicle_widget, 0);
        if (butt_widget) person_context_widget_visibility_set(butt_widget, 0);
        if (defaults_cfg.debug && !lookup_failure_logged) {
            lookup_failure_logged = 1;
            log_line("person-context sync unavailable context=%p customize=%p separator=%p menu=%p breasts=%p penis=%p testicle=%p butt=%p reason=\"could not read native Customize visibility\"",
                     context, customize_widget, separator_widget, menu_widget,
                     breasts_widget, penis_widget, testicle_widget, butt_widget);
        }
        return;
    }

    if (!separator_widget || !menu_widget || !breasts_widget ||
        !penis_widget || !testicle_widget || !butt_widget) {
        InterlockedExchange(&person_context_selected_person, 0);
        if (separator_widget) person_context_widget_visibility_set(separator_widget, 0);
        if (menu_widget) person_context_widget_visibility_set(menu_widget, 0);
        if (breasts_widget) person_context_widget_visibility_set(breasts_widget, 0);
        if (penis_widget) person_context_widget_visibility_set(penis_widget, 0);
        if (testicle_widget) person_context_widget_visibility_set(testicle_widget, 0);
        if (butt_widget) person_context_widget_visibility_set(butt_widget, 0);
        if (defaults_cfg.debug && !lookup_failure_logged) {
            lookup_failure_logged = 1;
            log_line("person-context sync unavailable context=%p customize=%p separator=%p menu=%p breasts=%p penis=%p testicle=%p butt=%p reason=\"one or more custom PhysX context widgets were not found\"",
                     context, customize_widget, separator_widget, menu_widget,
                     breasts_widget, penis_widget, testicle_widget, butt_widget);
        }
        return;
    }
    lookup_failure_logged = 0;

    if (context &&
        ptr_readable((BYTE*)context + PERSON_CONTEXT_CURRENT_PERSON_OFFSET,
                     sizeof(int))) {
        person = *(int*)((BYTE*)context +
                         PERSON_CONTEXT_CURRENT_PERSON_OFFSET);
    }
    if (person < 1 || person > 4) person = 0;
    context_visibility = (customize_visibility != 0 && person != 0) ? 1u : 0u;

    /*
     * Per-person configs already contain the normal global configuration with
     * the active body's sidecar overlaid on top.  Read only the effective
     * master switches here: enabled_person is the state these menu commands
     * will toggle, so using it for visibility would hide a disabled control
     * and make it impossible to turn back on.
     */
    if (context_visibility) {
        person_index = person - 1;
        effective_breasts_enabled =
            breasts_physics_person_cfg[person_index].enabled != 0;
        effective_penis_enabled =
            body_chain_physics_person_cfg[person_index].enabled != 0;
        effective_testicle_enabled =
            testicle_physics_person_cfg[person_index].enabled != 0;
        effective_butt_enabled =
            butt_physics_person_cfg[person_index].enabled != 0;
    }
    desired_breasts_visibility =
        (context_visibility && effective_breasts_enabled) ? 1u : 0u;
    desired_penis_visibility =
        (context_visibility && effective_penis_enabled) ? 1u : 0u;
    desired_testicle_visibility =
        (context_visibility && effective_testicle_enabled) ? 1u : 0u;
    desired_butt_visibility =
        (context_visibility && effective_butt_enabled) ? 1u : 0u;
    desired_separator_visibility =
        (desired_breasts_visibility || desired_penis_visibility ||
         desired_testicle_visibility || desired_butt_visibility) ? 1u : 0u;
    desired_menu_visibility = desired_separator_visibility;

    old_person = (int)InterlockedExchange(
        &person_context_selected_person,
        context_visibility ? person : 0);
    person_context_last_context = context;

    if (!person_context_widget_visibility_get(separator_widget,
                                              &separator_visibility) ||
        (separator_visibility != 0) !=
            (desired_separator_visibility != 0)) {
        person_context_widget_visibility_set(separator_widget,
                                             desired_separator_visibility);
    }
    if (!person_context_widget_visibility_get(menu_widget,
                                              &menu_visibility) ||
        (menu_visibility != 0) != (desired_menu_visibility != 0)) {
        person_context_widget_visibility_set(menu_widget,
                                             desired_menu_visibility);
    }
    if (!person_context_widget_visibility_get(breasts_widget,
                                              &breasts_visibility) ||
        (breasts_visibility != 0) !=
            (desired_breasts_visibility != 0)) {
        person_context_widget_visibility_set(breasts_widget,
                                             desired_breasts_visibility);
    }
    if (!person_context_widget_visibility_get(penis_widget,
                                              &penis_visibility) ||
        (penis_visibility != 0) != (desired_penis_visibility != 0)) {
        person_context_widget_visibility_set(penis_widget,
                                             desired_penis_visibility);
    }
    if (!person_context_widget_visibility_get(testicle_widget,
                                              &testicle_visibility) ||
        (testicle_visibility != 0) !=
            (desired_testicle_visibility != 0)) {
        person_context_widget_visibility_set(testicle_widget,
                                             desired_testicle_visibility);
    }
    if (!person_context_widget_visibility_get(butt_widget,
                                              &butt_visibility) ||
        (butt_visibility != 0) != (desired_butt_visibility != 0)) {
        person_context_widget_visibility_set(butt_widget,
                                             desired_butt_visibility);
    }

    if (defaults_cfg.debug &&
        (person_context_last_visibility != (int)context_visibility ||
         person_context_last_breasts_visibility !=
             (int)desired_breasts_visibility ||
         person_context_last_penis_visibility !=
             (int)desired_penis_visibility ||
         person_context_last_testicle_visibility !=
             (int)desired_testicle_visibility ||
         person_context_last_butt_visibility !=
             (int)desired_butt_visibility ||
         old_person != (context_visibility ? person : 0))) {
        log_line("person-context PhysX entries person=%d target_visibility=%u effective_breasts=%d effective_penis=%d effective_testicles=%d effective_butt=%d current_person_state=(breasts:%d,penis:%d,testicles:%d,butt:%d) visibility=(separator:%u,menu:%u,breasts:%u,penis:%u,testicles:%u,butt:%u) body_sidecar_active=%d body_sidecar=\"%s\" customize_visibility=%u context=%p customize=%p separator=%p menu=%p breasts=%p penis=%p testicle=%p butt=%p",
                 context_visibility ? person : 0,
                 context_visibility,
                 effective_breasts_enabled,
                 effective_penis_enabled,
                 effective_testicle_enabled,
                 effective_butt_enabled,
                 person_index >= 0
                     ? breasts_physics_person_cfg[person_index]
                           .enabled_person[person_index]
                     : 0,
                 person_index >= 0
                     ? body_chain_physics_person_cfg[person_index]
                           .enabled_person[person_index]
                     : 0,
                 person_index >= 0
                     ? testicle_physics_person_cfg[person_index]
                           .enabled_person[person_index]
                     : 0,
                 person_index >= 0
                     ? butt_physics_person_cfg[person_index]
                           .enabled_person[person_index]
                     : 0,
                 desired_separator_visibility,
                 desired_menu_visibility,
                 desired_breasts_visibility,
                 desired_penis_visibility,
                 desired_testicle_visibility,
                 desired_butt_visibility,
                 person_index >= 0
                     ? body_profile_person_sidecar_active[person_index]
                     : 0,
                 person_index >= 0 &&
                         body_profile_person_sidecar_active[person_index]
                     ? body_profile_person_sidecar_path[person_index]
                     : "",
                 customize_visibility,
                 context,
                 customize_widget,
                 separator_widget,
                 menu_widget,
                 breasts_widget,
                 penis_widget,
                 testicle_widget,
                 butt_widget);
    }
    person_context_last_visibility = (int)context_visibility;
    person_context_last_breasts_visibility =
        (int)desired_breasts_visibility;
    person_context_last_penis_visibility = (int)desired_penis_visibility;
    person_context_last_testicle_visibility =
        (int)desired_testicle_visibility;
    person_context_last_butt_visibility = (int)desired_butt_visibility;
}

static int person_context_toggle_physx(void *context,
                                       const char *subcmd)
{
    const body_chain_physics_config_t *effective_cfg;
    const char *section;
    const char *kind;
    int selected_person;
    int context_person = 0;
    int person_index;
    int physics_kind;

    enum {
        PERSON_CONTEXT_PHYSX_BREASTS = 0,
        PERSON_CONTEXT_PHYSX_PENIS = 1,
        PERSON_CONTEXT_PHYSX_TESTICLES = 2,
        PERSON_CONTEXT_PHYSX_BUTT = 3
    };

    if (!subcmd) return 0;
    if (strcmp(subcmd, "PhysX_Breasts_Toggle") == 0) {
        physics_kind = PERSON_CONTEXT_PHYSX_BREASTS;
        kind = "breasts";
    } else if (strcmp(subcmd, "PhysX_Penis_Toggle") == 0) {
        physics_kind = PERSON_CONTEXT_PHYSX_PENIS;
        kind = "penis";
    } else if (strcmp(subcmd, "PhysX_Testicle_Toggle") == 0) {
        physics_kind = PERSON_CONTEXT_PHYSX_TESTICLES;
        kind = "testicles";
    } else if (strcmp(subcmd, "PhysX_Butt_Toggle") == 0) {
        physics_kind = PERSON_CONTEXT_PHYSX_BUTT;
        kind = "butt";
    } else {
        return 0;
    }

    selected_person = (int)InterlockedCompareExchange(
        &person_context_selected_person, 0, 0);
    if (context &&
        ptr_readable((BYTE*)context + PERSON_CONTEXT_CURRENT_PERSON_OFFSET,
                     sizeof(int))) {
        context_person = *(int*)((BYTE*)context +
                                  PERSON_CONTEXT_CURRENT_PERSON_OFFSET);
    }
    if (selected_person < 1 || selected_person > 4 ||
        context_person != selected_person) {
        log_line("person-context toggle ignored action=%s selected_person=%d context_person=%d reason=\"stale or non-person context target\"",
                 kind, selected_person, context_person);
        return 1;
    }

    person_index = selected_person - 1;
    if (physics_kind == PERSON_CONTEXT_PHYSX_BREASTS) {
        effective_cfg = &breasts_physics_person_cfg[person_index];
        section = BREASTS_PHYSICS_CONFIG_SECTION;
    } else if (physics_kind == PERSON_CONTEXT_PHYSX_TESTICLES) {
        effective_cfg = &testicle_physics_person_cfg[person_index];
        section = TESTICLE_PHYSICS_CONFIG_SECTION;
    } else if (physics_kind == PERSON_CONTEXT_PHYSX_BUTT) {
        effective_cfg = &butt_physics_person_cfg[person_index];
        section = BUTT_PHYSICS_CONFIG_SECTION;
    } else {
        effective_cfg = &body_chain_physics_person_cfg[person_index];
        section = PENIS_PHYSICS_CONFIG_SECTION;
    }
    if (!effective_cfg->enabled) {
        log_line("person-context toggle ignored action=%s person=Person%02d reason=\"effective master switch is disabled\"",
                 kind, selected_person);
        return 1;
    }

    physx_toggle_person_ini_setting(
        section, kind, person_index,
        effective_cfg->enabled_person[person_index] != 0);
    return 1;
}

static int app_main_command_name(void *command_args,
                                 char *out,
                                 size_t outsz)
{
    HMODULE app;
    const void *cmd_ref;
    unsigned int cmd_hash;
    void *command_value;
    const char *text;

    if (out && outsz) out[0] = 0;
    if (!command_args || !out || outsz < 2) return 0;
    resolve_engine_symbols();
    app = GetModuleHandleA("ThriXXX010278-APP.dll");
    if (!app || !engine_StringRefHash32 || !engine_NameHashFind ||
        !ptr_readable((BYTE*)app + APP_MAIN_COMMAND_NAME_STRINGREF_OFFSET,
                      sizeof(void*))) {
        return 0;
    }

    cmd_ref = *(const void**)((BYTE*)app +
                              APP_MAIN_COMMAND_NAME_STRINGREF_OFFSET);
    if (!cmd_ref || !ptr_readable(cmd_ref, 1)) return 0;
    cmd_hash = engine_StringRefHash32(cmd_ref);
    command_value = engine_NameHashFind(command_args, cmd_hash, cmd_ref);
    if (!command_value || !ptr_readable(command_value, sizeof(char*))) {
        return 0;
    }
    text = *(const char* const*)command_value;
    if (!safe_cstr_a(text, 256)) return 0;
    lstrcpynA(out, text, (int)outsz);
    return out[0] != 0;
}

static int app_main_command_string_arg(void *command_args,
                                       const char *field_name,
                                       char *out,
                                       size_t outsz)
{
    const void *field_ref;
    unsigned int field_hash;
    void *field_value;
    const char *text;
    if (out && outsz) out[0] = 0;
    if (!command_args || !field_name || !field_name[0] ||
        !out || outsz < 2 || !engine_StringRefHash32 ||
        !engine_NameHashFind) {
        return 0;
    }
    field_ref = stringref_from_cstr_a(field_name);
    if (!field_ref) return 0;
    field_hash = engine_StringRefHash32(field_ref);
    field_value = engine_NameHashFind(command_args, field_hash, field_ref);
    if (!field_value || !ptr_readable(field_value, sizeof(char*))) {
        return 0;
    }
    text = *(const char* const*)field_value;
    if (!safe_cstr_a(text, 256)) return 0;
    lstrcpynA(out, text, (int)outsz);
    return out[0] != 0;
}

static DWORD __cdecl hook_AppMain_Command(void *command_args)
{
    char command_name[64];

    if (app_main_command_name(command_args, command_name,
                              sizeof(command_name))) {
        if (strcmp(command_name, "PersonContext_Mode") == 0) {
            char mode_name[32];
            if (app_main_command_string_arg(command_args, "Mode",
                                            mode_name,
                                            sizeof(mode_name))) {
                body_chain_set_poseeditor_mode(
                    _stricmp(mode_name, "PoseEdit") == 0,
                    "PersonContext_Mode.Mode", mode_name);
            } else {
                int poseeditor = 0;
                if (body_chain_read_poseeditor_visibility(&poseeditor) &&
                    poseeditor) {
                    body_chain_set_poseeditor_mode(
                        1,
                        "PersonContext_Mode + "
                        "GUI:PoseEdit_RootGroup.Visibility",
                        "PoseEdit");
                }
            }
        }
        if (person_context_toggle_physx(person_context_last_context,
                                       command_name)) {
            return 0;
        }
    }
    return real_AppMain_Command ? real_AppMain_Command(command_args)
                                : 0x80000001u;
}

static void patch_app_main_command(void)
{
    static const BYTE expected[APP_MAIN_COMMAND_STOLEN_LEN] = {
        0x55, 0x8b, 0xec, 0x53, 0x56
    };
    HMODULE app;
    BYTE *target;

    if (app_main_command_hook_installed) return;
    app = GetModuleHandleA("ThriXXX010278-APP.dll");
    if (!app) return;
    target = (BYTE*)GetProcAddress(
        app, "?Command@AppMain@@YA?AW4EResult@Bionic@@ABVNameHash@3@@Z");
    if (!target) target = (BYTE*)GetProcAddress(app, (LPCSTR)34);
    if (!target || !ptr_executable(target)) return;
    if (target[0] != 0xe9 && memcmp(target, expected, sizeof(expected)) != 0) {
        if (!app_main_command_hook_logged) {
            app_main_command_hook_logged = 1;
            log_line("person-context AppMain command hook unavailable target=%p bytes=%02x %02x %02x %02x %02x reason=\"unsupported APP module build\"",
                     target, target[0], target[1], target[2],
                     target[3], target[4]);
        }
        return;
    }
    if (install_inline_hook(target, (void*)hook_AppMain_Command,
                            APP_MAIN_COMMAND_STOLEN_LEN,
                            (void**)&real_AppMain_Command)) {
        app_main_command_hook_installed = 1;
        log_line("person-context AppMain command hook installed target=%p trampoline=%p commands=4 note=\"consumes only PhysX_Breasts_Toggle, PhysX_Penis_Toggle, PhysX_Testicle_Toggle, and PhysX_Butt_Toggle before TK17's undefined-command path\"",
                 target, (void*)real_AppMain_Command);
    }
}

static void patch_person_context_rebuild(void)
{
    static const BYTE expected[PERSON_CONTEXT_REBUILD_STOLEN_LEN] = {
        0x55, 0x8b, 0xec, 0x83, 0xe4, 0xf8
    };
    HMODULE exe;
    BYTE *target;
    if (person_context_rebuild_hook_installed) return;
    exe = GetModuleHandleA(NULL);
    if (!exe) return;
    target = (BYTE*)exe + PERSON_CONTEXT_REBUILD_OFFSET;
    if (!ptr_executable(target)) return;
    if (target[0] != 0xe9 && memcmp(target, expected, sizeof(expected)) != 0) {
        if (!person_context_rebuild_hook_logged) {
            person_context_rebuild_hook_logged = 1;
            log_line("person-context hook unavailable target=%p bytes=%02x %02x %02x %02x %02x %02x reason=\"unsupported TK17 executable build\"",
                     target, target[0], target[1], target[2],
                     target[3], target[4], target[5]);
        }
        return;
    }
    if (install_inline_hook(target, (void*)hook_PersonContext_Rebuild,
                            PERSON_CONTEXT_REBUILD_STOLEN_LEN,
                            (void**)&real_PersonContext_Rebuild)) {
        person_context_rebuild_hook_installed = 1;
        log_line("person-context hook installed target=%p trampoline=%p note=\"mirrors TK17 person-target visibility to the PhysX context menu\"",
                 target, (void*)real_PersonContext_Rebuild);
    }
}

static void THISCALL hook_PersonContext_Rebuild(void *self,
                                                void *command_name,
                                                void *command_args)
{
    if (real_PersonContext_Rebuild) {
        real_PersonContext_Rebuild(self, command_name, command_args);
    }
    person_context_sync_physx_menu(self);
}

static void THISCALL hook_Object_iNameSet(void *self,
                                          const void *member,
                                          const void *name_ref)
{
    char name_copy[128];
    char room_name_copy[MAX_PATH * 2];
    static int addon_object_name_log_count;
    static int physx_object_name_log_count;
    const char *name = stringref_cstr_a(name_ref);

    name_copy[0] = 0;
    room_name_copy[0] = 0;
    if (name && (contains_i(name, "/Room/") ||
                 contains_i(name, "\\Room\\"))) {
        lstrcpynA(room_name_copy, name, sizeof(room_name_copy));
    }
    if (name && addon_chain_name_interesting(name)) {
        lstrcpynA(name_copy, name, sizeof(name_copy));
    }

    if (tramp_Object_iNameSet) {
        tramp_Object_iNameSet(self, member, name_ref);
    }

    if (room_name_copy[0]) {
        physx_note_room_object_name_a(room_name_copy);
    }

    if (name_copy[0] && self && !is_nil_engine_object(NULL, self)) {
        remember_named_node(name_copy, self);
        if ((defaults_cfg.debug || addon_physics_probe_enabled) &&
            (contains_i(name_copy, "physx_") ||
             addon_binding_probe_name_interesting(name_copy)) &&
            physx_object_name_log_count < 120) {
            physx_object_name_log_count++;
            log_line("addon object-name probe object=%p name=\"%s\" ref=%p index=%d named_nodes=%d note=\"captured custom add-on candidate through Object::iNameSet; read-only binding probe\"",
                     self, name_copy, name_ref, physx_object_name_log_count,
                     named_node_count);
        }
        if (defaults_cfg.debug && addon_object_name_log_count < 240) {
            addon_object_name_log_count++;
            log_line("addon object-name interest object=%p name=\"%s\" ref=%p index=%d named_nodes=%d note=\"captured through Object::iNameSet\"",
                     self, name_copy, name_ref, addon_object_name_log_count,
                     named_node_count);
        }
    }
}

static void note_addon_clone_name(const void *source, void *clone,
                                  const char *source_api)
{
    const char *name = NULL;
    int i;
    static int addon_clone_log_count;
    static int physx_clone_log_count;
    if (source) {
        for (i = 0; i < named_node_count; i++) {
            if (named_nodes[i].object == (void*)source &&
                addon_chain_name_interesting(named_nodes[i].name)) {
                name = named_nodes[i].name;
                break;
            }
        }
    }
    if (name && clone && !is_nil_engine_object(NULL, clone)) {
        char name_copy[128];
        lstrcpynA(name_copy, name, sizeof(name_copy));
        remember_named_node(name_copy, clone);
        if ((defaults_cfg.debug || addon_physics_probe_enabled) &&
            (contains_i(name_copy, "physx_") ||
             addon_binding_probe_name_interesting(name_copy)) &&
            physx_clone_log_count < 120) {
            physx_clone_log_count++;
            log_line("addon clone-name probe api=\"%s\" source=%p clone=%p name=\"%s\" index=%d named_nodes=%d note=\"custom add-on candidate was carried into a cloned room object; read-only binding probe\"",
                     source_api ? source_api : "", source, clone, name_copy,
                     physx_clone_log_count, named_node_count);
        }
        if (defaults_cfg.debug && addon_clone_log_count < 240) {
            addon_clone_log_count++;
            log_line("addon clone-name interest api=\"%s\" source=%p clone=%p name=\"%s\" index=%d named_nodes=%d note=\"captured add-on source object was cloned into live room instance\"",
                     source_api ? source_api : "", source, clone, name_copy,
                     addon_clone_log_count,
                     named_node_count);
        }
    }
}

static void *__cdecl hook_CloneObject(const void *source)
{
    void *clone = NULL;
    if (tramp_CloneObject) {
        clone = tramp_CloneObject(source);
    }
    note_addon_clone_name(source, clone, "CloneObject");
    return clone;
}

static void *__cdecl hook_CloneNode(void *source, void *clone_map)
{
    void *clone = NULL;
    if (tramp_CloneNode) {
        clone = tramp_CloneNode(source, clone_map);
    }
    note_addon_clone_name(source, clone, "CloneNode");
    return clone;
}

static void __cdecl hook_UpdateTraverse(void *object,
                                        const float *matrix,
                                        unsigned int flags)
{
    LONGLONG perf_start = physx_perf_counter();
    physx_body_chain_apply_traverse_overlay(object, 1);
    if (object) {
        physx_addon_apply_traverse_overlay(object, 0);
    }
    physx_addon_apply_traverse_overlay(object, 1);
    physx_perf_add(PHYSX_PERF_TRAVERSE_OVERLAY, perf_start);
    if (tramp_UpdateTraverse) {
        tramp_UpdateTraverse(object, matrix, flags);
    } else if (real_UpdateTraverse &&
               real_UpdateTraverse != hook_UpdateTraverse) {
        real_UpdateTraverse(object, matrix, flags);
    }
}

static DWORD THISCALL hook_AppBase_ProcessAnimation(void *self)
{
    DWORD result = 0x80000001u;
    if (tramp_AppBase_ProcessAnimation) {
        result = tramp_AppBase_ProcessAnimation(self);
    } else if (real_AppBase_ProcessAnimation &&
               real_AppBase_ProcessAnimation !=
                   hook_AppBase_ProcessAnimation) {
        result = real_AppBase_ProcessAnimation(self);
    }
    physx_body_chain_apply_post_animation_ownership();
    return result;
}

/* FreeMode calls SYS+0xE2A70 directly for its live output-joint animation,
   bypassing the script-member table. While runtime penis PhysX owns an exact
   mapped joint, retain the authored value for handoff but pass the joint's
   current PhysX value through the original native writer. This preserves
   TK17's validation/dirty-flag path without allowing animation to overwrite
   the solver. PoseEditor and every unrelated target bypass this filter. */
static void THISCALL hook_RuntimeRotationVectorWrite(
    void *self, const float *value)
{
    float before[3] = {0.0f, 0.0f, 0.0f};
    float input[3] = {0.0f, 0.0f, 0.0f};
    const float *applied = value;
    DWORD *rotation_limit_flags = NULL;
    DWORD saved_rotation_limit_flags = 0;
    int bypass_object_scene_limits = 0;
    runtime_body_chain_ownership_state_t *matched_ownership = NULL;
    int matched_person = -1;
    int matched_joint = -1;
    int person_index;
    physx_sidecar_t *addon_sc = NULL;
    physx_chain_t *addon_chain = NULL;
    physx_target_t *addon_target = NULL;

    if (self && value &&
        InterlockedCompareExchange(
            &addon_object_bone_limit_write_active, 0, 0) &&
        addon_object_bone_limit_write_thread == GetCurrentThreadId() &&
        ptr_readable((BYTE*)self + 0x80, sizeof(DWORD))) {
        /* SYS+0xE2A70 stores RotationLimitEnableMask in bits 8..13.
           The shared bone solver already clamped this output to the sidecar
           limit, so the authored object mask must not clamp it a second time. */
        rotation_limit_flags = (DWORD*)((BYTE*)self + 0x80);
        saved_rotation_limit_flags = *rotation_limit_flags;
        *rotation_limit_flags = saved_rotation_limit_flags & ~0x00003f00u;
        bypass_object_scene_limits = 1;
    }

    if (self && value && body_chain_runtime_mode_active() &&
        InterlockedCompareExchange(
            &body_chain_runtime_penis_native_filter_active, 0, 0) &&
        ptr_readable(value, sizeof(float) * 3) &&
        ptr_readable((BYTE*)self + 0x64, sizeof(float) * 3) &&
        body_chain_vec3_sane_limit(value, 720.0f) &&
        body_chain_vec3_sane_limit(
            (const float*)((BYTE*)self + 0x64), 720.0f)) {
        for (person_index = 0;
             person_index < 4 && matched_person < 0;
             person_index++) {
            runtime_body_chain_ownership_state_t *ownership =
                &runtime_body_chain_ownership_states[person_index];
            body_chain_person_state_t *state =
                &runtime_body_chain_person_states[person_index];
            int joint_index;
            if (!ownership->ownership_active ||
                !ownership->mapping_ready || !ownership->pose_valid ||
                !state->initialized ||
                !body_chain_physics_person_cfg[person_index]
                     .override_animation) {
                continue;
            }
            for (joint_index = 0; joint_index < 3; joint_index++) {
                void *output_receiver =
                    ownership->source_joint_raw[joint_index]
                        ? (BYTE*)ownership->source_joint_raw[joint_index] +
                              0x08
                        : NULL;
                if (self == output_receiver &&
                    ownership->source_joint_raw[joint_index] ==
                        state->joint_raw[joint_index]) {
                    matched_person = person_index;
                    matched_joint = joint_index;
                    matched_ownership = ownership;
                    break;
                }
            }
        }
        if (matched_ownership && matched_joint >= 0) {
            memcpy(before, (BYTE*)self + 0x64, sizeof(before));
            memcpy(input, value, sizeof(input));
            memcpy(
                matched_ownership
                    ->native_output_animation_rotation[matched_joint],
                input, sizeof(input));
            matched_ownership->native_output_animation_valid_mask |=
                1u << matched_joint;
            applied = before;
        }
    }

    if (self && value &&
        addon_should_suppress_animation_write(
            self, SCRIPT_PROPERTY_SSIMPLE_ROTATION,
            &addon_sc, &addon_chain, &addon_target)) {
        static const float neutral[3] = { 0.0f, 0.0f, 0.0f };
        if (addon_sc && addon_sc->room_scene_sidecar && addon_target &&
            InterlockedCompareExchange(
                (volatile LONG*)&addon_target->object_output_applied,
                0, 0) &&
            physx_vec3_sane_limit(
                addon_target->object_output_rotation, 720.0f)) {
            applied = addon_target->object_output_rotation;
        } else {
            applied = neutral;
        }
        addon_note_animation_write_suppressed(
            addon_sc, addon_chain, addon_target,
            SCRIPT_PROPERTY_SSIMPLE_ROTATION, self);
    }

    if (tramp_RuntimeRotationVectorWrite) {
        tramp_RuntimeRotationVectorWrite(self, applied);
    } else if (real_RuntimeRotationVectorWrite &&
               real_RuntimeRotationVectorWrite !=
                   hook_RuntimeRotationVectorWrite) {
        real_RuntimeRotationVectorWrite(self, applied);
    }

    if (bypass_object_scene_limits) {
        *rotation_limit_flags = saved_rotation_limit_flags;
    }

    if (defaults_cfg.debug && matched_ownership &&
        !matched_ownership->native_output_writer_logged) {
        matched_ownership->native_output_writer_logged = 1;
        log_line("runtime native output animation held person=\"%s\" joint=%d receiver=%p ownership=1 captured_mask=0x%x note=\"diagnostic emitted once per mapping; the behavior-critical writer filter remains active\"",
                 body_chain_person_name(matched_person),
                 matched_joint + 1,
                 self,
                 matched_ownership->native_output_animation_valid_mask);
    }
}

static void THISCALL hook_AppTracker_SetWorldMatrixInverse(void *self, const float *matrix)
{
    const float *applied_matrix = matrix;
    LONG test_active = InterlockedCompareExchange(
        &camera_contamination_test_cfg.active, 0, 0);
    LONG test_phase = InterlockedCompareExchange(
        &camera_contamination_test_cfg.phase, 0, 0);

    if (matrix && test_active && test_phase >= 0 &&
        test_phase < CAMERA_CONTAMINATION_TEST_STEP_COUNT) {
        memcpy(camera_contamination_test_engine_camera,
               matrix, sizeof(camera_contamination_test_engine_camera));
        memcpy(camera_contamination_test_applied_camera,
               matrix,
               sizeof(camera_contamination_test_applied_camera));
        camera_contamination_test_camera_valid = 1;
    }

    if (applied_matrix) {
        int i;
        float delta = 0.0f;
        float rotation_delta = 0.0f;
        if (captured_camera_inverse_valid) {
            rotation_delta = camera_rotation_delta(applied_matrix, captured_camera_inverse);
            for (i = 0; i < 16; i++) {
                delta += physx_absf(
                    applied_matrix[i] - captured_camera_inverse[i]);
            }
        }
        memcpy(captured_camera_inverse, applied_matrix,
               sizeof(captured_camera_inverse));
        captured_camera_inverse_valid = 1;
        captured_camera_tick = GetTickCount();
        if (rotation_delta > 0.0005f) {
            captured_camera_rotation_change_tick = captured_camera_tick;
            InterlockedIncrement(&captured_camera_rotation_version);
        }
        if (delta > 0.0005f) {
            captured_camera_change_tick = captured_camera_tick;
            InterlockedIncrement(&captured_camera_version);
            if (!last_camera_gate_log_tick || captured_camera_tick - last_camera_gate_log_tick >= 1000) {
                last_camera_gate_log_tick = captured_camera_tick;
                log_line("camera matrix changed version=%ld delta=%.6f pos=(%.5f,%.5f,%.5f) note=\"used only as contamination signal; body-chain physics does not follow camera movement\"",
                         captured_camera_version, delta,
                         captured_camera_inverse[12], captured_camera_inverse[13], captured_camera_inverse[14]);
            }
        }
    }
    if (tramp_AppTracker_SetWorldMatrixInverse) {
        tramp_AppTracker_SetWorldMatrixInverse(self, applied_matrix);
    } else if (real_AppTracker_SetWorldMatrixInverse &&
               real_AppTracker_SetWorldMatrixInverse != hook_AppTracker_SetWorldMatrixInverse) {
        real_AppTracker_SetWorldMatrixInverse(self, applied_matrix);
    }
}

static int patch_vtable_slot(void *obj, int index, void *hook, void **real)
{
    void **vt;
    DWORD old;
    if (!obj || !hook) return 0;
    vt = *(void***)obj;
    if (!vt || !ptr_readable(vt, sizeof(void*) * (index + 1))) return 0;
    if (real && !*real && vt[index] != hook) *real = vt[index];
    if (vt[index] == hook) return 1;
    if (!VirtualProtect(&vt[index], sizeof(void*), PAGE_EXECUTE_READWRITE, &old)) return 0;
    vt[index] = hook;
    VirtualProtect(&vt[index], sizeof(void*), old, &old);
    return 1;
}

