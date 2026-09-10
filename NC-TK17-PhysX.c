#define WIN32_LEAN_AND_MEAN
#define CINTERFACE
#define COBJMACROS
#include <windows.h>
#include <tlhelp32.h>
#include <d3d8.h>
#include <GL/gl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>
#include <ctype.h>
#include "physx_contact_math.h"
#include "physx_body_pose.h"
#include "physx_body_motion.h"
#include "physx_body_dynamics.h"
#include "physx_gravity_sample.h"
#include "physx_single_bone_contact.h"
#include "physx_collision_frame.h"

static int raw_profile_key_exists_a(const char *section, const char *key,
                                    const char *path)
{
    static const char missing[] = "\x1fmissing\x1f";
    char value[32];
    GetPrivateProfileStringA(section, key, missing, value, sizeof(value), path);
    return strcmp(value, missing) != 0;
}

#define BODY_PROFILE_PERSON_COUNT 4
#define BODY_PROFILE_BODY_SLOT_COUNT 3
#define BODY_PROFILE_PENDING_COUNT 8
#define BODY_PROFILE_SIDECAR_COUNT 64
#define BODY_PROFILE_SIGNATURE_COUNT 16
#define BODY_PROFILE_SIGNATURE_LEN 96
#define BODY_PROFILE_SIGNATURE_MIN_HITS 3

typedef struct body_profile_pending_bind_t {
    int active;
    int person_index;
    int body_slot;
    DWORD tick;
} body_profile_pending_bind_t;

typedef struct body_profile_pending_open_t {
    int active;
    int body_slot;
    DWORD tick;
    char body_path[MAX_PATH * 4];
    char sidecar_path[MAX_PATH * 4];
} body_profile_pending_open_t;

typedef struct body_profile_sidecar_entry_t {
    int active;
    int body_slot;
    DWORD body_hash;
    char addon_cname[160];
    char body_path[MAX_PATH * 4];
    char sidecar_path[MAX_PATH * 4];
    char signature[BODY_PROFILE_SIGNATURE_COUNT][BODY_PROFILE_SIGNATURE_LEN];
    int signature_count;
    DWORD person_signature_mask[BODY_PROFILE_PERSON_COUNT];
} body_profile_sidecar_entry_t;

static char body_profile_person_sidecar_path[BODY_PROFILE_PERSON_COUNT][MAX_PATH * 4];
static char body_profile_person_body_path[BODY_PROFILE_PERSON_COUNT][MAX_PATH * 4];
static FILETIME body_profile_person_sidecar_write_time[BODY_PROFILE_PERSON_COUNT];
static int body_profile_person_sidecar_active[BODY_PROFILE_PERSON_COUNT];
static DWORD body_profile_person_body_hash[BODY_PROFILE_PERSON_COUNT];
static int body_profile_person_bind_strength[BODY_PROFILE_PERSON_COUNT];
static body_profile_pending_bind_t body_profile_pending_bind[BODY_PROFILE_PENDING_COUNT];
static body_profile_pending_open_t body_profile_pending_open[BODY_PROFILE_PENDING_COUNT];
static body_profile_sidecar_entry_t body_profile_sidecars[BODY_PROFILE_SIDECAR_COUNT];
static volatile LONG body_profile_reload_pending;

static const char *body_profile_basename_a(const char *path)
{
    const char *slash;
    const char *backslash;
    if (!path) return "";
    slash = strrchr(path, '/');
    backslash = strrchr(path, '\\');
    if (slash && (!backslash || slash > backslash)) return slash + 1;
    if (backslash) return backslash + 1;
    return path;
}

static int body_profile_body_slot_from_name_a(const char *path, int *slot_out)
{
    const char *name = body_profile_basename_a(path);
    if (_stricmp(name, "body01.bs") == 0) {
        if (slot_out) *slot_out = 0;
        return 1;
    }
    if (_stricmp(name, "body02.bs") == 0) {
        if (slot_out) *slot_out = 1;
        return 1;
    }
    if (_stricmp(name, "body03.bs") == 0) {
        if (slot_out) *slot_out = 2;
        return 1;
    }
    return 0;
}

static int body_profile_sidecar_slot_from_name_a(const char *path,
                                                 int *slot_out)
{
    const char *name = body_profile_basename_a(path);
    if (_stricmp(name, "body01.physx.ini") == 0) {
        if (slot_out) *slot_out = 0;
        return 1;
    }
    if (_stricmp(name, "body02.physx.ini") == 0) {
        if (slot_out) *slot_out = 1;
        return 1;
    }
    if (_stricmp(name, "body03.physx.ini") == 0) {
        if (slot_out) *slot_out = 2;
        return 1;
    }
    return 0;
}

static int body_profile_is_body_sidecar_name(const char *path)
{
    return body_profile_sidecar_slot_from_name_a(path, NULL);
}

static int body_profile_is_global_config_path_a(const char *path)
{
    return _stricmp(body_profile_basename_a(path), "NC-TK17-PhysX.ini") == 0;
}

static int body_profile_section_allowed_a(const char *section)
{
    if (!section) return 0;
    return _stricmp(section, "penis_physics") == 0 ||
           _stricmp(section, "testicle_physics") == 0 ||
           _stricmp(section, "body_colliders") == 0;
}

static int body_profile_sidecar_has_key_a(int slot, const char *section,
                                          const char *key)
{
    (void)slot;
    (void)section;
    (void)key;
    return 0;
}

static const char *body_profile_override_path_a(const char *section,
                                                const char *key,
                                                const char *path)
{
    (void)section;
    (void)key;
    if (!body_profile_is_global_config_path_a(path) ||
        !body_profile_section_allowed_a(section) || !key || !key[0]) {
        return path;
    }
    return path;
}

static const char *resolve_profile_section_a(const char *section, const char *key,
                                             const char *path)
{
    if (!section || _stricmp(section, "body_chain_physics") != 0) return section;
    if (raw_profile_key_exists_a("penis_physics", key, path) ||
        body_profile_override_path_a("penis_physics", key, path) != path) {
        return "penis_physics";
    }
    if (raw_profile_key_exists_a("penis_physics_internal", key, path)) {
        return "penis_physics_internal";
    }
    return section;
}

static DWORD WINAPI physx_get_private_profile_string_a(const char *section,
                                                        const char *key,
                                                        const char *fallback,
                                                        char *out,
                                                        DWORD out_size,
                                                        const char *path)
{
    const char *resolved = resolve_profile_section_a(section, key, path);
    const char *read_path = body_profile_override_path_a(resolved, key, path);
    return GetPrivateProfileStringA(resolved, key, fallback, out, out_size,
                                    read_path);
}

static UINT WINAPI physx_get_private_profile_int_a(const char *section,
                                                   const char *key,
                                                   INT fallback,
                                                   const char *path)
{
    const char *resolved = resolve_profile_section_a(section, key, path);
    const char *read_path = body_profile_override_path_a(resolved, key, path);
    return GetPrivateProfileIntA(resolved, key, fallback, read_path);
}

#define GetPrivateProfileStringA physx_get_private_profile_string_a
#define GetPrivateProfileIntA physx_get_private_profile_int_a

typedef IDirect3D8 *(WINAPI *Direct3DCreate8_t)(UINT);
typedef HRESULT (WINAPI *d3d8_CreateDevice_t)(IDirect3D8 *, UINT, D3DDEVTYPE, HWND, DWORD, D3DPRESENT_PARAMETERS *, IDirect3DDevice8 **);
typedef HRESULT (WINAPI *d3d8_Present_t)(IDirect3DDevice8 *, const RECT *, const RECT *, HWND, const RGNDATA *);
typedef HRESULT (WINAPI *d3d8_BeginScene_t)(IDirect3DDevice8 *);
typedef HRESULT (WINAPI *d3d8_EndScene_t)(IDirect3DDevice8 *);
typedef HRESULT (WINAPI *d3d8_SetTransform_t)(IDirect3DDevice8 *, D3DTRANSFORMSTATETYPE, const D3DMATRIX *);
typedef BOOL (WINAPI *SwapBuffers_t)(HDC);
typedef HANDLE (WINAPI *CreateFileA_t)(LPCSTR, DWORD, DWORD, LPSECURITY_ATTRIBUTES, DWORD, DWORD, HANDLE);
typedef HANDLE (WINAPI *CreateFileW_t)(LPCWSTR, DWORD, DWORD, LPSECURITY_ATTRIBUTES, DWORD, DWORD, HANDLE);
typedef void *(__cdecl *app_find_objc_t)(const char *);
typedef void *(__cdecl *app_engine_t)(void);
typedef void *(__cdecl *app_user_main_t)(void);
typedef void *(__cdecl *search_tree_t)(void *, const void *, void *);
typedef void (__stdcall *poseedit_init_tracks_t)(void);
typedef void *(__stdcall *script_find_object_c_t)(char *);
typedef void (__stdcall *script_import_object_tree_names_t)(void *);
typedef void (__cdecl *set_ts_node_name_t)(void *, const void *);

#if defined(__GNUC__)
#define THISCALL __attribute__((thiscall))
#else
#define THISCALL __thiscall
#endif

typedef void *(THISCALL *get_weak_obj_target_t)(void *);
typedef void (THISCALL *script_get_index_scriptobject_t)(void *, const void *, void *);
typedef void (THISCALL *apptracker_set_world_matrix_inverse_t)(void *, const float *);
typedef void (THISCALL *config_editor_param_change_t)(void *, const char *, const char *, DWORD, DWORD);
typedef void (THISCALL *person_context_rebuild_t)(void *, void *, void *);
typedef DWORD (__cdecl *app_main_command_t)(void *);
typedef unsigned int (THISCALL *stringref_hash32_t)(const void *);
typedef void *(THISCALL *namehash_find_t)(void *, unsigned int, const void *);
typedef void (THISCALL *object_i_name_set_t)(void *, const void *, const void *);
typedef void *(__cdecl *clone_object_t)(const void *);
typedef void *(__cdecl *clone_node_t)(void *, void *);
typedef int (THISCALL *script_bool_property_t)(void *, DWORD);
typedef unsigned int (THISCALL *script_u32_property_t)(void *, DWORD);
typedef void (THISCALL *script_u32_set_property_t)(void *, DWORD, unsigned int);
typedef void (THISCALL *script_vector3_set_property_t)(void *, DWORD,
                                                       const float *);
typedef int (THISCALL *script_index_count_property_t)(void *, DWORD);
typedef unsigned int (THISCALL *tbase_get_matrix_version_t)(void *);
typedef void (THISCALL *tbase_set_matrix_version_t)(void *, unsigned int);
typedef void (THISCALL *poseedit_update_objects_from_tracks_t)(void *);
typedef unsigned char (THISCALL *poseedit_track_evaluate_t)(void *, float *, double);
typedef void (THISCALL *poseedit_track_update_t)(void *, double);
typedef void (__cdecl *update_traverse_t)(void *, const float *, unsigned int);
typedef DWORD (THISCALL *appbase_process_animation_t)(void *);
typedef void (THISCALL *runtime_rotation_vector_write_t)(void *,
                                                         const float *);
typedef void (__cdecl *model_pivot_t)(void *, float *);

static int install_inline_hook(void *target, void *hook, size_t stolen_len, void **trampoline);
static void patch_poseedit_inittracks_hook(void);
static void restore_poseedit_inittracks_hook(void);
static void patch_config_editor_param_change(void);
static void patch_person_context_rebuild(void);
static void patch_app_main_command(void);
static void patch_runtime_animation_member_setters(void);
static void restore_runtime_animation_member_setters(void);
static void patch_addon_constraint_count_getter(void);
static void restore_addon_constraint_count_getter(void);
static void reset_runtime_body_chain_physics(const char *reason);
static void body_chain_prepare_runtime_mode_from_poseeditor(void);
static void body_chain_poll_poseeditor_mode(DWORD now);
static int physx_body_chain_apply_traverse_overlay(void *object,
                                                   int allow_global);
static void physx_body_chain_apply_post_animation_ownership(void);
static void restore_collision_auto_test_active(void);
static void THISCALL hook_AppTracker_SetWorldMatrixInverse(void *self, const float *matrix);
static void THISCALL hook_ConfigEditor_ParamChange(void *self, const char *param_name,
                                                   const char *string_value,
                                                   DWORD value_arg, DWORD event_arg);
static void THISCALL hook_PersonContext_Rebuild(void *self, void *command_name,
                                                void *command_args);
static DWORD __cdecl hook_AppMain_Command(void *command_args);
static void THISCALL hook_Object_iNameSet(void *self, const void *member,
                                          const void *name_ref);
static void __cdecl hook_SetTSNodeName(void *object, const void *name_ref);
static void physx_note_room_object_name_a(const char *object_name);
static int room_wind_direction(float out[3]);
static unsigned int room_wind_generation(void);
static float room_wind_body_strength(const char *person,
                                     const char *system_name,
                                     float wind_scale,
                                     DWORD now);
static void *__cdecl hook_CloneObject(const void *source);
static void *__cdecl hook_CloneNode(void *source, void *clone_map);
static void __cdecl hook_UpdateTraverse(void *object, const float *matrix, unsigned int flags);
static DWORD THISCALL hook_AppBase_ProcessAnimation(void *self);
static void THISCALL hook_RuntimeRotationVectorWrite(
    void *self, const float *value);
static void THISCALL hook_RuntimeJointRotationAxisWrite(
    void *self, DWORD member_id, const float *value);
static void THISCALL hook_SSimpleTransform_RotationSet(
    void *self, DWORD member_id, const float *value);
static void THISCALL hook_SJoint_RotationAxisSet(
    void *self, DWORD member_id, const float *value);
static int THISCALL hook_TBaseTransform_ConstraintArrayCount(
    void *self, DWORD member_id);
static void __stdcall hook_PoseEdit_InitTracks(void);
static float physx_clampf(float v, float lo, float hi);
static int ptr_executable(const void *p);
static void trim_in_place(char *s);
static const char *body_chain_collider_node_label(int node_index);

#define BODY_CHAIN_ANIM_OVERRIDE_MAX_OFFSETS 8
#define POSEEDITOR_VISIBILITY_EXIT_DEBOUNCE_MS 6500u
#define POSEEDIT_INITTRACKS_PATCH_ADDR ((BYTE*)0x004CE710)
#define POSEEDIT_UPDATE_OBJECTS_FROM_TRACKS_ADDR ((BYTE*)0x004CC760)
#define POSEEDIT_TRACK_EVALUATE_ADDR ((BYTE*)0x004F1B10)
#define POSEEDIT_TRACK_UPDATE_ADDR ((BYTE*)0x004F19E0)
#define POSEEDIT_CURRENT_FRAME_OFFSET 0x1F4
#define POSEEDIT_PERSON_MODULE_OFFSET 0x028
#define POSEEDIT_SCENE_CONTEXT_OFFSET 0x028
#define POSEEDIT_SCENE_PERSON_ARRAY_OFFSET 0x0D0
#define SCENE_PERSON_SCRIPT_OBJECT_OFFSET 0x058
#define PERSON_MODULE_PERSON_STATE_PTR_OFFSET 0x0d0
#define MAIN_PERSON_MODULE_OFFSET 0x454
#define PERSON_STATE_PERSON_INDEX_OFFSET 0x008
/* Packed PE Mod PersonState includes field_new01; person_inertia is 0x0fc. */
#define PERSON_STATE_INERTIA_OFFSET 0x0fc
#define PERSON_INERTIA_PERSON_ID_OFFSET 0x004
#define PERSON_INERTIA_PERSON_NAME_OFFSET 0x008
#define PERSON_INERTIA_TEST_ENABLE_OFFSET 0x238
#define PERSON_INERTIA_TEST_FLAGS_OFFSET 0x23c
#define PERSON_INERTIA_TEST_FIELD8_OFFSET 0x240
#define PERSON_INERTIA_BREAST_L_ENABLE_OFFSET 0x010
#define PERSON_INERTIA_BREAST_L_FLAGS_OFFSET 0x014
#define PERSON_INERTIA_BREAST_L_FIELD8_OFFSET 0x018
#define PERSON_INERTIA_BREAST_R_ENABLE_OFFSET 0x124
#define PERSON_INERTIA_BREAST_R_FLAGS_OFFSET 0x128
#define PERSON_INERTIA_BREAST_R_FIELD8_OFFSET 0x12c
#define PERSON_INERTIA_BUTT_L_ENABLE_OFFSET 0x34c
#define PERSON_INERTIA_BUTT_L_FLAGS_OFFSET 0x350
#define PERSON_INERTIA_BUTT_L_FIELD8_OFFSET 0x354
#define PERSON_INERTIA_BUTT_R_ENABLE_OFFSET 0x460
#define PERSON_INERTIA_BUTT_R_FLAGS_OFFSET 0x464
#define PERSON_INERTIA_BUTT_R_FIELD8_OFFSET 0x468
#define PERSON_INERTIA_POSEEDIT_FLAG 0x4
#define SCRIPT_OBJECT_META_BACK_OFFSET 0x018
#define SCRIPT_OBJECT_DISPATCH_TABLE_OFFSET 0x10C
#define SCRIPT_OBJECT_BOOL_DISPATCH_OFFSET 0x180
#define SCRIPT_OBJECT_BOOL_SET_DISPATCH_OFFSET 0x184
#define SCRIPT_PROPERTY_PERSON_VISIBLE 0x06FFF043
#define SCRIPT_OBJECT_U32_DISPATCH_TABLE_OFFSET 0x360
#define SCRIPT_OBJECT_U32_GET_DISPATCH_OFFSET 0x140
#define SCRIPT_OBJECT_U32_SET_DISPATCH_OFFSET 0x144
#define SCRIPT_PROPERTY_WIDGET_VISIBILITY 0x05FFF0D8
#define SCRIPT_PROPERTY_SSIMPLE_ROTATION 0x02FFF04A
#define SCRIPT_PROPERTY_SJOINT_ROTATION_AXIS 0x01FFF04E
#define SCRIPT_PROPERTY_TBASE_CONSTRAINT_ARRAY 0x03FFF049
#define RUNTIME_ROTATION_VECTOR_WRITE_RVA 0x000E2A70u
#define RUNTIME_JOINT_ROTATION_AXIS_WRITE_RVA 0x000DCA90u
#define PERSON_CONTEXT_CURRENT_PERSON_OFFSET 0x4E0
#define PERSON_CONTEXT_REBUILD_OFFSET 0x000B3B40
#define PERSON_CONTEXT_REBUILD_STOLEN_LEN 6
#define APP_MAIN_COMMAND_NAME_STRINGREF_OFFSET 0x0003321C
#define APP_MAIN_COMMAND_STOLEN_LEN 5
#define POSEEDIT_EDITPOSE_OFFSET 0x300
#define POSEEDIT_TRACKS_OFFSET 0x074
#define POSEEDIT_TRACK_SIZE 0x050
#define POSEEDIT_FALLBACK_TOTAL_TRACKS 305
#define POSEEDIT_TRACK_PENIS_JOINT01 144
#define POSEEDIT_TRACK_PENIS_JOINT02 171
#define POSEEDIT_TRACK_PENIS_JOINT03 172
#define POSEEDIT_TRACK_TESTICLES_JOINT01 149
#define POSEEDIT_TRACK_TESTICLES_JOINT02 304
#define POSEEDIT_EXTRA_PHYSICS_TRACK_COUNT 2
#define POSEEDIT_PAIRED_BONE_TRACK_MAX 6
#define POSEEDIT_TRACK_BREAST_LEFT 99
#define POSEEDIT_TRACK_BREAST_RIGHT 100
#define POSEEDIT_TRACK_BREAST_L_HIGH 340
#define POSEEDIT_TRACK_BREAST_R_HIGH 368
#define POSEEDIT_TRACK_BREAST_L_LOW 341
#define POSEEDIT_TRACK_BREAST_R_LOW 369
#define POSEEDIT_TRACK_BUTT_SQUEEZE_LEFT 185
#define POSEEDIT_TRACK_BUTT_SQUEEZE_RIGHT 190
#define POSEEDIT_TRACK_BUTT_LEFT 139
#define POSEEDIT_TRACK_BUTT_RIGHT 140
#define CONFIG_EDITOR_PARAM_CHANGE_OFFSET 0x000fe280
#define CONFIG_EDITOR_PARAM_CHANGE_STOLEN_LEN 6
#define BODY_COLLIDER_NODE_COUNT 84
#define BODY_COLLIDER_LIMB_PAIR_COUNT 4
#define BODY_COLLIDER_EXTRA_EDGE_COUNT 56
#define BODY_COLLIDER_DIRECT_NODE_COUNT 70
#define BODY_COLLIDER_FINGER_COUNT 5
#define PHYSX_COLLISION_SCOPE_SELF   (1u << 0)
#define PHYSX_COLLISION_SCOPE_BODY   (1u << 1)
#define PHYSX_COLLISION_SCOPE_ADDONS (1u << 2)
#define PHYSX_COLLISION_SCOPE_BODY_ALL (1u << 3)
#define PHYSX_COLLISION_SCOPE_CUSTOM (1u << 4)
#define PHYSX_COLLISION_SCOPE_ROOM   (1u << 5)
#define BODY_COLLIDERS_CONFIG_SECTION "body_colliders"
#define BODY_COLLIDERS_LEGACY_CONFIG_SECTION "body_chain_colliders"
#define PENIS_PHYSICS_CONFIG_SECTION "penis_physics"
#define TESTICLE_PHYSICS_CONFIG_SECTION "testicle_physics"
#define BREASTS_PHYSICS_CONFIG_SECTION "breasts_physics"
#define BREASTS_PHYSICS_INTERNAL_CONFIG_SECTION "breasts_physics_internal"
#define BUTT_PHYSICS_CONFIG_SECTION "butt_physics"
#define BUTT_PHYSICS_INTERNAL_CONFIG_SECTION "butt_physics_internal"
#define PENIS_PHYSICS_INTERNAL_CONFIG_SECTION "penis_physics_internal"
#define TESTICLE_PHYSICS_INTERNAL_CONFIG_SECTION "testicle_physics_internal"
#define PENIS_PHYSICS_LEGACY_CONFIG_SECTION "body_chain_physics"
#define BODY_CHAIN_ENGINE_POINT_STALE_MS 500
#define BODY_CHAIN_COLLIDER_OBJECT_VERIFY_MS 500
#define BODY_CHAIN_COLLISION_POINT_HOLD_MS 80
#define BODY_CHAIN_COLLIDER_SETTLE_MS 650
#define BODY_CHAIN_COLLISION_TARGET_PENIS 0
#define BODY_CHAIN_COLLISION_TARGET_TESTICLES 1
#define BODY_CHAIN_COLLISION_SCOPE_GENITALS_ONLY 0
#define BODY_CHAIN_COLLISION_SCOPE_GENITALS_ONLY_ALL 1
#define BODY_CHAIN_COLLISION_SCOPE_PELVIS_AND_GENITALS_ONLY 2
#define BODY_CHAIN_COLLISION_SCOPE_PELVIS_AND_GENITALS_ONLY_ALL 3
#define BODY_CHAIN_COLLISION_SCOPE_PELVIS_GENITALS_AND_HANDS_ONLY 4
#define BODY_CHAIN_COLLISION_SCOPE_PELVIS_GENITALS_AND_HANDS_ONLY_ALL 5
#define BODY_CHAIN_COLLISION_SCOPE_FULL_BODY 6
#define BODY_CHAIN_COLLISION_SCOPE_FULL_BODY_ALL 7
#define BODY_CHAIN_COLLISION_SCOPE_HANDS_ONLY 8
#define BODY_CHAIN_COLLISION_SCOPE_HANDS_ONLY_ALL 9
#define BODY_CHAIN_COLLIDER_GROUP_GENITALS 0x01
#define BODY_CHAIN_COLLIDER_GROUP_PELVIS 0x02
#define BODY_CHAIN_COLLIDER_GROUP_HANDS 0x04
#define BODY_CHAIN_COLLIDER_GROUP_FULL_BODY 0x08
#define BODY_CHAIN_COLLIDER_GROUP_BREAST_SOURCE 0x10
#define BODY_CHAIN_COLLIDER_GROUP_BUTT_SOURCE 0x20
#define BODY_CHAIN_COLLIDER_GROUP_ALL_TARGETS 0x0f
#define BODY_CHAIN_COLLIDER_GROUP_ALL 0x3f
#define BODY_CHAIN_COLLIDER_ROOT_QUIET_MS 1500
#define BODY_CHAIN_COLLIDER_STABLE_SAMPLES 3
#define BODY_CHAIN_COLLIDER_STABLE_DELTA 0.120f
#define BODY_CHAIN_MAX_CONTACTS 24
#define PENIS_PHYSICS_MISSING_RETRY_MS 100
#define TESTICLE_PHYSICS_CACHE_VERIFY_MS 500
#define TESTICLE_PHYSICS_MISSING_RETRY_MS 100
#define TESTICLE_INERTIA_CACHE_VERIFY_MS 100
#define BREASTS_PHYSICS_CACHE_VERIFY_MS 500
#define BREASTS_PHYSICS_MISSING_RETRY_MS 100
#define BUTT_PHYSICS_CACHE_VERIFY_MS 500
#define BUTT_PHYSICS_MISSING_RETRY_MS 100
#define CAMERA_CONTAMINATION_TEST_STEP_COUNT 11

enum body_collider_node_index_t {
    BODY_COLLIDER_ROOT = 0,
    BODY_COLLIDER_STOMACH_01,
    BODY_COLLIDER_STOMACH_02,
    BODY_COLLIDER_HIP_L,
    BODY_COLLIDER_HIP_R,
    BODY_COLLIDER_KNEE_L,
    BODY_COLLIDER_KNEE_R,
    BODY_COLLIDER_THIGH_L,
    BODY_COLLIDER_THIGH_R,
    BODY_COLLIDER_TESTICLES_01,
    BODY_COLLIDER_TESTICLES_02,
    BODY_COLLIDER_TESTICLES_MID,
    BODY_COLLIDER_STOMACH_03,
    BODY_COLLIDER_STOMACH_04,
    BODY_COLLIDER_NECK_01,
    BODY_COLLIDER_ANKLE_L,
    BODY_COLLIDER_ANKLE_R,
    BODY_COLLIDER_BALL_L,
    BODY_COLLIDER_BALL_R,
    BODY_COLLIDER_BREAST_L,
    BODY_COLLIDER_BREAST_R,
    BODY_COLLIDER_HEAD_02,
    BODY_COLLIDER_CLAVICLE_L,
    BODY_COLLIDER_CLAVICLE_R,
    BODY_COLLIDER_SHOULDER_L,
    BODY_COLLIDER_SHOULDER_R,
    BODY_COLLIDER_ELBOW_L,
    BODY_COLLIDER_ELBOW_R,
    BODY_COLLIDER_FOREARM_L,
    BODY_COLLIDER_FOREARM_R,
    BODY_COLLIDER_WRIST_L,
    BODY_COLLIDER_WRIST_R,
    BODY_COLLIDER_PALM_L,
    BODY_COLLIDER_PALM_R,
    BODY_COLLIDER_FINGER01_L_01,
    BODY_COLLIDER_FINGER01_L_02,
    BODY_COLLIDER_FINGER01_L_03,
    BODY_COLLIDER_FINGER01_L_END,
    BODY_COLLIDER_FINGER01_R_01,
    BODY_COLLIDER_FINGER01_R_02,
    BODY_COLLIDER_FINGER01_R_03,
    BODY_COLLIDER_FINGER01_R_END,
    BODY_COLLIDER_FINGER02_L_01,
    BODY_COLLIDER_FINGER02_L_02,
    BODY_COLLIDER_FINGER02_L_03,
    BODY_COLLIDER_FINGER02_L_04,
    BODY_COLLIDER_FINGER02_L_END,
    BODY_COLLIDER_FINGER02_R_01,
    BODY_COLLIDER_FINGER02_R_02,
    BODY_COLLIDER_FINGER02_R_03,
    BODY_COLLIDER_FINGER02_R_04,
    BODY_COLLIDER_FINGER02_R_END,
    BODY_COLLIDER_FINGER03_L_01,
    BODY_COLLIDER_FINGER03_L_02,
    BODY_COLLIDER_FINGER03_L_03,
    BODY_COLLIDER_FINGER03_L_04,
    BODY_COLLIDER_FINGER03_L_END,
    BODY_COLLIDER_FINGER03_R_01,
    BODY_COLLIDER_FINGER03_R_02,
    BODY_COLLIDER_FINGER03_R_03,
    BODY_COLLIDER_FINGER03_R_04,
    BODY_COLLIDER_FINGER03_R_END,
    BODY_COLLIDER_FINGER04_L_01,
    BODY_COLLIDER_FINGER04_L_02,
    BODY_COLLIDER_FINGER04_L_03,
    BODY_COLLIDER_FINGER04_L_04,
    BODY_COLLIDER_FINGER04_L_END,
    BODY_COLLIDER_FINGER04_R_01,
    BODY_COLLIDER_FINGER04_R_02,
    BODY_COLLIDER_FINGER04_R_03,
    BODY_COLLIDER_FINGER04_R_04,
    BODY_COLLIDER_FINGER04_R_END,
    BODY_COLLIDER_FINGER05_L_01,
    BODY_COLLIDER_FINGER05_L_02,
    BODY_COLLIDER_FINGER05_L_03,
    BODY_COLLIDER_FINGER05_L_04,
    BODY_COLLIDER_FINGER05_L_END,
    BODY_COLLIDER_FINGER05_R_01,
    BODY_COLLIDER_FINGER05_R_02,
    BODY_COLLIDER_FINGER05_R_03,
    BODY_COLLIDER_FINGER05_R_04,
    BODY_COLLIDER_FINGER05_R_END,
    BODY_COLLIDER_BUTT_L,
    BODY_COLLIDER_BUTT_R
};

enum body_collider_limb_pair_index_t {
    BODY_COLLIDER_LIMB_PAIR_L_HIP_THIGH = 0,
    BODY_COLLIDER_LIMB_PAIR_L_THIGH_KNEE,
    BODY_COLLIDER_LIMB_PAIR_R_HIP_THIGH,
    BODY_COLLIDER_LIMB_PAIR_R_THIGH_KNEE
};

typedef struct body_collider_direct_node_def_t {
    int node_index;
    const char *node_name;
    const char *fallback_name;
    int side_sign;
    const char *source_name;
} body_collider_direct_node_def_t;

typedef struct body_collider_extra_edge_def_t {
    int start_node;
    int end_node;
    const char *name;
    DWORD d3d_color;
    unsigned char r;
    unsigned char g;
    unsigned char b;
} body_collider_extra_edge_def_t;

typedef struct physx_defaults_t {
    float stiffness;
    float damping;
    float gravity[3];
    float limit_angle;
    int debug;
    int performance_profile;
    int config_reload_poll_ms;
} physx_defaults_t;

typedef enum physx_perf_phase_t {
    PHYSX_PERF_HOUSEKEEPING = 0,
    PHYSX_PERF_BINDINGS,
    PHYSX_PERF_DIAGNOSTICS,
    PHYSX_PERF_COLLIDER_REFRESH,
    PHYSX_PERF_AXIS_REFERENCES,
    PHYSX_PERF_PENIS_PHYSICS,
    PHYSX_PERF_PENIS_OWNERSHIP,
    PHYSX_PERF_PENIS_GRAVITY,
    PHYSX_PERF_PENIS_COLLISION,
    PHYSX_PERF_PENIS_SETUP,
    PHYSX_PERF_PENIS_ORIENTATION,
    PHYSX_PERF_PENIS_TRANSLATION,
    PHYSX_PERF_PENIS_SOLVER,
    PHYSX_PERF_TESTICLE_PHYSICS,
    PHYSX_PERF_TESTICLE_ACTIVE,
    PHYSX_PERF_BREASTS_PHYSICS,
    PHYSX_PERF_BREASTS_ACTIVE,
    PHYSX_PERF_BUTT_PHYSICS,
    PHYSX_PERF_BUTT_ACTIVE,
    PHYSX_PERF_ADDON_SIMULATION,
    PHYSX_PERF_ADDON_ACTIVATION,
    PHYSX_PERF_ADDON_DRIVE,
    PHYSX_PERF_ADDON_GUARD,
    PHYSX_PERF_ADDON_PARENT_PIVOT,
    PHYSX_PERF_ADDON_PARENT_TRANSLATION,
    PHYSX_PERF_ADDON_PARENT_ROTATION,
    PHYSX_PERF_ADDON_GRAVITY,
    PHYSX_PERF_ADDON_ROTATION_BINDING,
    PHYSX_PERF_ADDON_VALIDATION,
    PHYSX_PERF_ADDON_SOLVER,
    PHYSX_PERF_ADDON_OUTPUT,
    PHYSX_PERF_ADDON_SUPPRESSION_CACHE,
    PHYSX_PERF_LATE_OWNERSHIP,
    PHYSX_PERF_LATE_TESTICLES,
    PHYSX_PERF_LATE_BREASTS,
    PHYSX_PERF_LATE_BUTT,
    PHYSX_PERF_TRAVERSE_OVERLAY,
    PHYSX_PERF_ADDON_BODY_COLLISION,
    PHYSX_PERF_ADDON_SELF_COLLISION,
    PHYSX_PERF_ADDON_ADDONS_COLLISION,
    PHYSX_PERF_TOTAL,
    PHYSX_PERF_PHASE_COUNT
} physx_perf_phase_t;

typedef struct physx_perf_state_t {
    LARGE_INTEGER frequency;
    LONGLONG accumulated[PHYSX_PERF_PHASE_COUNT];
    LONGLONG maximum[PHYSX_PERF_PHASE_COUNT];
    DWORD calls[PHYSX_PERF_PHASE_COUNT];
    DWORD report_tick;
    int ready;
} physx_perf_state_t;

typedef struct body_probe_config_t {
    int enabled;
    char person[32];
    char node[128];
    int use_object;
    int offset;
    int axis;
    float amount;
    int duration_ms;
    int cycle;
    DWORD start_tick;
    int state;
    int cycle_index;
    void *raw;
    char runtime[384];
    float original[3];
} body_probe_config_t;

typedef struct transform_probe_config_t {
    int enabled;
    char person[32];
    int interval_ms;
    int scan_floats;
    int top_count;
    int include_object;
    float threshold;
    DWORD last_tick;
} transform_probe_config_t;

typedef struct axis_map_probe_config_t {
    int enabled;
    char person[32];
    int interval_ms;
    float threshold;
    float phase_start_threshold;
    int phase_seconds;
    int phase_start_delay_ms;
    DWORD phase_start_tick;
    int phase_logged;
    DWORD last_tick;
} axis_map_probe_config_t;

typedef struct root_drive_probe_config_t {
    int enabled;
    char person[32];
    char source_node[128];
    char target_node[128];
    int source_offset;
    int source_axis;
    int target_offset;
    int target_axis;
    float scale;
    float max_amount;
    float threshold;
    int interval_ms;
    DWORD last_tick;
    void *source_raw;
    void *target_raw;
    int initialized;
    int resolved_logged;
    float source_rest[3];
    float source_prev[3];
    float target_rest[3];
} root_drive_probe_config_t;

typedef struct write_sweep_probe_config_t {
    int enabled;
    char person[32];
    int duration_ms;
    int start_delay_ms;
    DWORD session_start_tick;
    int delay_logged;
    DWORD start_tick;
    int state;
    int index;
    void *base;
    float original[3];
} write_sweep_probe_config_t;

typedef struct collision_auto_test_config_t {
    int enabled;
    char person[32];
    char mode[32];
    int start_delay_ms;
    int phase_ms;
    int rest_ms;
    float root_amount;
    float testicle_amount;
    float hip_amount;
    DWORD ready_tick;
    DWORD phase_tick;
    int phase;
    int state;
    int completed;
    void *active_raw;
    int active_offset;
    int active_axis;
    int active_basis_mode;
    int active_basis_offsets[3];
    float active_original[9];
    void *secondary_raw;
    int secondary_offset;
    float secondary_original[3];
} collision_auto_test_config_t;

typedef struct collision_auto_test_step_t {
    const char *node;
    const char *fallback_node;
    int offset;
    int axis;
    float amount_sign;
    int root_step;
    int basis_rotation;
    const char *secondary_node;
    int secondary_offset;
    int secondary_axis;
    float secondary_sign;
    const char *label;
} collision_auto_test_step_t;

typedef struct camera_contamination_test_config_t {
    int enabled;
    char person[32];
    int start_delay_ms;
    int hold_ms;
    int sample_ms;
    int input_drag_ms;
    int input_yaw_pixels;
    int input_pitch_pixels;
    int auto_focus;
    DWORD ready_tick;
    DWORD phase_tick;
    DWORD last_sample_tick;
    DWORD input_drag_start_tick;
    DWORD input_drag_last_log_tick;
    volatile LONG active;
    volatile LONG phase;
    int completed;
    unsigned int sample_index;
    int input_drag_sent_x;
    int input_drag_sent_y;
    int input_drag_button_down;
    int input_drag_foreground;
} camera_contamination_test_config_t;

typedef struct camera_contamination_test_step_t {
    float yaw_degrees;
    float pitch_degrees;
    int input_dx;
    int input_dy;
    const char *label;
} camera_contamination_test_step_t;

typedef struct write_sweep_candidate_t {
    const char *node;
    int offset;
    int axis;
    float amount;
    const char *label;
} write_sweep_candidate_t;

typedef struct body_chain_physics_config_t {
    int enabled;
    int wind_enabled;
    float wind_scale;
    int wind_source_axis[3];
    int wind_tail_axis[3];
    float wind_axis_scale[3];
    int collision_scope;
    int root_offset;
    int output_offset;
    int animation_output_offset;
    int animation_override_offsets[BODY_CHAIN_ANIM_OVERRIDE_MAX_OFFSETS];
    int animation_override_offset_count;
    int override_animation;
    int joint01_pose_override;
    int joint01_pose_override_offsets[BODY_CHAIN_ANIM_OVERRIDE_MAX_OFFSETS];
    int joint01_pose_override_offset_count;
    int joint01_transform_lock;
    int joint01_transform_lock_offset;
    int poseeditor_track_override;
    int poseeditor_track_diagnostic;
    int poseeditor_total_tracks;
    int translation_source_axis[3];
    int translation_tail_axis[3];
    float translation_scale[3];
    int face_down_translation_channel;
    float face_down_translation_sign;
    int rotation_source_axis[3];
    int rotation_tail_axis[3];
    float rotation_scale[3];
    float translation_deadzone;
    float rotation_deadzone;
    /* Derived compatibility mapping used by gravity and collision geometry. */
    int horizontal_source_axis;
    int vertical_source_axis;
    float horizontal_source_vector[3];
    float vertical_source_vector[3];
    int horizontal_output_axis;
    int vertical_output_axis;
    int horizontal_secondary_output_axis;
    float horizontal_secondary_output_scale;
    float horizontal_sign;
    float vertical_sign;
    float drive_scale;
    float horizontal_drive_scale;
    float vertical_drive_scale;
    float horizontal_deadzone;
    float vertical_deadzone;
    float gravity_angle;
    float gravity_horizontal_curve;
    float gravity_vertical_curve;
    float gravity_inverted_strength;
    int gravity_inverted_tail_axis;
    float gravity_inverted_sign;
    float chain_total_bend_max;
    float chain_total_twist_max;
    float stiffness;
    float damping;
    float max_angle;
    float link_max_angle[3][3];
    float link_min_angle[3][3];
    float link_gain[3];
    int interval_ms;
    int zero_output_rest;
    /* Public paired-body controls. Penis/testicle configs leave these
       unused; breasts and butt copy and overlay them per selected body. */
    int bone_translation_enabled;
    int bone_translation_space;
    float bone_translation_scale[3];
    float bone_translation_gravity_sag;
    float bone_translation_stiffness;
    float bone_translation_damping;
    float bone_translation_max_offset[3];
    float gravity_inward_strength;
    float gravity_outward_strength;
    int enabled_person[4];
    DWORD last_tick;
    int room_collision_enabled;
} body_chain_physics_config_t;

typedef struct physics_environment_config_t {
    int world_gravity_probe;
    int wind_enabled;
    float world_gravity[3];
    float gravity_horizontal_source_vector[3];
    float gravity_vertical_source_vector[3];
    int gravity_apply_to_body_chain;
    float gravity_body_chain_scale;
    float gravity_horizontal_body_chain_scale;
    float gravity_vertical_body_chain_scale;
    float gravity_horizontal_secondary_body_chain_scale;
    float gravity_vertical_secondary_body_chain_scale;
    int gravity_horizontal_tail_axis;
    int gravity_vertical_tail_axis;
    int body_chain_camera_relative_orientation;
    float body_chain_camera_coast_stiffness_scale;
    float body_chain_camera_coast_damping_scale;
    int body_chain_camera_quarantine_ms;
    int gravity_dynamic_body_basis;
    int gravity_basis_camera_compensate;
    char gravity_basis_node[64];
    int gravity_horizontal_basis_offset;
    int gravity_vertical_basis_offset;
    int gravity_horizontal_secondary_basis_offset;
    float gravity_horizontal_basis_sign;
    float gravity_vertical_basis_sign;
    float gravity_horizontal_secondary_basis_sign;
    int gravity_zero_at_start;
    float gravity_response_ms;
    float gravity_max_degrees_per_second;
    int gravity_probe_settle_ms;
    int gravity_probe_confirm_ms;
    int gravity_probe_camera_quiet_ms;
    int gravity_probe_log_ms;
    float gravity_probe_motion_epsilon;
    float gravity_probe_invalidate_epsilon;
    int gravity_probe_require_nonzero_root;
} physics_environment_config_t;

typedef struct body_chain_collider_config_t {
    int enabled;
    int debug_draw;
    int response_enabled;
    int breasts_collision_enabled;
    int butt_collision_enabled;
    int penis_collision_enabled;
    int testicle_collision_enabled;
    int diagnostic;
    int root_local_offsets;
    int live_testicle_bones;
    int testicles_bone_head_mode;
    int testicle_rotation_offset;
    int position_offset;
    float local_offset[BODY_COLLIDER_NODE_COUNT][3];
    float body_chain_base_offset[3];
    float testicle_pivot[2][3];
    float testicle_head[2][3];
    float testicle_fine_offset[2][3];
    float stomach_fine_offset[2][3];
    float stomach_extra_fine_offset[2][3];
    float hip_fine_offset[3];
    float knee_fine_offset[3];
    float thigh_fine_offset[3];
    float ankle_fine_offset[3];
    float ball_fine_offset[3];
    float breast_fine_offset[3];
    float butt_fine_offset[3];
    float neck_fine_offset[3];
    float head_fine_offset[3];
    float clavicle_fine_offset[3];
    float shoulder_fine_offset[3];
    float elbow_fine_offset[3];
    float forearm_fine_offset[3];
    float wrist_fine_offset[3];
    float palm_fine_offset[3];
    float finger_fine_offset[BODY_COLLIDER_FINGER_COUNT][3];
    float pelvis_radius;
    float stomach_radius[2][3];
    float stomach_extra_radius[2][3];
    float hip_radius;
    float thigh_radius;
    float knee_radius;
    float testicles_radius;
    float ankle_radius[3];
    float ball_radius[3];
    float breast_radius[3];
    float butt_radius[3];
    float neck_radius[3];
    float head_radius[3];
    float clavicle_radius[3];
    float shoulder_radius[3];
    float elbow_radius[3];
    float forearm_radius[3];
    float wrist_radius[3];
    float palm_radius[3];
    float finger_radius[BODY_COLLIDER_FINGER_COUNT][3];
    float node_radius[BODY_COLLIDER_NODE_COUNT][3];
    float node_fine_offset[BODY_COLLIDER_NODE_COUNT][3];
    float pelvis_capsule_radius;
    float response_radius_scale;
    float chain_radius;
    float link_length[3];
    float response_strength;
    float response_max_degrees_per_tick;
    int collision_iterations;
    float collision_slop;
    int health_log_ms;
    int response_log_ms;
} body_chain_collider_config_t;

typedef struct body_chain_collider_person_state_t {
    collision_frame_sample_t contact_frame_sample;
    float contact_view_to_world[9], contact_world_to_view[9], contact_world_origin[3];
    int contact_frame_valid;
    DWORD contact_frame_log_tick;
    void *raw[BODY_COLLIDER_NODE_COUNT];
    int position_offset[BODY_COLLIDER_NODE_COUNT];
    char source_name[BODY_COLLIDER_NODE_COUNT][128];
    float view_position[BODY_COLLIDER_NODE_COUNT][3];
    float world_position[BODY_COLLIDER_NODE_COUNT][3];
    float local_position[BODY_COLLIDER_NODE_COUNT][3];
    int valid[BODY_COLLIDER_NODE_COUNT];
    float basis_h[3];
    float basis_v[3];
    float basis_s[3];
    int basis_valid;
    int active_scope_mask;
    float chain_local_point[4][3];
    int chain_point_valid[4];
    int chain_points_ready;
    int chain_points_fresh;
    DWORD chain_points_update_tick;
    float limb_local_position[6][3];
    int limb_points_ready;
    DWORD limb_points_update_tick;
    float stomach_local_position[2][3];
    int stomach_points_ready;
    DWORD stomach_points_update_tick;
    float testicle_local_position[2][3];
    float testicle_joint_position[3][3];
    int testicle_points_ready;
    DWORD testicle_points_update_tick;
    float extra_local_position[BODY_COLLIDER_NODE_COUNT][3];
    void *extra_object[BODY_COLLIDER_NODE_COUNT];
    DWORD extra_object_verify_tick[BODY_COLLIDER_NODE_COUNT];
    int extra_point_ready[BODY_COLLIDER_NODE_COUNT];
    DWORD extra_point_update_tick[BODY_COLLIDER_NODE_COUNT];
    int sample_ready;
    DWORD settle_start_tick;
    int settle_sample_count;
    int settle_logged;
    DWORD last_settle_log_tick;
    float settle_prev_local[BODY_COLLIDER_NODE_COUNT][3];
    float settle_prev_chain[4][3];
    int settle_prev_chain_ready;
    float settle_prev_root_view[3];
    int settle_prev_root_ready;
    DWORD settle_root_quiet_tick;
    int ready;
    int ready_logged;
    void *testicle_rotation_raw[2];
    float testicle_rotation_rest[2][3];
    int testicle_rotation_ready[2];
    DWORD last_testicle_rotation_log_tick;
    DWORD last_testicle_reject_log_tick;
    DWORD last_chain_point_log_tick;
    DWORD last_chain_point_reject_log_tick;
    unsigned int chain_point_reject_count;
    DWORD last_limb_reject_log_tick;
    DWORD last_stomach_reject_log_tick;
    LONG scene_liveness_camera_version;
    int scene_liveness_static_camera_samples;
    float scene_liveness_root_view[3];
    int scene_liveness_quarantined;
    void *scene_liveness_quarantined_root_raw;
    int scene_liveness_engine_invisible;
    DWORD last_scene_liveness_log_tick;
    DWORD last_health_log_tick;
    DWORD last_response_log_tick;
} body_chain_collider_person_state_t;

typedef struct body_chain_person_state_t {
    DWORD last_tick;
    DWORD resolve_retry_tick;
    DWORD init_tick;
    int initialized;
    int active_logged;
    unsigned int wind_generation;
    int wind_logged;
    DWORD last_log_tick;
    DWORD cache_verify_tick;
    DWORD health_log_tick;
    DWORD gravity_probe_log_tick;
    DWORD gravity_probe_stable_tick;
    DWORD gravity_probe_candidate_tick;
    int gravity_probe_captured;
    int gravity_probe_promoted;
    int gravity_probe_sampled;
    float gravity_probe_root[3];
    float gravity_probe_prev_root[3];
    float gravity_probe_last_root[3];
    float gravity_probe_root_world[3];
    float gravity_probe_prev_root_world[3];
    float gravity_probe_last_root_world[3];
    int gravity_probe_world_valid;
    float gravity_drive[3];
    float gravity_drive_filtered[3];
    int gravity_drive_filtered_valid;
    float gravity_drive_ref[3];
    int gravity_drive_ref_valid;
    DWORD gravity_basis_stable_tick;
    int gravity_basis_sampled;
    float gravity_basis_prev[3];
    int gravity_camera_hold_active;
    gravity_sample_t gravity_sample, geometry_sample;
    LONG gravity_probe_candidate_camera_version;
    int gravity_probe_reactivation_ready;
    DWORD gravity_probe_reactivation_tick;
    void *gravity_probe_reactivation_root_raw;
    LONG camera_seen_version;
    DWORD gravity_response_trace_tick;
    LONG root_drive_camera_seen_version;
    DWORD root_drive_camera_quarantine_tick;
    DWORD root_drive_camera_last_untrusted_tick;
    DWORD root_drive_camera_ramp_log_tick;
    DWORD root_drive_untrusted_log_tick;
    DWORD late_ownership_log_tick;
    DWORD live_ownership_simulation_serial;
    LONG live_ownership_node_generation;
    DWORD ownership_candidate_tick;
    unsigned int ownership_candidate_samples;
    void *ownership_candidate_root_raw;
    void *ownership_candidate_joint_raw[3];
    int root_drive_last_step_valid;
    float root_drive_last_h_step;
    float root_drive_last_v_step;
    float root_drive_last_d_step;
    body_motion_filter_t root_drive_filter;
    body_dynamics_t dynamics;
    int dynamics_valid;
    float dynamics_gravity_direction[3];
    int dynamics_gravity_valid;
    int dynamics_gravity_active;
    void *camera_relative_trs_raw;
    int camera_relative_live_prev_valid;
    float camera_relative_live_prev[9];
    int camera_relative_live_current_valid;
    float camera_relative_live_current[9];
    float camera_relative_h_coeff[9];
    float camera_relative_v_coeff[9];
    float camera_relative_d_coeff[9];
    unsigned int camera_relative_calibration_samples;
    float camera_relative_h_peak;
    float camera_relative_v_peak;
    float camera_relative_d_peak;
    DWORD camera_relative_live_log_tick;
    void *root_raw;
    void *joint_raw[3];
    void *anim_joint_raw[3];
    float anim_rest[3][BODY_CHAIN_ANIM_OVERRIDE_MAX_OFFSETS][3];
    int anim_rest_valid;
    float pose_compensation[3][3];
    int pose_compensation_valid;
    int pose_compensation_logged;
    float joint01_pose_rest[BODY_CHAIN_ANIM_OVERRIDE_MAX_OFFSETS][3];
    int joint01_pose_rest_valid;
    int joint01_transform_lock_logged;
    void *pose_track_base;
    void *pose_track_saved_obj;
    void *pose_track_saved_track_data;
    int pose_track_suppressed;
    void *pose_track_extra_base[POSEEDIT_EXTRA_PHYSICS_TRACK_COUNT];
    void *pose_track_extra_saved_obj[POSEEDIT_EXTRA_PHYSICS_TRACK_COUNT];
    void *pose_track_extra_saved_track_data[POSEEDIT_EXTRA_PHYSICS_TRACK_COUNT];
    int pose_track_extra_suppressed[POSEEDIT_EXTRA_PHYSICS_TRACK_COUNT];
    int pose_track_logged;
    int pose_track_extra_logged;
    DWORD pose_track_scan_tick;
    float root_prev[3];
    float root_prev_world[3];
    int root_prev_world_valid;
    int root_translation_initialized;
    void *root_translation_raw;
    float root_translation_parent_rest[9];
    int root_translation_parent_rest_valid;
    void *root_translation_parent_rest_root_raw;
    void *root_translation_parent_rest_trs_raw;
    int root_translation_camera_hold_active;
    float rest[3][3];
    float output_handoff_rest[3][3];
    int output_handoff_rest_valid;
    float angle[3][3];
    float velocity[3][3];
    float collision_step_points[4][3];
    float collision_step_angle[3][3];
    DWORD collision_step_tick;
    float collision_step_dt;
    DWORD collision_solve_log_tick;
    int collision_step_valid;
    int collision_step_engine_points;
    unsigned int collision_pose_trace_count;
    body_contact_pose_t collision_pose;
    int collision_pose_valid;
    float collision_contact_direction[3][2];
    float collision_prev_max_penetration;
    float collision_rest_penetration;
    int collision_rest_ticks;
    int collision_rest_valid;
    int collision_rest_grace_ticks;
    int collision_impact_ticks;
    int collision_manifold_contacts;
    int collision_room_contacts;
    int collision_multi_support_grace_ticks;
    float collision_prev_chain_points[4][3];
    int collision_prev_chain_points_ready;
    int room_collision_track_valid[3];
    LONG room_collision_track_generation[3];
    DWORD room_collision_track_tick[3];
    float room_collision_track_world[3][3];
    float collision_prev_collider_points[4][BODY_COLLIDER_NODE_COUNT][3];
    int collision_prev_collider_valid[4][BODY_COLLIDER_NODE_COUNT];
    int collision_prev_collider_ready[4];
} body_chain_person_state_t;

typedef struct tk17_testicle_inertia_gate_state_t {
    void *person_state;
    void *person_inertia;
    int *enable_ptr;
    int *flags_ptr;
    int *field8_ptr;
    int saved_enable;
    int saved_flags;
    int saved_field8;
    int saved_valid;
    int active;
    DWORD log_tick;
    DWORD wait_log_tick;
    DWORD verify_tick;
    DWORD retry_tick;
} tk17_testicle_inertia_gate_state_t;

typedef struct poseeditor_paired_bone_track_state_t {
    void *base[POSEEDIT_PAIRED_BONE_TRACK_MAX];
    void *saved_obj[POSEEDIT_PAIRED_BONE_TRACK_MAX];
    void *saved_track_data[POSEEDIT_PAIRED_BONE_TRACK_MAX];
    int suppressed[POSEEDIT_PAIRED_BONE_TRACK_MAX];
    int logged;
    DWORD wait_log_tick;
} poseeditor_paired_bone_track_state_t;

typedef struct poseeditor_paired_bone_track_def_t {
    int track_id;
    const char *node_suffix;
    const char *label;
    int preserve_target;
} poseeditor_paired_bone_track_def_t;

typedef struct breasts_physics_person_state_t {
    body_chain_person_state_t motion;
    /* Breast movement follows spine_joint04, but orientation gravity keeps
       the character root as its canonical frame.  Keeping an independent
       probe state prevents the spine's rotated local basis from swapping
       the established prone/supine and left/right gravity channels. */
    body_chain_person_state_t gravity_motion;
    gravity_sample_t spine_gravity_sample;
    void *source_joint_raw[2];
    void *animation_joint_raw[2];
    void *translation_parent_joint_raw[2];
    float source_handoff[2][3];
    float source_translation_handoff[2][3];
    float rest_rotation[2][3];
    float rotation[2][3];
    float angular_velocity[2][3];
    float bone_translation[2][3];
    float bone_translation_velocity[2][3];
    int contact_translation_active;
    float contact_previous_world[2][3];
    DWORD contact_previous_tick[2], contact_log_tick;
    LONG contact_previous_generation[2];
    void *contact_previous_parent[2];
    float gravity_reference[3];
    float gravity_relative[3];
    float animation_rows[2][9];
    DWORD last_tick;
    DWORD resolve_retry_tick;
    DWORD ownership_candidate_tick;
    DWORD cache_verify_tick;
    DWORD log_tick;
    DWORD live_ownership_simulation_serial;
    LONG live_ownership_node_generation;
    unsigned int ownership_candidate_samples;
    void *ownership_candidate_root_raw;
    void *ownership_candidate_source_raw[2];
    void *ownership_candidate_animation_raw[2];
    void *ownership_candidate_translation_parent_raw[2];
    int animation_rows_valid;
    int gravity_reference_valid;
    int initialized;
    int output_applied;
    int bone_translation_applied;
    int active_logged;
    poseeditor_paired_bone_track_state_t pose_tracks;
} breasts_physics_person_state_t;

/* Runtime ownership is deliberately separate from body_chain_person_state_t.
   PoseEditor keeps using its established state and track path unchanged. */
typedef struct runtime_body_chain_ownership_state_t {
    void *source_joint_raw[3];
    void *animation_joint_raw[3];
    void *pose_editpose;
    unsigned int mapping_samples;
    DWORD mapping_first_tick;
    DWORD mapping_last_tick;
    int mapping_ready;
    int pose_valid;
    int ownership_active;
    int ownership_logged;
    float rotation[3][3];
    float animation_rotation[3][3];
    float native_output_animation_rotation[3][3];
    float native_source_axis_animation_rotation[3][3];
    float animation_rows[3][9];
    int animation_rows_valid;
    unsigned int animation_value_valid_mask;
    unsigned int native_output_animation_valid_mask;
    unsigned int native_source_axis_animation_valid_mask;
    int runtime_writer_logged;
    int native_output_writer_logged;
    int native_source_axis_writer_logged;
} runtime_body_chain_ownership_state_t;

/* Preserve one authored body-axis baseline per person and mode. Penis and
   testicle solvers share it, while PoseEditor and runtime modes remain
   isolated from one another. */
typedef struct body_chain_axis_reference_cache_t {
    void *root_raw;
    void *trs_raw;
    float basis[9];
    int valid;
    LONG node_generation;
    DWORD next_validation_tick;
} body_chain_axis_reference_cache_t;

typedef struct body_chain_contact_t {
    int segment;
    float segment_t;
    float penetration;
    float target_sep;
    float chain[3];
    float body[3];
    float normal[3];
} body_chain_contact_t;

static void body_chain_store_contact(body_chain_contact_t *contacts,
                                     int *contact_count,
                                     int max_contacts,
                                     int segment,
                                     float segment_t,
                                     float penetration,
                                     const float chain[3],
                                     const float body[3],
                                     const float normal[3])
{
    int slot = -1;
    int i;
    int segment_contact_count = 0;
    int weakest_segment_slot = -1;
    const float normal_merge_dot = 0.995f;
    if (!contacts || !contact_count || max_contacts <= 0 ||
        !chain || !body || !normal || penetration <= 0.0f ||
        segment < 0 || segment >= 3) {
        return;
    }

    /* Build a compact contact manifold. Adjacent body capsules overlap by
       design; treating every primitive hit as a separate constraint makes
       those overlaps fight over the same chain segment at rest. */
    for (i = 0; i < *contact_count; i++) {
        float normal_dot;
        if (contacts[i].segment != segment) continue;
        segment_contact_count++;
        if (weakest_segment_slot < 0 ||
            contacts[i].penetration <
                contacts[weakest_segment_slot].penetration) {
            weakest_segment_slot = i;
        }
        normal_dot =
            contacts[i].normal[0] * normal[0] +
            contacts[i].normal[1] * normal[1] +
            contacts[i].normal[2] * normal[2];
        if (normal_dot >= normal_merge_dot &&
            fabsf(contacts[i].segment_t - segment_t) < 0.025f) {
            if (penetration <= contacts[i].penetration) return;
            slot = i;
            break;
        }
    }

    /* Retain different lever arms and crease supports; only near-identical
       constraints above are redundant. Bound storage per segment. */
    if (slot < 0 && segment_contact_count >= 8) {
        if (weakest_segment_slot < 0 ||
            penetration <= contacts[weakest_segment_slot].penetration) {
            return;
        }
        slot = weakest_segment_slot;
    }

    if (slot < 0 && *contact_count < max_contacts) {
        slot = *contact_count;
        (*contact_count)++;
    } else if (slot < 0) {
        int weakest = 0;
        for (i = 1; i < max_contacts; i++) {
            if (contacts[i].penetration < contacts[weakest].penetration) {
                weakest = i;
            }
        }
        if (penetration <= contacts[weakest].penetration) return;
        slot = weakest;
    }
    contacts[slot].segment = segment;
    contacts[slot].segment_t = physx_clampf(segment_t, 0.0f, 1.0f);
    contacts[slot].penetration = penetration;
    contacts[slot].target_sep =
        (chain[0] - body[0]) * normal[0] +
        (chain[1] - body[1]) * normal[1] +
        (chain[2] - body[2]) * normal[2] +
        penetration;
    for (i = 0; i < 3; i++) {
        contacts[slot].chain[i] = chain[i];
        contacts[slot].body[i] = body[i];
        contacts[slot].normal[i] = normal[i];
    }
}

static void reset_body_chain_gravity_basis_gate(body_chain_person_state_t *state)
{
    if (!state) return;
    state->gravity_basis_stable_tick = 0;
    state->gravity_basis_sampled = 0;
    state->gravity_basis_prev[0] = 0.0f;
    state->gravity_basis_prev[1] = 0.0f;
    state->gravity_basis_prev[2] = 0.0f;
    state->gravity_drive_filtered[0] = 0.0f;
    state->gravity_drive_filtered[1] = 0.0f;
    state->gravity_drive_filtered[2] = 0.0f;
    state->gravity_drive_filtered_valid = 0;
}

static void reset_body_chain_gravity_state(body_chain_person_state_t *state)
{
    if (!state) return;
    memset(state->gravity_probe_root, 0, sizeof(state->gravity_probe_root));
    memset(state->gravity_probe_root_world, 0, sizeof(state->gravity_probe_root_world));
    memset(state->gravity_probe_prev_root_world, 0, sizeof(state->gravity_probe_prev_root_world));
    memset(state->gravity_probe_last_root_world, 0, sizeof(state->gravity_probe_last_root_world));
    memset(state->gravity_drive, 0, sizeof(state->gravity_drive));
    memset(state->gravity_drive_ref, 0, sizeof(state->gravity_drive_ref));
    state->gravity_drive_ref_valid = 0;
    state->gravity_camera_hold_active = 0;
    state->gravity_probe_world_valid = 0;
    reset_body_chain_gravity_basis_gate(state);
    memset(&state->gravity_sample, 0, sizeof(state->gravity_sample));
    memset(&state->geometry_sample, 0, sizeof(state->geometry_sample));
}

typedef struct body_chain_gravity_snapshot_t {
    int valid;
    void *root_raw;
    int gravity_probe_captured;
    int gravity_probe_promoted;
    int gravity_probe_sampled;
    float gravity_probe_root[3];
    float gravity_probe_root_world[3];
    int gravity_probe_world_valid;
    float gravity_drive[3];
    float gravity_drive_filtered[3];
    int gravity_drive_filtered_valid;
    float gravity_drive_ref[3];
    int gravity_drive_ref_valid;
    DWORD gravity_basis_stable_tick;
    int gravity_basis_sampled;
    float gravity_basis_prev[3];
} body_chain_gravity_snapshot_t;

typedef struct physx_target_t {
    char name[128];
    int addon_simulated_target;
    int object_transform_target;
    int object_scene_translation_valid;
    int object_scene_rotation_valid;
    int object_scene_limits_valid;
    float object_scene_translation[3];
    float object_scene_rotation[3];
    float object_scene_rotation_min[3];
    float object_scene_rotation_max[3];
    float object_proxy_rest[3];
    float object_proxy_translation[3];
    float object_output_rotation[3];
    int object_output_applied;
    int object_output_logged;
    int constraint_suppression_logged;
    int animation_suppression_seen;
    int animation_suppression_logged;
    void *raw_object;
    void *object;
    void *s_raw_object;
    void *s_object;
    void *s_translation_base;
    void *s_rotation_base;
    void *addon_rotation_base;
    const char *s_translation_source;
    const char *s_rotation_source;
    const char *addon_rotation_source;
    int s_translation_probe_logged;
    int s_translation_offset;
    int s_rotation_probe_logged;
    int s_rotation_offset;
    int addon_rotation_offset;
    int s_rotation_min_offset;
    int s_rotation_max_offset;
    int found_logged;
    int s_found_logged;
    int s_missing_logged;
    int missing_logged;
    int nil_logged;
    int variants_logged;
    int script_variants_logged;
    int tree_search_logged;
    int tree_search_named_roots_logged;
    int addon_object_name_fallback;
    int addon_skin_bound;
    DWORD addon_skin_bound_tick;
    int addon_resolve_miss_count;
    int addon_resolve_backoff_logged;
    DWORD addon_resolve_retry_tick;
    int sim_initialized;
    int sim_started_logged;
    DWORD sim_motion_probe_tick;
    DWORD sim_rotation_probe_tick;
    DWORD sim_start_tick;
    int sim_world_anchor_initialized;
    float sim_rest[3];
    float sim_parent_rest[3];
    float sim_world_parent_rest[3];
    float sim_rotation_rest[3];
    int sim_addon_rotation_rest_valid;
    float sim_addon_rotation_rest[3];
    int addon_joint_orientation_valid;
    float addon_joint_orientation[3];
    float sim_offset[3];
    float sim_velocity[3];
    int sim_output_velocity_initialized;
    int sim_contact_corrected;
    DWORD contact_basis_tick;
    DWORD contact_pivot_tick;
    float contact_pivot_body[3];
    DWORD contact_terminal_tick;
    DWORD contact_terminal_log_tick;
    float contact_terminal_body[3];
    float contact_live_body_basis[9];
    float contact_parent_body_basis[9];
    int contact_trace_valid;
    DWORD contact_trace_tick;
    DWORD contact_trace_log_tick;
    float contact_trace_pivot[3];
    float contact_trace_solver[3];
    float contact_trace_rotation[3];
    DWORD contact_response_tick;
    DWORD contact_response_log_tick;
    LONG contact_response_generation;
    float contact_response_pivot[3];
    float contact_response_request[3];
    float contact_response_margin;
    char contact_response_feature[96];
    float sim_output_offset_prev[3];
    float sim_output_velocity[3];
    int addon_gravity_diag_initialized;
    int addon_gravity_diag_samples;
    float addon_gravity_diag_start_offset[3];
    float addon_gravity_diag_max_delta;
    float addon_gravity_diag_max_bend_len;
    float addon_gravity_diag_last_delta[3];
    float addon_gravity_diag_last_bend[3];
    float addon_gravity_diag_last_rotation[3];
    float sim_length;
    int joint_settings_initialized;
    float joint_gain;
    float limit_angle;
    float joint_min_angle[3];
    float joint_max_angle[3];
    int gravity_settings_initialized;
    int gravity_horizontal_source_axis;
    float gravity_horizontal_source_sign;
    int gravity_horizontal_tail_axis;
    int gravity_horizontal_tail_axis_explicit;
    float gravity_horizontal_scale;
    int gravity_vertical_source_axis;
    float gravity_vertical_source_sign;
    int gravity_vertical_tail_axis;
    int gravity_vertical_tail_axis_explicit;
    float gravity_vertical_scale;
    int gravity_inverted_configured;
    float gravity_inverted_strength;
    int gravity_inverted_tail_axis;
    float gravity_inverted_sign;
    DWORD collision_response_log_tick;
    DWORD room_collision_response_log_tick;
    DWORD room_collision_terminal_response_log_tick;
    int room_collision_track_valid;
    LONG room_collision_track_generation;
    DWORD room_collision_track_tick;
    float room_collision_track_world[3];
    int room_collision_contact_valid;
    DWORD room_collision_contact_tick;
    int room_collision_rest_frames;
    float room_collision_contact_direction[3];
    int room_collision_world_contact_valid;
    DWORD room_collision_world_contact_tick;
    float room_collision_world_contact_direction[3];
    int room_collision_world_contact_mesh_index;
    int room_collision_terminal_world_contact_valid;
    DWORD room_collision_terminal_world_contact_tick;
    float room_collision_terminal_world_contact_direction[3];
    int room_collision_terminal_world_contact_mesh_index;
    int room_collision_terminal_track_valid;
    LONG room_collision_terminal_track_generation;
    DWORD room_collision_terminal_track_tick;
    float room_collision_terminal_track_world[3];
    int room_collision_terminal_axis_valid;
    LONG room_collision_terminal_axis_generation;
    int room_collision_terminal_axis_base;
    int room_collision_terminal_axis_index;
    float room_collision_terminal_axis_sign;
    int room_collision_rest_pose_valid;
    int room_collision_rest_pose_sleeping;
    int room_collision_rest_pose_frames;
    LONG room_collision_rest_pose_generation;
    DWORD room_collision_rest_pose_tick;
    float room_collision_rest_pose_normal[3];
    float room_collision_rest_pose_offset[3];
    int addon_collision_correction_valid;
    DWORD addon_collision_correction_tick;
    float addon_collision_correction_prev[3];
    int addon_collision_response_stable;
    DWORD addon_collision_direct_contact_tick;
    float addon_collision_direct_correction[3];
    int addon_visual_pose_valid;
    int addon_visual_write_translation;
    DWORD addon_visual_tick;
    float addon_visual_rotation[3];
    float addon_visual_translation[3];
    void *addon_write_guard_object;
    void *addon_write_guard_raw_object;
    void *addon_write_guard_s_object;
    void *addon_write_guard_s_rotation_base;
    void *addon_write_guard_s_translation_base;
    int addon_write_guard_s_rotation_offset;
    int addon_write_guard_s_translation_offset;
    int addon_write_guard_samples;
    DWORD addon_write_guard_first_tick;
    DWORD addon_write_guard_last_sample_tick;
    int addon_write_guard_ready;
    DWORD addon_layout_retry_last_tick;
    int addon_layout_retry_pending;
    void *addon_validation_parent_ptr;
    void *addon_validation_target_ptr;
    LONG addon_validation_generation;
    DWORD addon_validation_tick;
    int addon_validation_ready;
} physx_target_t;

typedef struct physx_chain_t {
    char name[128];
    char type[64];
    int addon_chain;
    int object_transform_chain;
    int rigid_simulation;
    int inertial_simulation;
    char addon_owner_person[16];
    char attach_name[128];
    char anchor_name[128];
    char parent_name[128];
    char parent_source[64];
    char drive_name[128];
    int drive_offset_override;
    int drive_auto_initialized;
    int drive_auto_logged;
    int drive_auto_offset;
    float drive_auto_prev[24][3];
    void *anchor_raw_object;
    void *anchor_object;
    float *anchor_vector;
    int anchor_offset;
    int anchor_initialized;
    int anchor_logged;
    int attach_logged;
    int anchor_alias_logged;
    int inertial_local_drive_logged;
    float anchor_rest[3];
    void *addon_parent_rotation_base;
    int addon_parent_rotation_offset;
    int addon_parent_rotation_initialized;
    int addon_parent_rotation_logged;
    DWORD addon_parent_rotation_log_tick;
    float addon_parent_rotation_prev[3];
    void *addon_parent_rotation_auto_base;
    int addon_parent_rotation_auto_logged;
    DWORD addon_parent_rotation_auto_log_tick;
    char addon_body_root_person[16];
    void *addon_body_root_raw;
    int addon_body_root_initialized;
    float addon_body_root_prev[3];
    DWORD addon_body_root_log_tick;
    DWORD addon_body_root_miss_log_tick;
    void *addon_parent_runtime_raw;
    char addon_parent_runtime_name[256];
    char addon_parent_runtime_parent[128];
    char addon_parent_runtime_owner[16];
    DWORD addon_parent_runtime_resolve_tick;
    DWORD addon_parent_runtime_log_tick;
    DWORD addon_parent_runtime_miss_log_tick;
    void *addon_parent_camera_relative_trs_raw;
    void *addon_parent_camera_relative_parent_raw;
    int addon_parent_camera_relative_initialized;
    float addon_parent_camera_relative_prev[9];
    DWORD addon_parent_camera_relative_log_tick;
    DWORD addon_parent_camera_relative_miss_log_tick;
    int addon_parent_translation_camera_relative;
    int addon_parent_rotation_camera_relative;
    int addon_parent_rotation_camera_relative_available;
    void *addon_parent_camera_relative_translation_raw;
    int addon_parent_camera_relative_translation_offset;
    int addon_parent_camera_relative_translation_mode;
    int addon_parent_camera_relative_translation_initialized;
    float addon_parent_camera_relative_translation_prev[3];
    int addon_parent_camera_relative_translation_basis_initialized;
    float addon_parent_camera_relative_translation_parent_rest[9];
    float addon_parent_camera_relative_translation_model_rest[9];
    int addon_parent_camera_relative_translation_global_initialized;
    float addon_parent_camera_relative_translation_global_prev[3];
    int addon_parent_camera_relative_translation_global_pending_valid;
    float addon_parent_camera_relative_translation_global_pending[3];
    LONG addon_parent_camera_relative_translation_global_pending_version;
    int addon_parent_camera_relative_translation_camera_hold_active;
    DWORD addon_parent_camera_relative_translation_log_tick;
    int addon_parent_camera_relative_rotation_pending_valid;
    float addon_parent_camera_relative_rotation_pending[3];
    LONG addon_parent_camera_relative_rotation_pending_version;
    LONG addon_root_drive_camera_seen_version;
    DWORD addon_root_drive_camera_quarantine_tick;
    DWORD addon_root_drive_camera_last_untrusted_tick;
    DWORD addon_root_drive_camera_log_tick;
    float addon_gravity_trusted_drive[3];
    int addon_gravity_trusted_valid;
    gravity_sample_t addon_gravity_sample;
    int addon_gravity_camera_hold_active;
    int addon_gravity_camera_release_active;
    DWORD addon_gravity_camera_log_tick;
    int addon_stationary_camera_hold_active;
    DWORD addon_stationary_parent_motion_tick;
    int addon_gravity_diag_state;
    DWORD addon_gravity_diag_start_tick;
    DWORD addon_gravity_diag_status_tick;
    int addon_gravity_diag_sample_count;
    int addon_gravity_diag_done_logged;
    int addon_gravity_diag_any_valid;
    float addon_gravity_diag_drive[3];
    physx_target_t targets[32];
    int target_count;
    float stiffness;
    float damping;
    float gravity[3];
    float limit_angle;
    int simulate;
    int gravity_enabled;
    int wind_enabled;
    float wind_scale;
    /* Negative values inherit the active room [wind] defaults. */
    float wind_sway_strength;
    float wind_sway_frequency;
    int collision_enabled;
    unsigned int collision_scope;
    int collision_custom_enabled;
    int collision_custom_all_persons;
    int collision_custom_target_count;
    unsigned char collision_custom_target_mask[BODY_COLLIDER_NODE_COUNT];
    float collision_radius;
    float collision_terminal_scale;
    int collision_debug_draw;
    DWORD collision_debug_color;
    DWORD collision_response_log_tick;
    int addon_collision_prev_ready;
    int addon_collision_prev_valid[32];
    float addon_collision_prev_points[32][3];
    DWORD addon_collision_prev_tick;
    int addon_room_contact_normal_valid;
    LONG addon_room_contact_normal_generation;
    DWORD addon_room_contact_normal_tick;
    float addon_room_contact_normal[3];
    int addon_room_rest_sleeping;
    int addon_room_rest_frames;
    LONG addon_room_rest_generation;
    DWORD addon_room_rest_tick;
    float drive_scale;
    float drive_strength;
    int translation_horizontal_source_axis;
    float translation_horizontal_source_sign;
    int translation_horizontal_tail_axis;
    float translation_horizontal_scale;
    int translation_vertical_source_axis;
    float translation_vertical_source_sign;
    int translation_vertical_tail_axis;
    float translation_vertical_scale;
    int translation_depth_source_axis;
    float translation_depth_source_sign;
    int translation_depth_tail_axis;
    float translation_depth_scale;
    int rotation_drive_horizontal_source_axis;
    int rotation_drive_horizontal_tail_axis;
    float rotation_drive_horizontal_scale;
    int rotation_drive_vertical_source_axis;
    int rotation_drive_vertical_tail_axis;
    float rotation_drive_vertical_scale;
    int rotation_drive_twist_source_axis;
    int rotation_drive_twist_tail_axis;
    float rotation_drive_twist_scale;
    int rotation_solver_full_angle;
    int gravity_horizontal_source_axis;
    float gravity_horizontal_source_sign;
    int gravity_horizontal_tail_axis;
    int gravity_horizontal_tail_axis_explicit;
    float gravity_horizontal_scale;
    int gravity_vertical_source_axis;
    float gravity_vertical_source_sign;
    int gravity_vertical_tail_axis;
    int gravity_vertical_tail_axis_explicit;
    float gravity_vertical_scale;
    int gravity_inverted_configured;
    float gravity_inverted_strength;
    int gravity_inverted_tail_axis;
    float gravity_inverted_sign;
    float joint_gain;
    float joint_min_angle[3];
    float joint_max_angle[3];
    int skinned_matrix_enabled;
    int skinned_matrix_translation_enabled;
    float skinned_matrix_scale;
    float root_bend_scale;
    float gravity_scale;
    float addon_wind_trusted_drive[3];
    int addon_wind_trusted_valid;
    LONG addon_wind_camera_seen_version;
    DWORD addon_wind_camera_quarantine_tick;
    DWORD addon_wind_update_tick;
    unsigned int addon_wind_generation;
    int addon_wind_logged;
    float max_offset;
    float startup_impulse;
    DWORD binding_probe_tick;
    int binding_probe_count;
    DWORD skin_probe_tick;
    int skin_probe_count;
    int skin_probe_found;
    DWORD skin_probe_found_tick;
    int skin_memory_scan_armed_logged;
    int addon_root_window_serial;
    DWORD addon_root_seen_tick;
    DWORD addon_root_settle_until_tick;
    int addon_live_layout_pending;
    DWORD addon_live_layout_retry_tick;
    int addon_scene_visible;
    DWORD addon_test_start_tick;
    DWORD addon_test_status_tick;
    int addon_test_done_logged;
    DWORD addon_output_room_ready_tick;
    DWORD addon_output_start_tick;
    DWORD addon_output_status_tick;
    int addon_output_room_ready_logged;
    int addon_output_started_logged;
    int addon_output_done_logged;
} physx_chain_t;

typedef struct physx_sidecar_t {
    char path[MAX_PATH * 4];
    FILETIME write_time;
    physx_chain_t chains[64];
    int chain_count;
    int enabled;
    int write_test;
    char write_test_target[128];
    int write_test_axis;
    float write_test_amount;
    int write_test_duration_ms;
    int write_test_state;
    DWORD write_test_start_tick;
    float write_test_original[3];
    int loaded;
    int room_scene_sidecar;
    int root_logged;
    int root_import_attempted;
    void *root_raw_object;
    void *root_object;
    void *script_root_raw_object;
    void *script_root_object;
    int addon_registered_logged;
    int addon_scene_active;
    DWORD addon_scene_active_tick;
    char addon_owner_person[16];
    DWORD addon_owner_seen_tick;
    int addon_owner_logged;
    LONG addon_live_root_cache_generation[4];
    DWORD addon_live_root_cache_tick[4];
    unsigned char addon_live_root_cache_valid[4];
    unsigned char addon_live_root_cache_value[4];
    DWORD addon_live_root_fallback_scene_tick[4];
    char addon_id_cache[256];
    void *addon_equipment_definition_cache;
} physx_sidecar_t;

typedef struct named_node_t {
    char name[128];
    void *object;
    int logged;
    DWORD first_seen_tick;
} named_node_t;

typedef struct motion_probe_t {
    char name[128];
    char source[16];
    void *base;
    int initialized;
    DWORD last_log_tick;
    DWORD last_quiet_log_tick;
    float baseline[256];
} motion_probe_t;

typedef struct body_chain_probe_t {
    const char *name;
    char seen_name[256];
    void *hook_object;
    void *runtime_raw;
    void *runtime_object;
    int seen_logged;
    int resolved_logged;
    int missing_logged;
} body_chain_probe_t;

typedef struct transform_probe_sample_t {
    char name[256];
    char source[16];
    void *base;
    int initialized;
    DWORD last_log_tick;
    float baseline[512];
} transform_probe_sample_t;

typedef struct axis_map_probe_sample_t {
    const char *label;
    const char *suffix;
    int offset;
    void *base;
    int initialized;
    float baseline[3];
    float previous[3];
} axis_map_probe_sample_t;

typedef struct axis_root_scan_t {
    void *base;
    int initialized;
    int last_best_off;
    float last_best_step;
    float previous[512];
} axis_root_scan_t;

static HINSTANCE self_module;
static CRITICAL_SECTION log_lock;
static int log_ready;
static int config_loaded;
static FILETIME config_write_time;
static int config_reload_pending;
static FILETIME config_reload_pending_write_time;
static DWORD config_reload_pending_tick;
static DWORD config_reload_pending_log_tick;
static DWORD config_hot_reload_tick;
static DWORD body_chain_physics_settings_change_tick[4];
static int body_chain_physics_settings_change_enabled[4];
static DWORD testicle_physics_settings_change_tick[4];
static int testicle_physics_settings_change_enabled[4];
static DWORD body_chain_collision_settings_change_tick[4];
static int body_chain_collision_settings_change_enabled[4];
static char config_path[MAX_PATH * 4];
static physx_defaults_t defaults_cfg = {
    0.65f, 0.25f, { 0.0f, -1.0f, 0.0f }, 35.0f, 0, 0, 250
};
static physx_perf_state_t physx_perf_state;
static int addon_physics_enabled = 0;
static int addon_physics_probe_enabled = 0;
static int addon_sidecar_hot_reload_enabled = 1;
static int addon_sidecar_hot_reload_interval_ms = 1000;
static int addon_sidecar_hot_reload_max_checks_per_tick = 8;
static volatile LONG addon_traverse_overlay_active;
static volatile LONG body_chain_runtime_ownership_active;
static volatile LONG body_chain_traverse_overlay_active;
static volatile LONG body_chain_runtime_penis_native_filter_active;
static volatile LONG body_chain_poseeditor_mode_active;
static volatile LONG body_chain_runtime_mode_transition_pending;
static DWORD body_chain_poseeditor_mode_probe_tick;
static DWORD body_chain_poseeditor_mode_probe_unavailable_log_tick;
static DWORD body_chain_poseeditor_exit_candidate_tick;
static int body_chain_poseeditor_mode_probe_ready;
static body_probe_config_t body_probe_cfg = { 0, "Person02", "Spenis_joint01", 0, 0x06c, 0, 2.0f, 600, 0, 0, 0, 0, NULL, "", {0,0,0} };
static transform_probe_config_t transform_probe_cfg = { 0, "Person02", 1000, 512, 6, 1, 0.005f, 0 };
static axis_map_probe_config_t axis_map_probe_cfg = { 0, "Person02", 500, 0.0005f, 0.025f, 7, 5000, 0, -1, 0 };
static root_drive_probe_config_t root_drive_probe_cfg = {
    0, "Person02", "root", "Spenis_joint03",
    0x0e8, 2, 0x06c, 1,
    1.0f, 0.12f, 0.0005f, 16,
    0, NULL, NULL, 0, 0,
    {0,0,0}, {0,0,0}, {0,0,0}
};
static write_sweep_probe_config_t write_sweep_probe_cfg = {
    0, "Person02", 1200, 5000, 0, 0, 0, 0, 0, NULL, {0,0,0}
};
static collision_auto_test_config_t collision_auto_test_cfg = {
    .enabled = 0,
    .person = "Person02",
    .mode = "testicles",
    .start_delay_ms = 5000,
    .phase_ms = 1800,
    .rest_ms = 900,
    .root_amount = 90.0f,
    .testicle_amount = 35.0f,
    .hip_amount = 35.0f,
    .active_offset = -1,
    .active_axis = -1
};
static camera_contamination_test_config_t camera_contamination_test_cfg = {
    .enabled = 0,
    .person = "Person02",
    .start_delay_ms = 15000,
    .hold_ms = 2500,
    .sample_ms = 100,
    .input_drag_ms = 650,
    .input_yaw_pixels = 420,
    .input_pitch_pixels = 260,
    .auto_focus = 1,
    .phase = -1
};
static const camera_contamination_test_step_t
camera_contamination_test_steps[CAMERA_CONTAMINATION_TEST_STEP_COUNT] = {
    {   0.0f,   0.0f,    0,    0, "baseline-A1" },
    {  35.0f,   0.0f,  420,    0, "yaw-plus-35" },
    {   0.0f,   0.0f, -420,    0, "baseline-A2" },
    { -35.0f,   0.0f, -420,    0, "yaw-minus-35" },
    {   0.0f,   0.0f,  420,    0, "baseline-A3" },
    {   0.0f,  22.0f,    0, -260, "pitch-plus-22" },
    {   0.0f,   0.0f,    0,  260, "baseline-A4" },
    {   0.0f, -22.0f,    0,  260, "pitch-minus-22" },
    {   0.0f,   0.0f,    0, -260, "baseline-A5" },
    {  28.0f,  16.0f,  336, -189, "yaw-plus-28-pitch-plus-16" },
    {   0.0f,   0.0f, -336,  189, "baseline-A6" }
};
static physics_environment_config_t physics_environment_cfg = {
    .world_gravity_probe = 0,
    .wind_enabled = 1,
    .world_gravity = { 0.0f, -1.0f, 0.0f },
    .gravity_horizontal_source_vector = { 0.0f, 0.0f, 0.0f },
    .gravity_vertical_source_vector = { 0.0f, -1.0f, 0.0f },
    .gravity_apply_to_body_chain = 0,
    .gravity_body_chain_scale = 0.05f,
    .gravity_horizontal_body_chain_scale = 0.05f,
    .gravity_vertical_body_chain_scale = 0.05f,
    .gravity_horizontal_secondary_body_chain_scale = 0.0f,
    .gravity_vertical_secondary_body_chain_scale = 0.0f,
    .gravity_horizontal_tail_axis = 1,
    .gravity_vertical_tail_axis = 2,
    .body_chain_camera_relative_orientation = 0,
    .body_chain_camera_coast_stiffness_scale = 0.35f,
    .body_chain_camera_coast_damping_scale = 0.15f,
    .body_chain_camera_quarantine_ms = 450,
    .gravity_dynamic_body_basis = 1,
    .gravity_basis_camera_compensate = 1,
    .gravity_basis_node = "root",
    .gravity_horizontal_basis_offset = 0x088,
    .gravity_vertical_basis_offset = 0x098,
    .gravity_horizontal_secondary_basis_offset = 0x078,
    .gravity_horizontal_basis_sign = 1.0f,
    .gravity_vertical_basis_sign = 1.0f,
    .gravity_horizontal_secondary_basis_sign = 1.0f,
    .gravity_zero_at_start = 0,
    .gravity_response_ms = 250.0f,
    .gravity_max_degrees_per_second = 240.0f,
    .gravity_probe_settle_ms = 1500,
    .gravity_probe_confirm_ms = 1000,
    .gravity_probe_camera_quiet_ms = 3500,
    .gravity_probe_log_ms = 2000,
    .gravity_probe_motion_epsilon = 0.005f,
    .gravity_probe_invalidate_epsilon = 0.10f,
    .gravity_probe_require_nonzero_root = 1
};
static body_chain_physics_config_t body_chain_physics_global_cfg = {
    .enabled = 0,
    .wind_enabled = 1,
    .wind_scale = 1.0f,
    .wind_source_axis = { 0, 1, 2 },
    .wind_tail_axis = { 2, 0, 1 },
    .wind_axis_scale = { -1.0f, -1.0f, 0.0f },
    .collision_scope = BODY_CHAIN_COLLISION_SCOPE_FULL_BODY_ALL,
    .root_offset = 0x0e8,
    .output_offset = 0x06c,
    .animation_output_offset = 0x06c,
    .animation_override_offsets = { 0x06c, 0x078, 0x088, 0x098, -1, -1, -1, -1 },
    .animation_override_offset_count = 4,
    .override_animation = 1,
    .joint01_pose_override = 1,
    .joint01_pose_override_offsets = { 0x014, 0x018, 0x024, -1, -1, -1, -1, -1 },
    .joint01_pose_override_offset_count = 3,
    .joint01_transform_lock = 1,
    .joint01_transform_lock_offset = 0x078,
    .poseeditor_track_override = 0,
    .poseeditor_track_diagnostic = 0,
    .poseeditor_total_tracks = POSEEDIT_FALLBACK_TOTAL_TRACKS,
    .translation_source_axis = { 0, 1, 2 },
    .translation_tail_axis = { 2, 1, 0 },
    .translation_scale = { 0.10f, 0.10f, -0.10f },
    .face_down_translation_channel = 1,
    .face_down_translation_sign = -1.0f,
    .rotation_source_axis = { 0, 1, 2 },
    .rotation_tail_axis = { 2, 1, 0 },
    .rotation_scale = { -0.25f, 0.25f, 0.25f },
    .translation_deadzone = 0.001f,
    .rotation_deadzone = 0.001f,
    .horizontal_source_axis = 0,
    .vertical_source_axis = 2,
    .horizontal_source_vector = { 1.0f, 0.0f, 0.0f },
    .vertical_source_vector = { 0.0f, 0.0f, -1.0f },
    .horizontal_output_axis = 2,
    .vertical_output_axis = 1,
    .horizontal_secondary_output_axis = -1,
    .horizontal_secondary_output_scale = 0.0f,
    .horizontal_sign = -1.0f,
    .vertical_sign = 1.0f,
    .drive_scale = 850.0f,
    .horizontal_drive_scale = 300.0f,
    .vertical_drive_scale = 300.0f,
    .horizontal_deadzone = 0.001f,
    .vertical_deadzone = 0.001f,
    .gravity_angle = 0.0f,
    .gravity_horizontal_curve = 1.0f,
    .gravity_vertical_curve = 1.0f,
    .gravity_inverted_strength = 30.0f,
    .gravity_inverted_tail_axis = 2,
    .gravity_inverted_sign = -1.0f,
    .chain_total_bend_max = 120.0f,
    .chain_total_twist_max = 90.0f,
    .stiffness = 60.0f,
    .damping = 2.5f,
    .max_angle = 200.0f,
    .link_max_angle = {
        { 32.0f, 32.0f, 32.0f },
        { 32.0f, 32.0f, 32.0f },
        { 32.0f, 32.0f, 32.0f }
    },
    .link_min_angle = {
        { -32.0f, -32.0f, -32.0f },
        { -32.0f, -32.0f, -32.0f },
        { -32.0f, -32.0f, -32.0f }
    },
    .link_gain = { 0.75f, 1.10f, 1.35f },
    .interval_ms = 16,
    .zero_output_rest = 1,
    .enabled_person = { 0, 1, 0, 0 }
};
static body_chain_physics_config_t testicle_physics_global_cfg = {
    .enabled = 0,
    .wind_enabled = 1,
    .wind_scale = 1.0f,
    .wind_source_axis = { 0, 1, 2 },
    .wind_tail_axis = { 2, 0, 1 },
    .wind_axis_scale = { -1.0f, -1.0f, 0.0f },
    .collision_scope = BODY_CHAIN_COLLISION_SCOPE_FULL_BODY_ALL,
    .root_offset = 0x0e8,
    .output_offset = 0x06c,
    .animation_output_offset = 0x06c,
    .animation_override_offsets = { 0x06c, 0x078, 0x088, 0x098, -1, -1, -1, -1 },
    .animation_override_offset_count = 4,
    .override_animation = 1,
    .joint01_pose_override = 0,
    .joint01_pose_override_offsets = { 0x014, 0x018, 0x024, -1, -1, -1, -1, -1 },
    .joint01_pose_override_offset_count = 3,
    .joint01_transform_lock = 0,
    .joint01_transform_lock_offset = 0x078,
    .poseeditor_track_override = 1,
    .poseeditor_track_diagnostic = 0,
    .poseeditor_total_tracks = POSEEDIT_FALLBACK_TOTAL_TRACKS,
    .translation_source_axis = { 0, 1, 2 },
    .translation_tail_axis = { 2, 1, 0 },
    .translation_scale = { 0.10f, 0.10f, -0.10f },
    .face_down_translation_channel = 1,
    .face_down_translation_sign = -1.0f,
    .rotation_source_axis = { 0, 1, 2 },
    .rotation_tail_axis = { 2, 1, 0 },
    .rotation_scale = { -0.25f, 0.25f, 0.25f },
    .translation_deadzone = 0.001f,
    .rotation_deadzone = 0.001f,
    .horizontal_source_axis = 0,
    .vertical_source_axis = 2,
    .horizontal_source_vector = { 1.0f, 0.0f, 0.0f },
    .vertical_source_vector = { 0.0f, 0.0f, -1.0f },
    .horizontal_output_axis = 2,
    .vertical_output_axis = 1,
    .horizontal_secondary_output_axis = -1,
    .horizontal_secondary_output_scale = 0.0f,
    .horizontal_sign = -1.0f,
    .vertical_sign = 1.0f,
    .drive_scale = 850.0f,
    .horizontal_drive_scale = 300.0f,
    .vertical_drive_scale = 300.0f,
    .horizontal_deadzone = 0.001f,
    .vertical_deadzone = 0.001f,
    .gravity_angle = 0.0f,
    .gravity_horizontal_curve = 1.0f,
    .gravity_vertical_curve = 1.0f,
    .gravity_inverted_strength = 30.0f,
    .gravity_inverted_tail_axis = 2,
    .gravity_inverted_sign = -1.0f,
    .chain_total_bend_max = 120.0f,
    .chain_total_twist_max = 90.0f,
    .stiffness = 100.0f,
    .damping = 8.0f,
    .max_angle = 120.0f,
    .link_max_angle = {
        { 80.0f, 80.0f, 80.0f },
        { 60.0f, 60.0f, 60.0f },
        { 1.0f, 1.0f, 1.0f }
    },
    .link_min_angle = {
        { -80.0f, -80.0f, -80.0f },
        { -60.0f, -60.0f, -60.0f },
        { -1.0f, -1.0f, -1.0f }
    },
    .link_gain = { 1.0f, 0.75f, 0.0f },
    .interval_ms = 16,
    .zero_output_rest = 1,
    .enabled_person = { 0, 1, 0, 0 }
};
static body_chain_physics_config_t breasts_physics_global_cfg = {
    .enabled = 1,
    .wind_enabled = 1,
    .wind_scale = 1.0f,
    .wind_source_axis = { 0, 1, 2 },
    .wind_tail_axis = { 2, 0, 1 },
    .wind_axis_scale = { -1.0f, 1.0f, 0.0f },
    .collision_scope = BODY_CHAIN_COLLISION_SCOPE_FULL_BODY_ALL,
    .root_offset = 0x0e8,
    .output_offset = 0x06c,
    .override_animation = 1,
    .poseeditor_track_override = 1,
    .translation_source_axis = { 0, 1, 2 },
    .translation_tail_axis = { 0, 1, 2 },
    .translation_scale = { 0.25f, 0.25f, 0.25f },
    .rotation_source_axis = { 0, 1, 2 },
    .rotation_tail_axis = { 2, 0, 1 },
    .rotation_scale = { -4.0f, -4.0f, 4.0f },
    .translation_deadzone = 0.001f,
    .rotation_deadzone = 0.001f,
    .gravity_horizontal_curve = 1.0f,
    .gravity_vertical_curve = 1.0f,
    .gravity_angle = 10.0f,
    .gravity_inverted_strength = 10.0f,
    .gravity_inverted_tail_axis = 1,
    .gravity_inverted_sign = 1.0f,
    .stiffness = 100.0f,
    .damping = 8.0f,
    .max_angle = 30.0f,
    .link_max_angle = {
        { 30.0f, 30.0f, 30.0f },
        { 1.0f, 1.0f, 1.0f },
        { 1.0f, 1.0f, 1.0f }
    },
    .link_min_angle = {
        { -30.0f, -30.0f, -30.0f },
        { -1.0f, -1.0f, -1.0f },
        { -1.0f, -1.0f, -1.0f }
    },
    .link_gain = { 1.0f, 0.0f, 0.0f },
    .interval_ms = 16,
    .zero_output_rest = 0,
    .bone_translation_enabled = 1,
    .bone_translation_space = 1,
    .bone_translation_scale = { -0.50f, -0.50f, -0.50f },
    .bone_translation_gravity_sag = 0.0f,
    .bone_translation_stiffness = 120.0f,
    .bone_translation_damping = 12.0f,
    .bone_translation_max_offset = { 0.015f, 0.015f, 0.015f },
    .gravity_inward_strength = 5.0f,
    .gravity_outward_strength = 5.0f,
    .enabled_person = { 1, 1, 1, 1 }
};
/* Channel-order mirror controls: horizontal, vertical, depth/twist. */
static float breasts_physics_translation_sign[2][3] = {
    { 1.0f, 1.0f, 1.0f },
    { -1.0f, 1.0f, -1.0f }
};
static float breasts_physics_rotation_sign[2][3] = {
    { 1.0f, 1.0f, 1.0f },
    { -1.0f, 1.0f, -1.0f }
};
static float breasts_physics_wind_sign[2][3] = {
    { 1.0f, 1.0f, 1.0f },
    { 1.0f, 1.0f, 1.0f }
};
/* Breast gravity uses its own mapping because mirrored local breast axes do
   not share the same per-side signs as movement-driven spring rotation. */
static int breasts_physics_gravity_tail_axis[3] = { 1, 2, 1 };
static float breasts_physics_gravity_sign[2][3] = {
    { 1.0f, 1.0f, 1.0f },
    { 1.0f, -1.0f, 1.0f }
};
static int breasts_physics_gravity_inward_outward_tail_axis = 2;
static float breasts_physics_gravity_inward_outward_sign = -1.0f;
/* Optional physical movement of the Sbreast_scale_* joint itself. This is
   independent from the existing translation-driven rotational swing. */
/* 0 keeps direct joint-local output mapping; 1 treats the mapped target as
   body-relative and converts it into each breast parent's local basis. */
static int breasts_physics_bone_translation_offset = 0x07c;
/* Constant world-gravity displacement as a fraction of the configured bone
   translation limits. It shares the normal translation spring. */
static int breasts_physics_bone_translation_source_axis[3] = { 0, 1, 2 };
static int breasts_physics_bone_translation_tail_axis[3] = { 0, 1, 2 };
static float breasts_physics_bone_translation_sign[2][3] = {
    { 1.0f, 1.0f, 1.0f },
    { 1.0f, 1.0f, 1.0f }
};
/* Butt PhysX is intentionally independent from breast settings. It targets
   only butt_L_joint01 and butt_R_joint01; their child joints inherit the
   resulting parent transform without receiving their own simulation. */
static body_chain_physics_config_t butt_physics_global_cfg = {
    .enabled = 1,
    .wind_enabled = 1,
    .wind_scale = 1.0f,
    .wind_source_axis = { 0, 1, 2 },
    .wind_tail_axis = { 2, 0, 1 },
    .wind_axis_scale = { -1.0f, 1.0f, 0.0f },
    .collision_scope = BODY_CHAIN_COLLISION_SCOPE_FULL_BODY_ALL,
    .root_offset = 0x0e8,
    .output_offset = 0x06c,
    .override_animation = 1,
    .poseeditor_track_override = 1,
    .translation_source_axis = { 0, 1, 2 },
    .translation_tail_axis = { 0, 1, 2 },
    .translation_scale = { 0.10f, -0.05f, 0.10f },
    .rotation_source_axis = { 0, 1, 2 },
    .rotation_tail_axis = { 2, 2, 1 },
    .rotation_scale = { -2.0f, -2.0f, 2.0f },
    .translation_deadzone = 0.001f,
    .rotation_deadzone = 0.001f,
    .gravity_horizontal_curve = 1.0f,
    .gravity_vertical_curve = 1.0f,
    .gravity_angle = 10.0f,
    .gravity_inverted_strength = 10.0f,
    .gravity_inverted_tail_axis = 1,
    .gravity_inverted_sign = 1.0f,
    .stiffness = 150.0f,
    .damping = 5.0f,
    .max_angle = 30.0f,
    .link_max_angle = {
        { 30.0f, 30.0f, 30.0f },
        { 1.0f, 1.0f, 1.0f },
        { 1.0f, 1.0f, 1.0f }
    },
    .link_min_angle = {
        { -30.0f, -30.0f, -30.0f },
        { -1.0f, -1.0f, -1.0f },
        { -1.0f, -1.0f, -1.0f }
    },
    .link_gain = { 1.0f, 0.0f, 0.0f },
    .interval_ms = 16,
    .zero_output_rest = 0,
    .bone_translation_enabled = 1,
    .bone_translation_space = 1,
    .bone_translation_scale = { -0.50f, -0.50f, -0.50f },
    .bone_translation_stiffness = 120.0f,
    .bone_translation_damping = 12.0f,
    .bone_translation_max_offset = { 0.015f, 0.015f, 0.015f },
    .enabled_person = { 1, 1, 1, 1 }
};
static float butt_physics_translation_sign[2][3] = {
    { 1.0f, 1.0f, 1.0f },
    { 1.0f, 1.0f, -1.0f }
};
static float butt_physics_rotation_sign[2][3] = {
    { 1.0f, -1.0f, 1.0f },
    { -1.0f, 1.0f, 1.0f }
};
static float butt_physics_wind_sign[2][3] = {
    { 1.0f, 1.0f, 1.0f },
    { 1.0f, 1.0f, 1.0f }
};
static int butt_physics_gravity_tail_axis[3] = { 1, 2, 1 };
static float butt_physics_gravity_sign[2][3] = {
    { 1.0f, 1.0f, 1.0f },
    { 1.0f, -1.0f, 1.0f }
};
static int butt_physics_bone_translation_offset = 0x07c;
static int butt_physics_bone_translation_source_axis[3] = { 0, 1, 2 };
static int butt_physics_bone_translation_tail_axis[3] = { 0, 1, 2 };
static float butt_physics_bone_translation_sign[2][3] = {
    { 1.0f, 1.0f, 1.0f },
    { 1.0f, 1.0f, 1.0f }
};
static body_chain_collider_config_t body_chain_collider_global_cfg = {
    .enabled = 0,
    .debug_draw = 1,
    .response_enabled = 0,
    .breasts_collision_enabled = 0,
    .butt_collision_enabled = 0,
    .penis_collision_enabled = 0,
    .testicle_collision_enabled = 0,
    .diagnostic = 0,
    .root_local_offsets = 1,
    .live_testicle_bones = 0,
    .testicles_bone_head_mode = 1,
    .testicle_rotation_offset = 0x06c,
    .position_offset = 0x0e8,
    .local_offset = {
        { 0.0f, 0.0f, 0.0f },
        { 0.170f, 0.000f, 0.030f },
        { 0.245f, 0.000f, 0.045f },
        { -0.021f, 0.017f, 0.113f },
        { -0.021f, 0.017f, -0.113f },
        { -0.360f, -0.020f, 0.100f },
        { -0.360f, -0.020f, -0.100f },
        { -0.1905f, -0.0015f, 0.1065f },
        { -0.1905f, -0.0015f, -0.1065f },
        { -0.03579f, -0.02010f, 0.0f },
        { -0.06393f, -0.00453f, 0.0f },
        { -0.04986f, -0.01232f, 0.0f }
    },
    .body_chain_base_offset = { -0.05291f, 0.10569f, 0.0f },
    .testicle_pivot = {
        { -0.03579f, -0.02010f, 0.0f },
        { -0.06393f, -0.00453f, 0.0f }
    },
    .testicle_head = {
        { -0.08870f, 0.08559f, 0.0f },
        { -0.11684f, 0.10116f, 0.0f }
    },
    .testicle_fine_offset = {
        { 0.0f, 0.0f, 0.0f },
        { 0.0f, 0.0f, 0.0f }
    },
    .stomach_fine_offset = {
        { 0.0f, 0.0f, 0.0f },
        { 0.0f, 0.0f, 0.0f }
    },
    .stomach_extra_fine_offset = {
        { 0.0f, 0.0f, 0.0f },
        { 0.0f, 0.0f, 0.0f }
    },
    .hip_fine_offset = { 0.0f, 0.0f, 0.0f },
    .knee_fine_offset = { 0.0f, 0.0f, 0.0f },
    .thigh_fine_offset = { 0.0f, 0.0f, 0.0f },
    .pelvis_radius = 0.14f,
    .stomach_radius = {
        { 0.090f, 0.050f, 0.120f },
        { 0.085f, 0.045f, 0.110f }
    },
    .stomach_extra_radius = {
        { 0.080f, 0.085f, 0.105f },
        { 0.060f, 0.075f, 0.085f }
    },
    .hip_radius = 0.11f,
    .thigh_radius = -1.0f,
    .knee_radius = 0.075f,
    .testicles_radius = 0.075f,
    .ankle_radius = { 0.035f, 0.035f, 0.035f },
    .ball_radius = { 0.030f, 0.030f, 0.030f },
    .breast_radius = { 0.075f, 0.075f, 0.075f },
    .butt_radius = { 0.100f, 0.100f, 0.100f },
    .neck_radius = { 0.045f, 0.055f, 0.045f },
    .head_radius = { 0.090f, 0.105f, 0.090f },
    .clavicle_radius = { 0.035f, 0.035f, 0.035f },
    .shoulder_radius = { 0.055f, 0.055f, 0.055f },
    .elbow_radius = { 0.040f, 0.040f, 0.040f },
    .forearm_radius = { 0.035f, 0.035f, 0.035f },
    .wrist_radius = { 0.030f, 0.030f, 0.030f },
    .palm_radius = { 0.035f, 0.025f, 0.045f },
    .finger_radius = {
        { 0.012f, 0.012f, 0.012f },
        { 0.011f, 0.011f, 0.011f },
        { 0.011f, 0.011f, 0.011f },
        { 0.010f, 0.010f, 0.010f },
        { 0.009f, 0.009f, 0.009f }
    },
    .pelvis_capsule_radius = 0.10f,
    .response_radius_scale = 0.35f,
    .chain_radius = 0.018f,
    .link_length = { 0.070f, 0.060f, 0.055f },
    .response_strength = 1.00f,
    .response_max_degrees_per_tick = 6.0f,
    .collision_iterations = 1,
    .collision_slop = 0.0015f,
    .health_log_ms = 2000,
    .response_log_ms = 1000
};
static body_chain_physics_config_t body_chain_physics_person_cfg[4];
static body_chain_physics_config_t testicle_physics_person_cfg[4];
static body_chain_physics_config_t breasts_physics_person_cfg[4];
static body_chain_physics_config_t butt_physics_person_cfg[4];
static body_chain_collider_config_t body_chain_collider_person_cfg[4];
static body_chain_physics_config_t *body_chain_physics_cfg_ptr =
    &body_chain_physics_global_cfg;
static body_chain_physics_config_t *testicle_physics_cfg_ptr =
    &testicle_physics_global_cfg;
static body_chain_physics_config_t *breasts_physics_cfg_ptr =
    &breasts_physics_global_cfg;
static body_chain_physics_config_t *butt_physics_cfg_ptr =
    &butt_physics_global_cfg;
static body_chain_collider_config_t *body_chain_collider_cfg_ptr =
    &body_chain_collider_global_cfg;

#define body_chain_physics_cfg (*body_chain_physics_cfg_ptr)
#define testicle_physics_cfg (*testicle_physics_cfg_ptr)
#define breasts_physics_cfg (*breasts_physics_cfg_ptr)
#define butt_physics_cfg (*butt_physics_cfg_ptr)
#define body_chain_collider_cfg (*body_chain_collider_cfg_ptr)

#define breasts_physics_bone_translation_enabled \
    (breasts_physics_cfg.bone_translation_enabled)
#define breasts_physics_bone_translation_space \
    (breasts_physics_cfg.bone_translation_space)
#define breasts_physics_bone_translation_scale \
    (breasts_physics_cfg.bone_translation_scale)
#define breasts_physics_bone_translation_gravity_sag \
    (breasts_physics_cfg.bone_translation_gravity_sag)
#define breasts_physics_bone_translation_stiffness \
    (breasts_physics_cfg.bone_translation_stiffness)
#define breasts_physics_bone_translation_damping \
    (breasts_physics_cfg.bone_translation_damping)
#define breasts_physics_bone_translation_max_offset \
    (breasts_physics_cfg.bone_translation_max_offset)
#define breasts_physics_gravity_inward_strength \
    (breasts_physics_cfg.gravity_inward_strength)
#define breasts_physics_gravity_outward_strength \
    (breasts_physics_cfg.gravity_outward_strength)

#define butt_physics_bone_translation_enabled \
    (butt_physics_cfg.bone_translation_enabled)
#define butt_physics_bone_translation_space \
    (butt_physics_cfg.bone_translation_space)
#define butt_physics_bone_translation_scale \
    (butt_physics_cfg.bone_translation_scale)
#define butt_physics_bone_translation_stiffness \
    (butt_physics_cfg.bone_translation_stiffness)
#define butt_physics_bone_translation_damping \
    (butt_physics_cfg.bone_translation_damping)
#define butt_physics_bone_translation_max_offset \
    (butt_physics_cfg.bone_translation_max_offset)

static void body_profile_set_active_person_config(int person_index)
{
    if (person_index >= 0 && person_index < 4) {
        body_chain_physics_cfg_ptr =
            &body_chain_physics_person_cfg[person_index];
        testicle_physics_cfg_ptr =
            &testicle_physics_person_cfg[person_index];
        breasts_physics_cfg_ptr =
            &breasts_physics_person_cfg[person_index];
        butt_physics_cfg_ptr =
            &butt_physics_person_cfg[person_index];
        body_chain_collider_cfg_ptr =
            &body_chain_collider_person_cfg[person_index];
    } else {
        body_chain_physics_cfg_ptr = &body_chain_physics_global_cfg;
        testicle_physics_cfg_ptr = &testicle_physics_global_cfg;
        breasts_physics_cfg_ptr = &breasts_physics_global_cfg;
        butt_physics_cfg_ptr = &butt_physics_global_cfg;
        body_chain_collider_cfg_ptr = &body_chain_collider_global_cfg;
    }
}

static int body_chain_collision_scope_valid(int scope)
{
    return scope >= BODY_CHAIN_COLLISION_SCOPE_GENITALS_ONLY &&
           scope <= BODY_CHAIN_COLLISION_SCOPE_HANDS_ONLY_ALL;
}

static int body_chain_collision_scope_all_persons(int scope)
{
    return body_chain_collision_scope_valid(scope) && (scope & 1) != 0;
}

static int body_chain_collision_scope_collider_mask(int scope)
{
    switch (scope) {
    case BODY_CHAIN_COLLISION_SCOPE_GENITALS_ONLY:
    case BODY_CHAIN_COLLISION_SCOPE_GENITALS_ONLY_ALL:
        return BODY_CHAIN_COLLIDER_GROUP_GENITALS;
    case BODY_CHAIN_COLLISION_SCOPE_PELVIS_AND_GENITALS_ONLY:
    case BODY_CHAIN_COLLISION_SCOPE_PELVIS_AND_GENITALS_ONLY_ALL:
        return BODY_CHAIN_COLLIDER_GROUP_GENITALS |
               BODY_CHAIN_COLLIDER_GROUP_PELVIS;
    case BODY_CHAIN_COLLISION_SCOPE_PELVIS_GENITALS_AND_HANDS_ONLY:
    case BODY_CHAIN_COLLISION_SCOPE_PELVIS_GENITALS_AND_HANDS_ONLY_ALL:
        return BODY_CHAIN_COLLIDER_GROUP_GENITALS |
               BODY_CHAIN_COLLIDER_GROUP_PELVIS |
               BODY_CHAIN_COLLIDER_GROUP_HANDS;
    case BODY_CHAIN_COLLISION_SCOPE_HANDS_ONLY:
    case BODY_CHAIN_COLLISION_SCOPE_HANDS_ONLY_ALL:
        return BODY_CHAIN_COLLIDER_GROUP_HANDS;
    default:
        return BODY_CHAIN_COLLIDER_GROUP_ALL_TARGETS;
    }
}

static int breasts_physics_person_enabled(int person_index)
{
    const body_chain_physics_config_t *cfg;
    if (person_index < 0 || person_index >= 4) return 0;
    cfg = &breasts_physics_person_cfg[person_index];
    return cfg->enabled && cfg->enabled_person[person_index];
}

static int butt_physics_person_enabled(int person_index)
{
    const body_chain_physics_config_t *cfg;
    if (person_index < 0 || person_index >= 4) return 0;
    cfg = &butt_physics_person_cfg[person_index];
    return cfg->enabled && cfg->enabled_person[person_index];
}

static int body_chain_collision_scope_includes_hands(int scope)
{
    return (body_chain_collision_scope_collider_mask(scope) &
            BODY_CHAIN_COLLIDER_GROUP_HANDS) != 0;
}

static int body_chain_collision_scope_is_full_body(int scope)
{
    return (body_chain_collision_scope_collider_mask(scope) &
            BODY_CHAIN_COLLIDER_GROUP_FULL_BODY) != 0;
}

static int body_chain_collider_node_is_hand(int node)
{
    return node >= BODY_COLLIDER_WRIST_L &&
           node <= BODY_COLLIDER_FINGER05_R_END;
}

static const char *body_chain_collision_scope_name(int scope)
{
    switch (scope) {
    case BODY_CHAIN_COLLISION_SCOPE_GENITALS_ONLY:
        return "genitals_only";
    case BODY_CHAIN_COLLISION_SCOPE_GENITALS_ONLY_ALL:
        return "genitals_only_all";
    case BODY_CHAIN_COLLISION_SCOPE_PELVIS_AND_GENITALS_ONLY:
        return "pelvis_and_genitals_only";
    case BODY_CHAIN_COLLISION_SCOPE_PELVIS_AND_GENITALS_ONLY_ALL:
        return "pelvis_and_genitals_only_all";
    case BODY_CHAIN_COLLISION_SCOPE_PELVIS_GENITALS_AND_HANDS_ONLY:
        return "pelvis_genitals_hands_only";
    case BODY_CHAIN_COLLISION_SCOPE_PELVIS_GENITALS_AND_HANDS_ONLY_ALL:
        return "pelvis_genitals_hands_only_all";
    case BODY_CHAIN_COLLISION_SCOPE_FULL_BODY:
        return "full_body";
    case BODY_CHAIN_COLLISION_SCOPE_FULL_BODY_ALL:
        return "full_body_all";
    case BODY_CHAIN_COLLISION_SCOPE_HANDS_ONLY:
        return "hands_only";
    case BODY_CHAIN_COLLISION_SCOPE_HANDS_ONLY_ALL:
        return "hands_only_all";
    default:
        return "full_body_all";
    }
}

static body_chain_person_state_t body_chain_person_states[4];
static body_chain_person_state_t testicle_physics_states[4];
static body_chain_person_state_t runtime_body_chain_person_states[4];
static body_chain_person_state_t runtime_testicle_physics_states[4];
static runtime_body_chain_ownership_state_t
    runtime_body_chain_ownership_states[4];
static runtime_body_chain_ownership_state_t
    runtime_testicle_ownership_states[4];
static body_chain_axis_reference_cache_t
    poseeditor_body_chain_axis_reference_cache[4];
static body_chain_axis_reference_cache_t
    runtime_body_chain_axis_reference_cache[4];
static body_chain_axis_reference_cache_t
    poseeditor_breasts_axis_reference_cache[4];
static body_chain_axis_reference_cache_t
    runtime_breasts_axis_reference_cache[4];
static body_chain_gravity_snapshot_t body_chain_room_gravity_cache[4];
static body_chain_gravity_snapshot_t testicle_room_gravity_cache[4];
static body_chain_gravity_snapshot_t runtime_body_chain_room_gravity_cache[4];
static body_chain_gravity_snapshot_t runtime_testicle_room_gravity_cache[4];
static tk17_testicle_inertia_gate_state_t tk17_testicle_inertia_gate_states[4];
static tk17_testicle_inertia_gate_state_t
    tk17_breasts_inertia_gate_states[4][2];
static tk17_testicle_inertia_gate_state_t
    tk17_butt_inertia_gate_states[4][2];
static breasts_physics_person_state_t breasts_physics_states[4];
static body_chain_gravity_snapshot_t breasts_physics_room_gravity_cache[4];
static breasts_physics_person_state_t butt_physics_states[4];
static body_chain_gravity_snapshot_t butt_physics_room_gravity_cache[4];
static body_chain_collider_person_state_t body_chain_collider_states[4];
static const body_collider_direct_node_def_t body_collider_direct_nodes[BODY_COLLIDER_DIRECT_NODE_COUNT] = {
    { BODY_COLLIDER_STOMACH_03, "spine_joint03", "Sspine_joint03", 1, "engine-pivot:spine_joint03" },
    { BODY_COLLIDER_STOMACH_04, "spine_joint04", "Sspine_joint04", 1, "engine-pivot:spine_joint04" },
    { BODY_COLLIDER_NECK_01, "neck_joint01", "Sneck_joint01", 1, "engine-pivot:neck_joint01" },
    { BODY_COLLIDER_ANKLE_L, "ankle_L_joint", "Sankle_L_joint", 1, "engine-pivot:ankle_L_joint" },
    { BODY_COLLIDER_ANKLE_R, "ankle_R_joint", "Sankle_R_joint", -1, "engine-pivot:ankle_R_joint" },
    { BODY_COLLIDER_BALL_L, "ball_L_joint", "Sball_L_joint", 1, "engine-pivot:ball_L_joint" },
    { BODY_COLLIDER_BALL_R, "ball_R_joint", "Sball_R_joint", -1, "engine-pivot:ball_R_joint" },
    { BODY_COLLIDER_BREAST_L, "breast_scale_L_joint", "Sbreast_scale_L_joint", 1, "engine-pivot:breast_scale_L_joint" },
    { BODY_COLLIDER_BREAST_R, "breast_scale_R_joint", "Sbreast_scale_R_joint", -1, "engine-pivot:breast_scale_R_joint" },
    { BODY_COLLIDER_HEAD_02, "head_joint02", "Shead_joint02", 1, "engine-pivot:head_joint02" },
    { BODY_COLLIDER_CLAVICLE_L, "clavicle_L_joint", "Sclavicle_L_joint", 1, "engine-pivot:clavicle_L_joint" },
    { BODY_COLLIDER_CLAVICLE_R, "clavicle_R_joint", "Sclavicle_R_joint", -1, "engine-pivot:clavicle_R_joint" },
    { BODY_COLLIDER_SHOULDER_L, "shoulder_L_joint", "Sshoulder_L_joint", 1, "engine-pivot:shoulder_L_joint" },
    { BODY_COLLIDER_SHOULDER_R, "shoulder_R_joint", "Sshoulder_R_joint", -1, "engine-pivot:shoulder_R_joint" },
    { BODY_COLLIDER_ELBOW_L, "elbow_L_joint", "Selbow_L_joint", 1, "engine-pivot:elbow_L_joint" },
    { BODY_COLLIDER_ELBOW_R, "elbow_R_joint", "Selbow_R_joint", -1, "engine-pivot:elbow_R_joint" },
    { BODY_COLLIDER_FOREARM_L, "forearm_L_joint", "Sforearm_L_joint", 1, "engine-pivot:forearm_L_joint" },
    { BODY_COLLIDER_FOREARM_R, "forearm_R_joint", "Sforearm_R_joint", -1, "engine-pivot:forearm_R_joint" },
    { BODY_COLLIDER_WRIST_L, "wrist_L_joint", "Swrist_L_joint", 1, "engine-pivot:wrist_L_joint" },
    { BODY_COLLIDER_WRIST_R, "wrist_R_joint", "Swrist_R_joint", -1, "engine-pivot:wrist_R_joint" },
    { BODY_COLLIDER_FINGER01_L_01, "finger01_L_joint01", "Sfinger01_L_joint01", 1, "engine-pivot:finger01_L_joint01" },
    { BODY_COLLIDER_FINGER01_L_02, "finger01_L_joint02", "Sfinger01_L_joint02", 1, "engine-pivot:finger01_L_joint02" },
    { BODY_COLLIDER_FINGER01_L_03, "finger01_L_joint03", "Sfinger01_L_joint03", 1, "engine-pivot:finger01_L_joint03" },
    { BODY_COLLIDER_FINGER01_L_END, "finger01_L_jointEnd", "Sfinger01_L_jointEnd", 1, "engine-pivot:finger01_L_jointEnd" },
    { BODY_COLLIDER_FINGER01_R_01, "finger01_R_joint01", "Sfinger01_R_joint01", -1, "engine-pivot:finger01_R_joint01" },
    { BODY_COLLIDER_FINGER01_R_02, "finger01_R_joint02", "Sfinger01_R_joint02", -1, "engine-pivot:finger01_R_joint02" },
    { BODY_COLLIDER_FINGER01_R_03, "finger01_R_joint03", "Sfinger01_R_joint03", -1, "engine-pivot:finger01_R_joint03" },
    { BODY_COLLIDER_FINGER01_R_END, "finger01_R_jointEnd", "Sfinger01_R_jointEnd", -1, "engine-pivot:finger01_R_jointEnd" },
    { BODY_COLLIDER_FINGER02_L_01, "finger02_L_joint01", "Sfinger02_L_joint01", 1, "engine-pivot:finger02_L_joint01" },
    { BODY_COLLIDER_FINGER02_L_02, "finger02_L_joint02", "Sfinger02_L_joint02", 1, "engine-pivot:finger02_L_joint02" },
    { BODY_COLLIDER_FINGER02_L_03, "finger02_L_joint03", "Sfinger02_L_joint03", 1, "engine-pivot:finger02_L_joint03" },
    { BODY_COLLIDER_FINGER02_L_04, "finger02_L_joint04", "Sfinger02_L_joint04", 1, "engine-pivot:finger02_L_joint04" },
    { BODY_COLLIDER_FINGER02_L_END, "finger02_L_jointEnd", "Sfinger02_L_jointEnd", 1, "engine-pivot:finger02_L_jointEnd" },
    { BODY_COLLIDER_FINGER02_R_01, "finger02_R_joint01", "Sfinger02_R_joint01", -1, "engine-pivot:finger02_R_joint01" },
    { BODY_COLLIDER_FINGER02_R_02, "finger02_R_joint02", "Sfinger02_R_joint02", -1, "engine-pivot:finger02_R_joint02" },
    { BODY_COLLIDER_FINGER02_R_03, "finger02_R_joint03", "Sfinger02_R_joint03", -1, "engine-pivot:finger02_R_joint03" },
    { BODY_COLLIDER_FINGER02_R_04, "finger02_R_joint04", "Sfinger02_R_joint04", -1, "engine-pivot:finger02_R_joint04" },
    { BODY_COLLIDER_FINGER02_R_END, "finger02_R_jointEnd", "Sfinger02_R_jointEnd", -1, "engine-pivot:finger02_R_jointEnd" },
    { BODY_COLLIDER_FINGER03_L_01, "finger03_L_joint01", "Sfinger03_L_joint01", 1, "engine-pivot:finger03_L_joint01" },
    { BODY_COLLIDER_FINGER03_L_02, "finger03_L_joint02", "Sfinger03_L_joint02", 1, "engine-pivot:finger03_L_joint02" },
    { BODY_COLLIDER_FINGER03_L_03, "finger03_L_joint03", "Sfinger03_L_joint03", 1, "engine-pivot:finger03_L_joint03" },
    { BODY_COLLIDER_FINGER03_L_04, "finger03_L_joint04", "Sfinger03_L_joint04", 1, "engine-pivot:finger03_L_joint04" },
    { BODY_COLLIDER_FINGER03_L_END, "finger03_L_jointEnd", "Sfinger03_L_jointEnd", 1, "engine-pivot:finger03_L_jointEnd" },
    { BODY_COLLIDER_FINGER03_R_01, "finger03_R_joint01", "Sfinger03_R_joint01", -1, "engine-pivot:finger03_R_joint01" },
    { BODY_COLLIDER_FINGER03_R_02, "finger03_R_joint02", "Sfinger03_R_joint02", -1, "engine-pivot:finger03_R_joint02" },
    { BODY_COLLIDER_FINGER03_R_03, "finger03_R_joint03", "Sfinger03_R_joint03", -1, "engine-pivot:finger03_R_joint03" },
    { BODY_COLLIDER_FINGER03_R_04, "finger03_R_joint04", "Sfinger03_R_joint04", -1, "engine-pivot:finger03_R_joint04" },
    { BODY_COLLIDER_FINGER03_R_END, "finger03_R_jointEnd", "Sfinger03_R_jointEnd", -1, "engine-pivot:finger03_R_jointEnd" },
    { BODY_COLLIDER_FINGER04_L_01, "finger04_L_joint01", "Sfinger04_L_joint01", 1, "engine-pivot:finger04_L_joint01" },
    { BODY_COLLIDER_FINGER04_L_02, "finger04_L_joint02", "Sfinger04_L_joint02", 1, "engine-pivot:finger04_L_joint02" },
    { BODY_COLLIDER_FINGER04_L_03, "finger04_L_joint03", "Sfinger04_L_joint03", 1, "engine-pivot:finger04_L_joint03" },
    { BODY_COLLIDER_FINGER04_L_04, "finger04_L_joint04", "Sfinger04_L_joint04", 1, "engine-pivot:finger04_L_joint04" },
    { BODY_COLLIDER_FINGER04_L_END, "finger04_L_jointEnd", "Sfinger04_L_jointEnd", 1, "engine-pivot:finger04_L_jointEnd" },
    { BODY_COLLIDER_FINGER04_R_01, "finger04_R_joint01", "Sfinger04_R_joint01", -1, "engine-pivot:finger04_R_joint01" },
    { BODY_COLLIDER_FINGER04_R_02, "finger04_R_joint02", "Sfinger04_R_joint02", -1, "engine-pivot:finger04_R_joint02" },
    { BODY_COLLIDER_FINGER04_R_03, "finger04_R_joint03", "Sfinger04_R_joint03", -1, "engine-pivot:finger04_R_joint03" },
    { BODY_COLLIDER_FINGER04_R_04, "finger04_R_joint04", "Sfinger04_R_joint04", -1, "engine-pivot:finger04_R_joint04" },
    { BODY_COLLIDER_FINGER04_R_END, "finger04_R_jointEnd", "Sfinger04_R_jointEnd", -1, "engine-pivot:finger04_R_jointEnd" },
    { BODY_COLLIDER_FINGER05_L_01, "finger05_L_joint01", "Sfinger05_L_joint01", 1, "engine-pivot:finger05_L_joint01" },
    { BODY_COLLIDER_FINGER05_L_02, "finger05_L_joint02", "Sfinger05_L_joint02", 1, "engine-pivot:finger05_L_joint02" },
    { BODY_COLLIDER_FINGER05_L_03, "finger05_L_joint03", "Sfinger05_L_joint03", 1, "engine-pivot:finger05_L_joint03" },
    { BODY_COLLIDER_FINGER05_L_04, "finger05_L_joint04", "Sfinger05_L_joint04", 1, "engine-pivot:finger05_L_joint04" },
    { BODY_COLLIDER_FINGER05_L_END, "finger05_L_jointEnd", "Sfinger05_L_jointEnd", 1, "engine-pivot:finger05_L_jointEnd" },
    { BODY_COLLIDER_FINGER05_R_01, "finger05_R_joint01", "Sfinger05_R_joint01", -1, "engine-pivot:finger05_R_joint01" },
    { BODY_COLLIDER_FINGER05_R_02, "finger05_R_joint02", "Sfinger05_R_joint02", -1, "engine-pivot:finger05_R_joint02" },
    { BODY_COLLIDER_FINGER05_R_03, "finger05_R_joint03", "Sfinger05_R_joint03", -1, "engine-pivot:finger05_R_joint03" },
    { BODY_COLLIDER_FINGER05_R_04, "finger05_R_joint04", "Sfinger05_R_joint04", -1, "engine-pivot:finger05_R_joint04" },
    { BODY_COLLIDER_FINGER05_R_END, "finger05_R_jointEnd", "Sfinger05_R_jointEnd", -1, "engine-pivot:finger05_R_jointEnd" },
    { BODY_COLLIDER_BUTT_L, "butt_L_joint01", "Sbutt_L_joint01", 1, "engine-pivot:butt_L_joint01" },
    { BODY_COLLIDER_BUTT_R, "butt_R_joint01", "Sbutt_R_joint01", -1, "engine-pivot:butt_R_joint01" }
};

static const body_collider_extra_edge_def_t body_collider_extra_edges[BODY_COLLIDER_EXTRA_EDGE_COUNT] = {
    { BODY_COLLIDER_STOMACH_02, BODY_COLLIDER_STOMACH_03, "spine02_to_spine03", 0xff80ff40, 128, 255, 64 },
    { BODY_COLLIDER_STOMACH_03, BODY_COLLIDER_STOMACH_04, "spine03_to_spine04", 0xff80ff40, 128, 255, 64 },
    { BODY_COLLIDER_STOMACH_04, BODY_COLLIDER_NECK_01, "spine04_to_neck01", 0xff80ff40, 128, 255, 64 },
    { BODY_COLLIDER_KNEE_L, BODY_COLLIDER_ANKLE_L, "knee_L_to_ankle_L", 0xff20ffff, 32, 255, 255 },
    { BODY_COLLIDER_ANKLE_L, BODY_COLLIDER_BALL_L, "ankle_L_to_ball_L", 0xff20ffff, 32, 255, 255 },
    { BODY_COLLIDER_KNEE_R, BODY_COLLIDER_ANKLE_R, "knee_R_to_ankle_R", 0xff20ffff, 32, 255, 255 },
    { BODY_COLLIDER_ANKLE_R, BODY_COLLIDER_BALL_R, "ankle_R_to_ball_R", 0xff20ffff, 32, 255, 255 },
    { BODY_COLLIDER_NECK_01, BODY_COLLIDER_HEAD_02, "neck01_to_head02", 0xffff4040, 255, 64, 64 },
    { BODY_COLLIDER_CLAVICLE_L, BODY_COLLIDER_SHOULDER_L, "clavicle_L_to_shoulder_L", 0xffffffff, 255, 255, 255 },
    { BODY_COLLIDER_SHOULDER_L, BODY_COLLIDER_ELBOW_L, "shoulder_L_to_elbow_L", 0xffffffff, 255, 255, 255 },
    { BODY_COLLIDER_ELBOW_L, BODY_COLLIDER_FOREARM_L, "elbow_L_to_forearm_L", 0xffffffff, 255, 255, 255 },
    { BODY_COLLIDER_FOREARM_L, BODY_COLLIDER_WRIST_L, "forearm_L_to_wrist_L", 0xffffffff, 255, 255, 255 },
    { BODY_COLLIDER_WRIST_L, BODY_COLLIDER_PALM_L, "wrist_L_to_palm_L", 0xffffffff, 255, 255, 255 },
    { BODY_COLLIDER_CLAVICLE_R, BODY_COLLIDER_SHOULDER_R, "clavicle_R_to_shoulder_R", 0xffffffff, 255, 255, 255 },
    { BODY_COLLIDER_SHOULDER_R, BODY_COLLIDER_ELBOW_R, "shoulder_R_to_elbow_R", 0xffffffff, 255, 255, 255 },
    { BODY_COLLIDER_ELBOW_R, BODY_COLLIDER_FOREARM_R, "elbow_R_to_forearm_R", 0xffffffff, 255, 255, 255 },
    { BODY_COLLIDER_FOREARM_R, BODY_COLLIDER_WRIST_R, "forearm_R_to_wrist_R", 0xffffffff, 255, 255, 255 },
    { BODY_COLLIDER_WRIST_R, BODY_COLLIDER_PALM_R, "wrist_R_to_palm_R", 0xffffffff, 255, 255, 255 },
    { BODY_COLLIDER_FINGER01_L_01, BODY_COLLIDER_FINGER01_L_02, "finger01_L_01_to_02", 0xffff4040, 255, 64, 64 },
    { BODY_COLLIDER_FINGER01_L_02, BODY_COLLIDER_FINGER01_L_03, "finger01_L_02_to_03", 0xffff4040, 255, 64, 64 },
    { BODY_COLLIDER_FINGER01_L_03, BODY_COLLIDER_FINGER01_L_END, "finger01_L_03_to_end", 0xffff4040, 255, 64, 64 },
    { BODY_COLLIDER_FINGER01_R_01, BODY_COLLIDER_FINGER01_R_02, "finger01_R_01_to_02", 0xffff4040, 255, 64, 64 },
    { BODY_COLLIDER_FINGER01_R_02, BODY_COLLIDER_FINGER01_R_03, "finger01_R_02_to_03", 0xffff4040, 255, 64, 64 },
    { BODY_COLLIDER_FINGER01_R_03, BODY_COLLIDER_FINGER01_R_END, "finger01_R_03_to_end", 0xffff4040, 255, 64, 64 },
    { BODY_COLLIDER_FINGER02_L_01, BODY_COLLIDER_FINGER02_L_02, "finger02_L_01_to_02", 0xffff4040, 255, 64, 64 },
    { BODY_COLLIDER_FINGER02_L_02, BODY_COLLIDER_FINGER02_L_03, "finger02_L_02_to_03", 0xffff4040, 255, 64, 64 },
    { BODY_COLLIDER_FINGER02_L_03, BODY_COLLIDER_FINGER02_L_04, "finger02_L_03_to_04", 0xffff4040, 255, 64, 64 },
    { BODY_COLLIDER_FINGER02_L_04, BODY_COLLIDER_FINGER02_L_END, "finger02_L_04_to_end", 0xffff4040, 255, 64, 64 },
    { BODY_COLLIDER_FINGER02_R_01, BODY_COLLIDER_FINGER02_R_02, "finger02_R_01_to_02", 0xffff4040, 255, 64, 64 },
    { BODY_COLLIDER_FINGER02_R_02, BODY_COLLIDER_FINGER02_R_03, "finger02_R_02_to_03", 0xffff4040, 255, 64, 64 },
    { BODY_COLLIDER_FINGER02_R_03, BODY_COLLIDER_FINGER02_R_04, "finger02_R_03_to_04", 0xffff4040, 255, 64, 64 },
    { BODY_COLLIDER_FINGER02_R_04, BODY_COLLIDER_FINGER02_R_END, "finger02_R_04_to_end", 0xffff4040, 255, 64, 64 },
    { BODY_COLLIDER_FINGER03_L_01, BODY_COLLIDER_FINGER03_L_02, "finger03_L_01_to_02", 0xffff4040, 255, 64, 64 },
    { BODY_COLLIDER_FINGER03_L_02, BODY_COLLIDER_FINGER03_L_03, "finger03_L_02_to_03", 0xffff4040, 255, 64, 64 },
    { BODY_COLLIDER_FINGER03_L_03, BODY_COLLIDER_FINGER03_L_04, "finger03_L_03_to_04", 0xffff4040, 255, 64, 64 },
    { BODY_COLLIDER_FINGER03_L_04, BODY_COLLIDER_FINGER03_L_END, "finger03_L_04_to_end", 0xffff4040, 255, 64, 64 },
    { BODY_COLLIDER_FINGER03_R_01, BODY_COLLIDER_FINGER03_R_02, "finger03_R_01_to_02", 0xffff4040, 255, 64, 64 },
    { BODY_COLLIDER_FINGER03_R_02, BODY_COLLIDER_FINGER03_R_03, "finger03_R_02_to_03", 0xffff4040, 255, 64, 64 },
    { BODY_COLLIDER_FINGER03_R_03, BODY_COLLIDER_FINGER03_R_04, "finger03_R_03_to_04", 0xffff4040, 255, 64, 64 },
    { BODY_COLLIDER_FINGER03_R_04, BODY_COLLIDER_FINGER03_R_END, "finger03_R_04_to_end", 0xffff4040, 255, 64, 64 },
    { BODY_COLLIDER_FINGER04_L_01, BODY_COLLIDER_FINGER04_L_02, "finger04_L_01_to_02", 0xffff4040, 255, 64, 64 },
    { BODY_COLLIDER_FINGER04_L_02, BODY_COLLIDER_FINGER04_L_03, "finger04_L_02_to_03", 0xffff4040, 255, 64, 64 },
    { BODY_COLLIDER_FINGER04_L_03, BODY_COLLIDER_FINGER04_L_04, "finger04_L_03_to_04", 0xffff4040, 255, 64, 64 },
    { BODY_COLLIDER_FINGER04_L_04, BODY_COLLIDER_FINGER04_L_END, "finger04_L_04_to_end", 0xffff4040, 255, 64, 64 },
    { BODY_COLLIDER_FINGER04_R_01, BODY_COLLIDER_FINGER04_R_02, "finger04_R_01_to_02", 0xffff4040, 255, 64, 64 },
    { BODY_COLLIDER_FINGER04_R_02, BODY_COLLIDER_FINGER04_R_03, "finger04_R_02_to_03", 0xffff4040, 255, 64, 64 },
    { BODY_COLLIDER_FINGER04_R_03, BODY_COLLIDER_FINGER04_R_04, "finger04_R_03_to_04", 0xffff4040, 255, 64, 64 },
    { BODY_COLLIDER_FINGER04_R_04, BODY_COLLIDER_FINGER04_R_END, "finger04_R_04_to_end", 0xffff4040, 255, 64, 64 },
    { BODY_COLLIDER_FINGER05_L_01, BODY_COLLIDER_FINGER05_L_02, "finger05_L_01_to_02", 0xffff4040, 255, 64, 64 },
    { BODY_COLLIDER_FINGER05_L_02, BODY_COLLIDER_FINGER05_L_03, "finger05_L_02_to_03", 0xffff4040, 255, 64, 64 },
    { BODY_COLLIDER_FINGER05_L_03, BODY_COLLIDER_FINGER05_L_04, "finger05_L_03_to_04", 0xffff4040, 255, 64, 64 },
    { BODY_COLLIDER_FINGER05_L_04, BODY_COLLIDER_FINGER05_L_END, "finger05_L_04_to_end", 0xffff4040, 255, 64, 64 },
    { BODY_COLLIDER_FINGER05_R_01, BODY_COLLIDER_FINGER05_R_02, "finger05_R_01_to_02", 0xffff4040, 255, 64, 64 },
    { BODY_COLLIDER_FINGER05_R_02, BODY_COLLIDER_FINGER05_R_03, "finger05_R_02_to_03", 0xffff4040, 255, 64, 64 },
    { BODY_COLLIDER_FINGER05_R_03, BODY_COLLIDER_FINGER05_R_04, "finger05_R_03_to_04", 0xffff4040, 255, 64, 64 },
    { BODY_COLLIDER_FINGER05_R_04, BODY_COLLIDER_FINGER05_R_END, "finger05_R_04_to_end", 0xffff4040, 255, 64, 64 }
};

static void body_chain_set_vec3(float out[3], float x, float y, float z)
{
    if (!out) return;
    out[0] = x;
    out[1] = y;
    out[2] = z;
}

static void body_chain_copy_vec3(float out[3], const float in[3])
{
    if (!out || !in) return;
    out[0] = in[0];
    out[1] = in[1];
    out[2] = in[2];
}

static void body_chain_set_radius_scalar(float out[3], float radius)
{
    body_chain_set_vec3(out, radius, radius, radius);
}

static void parse_radius_vec3(const char *s, float out[3])
{
    float x, y, z;
    if (!s || !s[0] || !out) return;
    if (sscanf(s, "%f,%f,%f", &x, &y, &z) == 3 ||
        sscanf(s, "%f %f %f", &x, &y, &z) == 3) {
        out[0] = x;
        out[1] = y;
        out[2] = z;
    } else if (sscanf(s, "%f", &x) == 1) {
        out[0] = x;
        out[1] = x;
        out[2] = x;
    }
}

static void profile_radius_vec3(const char *section,
                                const char *key,
                                float out[3],
                                const char *path)
{
    char buf[128];
    GetPrivateProfileStringA(section, key, "", buf, sizeof(buf), path);
    trim_in_place(buf);
    parse_radius_vec3(buf, out);
}

static int profile_string_found(const char *section,
                                const char *key,
                                char *out,
                                DWORD out_size,
                                const char *path)
{
    static const char sentinel[] = "\x1f__missing__\x1f";
    if (!section || !key || !out || out_size == 0) return 0;
    GetPrivateProfileStringA(section, key, sentinel, out, out_size, path);
    return strcmp(out, sentinel) != 0;
}

static int body_colliders_profile_string(const char *key,
                                         const char *fallback,
                                         char *out,
                                         DWORD out_size,
                                         const char *path)
{
    if (!out || out_size == 0) return 0;
    if (profile_string_found(BODY_COLLIDERS_CONFIG_SECTION,
                             key, out, out_size, path)) {
        return 1;
    }
    if (profile_string_found(BODY_COLLIDERS_LEGACY_CONFIG_SECTION,
                             key, out, out_size, path)) {
        return 1;
    }
    lstrcpynA(out, fallback ? fallback : "", (int)out_size);
    return 0;
}

static int body_colliders_profile_string_alias(const char *preferred_key,
                                                const char *legacy_key,
                                                const char *fallback,
                                                char *out,
                                                DWORD out_size,
                                                const char *path)
{
    if (!out || out_size == 0) return 0;
    if (preferred_key &&
        body_colliders_profile_string(preferred_key, "", out, out_size, path)) {
        return 1;
    }
    if (legacy_key &&
        body_colliders_profile_string(legacy_key, "", out, out_size, path)) {
        return 1;
    }
    lstrcpynA(out, fallback ? fallback : "", (int)out_size);
    return 0;
}

static float body_colliders_profile_float(const char *key,
                                          float fallback,
                                          const char *path)
{
    char buf[128];
    if (!body_colliders_profile_string(key, "", buf, sizeof(buf), path)) {
        return fallback;
    }
    trim_in_place(buf);
    if (!buf[0]) return fallback;
    return (float)atof(buf);
}

static int body_colliders_profile_bool(const char *key,
                                       int fallback,
                                       const char *path)
{
    char buf[64];
    if (!body_colliders_profile_string(key, "", buf, sizeof(buf), path)) {
        return fallback ? 1 : 0;
    }
    trim_in_place(buf);
    if (!buf[0]) return fallback ? 1 : 0;
    if (_stricmp(buf, "1") == 0 || _stricmp(buf, "true") == 0 ||
        _stricmp(buf, "yes") == 0 || _stricmp(buf, "on") == 0) return 1;
    if (_stricmp(buf, "0") == 0 || _stricmp(buf, "false") == 0 ||
        _stricmp(buf, "no") == 0 || _stricmp(buf, "off") == 0) return 0;
    return atoi(buf) ? 1 : 0;
}

static int body_colliders_profile_int(const char *key,
                                      int fallback,
                                      const char *path)
{
    char buf[64];
    char *end = NULL;
    long value;
    if (!body_colliders_profile_string(key, "", buf, sizeof(buf), path)) {
        return fallback;
    }
    trim_in_place(buf);
    if (!buf[0]) return fallback;
    value = strtol(buf, &end, 0);
    if (end == buf) return fallback;
    return (int)value;
}

static void body_colliders_profile_radius_vec3(const char *key,
                                               float out[3],
                                               const char *path)
{
    char buf[128];
    if (!key || !out) return;
    if (!body_colliders_profile_string(key, "", buf, sizeof(buf), path)) {
        return;
    }
    trim_in_place(buf);
    parse_radius_vec3(buf, out);
}

static void body_colliders_profile_radius_vec3_alias(
    const char *preferred_key,
    const char *legacy_key,
    float out[3],
    const char *path)
{
    char buf[128];
    if (!preferred_key || !out) return;
    if (!body_colliders_profile_string_alias(preferred_key, legacy_key, "",
                                             buf, sizeof(buf), path)) {
        return;
    }
    trim_in_place(buf);
    parse_radius_vec3(buf, out);
}

static void body_chain_clamp_radius_vec3(float out[3])
{
    int axis;
    if (!out) return;
    for (axis = 0; axis < 3; axis++) {
        out[axis] = physx_clampf(out[axis], 0.005f, 2.0f);
    }
}

static float body_chain_radius_vec3_max(const float radius[3])
{
    float r;
    if (!radius) return 0.0f;
    r = radius[0];
    if (radius[1] > r) r = radius[1];
    if (radius[2] > r) r = radius[2];
    return r;
}

static void body_chain_set_node_radius(int node_index, const float radius[3])
{
    if (node_index < 0 || node_index >= BODY_COLLIDER_NODE_COUNT || !radius) {
        return;
    }
    body_chain_copy_vec3(body_chain_collider_cfg.node_radius[node_index],
                         radius);
}

static void body_chain_set_node_fine(int node_index, const float fine[3])
{
    if (node_index < 0 || node_index >= BODY_COLLIDER_NODE_COUNT || !fine) {
        return;
    }
    body_chain_copy_vec3(body_chain_collider_cfg.node_fine_offset[node_index],
                         fine);
}

static void body_chain_apply_radius_to_pair(int left_node,
                                            int right_node,
                                            const float radius[3])
{
    body_chain_set_node_radius(left_node, radius);
    body_chain_set_node_radius(right_node, radius);
}

static void body_chain_apply_fine_to_pair(int left_node,
                                          int right_node,
                                          const float fine[3])
{
    body_chain_set_node_fine(left_node, fine);
    body_chain_set_node_fine(right_node, fine);
}

static void body_chain_collider_apply_config_to_nodes(void)
{
    float scalar_radius[3];
    int i;
    for (i = 0; i < BODY_COLLIDER_NODE_COUNT; i++) {
        body_chain_set_vec3(body_chain_collider_cfg.node_radius[i],
                            0.020f, 0.020f, 0.020f);
        body_chain_set_vec3(body_chain_collider_cfg.node_fine_offset[i],
                            0.0f, 0.0f, 0.0f);
    }

    body_chain_set_radius_scalar(scalar_radius,
                                 body_chain_collider_cfg.pelvis_radius);
    body_chain_set_node_radius(BODY_COLLIDER_ROOT, scalar_radius);
    body_chain_set_node_radius(BODY_COLLIDER_STOMACH_01,
                               body_chain_collider_cfg.stomach_radius[0]);
    body_chain_set_node_radius(BODY_COLLIDER_STOMACH_02,
                               body_chain_collider_cfg.stomach_radius[1]);
    body_chain_set_node_radius(BODY_COLLIDER_STOMACH_03,
                               body_chain_collider_cfg.stomach_extra_radius[0]);
    body_chain_set_node_radius(BODY_COLLIDER_STOMACH_04,
                               body_chain_collider_cfg.stomach_extra_radius[1]);

    body_chain_set_radius_scalar(scalar_radius,
                                 body_chain_collider_cfg.hip_radius);
    body_chain_apply_radius_to_pair(BODY_COLLIDER_HIP_L,
                                    BODY_COLLIDER_HIP_R,
                                    scalar_radius);
    body_chain_set_radius_scalar(scalar_radius,
                                 body_chain_collider_cfg.thigh_radius);
    body_chain_apply_radius_to_pair(BODY_COLLIDER_THIGH_L,
                                    BODY_COLLIDER_THIGH_R,
                                    scalar_radius);
    body_chain_set_radius_scalar(scalar_radius,
                                 body_chain_collider_cfg.knee_radius);
    body_chain_apply_radius_to_pair(BODY_COLLIDER_KNEE_L,
                                    BODY_COLLIDER_KNEE_R,
                                    scalar_radius);
    body_chain_set_radius_scalar(scalar_radius,
                                 body_chain_collider_cfg.testicles_radius);
    body_chain_set_node_radius(BODY_COLLIDER_TESTICLES_01, scalar_radius);
    body_chain_set_node_radius(BODY_COLLIDER_TESTICLES_02, scalar_radius);
    body_chain_set_node_radius(BODY_COLLIDER_TESTICLES_MID, scalar_radius);

    body_chain_apply_radius_to_pair(BODY_COLLIDER_ANKLE_L,
                                    BODY_COLLIDER_ANKLE_R,
                                    body_chain_collider_cfg.ankle_radius);
    body_chain_apply_radius_to_pair(BODY_COLLIDER_BALL_L,
                                    BODY_COLLIDER_BALL_R,
                                    body_chain_collider_cfg.ball_radius);
    body_chain_apply_radius_to_pair(BODY_COLLIDER_BREAST_L,
                                    BODY_COLLIDER_BREAST_R,
                                    body_chain_collider_cfg.breast_radius);
    body_chain_apply_radius_to_pair(BODY_COLLIDER_BUTT_L,
                                    BODY_COLLIDER_BUTT_R,
                                    body_chain_collider_cfg.butt_radius);
    body_chain_set_node_radius(BODY_COLLIDER_NECK_01,
                               body_chain_collider_cfg.neck_radius);
    body_chain_set_node_radius(BODY_COLLIDER_HEAD_02,
                               body_chain_collider_cfg.head_radius);
    body_chain_apply_radius_to_pair(BODY_COLLIDER_CLAVICLE_L,
                                    BODY_COLLIDER_CLAVICLE_R,
                                    body_chain_collider_cfg.clavicle_radius);
    body_chain_apply_radius_to_pair(BODY_COLLIDER_SHOULDER_L,
                                    BODY_COLLIDER_SHOULDER_R,
                                    body_chain_collider_cfg.shoulder_radius);
    body_chain_apply_radius_to_pair(BODY_COLLIDER_ELBOW_L,
                                    BODY_COLLIDER_ELBOW_R,
                                    body_chain_collider_cfg.elbow_radius);
    body_chain_apply_radius_to_pair(BODY_COLLIDER_FOREARM_L,
                                    BODY_COLLIDER_FOREARM_R,
                                    body_chain_collider_cfg.forearm_radius);
    body_chain_apply_radius_to_pair(BODY_COLLIDER_WRIST_L,
                                    BODY_COLLIDER_WRIST_R,
                                    body_chain_collider_cfg.wrist_radius);
    body_chain_apply_radius_to_pair(BODY_COLLIDER_PALM_L,
                                    BODY_COLLIDER_PALM_R,
                                    body_chain_collider_cfg.palm_radius);

    for (i = 0; i < 4; i++) {
        body_chain_set_node_radius(BODY_COLLIDER_FINGER01_L_01 + i,
                                   body_chain_collider_cfg.finger_radius[0]);
        body_chain_set_node_radius(BODY_COLLIDER_FINGER01_R_01 + i,
                                   body_chain_collider_cfg.finger_radius[0]);
    }
    for (i = 0; i < 5; i++) {
        body_chain_set_node_radius(BODY_COLLIDER_FINGER02_L_01 + i,
                                   body_chain_collider_cfg.finger_radius[1]);
        body_chain_set_node_radius(BODY_COLLIDER_FINGER02_R_01 + i,
                                   body_chain_collider_cfg.finger_radius[1]);
        body_chain_set_node_radius(BODY_COLLIDER_FINGER03_L_01 + i,
                                   body_chain_collider_cfg.finger_radius[2]);
        body_chain_set_node_radius(BODY_COLLIDER_FINGER03_R_01 + i,
                                   body_chain_collider_cfg.finger_radius[2]);
        body_chain_set_node_radius(BODY_COLLIDER_FINGER04_L_01 + i,
                                   body_chain_collider_cfg.finger_radius[3]);
        body_chain_set_node_radius(BODY_COLLIDER_FINGER04_R_01 + i,
                                   body_chain_collider_cfg.finger_radius[3]);
        body_chain_set_node_radius(BODY_COLLIDER_FINGER05_L_01 + i,
                                   body_chain_collider_cfg.finger_radius[4]);
        body_chain_set_node_radius(BODY_COLLIDER_FINGER05_R_01 + i,
                                   body_chain_collider_cfg.finger_radius[4]);
    }

    body_chain_set_node_fine(BODY_COLLIDER_STOMACH_01,
                             body_chain_collider_cfg.stomach_fine_offset[0]);
    body_chain_set_node_fine(BODY_COLLIDER_STOMACH_02,
                             body_chain_collider_cfg.stomach_fine_offset[1]);
    body_chain_apply_fine_to_pair(BODY_COLLIDER_HIP_L,
                                  BODY_COLLIDER_HIP_R,
                                  body_chain_collider_cfg.hip_fine_offset);
    body_chain_apply_fine_to_pair(BODY_COLLIDER_THIGH_L,
                                  BODY_COLLIDER_THIGH_R,
                                  body_chain_collider_cfg.thigh_fine_offset);
    body_chain_apply_fine_to_pair(BODY_COLLIDER_KNEE_L,
                                  BODY_COLLIDER_KNEE_R,
                                  body_chain_collider_cfg.knee_fine_offset);
    body_chain_apply_fine_to_pair(BODY_COLLIDER_ANKLE_L,
                                  BODY_COLLIDER_ANKLE_R,
                                  body_chain_collider_cfg.ankle_fine_offset);
    body_chain_apply_fine_to_pair(BODY_COLLIDER_BALL_L,
                                  BODY_COLLIDER_BALL_R,
                                  body_chain_collider_cfg.ball_fine_offset);
    body_chain_apply_fine_to_pair(BODY_COLLIDER_BREAST_L,
                                  BODY_COLLIDER_BREAST_R,
                                  body_chain_collider_cfg.breast_fine_offset);
    body_chain_apply_fine_to_pair(BODY_COLLIDER_BUTT_L,
                                  BODY_COLLIDER_BUTT_R,
                                  body_chain_collider_cfg.butt_fine_offset);
    body_chain_set_node_fine(BODY_COLLIDER_STOMACH_03,
                             body_chain_collider_cfg.stomach_extra_fine_offset[0]);
    body_chain_set_node_fine(BODY_COLLIDER_STOMACH_04,
                             body_chain_collider_cfg.stomach_extra_fine_offset[1]);
    body_chain_set_node_fine(BODY_COLLIDER_NECK_01,
                             body_chain_collider_cfg.neck_fine_offset);
    body_chain_set_node_fine(BODY_COLLIDER_HEAD_02,
                             body_chain_collider_cfg.head_fine_offset);
    body_chain_apply_fine_to_pair(BODY_COLLIDER_CLAVICLE_L,
                                  BODY_COLLIDER_CLAVICLE_R,
                                  body_chain_collider_cfg.clavicle_fine_offset);
    body_chain_apply_fine_to_pair(BODY_COLLIDER_SHOULDER_L,
                                  BODY_COLLIDER_SHOULDER_R,
                                  body_chain_collider_cfg.shoulder_fine_offset);
    body_chain_apply_fine_to_pair(BODY_COLLIDER_ELBOW_L,
                                  BODY_COLLIDER_ELBOW_R,
                                  body_chain_collider_cfg.elbow_fine_offset);
    body_chain_apply_fine_to_pair(BODY_COLLIDER_FOREARM_L,
                                  BODY_COLLIDER_FOREARM_R,
                                  body_chain_collider_cfg.forearm_fine_offset);
    body_chain_apply_fine_to_pair(BODY_COLLIDER_WRIST_L,
                                  BODY_COLLIDER_WRIST_R,
                                  body_chain_collider_cfg.wrist_fine_offset);
    body_chain_apply_fine_to_pair(BODY_COLLIDER_PALM_L,
                                  BODY_COLLIDER_PALM_R,
                                  body_chain_collider_cfg.palm_fine_offset);

    for (i = 0; i < 4; i++) {
        body_chain_set_node_fine(BODY_COLLIDER_FINGER01_L_01 + i,
                                 body_chain_collider_cfg.finger_fine_offset[0]);
        body_chain_set_node_fine(BODY_COLLIDER_FINGER01_R_01 + i,
                                 body_chain_collider_cfg.finger_fine_offset[0]);
    }
    for (i = 0; i < 5; i++) {
        body_chain_set_node_fine(BODY_COLLIDER_FINGER02_L_01 + i,
                                 body_chain_collider_cfg.finger_fine_offset[1]);
        body_chain_set_node_fine(BODY_COLLIDER_FINGER02_R_01 + i,
                                 body_chain_collider_cfg.finger_fine_offset[1]);
        body_chain_set_node_fine(BODY_COLLIDER_FINGER03_L_01 + i,
                                 body_chain_collider_cfg.finger_fine_offset[2]);
        body_chain_set_node_fine(BODY_COLLIDER_FINGER03_R_01 + i,
                                 body_chain_collider_cfg.finger_fine_offset[2]);
        body_chain_set_node_fine(BODY_COLLIDER_FINGER04_L_01 + i,
                                 body_chain_collider_cfg.finger_fine_offset[3]);
        body_chain_set_node_fine(BODY_COLLIDER_FINGER04_R_01 + i,
                                 body_chain_collider_cfg.finger_fine_offset[3]);
        body_chain_set_node_fine(BODY_COLLIDER_FINGER05_L_01 + i,
                                 body_chain_collider_cfg.finger_fine_offset[4]);
        body_chain_set_node_fine(BODY_COLLIDER_FINGER05_R_01 + i,
                                 body_chain_collider_cfg.finger_fine_offset[4]);
    }
}

static void body_chain_profile_radius_pair(const char *key,
                                           float *scalar_value,
                                           int left_node,
                                           int right_node,
                                           const char *path)
{
    float radius[3];
    if (!key || !scalar_value) return;
    body_chain_set_radius_scalar(radius, *scalar_value);
    body_colliders_profile_radius_vec3(key, radius, path);
    body_chain_clamp_radius_vec3(radius);
    *scalar_value = body_chain_radius_vec3_max(radius);
    body_chain_apply_radius_to_pair(left_node, right_node, radius);
}

static void body_chain_profile_radius_nodes(const char *key,
                                            float radius[3],
                                            int node_a,
                                            int node_b,
                                            const char *path)
{
    if (!key || !radius) return;
    body_colliders_profile_radius_vec3(key, radius, path);
    body_chain_clamp_radius_vec3(radius);
    body_chain_set_node_radius(node_a, radius);
    if (node_b >= 0) {
        body_chain_set_node_radius(node_b, radius);
    }
}

static void body_chain_profile_radius_nodes_alias(const char *preferred_key,
                                                  const char *legacy_key,
                                                  float radius[3],
                                                  int node_a,
                                                  int node_b,
                                                  const char *path)
{
    if (!preferred_key || !radius) return;
    body_colliders_profile_radius_vec3_alias(preferred_key, legacy_key,
                                             radius, path);
    body_chain_clamp_radius_vec3(radius);
    body_chain_set_node_radius(node_a, radius);
    if (node_b >= 0) {
        body_chain_set_node_radius(node_b, radius);
    }
}

static physx_sidecar_t sidecars[32];
static int sidecar_count;
static DWORD last_update_tick;
static DWORD last_sim_tick;
static DWORD physx_simulation_serial;
static volatile LONG physx_late_ownership_active;
static app_find_objc_t engine_FindObjC;
static app_engine_t engine_AppMainEngine;
static app_user_main_t engine_AppMainUserMain;
static model_pivot_t engine_GetModelViewRotationPivot;
static search_tree_t engine_SearchTree;
static int engine_symbols_attempted;
static Direct3DCreate8_t real_Direct3DCreate8;
static d3d8_CreateDevice_t real_d3d8_CreateDevice;
static d3d8_Present_t real_d3d8_Present;
static d3d8_BeginScene_t real_d3d8_BeginScene;
static d3d8_EndScene_t real_d3d8_EndScene;
static d3d8_SetTransform_t real_d3d8_SetTransform;
static SwapBuffers_t real_SwapBuffers;
static CreateFileA_t real_CreateFileA;
static CreateFileW_t real_CreateFileW;
static get_weak_obj_target_t engine_GetWeakObjTarget;
static script_get_index_scriptobject_t engine_ScriptObjectGetIndexScriptObject;
static apptracker_set_world_matrix_inverse_t real_AppTracker_SetWorldMatrixInverse;
static apptracker_set_world_matrix_inverse_t tramp_AppTracker_SetWorldMatrixInverse;
static config_editor_param_change_t real_ConfigEditor_ParamChange;
static person_context_rebuild_t real_PersonContext_Rebuild;
static app_main_command_t real_AppMain_Command;
static stringref_hash32_t engine_StringRefHash32;
static namehash_find_t engine_NameHashFind;
static object_i_name_set_t real_Object_iNameSet;
static object_i_name_set_t tramp_Object_iNameSet;
static clone_object_t real_CloneObject;
static clone_object_t tramp_CloneObject;
static clone_node_t real_CloneNode;
static clone_node_t tramp_CloneNode;
static update_traverse_t real_UpdateTraverse;
static update_traverse_t tramp_UpdateTraverse;
static appbase_process_animation_t real_AppBase_ProcessAnimation;
static appbase_process_animation_t tramp_AppBase_ProcessAnimation;
static runtime_rotation_vector_write_t real_RuntimeRotationVectorWrite;
static runtime_rotation_vector_write_t tramp_RuntimeRotationVectorWrite;
static int runtime_rotation_vector_write_hook_logged;
static script_vector3_set_property_t tramp_RuntimeJointRotationAxisWrite;
static int runtime_joint_rotation_axis_write_hook_logged;
static tbase_get_matrix_version_t engine_TBaseTransformGetMatrixVersion;
static tbase_set_matrix_version_t engine_TBaseTransformSetMatrixVersion;
static void ***engine_G_MasterIsMVTBL_ptr;
static void **runtime_ssimple_rotation_set_slot;
static void **runtime_sjoint_rotation_axis_set_slot;
static script_vector3_set_property_t
    real_SSimpleTransform_RotationSet;
static script_vector3_set_property_t real_SJoint_RotationAxisSet;
static int runtime_animation_member_setters_installed;
static int runtime_animation_member_setters_logged;
static void **addon_constraint_count_getter_slot;
static script_index_count_property_t real_AddonConstraintArrayCount;
static int addon_constraint_count_getter_installed;
static int addon_constraint_count_getter_logged;
/* type=object uses the bone solver's limits. This guard bypasses the native
   SSimpleTransform scene mask only for the synchronous PhysX output write. */
static volatile LONG addon_object_bone_limit_write_active;
static DWORD addon_object_bone_limit_write_thread;
static void **engine_G_NilWeakObjTarget_ptr;
static void **engine_G_NilObject_ptr;
static void **engine_G_NullArray_ptr;
static void **engine_ComponentArray_ptr;
static set_ts_node_name_t real_SetTSNodeName;
static poseedit_init_tracks_t real_PoseEdit_InitTracks;
static int poseedit_inittracks_hook_installed;
static int poseedit_inittracks_hook_logged;
static int config_editor_param_change_hook_installed;
static int config_editor_param_change_hook_logged;
static int person_context_rebuild_hook_installed;
static int person_context_rebuild_hook_logged;
static int app_main_command_hook_installed;
static int app_main_command_hook_logged;
static volatile LONG person_context_selected_person;
static void *person_context_last_context;
static int person_context_last_visibility = -1;
static int person_context_last_breasts_visibility = -1;
static int person_context_last_penis_visibility = -1;
static int person_context_last_testicle_visibility = -1;
static int person_context_last_butt_visibility = -1;
static void *captured_poseedit_this;
static void *captured_poseedit_editpose;
static int captured_poseedit_tracks_offset = POSEEDIT_TRACKS_OFFSET;
static int captured_poseedit_tracks_offset_detected;
static DWORD captured_poseedit_tick;
static unsigned int poseeditor_track_handoff_pending_mask;
static DWORD poseeditor_track_handoff_pending_tick;
static void *poseeditor_track_handoff_poseedit;
static void *poseeditor_track_handoff_editpose;
static int apptracker_world_matrix_inverse_inline_installed;
static float captured_camera_inverse[16];
static int captured_camera_inverse_valid;
static float camera_contamination_test_engine_camera[16];
static float camera_contamination_test_applied_camera[16];
static int camera_contamination_test_camera_valid;
static volatile LONG camera_contamination_test_d3d_view_overrides;
static DWORD camera_contamination_test_d3d_view_log_tick;
static LONG captured_camera_version;
static DWORD captured_camera_tick;
static DWORD captured_camera_change_tick;
/* Diagnostic only: distinguish camera rotation from translation. Gravity
   guards use the full camera version because engine samples can be stale. */
static LONG captured_camera_rotation_version;
static DWORD captured_camera_rotation_change_tick;
static DWORD last_camera_gate_log_tick;
static DWORD plugin_attach_tick;
static DWORD script_engine_late_attempt_tick;
static int script_engine_late_attempt_count;
static void *captured_app_base;
static void *captured_script_engine;
static int captured_script_engine_logged;
static named_node_t named_nodes[16384];
static int named_node_count;
static volatile LONG named_node_generation = 1;

/* Runtime target resolution is called from the animation/physics hot path.
   The matching root normally remains stable for the lifetime of a room, but
   the old implementation formatted and searched every captured root again on
   every call.  Cache only the successful root index, never the target object:
   each hit still goes through FindObjC and the normal nil-object validation,
   so object replacement and destruction retain the engine's behaviour. */
#define RUNTIME_EXACT_ROOT_HINT_SLOTS 64
#define RUNTIME_EXACT_ROOT_HINT_NAME 192
typedef struct runtime_exact_root_hint_t {
    DWORD hash;
    LONG generation;
    int root_index;
    char name[RUNTIME_EXACT_ROOT_HINT_NAME];
} runtime_exact_root_hint_t;
static __thread runtime_exact_root_hint_t
    runtime_exact_root_hints[RUNTIME_EXACT_ROOT_HINT_SLOTS];
static int tsnode_probe_count;
static DWORD addon_tsnode_window_until_tick;
static int addon_tsnode_window_count;
static int addon_tsnode_window_serial;
static char addon_tsnode_window_root[128];
static int tree_candidate_log_budget = 12;
static motion_probe_t motion_probes[128];
static int motion_probe_count;
static transform_probe_sample_t transform_probe_samples[64];
static int transform_probe_sample_count;
static axis_map_probe_sample_t axis_map_probe_samples[] = {
    { "root_pos", "root", 0x0e8, NULL, 0, {0,0,0}, {0,0,0} },
    { "root_pos_alt", "root", 0x3a8, NULL, 0, {0,0,0}, {0,0,0} },
    { "root_basis", "root", 0x0b0, NULL, 0, {0,0,0}, {0,0,0} },
    { "root_basis_alt", "root", 0x0a8, NULL, 0, {0,0,0}, {0,0,0} },
    { "joint01_basis", "penis_joint01", 0x078, NULL, 0, {0,0,0}, {0,0,0} },
    { "joint01_basis_alt", "penis_joint01", 0x0b0, NULL, 0, {0,0,0}, {0,0,0} },
    { "joint02_basis", "penis_joint02", 0x078, NULL, 0, {0,0,0}, {0,0,0} },
    { "joint02_basis_alt", "penis_joint02", 0x0b0, NULL, 0, {0,0,0}, {0,0,0} },
    { "joint03_basis", "penis_joint03", 0x0b0, NULL, 0, {0,0,0}, {0,0,0} },
    { "joint03_pos", "penis_joint03", 0x0e8, NULL, 0, {0,0,0}, {0,0,0} },
    { "Sroot_final", "Sroot", 0x388, NULL, 0, {0,0,0}, {0,0,0} },
    { "Sroot_final_alt", "Sroot", 0x208, NULL, 0, {0,0,0}, {0,0,0} },
    { "Spenis01_final", "Spenis_joint01", 0x06c, NULL, 0, {0,0,0}, {0,0,0} },
    { "Spenis01_final_yzx", "Spenis_joint01", 0x070, NULL, 0, {0,0,0}, {0,0,0} },
    { "Spenis02_rot", "Spenis_joint02", 0x0d8, NULL, 0, {0,0,0}, {0,0,0} },
    { "Spenis02_final", "Spenis_joint02", 0x06c, NULL, 0, {0,0,0}, {0,0,0} },
    { "Spenis02_final_x", "Spenis_joint02", 0x074, NULL, 0, {0,0,0}, {0,0,0} },
    { "Spenis03_rot", "Spenis_joint03", 0x0d8, NULL, 0, {0,0,0}, {0,0,0} },
    { "Spenis03_final", "Spenis_joint03", 0x06c, NULL, 0, {0,0,0}, {0,0,0} },
    { "Spenis03_final_x", "Spenis_joint03", 0x074, NULL, 0, {0,0,0}, {0,0,0} }
};
static axis_root_scan_t axis_root_scan;
static const write_sweep_candidate_t write_sweep_candidates[] = {
    { "Spenis_joint01", 0x06c, 0, 20.0f, "Sjoint01 raw 06c x" },
    { "Spenis_joint01", 0x06c, 1, 20.0f, "Sjoint01 raw 06c y" },
    { "Spenis_joint01", 0x06c, 2, 20.0f, "Sjoint01 raw 06c z" },
    { "Spenis_joint02", 0x06c, 0, 20.0f, "Sjoint02 raw 06c x" },
    { "Spenis_joint02", 0x06c, 1, 20.0f, "Sjoint02 raw 06c y" },
    { "Spenis_joint02", 0x06c, 2, 20.0f, "Sjoint02 raw 06c z" },
    { "Spenis_joint03", 0x06c, 0, 20.0f, "Sjoint03 raw 06c x" },
    { "Spenis_joint03", 0x06c, 1, 20.0f, "Sjoint03 raw 06c y" },
    { "Spenis_joint03", 0x06c, 2, 20.0f, "Sjoint03 raw 06c z" }
};
#define COLLISION_AUTO_UPRIGHT(TEST_AXIS, TEST_SIGN, LABEL) \
    { "Stesticles_joint01", NULL, 0x06c, TEST_AXIS, TEST_SIGN, 0, 0, \
      NULL, 0, 0, 0.0f, LABEL }
#define COLLISION_AUTO_COMBINED(ROOT_AXIS, ROOT_SIGN, TEST_AXIS, TEST_SIGN, LABEL) \
    { "root", NULL, 0, ROOT_AXIS, ROOT_SIGN, 1, 1, \
      "Stesticles_joint01", 0x06c, TEST_AXIS, TEST_SIGN, LABEL }

static const collision_auto_test_step_t collision_auto_test_steps[] = {
    COLLISION_AUTO_UPRIGHT(0,  1.0f, "upright testicle x positive"),
    COLLISION_AUTO_UPRIGHT(0, -1.0f, "upright testicle x negative"),
    COLLISION_AUTO_UPRIGHT(1,  1.0f, "upright testicle y positive"),
    COLLISION_AUTO_UPRIGHT(1, -1.0f, "upright testicle y negative"),
    COLLISION_AUTO_UPRIGHT(2,  1.0f, "upright testicle z positive"),
    COLLISION_AUTO_UPRIGHT(2, -1.0f, "upright testicle z negative"),

    COLLISION_AUTO_COMBINED(0,  1.0f, 0,  1.0f, "root x positive plus testicle x positive"),
    COLLISION_AUTO_COMBINED(0,  1.0f, 0, -1.0f, "root x positive plus testicle x negative"),
    COLLISION_AUTO_COMBINED(0,  1.0f, 1,  1.0f, "root x positive plus testicle y positive"),
    COLLISION_AUTO_COMBINED(0,  1.0f, 1, -1.0f, "root x positive plus testicle y negative"),
    COLLISION_AUTO_COMBINED(0,  1.0f, 2,  1.0f, "root x positive plus testicle z positive"),
    COLLISION_AUTO_COMBINED(0,  1.0f, 2, -1.0f, "root x positive plus testicle z negative"),

    COLLISION_AUTO_COMBINED(0, -1.0f, 0,  1.0f, "root x negative plus testicle x positive"),
    COLLISION_AUTO_COMBINED(0, -1.0f, 0, -1.0f, "root x negative plus testicle x negative"),
    COLLISION_AUTO_COMBINED(0, -1.0f, 1,  1.0f, "root x negative plus testicle y positive"),
    COLLISION_AUTO_COMBINED(0, -1.0f, 1, -1.0f, "root x negative plus testicle y negative"),
    COLLISION_AUTO_COMBINED(0, -1.0f, 2,  1.0f, "root x negative plus testicle z positive"),
    COLLISION_AUTO_COMBINED(0, -1.0f, 2, -1.0f, "root x negative plus testicle z negative"),

    COLLISION_AUTO_COMBINED(1,  1.0f, 0,  1.0f, "root y positive plus testicle x positive"),
    COLLISION_AUTO_COMBINED(1,  1.0f, 0, -1.0f, "root y positive plus testicle x negative"),
    COLLISION_AUTO_COMBINED(1,  1.0f, 1,  1.0f, "root y positive plus testicle y positive"),
    COLLISION_AUTO_COMBINED(1,  1.0f, 1, -1.0f, "root y positive plus testicle y negative"),
    COLLISION_AUTO_COMBINED(1,  1.0f, 2,  1.0f, "root y positive plus testicle z positive"),
    COLLISION_AUTO_COMBINED(1,  1.0f, 2, -1.0f, "root y positive plus testicle z negative"),

    COLLISION_AUTO_COMBINED(1, -1.0f, 0,  1.0f, "root y negative plus testicle x positive"),
    COLLISION_AUTO_COMBINED(1, -1.0f, 0, -1.0f, "root y negative plus testicle x negative"),
    COLLISION_AUTO_COMBINED(1, -1.0f, 1,  1.0f, "root y negative plus testicle y positive"),
    COLLISION_AUTO_COMBINED(1, -1.0f, 1, -1.0f, "root y negative plus testicle y negative"),
    COLLISION_AUTO_COMBINED(1, -1.0f, 2,  1.0f, "root y negative plus testicle z positive"),
    COLLISION_AUTO_COMBINED(1, -1.0f, 2, -1.0f, "root y negative plus testicle z negative"),

    COLLISION_AUTO_COMBINED(2,  1.0f, 0,  1.0f, "root z positive plus testicle x positive"),
    COLLISION_AUTO_COMBINED(2,  1.0f, 0, -1.0f, "root z positive plus testicle x negative"),
    COLLISION_AUTO_COMBINED(2,  1.0f, 1,  1.0f, "root z positive plus testicle y positive"),
    COLLISION_AUTO_COMBINED(2,  1.0f, 1, -1.0f, "root z positive plus testicle y negative"),
    COLLISION_AUTO_COMBINED(2,  1.0f, 2,  1.0f, "root z positive plus testicle z positive"),
    COLLISION_AUTO_COMBINED(2,  1.0f, 2, -1.0f, "root z positive plus testicle z negative"),

    COLLISION_AUTO_COMBINED(2, -1.0f, 0,  1.0f, "root z negative plus testicle x positive"),
    COLLISION_AUTO_COMBINED(2, -1.0f, 0, -1.0f, "root z negative plus testicle x negative"),
    COLLISION_AUTO_COMBINED(2, -1.0f, 1,  1.0f, "root z negative plus testicle y positive"),
    COLLISION_AUTO_COMBINED(2, -1.0f, 1, -1.0f, "root z negative plus testicle y negative"),
    COLLISION_AUTO_COMBINED(2, -1.0f, 2,  1.0f, "root z negative plus testicle z positive"),
    COLLISION_AUTO_COMBINED(2, -1.0f, 2, -1.0f, "root z negative plus testicle z negative")
};

#undef COLLISION_AUTO_COMBINED
#undef COLLISION_AUTO_UPRIGHT

#define COLLISION_AUTO_ROOT_ONLY(ROOT_AXIS, ROOT_SIGN, LABEL) \
    { "root", NULL, 0, ROOT_AXIS, ROOT_SIGN, 1, 1, \
      NULL, 0, 0, 0.0f, LABEL }
#define COLLISION_AUTO_LIMB(NODE, FALLBACK_NODE, LIMB_AXIS, LIMB_SIGN, LABEL) \
    { NODE, FALLBACK_NODE, 0x06c, LIMB_AXIS, LIMB_SIGN, 0, 0, \
      NULL, 0, 0, 0.0f, LABEL }
#define COLLISION_AUTO_ROOT_LIMB(ROOT_AXIS, ROOT_SIGN, NODE, LIMB_AXIS, LIMB_SIGN, LABEL) \
    { "root", NULL, 0, ROOT_AXIS, ROOT_SIGN, 1, 1, \
      NODE, 0x06c, LIMB_AXIS, LIMB_SIGN, LABEL }

static const collision_auto_test_step_t collision_auto_test_hip_steps[] = {
    COLLISION_AUTO_LIMB("Ship_R_joint", "hip_R_joint", 0,  1.0f, "upright hip_R x positive"),
    COLLISION_AUTO_LIMB("Ship_R_joint", "hip_R_joint", 0, -1.0f, "upright hip_R x negative"),
    COLLISION_AUTO_LIMB("Ship_R_joint", "hip_R_joint", 1,  1.0f, "upright hip_R y positive"),
    COLLISION_AUTO_LIMB("Ship_R_joint", "hip_R_joint", 1, -1.0f, "upright hip_R y negative"),
    COLLISION_AUTO_LIMB("Ship_R_joint", "hip_R_joint", 2,  1.0f, "upright hip_R z positive"),
    COLLISION_AUTO_LIMB("Ship_R_joint", "hip_R_joint", 2, -1.0f, "upright hip_R z negative"),
    COLLISION_AUTO_LIMB("Ship_L_joint", "hip_L_joint", 0,  1.0f, "upright hip_L x positive"),
    COLLISION_AUTO_LIMB("Ship_L_joint", "hip_L_joint", 0, -1.0f, "upright hip_L x negative"),
    COLLISION_AUTO_LIMB("Ship_L_joint", "hip_L_joint", 1,  1.0f, "upright hip_L y positive"),
    COLLISION_AUTO_LIMB("Ship_L_joint", "hip_L_joint", 1, -1.0f, "upright hip_L y negative"),
    COLLISION_AUTO_LIMB("Ship_L_joint", "hip_L_joint", 2,  1.0f, "upright hip_L z positive"),
    COLLISION_AUTO_LIMB("Ship_L_joint", "hip_L_joint", 2, -1.0f, "upright hip_L z negative"),

    COLLISION_AUTO_LIMB("Sknee_R_joint", "knee_R_joint", 0,  1.0f, "upright knee_R x positive"),
    COLLISION_AUTO_LIMB("Sknee_R_joint", "knee_R_joint", 0, -1.0f, "upright knee_R x negative"),
    COLLISION_AUTO_LIMB("Sknee_R_joint", "knee_R_joint", 1,  1.0f, "upright knee_R y positive"),
    COLLISION_AUTO_LIMB("Sknee_R_joint", "knee_R_joint", 1, -1.0f, "upright knee_R y negative"),
    COLLISION_AUTO_LIMB("Sknee_R_joint", "knee_R_joint", 2,  1.0f, "upright knee_R z positive"),
    COLLISION_AUTO_LIMB("Sknee_R_joint", "knee_R_joint", 2, -1.0f, "upright knee_R z negative"),
    COLLISION_AUTO_LIMB("Sknee_L_joint", "knee_L_joint", 0,  1.0f, "upright knee_L x positive"),
    COLLISION_AUTO_LIMB("Sknee_L_joint", "knee_L_joint", 0, -1.0f, "upright knee_L x negative"),
    COLLISION_AUTO_LIMB("Sknee_L_joint", "knee_L_joint", 1,  1.0f, "upright knee_L y positive"),
    COLLISION_AUTO_LIMB("Sknee_L_joint", "knee_L_joint", 1, -1.0f, "upright knee_L y negative"),
    COLLISION_AUTO_LIMB("Sknee_L_joint", "knee_L_joint", 2,  1.0f, "upright knee_L z positive"),
    COLLISION_AUTO_LIMB("Sknee_L_joint", "knee_L_joint", 2, -1.0f, "upright knee_L z negative"),

    COLLISION_AUTO_ROOT_ONLY(0,  1.0f, "root x positive hip capsule sweep"),
    COLLISION_AUTO_ROOT_ONLY(0, -1.0f, "root x negative hip capsule sweep"),
    COLLISION_AUTO_ROOT_ONLY(1,  1.0f, "root y positive hip capsule sweep"),
    COLLISION_AUTO_ROOT_ONLY(1, -1.0f, "root y negative hip capsule sweep"),
    COLLISION_AUTO_ROOT_ONLY(2,  1.0f, "root z positive hip capsule sweep"),
    COLLISION_AUTO_ROOT_ONLY(2, -1.0f, "root z negative hip capsule sweep"),

    COLLISION_AUTO_ROOT_LIMB(0,  1.0f, "Ship_R_joint", 1,  1.0f, "root x positive plus hip_R y positive"),
    COLLISION_AUTO_ROOT_LIMB(0,  1.0f, "Ship_R_joint", 1, -1.0f, "root x positive plus hip_R y negative"),
    COLLISION_AUTO_ROOT_LIMB(0,  1.0f, "Ship_L_joint", 1,  1.0f, "root x positive plus hip_L y positive"),
    COLLISION_AUTO_ROOT_LIMB(0,  1.0f, "Ship_L_joint", 1, -1.0f, "root x positive plus hip_L y negative"),
    COLLISION_AUTO_ROOT_LIMB(0, -1.0f, "Ship_R_joint", 1,  1.0f, "root x negative plus hip_R y positive"),
    COLLISION_AUTO_ROOT_LIMB(0, -1.0f, "Ship_R_joint", 1, -1.0f, "root x negative plus hip_R y negative"),
    COLLISION_AUTO_ROOT_LIMB(0, -1.0f, "Ship_L_joint", 1,  1.0f, "root x negative plus hip_L y positive"),
    COLLISION_AUTO_ROOT_LIMB(0, -1.0f, "Ship_L_joint", 1, -1.0f, "root x negative plus hip_L y negative")
};

#undef COLLISION_AUTO_ROOT_LIMB
#undef COLLISION_AUTO_LIMB
#undef COLLISION_AUTO_ROOT_ONLY

static int collision_auto_test_is_hip_mode(void)
{
    return _stricmp(collision_auto_test_cfg.mode, "hip") == 0 ||
           _stricmp(collision_auto_test_cfg.mode, "hips") == 0 ||
           _stricmp(collision_auto_test_cfg.mode, "limb") == 0 ||
           _stricmp(collision_auto_test_cfg.mode, "limbs") == 0;
}

static const collision_auto_test_step_t *collision_auto_test_active_steps(int *count)
{
    if (collision_auto_test_is_hip_mode()) {
        if (count) {
            *count = (int)(sizeof(collision_auto_test_hip_steps) /
                           sizeof(collision_auto_test_hip_steps[0]));
        }
        return collision_auto_test_hip_steps;
    }
    if (count) {
        *count = (int)(sizeof(collision_auto_test_steps) /
                       sizeof(collision_auto_test_steps[0]));
    }
    return collision_auto_test_steps;
}

static float collision_auto_test_channel_amount(void)
{
    return collision_auto_test_is_hip_mode() ?
        collision_auto_test_cfg.hip_amount :
        collision_auto_test_cfg.testicle_amount;
}

static DWORD last_body_probe_tick;
static body_chain_probe_t body_chain_probe[] = {
#define BODY_PERSON_PROBES(P) \
    { P "Anim", "", NULL, NULL, NULL, 0, 0, 0 }, \
    { P "Anim:Model01:TRS_group", "", NULL, NULL, NULL, 0, 0, 0 }, \
    { P "Anim:Model01:STRS_group", "", NULL, NULL, NULL, 0, 0, 0 }, \
    { P "Anim:Model01:TRS_armature", "", NULL, NULL, NULL, 0, 0, 0 }, \
    { P "Anim:Model01:root", "", NULL, NULL, NULL, 0, 0, 0 }, \
    { P "Anim:Model01:Sroot", "", NULL, NULL, NULL, 0, 0, 0 }, \
    { P "Anim:Model01:hip_L_joint", "", NULL, NULL, NULL, 0, 0, 0 }, \
    { P "Anim:Model01:hip_R_joint", "", NULL, NULL, NULL, 0, 0, 0 }, \
    { P "Anim:Model01:knee_L_joint", "", NULL, NULL, NULL, 0, 0, 0 }, \
    { P "Anim:Model01:knee_R_joint", "", NULL, NULL, NULL, 0, 0, 0 }, \
    { P "Anim:Model01:Ship_L_joint", "", NULL, NULL, NULL, 0, 0, 0 }, \
    { P "Anim:Model01:Ship_R_joint", "", NULL, NULL, NULL, 0, 0, 0 }, \
    { P "Anim:Model01:Sknee_L_joint", "", NULL, NULL, NULL, 0, 0, 0 }, \
    { P "Anim:Model01:Sknee_R_joint", "", NULL, NULL, NULL, 0, 0, 0 }, \
    { P "Anim:Model01:testicles_joint01", "", NULL, NULL, NULL, 0, 0, 0 }, \
    { P "Anim:Model01:testicles_joint02", "", NULL, NULL, NULL, 0, 0, 0 }, \
    { P "Anim:Model01:penis_joint01", "", NULL, NULL, NULL, 0, 0, 0 }, \
    { P "Anim:Model01:penis_joint02", "", NULL, NULL, NULL, 0, 0, 0 }, \
    { P "Anim:Model01:penis_joint03", "", NULL, NULL, NULL, 0, 0, 0 }, \
    { P "Anim:Model01:penis_jointEnd", "", NULL, NULL, NULL, 0, 0, 0 }, \
    { P "Anim:Model01:Spenis_joint01", "", NULL, NULL, NULL, 0, 0, 0 }, \
    { P "Anim:Model01:Spenis_joint02", "", NULL, NULL, NULL, 0, 0, 0 }, \
    { P "Anim:Model01:Spenis_joint03", "", NULL, NULL, NULL, 0, 0, 0 }, \
    { P "Anim:Model01:Spenis_jointEnd", "", NULL, NULL, NULL, 0, 0, 0 }
    BODY_PERSON_PROBES("Person01"),
    BODY_PERSON_PROBES("Person02"),
    BODY_PERSON_PROBES("Person03"),
    BODY_PERSON_PROBES("Person04")
#undef BODY_PERSON_PROBES
};
static void patch_module(HMODULE mod);
static void patch_all_modules(void);
static void patch_d3d8_object(IDirect3D8 *d3d);
static void patch_d3d8_device(IDirect3DDevice8 *dev);
static void sibling_file_path(const char *filename, char *out, size_t outsz);
static void reset_body_chain_physics(void);
static void reset_breasts_physics_person_state(
    int person_index, breasts_physics_person_state_t *state,
    int restore_output);
static void reset_butt_physics_person_state(
    int person_index, breasts_physics_person_state_t *state,
    int restore_output);
static void reset_body_chain_collider_states(void);

static int normal_log_starts_with(const char *text, const char *prefix)
{
    size_t prefix_len;
    if (!text || !prefix) return 0;
    prefix_len = strlen(prefix);
    return _strnicmp(text, prefix, prefix_len) == 0;
}

static int normal_log_line_allowed(const char *fmt)
{
    if (!fmt || !fmt[0]) return 0;
    if (defaults_cfg.debug) return 1;
    if (defaults_cfg.performance_profile &&
        normal_log_starts_with(fmt, "performance profile ")) {
        return 1;
    }

    /* Normal logs are intentionally small: lifecycle, user-facing binding,
       reload results, and actionable failures only. */
    if (normal_log_starts_with(fmt, "NC-TK17-PhysX.dll ") ||
        strcmp(fmt, "on_create") == 0 ||
        normal_log_starts_with(fmt, "addon physics active ") ||
        normal_log_starts_with(fmt, "addon safety ") ||
        normal_log_starts_with(fmt, "sidecar loaded ") ||
        normal_log_starts_with(fmt, "addon sidecar reloaded ") ||
        normal_log_starts_with(fmt, "addon sidecar hot-reloaded ") ||
        normal_log_starts_with(fmt, "body-profile sidecar active ") ||
        normal_log_starts_with(fmt, "body-profile sidecar fallback ") ||
        normal_log_starts_with(fmt, "body-profile sidecar removed ") ||
        normal_log_starts_with(fmt, "body-profile sidecar reload ") ||
        normal_log_starts_with(fmt, "settings write failed ") ||
        normal_log_starts_with(fmt, "settings ignored ") ||
        normal_log_starts_with(fmt, "gravity responsiveness ") ||
        normal_log_starts_with(fmt, "single-bone contact ")) {
        return 1;
    }

    return strstr(fmt, " failed") != NULL ||
           strstr(fmt, " error") != NULL ||
           strstr(fmt, " unsupported") != NULL ||
           strstr(fmt, " overflow") != NULL ||
           strstr(fmt, " table full") != NULL;
}

static void log_line(const char *fmt, ...)
{
    char path[MAX_PATH * 4];
    FILE *f;
    va_list ap;
    if (!log_ready || !normal_log_line_allowed(fmt)) return;
    path[0] = 0;
    if (self_module) {
        GetModuleFileNameA(self_module, path, sizeof(path));
        {
            char *slash = strrchr(path, '\\');
            if (slash) slash[1] = 0;
        }
        lstrcatA(path, "..\\Logs\\NC-TK17-PhysX.log");
    } else {
        lstrcpynA(path, "Logs\\NC-TK17-PhysX.log", sizeof(path));
    }
    EnterCriticalSection(&log_lock);
    f = fopen(path, "ab");
    if (f) {
        SYSTEMTIME st;
        GetLocalTime(&st);
        fprintf(f, "%04u-%02u-%02u %02u:%02u:%02u.%03u ",
                st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond, st.wMilliseconds);
        va_start(ap, fmt);
        vfprintf(f, fmt, ap);
        va_end(ap);
        fputs("\r\n", f);
        fclose(f);
    }
    LeaveCriticalSection(&log_lock);
}

/* Temporary trace for the pose/undo test. Normal logging is capped at 600 rows
   over three minutes; explicit debug mode keeps the throttled trace available. */
static int gravity_response_trace_due(DWORD now, DWORD *last)
{
    static DWORD start;
    static unsigned int count;
    static int started;
    if (!started) { start = now; started = 1; }
    if ((!defaults_cfg.debug && (count >= 600u || now - start > 180000u)) ||
        (*last && now - *last < 1000u)) return 0;
    *last = now;
    if (count < 600u) count++;
    return 1;
}

static void physx_perf_prepare(DWORD now)
{
    if (!defaults_cfg.performance_profile) {
        if (physx_perf_state.ready) {
            memset(&physx_perf_state, 0, sizeof(physx_perf_state));
        }
        return;
    }
    if (!physx_perf_state.ready) {
        memset(&physx_perf_state, 0, sizeof(physx_perf_state));
        if (!QueryPerformanceFrequency(&physx_perf_state.frequency) ||
            physx_perf_state.frequency.QuadPart <= 0) {
            return;
        }
        physx_perf_state.report_tick = now;
        physx_perf_state.ready = 1;
    }
}

static LONGLONG physx_perf_counter(void)
{
    LARGE_INTEGER value;
    if (!defaults_cfg.performance_profile || !physx_perf_state.ready) return 0;
    QueryPerformanceCounter(&value);
    return value.QuadPart;
}

static void physx_perf_add(physx_perf_phase_t phase, LONGLONG start)
{
    LARGE_INTEGER end;
    LONGLONG elapsed;
    if (!start || phase < 0 || phase >= PHYSX_PERF_PHASE_COUNT ||
        !defaults_cfg.performance_profile || !physx_perf_state.ready) {
        return;
    }
    QueryPerformanceCounter(&end);
    elapsed = end.QuadPart - start;
    if (elapsed < 0) return;
    physx_perf_state.accumulated[phase] += elapsed;
    if (elapsed > physx_perf_state.maximum[phase]) {
        physx_perf_state.maximum[phase] = elapsed;
    }
    physx_perf_state.calls[phase]++;
}

static double physx_perf_ms(LONGLONG ticks)
{
    if (!physx_perf_state.ready || physx_perf_state.frequency.QuadPart <= 0) {
        return 0.0;
    }
    return (double)ticks * 1000.0 /
           (double)physx_perf_state.frequency.QuadPart;
}

static double physx_perf_average_per_frame(physx_perf_phase_t phase,
                                           DWORD frames)
{
    if (!frames) return 0.0;
    return physx_perf_ms(physx_perf_state.accumulated[phase]) /
           (double)frames;
}

static void physx_perf_report(DWORD now)
{
    DWORD frames;
    DWORD window_ms;
    if (!defaults_cfg.performance_profile || !physx_perf_state.ready) return;
    window_ms = now - physx_perf_state.report_tick;
    if (window_ms < 1000u) return;
    frames = physx_perf_state.calls[PHYSX_PERF_TOTAL];
    if (frames) {
        log_line("performance profile window_ms=%lu frames=%lu avg_ms total=%.3f housekeeping=%.3f bindings=%.3f diagnostics=%.3f colliders=%.3f axes=%.3f penis=%.3f penis_ownership=%.3f penis_gravity=%.3f penis_collision=%.3f penis_setup=%.3f penis_orientation=%.3f penis_translation=%.3f penis_solver=%.3f testicles=%.3f testicle_active=%.3f breasts=%.3f breasts_active=%.3f butt=%.3f butt_active=%.3f addons=%.3f addon_activation=%.3f addon_drive=%.3f addon_guard=%.3f addon_parent_pivot=%.3f addon_parent_translation=%.3f addon_parent_rotation=%.3f addon_gravity=%.3f addon_rotation_binding=%.3f addon_validation=%.3f addon_solver=%.3f addon_output=%.3f addon_cache=%.3f late=%.3f late_testicles=%.3f late_breasts=%.3f late_butt=%.3f traverse=%.3f addon_collision_body=%.3f addon_collision_self=%.3f addon_collision_addons=%.3f max_ms total=%.3f colliders=%.3f addons=%.3f calls late=%lu traverse=%lu body_collision=%lu self_collision=%lu addons_collision=%lu",
                 (unsigned long)window_ms,
                 (unsigned long)frames,
                 physx_perf_average_per_frame(PHYSX_PERF_TOTAL, frames),
                 physx_perf_average_per_frame(PHYSX_PERF_HOUSEKEEPING, frames),
                 physx_perf_average_per_frame(PHYSX_PERF_BINDINGS, frames),
                 physx_perf_average_per_frame(PHYSX_PERF_DIAGNOSTICS, frames),
                 physx_perf_average_per_frame(PHYSX_PERF_COLLIDER_REFRESH, frames),
                 physx_perf_average_per_frame(PHYSX_PERF_AXIS_REFERENCES, frames),
                 physx_perf_average_per_frame(PHYSX_PERF_PENIS_PHYSICS, frames),
                 physx_perf_average_per_frame(PHYSX_PERF_PENIS_OWNERSHIP, frames),
                 physx_perf_average_per_frame(PHYSX_PERF_PENIS_GRAVITY, frames),
                 physx_perf_average_per_frame(PHYSX_PERF_PENIS_COLLISION, frames),
                 physx_perf_average_per_frame(PHYSX_PERF_PENIS_SETUP, frames),
                 physx_perf_average_per_frame(PHYSX_PERF_PENIS_ORIENTATION, frames),
                 physx_perf_average_per_frame(PHYSX_PERF_PENIS_TRANSLATION, frames),
                 physx_perf_average_per_frame(PHYSX_PERF_PENIS_SOLVER, frames),
                 physx_perf_average_per_frame(PHYSX_PERF_TESTICLE_PHYSICS, frames),
                 physx_perf_average_per_frame(PHYSX_PERF_TESTICLE_ACTIVE, frames),
                 physx_perf_average_per_frame(PHYSX_PERF_BREASTS_PHYSICS, frames),
                 physx_perf_average_per_frame(PHYSX_PERF_BREASTS_ACTIVE, frames),
                 physx_perf_average_per_frame(PHYSX_PERF_BUTT_PHYSICS, frames),
                 physx_perf_average_per_frame(PHYSX_PERF_BUTT_ACTIVE, frames),
                 physx_perf_average_per_frame(PHYSX_PERF_ADDON_SIMULATION, frames),
                 physx_perf_average_per_frame(PHYSX_PERF_ADDON_ACTIVATION, frames),
                 physx_perf_average_per_frame(PHYSX_PERF_ADDON_DRIVE, frames),
                 physx_perf_average_per_frame(PHYSX_PERF_ADDON_GUARD, frames),
                 physx_perf_average_per_frame(PHYSX_PERF_ADDON_PARENT_PIVOT, frames),
                 physx_perf_average_per_frame(PHYSX_PERF_ADDON_PARENT_TRANSLATION, frames),
                 physx_perf_average_per_frame(PHYSX_PERF_ADDON_PARENT_ROTATION, frames),
                 physx_perf_average_per_frame(PHYSX_PERF_ADDON_GRAVITY, frames),
                 physx_perf_average_per_frame(PHYSX_PERF_ADDON_ROTATION_BINDING, frames),
                 physx_perf_average_per_frame(PHYSX_PERF_ADDON_VALIDATION, frames),
                 physx_perf_average_per_frame(PHYSX_PERF_ADDON_SOLVER, frames),
                 physx_perf_average_per_frame(PHYSX_PERF_ADDON_OUTPUT, frames),
                 physx_perf_average_per_frame(PHYSX_PERF_ADDON_SUPPRESSION_CACHE, frames),
                 physx_perf_average_per_frame(PHYSX_PERF_LATE_OWNERSHIP, frames),
                 physx_perf_average_per_frame(PHYSX_PERF_LATE_TESTICLES, frames),
                 physx_perf_average_per_frame(PHYSX_PERF_LATE_BREASTS, frames),
                 physx_perf_average_per_frame(PHYSX_PERF_LATE_BUTT, frames),
                 physx_perf_average_per_frame(PHYSX_PERF_TRAVERSE_OVERLAY, frames),
                 physx_perf_average_per_frame(PHYSX_PERF_ADDON_BODY_COLLISION, frames),
                 physx_perf_average_per_frame(PHYSX_PERF_ADDON_SELF_COLLISION, frames),
                 physx_perf_average_per_frame(PHYSX_PERF_ADDON_ADDONS_COLLISION, frames),
                 physx_perf_ms(physx_perf_state.maximum[PHYSX_PERF_TOTAL]),
                 physx_perf_ms(physx_perf_state.maximum[PHYSX_PERF_COLLIDER_REFRESH]),
                 physx_perf_ms(physx_perf_state.maximum[PHYSX_PERF_ADDON_SIMULATION]),
                 (unsigned long)physx_perf_state.calls[PHYSX_PERF_LATE_OWNERSHIP],
                 (unsigned long)physx_perf_state.calls[PHYSX_PERF_TRAVERSE_OVERLAY],
                 (unsigned long)physx_perf_state.calls[PHYSX_PERF_ADDON_BODY_COLLISION],
                 (unsigned long)physx_perf_state.calls[PHYSX_PERF_ADDON_SELF_COLLISION],
                 (unsigned long)physx_perf_state.calls[PHYSX_PERF_ADDON_ADDONS_COLLISION]);
    }
    memset(physx_perf_state.accumulated, 0,
           sizeof(physx_perf_state.accumulated));
    memset(physx_perf_state.maximum, 0,
           sizeof(physx_perf_state.maximum));
    memset(physx_perf_state.calls, 0, sizeof(physx_perf_state.calls));
    physx_perf_state.report_tick = now;
}

static int body_chain_copy_gravity_snapshot_from_state(
    const body_chain_person_state_t *state,
    body_chain_gravity_snapshot_t *snapshot,
    void *current_root_raw)
{
    if (!snapshot) return 0;
    memset(snapshot, 0, sizeof(*snapshot));
    if (!state ||
        !state->initialized ||
        !state->root_raw ||
        state->root_raw != current_root_raw ||
        !state->gravity_probe_captured ||
        !state->gravity_probe_promoted ||
        !state->gravity_drive_ref_valid ||
        !physics_environment_cfg.gravity_apply_to_body_chain ||
        !physics_environment_cfg.gravity_zero_at_start) {
        return 0;
    }
    snapshot->valid = 1;
    snapshot->root_raw = state->root_raw;
    snapshot->gravity_probe_captured = state->gravity_probe_captured;
    snapshot->gravity_probe_promoted = state->gravity_probe_promoted;
    snapshot->gravity_probe_sampled = state->gravity_probe_sampled;
    memcpy(snapshot->gravity_probe_root, state->gravity_probe_root,
           sizeof(snapshot->gravity_probe_root));
    memcpy(snapshot->gravity_probe_root_world, state->gravity_probe_root_world,
           sizeof(snapshot->gravity_probe_root_world));
    snapshot->gravity_probe_world_valid = state->gravity_probe_world_valid;
    memcpy(snapshot->gravity_drive, state->gravity_drive,
           sizeof(snapshot->gravity_drive));
    memcpy(snapshot->gravity_drive_filtered, state->gravity_drive_filtered,
           sizeof(snapshot->gravity_drive_filtered));
    snapshot->gravity_drive_filtered_valid =
        state->gravity_drive_filtered_valid;
    memcpy(snapshot->gravity_drive_ref, state->gravity_drive_ref,
           sizeof(snapshot->gravity_drive_ref));
    snapshot->gravity_drive_ref_valid = state->gravity_drive_ref_valid;
    snapshot->gravity_basis_stable_tick = state->gravity_basis_stable_tick;
    snapshot->gravity_basis_sampled = state->gravity_basis_sampled;
    memcpy(snapshot->gravity_basis_prev, state->gravity_basis_prev,
           sizeof(snapshot->gravity_basis_prev));
    return 1;
}

static void body_chain_capture_gravity_snapshot(
    const body_chain_person_state_t *state,
    body_chain_gravity_snapshot_t *snapshot,
    void *current_root_raw,
    DWORD now)
{
    if (!snapshot) return;
    memset(snapshot, 0, sizeof(*snapshot));
    if (!config_hot_reload_tick ||
        now - config_hot_reload_tick > 5000u) {
        return;
    }
    body_chain_copy_gravity_snapshot_from_state(state, snapshot, current_root_raw);
}

static int body_chain_restore_gravity_snapshot_ex(
    body_chain_person_state_t *state,
    const body_chain_gravity_snapshot_t *snapshot,
    const float current_root[3],
    DWORD now,
    const char *person,
    const char *system_name,
    const char *source,
    const char *note,
    int log_event)
{
    if (!state || !snapshot || !snapshot->valid || !current_root) return 0;
    state->gravity_probe_captured = snapshot->gravity_probe_captured;
    state->gravity_probe_promoted = snapshot->gravity_probe_promoted;
    /* Preserve the room's neutral pose, but reconfirm live sampling after a
       state handoff; a cached reference is not a current matrix sample. */
    memset(&state->gravity_sample, 0, sizeof(state->gravity_sample));
    memset(&state->geometry_sample, 0, sizeof(state->geometry_sample));
    state->gravity_probe_sampled = snapshot->gravity_probe_sampled;
    memcpy(state->gravity_probe_root, snapshot->gravity_probe_root,
           sizeof(state->gravity_probe_root));
    memcpy(state->gravity_probe_root_world, snapshot->gravity_probe_root_world,
           sizeof(state->gravity_probe_root_world));
    state->gravity_probe_world_valid = snapshot->gravity_probe_world_valid;
    memcpy(state->gravity_drive, snapshot->gravity_drive,
           sizeof(state->gravity_drive));
    memcpy(state->gravity_drive_filtered, snapshot->gravity_drive_filtered,
           sizeof(state->gravity_drive_filtered));
    state->gravity_drive_filtered_valid =
        snapshot->gravity_drive_filtered_valid;
    memcpy(state->gravity_drive_ref, snapshot->gravity_drive_ref,
           sizeof(state->gravity_drive_ref));
    state->gravity_drive_ref_valid = snapshot->gravity_drive_ref_valid;
    state->gravity_basis_stable_tick = snapshot->gravity_basis_stable_tick;
    state->gravity_basis_sampled = snapshot->gravity_basis_sampled;
    memcpy(state->gravity_basis_prev, snapshot->gravity_basis_prev,
           sizeof(state->gravity_basis_prev));
    state->gravity_probe_prev_root[0] = current_root[0];
    state->gravity_probe_prev_root[1] = current_root[1];
    state->gravity_probe_prev_root[2] = current_root[2];
    state->gravity_probe_last_root[0] = current_root[0];
    state->gravity_probe_last_root[1] = current_root[1];
    state->gravity_probe_last_root[2] = current_root[2];
    state->gravity_probe_candidate_tick = now;
    state->gravity_probe_stable_tick = now;
    state->gravity_camera_hold_active = 0;
    if (log_event) {
        log_line("physics-environment gravity-reference preserved person=\"%s\" system=\"%s\" source=\"%s\" root_raw=%p gravity_ref=(h=%.6f,v=%.6f,hs=%.6f) note=\"%s\"",
                 person ? person : "",
                 system_name ? system_name : "",
                 source ? source : "",
                 snapshot->root_raw,
                 state->gravity_drive_ref[0],
                 state->gravity_drive_ref[1],
                 state->gravity_drive_ref[2],
                 note ? note : "");
    }
    return 1;
}

static int body_chain_restore_gravity_snapshot(
    body_chain_person_state_t *state,
    const body_chain_gravity_snapshot_t *snapshot,
    const float current_root[3],
    DWORD now,
    const char *person,
    const char *system_name)
{
    return body_chain_restore_gravity_snapshot_ex(
        state, snapshot, current_root, now, person, system_name,
        "hot-reload",
        "INI hot-reload kept existing neutral gravity reference instead of recalibrating from the current pose",
        defaults_cfg.debug);
}

static void body_chain_cache_room_gravity_snapshot(
    body_chain_gravity_snapshot_t *cache,
    const body_chain_person_state_t *state,
    void *current_root_raw,
    const char *person,
    const char *system_name)
{
    body_chain_gravity_snapshot_t snapshot;
    int changed;
    if (!cache ||
        !body_chain_copy_gravity_snapshot_from_state(
            state, &snapshot, current_root_raw)) {
        return;
    }
    changed =
        !cache->valid ||
        cache->root_raw != snapshot.root_raw ||
        memcmp(cache->gravity_drive_ref,
               snapshot.gravity_drive_ref,
               sizeof(cache->gravity_drive_ref)) != 0 ||
        memcmp(cache->gravity_probe_root,
               snapshot.gravity_probe_root,
               sizeof(cache->gravity_probe_root)) != 0;
    *cache = snapshot;
    if (changed) {
        log_line("physics-environment gravity-reference cached person=\"%s\" system=\"%s\" root_raw=%p gravity_ref=(h=%.6f,v=%.6f,hs=%.6f) note=\"room/body gravity baseline will survive physics toggles and INI hot-reloads until the root changes\"",
                 person ? person : "",
                 system_name ? system_name : "",
                 cache->root_raw,
                 cache->gravity_drive_ref[0],
                 cache->gravity_drive_ref[1],
                 cache->gravity_drive_ref[2]);
    }
}

static int body_chain_restore_room_gravity_snapshot(
    body_chain_person_state_t *state,
    body_chain_gravity_snapshot_t *cache,
    void *current_root_raw,
    const float current_root[3],
    DWORD now,
    const char *person,
    const char *system_name)
{
    if (!cache || !cache->valid) return 0;
    if (!current_root_raw || cache->root_raw != current_root_raw) {
        log_line("physics-environment gravity-reference cache-invalidated person=\"%s\" system=\"%s\" cached_root_raw=%p current_root_raw=%p note=\"root/body changed, so the next valid room activation will capture a fresh gravity reference\"",
                 person ? person : "",
                 system_name ? system_name : "",
                 cache->root_raw,
                 current_root_raw);
        memset(cache, 0, sizeof(*cache));
        return 0;
    }
    if (!physics_environment_cfg.gravity_apply_to_body_chain ||
        !physics_environment_cfg.gravity_zero_at_start) {
        return 0;
    }
    return body_chain_restore_gravity_snapshot_ex(
        state, cache, current_root, now, person, system_name,
        "room-cache",
        "restored the room/body gravity baseline instead of recalibrating from the current pose",
        1);
}

static void clear_log_file_on_startup(void)
{
    char path[MAX_PATH * 4];
    FILE *f;
    path[0] = 0;
    if (self_module) {
        GetModuleFileNameA(self_module, path, sizeof(path));
        {
            char *slash = strrchr(path, '\\');
            if (slash) slash[1] = 0;
        }
        lstrcatA(path, "..\\Logs\\NC-TK17-PhysX.log");
    } else {
        lstrcpynA(path, "Logs\\NC-TK17-PhysX.log", sizeof(path));
    }
    f = fopen(path, "wb");
    if (f) fclose(f);
}

static void debug_line(const char *fmt, ...)
{
    char msg[1024];
    va_list ap;
    if (!defaults_cfg.debug) return;
    va_start(ap, fmt);
    _vsnprintf(msg, sizeof(msg), fmt, ap);
    msg[sizeof(msg) - 1] = 0;
    va_end(ap);
    log_line("%s", msg);
}

static int contains_i(const char *s, const char *needle)
{
    size_t n;
    if (!s || !needle) return 0;
    n = strlen(needle);
    if (!n) return 1;
    for (; *s; s++) {
        if (_strnicmp(s, needle, n) == 0) return 1;
    }
    return 0;
}

static int ends_with_i(const char *s, const char *suffix)
{
    size_t a, b;
    if (!s || !suffix) return 0;
    a = strlen(s);
    b = strlen(suffix);
    if (b > a) return 0;
    return _stricmp(s + a - b, suffix) == 0;
}

static void trim_in_place(char *s)
{
    char *e;
    if (!s) return;
    while (*s == ' ' || *s == '\t' || *s == '\r' || *s == '\n') memmove(s, s + 1, strlen(s));
    e = s + strlen(s);
    while (e > s && (e[-1] == ' ' || e[-1] == '\t' || e[-1] == '\r' || e[-1] == '\n')) *--e = 0;
}

/* VirtualQuery is comparatively expensive, and the physics paths validate the
   same engine allocations many times during one rendered frame.  Keep a tiny
   thread-local cache of positive VirtualQuery results for the current frame.
   The cache is advanced by the D3D8/OpenGL presentation hooks, so mappings are
   never trusted across rendered frames. */
#define PTR_READABLE_CACHE_SLOTS 256

typedef struct ptr_readable_cache_entry_t {
    BYTE *base;
    BYTE *end;
    LONG epoch;
} ptr_readable_cache_entry_t;

static volatile LONG ptr_readable_cache_epoch = 1;
static __thread ptr_readable_cache_entry_t
    ptr_readable_cache[PTR_READABLE_CACHE_SLOTS];

static void ptr_readable_cache_advance_frame(void)
{
    LONG next = InterlockedIncrement(&ptr_readable_cache_epoch);
    if (next == 0) InterlockedIncrement(&ptr_readable_cache_epoch);
}

static int ptr_readable(const void *p, size_t bytes)
{
    MEMORY_BASIC_INFORMATION mbi;
    BYTE *cur = (BYTE*)p;
    BYTE *end = cur + bytes;
    LONG epoch = InterlockedCompareExchange(&ptr_readable_cache_epoch, 0, 0);

    if (!p) return 0;
    if (end < cur) return 0;
    while (cur < end) {
        uintptr_t page = (uintptr_t)cur >> 12;
        uintptr_t cache_hash = page ^ (page >> 8) ^ (page >> 16);
        ptr_readable_cache_entry_t *entry =
            &ptr_readable_cache[cache_hash & (PTR_READABLE_CACHE_SLOTS - 1)];

        if (entry->epoch == epoch && cur >= entry->base && cur < entry->end) {
            cur = entry->end;
            continue;
        }
        if (!VirtualQuery(cur, &mbi, sizeof(mbi))) return 0;
        if (mbi.State != MEM_COMMIT || (mbi.Protect & (PAGE_NOACCESS | PAGE_GUARD))) return 0;
        entry->base = (BYTE*)mbi.BaseAddress;
        entry->end = entry->base + mbi.RegionSize;
        entry->epoch = epoch;
        cur = entry->end;
    }
    return 1;
}

static int safe_cstr_a(const char *s, size_t max)
{
    size_t i;
    if (!s || !ptr_readable(s, 1)) return 0;
    for (i = 0; i < max; i++) {
        unsigned char ch;
        if (!ptr_readable(s + i, 1)) return 0;
        ch = (unsigned char)s[i];
        if (!ch) return i > 0;
        if (ch < 32 || ch > 126) return 0;
    }
    return 0;
}

static const char *stringref_cstr_a(const void *ref)
{
    const char *direct;
    const char *indirect;
    if (!ref) return NULL;
    direct = (const char*)ref;
    if (safe_cstr_a(direct, 256)) return direct;
    if (ptr_readable(ref, sizeof(char*))) {
        indirect = *(const char* const*)ref;
        if (safe_cstr_a(indirect, 256)) return indirect;
    }
    return NULL;
}

static const void *stringref_from_cstr_a(const char *s)
{
    static unsigned char storage[512 + sizeof(int)];
    int len;
    if (!s) return NULL;
    len = (int)strlen(s);
    if (len <= 0 || len >= (int)(sizeof(storage) - sizeof(int))) return NULL;
    *(int*)storage = len;
    memcpy(storage + sizeof(int), s, (size_t)len + 1);
    return storage + sizeof(int);
}

static int get_file_write_time_a(const char *path, FILETIME *write_time)
{
    WIN32_FILE_ATTRIBUTE_DATA data;
    if (write_time) memset(write_time, 0, sizeof(*write_time));
    if (!path || !GetFileAttributesExA(path, GetFileExInfoStandard, &data)) return 0;
    if (write_time) *write_time = data.ftLastWriteTime;
    return 1;
}

static int filetime_differs(const FILETIME *a, const FILETIME *b)
{
    if (!a || !b) return 1;
    return a->dwLowDateTime != b->dwLowDateTime || a->dwHighDateTime != b->dwHighDateTime;
}

static void config_file_path(char *out, size_t outsz)
{
    out[0] = 0;
    if (self_module) {
        GetModuleFileNameA(self_module, out, (DWORD)outsz);
        {
            char *slash = strrchr(out, '\\');
            if (slash) slash[1] = 0;
        }
    }
    lstrcatA(out, "NC-TK17-PhysX.ini");
}

static void sibling_file_path(const char *filename, char *out, size_t outsz)
{
    out[0] = 0;
    if (config_path[0]) {
        lstrcpynA(out, config_path, (int)outsz);
        {
            char *slash = strrchr(out, '\\');
            if (slash) slash[1] = 0;
            else out[0] = 0;
        }
    } else if (self_module) {
        GetModuleFileNameA(self_module, out, (DWORD)outsz);
        {
            char *slash = strrchr(out, '\\');
            if (slash) slash[1] = 0;
        }
    }
    lstrcatA(out, filename);
}

static void *resolve_find_obj(const char *name, void **raw_out);
static void *resolve_script_engine_obj(const char *name, void **raw_out);
static void capture_script_engine(void *app_base, void *script_engine, const char *source);
static int is_nil_engine_object(void *raw, void *obj);

#include "physx_config.c"

static void resolve_engine_symbols(void)
{
    HMODULE sys;
    HMODULE app;
    if (engine_symbols_attempted && engine_FindObjC &&
        engine_GetModelViewRotationPivot &&
        real_AppTracker_SetWorldMatrixInverse &&
        tramp_AppBase_ProcessAnimation &&
        tramp_RuntimeRotationVectorWrite &&
        tramp_RuntimeJointRotationAxisWrite &&
        runtime_animation_member_setters_installed &&
        addon_constraint_count_getter_installed &&
        engine_StringRefHash32 && engine_NameHashFind) return;
    sys = GetModuleHandleA("ThriXXX010278-SYS.dll");
    app = GetModuleHandleA("ThriXXX010278-APP.dll");
    if (!sys && !app) return;
    engine_symbols_attempted = 1;
    if (app) {
        engine_FindObjC = (app_find_objc_t)GetProcAddress(app, "?FindObjC@AppMain@@YAPAVScriptObject@Bionic@@PBD@Z");
        engine_AppMainEngine = (app_engine_t)GetProcAddress(app, "?Engine@AppMain@@YAPAVScriptEngine@Bionic@@XZ");
        engine_AppMainUserMain = (app_user_main_t)GetProcAddress(
            app, "?UserMain@AppMain@@YAPAXXZ");
        real_AppBase_ProcessAnimation = real_AppBase_ProcessAnimation ?
            real_AppBase_ProcessAnimation :
            (appbase_process_animation_t)GetProcAddress(
                app,
                "?ProcessAnimation@AppBase@@QAE?AW4EResult@Bionic@@XZ");
        if (real_AppBase_ProcessAnimation &&
            !tramp_AppBase_ProcessAnimation) {
            if (install_inline_hook(
                    (void*)real_AppBase_ProcessAnimation,
                    (void*)hook_AppBase_ProcessAnimation,
                    5,
                    (void**)&tramp_AppBase_ProcessAnimation)) {
                log_line("runtime AppBase::ProcessAnimation hook installed target=%p trampoline=%p note=\"runtime-only body PhysX ownership; PoseEditor path is excluded\"",
                         (void*)real_AppBase_ProcessAnimation,
                         (void*)tramp_AppBase_ProcessAnimation);
            } else {
                log_line("runtime AppBase::ProcessAnimation hook not-installed target=%p reason=\"inline patch failed\"",
                         (void*)real_AppBase_ProcessAnimation);
            }
        }
        engine_SearchTree = (search_tree_t)GetProcAddress(app, "?SearchTree@Bionic@@YAPAVScriptObject@1@PAV21@ABVStringRef@1@PAV?$Array@V?$Obj@VScriptObject@Bionic@@@Bionic@@@1@@Z");
        engine_ComponentArray_ptr = (void**)GetProcAddress(app, "?ComponentArray@eAppModel@@3PBVClassMember_Index_ScriptObject@Bionic@@B");
        real_AppTracker_SetWorldMatrixInverse = real_AppTracker_SetWorldMatrixInverse ? real_AppTracker_SetWorldMatrixInverse :
            (apptracker_set_world_matrix_inverse_t)GetProcAddress(app, "?SetWorldMatrixInverse@AppTracker@@QAEXABVMatrix4f@Bionic@@@Z");
        if (real_AppTracker_SetWorldMatrixInverse && !apptracker_world_matrix_inverse_inline_installed) {
            apptracker_world_matrix_inverse_inline_installed =
                install_inline_hook((void*)real_AppTracker_SetWorldMatrixInverse,
                                    (void*)hook_AppTracker_SetWorldMatrixInverse,
                                    7,
                                    (void**)&tramp_AppTracker_SetWorldMatrixInverse);
            log_line("camera AppTracker::SetWorldMatrixInverse hook %s target=%p trampoline=%p note=\"camera capture is used only to reject view-space body inputs; camera is not a physics drive\"",
                     apptracker_world_matrix_inverse_inline_installed ? "installed" : "not-installed",
                     (void*)real_AppTracker_SetWorldMatrixInverse,
                     (void*)tramp_AppTracker_SetWorldMatrixInverse);
        }
    }
    if (sys) {
        /* SSimpleTransform's public Rotation setter delegates to this
           lower-level vector writer. FreeMode calls the native setter
           directly, bypassing the script-member table. This guarded hook is
           the runtime-only ownership boundary for mapped penis output joints. */
        if (!tramp_RuntimeRotationVectorWrite) {
            static const BYTE expected[] = {
                0x55, 0x8b, 0xec, 0x8b, 0x91, 0x80, 0x00, 0x00, 0x00
            };
            BYTE *target =
                (BYTE*)sys + RUNTIME_ROTATION_VECTOR_WRITE_RVA;
            if (ptr_readable(target, sizeof(expected)) &&
                memcmp(target, expected, sizeof(expected)) == 0) {
                real_RuntimeRotationVectorWrite =
                    (runtime_rotation_vector_write_t)target;
                if (install_inline_hook(
                        target,
                        (void*)hook_RuntimeRotationVectorWrite,
                        9,
                        (void**)&tramp_RuntimeRotationVectorWrite)) {
                    log_line("runtime low-level rotation ownership hook installed target=%p sys_rva=0xe2a70 trampoline=%p note=\"mapped FreeMode penis animation writes are captured and held only while PhysX owns that person; PoseEditor and unrelated joints pass through\"",
                             target,
                             (void*)tramp_RuntimeRotationVectorWrite);
                } else {
                    log_line("runtime low-level rotation ownership hook not-installed target=%p sys_rva=0xe2a70 reason=\"inline patch failed\"",
                             target);
                }
            } else if (!runtime_rotation_vector_write_hook_logged) {
                runtime_rotation_vector_write_hook_logged = 1;
                log_line("runtime low-level rotation ownership hook not-installed target=%p sys_rva=0xe2a70 reason=\"engine prologue did not match this build\"",
                         target);
            }
        }
        /* FreeMode's penis animation scheduler also writes
           SJoint.RotationAxis directly on Spenis_joint01/02/03.  That is a
           separate native channel from SSimpleTransform.Rotation, so it
           needs its own exact runtime-only ownership boundary. */
        if (!tramp_RuntimeJointRotationAxisWrite) {
            static const BYTE expected[] = {
                0x55, 0x8b, 0xec, 0x8b, 0x55, 0x0c
            };
            BYTE *target =
                (BYTE*)sys + RUNTIME_JOINT_ROTATION_AXIS_WRITE_RVA;
            if (ptr_readable(target, sizeof(expected)) &&
                memcmp(target, expected, sizeof(expected)) == 0) {
                if (install_inline_hook(
                        target,
                        (void*)hook_RuntimeJointRotationAxisWrite,
                        6,
                        (void**)&tramp_RuntimeJointRotationAxisWrite)) {
                    log_line("runtime SJoint RotationAxis ownership hook installed target=%p sys_rva=0xdca90 trampoline=%p note=\"mapped FreeMode penis-axis animation is captured and neutralized only while PhysX owns that person; PoseEditor and unrelated joints pass through\"",
                             target,
                             (void*)tramp_RuntimeJointRotationAxisWrite);
                } else {
                    log_line("runtime SJoint RotationAxis ownership hook not-installed target=%p sys_rva=0xdca90 reason=\"inline patch failed\"",
                             target);
                }
            } else if (!runtime_joint_rotation_axis_write_hook_logged) {
                runtime_joint_rotation_axis_write_hook_logged = 1;
                log_line("runtime SJoint RotationAxis ownership hook not-installed target=%p sys_rva=0xdca90 reason=\"engine prologue did not match this build\"",
                         target);
            }
        }
        engine_StringRefHash32 = engine_StringRefHash32 ?
            engine_StringRefHash32 :
            (stringref_hash32_t)GetProcAddress(
                sys, "?Hash32@StringRef@Bionic@@QBEIXZ");
        if (!engine_StringRefHash32) {
            engine_StringRefHash32 =
                (stringref_hash32_t)GetProcAddress(sys, (LPCSTR)1640);
        }
        engine_NameHashFind = engine_NameHashFind ?
            engine_NameHashFind :
            (namehash_find_t)GetProcAddress(
                sys, "?Find@NameHash@Bionic@@QBEPAXIABVStringRef@2@@Z");
        if (!engine_NameHashFind) {
            engine_NameHashFind =
                (namehash_find_t)GetProcAddress(sys, (LPCSTR)1123);
        }
        engine_GetModelViewRotationPivot =
            (model_pivot_t)GetProcAddress(
                sys,
                "?GetModelViewRotationPivot@Bionic@@YAXPAVScriptObject@1@AAVVector3f@1@@Z");
        engine_GetWeakObjTarget = (get_weak_obj_target_t)GetProcAddress(sys, "?GetWeakObjTarget@Abstract@Bionic@@QBEPBVWeakObjTarget@2@XZ");
        if (!engine_GetWeakObjTarget) engine_GetWeakObjTarget = (get_weak_obj_target_t)GetProcAddress(sys, (LPCSTR)1618);
        engine_ScriptObjectGetIndexScriptObject = (script_get_index_scriptobject_t)GetProcAddress(sys, "?Get@ScriptObject@Bionic@@QBEXPBVClassMember_Index_ScriptObject@2@AAV?$Array@V?$Obj@VScriptObject@Bionic@@@Bionic@@@2@@Z");
        if (!engine_ScriptObjectGetIndexScriptObject) engine_ScriptObjectGetIndexScriptObject = (script_get_index_scriptobject_t)GetProcAddress(sys, (LPCSTR)1237);
        engine_TBaseTransformGetMatrixVersion = engine_TBaseTransformGetMatrixVersion ?
            engine_TBaseTransformGetMatrixVersion :
            (tbase_get_matrix_version_t)GetProcAddress(sys, "?GetMatrixVersion@TBaseTransform@Bionic@@QBEIXZ");
        engine_TBaseTransformSetMatrixVersion = engine_TBaseTransformSetMatrixVersion ?
            engine_TBaseTransformSetMatrixVersion :
            (tbase_set_matrix_version_t)GetProcAddress(sys, "?SetMatrixVersion@TBaseTransform@Bionic@@QAEXI@Z");
        real_UpdateTraverse = real_UpdateTraverse ? real_UpdateTraverse :
            (update_traverse_t)GetProcAddress(sys, "?UpdateTraverse@Bionic@@YAXPAVScriptObject@1@ABVMatrix4f@1@I@Z");
        if (real_UpdateTraverse && !tramp_UpdateTraverse) {
            if (install_inline_hook((void*)real_UpdateTraverse,
                                    (void*)hook_UpdateTraverse,
                                    9,
                                    (void**)&tramp_UpdateTraverse)) {
                log_line("addon UpdateTraverse hook installed target=%p trampoline=%p note=\"applies sidecar PhysX output during TK17's own transform traversal so skinned add-ons can see custom bone poses\"",
                         (void*)real_UpdateTraverse,
                         (void*)tramp_UpdateTraverse);
            } else {
                log_line("addon UpdateTraverse hook not-installed target=%p reason=\"inline patch failed\"",
                         (void*)real_UpdateTraverse);
            }
        }
        real_Object_iNameSet = real_Object_iNameSet ? real_Object_iNameSet :
            (object_i_name_set_t)GetProcAddress(sys, "?iNameSet@Object@Bionic@@QAEXPBVClassMember_String@2@ABVStringRef@2@@Z");
        if (real_Object_iNameSet && !tramp_Object_iNameSet) {
            if (install_inline_hook((void*)real_Object_iNameSet,
                                    (void*)hook_Object_iNameSet,
                                    6,
                                    (void**)&tramp_Object_iNameSet)) {
                log_line("addon Object::iNameSet hook installed target=%p trampoline=%p note=\"captures custom add-on Object.Name joints that TK17 does not expose through SetTSNodeName\"",
                         (void*)real_Object_iNameSet,
                         (void*)tramp_Object_iNameSet);
            } else {
                log_line("addon Object::iNameSet hook not-installed target=%p reason=\"inline patch failed\"",
                         (void*)real_Object_iNameSet);
            }
        }
        real_CloneObject = real_CloneObject ? real_CloneObject :
            (clone_object_t)GetProcAddress(sys, "?CloneObject@Bionic@@YAPAVScriptObject@1@PBV21@@Z");
        if (real_CloneObject && !tramp_CloneObject) {
            if (install_inline_hook((void*)real_CloneObject,
                                    (void*)hook_CloneObject,
                                    7,
                                    (void**)&tramp_CloneObject)) {
                log_line("addon CloneObject hook installed target=%p trampoline=%p note=\"rebinding captured add-on Object.Name source bones to their live cloned room objects\"",
                         (void*)real_CloneObject,
                         (void*)tramp_CloneObject);
            } else {
                log_line("addon CloneObject hook not-installed target=%p reason=\"inline patch failed\"",
                         (void*)real_CloneObject);
            }
        }
        real_CloneNode = real_CloneNode ? real_CloneNode :
            (clone_node_t)GetProcAddress(sys, "?CloneNode@Bionic@@YAPAVScriptObject@1@PAV21@AAUCloneMap@1@@Z");
        if (real_CloneNode && !tramp_CloneNode) {
            if (install_inline_hook((void*)real_CloneNode,
                                    (void*)hook_CloneNode,
                                    6,
                                    (void**)&tramp_CloneNode)) {
                log_line("addon CloneNode hook installed target=%p trampoline=%p note=\"rebinding captured add-on node names to live cloned room nodes\"",
                         (void*)real_CloneNode,
                         (void*)tramp_CloneNode);
            } else {
                log_line("addon CloneNode hook not-installed target=%p reason=\"inline patch failed\"",
                         (void*)real_CloneNode);
            }
        }
        engine_G_NilWeakObjTarget_ptr = (void**)GetProcAddress(sys, "?G_NilWeakObjTarget@Bionic@@3QAVWeakObjTarget@1@A");
        if (!engine_G_NilWeakObjTarget_ptr) engine_G_NilWeakObjTarget_ptr = (void**)GetProcAddress(sys, (LPCSTR)1216);
        engine_G_NilObject_ptr = (void**)GetProcAddress(sys, "?G_NilObject@Bionic@@3QAVScriptObject@1@A");
        if (!engine_G_NilObject_ptr) engine_G_NilObject_ptr = (void**)GetProcAddress(sys, (LPCSTR)1214);
        engine_G_NullArray_ptr = (void**)GetProcAddress(sys, "?G_NullArray@Bionic@@3QBXB");
        if (!engine_G_NullArray_ptr) engine_G_NullArray_ptr = (void**)GetProcAddress(sys, (LPCSTR)1217);
        engine_G_MasterIsMVTBL_ptr = engine_G_MasterIsMVTBL_ptr ?
            engine_G_MasterIsMVTBL_ptr :
            (void***)GetProcAddress(
                sys, "?G_MasterIsMVTBL@Bionic@@3PAPAVMVTBL@1@A");
        if (!engine_G_MasterIsMVTBL_ptr) {
            engine_G_MasterIsMVTBL_ptr =
                (void***)GetProcAddress(sys, (LPCSTR)1211);
        }
        patch_runtime_animation_member_setters();
        patch_addon_constraint_count_getter();
    }
    log_line("symbols FindObjC=%p AppMainEngine=%p ScriptEngine=%p ModelViewPivot=%p SearchTree=%p ComponentArraySlot=%p ComponentArray=%p ScriptGetIndex=%p TBaseGetVersion=%p TBaseSetVersion=%p UpdateTraverse=%p UpdateTraverseTrampoline=%p NullArraySlot=%p NullArray=%p GetWeakObjTarget=%p NilWeakSlot=%p NilWeak=%p NilObjectSlot=%p NilObject=%p AppTrackerSetWorldMatrixInverse=%p SetTSNodeName=%p ObjectNameSet=%p ObjectNameSetTrampoline=%p CloneObject=%p CloneObjectTrampoline=%p CloneNode=%p CloneNodeTrampoline=%p",
             (void*)engine_FindObjC,
             (void*)engine_AppMainEngine,
             captured_script_engine,
             (void*)engine_GetModelViewRotationPivot,
             (void*)engine_SearchTree,
             (void*)engine_ComponentArray_ptr,
             (engine_ComponentArray_ptr && ptr_readable(engine_ComponentArray_ptr, sizeof(void*))) ? *engine_ComponentArray_ptr : NULL,
             (void*)engine_ScriptObjectGetIndexScriptObject,
             (void*)engine_TBaseTransformGetMatrixVersion,
             (void*)engine_TBaseTransformSetMatrixVersion,
             (void*)real_UpdateTraverse,
             (void*)tramp_UpdateTraverse,
             (void*)engine_G_NullArray_ptr,
             (engine_G_NullArray_ptr && ptr_readable(engine_G_NullArray_ptr, sizeof(void*))) ? *engine_G_NullArray_ptr : NULL,
             (void*)engine_GetWeakObjTarget,
             (void*)engine_G_NilWeakObjTarget_ptr,
             (engine_G_NilWeakObjTarget_ptr && ptr_readable(engine_G_NilWeakObjTarget_ptr, sizeof(void*))) ? *engine_G_NilWeakObjTarget_ptr : NULL,
             (void*)engine_G_NilObject_ptr,
             (engine_G_NilObject_ptr && ptr_readable(engine_G_NilObject_ptr, sizeof(void*))) ? *engine_G_NilObject_ptr : NULL,
             (void*)real_AppTracker_SetWorldMatrixInverse,
             (void*)real_SetTSNodeName,
             (void*)real_Object_iNameSet,
             (void*)tramp_Object_iNameSet,
             (void*)real_CloneObject,
             (void*)tramp_CloneObject,
             (void*)real_CloneNode,
             (void*)tramp_CloneNode);
}

static int is_nil_engine_object(void *raw, void *obj)
{
    void *nil_weak = engine_G_NilWeakObjTarget_ptr ? *engine_G_NilWeakObjTarget_ptr : NULL;
    void *nil_obj = engine_G_NilObject_ptr ? *engine_G_NilObject_ptr : NULL;
    return (raw && nil_weak && raw == nil_weak) ||
           (obj && nil_weak && obj == nil_weak) ||
           (obj && nil_obj && obj == nil_obj);
}

static void capture_script_engine(void *app_base, void *script_engine, const char *source)
{
    void *app_se = NULL;
    if (app_base && ptr_readable(app_base, sizeof(void*))) app_se = *(void**)app_base;
    if (!script_engine && app_se) script_engine = app_se;
    if (!captured_app_base && app_base) captured_app_base = app_base;
    if (!captured_script_engine && script_engine) captured_script_engine = script_engine;
    if (!captured_script_engine_logged && captured_script_engine) {
        captured_script_engine_logged = 1;
        log_line("script engine captured source=\"%s\" appBase=%p appBase.scriptengine=%p ecx.scriptengine=%p using=%p",
                 source ? source : "", app_base, app_se, script_engine, captured_script_engine);
    }
}

static void try_capture_script_engine_late(DWORD now, const char *source)
{
    void *script_engine = NULL;
    if (captured_script_engine) return;
    if (!engine_AppMainEngine) return;
    if (plugin_attach_tick && now - plugin_attach_tick < 15000u) return;
    if (script_engine_late_attempt_tick &&
        now - script_engine_late_attempt_tick < 5000u) {
        return;
    }
    if (script_engine_late_attempt_count >= 8) return;
    script_engine_late_attempt_tick = now;
    script_engine_late_attempt_count++;
    script_engine = engine_AppMainEngine();
    if (script_engine) {
        capture_script_engine(NULL, script_engine, source);
    } else if (defaults_cfg.debug) {
        log_line("script engine late capture pending source=\"%s\" attempt=%d AppMainEngine=%p note=\"AppMain::Engine returned null after startup; will retry\"",
                 source ? source : "",
                 script_engine_late_attempt_count,
                 (void*)engine_AppMainEngine);
    }
}

static void *resolve_find_obj(const char *name, void **raw_out)
{
    void *raw = NULL;
    void *weak = NULL;
    if (raw_out) *raw_out = NULL;
    if (!engine_FindObjC || !name || !name[0]) return NULL;
    raw = engine_FindObjC(name);
    if (raw_out) *raw_out = raw;
    if (raw && engine_GetWeakObjTarget) {
        weak = engine_GetWeakObjTarget(raw);
    }
    return weak ? weak : raw;
}

static void *resolve_script_engine_obj(const char *name, void **raw_out)
{
    void *se = captured_script_engine;
    void **vt;
    void *raw = NULL;
    void *weak = NULL;
    script_find_object_c_t find_object_c;
    if (raw_out) *raw_out = NULL;
    if (!se || !name || !name[0]) return NULL;
    if (!ptr_readable(se, sizeof(void*))) return NULL;
    vt = *(void***)se;
    if (!vt || !ptr_readable(vt, sizeof(void*) * 28)) return NULL;
    find_object_c = (script_find_object_c_t)vt[27];
    if (!find_object_c) return NULL;
    __asm__ ("movl %0, %%ecx" : : "r"(se) : "ecx");
    raw = find_object_c((char*)name);
    if (raw_out) *raw_out = raw;
    if (raw && engine_GetWeakObjTarget) {
        weak = engine_GetWeakObjTarget(raw);
    }
    return weak ? weak : raw;
}

static void import_script_object_tree_names(void *root_obj)
{
    void *se = captured_script_engine;
    void **vt;
    script_import_object_tree_names_t import_names;
    if (!se || !root_obj || is_nil_engine_object(NULL, root_obj)) return;
    if (!ptr_readable(se, sizeof(void*))) return;
    vt = *(void***)se;
    if (!vt || !ptr_readable(vt, sizeof(void*) * 36)) return;
    import_names = (script_import_object_tree_names_t)vt[35];
    if (!import_names) return;
    __asm__ ("movl %0, %%ecx" : : "r"(se) : "ecx");
    import_names(root_obj);
}

static void dump_component_array_for_node(const char *name, void *object)
{
    void *member = NULL;
    void *member_value = NULL;
    void *null_array = NULL;
    void *arr = NULL;
    int count = -1;
    int i;
    if (!name || !object || is_nil_engine_object(NULL, object)) return;
    if (!engine_ScriptObjectGetIndexScriptObject || !engine_ComponentArray_ptr) {
        log_line("component array unavailable root_name=\"%s\" object=%p ScriptGetIndex=%p ComponentArraySlot=%p",
                 name, object, (void*)engine_ScriptObjectGetIndexScriptObject, (void*)engine_ComponentArray_ptr);
        return;
    }
    if (ptr_readable(engine_ComponentArray_ptr, sizeof(void*))) {
        member_value = *engine_ComponentArray_ptr;
    }
    member = engine_ComponentArray_ptr;
    if (engine_G_NullArray_ptr && ptr_readable(engine_G_NullArray_ptr, sizeof(void*))) {
        null_array = *engine_G_NullArray_ptr;
    }
    arr = null_array;
    log_line("component array query root_name=\"%s\" object=%p member_slot=%p member_value=%p using_member=%p initial_array=%p",
             name, object, (void*)engine_ComponentArray_ptr, member_value, member, arr);
    if (!member || !ptr_readable(member, sizeof(void*))) {
        log_line("component array skipped root_name=\"%s\" reason=\"member_unreadable\" member=%p", name, member);
        return;
    }
    log_line("component array skipped root_name=\"%s\" reason=\"scriptobject_get_probe_disabled\" object=%p member=%p",
             name, object, member);
    return;
    engine_ScriptObjectGetIndexScriptObject(object, member, &arr);
    if (!arr || (null_array && arr == null_array)) {
        log_line("component array empty root_name=\"%s\" object=%p array=%p null_array=%p",
                 name, object, arr, null_array);
        return;
    }
    if (!ptr_readable((BYTE*)arr - sizeof(int), sizeof(int))) {
        log_line("component array unreadable-count root_name=\"%s\" object=%p array=%p null_array=%p",
                 name, object, arr, null_array);
        return;
    }
    count = *(int*)((BYTE*)arr - sizeof(int));
    if (count < 0 || count > 512) {
        log_line("component array suspicious-count root_name=\"%s\" object=%p array=%p count=%d null_array=%p",
                 name, object, arr, count, null_array);
        return;
    }
    log_line("component array root_name=\"%s\" object=%p array=%p count=%d null_array=%p",
             name, object, arr, count, null_array);
    if (count <= 0 || !ptr_readable(arr, sizeof(void*) * (size_t)count)) return;
    for (i = 0; i < count && i < 24; i++) {
        void *item = ((void**)arr)[i];
        void *weak = NULL;
        if (item && engine_GetWeakObjTarget && ptr_readable(item, sizeof(void*))) {
            weak = engine_GetWeakObjTarget(item);
        }
        log_line("component[%d] root_name=\"%s\" raw=%p weak=%p", i, name, item, weak);
    }
}

static int get_scriptobject_component_array(void *object, void ***items_out, int *count_out)
{
    void *arr = NULL;
    void *null_array = NULL;
    int count;
    if (items_out) *items_out = NULL;
    if (count_out) *count_out = 0;
    if (!object || is_nil_engine_object(NULL, object)) return 0;
    if (!engine_ScriptObjectGetIndexScriptObject || !engine_ComponentArray_ptr) return 0;
    if (!ptr_readable(engine_ComponentArray_ptr, sizeof(void*))) return 0;
    if (engine_G_NullArray_ptr && ptr_readable(engine_G_NullArray_ptr, sizeof(void*))) {
        null_array = *engine_G_NullArray_ptr;
    }
    arr = null_array;
    engine_ScriptObjectGetIndexScriptObject(object, engine_ComponentArray_ptr, &arr);
    if (!arr || (null_array && arr == null_array)) return 0;
    if (!ptr_readable((BYTE*)arr - sizeof(int), sizeof(int))) return 0;
    count = *(int*)((BYTE*)arr - sizeof(int));
    if (count <= 0 || count > 512) return 0;
    if (!ptr_readable(arr, sizeof(void*) * (size_t)count)) return 0;
    if (items_out) *items_out = (void**)arr;
    if (count_out) *count_out = count;
    return 1;
}

static int ptr_seen_in_list(void **seen, int seen_count, void *ptr)
{
    int i;
    if (!ptr) return 1;
    for (i = 0; i < seen_count; i++) {
        if (seen[i] == ptr) return 1;
    }
    return 0;
}

static void *resolve_component_array_target_recursive(void *object,
                                                      const char *root_name,
                                                      const char **target_variants,
                                                      int variant_count,
                                                      int depth,
                                                      void **seen,
                                                      int *seen_count,
                                                      char *matched,
                                                      size_t matched_sz)
{
    void **items = NULL;
    int count = 0;
    int i, vi;
    static int component_summary_log_count;
    static int component_found_log_count;
    if (!object || depth < 0 || !seen || !seen_count) return NULL;
    if (ptr_seen_in_list(seen, *seen_count, object)) return NULL;
    if (*seen_count < 256) seen[(*seen_count)++] = object;
    if (!get_scriptobject_component_array(object, &items, &count)) return NULL;
    if (component_summary_log_count < 80) {
        component_summary_log_count++;
        log_line("component array search root_name=\"%s\" object=%p depth=%d count=%d",
                 root_name ? root_name : "", object, depth, count);
    }
    if (count > 64) count = 64;
    for (i = 0; i < count; i++) {
        void *raw = items[i];
        void *item = raw;
        if (!item) continue;
        if (engine_GetWeakObjTarget && ptr_readable(item, sizeof(void*))) {
            void *weak = engine_GetWeakObjTarget(item);
            if (weak) item = weak;
        }
        if (!item || is_nil_engine_object(raw, item)) continue;
        if (engine_SearchTree) {
            for (vi = 0; vi < variant_count; vi++) {
                const char *name = target_variants[vi];
                const void *ref;
                void *found;
                if (!name || !name[0]) continue;
                ref = stringref_from_cstr_a(name);
                if (!ref) continue;
                found = engine_SearchTree(item, ref, NULL);
                if (found && !is_nil_engine_object(NULL, found)) {
                    if (matched && matched_sz) {
                        char tmp[384];
                        _snprintf(tmp, sizeof(tmp), "component[%d]:%s", i, name);
                        lstrcpynA(matched, tmp, (int)matched_sz);
                    }
                    if (component_found_log_count < 80) {
                        component_found_log_count++;
                        log_line("component array target found root_name=\"%s\" parent=%p component_index=%d raw=%p object=%p target=\"%s\" found=%p depth=%d",
                                 root_name ? root_name : "", object, i, raw, item, name, found, depth);
                    }
                    return found;
                }
            }
        }
        if (depth > 0) {
            void *found = resolve_component_array_target_recursive(item, root_name,
                                                                   target_variants,
                                                                   variant_count,
                                                                   depth - 1,
                                                                   seen, seen_count,
                                                                   matched, matched_sz);
            if (found) return found;
        }
    }
    return NULL;
}

static void *resolve_component_array_target(void *root,
                                            const char *root_name,
                                            const char **target_variants,
                                            int variant_count,
                                            void **raw_out,
                                            char *matched,
                                            size_t matched_sz)
{
    void *seen[256];
    int seen_count = 0;
    void *found;
    if (raw_out) *raw_out = NULL;
    if (matched && matched_sz) matched[0] = 0;
    if (!root || !target_variants || variant_count <= 0) return NULL;
    found = resolve_component_array_target_recursive(root, root_name,
                                                     target_variants,
                                                     variant_count,
                                                     2,
                                                     seen, &seen_count,
                                                     matched, matched_sz);
    if (raw_out) *raw_out = found;
    return found;
}

static int addon_declared_name_matches_alias(const char *runtime_name,
                                             const char *declared_name)
{
    char alias[192];
    if (!runtime_name || !declared_name ||
        !runtime_name[0] || !declared_name[0]) {
        return 0;
    }
    if (_stricmp(runtime_name, declared_name) == 0) return 1;
    _snprintf(alias, sizeof(alias), "S%s", declared_name);
    if (_stricmp(runtime_name, alias) == 0) return 1;
    _snprintf(alias, sizeof(alias), "local_%s", declared_name);
    if (_stricmp(runtime_name, alias) == 0) return 1;
    _snprintf(alias, sizeof(alias), "local_S%s", declared_name);
    return _stricmp(runtime_name, alias) == 0;
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

static int named_node_allows_duplicate_name(const char *name)
{
    if (!name || !name[0]) return 0;
    return addon_declared_sidecar_name(name) ||
           contains_i(name, "physx_") ||
           contains_i(name, "NcHair") ||
           contains_i(name, "Hair/") ||
           contains_i(name, "hair_") ||
           _stricmp(name, "head_joint02") == 0 ||
           _stricmp(name, "Shead_joint02") == 0 ||
           _stricmp(name, "local_head_joint02") == 0 ||
           _stricmp(name, "local_Shead_joint02") == 0;
}

static int addon_chain_armature_group_name(const char *name)
{
    if (!name || !name[0]) return 0;
    return _stricmp(name, "root_rotation_group") == 0 ||
           _stricmp(name, "root_pivot_rotation_group") == 0 ||
           _stricmp(name, "root_main_group") == 0;
}

static int addon_binding_probe_name_interesting(const char *name)
{
    if (!name || !name[0]) return 0;
    return addon_chain_armature_group_name(name) ||
           _stricmp(name, "head_joint02") == 0 ||
           _stricmp(name, "Shead_joint02") == 0 ||
           _stricmp(name, "local_head_joint02") == 0 ||
           _stricmp(name, "local_Shead_joint02") == 0;
}

static void remember_named_node(const char *name, void *object)
{
    int i;
    DWORD now = GetTickCount();
    if (!name || !name[0] || !object || is_nil_engine_object(NULL, object)) return;
    for (i = 0; i < named_node_count; i++) {
        if (_stricmp(named_nodes[i].name, name) == 0) {
            if (named_node_allows_duplicate_name(name) &&
                named_nodes[i].object != object) {
                continue;
            }
            if (named_nodes[i].object != object) {
                named_nodes[i].logged = 0;
                named_nodes[i].first_seen_tick = now;
                InterlockedIncrement(&named_node_generation);
            } else if (!named_nodes[i].first_seen_tick) {
                named_nodes[i].first_seen_tick = now;
                InterlockedIncrement(&named_node_generation);
            }
            named_nodes[i].object = object;
            return;
        }
    }
    if (named_node_count >= (int)(sizeof(named_nodes) / sizeof(named_nodes[0]))) return;
    lstrcpynA(named_nodes[named_node_count].name, name, sizeof(named_nodes[named_node_count].name));
    named_nodes[named_node_count].object = object;
    named_nodes[named_node_count].logged = 0;
    named_nodes[named_node_count].first_seen_tick = now;
    named_node_count++;
    InterlockedIncrement(&named_node_generation);
}

static void *find_named_node(const char *name)
{
    int i;
    if (!name || !name[0]) return NULL;
    for (i = named_node_count - 1; i >= 0; i--) {
        if (_stricmp(named_nodes[i].name, name) == 0) return named_nodes[i].object;
    }
    return NULL;
}

static body_chain_probe_t *find_body_chain_probe(const char *name)
{
    int i;
    if (!name || !name[0]) return NULL;
    for (i = 0; i < (int)(sizeof(body_chain_probe) / sizeof(body_chain_probe[0])); i++) {
        if (_stricmp(body_chain_probe[i].name, name) == 0) return &body_chain_probe[i];
    }
    return NULL;
}

static body_chain_probe_t *find_body_chain_probe_containing(const char *name)
{
    int i;
    if (!name || !name[0]) return NULL;
    for (i = 0; i < (int)(sizeof(body_chain_probe) / sizeof(body_chain_probe[0])); i++) {
        if (contains_i(name, body_chain_probe[i].name)) return &body_chain_probe[i];
    }
    return NULL;
}

static int body_chain_name_interesting(const char *name)
{
    return name &&
           (contains_i(name, "Model0") ||
            contains_i(name, "TRS_group") ||
            contains_i(name, "TRS_armature") ||
            contains_i(name, "penis_joint") ||
            contains_i(name, "testicles_joint") ||
            contains_i(name, "hip_L_joint") ||
            contains_i(name, "hip_R_joint") ||
            contains_i(name, "knee_L_joint") ||
            contains_i(name, "knee_R_joint"));
}

static int addon_chain_name_interesting(const char *name)
{
    if (!name || !name[0]) return 0;
    if (addon_declared_sidecar_name(name)) return 1;
    if ((addon_physics_enabled || addon_physics_probe_enabled) &&
        addon_binding_probe_name_interesting(name)) {
        return 1;
    }
    return (contains_i(name, "physx_") ||
            contains_i(name, "NcHair") ||
            contains_i(name, "Hair/") ||
            contains_i(name, "hair_")) &&
           !addon_chain_armature_group_name(name);
}

static void *resolve_runtime_named_target(const char *name, void **raw_out, char *matched, size_t matched_sz)
{
    int i, v;
    char s_name[192];
    char mesh_name[192];
    char mesh_s_name[192];
    if (raw_out) *raw_out = NULL;
    if (matched && matched_sz) matched[0] = 0;
    if (!name || !name[0]) return NULL;
    _snprintf(s_name, sizeof(s_name), "S%s", name);
    _snprintf(mesh_name, sizeof(mesh_name), "%s_mesh", name);
    _snprintf(mesh_s_name, sizeof(mesh_s_name), "S%s_mesh", name);
    for (i = 0; i < named_node_count; i++) {
        const char *root_name = named_nodes[i].name;
        const char *variants[4];
        variants[0] = name;
        variants[1] = mesh_name;
        variants[2] = s_name;
        variants[3] = mesh_s_name;
        if (!root_name || !root_name[0]) continue;
        for (v = 0; v < 4; v++) {
            char full_name[384];
            void *raw = NULL;
            void *obj = NULL;
            _snprintf(full_name, sizeof(full_name), "%s:%s", root_name, variants[v]);
            obj = resolve_find_obj(full_name, &raw);
            if (obj && !is_nil_engine_object(raw, obj)) {
                if (raw_out) *raw_out = raw;
                if (matched && matched_sz) lstrcpynA(matched, full_name, (int)matched_sz);
                return obj;
            }
            if (captured_script_engine) {
                obj = resolve_script_engine_obj(full_name, &raw);
                if (obj && !is_nil_engine_object(raw, obj)) {
                    if (raw_out) *raw_out = raw;
                    if (matched && matched_sz) lstrcpynA(matched, full_name, (int)matched_sz);
                    return obj;
                }
            }
        }
    }
    return NULL;
}

static DWORD runtime_exact_name_hash(const char *name, size_t *length_out)
{
    DWORD hash = 2166136261u;
    size_t length = 0;
    if (name) {
        while (name[length]) {
            hash ^= (BYTE)name[length];
            hash *= 16777619u;
            length++;
        }
    }
    if (length_out) *length_out = length;
    return hash ? hash : 1u;
}

static int runtime_exact_name_equal(const char *a, const char *b)
{
    if (!a || !b) return 0;
    while (*a && *a == *b) {
        a++;
        b++;
    }
    return *a == *b;
}

static void runtime_exact_join_name(char *out, size_t outsz,
                                    const char *root_name,
                                    const char *name)
{
    size_t pos = 0;
    if (!out || !outsz) return;
    if (root_name) {
        while (*root_name && pos + 1 < outsz) {
            out[pos++] = *root_name++;
        }
    }
    if (pos + 1 < outsz) out[pos++] = ':';
    if (name) {
        while (*name && pos + 1 < outsz) {
            out[pos++] = *name++;
        }
    }
    out[pos] = 0;
}

static void *resolve_runtime_exact_at_root(int root_index,
                                           const char *name,
                                           void **raw_out,
                                           char *matched,
                                           size_t matched_sz)
{
    const char *root_name;
    char full_name[384];
    void *raw = NULL;
    void *obj = NULL;
    if (root_index < 0 || root_index >= named_node_count) return NULL;
    root_name = named_nodes[root_index].name;
    if (!root_name || !root_name[0]) return NULL;
    runtime_exact_join_name(full_name, sizeof(full_name), root_name, name);
    obj = resolve_find_obj(full_name, &raw);
    if (!obj || is_nil_engine_object(raw, obj)) {
        if (!captured_script_engine) return NULL;
        obj = resolve_script_engine_obj(full_name, &raw);
        if (!obj || is_nil_engine_object(raw, obj)) return NULL;
    }
    if (raw_out) *raw_out = raw;
    if (matched && matched_sz) {
        lstrcpynA(matched, full_name, (int)matched_sz);
    }
    return obj;
}

static void *resolve_runtime_exact_target(const char *name, void **raw_out, char *matched, size_t matched_sz)
{
    int i;
    int failed_hint = -1;
    size_t name_length = 0;
    DWORD hash;
    LONG generation;
    runtime_exact_root_hint_t *hint = NULL;
    void *obj;
    if (raw_out) *raw_out = NULL;
    if (matched && matched_sz) matched[0] = 0;
    if (!name || !name[0]) return NULL;

    hash = runtime_exact_name_hash(name, &name_length);
    generation = InterlockedCompareExchange(&named_node_generation, 0, 0);
    if (name_length < RUNTIME_EXACT_ROOT_HINT_NAME) {
        hint = &runtime_exact_root_hints[
            hash & (RUNTIME_EXACT_ROOT_HINT_SLOTS - 1)];
        if (hint->hash == hash && hint->generation == generation &&
            runtime_exact_name_equal(hint->name, name)) {
            failed_hint = hint->root_index;
            obj = resolve_runtime_exact_at_root(failed_hint, name,
                                                raw_out, matched, matched_sz);
            if (obj) return obj;
            hint->generation = 0;
        }
    }

    for (i = 0; i < named_node_count; i++) {
        if (i == failed_hint) continue;
        obj = resolve_runtime_exact_at_root(i, name,
                                            raw_out, matched, matched_sz);
        if (obj) {
            if (hint) {
                hint->hash = hash;
                hint->generation = generation;
                hint->root_index = i;
                lstrcpynA(hint->name, name, sizeof(hint->name));
            }
            return obj;
        }
    }
    return NULL;
}

static float physx_absf(float v)
{
    return v < 0.0f ? -v : v;
}

static int sane_probe_float(float v)
{
    return v > -10000.0f && v < 10000.0f;
}

static float physx_clampf(float v, float lo, float hi);
static float physx_vec3_len(const float v[3]);

static void sample_motion_probe(const char *name, const char *source, void *base, DWORD now)
{
    int i, j;
    motion_probe_t *probe = NULL;
    if (!name || !source || !base || !ptr_readable(base, sizeof(float) * 256)) return;
    for (i = 0; i < motion_probe_count; i++) {
        if (_stricmp(motion_probes[i].name, name) == 0 &&
            _stricmp(motion_probes[i].source, source) == 0) {
            probe = &motion_probes[i];
            break;
        }
    }
    if (!probe) {
        if (motion_probe_count >= (int)(sizeof(motion_probes) / sizeof(motion_probes[0]))) return;
        probe = &motion_probes[motion_probe_count++];
        memset(probe, 0, sizeof(*probe));
        lstrcpynA(probe->name, name, sizeof(probe->name));
        lstrcpynA(probe->source, source, sizeof(probe->source));
    }
    if (probe->base != base) {
        probe->base = base;
        probe->initialized = 0;
        probe->last_log_tick = 0;
        probe->last_quiet_log_tick = 0;
    }
    if (!probe->initialized) {
        memcpy(probe->baseline, base, sizeof(probe->baseline));
        probe->initialized = 1;
        log_line("motion probe baseline name=\"%s\" source=%s base=%p", name, source, base);
        return;
    }
    if (now - probe->last_log_tick < 1000) return;
    {
        float *cur = (float*)base;
        int best_off[3] = { -1, -1, -1 };
        float best_delta[3] = { 0.0f, 0.0f, 0.0f };
        float best_v[3][3];
        float best_b[3][3];
        memset(best_v, 0, sizeof(best_v));
        memset(best_b, 0, sizeof(best_b));
        for (i = 0; i <= 256 - 3; i++) {
            float delta;
            if (!sane_probe_float(cur[i]) || !sane_probe_float(cur[i + 1]) || !sane_probe_float(cur[i + 2])) continue;
            if (!sane_probe_float(probe->baseline[i]) ||
                !sane_probe_float(probe->baseline[i + 1]) ||
                !sane_probe_float(probe->baseline[i + 2])) continue;
            delta = physx_absf(cur[i] - probe->baseline[i]) +
                    physx_absf(cur[i + 1] - probe->baseline[i + 1]) +
                    physx_absf(cur[i + 2] - probe->baseline[i + 2]);
            for (j = 0; j < 3; j++) {
                if (delta > best_delta[j]) {
                    int k;
                    for (k = 2; k > j; k--) {
                        best_delta[k] = best_delta[k - 1];
                        best_off[k] = best_off[k - 1];
                        memcpy(best_v[k], best_v[k - 1], sizeof(best_v[k]));
                        memcpy(best_b[k], best_b[k - 1], sizeof(best_b[k]));
                    }
                    best_delta[j] = delta;
                    best_off[j] = i * (int)sizeof(float);
                    best_v[j][0] = cur[i];
                    best_v[j][1] = cur[i + 1];
                    best_v[j][2] = cur[i + 2];
                    best_b[j][0] = probe->baseline[i];
                    best_b[j][1] = probe->baseline[i + 1];
                    best_b[j][2] = probe->baseline[i + 2];
                    break;
                }
            }
        }
        if (best_off[0] >= 0 && best_delta[0] > 0.005f) {
            probe->last_log_tick = now;
            log_line("motion probe changed name=\"%s\" source=%s base=%p best0=0x%03x delta0=%.5f base0=(%.5f,%.5f,%.5f) cur0=(%.5f,%.5f,%.5f) best1=0x%03x delta1=%.5f cur1=(%.5f,%.5f,%.5f) best2=0x%03x delta2=%.5f cur2=(%.5f,%.5f,%.5f)",
                     name, source, base,
                     best_off[0], best_delta[0],
                     best_b[0][0], best_b[0][1], best_b[0][2],
                     best_v[0][0], best_v[0][1], best_v[0][2],
                     best_off[1], best_delta[1],
                     best_v[1][0], best_v[1][1], best_v[1][2],
                     best_off[2], best_delta[2],
                     best_v[2][0], best_v[2][1], best_v[2][2]);
        } else if (now - probe->last_quiet_log_tick >= 5000) {
            probe->last_quiet_log_tick = now;
            log_line("motion probe quiet name=\"%s\" source=%s base=%p", name, source, base);
        }
    }
}

static void run_world_motion_probes(DWORD now)
{
    static const char *names[] = { NULL };
    int i;
    if (!engine_FindObjC || named_node_count <= 0) return;
    for (i = 0; i < (int)(sizeof(names) / sizeof(names[0])); i++) {
        void *raw = NULL;
        void *obj = NULL;
        char matched[384];
        if (!names[i]) continue;
        obj = resolve_runtime_exact_target(names[i], &raw, matched, sizeof(matched));
        if (obj && !is_nil_engine_object(raw, obj)) {
            sample_motion_probe(names[i], "raw", raw, now);
            sample_motion_probe(names[i], "object", obj, now);
        }
    }
}

static void run_body_chain_probe(DWORD now)
{
    int i;
    int seen_count = 0;
    int resolved_count = 0;
    if (last_body_probe_tick && now - last_body_probe_tick < 1000) return;
    last_body_probe_tick = now;
    if (!engine_FindObjC) return;
    for (i = 0; i < (int)(sizeof(body_chain_probe) / sizeof(body_chain_probe[0])); i++) {
        body_chain_probe_t *probe = &body_chain_probe[i];
        char matched[384];
        void *raw = NULL;
        void *obj = NULL;
        if (!probe->hook_object) probe->hook_object = find_named_node(probe->name);
        if (probe->hook_object && !probe->seen_logged) {
            probe->seen_logged = 1;
            if (!probe->seen_name[0]) lstrcpynA(probe->seen_name, probe->name, sizeof(probe->seen_name));
            log_line("body-chain tsnode seen name=\"%s\" seen_as=\"%s\" object=%p named_roots=%d",
                     probe->name, probe->seen_name, probe->hook_object, named_node_count);
        }
        if (probe->hook_object) seen_count++;
        obj = resolve_find_obj(probe->name, &raw);
        if (obj && !is_nil_engine_object(raw, obj)) {
            lstrcpynA(matched, probe->name, sizeof(matched));
        } else if (captured_script_engine) {
            obj = resolve_script_engine_obj(probe->name, &raw);
            if (obj && !is_nil_engine_object(raw, obj)) {
                lstrcpynA(matched, probe->name, sizeof(matched));
            }
        }
        if (!obj || is_nil_engine_object(raw, obj)) {
            obj = resolve_runtime_exact_target(probe->name, &raw, matched, sizeof(matched));
        }
        if (obj && !is_nil_engine_object(raw, obj)) {
            probe->runtime_raw = raw;
            probe->runtime_object = obj;
            resolved_count++;
            if (!probe->resolved_logged) {
                probe->resolved_logged = 1;
                log_line("body-chain runtime resolved name=\"%s\" runtime=\"%s\" raw=%p object=%p hook_object=%p reason=\"direct PersonXXAnim/Model01 path probe\"",
                         probe->name, matched, raw, obj, probe->hook_object);
            }
        } else if (!probe->missing_logged && probe->hook_object) {
            probe->missing_logged = 1;
            log_line("body-chain runtime unresolved name=\"%s\" hook_object=%p named_roots=%d reason=\"seen by SetTSNodeName but root:name lookup not resolved yet\"",
                     probe->name, probe->hook_object, named_node_count);
        }
    }
    {
        static DWORD last_waiting_summary_tick;
        static int ready_summary_logged;
        if (resolved_count >= 5) {
            if (!ready_summary_logged) {
                ready_summary_logged = 1;
                log_line("body-chain probe ready seen=%d resolved=%d named_nodes=%d note=\"runtime body nodes resolved; suppressing further probe summaries\"",
                         seen_count, resolved_count, named_node_count);
            }
        } else if (!last_waiting_summary_tick ||
                   now - last_waiting_summary_tick >= 5000) {
            last_waiting_summary_tick = now;
            log_line("body-chain probe waiting seen=%d resolved=%d named_nodes=%d note=\"waiting for runtime body nodes\"",
                     seen_count, resolved_count, named_node_count);
        }
    }
}

static transform_probe_sample_t *get_transform_probe_sample(const char *name, const char *source, void *base)
{
    int i;
    transform_probe_sample_t *sample;
    if (!name || !source || !base) return NULL;
    for (i = 0; i < transform_probe_sample_count; i++) {
        if (_stricmp(transform_probe_samples[i].name, name) == 0 &&
            _stricmp(transform_probe_samples[i].source, source) == 0) {
            sample = &transform_probe_samples[i];
            if (sample->base != base) {
                sample->base = base;
                sample->initialized = 0;
                sample->last_log_tick = 0;
            }
            return sample;
        }
    }
    if (transform_probe_sample_count >= (int)(sizeof(transform_probe_samples) / sizeof(transform_probe_samples[0]))) return NULL;
    sample = &transform_probe_samples[transform_probe_sample_count++];
    memset(sample, 0, sizeof(*sample));
    lstrcpynA(sample->name, name, sizeof(sample->name));
    lstrcpynA(sample->source, source, sizeof(sample->source));
    sample->base = base;
    return sample;
}

static void sample_transform_probe(const char *name, const char *source, void *base, DWORD now)
{
    enum { MAX_TOP = 8 };
    transform_probe_sample_t *sample;
    float *cur;
    int scan_floats;
    int top_count;
    int best_off[MAX_TOP];
    float best_delta[MAX_TOP];
    float best_cur[MAX_TOP][3];
    float best_base[MAX_TOP][3];
    float best_len[MAX_TOP];
    int i, j;
    if (!transform_probe_cfg.enabled) return;
    if (!name || !source || !base) return;
    scan_floats = transform_probe_cfg.scan_floats;
    if (scan_floats > 512) scan_floats = 512;
    if (scan_floats < 3) return;
    if (!ptr_readable(base, sizeof(float) * (size_t)scan_floats)) return;
    sample = get_transform_probe_sample(name, source, base);
    if (!sample) return;
    cur = (float*)base;
    if (!sample->initialized) {
        memcpy(sample->baseline, cur, sizeof(float) * (size_t)scan_floats);
        sample->initialized = 1;
        log_line("transform probe baseline name=\"%s\" source=%s base=%p scan_floats=%d",
                 name, source, base, scan_floats);
        return;
    }
    if (now - sample->last_log_tick < (DWORD)transform_probe_cfg.interval_ms) return;
    top_count = transform_probe_cfg.top_count;
    if (top_count > MAX_TOP) top_count = MAX_TOP;
    if (top_count < 1) top_count = 1;
    for (i = 0; i < MAX_TOP; i++) {
        best_off[i] = -1;
        best_delta[i] = 0.0f;
        best_len[i] = 0.0f;
        memset(best_cur[i], 0, sizeof(best_cur[i]));
        memset(best_base[i], 0, sizeof(best_base[i]));
    }
    for (i = 0; i <= scan_floats - 3; i++) {
        float delta;
        float len;
        if (!sane_probe_float(cur[i]) || !sane_probe_float(cur[i + 1]) || !sane_probe_float(cur[i + 2])) continue;
        if (!sane_probe_float(sample->baseline[i]) ||
            !sane_probe_float(sample->baseline[i + 1]) ||
            !sane_probe_float(sample->baseline[i + 2])) continue;
        delta = physx_absf(cur[i] - sample->baseline[i]) +
                physx_absf(cur[i + 1] - sample->baseline[i + 1]) +
                physx_absf(cur[i + 2] - sample->baseline[i + 2]);
        if (delta < transform_probe_cfg.threshold) continue;
        len = physx_vec3_len(&cur[i]);
        for (j = 0; j < top_count; j++) {
            if (delta > best_delta[j]) {
                int k;
                for (k = top_count - 1; k > j; k--) {
                    best_delta[k] = best_delta[k - 1];
                    best_off[k] = best_off[k - 1];
                    best_len[k] = best_len[k - 1];
                    memcpy(best_cur[k], best_cur[k - 1], sizeof(best_cur[k]));
                    memcpy(best_base[k], best_base[k - 1], sizeof(best_base[k]));
                }
                best_delta[j] = delta;
                best_off[j] = i * (int)sizeof(float);
                best_len[j] = len;
                best_cur[j][0] = cur[i];
                best_cur[j][1] = cur[i + 1];
                best_cur[j][2] = cur[i + 2];
                best_base[j][0] = sample->baseline[i];
                best_base[j][1] = sample->baseline[i + 1];
                best_base[j][2] = sample->baseline[i + 2];
                break;
            }
        }
    }
    if (best_off[0] < 0) return;
    sample->last_log_tick = now;
    log_line("transform probe changed name=\"%s\" source=%s base=%p top0=0x%03x d0=%.5f len0=%.5f base0=(%.5f,%.5f,%.5f) cur0=(%.5f,%.5f,%.5f) top1=0x%03x d1=%.5f len1=%.5f cur1=(%.5f,%.5f,%.5f) top2=0x%03x d2=%.5f len2=%.5f cur2=(%.5f,%.5f,%.5f) top3=0x%03x d3=%.5f len3=%.5f cur3=(%.5f,%.5f,%.5f) top4=0x%03x d4=%.5f len4=%.5f cur4=(%.5f,%.5f,%.5f) top5=0x%03x d5=%.5f len5=%.5f cur5=(%.5f,%.5f,%.5f)",
             name, source, base,
             best_off[0], best_delta[0], best_len[0],
             best_base[0][0], best_base[0][1], best_base[0][2],
             best_cur[0][0], best_cur[0][1], best_cur[0][2],
             best_off[1], best_delta[1], best_len[1], best_cur[1][0], best_cur[1][1], best_cur[1][2],
             best_off[2], best_delta[2], best_len[2], best_cur[2][0], best_cur[2][1], best_cur[2][2],
             best_off[3], best_delta[3], best_len[3], best_cur[3][0], best_cur[3][1], best_cur[3][2],
             best_off[4], best_delta[4], best_len[4], best_cur[4][0], best_cur[4][1], best_cur[4][2],
             best_off[5], best_delta[5], best_len[5], best_cur[5][0], best_cur[5][1], best_cur[5][2]);
}

static void run_transform_probe(DWORD now)
{
    static const char *suffixes[] = {
        "TRS_group",
        "STRS_group",
        "TRS_armature",
        "root",
        "Sroot",
        "penis_joint01",
        "penis_joint02",
        "penis_joint03",
        "penis_jointEnd",
        "Spenis_joint01",
        "Spenis_joint02",
        "Spenis_joint03",
        "Spenis_jointEnd"
    };
    int i;
    if (!transform_probe_cfg.enabled) return;
    if (!engine_FindObjC) return;
    if (transform_probe_cfg.last_tick &&
        now - transform_probe_cfg.last_tick < (DWORD)transform_probe_cfg.interval_ms) return;
    transform_probe_cfg.last_tick = now;
    for (i = 0; i < (int)(sizeof(suffixes) / sizeof(suffixes[0])); i++) {
        char name[256];
        char matched[384];
        void *raw = NULL;
        void *obj = NULL;
        _snprintf(name, sizeof(name), "%sAnim:Model01:%s", transform_probe_cfg.person, suffixes[i]);
        name[sizeof(name) - 1] = 0;
        matched[0] = 0;
        obj = resolve_find_obj(name, &raw);
        if (obj && !is_nil_engine_object(raw, obj)) {
            lstrcpynA(matched, name, sizeof(matched));
        } else if (captured_script_engine) {
            obj = resolve_script_engine_obj(name, &raw);
            if (obj && !is_nil_engine_object(raw, obj)) {
                lstrcpynA(matched, name, sizeof(matched));
            }
        }
        if (!obj || is_nil_engine_object(raw, obj)) {
            obj = resolve_runtime_exact_target(name, &raw, matched, sizeof(matched));
        }
        if (raw) sample_transform_probe(name, "raw", raw, now);
        if (transform_probe_cfg.include_object && obj && !is_nil_engine_object(raw, obj)) {
            sample_transform_probe(name, "object", obj, now);
        }
    }
}

static void *resolve_axis_map_raw(const char *name)
{
    char matched[384];
    void *raw = NULL;
    void *obj = NULL;
    if (!name || !name[0]) return NULL;
    matched[0] = 0;
    obj = resolve_find_obj(name, &raw);
    if (!raw && captured_script_engine) {
        obj = resolve_script_engine_obj(name, &raw);
    }
    if (!raw || is_nil_engine_object(raw, obj)) {
        obj = resolve_runtime_exact_target(name, &raw, matched, sizeof(matched));
    }
    (void)obj;
    return raw;
}

static void run_axis_root_scan(DWORD now)
{
    enum { ROOT_SCAN_FLOATS = 512, ROOT_SCAN_TOP = 6 };
    char name[256];
    void *raw;
    float *cur;
    int best_off[ROOT_SCAN_TOP];
    float best_step[ROOT_SCAN_TOP];
    float best_cur[ROOT_SCAN_TOP][3];
    float best_delta[ROOT_SCAN_TOP][3];
    int i, j;
    (void)now;
    axis_root_scan.last_best_off = -1;
    axis_root_scan.last_best_step = 0.0f;
    _snprintf(name, sizeof(name), "%sAnim:Model01:root", axis_map_probe_cfg.person);
    name[sizeof(name) - 1] = 0;
    raw = resolve_axis_map_raw(name);
    if (!raw || !ptr_readable(raw, sizeof(float) * ROOT_SCAN_FLOATS)) return;
    if (raw != axis_root_scan.base) {
        axis_root_scan.base = raw;
        axis_root_scan.initialized = 0;
    }
    cur = (float*)raw;
    if (!axis_root_scan.initialized) {
        memcpy(axis_root_scan.previous, cur, sizeof(float) * ROOT_SCAN_FLOATS);
        axis_root_scan.initialized = 1;
        log_line("axis-root-scan baseline person=\"%s\" node=\"%s\" raw=%p scan_floats=%d",
                 axis_map_probe_cfg.person, name, raw, ROOT_SCAN_FLOATS);
        return;
    }
    for (i = 0; i < ROOT_SCAN_TOP; i++) {
        best_off[i] = -1;
        best_step[i] = 0.0f;
        memset(best_cur[i], 0, sizeof(best_cur[i]));
        memset(best_delta[i], 0, sizeof(best_delta[i]));
    }
    for (i = 0; i <= ROOT_SCAN_FLOATS - 3; i++) {
        float delta[3];
        float step_len;
        if (!sane_probe_float(cur[i]) || !sane_probe_float(cur[i + 1]) || !sane_probe_float(cur[i + 2])) continue;
        if (!sane_probe_float(axis_root_scan.previous[i]) ||
            !sane_probe_float(axis_root_scan.previous[i + 1]) ||
            !sane_probe_float(axis_root_scan.previous[i + 2])) continue;
        delta[0] = cur[i] - axis_root_scan.previous[i];
        delta[1] = cur[i + 1] - axis_root_scan.previous[i + 1];
        delta[2] = cur[i + 2] - axis_root_scan.previous[i + 2];
        step_len = physx_vec3_len(delta);
        if (step_len < axis_map_probe_cfg.threshold) continue;
        for (j = 0; j < ROOT_SCAN_TOP; j++) {
            if (step_len > best_step[j]) {
                int k;
                for (k = ROOT_SCAN_TOP - 1; k > j; k--) {
                    best_off[k] = best_off[k - 1];
                    best_step[k] = best_step[k - 1];
                    memcpy(best_cur[k], best_cur[k - 1], sizeof(best_cur[k]));
                    memcpy(best_delta[k], best_delta[k - 1], sizeof(best_delta[k]));
                }
                best_off[j] = i * (int)sizeof(float);
                best_step[j] = step_len;
                best_cur[j][0] = cur[i];
                best_cur[j][1] = cur[i + 1];
                best_cur[j][2] = cur[i + 2];
                best_delta[j][0] = delta[0];
                best_delta[j][1] = delta[1];
                best_delta[j][2] = delta[2];
                break;
            }
        }
    }
    memcpy(axis_root_scan.previous, cur, sizeof(float) * ROOT_SCAN_FLOATS);
    if (best_off[0] < 0) return;
    axis_root_scan.last_best_off = best_off[0];
    axis_root_scan.last_best_step = best_step[0];
    log_line("axis-root-scan step person=\"%s\" phase=%d node=\"%s\" raw=%p top0=0x%03x len0=%.5f step0=(%.5f,%.5f,%.5f) cur0=(%.5f,%.5f,%.5f) top1=0x%03x len1=%.5f step1=(%.5f,%.5f,%.5f) cur1=(%.5f,%.5f,%.5f) top2=0x%03x len2=%.5f step2=(%.5f,%.5f,%.5f) cur2=(%.5f,%.5f,%.5f) top3=0x%03x len3=%.5f step3=(%.5f,%.5f,%.5f) cur3=(%.5f,%.5f,%.5f) top4=0x%03x len4=%.5f step4=(%.5f,%.5f,%.5f) cur4=(%.5f,%.5f,%.5f) top5=0x%03x len5=%.5f step5=(%.5f,%.5f,%.5f) cur5=(%.5f,%.5f,%.5f)",
             axis_map_probe_cfg.person, axis_map_probe_cfg.phase_logged, name, raw,
             best_off[0], best_step[0], best_delta[0][0], best_delta[0][1], best_delta[0][2], best_cur[0][0], best_cur[0][1], best_cur[0][2],
             best_off[1], best_step[1], best_delta[1][0], best_delta[1][1], best_delta[1][2], best_cur[1][0], best_cur[1][1], best_cur[1][2],
             best_off[2], best_step[2], best_delta[2][0], best_delta[2][1], best_delta[2][2], best_cur[2][0], best_cur[2][1], best_cur[2][2],
             best_off[3], best_step[3], best_delta[3][0], best_delta[3][1], best_delta[3][2], best_cur[3][0], best_cur[3][1], best_cur[3][2],
             best_off[4], best_step[4], best_delta[4][0], best_delta[4][1], best_delta[4][2], best_cur[4][0], best_cur[4][1], best_cur[4][2],
             best_off[5], best_step[5], best_delta[5][0], best_delta[5][1], best_delta[5][2], best_cur[5][0], best_cur[5][1], best_cur[5][2]);
}

static void run_axis_map_probe(DWORD now)
{
    static const char *phase_labels[] = {
        "wait/settle",
        "move root up/down",
        "move root left/right",
        "rotate root up/down",
        "rotate root left/right",
        "done"
    };
    int i;
    if (!axis_map_probe_cfg.enabled) return;
    if (!engine_FindObjC) return;
    if (axis_map_probe_cfg.last_tick &&
        now - axis_map_probe_cfg.last_tick < (DWORD)axis_map_probe_cfg.interval_ms) return;
    axis_map_probe_cfg.last_tick = now;
    if (!axis_root_scan.initialized) {
        run_axis_root_scan(now);
        if (!axis_root_scan.initialized) return;
    }
    run_axis_root_scan(now);
    if (axis_map_probe_cfg.phase_seconds > 0) {
        DWORD elapsed;
        int phase;
        int phase_count = (int)(sizeof(phase_labels) / sizeof(phase_labels[0]));
        if (!axis_map_probe_cfg.phase_start_tick) {
            if (axis_map_probe_cfg.phase_logged != 0) {
                axis_map_probe_cfg.phase_logged = 0;
                log_line("axis-map phase schedule person=\"%s\" start_mode=first_root_motion start_threshold=%.5f phase_seconds=%d phases=\"1 move root up/down; 2 move root left/right; 3 rotate root up/down; 4 rotate root left/right\"",
                         axis_map_probe_cfg.person,
                         axis_map_probe_cfg.phase_start_threshold,
                         axis_map_probe_cfg.phase_seconds);
                log_line("axis-map phase begin person=\"%s\" phase=0 label=\"wait for first root movement\"",
                         axis_map_probe_cfg.person);
            }
            if (axis_root_scan.last_best_step < axis_map_probe_cfg.phase_start_threshold) {
                return;
            }
            axis_map_probe_cfg.phase_start_tick = now;
            axis_map_probe_cfg.phase_logged = -1;
            log_line("axis-map first root movement person=\"%s\" top=0x%03x len=%.5f threshold=%.5f",
                     axis_map_probe_cfg.person,
                     axis_root_scan.last_best_off,
                     axis_root_scan.last_best_step,
                     axis_map_probe_cfg.phase_start_threshold);
        }
        elapsed = now - axis_map_probe_cfg.phase_start_tick;
        phase = 1 + (int)(elapsed / (DWORD)(axis_map_probe_cfg.phase_seconds * 1000));
        if (phase >= phase_count) phase = phase_count - 1;
        if (phase != axis_map_probe_cfg.phase_logged) {
            axis_map_probe_cfg.phase_logged = phase;
            log_line("axis-map phase begin person=\"%s\" phase=%d label=\"%s\"",
                     axis_map_probe_cfg.person, phase, phase_labels[phase]);
        }
    }
    for (i = 0; i < (int)(sizeof(axis_map_probe_samples) / sizeof(axis_map_probe_samples[0])); i++) {
        axis_map_probe_sample_t *sample = &axis_map_probe_samples[i];
        char name[256];
        void *raw;
        float *v;
        float step[3];
        float total[3];
        float step_len;
        float total_len;
        _snprintf(name, sizeof(name), "%sAnim:Model01:%s", axis_map_probe_cfg.person, sample->suffix);
        name[sizeof(name) - 1] = 0;
        raw = resolve_axis_map_raw(name);
        if (!raw) continue;
        if (raw != sample->base) {
            sample->base = raw;
            sample->initialized = 0;
        }
        if (!ptr_readable((BYTE*)raw + sample->offset, sizeof(float) * 3)) continue;
        v = (float*)((BYTE*)raw + sample->offset);
        if (!sane_probe_float(v[0]) || !sane_probe_float(v[1]) || !sane_probe_float(v[2])) continue;
        if (!sample->initialized) {
            sample->baseline[0] = v[0];
            sample->baseline[1] = v[1];
            sample->baseline[2] = v[2];
            sample->previous[0] = v[0];
            sample->previous[1] = v[1];
            sample->previous[2] = v[2];
            sample->initialized = 1;
            log_line("axis-map baseline person=\"%s\" label=%s node=\"%s\" raw=%p offset=0x%03x value=(%.5f,%.5f,%.5f)",
                     axis_map_probe_cfg.person, sample->label, name, raw, sample->offset,
                     v[0], v[1], v[2]);
            continue;
        }
        step[0] = v[0] - sample->previous[0];
        step[1] = v[1] - sample->previous[1];
        step[2] = v[2] - sample->previous[2];
        total[0] = v[0] - sample->baseline[0];
        total[1] = v[1] - sample->baseline[1];
        total[2] = v[2] - sample->baseline[2];
        step_len = physx_vec3_len(step);
        total_len = physx_vec3_len(total);
        sample->previous[0] = v[0];
        sample->previous[1] = v[1];
        sample->previous[2] = v[2];
        if (step_len < axis_map_probe_cfg.threshold) continue;
        log_line("axis-map sample person=\"%s\" phase=%d label=%s node=\"%s\" raw=%p offset=0x%03x cur=(%.5f,%.5f,%.5f) step=(%.5f,%.5f,%.5f) step_len=%.5f total=(%.5f,%.5f,%.5f) total_len=%.5f",
                 axis_map_probe_cfg.person, axis_map_probe_cfg.phase_logged, sample->label, name, raw, sample->offset,
                 v[0], v[1], v[2],
                 step[0], step[1], step[2], step_len,
                 total[0], total[1], total[2], total_len);
    }
}

static void make_body_runtime_name(char *out, size_t outsz, const char *person, const char *node)
{
    if (!out || outsz == 0) return;
    out[0] = 0;
    if (!person || !person[0] || !node || !node[0]) return;
    if (strstr(node, "Anim:")) {
        lstrcpynA(out, node, (int)outsz);
    } else {
        _snprintf(out, outsz, "%sAnim:Model01:%s", person, node);
        out[outsz - 1] = 0;
    }
}

static void run_root_drive_probe(DWORD now)
{
    char source_name[256];
    char target_name[256];
    void *source_raw;
    void *target_raw;
    float *source;
    float *target;
    float source_step[3];
    float source_delta[3];
    float write_amount;
    float step_len;
    int i;
    if (!root_drive_probe_cfg.enabled) return;
    if (!engine_FindObjC) return;
    if (root_drive_probe_cfg.source_axis < -1 || root_drive_probe_cfg.source_axis > 2) return;
    if (root_drive_probe_cfg.target_axis < 0 || root_drive_probe_cfg.target_axis > 2) return;
    if (root_drive_probe_cfg.source_offset < 0 || root_drive_probe_cfg.target_offset < 0) return;
    if (root_drive_probe_cfg.last_tick &&
        now - root_drive_probe_cfg.last_tick < (DWORD)root_drive_probe_cfg.interval_ms) return;
    root_drive_probe_cfg.last_tick = now;

    make_body_runtime_name(source_name, sizeof(source_name), root_drive_probe_cfg.person, root_drive_probe_cfg.source_node);
    make_body_runtime_name(target_name, sizeof(target_name), root_drive_probe_cfg.person, root_drive_probe_cfg.target_node);
    if (!source_name[0] || !target_name[0]) return;

    source_raw = resolve_axis_map_raw(source_name);
    target_raw = resolve_axis_map_raw(target_name);
    if (!source_raw || !target_raw) return;
    if (!ptr_readable((BYTE*)source_raw + root_drive_probe_cfg.source_offset, sizeof(float) * 3)) return;
    if (!ptr_readable((BYTE*)target_raw + root_drive_probe_cfg.target_offset, sizeof(float) * 3)) return;
    source = (float*)((BYTE*)source_raw + root_drive_probe_cfg.source_offset);
    target = (float*)((BYTE*)target_raw + root_drive_probe_cfg.target_offset);
    for (i = 0; i < 3; i++) {
        if (!sane_probe_float(source[i]) || !sane_probe_float(target[i])) return;
    }

    if (source_raw != root_drive_probe_cfg.source_raw ||
        target_raw != root_drive_probe_cfg.target_raw ||
        !root_drive_probe_cfg.initialized ||
        ((root_drive_probe_cfg.target_rest[0] == 0.0f &&
          root_drive_probe_cfg.target_rest[1] == 0.0f &&
          root_drive_probe_cfg.target_rest[2] == 0.0f) &&
         (target[0] != 0.0f || target[1] != 0.0f || target[2] != 0.0f))) {
        root_drive_probe_cfg.source_raw = source_raw;
        root_drive_probe_cfg.target_raw = target_raw;
        root_drive_probe_cfg.initialized = 1;
        root_drive_probe_cfg.resolved_logged = 0;
        for (i = 0; i < 3; i++) {
            root_drive_probe_cfg.source_rest[i] = source[i];
            root_drive_probe_cfg.source_prev[i] = source[i];
            root_drive_probe_cfg.target_rest[i] = target[i];
        }
    }

    for (i = 0; i < 3; i++) {
        source_step[i] = source[i] - root_drive_probe_cfg.source_prev[i];
        source_delta[i] = source[i] - root_drive_probe_cfg.source_rest[i];
        root_drive_probe_cfg.source_prev[i] = source[i];
    }
    step_len = physx_vec3_len(source_step);
    {
        int drive_axis = root_drive_probe_cfg.source_axis;
        if (drive_axis < 0) {
            drive_axis = 0;
            if (physx_absf(source_step[1]) > physx_absf(source_step[drive_axis])) drive_axis = 1;
            if (physx_absf(source_step[2]) > physx_absf(source_step[drive_axis])) drive_axis = 2;
        }
        write_amount = source_step[drive_axis] * root_drive_probe_cfg.scale;
        write_amount = physx_clampf(write_amount,
                                    -root_drive_probe_cfg.max_amount,
                                    root_drive_probe_cfg.max_amount);
        target[root_drive_probe_cfg.target_axis] =
            root_drive_probe_cfg.target_rest[root_drive_probe_cfg.target_axis] + write_amount;

        if (!root_drive_probe_cfg.resolved_logged ||
            step_len >= root_drive_probe_cfg.threshold) {
            root_drive_probe_cfg.resolved_logged = 1;
            log_line("root-drive-probe person=\"%s\" source=\"%s\" target=\"%s\" source_raw=%p target_raw=%p source_offset=0x%03x source_axis=%d actual_axis=%d target_offset=0x%03x target_axis=%d source_cur=(%.5f,%.5f,%.5f) source_step=(%.5f,%.5f,%.5f) source_delta=(%.5f,%.5f,%.5f) write=%.5f target_rest=(%.5f,%.5f,%.5f) target_cur=(%.5f,%.5f,%.5f)",
                     root_drive_probe_cfg.person, source_name, target_name,
                     source_raw, target_raw,
                     root_drive_probe_cfg.source_offset, root_drive_probe_cfg.source_axis,
                     drive_axis,
                     root_drive_probe_cfg.target_offset, root_drive_probe_cfg.target_axis,
                     source[0], source[1], source[2],
                     source_step[0], source_step[1], source_step[2],
                     source_delta[0], source_delta[1], source_delta[2],
                     write_amount,
                     root_drive_probe_cfg.target_rest[0],
                     root_drive_probe_cfg.target_rest[1],
                     root_drive_probe_cfg.target_rest[2],
                     target[0], target[1], target[2]);
        }
    }
}

static void run_write_sweep_probe(DWORD now)
{
    const write_sweep_candidate_t *cand;
    char name[256];
    void *raw;
    float *v;
    int count = (int)(sizeof(write_sweep_candidates) / sizeof(write_sweep_candidates[0]));
    if (!write_sweep_probe_cfg.enabled) return;
    if (!engine_FindObjC) return;
    if (count <= 0) return;
    if (write_sweep_probe_cfg.index < 0 || write_sweep_probe_cfg.index >= count) {
        write_sweep_probe_cfg.index = 0;
        write_sweep_probe_cfg.state = 0;
        write_sweep_probe_cfg.base = NULL;
    }
    cand = &write_sweep_candidates[write_sweep_probe_cfg.index];
    make_body_runtime_name(name, sizeof(name), write_sweep_probe_cfg.person, cand->node);
    if (!name[0]) return;
    raw = resolve_axis_map_raw(name);
    if (!raw || !ptr_readable((BYTE*)raw + cand->offset, sizeof(float) * 3)) {
        if (!write_sweep_probe_cfg.session_start_tick) return;
        if (write_sweep_probe_cfg.state == 0) {
            log_line("write-sweep skipped index=%d/%d label=\"%s\" node=\"%s\" raw=%p offset=0x%03x reason=\"unresolved or unreadable\"",
                     write_sweep_probe_cfg.index + 1, count, cand->label, name, raw, cand->offset);
            write_sweep_probe_cfg.index++;
            if (write_sweep_probe_cfg.index >= count) write_sweep_probe_cfg.index = 0;
        }
        return;
    }
    v = (float*)((BYTE*)raw + cand->offset);
    if (!sane_probe_float(v[0]) || !sane_probe_float(v[1]) || !sane_probe_float(v[2])) {
        if (!write_sweep_probe_cfg.session_start_tick) return;
        if (write_sweep_probe_cfg.state == 0) {
            log_line("write-sweep skipped index=%d/%d label=\"%s\" node=\"%s\" raw=%p offset=0x%03x current=(%.5f,%.5f,%.5f) reason=\"unsane floats\"",
                     write_sweep_probe_cfg.index + 1, count, cand->label, name, raw, cand->offset,
                     v[0], v[1], v[2]);
            write_sweep_probe_cfg.index++;
            if (write_sweep_probe_cfg.index >= count) write_sweep_probe_cfg.index = 0;
        }
        return;
    }
    if (!write_sweep_probe_cfg.session_start_tick) {
        write_sweep_probe_cfg.session_start_tick = now;
        log_line("write-sweep armed person=\"%s\" first_index=%d/%d label=\"%s\" node=\"%s\" raw=%p start_delay_ms=%d reason=\"first readable body target resolved; delaying visible writes from this point\"",
                 write_sweep_probe_cfg.person, write_sweep_probe_cfg.index + 1, count,
                 cand->label, name, raw, write_sweep_probe_cfg.start_delay_ms);
    }
    if (now - write_sweep_probe_cfg.session_start_tick < (DWORD)write_sweep_probe_cfg.start_delay_ms) {
        if (!write_sweep_probe_cfg.delay_logged) {
            write_sweep_probe_cfg.delay_logged = 1;
            log_line("write-sweep waiting person=\"%s\" start_delay_ms=%d reason=\"allow PoseEditor/runtime bones to finish resolving after first readable target\"",
                     write_sweep_probe_cfg.person, write_sweep_probe_cfg.start_delay_ms);
        }
        return;
    }
    if (write_sweep_probe_cfg.state == 0 || write_sweep_probe_cfg.base != raw) {
        write_sweep_probe_cfg.base = raw;
        write_sweep_probe_cfg.original[0] = v[0];
        write_sweep_probe_cfg.original[1] = v[1];
        write_sweep_probe_cfg.original[2] = v[2];
        v[cand->axis] = write_sweep_probe_cfg.original[cand->axis] + cand->amount;
        write_sweep_probe_cfg.start_tick = now;
        write_sweep_probe_cfg.state = 1;
        log_line("write-sweep applied index=%d/%d label=\"%s\" node=\"%s\" raw=%p offset=0x%03x axis=%d amount=%.5f before=(%.5f,%.5f,%.5f) after=(%.5f,%.5f,%.5f)",
                 write_sweep_probe_cfg.index + 1, count, cand->label, name, raw,
                 cand->offset, cand->axis, cand->amount,
                 write_sweep_probe_cfg.original[0],
                 write_sweep_probe_cfg.original[1],
                 write_sweep_probe_cfg.original[2],
                 v[0], v[1], v[2]);
        return;
    }
    if (now - write_sweep_probe_cfg.start_tick < (DWORD)write_sweep_probe_cfg.duration_ms) {
        v[cand->axis] = write_sweep_probe_cfg.original[cand->axis] + cand->amount;
        return;
    }
    v[0] = write_sweep_probe_cfg.original[0];
    v[1] = write_sweep_probe_cfg.original[1];
    v[2] = write_sweep_probe_cfg.original[2];
    log_line("write-sweep restored index=%d/%d label=\"%s\" node=\"%s\" raw=%p offset=0x%03x restored=(%.5f,%.5f,%.5f)",
             write_sweep_probe_cfg.index + 1, count, cand->label, name, raw,
             cand->offset, v[0], v[1], v[2]);
    write_sweep_probe_cfg.index++;
    if (write_sweep_probe_cfg.index >= count) write_sweep_probe_cfg.index = 0;
    write_sweep_probe_cfg.state = 0;
    write_sweep_probe_cfg.base = NULL;
}

static int collision_auto_test_person_index(const char *person)
{
    if (!person) return -1;
    if (_stricmp(person, "Person01") == 0) return 0;
    if (_stricmp(person, "Person02") == 0) return 1;
    if (_stricmp(person, "Person03") == 0) return 2;
    if (_stricmp(person, "Person04") == 0) return 3;
    return -1;
}

static void restore_collision_auto_test_active(void)
{
    int row;
    float *v;
    if (collision_auto_test_cfg.secondary_raw &&
        collision_auto_test_cfg.secondary_offset >= 0 &&
        ptr_readable((BYTE*)collision_auto_test_cfg.secondary_raw +
                     collision_auto_test_cfg.secondary_offset,
                     sizeof(float) * 3)) {
        v = (float*)((BYTE*)collision_auto_test_cfg.secondary_raw +
                    collision_auto_test_cfg.secondary_offset);
        v[0] = collision_auto_test_cfg.secondary_original[0];
        v[1] = collision_auto_test_cfg.secondary_original[1];
        v[2] = collision_auto_test_cfg.secondary_original[2];
    }
    collision_auto_test_cfg.secondary_raw = NULL;
    collision_auto_test_cfg.secondary_offset = -1;
    if (!collision_auto_test_cfg.active_raw) {
        return;
    }
    if (collision_auto_test_cfg.active_basis_mode) {
        for (row = 0; row < 3; row++) {
            int offset = collision_auto_test_cfg.active_basis_offsets[row];
            if (offset < 0 ||
                !ptr_readable((BYTE*)collision_auto_test_cfg.active_raw + offset,
                              sizeof(float) * 3)) {
                collision_auto_test_cfg.active_raw = NULL;
                collision_auto_test_cfg.active_basis_mode = 0;
                return;
            }
        }
        for (row = 0; row < 3; row++) {
            int offset = collision_auto_test_cfg.active_basis_offsets[row];
            v = (float*)((BYTE*)collision_auto_test_cfg.active_raw + offset);
            v[0] = collision_auto_test_cfg.active_original[row * 3 + 0];
            v[1] = collision_auto_test_cfg.active_original[row * 3 + 1];
            v[2] = collision_auto_test_cfg.active_original[row * 3 + 2];
        }
        collision_auto_test_cfg.active_raw = NULL;
        collision_auto_test_cfg.active_basis_mode = 0;
        return;
    }
    if (collision_auto_test_cfg.active_offset < 0) return;
    if (!ptr_readable((BYTE*)collision_auto_test_cfg.active_raw +
                      collision_auto_test_cfg.active_offset,
                      sizeof(float) * 3)) {
        collision_auto_test_cfg.active_raw = NULL;
        return;
    }
    v = (float*)((BYTE*)collision_auto_test_cfg.active_raw +
                 collision_auto_test_cfg.active_offset);
    v[0] = collision_auto_test_cfg.active_original[0];
    v[1] = collision_auto_test_cfg.active_original[1];
    v[2] = collision_auto_test_cfg.active_original[2];
    collision_auto_test_cfg.active_raw = NULL;
}

static void collision_auto_test_rotate_vector(const float in[3], int axis,
                                              float radians, float out[3])
{
    float c = (float)cos((double)radians);
    float s = (float)sin((double)radians);
    out[0] = in[0];
    out[1] = in[1];
    out[2] = in[2];
    if (axis == 0) {
        out[1] = c * in[1] - s * in[2];
        out[2] = s * in[1] + c * in[2];
    } else if (axis == 1) {
        out[0] = c * in[0] + s * in[2];
        out[2] = -s * in[0] + c * in[2];
    } else {
        out[0] = c * in[0] - s * in[1];
        out[1] = s * in[0] + c * in[1];
    }
}

static int collision_auto_test_apply_root_basis(void *raw, int axis,
                                                float degrees, int capture)
{
    int offsets[3];
    int row;
    float radians = degrees * 0.01745329251994329577f;
    offsets[0] = physics_environment_cfg.gravity_horizontal_secondary_basis_offset;
    offsets[1] = physics_environment_cfg.gravity_horizontal_basis_offset;
    offsets[2] = physics_environment_cfg.gravity_vertical_basis_offset;
    if (offsets[0] == offsets[1] || offsets[0] == offsets[2] ||
        offsets[1] == offsets[2]) {
        return 0;
    }
    for (row = 0; row < 3; row++) {
        float *v;
        if (offsets[row] < 0 ||
            !ptr_readable((BYTE*)raw + offsets[row], sizeof(float) * 3)) {
            return 0;
        }
        v = (float*)((BYTE*)raw + offsets[row]);
        if (!sane_probe_float(v[0]) || !sane_probe_float(v[1]) ||
            !sane_probe_float(v[2]) ||
            physx_vec3_len(v) < 0.25f || physx_vec3_len(v) > 2.0f) {
            return 0;
        }
    }
    if (capture) {
        collision_auto_test_cfg.active_raw = raw;
        collision_auto_test_cfg.active_basis_mode = 1;
        collision_auto_test_cfg.active_axis = axis;
        for (row = 0; row < 3; row++) {
            float *v = (float*)((BYTE*)raw + offsets[row]);
            collision_auto_test_cfg.active_basis_offsets[row] = offsets[row];
            collision_auto_test_cfg.active_original[row * 3 + 0] = v[0];
            collision_auto_test_cfg.active_original[row * 3 + 1] = v[1];
            collision_auto_test_cfg.active_original[row * 3 + 2] = v[2];
        }
    }
    if (collision_auto_test_cfg.active_raw != raw ||
        !collision_auto_test_cfg.active_basis_mode) {
        return 0;
    }
    for (row = 0; row < 3; row++) {
        float rotated[3];
        float *v = (float*)((BYTE*)raw +
                           collision_auto_test_cfg.active_basis_offsets[row]);
        collision_auto_test_rotate_vector(
            &collision_auto_test_cfg.active_original[row * 3], axis,
            radians, rotated);
        v[0] = rotated[0];
        v[1] = rotated[1];
        v[2] = rotated[2];
    }
    return 1;
}

static void collision_auto_test_log_snapshot(const char *event,
                                             const collision_auto_test_step_t *step,
                                             int person_index)
{
    body_chain_person_state_t *chain;
    body_chain_collider_person_state_t *collider;
    int step_count = 0;
    if (person_index < 0 || person_index >= 4 || !step) return;
    chain = &body_chain_person_states[person_index];
    collider = &body_chain_collider_states[person_index];
    collision_auto_test_active_steps(&step_count);
    log_line("collision-auto-test %s mode=\"%s\" phase=%d/%d label=\"%s\" chain_angle=(%.3f,%.3f;%.3f,%.3f;%.3f,%.3f) gravity=(%.4f,%.4f,%.4f) test01=(%.5f,%.5f,%.5f) test02=(%.5f,%.5f,%.5f) hipL=(%.5f,%.5f,%.5f) hipR=(%.5f,%.5f,%.5f) kneeL=(%.5f,%.5f,%.5f) kneeR=(%.5f,%.5f,%.5f) collider_ready=%d",
             event,
             collision_auto_test_cfg.mode,
             collision_auto_test_cfg.phase + 1,
             step_count,
             step->label,
             chain->angle[0][0], chain->angle[0][1],
             chain->angle[1][0], chain->angle[1][1],
             chain->angle[2][0], chain->angle[2][1],
             chain->gravity_drive[0], chain->gravity_drive[1],
             chain->gravity_drive[2],
             collider->local_position[BODY_COLLIDER_TESTICLES_01][0],
             collider->local_position[BODY_COLLIDER_TESTICLES_01][1],
             collider->local_position[BODY_COLLIDER_TESTICLES_01][2],
             collider->local_position[BODY_COLLIDER_TESTICLES_02][0],
             collider->local_position[BODY_COLLIDER_TESTICLES_02][1],
             collider->local_position[BODY_COLLIDER_TESTICLES_02][2],
             collider->local_position[BODY_COLLIDER_HIP_L][0],
             collider->local_position[BODY_COLLIDER_HIP_L][1],
             collider->local_position[BODY_COLLIDER_HIP_L][2],
             collider->local_position[BODY_COLLIDER_HIP_R][0],
             collider->local_position[BODY_COLLIDER_HIP_R][1],
             collider->local_position[BODY_COLLIDER_HIP_R][2],
             collider->local_position[BODY_COLLIDER_KNEE_L][0],
             collider->local_position[BODY_COLLIDER_KNEE_L][1],
             collider->local_position[BODY_COLLIDER_KNEE_L][2],
             collider->local_position[BODY_COLLIDER_KNEE_R][0],
             collider->local_position[BODY_COLLIDER_KNEE_R][1],
             collider->local_position[BODY_COLLIDER_KNEE_R][2],
             collider->ready);
}

static void *collision_auto_test_resolve_step(
    const collision_auto_test_step_t *step,
    char resolved_name[256])
{
    void *raw;
    if (!step || !resolved_name) return NULL;
    make_body_runtime_name(resolved_name, 256,
                           collision_auto_test_cfg.person, step->node);
    raw = resolve_axis_map_raw(resolved_name);
    if (!raw && step->fallback_node && step->fallback_node[0]) {
        make_body_runtime_name(resolved_name, 256,
                               collision_auto_test_cfg.person,
                               step->fallback_node);
        raw = resolve_axis_map_raw(resolved_name);
    }
    return raw;
}

static void collision_auto_test_log_done(int step_count)
{
    log_line("============================================================");
    log_line("================ COLLISION AUTO TEST DONE!!!!! ==============");
    log_line("DONE!!!!! person=\"%s\" mode=\"%s\" steps=%d all_channels_restored=1 normal_physics_resumed=1",
             collision_auto_test_cfg.person,
             collision_auto_test_cfg.mode,
             step_count);
    log_line("============================================================");
}

static void collision_auto_test_reset_chain_phase(int person_index)
{
    body_chain_person_state_t *chain;
    int i;
    if (person_index < 0 || person_index >= 4) return;
    chain = &body_chain_person_states[person_index];
    for (i = 0; i < 3; i++) {
        chain->angle[i][0] = 0.0f;
        chain->angle[i][1] = 0.0f;
        chain->velocity[i][0] = 0.0f;
        chain->velocity[i][1] = 0.0f;
    }
    chain->gravity_drive_filtered[0] = 0.0f;
    chain->gravity_drive_filtered[1] = 0.0f;
    chain->gravity_drive_filtered[2] = 0.0f;
    chain->gravity_drive_filtered_valid = 0;
    chain->root_drive_last_step_valid = 0;
    chain->root_drive_last_h_step = 0.0f;
    chain->root_drive_last_v_step = 0.0f;
    chain->root_drive_last_d_step = 0.0f;
}

static void run_collision_auto_test(DWORD now)
{
    int person_index;
    int step_count = 0;
    const collision_auto_test_step_t *steps;
    const collision_auto_test_step_t *step;
    body_chain_person_state_t *chain;
    body_chain_collider_person_state_t *collider;
    char resolved_name[256];
    char secondary_name[256];
    void *raw;
    void *secondary_raw;
    float *v;
    float *secondary_v;
    float amount;
    float secondary_amount;

    if (!collision_auto_test_cfg.enabled ||
        collision_auto_test_cfg.completed) {
        return;
    }
    steps = collision_auto_test_active_steps(&step_count);
    if (!steps || step_count <= 0) return;
    person_index = collision_auto_test_person_index(
        collision_auto_test_cfg.person);
    if (person_index < 0) return;
    chain = &body_chain_person_states[person_index];
    collider = &body_chain_collider_states[person_index];
    if (!chain->initialized || !chain->gravity_probe_promoted ||
        !collider->ready) {
        collision_auto_test_cfg.ready_tick = 0;
        return;
    }
    if (!collision_auto_test_cfg.ready_tick) {
        collision_auto_test_cfg.ready_tick = now;
        log_line("collision-auto-test armed person=\"%s\" mode=\"%s\" start_delay_ms=%d phase_ms=%d rest_ms=%d steps=%d note=\"leave the model and camera alone; the plugin will move and restore each diagnostic channel automatically\"",
                 collision_auto_test_cfg.person,
                 collision_auto_test_cfg.mode,
                 collision_auto_test_cfg.start_delay_ms,
                 collision_auto_test_cfg.phase_ms,
                 collision_auto_test_cfg.rest_ms,
                 step_count);
        return;
    }
    if (now - collision_auto_test_cfg.ready_tick <
        (DWORD)collision_auto_test_cfg.start_delay_ms) {
        return;
    }
    if (collision_auto_test_cfg.phase >= step_count) {
        collision_auto_test_cfg.completed = 1;
        collision_auto_test_log_done(step_count);
        return;
    }

    step = &steps[collision_auto_test_cfg.phase];
    if (collision_auto_test_cfg.state == 2) {
        collision_auto_test_reset_chain_phase(person_index);
        if (now - collision_auto_test_cfg.phase_tick <
            (DWORD)collision_auto_test_cfg.rest_ms) {
            return;
        }
        collision_auto_test_cfg.phase++;
        collision_auto_test_cfg.state = 0;
        collision_auto_test_cfg.phase_tick = now;
        collision_auto_test_cfg.active_raw = NULL;
        collision_auto_test_cfg.active_basis_mode = 0;
        collision_auto_test_cfg.secondary_raw = NULL;
        collision_auto_test_cfg.secondary_offset = -1;
        if (collision_auto_test_cfg.phase >= step_count) {
            collision_auto_test_cfg.completed = 1;
            collision_auto_test_log_done(step_count);
        }
        return;
    }

    raw = collision_auto_test_resolve_step(step, resolved_name);
    if (!raw) {
        if (collision_auto_test_cfg.state == 0) {
            log_line("collision-auto-test skipped phase=%d/%d label=\"%s\" primary=\"%s\" fallback=\"%s\" reason=\"rotation channel unresolved\"",
                     collision_auto_test_cfg.phase + 1, step_count,
                     step->label, step->node,
                     step->fallback_node ? step->fallback_node : "");
            collision_auto_test_cfg.state = 2;
            collision_auto_test_cfg.phase_tick = now;
        }
        return;
    }
    amount = (step->root_step ? collision_auto_test_cfg.root_amount :
              collision_auto_test_channel_amount()) * step->amount_sign;

    if (step->basis_rotation) {
        if (collision_auto_test_cfg.state == 0 ||
            collision_auto_test_cfg.active_raw != raw ||
            !collision_auto_test_cfg.active_basis_mode) {
            if (!collision_auto_test_apply_root_basis(raw, step->axis, 0.0f, 1)) {
                log_line("collision-auto-test skipped phase=%d/%d label=\"%s\" node=\"%s\" raw=%p reason=\"live root basis rows are unreadable\"",
                         collision_auto_test_cfg.phase + 1, step_count,
                         step->label, resolved_name, raw);
                collision_auto_test_cfg.state = 2;
                collision_auto_test_cfg.phase_tick = now;
                collision_auto_test_cfg.active_raw = NULL;
                collision_auto_test_cfg.active_basis_mode = 0;
                return;
            }
            if (step->secondary_node && step->secondary_node[0]) {
                make_body_runtime_name(secondary_name, sizeof(secondary_name),
                                       collision_auto_test_cfg.person,
                                       step->secondary_node);
                secondary_raw = resolve_axis_map_raw(secondary_name);
                if (!secondary_raw ||
                    !ptr_readable((BYTE*)secondary_raw + step->secondary_offset,
                                  sizeof(float) * 3)) {
                    restore_collision_auto_test_active();
                    log_line("collision-auto-test skipped phase=%d/%d label=\"%s\" secondary=\"%s\" reason=\"secondary rotation channel unresolved\"",
                             collision_auto_test_cfg.phase + 1, step_count,
                             step->label, secondary_name);
                    collision_auto_test_cfg.state = 2;
                    collision_auto_test_cfg.phase_tick = now;
                    return;
                }
                secondary_v = (float*)((BYTE*)secondary_raw +
                                       step->secondary_offset);
                if (!sane_probe_float(secondary_v[0]) ||
                    !sane_probe_float(secondary_v[1]) ||
                    !sane_probe_float(secondary_v[2])) {
                    restore_collision_auto_test_active();
                    log_line("collision-auto-test skipped phase=%d/%d label=\"%s\" secondary=\"%s\" reason=\"secondary rotation channel invalid\"",
                             collision_auto_test_cfg.phase + 1, step_count,
                             step->label, secondary_name);
                    collision_auto_test_cfg.state = 2;
                    collision_auto_test_cfg.phase_tick = now;
                    return;
                }
                collision_auto_test_cfg.secondary_raw = secondary_raw;
                collision_auto_test_cfg.secondary_offset =
                    step->secondary_offset;
                collision_auto_test_cfg.secondary_original[0] = secondary_v[0];
                collision_auto_test_cfg.secondary_original[1] = secondary_v[1];
                collision_auto_test_cfg.secondary_original[2] = secondary_v[2];
            }
            collision_auto_test_cfg.phase_tick = now;
            collision_auto_test_cfg.state = 1;
            collision_auto_test_log_snapshot("begin", step, person_index);
            log_line("collision-auto-test root-basis-write phase=%d/%d label=\"%s\" node=\"%s\" raw=%p offsets=(0x%03x,0x%03x,0x%03x) axis=%d amount=%.2f original_rows=((%.4f,%.4f,%.4f),(%.4f,%.4f,%.4f),(%.4f,%.4f,%.4f)) note=\"actual root basis, not root_target01\"",
                     collision_auto_test_cfg.phase + 1, step_count,
                     step->label, resolved_name, raw,
                     collision_auto_test_cfg.active_basis_offsets[0],
                     collision_auto_test_cfg.active_basis_offsets[1],
                     collision_auto_test_cfg.active_basis_offsets[2],
                     step->axis, amount,
                     collision_auto_test_cfg.active_original[0],
                     collision_auto_test_cfg.active_original[1],
                     collision_auto_test_cfg.active_original[2],
                     collision_auto_test_cfg.active_original[3],
                     collision_auto_test_cfg.active_original[4],
                     collision_auto_test_cfg.active_original[5],
                     collision_auto_test_cfg.active_original[6],
                     collision_auto_test_cfg.active_original[7],
                     collision_auto_test_cfg.active_original[8]);
            if (collision_auto_test_cfg.secondary_raw) {
                log_line("collision-auto-test secondary-write phase=%d/%d label=\"%s\" node=\"%s\" raw=%p offset=0x%03x axis=%d amount=%.2f original=(%.3f,%.3f,%.3f)",
                         collision_auto_test_cfg.phase + 1, step_count,
                         step->label, secondary_name,
                         collision_auto_test_cfg.secondary_raw,
                         collision_auto_test_cfg.secondary_offset,
                         step->secondary_axis,
                         collision_auto_test_channel_amount() *
                            step->secondary_sign,
                         collision_auto_test_cfg.secondary_original[0],
                         collision_auto_test_cfg.secondary_original[1],
                         collision_auto_test_cfg.secondary_original[2]);
            }
        }
        if (!collision_auto_test_apply_root_basis(raw, step->axis,
                                                  amount, 0)) {
            restore_collision_auto_test_active();
            collision_auto_test_cfg.state = 2;
            collision_auto_test_cfg.phase_tick = now;
            return;
        }
        if (collision_auto_test_cfg.secondary_raw) {
            secondary_v = (float*)((BYTE*)collision_auto_test_cfg.secondary_raw +
                                   collision_auto_test_cfg.secondary_offset);
            secondary_amount = collision_auto_test_channel_amount() *
                               step->secondary_sign;
            secondary_v[0] = collision_auto_test_cfg.secondary_original[0];
            secondary_v[1] = collision_auto_test_cfg.secondary_original[1];
            secondary_v[2] = collision_auto_test_cfg.secondary_original[2];
            secondary_v[step->secondary_axis] =
                collision_auto_test_cfg.secondary_original[step->secondary_axis] +
                secondary_amount;
        }
        if (now - collision_auto_test_cfg.phase_tick <
            (DWORD)collision_auto_test_cfg.phase_ms) {
            return;
        }
        collision_auto_test_log_snapshot("end", step, person_index);
        restore_collision_auto_test_active();
        log_line("collision-auto-test combined-restored phase=%d/%d label=\"%s\" node=\"%s\" root_nine_values_restored=1 secondary_restored=%d",
                 collision_auto_test_cfg.phase + 1, step_count,
                 step->label, resolved_name,
                 step->secondary_node && step->secondary_node[0]);
        collision_auto_test_cfg.state = 2;
        collision_auto_test_cfg.phase_tick = now;
        return;
    }

    if (!ptr_readable((BYTE*)raw + step->offset, sizeof(float) * 3)) {
        if (collision_auto_test_cfg.state == 0) {
            log_line("collision-auto-test skipped phase=%d/%d label=\"%s\" node=\"%s\" raw=%p offset=0x%03x reason=\"rotation channel unreadable\"",
                     collision_auto_test_cfg.phase + 1, step_count,
                     step->label, resolved_name, raw, step->offset);
            collision_auto_test_cfg.state = 2;
            collision_auto_test_cfg.phase_tick = now;
        }
        return;
    }
    v = (float*)((BYTE*)raw + step->offset);
    if (!sane_probe_float(v[0]) || !sane_probe_float(v[1]) ||
        !sane_probe_float(v[2])) {
        if (collision_auto_test_cfg.state == 0) {
            log_line("collision-auto-test skipped phase=%d/%d label=\"%s\" node=\"%s\" raw=%p offset=0x%03x reason=\"rotation channel contains invalid values\"",
                     collision_auto_test_cfg.phase + 1, step_count,
                     step->label, resolved_name, raw, step->offset);
            collision_auto_test_cfg.state = 2;
            collision_auto_test_cfg.phase_tick = now;
        }
        return;
    }

    if (collision_auto_test_cfg.state == 0 ||
        collision_auto_test_cfg.active_raw != raw) {
        collision_auto_test_cfg.active_raw = raw;
        collision_auto_test_cfg.active_offset = step->offset;
        collision_auto_test_cfg.active_axis = step->axis;
        collision_auto_test_cfg.active_original[0] = v[0];
        collision_auto_test_cfg.active_original[1] = v[1];
        collision_auto_test_cfg.active_original[2] = v[2];
        collision_auto_test_cfg.phase_tick = now;
        collision_auto_test_cfg.state = 1;
        collision_auto_test_log_snapshot("begin", step, person_index);
        log_line("collision-auto-test write phase=%d/%d label=\"%s\" node=\"%s\" raw=%p offset=0x%03x axis=%d amount=%.2f original=(%.3f,%.3f,%.3f)",
                 collision_auto_test_cfg.phase + 1, step_count,
                 step->label, resolved_name, raw, step->offset,
                 step->axis, amount,
                 collision_auto_test_cfg.active_original[0],
                 collision_auto_test_cfg.active_original[1],
                 collision_auto_test_cfg.active_original[2]);
    }

    if (collision_auto_test_cfg.active_raw != raw) return;
    v[0] = collision_auto_test_cfg.active_original[0];
    v[1] = collision_auto_test_cfg.active_original[1];
    v[2] = collision_auto_test_cfg.active_original[2];
    v[step->axis] = collision_auto_test_cfg.active_original[step->axis] + amount;
    if (now - collision_auto_test_cfg.phase_tick <
        (DWORD)collision_auto_test_cfg.phase_ms) {
        return;
    }

    collision_auto_test_log_snapshot("end", step, person_index);
    v[0] = collision_auto_test_cfg.active_original[0];
    v[1] = collision_auto_test_cfg.active_original[1];
    v[2] = collision_auto_test_cfg.active_original[2];
    log_line("collision-auto-test restored phase=%d/%d label=\"%s\" node=\"%s\" restored=(%.3f,%.3f,%.3f)",
             collision_auto_test_cfg.phase + 1, step_count,
             step->label, resolved_name,
             v[0], v[1], v[2]);
    collision_auto_test_cfg.state = 2;
    collision_auto_test_cfg.phase_tick = now;
    collision_auto_test_cfg.active_raw = NULL;
}

static const char *body_chain_person_name(int index)
{
    static const char *names[4] = { "Person01", "Person02", "Person03", "Person04" };
    if (index < 0 || index >= 4) return "Person02";
    return names[index];
}

static unsigned int restore_poseeditor_joint01_track(
    body_chain_person_state_t *state)
{
    BYTE *base;
    void *nil_weak = engine_G_NilWeakObjTarget_ptr ?
        *engine_G_NilWeakObjTarget_ptr : NULL;
    void *null_array = engine_G_NullArray_ptr ?
        *engine_G_NullArray_ptr : NULL;
    DWORD old;
    int i;
    unsigned int restored_mask = 0;
    if (!state) return 0;
    if (state->pose_track_suppressed && state->pose_track_base) {
        base = (BYTE*)state->pose_track_base;
        if (ptr_readable(base, POSEEDIT_TRACK_SIZE) &&
            *(void**)(base + 0x04) == nil_weak &&
            *(void**)(base + 0x24) == null_array &&
            VirtualProtect(base, POSEEDIT_TRACK_SIZE, PAGE_READWRITE, &old)) {
            *(void**)(base + 0x04) = state->pose_track_saved_obj;
            *(void**)(base + 0x24) = state->pose_track_saved_track_data;
            VirtualProtect(base, POSEEDIT_TRACK_SIZE, old, &old);
            restored_mask |= 1u;
        }
    }
    state->pose_track_suppressed = 0;
    state->pose_track_base = NULL;
    state->pose_track_saved_obj = NULL;
    state->pose_track_saved_track_data = NULL;
    for (i = 0; i < POSEEDIT_EXTRA_PHYSICS_TRACK_COUNT; i++) {
        if (state->pose_track_extra_suppressed[i] &&
            state->pose_track_extra_base[i]) {
            base = (BYTE*)state->pose_track_extra_base[i];
            if (ptr_readable(base, POSEEDIT_TRACK_SIZE) &&
                *(void**)(base + 0x04) == nil_weak &&
                *(void**)(base + 0x24) == null_array &&
                VirtualProtect(base, POSEEDIT_TRACK_SIZE,
                               PAGE_READWRITE, &old)) {
                *(void**)(base + 0x04) =
                    state->pose_track_extra_saved_obj[i];
                *(void**)(base + 0x24) =
                    state->pose_track_extra_saved_track_data[i];
                VirtualProtect(base, POSEEDIT_TRACK_SIZE, old, &old);
                restored_mask |= 1u << (i + 1);
            }
        }
        state->pose_track_extra_suppressed[i] = 0;
        state->pose_track_extra_base[i] = NULL;
        state->pose_track_extra_saved_obj[i] = NULL;
        state->pose_track_extra_saved_track_data[i] = NULL;
    }
    return restored_mask;
}

static BYTE *poseedit_track_slot(int person_index, int track_id)
{
    BYTE *editpose = (BYTE*)captured_poseedit_editpose;
    int track_index;
    if (person_index < 0 || person_index >= 4 || track_id < 0) return NULL;
    if (!editpose || !ptr_readable(editpose, captured_poseedit_tracks_offset + POSEEDIT_TRACK_SIZE)) return NULL;
    track_index = person_index * body_chain_physics_cfg.poseeditor_total_tracks + track_id;
    return editpose + captured_poseedit_tracks_offset + (track_index * POSEEDIT_TRACK_SIZE);
}

static int poseedit_current_frame(double *frame_out)
{
    BYTE *poseedit = (BYTE*)captured_poseedit_this;
    if (!frame_out || !poseedit ||
        !ptr_readable(poseedit + POSEEDIT_CURRENT_FRAME_OFFSET,
                      sizeof(int))) {
        return 0;
    }
    *frame_out =
        (double)*(int*)(poseedit + POSEEDIT_CURRENT_FRAME_OFFSET);
    return 1;
}

static int poseedit_track_apply_zero(BYTE *track,
                                     void *target_override,
                                     double current_frame)
{
    poseedit_track_update_t update_track;
    struct {
        int count;
        BYTE key[0x30];
    } zero_array;
    BYTE track_copy[POSEEDIT_TRACK_SIZE];
    void *track_obj;

    if (!track || !ptr_readable(track, POSEEDIT_TRACK_SIZE) ||
        !ptr_executable(POSEEDIT_TRACK_UPDATE_ADDR)) {
        return 0;
    }
    track_obj = target_override ? target_override : *(void**)(track + 0x04);
    if (!track_obj || !ptr_readable(track_obj, sizeof(DWORD))) {
        return 0;
    }
    memset(&zero_array, 0, sizeof(zero_array));
    memset(track_copy, 0, sizeof(track_copy));
    zero_array.count = 1;
    memcpy(track_copy, track, POSEEDIT_TRACK_SIZE);
    *(void**)(track_copy + 0x04) = track_obj;
    *(void**)(track_copy + 0x24) = zero_array.key;
    *(unsigned int*)(track_copy + 0x3c) &= ~1u;
    update_track = (poseedit_track_update_t)POSEEDIT_TRACK_UPDATE_ADDR;
    update_track(track_copy, current_frame);
    return 1;
}

static int validate_poseeditor_track_slot(BYTE *base, void *expected_obj)
{
    if (!base || !expected_obj || !ptr_readable(base, POSEEDIT_TRACK_SIZE)) return 0;
    return *(void**)(base + 0x04) == expected_obj;
}

static void *poseedit_scene_person(int person_index)
{
    BYTE *poseedit = (BYTE*)captured_poseedit_this;
    BYTE *scene_context;
    void **person_array;
    int count;
    int slot;
    if (person_index < 0 || person_index >= 4 || !poseedit ||
        !ptr_readable(poseedit + POSEEDIT_SCENE_CONTEXT_OFFSET,
                      sizeof(void*))) {
        return NULL;
    }
    scene_context = *(BYTE**)(poseedit + POSEEDIT_SCENE_CONTEXT_OFFSET);
    if (!scene_context ||
        !ptr_readable(scene_context + POSEEDIT_SCENE_PERSON_ARRAY_OFFSET,
                      sizeof(void*))) {
        return NULL;
    }
    person_array = *(void***)(scene_context +
                              POSEEDIT_SCENE_PERSON_ARRAY_OFFSET);
    if (!person_array ||
        !ptr_readable((BYTE*)person_array - sizeof(int), sizeof(int))) {
        return NULL;
    }
    count = *(int*)((BYTE*)person_array - sizeof(int));
    if (count < 0 || count > 64) {
        return NULL;
    }
    /* PoseEdit::InitTracks uses one-based person slots here. */
    slot = person_index + 1;
    if (slot >= count) {
        return NULL;
    }
    if (!ptr_readable(person_array, sizeof(void*) * (size_t)count)) {
        return NULL;
    }
    return person_array[slot];
}

static void *poseedit_person_state(int person_index)
{
    BYTE *poseedit = (BYTE*)captured_poseedit_this;
    BYTE *person_module;
    void **person_state_array;
    int count;
    int slot;
    if (person_index < 0 || person_index >= 4 || !poseedit ||
        !ptr_readable(poseedit + POSEEDIT_PERSON_MODULE_OFFSET,
                      sizeof(void*))) {
        return NULL;
    }
    person_module = *(BYTE**)(poseedit + POSEEDIT_PERSON_MODULE_OFFSET);
    if (!person_module ||
        !ptr_readable(person_module + PERSON_MODULE_PERSON_STATE_PTR_OFFSET,
                      sizeof(void*))) {
        return NULL;
    }
    person_state_array =
        *(void***)(person_module + PERSON_MODULE_PERSON_STATE_PTR_OFFSET);
    if (!person_state_array ||
        !ptr_readable((BYTE*)person_state_array - sizeof(int), sizeof(int))) {
        return NULL;
    }
    count = *(int*)((BYTE*)person_state_array - sizeof(int));
    if (count < 0 || count > 64) {
        return NULL;
    }
    slot = person_index + 1;
    if (slot >= count) {
        return NULL;
    }
    if (!ptr_readable(person_state_array, sizeof(void*) * (size_t)count)) {
        return NULL;
    }
    return person_state_array[slot];
}

static void *runtime_person_module(void)
{
    BYTE *main_object;
    BYTE *person_module;
    void *module_app_base;
    if (!engine_AppMainUserMain) return NULL;
    main_object = (BYTE*)engine_AppMainUserMain();
    if (!main_object ||
        !ptr_readable(main_object + MAIN_PERSON_MODULE_OFFSET,
                      sizeof(void*))) {
        return NULL;
    }
    person_module =
        *(BYTE**)(main_object + MAIN_PERSON_MODULE_OFFSET);
    if (!person_module ||
        !ptr_readable(person_module + sizeof(void*), sizeof(void*)) ||
        !ptr_readable(person_module + PERSON_MODULE_PERSON_STATE_PTR_OFFSET,
                      sizeof(void*))) {
        return NULL;
    }
    module_app_base = *(void**)(person_module + sizeof(void*));
    if (!module_app_base ||
        (captured_app_base && module_app_base != captured_app_base)) {
        return NULL;
    }
    return person_module;
}

static void *runtime_person_state(int person_index)
{
    BYTE *person_module = (BYTE*)runtime_person_module();
    void **person_state_array;
    int count;
    int slot;
    if (person_index < 0 || person_index >= 4 || !person_module) {
        return NULL;
    }
    person_state_array =
        *(void***)(person_module + PERSON_MODULE_PERSON_STATE_PTR_OFFSET);
    if (!person_state_array ||
        !ptr_readable((BYTE*)person_state_array - sizeof(int),
                      sizeof(int))) {
        return NULL;
    }
    count = *(int*)((BYTE*)person_state_array - sizeof(int));
    slot = person_index + 1;
    if (count < 0 || count > 64 || slot >= count ||
        !ptr_readable(person_state_array,
                      sizeof(void*) * (size_t)count)) {
        return NULL;
    }
    return person_state_array[slot];
}

static void *body_chain_mode_person_state(int person_index)
{
    if (InterlockedCompareExchange(&body_chain_poseeditor_mode_active,
                                   0, 0)) {
        return poseedit_person_state(person_index);
    }
    return runtime_person_state(person_index);
}

static int poseedit_scene_person_visible(int person_index)
{
    BYTE *person = (BYTE*)poseedit_scene_person(person_index);
    BYTE *script_object;
    BYTE *metadata;
    BYTE *dispatch_table;
    script_bool_property_t getter;
    int visible;

    if (!person ||
        !ptr_readable(person + SCENE_PERSON_SCRIPT_OBJECT_OFFSET,
                      sizeof(void*))) {
        return -1;
    }
    script_object = *(BYTE**)(person + SCENE_PERSON_SCRIPT_OBJECT_OFFSET);
    if (!script_object ||
        !ptr_readable(script_object - SCRIPT_OBJECT_META_BACK_OFFSET,
                      sizeof(void*))) {
        return -1;
    }
    metadata = *(BYTE**)(script_object - SCRIPT_OBJECT_META_BACK_OFFSET);
    if (!metadata ||
        !ptr_readable(metadata + SCRIPT_OBJECT_DISPATCH_TABLE_OFFSET,
                      sizeof(void*))) {
        return -1;
    }
    dispatch_table = *(BYTE**)(metadata +
                               SCRIPT_OBJECT_DISPATCH_TABLE_OFFSET);
    if (!dispatch_table ||
        !ptr_readable(dispatch_table + SCRIPT_OBJECT_BOOL_DISPATCH_OFFSET,
                      sizeof(void*))) {
        return -1;
    }
    getter = *(script_bool_property_t*)(dispatch_table +
                                        SCRIPT_OBJECT_BOOL_DISPATCH_OFFSET);
    if (!ptr_executable((const void*)getter)) {
        return -1;
    }

    /* Same predicate used by TK17 Camera_SetFirstPerson at 0x0048D13C. */
    visible = getter(script_object, SCRIPT_PROPERTY_PERSON_VISIBLE);
    return visible == 1;
}

static int physx_safe_cstr_equals_i(const char *s, const char *expected)
{
    char buf[32];
    int i;
    if (!s || !expected) return 0;
    if (!safe_cstr_a(s, sizeof(buf) - 1)) return 0;
    for (i = 0; i < (int)sizeof(buf) - 1; i++) {
        if (!ptr_readable(s + i, 1)) return 0;
        buf[i] = s[i];
        if (!buf[i]) break;
    }
    buf[sizeof(buf) - 1] = 0;
    return _stricmp(buf, expected) == 0;
}

static int physx_write_int_value(int *ptr, int value)
{
    DWORD old;
    if (!ptr || !ptr_readable(ptr, sizeof(int))) return 0;
    if (!VirtualProtect(ptr, sizeof(int), PAGE_READWRITE, &old)) return 0;
    *ptr = value;
    VirtualProtect(ptr, sizeof(int), old, &old);
    return 1;
}

static int physx_write_byte_value(BYTE *ptr, BYTE value)
{
    DWORD old;
    if (!ptr || !ptr_readable(ptr, sizeof(BYTE))) return 0;
    if (!VirtualProtect(ptr, sizeof(BYTE), PAGE_READWRITE, &old)) return 0;
    *ptr = value;
    VirtualProtect(ptr, sizeof(BYTE), old, &old);
    return 1;
}

static int tk17_testicle_inertia_values_sane(int enable_value,
                                              int field8_value)
{
    /* Enable's low byte is the script Boolean. Its upper bytes are not
       consistently initialized, so they must never be overwritten. */
    return (enable_value & 0xff) <= 1 &&
           (field8_value == 0 || field8_value == 1);
}

static int resolve_tk17_testicle_inertia_enable(
    int person_index, const char *person_name,
    void **person_state_out, void **person_inertia_out,
    int **enable_ptr_out, int *enable_value_out)
{
    BYTE *poseedit;
    BYTE *person_module;
    BYTE *person_state;
    BYTE *person_inertia;
    void *module_app_base;
    void *inertia_app_base;
    int person_slot = 0;
    int *enable_ptr;
    int enable_value;
    int field8_value;

    if (person_state_out) *person_state_out = NULL;
    if (person_inertia_out) *person_inertia_out = NULL;
    if (enable_ptr_out) *enable_ptr_out = NULL;
    if (enable_value_out) *enable_value_out = 0;
    if (person_index < 0 || person_index >= 4 || !person_name) return 0;

    poseedit = (BYTE*)captured_poseedit_this;
    if (InterlockedCompareExchange(&body_chain_poseeditor_mode_active,
                                   0, 0)) {
        if (!poseedit ||
            !ptr_readable(poseedit + POSEEDIT_PERSON_MODULE_OFFSET,
                          sizeof(void*))) {
            return 0;
        }
        person_module =
            *(BYTE**)(poseedit + POSEEDIT_PERSON_MODULE_OFFSET);
    } else {
        person_module = (BYTE*)runtime_person_module();
    }
    if (!person_module ||
        !ptr_readable(person_module + sizeof(void*), sizeof(void*))) {
        return 0;
    }
    module_app_base = *(void**)(person_module + sizeof(void*));

    person_state = (BYTE*)body_chain_mode_person_state(person_index);
    if (!person_state ||
        !ptr_readable(person_state + PERSON_STATE_PERSON_INDEX_OFFSET,
                      sizeof(int)) ||
        !ptr_readable(person_state + PERSON_STATE_INERTIA_OFFSET,
                      sizeof(void*))) {
        return 0;
    }

    person_slot = *(int*)(person_state + PERSON_STATE_PERSON_INDEX_OFFSET);
    person_inertia = *(BYTE**)(person_state + PERSON_STATE_INERTIA_OFFSET);
    if (!person_inertia ||
        !ptr_readable(person_inertia, sizeof(void*)) ||
        !ptr_readable(person_inertia + PERSON_INERTIA_TEST_ENABLE_OFFSET,
                      sizeof(int)) ||
        !ptr_readable(person_inertia + PERSON_INERTIA_TEST_FLAGS_OFFSET,
                      sizeof(int)) ||
        !ptr_readable(person_inertia + PERSON_INERTIA_TEST_FIELD8_OFFSET,
                      sizeof(int))) {
        return 0;
    }
    inertia_app_base = *(void**)person_inertia;
    if (!module_app_base || inertia_app_base != module_app_base) {
        return 0;
    }

    /* PersonInertia's strings are asset identifiers, not stable PersonXX
       runtime labels. Ownership comes from the live PoseEdit person slot. */
    if (person_slot != person_index &&
        person_slot != person_index + 1) {
        return 0;
    }

    enable_ptr =
        (int*)(person_inertia + PERSON_INERTIA_TEST_ENABLE_OFFSET);
    enable_value = *enable_ptr;
    field8_value =
        *(int*)(person_inertia + PERSON_INERTIA_TEST_FIELD8_OFFSET);
    if (!tk17_testicle_inertia_values_sane(enable_value, field8_value)) {
        return 0;
    }

    if (person_state_out) *person_state_out = person_state;
    if (person_inertia_out) *person_inertia_out = person_inertia;
    if (enable_ptr_out) *enable_ptr_out = enable_ptr;
    if (enable_value_out) *enable_value_out = enable_value;
    return 1;
}

static void restore_tk17_testicle_inertia_for_person(int person_index,
                                                     DWORD now)
{
    tk17_testicle_inertia_gate_state_t *gate;
    const char *person_name;
    void *live_person_state = NULL;
    void *live_person_inertia = NULL;
    int *live_enable_ptr = NULL;
    int live_enable_value = 0;
    int live_mapping = 0;
    int current = -1;
    if (person_index < 0 || person_index >= 4) return;
    person_name = body_chain_person_name(person_index);
    gate = &tk17_testicle_inertia_gate_states[person_index];
    if (gate->active && gate->saved_valid) {
        live_mapping =
            resolve_tk17_testicle_inertia_enable(
                person_index, person_name,
                &live_person_state, &live_person_inertia,
                &live_enable_ptr, &live_enable_value) &&
            live_person_state == gate->person_state &&
            live_person_inertia == gate->person_inertia &&
            live_enable_ptr == gate->enable_ptr &&
            gate->flags_ptr ==
                (int*)((BYTE*)live_person_inertia +
                       PERSON_INERTIA_TEST_FLAGS_OFFSET) &&
            gate->field8_ptr ==
                (int*)((BYTE*)live_person_inertia +
                       PERSON_INERTIA_TEST_FIELD8_OFFSET);
    }
    if (live_mapping &&
        ptr_readable(gate->enable_ptr, sizeof(int)) &&
        ptr_readable(gate->flags_ptr, sizeof(int)) &&
        ptr_readable(gate->field8_ptr, sizeof(int)) &&
        tk17_testicle_inertia_values_sane(*gate->enable_ptr,
                                          *gate->field8_ptr)) {
        current = *gate->enable_ptr;
        if ((current & 0xff) != gate->saved_enable) {
            physx_write_byte_value((BYTE*)gate->enable_ptr,
                                   (BYTE)gate->saved_enable);
        }
        if (defaults_cfg.debug &&
            (!gate->log_tick || now - gate->log_tick >= 1000)) {
            log_line("testicle-physics tk17-testicle-inertia restored person=\"%s\" enable_raw_before=%d enable_byte_restore=%d flags_observed=0x%x field8_observed=%d person_state=%p person_inertia=%p note=\"Testicle PhysX no longer owns this person; only Enable's low Boolean byte was restored; internal runtime fields were never modified\"",
                     body_chain_person_name(person_index),
                     current,
                     current & 0xff,
                     *gate->flags_ptr,
                     *gate->field8_ptr,
                     gate->person_state,
                     gate->person_inertia);
        }
    } else if (gate->active && defaults_cfg.debug) {
        log_line("testicle-physics tk17-testicle-inertia restore skipped person=\"%s\" reason=\"live PersonInertia ownership changed or fields were invalid\" saved_person_state=%p live_person_state=%p saved_person_inertia=%p live_person_inertia=%p note=\"fail-closed; no stale inertia pointer was written\"",
                 person_name,
                 gate->person_state, live_person_state,
                 gate->person_inertia, live_person_inertia);
    }
    memset(gate, 0, sizeof(*gate));
}

static void restore_tk17_testicle_inertia_all(DWORD now)
{
    int i;
    for (i = 0; i < 4; i++) {
        restore_tk17_testicle_inertia_for_person(i, now);
    }
}

static int suppress_tk17_testicle_inertia_for_person(int person_index,
                                                     DWORD now)
{
    const char *person_name = body_chain_person_name(person_index);
    tk17_testicle_inertia_gate_state_t *gate;
    void *person_state = NULL;
    void *person_inertia = NULL;
    int *enable_ptr = NULL;
    int *flags_ptr = NULL;
    int *field8_ptr = NULL;
    int enable_value = 0;
    int flags_value = 0;
    int field8_value = 0;
    int wrote = 0;
    int cached_gate = 0;
    BYTE *current_person_state = NULL;

    if (person_index < 0 || person_index >= 4) return 0;
    gate = &tk17_testicle_inertia_gate_states[person_index];
    current_person_state =
        (BYTE*)body_chain_mode_person_state(person_index);
    cached_gate =
        gate->active && gate->person_state && gate->person_inertia &&
        current_person_state == gate->person_state &&
        ptr_readable(current_person_state + PERSON_STATE_INERTIA_OFFSET,
                     sizeof(void*)) &&
        *(void**)(current_person_state + PERSON_STATE_INERTIA_OFFSET) ==
            gate->person_inertia &&
        gate->enable_ptr && gate->flags_ptr && gate->field8_ptr &&
        ptr_readable(gate->enable_ptr, sizeof(int)) &&
        ptr_readable(gate->flags_ptr, sizeof(int)) &&
        ptr_readable(gate->field8_ptr, sizeof(int)) &&
        gate->verify_tick &&
        now - gate->verify_tick <
            (DWORD)TESTICLE_INERTIA_CACHE_VERIFY_MS;
    if (cached_gate) {
        person_state = gate->person_state;
        person_inertia = gate->person_inertia;
        enable_ptr = gate->enable_ptr;
        flags_ptr = gate->flags_ptr;
        field8_ptr = gate->field8_ptr;
        enable_value = *enable_ptr;
    } else {
        if (!gate->active && gate->retry_tick &&
            now - gate->retry_tick <
                (DWORD)TESTICLE_PHYSICS_MISSING_RETRY_MS) {
            return 0;
        }
        if (!resolve_tk17_testicle_inertia_enable(person_index, person_name,
                                                  &person_state,
                                                  &person_inertia,
                                                  &enable_ptr,
                                                  &enable_value)) {
            gate->retry_tick = now;
            gate->verify_tick = 0;
            if (defaults_cfg.debug &&
                (!gate->wait_log_tick || now - gate->wait_log_tick >= 2000)) {
                gate->wait_log_tick = now;
                log_line("testicle-physics tk17-testicle-inertia waiting person=\"%s\" reason=\"PoseEdit.person_module->person_state_ptr/PersonInertia/test_inertia.Enable not resolved yet\"",
                         person_name);
            }
            return 0;
        }
        gate->verify_tick = now;
        gate->retry_tick = 0;
    }
    if (!person_inertia) {
        if (defaults_cfg.debug &&
            (!gate->wait_log_tick || now - gate->wait_log_tick >= 2000)) {
            gate->wait_log_tick = now;
            log_line("testicle-physics tk17-testicle-inertia waiting person=\"%s\" reason=\"PoseEdit.person_module->person_state_ptr/PersonInertia/test_inertia.Enable not resolved yet\"",
                     person_name);
        }
        return 0;
    }

    if (gate->active && gate->enable_ptr && gate->enable_ptr != enable_ptr) {
        memset(gate, 0, sizeof(*gate));
    }
    if (!flags_ptr) {
        flags_ptr =
            (int*)((BYTE*)person_inertia + PERSON_INERTIA_TEST_FLAGS_OFFSET);
    }
    if (!field8_ptr) {
        field8_ptr =
            (int*)((BYTE*)person_inertia + PERSON_INERTIA_TEST_FIELD8_OFFSET);
    }
    if (!ptr_readable(flags_ptr, sizeof(int)) ||
        !ptr_readable(field8_ptr, sizeof(int))) {
        return 0;
    }
    flags_value = *flags_ptr;
    field8_value = *field8_ptr;
    if (!tk17_testicle_inertia_values_sane(enable_value, field8_value)) {
        if (defaults_cfg.debug &&
            (!gate->wait_log_tick || now - gate->wait_log_tick >= 2000)) {
            gate->wait_log_tick = now;
            log_line("testicle-physics tk17-testicle-inertia rejected person=\"%s\" enable=%d flags=0x%x field8=%d person_state=%p person_inertia=%p reason=\"fields do not match live PersonInertia.test_inertia layout\"",
                     person_name, enable_value, flags_value, field8_value,
                     person_state, person_inertia);
        }
        memset(gate, 0, sizeof(*gate));
        gate->retry_tick = now;
        return 0;
    }
    if (!gate->active) {
        gate->person_state = person_state;
        gate->person_inertia = person_inertia;
        gate->enable_ptr = enable_ptr;
        gate->flags_ptr = flags_ptr;
        gate->field8_ptr = field8_ptr;
        gate->saved_enable =
            InterlockedCompareExchange(&body_chain_poseeditor_mode_active,
                                       0, 0)
                ? (enable_value & 0xff)
                : 1;
        gate->saved_flags = flags_value;
        gate->saved_field8 = field8_value;
        gate->saved_valid = 1;
        gate->active = 1;
        gate->verify_tick = now;
        gate->retry_tick = 0;
    }

    if ((enable_value & 0xff) != 0) {
        wrote = physx_write_byte_value((BYTE*)enable_ptr, 0);
    }
    if (defaults_cfg.debug &&
        (wrote ||
         !gate->log_tick || now - gate->log_tick >= 5000)) {
        gate->log_tick = now;
        log_line("testicle-physics tk17-testicle-inertia disabled person=\"%s\" enable_raw_before=%d enable_byte_before=%d flags_observed=0x%x field8_observed=%d wrote=%d person_state=%p person_inertia=%p note=\"plugin-only targeted gate; only Enable's low Boolean byte is written; internal runtime fields remain engine-owned\"",
                 person_name,
                 enable_value,
                 enable_value & 0xff,
                 flags_value,
                 field8_value,
                 wrote,
                 person_state,
                 person_inertia);
    }
    return 1;
}

/* Breast and butt inertia live in the same PersonInertia block as the
   already-cached testicle gate. Reuse a verified mapping briefly while
   continuing to check the owning PersonState, block pointer, and fields on
   every access. */
static int tk17_inertia_gate_cache_valid(
    const tk17_testicle_inertia_gate_state_t *gate,
    int person_index, DWORD now)
{
    BYTE *current_person_state;
    if (!gate || person_index < 0 || person_index >= 4 ||
        !gate->active || !gate->person_state || !gate->person_inertia ||
        !gate->enable_ptr || !gate->flags_ptr || !gate->field8_ptr ||
        !gate->verify_tick ||
        now - gate->verify_tick >=
            (DWORD)TESTICLE_INERTIA_CACHE_VERIFY_MS) {
        return 0;
    }
    current_person_state =
        (BYTE*)body_chain_mode_person_state(person_index);
    return current_person_state == gate->person_state &&
        ptr_readable(current_person_state + PERSON_STATE_INERTIA_OFFSET,
                     sizeof(void*)) &&
        *(void**)(current_person_state + PERSON_STATE_INERTIA_OFFSET) ==
            gate->person_inertia &&
        ptr_readable(gate->enable_ptr, sizeof(int)) &&
        ptr_readable(gate->flags_ptr, sizeof(int)) &&
        ptr_readable(gate->field8_ptr, sizeof(int));
}

static void tk17_breasts_inertia_offsets(int side, int *enable_offset,
                                         int *flags_offset,
                                         int *field8_offset)
{
    if (side == 0) {
        *enable_offset = PERSON_INERTIA_BREAST_L_ENABLE_OFFSET;
        *flags_offset = PERSON_INERTIA_BREAST_L_FLAGS_OFFSET;
        *field8_offset = PERSON_INERTIA_BREAST_L_FIELD8_OFFSET;
    } else {
        *enable_offset = PERSON_INERTIA_BREAST_R_ENABLE_OFFSET;
        *flags_offset = PERSON_INERTIA_BREAST_R_FLAGS_OFFSET;
        *field8_offset = PERSON_INERTIA_BREAST_R_FIELD8_OFFSET;
    }
}

static int resolve_tk17_breasts_inertia_enable(
    int person_index, int side, void **person_state_out,
    void **person_inertia_out, int **enable_ptr_out)
{
    void *person_state = NULL;
    void *person_inertia = NULL;
    int *test_enable = NULL;
    int test_enable_value = 0;
    int enable_offset, flags_offset, field8_offset;
    int *enable_ptr;
    int field8_value;
    if (side < 0 || side > 1) return 0;
    if (!resolve_tk17_testicle_inertia_enable(
            person_index, body_chain_person_name(person_index),
            &person_state, &person_inertia, &test_enable,
            &test_enable_value)) {
        return 0;
    }
    (void)test_enable;
    (void)test_enable_value;
    tk17_breasts_inertia_offsets(side, &enable_offset, &flags_offset,
                                 &field8_offset);
    if (!ptr_readable((BYTE*)person_inertia + enable_offset, sizeof(int)) ||
        !ptr_readable((BYTE*)person_inertia + flags_offset, sizeof(int)) ||
        !ptr_readable((BYTE*)person_inertia + field8_offset, sizeof(int))) {
        return 0;
    }
    enable_ptr = (int*)((BYTE*)person_inertia + enable_offset);
    field8_value = *(int*)((BYTE*)person_inertia + field8_offset);
    if (!tk17_testicle_inertia_values_sane(*enable_ptr, field8_value)) {
        return 0;
    }
    if (person_state_out) *person_state_out = person_state;
    if (person_inertia_out) *person_inertia_out = person_inertia;
    if (enable_ptr_out) *enable_ptr_out = enable_ptr;
    return 1;
}

static void restore_tk17_breasts_inertia_side(int person_index, int side,
                                              DWORD now)
{
    tk17_testicle_inertia_gate_state_t *gate;
    void *live_person_state = NULL;
    void *live_person_inertia = NULL;
    int *live_enable_ptr = NULL;
    int enable_offset, flags_offset, field8_offset;
    int live_mapping = 0;
    (void)now;
    if (person_index < 0 || person_index >= 4 || side < 0 || side > 1) return;
    gate = &tk17_breasts_inertia_gate_states[person_index][side];
    tk17_breasts_inertia_offsets(side, &enable_offset, &flags_offset,
                                 &field8_offset);
    if (gate->active && gate->saved_valid) {
        live_mapping = resolve_tk17_breasts_inertia_enable(
            person_index, side, &live_person_state, &live_person_inertia,
            &live_enable_ptr) &&
            live_person_state == gate->person_state &&
            live_person_inertia == gate->person_inertia &&
            live_enable_ptr == gate->enable_ptr &&
            gate->flags_ptr ==
                (int*)((BYTE*)live_person_inertia + flags_offset) &&
            gate->field8_ptr ==
                (int*)((BYTE*)live_person_inertia + field8_offset);
    }
    if (live_mapping &&
        ptr_readable(gate->enable_ptr, sizeof(int)) &&
        ptr_readable(gate->field8_ptr, sizeof(int)) &&
        tk17_testicle_inertia_values_sane(*gate->enable_ptr,
                                          *gate->field8_ptr)) {
        if ((*gate->enable_ptr & 0xff) != gate->saved_enable) {
            physx_write_byte_value((BYTE*)gate->enable_ptr,
                                   (BYTE)gate->saved_enable);
        }
        if (defaults_cfg.debug) {
            log_line("breasts-physics tk17-breast-inertia restored person=\"%s\" side=%s enable=%d",
                     body_chain_person_name(person_index),
                     side == 0 ? "left" : "right", gate->saved_enable);
        }
    } else if (gate->active && defaults_cfg.debug) {
        log_line("breasts-physics tk17-breast-inertia restore skipped person=\"%s\" side=%s reason=\"live ownership changed\"",
                 body_chain_person_name(person_index),
                 side == 0 ? "left" : "right");
    }
    memset(gate, 0, sizeof(*gate));
}

static void restore_tk17_breasts_inertia_for_person(int person_index,
                                                    DWORD now)
{
    restore_tk17_breasts_inertia_side(person_index, 0, now);
    restore_tk17_breasts_inertia_side(person_index, 1, now);
}

static void restore_tk17_breasts_inertia_all(DWORD now)
{
    int i;
    for (i = 0; i < 4; i++) {
        restore_tk17_breasts_inertia_for_person(i, now);
    }
}

static int suppress_tk17_breasts_inertia_side(int person_index, int side,
                                              DWORD now)
{
    tk17_testicle_inertia_gate_state_t *gate;
    void *person_state = NULL;
    void *person_inertia = NULL;
    int *enable_ptr = NULL;
    int *flags_ptr;
    int *field8_ptr;
    int enable_offset, flags_offset, field8_offset;
    int enable_value;
    int wrote = 0;
    int cached_gate = 0;
    if (person_index < 0 || person_index >= 4 || side < 0 || side > 1) return 0;
    gate = &tk17_breasts_inertia_gate_states[person_index][side];
    tk17_breasts_inertia_offsets(side, &enable_offset, &flags_offset,
                                 &field8_offset);
    if (tk17_inertia_gate_cache_valid(gate, person_index, now) &&
        tk17_testicle_inertia_values_sane(*gate->enable_ptr,
                                          *gate->field8_ptr)) {
        person_state = gate->person_state;
        person_inertia = gate->person_inertia;
        enable_ptr = gate->enable_ptr;
        flags_ptr = gate->flags_ptr;
        field8_ptr = gate->field8_ptr;
        cached_gate = 1;
    } else if (!resolve_tk17_breasts_inertia_enable(
                   person_index, side, &person_state, &person_inertia,
                   &enable_ptr)) {
        gate->retry_tick = now;
        if (defaults_cfg.debug &&
            (!gate->wait_log_tick || now - gate->wait_log_tick >= 2000u)) {
            gate->wait_log_tick = now;
            log_line("breasts-physics tk17-breast-inertia waiting person=\"%s\" side=%s",
                     body_chain_person_name(person_index),
                     side == 0 ? "left" : "right");
        }
        return 0;
    } else {
        flags_ptr = (int*)((BYTE*)person_inertia + flags_offset);
        field8_ptr = (int*)((BYTE*)person_inertia + field8_offset);
    }
    enable_value = *enable_ptr;
    if (!gate->active || gate->person_state != person_state ||
        gate->person_inertia != person_inertia ||
        gate->enable_ptr != enable_ptr) {
        if (gate->active) {
            restore_tk17_breasts_inertia_side(person_index, side, now);
            gate = &tk17_breasts_inertia_gate_states[person_index][side];
        }
        gate->person_state = person_state;
        gate->person_inertia = person_inertia;
        gate->enable_ptr = enable_ptr;
        gate->flags_ptr = flags_ptr;
        gate->field8_ptr = field8_ptr;
        gate->saved_enable =
            InterlockedCompareExchange(&body_chain_poseeditor_mode_active,
                                       0, 0)
                ? (enable_value & 0xff) : 1;
        gate->saved_flags = *flags_ptr;
        gate->saved_field8 = *field8_ptr;
        gate->saved_valid = 1;
        gate->active = 1;
    }
    if (!cached_gate) gate->verify_tick = now;
    gate->retry_tick = 0;
    if ((enable_value & 0xff) != 0) {
        wrote = physx_write_byte_value((BYTE*)enable_ptr, 0);
    }
    if (defaults_cfg.debug &&
        (wrote || !gate->log_tick || now - gate->log_tick >= 5000u)) {
        gate->log_tick = now;
        log_line("breasts-physics tk17-breast-inertia disabled person=\"%s\" side=%s enable_before=%d wrote=%d flags=0x%x field8=%d",
                 body_chain_person_name(person_index),
                 side == 0 ? "left" : "right", enable_value & 0xff,
                 wrote, *flags_ptr, *field8_ptr);
    }
    return 1;
}

static int suppress_tk17_breasts_inertia_for_person(int person_index,
                                                    DWORD now)
{
    if (!suppress_tk17_breasts_inertia_side(person_index, 0, now)) return 0;
    if (!suppress_tk17_breasts_inertia_side(person_index, 1, now)) {
        restore_tk17_breasts_inertia_side(person_index, 0, now);
        return 0;
    }
    return 1;
}

static void tk17_butt_inertia_offsets(int side, int *enable_offset,
                                      int *flags_offset,
                                      int *field8_offset)
{
    if (side == 0) {
        *enable_offset = PERSON_INERTIA_BUTT_L_ENABLE_OFFSET;
        *flags_offset = PERSON_INERTIA_BUTT_L_FLAGS_OFFSET;
        *field8_offset = PERSON_INERTIA_BUTT_L_FIELD8_OFFSET;
    } else {
        *enable_offset = PERSON_INERTIA_BUTT_R_ENABLE_OFFSET;
        *flags_offset = PERSON_INERTIA_BUTT_R_FLAGS_OFFSET;
        *field8_offset = PERSON_INERTIA_BUTT_R_FIELD8_OFFSET;
    }
}

static int resolve_tk17_butt_inertia_enable(
    int person_index, int side, void **person_state_out,
    void **person_inertia_out, int **enable_ptr_out)
{
    void *person_state = NULL;
    void *person_inertia = NULL;
    int *test_enable = NULL;
    int test_enable_value = 0;
    int enable_offset, flags_offset, field8_offset;
    int *enable_ptr;
    int field8_value;
    if (side < 0 || side > 1) return 0;
    if (!resolve_tk17_testicle_inertia_enable(
            person_index, body_chain_person_name(person_index),
            &person_state, &person_inertia, &test_enable,
            &test_enable_value)) {
        return 0;
    }
    (void)test_enable;
    (void)test_enable_value;
    tk17_butt_inertia_offsets(side, &enable_offset, &flags_offset,
                              &field8_offset);
    if (!ptr_readable((BYTE*)person_inertia + enable_offset, sizeof(int)) ||
        !ptr_readable((BYTE*)person_inertia + flags_offset, sizeof(int)) ||
        !ptr_readable((BYTE*)person_inertia + field8_offset, sizeof(int))) {
        return 0;
    }
    enable_ptr = (int*)((BYTE*)person_inertia + enable_offset);
    field8_value = *(int*)((BYTE*)person_inertia + field8_offset);
    /* Butt field_8 is engine runtime state, not the Boolean layout guard used
       by testicle/breast inertia. Native Butt inertia may change it after an
       OFF transition, so rejecting non-0/1 values prevents PhysX from ever
       reacquiring the same valid block. We own only Enable's low byte. */
    if ((*enable_ptr & 0xff) > 1) {
        return 0;
    }
    (void)field8_value;
    if (person_state_out) *person_state_out = person_state;
    if (person_inertia_out) *person_inertia_out = person_inertia;
    if (enable_ptr_out) *enable_ptr_out = enable_ptr;
    return 1;
}

static void restore_tk17_butt_inertia_side(int person_index, int side,
                                           DWORD now)
{
    tk17_testicle_inertia_gate_state_t *gate;
    void *live_person_state = NULL;
    void *live_person_inertia = NULL;
    int *live_enable_ptr = NULL;
    int enable_offset, flags_offset, field8_offset;
    int live_mapping = 0;
    (void)now;
    if (person_index < 0 || person_index >= 4 || side < 0 || side > 1) return;
    gate = &tk17_butt_inertia_gate_states[person_index][side];
    tk17_butt_inertia_offsets(side, &enable_offset, &flags_offset,
                              &field8_offset);
    if (gate->active && gate->saved_valid) {
        live_mapping = resolve_tk17_butt_inertia_enable(
            person_index, side, &live_person_state, &live_person_inertia,
            &live_enable_ptr) &&
            live_person_state == gate->person_state &&
            live_person_inertia == gate->person_inertia &&
            live_enable_ptr == gate->enable_ptr &&
            gate->flags_ptr ==
                (int*)((BYTE*)live_person_inertia + flags_offset) &&
            gate->field8_ptr ==
                (int*)((BYTE*)live_person_inertia + field8_offset);
    }
    if (live_mapping &&
        ptr_readable(gate->enable_ptr, sizeof(int)) &&
        ptr_readable(gate->field8_ptr, sizeof(int)) &&
        (*gate->enable_ptr & 0xff) <= 1) {
        if ((*gate->enable_ptr & 0xff) != gate->saved_enable) {
            physx_write_byte_value((BYTE*)gate->enable_ptr,
                                   (BYTE)gate->saved_enable);
        }
        if (defaults_cfg.debug) {
            log_line("butt-physics tk17-butt-inertia restored person=\"%s\" side=%s enable=%d",
                     body_chain_person_name(person_index),
                     side == 0 ? "left" : "right", gate->saved_enable);
        }
    } else if (gate->active && defaults_cfg.debug) {
        log_line("butt-physics tk17-butt-inertia restore skipped person=\"%s\" side=%s reason=\"live ownership changed\"",
                 body_chain_person_name(person_index),
                 side == 0 ? "left" : "right");
    }
    memset(gate, 0, sizeof(*gate));
}

static void restore_tk17_butt_inertia_for_person(int person_index,
                                                 DWORD now)
{
    restore_tk17_butt_inertia_side(person_index, 0, now);
    restore_tk17_butt_inertia_side(person_index, 1, now);
}

static void restore_tk17_butt_inertia_all(DWORD now)
{
    int i;
    for (i = 0; i < 4; i++) {
        restore_tk17_butt_inertia_for_person(i, now);
    }
}

static int suppress_tk17_butt_inertia_side(int person_index, int side,
                                           DWORD now)
{
    tk17_testicle_inertia_gate_state_t *gate;
    void *person_state = NULL;
    void *person_inertia = NULL;
    int *enable_ptr = NULL;
    int *flags_ptr;
    int *field8_ptr;
    int enable_offset, flags_offset, field8_offset;
    int enable_value;
    int wrote = 0;
    int cached_gate = 0;
    if (person_index < 0 || person_index >= 4 || side < 0 || side > 1) return 0;
    gate = &tk17_butt_inertia_gate_states[person_index][side];
    tk17_butt_inertia_offsets(side, &enable_offset, &flags_offset,
                              &field8_offset);
    if (tk17_inertia_gate_cache_valid(gate, person_index, now) &&
        (*gate->enable_ptr & 0xff) <= 1) {
        person_state = gate->person_state;
        person_inertia = gate->person_inertia;
        enable_ptr = gate->enable_ptr;
        flags_ptr = gate->flags_ptr;
        field8_ptr = gate->field8_ptr;
        cached_gate = 1;
    } else if (!resolve_tk17_butt_inertia_enable(
                   person_index, side, &person_state, &person_inertia,
                   &enable_ptr)) {
        gate->retry_tick = now;
        if (defaults_cfg.debug &&
            (!gate->wait_log_tick || now - gate->wait_log_tick >= 2000u)) {
            gate->wait_log_tick = now;
            log_line("butt-physics tk17-butt-inertia waiting person=\"%s\" side=%s",
                     body_chain_person_name(person_index),
                     side == 0 ? "left" : "right");
        }
        return 0;
    } else {
        flags_ptr = (int*)((BYTE*)person_inertia + flags_offset);
        field8_ptr = (int*)((BYTE*)person_inertia + field8_offset);
    }
    enable_value = *enable_ptr;
    if (!gate->active || gate->person_state != person_state ||
        gate->person_inertia != person_inertia ||
        gate->enable_ptr != enable_ptr) {
        if (gate->active) {
            restore_tk17_butt_inertia_side(person_index, side, now);
            gate = &tk17_butt_inertia_gate_states[person_index][side];
        }
        gate->person_state = person_state;
        gate->person_inertia = person_inertia;
        gate->enable_ptr = enable_ptr;
        gate->flags_ptr = flags_ptr;
        gate->field8_ptr = field8_ptr;
        /* Preserve the person's actual content/runtime setting. If native
           butt inertia was already disabled by game content, releasing
           PhysX ownership must not turn it back on. */
        gate->saved_enable = enable_value & 0xff;
        gate->saved_flags = *flags_ptr;
        gate->saved_field8 = *field8_ptr;
        gate->saved_valid = 1;
        gate->active = 1;
    }
    if (!cached_gate) gate->verify_tick = now;
    gate->retry_tick = 0;
    if ((enable_value & 0xff) != 0) {
        wrote = physx_write_byte_value((BYTE*)enable_ptr, 0);
    }
    if (defaults_cfg.debug &&
        (wrote || !gate->log_tick || now - gate->log_tick >= 5000u)) {
        gate->log_tick = now;
        log_line("butt-physics tk17-butt-inertia disabled person=\"%s\" side=%s enable_before=%d wrote=%d flags=0x%x field8=%d note=\"only Enable's low Boolean byte is written\"",
                 body_chain_person_name(person_index),
                 side == 0 ? "left" : "right", enable_value & 0xff,
                 wrote, *flags_ptr, *field8_ptr);
    }
    return 1;
}

static int suppress_tk17_butt_inertia_for_person(int person_index,
                                                 DWORD now)
{
    if (!suppress_tk17_butt_inertia_side(person_index, 0, now)) return 0;
    if (!suppress_tk17_butt_inertia_side(person_index, 1, now)) {
        restore_tk17_butt_inertia_side(person_index, 0, now);
        return 0;
    }
    return 1;
}

static void log_poseedit_fixed_offset_candidate(void *pe, int pe_field_offset)
{
    BYTE *editpose;
    BYTE *track144;
    BYTE *track149;
    int person_index = 1;
    int track144_index;
    int track149_index;
    void *slot144_obj = NULL;
    void *slot144_data = NULL;
    void *slot149_obj = NULL;
    void *slot149_data = NULL;

    if (!pe || !ptr_readable((BYTE*)pe + pe_field_offset, sizeof(void*))) {
        log_line("PoseEdit fixed-offset candidate pe=%p pe_field_offset=0x%03x editpose=%p readable=0 note=\"fixed offset only; no scan/no write\"",
                 pe, pe_field_offset, NULL);
        return;
    }

    editpose = *(BYTE**)((BYTE*)pe + pe_field_offset);
    if (!editpose) {
        log_line("PoseEdit fixed-offset candidate pe=%p pe_field_offset=0x%03x editpose=%p readable=0 note=\"fixed offset only; null candidate\"",
                 pe, pe_field_offset, editpose);
        return;
    }

    track144_index = person_index * body_chain_physics_cfg.poseeditor_total_tracks + POSEEDIT_TRACK_PENIS_JOINT01;
    track149_index = person_index * body_chain_physics_cfg.poseeditor_total_tracks + POSEEDIT_TRACK_TESTICLES_JOINT01;
    track144 = editpose + POSEEDIT_TRACKS_OFFSET + (track144_index * POSEEDIT_TRACK_SIZE);
    track149 = editpose + POSEEDIT_TRACKS_OFFSET + (track149_index * POSEEDIT_TRACK_SIZE);

    if (ptr_readable(track144, POSEEDIT_TRACK_SIZE)) {
        slot144_obj = ptr_readable(track144 + 0x04, sizeof(void*)) ? *(void**)(track144 + 0x04) : NULL;
        slot144_data = ptr_readable(track144 + 0x24, sizeof(void*)) ? *(void**)(track144 + 0x24) : NULL;
    }
    if (ptr_readable(track149, POSEEDIT_TRACK_SIZE)) {
        slot149_obj = ptr_readable(track149 + 0x04, sizeof(void*)) ? *(void**)(track149 + 0x04) : NULL;
        slot149_data = ptr_readable(track149 + 0x24, sizeof(void*)) ? *(void**)(track149 + 0x24) : NULL;
    }

    log_line("PoseEdit fixed-offset candidate pe=%p pe_field_offset=0x%03x editpose=%p tracks=%p total_tracks=%d person=2 track144=%p obj144=%p data144=%p track149=%p obj149=%p data149=%p readable144=%d readable149=%d note=\"fixed offset only; no scan/no write\"",
             pe,
             pe_field_offset,
             editpose,
             editpose + POSEEDIT_TRACKS_OFFSET,
             body_chain_physics_cfg.poseeditor_total_tracks,
             track144,
             slot144_obj,
             slot144_data,
             track149,
             slot149_obj,
             slot149_data,
             ptr_readable(track144, POSEEDIT_TRACK_SIZE),
             ptr_readable(track149, POSEEDIT_TRACK_SIZE));
}

static void log_poseedit_fixed_offsets(void *pe)
{
    static const int offsets[] = { 0x2ec, 0x2f0, 0x2f4, 0x300, 0x304, 0x308 };
    static void *last_logged_pe;
    int i;
    if (!body_chain_physics_cfg.poseeditor_track_diagnostic) return;
    if (last_logged_pe == pe) return;
    last_logged_pe = pe;
    for (i = 0; i < (int)(sizeof(offsets) / sizeof(offsets[0])); ++i) {
        log_poseedit_fixed_offset_candidate(pe, offsets[i]);
    }
}

static BYTE *scan_poseedit_candidate_for_track(BYTE *editpose,
                                               int person_index,
                                               int track_id,
                                               void *expected_obj,
                                               void *neighbor_obj,
                                               int neighbor_track_id,
                                               int pe_field_offset)
{
    BYTE *scan;
    BYTE *end;
    size_t scan_bytes;
    int expected_index;
    if (!editpose || !expected_obj || person_index < 0 || person_index >= 4) return NULL;
    expected_index = person_index * body_chain_physics_cfg.poseeditor_total_tracks + track_id;
    scan_bytes = (size_t)body_chain_physics_cfg.poseeditor_total_tracks * 4u * POSEEDIT_TRACK_SIZE + 0x1000u;
    if (scan_bytes > 0x80000u) scan_bytes = 0x80000u;
    if (!ptr_readable(editpose, scan_bytes)) return NULL;
    scan = editpose;
    end = editpose + scan_bytes;
    for (; scan + sizeof(void*) <= end; scan += sizeof(void*)) {
        BYTE *base;
        int detected_offset;
        BYTE *neighbor_base;
        if (*(void**)scan != expected_obj) continue;
        if (scan < editpose + 0x04) continue;
        base = scan - 0x04;
        if (!ptr_readable(base, POSEEDIT_TRACK_SIZE)) continue;
        if (*(void**)(base + 0x04) != expected_obj) continue;
        detected_offset = (int)(base - editpose) - (expected_index * POSEEDIT_TRACK_SIZE);
        if (detected_offset < 0 || detected_offset > 0x4000) continue;
        neighbor_base = editpose + detected_offset +
            ((person_index * body_chain_physics_cfg.poseeditor_total_tracks + neighbor_track_id) * POSEEDIT_TRACK_SIZE);
        if (neighbor_obj && ptr_readable(neighbor_base, POSEEDIT_TRACK_SIZE) &&
            *(void**)(neighbor_base + 0x04) != neighbor_obj) {
            continue;
        }
        if (captured_poseedit_tracks_offset != detected_offset ||
            captured_poseedit_editpose != editpose ||
            !captured_poseedit_tracks_offset_detected) {
            captured_poseedit_editpose = editpose;
            captured_poseedit_tracks_offset = detected_offset;
            captured_poseedit_tracks_offset_detected = 1;
            log_line("PoseEdit track table offset detected pe=%p pe_field_offset=0x%03x editpose=%p tracks_offset=0x%03x total_tracks=%d person_index=%d track_id=%d base=%p expected_obj=%p neighbor_track_id=%d neighbor_obj=%p note=\"derived from live PoseTrack object pointer inside captured PoseEdit object only\"",
                     captured_poseedit_this,
                     pe_field_offset,
                     editpose,
                     captured_poseedit_tracks_offset,
                     body_chain_physics_cfg.poseeditor_total_tracks,
                     person_index + 1,
                     track_id,
                     base,
                     expected_obj,
                     neighbor_track_id,
                     neighbor_obj);
        }
        return base;
    }
    return NULL;
}

static BYTE *find_poseedit_track_slot_by_object(int person_index,
                                                int track_id,
                                                void *expected_obj,
                                                void *neighbor_obj,
                                                int neighbor_track_id)
{
    /* Keep this deliberately narrow. Scanning arbitrary PoseEdit pointer fields
       caused crashes in PoseEditor; the next fix should hook the playback writer. */
    return scan_poseedit_candidate_for_track((BYTE*)captured_poseedit_editpose,
                                            person_index, track_id,
                                            expected_obj, neighbor_obj,
                                            neighbor_track_id,
                                            POSEEDIT_EDITPOSE_OFFSET);
}

static unsigned int restore_poseeditor_paired_bone_tracks(
    poseeditor_paired_bone_track_state_t *ownership, int track_count)
{
    void *nil_weak = engine_G_NilWeakObjTarget_ptr ?
        *engine_G_NilWeakObjTarget_ptr : NULL;
    void *null_array = engine_G_NullArray_ptr ?
        *engine_G_NullArray_ptr : NULL;
    unsigned int restored_mask = 0;
    int i;
    if (!ownership) return 0;
    if (track_count < 0) track_count = 0;
    if (track_count > POSEEDIT_PAIRED_BONE_TRACK_MAX) {
        track_count = POSEEDIT_PAIRED_BONE_TRACK_MAX;
    }
    for (i = 0; i < track_count; i++) {
        BYTE *base = (BYTE*)ownership->base[i];
        void *current_obj;
        void *current_track_data;
        int needs_write;
        DWORD old;
        if (ownership->suppressed[i] && base &&
            ptr_readable(base, POSEEDIT_TRACK_SIZE)) {
            current_obj = *(void**)(base + 0x04);
            current_track_data = *(void**)(base + 0x24);
            needs_write =
                (current_obj == nil_weak && ownership->saved_obj[i]) ||
                (current_track_data == null_array &&
                 ownership->saved_track_data[i]);
            if (needs_write &&
                VirtualProtect(base, POSEEDIT_TRACK_SIZE,
                               PAGE_READWRITE, &old)) {
                /* PoseEditor can install a new keyframe array while a track
                   is suppressed. Keep that newer array and restore only the
                   ownership field that still contains our sentinel. */
                if (current_obj == nil_weak && ownership->saved_obj[i]) {
                    *(void**)(base + 0x04) = ownership->saved_obj[i];
                }
                if (current_track_data == null_array &&
                    ownership->saved_track_data[i]) {
                    *(void**)(base + 0x24) =
                        ownership->saved_track_data[i];
                }
                VirtualProtect(base, POSEEDIT_TRACK_SIZE, old, &old);
            }
            current_obj = *(void**)(base + 0x04);
            current_track_data = *(void**)(base + 0x24);
            if (current_obj && current_obj != nil_weak &&
                current_track_data && current_track_data != null_array) {
                restored_mask |= 1u << i;
            }
        }
        ownership->base[i] = NULL;
        ownership->saved_obj[i] = NULL;
        ownership->saved_track_data[i] = NULL;
        ownership->suppressed[i] = 0;
    }
    ownership->logged = 0;
    ownership->wait_log_tick = 0;
    return restored_mask;
}

static int suppress_poseeditor_paired_bone_tracks(
    int person_index, const char *person, const char *system_name,
    poseeditor_paired_bone_track_state_t *ownership,
    const poseeditor_paired_bone_track_def_t *defs, int track_count)
{
    BYTE *base[POSEEDIT_PAIRED_BONE_TRACK_MAX] = { NULL };
    void *target_obj[POSEEDIT_PAIRED_BONE_TRACK_MAX] = { NULL };
    void *target_raw[POSEEDIT_PAIRED_BONE_TRACK_MAX] = { NULL };
    void *track_target[POSEEDIT_PAIRED_BONE_TRACK_MAX] = { NULL };
    void *track_data[POSEEDIT_PAIRED_BONE_TRACK_MAX] = { NULL };
    int needs_bind[POSEEDIT_PAIRED_BONE_TRACK_MAX] = { 0 };
    void *nil_weak = engine_G_NilWeakObjTarget_ptr ?
        *engine_G_NilWeakObjTarget_ptr : NULL;
    void *null_array = engine_G_NullArray_ptr ?
        *engine_G_NullArray_ptr : NULL;
    DWORD now = GetTickCount();
    int i;

    if (person_index < 0 || person_index >= 4 || !person || !system_name ||
        !ownership || !defs || track_count < 1 ||
        track_count > POSEEDIT_PAIRED_BONE_TRACK_MAX ||
        !captured_poseedit_this || !captured_poseedit_editpose ||
        !nil_weak || !null_array) {
        return 0;
    }

    for (i = 0; i < track_count; i++) {
        char object_name[256];
        void *slot_obj;
        void *slot_data;
        base[i] = poseedit_track_slot(person_index, defs[i].track_id);
        if (ownership->suppressed[i] &&
            ownership->base[i] == base[i] && base[i] &&
            ptr_readable(base[i], POSEEDIT_TRACK_SIZE) &&
            *(void**)(base[i] + 0x24) == null_array &&
            ((defs[i].preserve_target &&
              *(void**)(base[i] + 0x04) == ownership->saved_obj[i]) ||
             (!defs[i].preserve_target &&
              *(void**)(base[i] + 0x04) == nil_weak))) {
            continue;
        }

        make_body_runtime_name(object_name, sizeof(object_name),
                               person, defs[i].node_suffix);
        target_obj[i] = resolve_find_obj(object_name, &target_raw[i]);
        if ((!target_obj[i] ||
             is_nil_engine_object(target_raw[i], target_obj[i])) &&
            captured_script_engine) {
            target_obj[i] = resolve_script_engine_obj(
                object_name, &target_raw[i]);
        }
        if (!target_obj[i] ||
            is_nil_engine_object(target_raw[i], target_obj[i])) {
            if (!ownership->wait_log_tick ||
                now - ownership->wait_log_tick >= 2000u) {
                ownership->wait_log_tick = now;
                log_line("%s poseeditor-track waiting person=\"%s\" track_id=%d label=\"%s\" object=\"%s\" reason=\"could not resolve live track target\"",
                         system_name, person, defs[i].track_id,
                         defs[i].label, object_name);
            }
            return 0;
        }
        slot_obj = (base[i] &&
                    ptr_readable(base[i], POSEEDIT_TRACK_SIZE)) ?
            *(void**)(base[i] + 0x04) : NULL;
        slot_data = (base[i] &&
                     ptr_readable(base[i], POSEEDIT_TRACK_SIZE)) ?
            *(void**)(base[i] + 0x24) : NULL;
        if (ownership->suppressed[i] &&
            ownership->base[i] == base[i] &&
            slot_data && slot_data != null_array &&
            ptr_readable((BYTE*)slot_data - sizeof(int), sizeof(int))) {
            if (defs[i].preserve_target &&
                (slot_obj == target_obj[i] || slot_obj == target_raw[i])) {
                track_target[i] = slot_obj;
            } else if (!defs[i].preserve_target && slot_obj == nil_weak) {
                void *saved_obj = ownership->saved_obj[i];
                track_target[i] =
                    (saved_obj == target_obj[i] ||
                     saved_obj == target_raw[i]) ?
                        saved_obj : target_obj[i];
            }
            if (track_target[i]) {
                track_data[i] = slot_data;
                needs_bind[i] = 1;
                continue;
            }
        }
        if (!base[i] || !ptr_readable(base[i], POSEEDIT_TRACK_SIZE) ||
            (!validate_poseeditor_track_slot(base[i], target_obj[i]) &&
             !validate_poseeditor_track_slot(base[i], target_raw[i]))) {
            BYTE *found = find_poseedit_track_slot_by_object(
                person_index, defs[i].track_id, target_obj[i], NULL, -1);
            if (!found && target_raw[i]) {
                found = find_poseedit_track_slot_by_object(
                    person_index, defs[i].track_id,
                    target_raw[i], NULL, -1);
            }
            if (found) base[i] = found;
        }
        if (!base[i] || !ptr_readable(base[i], POSEEDIT_TRACK_SIZE) ||
            (!validate_poseeditor_track_slot(base[i], target_obj[i]) &&
             !validate_poseeditor_track_slot(base[i], target_raw[i]))) {
            if (!ownership->wait_log_tick ||
                now - ownership->wait_log_tick >= 2000u) {
                ownership->wait_log_tick = now;
                log_line("%s poseeditor-track mismatch person=\"%s\" track_id=%d label=\"%s\" base=%p slot_obj=%p expected_obj=%p expected_raw=%p note=\"track was not modified\"",
                         system_name, person, defs[i].track_id,
                         defs[i].label, base[i],
                         (base[i] && ptr_readable(base[i] + 0x04,
                                                  sizeof(void*))) ?
                             *(void**)(base[i] + 0x04) : NULL,
                         target_obj[i], target_raw[i]);
            }
            return 0;
        }
        slot_obj = *(void**)(base[i] + 0x04);
        slot_data = *(void**)(base[i] + 0x24);
        if (!slot_data || slot_data == null_array ||
            !ptr_readable((BYTE*)slot_data - sizeof(int), sizeof(int))) {
            return 0;
        }
        track_target[i] = slot_obj;
        track_data[i] = slot_data;
        needs_bind[i] = 1;
    }

    {
        double current_frame = 0.0;
        int has_track_to_bind = 0;
        for (i = 0; i < track_count; i++) {
            if (needs_bind[i]) {
                has_track_to_bind = 1;
                break;
            }
        }
        if (has_track_to_bind && !poseedit_current_frame(&current_frame)) {
            return 0;
        }
        for (i = 0; i < track_count; i++) {
            if (needs_bind[i] &&
                !poseedit_track_apply_zero(base[i], track_target[i],
                                           current_frame)) {
                return 0;
            }
        }
    }

    for (i = 0; i < track_count; i++) {
        DWORD old;
        if (!needs_bind[i]) continue;
        if (!VirtualProtect(base[i], POSEEDIT_TRACK_SIZE,
                            PAGE_READWRITE, &old)) {
            return 0;
        }
        ownership->base[i] = base[i];
        ownership->saved_obj[i] = track_target[i];
        ownership->saved_track_data[i] = track_data[i];
        if (!defs[i].preserve_target) {
            *(void**)(base[i] + 0x04) = nil_weak;
        }
        *(void**)(base[i] + 0x24) = null_array;
        VirtualProtect(base[i], POSEEDIT_TRACK_SIZE, old, &old);
        ownership->suppressed[i] = 1;
    }
    ownership->wait_log_tick = 0;
    if (!ownership->logged) {
        ownership->logged = 1;
        log_line("%s poseeditor-tracks suppressed person=\"%s\" person_index=%d count=%d note=\"current values neutralized immediately, then live PoseTrack updates disconnected for this person; saved keyframes and tracks.ini remain untouched\"",
                 system_name, person, person_index + 1, track_count);
    }
    return 1;
}

static int body_chain_collider_person_tracks_current(int person_index,
                                                     const char *person)
{
    char joint_name[256];
    char testicles_name[256];
    void *joint_raw = NULL;
    void *joint_obj = NULL;
    void *testicles_raw = NULL;
    void *testicles_obj = NULL;
    BYTE *base;
    BYTE *found;

    if (person_index < 0 || person_index >= 4 || !person || !person[0]) {
        return 0;
    }

    /* Active PhysX chains temporarily blank the PoseEdit slot, so do not use
       that slot as a liveness check while we own it. Passive colliders still
       have normal PoseEdit tracks and can be validated this way. */
    if (body_chain_person_states[person_index].pose_track_suppressed) {
        return 1;
    }

    if (!captured_poseedit_editpose) {
        return 1;
    }

    make_body_runtime_name(joint_name, sizeof(joint_name), person, "penis_joint01");
    make_body_runtime_name(testicles_name, sizeof(testicles_name), person, "testicles_joint01");

    joint_obj = resolve_find_obj(joint_name, &joint_raw);
    if ((!joint_obj || is_nil_engine_object(joint_raw, joint_obj)) &&
        captured_script_engine) {
        joint_obj = resolve_script_engine_obj(joint_name, &joint_raw);
    }
    testicles_obj = resolve_find_obj(testicles_name, &testicles_raw);
    if ((!testicles_obj || is_nil_engine_object(testicles_raw, testicles_obj)) &&
        captured_script_engine) {
        testicles_obj = resolve_script_engine_obj(testicles_name, &testicles_raw);
    }

    if (!joint_obj || !testicles_obj ||
        is_nil_engine_object(joint_raw, joint_obj) ||
        is_nil_engine_object(testicles_raw, testicles_obj)) {
        return 0;
    }

    base = poseedit_track_slot(person_index, POSEEDIT_TRACK_PENIS_JOINT01);
    if (base && ptr_readable(base, POSEEDIT_TRACK_SIZE) &&
        (validate_poseeditor_track_slot(base, joint_obj) ||
         validate_poseeditor_track_slot(base, joint_raw))) {
        return 1;
    }

    found = find_poseedit_track_slot_by_object(person_index,
                                               POSEEDIT_TRACK_PENIS_JOINT01,
                                               joint_obj,
                                               testicles_obj,
                                               POSEEDIT_TRACK_TESTICLES_JOINT01);
    if (!found && joint_raw) {
        found = find_poseedit_track_slot_by_object(person_index,
                                                   POSEEDIT_TRACK_PENIS_JOINT01,
                                                   joint_raw,
                                                   testicles_raw,
                                                   POSEEDIT_TRACK_TESTICLES_JOINT01);
    }
    return found != NULL;
}

static int suppress_poseeditor_joint01_track_for_person(int person_index,
                                                        const char *person,
                                                        body_chain_person_state_t *state,
                                                        DWORD now)
{
    char joint_name[256];
    char testicles_name[256];
    void *joint_raw = NULL;
    void *joint_obj = NULL;
    void *testicles_raw = NULL;
    void *testicles_obj = NULL;
    void *nil_weak = engine_G_NilWeakObjTarget_ptr ? *engine_G_NilWeakObjTarget_ptr : NULL;
    void *null_array = engine_G_NullArray_ptr ? *engine_G_NullArray_ptr : NULL;
    BYTE *base;
    BYTE *testicles_base;
    DWORD old;
    int current_pose_rebind = 0;
    int current_pose_data_only = 0;
    void *current_track_obj = NULL;
    void *current_track_data = NULL;
    double current_frame = 0.0;

    if (!body_chain_physics_cfg.override_animation ||
        (!body_chain_physics_cfg.poseeditor_track_override &&
         !body_chain_physics_cfg.poseeditor_track_diagnostic) ||
        !person || !state) {
        return 1;
    }

    if (state->pose_track_suppressed &&
        state->pose_track_base &&
        ptr_readable(state->pose_track_base, 0x50)) {
        base = (BYTE*)state->pose_track_base;
        if (*(void**)(base + 0x04) == nil_weak &&
            *(void**)(base + 0x24) == null_array) {
            return 1;
        }
        current_pose_rebind = 1;
    }

    if (!current_pose_rebind && state->pose_track_scan_tick &&
        now - state->pose_track_scan_tick < 1000) {
        return 0;
    }
    state->pose_track_scan_tick = now;

    make_body_runtime_name(joint_name, sizeof(joint_name), person, "penis_joint01");
    make_body_runtime_name(testicles_name, sizeof(testicles_name), person, "testicles_joint01");
    joint_obj = resolve_find_obj(joint_name, &joint_raw);
    if ((!joint_obj || is_nil_engine_object(joint_raw, joint_obj)) && captured_script_engine) {
        joint_obj = resolve_script_engine_obj(joint_name, &joint_raw);
    }
    testicles_obj = resolve_find_obj(testicles_name, &testicles_raw);
    if ((!testicles_obj || is_nil_engine_object(testicles_raw, testicles_obj)) && captured_script_engine) {
        testicles_obj = resolve_script_engine_obj(testicles_name, &testicles_raw);
    }
    if (!joint_obj || !testicles_obj ||
        is_nil_engine_object(joint_raw, joint_obj) ||
        is_nil_engine_object(testicles_raw, testicles_obj)) {
        if (!state->pose_track_logged) {
            state->pose_track_logged = 1;
            log_line("body-chain-physics poseeditor-track waiting person=\"%s\" joint_name=\"%s\" joint_raw=%p joint_obj=%p testicles_name=\"%s\" testicles_raw=%p testicles_obj=%p reason=\"could not resolve live PoseEditor track objects yet\"",
                     person, joint_name, joint_raw, joint_obj,
                     testicles_name, testicles_raw, testicles_obj);
        }
        return 0;
    }

    base = poseedit_track_slot(person_index, POSEEDIT_TRACK_PENIS_JOINT01);
    testicles_base = poseedit_track_slot(person_index, POSEEDIT_TRACK_TESTICLES_JOINT01);
    if (!base || !ptr_readable(base, POSEEDIT_TRACK_SIZE)) {
        if (!state->pose_track_logged) {
            state->pose_track_logged = 1;
            log_line("body-chain-physics poseeditor-track waiting person=\"%s\" joint_obj=%p testicles_obj=%p editpose=%p poseedit=%p reason=\"PoseEdit table not captured yet; reload PoseEditor room if this stays null\"",
                     person, joint_obj, testicles_obj,
                     captured_poseedit_editpose, captured_poseedit_this);
        }
        return 0;
    }
    if (current_pose_rebind && base == state->pose_track_base &&
        *(void**)(base + 0x04) == nil_weak &&
        *(void**)(base + 0x24) != null_array) {
        current_track_data = *(void**)(base + 0x24);
        if (!current_track_data ||
            !ptr_readable((BYTE*)current_track_data - sizeof(int),
                          sizeof(int))) {
            return 0;
        }
        if (state->pose_track_saved_obj == joint_obj ||
            state->pose_track_saved_obj == joint_raw) {
            current_track_obj = state->pose_track_saved_obj;
        } else {
            current_track_obj = joint_obj;
        }
        current_pose_data_only = 1;
    }
    if (!current_pose_data_only &&
        !validate_poseeditor_track_slot(base, joint_obj)) {
        BYTE *found = find_poseedit_track_slot_by_object(person_index,
                                                         POSEEDIT_TRACK_PENIS_JOINT01,
                                                         joint_obj,
                                                         testicles_obj,
                                                         POSEEDIT_TRACK_TESTICLES_JOINT01);
        if (!found && joint_raw) {
            found = find_poseedit_track_slot_by_object(person_index,
                                                       POSEEDIT_TRACK_PENIS_JOINT01,
                                                       joint_raw,
                                                       testicles_raw,
                                                       POSEEDIT_TRACK_TESTICLES_JOINT01);
        }
        if (found) {
            base = found;
            testicles_base = poseedit_track_slot(person_index, POSEEDIT_TRACK_TESTICLES_JOINT01);
        }
    }
    if (!current_pose_data_only &&
        !validate_poseeditor_track_slot(base, joint_obj) &&
        !validate_poseeditor_track_slot(base, joint_raw)) {
        if (!state->pose_track_logged) {
            state->pose_track_logged = 1;
            log_line("body-chain-physics poseeditor-track mismatch person=\"%s\" person_index=%d track_id=%d base=%p slot_obj=%p expected_joint_obj=%p testicles_track=%p testicles_slot_obj=%p expected_testicles_obj=%p editpose=%p tracks_offset=0x%03x total_tracks=%d note=\"calculated slot and captured-EditPose scan did not find a matching live PoseTrack, so it will not be modified\"",
                     person, person_index + 1,
                     POSEEDIT_TRACK_PENIS_JOINT01,
                     base,
                     ptr_readable(base + 0x04, sizeof(void*)) ? *(void**)(base + 0x04) : NULL,
                     joint_obj,
                     testicles_base,
                     (testicles_base && ptr_readable(testicles_base + 0x04, sizeof(void*))) ? *(void**)(testicles_base + 0x04) : NULL,
                     testicles_obj,
                     captured_poseedit_editpose,
                     captured_poseedit_tracks_offset,
                     body_chain_physics_cfg.poseeditor_total_tracks);
        }
        return 0;
    }

    if (body_chain_physics_cfg.poseeditor_track_diagnostic && !state->pose_track_logged) {
        state->pose_track_logged = 1;
        log_line("body-chain-physics poseeditor-track exact-slot person=\"%s\" person_index=%d track_id=%d base=%p joint_name=\"%s\" joint_raw=%p joint_obj=%p testicles_track=%p testicles_name=\"%s\" testicles_raw=%p testicles_obj=%p override_requested=%d editpose=%p poseedit=%p note=\"tracks.ini untouched; per-person slot resolved through PoseEdit table\"",
                 person, person_index + 1,
                 POSEEDIT_TRACK_PENIS_JOINT01,
                 base,
                 joint_name, joint_raw, joint_obj,
                 testicles_base,
                 testicles_name, testicles_raw, testicles_obj,
                 body_chain_physics_cfg.poseeditor_track_override,
                 captured_poseedit_editpose,
                 captured_poseedit_this);
    }

    if (!body_chain_physics_cfg.poseeditor_track_override) return 1;

    if (current_pose_rebind) {
        if (!current_pose_data_only) {
            current_track_obj = *(void**)(base + 0x04);
            current_track_data = *(void**)(base + 0x24);
        }
        if (!current_track_obj || !current_track_data ||
            current_track_data == null_array ||
            !ptr_readable(current_track_obj, sizeof(DWORD)) ||
            !ptr_readable((BYTE*)current_track_data - sizeof(int),
                          sizeof(int)) ||
            !poseedit_current_frame(&current_frame) ||
            !poseedit_track_apply_zero(base, current_track_obj,
                                       current_frame)) {
            return 0;
        }
    }

    if (VirtualProtect(base, POSEEDIT_TRACK_SIZE, PAGE_READWRITE, &old)) {
        state->pose_track_base = base;
        state->pose_track_saved_obj = current_pose_rebind ?
            current_track_obj : *(void**)(base + 0x04);
        state->pose_track_saved_track_data = current_pose_rebind ?
            current_track_data : *(void**)(base + 0x24);
        *(void**)(base + 0x04) = nil_weak;
        *(void**)(base + 0x24) = null_array;
        VirtualProtect(base, 0x50, old, &old);
        state->pose_track_suppressed = 1;
        if (current_pose_rebind) {
            log_line("body-chain-physics poseeditor-track current-pose rebound person=\"%s\" person_index=%d track_id=%d base=%p saved_obj=%p saved_track_data=%p note=\"TK17 populated a new pose while PhysX owned the joint; replacing stale saved ownership before suppressing the current track\"",
                     person, person_index + 1,
                     POSEEDIT_TRACK_PENIS_JOINT01,
                     base, state->pose_track_saved_obj,
                     state->pose_track_saved_track_data);
        } else {
            log_line("body-chain-physics poseeditor-track suppressed person=\"%s\" person_index=%d track_id=%d base=%p joint_obj=%p testicles_track=%p testicles_obj=%p saved_obj=%p saved_track_data=%p note=\"per-person live PoseTrack override; tracks.ini untouched\"",
                     person, person_index + 1,
                     POSEEDIT_TRACK_PENIS_JOINT01,
                     base, joint_obj, testicles_base, testicles_obj,
                     state->pose_track_saved_obj,
                     state->pose_track_saved_track_data);
        }
        return 1;
    }

    return 0;
}

static int suppress_poseeditor_joint02_03_tracks_for_person(
    int person_index,
    const char *person,
    body_chain_person_state_t *state)
{
    static const int track_ids[POSEEDIT_EXTRA_PHYSICS_TRACK_COUNT] = {
        POSEEDIT_TRACK_PENIS_JOINT02,
        POSEEDIT_TRACK_PENIS_JOINT03
    };
    static const char *joint_suffixes[POSEEDIT_EXTRA_PHYSICS_TRACK_COUNT] = {
        "penis_joint02",
        "penis_joint03"
    };
    char joint_name[POSEEDIT_EXTRA_PHYSICS_TRACK_COUNT][256];
    void *joint_raw[POSEEDIT_EXTRA_PHYSICS_TRACK_COUNT] = { NULL, NULL };
    void *joint_obj[POSEEDIT_EXTRA_PHYSICS_TRACK_COUNT] = { NULL, NULL };
    void *track_target[POSEEDIT_EXTRA_PHYSICS_TRACK_COUNT] = { NULL, NULL };
    void *track_data[POSEEDIT_EXTRA_PHYSICS_TRACK_COUNT] = { NULL, NULL };
    BYTE *base[POSEEDIT_EXTRA_PHYSICS_TRACK_COUNT] = { NULL, NULL };
    void *nil_weak = engine_G_NilWeakObjTarget_ptr ?
        *engine_G_NilWeakObjTarget_ptr : NULL;
    void *null_array = engine_G_NullArray_ptr ?
        *engine_G_NullArray_ptr : NULL;
    int track_needs_bind[POSEEDIT_EXTRA_PHYSICS_TRACK_COUNT] = { 0, 0 };
    int current_pose_rebound = 0;
    double current_frame = 0.0;
    int i;

    if (!body_chain_physics_cfg.override_animation ||
        !body_chain_physics_cfg.poseeditor_track_override ||
        !person || !state) {
        return 1;
    }

    for (i = 0; i < POSEEDIT_EXTRA_PHYSICS_TRACK_COUNT; i++) {
        void *slot_obj;
        void *slot_data;
        base[i] = poseedit_track_slot(person_index, track_ids[i]);
        if (!base[i] || !ptr_readable(base[i], POSEEDIT_TRACK_SIZE)) {
            return 0;
        }
        if (state->pose_track_extra_suppressed[i] &&
            state->pose_track_extra_base[i] == base[i] &&
            *(void**)(base[i] + 0x04) == nil_weak &&
            *(void**)(base[i] + 0x24) == null_array) {
            continue;
        }
        track_needs_bind[i] = 1;
        make_body_runtime_name(joint_name[i], sizeof(joint_name[i]),
                               person, joint_suffixes[i]);
        joint_obj[i] = resolve_find_obj(joint_name[i], &joint_raw[i]);
        if ((!joint_obj[i] ||
             is_nil_engine_object(joint_raw[i], joint_obj[i])) &&
            captured_script_engine) {
            joint_obj[i] =
                resolve_script_engine_obj(joint_name[i], &joint_raw[i]);
        }
        if (!joint_obj[i] ||
            is_nil_engine_object(joint_raw[i], joint_obj[i])) {
            if (!state->pose_track_extra_logged) {
                state->pose_track_extra_logged = 1;
                log_line("body-chain-physics poseeditor-extra-track waiting person=\"%s\" track_id=%d joint_name=\"%s\" joint_raw=%p joint_obj=%p reason=\"could not resolve live PoseEditor joint object\"",
                         person, track_ids[i], joint_name[i],
                         joint_raw[i], joint_obj[i]);
            }
            return 0;
        }
        slot_obj = *(void**)(base[i] + 0x04);
        slot_data = *(void**)(base[i] + 0x24);
        if (!slot_data || slot_data == null_array ||
            !ptr_readable((BYTE*)slot_data - sizeof(int), sizeof(int))) {
            return 0;
        }
        if (state->pose_track_extra_suppressed[i] &&
            state->pose_track_extra_base[i] == base[i] &&
            slot_obj == nil_weak) {
            void *saved_obj = state->pose_track_extra_saved_obj[i];
            if (saved_obj == joint_obj[i] || saved_obj == joint_raw[i]) {
                track_target[i] = saved_obj;
            } else {
                track_target[i] = joint_obj[i];
            }
        } else if (validate_poseeditor_track_slot(base[i], joint_obj[i]) ||
                   validate_poseeditor_track_slot(base[i], joint_raw[i])) {
            track_target[i] = slot_obj;
        } else {
            if (!state->pose_track_extra_logged) {
                state->pose_track_extra_logged = 1;
                log_line("body-chain-physics poseeditor-extra-track mismatch person=\"%s\" person_index=%d track_id=%d base=%p slot_obj=%p expected_obj=%p expected_raw=%p note=\"joint02/03 tracks were not modified\"",
                         person, person_index + 1, track_ids[i], base[i],
                         (base[i] &&
                          ptr_readable(base[i] + 0x04, sizeof(void*))) ?
                            *(void**)(base[i] + 0x04) : NULL,
                         joint_obj[i], joint_raw[i]);
            }
            return 0;
        }
        if (!track_target[i] ||
            !ptr_readable(track_target[i], sizeof(DWORD))) {
            return 0;
        }
        track_data[i] = slot_data;
    }

    if (!track_needs_bind[0] && !track_needs_bind[1]) {
        return 1;
    }
    if (!poseedit_current_frame(&current_frame)) {
        return 0;
    }
    for (i = 0; i < POSEEDIT_EXTRA_PHYSICS_TRACK_COUNT; i++) {
        DWORD old;
        int was_suppressed;
        if (!track_needs_bind[i]) continue;
        was_suppressed = state->pose_track_extra_suppressed[i];
        if (!poseedit_track_apply_zero(base[i], track_target[i],
                                       current_frame)) {
            return 0;
        }
        if (!VirtualProtect(base[i], POSEEDIT_TRACK_SIZE,
                            PAGE_READWRITE, &old)) {
            return 0;
        }
        state->pose_track_extra_base[i] = base[i];
        state->pose_track_extra_saved_obj[i] = track_target[i];
        state->pose_track_extra_saved_track_data[i] = track_data[i];
        *(void**)(base[i] + 0x04) = nil_weak;
        *(void**)(base[i] + 0x24) = null_array;
        VirtualProtect(base[i], POSEEDIT_TRACK_SIZE, old, &old);
        state->pose_track_extra_suppressed[i] = 1;
        if (was_suppressed) current_pose_rebound = 1;
    }

    if (current_pose_rebound) {
        log_line("body-chain-physics poseeditor-extra-tracks current-pose rebound person=\"%s\" person_index=%d tracks=(%d,%d) bases=(%p,%p) saved_track_data=(%p,%p) note=\"new pose joint02/03 tracks were neutralized through TK17 and replaced the stale saved pose before suppression\"",
                 person, person_index + 1,
                 POSEEDIT_TRACK_PENIS_JOINT02,
                 POSEEDIT_TRACK_PENIS_JOINT03,
                 state->pose_track_extra_base[0],
                 state->pose_track_extra_base[1],
                 state->pose_track_extra_saved_track_data[0],
                 state->pose_track_extra_saved_track_data[1]);
    } else {
        log_line("body-chain-physics poseeditor-extra-tracks suppressed person=\"%s\" person_index=%d tracks=(%d,%d) bases=(%p,%p) note=\"penis_joint02/03 PoseEditor tracks are owned by PhysX for this person; tracks.ini untouched\"",
                 person, person_index + 1,
                 POSEEDIT_TRACK_PENIS_JOINT02,
                 POSEEDIT_TRACK_PENIS_JOINT03,
                 state->pose_track_extra_base[0],
                 state->pose_track_extra_base[1]);
    }
    return 1;
}

static int suppress_poseeditor_testicle_tip_track_for_person(
    int person_index,
    const char *person,
    body_chain_person_state_t *state)
{
    const int extra_index = 0;
    char joint_name[256];
    void *joint_raw = NULL;
    void *joint_obj = NULL;
    BYTE *base;
    void *nil_weak = engine_G_NilWeakObjTarget_ptr ?
        *engine_G_NilWeakObjTarget_ptr : NULL;
    void *null_array = engine_G_NullArray_ptr ?
        *engine_G_NullArray_ptr : NULL;
    DWORD old;

    if (!testicle_physics_cfg.override_animation ||
        !testicle_physics_cfg.poseeditor_track_override ||
        !person || !state) {
        return 1;
    }

    make_body_runtime_name(joint_name, sizeof(joint_name),
                           person, "testicles_joint02");
    joint_obj = resolve_find_obj(joint_name, &joint_raw);
    if ((!joint_obj || is_nil_engine_object(joint_raw, joint_obj)) &&
        captured_script_engine) {
        joint_obj = resolve_script_engine_obj(joint_name, &joint_raw);
    }
    if (!joint_obj || is_nil_engine_object(joint_raw, joint_obj)) {
        if (!state->pose_track_extra_logged) {
            state->pose_track_extra_logged = 1;
            log_line("testicle-physics poseeditor-tip-track waiting person=\"%s\" track_id=%d joint_name=\"%s\" joint_raw=%p joint_obj=%p reason=\"could not resolve live PoseEditor tip joint object\"",
                     person, POSEEDIT_TRACK_TESTICLES_JOINT02,
                     joint_name, joint_raw, joint_obj);
        }
        return 0;
    }

    base = poseedit_track_slot(person_index, POSEEDIT_TRACK_TESTICLES_JOINT02);
    if (state->pose_track_extra_suppressed[extra_index] &&
        state->pose_track_extra_base[extra_index]) {
        BYTE *saved_base =
            (BYTE*)state->pose_track_extra_base[extra_index];
        if (saved_base == base &&
            ptr_readable(saved_base, POSEEDIT_TRACK_SIZE) &&
            *(void**)(saved_base + 0x04) == nil_weak &&
            *(void**)(saved_base + 0x24) == null_array) {
            return 1;
        }
        state->pose_track_extra_suppressed[extra_index] = 0;
        state->pose_track_extra_base[extra_index] = NULL;
        state->pose_track_extra_saved_obj[extra_index] = NULL;
        state->pose_track_extra_saved_track_data[extra_index] = NULL;
    }
    if (!base ||
        !ptr_readable(base, POSEEDIT_TRACK_SIZE) ||
        (!validate_poseeditor_track_slot(base, joint_obj) &&
         !validate_poseeditor_track_slot(base, joint_raw))) {
        BYTE *found = find_poseedit_track_slot_by_object(
            person_index,
            POSEEDIT_TRACK_TESTICLES_JOINT02,
            joint_obj,
            NULL,
            -1);
        if (!found && joint_raw) {
            found = find_poseedit_track_slot_by_object(
                person_index,
                POSEEDIT_TRACK_TESTICLES_JOINT02,
                joint_raw,
                NULL,
                -1);
        }
        if (found) base = found;
    }
    if (!base ||
        !ptr_readable(base, POSEEDIT_TRACK_SIZE) ||
        (!validate_poseeditor_track_slot(base, joint_obj) &&
         !validate_poseeditor_track_slot(base, joint_raw))) {
        if (!state->pose_track_extra_logged) {
            state->pose_track_extra_logged = 1;
            log_line("testicle-physics poseeditor-tip-track mismatch person=\"%s\" person_index=%d track_id=%d base=%p slot_obj=%p expected_obj=%p expected_raw=%p note=\"testicles_joint02 tip track was not modified\"",
                     person, person_index + 1,
                     POSEEDIT_TRACK_TESTICLES_JOINT02,
                     base,
                     (base &&
                      ptr_readable(base + 0x04, sizeof(void*))) ?
                        *(void**)(base + 0x04) : NULL,
                     joint_obj, joint_raw);
        }
        return 0;
    }
    if (VirtualProtect(base, POSEEDIT_TRACK_SIZE, PAGE_READWRITE, &old)) {
        state->pose_track_extra_base[extra_index] = base;
        state->pose_track_extra_saved_obj[extra_index] =
            *(void**)(base + 0x04);
        state->pose_track_extra_saved_track_data[extra_index] =
            *(void**)(base + 0x24);
        *(void**)(base + 0x04) = nil_weak;
        *(void**)(base + 0x24) = null_array;
        VirtualProtect(base, POSEEDIT_TRACK_SIZE, old, &old);
        state->pose_track_extra_suppressed[extra_index] = 1;
        log_line("testicle-physics poseeditor-tip-track suppressed person=\"%s\" person_index=%d track_id=%d base=%p joint_obj=%p saved_obj=%p saved_track_data=%p note=\"testicles_joint02 tip PoseEditor track is owned by PhysX for this person; tracks.ini untouched\"",
                 person, person_index + 1,
                 POSEEDIT_TRACK_TESTICLES_JOINT02,
                 base, joint_obj,
                 state->pose_track_extra_saved_obj[extra_index],
                 state->pose_track_extra_saved_track_data[extra_index]);
        return 1;
    }
    return 0;
}

static int suppress_poseeditor_testicle_track_for_person(
    int person_index,
    const char *person,
    body_chain_person_state_t *state)
{
    char joint_name[256];
    void *joint_raw = NULL;
    void *joint_obj = NULL;
    BYTE *base;
    void *nil_weak = engine_G_NilWeakObjTarget_ptr ?
        *engine_G_NilWeakObjTarget_ptr : NULL;
    void *null_array = engine_G_NullArray_ptr ?
        *engine_G_NullArray_ptr : NULL;
    DWORD old;

    if (!testicle_physics_cfg.override_animation ||
        !testicle_physics_cfg.poseeditor_track_override ||
        !person || !state) {
        return 1;
    }
    make_body_runtime_name(joint_name, sizeof(joint_name),
                           person, "testicles_joint01");
    joint_obj = resolve_find_obj(joint_name, &joint_raw);
    if ((!joint_obj || is_nil_engine_object(joint_raw, joint_obj)) &&
        captured_script_engine) {
        joint_obj = resolve_script_engine_obj(joint_name, &joint_raw);
    }
    if (!joint_obj || is_nil_engine_object(joint_raw, joint_obj)) {
        if (!state->pose_track_logged) {
            state->pose_track_logged = 1;
            log_line("testicle-physics poseeditor-track waiting person=\"%s\" track_id=%d joint_name=\"%s\" joint_raw=%p joint_obj=%p reason=\"could not resolve live PoseEditor joint object\"",
                     person, POSEEDIT_TRACK_TESTICLES_JOINT01,
                     joint_name, joint_raw, joint_obj);
        }
        return 0;
    }

    base = poseedit_track_slot(person_index, POSEEDIT_TRACK_TESTICLES_JOINT01);
    if (state->pose_track_suppressed && state->pose_track_base) {
        BYTE *saved_base = (BYTE*)state->pose_track_base;
        if (saved_base == base &&
            ptr_readable(saved_base, POSEEDIT_TRACK_SIZE) &&
            *(void**)(saved_base + 0x04) == nil_weak &&
            *(void**)(saved_base + 0x24) == null_array) {
            return suppress_poseeditor_testicle_tip_track_for_person(
                person_index, person, state);
        }
        state->pose_track_suppressed = 0;
        state->pose_track_base = NULL;
        state->pose_track_saved_obj = NULL;
        state->pose_track_saved_track_data = NULL;
    }
    if (!base ||
        !ptr_readable(base, POSEEDIT_TRACK_SIZE) ||
        (!validate_poseeditor_track_slot(base, joint_obj) &&
         !validate_poseeditor_track_slot(base, joint_raw))) {
        BYTE *found = find_poseedit_track_slot_by_object(
            person_index,
            POSEEDIT_TRACK_TESTICLES_JOINT01,
            joint_obj,
            NULL,
            -1);
        if (!found && joint_raw) {
            found = find_poseedit_track_slot_by_object(
                person_index,
                POSEEDIT_TRACK_TESTICLES_JOINT01,
                joint_raw,
                NULL,
                -1);
        }
        if (found) base = found;
    }
    if (!base ||
        !ptr_readable(base, POSEEDIT_TRACK_SIZE) ||
        (!validate_poseeditor_track_slot(base, joint_obj) &&
         !validate_poseeditor_track_slot(base, joint_raw))) {
        if (!state->pose_track_logged) {
            state->pose_track_logged = 1;
            log_line("testicle-physics poseeditor-track mismatch person=\"%s\" person_index=%d track_id=%d base=%p slot_obj=%p expected_obj=%p expected_raw=%p note=\"testicles_joint01 track was not modified\"",
                     person, person_index + 1,
                     POSEEDIT_TRACK_TESTICLES_JOINT01,
                     base,
                     (base &&
                      ptr_readable(base + 0x04, sizeof(void*))) ?
                        *(void**)(base + 0x04) : NULL,
                     joint_obj, joint_raw);
        }
        return 0;
    }
    if (VirtualProtect(base, POSEEDIT_TRACK_SIZE, PAGE_READWRITE, &old)) {
        state->pose_track_base = base;
        state->pose_track_saved_obj = *(void**)(base + 0x04);
        state->pose_track_saved_track_data = *(void**)(base + 0x24);
        *(void**)(base + 0x04) = nil_weak;
        *(void**)(base + 0x24) = null_array;
        VirtualProtect(base, POSEEDIT_TRACK_SIZE, old, &old);
        state->pose_track_suppressed = 1;
        log_line("testicle-physics poseeditor-track suppressed person=\"%s\" person_index=%d track_id=%d base=%p joint_obj=%p saved_obj=%p saved_track_data=%p note=\"testicles_joint01 PoseEditor track is owned by PhysX for this person; tracks.ini untouched\"",
                 person, person_index + 1,
                 POSEEDIT_TRACK_TESTICLES_JOINT01,
                 base, joint_obj,
                 state->pose_track_saved_obj,
                 state->pose_track_saved_track_data);
        return suppress_poseeditor_testicle_tip_track_for_person(
            person_index, person, state);
    }
    return 0;
}

static unsigned int reset_body_chain_person_state(
    body_chain_person_state_t *state)
{
    int i;
    unsigned int restored_pose_track_mask;
    if (!state) return 0;
    restored_pose_track_mask = restore_poseeditor_joint01_track(state);
    state->last_tick = 0;
    state->resolve_retry_tick = 0;
    state->initialized = 0;
    state->active_logged = 0;
    state->last_log_tick = 0;
    state->cache_verify_tick = 0;
    state->health_log_tick = 0;
    state->gravity_probe_log_tick = 0;
    state->gravity_probe_stable_tick = 0;
    state->gravity_probe_candidate_tick = 0;
    state->gravity_probe_captured = 0;
    state->gravity_probe_promoted = 0;
    state->gravity_probe_sampled = 0;
    state->gravity_probe_candidate_camera_version = 0;
    state->camera_seen_version = 0;
    state->gravity_response_trace_tick = 0;
    state->root_drive_camera_seen_version = 0;
    state->root_drive_camera_quarantine_tick = 0;
    state->root_drive_camera_last_untrusted_tick = 0;
    state->root_drive_camera_ramp_log_tick = 0;
    state->root_drive_untrusted_log_tick = 0;
    state->late_ownership_log_tick = 0;
    state->ownership_candidate_tick = 0;
    state->ownership_candidate_samples = 0;
    state->ownership_candidate_root_raw = NULL;
    for (i = 0; i < 3; i++) {
        state->ownership_candidate_joint_raw[i] = NULL;
    }
    state->root_drive_last_step_valid = 0;
    state->root_drive_last_h_step = 0.0f;
    state->root_drive_last_v_step = 0.0f;
    state->root_drive_last_d_step = 0.0f;
    memset(&state->root_drive_filter, 0, sizeof(state->root_drive_filter));
    state->dynamics_valid=state->dynamics_gravity_valid=0;
    state->camera_relative_trs_raw = NULL;
    state->camera_relative_live_prev_valid = 0;
    state->camera_relative_live_current_valid = 0;
    state->camera_relative_calibration_samples = 0;
    state->camera_relative_h_peak = 0.0f;
    state->camera_relative_v_peak = 0.0f;
    state->camera_relative_d_peak = 0.0f;
    state->camera_relative_live_log_tick = 0;
    for (i = 0; i < 9; i++) {
        state->camera_relative_live_prev[i] = 0.0f;
        state->camera_relative_live_current[i] = 0.0f;
        state->camera_relative_h_coeff[i] = 0.0f;
        state->camera_relative_v_coeff[i] = 0.0f;
        state->camera_relative_d_coeff[i] = 0.0f;
    }
    state->root_raw = NULL;
    state->gravity_probe_reactivation_ready = 0;
    state->gravity_probe_reactivation_tick = 0;
    state->gravity_probe_reactivation_root_raw = NULL;
    state->init_tick = 0;
    state->gravity_probe_prev_root[0] = 0.0f;
    state->gravity_probe_prev_root[1] = 0.0f;
    state->gravity_probe_prev_root[2] = 0.0f;
    state->gravity_probe_last_root[0] = 0.0f;
    state->gravity_probe_last_root[1] = 0.0f;
    state->gravity_probe_last_root[2] = 0.0f;
    state->root_prev_world[0] = 0.0f;
    state->root_prev_world[1] = 0.0f;
    state->root_prev_world[2] = 0.0f;
    state->root_prev_world_valid = 0;
    state->root_translation_initialized = 0;
    state->root_translation_raw = NULL;
    state->root_translation_parent_rest_valid = 0;
    state->root_translation_parent_rest_root_raw = NULL;
    state->root_translation_parent_rest_trs_raw = NULL;
    state->root_translation_camera_hold_active = 0;
    for (i = 0; i < 9; i++) {
        state->root_translation_parent_rest[i] = 0.0f;
    }
    reset_body_chain_gravity_state(state);
    state->anim_rest_valid = 0;
    for (i = 0; i < 3; i++) {
        int j, axis;
        state->joint_raw[i] = NULL;
        state->anim_joint_raw[i] = NULL;
        for (axis = 0; axis < 3; axis++) {
            state->output_handoff_rest[i][axis] = 0.0f;
            state->pose_compensation[i][axis] = 0.0f;
        }
        for (j = 0; j < BODY_CHAIN_ANIM_OVERRIDE_MAX_OFFSETS; j++) {
            for (axis = 0; axis < 3; axis++) {
                state->anim_rest[i][j][axis] = 0.0f;
            }
        }
        for (axis = 0; axis < 3; axis++) {
            state->angle[i][axis] = 0.0f;
            state->velocity[i][axis] = 0.0f;
        }
        state->collision_contact_direction[i][0] = 0.0f;
        state->collision_contact_direction[i][1] = 0.0f;
    }
    state->output_handoff_rest_valid = 0;
    state->collision_step_valid = 0;
    state->collision_step_tick = 0;
    state->collision_prev_max_penetration = 0.0f;
    state->collision_rest_penetration = 0.0f;
    state->collision_rest_ticks = 0;
    state->collision_rest_valid = 0;
    state->collision_rest_grace_ticks = 0;
    state->collision_impact_ticks = 0;
    state->collision_manifold_contacts = 0;
    state->collision_room_contacts = 0;
    state->collision_multi_support_grace_ticks = 0;
    state->collision_prev_chain_points_ready = 0;
    memset(state->collision_prev_chain_points, 0,
           sizeof(state->collision_prev_chain_points));
    memset(state->room_collision_track_valid, 0,
           sizeof(state->room_collision_track_valid));
    memset(state->room_collision_track_generation, 0,
           sizeof(state->room_collision_track_generation));
    memset(state->room_collision_track_tick, 0,
           sizeof(state->room_collision_track_tick));
    memset(state->room_collision_track_world, 0,
           sizeof(state->room_collision_track_world));
    memset(state->collision_prev_collider_points, 0,
           sizeof(state->collision_prev_collider_points));
    memset(state->collision_prev_collider_valid, 0,
           sizeof(state->collision_prev_collider_valid));
    memset(state->collision_prev_collider_ready, 0,
           sizeof(state->collision_prev_collider_ready));
    for (i = 0; i < BODY_CHAIN_ANIM_OVERRIDE_MAX_OFFSETS; i++) {
        int axis;
        for (axis = 0; axis < 3; axis++) {
            state->joint01_pose_rest[i][axis] = 0.0f;
        }
    }
    state->joint01_pose_rest_valid = 0;
    state->joint01_transform_lock_logged = 0;
    state->pose_compensation_valid = 0;
    state->pose_compensation_logged = 0;
    state->pose_track_base = NULL;
    state->pose_track_saved_obj = NULL;
    state->pose_track_saved_track_data = NULL;
    state->pose_track_suppressed = 0;
    state->pose_track_logged = 0;
    state->pose_track_extra_logged = 0;
    for (i = 0; i < POSEEDIT_EXTRA_PHYSICS_TRACK_COUNT; i++) {
        state->pose_track_extra_base[i] = NULL;
        state->pose_track_extra_saved_obj[i] = NULL;
        state->pose_track_extra_saved_track_data[i] = NULL;
        state->pose_track_extra_suppressed[i] = 0;
    }
    state->pose_track_scan_tick = 0;
    return restored_pose_track_mask;
}

static unsigned int reset_body_chain_person_state_for_reactivation(
    body_chain_person_state_t *state)
{
    float parent_rest[9];
    void *reference_root_raw;
    void *reference_trs_raw;
    int preserve_reference;
    unsigned int restored_pose_track_mask;

    if (!state) return 0;
    preserve_reference =
        state->root_translation_parent_rest_valid &&
        state->root_translation_parent_rest_root_raw &&
        state->root_translation_parent_rest_trs_raw;
    reference_root_raw = state->root_translation_parent_rest_root_raw;
    reference_trs_raw = state->root_translation_parent_rest_trs_raw;
    if (preserve_reference) {
        memcpy(parent_rest, state->root_translation_parent_rest,
               sizeof(parent_rest));
    }

    restored_pose_track_mask = reset_body_chain_person_state(state);

    /* An intentional off/on toggle must not redefine a rotated body as the
       authored zero orientation. Keep only the axis reference; velocities,
       contacts, outputs, ownership, and all other runtime state still reset.
       Pointer matching during reinitialization rejects this reference if the
       room, skeleton, root, or TRS_group has changed. */
    if (preserve_reference) {
        memcpy(state->root_translation_parent_rest, parent_rest,
               sizeof(parent_rest));
        state->root_translation_parent_rest_valid = 1;
        state->root_translation_parent_rest_root_raw = reference_root_raw;
        state->root_translation_parent_rest_trs_raw = reference_trs_raw;
    }
    return restored_pose_track_mask;
}

static void schedule_poseeditor_track_handoff_refresh(
    int person_index,
    unsigned int restored_ownership_mask,
    DWORD now)
{
    if (person_index < 0 || person_index >= 4 || !restored_ownership_mask ||
        !captured_poseedit_this || !captured_poseedit_editpose) {
        return;
    }
    poseeditor_track_handoff_pending_mask |= 1u << person_index;
    poseeditor_track_handoff_pending_tick = now;
    poseeditor_track_handoff_poseedit = captured_poseedit_this;
    poseeditor_track_handoff_editpose = captured_poseedit_editpose;
    log_line("body-physics poseeditor handoff scheduled person=\"Person%02d\" restored_ownership=0x%x delay=next-frame note=\"normal PoseEditor ownership is reconnected; a native track evaluation will apply the current keyframe\"",
             person_index + 1, restored_ownership_mask);
}

static void run_poseeditor_track_handoff_refresh(DWORD now)
{
    poseedit_update_objects_from_tracks_t update_objects;
    unsigned int person_mask = poseeditor_track_handoff_pending_mask;
    void *poseedit = poseeditor_track_handoff_poseedit;
    void *editpose = poseeditor_track_handoff_editpose;

    if (!person_mask || now - poseeditor_track_handoff_pending_tick < 1u) {
        return;
    }

    poseeditor_track_handoff_pending_mask = 0;
    poseeditor_track_handoff_pending_tick = 0;
    poseeditor_track_handoff_poseedit = NULL;
    poseeditor_track_handoff_editpose = NULL;

    if (poseedit != captured_poseedit_this ||
        editpose != captured_poseedit_editpose ||
        !ptr_readable((BYTE*)poseedit + POSEEDIT_EDITPOSE_OFFSET,
                      sizeof(void*)) ||
        *(void**)((BYTE*)poseedit + POSEEDIT_EDITPOSE_OFFSET) != editpose ||
        !ptr_readable((BYTE*)editpose + captured_poseedit_tracks_offset,
                      POSEEDIT_TRACK_SIZE)) {
        log_line("body-chain-physics poseeditor handoff skipped persons=0x%x reason=\"PoseEditor instance changed before deferred refresh\" note=\"the replacement PoseEditor instance evaluates its own tracks during initialization\"",
                 person_mask);
        return;
    }

    update_objects = (poseedit_update_objects_from_tracks_t)
        POSEEDIT_UPDATE_OBJECTS_FROM_TRACKS_ADDR;
    if (!ptr_executable((const void*)update_objects)) {
        log_line("body-chain-physics poseeditor handoff skipped persons=0x%x reason=\"native UpdateObjectsFromTracks entry is not executable\"",
                 person_mask);
        return;
    }

    update_objects(poseedit);
    log_line("body-chain-physics poseeditor handoff completed persons=0x%x poseedit=%p editpose=%p note=\"current pose tracks were evaluated once after PhysX ownership release\"",
             person_mask, poseedit, editpose);
}

static void reset_body_chain_collider_state(body_chain_collider_person_state_t *state)
{
    int i;
    if (!state) return;
    memset(state, 0, sizeof(*state));
    for (i = 0; i < BODY_COLLIDER_NODE_COUNT; i++) {
        state->position_offset[i] = -3;
    }
}

static void reset_body_chain_collider_states(void)
{
    int i;
    for (i = 0; i < 4; i++) {
        reset_body_chain_collider_state(&body_chain_collider_states[i]);
    }
}

static void reset_body_chain_collider_state_preserve_scene_liveness(
    body_chain_collider_person_state_t *state)
{
    LONG camera_version;
    int static_camera_samples;
    float root_view[3];
    int quarantined;
    void *quarantined_root_raw;
    int engine_invisible;
    DWORD last_log_tick;
    if (!state) return;
    camera_version = state->scene_liveness_camera_version;
    static_camera_samples = state->scene_liveness_static_camera_samples;
    memcpy(root_view, state->scene_liveness_root_view, sizeof(root_view));
    quarantined = state->scene_liveness_quarantined;
    quarantined_root_raw = state->scene_liveness_quarantined_root_raw;
    engine_invisible = state->scene_liveness_engine_invisible;
    last_log_tick = state->last_scene_liveness_log_tick;
    reset_body_chain_collider_state(state);
    state->scene_liveness_camera_version = camera_version;
    state->scene_liveness_static_camera_samples = static_camera_samples;
    memcpy(state->scene_liveness_root_view, root_view, sizeof(root_view));
    state->scene_liveness_quarantined = quarantined;
    state->scene_liveness_quarantined_root_raw = quarantined_root_raw;
    state->scene_liveness_engine_invisible = engine_invisible;
    state->last_scene_liveness_log_tick = last_log_tick;
}

static void clear_body_chain_prev_collider_for_person(int collider_person_index)
{
    int i;
    if (collider_person_index < 0 || collider_person_index >= 4) {
        return;
    }
    for (i = 0; i < 4; i++) {
        body_chain_person_states[i]
            .collision_prev_collider_ready[collider_person_index] = 0;
        memset(body_chain_person_states[i]
                   .collision_prev_collider_valid[collider_person_index],
               0,
               sizeof(body_chain_person_states[i]
                          .collision_prev_collider_valid[collider_person_index]));
        memset(body_chain_person_states[i]
                   .collision_prev_collider_points[collider_person_index],
               0,
               sizeof(body_chain_person_states[i]
                          .collision_prev_collider_points[collider_person_index]));
        runtime_body_chain_person_states[i]
            .collision_prev_collider_ready[collider_person_index] = 0;
        memset(runtime_body_chain_person_states[i]
                   .collision_prev_collider_valid[collider_person_index],
               0,
               sizeof(runtime_body_chain_person_states[i]
                          .collision_prev_collider_valid[collider_person_index]));
        memset(runtime_body_chain_person_states[i]
                   .collision_prev_collider_points[collider_person_index],
               0,
               sizeof(runtime_body_chain_person_states[i]
                          .collision_prev_collider_points[collider_person_index]));
    }
}

static void reset_body_chain_physics(void)
{
    int i;
    for (i = 0; i < 4; i++) {
        body_profile_set_active_person_config(i);
        reset_body_chain_person_state(&body_chain_person_states[i]);
        reset_body_chain_person_state(&testicle_physics_states[i]);
        reset_breasts_physics_person_state(
            i, &breasts_physics_states[i], 0);
        reset_butt_physics_person_state(
            i, &butt_physics_states[i], 0);
        reset_body_chain_collider_state(&body_chain_collider_states[i]);
    }
    body_profile_set_active_person_config(-1);
    /* InitTracks rebuilds solver/output ownership but can reuse the same
       root/TRS objects. Preserve the already-validated authored body axes;
       body_chain_camera_neutral_pivot_step still rejects this cache if those
       live object pointers actually change. */
    memset(body_chain_room_gravity_cache, 0, sizeof(body_chain_room_gravity_cache));
    memset(testicle_room_gravity_cache, 0, sizeof(testicle_room_gravity_cache));
}

static int resolve_body_chain_raws(const char *person, void **root_raw_out, void *joint_raw_out[3])
{
    static const char *joint_names[3] = {
        "Spenis_joint01", "Spenis_joint02", "Spenis_joint03"
    };
    char name[256];
    int i;
    if (!person || !person[0] || !root_raw_out || !joint_raw_out) return 0;
    make_body_runtime_name(name, sizeof(name), person, "root");
    *root_raw_out = resolve_axis_map_raw(name);
    if (!*root_raw_out) return 0;
    for (i = 0; i < 3; i++) {
        make_body_runtime_name(name, sizeof(name), person, joint_names[i]);
        joint_raw_out[i] = resolve_axis_map_raw(name);
        if (!joint_raw_out[i]) return 0;
    }
    return 1;
}

static unsigned int restore_body_chain_output_rest_for_person(
    int person_index,
    body_chain_person_state_t *state)
{
    const char *person = body_chain_person_name(person_index);
    void *live_root = NULL;
    void *live_joint[3] = { NULL, NULL, NULL };
    float before[3][3];
    const float (*handoff_rest)[3];
    float *out[3];
    int i, axis;

    if (person_index < 0 || person_index >= 4 || !state ||
        !state->initialized || !state->root_raw) {
        return 0;
    }
    if (!resolve_body_chain_raws(person, &live_root, live_joint) ||
        live_root != state->root_raw) {
        log_line("body-chain-physics output handoff skipped person=\"%s\" reason=\"live skeleton root changed before ownership release\"",
                 person);
        return 0;
    }
    handoff_rest = state->output_handoff_rest_valid
        ? state->output_handoff_rest
        : state->rest;

    /* PoseEditor drives penis_joint01/02/03, while PhysX writes the separate
       Spenis_joint01/02/03 output channels.  Reconnecting PoseTracks alone
       cannot clear the last values left in those output channels. */
    for (i = 0; i < 3; i++) {
        if (live_joint[i] != state->joint_raw[i] ||
            !ptr_readable((BYTE*)live_joint[i] +
                              body_chain_physics_cfg.output_offset,
                          sizeof(float) * 3)) {
            log_line("body-chain-physics output handoff skipped person=\"%s\" joint=%d reason=\"live Spenis ownership changed before release\"",
                     person, i + 1);
            return 0;
        }
        out[i] = (float*)((BYTE*)live_joint[i] +
                          body_chain_physics_cfg.output_offset);
        for (axis = 0; axis < 3; axis++) {
            if (!sane_probe_float(out[i][axis]) ||
                !sane_probe_float(handoff_rest[i][axis])) {
                log_line("body-chain-physics output handoff skipped person=\"%s\" joint=%d axis=%d reason=\"non-finite output or captured rest value\"",
                         person, i + 1, axis);
                return 0;
            }
            before[i][axis] = out[i][axis];
        }
    }

    for (i = 0; i < 3; i++) {
        for (axis = 0; axis < 3; axis++) {
            out[i][axis] = handoff_rest[i][axis];
        }
    }
    log_line("body-chain-physics output handoff restored person=\"%s\" output_offset=0x%03x exact_snapshot=%d before=(%.3f,%.3f,%.3f;%.3f,%.3f,%.3f;%.3f,%.3f,%.3f) restored=(%.3f,%.3f,%.3f;%.3f,%.3f,%.3f;%.3f,%.3f,%.3f) note=\"restored the pre-PhysX Spenis_joint01/02/03 values before PoseEditor track evaluation\"",
             person, body_chain_physics_cfg.output_offset,
             state->output_handoff_rest_valid,
             before[0][0], before[0][1], before[0][2],
             before[1][0], before[1][1], before[1][2],
             before[2][0], before[2][1], before[2][2],
             handoff_rest[0][0], handoff_rest[0][1], handoff_rest[0][2],
             handoff_rest[1][0], handoff_rest[1][1], handoff_rest[1][2],
             handoff_rest[2][0], handoff_rest[2][1], handoff_rest[2][2]);
    return 0x7u;
}

static int resolve_body_chain_anim_raws(const char *person, void *anim_joint_raw_out[3])
{
    static const char *joint_names[3] = {
        "penis_joint01", "penis_joint02", "penis_joint03"
    };
    char name[256];
    int i;
    if (!person || !person[0] || !anim_joint_raw_out) return 0;
    for (i = 0; i < 3; i++) {
        make_body_runtime_name(name, sizeof(name), person, joint_names[i]);
        anim_joint_raw_out[i] = resolve_axis_map_raw(name);
        if (!anim_joint_raw_out[i]) return 0;
    }
    return 1;
}

static int neutralize_body_chain_animation(const char *person, body_chain_person_state_t *state)
{
    void *anim_joint_raw[3] = { NULL, NULL, NULL };
    int i, j, axis;
    if (!body_chain_physics_cfg.override_animation || !person || !state) return 1;
    if (state->anim_joint_raw[0] &&
        state->anim_joint_raw[1] &&
        state->anim_joint_raw[2]) {
        for (i = 0; i < 3; i++) {
            anim_joint_raw[i] = state->anim_joint_raw[i];
        }
    } else if (!resolve_body_chain_anim_raws(person, anim_joint_raw)) {
        return 0;
    }
    for (i = 0; i < 3; i++) {
        state->anim_joint_raw[i] = anim_joint_raw[i];
        for (j = 0; j < body_chain_physics_cfg.animation_override_offset_count; j++) {
            int offset = body_chain_physics_cfg.animation_override_offsets[j];
            float *v;
            if (offset < 0 ||
                !ptr_readable((BYTE*)anim_joint_raw[i] + offset, sizeof(float) * 3)) {
                state->anim_joint_raw[0] = NULL;
                state->anim_joint_raw[1] = NULL;
                state->anim_joint_raw[2] = NULL;
                state->anim_rest_valid = 0;
                return 0;
            }
            v = (float*)((BYTE*)anim_joint_raw[i] + offset);
            if (!state->anim_rest_valid) {
                for (axis = 0; axis < 3; axis++) {
                    state->anim_rest[i][j][axis] = v[axis];
                }
            } else {
                for (axis = 0; axis < 3; axis++) {
                    v[axis] = state->anim_rest[i][j][axis];
                }
            }
        }
    }
    if (!state->anim_rest_valid) {
        state->anim_rest_valid = 1;
        log_line("body-chain-physics override-animation captured person=\"%s\" offsets=(0x%03x,0x%03x,0x%03x,0x%03x,0x%03x,0x%03x,0x%03x,0x%03x) count=%d note=\"restoring animated TJoint channels to baseline while PhysX owns Spenis joints\"",
                 person,
                 body_chain_physics_cfg.animation_override_offsets[0],
                 body_chain_physics_cfg.animation_override_offsets[1],
                 body_chain_physics_cfg.animation_override_offsets[2],
                 body_chain_physics_cfg.animation_override_offsets[3],
                 body_chain_physics_cfg.animation_override_offsets[4],
                 body_chain_physics_cfg.animation_override_offsets[5],
                 body_chain_physics_cfg.animation_override_offsets[6],
                 body_chain_physics_cfg.animation_override_offsets[7],
                 body_chain_physics_cfg.animation_override_offset_count);
    }
    return 1;
}

static int neutralize_body_chain_joint01_pose(const char *person,
                                              body_chain_person_state_t *state,
                                              void *joint01_raw)
{
    int j, axis;
    if (!body_chain_physics_cfg.override_animation ||
        !body_chain_physics_cfg.joint01_pose_override ||
        !person || !state || !joint01_raw) {
        return 1;
    }
    for (j = 0; j < body_chain_physics_cfg.joint01_pose_override_offset_count; j++) {
        int offset = body_chain_physics_cfg.joint01_pose_override_offsets[j];
        float *v;
        if (offset < 0 ||
            !ptr_readable((BYTE*)joint01_raw + offset, sizeof(float) * 3)) {
            state->joint01_pose_rest_valid = 0;
            return 0;
        }
        v = (float*)((BYTE*)joint01_raw + offset);
        for (axis = 0; axis < 3; axis++) {
            if (!sane_probe_float(v[axis])) {
                state->joint01_pose_rest_valid = 0;
                return 0;
            }
        }
        if (!state->joint01_pose_rest_valid) {
            for (axis = 0; axis < 3; axis++) {
                state->joint01_pose_rest[j][axis] = v[axis];
            }
        } else {
            for (axis = 0; axis < 3; axis++) {
                v[axis] = state->joint01_pose_rest[j][axis];
            }
        }
    }
    if (!state->joint01_pose_rest_valid) {
        state->joint01_pose_rest_valid = 1;
        log_line("body-chain-physics joint01-pose-override captured person=\"%s\" raw=%p offsets=(0x%03x,0x%03x,0x%03x,0x%03x,0x%03x,0x%03x,0x%03x,0x%03x) count=%d note=\"PoseEditor animates penis_joint01 as SSimpleTransform.Rotation; restoring Spenis_joint01 pose channels before PhysX output\"",
                 person,
                 joint01_raw,
                 body_chain_physics_cfg.joint01_pose_override_offsets[0],
                 body_chain_physics_cfg.joint01_pose_override_offsets[1],
                 body_chain_physics_cfg.joint01_pose_override_offsets[2],
                 body_chain_physics_cfg.joint01_pose_override_offsets[3],
                 body_chain_physics_cfg.joint01_pose_override_offsets[4],
                 body_chain_physics_cfg.joint01_pose_override_offsets[5],
                 body_chain_physics_cfg.joint01_pose_override_offsets[6],
                 body_chain_physics_cfg.joint01_pose_override_offsets[7],
                 body_chain_physics_cfg.joint01_pose_override_offset_count);
    }
    return 1;
}

static int lock_body_chain_joint01_transform(const char *person,
                                             body_chain_person_state_t *state)
{
    static const float identity_rows[3][3] = {
        { 1.0f, 0.0f, 0.0f },
        { 0.0f, 1.0f, 0.0f },
        { 0.0f, 0.0f, 1.0f }
    };
    void *anim_joint_raw[3] = { NULL, NULL, NULL };
    BYTE *base;
    int row, axis;
    if (!body_chain_physics_cfg.override_animation ||
        !body_chain_physics_cfg.joint01_transform_lock ||
        !person || !state) {
        return 1;
    }
    if (state->anim_joint_raw[0]) {
        anim_joint_raw[0] = state->anim_joint_raw[0];
    } else if (!resolve_body_chain_anim_raws(person, anim_joint_raw)) {
        return 0;
    }
    state->anim_joint_raw[0] = anim_joint_raw[0];
    base = (BYTE*)anim_joint_raw[0] + body_chain_physics_cfg.joint01_transform_lock_offset;
    if (!ptr_readable(base, 0x20 + sizeof(float) * 3)) {
        state->anim_joint_raw[0] = NULL;
        return 0;
    }
    for (row = 0; row < 3; row++) {
        float *v = (float*)(base + row * 0x10);
        for (axis = 0; axis < 3; axis++) {
            v[axis] = identity_rows[row][axis];
        }
    }
    if (!state->joint01_transform_lock_logged) {
        state->joint01_transform_lock_logged = 1;
        log_line("body-chain-physics joint01-transform-lock active person=\"%s\" raw=%p rows=(0x%03x,0x%03x,0x%03x) note=\"forcing penis_joint01 local rotation identity so PoseEditor animation cannot own the first chain joint\"",
                 person,
                 anim_joint_raw[0],
                 body_chain_physics_cfg.joint01_transform_lock_offset,
                 body_chain_physics_cfg.joint01_transform_lock_offset + 0x10,
                 body_chain_physics_cfg.joint01_transform_lock_offset + 0x20);
    }
    return 1;
}

static int capture_body_chain_pose_compensation(
    int person_index,
    const char *person,
    body_chain_person_state_t *state)
{
    static const int track_ids[2] = {
        POSEEDIT_TRACK_PENIS_JOINT02,
        POSEEDIT_TRACK_PENIS_JOINT03
    };
    static const char *joint_suffixes[2] = {
        "penis_joint02",
        "penis_joint03"
    };
    poseedit_track_evaluate_t evaluate_track;
    BYTE *tracks[2] = { NULL, NULL };
    float captured_pose[2][3];
    double current_frame;
    int i, axis;

    if (!body_chain_physics_cfg.override_animation ||
        !body_chain_physics_cfg.poseeditor_track_override ||
        !person || !state) {
        return 1;
    }
    if (state->pose_compensation_valid) return 1;
    if (person_index < 0 || person_index >= 4 ||
        !captured_poseedit_this || !captured_poseedit_editpose ||
        !ptr_executable(POSEEDIT_TRACK_EVALUATE_ADDR) ||
        !poseedit_current_frame(&current_frame)) {
        return 0;
    }
    evaluate_track =
        (poseedit_track_evaluate_t)POSEEDIT_TRACK_EVALUATE_ADDR;

    for (axis = 0; axis < 3; axis++) {
        state->pose_compensation[0][axis] = 0.0f;
    }
    for (i = 0; i < 2; i++) {
        char joint_name[256];
        void *joint_raw = NULL;
        void *joint_obj = NULL;
        void *track_obj;
        BYTE *track = poseedit_track_slot(person_index, track_ids[i]);
        BYTE *track_data;
        int key_count;
        float evaluated[3] = { 0.0f, 0.0f, 0.0f };

        if (!track || !ptr_readable(track, POSEEDIT_TRACK_SIZE)) {
            return 0;
        }
        make_body_runtime_name(joint_name, sizeof(joint_name),
                               person, joint_suffixes[i]);
        joint_obj = resolve_find_obj(joint_name, &joint_raw);
        if ((!joint_obj || is_nil_engine_object(joint_raw, joint_obj)) &&
            captured_script_engine) {
            joint_obj = resolve_script_engine_obj(joint_name, &joint_raw);
        }
        track_obj = *(void**)(track + 0x04);
        if (!joint_obj || is_nil_engine_object(joint_raw, joint_obj) ||
            (track_obj != joint_obj && track_obj != joint_raw) ||
            !ptr_readable(track_obj, sizeof(DWORD))) {
            return 0;
        }
        track_data = *(BYTE**)(track + 0x24);
        if (!track_data ||
            (engine_G_NullArray_ptr &&
             track_data == *(BYTE**)engine_G_NullArray_ptr) ||
            !ptr_readable(track_data - sizeof(int), sizeof(int))) {
            return 0;
        }
        key_count = *(int*)(track_data - sizeof(int));
        if (key_count <= 0 || key_count > 100000 ||
            !ptr_readable(track_data,
                          (size_t)key_count * 0x30u) ||
            !evaluate_track(track, evaluated, current_frame)) {
            return 0;
        }
        for (axis = 0; axis < 3; axis++) {
            if (!sane_probe_float(evaluated[axis])) {
                state->pose_compensation_valid = 0;
                return 0;
            }
            captured_pose[i][axis] = evaluated[axis];
        }
        tracks[i] = track;
    }

    /*
     * Apply a zero vector through PoseTrack::Update, the same engine path used
     * by PoseEdit::UpdateObjectsFromTracks.  A stack copy and a one-key zero
     * array leave the real track/keyframes untouched.  This resets the live
     * TJoint through its validated script-property setter before ownership is
     * disconnected, avoiding both Euler inverse approximations and raw joint
     * memory writes.
    */
    for (i = 0; i < 2; i++) {
        if (!poseedit_track_apply_zero(tracks[i], NULL, current_frame)) {
            return 0;
        }
        for (axis = 0; axis < 3; axis++) {
            state->pose_compensation[i + 1][axis] = 0.0f;
        }
    }
    state->pose_compensation_valid = 1;
    if (!state->pose_compensation_logged) {
        state->pose_compensation_logged = 1;
        log_line("body-chain-physics poseeditor ownership neutralized person=\"%s\" frame=%.3f previous_joint02=(%.3f,%.3f,%.3f) previous_joint03=(%.3f,%.3f,%.3f) applied=(0.000,0.000,0.000) source=\"native PoseTrack evaluator/update\" note=\"TK17's validated property path resets all PoseEditor axes before PhysX disconnects the tracks; real keyframes remain untouched\"",
                 person,
                 current_frame,
                 captured_pose[0][0],
                 captured_pose[0][1],
                 captured_pose[0][2],
                 captured_pose[1][0],
                 captured_pose[1][1],
                 captured_pose[1][2]);
    }
    return 1;
}

static int resolve_testicle_physics_raws(const char *person,
                                         void **root_raw_out,
                                         void *joint_raw_out[3])
{
    static const char *joint_names[3] = {
        "Stesticles_joint01", "Stesticles_joint02", "Stesticles_jointEnd"
    };
    char name[256];
    int i;
    if (!person || !person[0] || !root_raw_out || !joint_raw_out) return 0;
    make_body_runtime_name(name, sizeof(name), person, "root");
    *root_raw_out = resolve_axis_map_raw(name);
    if (!*root_raw_out) return 0;
    for (i = 0; i < 3; i++) {
        make_body_runtime_name(name, sizeof(name), person, joint_names[i]);
        joint_raw_out[i] = resolve_axis_map_raw(name);
        if (!joint_raw_out[i]) return 0;
    }
    return 1;
}

static int resolve_testicle_physics_anim_raws(const char *person,
                                              void *anim_joint_raw_out[3])
{
    static const char *joint_names[3] = {
        "testicles_joint01", "testicles_joint02", "testicles_jointEnd"
    };
    char name[256];
    int i;
    if (!person || !person[0] || !anim_joint_raw_out) return 0;
    for (i = 0; i < 3; i++) {
        make_body_runtime_name(name, sizeof(name), person, joint_names[i]);
        anim_joint_raw_out[i] = resolve_axis_map_raw(name);
        if (!anim_joint_raw_out[i]) return 0;
    }
    return 1;
}

static int neutralize_testicle_physics_animation(const char *person,
                                                 body_chain_person_state_t *state)
{
    void *anim_joint_raw[3] = { NULL, NULL, NULL };
    int animation_owner_changed = 0;
    int i, j, axis;
    if (!testicle_physics_cfg.override_animation || !person || !state) return 1;
    if (testicle_physics_cfg.poseeditor_track_override) {
        if (state->pose_track_suppressed &&
            state->pose_track_extra_suppressed[0]) {
            /* PoseEditor no longer drives joint01, joint02, or jointEnd once
               their live tracks are blanked. Do not also write broad TJoint
               transform offsets: those objects can be recycled while a pose
               expands, which can corrupt an unrelated skeleton bone. */
            state->anim_joint_raw[0] = NULL;
            state->anim_joint_raw[1] = NULL;
            state->anim_joint_raw[2] = NULL;
            state->anim_rest_valid = 0;
            return 1;
        }
        state->anim_joint_raw[0] = NULL;
        state->anim_joint_raw[1] = NULL;
        state->anim_joint_raw[2] = NULL;
        state->anim_rest_valid = 0;
        return 0;
    }
    if (!resolve_testicle_physics_anim_raws(person, anim_joint_raw)) {
        state->anim_joint_raw[0] = NULL;
        state->anim_joint_raw[1] = NULL;
        state->anim_joint_raw[2] = NULL;
        state->anim_rest_valid = 0;
        return 0;
    }
    for (i = 0; i < 3; i++) {
        if (state->anim_joint_raw[i] != anim_joint_raw[i]) {
            animation_owner_changed = 1;
        }
    }
    if (animation_owner_changed) {
        for (i = 0; i < 3; i++) {
            state->anim_joint_raw[i] = anim_joint_raw[i];
        }
        state->anim_rest_valid = 0;
        return 0;
    }
    for (i = 0; i < 3; i++) {
        for (j = 0; j < testicle_physics_cfg.animation_override_offset_count; j++) {
            int offset = testicle_physics_cfg.animation_override_offsets[j];
            float neutral[3] = { 0.0f, 0.0f, 0.0f };
            float *v;
            if (offset < 0 ||
                !ptr_readable((BYTE*)anim_joint_raw[i] + offset, sizeof(float) * 3)) {
                state->anim_joint_raw[0] = NULL;
                state->anim_joint_raw[1] = NULL;
                state->anim_joint_raw[2] = NULL;
                state->anim_rest_valid = 0;
                return 0;
            }
            v = (float*)((BYTE*)anim_joint_raw[i] + offset);
            if (offset == 0x078) {
                neutral[0] = 1.0f;
            } else if (offset == 0x088) {
                neutral[1] = 1.0f;
            } else if (offset == 0x098) {
                neutral[2] = 1.0f;
            }
            if (!state->anim_rest_valid) {
                for (axis = 0; axis < 3; axis++) {
                    state->anim_rest[i][j][axis] = neutral[axis];
                }
            }
            for (axis = 0; axis < 3; axis++) {
                v[axis] = state->anim_rest[i][j][axis];
            }
        }
    }
    if (!state->anim_rest_valid) {
        state->anim_rest_valid = 1;
        log_line("testicle-physics override-animation neutralized person=\"%s\" offsets=(0x%03x,0x%03x,0x%03x,0x%03x,0x%03x,0x%03x,0x%03x,0x%03x) count=%d note=\"locking original testicle TJoint channels to neutral while PhysX owns Stesticles joints, including the testicle end bone\"",
                 person,
                 testicle_physics_cfg.animation_override_offsets[0],
                 testicle_physics_cfg.animation_override_offsets[1],
                 testicle_physics_cfg.animation_override_offsets[2],
                 testicle_physics_cfg.animation_override_offsets[3],
                 testicle_physics_cfg.animation_override_offsets[4],
                 testicle_physics_cfg.animation_override_offsets[5],
                 testicle_physics_cfg.animation_override_offsets[6],
                 testicle_physics_cfg.animation_override_offsets[7],
                 testicle_physics_cfg.animation_override_offset_count);
    }
    return 1;
}

static int read_normalized_basis_vector(void *raw, int offset, float out[3])
{
    float *v;
    float len;
    if (!raw || offset < 0 || !out ||
        !ptr_readable((BYTE*)raw + offset, sizeof(float) * 3)) {
        return 0;
    }
    v = (float*)((BYTE*)raw + offset);
    if (!sane_probe_float(v[0]) ||
        !sane_probe_float(v[1]) ||
        !sane_probe_float(v[2])) {
        return 0;
    }
    len = physx_vec3_len(v);
    if (len < 0.000001f || len > 1000.0f) {
        return 0;
    }
    out[0] = v[0] / len;
    out[1] = v[1] / len;
    out[2] = v[2] / len;
    return 1;
}

static float vec3_dot(const float a[3], const float b[3])
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static void body_gravity_direction_drive(const float gravity_raw[3],
                                         const float gravity_ref[3],
                                         const float previous_drive[3],
                                         float out[3])
{
    const float half_pi = 1.57079632679489661923f;
    float current[3];
    float reference[3];
    float aligned[3];
    float axis[3];
    float first_cross[3];
    float second_cross[3];
    float current_len;
    float reference_len;
    float aligned_len;
    float axis_len_sq;
    float cosine;
    float factor;
    float perpendicular_len;
    float bend_angle;
    float bend_scale;
    float previous_len;

    if (!gravity_raw || !gravity_ref || !out) return;

    current_len = physx_vec3_len(gravity_raw);
    reference_len = physx_vec3_len(gravity_ref);
    if (current_len < 0.000001f || reference_len < 0.000001f) {
        out[0] = 0.0f;
        out[1] = 0.0f;
        out[2] = 0.0f;
        return;
    }
    current[0] = gravity_raw[0] / current_len;
    current[1] = gravity_raw[1] / current_len;
    current[2] = gravity_raw[2] / current_len;
    reference[0] = gravity_ref[0] / reference_len;
    reference[1] = gravity_ref[1] / reference_len;
    reference[2] = gravity_ref[2] / reference_len;

    /* Rotate the captured neutral gravity direction onto local -Z. This
       removes the model's small bind-pose tilt before solving bend angles. */
    axis[0] = -reference[1];
    axis[1] = reference[0];
    axis[2] = 0.0f;
    cosine = physx_clampf(-reference[2], -1.0f, 1.0f);
    axis_len_sq = vec3_dot(axis, axis);
    if (axis_len_sq > 0.000001f) {
        first_cross[0] = axis[1] * current[2] - axis[2] * current[1];
        first_cross[1] = axis[2] * current[0] - axis[0] * current[2];
        first_cross[2] = axis[0] * current[1] - axis[1] * current[0];
        second_cross[0] = axis[1] * first_cross[2] - axis[2] * first_cross[1];
        second_cross[1] = axis[2] * first_cross[0] - axis[0] * first_cross[2];
        second_cross[2] = axis[0] * first_cross[1] - axis[1] * first_cross[0];
        factor = (1.0f - cosine) / axis_len_sq;
        aligned[0] = current[0] + first_cross[0] + second_cross[0] * factor;
        aligned[1] = current[1] + first_cross[1] + second_cross[1] * factor;
        aligned[2] = current[2] + first_cross[2] + second_cross[2] * factor;
    } else if (cosine < 0.0f) {
        /* Deterministic 180-degree calibration for an antipodal reference. */
        aligned[0] = current[0];
        aligned[1] = -current[1];
        aligned[2] = -current[2];
    } else {
        aligned[0] = current[0];
        aligned[1] = current[1];
        aligned[2] = current[2];
    }

    aligned_len = physx_vec3_len(aligned);
    if (aligned_len > 0.000001f) {
        aligned[0] /= aligned_len;
        aligned[1] /= aligned_len;
        aligned[2] /= aligned_len;
    }
    perpendicular_len = (float)sqrt((double)(aligned[0] * aligned[0] +
                                               aligned[1] * aligned[1]));
    bend_angle = (float)atan2((double)perpendicular_len,
                              (double)(-aligned[2]));
    bend_scale = bend_angle / half_pi;
    previous_len = previous_drive ?
        (float)sqrt((double)(previous_drive[0] * previous_drive[0] +
                             previous_drive[1] * previous_drive[1])) : 0.0f;
    if (aligned[2] > 0.94f && previous_len > 0.1f) {
        /* Near exactly upside down the fall plane is mathematically
           ambiguous. Keep the established plane until the orientation has
           moved far enough away from the singularity. */
        out[0] = previous_drive[0] / previous_len * bend_scale;
        out[1] = previous_drive[1] / previous_len * bend_scale;
    } else if (perpendicular_len > 0.0001f) {
        out[0] = aligned[0] / perpendicular_len * bend_scale;
        out[1] = aligned[1] / perpendicular_len * bend_scale;
    } else if (aligned[2] > 0.0f) {
        /* Perfectly upside down has no natural fall plane. Preserve an
           established direction, otherwise consistently fall vertically. */
        if (previous_len > 0.1f) {
            out[0] = previous_drive[0] / previous_len * 2.0f;
            out[1] = previous_drive[1] / previous_len * 2.0f;
        } else {
            out[0] = 0.0f;
            out[1] = 2.0f;
        }
    } else {
        out[0] = 0.0f;
        out[1] = 0.0f;
    }
    out[2] = aligned[2] + 1.0f;
}

static void update_body_chain_gravity_filter(body_chain_person_state_t *state,
                                             float dt)
{
    float response_seconds;
    float alpha;
    float delta[2];
    float delta_len;
    float max_step;
    int axis;

    if (!state) return;
    if (!state->gravity_sample.trusted_valid && !state->gravity_drive_filtered_valid) return;
    if (!state->gravity_drive_filtered_valid) {
        for (axis = 0; axis < 3; axis++) {
            state->gravity_drive_filtered[axis] = state->gravity_drive[axis];
        }
        state->gravity_drive_filtered_valid = 1;
        return;
    }

    response_seconds = physics_environment_cfg.gravity_response_ms * 0.001f;
    alpha = response_seconds > 0.0f ? 1.0f-expf(-dt/response_seconds) : 1.0f;
    alpha = physx_clampf(alpha, 0.0f, 1.0f);
    delta[0] = (state->gravity_drive[0] - state->gravity_drive_filtered[0]) * alpha;
    delta[1] = (state->gravity_drive[1] - state->gravity_drive_filtered[1]) * alpha;

    max_step = physics_environment_cfg.gravity_max_degrees_per_second > 0.0f ?
        physics_environment_cfg.gravity_max_degrees_per_second / 90.0f * dt : 0.0f;
    delta_len = (float)sqrt((double)(delta[0] * delta[0] + delta[1] * delta[1]));
    if (max_step > 0.0f && delta_len > max_step) {
        float scale = max_step / delta_len;
        delta[0] *= scale;
        delta[1] *= scale;
    }
    state->gravity_drive_filtered[0] += delta[0];
    state->gravity_drive_filtered[1] += delta[1];
    state->gravity_drive_filtered[2] +=
        (state->gravity_drive[2] - state->gravity_drive_filtered[2]) * alpha;
}

static float camera_rotation_delta(const float current[16], const float previous[16])
{
    float delta = 0.0f;
    int row, col;
    for (row = 0; row < 3; row++) for (col = 0; col < 3; col++)
        delta += physx_absf(current[row * 4 + col] - previous[row * 4 + col]);
    return delta;
}

/* Called once per simulation frame per force source; substeps reuse the
   accepted sample. The full camera version covers translation and rotation. */
static int gravity_sample_live(gravity_sample_t *sample, const void *source,
    const float candidate[3], int valid, DWORD now, float out[3])
{
    DWORD quiet = (DWORD)physics_environment_cfg.gravity_probe_camera_quiet_ms;
    DWORD age = captured_camera_change_tick ? now-captured_camera_change_tick : 0xffffffffu;
    if (quiet < (DWORD)physics_environment_cfg.body_chain_camera_quarantine_ms)
        quiet = (DWORD)physics_environment_cfg.body_chain_camera_quarantine_ms;
    return gravity_sample_update(sample, (uintptr_t)source, candidate, valid,
        physx_simulation_serial, now, (uint32_t)captured_camera_version,
        captured_camera_inverse_valid, age, quiet, out);
}

static int camera_world_to_view_direction(const float world_dir[3], float out[3])
{
    float m[16];
    float v[3];
    float len;
    if (!world_dir || !out || !captured_camera_inverse_valid) {
        return 0;
    }
    memcpy(m, captured_camera_inverse, sizeof(m));

    /* SetWorldMatrixInverse exposes the camera-to-world rotation. Project the
       world direction onto its right/top/back rows to obtain view space. */
    v[0] = world_dir[0] * m[0] + world_dir[1] * m[1] + world_dir[2] * m[2];
    v[1] = world_dir[0] * m[4] + world_dir[1] * m[5] + world_dir[2] * m[6];
    v[2] = world_dir[0] * m[8] + world_dir[1] * m[9] + world_dir[2] * m[10];
    if (!sane_probe_float(v[0]) ||
        !sane_probe_float(v[1]) ||
        !sane_probe_float(v[2])) {
        return 0;
    }
    len = physx_vec3_len(v);
    if (len < 0.000001f || len > 10.0f) {
        return 0;
    }
    out[0] = v[0] / len;
    out[1] = v[1] / len;
    out[2] = v[2] / len;
    return 1;
}

static int camera_view_to_world_point(const float view[3], float out[3])
{
    const float *m = captured_camera_inverse;
    if (!view || !out || !captured_camera_inverse_valid) return 0;

    /* Raw TK17 node positions at 0x0e8 are view-space. AppTracker's inverse
       matrix supplies the camera-to-world basis and camera position. */
    out[0] = m[12] + view[0] * m[0] + view[1] * m[4] + view[2] * m[8];
    out[1] = m[13] + view[0] * m[1] + view[1] * m[5] + view[2] * m[9];
    out[2] = m[14] + view[0] * m[2] + view[1] * m[6] + view[2] * m[10];
    return sane_probe_float(out[0]) &&
           sane_probe_float(out[1]) &&
           sane_probe_float(out[2]);
}

static void *body_collider_engine_pivot_object(const char *person,
                                               const char *node,
                                               const char *fallback_node);
static int body_collider_engine_pivot_view_object(void *object,
                                                  float view[3]);

static int body_collider_engine_pivot_view(const char *person,
                                           const char *node,
                                           const char *fallback_node,
                                           float view[3])
{
    void *object;
    object = body_collider_engine_pivot_object(person, node, fallback_node);
    return body_collider_engine_pivot_view_object(object, view);
}

static void *body_collider_engine_pivot_object(const char *person,
                                               const char *node,
                                               const char *fallback_node)
{
    char name[256];
    void *object;
    resolve_engine_symbols();
    if (!person || !node || !engine_FindObjC) {
        return NULL;
    }
    make_body_runtime_name(name, sizeof(name), person, node);
    object = engine_FindObjC(name);
    if ((!object || is_nil_engine_object(object, object)) && fallback_node) {
        make_body_runtime_name(name, sizeof(name), person, fallback_node);
        object = engine_FindObjC(name);
    }
    if (!object || is_nil_engine_object(object, object)) return NULL;
    return object;
}

static int body_collider_engine_pivot_view_object(void *object,
                                                  float view[3])
{
    /* Named lookups return a current engine object. Cached callers validate
       readability before entering this hot transform query. */
    if (!object || !view || !engine_GetModelViewRotationPivot) {
        return 0;
    }
    /* This TK17 API is explicitly a model-view pivot query. Its result is already in the
       coordinate space consumed by the active renderer projection. */
    engine_GetModelViewRotationPivot(object, view);
    if (!sane_probe_float(view[0]) ||
        !sane_probe_float(view[1]) ||
        !sane_probe_float(view[2])) {
        return 0;
    }
    return 1;
}

static int body_collider_engine_testicle_views(int person_index,
                                               float test01[3],
                                               float test02[3])
{
    char person[32];
    static int logged;
    if (person_index < 0 || person_index >= 4 || !test01 || !test02) return 0;
    _snprintf(person, sizeof(person), "Person%02d", person_index + 1);
    if (!body_collider_engine_pivot_view(person, "testicles_joint01",
                                         "Stesticles_joint01", test01) ||
        !body_collider_engine_pivot_view(person, "testicles_joint02",
                                         "Stesticles_joint02", test02)) {
        return 0;
    }
    if (!logged) {
        logged = 1;
        log_line("body-chain-colliders debug engine-pivots active person=\"%s\" test01_view=(%.5f,%.5f,%.5f) test02_view=(%.5f,%.5f,%.5f) note=\"TK17 GetModelViewRotationPivot values projected directly; collision solver unchanged\"",
                 person,
                 test01[0], test01[1], test01[2],
                 test02[0], test02[1], test02[2]);
    }
    return 1;
}

static int body_chain_mat3_inverse(const float m[9], float out[9])
{
    float c00 = m[4] * m[8] - m[5] * m[7];
    float c01 = m[2] * m[7] - m[1] * m[8];
    float c02 = m[1] * m[5] - m[2] * m[4];
    float det =
        m[0] * (m[4] * m[8] - m[5] * m[7]) -
        m[1] * (m[3] * m[8] - m[5] * m[6]) +
        m[2] * (m[3] * m[7] - m[4] * m[6]);
    float inv_det;
    if (fabsf(det) < 0.000001f || !sane_probe_float(det)) return 0;
    inv_det = 1.0f / det;
    out[0] = c00 * inv_det;
    out[1] = c01 * inv_det;
    out[2] = c02 * inv_det;
    out[3] = (m[5] * m[6] - m[3] * m[8]) * inv_det;
    out[4] = (m[0] * m[8] - m[2] * m[6]) * inv_det;
    out[5] = (m[2] * m[3] - m[0] * m[5]) * inv_det;
    out[6] = (m[3] * m[7] - m[4] * m[6]) * inv_det;
    out[7] = (m[1] * m[6] - m[0] * m[7]) * inv_det;
    out[8] = (m[0] * m[4] - m[1] * m[3]) * inv_det;
    return 1;
}

static void body_chain_mat3_multiply(const float a[9],
                                     const float b[9],
                                     float out[9])
{
    int row, col;
    for (row = 0; row < 3; row++) {
        for (col = 0; col < 3; col++) {
            out[row * 3 + col] =
                a[row * 3 + 0] * b[0 * 3 + col] +
                a[row * 3 + 1] * b[1 * 3 + col] +
                a[row * 3 + 2] * b[2 * 3 + col];
        }
    }
}

static int body_chain_read_mat3_rows(void *raw, float out[9])
{
    static const int offsets[3] = { 0x078, 0x088, 0x098 };
    int row, col;
    if (!raw || !out) return 0;
    for (row = 0; row < 3; row++) {
        const float *src;
        if (!ptr_readable((BYTE*)raw + offsets[row], sizeof(float) * 3)) {
            return 0;
        }
        src = (const float*)((BYTE*)raw + offsets[row]);
        for (col = 0; col < 3; col++) {
            float value = src[col];
            if (!sane_probe_float(value) || physx_absf(value) > 8.0f) {
                return 0;
            }
            out[row * 3 + col] = value;
        }
    }
    return 1;
}

static int body_chain_relative_basis_vector(
    const body_chain_person_state_t *state,
    int offset,
    float out[3])
{
    int row;
    float len;
    if (!state || !out || !state->camera_relative_live_current_valid) {
        return 0;
    }
    if (offset == 0x078) {
        row = 0;
    } else if (offset == 0x088) {
        row = 1;
    } else if (offset == 0x098) {
        row = 2;
    } else {
        return 0;
    }
    out[0] = state->camera_relative_live_current[row * 3 + 0];
    out[1] = state->camera_relative_live_current[row * 3 + 1];
    out[2] = state->camera_relative_live_current[row * 3 + 2];
    if (!sane_probe_float(out[0]) ||
        !sane_probe_float(out[1]) ||
        !sane_probe_float(out[2])) {
        return 0;
    }
    len = physx_vec3_len(out);
    if (len < 0.000001f || len > 8.0f) {
        return 0;
    }
    out[0] /= len;
    out[1] /= len;
    out[2] /= len;
    return 1;
}

/* TK17 stores the animated root orientation in the same camera-contaminated
   space as TRS_group. Their measured row-vector convention cancels as
   root * inverse(TRS_group), leaving a camera-independent body orientation. */
static int body_chain_camera_relative_orientation_step(
    const char *person,
    body_chain_person_state_t *state,
    void *root_raw,
    float delta_out[9],
    float rotation_step_out[3],
    float *energy_out)
{
    char name[256];
    char matched[384];
    void *raw = NULL;
    void *obj = NULL;
    float root_matrix[9];
    float trs_matrix[9];
    float trs_inverse[9];
    float current[9];
    float previous_inverse[9];
    float relative_rotation[9];
    float energy = 0.0f;
    int i;

    if (energy_out) *energy_out = 0.0f;
    if (rotation_step_out) {
        rotation_step_out[0] = 0.0f;
        rotation_step_out[1] = 0.0f;
        rotation_step_out[2] = 0.0f;
    }
    if (!physics_environment_cfg.body_chain_camera_relative_orientation ||
        !person || !state || !root_raw || !delta_out) {
        if (state) state->camera_relative_live_prev_valid = 0;
        return 0;
    }

    raw = state->camera_relative_trs_raw;
    if (!body_chain_read_mat3_rows(raw, trs_matrix)) {
        make_body_runtime_name(name, sizeof(name), person, "TRS_group");
        matched[0] = 0;
        obj = resolve_find_obj(name, &raw);
        if ((!obj || is_nil_engine_object(raw, obj)) && captured_script_engine) {
            obj = resolve_script_engine_obj(name, &raw);
        }
        if (!obj || is_nil_engine_object(raw, obj)) {
        obj = resolve_runtime_exact_target(name, &raw, matched, sizeof(matched));
        }
        if (!obj || !raw || is_nil_engine_object(raw, obj) ||
            !body_chain_read_mat3_rows(raw, trs_matrix)) {
            state->camera_relative_trs_raw = NULL;
            state->camera_relative_live_prev_valid = 0;
            state->camera_relative_live_current_valid = 0;
            return 0;
        }
        state->camera_relative_trs_raw = raw;
    }

    if (!body_chain_read_mat3_rows(root_raw, root_matrix) ||
        !body_chain_mat3_inverse(trs_matrix, trs_inverse)) {
        state->camera_relative_live_prev_valid = 0;
        state->camera_relative_live_current_valid = 0;
        return 0;
    }
    body_chain_mat3_multiply(root_matrix, trs_inverse, current);
    for (i = 0; i < 9; i++) {
        if (!sane_probe_float(current[i]) || physx_absf(current[i]) > 8.0f) {
            state->camera_relative_live_prev_valid = 0;
            state->camera_relative_live_current_valid = 0;
            return 0;
        }
    }
    memcpy(state->camera_relative_live_current, current, sizeof(current));
    state->camera_relative_live_current_valid = 1;

    if (!state->camera_relative_live_prev_valid) {
        memcpy(state->camera_relative_live_prev, current, sizeof(current));
        state->camera_relative_live_prev_valid = 1;
        return 0;
    }

    for (i = 0; i < 9; i++) {
        float delta = current[i] - state->camera_relative_live_prev[i];
        delta_out[i] = delta;
        energy += delta * delta;
    }
    if (rotation_step_out &&
        body_chain_mat3_inverse(state->camera_relative_live_prev,
                                previous_inverse)) {
        const float rad_to_deg = 57.29577951308232f;
        body_chain_mat3_multiply(current, previous_inverse,
                                 relative_rotation);
        rotation_step_out[0] = physx_clampf(
            (relative_rotation[7] - relative_rotation[5]) *
                0.5f * rad_to_deg,
            -45.0f, 45.0f);
        rotation_step_out[1] = physx_clampf(
            (relative_rotation[2] - relative_rotation[6]) *
                0.5f * rad_to_deg,
            -45.0f, 45.0f);
        rotation_step_out[2] = physx_clampf(
            (relative_rotation[3] - relative_rotation[1]) *
                0.5f * rad_to_deg,
            -45.0f, 45.0f);
    }
    memcpy(state->camera_relative_live_prev, current, sizeof(current));
    if (!sane_probe_float(energy) || energy > 16.0f) {
        return 0;
    }
    if (energy_out) *energy_out = energy;
    return 1;
}

static int body_chain_normalize_basis_rows(float basis[9])
{
    int row;
    if (!basis) return 0;
    for (row = 0; row < 3; row++) {
        float *axis = &basis[row * 3];
        float len;
        if (!sane_probe_float(axis[0]) ||
            !sane_probe_float(axis[1]) ||
            !sane_probe_float(axis[2])) {
            return 0;
        }
        len = physx_vec3_len(axis);
        if (len < 0.000001f || len > 8.0f) return 0;
        axis[0] /= len;
        axis[1] /= len;
        axis[2] /= len;
    }
    return 1;
}

static void body_chain_transform_row_vector3(const float value[3],
                                             const float matrix[9],
                                             float out[3])
{
    out[0] = value[0] * matrix[0] +
             value[1] * matrix[3] +
             value[2] * matrix[6];
    out[1] = value[0] * matrix[1] +
             value[1] * matrix[4] +
             value[2] * matrix[7];
    out[2] = value[0] * matrix[2] +
             value[1] * matrix[5] +
             value[2] * matrix[8];
}

static int body_chain_vec3_sane_limit(const float value[3], float limit)
{
    return value && limit > 0.0f &&
           sane_probe_float(value[0]) &&
           sane_probe_float(value[1]) &&
           sane_probe_float(value[2]) &&
           physx_absf(value[0]) <= limit &&
           physx_absf(value[1]) <= limit &&
           physx_absf(value[2]) <= limit;
}

/* Separate actual body translation from root-pivot rotation. TK17 exposes
   both root and TRS_group positions in view space, so root - TRS_group in
   the TRS basis is camera-neutral and does not move when the root rotates
   around its own pivot. The TRS world step supplies whole-person movement. */
static int body_chain_camera_safe_translation_step(
    const char *person,
    body_chain_person_state_t *state,
    void *root_raw,
    int root_offset,
    float out[3],
    DWORD now)
{
    char name[256];
    char matched[384];
    void *trs_raw;
    void *obj = NULL;
    float root_matrix[9];
    float trs_matrix[9];
    float trs_inverse[9];
    float parent_relative_basis[9];
    float parent_rest_inverse[9];
    float parent_delta[9];
    float model_view_basis[9];
    float model_world_basis[9];
    float relative_view[3];
    float relative_local[3];
    float relative_step[3];
    float global_world[3];
    float global_world_step[3];
    float global_local_step[3] = { 0.0f, 0.0f, 0.0f };
    float combined_step[3];
    float *root_pos;
    float *trs_pos;
    float relative_len;
    DWORD quarantine_ms =
        (DWORD)physics_environment_cfg.body_chain_camera_quarantine_ms;
    DWORD camera_age_ms = captured_camera_change_tick ?
        now - captured_camera_change_tick : 0xffffffffu;
    int camera_active;
    int row;

    if (out) out[0] = out[1] = out[2] = 0.0f;
    if (!person || !state || !root_raw || !out || root_offset < 0 ||
        !physics_environment_cfg.body_chain_camera_relative_orientation ||
        !ptr_readable((BYTE*)root_raw + root_offset, sizeof(float) * 3)) {
        return 0;
    }
    if (quarantine_ms < 48u) quarantine_ms = 48u;
    camera_active = captured_camera_inverse_valid &&
        (state->root_drive_camera_seen_version != captured_camera_version ||
         camera_age_ms < quarantine_ms);

    trs_raw = state->camera_relative_trs_raw;
    if (!body_chain_read_mat3_rows(trs_raw, trs_matrix)) {
        make_body_runtime_name(name, sizeof(name), person, "TRS_group");
        matched[0] = 0;
        obj = resolve_find_obj(name, &trs_raw);
        if ((!obj || is_nil_engine_object(trs_raw, obj)) &&
            captured_script_engine) {
            obj = resolve_script_engine_obj(name, &trs_raw);
        }
        if (!obj || is_nil_engine_object(trs_raw, obj)) {
            obj = resolve_runtime_exact_target(name, &trs_raw,
                                               matched, sizeof(matched));
        }
        if (!obj || !trs_raw || is_nil_engine_object(trs_raw, obj) ||
            !body_chain_read_mat3_rows(trs_raw, trs_matrix)) {
            state->camera_relative_trs_raw = NULL;
            return 0;
        }
        state->camera_relative_trs_raw = trs_raw;
    }
    if (!ptr_readable((BYTE*)trs_raw + root_offset, sizeof(float) * 3) ||
        !body_chain_read_mat3_rows(root_raw, root_matrix) ||
        !body_chain_mat3_inverse(trs_matrix, trs_inverse)) {
        return 0;
    }
    root_pos = (float*)((BYTE*)root_raw + root_offset);
    trs_pos = (float*)((BYTE*)trs_raw + root_offset);
    if (!body_chain_vec3_sane_limit(root_pos, 64.0f) ||
        !body_chain_vec3_sane_limit(trs_pos, 64.0f)) {
        return 0;
    }

    body_chain_mat3_multiply(root_matrix, trs_inverse,
                             parent_relative_basis);
    if (!body_chain_normalize_basis_rows(parent_relative_basis)) return 0;
    relative_view[0] = root_pos[0] - trs_pos[0];
    relative_view[1] = root_pos[1] - trs_pos[1];
    relative_view[2] = root_pos[2] - trs_pos[2];
    body_chain_transform_row_vector3(relative_view, trs_inverse,
                                     relative_local);
    if (!body_chain_vec3_sane_limit(relative_local, 64.0f)) return 0;
    memcpy(model_view_basis, trs_matrix, sizeof(model_view_basis));
    if (!body_chain_normalize_basis_rows(model_view_basis)) return 0;

    if (!state->root_translation_initialized ||
        state->root_translation_raw != root_raw) {
        state->root_translation_initialized = 1;
        state->root_translation_raw = root_raw;
        memcpy(state->root_prev, relative_local, sizeof(relative_local));
        memcpy(state->root_translation_parent_rest, parent_relative_basis,
               sizeof(parent_relative_basis));
        state->root_prev_world_valid = 0;
        state->root_translation_camera_hold_active = camera_active ? 1 : 0;
        if (!camera_active && captured_camera_inverse_valid &&
            camera_view_to_world_point(trs_pos, global_world) &&
            body_chain_vec3_sane_limit(global_world, 4096.0f)) {
            memcpy(state->root_prev_world, global_world,
                   sizeof(global_world));
            state->root_prev_world_valid = 1;
        }
        state->root_drive_camera_seen_version = captured_camera_version;
        return 1;
    }

    relative_step[0] = relative_local[0] - state->root_prev[0];
    relative_step[1] = relative_local[1] - state->root_prev[1];
    relative_step[2] = relative_local[2] - state->root_prev[2];
    memcpy(state->root_prev, relative_local, sizeof(relative_local));
    if (!body_chain_vec3_sane_limit(relative_step, 8.0f)) return 0;
    relative_len = physx_vec3_len(relative_step);
    if (relative_len > 4.0f) return 0;
    if (camera_active && relative_len > 0.0f) {
        const float deadzone = 0.0010f;
        if (relative_len <= deadzone) {
            relative_step[0] = relative_step[1] = relative_step[2] = 0.0f;
        } else {
            float keep = (relative_len - deadzone) / relative_len;
            relative_step[0] *= keep;
            relative_step[1] *= keep;
            relative_step[2] *= keep;
        }
    }

    if (camera_active) {
        state->root_translation_camera_hold_active = 1;
    } else if (captured_camera_inverse_valid &&
               camera_view_to_world_point(trs_pos, global_world) &&
               body_chain_vec3_sane_limit(global_world, 4096.0f)) {
        for (row = 0; row < 3; row++) {
            const float *view_axis = &model_view_basis[row * 3];
            float *world_axis = &model_world_basis[row * 3];
            world_axis[0] = view_axis[0] * captured_camera_inverse[0] +
                            view_axis[1] * captured_camera_inverse[4] +
                            view_axis[2] * captured_camera_inverse[8];
            world_axis[1] = view_axis[0] * captured_camera_inverse[1] +
                            view_axis[1] * captured_camera_inverse[5] +
                            view_axis[2] * captured_camera_inverse[9];
            world_axis[2] = view_axis[0] * captured_camera_inverse[2] +
                            view_axis[1] * captured_camera_inverse[6] +
                            view_axis[2] * captured_camera_inverse[10];
        }
        if (body_chain_normalize_basis_rows(model_world_basis)) {
            if (!state->root_prev_world_valid ||
                state->root_translation_camera_hold_active) {
                memcpy(state->root_prev_world, global_world,
                       sizeof(global_world));
                state->root_prev_world_valid = 1;
                state->root_translation_camera_hold_active = 0;
            } else {
                global_world_step[0] = global_world[0] -
                    state->root_prev_world[0];
                global_world_step[1] = global_world[1] -
                    state->root_prev_world[1];
                global_world_step[2] = global_world[2] -
                    state->root_prev_world[2];
                memcpy(state->root_prev_world, global_world,
                       sizeof(global_world));
                if (body_chain_vec3_sane_limit(global_world_step, 8.0f) &&
                    physx_vec3_len(global_world_step) <= 4.0f) {
                    global_local_step[0] =
                        vec3_dot(global_world_step, &model_world_basis[0]);
                    global_local_step[1] =
                        vec3_dot(global_world_step, &model_world_basis[3]);
                    global_local_step[2] =
                        vec3_dot(global_world_step, &model_world_basis[6]);
                }
            }
        }
    }

    if (!body_chain_mat3_inverse(state->root_translation_parent_rest,
                                 parent_rest_inverse)) {
        return 0;
    }
    body_chain_mat3_multiply(parent_rest_inverse, parent_relative_basis,
                             parent_delta);
    if (!body_chain_normalize_basis_rows(parent_delta)) return 0;
    combined_step[0] = relative_step[0] + global_local_step[0];
    combined_step[1] = relative_step[1] + global_local_step[1];
    combined_step[2] = relative_step[2] + global_local_step[2];
    out[0] = vec3_dot(combined_step, &parent_delta[0]);
    out[1] = vec3_dot(combined_step, &parent_delta[3]);
    out[2] = vec3_dot(combined_step, &parent_delta[6]);
    state->root_drive_camera_seen_version = captured_camera_version;
    return body_chain_vec3_sane_limit(out, 8.0f);
}

static float body_chain_dot9(const float a[9], const float b[9])
{
    float result = 0.0f;
    int i;
    for (i = 0; i < 9; i++) result += a[i] * b[i];
    return result;
}

static void body_chain_train_camera_relative_orientation(
    body_chain_person_state_t *state,
    const float delta[9],
    float energy,
    float trusted_h,
    float trusted_v,
    float trusted_d)
{
    const float learning_rate = 0.12f;
    float prediction_h;
    float prediction_v;
    float prediction_d;
    float scale;
    int i;
    if (!state || !delta || energy < 0.0000001f || energy > 4.0f ||
        !sane_probe_float(trusted_h) || !sane_probe_float(trusted_v) ||
        !sane_probe_float(trusted_d) ||
        physx_absf(trusted_h) > 2.5f || physx_absf(trusted_v) > 2.5f ||
        physx_absf(trusted_d) > 2.5f) {
        return;
    }
    prediction_h = body_chain_dot9(state->camera_relative_h_coeff, delta);
    prediction_v = body_chain_dot9(state->camera_relative_v_coeff, delta);
    prediction_d = body_chain_dot9(state->camera_relative_d_coeff, delta);
    scale = learning_rate / (energy + 0.000001f);
    for (i = 0; i < 9; i++) {
        state->camera_relative_h_coeff[i] = physx_clampf(
            state->camera_relative_h_coeff[i] +
                (trusted_h - prediction_h) * delta[i] * scale,
            -20.0f, 20.0f);
        state->camera_relative_v_coeff[i] = physx_clampf(
            state->camera_relative_v_coeff[i] +
                (trusted_v - prediction_v) * delta[i] * scale,
            -20.0f, 20.0f);
        state->camera_relative_d_coeff[i] = physx_clampf(
            state->camera_relative_d_coeff[i] +
                (trusted_d - prediction_d) * delta[i] * scale,
            -20.0f, 20.0f);
    }
    if (state->camera_relative_calibration_samples < 1000000u) {
        state->camera_relative_calibration_samples++;
    }
    state->camera_relative_h_peak = physx_absf(trusted_h) >
            state->camera_relative_h_peak * 0.998f ?
        physx_absf(trusted_h) : state->camera_relative_h_peak * 0.998f;
    state->camera_relative_v_peak = physx_absf(trusted_v) >
            state->camera_relative_v_peak * 0.998f ?
        physx_absf(trusted_v) : state->camera_relative_v_peak * 0.998f;
    state->camera_relative_d_peak = physx_absf(trusted_d) >
            state->camera_relative_d_peak * 0.998f ?
        physx_absf(trusted_d) : state->camera_relative_d_peak * 0.998f;
}

static int body_chain_predict_camera_relative_orientation(
    const body_chain_person_state_t *state,
    const float delta[9],
    float energy,
    float *h_out,
    float *v_out,
    float *d_out)
{
    float h;
    float v;
    float d;
    float h_limit;
    float v_limit;
    float d_limit;
    if (!state || !delta || !h_out || !v_out || !d_out ||
        state->camera_relative_calibration_samples < 12u ||
        energy < 0.0000001f || energy > 4.0f) {
        return 0;
    }
    h = body_chain_dot9(state->camera_relative_h_coeff, delta);
    v = body_chain_dot9(state->camera_relative_v_coeff, delta);
    d = body_chain_dot9(state->camera_relative_d_coeff, delta);
    if (!sane_probe_float(h) || !sane_probe_float(v) ||
        !sane_probe_float(d)) return 0;
    h_limit = state->camera_relative_h_peak * 1.35f;
    v_limit = state->camera_relative_v_peak * 1.35f;
    d_limit = state->camera_relative_d_peak * 1.35f;
    if (h_limit < body_chain_physics_cfg.horizontal_deadzone * 4.0f) {
        h_limit = body_chain_physics_cfg.horizontal_deadzone * 4.0f;
    }
    if (v_limit < body_chain_physics_cfg.vertical_deadzone * 4.0f) {
        v_limit = body_chain_physics_cfg.vertical_deadzone * 4.0f;
    }
    if (d_limit < body_chain_physics_cfg.translation_deadzone * 4.0f) {
        d_limit = body_chain_physics_cfg.translation_deadzone * 4.0f;
    }
    *h_out = physx_clampf(h, -h_limit, h_limit);
    *v_out = physx_clampf(v, -v_limit, v_limit);
    *d_out = physx_clampf(d, -d_limit, d_limit);
    return 1;
}

#include "physx_camera_test.c"

static int room_collision_is_enabled(void);
static int room_collision_debug_any(void);
static int room_collision_resolve_sphere(const float center[3], float radius,
                                         float correction[3],
                                         int *mesh_index_out);
static int room_collision_resolve_swept_sphere(
    const float start[3], const float end[3], float radius,
    float correction[3], int *mesh_index_out);
static int single_bone_room_contacts(const float center[3],float radius,
    physx_contact_set_t *out);
static int room_collision_world_vector_to_body_local(
    const body_chain_collider_person_state_t *state,
    const float world_vector[3], float local[3]);
static int room_collision_body_local_point_to_world(
    const body_chain_collider_person_state_t *state,
    const float local[3], float world[3]);
static int body_collision_world_vector_to_local(
    const body_chain_collider_person_state_t *state,
    const float world_vector[3], float local[3]);
static int body_collision_local_point_to_world(
    const body_chain_collider_person_state_t *state,
    const float local[3], float world[3]);
#include "physx_colliders.c"
#include "physx_single_bone_contact.c"

static void run_butt_physics(DWORD now);
static int butt_physics_apply_all_outputs(int capture_animation);
static void run_butt_physics_late_ownership(DWORD now);
static void reset_butt_physics_all(int restore_output);

#include "physx_physics.c"
#include "physx_butt.c"
#include "physx_room_wind.c"
#include "physx_room_collision.c"
#include "physx_sidecar.c"

static void physx_tick(void)
{
    DWORD now = GetTickCount();
    LONGLONG total_start;
    LONGLONG phase_start;
    physx_perf_prepare(now);
    physx_simulation_serial++;
    if (!physx_simulation_serial) physx_simulation_serial = 1;
    total_start = physx_perf_counter();

    phase_start = physx_perf_counter();
    body_chain_poll_poseeditor_mode(now);
    body_profile_set_active_person_config(-1);
    refresh_global_config(now);
    {
        LONG transition = InterlockedExchange(
            &body_chain_runtime_mode_transition_pending, 0);
        if (transition == 2) {
            reset_runtime_body_chain_physics("leave-PoseEditor");
            body_chain_prepare_runtime_mode_from_poseeditor();
        } else if (transition == 1) {
            reset_runtime_body_chain_physics("enter-PoseEditor");
        }
    }
    patch_poseedit_inittracks_hook();
    scan_sidecars(now);
    run_sidecar_hot_reload(now);
    run_room_wind_hot_reload(now);
    sync_room_sidecar_scene_activity(now);
    run_room_collision_hot_reload(now);
    body_profile_observe_tk17_body_logs(now);
    physx_perf_add(PHYSX_PERF_HOUSEKEEPING, phase_start);

    /* Track ownership is restored during the body-physics phase.  Evaluate
       the current PoseEditor pose on the following rendered frame, after the
       settings callback and ownership release have both completed. */
    run_poseeditor_track_handoff_refresh(now);

    phase_start = physx_perf_counter();
    update_targets(now);
    body_profile_probe_runtime_bindings(now);
    physx_perf_add(PHYSX_PERF_BINDINGS, phase_start);

    phase_start = physx_perf_counter();
    if (defaults_cfg.debug) run_body_chain_probe(now);
    if (!axis_map_probe_cfg.enabled && !root_drive_probe_cfg.enabled && !write_sweep_probe_cfg.enabled && !body_chain_physics_cfg.enabled && !testicle_physics_cfg.enabled) run_world_motion_probes(now);
    run_transform_probe(now);
    run_axis_map_probe(now);
    run_root_drive_probe(now);
    run_write_sweep_probe(now);
    run_collision_auto_test(now);
    physx_perf_add(PHYSX_PERF_DIAGNOSTICS, phase_start);

    phase_start = physx_perf_counter();
    refresh_body_colliders_for_physics(now);
    physx_perf_add(PHYSX_PERF_COLLIDER_REFRESH, phase_start);

    phase_start = physx_perf_counter();
    body_chain_prime_axis_references(now);
    physx_perf_add(PHYSX_PERF_AXIS_REFERENCES, phase_start);

    phase_start = physx_perf_counter();
    run_body_chain_physics(now);
    physx_perf_add(PHYSX_PERF_PENIS_PHYSICS, phase_start);

    phase_start = physx_perf_counter();
    run_testicle_physics(now);
    physx_perf_add(PHYSX_PERF_TESTICLE_PHYSICS, phase_start);

    phase_start = physx_perf_counter();
    run_breasts_physics(now);
    physx_perf_add(PHYSX_PERF_BREASTS_PHYSICS, phase_start);

    phase_start = physx_perf_counter();
    run_butt_physics(now);
    physx_perf_add(PHYSX_PERF_BUTT_PHYSICS, phase_start);
    publish_body_chain_runtime_ownership();

    phase_start = physx_perf_counter();
    run_camera_contamination_test(now);
    run_write_tests(now);
    run_body_probe(now);
    run_addon_binding_probe(now);
    physx_perf_add(PHYSX_PERF_DIAGNOSTICS, phase_start);

    phase_start = physx_perf_counter();
    run_chain_simulations(now);
    physx_perf_add(PHYSX_PERF_ADDON_SIMULATION, phase_start);

    phase_start = physx_perf_counter();
    rebuild_addon_constraint_suppression_cache();
    rebuild_addon_animation_suppression_cache();
    physx_perf_add(PHYSX_PERF_ADDON_SUPPRESSION_CACHE, phase_start);
    body_profile_set_active_person_config(-1);
    physx_perf_add(PHYSX_PERF_TOTAL, total_start);
    physx_perf_report(now);
}

#include "physx_hooks_core.c"
#include "physx_collider_draw.c"
#include "physx_render_hooks.c"

__declspec(dllexport) int loadextension(void)
{
    load_global_config();
    resolve_engine_symbols();
    patch_all_modules();
    patch_config_editor_param_change();
    patch_person_context_rebuild();
    patch_app_main_command();
    scan_sidecars(GetTickCount());
    log_line("loadextension");
    return 1;
}

__declspec(dllexport) int on_create(void)
{
    load_global_config();
    resolve_engine_symbols();
    patch_all_modules();
    patch_config_editor_param_change();
    patch_person_context_rebuild();
    patch_app_main_command();
    scan_sidecars(GetTickCount());
    log_line("on_create");
    return 1;
}

BOOL WINAPI DllMain(HINSTANCE hinst, DWORD reason, LPVOID reserved)
{
    (void)reserved;
    if (reason == DLL_PROCESS_ATTACH) {
        self_module = hinst;
        plugin_attach_tick = GetTickCount();
        DisableThreadLibraryCalls(hinst);
        InitializeCriticalSection(&log_lock);
        clear_log_file_on_startup();
        log_ready = 1;
        log_line("NC-TK17-PhysX.dll attached");
        load_global_config();
    } else if (reason == DLL_PROCESS_DETACH) {
        camera_contamination_test_release_mouse();
        InterlockedExchange(&camera_contamination_test_cfg.active, 0);
        restore_collision_auto_test_active();
        restore_addon_constraint_count_getter();
        restore_runtime_animation_member_setters();
        restore_poseedit_inittracks_hook();
        reset_runtime_body_chain_physics("dll-detach");
        restore_tk17_testicle_inertia_all(GetTickCount());
        restore_tk17_breasts_inertia_all(GetTickCount());
        reset_butt_physics_all(1);
        reset_body_chain_physics();
        destroy_body_chain_hook5_overlay();
        log_line("NC-TK17-PhysX.dll detached");
        log_ready = 0;
        DeleteCriticalSection(&log_lock);
    }
    return TRUE;
}
