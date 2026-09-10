/* Interoperability surface for optional binary plugins.

   The ABI deliberately exposes a point query rather than PhysX's internal
   arrays. That keeps sidecar overlay details, collider layout changes, and
   the simulation's scoped-refresh implementation private to this DLL. */

/* Renderer discovery may happen after this plugin's startup IAT scan.
   A cooperating renderer consumer can register the actual device before
   adding its own wrappers. Never retake slots after successful registration:
   another plugin may legitimately have chained above us. */
__declspec(dllexport) int __cdecl NCTK17PhysX_RegisterD3D8DeviceV1(IDirect3DDevice8 *device)
{
    D3DDEVICE_CREATION_PARAMETERS parameters;
    if (!device || !ptr_readable(device, sizeof(void*)) ||
        !ptr_readable(*(void**)device, sizeof(void*) * 38)) return 0;
    if (real_d3d8_Present && real_d3d8_EndScene) return 1;
    memset(&parameters, 0, sizeof(parameters));
    if (SUCCEEDED(IDirect3DDevice8_GetCreationParameters(device, &parameters)))
        physx_d3d8_render_hwnd = parameters.hFocusWindow;
    patch_d3d8_device(device);
    return real_d3d8_Present && real_d3d8_EndScene;
}

__declspec(dllexport) unsigned int __cdecl NCTK17PhysX_BodyColliderStatusV1(void)
{
    unsigned int status = 0;
    int i;
    if (physx_simulation_serial) status |= 1u;
    if (captured_camera_inverse_valid) status |= 2u;
    if (body_chain_collider_global_cfg.enabled) status |= 4u;
    for (i = 0; i < 4; i++)
        if (body_chain_collider_states[i].ready && body_chain_collider_states[i].basis_valid)
            status |= 1u << (8 + i);
    return status;
}

#define NC_TK17_PHYSX_BODY_QUERY_API_V1 0x00010000u
#define NC_TK17_PHYSX_POST_ANIMATION_CALLBACK_SLOTS 8
#define NC_TK17_PHYSX_BLEND_OVERLAY_SLOTS 8
#define NC_TK17_PHYSX_BLEND_OVERLAY_MAX_AGE_MS 500u

typedef struct physx_public_blend_overlay_t {
    void *control;
    float weight;
    DWORD tick;
    volatile LONG active;
} physx_public_blend_overlay_t;

static physx_public_blend_overlay_t physx_public_blend_overlays[
    NC_TK17_PHYSX_BLEND_OVERLAY_SLOTS];
static volatile LONG physx_public_blend_overlay_active_count;

static int physx_public_is_blend_control(const void *object)
{
    if (!object || !engine_ObjectGetTypeInfo ||
        !engine_BlendControlTypeInfo ||
        !ptr_readable(object, sizeof(void*)))
        return 0;
    return engine_ObjectGetTypeInfo(object) == engine_BlendControlTypeInfo;
}

/* Short-lived and pointer-specific: this lets a temporary contribution
   survive compiled animation evaluation without changing PoseEditor tracks. */
__declspec(dllexport) int __cdecl
NCTK17PhysX_SetBlendControlOverlay(void *control, float weight, int active)
{
    int index;
    int free_index = -1;
    DWORD now = GetTickCount();
    if (!physx_public_is_blend_control(control)) return 0;
    if (active && (!_finite(weight) || weight < 0.0f || weight > 1.0f))
        return 0;
    for (index = 0; index < NC_TK17_PHYSX_BLEND_OVERLAY_SLOTS; index++) {
        physx_public_blend_overlay_t *entry =
            &physx_public_blend_overlays[index];
        if (entry->control == control) {
            if (!active) {
                if (InterlockedExchange(&entry->active, 0))
                    InterlockedDecrement(
                        &physx_public_blend_overlay_active_count);
                entry->control = NULL;
                return 1;
            }
            entry->weight = weight;
            entry->tick = now;
            MemoryBarrier();
            if (!InterlockedExchange(&entry->active, 1))
                InterlockedIncrement(
                    &physx_public_blend_overlay_active_count);
            return 1;
        }
        if (free_index < 0 &&
            (!InterlockedCompareExchange(&entry->active, 0, 0) ||
             now - entry->tick > NC_TK17_PHYSX_BLEND_OVERLAY_MAX_AGE_MS))
            free_index = index;
    }
    if (!active) return 1;
    if (free_index < 0) return 0;
    if (InterlockedExchange(
            &physx_public_blend_overlays[free_index].active, 0))
        InterlockedDecrement(&physx_public_blend_overlay_active_count);
    physx_public_blend_overlays[free_index].control = control;
    physx_public_blend_overlays[free_index].weight = weight;
    physx_public_blend_overlays[free_index].tick = now;
    MemoryBarrier();
    InterlockedExchange(&physx_public_blend_overlays[free_index].active, 1);
    InterlockedIncrement(&physx_public_blend_overlay_active_count);
    return 1;
}

static int physx_public_get_blend_control_overlay(void *control,
                                                  float *weight)
{
    int index;
    DWORD now;
    if (!control || !weight ||
        InterlockedCompareExchange(
            &physx_public_blend_overlay_active_count, 0, 0) <= 0)
        return 0;
    now = GetTickCount();
    for (index = 0; index < NC_TK17_PHYSX_BLEND_OVERLAY_SLOTS; index++) {
        physx_public_blend_overlay_t *entry =
            &physx_public_blend_overlays[index];
        if (!InterlockedCompareExchange(&entry->active, 0, 0) ||
            entry->control != control)
            continue;
        if (now - entry->tick > NC_TK17_PHYSX_BLEND_OVERLAY_MAX_AGE_MS) {
            if (InterlockedExchange(&entry->active, 0))
                InterlockedDecrement(
                    &physx_public_blend_overlay_active_count);
            entry->control = NULL;
            return 0;
        }
        *weight = entry->weight;
        return _finite(*weight) && *weight >= 0.0f && *weight <= 1.0f;
    }
    return 0;
}

/* Invoke the registered BlendControl.Weight setter so compiled animation
   consumers are invalidated. The TK17 implementation at SYS+0xD7CB0 reads
   and writes [ecx+0x10], proving that ECX is the BlendControl base pointer
   (not its +0x08 ScriptObject interface). */
__declspec(dllexport) int __cdecl
NCTK17PhysX_SetBlendControlWeight(void *control, float weight)
{
    void **slot;
    script_f32_set_property_t setter;
    float stored;
    static LONG trace_count;
    if (!physx_public_is_blend_control(control) || !_finite(weight) ||
        weight < 0.0f || weight > 1.0f)
        return 0;
    slot = runtime_animation_member_setter_slot(
        SCRIPT_PROPERTY_BLENDCONTROL_WEIGHT);
    if (!slot || !ptr_readable(slot, sizeof(void*)) ||
        !ptr_executable(*slot))
        return 0;
    setter = (script_f32_set_property_t)*slot;
    setter(control, SCRIPT_PROPERTY_BLENDCONTROL_WEIGHT, weight);
    if (!ptr_readable((const BYTE*)control + 0x10, sizeof(stored)))
        return 1;
    stored = *(const float*)((const BYTE*)control + 0x10);
    if (defaults_cfg.debug && InterlockedIncrement(&trace_count) <= 16)
        log_line("public BlendControl.Weight setter applied control=%p requested=%.3f stored=%.3f setter=%p",
                 control, weight, stored, (void*)setter);
    return _finite(stored) && physx_absf(stored - weight) <= 0.002f;
}

static PVOID volatile physx_post_animation_callbacks[
    NC_TK17_PHYSX_POST_ANIMATION_CALLBACK_SLOTS];

static void physx_public_run_post_animation_callbacks(void)
{
    int index;
    for (index = 0;
         index < NC_TK17_PHYSX_POST_ANIMATION_CALLBACK_SLOTS;
         index++) {
        physx_post_animation_callback_t callback =
            (physx_post_animation_callback_t)
                InterlockedCompareExchangePointer(
                    &physx_post_animation_callbacks[index], NULL, NULL);
        if (callback) callback();
    }
}

static int physx_public_has_post_animation_callbacks(void)
{
    int index;
    for (index = 0;
         index < NC_TK17_PHYSX_POST_ANIMATION_CALLBACK_SLOTS;
         index++) {
        if (InterlockedCompareExchangePointer(
                &physx_post_animation_callbacks[index], NULL, NULL))
            return 1;
    }
    return 0;
}

__declspec(dllexport) int __cdecl
NCTK17PhysX_RegisterPostAnimationCallback(
    physx_post_animation_callback_t callback)
{
    int index;
    if (!callback) return 0;
    for (index = 0;
         index < NC_TK17_PHYSX_POST_ANIMATION_CALLBACK_SLOTS;
         index++) {
        PVOID existing = InterlockedCompareExchangePointer(
            &physx_post_animation_callbacks[index], NULL, NULL);
        if (existing == (PVOID)callback) return 1;
        if (!existing && InterlockedCompareExchangePointer(
                &physx_post_animation_callbacks[index], (PVOID)callback,
                NULL) == NULL) {
            log_line("public post-animation callback registered slot=%d callback=%p note=\"temporary consumer overlays run after animation and before transform traversal\"",
                     index, (void*)callback);
            return 1;
        }
    }
    return 0;
}

__declspec(dllexport) int __cdecl
NCTK17PhysX_UnregisterPostAnimationCallback(
    physx_post_animation_callback_t callback)
{
    int index;
    if (!callback) return 0;
    for (index = 0;
         index < NC_TK17_PHYSX_POST_ANIMATION_CALLBACK_SLOTS;
         index++) {
        if (InterlockedCompareExchangePointer(
                &physx_post_animation_callbacks[index], NULL, NULL) ==
            (PVOID)callback) {
            return InterlockedCompareExchangePointer(
                       &physx_post_animation_callbacks[index], NULL,
                       (PVOID)callback) == (PVOID)callback;
        }
    }
    return 0;
}

typedef struct nc_tk17_physx_body_hit_v1_t {
    unsigned int size;
    unsigned int version;
    int person_index;       /* 1..4 */
    int primitive_kind;     /* 1=ellipsoid, 2=tapered capsule, 3=penis */
    int node_start;
    int node_end;
    float segment_t;
    float signed_distance;  /* <= tolerance is a hit */
} nc_tk17_physx_body_hit_v1_t;

static int physx_public_world_to_view_point(const float world[3],
                                            float view[3])
{
    const float *m = captured_camera_inverse;
    float delta[3];
    if (!world || !view || !captured_camera_inverse_valid) return 0;
    delta[0] = world[0] - m[12];
    delta[1] = world[1] - m[13];
    delta[2] = world[2] - m[14];
    view[0] = delta[0] * m[0] + delta[1] * m[1] + delta[2] * m[2];
    view[1] = delta[0] * m[4] + delta[1] * m[5] + delta[2] * m[6];
    view[2] = delta[0] * m[8] + delta[1] * m[9] + delta[2] * m[10];
    return sane_probe_float(view[0]) && sane_probe_float(view[1]) &&
           sane_probe_float(view[2]);
}

static float physx_public_visual_radius(
    const body_chain_collider_config_t *cfg, int node)
{
    float radius;
    if (!cfg || node < 0 || node >= BODY_COLLIDER_NODE_COUNT) return 0.0f;
    radius = body_chain_radius_vec3_max(cfg->node_radius[node]) *
             cfg->response_radius_scale;
    return physx_clampf(radius, 0.0001f, 4.0f);
}

static float physx_public_point_ellipsoid_margin(
    const body_chain_collider_person_state_t *state,
    const body_chain_collider_config_t *cfg, int node,
    const float point_view[3])
{
    float delta[3];
    float local[3];
    float radius[3];
    float normalized;
    float minimum_radius;
    int axis;
    if (!state || !cfg || !point_view || node < 0 ||
        node >= BODY_COLLIDER_NODE_COUNT || !state->valid[node]) {
        return 99999.0f;
    }
    for (axis = 0; axis < 3; axis++) {
        delta[axis] = point_view[axis] - state->view_position[node][axis];
        radius[axis] = physx_clampf(
            cfg->node_radius[node][axis] * cfg->response_radius_scale,
            0.0001f, 4.0f);
    }
    local[0] = vec3_dot(delta, state->basis_h);
    local[1] = vec3_dot(delta, state->basis_v);
    local[2] = vec3_dot(delta, state->basis_s);
    normalized = (float)sqrt((double)(
        (local[0] * local[0]) / (radius[0] * radius[0]) +
        (local[1] * local[1]) / (radius[1] * radius[1]) +
        (local[2] * local[2]) / (radius[2] * radius[2])));
    minimum_radius = radius[0];
    if (radius[1] < minimum_radius) minimum_radius = radius[1];
    if (radius[2] < minimum_radius) minimum_radius = radius[2];
    return (normalized - 1.0f) * minimum_radius;
}

static float physx_public_point_tapered_capsule_margin(
    const float point[3], const float start[3], const float end[3],
    float start_radius, float end_radius, float *segment_t_out)
{
    float direction[3];
    float relative[3];
    float closest[3];
    float length_squared;
    float t = 0.0f;
    float radius;
    float delta[3];
    int axis;
    for (axis = 0; axis < 3; axis++) {
        direction[axis] = end[axis] - start[axis];
        relative[axis] = point[axis] - start[axis];
    }
    length_squared = vec3_dot(direction, direction);
    if (length_squared > 0.000001f) {
        t = physx_clampf(vec3_dot(relative, direction) / length_squared,
                         0.0f, 1.0f);
    }
    for (axis = 0; axis < 3; axis++) {
        closest[axis] = start[axis] + direction[axis] * t;
        delta[axis] = point[axis] - closest[axis];
    }
    radius = start_radius + (end_radius - start_radius) * t;
    if (segment_t_out) *segment_t_out = t;
    return physx_vec3_len(delta) - radius;
}

static void physx_public_consider(
    float margin, int person_index, int primitive_kind,
    int node_start, int node_end, float segment_t,
    float *best_margin, nc_tk17_physx_body_hit_v1_t *best)
{
    if (!best_margin || !best || !sane_probe_float(margin) ||
        margin >= *best_margin) return;
    *best_margin = margin;
    best->person_index = person_index + 1;
    best->primitive_kind = primitive_kind;
    best->node_start = node_start;
    best->node_end = node_end;
    best->segment_t = segment_t;
    best->signed_distance = margin;
}

__declspec(dllexport) unsigned int __cdecl
NCTK17PhysX_BodyColliderApiVersion(void)
{
    return NC_TK17_PHYSX_BODY_QUERY_API_V1;
}

__declspec(dllexport) int __cdecl NCTK17PhysX_RequestBodyCollidersV1(void)
{
    static volatile LONG first_request_logged;
    InterlockedExchange(&body_chain_external_query_tick, (LONG)GetTickCount());
    if (InterlockedCompareExchange(&first_request_logged, 1, 0) == 0) {
        log_line("public body-collider API consumer active version=0x%08x note=\"full collider refresh is requested only during the short consumer heartbeat\"",
                 NC_TK17_PHYSX_BODY_QUERY_API_V1);
    }
    return body_chain_collider_global_cfg.enabled ? 1 : 0;
}

__declspec(dllexport) int __cdecl NCTK17PhysX_QueryBodyColliderV1(
    const float world_point[3], float tolerance,
    nc_tk17_physx_body_hit_v1_t *hit, unsigned int hit_size)
{
    static const int limb_edges[4][2] = {
        { BODY_COLLIDER_HIP_L, BODY_COLLIDER_THIGH_L },
        { BODY_COLLIDER_THIGH_L, BODY_COLLIDER_KNEE_L },
        { BODY_COLLIDER_HIP_R, BODY_COLLIDER_THIGH_R },
        { BODY_COLLIDER_THIGH_R, BODY_COLLIDER_KNEE_R }
    };
    nc_tk17_physx_body_hit_v1_t best;
    float point_view[3];
    float best_margin = 99999.0f;
    int ready_person_count = 0;
    int person_index;

    InterlockedExchange(&body_chain_external_query_tick, (LONG)GetTickCount());
    if (!world_point || !hit || hit_size < sizeof(*hit) ||
        !body_chain_collider_global_cfg.enabled ||
        !physx_public_world_to_view_point(world_point, point_view)) {
        return -1;
    }
    if (!sane_probe_float(tolerance)) tolerance = 0.0f;
    tolerance = physx_clampf(tolerance, 0.0f, 0.25f);
    memset(&best, 0, sizeof(best));
    best.size = sizeof(best);
    best.version = NC_TK17_PHYSX_BODY_QUERY_API_V1;
    best.node_start = -1;
    best.node_end = -1;
    best.signed_distance = best_margin;

    for (person_index = 0; person_index < 4; person_index++) {
        const body_chain_collider_person_state_t *state =
            &body_chain_collider_states[person_index];
        const body_chain_collider_config_t *person_cfg =
            &body_chain_collider_person_cfg[person_index];
        int node;
        int edge_index;
        if (!person_cfg->enabled || !state->ready || !state->basis_valid) {
            continue;
        }
        ready_person_count++;

        for (node = 0; node < BODY_COLLIDER_NODE_COUNT; node++) {
            float margin;
            if (!state->valid[node]) continue;
            margin = physx_public_point_ellipsoid_margin(
                state, person_cfg, node, point_view);
            physx_public_consider(margin, person_index, 1, node, -1, 0.0f,
                                  &best_margin, &best);
        }

        for (edge_index = 0; edge_index < BODY_COLLIDER_EXTRA_EDGE_COUNT;
             edge_index++) {
            const body_collider_extra_edge_def_t *edge =
                &body_collider_extra_edges[edge_index];
            float t;
            float margin;
            if (!state->valid[edge->start_node] ||
                !state->valid[edge->end_node]) continue;
            margin = physx_public_point_tapered_capsule_margin(
                point_view, state->view_position[edge->start_node],
                state->view_position[edge->end_node],
                physx_public_visual_radius(person_cfg, edge->start_node),
                physx_public_visual_radius(person_cfg, edge->end_node), &t);
            physx_public_consider(margin, person_index, 2,
                                  edge->start_node, edge->end_node, t,
                                  &best_margin, &best);
        }

        for (edge_index = 0; edge_index < 4; edge_index++) {
            int start_node = limb_edges[edge_index][0];
            int end_node = limb_edges[edge_index][1];
            float t;
            float margin;
            if (!state->valid[start_node] || !state->valid[end_node]) {
                continue;
            }
            margin = physx_public_point_tapered_capsule_margin(
                point_view, state->view_position[start_node],
                state->view_position[end_node],
                physx_public_visual_radius(person_cfg, start_node),
                physx_public_visual_radius(person_cfg, end_node), &t);
            physx_public_consider(margin, person_index, 2,
                                  start_node, end_node, t,
                                  &best_margin, &best);
        }

        if (state->valid[BODY_COLLIDER_TESTICLES_01] &&
            state->valid[BODY_COLLIDER_TESTICLES_02]) {
            float t;
            float margin = physx_public_point_tapered_capsule_margin(
                point_view,
                state->view_position[BODY_COLLIDER_TESTICLES_01],
                state->view_position[BODY_COLLIDER_TESTICLES_02],
                physx_public_visual_radius(
                    person_cfg, BODY_COLLIDER_TESTICLES_01),
                physx_public_visual_radius(
                    person_cfg, BODY_COLLIDER_TESTICLES_02), &t);
            physx_public_consider(
                margin, person_index, 2, BODY_COLLIDER_TESTICLES_01,
                BODY_COLLIDER_TESTICLES_02, t, &best_margin, &best);
        }

        if (person_cfg->penis_collision_enabled &&
            state->chain_points_ready && state->basis_valid) {
            float chain_view[4][3];
            int chain_ready = 1;
            int chain_index;
            for (chain_index = 0; chain_index < 4; chain_index++) {
                if (!state->chain_point_valid[chain_index] ||
                    !body_chain_collider_local_to_view(
                        state, state->chain_local_point[chain_index],
                        chain_view[chain_index])) {
                    chain_ready = 0;
                    break;
                }
            }
            if (chain_ready) {
                for (chain_index = 0; chain_index < 3; chain_index++) {
                    float t;
                    float margin =
                        physx_public_point_tapered_capsule_margin(
                            point_view, chain_view[chain_index],
                            chain_view[chain_index + 1],
                            person_cfg->chain_radius,
                            person_cfg->chain_radius, &t);
                    physx_public_consider(
                        margin, person_index, 3, chain_index,
                        chain_index + 1, t, &best_margin, &best);
                }
            }
        }
    }

    if (!ready_person_count) return -1;
    if (best.person_index && best_margin <= tolerance) {
        memcpy(hit, &best, sizeof(best));
        return 1;
    }
    memset(hit, 0, sizeof(*hit));
    hit->size = sizeof(*hit);
    hit->version = NC_TK17_PHYSX_BODY_QUERY_API_V1;
    hit->node_start = -1;
    hit->node_end = -1;
    hit->signed_distance = best_margin;
    return 0;
}
