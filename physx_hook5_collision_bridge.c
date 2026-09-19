/* Included after the shared collider projection/drawing helpers. */
typedef int (__cdecl *physx_debug_register_t)(void *);
typedef int (__cdecl *physx_debug_projection_t)(unsigned, float *);
static physx_debug_register_t physx_debug_register;
static physx_debug_projection_t physx_debug_projection;
static int physx_debug_registered;

static int physx_collision_debug_enabled(void)
{
    return body_collider_debug_any() ||
        addon_sidecar_collision_debug_any() || room_collision_debug_any();
}

static void __cdecl physx_hook5_collision_composite(ID3D11DeviceContext *context,
    ID3D11RenderTargetView *target, ID3D11DepthStencilView *depth,
    unsigned width, unsigned height)
{
    if (physx_shutting_down) return;
    ID3D11Device *device = NULL;
    static int success_logged, failure_logged;
    static DWORD failed_tick;
    static int retry_pending;
    (void)depth; /* Debug wireframes remain visible through scene geometry. */
    if (!physx_collision_debug_enabled() || !context || !target ||
        !physx_debug_projection || !width || !height) return;
    if (retry_pending && GetTickCount() - failed_tick < 5000) return;
    /* ABI version 1 returns Hook5's scene projection, including its FOV setting;
       the D3D8 projection can already have been replaced by the GUI matrix. */
    if (!physx_debug_projection(1, (float *)&body_chain_hook5_projection)) return;
    ID3D11DeviceContext_GetDevice(context, &device);
    if (!device) return;
    if (!physx_debug_surface_prepare(&body_chain_hook5_surface, device, width, height)) {
        failed_tick = GetTickCount();
        retry_pending = 1;
        if (!failure_logged) {
            failure_logged = 1;
            log_line("collision debug scene surface unavailable; skipping Hook5 debug draw");
        }
        ID3D11Device_Release(device);
        return;
    }
    retry_pending = 0;
    ID3D11Device_Release(device);
    physx_wire_collect(&body_chain_hook5_surface.lines);
    if (body_chain_hook5_surface.lines.count &&
        physx_debug_surface_draw(&body_chain_hook5_surface, context, target) &&
        !success_logged) {
        success_logged = 1;
        log_line("collision debug draw active renderer=hook5-batched-wireframe before_gui=1 vertices=%u upload_bytes=%u viewport=%ux%u",
            body_chain_hook5_surface.lines.count,
            body_chain_hook5_surface.lines.count * (unsigned)sizeof(physx_wire_vertex),width,height);
    }
    if (body_chain_hook5_surface.lines.dropped && !failure_logged) {
        failure_logged=1;
        log_line("collision wireframe batch limit reached; omitted_segments=%u",body_chain_hook5_surface.lines.dropped);
    }
}

static void physx_hook5_collision_register(void)
{
    static DWORD retry_tick;
    static int attempted, missing_logged;
    int enabled = physx_collision_debug_enabled();
    HMODULE extended;
    if (!enabled && !physx_debug_registered) return;
    if (!physx_debug_register) {
        DWORD now = GetTickCount();
        if (attempted && now - retry_tick < 1000) return;
        attempted = 1; retry_tick = now;
        extended = GetModuleHandleA("NC-TK17-Hook5-Extended.dll");
        if (extended) {
            physx_debug_register = (physx_debug_register_t)GetProcAddress(extended,
                "nc_hook5_extended_register_debug_composite_callback");
            physx_debug_projection = (physx_debug_projection_t)GetProcAddress(extended,
                "nc_hook5_extended_get_projection_matrix");
        }
        if (!physx_debug_register || !physx_debug_projection) {
            physx_debug_register = NULL;
            if (!missing_logged) {
                missing_logged = 1;
                log_line("collision debug draw needs updated NC-TK17-Hook5-Extended scene bridge; no desktop overlay fallback");
            }
            return;
        }
    }
    if (enabled != physx_debug_registered &&
        physx_debug_register(enabled ? (void *)physx_hook5_collision_composite : NULL)) {
        physx_debug_registered = enabled;
        log_line("collision debug Hook5 scene callback enabled=%d", enabled);
        if (!enabled) physx_debug_surface_destroy(&body_chain_hook5_surface);
    }
}

static void physx_hook5_collision_shutdown(void)
{
    if (physx_debug_registered && physx_debug_register &&
        GetModuleHandleA("NC-TK17-Hook5-Extended.dll"))
        physx_debug_register(NULL);
    physx_debug_registered = 0;
    physx_debug_register = NULL;
    physx_debug_projection = NULL;
    physx_debug_surface_destroy(&body_chain_hook5_surface);
}
