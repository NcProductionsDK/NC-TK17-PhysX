static int physx_d3d8_hook5_active(void)
{
    static int checked;
    static int active;
    static char module_path[MAX_PATH * 2];
    HMODULE d3d8;

    if (checked) return active;
    checked = 1;
    module_path[0] = 0;
    d3d8 = GetModuleHandleA("d3d8.dll");
    if (d3d8) {
        GetModuleFileNameA(d3d8, module_path, sizeof(module_path));
        active =
            contains_i(module_path, "\\Binaries\\d3d8.dll") ||
            contains_i(module_path, "/Binaries/d3d8.dll");
    }
    log_line("graphics d3d8 runtime module=\"%s\" hook5_active=%d note=\"Hook5 is detected by the loaded d3d8.dll path; collider visuals stay in PhysX and do not patch Hook5 code\"",
             module_path[0] ? module_path : "(not-loaded)", active);
    return active;
}

static HWND physx_d3d8_render_hwnd;
static volatile LONG physx_render_frame_tick_done;
static volatile LONG physx_render_frame_late_ownership_done;

static void physx_tick_once_for_render_frame(void)
{
    static int optimization_logged;
    if (InterlockedCompareExchange(&physx_render_frame_tick_done, 1, 0) == 0) {
        if (!optimization_logged) {
            optimization_logged = 1;
            log_line("physics performance path active full_tick=once-per-presented-frame body_refresh=once-per-person-per-tick addon_body_nodes=cached-per-person-pair-per-tick sidecar_collision_default=opt-in");
        }
        physx_tick();
    }
}

static void physx_late_ownership_once_for_render_frame(void)
{
    if (InterlockedCompareExchange(
            &physx_render_frame_late_ownership_done, 1, 0) == 0) {
        physx_late_frame_ownership_tick();
    }
}

static void draw_body_chain_colliders_d3d8_hook5_overlay(IDirect3DDevice8 *self)
{
    static int overlay_active;
    static int overlay_logged;
    static int overlay_failed_logged;
    HRESULT hr;

    if (!self || overlay_active) return;
    if (!((body_chain_collider_cfg.enabled &&
          body_chain_collider_cfg.debug_draw) ||
          addon_sidecar_collision_debug_any() ||
          room_collision_debug_any())) {
        return;
    }
    if (!real_d3d8_BeginScene || !real_d3d8_EndScene) return;

    overlay_active = 1;
    hr = real_d3d8_BeginScene(self);
    if (SUCCEEDED(hr)) {
        draw_body_chain_colliders_d3d8(self);
        real_d3d8_EndScene(self);
        if (!overlay_logged) {
            overlay_logged = 1;
            log_line("body-chain-colliders draw d3d8 hook5-overlay active device=%p note=\"opened a tiny post-Hook5 overlay scene so collider visuals are presented after Hook5 EndScene compositing\"",
                     self);
        }
    } else if (!overlay_failed_logged) {
        overlay_failed_logged = 1;
        log_line("body-chain-colliders draw d3d8 hook5-overlay skipped device=%p hr=0x%08lx note=\"BeginScene failed after Hook5 EndScene; falling back to normal EndScene timing\"",
                 self, (DWORD)hr);
    }
    overlay_active = 0;
}

static HRESULT WINAPI hook_d3d8_Present(IDirect3DDevice8 *self, const RECT *src_rect, const RECT *dst_rect,
                                        HWND dst_window_override, const RGNDATA *dirty_region)
{
    HRESULT hr;
    int hook5_active = physx_d3d8_hook5_active();
    HWND overlay_hwnd = dst_window_override ? dst_window_override :
        physx_d3d8_render_hwnd;
    physx_tick_once_for_render_frame();
    physx_late_ownership_once_for_render_frame();
    hr = real_d3d8_Present ? real_d3d8_Present(self, src_rect, dst_rect, dst_window_override, dirty_region) : D3DERR_INVALIDCALL;
    if (hook5_active && SUCCEEDED(hr)) {
        draw_body_chain_colliders_layered_hook5_d3d8(self, overlay_hwnd);
    }
    InterlockedExchange(&physx_render_frame_tick_done, 0);
    InterlockedExchange(&physx_render_frame_late_ownership_done, 0);
    ptr_readable_cache_advance_frame();
    return hr;
}

static HRESULT WINAPI hook_d3d8_SetTransform(IDirect3DDevice8 *self,
                                             D3DTRANSFORMSTATETYPE state,
                                             const D3DMATRIX *matrix)
{
    (void)state;
    return real_d3d8_SetTransform ?
        real_d3d8_SetTransform(self, state, matrix) :
        D3DERR_INVALIDCALL;
}

static HRESULT WINAPI hook_d3d8_EndScene(IDirect3DDevice8 *self)
{
    HRESULT hr;
    int hook5_active = physx_d3d8_hook5_active();
    physx_tick_once_for_render_frame();
    physx_late_ownership_once_for_render_frame();
    draw_body_chain_colliders_d3d8(self);
    hr = real_d3d8_EndScene ? real_d3d8_EndScene(self) : D3DERR_INVALIDCALL;
    if (hook5_active) {
        static int hook5_overlay_disabled_logged;
        if (!hook5_overlay_disabled_logged) {
            hook5_overlay_disabled_logged = 1;
            log_line("body-chain-colliders draw d3d8 hook5-overlay disabled note=\"post-EndScene overlay can black-screen Hook5; using normal in-scene timing while keeping Hook5 detection and direct D3D8 hook\""); 
        }
    }
    return hr;
}

static BOOL WINAPI hook_SwapBuffers(HDC hdc)
{
    BOOL ok;
    physx_tick_once_for_render_frame();
    physx_late_ownership_once_for_render_frame();
    draw_body_chain_colliders_opengl();
    ok = real_SwapBuffers ? real_SwapBuffers(hdc) : FALSE;
    InterlockedExchange(&physx_render_frame_tick_done, 0);
    InterlockedExchange(&physx_render_frame_late_ownership_done, 0);
    ptr_readable_cache_advance_frame();
    return ok;
}

static HANDLE WINAPI hook_CreateFileA(LPCSTR file_name,
                                      DWORD desired_access,
                                      DWORD share_mode,
                                      LPSECURITY_ATTRIBUTES security_attributes,
                                      DWORD creation_disposition,
                                      DWORD flags_and_attributes,
                                      HANDLE template_file)
{
    int body_marker = body_profile_path_has_body_select_marker_a(file_name);
    HANDLE h = real_CreateFileA ?
        real_CreateFileA(file_name, desired_access, share_mode,
                         security_attributes, creation_disposition,
                         flags_and_attributes, template_file) :
        CreateFileA(file_name, desired_access, share_mode,
                    security_attributes, creation_disposition,
                    flags_and_attributes, template_file);
    if (body_marker) {
        body_profile_note_body_file_a(file_name);
    } else if (h != INVALID_HANDLE_VALUE) {
        body_profile_note_body_file_a(file_name);
        physx_note_room_scene_file_a(file_name);
        physx_note_addon_scene_file_a(file_name);
        physx_note_activemod_file_a(file_name);
    }
    return h;
}

static HANDLE WINAPI hook_CreateFileW(LPCWSTR file_name,
                                      DWORD desired_access,
                                      DWORD share_mode,
                                      LPSECURITY_ATTRIBUTES security_attributes,
                                      DWORD creation_disposition,
                                      DWORD flags_and_attributes,
                                      HANDLE template_file)
{
    int body_marker = body_profile_path_has_body_select_marker_w(file_name);
    HANDLE h = real_CreateFileW ?
        real_CreateFileW(file_name, desired_access, share_mode,
                         security_attributes, creation_disposition,
                         flags_and_attributes, template_file) :
        CreateFileW(file_name, desired_access, share_mode,
                    security_attributes, creation_disposition,
                    flags_and_attributes, template_file);
    if (body_marker) {
        body_profile_note_body_file_w(file_name);
    } else if (h != INVALID_HANDLE_VALUE) {
        body_profile_note_body_file_w(file_name);
        physx_note_room_scene_file_w(file_name);
        physx_note_addon_scene_file_w(file_name);
        physx_note_activemod_file_w(file_name);
    }
    return h;
}

static HRESULT WINAPI hook_d3d8_CreateDevice(IDirect3D8 *self, UINT adapter, D3DDEVTYPE device_type,
                                             HWND focus_window, DWORD behavior_flags,
                                             D3DPRESENT_PARAMETERS *presentation_parameters,
                                             IDirect3DDevice8 **returned_device)
{
    HRESULT hr = real_d3d8_CreateDevice ? real_d3d8_CreateDevice(self, adapter, device_type, focus_window,
                                                                 behavior_flags, presentation_parameters,
                                                                 returned_device) : D3DERR_INVALIDCALL;
    if (SUCCEEDED(hr) && returned_device && *returned_device) {
        if (focus_window) {
            physx_d3d8_render_hwnd = focus_window;
        } else if (presentation_parameters &&
                   presentation_parameters->hDeviceWindow) {
            physx_d3d8_render_hwnd = presentation_parameters->hDeviceWindow;
        }
        log_line("D3D8 CreateDevice device=%p hwnd=%p", *returned_device,
                 physx_d3d8_render_hwnd);
        patch_d3d8_device(*returned_device);
    }
    return hr;
}

static IDirect3D8 *WINAPI hook_Direct3DCreate8(UINT sdk_version)
{
    IDirect3D8 *d3d = real_Direct3DCreate8 ? real_Direct3DCreate8(sdk_version) : NULL;
    if (d3d) {
        log_line("Direct3DCreate8 object=%p", d3d);
        patch_d3d8_object(d3d);
    }
    return d3d;
}

static FARPROC WINAPI hook_GetProcAddress(HMODULE mod, LPCSTR name)
{
    FARPROC ret = GetProcAddress(mod, name);
    char modname[MAX_PATH * 2];
    if (!mod || !name) return ret;
    modname[0] = 0;
    GetModuleFileNameA(mod, modname, sizeof(modname));
    if (contains_i(modname, "d3d8.dll") && strcmp(name, "Direct3DCreate8") == 0) {
        if (!real_Direct3DCreate8 && ret && ret != (FARPROC)hook_Direct3DCreate8) {
            real_Direct3DCreate8 = (Direct3DCreate8_t)ret;
        }
        return (FARPROC)hook_Direct3DCreate8;
    }
    if (contains_i(modname, "gdi32.dll") && strcmp(name, "SwapBuffers") == 0) {
        if (!real_SwapBuffers && ret && ret != (FARPROC)hook_SwapBuffers) {
            real_SwapBuffers = (SwapBuffers_t)ret;
        }
        return (FARPROC)hook_SwapBuffers;
    }
    return ret;
}

static void __cdecl hook_SetTSNodeName(void *object, const void *name_ref)
{
    const char *name = stringref_cstr_a(name_ref);
    static int body_interest_log_count;
    static int addon_interest_log_count;
    DWORD now = GetTickCount();
    int addon_root_event;
    if (defaults_cfg.debug && name && tsnode_probe_count < 160) {
        tsnode_probe_count++;
        log_line("tsnode probe object=%p name=\"%s\" ref=%p index=%d", object, name, name_ref, tsnode_probe_count);
    }
    remember_named_node(name, object);
    physx_note_room_object_name_a(name);
    body_profile_note_tsnode_name_a(name);
    addon_root_event = addon_sidecar_note_live_root_name(name, object, now);
    if (defaults_cfg.debug && addon_root_event) {
        log_line("addon tsnode live root observed root=\"%s\" object=%p note=\"updated uniform PrimaryZone ownership and retired only add-ons that TK17 treats as replacements in the same equipment slot\"",
                 name, object);
    }
    {
        body_chain_probe_t *probe = find_body_chain_probe(name);
        if (!probe) probe = find_body_chain_probe_containing(name);
        if (probe) {
            probe->hook_object = object;
            if (!probe->seen_name[0]) lstrcpynA(probe->seen_name, name, sizeof(probe->seen_name));
        }
        if (defaults_cfg.debug && name && body_chain_name_interesting(name) && body_interest_log_count < 120) {
            body_interest_log_count++;
            log_line("body-chain tsnode interest object=%p name=\"%s\" matched=\"%s\" ref=%p index=%d",
                     object, name, probe ? probe->name : "", name_ref, body_interest_log_count);
        }
        if ((defaults_cfg.debug || addon_physics_probe_enabled) &&
            name && addon_chain_name_interesting(name) &&
            addon_interest_log_count < 240) {
            addon_interest_log_count++;
            log_line("addon tsnode interest object=%p name=\"%s\" ref=%p index=%d named_nodes=%d",
                     object, name, name_ref, addon_interest_log_count, named_node_count);
        }
        if (defaults_cfg.debug && name &&
            addon_tsnode_window_until_tick &&
            now <= addon_tsnode_window_until_tick &&
            addon_tsnode_window_count < 160 &&
            !contains_i(name, "Texture") &&
            !contains_i(name, "Shader") &&
            !contains_i(name, "Phong") &&
            !contains_i(name, "GUI/") &&
            !contains_i(name, "_Icon")) {
            addon_tsnode_window_count++;
            log_line("addon tsnode window name root=\"%s\" object=%p name=\"%s\" ref=%p index=%d named_nodes=%d",
                     addon_tsnode_window_root, object, name, name_ref,
                     addon_tsnode_window_count, named_node_count);
        } else if (addon_tsnode_window_until_tick &&
                   now > addon_tsnode_window_until_tick) {
            addon_tsnode_window_until_tick = 0;
        }
    }
    if (real_SetTSNodeName) real_SetTSNodeName(object, name_ref);
}

static void patch_d3d8_object(IDirect3D8 *d3d)
{
    patch_vtable_slot(d3d, 15, hook_d3d8_CreateDevice, (void**)&real_d3d8_CreateDevice);
}

static void capture_d3d8_device_slot(void *obj, int index, void **real)
{
    void **vt;
    if (!obj || !real || *real) return;
    vt = *(void***)obj;
    if (!vt || !ptr_readable(vt, sizeof(void*) * (index + 1))) return;
    *real = vt[index];
}

static void patch_d3d8_device(IDirect3DDevice8 *dev)
{
    patch_vtable_slot(dev, 15, hook_d3d8_Present, (void**)&real_d3d8_Present);
    capture_d3d8_device_slot(dev, 34, (void**)&real_d3d8_BeginScene);
    patch_vtable_slot(dev, 35, hook_d3d8_EndScene, (void**)&real_d3d8_EndScene);
    patch_vtable_slot(dev, 37, hook_d3d8_SetTransform, (void**)&real_d3d8_SetTransform);
}

static void patch_iat(HMODULE mod, const char *dll, const char *name, void *hook, void **real)
{
    BYTE *base = (BYTE*)mod;
    IMAGE_DOS_HEADER *dos = (IMAGE_DOS_HEADER*)base;
    IMAGE_NT_HEADERS *nt;
    IMAGE_IMPORT_DESCRIPTOR *imp;
    DWORD old;
    if (!base || dos->e_magic != IMAGE_DOS_SIGNATURE) return;
    nt = (IMAGE_NT_HEADERS*)(base + dos->e_lfanew);
    if (nt->Signature != IMAGE_NT_SIGNATURE) return;
    if (!nt->OptionalHeader.DataDirectory[IMAGE_DIRECTORY_ENTRY_IMPORT].VirtualAddress) return;
    imp = (IMAGE_IMPORT_DESCRIPTOR*)(base + nt->OptionalHeader.DataDirectory[IMAGE_DIRECTORY_ENTRY_IMPORT].VirtualAddress);
    for (; imp->Name; imp++) {
        const char *dllname = (const char*)(base + imp->Name);
        IMAGE_THUNK_DATA *orig = imp->OriginalFirstThunk ? (IMAGE_THUNK_DATA*)(base + imp->OriginalFirstThunk) : NULL;
        IMAGE_THUNK_DATA *thunk = (IMAGE_THUNK_DATA*)(base + imp->FirstThunk);
        if (_stricmp(dllname, dll) != 0) continue;
        if (!orig) orig = thunk;
        for (; orig->u1.AddressOfData; orig++, thunk++) {
            IMAGE_IMPORT_BY_NAME *byn;
            if (orig->u1.Ordinal & IMAGE_ORDINAL_FLAG) continue;
            byn = (IMAGE_IMPORT_BY_NAME*)(base + orig->u1.AddressOfData);
            if (strcmp((char*)byn->Name, name) != 0) continue;
            if (real && !*real) *real = (void*)thunk->u1.Function;
            if ((void*)thunk->u1.Function == hook) continue;
            if (VirtualProtect(&thunk->u1.Function, sizeof(void*), PAGE_READWRITE, &old)) {
                thunk->u1.Function = (DWORD_PTR)hook;
                VirtualProtect(&thunk->u1.Function, sizeof(void*), old, &old);
            }
        }
    }
}

static int physx_module_name_contains(HMODULE mod, const char *needle)
{
    char path[MAX_PATH * 2];
    path[0] = 0;
    if (!mod || !needle) return 0;
    GetModuleFileNameA(mod, path, sizeof(path));
    return contains_i(path, needle);
}

static void patch_module(HMODULE mod)
{
    int is_sys;
    int is_app;
    int is_ima;
    int is_dx8;
    int is_dx3;
    if (!mod) return;
    is_sys = physx_module_name_contains(mod, "ThriXXX010278-SYS.dll");
    is_app = physx_module_name_contains(mod, "ThriXXX010278-APP.dll");
    is_ima = physx_module_name_contains(mod, "ThriXXX010278-IMA.dll");
    is_dx8 = physx_module_name_contains(mod, "ThriXXX010278-DX8.dll");
    is_dx3 = physx_module_name_contains(mod, "ThriXXX010278-DX3.dll");
    if (is_sys || is_dx8 || is_dx3) {
        patch_iat(mod, "KERNEL32.dll", "GetProcAddress",
                  hook_GetProcAddress, NULL);
        patch_iat(mod, "D3D8.dll", "Direct3DCreate8",
                  hook_Direct3DCreate8, (void**)&real_Direct3DCreate8);
    }
    if (is_sys || is_app || is_ima || is_dx8 || is_dx3) {
        patch_iat(mod, "KERNEL32.dll", "CreateFileA",
                  hook_CreateFileA, (void**)&real_CreateFileA);
        patch_iat(mod, "KERNEL32.dll", "CreateFileW",
                  hook_CreateFileW, (void**)&real_CreateFileW);
    }
    if (is_sys) {
        patch_iat(mod, "GDI32.dll", "SwapBuffers",
                  hook_SwapBuffers, (void**)&real_SwapBuffers);
    }
    if (is_app || is_ima) {
        patch_iat(mod, "ThriXXX010278-SYS.dll",
                  "?SetTSNodeName@Bionic@@YAXPAVScriptObject@1@ABVStringRef@1@@Z",
                  hook_SetTSNodeName, (void**)&real_SetTSNodeName);
    }
}

static void patch_all_modules(void)
{
    HANDLE snap;
    MODULEENTRY32 me;
    DWORD pid = GetCurrentProcessId();
    snap = CreateToolhelp32Snapshot(TH32CS_SNAPMODULE, pid);
    if (snap == INVALID_HANDLE_VALUE) return;
    memset(&me, 0, sizeof(me));
    me.dwSize = sizeof(me);
    if (Module32First(snap, &me)) {
        do {
            patch_module(me.hModule);
        } while (Module32Next(snap, &me));
    }
    CloseHandle(snap);
}

