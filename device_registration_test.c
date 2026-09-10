#include "NC-TK17-PhysX.c"
#define CHECK(c) do { if (!(c)) { fprintf(stderr, "FAIL %d: %s\n", __LINE__, #c); exit(1); } } while (0)
static HRESULT WINAPI original_present(IDirect3DDevice8 *d, const RECT *a, const RECT *b, HWND w, const RGNDATA *r)
{ (void)d;(void)a;(void)b;(void)w;(void)r; return S_OK; }
static HRESULT WINAPI original_end(IDirect3DDevice8 *d) { (void)d; return S_OK; }
static HRESULT WINAPI creation(IDirect3DDevice8 *d, D3DDEVICE_CREATION_PARAMETERS *p)
{ (void)d; memset(p,0,sizeof(*p)); p->hFocusWindow=(HWND)123; return S_OK; }
int main(void)
{
    IDirect3DDevice8Vtbl table = {0};
    IDirect3DDevice8 device = {&table};
    table.Present = original_present; table.EndScene = original_end;
    table.BeginScene = original_end; table.GetCreationParameters = creation;
    CHECK(!NCTK17PhysX_RegisterD3D8DeviceV1(NULL));
    CHECK(NCTK17PhysX_RegisterD3D8DeviceV1(&device));
    CHECK(table.Present == hook_d3d8_Present && table.EndScene == hook_d3d8_EndScene);
    CHECK(real_d3d8_Present == original_present && real_d3d8_EndScene == original_end);
    CHECK(physx_d3d8_render_hwnd == (HWND)123);
    table.Present = original_present; /* Simulate a later plugin wrapping us. */
    CHECK(NCTK17PhysX_RegisterD3D8DeviceV1(&device));
    CHECK(table.Present == original_present);
    physx_simulation_serial = 1; captured_camera_inverse_valid = 1;
    body_chain_collider_global_cfg.enabled = 1;
    body_chain_collider_states[1].ready = body_chain_collider_states[1].basis_valid = 1;
    CHECK(NCTK17PhysX_BodyColliderStatusV1() == 0x207);
    puts("PASS: late device registration installs PhysX callbacks, preserves predecessors and later wrappers, exposes readiness");
    return 0;
}
