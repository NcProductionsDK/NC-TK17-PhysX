#include "NC-TK17-PhysX.c"
#include <assert.h>

static BYTE editor[0x400], pose[POSEEDIT_TRACKS_OFFSET + 2*POSEEDIT_TRACK_SIZE];
static int presents, swaps;
static HRESULT present_result=S_OK, device_status=S_OK;
static BOOL swap_result=TRUE;
static const RECT source={1,2,3,4}, destination={5,6,7,8};
static HRESULT WINAPI fake_present(IDirect3DDevice8 *device, const RECT *src,
    const RECT *dst, HWND window, const RGNDATA *dirty)
{
    assert(device && src==&source && dst==&destination && window==(HWND)123 && !dirty);
    presents++;
    return present_result;
}
static HRESULT WINAPI fake_status(IDirect3DDevice8 *device)
{
    assert(device);
    return device_status;
}
static BOOL WINAPI fake_swap(HDC dc)
{
    assert(dc);
    swaps++;
    return swap_result;
}
static DWORD THISCALL fake_queue(void *self, void *p, void *a, void *o) { return 0; }
static void THISCALL fake_reset(void *self) {}
static void prepare_frame(void)
{
    /* The solver/reset behavior has separate production integration tests.
       Exercise the real presentation wrapper and its end-of-frame bookkeeping. */
    physx_render_frame_tick_done=physx_render_frame_late_ownership_done=1;
    physx_genital_early_sample_done=1;
    physx_genital_early_attempted[0]=physx_genital_early_attempted[1]=15;
}
static void check_frame_reset(void)
{
    assert(!physx_render_frame_tick_done && !physx_render_frame_late_ownership_done);
    assert(!physx_genital_early_sample_done && !physx_genital_early_attempted[0] && !physx_genital_early_attempted[1]);
}
static HRESULT present(IDirect3DDevice8 *device)
{
    prepare_frame();
    HRESULT result=hook_d3d8_Present(device,&source,&destination,(HWND)123,NULL);
    check_frame_reset();
    return result;
}
static BOOL swap(HDC dc)
{
    prepare_frame();
    BOOL result=hook_SwapBuffers(dc);
    check_frame_reset();
    return result;
}

int main(void)
{
    captured_poseedit_this=editor;
    captured_poseedit_editpose=pose;
    *(void**)(editor+POSEEDIT_EDITPOSE_OFFSET)=pose;
    body_chain_poseeditor_mode_active=1;
    tramp_PoseEdit_QueuePose=fake_queue;
    tramp_PoseEdit_ResetPose=fake_reset;
    real_d3d8_Present=fake_present;
    real_SwapBuffers=fake_swap;
    body_chain_collider_global_cfg.debug_draw=0;
    IDirect3DDevice8Vtbl vtable={0};
    vtable.TestCooperativeLevel=fake_status;
    IDirect3DDevice8 device={&vtable}, replacement={&vtable};

    assert(present(&device)==S_OK && presents==1);
    *(int*)(editor+POSEEDIT_RESET_PENDING_OFFSET)=1;
    unsigned int serial=physx_simulation_serial;
    for(int frame=0;frame<20;frame++) assert(present(&device)==S_OK);
    assert(presents==21 && physx_simulation_serial==serial);
    assert(*(int*)(editor+POSEEDIT_RESET_PENDING_OFFSET)==1);
    *(int*)(editor+POSEEDIT_RESET_PENDING_OFFSET)=0;
    assert(present(&device)==S_OK && presents==22);

    /* A command can open a dialog before it queues an actual pose. */
    poseedit_file_command_depth=1;
    assert(physx_poseedit_transition_busy() && !physx_poseedit_native_reset_pending());
    assert(present(&device)==S_OK && presents==23);
    poseedit_file_command_depth=0;
    *(int*)(editor+POSEEDIT_RESET_PENDING_OFFSET)=1;
    body_chain_poseeditor_mode_active=0;
    assert(present(&device)==S_OK && presents==24);
    body_chain_poseeditor_mode_active=1;
    assert(present(&replacement)==S_OK && presents==25);
    assert(present(&replacement)==S_OK && presents==26);
    device_status=present_result=D3DERR_DEVICELOST;
    assert(present(&replacement)==D3DERR_DEVICELOST && presents==27);
    device_status=present_result=S_OK;
    assert(present(&replacement)==S_OK && presents==28);
    assert(present(&replacement)==S_OK && presents==29);
    tramp_PoseEdit_ResetPose=NULL;
    assert(present(&replacement)==S_OK && presents==30);
    tramp_PoseEdit_ResetPose=fake_reset;
    puts("PASS: D3D8 presents every native fade/loading/dialog frame and preserves graphics errors, mode changes and frame bookkeeping");

    assert(swap((HDC)1) && swaps==1);
    for(int frame=0;frame<5;frame++) assert(swap((HDC)1));
    assert(swaps==6);
    *(int*)(editor+POSEEDIT_RESET_PENDING_OFFSET)=0;
    assert(swap((HDC)1) && swaps==7);
    poseedit_file_command_depth=1;
    assert(swap((HDC)1) && swaps==8);
    poseedit_file_command_depth=0;
    *(int*)(editor+POSEEDIT_RESET_PENDING_OFFSET)=1;
    assert(swap((HDC)2) && swaps==9);
    body_chain_poseeditor_mode_active=0;
    swap_result=FALSE;
    assert(!swap((HDC)2) && swaps==10);
    body_chain_poseeditor_mode_active=1;
    swap_result=TRUE;
    assert(swap((HDC)2) && swaps==11);
    puts("PASS: OpenGL presents every native fade/loading/dialog frame and preserves swap failures and frame bookkeeping");
    return 0;
}
