#include "physx_module_lifetime.h"
static HMODULE self;
static int callback(void) { return 42; }
__declspec(dllexport) int install(int pin,int (**published)(void))
{
    /* Invalid handle exercises the failure path before callback publication. */
    if(pin && !physx_pin_hook_module(pin<0 ? NULL : self)) return 0;
    *published=callback;
    return 1;
}
BOOL WINAPI DllMain(HINSTANCE instance,DWORD reason,LPVOID reserved)
{
    (void)reserved;
    if(reason==DLL_PROCESS_ATTACH) self=instance;
    return TRUE;
}
