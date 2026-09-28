#ifndef NC_PHYSX_MODULE_LIFETIME_H
#define NC_PHYSX_MODULE_LIFETIME_H
#include <windows.h>

/* Engine inline hooks, IAT replacements and cached public callbacks can outlive
   a caller's LoadLibrary reference. They cannot be torn down safely by DllMain
   while another thread is executing them. Pin before publishing any hooks.
   Windows still releases the module at process exit; no per-frame work. */
static int physx_pin_hook_module(HMODULE module)
{
    HMODULE held=NULL;
    return module && GetModuleHandleExA(
        GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS|GET_MODULE_HANDLE_EX_FLAG_PIN,
        (LPCSTR)module,&held) && held==module;
}
#endif
