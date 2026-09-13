#ifndef PHYSX_ADDON_PIVOT_H
#define PHYSX_ADDON_PIVOT_H

/* GetModelViewRotationPivot takes a ScriptObject, not an arbitrary resolved
   payload. Its SYS.dll entry reads [object-0x18], then metadata+0x10c and
   dispatch+0xc0 before the first call. DriverBra1's resolved payload has a
   NULL metadata pointer and faults at SYS+0x8aec7 (read address 0x10c).
   Validate that dispatch chain without calling it to identify the compatible
   representation. Preserve existing valid resolved objects; otherwise try the
   exact raw object from the same binding. Never substitute a different bone. */
static int addon_pivot_script_object_valid(void *object)
{
    BYTE *metadata,*dispatch;
    void *getter;
    if((uintptr_t)object<0x18u || is_nil_engine_object(NULL,object) ||
       !ptr_readable((BYTE*)object-0x18,sizeof(metadata))) return 0;
    memcpy(&metadata,(BYTE*)object-0x18,sizeof(metadata));
    if(!metadata || !ptr_readable(metadata+0x10c,sizeof(dispatch))) return 0;
    memcpy(&dispatch,metadata+0x10c,sizeof(dispatch));
    if(!dispatch || !ptr_readable(dispatch+0xc0,sizeof(getter))) return 0;
    memcpy(&getter,dispatch+0xc0,sizeof(getter));
    return ptr_executable(getter);
}

static void *addon_pivot_script_object(void *raw,void *resolved)
{
    if(addon_pivot_script_object_valid(resolved)) return resolved;
    if(raw!=resolved && addon_pivot_script_object_valid(raw)) return raw;
    return NULL;
}
#endif
