#ifndef PHYSX_FAULT_TRACE_H
#define PHYSX_FAULT_TRACE_H

/* Diagnostic only: observe first-chance faults and ALWAYS continue the normal
   exception search. Use a separate preopened file, never the normal log lock. */
static HANDLE physx_fault_file = INVALID_HANDLE_VALUE;
static PVOID physx_fault_handler;
static volatile LONG physx_fault_busy;
static __thread const char *physx_fault_stage;
static __thread const void *physx_fault_raw;
static __thread const void *physx_fault_object;

static void physx_fault_write(const char *s)
{
    DWORD written;
    WriteFile(physx_fault_file,s,(DWORD)strlen(s),&written,NULL);
}

static void physx_fault_address(const char *label,ULONG_PTR address)
{
    MEMORY_BASIC_INFORMATION mbi;
    char module[MAX_PATH]={0},line[MAX_PATH+128];
    ULONG_PTR base=0;
    if(VirtualQuery((void*)address,&mbi,sizeof(mbi)) && mbi.Type==MEM_IMAGE) {
        base=(ULONG_PTR)mbi.AllocationBase;
        GetModuleFileNameA((HMODULE)base,module,sizeof(module));
    }
    _snprintf(line,sizeof(line),"%s=%08lx module=%s base=%08lx rva=%08lx\r\n",
        label,(unsigned long)address,module,(unsigned long)base,
        (unsigned long)(base?address-base:0));
    line[sizeof(line)-1]=0;physx_fault_write(line);
}

static LONG CALLBACK physx_fault_observe(EXCEPTION_POINTERS *info)
{
    EXCEPTION_RECORD *e;
    CONTEXT *c;
    char line[512];DWORD stack[48];SIZE_T got=0;unsigned int i;
    if(!PHYSX_FAULT_TRACE_ENABLED || physx_fault_file==INVALID_HANDLE_VALUE ||
       !info || !info->ExceptionRecord || !info->ContextRecord) return EXCEPTION_CONTINUE_SEARCH;
    e=info->ExceptionRecord;c=info->ContextRecord;
    if(e->ExceptionCode!=EXCEPTION_ACCESS_VIOLATION &&
       e->ExceptionCode!=EXCEPTION_IN_PAGE_ERROR &&
       e->ExceptionCode!=EXCEPTION_ILLEGAL_INSTRUCTION &&
       e->ExceptionCode!=EXCEPTION_INT_DIVIDE_BY_ZERO) return EXCEPTION_CONTINUE_SEARCH;
    if(InterlockedCompareExchange(&physx_fault_busy,1,0)) return EXCEPTION_CONTINUE_SEARCH;
    _snprintf(line,sizeof(line),"FIRST-CHANCE exception=%08lx tick=%lu thread=%lu stage=%s raw=%p object=%p\r\n",
        (unsigned long)e->ExceptionCode,(unsigned long)GetTickCount(),
        (unsigned long)GetCurrentThreadId(),physx_fault_stage?physx_fault_stage:"outside-addon-pivot",
        physx_fault_raw,physx_fault_object);
    line[sizeof(line)-1]=0;physx_fault_write(line);
    physx_fault_address("fault_pc",(ULONG_PTR)e->ExceptionAddress);
    for(i=0;i<e->NumberParameters && i<EXCEPTION_MAXIMUM_PARAMETERS;i++) {
        _snprintf(line,sizeof(line),"exception_parameter[%u]=%08lx\r\n",i,(unsigned long)e->ExceptionInformation[i]);
        physx_fault_write(line);
    }
    _snprintf(line,sizeof(line),"eax=%08lx ebx=%08lx ecx=%08lx edx=%08lx esi=%08lx edi=%08lx ebp=%08lx esp=%08lx eip=%08lx\r\n",
        c->Eax,c->Ebx,c->Ecx,c->Edx,c->Esi,c->Edi,c->Ebp,c->Esp,c->Eip);
    physx_fault_write(line);
    ReadProcessMemory(GetCurrentProcess(),(void*)(ULONG_PTR)c->Esp,stack,sizeof(stack),&got);
    for(i=0;i<got/sizeof(DWORD);i++) {
        char label[32];_snprintf(label,sizeof(label),"stack_word[%u]",i);
        physx_fault_address(label,stack[i]);
    }
    physx_fault_write("END FIRST-CHANCE (may be handled by the game; this observer does not recover faults)\r\n");
    FlushFileBuffers(physx_fault_file);
    InterlockedExchange(&physx_fault_busy,0);
    return EXCEPTION_CONTINUE_SEARCH;
}

static void physx_fault_trace_start(const char *path)
{
    physx_fault_file=CreateFileA(path,GENERIC_WRITE,FILE_SHARE_READ,NULL,CREATE_ALWAYS,FILE_ATTRIBUTE_NORMAL,NULL);
    if(physx_fault_file!=INVALID_HANDLE_VALUE)
        physx_fault_handler=AddVectoredExceptionHandler(1,physx_fault_observe);
}

static void physx_fault_trace_stop(void)
{
    if(physx_fault_handler) RemoveVectoredExceptionHandler(physx_fault_handler);
    physx_fault_handler=NULL;
    if(physx_fault_file!=INVALID_HANDLE_VALUE) CloseHandle(physx_fault_file);
    physx_fault_file=INVALID_HANDLE_VALUE;
}
#endif
