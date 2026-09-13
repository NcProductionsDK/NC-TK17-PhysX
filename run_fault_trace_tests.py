"""Exercise the diagnostic observer with a software-raised, handled exception."""
from pathlib import Path
import os, subprocess

root=Path(__file__).resolve().parent
build=root/'build'
source=build/'fault_trace_test.c'
source.write_text(r'''
#include <windows.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
static int enabled=1,handled;
#define PHYSX_FAULT_TRACE_ENABLED enabled
#include "../physx_fault_trace.h"
static LONG CALLBACK downstream(EXCEPTION_POINTERS *p) {
    if(p->ExceptionRecord->ExceptionCode==EXCEPTION_ACCESS_VIOLATION) {
        handled++;return EXCEPTION_CONTINUE_EXECUTION;
    }
    return EXCEPTION_CONTINUE_SEARCH;
}
int main(int argc,char **argv) {
    ULONG_PTR info[2]={0,0x1234};LARGE_INTEGER size;
    assert(argc==2);
    PVOID next=AddVectoredExceptionHandler(0,downstream);assert(next);
    physx_fault_trace_start(argv[1]);assert(physx_fault_handler);
    physx_fault_stage="test-pivot";physx_fault_raw=(void*)0x5678;physx_fault_object=(void*)0x9abc;
    RaiseException(EXCEPTION_ACCESS_VIOLATION,0,2,info);
    assert(handled==1);assert(GetFileSizeEx(physx_fault_file,&size));assert(size.QuadPart>0);
    LONGLONG first=size.QuadPart;enabled=0;
    RaiseException(EXCEPTION_ACCESS_VIOLATION,0,2,info);
    assert(handled==2);assert(GetFileSizeEx(physx_fault_file,&size));assert(size.QuadPart==first);
    physx_fault_trace_stop();assert(!physx_fault_handler && physx_fault_file==INVALID_HANDLE_VALUE);
    RaiseException(EXCEPTION_ACCESS_VIOLATION,0,2,info);assert(handled==3);
    RemoveVectoredExceptionHandler(next);
    puts("PASS: records exception, passes it to downstream handler, honors debug off and removes observer cleanly");
    return 0;
}
''')
gcc=Path(r'C:\msys64\mingw32\bin\gcc.exe')
env=dict(os.environ,PATH=str(gcc.parent)+os.pathsep+os.environ['PATH'])
exe=build/'fault_trace_test.exe';log=build/'fault_trace_test.log'
subprocess.run([str(gcc),'-m32','-O2','-Wall','-Wextra','-Werror','-static-libgcc','-o',str(exe),str(source)],env=env,check=True)
subprocess.run([str(exe),str(log)],env=env,check=True,timeout=20)
text=log.read_text()
assert text.count('FIRST-CHANCE exception=')==1
for value in ('stage=test-pivot','exception_parameter[1]=00001234','fault_pc=','module=','rva=','eip=','stack_word[0]='):
    assert value in text,value
print('PASS: fault report includes stage, fault address, module/RVA, registers and raw stack words')
