"""Exercise production region-cache lookup and frame invalidation on Windows."""
import os
from pathlib import Path
import subprocess

root = Path(__file__).resolve().parent
source = (root / 'NC-TK17-PhysX.c').read_text(encoding='utf-8')
production = source[source.index('#define PTR_READABLE_CACHE_SLOTS'):source.index('static int ptr_writable(')]
fixture = r'''
#include <windows.h>
#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#include <string.h>
static unsigned queries;
static SIZE_T counted_query(LPCVOID p, PMEMORY_BASIC_INFORMATION m, SIZE_T n) {
    queries++; return VirtualQuery(p,m,n);
}
#define VirtualQuery counted_query
'''
fixture += production
fixture += r'''
static ptr_readable_cache_entry_t reference_cache[PTR_READABLE_CACHE_SLOTS];
static int reference(const void *p,size_t bytes) {
    MEMORY_BASIC_INFORMATION mbi;
    BYTE *cur=(BYTE*)p,*end=cur+bytes;
    LONG epoch=InterlockedCompareExchange(&ptr_readable_cache_epoch,0,0);
    if (!p || end<cur) return 0;
    while (cur<end) {
        uintptr_t page=(uintptr_t)cur>>12;
        uintptr_t hash=page^(page>>8)^(page>>16);
        ptr_readable_cache_entry_t *entry=&reference_cache[hash&(PTR_READABLE_CACHE_SLOTS-1)];
        if (entry->epoch==epoch && cur>=entry->base && cur<entry->end) {
            cur=entry->end; continue;
        }
        if (!VirtualQuery(cur,&mbi,sizeof(mbi))) return 0;
        if (mbi.State!=MEM_COMMIT || (mbi.Protect&(PAGE_NOACCESS|PAGE_GUARD))) return 0;
        entry->base=(BYTE*)mbi.BaseAddress;
        entry->end=entry->base+mbi.RegionSize;
        entry->epoch=epoch; cur=entry->end;
    }
    return 1;
}
static unsigned slot_for(const void *p) {
    uintptr_t page=(uintptr_t)p>>12;
    return (page^(page>>8)^(page>>16))&(PTR_READABLE_CACHE_SLOTS-1);
}
static DWORD WINAPI thread_check(void *p) {
    unsigned before=queries;
    assert(ptr_readable(p,16));
    assert(queries==before+1); /* This thread has no cached region. */
    ptr_readable_cache_advance_frame();
    return 0;
}
static double timer(void) {
    LARGE_INTEGER t,f; QueryPerformanceCounter(&t); QueryPerformanceFrequency(&f);
    return (double)t.QuadPart/f.QuadPart;
}
int main(void) {
    SYSTEM_INFO info; GetSystemInfo(&info);
    size_t page=info.dwPageSize, pages=1024;
    BYTE *p=VirtualAlloc(NULL,page*pages,MEM_RESERVE|MEM_COMMIT,PAGE_READWRITE);
    assert(p); DWORD old;
    assert(!ptr_readable(NULL,0)); assert(ptr_readable(p,0));
    assert(!ptr_readable((void*)(uintptr_t)0xfffffff0u,64));
    ptr_readable_cache_advance_frame(); queries=0;
    for (unsigned i=0;i<64;i++) assert(ptr_readable(p+i*page,16));
    assert(queries==1);
    ptr_readable_cache_advance_frame(); queries=0;
    for (unsigned i=0;i<64;i++) assert(reference(p+i*page,16));
    assert(queries==64);
    puts("PASS: 64 successive pages in one region require 1 query instead of 64");

    /* Verify true region boundaries and invalidation after changing mappings. */
    assert(VirtualProtect(p+2*page,page,PAGE_NOACCESS,&old));
    assert(VirtualProtect(p+4*page,page,PAGE_READONLY,&old));
    assert(VirtualProtect(p+6*page,page,PAGE_READWRITE|PAGE_GUARD,&old));
    assert(VirtualFree(p+8*page,page,MEM_DECOMMIT));
    ptr_readable_cache_advance_frame();
    assert(ptr_readable(p+4*page,16));
    assert(!ptr_readable(p+2*page-8,16));
    assert(!ptr_readable(p+6*page,1));
    assert(!ptr_readable(p+8*page,1));
    MEMORY_BASIC_INFORMATION mbi;
    assert(VirtualQuery(p+6*page,&mbi,sizeof(mbi)) && (mbi.Protect&PAGE_GUARD));
    for (unsigned frame=0;frame<5;frame++) {
        ptr_readable_cache_advance_frame();
        for (unsigned i=0;i<16000;i++) {
            unsigned offset=(i*7919u)%(unsigned)(page*10);
            size_t size=(i*313u)%(page*3);
            assert(ptr_readable(p+offset,size)==reference(p+offset,size));
        }
    }
    assert(VirtualProtect(p+2*page,page,PAGE_READWRITE,&old));
    ptr_readable_cache_advance_frame(); assert(ptr_readable(p+2*page,16));
    assert(VirtualProtect(p+2*page,page,PAGE_NOACCESS,&old));
    ptr_readable_cache_advance_frame(); assert(!ptr_readable(p+2*page,16));
    puts("PASS: mixed-region ranges, guard/read-only/uncommitted memory and frame invalidation");

    /* Reusing a hash slot must not leave a copied/stale last-region entry. */
    unsigned collision=0;
    for (unsigned i=10;i<pages;i++) if (slot_for(p+i*page)==slot_for(p)) { collision=i; break; }
    assert(collision);
    ptr_readable_cache_advance_frame(); queries=0;
    assert(ptr_readable(p,16));
    assert(ptr_readable(p+collision*page,16));
    assert(ptr_readable(p,16)); assert(queries==3);
    assert(!ptr_readable(p+2*page,16));
    puts("PASS: conflicting hash slots and failed lookups preserve region bounds");

    ptr_readable_cache_advance_frame(); assert(ptr_readable(p,16));
    HANDLE thread=CreateThread(NULL,0,thread_check,p,0,NULL); assert(thread);
    assert(WaitForSingleObject(thread,INFINITE)==WAIT_OBJECT_0);
    DWORD result; assert(GetExitCodeThread(thread,&result) && result==0); CloseHandle(thread);
    unsigned before=queries; assert(ptr_readable(p,16)); assert(queries==before+1);
    puts("PASS: thread-local entries and cross-thread frame epoch invalidation");

    /* Restore a uniform region and compare both ordinary and page-striding accesses. */
    assert(VirtualAlloc(p+8*page,page,MEM_COMMIT,PAGE_READWRITE));
    assert(VirtualProtect(p,page*pages,PAGE_READWRITE,&old));
    volatile unsigned sink=0;
    for (int stride=0;stride<2;stride++) for (int mode=0;mode<2;mode++) {
        double start=timer(); queries=0;
        for (unsigned frame=0;frame<4000;frame++) {
            ptr_readable_cache_advance_frame();
            for (unsigned i=0;i<64;i++) {
                BYTE *address=p+(stride?i*page:0);
                sink+=mode?ptr_readable(address,16):reference(address,16);
            }
        }
        printf("BENCH %s %s: %.3f ms, %u queries / 256000 checks\n",
               stride?"page-stride":"same-page",mode?"fixed":"original",
               (timer()-start)*1000,queries);
    }
    assert(sink==1024000); assert(VirtualFree(p,0,MEM_RELEASE));
    return 0;
}
'''
build = root / 'build/readable-cache-tests'
build.mkdir(parents=True, exist_ok=True)
cfile, exe = build / 'readable_cache.c', build / 'readable_cache.exe'
cfile.write_text(fixture, encoding='utf-8')
env = dict(os.environ)
env['PATH'] = r'C:\msys64\mingw32\bin;' + env.get('PATH', '')
subprocess.run([r'C:\msys64\mingw32\bin\gcc.exe', '-m32', '-O2', '-Wall',
                '-Wextra', '-Werror', str(cfile), '-o', str(exe)], env=env, check=True)
subprocess.run([str(exe)], env=env, check=True)
