
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
#define PTR_READABLE_CACHE_SLOTS 256
#define PTR_READABLE_RECENT_REGIONS 16

typedef struct ptr_readable_cache_entry_t {
    BYTE *base;
    BYTE *end;
    LONG epoch;
} ptr_readable_cache_entry_t;

static volatile LONG ptr_readable_cache_epoch = 1;
static __thread ptr_readable_cache_entry_t
    ptr_readable_cache[PTR_READABLE_CACHE_SLOTS];
static __thread unsigned int ptr_readable_cache_last_slot;
/* The page hash cannot find a previously queried region through a different
   page after another region became "last". Keep a small associative fallback
   of whole regions. Only hash misses search it; ordinary hits are unchanged.
   These positive entries have exactly the same frame epoch as the page cache. */
static __thread ptr_readable_cache_entry_t
    ptr_readable_recent_regions[PTR_READABLE_RECENT_REGIONS];
static __thread unsigned int ptr_readable_recent_next;

static int ptr_readable_find_recent(BYTE *cur, LONG epoch,
                                     ptr_readable_cache_entry_t *result)
{
    unsigned int i;
    for (i = 0; i < PTR_READABLE_RECENT_REGIONS; ++i) {
        const ptr_readable_cache_entry_t *entry = &ptr_readable_recent_regions[i];
        if (entry->epoch == epoch && cur >= entry->base && cur < entry->end) {
            *result = *entry;
            return 1;
        }
    }
    return 0;
}

static void ptr_readable_cache_advance_frame(void)
{
    LONG next = InterlockedIncrement(&ptr_readable_cache_epoch);
    if (next == 0) InterlockedIncrement(&ptr_readable_cache_epoch);
}

static int ptr_readable(const void *p, size_t bytes)
{
    MEMORY_BASIC_INFORMATION mbi;
    BYTE *cur = (BYTE*)p;
    BYTE *end;
    LONG epoch = InterlockedCompareExchange(&ptr_readable_cache_epoch, 0, 0);

    if (!p || bytes > UINTPTR_MAX - (uintptr_t)p) return 0;
    end = (BYTE*)((uintptr_t)p + bytes);
    while (cur < end) {
        /* Entries describe regions, not individual pages. Reuse the last
           entry when scanning another page of that same region, before
           consulting the page hash. It retains the same frame lifetime. */
        ptr_readable_cache_entry_t *entry =
            &ptr_readable_cache[ptr_readable_cache_last_slot];
        if (entry->epoch == epoch && cur >= entry->base && cur < entry->end) {
            cur = entry->end;
            continue;
        }
        uintptr_t page = (uintptr_t)cur >> 12;
        uintptr_t cache_hash = page ^ (page >> 8) ^ (page >> 16);
        unsigned int slot = cache_hash & (PTR_READABLE_CACHE_SLOTS - 1);
        entry = &ptr_readable_cache[slot];

        if (entry->epoch == epoch && cur >= entry->base && cur < entry->end) {
            ptr_readable_cache_last_slot = slot;
            cur = entry->end;
            continue;
        }
        if (ptr_readable_find_recent(cur, epoch, entry)) {
            ptr_readable_cache_last_slot = slot;
            cur = entry->end;
            continue;
        }
        if (!VirtualQuery(cur, &mbi, sizeof(mbi))) return 0;
        if (mbi.State != MEM_COMMIT || (mbi.Protect & (PAGE_NOACCESS | PAGE_GUARD))) return 0;
        entry->base = (BYTE*)mbi.BaseAddress;
        entry->end = entry->base + mbi.RegionSize;
        entry->epoch = epoch;
        ptr_readable_recent_regions[ptr_readable_recent_next] = *entry;
        ptr_readable_recent_next = (ptr_readable_recent_next + 1) &
                                  (PTR_READABLE_RECENT_REGIONS - 1);
        ptr_readable_cache_last_slot = slot;
        cur = entry->end;
    }
    return 1;
}


static ptr_readable_cache_entry_t reference_cache[PTR_READABLE_CACHE_SLOTS];
static unsigned reference_last;
static int reference(const void *p,size_t bytes) {
    MEMORY_BASIC_INFORMATION mbi;
    BYTE *cur=(BYTE*)p,*end=cur+bytes;
    LONG epoch=InterlockedCompareExchange(&ptr_readable_cache_epoch,0,0);
    if (!p || end<cur) return 0;
    while (cur<end) {
        ptr_readable_cache_entry_t *last=&reference_cache[reference_last];
        if (last->epoch==epoch && cur>=last->base && cur<last->end) {
            cur=last->end; continue;
        }
        uintptr_t page=(uintptr_t)cur>>12;
        uintptr_t hash=page^(page>>8)^(page>>16);
        ptr_readable_cache_entry_t *entry=&reference_cache[hash&(PTR_READABLE_CACHE_SLOTS-1)];
        if (entry->epoch==epoch && cur>=entry->base && cur<entry->end) {
            reference_last=(unsigned)(entry-reference_cache);
            cur=entry->end; continue;
        }
        if (!VirtualQuery(cur,&mbi,sizeof(mbi))) return 0;
        if (mbi.State!=MEM_COMMIT || (mbi.Protect&(PAGE_NOACCESS|PAGE_GUARD))) return 0;
        entry->base=(BYTE*)mbi.BaseAddress;
        entry->end=entry->base+mbi.RegionSize;
        entry->epoch=epoch; cur=entry->end;
        reference_last=(unsigned)(entry-reference_cache);
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
    assert(queries==1);
    puts("PASS: successive pages in one region still require one query in both versions");

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
    assert(ptr_readable(p,16)); assert(queries==2);
    assert(!ptr_readable(p+2*page,16));
    puts("PASS: conflicting hash slots reuse recent regions without losing bounds");

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
    assert(sink==1024000);

    /* Alternating allocations defeats the previous last-region shortcut.
       Different pages in the same allocation must reuse its region query. */
    BYTE *regions[32];
    for (unsigned r=0;r<32;r++) {
        regions[r]=VirtualAlloc(NULL,page*64,MEM_RESERVE|MEM_COMMIT,PAGE_READWRITE);
        assert(regions[r]);
    }
    for (unsigned count=2;count<=16;count*=8) {
        ptr_readable_cache_advance_frame(); queries=0;
        for (unsigned i=0;i<64;i++) for (unsigned r=0;r<count;r++)
            assert(ptr_readable(regions[r]+i*page,16));
        assert(queries==count);
        printf("PASS: %u alternating regions across 64 pages require %u queries\n",count,queries);
    }
    /* More regions than the recent table can retain: replacement is allowed
       to cost queries, never to make an invalid range readable. */
    for (unsigned r=0;r<32;r++)
        assert(VirtualProtect(regions[r]+page*63,page,PAGE_NOACCESS,&old));
    ptr_readable_cache_advance_frame();
    for (unsigned i=0;i<256;i++) {
        unsigned r=i%32;
        assert(ptr_readable(regions[r]+(i%62)*page,16));
        assert(!ptr_readable(regions[r]+page*63-8,16));
        assert(!ptr_readable(regions[r]+page*63,16));
    }
    puts("PASS: recent-region eviction and protected range crossings");
    for (unsigned r=0;r<32;r++)
        assert(VirtualProtect(regions[r]+page*63,page,PAGE_READWRITE,&old));
    /* Seven alternating-order trials avoid accepting a single noisy timing.
       Pattern 0 is an ordinary hot page, 1/2 revisit 2/16 regions, and 3
       is a cold single check each frame (no possible reuse). */
    for (unsigned pattern=0;pattern<4;pattern++) {
        for (unsigned trial=0;trial<7;trial++) for (unsigned turn=0;turn<2;turn++) {
            unsigned mode=(trial+turn)&1;
            unsigned checks=pattern==3?1:64;
            double start=timer(); queries=0;
            for (unsigned frame=0;frame<1000;frame++) {
                ptr_readable_cache_advance_frame();
                for (unsigned i=0;i<checks;i++) {
                    unsigned region=pattern==1?i%2:pattern==2?i%16:0;
                    unsigned offset=pattern==0?0:i;
                    BYTE *address=regions[region]+offset*page;
                    sink+=mode?ptr_readable(address,16):reference(address,16);
                }
            }
            printf("TRIAL pattern=%u trial=%u mode=%s ms=%.3f queries=%u\n",
                pattern,trial,mode?"candidate":"baseline",(timer()-start)*1000,queries);
        }
    }
    for (unsigned r=0;r<32;r++) assert(VirtualFree(regions[r],0,MEM_RELEASE));
    assert(VirtualFree(p,0,MEM_RELEASE));
    return 0;
}
