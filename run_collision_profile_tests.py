"""Check profile accounting and rerun collision fixtures with timers enabled.

Run run_body_contact_tests.py and run_collision_tests.py first to generate their
production-source fixtures. No test changes physics settings in the game.
"""
import os
from pathlib import Path
import subprocess
import re

root = Path(__file__).resolve().parent
build = root / 'build/collision-profile-tests'
build.mkdir(parents=True, exist_ok=True)
gcc = Path(r'C:\msys64\mingw32\bin\gcc.exe')
env = dict(os.environ, PATH=str(gcc.parent) + os.pathsep + os.environ['PATH'])

accounting = r'''
#include <assert.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
typedef unsigned long DWORD;
typedef long long LONGLONG;
typedef struct {LONGLONG QuadPart;} LARGE_INTEGER;
static LONGLONG clock_ticks=100;
static int queries,lines;
static char records[4][2048];
static int QueryPerformanceCounter(LARGE_INTEGER *p){queries++;p->QuadPart=++clock_ticks;return 1;}
static void log_line(const char *fmt,...){va_list a;va_start(a,fmt);assert(lines<4);vsnprintf(records[lines++],2048,fmt,a);va_end(a);}
#define PHYSX_COLLISION_PROFILE_ENABLED 1
#include "physx_collision_profile.h"
static int sample(int early){
 COLLISION_PROFILE_SCOPE(outer,CP_BODY_SOLVE);
 COLLISION_PROFILE_COUNT(CP_BODY_CONTACTS,3);
 if(early)return 7;
 {COLLISION_PROFILE_SCOPE(inner,CP_BODY_REFINE);clock_ticks+=5;}
 COLLISION_PROFILE_END(outer);
 clock_ticks+=100;
 return 9;
}
int main(void){
 collision_profile_clear();
 assert(sample(0)==9&&sample(1)==7&&queries==0);
 assert(collision_profile_state.calls[CP_BODY_SOLVE]==0);
 collision_profile_state.enabled=1;
 assert(sample(0)==9&&sample(1)==7);
 assert(queries==6);
 assert(collision_profile_state.calls[CP_BODY_SOLVE]==2);
 assert(collision_profile_state.calls[CP_BODY_REFINE]==1);
 assert(collision_profile_state.ticks[CP_BODY_SOLVE]==9);
 assert(collision_profile_state.maximum[CP_BODY_SOLVE]==8);
 assert(collision_profile_state.counts[CP_BODY_CONTACTS]==6);
 collision_profile_report(1000,2,1000);
 assert(lines==4);
 assert(strstr(records[0],"body_solve=4.500000"));
 assert(strstr(records[1],"body_solve=8.000000"));
 assert(strstr(records[2],"body_solve=2"));
 assert(strstr(records[3],"body_contacts=6"));
 collision_profile_clear();assert(!collision_profile_state.enabled);
 collision_profile_state.enabled=1;lines=0;
 collision_profile_report(1000,0,1000);assert(lines==0);
 collision_profile_report(1000,1,0);assert(lines==0);
 assert(!collision_profile_state.calls[CP_BODY_SOLVE]&&!collision_profile_state.counts[CP_BODY_CONTACTS]);
 puts("PASS: disabled profiler makes no clock queries; early returns, nested scopes, explicit end, average/max/calls, count reset and zero-frame reports");
 return 0;
}
'''
source = build / 'accounting.c'
source.write_text(accounting)
exe = build / 'accounting.exe'
subprocess.run([str(gcc), '-m32', '-O2', '-Wall', '-Wextra', '-Werror', '-I', str(root), str(source), '-o', str(exe)], env=env, check=True)
subprocess.run([str(exe)], env=env, check=True)

# Use the actual production filter AND file writer with debug disabled. The
# logger filters format strings before expansion, so a generic "%s" call would
# discard every collision record even if the formatted text has the right prefix.
main_source = (root / 'NC-TK17-PhysX.c').read_text()


def function(name):
    match = re.search(r'^static [^;{}]*\b' + name + r'\([^;{}]*\)\s*\{', main_source, re.M)
    if not match:
        raise ValueError(name)
    pos, depth = match.end(), 1
    while depth:
        depth += (main_source[pos] == '{') - (main_source[pos] == '}')
        pos += 1
    return main_source[match.start():pos] + '\n'


logging = r'''
#include <windows.h>
#include <assert.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
static struct {int debug,performance_profile;} defaults_cfg={0,1};
static int log_ready=1;
static HMODULE self_module;
static CRITICAL_SECTION log_lock;
'''
logging += ''.join(function(n) for n in ('normal_log_starts_with', 'normal_log_line_allowed', 'log_line'))
logging += r'''
#define PHYSX_COLLISION_PROFILE_ENABLED 1
#include "physx_collision_profile.h"
int main(void){
 char text[8192]={0};FILE *file;size_t size;int lines=0;
 InitializeCriticalSection(&log_lock);
 collision_profile_state.enabled=1;
 {COLLISION_PROFILE_SCOPE(scope,CP_BODY_SOLVE);COLLISION_PROFILE_COUNT(CP_BODY_CONTACTS,3);}
 collision_profile_report(1000,2,1000);
 file=fopen("Logs/NC-TK17-PhysX.log","rb");assert(file);
 size=fread(text,1,sizeof(text)-1,file);fclose(file);assert(size>0);
 for(size_t i=0;i<size;i++) { lines+=text[i]=='\n'; }
 assert(lines==4);
 assert(strstr(text,"collision profile window_ms=1000 frames=2 avg_ms"));
 assert(strstr(text,"collision profile window_ms=1000 frames=2 max_call_ms"));
 assert(strstr(text,"collision profile window_ms=1000 frames=2 calls"));
 assert(strstr(text,"collision profile window_ms=1000 frames=2 counts"));
 assert(strstr(text,"body_contacts=3"));
 defaults_cfg.performance_profile=0;
 assert(!normal_log_line_allowed("collision profile %s"));
 assert(!normal_log_line_allowed("performance profile %s"));
 assert(!normal_log_line_allowed("unrelated verbose trace %s"));
 defaults_cfg.performance_profile=1;
 assert(normal_log_line_allowed("performance profile %s"));
 assert(!normal_log_line_allowed("unrelated verbose trace %s"));
 defaults_cfg.debug=1;assert(normal_log_line_allowed("unrelated verbose trace %s"));
 collision_profile_clear();DeleteCriticalSection(&log_lock);
 puts("PASS: actual production logger writes all four collision records with debug off/profile on; disabled filtering and unrelated logs preserved");
 return 0;
}
'''
log_dir = build / 'Logs'
log_dir.mkdir(exist_ok=True)
test_log = log_dir / 'NC-TK17-PhysX.log'
test_log.write_bytes(b'')
source = build / 'logging.c'
source.write_text(logging)
exe = build / 'logging.exe'
subprocess.run([str(gcc), '-m32', '-O2', '-Wall', '-Wextra', '-Werror', '-I', str(root), str(source), '-o', str(exe)], env=env, check=True)
subprocess.run([str(exe)], cwd=build, env=env, check=True)

# Force-include the real production profiler, keeping its logger local to the
# header so fixture log stubs remain independent. Constructors enable timing
# before each fixture's original main; every original assertion still runs.
shim = build / 'enabled.h'
shim.write_text(r'''
#include <windows.h>
#include <stdio.h>
#include <string.h>
#define PHYSX_COLLISION_PROFILE_ENABLED 1
static void profile_test_log(const char *fmt,...){(void)fmt;}
#define log_line profile_test_log
#include "physx_collision_profile.h"
#undef log_line
static void __attribute__((constructor)) profile_test_start(void){collision_profile_state.enabled=1;}
static void __attribute__((destructor)) profile_test_finish(void){
 unsigned long long calls=0,counts=0;LARGE_INTEGER f;
 for(int i=0;i<CP_PHASE_COUNT;i++)calls+=collision_profile_state.calls[i];
 for(int i=0;i<CP_COUNTER_COUNT;i++)counts+=collision_profile_state.counts[i];
 if(!calls){fputs("No collision timer was exercised\n",stderr);abort();}
 printf("PROFILE EXERCISED calls=%llu workload_counts=%llu\n",calls,counts);
 QueryPerformanceFrequency(&f);collision_profile_report(1000,1,f.QuadPart);
 collision_profile_clear();
}
''')
for name in ('body_contact_test', 'room_collision_query_test', 'sidecar_collision_test'):
    fixture = root / 'build' / (name + '.c')
    if not fixture.exists():
        raise SystemExit('Generate collision fixtures first: ' + str(fixture))
    exe = build / (name + '.exe')
    subprocess.run([str(gcc), '-m32', '-O2', '-Wall', '-Wextra', '-Werror', '-Wno-unused-function',
                    '-I', str(root), '-include', str(shim), str(fixture), '-o', str(exe)], env=env, check=True)
    result = subprocess.run([str(exe)], env=env, check=True, capture_output=True, text=True)
    print(name + ': all original assertions passed with production timers enabled')
    print(result.stdout.splitlines()[-1])
