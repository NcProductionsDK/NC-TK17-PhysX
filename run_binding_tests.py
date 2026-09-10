"""Exercise production sidecar layout detection and write quarantine off-engine."""
import os
from pathlib import Path
import re
import subprocess

root = Path(__file__).resolve().parent
source = (root / 'physx_sidecar.c').read_text()
main_source = (root / 'NC-TK17-PhysX.c').read_text()


def function(name):
    match = re.search(r'^static [^;{}]*\b' + name + r'\([^;{}]*\)\s*\{',
                      source, re.M)
    if not match:
        raise ValueError(name)
    pos = match.end()
    depth = 1
    while depth:
        depth += (source[pos] == '{') - (source[pos] == '}')
        pos += 1
    return source[match.start():pos] + '\n'


names = ['addon_vector_basis_like', 'addon_basis_rows_orthonormal_enough',
         'addon_normalize_basis_rows', 'addon_object_live_matrix_matches_scene',
         'assume_addon_s_transform_layout', 'addon_target_reset_write_guard',
         'addon_target_write_mapping_matches_guard',
         'addon_target_sample_write_guard']
functions = ''.join(function(name) for name in names)
# Use the real declarations for every field touched by the production functions.
fields = sorted(set(re.findall(r'target->(\w+)', functions)))
declarations = []
for field in fields:
    match = re.search(r'^    [^;\n]*\b' + field + r'(?:\[[^\]]+\])*;',
                      main_source, re.M)
    if not match:
        raise ValueError(field)
    declarations.append(match.group())

fixture = r'''
#include <windows.h>
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
static float physx_absf(float x) { return fabsf(x); }
static float physx_vec3_len(const float *v) {
    return sqrtf(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
}
static float vec3_dot(const float *a,const float *b) {
    return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
}
static int sane_probe_float(float x) { return isfinite(x); }
static int physx_vec3_sane_limit(const float *v,float limit) {
    return isfinite(v[0]) && isfinite(v[1]) && isfinite(v[2]) &&
        fabsf(v[0])<=limit && fabsf(v[1])<=limit && fabsf(v[2])<=limit;
}
static int ptr_readable(const void *p, size_t n) {
    MEMORY_BASIC_INFORMATION m;
    if (!p || !VirtualQuery(p,&m,sizeof(m)) || m.State!=MEM_COMMIT ||
        (m.Protect & (PAGE_NOACCESS|PAGE_GUARD))) return 0;
    return (const BYTE*)p+n <= (const BYTE*)m.BaseAddress+m.RegionSize;
}
static int is_nil_engine_object(void *raw,void *object) {
    (void)raw; return !object;
}
static struct { int debug; } defaults_cfg = {0};
static void log_line(const char *format,...) { (void)format; }
static float expected[3]={0.03655f,0.00003f,0.09398f};
static int pose_reads;
static void read_addon_sjoint_scene_pose(const char *path,const char *name,
        float *t,int *has_t,float *r,int *has_r) {
    (void)path; (void)name;
    memcpy(t,expected,sizeof(expected)); memset(r,0,3*sizeof(float));
    *has_t=1; *has_r=1; pose_reads++;
}
'''
fixture += 'typedef struct {\n' + '\n'.join(declarations) + '\n} physx_target_t;\n'
fixture += r'''
static void reset_addon_sjoint_orientation_rest(physx_target_t *t) {
    t->addon_joint_orientation_valid=0;
}
'''
fixture += function('find_vector3_offset')
fixture += functions
fixture += r'''
static void set_matrix(BYTE *base) {
    int i;
    memset(base,0,0x54);
    for(i=0;i<3;i++) ((float*)(base+0x18+i*0x10))[i]=1;
    memcpy(base+0x48,expected,sizeof(expected));
}
static void set_target(physx_target_t *t,BYTE *base,int native) {
    memset(t,0,sizeof(*t));
    t->object=t; t->s_object=base; t->addon_simulated_target=1;
    t->s_rotation_base=t->s_translation_base=base;
    t->s_rotation_offset=native?0x6c:0x38;
    t->s_translation_offset=native?0x7c:0x48;
}
int main(void) {
    SYSTEM_INFO si;
    BYTE *allocation,*edge,*native;
    BYTE snapshot[0x54];
    DWORD old_protect;
    physx_target_t t,room,sibling;
    int reads;
    GetSystemInfo(&si);
    allocation=VirtualAlloc(NULL,si.dwPageSize*2,MEM_RESERVE|MEM_COMMIT,PAGE_READWRITE);
    assert(allocation);
    assert(VirtualProtect(allocation+si.dwPageSize,si.dwPageSize,PAGE_NOACCESS,&old_protect));
    edge=allocation+si.dwPageSize-0x54;
    native=allocation;
    set_matrix(edge);
    memcpy(snapshot,edge,sizeof(snapshot));
    assert(!ptr_readable(edge,0x1000));
    assert(addon_object_live_matrix_matches_scene(edge,expected));
    assert(!addon_object_live_matrix_matches_scene(edge+4,expected));
    assert(!addon_object_live_matrix_matches_scene(NULL,expected));
    ((float*)(edge+0x48))[0]+=0.01f;
    assert(!addon_object_live_matrix_matches_scene(edge,expected));
    set_matrix(edge);
    memcpy(edge+0x28,edge+0x18,3*sizeof(float));
    assert(!addon_object_live_matrix_matches_scene(edge,expected));
    set_matrix(edge);
    ((float*)(edge+0x38))[0]=NAN;
    assert(!addon_object_live_matrix_matches_scene(edge,expected));
    set_matrix(edge);
    set_target(&t,edge,1);
    assert(assume_addon_s_transform_layout(&t,"fixture"));
    assert(t.s_rotation_offset==0x38 && t.s_translation_offset==0x48);
    assert(!strcmp(t.s_rotation_source,"object-live-matrix"));
    assert(!memcmp(snapshot,edge,sizeof(snapshot)));
    puts("PASS: wrapper matrix beside an inaccessible page; invalid bases, basis and translation rejected; detection never writes");

    set_target(&sibling,edge,0);
    assert(!addon_target_sample_write_guard(&sibling,100,0,"fixture"));
    assert(!addon_target_sample_write_guard(&sibling,116,0,"fixture"));
    assert(addon_target_sample_write_guard(&sibling,132,0,"fixture"));
    set_target(&t,edge,1);
    assert(!addon_target_sample_write_guard(&t,100,0,"fixture"));
    assert(t.addon_layout_retry_pending && t.s_rotation_offset==0x38);
    assert(!addon_target_sample_write_guard(&t,116,0,"fixture"));
    assert(!addon_target_sample_write_guard(&t,132,0,"fixture"));
    assert(!addon_target_sample_write_guard(&t,132,0,"fixture"));
    assert(addon_target_sample_write_guard(&t,148,0,"fixture"));
    assert(!t.addon_layout_retry_pending);
    assert(addon_target_write_mapping_matches_guard(&t));
    assert(addon_target_write_mapping_matches_guard(&sibling));
    puts("PASS: rejected root recovers automatically, still waits three samples/32ms, leaves sibling ready");

    memset(native,0,0x100);
    memcpy(native+0x7c,expected,sizeof(expected));
    set_target(&t,native,1);
    reads=pose_reads;
    assert(!addon_target_sample_write_guard(&t,0xfffffff0u,0,"fixture"));
    assert(pose_reads==reads+1);
    assert(!addon_target_sample_write_guard(&t,0x10u,0,"fixture"));
    assert(pose_reads==reads+1);
    assert(!addon_target_sample_write_guard(&t,0x3d8u,0,"fixture"));
    assert(pose_reads==reads+2 && !t.addon_write_guard_ready);
    set_matrix(native);
    assert(!addon_target_sample_write_guard(&t,0x7c0u,0,"fixture"));
    assert(t.s_rotation_offset==0x38);
    puts("PASS: persistently rejected layout stays quarantined; retries limited to 1Hz across tick wrap; later valid matrix detected");

    set_target(&room,native,1);
    reads=pose_reads;
    assert(!addon_target_sample_write_guard(&room,100,1,"fixture"));
    assert(!addon_target_sample_write_guard(&room,116,1,"fixture"));
    assert(addon_target_sample_write_guard(&room,132,1,"fixture"));
    assert(pose_reads==reads && room.s_rotation_offset==0x6c);
    room.s_translation_base=edge;
    assert(!addon_target_sample_write_guard(&room,148,1,NULL));
    assert(!room.addon_write_guard_ready);
    puts("PASS: native room layout remains supported; mismatched bases still rejected");
    assert(VirtualFree(allocation,0,MEM_RELEASE));
    return 0;
}
'''
build = root / 'build'
build.mkdir(exist_ok=True)
test_c = build / 'binding_test.c'
test_c.write_text(fixture)
gcc = Path(r'C:\msys64\mingw32\bin\gcc.exe')
env = dict(os.environ)
env['PATH'] = str(gcc.parent) + os.pathsep + env['PATH']
exe = build / 'binding_test.exe'
subprocess.run([str(gcc), '-m32', '-O2', '-Wall', '-Wextra', '-Werror',
                '-static-libgcc', '-o', str(exe), str(test_c)], env=env, check=True)
subprocess.run([str(exe)], env=env, check=True)
