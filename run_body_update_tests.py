"""Exercise production scheduling, Windows INI reads and collider due checks."""
from pathlib import Path
import os
import re
import subprocess

ROOT = Path(__file__).resolve().parent


def function(file, name):
    text = (ROOT / file).read_text()
    match = re.search(r'^static [^;{}]*\b' + name + r'\([^;{}]*\)\s*\{', text, re.M)
    end, depth = match.end(), 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[match.start():end] + '\n'


source = r'''
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../physx_body_update.h"

static void legacy(void) {
    const uint32_t starts[]={0,1,100,0xfffffff0u};
    for(unsigned int s=0;s<4;s++) for(int interval=16;interval<=1000;interval+=16) {
        body_update_clock_t c={0};
        uint32_t last=starts[s];
        for(unsigned int n=0;n<3000;n++) {
            uint32_t now=starts[s]+n;
            int expected=!last || (uint32_t)(now-last)>=(uint32_t)interval;
            assert(body_update_due(&c,0,interval,last,now,n*1000u)==expected);
            if(expected) {
                assert(body_update_elapsed(&c,0,last,now,n*1000u)==(last?(uint32_t)(now-last):0));
                last=now;
            }
        }
    }
    puts("PASS: legacy scheduling and elapsed values match across intervals/startup/DWORD wrap");
}

static void render_rates(void) {
    const int renders[]={30,60,90,120,144,240};
    const int rates[]={-1,30,60,90,120,144,240};
    for(unsigned int r=0;r<6;r++) for(unsigned int t=0;t<7;t++) {
        body_update_clock_t c={0};
        uint32_t last=0; unsigned int count=0; uint64_t elapsed_total=0,previous_us=0;
        for(int n=0;n<=renders[r]*10;n++) {
            uint64_t us=1000000u+(uint64_t)n*1000000u/renders[r];
            /* Deliberately emulate a coarse 16 ms clock, with repeated ticks. */
            uint32_t now=(uint32_t)(us/16000u)*16u;
            if(body_update_due(&c,rates[t],16,last,now,us)) {
                uint32_t elapsed=body_update_elapsed(&c,rates[t],last,now,us);
                if(count) assert(elapsed==us/1000-previous_us/1000 && elapsed>0);
                else assert(!elapsed);
                elapsed_total+=elapsed; previous_us=us;
                last=now; count++;
                assert(!body_update_due(&c,rates[t],16,last,now,us));
            }
        }
        int expected=(rates[t]<0 || rates[t]>renders[r])?renders[r]:rates[t];
        assert(count>=(unsigned int)(expected*10) && count<=(unsigned int)(expected*10+1));
        assert(elapsed_total==10000);
        if(renders[r]==90) printf("90 FPS fixture: setting %d -> %.1f body updates/s\n",rates[t],(count-1)/10.0);
    }
    /* Phase deadlines must not degrade 60 Hz at 90 FPS into a 45 Hz gate. */
    puts("PASS: render-rate updates and target rates stay on pace without time drift or duplicate same-frame solves");
}

static void transitions(void) {
    body_update_clock_t c={0};
    assert(body_update_elapsed(&c,-1,0,1000,1000000)==0);
    assert(body_update_due(&c,-1,16,1000,1000,1011000));
    assert(body_update_elapsed(&c,-1,1000,1000,1011000)==11);
    assert(body_update_due(&c,60,16,1000,1000,1012000));
    assert(body_update_elapsed(&c,60,1000,1000,1012000)==0);
    assert(body_update_elapsed(&c,60,1000,2000,2000000)==988);
    assert(!body_update_due(&c,60,16,2000,2000,2000000));
    assert(c.next_us>2000000); /* One bounded solve after a stall; no backlog. */
    assert(body_update_due(&c,60,16,2000,2016,100)); /* Clock reset/fallback. */
    assert(body_update_elapsed(&c,60,2000,2016,100)==0);
    assert(body_update_elapsed(&c,0,2016,2032,16000)==0);
    assert(body_update_elapsed(&c,0,2032,2048,32000)==16);
    assert(body_update_elapsed(&c,-1,2048,2064,48000)==0);
    assert(!body_update_due(&c,-1,16,2064,2064,48999));
    assert(body_update_due(&c,-1,16,2064,2064,49000));
    assert(body_update_elapsed(&c,-1,2064,2064,49000)==1);
    assert(body_update_elapsed(&c,-1,2064,3000,UINT64_C(5000000000000))==UINT32_MAX);
    assert(body_update_clamp_rate(-9)==0 && body_update_clamp_rate(-1)==-1);
    assert(body_update_clamp_rate(0)==0 && body_update_clamp_rate(120)==120);
    assert(body_update_clamp_rate(999)==240);
    puts("PASS: live mode changes, clock reset, stalls, long pauses and extreme render rates are bounded");
}

typedef struct { int enabled,enabled_person[4],room_collision_enabled,interval_ms,update_rate_hz; } body_chain_physics_config_t;
typedef struct { DWORD last_tick; body_update_clock_t update_clock; } body_chain_person_state_t;
typedef struct { DWORD last_tick; body_update_clock_t update_clock; } breasts_physics_person_state_t;
static body_chain_physics_config_t body_chain_physics_cfg,testicle_physics_cfg,breasts_physics_cfg,butt_physics_cfg;
static struct {int enabled,debug_draw,breasts_collision_enabled,butt_collision_enabled,penis_collision_enabled,testicle_collision_enabled;} body_chain_collider_cfg;
static breasts_physics_person_state_t breasts_physics_states[4],butt_physics_states[4];
static body_chain_person_state_t chains[2][4];
static uint64_t body_update_frame_us;
static int body_update_precise_frame;
static DWORD physx_simulation_serial;
static body_chain_physics_config_t body_chain_physics_person_cfg[4],testicle_physics_person_cfg[4];
static body_chain_physics_config_t breasts_physics_person_cfg[4],butt_physics_person_cfg[4];
#define breasts_physics_global_cfg breasts_physics_cfg
#define butt_physics_global_cfg butt_physics_cfg
typedef struct {int unused;} body_chain_collider_person_state_t;
#define BODY_CHAIN_COLLIDER_GROUP_ALL 3
static int room_collision_is_enabled(void){return 0;}
static int refreshed;
static void body_profile_set_active_person_config(int person){(void)person;}
static int breasts_physics_person_enabled(int p){return breasts_physics_cfg.enabled && breasts_physics_cfg.enabled_person[p];}
static int butt_physics_person_enabled(int p){return butt_physics_cfg.enabled && butt_physics_cfg.enabled_person[p];}
static body_chain_person_state_t *body_chain_active_person_state(int p,int t){return &chains[t][p];}
#define PENIS_PHYSICS_CONFIG_SECTION "penis_physics"
#define TESTICLE_PHYSICS_CONFIG_SECTION "testicle_physics"
#define BREASTS_PHYSICS_CONFIG_SECTION "breasts_physics"
#define BUTT_PHYSICS_CONFIG_SECTION "butt_physics"
'''
source += function('physx_physics.c', 'body_collider_person_physics_due')
source += function('NC-TK17-PhysX.c', 'body_update_prepare_frame')
# Keep the production cache/validation prefix; replace expensive bone sampling
# below it with a counter so identical coarse ticks can be tested explicitly.
refresh = function('physx_colliders.c', 'update_body_chain_colliders_for_person_scope')
source += refresh.split('    state = &body_chain_collider_states[person_index];')[0]
source += '    (void)state;(void)person;(void)i;(void)ready;(void)scene_person_visible;refreshed++;\n}\n'

# Execute the actual new assignments through Windows' INI parser, including
# negative values, absent keys and the global -> selected-body override path.
config = (ROOT / 'physx_config.c').read_text()
assignments = re.findall(r'(?:body_chain_physics_cfg|testicle_physics_cfg|breasts_physics_global_cfg|butt_physics_global_cfg)\.update_rate_hz\s*=\s*body_update_clamp_rate\([\s\S]*?;', config)
assert len(assignments) == 6
source += 'static void global_read(const char *config_path){\n' + '\n'.join(a for a in assignments if 'config_path' in a) + '\n}\n'
source += 'static void sidecar_read(const char *path){\n' + '\n'.join(a for a in assignments if 'config_path' not in a) + '\n}\n'
paired_read = re.search(r'cfg->update_rate_hz\s*=\s*body_update_clamp_rate\([\s\S]*?;', config).group()
source += 'static void paired_read(body_chain_physics_config_t *cfg,const char *section,const char *path){' + paired_read + '}\n'
source += r'''
static void integration(void) {
    char dir[MAX_PATH],path[MAX_PATH];
    assert(GetTempPathA(sizeof(dir),dir));
    assert(GetTempFileNameA(dir,"bup",0,path));
    assert(WritePrivateProfileStringA("penis_physics","update_rate_hz","-1",path));
    assert(WritePrivateProfileStringA("testicle_physics","update_rate_hz","90",path));
    global_read(path);
    assert(body_chain_physics_cfg.update_rate_hz==-1 && testicle_physics_cfg.update_rate_hz==90);
    assert(WritePrivateProfileStringA("penis_physics","update_rate_hz",NULL,path));
    sidecar_read(path);
    assert(body_chain_physics_cfg.update_rate_hz==-1);
    assert(WritePrivateProfileStringA("testicle_physics","update_rate_hz","0",path));
    sidecar_read(path);
    assert(testicle_physics_cfg.update_rate_hz==0);
    global_read(path);
    assert(body_chain_physics_cfg.update_rate_hz==0); /* Removal restores default. */
    assert(WritePrivateProfileStringA("penis_physics","update_rate_hz","900",path));
    global_read(path); assert(body_chain_physics_cfg.update_rate_hz==240);
    assert(WritePrivateProfileStringA("breasts_physics","update_rate_hz","-1",path));
    assert(WritePrivateProfileStringA("butt_physics","update_rate_hz","60",path));
    global_read(path);
    assert(breasts_physics_cfg.update_rate_hz==-1 && butt_physics_cfg.update_rate_hz==60);
    for(int i=0;i<2;i++) {
        const char *section=i?"butt_physics":"breasts_physics";
        body_chain_physics_config_t *cfg=i?&butt_physics_cfg:&breasts_physics_cfg;
        assert(WritePrivateProfileStringA(section,"update_rate_hz",NULL,path));
        int inherited=cfg->update_rate_hz;
        paired_read(cfg,section,path);assert(cfg->update_rate_hz==inherited);
        assert(WritePrivateProfileStringA(section,"update_rate_hz","0",path));
        paired_read(cfg,section,path);assert(!cfg->update_rate_hz);
        assert(WritePrivateProfileStringA(section,"update_rate_hz",NULL,path));
        cfg->update_rate_hz=-1;global_read(path);assert(!cfg->update_rate_hz);
    }
    assert(DeleteFileA(path));

    body_chain_physics_cfg.enabled=1;body_chain_physics_cfg.enabled_person[0]=1;
    body_chain_physics_cfg.interval_ms=16;
    body_chain_collider_cfg.penis_collision_enabled=1;
    chains[0][0].last_tick=1000;
    body_update_frame_us=1011000;
    body_chain_physics_cfg.update_rate_hz=0;
    assert(!body_collider_person_physics_due(0,1000));
    body_chain_physics_cfg.update_rate_hz=-1;
    body_update_elapsed(&chains[0][0].update_clock,-1,0,1000,1000000);
    assert(body_collider_person_physics_due(0,1000)); /* Coarse tick unchanged. */
    body_update_elapsed(&chains[0][0].update_clock,-1,1000,1000,1011000);
    assert(!body_collider_person_physics_due(0,1000));
    testicle_physics_cfg=body_chain_physics_cfg;
    body_chain_physics_cfg.enabled=0;
    body_chain_collider_cfg.testicle_collision_enabled=1;
    assert(body_collider_person_physics_due(0,1000));
    body_update_elapsed(&chains[1][0].update_clock,-1,0,1000,1011000);
    assert(!body_collider_person_physics_due(0,1000));
    testicle_physics_cfg.enabled=0;
    for(int i=0;i<2;i++) {
        body_chain_physics_config_t *cfg=i?&butt_physics_cfg:&breasts_physics_cfg;
        breasts_physics_person_state_t *state=i?&butt_physics_states[0]:&breasts_physics_states[0];
        cfg->enabled=cfg->enabled_person[0]=cfg->room_collision_enabled=1;
        cfg->interval_ms=16;cfg->update_rate_hz=0;state->last_tick=1000;
        assert(!body_collider_person_physics_due(0,1000));
        cfg->update_rate_hz=-1;
        body_update_elapsed(&state->update_clock,-1,0,1000,1000000);
        assert(body_collider_person_physics_due(0,1000));
        body_update_elapsed(&state->update_clock,-1,1000,1000,1011000);
        assert(!body_collider_person_physics_due(0,1000));
        cfg->enabled=0;
    }
    puts("PASS: all four sections inherit/override rates and all collider due paths follow precise body scheduling");
}
static void frame_clock_and_cache(void) {
    body_update_prepare_frame(1000);
    assert(!body_update_precise_frame);
    body_chain_physics_person_cfg[1].enabled=1;
    body_chain_physics_person_cfg[1].enabled_person[1]=1;
    body_chain_physics_person_cfg[1].update_rate_hz=-1;
    body_update_prepare_frame(1000);
    assert(body_update_precise_frame && body_update_frame_us>0);
    uint64_t first=body_update_frame_us;
    Sleep(2);
    body_update_prepare_frame(1000);
    assert(body_update_frame_us>first); /* Actual QPC, identical coarse input. */
    body_chain_physics_person_cfg[1].enabled=0;
    testicle_physics_person_cfg[2]=body_chain_physics_person_cfg[1];
    testicle_physics_person_cfg[2].enabled=1;
    testicle_physics_person_cfg[2].enabled_person[2]=1;
    body_update_prepare_frame(1000);assert(body_update_precise_frame);
    testicle_physics_person_cfg[2].enabled=0;
    for(int i=0;i<2;i++) {
        body_chain_physics_config_t *cfg=i?&butt_physics_person_cfg[0]:&breasts_physics_person_cfg[0];
        cfg->enabled=cfg->enabled_person[0]=1;cfg->update_rate_hz=-1;
        body_update_prepare_frame(1000);assert(body_update_precise_frame);
        cfg->enabled=0;
    }

    body_chain_collider_cfg.enabled=1;
    physx_simulation_serial=1;
    update_body_chain_colliders_for_person_scope(0,1000,1);assert(refreshed==1);
    update_body_chain_colliders_for_person_scope(0,1000,1);assert(refreshed==1);
    physx_simulation_serial++;
    update_body_chain_colliders_for_person_scope(0,1000,1);assert(refreshed==2);
    update_body_chain_colliders_for_person_scope(0,1000,3);assert(refreshed==3);
    update_body_chain_colliders_for_person_scope(1,1000,1);assert(refreshed==4);
    body_update_precise_frame=0;physx_simulation_serial++;
    update_body_chain_colliders_for_person_scope(0,1000,1);assert(refreshed==4);
    update_body_chain_colliders_for_person_scope(0,1016,1);assert(refreshed==5);
    puts("PASS: actual frame clock advances within coarse ticks; collider cache refreshes per precise frame and preserves legacy reuse");
}
int main(void){legacy();render_rates();transitions();integration();frame_clock_and_cache();return 0;}
'''
build = ROOT / 'build'
build.mkdir(exist_ok=True)
test = build / 'body_update_test.c'
test.write_text(source)
gcc = Path(r'C:\msys64\mingw32\bin\gcc.exe')
env = dict(os.environ, PATH=str(gcc.parent) + os.pathsep + os.environ['PATH'])
exe = build / 'body_update_test.exe'
subprocess.run([str(gcc), '-m32', '-O2', '-Wall', '-Wextra', '-Werror',
                '-static-libgcc', '-o', str(exe), str(test)], env=env, check=True)
subprocess.run([str(exe)], env=env, check=True, timeout=30)
