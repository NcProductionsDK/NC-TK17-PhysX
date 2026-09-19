"""Production breast/butt aliases and collision offsets with Windows INI files."""
from pathlib import Path
import os
import re
import subprocess

root = Path(__file__).resolve().parent


def function(file, name):
    code = (root / file).read_text()
    match = re.search(r'^static [^;{}]*\b' + name + r'\([^;{}]*\)\s*\{', code, re.M)
    pos, depth = match.end(), 1
    while depth:
        depth += (code[pos] == '{') - (code[pos] == '}')
        pos += 1
    return code[match.start():pos] + '\n'


source = r'''
#include <windows.h>
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
typedef struct {
 float max_angle,link_max_angle[3][3],link_min_angle[3][3],link_gain[3];
 unsigned collision_offset_bounds;
 float collision_min_offset[3],collision_max_offset[3];
} body_chain_physics_config_t;
static void log_line(const char *fmt,...){(void)fmt;}
'''
for file, names in [
    ('NC-TK17-PhysX.c', ['trim_in_place', 'profile_string_found']),
    ('physx_config.c', ['profile_float', 'fill_vec3', 'profile_vec3_or_float',
                        'profile_min_angle_vec3_or_float', 'profile_paired_angle_settings',
                        'profile_paired_collision_offsets']),
]:
    for name in names:
        source += function(file, name)
source += r'''
static void assert_close(float a,float b){assert(fabsf(a-b)<1e-6f);}
int main(int argc,char **argv){
 assert(argc==3);
 const char *sections[]={"breasts_physics","butt_physics"};
 for(int s=0;s<2;s++){
  const char *section=sections[s];
  assert(WritePrivateProfileStringA(section,NULL,NULL,argv[1]));
  assert(WritePrivateProfileStringA(section,NULL,NULL,argv[2]));
  body_chain_physics_config_t cfg={0};cfg.max_angle=30;
  profile_paired_angle_settings(section,argv[1],&cfg,0);
  assert_close(cfg.link_max_angle[0][0],30);assert_close(cfg.link_min_angle[0][0],-30);assert_close(cfg.link_gain[0],1);
  WritePrivateProfileStringA(section,"joint01_max_angle","10,20,30",argv[1]);
  WritePrivateProfileStringA(section,"joint01_min_angle","-5,-15,-25",argv[1]);
  WritePrivateProfileStringA(section,"joint01_gain","0.7",argv[1]);
  profile_paired_angle_settings(section,argv[1],&cfg,0);
  for(int a=0;a<3;a++){assert_close(cfg.link_max_angle[0][a],10+10*a);assert_close(cfg.link_min_angle[0][a],-5-10*a);}
  assert_close(cfg.link_gain[0],.7f);
  WritePrivateProfileStringA(section,"max_angle","40,50,60",argv[1]);
  profile_paired_angle_settings(section,argv[1],&cfg,0);
  assert_close(cfg.link_max_angle[0][2],60);assert_close(cfg.link_min_angle[0][2],-25);assert_close(cfg.link_gain[0],.7f);
  WritePrivateProfileStringA(section,"min_angle","-12",argv[1]);
  WritePrivateProfileStringA(section,"gain","1.2",argv[1]);
  profile_paired_angle_settings(section,argv[1],&cfg,0);
  assert_close(cfg.link_min_angle[0][2],-12);assert_close(cfg.link_gain[0],1.2f);
  /* Legacy sidecar overrides the global config, even when global uses aliases. */
  WritePrivateProfileStringA(section,"joint01_gain","0.5",argv[2]);
  profile_paired_angle_settings(section,argv[2],&cfg,1);
  assert_close(cfg.link_gain[0],.5f);assert_close(cfg.link_max_angle[0][2],60);assert_close(cfg.link_min_angle[0][2],-12);
  WritePrivateProfileStringA(section,"max_angle","15",argv[2]);
  WritePrivateProfileStringA(section,"gain","0.9",argv[2]);
  profile_paired_angle_settings(section,argv[2],&cfg,1);
  /* A missing minimum inherits independently from the changed maximum. */
  assert_close(cfg.link_gain[0],.9f);assert_close(cfg.link_max_angle[0][2],15);assert_close(cfg.link_min_angle[0][2],-12);
  WritePrivateProfileStringA(section,NULL,NULL,argv[1]);
  profile_paired_angle_settings(section,argv[1],&cfg,0);
  assert_close(cfg.link_gain[0],1);assert_close(cfg.link_max_angle[0][2],30);assert_close(cfg.link_min_angle[0][2],-30);

  profile_paired_collision_offsets(section,argv[1],&cfg,0);assert(cfg.collision_offset_bounds==0);
  WritePrivateProfileStringA(section,"collision_min_offset","-0.01, -0.02, 0 ; comment",argv[1]);
  WritePrivateProfileStringA(section,"collision_max_offset","0.03 0.04 0.05",argv[1]);
  profile_paired_collision_offsets(section,argv[1],&cfg,0);assert(cfg.collision_offset_bounds==3);
  assert_close(cfg.collision_min_offset[0],-.01f);assert_close(cfg.collision_min_offset[1],-.02f);assert_close(cfg.collision_min_offset[2],0);
  assert_close(cfg.collision_max_offset[0],.03f);assert_close(cfg.collision_max_offset[2],.05f);
  WritePrivateProfileStringA(section,"collision_max_offset","0,0,0",argv[2]);
  profile_paired_collision_offsets(section,argv[2],&cfg,1);
  assert_close(cfg.collision_min_offset[0],-.01f);assert_close(cfg.collision_max_offset[2],0);
  const char *bad[]={"nan,0,0","0,inf,0","0.01","0,0","0,0,0junk","0,0,0,0","0-1-2","1,0,0"};
  for(unsigned i=0;i<sizeof(bad)/sizeof(bad[0]);i++){
   WritePrivateProfileStringA(section,"collision_min_offset",bad[i],argv[2]);
   profile_paired_collision_offsets(section,argv[2],&cfg,1);
   assert_close(cfg.collision_min_offset[0],-.01f);assert(cfg.collision_offset_bounds==3);
  }
  WritePrivateProfileStringA(section,"collision_min_offset",NULL,argv[1]);
  profile_paired_collision_offsets(section,argv[1],&cfg,0);assert(cfg.collision_offset_bounds==2);
  WritePrivateProfileStringA(section,"collision_max_offset","-0.1,0,0",argv[1]);
  profile_paired_collision_offsets(section,argv[1],&cfg,0);assert(cfg.collision_offset_bounds==0);
  WritePrivateProfileStringA(section,"collision_min_offset","0,0,0",argv[1]);
  WritePrivateProfileStringA(section,"collision_max_offset","0,0,0",argv[1]);
  profile_paired_collision_offsets(section,argv[1],&cfg,0);assert(cfg.collision_offset_bounds==3);
  WritePrivateProfileStringA(section,NULL,NULL,argv[1]);
  profile_paired_collision_offsets(section,argv[1],&cfg,0);assert(cfg.collision_offset_bounds==0);
 }
 puts("PASS: breast/butt aliases take precedence per key, legacy keys work globally and in sidecars, scalar/vector angles and missing-key defaults remain supported");
 puts("PASS: optional XYZ collision bounds inherit independently, zero is valid, malformed/non-finite/wrong-sign input is rejected, removal restores legacy behavior");
 return 0;
}
'''
build = root / 'build'
build.mkdir(exist_ok=True)
c = build / 'paired_config_test.c'
c.write_text(source)
gcc = Path(r'C:\msys64\mingw32\bin\gcc.exe')
env = dict(os.environ, PATH=str(gcc.parent) + os.pathsep + os.environ['PATH'])
exe = build / 'paired_config_test.exe'
subprocess.run([str(gcc), '-m32', '-O2', '-Wall', '-Wextra', '-Werror',
                '-static-libgcc', '-o', str(exe), str(c)], env=env, check=True)
subprocess.run([str(exe), str(build / 'paired_config_test.ini'),
                str(build / 'paired_config_body_test.ini')], env=env, check=True, timeout=30)
