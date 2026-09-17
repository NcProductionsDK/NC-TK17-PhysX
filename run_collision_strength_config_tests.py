"""Exercise the production INI reader with Windows profile APIs."""
from pathlib import Path
import os
import re
import subprocess

root = Path(__file__).resolve().parent
config = (root / 'physx_config.c').read_text()

def function(name):
    match = re.search(r'^static [^;{}]*\b' + name + r'\([^;{}]*\)\s*\{', config, re.M)
    pos, depth = match.end(), 1
    while depth:
        depth += (config[pos] == '{') - (config[pos] == '}')
        pos += 1
    return config[match.start():pos] + '\n'

source = r'''
#include <windows.h>
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
static float physx_clampf(float v,float lo,float hi){return fmaxf(lo,fminf(hi,v));}
'''
source += function('profile_collision_strength')
source += r'''
int main(int argc,char **argv){
 const char *sections[]={"breasts_physics","penis_physics","testicle_physics","butt_physics"};
 const char *values[]={"0.1","0.25","0.75","1.0","0.0","-3","2.5","nan","invalid","0.25oops"};
 float expected[]={.1f,.25f,.75f,1,.1f,.1f,1,1,1,1};
 assert(argc==2);
 for(int s=0;s<4;s++){
  assert(WritePrivateProfileStringA(sections[s],"collision_strength",NULL,argv[1]));
  assert(profile_collision_strength(sections[s],1,argv[1])==1);
  assert(profile_collision_strength(sections[s],.25f,argv[1])==.25f);
  for(int i=0;i<10;i++){
   assert(WritePrivateProfileStringA(sections[s],"collision_strength",values[i],argv[1]));
   assert(fabsf(profile_collision_strength(sections[s],1,argv[1])-expected[i])<1e-6f);
  }
  assert(WritePrivateProfileStringA(sections[s],"collision_strength",NULL,argv[1]));
  assert(profile_collision_strength(sections[s],1,argv[1])==1);
 }
 puts("PASS: all four INI sections accept floats, clamp to 0.1-1, default to 1, inherit body-profile fallback, reject NaN, and reset after key removal");
 return 0;
}
'''
build = root / 'build'
build.mkdir(exist_ok=True)
c = build / 'collision_strength_config_test.c'
c.write_text(source)
gcc = Path(r'C:\msys64\mingw32\bin\gcc.exe')
env = dict(os.environ, PATH=str(gcc.parent) + os.pathsep + os.environ['PATH'])
exe = build / 'collision_strength_config_test.exe'
subprocess.run([str(gcc), '-m32', '-O2', '-Wall', '-Wextra', '-Werror',
                '-static-libgcc', '-o', str(exe), str(c)], env=env, check=True)
subprocess.run([str(exe), str(build / 'collision_strength_test.ini')], env=env,
               check=True, timeout=30)
