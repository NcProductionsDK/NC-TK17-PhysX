
#include <windows.h>
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
static float physx_clampf(float v,float lo,float hi){return fmaxf(lo,fminf(hi,v));}
static float profile_collision_strength(const char *section,float fallback,const char *path)
{
    char buf[128],*end;float value;
    GetPrivateProfileStringA(section,"collision_strength","",buf,sizeof(buf),path);
    if(!buf[0]) return fallback;
    value=strtof(buf,&end);
    if(end==buf) return fallback;
    while(*end==' ' || *end=='\t') end++;
    if(*end) return fallback;
    return isfinite(value)?physx_clampf(value,0.1f,1.0f):fallback;
}

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
