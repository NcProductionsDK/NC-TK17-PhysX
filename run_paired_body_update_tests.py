"""Production paired-body motion gain, angular integration and publish logging."""
from pathlib import Path
import os, re, subprocess
ROOT = Path(__file__).resolve().parent

def function(file, name):
    s = (ROOT / file).read_text()
    m = re.search(r'^static [^;{}]*\b' + name + r'\([^;{}]*\)\s*\{', s, re.M)
    p, d = m.end(), 1
    while d:
        d += (s[p] == '{') - (s[p] == '}'); p += 1
    return s[m.start():p] + '\n'

source = r'''
#include <assert.h>
#include <stdio.h>
#include <stdarg.h>
#include "../physx_body_motion.h"
typedef unsigned long DWORD;
typedef struct {
    int update_rate_hz,translation_tail_axis[3],rotation_source_axis[3],rotation_tail_axis[3];
    float stiffness,damping,link_max_angle[3][3],link_min_angle[3][3];
    float translation_scale[3],rotation_scale[3],rotation_deadzone,translation_deadzone;
} body_chain_physics_config_t;
static float physx_clampf(float v,float a,float b){return fmaxf(a,fminf(b,v));}
static float physx_absf(float v){return fabsf(v);}
#define BODY_CHAIN_TRANSLATION_RESPONSE 3000.0f
static float breasts_physics_translation_sign[2][3]={{1,-1,1},{-1,1,-1}};
static float breasts_physics_rotation_sign[2][3]={{-1,1,1},{1,-1,1}};
static float butt_physics_translation_sign[2][3]={{1,1,-1},{-1,1,1}};
static float butt_physics_rotation_sign[2][3]={{1,-1,1},{-1,1,1}};
static struct {int performance_profile;} defaults_cfg;
static char record[512];static int records;
static void log_line(const char *fmt,...) {
    va_list a;va_start(a,fmt);vsnprintf(record,sizeof(record),fmt,a);va_end(a);records++;
}
'''
for name in ('body_chain_link_axis_limit','body_chain_link_axis_min_limit','body_chain_clamp_link_axis_angle'):
    source += function('physx_config.c', name)
for name in ('paired_body_duration','paired_body_scale_motion','paired_body_rotation_step','breasts_physics_build_side_drive','body_update_record_publish'):
    source += function('physx_physics.c', name)
source += function('physx_butt.c','butt_physics_build_side_drive')
source += 'typedef struct {int unused;} breasts_physics_person_state_t;\n'
for body, file in (('breasts','physx_physics.c'),('butt','physx_butt.c')):
    source += f'''
static int {body}_physics_bone_translation_source_axis[3]={{2,0,1}};
static int {body}_physics_bone_translation_tail_axis[3]={{1,2,0}};
static float {body}_physics_bone_translation_scale[3]={{2,3,-4}};
static float {body}_physics_bone_translation_sign[2][3]={{{{1,-1,1}},{{-1,1,1}}}};
static int {body}_physics_bone_translation_space;
static int {body}_physics_body_translation_to_parent_local(const breasts_physics_person_state_t *s,int side,const float p[3],float out[3])
{{(void)s;(void)side;out[0]=p[1];out[1]=-p[0];out[2]=p[2];return 1;}}
'''
    source += function(file, f'{body}_physics_build_bone_translation_target')
source += r'''
static body_chain_physics_config_t config(void) {
    body_chain_physics_config_t c={0};c.stiffness=100;c.damping=8;
    for(int a=0;a<3;a++) {
        c.translation_tail_axis[a]=a;c.rotation_source_axis[a]=a;c.rotation_tail_axis[a]=a;
        c.translation_scale[a]=.1f;c.rotation_scale[a]=2;
        for(int j=0;j<3;j++){c.link_max_angle[j][a]=80;c.link_min_angle[j][a]=-70;}
    }
    c.rotation_deadzone=.005f;
    return c;
}
static void legacy(void) {
    body_chain_physics_config_t c=config();
    for(unsigned int ms=0;ms<2000;ms++) {
        float old=ms?(float)ms/1000.0f:.016f;if(old<=0)old=.016f;if(old>.025f)old=.025f;
        assert(paired_body_duration(0,ms)==old);
        float pos[3]={1,-2,3},vel[3]={4,-5,6},refp[3],refv[3],target[3]={9,-8,7};
        memcpy(refp,pos,sizeof(pos));memcpy(refv,vel,sizeof(vel));
        paired_body_scale_motion(0,ms,pos,vel);
        assert(!memcmp(pos,refp,sizeof(pos)) && !memcmp(vel,refv,sizeof(vel)));
        paired_body_rotation_step(&c,pos,vel,target,old);
        for(int a=0;a<3;a++) {
            float accel=(target[a]-refp[a])*c.stiffness-refv[a]*c.damping;
            refv[a]+=accel*old;refp[a]+=refv[a]*old;
            refp[a]=body_chain_clamp_link_axis_angle(&c,0,a,refp[a]);
            assert(pos[a]==refp[a] && vel[a]==refv[a]);
        }
    }
    puts("PASS: legacy paired duration, movement and angular outputs match exactly for 0..1999 ms");
}
static void rates(void) {
    const unsigned int cadence[]={4,7,11,16,17,33,50,100};
    body_chain_physics_config_t c=config();c.update_rate_hz=-1;
    for(int body=0;body<2;body++)for(int side=0;side<2;side++) {
        float reference[3],p[3]={.0016f,-.0032f,.0048f},r[3]={.16f,.32f,-.16f};
        if(body)butt_physics_build_side_drive(&c,side,p,r,reference);
        else breasts_physics_build_side_drive(&c,side,p,r,reference);
        for(int k=0;k<8;k++) {
            unsigned int ms=cadence[k];float out[3];
            for(int a=0;a<3;a++){p[a]=(a+1)*.0001f*ms*(a==1?-1:1);r[a]=(a==1?2:1)*.01f*ms*(a==2?-1:1);}
            paired_body_scale_motion(-1,ms,p,r);
            if(body)butt_physics_build_side_drive(&c,side,p,r,out);
            else breasts_physics_build_side_drive(&c,side,p,r,out);
            for(int a=0;a<3;a++)assert(fabsf(out[a]-reference[a])<.000002f);
            float pos[3]={0},vel[3]={0};unsigned int elapsed=0;
            while(elapsed<6000) {
                unsigned int step=ms;if(step>6000-elapsed)step=6000-elapsed;
                paired_body_rotation_step(&c,pos,vel,reference,paired_body_duration(-1,step));elapsed+=step;
            }
            for(int a=0;a<3;a++){assert(fabsf(pos[a]-reference[a])<.0001f);assert(fabsf(vel[a])<.0001f);}
        }
    }
    for(int k=0;k<8;k++) {
        float pos[3]={0},vel[3]={0},target[3]={200,-200,20};
        for(int n=0;n<300;n++) {
            paired_body_rotation_step(&c,pos,vel,target,paired_body_duration(-1,cadence[k]));
            for(int a=0;a<3;a++)assert(isfinite(pos[a]) && isfinite(vel[a]) && pos[a]>=-70 && pos[a]<=80);
        }
    }
    for(int i=0;i<2;i++) {
        float p[3]={1,2,3},r[3]={4,5,6};paired_body_scale_motion(-1,i?5000:0,p,r);
        for(int a=0;a<3;a++)assert(p[a]==0 && r[a]==0);
    }
    assert(fabsf(paired_body_duration(-1,5000)-.1f)<1e-7f);
    puts("PASS: both sides of breasts/butt keep 16 ms movement gain and spring equilibrium at 10..250 Hz; limits and stalls bounded");
}
static void logging(void) {
    const char *names[]={"penis","testicles","breasts","butt"};
    defaults_cfg.performance_profile=0;body_update_record_publish(0,3,1000,-1);assert(!records);
    defaults_cfg.performance_profile=1;
    for(int system=0;system<4;system++) {
        body_update_record_publish(0,system,1000,-1);
        for(int n=1;n<=450;n++)body_update_record_publish(0,system,1000+n*5000/450,-1);
        assert(records==system+1 && strstr(record,names[system]) && strstr(record,"published_hz=90.0"));
    }
    puts("PASS: all four systems report completed updates independently under performance_profile");
}
static void translation_gain(void) {
    body_chain_physics_config_t cfg=config();breasts_physics_person_state_t state={0};
    cfg.translation_deadzone=.0001f;
    for(int body=0;body<2;body++)for(int side=0;side<2;side++)for(int space=0;space<2;space++) {
        float ref[3],p[3]={.0016f,.00005f,-.0048f};
        breasts_physics_bone_translation_space=butt_physics_bone_translation_space=space;
        if(body)butt_physics_build_bone_translation_target(&cfg,&state,side,p,ref);
        else breasts_physics_build_bone_translation_target(&cfg,&state,side,p,ref);
        for(unsigned int ms=4;ms<=100;ms++) {
            float r[3]={0},out[3];p[0]=.0016f*ms/16;p[1]=.00005f*ms/16;p[2]=-.0048f*ms/16;
            paired_body_scale_motion(-1,ms,p,r);
            if(body)butt_physics_build_bone_translation_target(&cfg,&state,side,p,out);
            else breasts_physics_build_bone_translation_target(&cfg,&state,side,p,out);
            for(int a=0;a<3;a++)assert(fabsf(out[a]-ref[a])<1e-7f);
        }
    }
    puts("PASS: optional breast/butt bone translation keeps gain, signs and deadzones across sample rates and local/body mapping");
}
int main(void){legacy();rates();translation_gain();logging();return 0;}
'''
build=ROOT/'build';build.mkdir(exist_ok=True)
test=build/'paired_body_update_test.c';test.write_text(source)
gcc=Path(r'C:\msys64\mingw32\bin\gcc.exe')
env=dict(os.environ,PATH=str(gcc.parent)+os.pathsep+os.environ['PATH'])
exe=build/'paired_body_update_test.exe'
subprocess.run([str(gcc),'-m32','-O2','-Wall','-Wextra','-Werror','-Wno-unused-function','-static-libgcc','-o',str(exe),str(test)],env=env,check=True)
subprocess.run([str(exe)],env=env,check=True,timeout=30)
