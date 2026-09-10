"""Test production body timing, input reconstruction and angular integration."""
from pathlib import Path
import os
import subprocess

ROOT = Path(__file__).resolve().parent
source = r'''
#include <assert.h>
#include <stdio.h>
#include "../physx_body_motion.h"
static void timing(void) {
    unsigned int ms;
    for(ms=1;ms<=100;ms++) {
        float dt=body_motion_duration(ms);int n=body_motion_substeps(dt);
        assert(fabsf(dt-ms*.001f)<1e-7f && n>=1 && n<=7);
        assert(dt/n<=.016001f);
        if(ms<=16) assert(n==1);
    }
    assert(body_motion_substeps(body_motion_duration(32))==2);
    assert(body_motion_substeps(body_motion_duration(48))==3);
    assert(fabsf(body_motion_duration(5000)-.1f)<1e-7f);
    assert(body_motion_input_scale(0)==0 && body_motion_input_scale(101)==0);
    puts("PASS: ordinary 1-100 ms intervals preserved; substeps <=16 ms; stalls bounded with no stale movement impulse");
}
static void reference_filter(void) {
    body_motion_filter_t f={0};float old1[3]={0},old2[3]={0};int n,a;
    for(n=0;n<150;n++) {
        float input[3]={sinf(n*.3f),cosf(n*.2f),n*.001f},out[3];
        body_motion_filter_sample(&f,input,16,1,out);
        for(a=0;a<3;a++) {
            assert(fabsf(out[a]-(.6f*input[a]+.28f*old1[a]+.12f*old2[a]))<2e-7f);
            old2[a]=old1[a];old1[a]=input[a];
        }
    }
    puts("PASS: 16 ms input reconstruction matches the confirmed 60/28/12 filter across history wrap");
}
static void rates(void) {
    const unsigned int cadence[]={1,7,16,17,33,50,100};unsigned int c,n,a;
    for(c=0;c<sizeof(cadence)/sizeof(cadence[0]);c++) {
        body_motion_filter_t f={0};float output[3];unsigned int elapsed=0;
        for(n=0;n<200;n++) {
            unsigned int ms=cadence[c];
            float scale=body_motion_input_scale(ms);
            float input[3]={.2f*ms*.001f*scale,-.1f*ms*.001f*scale,.3f*ms*.001f*scale};
            body_motion_filter_sample(&f,input,ms,1,output);elapsed+=ms;
            if(elapsed>=48) {
                assert(fabsf(output[0]-.0032f)<2e-8f);
                assert(fabsf(output[1]+.0016f)<2e-8f);
                assert(fabsf(output[2]-.0048f)<2e-8f);
            }
        }
        {float zero[3]={0};body_motion_filter_sample(&f,zero,48,1,output);
        for(a=0;a<3;a++) assert(output[a]==0);}
    }
    /* Vary the sampling interval inside one motion, not just between runs. */
    {body_motion_filter_t f={0};unsigned int total=0;
    for(n=0;n<500;n++) {
        unsigned int ms=cadence[n%7];float input[3]={.0032f,0,0},out[3];
        body_motion_filter_sample(&f,input,ms,1,out);total+=ms;
        if(total>=48) assert(fabsf(out[0]-.0032f)<2e-8f);
    }}
    puts("PASS: steady body velocity has identical gain across fixed/variable sample rates; smoothing clears after 48 ms");
}
static void invalid_and_camera(void) {
    body_motion_filter_t f={0};float input[3]={1,2,3},out[3];int a;
    body_motion_filter_sample(&f,input,16,1,out);
    body_motion_filter_sample(&f,input,16,0,out);
    for(a=0;a<3;a++) assert(out[a]==0);
    assert(f.count==0);
    body_motion_filter_sample(&f,input,16,1,out);assert(fabsf(out[0]-.6f)<1e-7f);
    body_motion_filter_sample(&f,input,1000,1,out);assert(f.count==0);
    input[0]=NAN;body_motion_filter_sample(&f,input,16,1,out);assert(f.count==0);
    puts("PASS: untrusted camera inputs, stalls and invalid samples clear history; trusted movement resumes immediately");
}
static void springs(void) {
    float angle=3,velocity=-2,old_angle=3,old_velocity=-2;int n;
    for(n=0;n<1000;n++) {
        float target=sinf(n*.03f)*10;
        float accel=(target-old_angle)*100-old_velocity*8;
        old_velocity+=accel*.016f;old_angle+=old_velocity*.016f;
        body_motion_spring_step(&angle,&velocity,target,100,8,.016f);
        assert(angle==old_angle && velocity==old_velocity);
    }
    for(int cadence=0;cadence<5;cadence++) {
        unsigned int elapsed=0;float simulated=0;angle=velocity=0;
        while(elapsed<2000) {
            unsigned int ms=cadence==0?16:cadence==1?33:cadence==2?50:cadence==3?7:(elapsed%2?50:16);
            if(ms>2000-elapsed)ms=2000-elapsed;
            float dt=body_motion_duration(ms);int count=body_motion_substeps(dt);
            for(n=0;n<count;n++) body_motion_spring_step(&angle,&velocity,20,100,8,dt/count);
            simulated+=dt;elapsed+=ms;
        }
        assert(fabsf(simulated-2)<.00001f && fabsf(angle-20)<.02f && fabsf(velocity)<.1f);
    }
    puts("PASS: established 16 ms angular step unchanged; slow/fast/variable schedules advance full time and converge to the same target");
}
static void limit_properties(void) {
    const float bounds[][2]={{-7.5f,7.5f},{-65,20},{-180,180},{10,30},{-30,-10}};
    int b,n;
    for(b=0;b<5;b++) for(n=0;n<1000;n++) {
        float lo=bounds[b][0],hi=bounds[b][1];
        float q=lo+(hi-lo)*(n%101)/100.0f;
        float v=(n%2?1:-1)*(1+n%300),before=v;
        body_motion_limit_brake(q,&v,lo,hi,100,.016f);
        assert(isfinite(v) && v*before>=0 && fabsf(v)<=fabsf(before));
    }
    /* The center and movement away from either nearby stop are unaffected. */
    {float a=0,v=2,expected_a=a,expected_v=v;
    body_motion_spring_step(&expected_a,&expected_v,1,100,8,.016f);
    body_motion_limited_spring_step(&a,&v,1,100,8,.016f,-20,20);
    assert(a==expected_a && v==expected_v);}
    {float v=-10;body_motion_limit_brake(19,&v,-20,20,100,.016f);assert(v==-10);
    v=10;body_motion_limit_brake(-19,&v,-20,20,100,.016f);assert(v==10);}
    /* Braking begins continuously at the band boundary. */
    {float v=10,q=15-.16f;
    body_motion_limit_brake(q,&v,-20,20,100,.016f);assert(v==10);
    v=10;body_motion_limit_brake(q+.0001f,&v,-20,20,100,.016f);assert(fabsf(v-10)<.00001f);}
    /* All stationary targets, including either limit, keep their equilibrium. */
    for(n=0;n<=100;n++) {
        float a=-7.5f+15*n/100.0f,v=0,target=a;
        body_motion_limited_spring_step(&a,&v,target,115,4,.016f,-7.5f,7.5f);
        assert(a==target && v==0);
    }
    {float a=2,v=-100;body_motion_limited_spring_step(&a,&v,1,100,8,.016f,1,1);
    assert(a==1 && v==0);}
    puts("PASS: limit braking only dissipates energy; unchanged center, inward release, equilibrium and locked-joint behavior");
}
static float limit_impact(int softened,float dt,float maximum,float *maximum_speed) {
    float angle=0,velocity=0,impact=0;int n;
    *maximum_speed=0;
    for(n=0;n<2000;n++) {
        float proposed=angle,v=velocity;
        body_motion_spring_step(&proposed,&v,maximum,115,4,dt);
        if(softened) body_motion_limit_brake(angle,&v,-maximum,maximum,115,dt);
        *maximum_speed=fmaxf(*maximum_speed,fabsf(v));
        if(angle+v*dt>=maximum){impact=fabsf(v);break;}
        if(softened) body_motion_limited_spring_step(&angle,&velocity,maximum,115,4,dt,-maximum,maximum);
        else {angle=proposed;velocity=v;}
        assert(isfinite(angle) && isfinite(velocity));
    }
    return impact;
}
static void limit_swings(void) {
    const float rates[]={20,30,60,144};int r;
    for(r=0;r<4;r++) {
        float duration=1/rates[r],dt=duration/body_motion_substeps(duration);
        float old_peak,new_peak;
        float old_impact=limit_impact(0,dt,7.5f,&old_peak);
        float new_impact=limit_impact(1,dt,7.5f,&new_peak);
        printf("LIMIT_IMPACT hz=%.0f old=%g new=%g peak_old=%g peak_new=%g\n",rates[r],old_impact,new_impact,old_peak,new_peak);
        assert(old_impact>1 && new_impact<old_impact*.6f);
        assert(new_peak>.7f*old_peak); /* Preserve the free portion of the swing. */
    }
    for(r=0;r<4;r++) {
        float dt=(1/rates[r])/body_motion_substeps(1/rates[r]),a=0,v=0;
        for(int n=0;n<1000;n++) body_motion_limited_spring_step(&a,&v,7.5f,115,4,dt,-7.5f,7.5f);
        assert(fabsf(a-7.5f)<.001f && fabsf(v)<.01f);
        body_motion_limited_spring_step(&a,&v,-7.5f,115,4,dt,-7.5f,7.5f);
        assert(a<7.5f && v<0); /* Respond on the very first reversal. */
        for(int n=0;n<2000;n++) {
            float target=n<1000?-7.5f:7.5f;
            body_motion_limited_spring_step(&a,&v,target,200,0,dt,-7.5f,7.5f);
            assert(isfinite(a) && isfinite(v) && a>=-7.5f && a<=7.5f);
        }
    }
    puts("PASS: narrow-root limit impact reduced across update rates, full range reachable and reversal remains immediate");
}
int main(void){timing();reference_filter();rates();invalid_and_camera();springs();limit_properties();limit_swings();return 0;}
'''
build = ROOT / 'build'
build.mkdir(exist_ok=True)
test = build / 'body_motion_test.c'
test.write_text(source)
gcc = Path(r'C:\msys64\mingw32\bin\gcc.exe')
env = dict(os.environ, PATH=str(gcc.parent)+os.pathsep+os.environ['PATH'])
exe = build / 'body_motion_test.exe'
subprocess.run([str(gcc), '-m32', '-O2', '-Wall', '-Wextra', '-Werror',
                '-static-libgcc', '-o', str(exe), str(test)], env=env, check=True)
subprocess.run([str(exe)], env=env, check=True, timeout=30)
