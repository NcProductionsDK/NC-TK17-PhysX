#ifndef PHYSX_BODY_MOTION_H
#define PHYSX_BODY_MOTION_H
#include <math.h>
#include <string.h>

/* Preserve the gain and 60/28/12 smoothing of the established 16 ms update.
   Inputs are displacement over a sample interval, not per-substep impulses. */
#define BODY_MOTION_REFERENCE_MS 16u
#define BODY_MOTION_MAX_ELAPSED_MS 100u
#define BODY_MOTION_HISTORY 64
typedef struct body_motion_filter_t {
    float value[BODY_MOTION_HISTORY][3];
    unsigned int duration[BODY_MOTION_HISTORY];
    unsigned int next, count;
} body_motion_filter_t;

static float body_motion_duration(unsigned int elapsed_ms)
{
    if (!elapsed_ms) elapsed_ms=BODY_MOTION_REFERENCE_MS;
    if (elapsed_ms>BODY_MOTION_MAX_ELAPSED_MS) elapsed_ms=BODY_MOTION_MAX_ELAPSED_MS;
    return (float)elapsed_ms*.001f;
}

static int body_motion_substeps(float duration)
{
    /* Subtract float roundoff so an exact 16/32/48 ms interval does not gain
       an extra step. A long stall advances at most 100 ms, without a backlog. */
    int steps=(int)ceilf(duration/.016f-.00001f);
    return steps<1?1:(steps>7?7:steps);
}

static float body_motion_input_scale(unsigned int elapsed_ms)
{
    /* Rebase after stalls; never turn a teleport or startup into a kick. */
    if (!elapsed_ms || elapsed_ms>BODY_MOTION_MAX_ELAPSED_MS) return 0;
    return (float)BODY_MOTION_REFERENCE_MS/(float)elapsed_ms;
}

static void body_motion_spring_step(float *angle,float *velocity,
    float target,float stiffness,float damping,float dt)
{
    float acceleration=(target-*angle)*stiffness-*velocity*damping;
    *velocity+=acceleration*dt;
    *angle+=*velocity*dt;
}

/* A directional dashpot eases fast swings into each joint's hard stop. It
   adds no resting torque and does not resist movement away from that stop.
   Use the proposed step to begin braking before a fast swing crosses a limit.
   The braking band is 25% of the distance from neutral to that side's limit,
   capped at 12 degrees. With ranges excluding zero, use the range midpoint.
   Implicit damping cannot reverse or amplify the proposed velocity. */
static void body_motion_limit_brake(float angle,float *velocity,
    float minimum,float maximum,float stiffness,float dt)
{
    float neutral,extent,width,distance,proximity,rate;
    if (!(dt>0) || !(maximum>minimum) || *velocity==0) return;
    neutral=minimum<=0 && maximum>=0 ? 0 : (minimum+maximum)*.5f;
    if (*velocity>0) {
        extent=maximum-neutral;
        distance=maximum-(angle+*velocity*dt);
    } else {
        extent=neutral-minimum;
        distance=(angle+*velocity*dt)-minimum;
    }
    width=fminf(12.0f,extent*.25f);
    if (!(width>.0001f) || distance>=width) return;
    proximity=fminf(1.0f,fmaxf(0.0f,1.0f-distance/width));
    rate=4.0f*sqrtf(fmaxf(0.0f,stiffness))*proximity*proximity;
    *velocity/=1.0f+rate*dt;
}

static void body_motion_limited_spring_step(float *angle,float *velocity,
    float target,float stiffness,float damping,float dt,float minimum,float maximum)
{
    float before=*angle;
    body_motion_spring_step(angle,velocity,target,stiffness,damping,dt);
    {
        float proposed=*velocity;
        body_motion_limit_brake(before,velocity,minimum,maximum,stiffness,dt);
        if (*velocity!=proposed) *angle=before+*velocity*dt;
    }
    /* This final guard handles locked joints and extreme inputs. Contact
       correction still uses the full configured interval, with no soft cap. */
    if (*angle>=maximum) {
        *angle=maximum;
        if (*velocity>0) *velocity=0;
    }
    if (*angle<=minimum) {
        *angle=minimum;
        if (*velocity<0) *velocity=0;
    }
}

/* Average each of three fixed 16 ms windows, then apply the existing weights.
   Sample intervals are integers to avoid drift across millisecond tick wrap.
   Missing startup history represents zero motion. At 16 ms this is exactly
   the previous three-tap filter, while variable-rate samples cover real time. */
static void body_motion_filter_sample(body_motion_filter_t *filter,
    const float input[3],unsigned int elapsed_ms,int trusted,float output[3])
{
    static const float weights[3]={.60f,.28f,.12f};
    unsigned int n,age=0;int a,window;
    memset(output,0,sizeof(float)*3);
    if (!trusted || !elapsed_ms || elapsed_ms>BODY_MOTION_MAX_ELAPSED_MS ||
        !isfinite(input[0]) || !isfinite(input[1]) || !isfinite(input[2])) {
        memset(filter,0,sizeof(*filter));return;
    }
    memcpy(filter->value[filter->next],input,sizeof(float)*3);
    filter->duration[filter->next]=elapsed_ms;
    filter->next=(filter->next+1)%BODY_MOTION_HISTORY;
    if(filter->count<BODY_MOTION_HISTORY) filter->count++;
    for(n=0;n<filter->count && age<48u;n++) {
        unsigned int index=(filter->next+BODY_MOTION_HISTORY-1-n)%BODY_MOTION_HISTORY;
        unsigned int end=age+filter->duration[index];
        for(window=0;window<3;window++) {
            unsigned int start=(unsigned int)window*16u,stop=start+16u;
            unsigned int left=age>start?age:start,right=end<stop?end:stop;
            if(right>left) for(a=0;a<3;a++)
                output[a]+=filter->value[index][a]*weights[window]*(float)(right-left)/16.0f;
        }
        age=end;
    }
}
#endif
