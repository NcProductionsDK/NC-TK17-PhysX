#ifndef PHYSX_SINGLE_BONE_CONTACT_H
#define PHYSX_SINGLE_BONE_CONTACT_H
#include <math.h>
#include <string.h>

/* Absolute translation constraints n.x >= bound, in the output parent's
   coordinates. Separate from chain contact math and its length constraints. */
#define SINGLE_BONE_CONTACTS 32
typedef struct single_bone_contacts_t {
    int count;
    float n[SINGLE_BONE_CONTACTS][3], bound[SINGLE_BONE_CONTACTS];
} single_bone_contacts_t;

static float single_bone_dot(const float a[3],const float b[3])
{ return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]; }

static void single_bone_store(single_bone_contacts_t *s,const float n[3],float bound)
{
    int i,slot;
    float len=sqrtf(single_bone_dot(n,n));
    float unit[3];
    if(!isfinite(len) || len<1e-6f || !isfinite(bound)) return;
    for(i=0;i<3;i++) unit[i]=n[i]/len;
    bound/=len;
    for(i=0;i<s->count;i++) if(single_bone_dot(unit,s->n[i])>.9999f) {
        if(bound>s->bound[i]) s->bound[i]=bound;
        return;
    }
    if(s->count>=SINGLE_BONE_CONTACTS) return;
    slot=s->count++;memcpy(s->n[slot],unit,sizeof(unit));s->bound[slot]=bound;
}

/* A local displacement dx moves the sphere by dx * parent_to_world.
   Thus a world normal becomes a covector with components dot(row_i, n).
   Keep scale/reflections; normalizing the parent's rows would change distance. */
static void single_bone_world_plane(single_bone_contacts_t *s,
    const float parent_to_world[9],const float world_normal[3],float depth,
    const float position[3])
{
    float n[3];int a;
    for(a=0;a<3;a++) n[a]=single_bone_dot(parent_to_world+3*a,world_normal);
    single_bone_store(s,n,single_bone_dot(n,position)+depth);
}

static float single_bone_violation(const single_bone_contacts_t *s,const float x[3])
{
    float sum=0;int i;
    for(i=0;i<s->count;i++) {
        float d=fmaxf(0,s->bound[i]-single_bone_dot(s->n[i],x));sum+=d*d;
    }
    return sum;
}

/* Bounded dual projection with box constraints. For impossible combinations,
   retain the least-violating bounded iterate instead of the last oscillation.
   Positional recovery never changes velocity implicitly. */
static void single_bone_project(const single_bone_contacts_t *s,
    const float lo[3],const float hi[3],float x[3])
{
    float lambda[SINGLE_BONE_CONTACTS]={0},box_lo[3]={0},box_hi[3]={0};
    float best[3],error;int a,i,iteration;
    for(a=0;a<3;a++) x[a]=fminf(hi[a],fmaxf(lo[a],x[a]));
    memcpy(best,x,sizeof(best));error=single_bone_violation(s,x);
    if(error==0) return;
    for(iteration=0;iteration<64;iteration++) {
        float change_max=0;
        for(i=0;i<s->count;i++) {
            float next=fminf(10000.0f,fmaxf(0,lambda[i]+s->bound[i]-single_bone_dot(s->n[i],x)));
            float change=next-lambda[i];lambda[i]=next;
            change_max=fmaxf(change_max,fabsf(change));
            for(a=0;a<3;a++) x[a]+=s->n[i][a]*change;
        }
        for(a=0;a<3;a++) {
            float next=fmaxf(0,box_lo[a]+lo[a]-x[a]);
            x[a]+=next-box_lo[a];box_lo[a]=next;
            next=fmaxf(0,box_hi[a]+x[a]-hi[a]);
            x[a]-=next-box_hi[a];box_hi[a]=next;
            x[a]=fminf(hi[a],fmaxf(lo[a],x[a]));
        }
        float e=single_bone_violation(s,x);
        if(e<=error) {error=e;memcpy(best,x,sizeof(best));}
        if(change_max<1e-7f) break;
    }
    memcpy(x,best,sizeof(best));
}

static void single_bone_velocity(const single_bone_contacts_t *s,
    const float position[3],const float limit[3],float velocity[3])
{
    single_bone_contacts_t active={0};float lo[3],hi[3];int a,i;
    for(i=0;i<s->count;i++)
        if(single_bone_dot(s->n[i],position)-s->bound[i]<2e-5f)
            single_bone_store(&active,s->n[i],0);
    for(a=0;a<3;a++) {
        lo[a]=position[a]<=-limit[a]+1e-6f?0:-1000;
        hi[a]=position[a]>=limit[a]-1e-6f?0:1000;
    }
    single_bone_project(&active,lo,hi,velocity);
}
#endif
