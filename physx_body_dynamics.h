#ifndef PHYSX_BODY_DYNAMICS_H
#define PHYSX_BODY_DYNAMICS_H
#include "physx_body_pose.h"

/* Reduced rod-chain model. Relative mass is proportional to measured length;
   two Gauss points per rod integrate its rotational inertia, not just the
   midpoint's translational motion. Angular coordinates remain in degrees. */
typedef struct body_dynamics_t {
    body_contact_pose_t reference;
    float inverse_inertia[3][2];
    float gravity_lever_norm;
    int segments, axis[2];
} body_dynamics_t;

static int body_dynamics_geometry(const body_contact_pose_t *pose,
    const float euler[3][3],const int axis[2],float inertia[3][2],float lever[3][2][3])
{
    float total=0;int j,a,k,c,s;
    static const float location[2]={.2113248654f,.7886751346f};
    /* Gravity needs only the lever; reference preparation also requests inertia. */
    if(inertia) memset(inertia,0,sizeof(float)*6);
    memset(lever,0,sizeof(float)*18);
    if(pose->segments<1 || pose->segments>3 || axis[0]<0 || axis[0]>2 ||
       axis[1]<0 || axis[1]>2 || axis[0]==axis[1]) return 0;
    for(j=0;j<pose->segments;j++) {
        if(!isfinite(pose->length[j]) || pose->length[j]<.001f || pose->length[j]>.5f) return 0;
        total+=pose->length[j];
    }
    for(j=0;j<pose->segments;j++) for(a=0;a<2;a++) {
        float plus[3][3],minus[3][3],p[4][3],m[4][3];
        memcpy(plus,euler,sizeof(plus));memcpy(minus,euler,sizeof(minus));
        plus[j][axis[a]]+=.1f;minus[j][axis[a]]-=.1f;
        if(!body_pose_evaluate(pose,plus,p) || !body_pose_evaluate(pose,minus,m)) return 0;
        for(k=j;k<pose->segments;k++) for(s=0;s<2;s++) {
            float mass=.5f*pose->length[k]/total;
            for(c=0;c<3;c++) {
                float derivative=((p[k][c]-m[k][c])*(1-location[s])+
                    (p[k+1][c]-m[k+1][c])*location[s])/.2f;
                if(!isfinite(derivative)) return 0;
                if(inertia) inertia[j][a]+=mass*derivative*derivative;
                lever[j][a][c]+=mass*derivative;
            }
        }
    }
    return 1;
}

/* Reference geometry supplies stable effective inertias. The identity floor
   prevents tiny distal links from acquiring near-zero inertia. This is a
   diagonal approximation; it does not claim off-diagonal/Coriolis dynamics. */
static int body_dynamics_prepare(const body_contact_pose_t *pose,
    const float neutral[3][3],int horizontal,int vertical,body_dynamics_t *out)
{
    body_dynamics_t model={0};float inertia[3][2],lever[3][2][3],sum=0,norm2=0;
    int j,a,c,axis[2]={horizontal,vertical};
    if(!body_dynamics_geometry(pose,neutral,axis,inertia,lever)) return 0;
    for(j=0;j<pose->segments;j++) for(a=0;a<2;a++) {
        sum+=inertia[j][a];
        for(c=0;c<3;c++) norm2+=lever[j][a][c]*lever[j][a][c];
    }
    if(!(sum>1e-12f) || !(norm2>1e-12f)) return 0;
    model.segments=pose->segments;model.axis[0]=horizontal;model.axis[1]=vertical;
    model.reference=*pose;
    memcpy(model.reference.euler,neutral,sizeof(model.reference.euler));
    memset(model.reference.origin,0,sizeof(model.reference.origin));
    model.gravity_lever_norm=sqrtf(norm2);
    for(j=0;j<pose->segments;j++) for(a=0;a<2;a++)
        model.inverse_inertia[j][a]=1.0f/(.65f+.35f*inertia[j][a]*(2*pose->segments)/sum);
    *out=model;return 1;
}

/* Turn existing angular gravity strength into a distributed force, then
   project it through the current COM derivatives. Normalization uses total
   reference leverage, never its projection onto gravity (which vanishes for
   a hanging chain). The 50% transition retains familiar profile tuning while
   adding pose-dependent gravity. Zero configured drive stays exactly zero. */
static int body_dynamics_gravity(const body_contact_pose_t *pose,
    const body_dynamics_t *model,const float euler[3][3],const float direction[3],
    const float configured[3][3],float out[3][3])
{
    float lever[3][2][3],strength2=0,length2=0,scale;
    int j,a,c;
    memcpy(out,configured,sizeof(float)*9);
    if(model->segments!=pose->segments || !(model->gravity_lever_norm>1e-6f)) return 0;
    for(c=0;c<3;c++) {if(!isfinite(direction[c])) return 0;length2+=direction[c]*direction[c];}
    for(j=0;j<model->segments;j++) for(a=0;a<2;a++) {
        float v=configured[j][model->axis[a]];
        if(!isfinite(v)) return 0;
        strength2+=v*v;
    }
    if(strength2==0) return 1;
    if(!body_dynamics_geometry(pose,euler,model->axis,NULL,lever)) return 0;
    /* Preserve a smoothed direction's reduced magnitude through reversal;
       renormalizing a near-zero vector would cause an abrupt force flip. */
    scale=sqrtf(strength2)/(model->gravity_lever_norm*fmaxf(1.0f,sqrtf(length2)));
    for(j=0;j<model->segments;j++) for(a=0;a<2;a++) {
        float torque=0;
        for(c=0;c<3;c++) torque+=lever[j][a][c]*direction[c];
        out[j][model->axis[a]]=.5f*configured[j][model->axis[a]]+.5f*scale*torque;
    }
    return 1;
}
#endif
