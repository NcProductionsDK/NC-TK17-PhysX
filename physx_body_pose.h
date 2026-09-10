#ifndef PHYSX_BODY_POSE_H
#define PHYSX_BODY_POSE_H
#include "physx_contact_math.h"

typedef struct body_contact_pose_t {
    float origin[3],length[3],bind[3];
    float euler[3][3];
    int segments;
} body_contact_pose_t;

static void body_pose_multiply(const float a[9],const float b[9],float out[9])
{
    float result[9];int i,j,k;
    for(i=0;i<3;i++) for(j=0;j<3;j++) {
        float v=0;for(k=0;k<3;k++) v+=a[i*3+k]*b[k*3+j];
        result[i*3+j]=v;
    }
    memcpy(out,result,sizeof(result));
}

/* The collider frame is root Y/Z/X. Built-in joints extend along their local
   X axis after an authored planar orientation. Euler rotation is local to
   each joint, followed by that orientation and its parent's full transform. */
static int body_pose_evaluate(const body_contact_pose_t *pose,
    const float euler[3][3],float points[4][3])
{
    float parent[9]={0,0,-1,-1,0,0,0,1,0};int j,a;
    if(pose->segments<1 || pose->segments>3) return 0;
    memcpy(points[0],pose->origin,sizeof(float)*3);
    for(j=0;j<pose->segments;j++) {
        float rotation[9],bind[9],degrees[3]={0,0,pose->bind[j]};
        for(a=0;a<3;a++) if(!isfinite(euler[j][a])) return 0;
        physx_contact_rotation_rows(euler[j],rotation);
        physx_contact_rotation_rows(degrees,bind);
        body_pose_multiply(rotation,bind,rotation);
        body_pose_multiply(rotation,parent,parent);
        for(a=0;a<3;a++) points[j+1][a]=points[j][a]+pose->length[j]*parent[a];
    }
    for(;j<3;j++) memcpy(points[j+1],points[j],sizeof(float)*3);
    return 1;
}

/* Recover lengths and authored planar orientations from a coherent sample.
   The pitch agreement check rejects frames/rigs this model cannot describe;
   it is not an unrestricted fit that can hide a wrong rotation mapping. */
static int body_pose_fit(const float points[4][3],const float euler[3][3],
    int segments,body_contact_pose_t *out)
{
    body_contact_pose_t pose={0};
    float parent[9]={0,0,-1,-1,0,0,0,1,0};int j,a,k;
    if(segments<1 || segments>3) return 0;
    pose.segments=segments;memcpy(pose.origin,points[0],sizeof(pose.origin));
    memcpy(pose.euler,euler,sizeof(pose.euler));
    for(j=0;j<segments;j++) {
        float d[3],local[3]={0},length2=0,rotation[9],bind[9],degrees[3]={0};
        for(a=0;a<3;a++) {
            if(!isfinite(points[j][a]) || !isfinite(points[j+1][a]) || !isfinite(euler[j][a])) return 0;
            d[a]=points[j+1][a]-points[j][a];length2+=d[a]*d[a];
        }
        if(length2<.000001f || length2>.25f) return 0;
        pose.length[j]=sqrtf(length2);
        for(a=0;a<3;a++) for(k=0;k<3;k++) local[a]+=d[k]*parent[a*3+k]/pose.length[j];
        physx_contact_rotation_rows(euler[j],rotation);
        if(fabsf(local[2]-rotation[2])>.002f ||
           local[0]*local[0]+local[1]*local[1]<.0004f ||
           rotation[0]*rotation[0]+rotation[1]*rotation[1]<.0004f) return 0;
        pose.bind[j]=(atan2f(local[1],local[0])-atan2f(rotation[1],rotation[0]))*57.29577951308232f;
        degrees[2]=pose.bind[j];physx_contact_rotation_rows(degrees,bind);
        body_pose_multiply(rotation,bind,rotation);
        body_pose_multiply(rotation,parent,parent);
    }
    *out=pose;return 1;
}
#endif
