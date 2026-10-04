#ifndef PHYSX_DEBUG_GEOMETRY_H
#define PHYSX_DEBUG_GEOMETRY_H
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

/* Homogeneous coordinates preserve perspective and let D3D clip near-plane
   intersections instead of dropping whole colliders or clamping their size. */
typedef struct physx_wire_vertex { float clip[4]; uint32_t rgba; } physx_wire_vertex;
typedef struct physx_wire_batch {
    physx_wire_vertex *vertices;
    unsigned count, capacity, dropped;
    float matrix[16]; /* local-to-clip, row vector convention */
} physx_wire_batch;
#define PHYSX_WIRE_MAX_VERTICES 1048576u
#define PHYSX_WIRE_STEPS 32

static void physx_wire_transform(const float m[16], const float p[3], float q[4])
{
    unsigned i;
    for (i=0;i<4;++i) q[i]=p[0]*m[i]+p[1]*m[4+i]+p[2]*m[8+i]+m[12+i];
}
static unsigned physx_wire_outcode(const float p[4])
{
    return (p[0]<-p[3]) | ((p[0]>p[3])<<1) | ((p[1]<-p[3])<<2) |
        ((p[1]>p[3])<<3) | ((p[2]<0)<<4) | ((p[2]>p[3])<<5) | ((p[3]<=0)<<6);
}
static void physx_wire_line(physx_wire_batch *b, const float a[3], const float z[3], uint32_t argb)
{
    physx_wire_vertex v[2]; unsigned i;
    physx_wire_transform(b->matrix,a,v[0].clip);
    physx_wire_transform(b->matrix,z,v[1].clip);
    for(i=0;i<4;++i) if(!isfinite(v[0].clip[i]) || !isfinite(v[1].clip[i])) return;
    if(physx_wire_outcode(v[0].clip)&physx_wire_outcode(v[1].clip)) return;
    if(b->count+2>b->capacity) {
        unsigned capacity=b->capacity ? b->capacity*2 : 4096;
        void *data;
        if(capacity>PHYSX_WIRE_MAX_VERTICES) { ++b->dropped; return; }
        data=realloc(b->vertices,capacity*sizeof(*b->vertices));
        if(!data) { ++b->dropped; return; }
        b->vertices=(physx_wire_vertex *)data; b->capacity=capacity;
    }
    v[0].rgba=v[1].rgba=0xff000000u | ((argb&255)<<16) | (argb&0xff00) | ((argb>>16)&255);
    b->vertices[b->count++]=v[0]; b->vertices[b->count++]=v[1];
}
static const float (*physx_wire_circle(void))[2]
{
    static float points[PHYSX_WIRE_STEPS+1][2]; static int ready;
    unsigned i;
    if(!ready) {
        for(i=0;i<PHYSX_WIRE_STEPS;++i) {
            float angle=(float)i*(6.28318530718f/PHYSX_WIRE_STEPS);
            points[i][0]=cosf(angle); points[i][1]=sinf(angle);
        }
        memcpy(points[PHYSX_WIRE_STEPS],points[0],sizeof(points[0])); ready=1;
    }
    return points;
}
static void physx_wire_ellipsoid(physx_wire_batch *b, const float center[3],
    const float axes[3], uint32_t color)
{
    const float (*circle)[2]=physx_wire_circle(); unsigned plane,i,k;
    if(axes[0]<=0 || axes[1]<=0 || axes[2]<=0) return;
    for(plane=0;plane<3;++plane) {
        float last[3]; unsigned u=plane, v=(plane+1)%3;
        for(i=0;i<=PHYSX_WIRE_STEPS;++i) {
            float p[3]; for(k=0;k<3;++k) p[k]=center[k];
            p[u]+=axes[u]*circle[i][0]; p[v]+=axes[v]*circle[i][1];
            if(i) physx_wire_line(b,last,p,color);
            memcpy(last,p,sizeof(last));
        }
    }
}
static void physx_wire_capsule(physx_wire_batch *b, const float start[3],
    const float end[3], float r0, float r1, uint32_t color)
{
    const float (*circle)[2]=physx_wire_circle();
    float d[3],u[3]={0},v[3],len; unsigned i,k,side;
    if(r0<=0 || r1<=0) return;
    for(k=0;k<3;++k) d[k]=end[k]-start[k];
    len=sqrtf(d[0]*d[0]+d[1]*d[1]+d[2]*d[2]);
    if(len<1e-6f) {
        float r=r0>r1?r0:r1,axes[3]={r,r,r};
        physx_wire_ellipsoid(b,start,axes,color); return;
    }
    for(k=0;k<3;++k) d[k]/=len;
    /* Cross with the least aligned axis for a stable orthonormal frame. */
    i=fabsf(d[0])<fabsf(d[1])?0:1; if(fabsf(d[2])<fabsf(d[i])) i=2;
    u[(i+1)%3]=d[(i+2)%3]; u[(i+2)%3]=-d[(i+1)%3];
    len=sqrtf(u[0]*u[0]+u[1]*u[1]+u[2]*u[2]);
    for(k=0;k<3;++k) u[k]/=len;
    v[0]=d[1]*u[2]-d[2]*u[1]; v[1]=d[2]*u[0]-d[0]*u[2]; v[2]=d[0]*u[1]-d[1]*u[0];
    /* End rings plus four generators and hemispherical caps. The solver's
       tapered segments interpolate radius at the closest point on the axis. */
    for(side=0;side<2;++side) {
        float last[3],r=side?r1:r0; const float *center=side?end:start;
        for(i=0;i<=PHYSX_WIRE_STEPS;++i) {
            float p[3]; for(k=0;k<3;++k) p[k]=center[k]+r*(u[k]*circle[i][0]+v[k]*circle[i][1]);
            if(i) physx_wire_line(b,last,p,color); memcpy(last,p,sizeof(last));
        }
    }
    for(side=0;side<4;++side) {
        float radial[3],a[3],z[3];
        for(k=0;k<3;++k) {
            radial[k]=u[k]*circle[side*8][0]+v[k]*circle[side*8][1];
            a[k]=start[k]+r0*radial[k]; z[k]=end[k]+r1*radial[k];
        }
        physx_wire_line(b,a,z,color);
        for(i=1;i<=8;++i) {
            float p[3],q[3];
            for(k=0;k<3;++k) {
                p[k]=start[k]+r0*(radial[k]*circle[i][0]-d[k]*circle[i][1]);
                q[k]=end[k]+r1*(radial[k]*circle[i][0]+d[k]*circle[i][1]);
            }
            physx_wire_line(b,a,p,color); physx_wire_line(b,z,q,color);
            memcpy(a,p,sizeof(a)); memcpy(z,q,sizeof(z));
        }
    }
}
#endif
