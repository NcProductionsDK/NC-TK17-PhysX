
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static void physx_contact_rotation_rows(const float degrees[3], float rows[9])
{
    double x=degrees[0]*0.017453292519943295;
    double y=degrees[1]*0.017453292519943295;
    double z=degrees[2]*0.017453292519943295;
    double sx=sin(x),cx=cos(x),sy=sin(y),cy=cos(y),sz=sin(z),cz=cos(z);
    rows[0]=(float)(cy*cz); rows[1]=(float)(cy*sz); rows[2]=(float)-sy;
    rows[3]=(float)(sx*sy*cz-cx*sz); rows[4]=(float)(sx*sy*sz+cx*cz); rows[5]=(float)(sx*cy);
    rows[6]=(float)(cx*sy*cz+sx*sz); rows[7]=(float)(cx*sy*sz-sx*cz); rows[8]=(float)(cx*cy);
}
#ifndef PHYSX_BODY_POSE_H
#define PHYSX_BODY_POSE_H


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

#ifndef PHYSX_BODY_DYNAMICS_H
#define PHYSX_BODY_DYNAMICS_H


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

#define BODY_CHAIN_MAX_CONTACTS 24
typedef unsigned long DWORD;
static float physx_clampf(float v,float a,float b){return fmaxf(a,fminf(b,v));}
static float physx_absf(float v){return fabsf(v);}
static int sane_probe_float(float v){return isfinite(v);}
static float vec3_dot(const float *a,const float *b){return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];}
static float physx_vec3_len(const float *v){return sqrtf(vec3_dot(v,v));}
typedef struct {int horizontal_output_axis,vertical_output_axis,rotation_tail_axis[3],output_offset,collision_scope;float link_min_angle[3][3],link_max_angle[3][3];} body_chain_physics_config_t;
static body_chain_physics_config_t body_chain_physics_cfg;
static struct {float response_strength,response_max_degrees_per_tick;int collision_iterations;float link_length[3];} body_chain_collider_cfg={1,20,2,{.5f,.5f,.5f}};
typedef struct {
    float angle[3][3],velocity[3][3],collision_step_points[4][3],collision_step_angle[3][3];
    DWORD collision_step_tick;
    float collision_step_dt;
    int collision_step_valid;
    int collision_step_engine_points;
    body_contact_pose_t collision_pose;
    int collision_pose_valid;
    body_dynamics_t dynamics;
    int dynamics_valid;
    int gravity_camera_hold_active,initialized,active_logged;
    void *joint_raw[3];
    float collision_contact_direction[3][2],collision_prev_max_penetration;
    int collision_rest_valid,collision_rest_ticks,collision_rest_grace_ticks,collision_impact_ticks,collision_multi_support_grace_ticks;
} body_chain_person_state_t;
typedef struct {
    float testicle_joint_position[3][3];
    int testicle_points_ready;
    DWORD testicle_points_update_tick;
    float chain_local_point[4][3];
    int chain_points_ready,chain_points_fresh,chain_point_valid[4];
    DWORD chain_points_update_tick;
    int ready,basis_valid;
} body_chain_collider_person_state_t;
#define BODY_MOTION_REFERENCE_MS 16u
#define BODY_MOTION_MAX_ELAPSED_MS 100u
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
static void body_motion_spring_step(float *angle,float *velocity,
    float target,float stiffness,float damping,float dt)
{
    float acceleration=(target-*angle)*stiffness-*velocity*damping;
    *velocity+=acceleration*dt;
    *angle+=*velocity*dt;
}
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
#define BODY_CHAIN_ENGINE_POINT_STALE_MS 500
#define BODY_CHAIN_COLLISION_POINT_HOLD_MS 80
typedef struct body_chain_contact_t {
    int segment;
    float segment_t;
    float penetration;
    float target_sep;
    float chain[3];
    float body[3];
    float normal[3];
} body_chain_contact_t;static float body_chain_link_axis_limit(const body_chain_physics_config_t *cfg,
                                        int link,
                                        int output_axis)
{
    if (!cfg || link < 0 || link >= 3 ||
        output_axis < 0 || output_axis > 2) {
        return 1.0f;
    }
    return physx_clampf(cfg->link_max_angle[link][output_axis],
                        1.0f, 360.0f);
}
static float body_chain_link_axis_min_limit(
    const body_chain_physics_config_t *cfg,
    int link,
    int output_axis)
{
    float max_angle;
    if (!cfg || link < 0 || link >= 3 ||
        output_axis < 0 || output_axis > 2) {
        return -1.0f;
    }
    max_angle = body_chain_link_axis_limit(cfg, link, output_axis);
    return physx_clampf(cfg->link_min_angle[link][output_axis],
                        -360.0f, max_angle);
}
static float body_chain_clamp_link_axis_angle(
    const body_chain_physics_config_t *cfg,
    int link,
    int output_axis,
    float angle)
{
    return physx_clampf(angle,
                        body_chain_link_axis_min_limit(cfg, link, output_axis),
                        body_chain_link_axis_limit(cfg, link, output_axis));
}
static void body_chain_store_contact(body_chain_contact_t *contacts,
                                     int *contact_count,
                                     int max_contacts,
                                     int segment,
                                     float segment_t,
                                     float penetration,
                                     const float chain[3],
                                     const float body[3],
                                     const float normal[3])
{
    int slot = -1;
    int i;
    int segment_contact_count = 0;
    int weakest_segment_slot = -1;
    const float normal_merge_dot = 0.995f;
    if (!contacts || !contact_count || max_contacts <= 0 ||
        !chain || !body || !normal || penetration <= 0.0f ||
        segment < 0 || segment >= 3) {
        return;
    }

    /* Build a compact contact manifold. Adjacent body capsules overlap by
       design; treating every primitive hit as a separate constraint makes
       those overlaps fight over the same chain segment at rest. */
    for (i = 0; i < *contact_count; i++) {
        float normal_dot;
        if (contacts[i].segment != segment) continue;
        segment_contact_count++;
        if (weakest_segment_slot < 0 ||
            contacts[i].penetration <
                contacts[weakest_segment_slot].penetration) {
            weakest_segment_slot = i;
        }
        normal_dot =
            contacts[i].normal[0] * normal[0] +
            contacts[i].normal[1] * normal[1] +
            contacts[i].normal[2] * normal[2];
        if (normal_dot >= normal_merge_dot &&
            fabsf(contacts[i].segment_t - segment_t) < 0.025f) {
            if (penetration <= contacts[i].penetration) return;
            slot = i;
            break;
        }
    }

    /* Retain different lever arms and crease supports; only near-identical
       constraints above are redundant. Bound storage per segment. */
    if (slot < 0 && segment_contact_count >= 8) {
        if (weakest_segment_slot < 0 ||
            penetration <= contacts[weakest_segment_slot].penetration) {
            return;
        }
        slot = weakest_segment_slot;
    }

    if (slot < 0 && *contact_count < max_contacts) {
        slot = *contact_count;
        (*contact_count)++;
    } else if (slot < 0) {
        int weakest = 0;
        for (i = 1; i < max_contacts; i++) {
            if (contacts[i].penetration < contacts[weakest].penetration) {
                weakest = i;
            }
        }
        if (penetration <= contacts[weakest].penetration) return;
        slot = weakest;
    }
    contacts[slot].segment = segment;
    contacts[slot].segment_t = physx_clampf(segment_t, 0.0f, 1.0f);
    contacts[slot].penetration = penetration;
    contacts[slot].target_sep =
        (chain[0] - body[0]) * normal[0] +
        (chain[1] - body[1]) * normal[1] +
        (chain[2] - body[2]) * normal[2] +
        penetration;
    for (i = 0; i < 3; i++) {
        contacts[slot].chain[i] = chain[i];
        contacts[slot].body[i] = body[i];
        contacts[slot].normal[i] = normal[i];
    }
}
static int body_chain_testicle_collision_points_local(
    const body_chain_collider_person_state_t *collider_state,
    const body_chain_person_state_t *chain_state,
    float points[4][3], int *engine_points, DWORD now)
{
    int i,a;
    if (engine_points) *engine_points=0;
    if (!points) return 0;
    memset(points,0,sizeof(float)*12);
    if (!collider_state || !chain_state || !collider_state->testicle_points_ready ||
        !collider_state->testicle_points_update_tick ||
        now-collider_state->testicle_points_update_tick>BODY_CHAIN_ENGINE_POINT_STALE_MS) return 0;
    for(i=0;i<3;i++) for(a=0;a<3;a++) {
        float value=collider_state->testicle_joint_position[i][a];
        if (!sane_probe_float(value)) return 0;
        points[i][a]=value;
    }
    /* Two physical segments; the unused fourth point has no extension. */
    memcpy(points[3],points[2],sizeof(float)*3);
    if(engine_points) *engine_points=1;
    return 1;
}
static int body_chain_simulated_points_local(
    const body_chain_person_state_t *chain_state,
    const body_chain_physics_config_t *cfg,
    const float angle_delta[3][2],
    float points[4][3])
{
    const float deg_to_rad = 0.01745329251994329577f;
    float pos[3] = { 0.0f, 0.0f, 0.0f };
    float cumulative_h = 0.0f;
    float cumulative_v = 0.0f;
    int i;

    if (!chain_state || !cfg || !points) return 0;
    memset(points, 0, sizeof(float) * 4 * 3);

    for (i = 0; i < 3; i++) {
        float len = body_chain_collider_cfg.link_length[i];
        float h_rad;
        float v_rad;
        float cv;
        cumulative_h += chain_state->angle[i][cfg->rotation_tail_axis[0]] +
            (angle_delta ? angle_delta[i][0] : 0.0f);
        cumulative_v += chain_state->angle[i][cfg->rotation_tail_axis[1]] +
            (angle_delta ? angle_delta[i][1] : 0.0f);
        h_rad = cumulative_h * deg_to_rad;
        v_rad = cumulative_v * deg_to_rad;
        cv = (float)cos((double)v_rad);
        pos[0] += -len * cv * (float)cos((double)h_rad);
        pos[1] +=  len * (float)sin((double)v_rad);
        pos[2] +=  len * cv * (float)sin((double)h_rad);
        points[i + 1][0] = pos[0];
        points[i + 1][1] = pos[1];
        points[i + 1][2] = pos[2];
    }
    return 1;
}
static int body_chain_engine_points_plausible(const float local[4][3])
{
    float seg_len[3] = {0.0f, 0.0f, 0.0f};
    float total = 0.0f;
    int i;
    if (!local) return 0;
    if (physx_vec3_len(local[0]) > 0.030f) return 0;
    for (i = 0; i < 4; i++) {
        if (!sane_probe_float(local[i][0]) ||
            !sane_probe_float(local[i][1]) ||
            !sane_probe_float(local[i][2])) {
            return 0;
        }
        if (physx_absf(local[i][0]) > 0.50f ||
            physx_absf(local[i][1]) > 0.50f ||
            physx_absf(local[i][2]) > 0.50f) {
            return 0;
        }
    }
    for (i = 1; i < 4; i++) {
        float dx = local[i][0] - local[i - 1][0];
        float dy = local[i][1] - local[i - 1][1];
        float dz = local[i][2] - local[i - 1][2];
        float len = (float)sqrt((double)(dx * dx + dy * dy + dz * dz));
        if (len < 0.003f || len > 0.180f) return 0;
        seg_len[i - 1] = len;
        total += len;
    }
    if (seg_len[0] < 0.035f && seg_len[1] > 0.120f) return 0;
    if (seg_len[0] > 0.0001f &&
        seg_len[1] > 0.100f &&
        seg_len[1] / seg_len[0] > 3.25f) {
        return 0;
    }
    if (seg_len[2] > 0.0001f &&
        seg_len[1] > 0.120f &&
        seg_len[1] / seg_len[2] > 3.50f) {
        return 0;
    }
    return total >= 0.060f && total <= 0.360f;
}
static int body_chain_collision_points_local(
    const body_chain_collider_person_state_t *collider_state,
    const body_chain_person_state_t *chain_state,
    float points[4][3],
    int *engine_points,
    DWORD now)
{
    int i;
    int live_points_fresh = 0;

    if (!points) return 0;
    memset(points, 0, sizeof(float) * 4 * 3);
    if (engine_points) *engine_points = 0;

    if (collider_state && collider_state->chain_points_ready) {
        live_points_fresh =
            !now ||
            (collider_state->chain_points_update_tick &&
             now - collider_state->chain_points_update_tick <=
                (collider_state->chain_points_fresh ?
                 BODY_CHAIN_ENGINE_POINT_STALE_MS :
                 BODY_CHAIN_COLLISION_POINT_HOLD_MS));
    }

    if (collider_state && collider_state->chain_points_ready &&
        live_points_fresh) {
        for (i = 0; i < 4; i++) {
            if (!collider_state->chain_point_valid[i]) break;
            points[i][0] = collider_state->chain_local_point[i][0];
            points[i][1] = collider_state->chain_local_point[i][1];
            points[i][2] = collider_state->chain_local_point[i][2];
        }
        if (i == 4) {
            if (engine_points) {
                *engine_points = collider_state->chain_points_fresh ? 1 : 2;
            }
            return 1;
        }
    }

    if (!body_chain_simulated_points_local(chain_state,
                                           &body_chain_physics_cfg,
                                           NULL, points)) {
        return 0;
    }
    return body_chain_engine_points_plausible(points);
}
static void body_chain_closest_segment_pair(const float p1[3],
                                            const float q1[3],
                                            const float p2[3],
                                            const float q2[3],
                                            float *s_out,
                                            float *t_out,
                                            float c1[3],
                                            float c2[3],
                                            float *dist_out)
{
    const float eps = 0.000001f;
    float d1[3] = { q1[0] - p1[0], q1[1] - p1[1], q1[2] - p1[2] };
    float d2[3] = { q2[0] - p2[0], q2[1] - p2[1], q2[2] - p2[2] };
    float r[3] = { p1[0] - p2[0], p1[1] - p2[1], p1[2] - p2[2] };
    float a = d1[0] * d1[0] + d1[1] * d1[1] + d1[2] * d1[2];
    float e = d2[0] * d2[0] + d2[1] * d2[1] + d2[2] * d2[2];
    float f = d2[0] * r[0] + d2[1] * r[1] + d2[2] * r[2];
    float s = 0.0f;
    float t = 0.0f;
    float dx, dy, dz;

    if (a <= eps && e <= eps) {
        s = 0.0f;
        t = 0.0f;
    } else if (a <= eps) {
        s = 0.0f;
        t = physx_clampf(f / e, 0.0f, 1.0f);
    } else {
        float c = d1[0] * r[0] + d1[1] * r[1] + d1[2] * r[2];
        if (e <= eps) {
            t = 0.0f;
            s = physx_clampf(-c / a, 0.0f, 1.0f);
        } else {
            float b = d1[0] * d2[0] + d1[1] * d2[1] + d1[2] * d2[2];
            float denom = a * e - b * b;
            if (denom != 0.0f) {
                s = physx_clampf((b * f - c * e) / denom, 0.0f, 1.0f);
            } else {
                s = 0.0f;
            }
            t = (b * s + f) / e;
            if (t < 0.0f) {
                t = 0.0f;
                s = physx_clampf(-c / a, 0.0f, 1.0f);
            } else if (t > 1.0f) {
                t = 1.0f;
                s = physx_clampf((b - c) / a, 0.0f, 1.0f);
            }
        }
    }

    c1[0] = p1[0] + d1[0] * s;
    c1[1] = p1[1] + d1[1] * s;
    c1[2] = p1[2] + d1[2] * s;
    c2[0] = p2[0] + d2[0] * t;
    c2[1] = p2[1] + d2[1] * t;
    c2[2] = p2[2] + d2[2] * t;
    dx = c1[0] - c2[0];
    dy = c1[1] - c2[1];
    dz = c1[2] - c2[2];
    if (s_out) *s_out = s;
    if (t_out) *t_out = t;
    if (dist_out) *dist_out = (float)sqrt((double)(dx * dx + dy * dy + dz * dz));
}
static float body_collider_wrap_degrees(float value)
{
    while (value > 180.0f) value -= 360.0f;
    while (value < -180.0f) value += 360.0f;
    return value;
}
static void body_collider_rotate_local_vector(const float in[3],
                                              const float rotation_delta[3],
                                              float out[3])
{
    const float deg_to_rad = 0.01745329251994329577f;
    int h_axis = body_chain_physics_cfg.horizontal_output_axis;
    int v_axis = body_chain_physics_cfg.vertical_output_axis;
    int roll_axis = 0;
    float h;
    float v;
    float roll;
    float x = in[0];
    float y = in[1];
    float z = in[2];
    float c;
    float s;
    float next;

    while (roll_axis == h_axis || roll_axis == v_axis) roll_axis++;
    if (roll_axis > 2) roll_axis = 0;
    h = body_collider_wrap_degrees(rotation_delta[h_axis]) * deg_to_rad;
    v = body_collider_wrap_degrees(rotation_delta[v_axis]) * deg_to_rad;
    roll = body_collider_wrap_degrees(rotation_delta[roll_axis]) * deg_to_rad;

    c = (float)cos((double)roll);
    s = (float)sin((double)roll);
    next = y * c - z * s;
    z = y * s + z * c;
    y = next;

    c = (float)cos((double)h);
    s = (float)sin((double)h);
    next = x * c + z * s;
    z = -x * s + z * c;
    x = next;

    c = (float)cos((double)-v);
    s = (float)sin((double)-v);
    next = x * c - y * s;
    y = x * s + y * c;
    x = next;

    out[0] = x;
    out[1] = y;
    out[2] = z;
}
static void body_collider_rotate_point_about_pivot(const float point[3],
                                                   const float pivot[3],
                                                   const float rotation_delta[3],
                                                   float out[3])
{
    float relative[3];
    float rotated[3];
    relative[0] = point[0] - pivot[0];
    relative[1] = point[1] - pivot[1];
    relative[2] = point[2] - pivot[2];
    body_collider_rotate_local_vector(relative, rotation_delta, rotated);
    out[0] = pivot[0] + rotated[0];
    out[1] = pivot[1] + rotated[1];
    out[2] = pivot[2] + rotated[2];
}
static int body_chain_live_points_with_delta_cfg(
    const body_chain_physics_config_t *cfg,
    const float live_points[4][3],
    const float angle_delta[3][2],
    float out[4][3])
{
    int joint;
    int point_index;
    if (!live_points || !out) {
        return 0;
    }
    memcpy(out, live_points, sizeof(float) * 4 * 3);

    if (angle_delta) {
        for (joint = 0; joint < 3; joint++) {
            float rot[3] = { 0.0f, 0.0f, 0.0f };
            int h_axis = cfg ?
                cfg->horizontal_output_axis :
                body_chain_physics_cfg.horizontal_output_axis;
            int v_axis = cfg ?
                cfg->vertical_output_axis :
                body_chain_physics_cfg.vertical_output_axis;
            if (h_axis >= 0 && h_axis < 3) {
                rot[h_axis] += angle_delta[joint][0];
            }
            if (v_axis >= 0 && v_axis < 3) {
                rot[v_axis] += angle_delta[joint][1];
            }
            if (physx_absf(rot[0]) < 0.000001f &&
                physx_absf(rot[1]) < 0.000001f &&
                physx_absf(rot[2]) < 0.000001f) {
                continue;
            }
            for (point_index = joint + 1; point_index < 4; point_index++) {
                float rotated[3];
                body_collider_rotate_point_about_pivot(
                    out[point_index], out[joint], rot, rotated);
                out[point_index][0] = rotated[0];
                out[point_index][1] = rotated[1];
                out[point_index][2] = rotated[2];
            }
        }
    }

    for (point_index = 0; point_index < 4; point_index++) {
        if (!sane_probe_float(out[point_index][0]) ||
            !sane_probe_float(out[point_index][1]) ||
            !sane_probe_float(out[point_index][2])) {
            return 0;
        }
    }
    return 1;
}
static float body_contact_inverse_inertia(const body_chain_person_state_t *state,int joint,int axis)
{
    if(state->dynamics_valid && joint<state->dynamics.segments)
        return state->dynamics.inverse_inertia[joint][axis];
    return 1.0f;
}
static int body_contact_predict(const body_chain_physics_config_t *cfg,
    const body_chain_person_state_t *state,const float base[4][3],
    const float correction[3][2],float points[4][3])
{
    COLLISION_PROFILE_COUNT(CP_BODY_PREDICTIONS, 1);
    if(state->collision_step_valid && state->collision_pose_valid) {
        float euler[3][3];int j,a;
        for(j=0;j<3;j++) for(a=0;a<3;a++) {
            euler[j][a]=state->collision_pose.euler[j][a]+state->angle[j][a]-state->collision_step_angle[j][a];
            if(correction && a==cfg->horizontal_output_axis) euler[j][a]+=correction[j][0];
            if(correction && a==cfg->vertical_output_axis) euler[j][a]+=correction[j][1];
        }
        return body_pose_evaluate(&state->collision_pose,euler,points);
    }
    return body_chain_live_points_with_delta_cfg(cfg,base,correction,points);
}
static int body_contact_candidate_points(
    const body_chain_person_state_t *state,
    const body_chain_physics_config_t *cfg, DWORD now, float points[4][3])
{
    float delta[3][2]={{0}};
    int i;
    if (!state->collision_step_valid || state->collision_step_tick != now) return 0;
    if(state->collision_pose_valid) return body_contact_predict(cfg,state,state->collision_step_points,NULL,points);
    for (i=0;i<3;i++) {
        delta[i][0]=state->angle[i][cfg->horizontal_output_axis]-
                    state->collision_step_angle[i][cfg->horizontal_output_axis];
        delta[i][1]=state->angle[i][cfg->vertical_output_axis]-
                    state->collision_step_angle[i][cfg->vertical_output_axis];
    }
    return body_chain_live_points_with_delta_cfg(cfg,state->collision_step_points,delta,points);
}
static void body_contact_point(const float points[4][3], const body_chain_contact_t *contact,
                                float out[3])
{
    int a;
    for(a=0;a<3;a++) out[a]=points[contact->segment][a]+contact->segment_t*
        (points[contact->segment+1][a]-points[contact->segment][a]);
}
static void body_contact_jacobian(const body_chain_physics_config_t *cfg,
    const body_chain_person_state_t *state, const float base[4][3],
    const float correction[3][2], const body_chain_contact_t *contact,
    const float direction[3], float jac[3][2], int respect_limits)
{
    COLLISION_PROFILE_COUNT(CP_BODY_JACOBIANS, 1);
    int j,a;
    memset(jac,0,sizeof(float)*6);
    for(j=0;j<=contact->segment;j++) for(a=0;a<2;a++) {
        float plus[3][2],minus[3][2],pp[4][3],mp[4][3],p[3],m[3],value=0;
        int k,axis=a?cfg->vertical_output_axis:cfg->horizontal_output_axis;
        float angle=state->angle[j][axis]+correction[j][a];
        memcpy(plus,correction,sizeof(plus));memcpy(minus,correction,sizeof(minus));
        plus[j][a]+=.1f;minus[j][a]-=.1f;
        body_contact_predict(cfg,state,base,plus,pp);
        body_contact_predict(cfg,state,base,minus,mp);
        body_contact_point(pp,contact,p);body_contact_point(mp,contact,m);
        for(k=0;k<3;k++) value+=(p[k]-m[k])*direction[k]/.2f;
        if(respect_limits && ((value>0 && body_chain_clamp_link_axis_angle(cfg,j,axis,angle+.01f)<=angle+.00001f) ||
           (value<0 && body_chain_clamp_link_axis_angle(cfg,j,axis,angle-.01f)>=angle-.00001f))) value=0;
        jac[j][a]=value;
    }
}
static void body_contact_pose_error(const float before[4][3],
    const float predicted[4][3],const float observed[4][3],int segments,float result[3])
{
    int i,a;memset(result,0,sizeof(float)*3);
    for(i=1;i<=segments;i++) {
        float error=0,expected=0,actual=0;
        for(a=0;a<3;a++) {
            float e=observed[i][a]-predicted[i][a];
            float p=predicted[i][a]-before[i][a];
            float o=observed[i][a]-before[i][a];
            error+=e*e;expected+=p*p;actual+=o*o;
        }
        result[0]=fmaxf(result[0],sqrtf(error));
        result[1]=fmaxf(result[1],sqrtf(expected));
        result[2]=fmaxf(result[2],sqrtf(actual));
    }
}
static void body_contact_bound_correction(float correction[3][2],int segments,float radius)
{
    float norm2=0;int j,a;
    for(j=0;j<segments;j++) for(a=0;a<2;a++) norm2+=correction[j][a]*correction[j][a];
    if(norm2>radius*radius) {
        float scale=radius/sqrtf(norm2);
        for(j=0;j<segments;j++) for(a=0;a<2;a++) correction[j][a]*=scale;
    }
}
static float body_contact_overlap_error(const float points[4][3],
    const body_chain_contact_t *contacts,int count)
{
    float error=0;int c,a;
    for(c=0;c<count;c++) {
        float p[3],sep=0,depth;
        body_contact_point(points,&contacts[c],p);
        for(a=0;a<3;a++) sep+=(p[a]-contacts[c].body[a])*contacts[c].normal[a];
        depth=fmaxf(0.0f,contacts[c].target_sep-sep);
        error+=depth*depth;
    }
    return error;
}
static void body_contact_limit_velocity(const body_chain_physics_config_t *cfg,
    body_chain_person_state_t *state,const float correction[3][2],int segments)
{
    int j,axis;
    for(j=0;j<segments;j++) for(axis=0;axis<3;axis++) {
        float angle=state->angle[j][axis],velocity=state->velocity[j][axis];
        if(correction && axis==cfg->horizontal_output_axis) angle+=correction[j][0];
        if(correction && axis==cfg->vertical_output_axis) angle+=correction[j][1];
        if((velocity>0 && angle>=body_chain_link_axis_limit(cfg,j,axis)-.00001f) ||
           (velocity<0 && angle<=body_chain_link_axis_min_limit(cfg,j,axis)+.00001f))
            state->velocity[j][axis]=0;
    }
}
static int body_contact_orient_from_history(float normal[3],const float previous[3],
    float previous_length,float radius,float slop)
{
    int a;
    if(previous_length<=.0001f || previous_length<fmaxf(.0001f,radius-slop) ||
       vec3_dot(normal,previous)>=0.0f) return 0;
    for(a=0;a<3;a++) normal[a]=-normal[a];
    return 1;
}
static void body_contact_refine_combined(const body_chain_physics_config_t *cfg,
    const body_chain_person_state_t *state,const float base[4][3],
    const body_chain_contact_t *contacts,int count,int segments,
    float cap,float radius,float relaxation,float correction[3][2])
{
    COLLISION_PROFILE_SCOPE(profile_refine, CP_BODY_REFINE);
    float points[4][3],error;int pass,c,j,a;
    body_contact_predict(cfg,state,base,correction,points);
    error=body_contact_overlap_error(points,contacts,count);
    for(pass=0;pass<8 && error>1e-10f;pass++) {
        float gradient[3][2]={{0}},denom=0,step[3][2],norm=0;
        int attempt,accepted=0;
        for(c=0;c<count;c++) {
            float p[3],sep=0,depth,jac[3][2];
            body_contact_point(points,&contacts[c],p);
            for(a=0;a<3;a++) sep+=(p[a]-contacts[c].body[a])*contacts[c].normal[a];
            depth=contacts[c].target_sep-sep;
            if(depth<=0) continue;
            body_contact_jacobian(cfg,state,base,correction,&contacts[c],contacts[c].normal,jac,0);
            for(j=0;j<segments;j++) for(a=0;a<2;a++) {
                gradient[j][a]+=depth*jac[j][a];
                denom+=jac[j][a]*jac[j][a];
            }
        }
        if(denom<1e-10f) break;
        memset(step,0,sizeof(step));
        for(j=0;j<segments;j++) for(a=0;a<2;a++) {
            step[j][a]=relaxation*gradient[j][a]/denom;
            norm+=step[j][a]*step[j][a];
        }
        if(norm<1e-12f) break;
        if(norm>4.0f) for(j=0;j<segments;j++) for(a=0;a<2;a++) step[j][a]*=2.0f/sqrtf(norm);
        for(attempt=0;attempt<8;attempt++) {
            float trial[3][2],trial_points[4][3],trial_error;
            memcpy(trial,correction,sizeof(trial));
            for(j=0;j<segments;j++) for(a=0;a<2;a++) {
                int axis=a?cfg->vertical_output_axis:cfg->horizontal_output_axis;
                float value=physx_clampf(correction[j][a]+step[j][a],-cap,cap);
                trial[j][a]=body_chain_clamp_link_axis_angle(cfg,j,axis,state->angle[j][axis]+value)-state->angle[j][axis];
            }
            body_contact_bound_correction(trial,segments,radius);
            body_contact_predict(cfg,state,base,trial,trial_points);
            trial_error=body_contact_overlap_error(trial_points,contacts,count);
            if(trial_error<error-1e-12f) {
                memcpy(correction,trial,sizeof(trial));memcpy(points,trial_points,sizeof(points));
                error=trial_error;accepted=1;break;
            }
            for(j=0;j<segments;j++) for(a=0;a<2;a++) step[j][a]*=.5f;
        }
        if(!accepted) break;
    }
}
static void body_contact_solve(const body_chain_physics_config_t *cfg,
    body_chain_person_state_t *state, const float base[4][3],
    body_chain_contact_t *contacts,int count,int segment_count,float correction[3][2])
{
    COLLISION_PROFILE_SCOPE(profile_solve, CP_BODY_SOLVE);
    float points[4][3];
    float velocity_jacobians[BODY_CHAIN_MAX_CONTACTS][3][2];
    float position_impulse[BODY_CHAIN_MAX_CONTACTS]={0};
    float best_correction[3][2]={{0}};
    float best_error,initial_error,best_norm=0;
    float cap=body_chain_collider_cfg.response_max_degrees_per_tick;
    float trust_radius=360.0f*physx_clampf(state->collision_step_dt>0 ?
        state->collision_step_dt : 1.0f/60.0f,1.0f/240.0f,.05f);
    float relaxation=physx_clampf(body_chain_collider_cfg.response_strength,0.0f,1.0f);
    int iterations=(int)physx_clampf((float)body_chain_collider_cfg.collision_iterations,1.0f,6.0f)*8;
    int pass,c,j,a;
    memset(correction,0,sizeof(float)*6);
    body_contact_limit_velocity(cfg,state,NULL,segment_count);
    if(body_chain_collider_cfg.response_strength<=0.0f) return;
    memcpy(points,base,sizeof(points));
    best_error=initial_error=body_contact_overlap_error(points,contacts,count);
    /* More than one support must constrain the same candidate. No joint is
       frozen merely because an earlier contact used it. */
    COLLISION_PROFILE_COUNT(CP_BODY_CONTACTS, count);
    COLLISION_PROFILE_SCOPE(profile_position, CP_BODY_POSITION);
    for(pass=0;pass<iterations;pass++) {
        COLLISION_PROFILE_COUNT(CP_BODY_PASSES, 1);
        float largest=0;
        for(c=0;c<count;c++) {
            float p[3],jac[3][2],denom=0,depth,sep=0,lambda,next_lambda;
            body_contact_point(points,&contacts[c],p);
            for(a=0;a<3;a++) sep+=(p[a]-contacts[c].body[a])*contacts[c].normal[a];
            depth=contacts[c].target_sep-sep;
            if(depth<=.000001f && position_impulse[c]<=0) continue;
            body_contact_jacobian(cfg,state,base,correction,&contacts[c],contacts[c].normal,jac,1);
            for(j=0;j<segment_count;j++) for(a=0;a<2;a++) denom+=jac[j][a]*jac[j][a];
            if(denom<1e-10f) continue;
            next_lambda=fmaxf(0.0f,position_impulse[c]+relaxation*depth/(denom+1e-9f));
            lambda=next_lambda-position_impulse[c];
            position_impulse[c]=next_lambda;
            for(j=0;j<segment_count;j++) for(a=0;a<2;a++) {
                int axis=a?cfg->vertical_output_axis:cfg->horizontal_output_axis;
                float before=correction[j][a];
                float step=lambda*jac[j][a];
                float next=physx_clampf(before+step,-cap,cap);
                next=body_chain_clamp_link_axis_angle(cfg,j,axis,state->angle[j][axis]+next)-state->angle[j][axis];
                correction[j][a]=next;
                if(physx_absf(next-before)>largest) largest=physx_absf(next-before);
            }
            body_contact_bound_correction(correction,segment_count,trust_radius);
            body_contact_predict(cfg,state,base,correction,points);
            {
                float error=body_contact_overlap_error(points,contacts,count),norm=0;
                for(j=0;j<segment_count;j++) for(a=0;a<2;a++) norm+=correction[j][a]*correction[j][a];
                /* Inconsistent supports and saturated joints can make later
                   iterations worse. Publish only an actual improvement over
                   the original candidate; prefer less movement on ties. */
                if(error<=initial_error && (error<best_error-1e-12f ||
                    (error<=best_error+1e-12f && norm<best_norm))) {
                    best_error=error;best_norm=norm;
                    memcpy(best_correction,correction,sizeof(best_correction));
                }
            }
        }
        if(largest<.00001f) break;
    }
    COLLISION_PROFILE_END(profile_position);
    memcpy(correction,best_correction,sizeof(best_correction));
    body_contact_refine_combined(cfg,state,base,contacts,count,segment_count,
        cap,trust_radius,relaxation,correction);
    body_contact_limit_velocity(cfg,state,correction,segment_count);
    /* Project contact velocity across all influencing joints. Positional
       correction/dt is deliberately absent: separation must not cause bounce.
       Free separation is untouched. Repeated passes restore all
       corner-support inequalities. */
    COLLISION_PROFILE_SCOPE(profile_velocity, CP_BODY_VELOCITY);
    for(pass=0;pass<8;pass++) for(c=0;c<count;c++) {
        float jac[3][2],denom=0,vn=0,lambda;
        /* Only velocity changes during these eight passes. Angles, final
           correction, contact normals and the sampled pose remain fixed, so
           each contact's geometric Jacobian is identical on every pass.
           Retain all passes, live velocity reads and joint-limit handling. */
        if(pass==0) {
            body_contact_jacobian(cfg,state,base,correction,&contacts[c],contacts[c].normal,jac,0);
            memcpy(velocity_jacobians[c],jac,sizeof(jac));
        } else {
            memcpy(jac,velocity_jacobians[c],sizeof(jac));
        }
        for(j=0;j<segment_count;j++) for(a=0;a<2;a++) {
            int axis=a?cfg->vertical_output_axis:cfg->horizontal_output_axis;
            denom+=jac[j][a]*jac[j][a]*body_contact_inverse_inertia(state,j,a);
            vn+=jac[j][a]*state->velocity[j][axis];
        }
        if(vn>=0 || denom<1e-10f) continue;
        lambda=-vn/denom;
        for(j=0;j<segment_count;j++) for(a=0;a<2;a++)
            state->velocity[j][a?cfg->vertical_output_axis:cfg->horizontal_output_axis]+=
                lambda*jac[j][a]*body_contact_inverse_inertia(state,j,a);
        body_contact_limit_velocity(cfg,state,correction,segment_count);
    }
}
static void body_contact_apply(body_chain_person_state_t *state,
    const body_chain_physics_config_t *cfg,const float correction[3][2],int segments,float penetration)
{
    int j,a;
    for(j=0;j<segments;j++) for(a=0;a<2;a++) {
        int axis=a?cfg->vertical_output_axis:cfg->horizontal_output_axis;
        state->angle[j][axis]=body_chain_clamp_link_axis_angle(cfg,j,axis,state->angle[j][axis]+correction[j][a]);
        state->collision_contact_direction[j][a]=0;
    }
    state->collision_prev_max_penetration=penetration;
    state->collision_rest_valid=0;state->collision_rest_ticks=0;
    state->collision_rest_grace_ticks=0;state->collision_impact_ticks=0;
    state->collision_multi_support_grace_ticks=0;
}
static int body_chain_penis_cross_sample(
    const body_chain_collider_person_state_t *collider,
    const body_chain_person_state_t *state,float points[4][3],DWORD now)
{
    int engine=0;
    if (state->collision_step_engine_points &&
        body_contact_candidate_points(state,&body_chain_physics_cfg,now,points)) return 1;
    /* Independent update intervals can leave the other chain's candidate one
       tick old. Keep its real live/held shape. Configured fallback link lengths
       are not a substitute for the visible chain during mutual contact. */
    if (!body_chain_collision_points_local(collider,state,points,&engine,now)) return 0;
    return engine!=0;
}

typedef unsigned char BYTE;
static body_chain_physics_config_t testicle_physics_cfg;
static body_chain_collider_person_state_t body_chain_collider_states[4];
static body_chain_person_state_t body_chain_person_states[4],runtime_body_chain_person_states[4],testicle_physics_states[4],runtime_testicle_physics_states[4];
static long body_chain_poseeditor_mode_active;
static long InterlockedCompareExchange(long *v,long a,long b){long old=*v;if(old==b)*v=a;return old;}
static struct {int gravity_horizontal_basis_offset,gravity_vertical_basis_offset,gravity_horizontal_secondary_basis_offset;} physics_environment_cfg={0x088,0x098,0x078};
static int pivot_hold,frame_updates;
static int body_chain_camera_pivot_hold_active(DWORD now){(void)now;return pivot_hold;}
static int ptr_readable(const void *p,size_t n){(void)n;return p!=NULL;}
static int body_chain_collision_scope_collider_mask(int scope){return scope;}
static void update_body_chain_colliders_for_person_scope(int p,DWORD now,int scope){(void)p;(void)now;(void)scope;frame_updates++;}
static void body_contact_trace_pose(const body_chain_collider_person_state_t *c,body_chain_person_state_t *s,const body_chain_physics_config_t *cfg,int t,DWORD now,const float observed[4][3],int engine){(void)c;(void)s;(void)cfg;(void)t;(void)now;(void)observed;(void)engine;}
static void body_contact_capture_step(body_chain_collider_person_state_t *collider,
    body_chain_person_state_t *state,int testicle,DWORD now)
{
    const body_chain_physics_config_t *cfg=testicle?&testicle_physics_cfg:&body_chain_physics_cfg;
    float sampled[4][3];
    int engine=0;
    if (state->collision_step_valid && state->collision_step_tick==now) return;
    /* Geometry also drives gravity/inertia when contact response is disabled. */
    if (!collider->ready ||
        !(testicle?body_chain_testicle_collision_points_local(collider,state,sampled,&engine,now):
                   body_chain_collision_points_local(collider,state,sampled,&engine,now))) {
        state->collision_step_valid=0;return;
    }
    body_contact_trace_pose(collider,state,cfg,testicle,now,sampled,engine);
    memcpy(state->collision_step_points,sampled,sizeof(sampled));
    memcpy(state->collision_step_angle,state->angle,sizeof(state->angle));
    state->collision_pose_valid=0;
    /* Read the same published Euler values represented by the sampled pivots.
       Held/camera-quarantined geometry cannot be fitted to newer output. */
    if(engine==1 && !body_chain_camera_pivot_hold_active(now) &&
       physics_environment_cfg.gravity_horizontal_basis_offset==0x088 &&
       physics_environment_cfg.gravity_vertical_basis_offset==0x098 &&
       physics_environment_cfg.gravity_horizontal_secondary_basis_offset==0x078) {
        float euler[3][3]={{0}};int j,complete=1;
        for(j=0;j<(testicle?2:3);j++) {
            if(!state->joint_raw[j] || cfg->output_offset<0 ||
               !ptr_readable((BYTE*)state->joint_raw[j]+cfg->output_offset,sizeof(float)*3)) {
                complete=0;break;
            }
            memcpy(euler[j],(BYTE*)state->joint_raw[j]+cfg->output_offset,sizeof(float)*3);
        }
        if(complete) state->collision_pose_valid=body_pose_fit(sampled,euler,testicle?2:3,&state->collision_pose);
    }
    state->collision_step_engine_points=engine;
    state->collision_step_tick=now;
    state->collision_step_valid=1;
}
static void body_contact_begin_step(int person,body_chain_person_state_t *state,int testicle,DWORD now,float dt)
{
    body_chain_collider_person_state_t *collider=&body_chain_collider_states[person];
    int pose=InterlockedCompareExchange(&body_chain_poseeditor_mode_active,0,0);
    body_chain_person_state_t *other=testicle?
        (pose?&body_chain_person_states[person]:&runtime_body_chain_person_states[person]):
        (pose?&testicle_physics_states[person]:&runtime_testicle_physics_states[person]);
    const body_chain_physics_config_t *cfg=testicle?&testicle_physics_cfg:&body_chain_physics_cfg;
    state->collision_step_dt=dt;
    update_body_chain_colliders_for_person_scope(person,now,
        body_chain_collision_scope_collider_mask(cfg->collision_scope));
    body_contact_capture_step(collider,state,testicle,now);
    /* Both active chains start from the same sampled frame. The second
       solve sees the first chain's corrected candidate, not its old pivots. */
    if(other->initialized && other->active_logged)
        body_contact_capture_step(collider,other,!testicle,now);
    if(state->collision_step_valid && state->collision_pose_valid) {
        float neutral[3][3];int j,a;
        for(j=0;j<3;j++) for(a=0;a<3;a++)
            neutral[j][a]=state->collision_pose.euler[j][a]-state->collision_step_angle[j][a];
        if(body_dynamics_prepare(&state->collision_pose,neutral,
            cfg->horizontal_output_axis,cfg->vertical_output_axis,&state->dynamics))
            state->dynamics_valid=1;
    }
}

static const float base[4][3]={{0,0,0},{-.1f,0,0},{-.1f,.1f,0},{-.1f,.2f,0}};
static void configure(void){
    int j,a;memset(&body_chain_physics_cfg,0,sizeof(body_chain_physics_cfg));
    body_chain_physics_cfg.horizontal_output_axis=2;body_chain_physics_cfg.vertical_output_axis=1;
    body_chain_physics_cfg.rotation_tail_axis[0]=2;body_chain_physics_cfg.rotation_tail_axis[1]=1;
    for(j=0;j<3;j++)for(a=0;a<3;a++){body_chain_physics_cfg.link_min_angle[j][a]=-90;body_chain_physics_cfg.link_max_angle[j][a]=90;}
}
static void support(body_chain_contact_t *c,int segment,float t,float depth,const float *normal,const float points[4][3]){
    memset(c,0,sizeof(*c));c->segment=segment;c->segment_t=t;c->penetration=depth;
    memcpy(c->normal,normal,3*sizeof(float));body_contact_point(points,c,c->chain);
    c->target_sep=vec3_dot(c->chain,normal)+depth;
}
static float gap(const float p[4][3],const body_chain_contact_t *c){float x[3];body_contact_point(p,c,x);return vec3_dot(x,c->normal)-c->target_sep;}
static void supports(void){
    body_chain_person_state_t state={0};body_chain_contact_t c[8];int count=0;
    float up[3]={0,1,0},side[3]={0,0,1},a[3]={-.025f,0,0},b[3]={-.09f,0,0},body[3]={0};
    float correction[3][2],out[4][3];
    body_chain_store_contact(c,&count,8,0,.25f,.004f,a,body,up);
    body_chain_store_contact(c,&count,8,0,.9f,.003f,b,body,up);
    assert(count==2);
    body_chain_store_contact(c,&count,8,0,.9f,.002f,b,body,up);assert(count==2);
    support(&c[0],0,1,.001f,up,base);support(&c[1],1,1,.005f,up,base);
    support(&c[2],1,.6f,.002f,side,base);
    body_contact_solve(&body_chain_physics_cfg,&state,base,c,3,3,correction);
    body_chain_live_points_with_delta_cfg(&body_chain_physics_cfg,base,correction,out);
    assert(gap(out,&c[0])>=-2e-6f && gap(out,&c[1])>=-2e-6f && gap(out,&c[2])>=-2e-6f);
    assert(fabsf(correction[0][1])>.01f);
    puts("PASS: distinct lever arms retained; duplicate suppressed; distal support can use upstream joint while preserving proximal/corner contacts");
}
static void candidate(void){
    body_chain_person_state_t s={0};float p[4][3];
    memcpy(s.collision_step_points,base,sizeof(base));s.collision_step_valid=1;s.collision_step_tick=42;
    s.angle[0][1]=20;
    assert(body_contact_candidate_points(&s,&body_chain_physics_cfg,42,p));
    assert(fabsf(p[1][1]-base[1][1])>.01f);
    assert(!body_contact_candidate_points(&s,&body_chain_physics_cfg,43,p));
    assert(!memcmp(s.collision_step_points,base,sizeof(base)));
    puts("PASS: predicted query follows angle changes without mutating sampled engine points; old tick rejected");
}
static void testicle_geometry(void){
    /* Recorded real pivots: the old midpoint/extrapolation path returned an
       extended tip and rotated the chain about the proximal sphere centre. */
    const float joints[3][3]={{-.02f,0,-.037f},{-.00265f,.00403f,-.08538f},
                             {-.00759f,.00670f,-.14275f}};
    body_chain_collider_person_state_t cache={0};body_chain_person_state_t s={0};
    float p[4][3],rotated[4][3],delta[3][2]={{0}};int engine=0;
    memcpy(cache.testicle_joint_position,joints,sizeof(joints));
    cache.testicle_points_ready=1;cache.testicle_points_update_tick=100;
    assert(body_chain_testicle_collision_points_local(&cache,&s,p,&engine,100));
    assert(engine==1 && !memcmp(p,joints,sizeof(joints)) && !memcmp(p[2],p[3],sizeof(p[2])));
    delta[1][1]=10;
    body_chain_live_points_with_delta_cfg(&body_chain_physics_cfg,p,delta,rotated);
    assert(!memcmp(rotated[0],joints[0],sizeof(p[0])) && !memcmp(rotated[1],joints[1],sizeof(p[1])));
    assert(fabsf(rotated[2][0]-p[2][0])+fabsf(rotated[2][1]-p[2][1])>1e-5f);
    assert(body_chain_testicle_collision_points_local(&cache,&s,p,&engine,400));
    assert(!body_chain_testicle_collision_points_local(&cache,&s,p,&engine,601) && engine==0);
    cache.testicle_points_ready=0;
    assert(!body_chain_testicle_collision_points_local(&cache,&s,p,&engine,100));
    cache.testicle_points_ready=1;cache.testicle_joint_position[2][1]=NAN;
    assert(!body_chain_testicle_collision_points_local(&cache,&s,p,&engine,100));
    puts("PASS: active testicle chain uses real pivots/tip; child rotates about real joint; held geometry preserved; expired/invalid geometry rejected");
}
static void asynchronous_cross_sample(void){
    body_chain_collider_person_state_t cache={0};body_chain_person_state_t s={0};
    float p[4][3],fallback[4][3];int j;
    cache.chain_points_ready=cache.chain_points_fresh=1;cache.chain_points_update_tick=100;
    memcpy(cache.chain_local_point,base,sizeof(base));
    for(j=0;j<4;j++)cache.chain_point_valid[j]=1;
    memcpy(s.collision_step_points,base,sizeof(base));
    s.collision_step_valid=s.collision_step_engine_points=1;s.collision_step_tick=100;
    s.angle[0][1]=20;
    assert(body_chain_penis_cross_sample(&cache,&s,p,100));
    assert(fabsf(p[1][1]-base[1][1])>.01f);
    assert(body_chain_penis_cross_sample(&cache,&s,p,116));
    assert(!memcmp(p,base,sizeof(base)));
    body_chain_simulated_points_local(&s,&body_chain_physics_cfg,NULL,fallback);
    assert(physx_vec3_len(fallback[3])>1.4f && physx_vec3_len(p[3])<.3f);
    cache.chain_points_fresh=0;
    assert(body_chain_penis_cross_sample(&cache,&s,p,150));
    assert(!body_chain_penis_cross_sample(&cache,&s,p,181));
    cache.chain_points_ready=0;
    assert(!body_chain_penis_cross_sample(&cache,&s,p,116));
    puts("PASS: staggered chain ticks retain live/held penis geometry; expired samples never become configured 1.5-unit fallback colliders");
}
static void velocity(void){
    body_chain_person_state_t s={0};body_chain_contact_t c;
    float up[3]={0,1,0},corr[3][2];
    support(&c,0,1,0,up,base);
    s.velocity[0][1]=-10;s.velocity[0][2]=3;
    body_contact_solve(&body_chain_physics_cfg,&s,base,&c,1,3,corr);
    assert(fabsf(s.velocity[0][1])<1e-5f);
    assert(fabsf(s.velocity[0][2]-3)<1e-6f);
    s.velocity[0][1]=10;
    body_contact_solve(&body_chain_physics_cfg,&s,base,&c,1,3,corr);
    assert(s.velocity[0][1]==10);
    puts("PASS: inward contact velocity removed; tangential sliding and outward release preserved; no position-correction impulse");
}
static void blocked_velocity(void){
    const float straight[4][3]={{0,0,0},{-.1f,0,0},{-.2f,0,0},{-.3f,0,0}};
    body_chain_person_state_t s={0};body_chain_contact_t c;
    float down[3]={0,-1,0},corr[3][2];
    body_chain_physics_cfg.link_max_angle[0][1]=7.5f;
    s.angle[0][1]=7.5f;s.velocity[0][1]=100;
    support(&c,1,1,0,down,straight);
    body_contact_solve(&body_chain_physics_cfg,&s,straight,&c,1,2,corr);
    if(fabsf(s.velocity[0][1])>1e-5f || fabsf(s.velocity[1][1])>1e-5f){
        fprintf(stderr,"FAIL: blocked root velocity leaked into contact: root=%g child=%g\n",s.velocity[0][1],s.velocity[1][1]);exit(1);
    }
    s.velocity[0][1]=-10;
    body_contact_solve(&body_chain_physics_cfg,&s,base,0,0,2,corr);
    assert(s.velocity[0][1]==-10);
    configure();
    puts("PASS: velocity into a joint limit is removed before contact coupling; movement away from the limit remains responsive");
}
static void limits(void){
    body_chain_person_state_t s={0};body_chain_contact_t c;float up[3]={0,1,0},corr[3][2];int j,a;
    support(&c,1,1,.02f,up,base);
    for(j=0;j<3;j++)for(a=0;a<3;a++){body_chain_physics_cfg.link_min_angle[j][a]=body_chain_physics_cfg.link_max_angle[j][a]=1;s.angle[j][a]=1;}
    body_contact_solve(&body_chain_physics_cfg,&s,base,&c,1,3,corr);
    for(j=0;j<3;j++)for(a=0;a<2;a++)assert(corr[j][a]==0);
    configure();memset(&s,0,sizeof(s));body_chain_collider_cfg.response_strength=0;
    body_contact_solve(&body_chain_physics_cfg,&s,base,&c,1,3,corr);
    for(j=0;j<3;j++)for(a=0;a<2;a++)assert(corr[j][a]==0);
    body_chain_collider_cfg.response_strength=1;
    puts("PASS: unreachable joint limits remain bounded; collision strength zero disables response");
}
static void settling(void){
    const float rates[6]={30,60,144,60,20,60};int r,n,j,a,segments,composed;
    for(composed=0;composed<=1;composed++) for(segments=2;segments<=3;segments++) for(r=0;r<6;r++){
        body_chain_person_state_t s={0};float minimum=100,maximum=-100;
        const float straight[4][3]={{0,0,0},{-.1f,0,0},{-.2f,0,0},{-.3f,0,0}};
        if(composed){
            assert(body_pose_fit(straight,s.angle,segments,&s.collision_pose));
            s.collision_step_valid=s.collision_pose_valid=1;
            assert(body_dynamics_prepare(&s.collision_pose,s.angle,2,1,&s.dynamics));s.dynamics_valid=1;
            /* Sideways bending stays active while all links press on the plane. */
            s.angle[0][2]=20;s.angle[1][2]=-15;s.angle[2][2]=10;
        }
        for(n=0;n<3120;n++){
            float duration=r==3?(n%60==0?.05f:.016f):r==5?body_motion_duration(n%60==0?500:16):1/rates[r];
            int steps=body_motion_substeps(duration),step;
            float dt=duration/steps;
            s.collision_step_dt=dt;
            float delta[3][2]={{0}},points[4][3],corr[3][2],up[3]={0,1,0};
            for(step=0;step<steps;step++) {
            body_chain_contact_t c[3];int count=0;
            float configured[3][3]={{0}},shaped[3][3],direction[3]={0,n<3000?-1:1,0};
            for(j=0;j<segments;j++) configured[j][1]=(composed?20:-20)*(n<3000?1:-1);
            memcpy(shaped,configured,sizeof(shaped));
            if(composed) assert(body_dynamics_gravity(&s.dynamics.reference,&s.dynamics,s.angle,direction,configured,shaped));
            for(j=0;j<segments;j++){
                float inverse=body_contact_inverse_inertia(&s,j,1);
                body_motion_limited_spring_step(&s.angle[j][1],&s.velocity[j][1],shaped[j][1],100*inverse,8*sqrtf(inverse),dt,-90,90);
                delta[j][1]=s.angle[j][1];
            }
            if(composed) body_contact_predict(&body_chain_physics_cfg,&s,straight,NULL,points);
            else body_chain_live_points_with_delta_cfg(&body_chain_physics_cfg,straight,delta,points);
            for(j=0;j<segments;j++) if(points[j+1][1]<0){
                support(&c[count],j,1,-points[j+1][1],up,points);count++;
            }
            body_contact_solve(&body_chain_physics_cfg,&s,points,c,count,segments,corr);
            body_contact_apply(&s,&body_chain_physics_cfg,corr,segments,0);
            for(j=0;j<segments;j++)for(a=0;a<2;a++)delta[j][a]=s.angle[j][a?1:2];
            if(composed) body_contact_predict(&body_chain_physics_cfg,&s,straight,NULL,points);
            else body_chain_live_points_with_delta_cfg(&body_chain_physics_cfg,straight,delta,points);
            for(j=0;j<segments;j++)assert(isfinite(s.angle[j][1]) && points[j+1][1]>-.00002f);
            }
            if(n>2500 && n<3000){minimum=fminf(minimum,points[segments][1]);maximum=fmaxf(maximum,points[segments][1]);}
            if(n==3119) assert(points[segments][1]>.002f);
        }
        printf("BODY_REST composed=%d segments=%d pattern=%d tip_range=%.9f\n",composed,segments,r,maximum-minimum);
        assert(maximum-minimum<.00002f);
    }
    puts("PASS: production substeps, angular springs and contact solve rest and release at 20/30/60/144Hz, uneven frames and capped stalls without contact sleep");
}
static float overlap_error(const float p[4][3],body_chain_contact_t *c,int count){
    float error=0;int i;for(i=0;i<count;i++){float d=fmaxf(0,-gap(p,&c[i]));error+=d*d;}return error;
}
static void competing_supports(void){
    const float straight[4][3]={{0,0,0},{-.1f,0,0},{-.2f,0,0},{-.3f,0,0}};
    body_chain_person_state_t s={0};body_chain_contact_t c[2];
    float up[3]={0,1,0},down[3]={0,-1,0},corr[3][2],out[4][3];
    support(&c[0],0,1,.004f,up,straight);
    support(&c[1],0,1,.006f,down,straight);
    body_contact_solve(&body_chain_physics_cfg,&s,straight,c,2,1,corr);
    body_chain_live_points_with_delta_cfg(&body_chain_physics_cfg,straight,corr,out);
    if(overlap_error(out,c,2)>=overlap_error(straight,c,2)-1e-6f){
        fprintf(stderr,"FAIL: competing supports stall despite available combined descent: error %g -> %g\n",overlap_error(straight,c,2),overlap_error(out,c,2));exit(1);
    }
    puts("PASS: competing contacts make combined progress when individual full corrections would worsen overlap");
}
static void embedded_history(void){
    float normal[3]={0,1,0},inside[3]={0,-.025f,0},outside[3]={0,-.11f,0};
    assert(!body_contact_orient_from_history(normal,inside,.025f,.1025f,.001f));
    assert(normal[1]==1);
    assert(body_contact_orient_from_history(normal,outside,.11f,.1025f,.001f));
    assert(normal[1]==-1);
    normal[1]=1;outside[1]=.11f;
    assert(!body_contact_orient_from_history(normal,outside,.11f,.1025f,.001f));
    puts("PASS: embedded history cannot reverse outward recovery; verified exterior entry side remains protected");
}
static void observed_pose_error(void){
    float before[4][3]={{0}},predicted[4][3]={{0}},observed[4][3]={{0}},error[3];
    predicted[2][0]=.01f;observed[2][0]=-.01f;
    body_contact_pose_error(before,predicted,observed,2,error);
    assert(fabsf(error[0]-.02f)<1e-7f && fabsf(error[1]-.01f)<1e-7f && fabsf(error[2]-.01f)<1e-7f);
    memcpy(observed,predicted,sizeof(predicted));
    body_contact_pose_error(before,predicted,observed,2,error);assert(error[0]==0);
    memset(observed,0,sizeof(observed));
    body_contact_pose_error(before,predicted,observed,2,error);assert(fabsf(error[0]-.01f)<1e-7f && error[2]==0);
    assert(predicted[2][0]==.01f && before[2][0]==0);
    puts("PASS: read-only pose audit distinguishes opposite movement, exact agreement and an unchanged engine sample");
}
static void composed_contact(void){
    const float points[4][3]={{0,0,0},{.0219286f,.0069399f,-.0729516f},
        {-.0250599f,.0207295f,-.1471537f},{-.0692438f,.0281818f,-.1677821f}};
    const float angles[3][3]={{0,-5.20544f,26.02263f},{0,-5.49461f,34.13186f},{0,-3.25490f,23.90110f}};
    body_chain_person_state_t s={0};body_chain_contact_t c[2];float up[3]={0,1,0},corr[3][2],out[4][3],raw[4][3];
    memcpy(s.angle,angles,sizeof(angles));memcpy(s.collision_step_angle,angles,sizeof(angles));
    memcpy(s.collision_step_points,points,sizeof(points));s.collision_step_valid=1;
    assert(body_pose_fit(points,angles,3,&s.collision_pose));s.collision_pose_valid=1;
    support(&c[0],0,1,.001f,up,points);support(&c[1],2,.8f,.002f,up,points);
    body_contact_solve(&body_chain_physics_cfg,&s,points,c,2,3,corr);
    body_contact_predict(&body_chain_physics_cfg,&s,points,corr,out);
    assert(gap(out,&c[0])>=-2e-6f && gap(out,&c[1])>=-2e-6f);
    body_contact_apply(&s,&body_chain_physics_cfg,corr,3,0);
    body_pose_evaluate(&s.collision_pose,s.angle,raw);
    for(int j=0;j<4;j++)for(int a=0;a<3;a++)assert(fabsf(raw[j][a]-out[j][a])<1e-7f);
    puts("PASS: composed output mapping separates proximal/distal supports and published angles reproduce the solved pose");
}
static void contact_near_limit(void){
    const float straight[4][3]={{0,0,0},{-.1f,0,0},{-.2f,0,0},{-.3f,0,0}};
    const float rates[]={20,30,60,144};int r,n;
    for(r=0;r<4;r++) {
        body_chain_person_state_t s={0};body_chain_contact_t c;
        float points[4][3],corr[3][2],up[3]={0,1,0},minimum=100,maximum=-100;
        float dt=(1/rates[r])/body_motion_substeps(1/rates[r]);
        configure();body_chain_physics_cfg.link_min_angle[0][1]=-7.5f;
        body_chain_physics_cfg.link_max_angle[0][1]=7.5f;
        assert(body_pose_fit(straight,s.angle,1,&s.collision_pose));
        s.collision_step_valid=s.collision_pose_valid=1;s.collision_step_dt=dt;
        s.angle[0][1]=6; /* The support sits inside the progressive braking band. */
        body_contact_predict(&body_chain_physics_cfg,&s,straight,NULL,points);
        support(&c,0,1,0,up,points);
        for(n=0;n<1600;n++) {
            body_motion_limited_spring_step(&s.angle[0][1],&s.velocity[0][1],
                n<1500?7.5f:-4,115,4,dt,-7.5f,7.5f);
            body_contact_predict(&body_chain_physics_cfg,&s,straight,NULL,points);
            body_contact_solve(&body_chain_physics_cfg,&s,points,&c,1,1,corr);
            body_contact_apply(&s,&body_chain_physics_cfg,corr,1,0);
            body_contact_predict(&body_chain_physics_cfg,&s,straight,NULL,points);
            assert(gap(points,&c)>-2e-6f);
            if(n>1200 && n<1500) {
                minimum=fminf(minimum,points[1][1]);maximum=fmaxf(maximum,points[1][1]);
            }
            if(n==1500) assert(s.velocity[0][1]<0);
        }
        assert(maximum-minimum<2e-6f && gap(points,&c)>.002f);
        /* A contact can still use angles beyond the beginning of the band. */
        s.angle[0][1]=6;s.velocity[0][1]=0;
        body_contact_predict(&body_chain_physics_cfg,&s,straight,NULL,points);
        {float toward_stop[3]={0,-1,0};
        support(&c,0,1,.0024f,toward_stop,points);}
        body_contact_solve(&body_chain_physics_cfg,&s,points,&c,1,1,corr);
        body_contact_apply(&s,&body_chain_physics_cfg,corr,1,0);
        assert(s.angle[0][1]>7.3f && s.angle[0][1]<=7.5f);
    }
    configure();
    puts("PASS: near-limit contact rests and releases at 20/30/60/144Hz; collision escape retains the full angular range");
}
static void inertia_contact_energy(void){
    const float straight[4][3]={{0,0,0},{-.1f,0,0},{-.2f,0,0},{-.3f,0,0}};
    body_chain_person_state_t s={0};body_chain_contact_t c;
    float points[4][3],normal[3]={0,1,0},correction[3][2]={{0}},jac[3][2],before=0,after=0,vn=0;
    int j,a;
    assert(body_pose_fit(straight,s.angle,3,&s.collision_pose));
    assert(body_dynamics_prepare(&s.collision_pose,s.angle,2,1,&s.dynamics));
    s.collision_step_valid=s.collision_pose_valid=s.dynamics_valid=1;
    body_contact_predict(&body_chain_physics_cfg,&s,straight,NULL,points);
    support(&c,2,1,0,normal,points);
    body_contact_jacobian(&body_chain_physics_cfg,&s,points,correction,&c,normal,jac,0);
    for(j=0;j<3;j++)for(a=0;a<2;a++) {
        float v=-jac[j][a]*10000;s.velocity[j][a?1:2]=v;
        before+=v*v/s.dynamics.inverse_inertia[j][a];
    }
    body_contact_solve(&body_chain_physics_cfg,&s,points,&c,1,3,correction);
    for(j=0;j<3;j++)for(a=0;a<2;a++) {
        float v=s.velocity[j][a?1:2];after+=v*v/s.dynamics.inverse_inertia[j][a];vn+=jac[j][a]*v;
    }
    assert(before>0 && after<=before+1e-5f && vn>=-1e-6f);
    puts("PASS: contact projects with matching inverse inertias, cancels inward speed and does not add kinetic energy");
}
static void thigh_recovery(void){
    float points[4][3]={{0,0,0},{.00379f,.05747f,-.05033f},
        {-.04599f,.12097f,-.08767f},{-.08744f,.14531f,-.09872f}};
    const float hip[2][3]={{-.08788f,.11241f,.03107f},{-.08788f,-.11241f,.03107f}};
    const float thigh[2][3]={{-.07939f,.11466f,-.15985f},{-.07939f,-.11466f,-.15985f}};
    body_chain_person_state_t s={0};int frame,i,b,a;float deepest=0;
    for(frame=0;frame<180;frame++){
        body_chain_contact_t c[6];int count=0;float corr[3][2],out[4][3];
        deepest=0;
        for(i=0;i<3;i++)for(b=0;b<2;b++){
            float ct,bt,p[3],q[3],dist,normal[3],depth;
            body_chain_closest_segment_pair(points[i],points[i+1],hip[b],thigh[b],&ct,&bt,p,q,&dist);
            if(i==0 && ct<.08f)continue;
            depth=.1025f-dist-.001f;deepest=fmaxf(deepest,depth);
            if(depth<=1e-6f || dist<1e-6f)continue;
            for(a=0;a<3;a++)normal[a]=(p[a]-q[a])/dist;
            body_chain_store_contact(c,&count,6,i,ct,fminf(depth,.05f),p,q,normal);
        }
        if(!count)break;
        body_contact_solve(&body_chain_physics_cfg,&s,points,c,count,3,corr);
        body_chain_live_points_with_delta_cfg(&body_chain_physics_cfg,points,corr,out);
        body_contact_apply(&s,&body_chain_physics_cfg,corr,3,deepest);
        memcpy(points,out,sizeof(points));
    }
    if(deepest>.00005f){fprintf(stderr,"FAIL: synthetic two-thigh recovery stalled: remaining penetration=%g\n",deepest);exit(1);}
    printf("PASS: synthetic embedded chain escapes two thigh capsules in %d bounded updates; residual=%g\n",frame,deepest);
}
static void deep_overlap(void){
    /* The game's failed candidate used a 45-degree per-joint setting and
       contacts within 0.0002 units of the opposing chain's centerline. */
    const float logged[4][3]={{0,0,0},{-.00251f,-.00008f,-.07645f},
        {-.08025f,-.00017f,-.11959f},{-.12943f,-.00020f,-.11576f}};
    int n,j,a;float largest=0;
    body_chain_collider_cfg.response_max_degrees_per_tick=45;
    body_chain_collider_cfg.response_strength=2;
    for(n=0;n<200;n++){
        body_chain_person_state_t s={0};body_chain_contact_t c[3];
        float correction[3][2],out[4][3],norm=0,normal[3];
        float phase=(float)n*.151f;
        normal[0]=cosf(phase)*.01f;normal[1]=sinf(phase);normal[2]=cosf(phase)*.99995f;
        support(&c[0],1,.369f,.01975f,normal,logged);
        normal[0]=-normal[0];normal[1]=-normal[1];normal[2]=-normal[2];
        support(&c[1],2,.7f,.025f,normal,logged);
        normal[0]=1;normal[1]=normal[2]=0;
        support(&c[2],0,.15f,.03f,normal,logged);
        body_contact_solve(&body_chain_physics_cfg,&s,logged,c,3,3,correction);
        body_chain_live_points_with_delta_cfg(&body_chain_physics_cfg,logged,correction,out);
        for(j=0;j<3;j++)for(a=0;a<2;a++)norm+=correction[j][a]*correction[j][a];
        largest=fmaxf(largest,sqrtf(norm));
        if(norm>36.0001f || overlap_error(out,c,3)>overlap_error(logged,c,3)+1e-9f){
            fprintf(stderr,"FAIL deep overlap case=%d correction_norm=%g error=%g -> %g\n",n,sqrtf(norm),overlap_error(logged,c,3),overlap_error(out,c,3));
            exit(1);
        }
    }
    printf("PASS: 200 deep/conflicting supports with 45-degree configuration reduce overlap; largest chain correction=%g degrees\n",largest);
    body_chain_collider_cfg.response_max_degrees_per_tick=20;
    body_chain_collider_cfg.response_strength=1;
}
static void warmup_geometry(void){
    testicle_physics_cfg=body_chain_physics_cfg;
    for(int testicle=0;testicle<=1;testicle++){
        body_chain_person_state_t s={0};body_chain_collider_person_state_t *c=&body_chain_collider_states[0];
        float published[3][3]={{0}};memset(c,0,sizeof(*c));
        c->basis_valid=1;c->chain_points_ready=c->chain_points_fresh=c->testicle_points_ready=1;
        c->chain_points_update_tick=c->testicle_points_update_tick=1000;
        for(int j=0;j<4;j++){
            c->chain_point_valid[j]=1;c->chain_local_point[j][0]=-.1f*j;
            if(j<3){c->testicle_joint_position[j][0]=-.1f*j;s.joint_raw[j]=published[j];}
        }
        body_contact_begin_step(0,&s,testicle,1000,.016f);
        assert(!c->ready && s.dynamics_valid && s.collision_pose_valid);
        assert(s.dynamics.segments==(testicle?2:3));
        body_dynamics_t before=s.dynamics;
        c->ready=1;c->chain_points_update_tick=c->testicle_points_update_tick=1016;
        body_contact_begin_step(0,&s,testicle,1016,.016f);
        assert(s.dynamics_valid && !memcmp(&before,&s.dynamics,sizeof(before)));
        /* Invalid basis, held/unconfirmed or old pivots cannot initialize a
           chain before the collision gate. Previously prepared dynamics survive. */
        c->ready=0;c->basis_valid=0;
        body_contact_begin_step(0,&s,testicle,1032,.016f);assert(!s.collision_step_valid);
        assert(!memcmp(&before,&s.dynamics,sizeof(before)));
        c->basis_valid=1;c->chain_points_update_tick=c->testicle_points_update_tick=1048;
        s.gravity_camera_hold_active=1;
        body_contact_begin_step(0,&s,testicle,1048,.016f);assert(!s.collision_step_valid);
        s.gravity_camera_hold_active=0;pivot_hold=1;
        c->chain_points_update_tick=c->testicle_points_update_tick=1064;
        body_contact_begin_step(0,&s,testicle,1064,.016f);assert(!s.collision_step_valid);
        pivot_hold=0;body_contact_begin_step(0,&s,testicle,1080,.016f);assert(!s.collision_step_valid);
        c->chain_points_update_tick=c->testicle_points_update_tick=1096;
        body_contact_begin_step(0,&s,testicle,1096,.016f);assert(s.collision_pose_valid && !c->ready);
        if(!testicle){
            c->chain_points_fresh=0;c->chain_points_update_tick=1112;
            body_contact_begin_step(0,&s,0,1112,.016f);assert(!s.collision_step_valid);
            c->chain_points_ready=0;
            body_contact_begin_step(0,&s,0,1128,.016f);assert(!s.collision_step_valid);
        }
    }
    assert(frame_updates>0);
    puts("PASS: production capture/begin initializes both chain models before collision promotion from current engine pivots only; ready toggle preserves model, unsafe/held/synthetic samples rejected");
}
int main(void){configure();warmup_geometry();return 0;}
