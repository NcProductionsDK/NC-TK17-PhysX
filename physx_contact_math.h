#ifndef PHYSX_CONTACT_MATH_H
#define PHYSX_CONTACT_MATH_H

#include <math.h>
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

/* Small, allocation-free contact solve shared by body and room queries.
   Normals are unit vectors pointing out of the obstacle. */
#define PHYSX_CONTACT_CAPACITY 8
typedef struct physx_contact_set_t {
    int count;
    float normal[PHYSX_CONTACT_CAPACITY][3];
    float penetration[PHYSX_CONTACT_CAPACITY];
} physx_contact_set_t;

static float physx_contact_dot(const float a[3], const float b[3])
{
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

/* Match the soft band at BOTH boundaries. Previously crossing -slop made
   the response drop from 0.08 * span to zero, creating negative stiffness. */
static float physx_contact_depth(float margin, float slop, float soft_margin)
{
    float span;
    if (slop < 0.0f) slop = 0.0f;
    if (soft_margin < 0.0f) soft_margin = 0.0f;
    if (margin >= soft_margin) return 0.0f;
    span = soft_margin + slop;
    if (margin < -slop) return -slop - margin + 0.08f * span;
    return span > 0.000001f ?
        0.08f * (soft_margin-margin) * (soft_margin-margin) / span : 0.0f;
}

/* Return the changed slot, or -1 when the existing support wins. */
static int physx_contact_store(physx_contact_set_t *set,
                                const float normal[3], float depth)
{
    int i, slot = -1, weakest = 0;
    if (depth <= 0.000001f) return -1;
    for (i = 0; i < set->count; i++) {
        /* Only deduplicate essentially coplanar supports. Averaging normals
           at corners changes the actual constraints as contact count varies. */
        if (physx_contact_dot(set->normal[i], normal) > 0.9999f) {
            if (depth > set->penetration[i]) {
                memcpy(set->normal[i], normal, sizeof(float)*3);
                set->penetration[i] = depth;
                return i;
            }
            return -1;
        }
        if (set->penetration[i] < set->penetration[weakest]) weakest = i;
    }
    if (set->count < PHYSX_CONTACT_CAPACITY) slot = set->count++;
    else if (depth > set->penetration[weakest]) slot = weakest;
    if (slot < 0) return -1;
    memcpy(set->normal[slot], normal, sizeof(float)*3);
    set->penetration[slot] = depth;
    return slot;
}

/* Projected dual solve: minimize displacement subject to n.dx >= depth.
   Updating accumulated nonnegative multipliers avoids counting the same
   correction twice where spheres/capsules or room triangles overlap.
   Bounded multipliers/iterations keep contradictory two-sided meshes finite. */
static void physx_contact_resolve_weights(const physx_contact_set_t *set,
    float out[3], float lambda[PHYSX_CONTACT_CAPACITY])
{
    int iteration, i, axis;
    memset(lambda, 0, sizeof(float)*PHYSX_CONTACT_CAPACITY);
    memset(out, 0, sizeof(float)*3);
    for (iteration = 0; iteration < 32; iteration++) {
        float largest = 0.0f;
        for (i = 0; i < set->count; i++) {
            float next = lambda[i] + set->penetration[i] -
                         physx_contact_dot(set->normal[i], out);
            float change;
            if (next < 0.0f) next = 0.0f;
            if (next > 1.0f) next = 1.0f;
            change = next - lambda[i];
            lambda[i] = next;
            if (fabsf(change) > largest) largest = fabsf(change);
            for (axis = 0; axis < 3; axis++)
                out[axis] += set->normal[i][axis] * change;
        }
        if (largest < 0.0000001f) break;
    }
}

static void physx_contact_resolve(const physx_contact_set_t *set, float out[3])
{
    float lambda[PHYSX_CONTACT_CAPACITY];
    physx_contact_resolve_weights(set,out,lambda);
}

/* Closest point on the link sphere satisfying a contact plane. A push followed
   by radial normalization loses part of the separation on every iteration.
   Infeasible contacts clamp to the reachable pole; the upstream solve must
   move the anchor. Do not invent a sideways direction at a singular pole. */
static void physx_contact_link_position(float offset[3], float length,
                                       const float correction[3])
{
    float depth = sqrtf(physx_contact_dot(correction, correction));
    float normal[3], tangent[3], height, radius, tangent_len;
    int axis;
    if (length <= 0.0001f || depth <= 0.000001f) return;
    for (axis = 0; axis < 3; axis++) normal[axis] = correction[axis]/depth;
    height = physx_contact_dot(offset, normal);
    for (axis = 0; axis < 3; axis++)
        tangent[axis] = offset[axis] - normal[axis]*height;
    tangent_len = sqrtf(physx_contact_dot(tangent, tangent));
    height += depth;
    if (height > length) height = length;
    if (height < -length) height = -length;
    radius = sqrtf(fmaxf(0.0f, length*length-height*height));
    if (tangent_len < 0.000001f && radius > 0.000001f) return;
    for (axis = 0; axis < 3; axis++)
        offset[axis] = normal[axis]*height +
            (tangent_len > 0.000001f ? tangent[axis]*radius/tangent_len : 0.0f);
}

/* Inelastic unilateral response on a fixed-length link. Project the contact
   normal into the link's tangent plane FIRST, so restoring link length cannot
   reintroduce inward velocity. Preserve separation and surface sliding. */
static void physx_contact_link_velocity(float velocity[3],
                                       const float offset[3],
                                       const float normal[3])
{
    float len2 = physx_contact_dot(offset, offset);
    float tangent[3], radial, normal_radial, effective, inward;
    int axis;
    if (len2 < 0.00000001f) return;
    radial = physx_contact_dot(velocity, offset) / len2;
    normal_radial = physx_contact_dot(normal, offset) / len2;
    for (axis = 0; axis < 3; axis++) {
        velocity[axis] -= offset[axis] * radial;
        tangent[axis] = normal[axis] - offset[axis] * normal_radial;
    }
    effective = physx_contact_dot(tangent, tangent);
    inward = physx_contact_dot(velocity, normal);
    if (inward < 0.0f && effective > 0.000001f) {
        float normal_impulse = -inward / sqrtf(effective);
        float speed, friction_scale;
        for (axis = 0; axis < 3; axis++)
            velocity[axis] -= tangent[axis] * inward / effective;
        /* Coulomb friction, bounded by the supporting impulse. After the
           normal solve this velocity is tangent to both link and surface.
           No inward impulse means no friction (including separating motion),
           and a repeated query cannot apply fixed per-iteration damping. */
        speed = sqrtf(physx_contact_dot(velocity, velocity));
        friction_scale = speed > 0.0000001f ?
            fmaxf(0.0f, 1.0f - 0.25f * normal_impulse / speed) : 0.0f;
        for (axis = 0; axis < 3; axis++) velocity[axis] *= friction_scale;
    }
}

#endif
