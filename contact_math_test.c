#include <assert.h>
#include <stdio.h>
#include "physx_contact_math.h"

static void near(float a, float b, float tolerance)
{
    if (fabsf(a-b) > tolerance) {
        fprintf(stderr, "Expected %.9g, got %.9g (tolerance %.9g)\n",
                b, a, tolerance);
        assert(0);
    }
}

static void response_curve(void)
{
    int i;
    float previous = 0.0f;
    for (i = 0; i <= 10000; i++) {
        float margin = 0.003f - i * 0.000002f;
        float depth = physx_contact_depth(margin, 0.001f, 0.002f);
        assert(depth >= previous);
        previous = depth;
    }
    near(physx_contact_depth(-0.001f-1e-8f, 0.001f, 0.002f),
         physx_contact_depth(-0.001f+1e-8f, 0.001f, 0.002f), 2e-8f);
    near(physx_contact_depth(0, 0, 0), 0, 0);
    near(physx_contact_depth(-0.01f, 0, 0), 0.01f, 1e-8f);
}

static void supports(void)
{
    physx_contact_set_t set = {0};
    float x[3] = {1,0,0}, y[3] = {0,1,0};
    float diagonal[3] = {0.6f,0.8f,0}, out[3], reversed[3];
    int i;
    for (i = 0; i < 100; i++) physx_contact_store(&set, y, 0.01f);
    assert(set.count == 1);
    physx_contact_resolve(&set, out);
    near(out[1], 0.01f, 1e-7f);
    physx_contact_store(&set, x, 0.02f);
    physx_contact_resolve(&set, out);
    near(out[0], 0.02f, 1e-7f);
    near(out[1], 0.01f, 1e-7f);
    memset(&set, 0, sizeof(set));
    physx_contact_store(&set, y, 0.01f);
    physx_contact_store(&set, diagonal, 0.01f);
    physx_contact_resolve(&set, out);
    near(out[1], 0.01f, 1e-6f);
    near(physx_contact_dot(out, diagonal), 0.01f, 1e-6f);
    /* Sum-of-normals gives (0.006, 0.018); minimum displacement is smaller. */
    assert(physx_contact_dot(out, out) < 0.00012f);
    memset(&set, 0, sizeof(set));
    physx_contact_store(&set, diagonal, 0.01f);
    physx_contact_store(&set, y, 0.01f);
    physx_contact_resolve(&set, reversed);
    for (i = 0; i < 3; i++) near(out[i], reversed[i], 1e-6f);
}

static void link_constraints(void)
{
    float offset[3] = {0.06f,-0.08f,0};
    float correction[3] = {0,0.01f,0}, normal[3] = {0,1,0};
    float velocity[3] = {-0.2f,-0.3f,0.04f};
    float outward[3];
    physx_contact_link_position(offset, 0.1f, correction);
    near(offset[1], -0.07f, 1e-7f);
    near(physx_contact_dot(offset, offset), 0.01f, 1e-7f);
    physx_contact_link_velocity(velocity, offset, normal);
    near(physx_contact_dot(velocity, offset), 0, 1e-7f);
    assert(physx_contact_dot(velocity, normal) >= -1e-7f);
    assert(velocity[2] >= 0 && velocity[2] <= 0.04f);
    outward[0] = -offset[1]; outward[1] = offset[0]; outward[2] = 0;
    physx_contact_link_velocity(outward, offset, normal);
    near(outward[1], offset[0], 1e-7f); /* separation survives */
    /* Unreachable and singular contacts must stay finite and keep length. */
    correction[1] = 1.0f;
    physx_contact_link_position(offset, 0.1f, correction);
    near(offset[1], 0.1f, 1e-7f);
    offset[0] = 0; offset[1] = -0.1f; offset[2] = 0;
    correction[1] = 0.01f;
    physx_contact_link_position(offset, 0.1f, correction);
    near(physx_contact_dot(offset, offset), 0.01f, 1e-7f);
}

static void friction_response(void)
{
    float offset[3]={1,0,0}, normal[3]={0,1,0};
    float velocity[3]={0,-0.1f,0.2f};
    physx_contact_link_velocity(velocity,offset,normal);
    near(velocity[1],0,1e-7f);
    near(velocity[2],0.175f,1e-7f); /* mu * normal impulse */
    physx_contact_link_velocity(velocity,offset,normal);
    near(velocity[2],0.175f,1e-7f); /* duplicate query adds no damping */
    velocity[1]=0.1f;
    physx_contact_link_velocity(velocity,offset,normal);
    near(velocity[1],0.1f,1e-7f);
    near(velocity[2],0.175f,1e-7f); /* lift-off unaffected */
    velocity[1]=-0.1f; velocity[2]=0.01f;
    physx_contact_link_velocity(velocity,offset,normal);
    near(velocity[2],0,1e-7f); /* small slip settles without reversing */
    {
        const float rates[]={30,60,144};
        int rate,frame;
        for(rate=0;rate<3;rate++) {
            float dt=1/rates[rate];
            velocity[0]=0; velocity[2]=0.4f;
            for(frame=0;frame<(int)rates[rate];frame++) {
                velocity[1]=-0.5f*dt;
                physx_contact_link_velocity(velocity,offset,normal);
            }
            near(velocity[2],0.275f,2e-6f);
        }
    }
}

static void resting_link(float dt)
{
    float offset[3] = {0.06f,-0.08f,0};
    float velocity[3] = {0,0,0}, normal[3] = {0,1,0};
    int frame, axis;
    /* Gravity presses the link against a floor for ten seconds, no sleep,
       no contact damping. Integrate and enforce length as the plugin does. */
    for (frame = 0; frame < (int)(10.0f/dt); frame++) {
        float len, correction[3] = {0,0,0};
        velocity[1] -= 0.5f * dt;
        for (axis = 0; axis < 3; axis++) offset[axis] += velocity[axis]*dt;
        len = sqrtf(physx_contact_dot(offset, offset));
        for (axis = 0; axis < 3; axis++) offset[axis] *= 0.1f/len;
        correction[1] = fmaxf(0.0f, -0.08f-offset[1]);
        physx_contact_link_position(offset, 0.1f, correction);
        physx_contact_link_velocity(velocity, offset, normal);
        near(offset[1], -0.08f, 2e-6f);
        near(offset[0], 0.06f, 2e-6f);
        near(physx_contact_dot(velocity, velocity), 0, 1e-8f);
    }
}

int main(void)
{
    response_curve();
    supports();
    link_constraints();
    friction_response();
    resting_link(1.0f/30.0f);
    resting_link(1.0f/60.0f);
    resting_link(1.0f/144.0f);
    puts("PASS: response continuity, duplicate/oblique/corner supports, contact order,");
    puts("link length/separation, sliding/release, singular contacts, resting at 30/60/144 Hz");
    return 0;
}
