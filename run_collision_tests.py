"""Compile the contact math and actual room-query source without launching TK17."""
import os
from pathlib import Path
import subprocess
import re

root = Path(__file__).resolve().parent
build = root / 'build'
build.mkdir(exist_ok=True)
gcc = Path(r'C:\msys64\mingw32\bin\gcc.exe')
env = dict(os.environ)
env['PATH'] = str(gcc.parent) + os.pathsep + env['PATH']
sidecar_source = (root / 'physx_sidecar.c').read_text()
length_start = sidecar_source.index('static float physx_sqrtf(')
production_lengths = sidecar_source[length_start:
    sidecar_source.index('static int addon_output_scene_visible(', length_start)]

def use_production_lengths(fixture):
    return fixture.replace(
        'static float physx_vec3_len(const float a[3]) { return sqrtf(vec3_dot(a,a)); }',
        production_lengths)

def run_test(source, name):
    exe = build / (name + '.exe')
    subprocess.run([str(gcc), '-m32', '-O2', '-Wall', '-Wextra',
                    '-Werror', '-Wno-unused-function', '-static-libgcc',
                    '-I', str(root), '-o', str(exe), str(source)],
                   env=env, check=True)
    subprocess.run([str(exe)], env=env, check=True)

run_test(root / 'contact_math_test.c', 'contact_math_test')
source = (root / 'physx_room_collision.c').read_text()
types = source[source.index('typedef struct room_collision_triangle_t'):source.index('typedef struct room_collision_config_t')]
queries = source[source.index('static float room_collision_aabb_distance_sq'):source.index('static int room_collision_world_vector_to_body_local')]
fixture = r'''
#include <assert.h>
#include <stdio.h>
#include "physx_contact_math.h"
static float vec3_dot(const float a[3], const float b[3]) { return physx_contact_dot(a,b); }
static float physx_vec3_len(const float a[3]) { return sqrtf(vec3_dot(a,a)); }
static float physx_clampf(float x, float a, float b) { return fmaxf(a,fminf(b,x)); }
static int sane_probe_float(float x) { return isfinite(x); }
static int room_collision_is_enabled(void) { return 1; }
static struct { float world_gravity[3]; } physics_environment_cfg = {{0,-1,0}};
'''
fixture += types
fixture += r'''
static room_collision_triangle_t triangles[3];
static room_collision_triangle_t *room_collision_triangles = triangles;
static room_collision_bvh_node_t nodes[1];
static room_collision_bvh_node_t *room_collision_bvh = nodes;
'''
fixture += queries
fixture += source[source.index("static int single_bone_room_contacts("):] 
fixture += r'''
int main(void) {
    const float floor_vertices[3][3] = {{-2,0,-2},{0,0,2},{2,0,-2}};
    const float wall_vertices[3][3] = {{0,-2,-2},{0,2,-2},{0,0,2}};
    float center[3] = {0.015f,0.015f,0}, correction[3], repeat[3];
    float start[3] = {0.2f,0.1f,0}, end[3] = {0.2f,-0.1f,0};
    int axis;
    {
        const float sizes[] = {0,1e-8f,1e-6f,1e-5f,1e-4f,0.001f,0.01f,1,100,10000};
        unsigned i;
        for (i=0;i<sizeof(sizes)/sizeof(sizes[0]);i++) {
            float vector[3] = {sizes[i],0,0};
            assert(fabsf(physx_vec3_len(vector)-sizes[i]) <=
                   fmaxf(1e-12f,sizes[i]*2e-6f));
        }
    }
    memcpy(triangles[0].v, floor_vertices, sizeof(floor_vertices));
    memcpy(triangles[1].v, wall_vertices, sizeof(wall_vertices));
    triangles[2] = triangles[0]; /* duplicated room mesh */
    nodes[0].count = 3;
    for (axis = 0; axis < 3; axis++) {
        nodes[0].minv[axis] = -2;
        nodes[0].maxv[axis] = 2;
    }
    physx_contact_set_t single={0};
    assert(single_bone_room_contacts(center,0.025f,&single) && single.count==2);
    assert(room_collision_resolve_sphere(center, 0.025f, correction, NULL));
    float single_push[3];physx_contact_resolve(&single,single_push);
    for(axis=0;axis<3;axis++) assert(fabsf(single_push[axis]-correction[axis])<1e-6f);
    assert(fabsf(correction[0]-0.01f) < 1e-6f);
    assert(fabsf(correction[1]-0.01f) < 1e-6f);
    for (axis = 0; axis < 3; axis++) center[axis] += correction[axis];
    room_collision_resolve_sphere(center, 0.025f, repeat, NULL);
    assert(physx_vec3_len(repeat) < 2e-6f);
    assert(room_collision_resolve_swept_sphere(start,end,0.025f,correction,NULL));
    assert(end[1]+correction[1] >= 0.025f-1e-6f);
    assert(fabsf(correction[0]) < 1e-6f);
    /* Shallow resting contact must still yield a full unit normal. */
    nodes[0].count=1;
    for (axis=0;axis<3;axis++) {
        float depth=axis==0 ? 0.000002f : (axis==1 ? 0.00001f : 0.0001f);
        center[0]=0.2f; center[1]=0.025f-depth; center[2]=0;
        assert(room_collision_resolve_sphere(center,0.025f,correction,NULL));
        assert(fabsf(correction[1]-depth)<1e-7f);
        assert(fabsf(correction[1]/physx_vec3_len(correction)-1.0f)<1e-6f);
    }
    puts("PASS: production vector lengths across scales and micrometre-scale room separation");
    puts("PASS: production room query, duplicate triangles, floor/wall corner, repeated separation, thin-floor sweep");
    return 0;
}
'''
generated = build / 'room_collision_query_test.c'
generated.write_text(use_production_lengths(fixture))
run_test(generated, 'room_collision_query_test')

sidecar = (root / 'physx_sidecar.c').read_text()
def function(name, source=None):
    source = sidecar if source is None else source
    match = re.search(r'static\s+[^;{}]*\b' + re.escape(name) + r'[^;{}]*\{', source)
    assert match, name
    start = match.start()
    brace = match.end()-1
    depth = 1
    end = brace + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end] + '\n'

integration = r'''
#include <assert.h>
#include <stdio.h>
#include "physx_contact_math.h"
typedef unsigned long DWORD;
typedef struct {
    float sim_offset[3], sim_velocity[3], sim_length;
    int addon_simulated_target, addon_visual_pose_valid;
    int sim_contact_corrected, sim_output_velocity_initialized;
    DWORD contact_basis_tick;
    DWORD contact_pivot_tick;
    float contact_pivot_body[3];
    DWORD contact_terminal_tick,contact_terminal_log_tick;
    float contact_terminal_body[3];
    float contact_live_body_basis[9],contact_parent_body_basis[9];
    float sim_rest[3],joint_min_angle[3],joint_max_angle[3];
    int joint_settings_initialized;
    char name[32];
    void *object;
    DWORD addon_visual_tick;
    float sim_output_offset_prev[3], sim_rotation_rest[3];
    float addon_visual_rotation[3], sim_output_velocity[3];
} physx_target_t;
typedef struct {
    int addon_chain; float collision_radius;
    int skinned_matrix_enabled, target_count, collision_enabled;
    float skinned_matrix_scale;
    physx_target_t *targets;
    int rotation_solver_full_angle, h_axis, v_axis;
    float limit_angle,root_bend_scale;
    char parent_name[32],name[32];
} physx_chain_t;
static float vec3_dot(const float a[3], const float b[3]) { return physx_contact_dot(a,b); }
static float physx_vec3_len(const float a[3]) { return sqrtf(vec3_dot(a,a)); }
static float addon_vec3_len_exact(const float a[3]) { return physx_vec3_len(a); }
static float physx_clampf(float x, float a, float b) { return fmaxf(a,fminf(b,x)); }
static float physx_absf(float x) { return fabsf(x); }
static int sane_probe_float(float x) { return isfinite(x); }
static int physx_vec3_sane_limit(const float a[3],float limit) {
    return isfinite(a[0]) && isfinite(a[1]) && isfinite(a[2]) &&
           fabsf(a[0])<=limit && fabsf(a[1])<=limit && fabsf(a[2])<=limit;
}
'''
for name in ['addon_chain_collision_normalize_target(',
             'addon_chain_collision_proximal_deadzone(',
             'addon_chain_attachment_contact_weight(',
             'addon_chain_apply_collision_correction(',
             'addon_chain_body_collision_response_margin(',
             'addon_chain_update_output_velocity(',
             'addon_chain_visible_room_correction_to_sim_delta(']:
    integration += function(name)
integration += r'''
static int publications;
static void addon_chain_contact_output_angles(physx_chain_t *chain, physx_target_t *target,
    float out[3], float *pitch, float *roll) {
    (void)chain; (void)pitch; (void)roll;
    memcpy(out,target->sim_offset,sizeof(float)*3);
}
static void physx_addon_apply_target_visual_pose(physx_target_t *target) {
    (void)target; publications++;
}
'''
integration += function('addon_chain_publish_final_contacts(')
integration += r'''
#define _stricmp stricmp
static int addon_chain_target_gravity_tail_axis(physx_chain_t *chain,physx_target_t *target,int vertical) {
    (void)target; return vertical ? chain->v_axis : chain->h_axis;
}
'''
for name in ['addon_chain_bend_to_output_angles(',
             'addon_chain_full_angle_output_delta_at(',
             'addon_chain_root_visual_roll_inverted(',
             'addon_chain_root_physx_target(',
             'addon_chain_contact_output_angles_at(',
             'addon_chain_contact_predict(',
             'addon_chain_contact_point_delta(',
             'addon_chain_solve_body_contact(']:
    integration += function(name)
integration += r'''
typedef struct { int unused; } physx_sidecar_t;
typedef struct { int ready,basis_valid; float basis_h[3],basis_v[3],basis_s[3]; } body_chain_collider_person_state_t;
static body_chain_collider_person_state_t body_chain_collider_states[4];
static int addon_chain_collision_first_target_index(physx_chain_t *chain) { (void)chain; return 1; }
static int addon_chain_target_body_local(physx_sidecar_t *sc,physx_chain_t *chain,
    physx_target_t *target,int person,const body_chain_collider_person_state_t *state,float out[3]) {
    (void)sc;(void)chain;(void)target;(void)person;(void)state;
    out[0]=out[1]=out[2]=0; return 1;
}
'''
integration += function('addon_chain_apply_visible_body_correction(')
integration += '\n#include "physx_chain_contact.c"\n'
integration += r'''
typedef struct {
    physx_contact_set_t contacts;
    float point[PHYSX_CONTACT_CAPACITY][3];
    int angular;
} addon_chain_body_manifold_t;
'''
integration += function('addon_chain_body_manifold_store(')
integration += function('addon_chain_body_manifold_resolve(')
integration += function('addon_chain_body_manifold_point_requests(')
integration += r'''
static int addon_chain_collision_person_index(physx_sidecar_t *sc,physx_chain_t *chain) {
    (void)sc;(void)chain; return 0;
}
static int body_chain_read_mat3_rows(void *object,float rows[9]) {
    memcpy(rows,object,sizeof(float)*9); return 1;
}
/* Engine-only terminal-axis discovery is exercised in game. Candidate tip
   geometry and response are covered independently in chain_contact_test.c. */
static int addon_chain_terminal_endpoint_body_local(physx_chain_t *chain,
    physx_target_t *target,const body_chain_collider_person_state_t *state,
    const float previous[3],const float start[3],float end[3]) {
    (void)chain;(void)target;(void)state;(void)previous;(void)start;(void)end; return 0;
}
'''
integration += function('addon_basis_rows_orthonormal_enough(')
integration += function('addon_normalize_basis_rows(')
integration += function('addon_chain_capture_contact_basis(')
integration += r'''
static float captured_camera_inverse[16];
static int captured_camera_inverse_valid=1;
static int body_collider_view_delta_to_local(const float v[3],const float h[3],
    const float vertical[3],const float s[3],float out[3]) {
    out[0]=vec3_dot(v,h); out[1]=vec3_dot(v,vertical); out[2]=vec3_dot(v,s); return 1;
}
'''
room_source = (root / 'physx_room_collision.c').read_text()
start = room_source.index('static int room_collision_world_vector_to_body_local(')
end = room_source.index('\n}', start)+2
integration += room_source[start:end]+'\n'
integration += function('addon_chain_solve_room_contact(')
integration += function('addon_chain_room_contact_request(')
integration += r'''
int main(void) {
    physx_chain_t chain = {.addon_chain=1,.collision_radius=0.025f};
    physx_target_t target = {.sim_offset={0.06f,-0.08f,0},
        .sim_velocity={-0.2f,-0.3f,0.04f},.sim_length=0.1f};
    float push[3] = {0,0.01f,0}, delta[3];
    float start[3] = {0,0,0}, end[3] = {0.06f,-0.08f,0};
    int axis;
    addon_chain_apply_collision_correction(&chain,&target,push);
    assert(target.sim_contact_corrected);
    assert(fabsf(target.sim_offset[1]+0.07f)<1e-6f);
    assert(fabsf(physx_vec3_len(target.sim_offset)-0.1f)<1e-6f);
    assert(target.sim_velocity[1]>=-1e-6f);
    assert(target.sim_velocity[2]>=0 && target.sim_velocity[2]<=0.04f);
    memcpy(target.sim_offset,end,sizeof(end));
    assert(addon_chain_visible_room_correction_to_sim_delta(&target,start,end,push,delta));
    for (axis=0;axis<3;axis++) target.sim_offset[axis] += delta[axis];
    assert(fabsf(target.sim_offset[1]+0.07f)<1e-6f);
    assert(fabsf(physx_vec3_len(target.sim_offset)-0.1f)<1e-6f);
    target.sim_length = 0;
    addon_chain_apply_collision_correction(&chain,&target,push);
    assert(fabsf(target.sim_offset[1]+0.06f)<1e-6f);
    target.sim_length = 0.1f;
    target.addon_simulated_target = target.addon_visual_pose_valid = 1;
    target.addon_visual_tick = 123;
    chain.skinned_matrix_enabled = chain.target_count = 1;
    chain.skinned_matrix_scale = 1;
    chain.targets = &target;
    addon_chain_publish_final_contacts(&chain,123);
    assert(publications==1);
    assert(target.addon_visual_rotation[1]==target.sim_offset[1]);
    assert(target.sim_output_velocity[1]==target.sim_velocity[1]);
    addon_chain_publish_final_contacts(&chain,123);
    assert(publications==1); /* unchanged output isn't republished */
    target.sim_offset[1] += 0.01f;
    addon_chain_publish_final_contacts(&chain,124);
    assert(publications==1); /* stale/unwritten targets aren't published */
    {
        float previous = 0.0f;
        int sample;
        /* Exercise the production margin path, not just the shared curve:
           the old sidecar-only gate skipped the entire shallow band. */
        for (sample=0;sample<=400;sample++) {
            float margin = 0.002f-sample*0.00001f;
            float response = addon_chain_body_collision_response_margin(
                &chain,margin-0.005f,0.005f,0.0f);
            float depth = physx_contact_depth(response,0.001f,0.002f);
            assert(depth >= previous-1e-8f);
            assert(depth-previous < 0.000011f);
            previous = depth;
        }
        assert(physx_contact_depth(addon_chain_body_collision_response_margin(
            &chain,0.0f,0.0f,0.0f),0.001f,0.002f)>0.0f);
    }
    {
        const float rates[3] = {30,60,144};
        int rate;
        for (rate=0;rate<3;rate++) {
            float dt=1.0f/rates[rate];
            target.sim_contact_corrected=1;
            target.sim_velocity[0]=0.02f; /* real sliding */
            target.sim_output_offset_prev[0]=target.sim_offset[0]-0.001f;
            addon_chain_update_output_velocity(&target,dt,1);
            assert(fabsf(target.sim_output_velocity[0]-0.02f)<1e-7f);
            assert(target.sim_output_offset_prev[0]==target.sim_offset[0]);
            target.sim_contact_corrected=0; /* contact released */
            target.sim_offset[0]+=0.04f*dt;
            target.sim_velocity[0]=0.04f;
            addon_chain_update_output_velocity(&target,dt,1);
            assert(fabsf(target.sim_output_velocity[0]-0.04f)<1e-5f);
            target.sim_offset[0]+=0.03f*dt;
            addon_chain_update_output_velocity(&target,dt,0); /* older rigid path */
            assert(fabsf(target.sim_output_velocity[0]-0.03f)<1e-5f);
        }
    }
    puts("PASS: production sidecar correction, velocity response, room visible-to-solver mapping, unconstrained target");
    puts("PASS: final upstream pose publication, no duplicate/stale writes, no depenetration velocity injection");
    puts("PASS: production shallow-contact continuity and contact/release velocity inheritance at 30/60/144 Hz");
    {
        int sample;
        for (sample=0;sample<3;sample++) {
            float depth=sample==0 ? 0.000002f : (sample==1 ? 0.00001f : 0.0001f);
            float shallow_push[3]={0,depth,0};
            target.sim_offset[0]=0.06f; target.sim_offset[1]=-0.08f; target.sim_offset[2]=0;
            target.sim_velocity[0]=-0.02f; target.sim_velocity[1]=-0.03f; target.sim_velocity[2]=0.004f;
            addon_chain_apply_collision_correction(&chain,&target,shallow_push);
            assert(fabsf(target.sim_offset[1]-(-0.08f+depth))<1e-7f);
            assert(target.sim_velocity[1]>=-1e-7f);
            assert(target.sim_velocity[2]>=0 && target.sim_velocity[2]<=0.004f);
        }
    }
    puts("PASS: production shallow sidecar contact cancels inward velocity and preserves sliding");
    {
        physx_target_t joints[3]={0};
        physx_sidecar_t sc={0};
        float endpoint[3]={0.06f,-0.08f,0}, push[3]={0,0.001f,0}, accepted[3];
        chain.targets=joints; chain.target_count=3;
        chain.h_axis=0; chain.v_axis=1; chain.limit_angle=180; chain.root_bend_scale=1;
        joints[1].addon_simulated_target=1; joints[1].sim_length=0.1f;
        joints[1].contact_basis_tick=1;
        joints[1].contact_live_body_basis[0]=joints[1].contact_live_body_basis[4]=joints[1].contact_live_body_basis[8]=1;
        joints[1].contact_parent_body_basis[0]=joints[1].contact_parent_body_basis[4]=joints[1].contact_parent_body_basis[8]=1;
        memcpy(joints[1].sim_offset,endpoint,sizeof(endpoint));
        joints[2]=joints[1];
        assert(addon_chain_apply_visible_body_correction(&sc,&chain,&joints[2],0,
            endpoint,push,accepted,NULL,NULL,NULL,NULL));
        assert(joints[1].sim_contact_corrected);
        assert(memcmp(joints[2].sim_offset,endpoint,sizeof(endpoint))==0);
        assert(!joints[2].sim_contact_corrected);
        assert(accepted[1]>0 && accepted[1]<0.0006f);
        assert(fabsf(physx_vec3_len(joints[1].sim_offset)-0.1f)<1e-7f);
        assert(!addon_chain_apply_visible_body_correction(&sc,&chain,&joints[1],0,
            endpoint,push,accepted,NULL,NULL,NULL,NULL));
        assert(physx_vec3_len(accepted)==0);
    }
    puts("PASS: visible body pivot response moves preceding bone, preserves contacted bone and rejects fixed anchor");
    {
        int pose;
        for(pose=0;pose<8;pose++) {
            physx_target_t joints[3]={0},saved;
            float parent_angles[3]={25.0f*pose,-13.0f*pose,39.0f*pose};
            float local[3]={0.02f,0.01f,-0.15f},pivot[3]={0,0,0};
            float baseline[3],desired[3],trial[3],request[3],delta[3],achieved[3],gradient[3],angles[3],rows[9];
            float length,depth;
            int i,j,k;
            chain.targets=joints; chain.target_count=3;
            chain.h_axis=0; chain.v_axis=1;
            chain.rotation_solver_full_angle=pose&1;
            chain.root_bend_scale=1.3f; chain.skinned_matrix_scale=0.8f;
            chain.limit_angle=180;
            strcpy(chain.name,"tail"); strcpy(chain.parent_name,"anchor");
            strcpy(joints[0].name,"anchor"); strcpy(joints[1].name,"tail");
            joints[1].addon_simulated_target=1; joints[1].sim_length=0.1f;
            joints[1].sim_rest[2]=-0.1f;
            joints[1].sim_offset[0]=0.02f; joints[1].sim_offset[1]=0.03f;
            joints[1].sim_offset[2]=-sqrtf(0.01f-0.0004f-0.0009f);
            joints[1].sim_rotation_rest[0]=(pose&2) ? 170 : 0;
            joints[1].contact_basis_tick=1;
            physx_contact_rotation_rows(parent_angles,joints[1].contact_parent_body_basis);
            addon_chain_contact_output_angles_at(&chain,&joints[1],angles,NULL,NULL,joints[1].sim_offset);
            for(i=0;i<3;i++) angles[i]=joints[1].sim_rotation_rest[i]+(angles[i]-joints[1].sim_rotation_rest[i])*chain.skinned_matrix_scale;
            physx_contact_rotation_rows(angles,rows);
            for(i=0;i<3;i++) for(j=0;j<3;j++) {
                float sum=0;
                for(k=0;k<3;k++) sum+=rows[i*3+k]*joints[1].contact_parent_body_basis[k*3+j];
                joints[1].contact_live_body_basis[i*3+j]=sum;
            }
            addon_chain_contact_predict(&chain,&joints[1],joints[1].sim_offset,local,baseline);
            {
                float expected_parent[9],live_rows[9];
                physx_sidecar_t sc={0};
                memcpy(expected_parent,joints[1].contact_parent_body_basis,sizeof(expected_parent));
                memcpy(live_rows,joints[1].contact_live_body_basis,sizeof(live_rows));
                joints[1].object=live_rows; joints[1].addon_visual_pose_valid=1;
                joints[1].addon_visual_tick=99;
                memcpy(joints[1].addon_visual_rotation,angles,sizeof(angles));
                body_chain_collider_states[0].ready=body_chain_collider_states[0].basis_valid=1;
                body_chain_collider_states[0].basis_h[0]=1;
                body_chain_collider_states[0].basis_v[1]=1;
                body_chain_collider_states[0].basis_s[2]=1;
                chain.collision_enabled=1;
                addon_chain_capture_contact_basis(&sc,&chain,100);
                assert(joints[1].contact_basis_tick==100);
                for(i=0;i<9;i++) assert(fabsf(joints[1].contact_parent_body_basis[i]-expected_parent[i])<1e-6f);
                joints[1].object=NULL;
            }
            memcpy(trial,joints[1].sim_offset,sizeof(trial)); trial[0]+=0.0003f;
            length=physx_vec3_len(trial); for(i=0;i<3;i++) trial[i]*=0.1f/length;
            addon_chain_contact_predict(&chain,&joints[1],trial,local,desired);
            for(i=0;i<3;i++) request[i]=desired[i]-baseline[i];
            depth=physx_vec3_len(request);
            assert(depth>0.000001f);
            saved=joints[1];
            assert(addon_chain_solve_body_contact(&chain,&joints[1],pivot,baseline,request,delta,achieved,gradient));
            assert(memcmp(&saved,&joints[1],sizeof(saved))==0); /* probes never mutate target */
            assert(vec3_dot(achieved,request)/depth>depth*0.95f);
            {
                float halfway[3],motion[3],endpoint_motion[3],anchor_motion[3];
                for(i=0;i<3;i++) halfway[i]=pivot[i]+0.5f*(baseline[i]-pivot[i]);
                addon_chain_contact_point_delta(&chain,&joints[1],pivot,baseline,delta,endpoint_motion);
                addon_chain_contact_point_delta(&chain,&joints[1],pivot,halfway,delta,motion);
                addon_chain_contact_point_delta(&chain,&joints[1],pivot,pivot,delta,anchor_motion);
                for(i=0;i<3;i++) {
                    assert(fabsf(endpoint_motion[i]-achieved[i])<1e-7f);
                    assert(fabsf(motion[i]-0.5f*endpoint_motion[i])<1e-7f);
                    assert(anchor_motion[i]==0);
                }
                assert(memcmp(&saved,&joints[1],sizeof(saved))==0);
                {
                    physx_sidecar_t sc={0};
                    float accepted_point[3],sample_point[3];
                    int sample_joints=0;
                    assert(addon_chain_apply_visible_body_correction(&sc,&chain,&joints[2],0,
                        baseline,request,accepted_point,halfway,sample_point,&sample_joints,halfway));
                    assert(sample_joints==1);
                    /* Full request is now evaluated at the half-lever contact;
                       old endpoint solve would deliver only half this amount. */
                    assert(vec3_dot(sample_point,request)/depth>depth*0.50f);
                    for(i=0;i<3;i++)
                        assert(fabsf(accepted_point[i]-2.0f*sample_point[i])<2e-7f);
                    assert(fabsf(physx_vec3_len(joints[1].sim_offset)-0.1f)<1e-7f);
                    joints[1]=saved;
                }
            }
            {
                float world_pivot[3]={1,-2,3},world_end[3],world_request[3],room_delta[3],room_gradient[3];
                captured_camera_inverse[1]=1; captured_camera_inverse[4]=-1;
                captured_camera_inverse[10]=1;
                world_end[0]=world_pivot[0]-baseline[1];
                world_end[1]=world_pivot[1]+baseline[0];
                world_end[2]=world_pivot[2]+baseline[2];
                world_request[0]=-request[1]; world_request[1]=request[0]; world_request[2]=request[2];
                assert(addon_chain_solve_room_contact(&chain,&joints[1],&body_chain_collider_states[0],
                    world_pivot,world_end,world_request,room_delta,room_gradient));
                for(i=0;i<3;i++) assert(fabsf(room_delta[i]-delta[i])<2e-6f);
                assert(memcmp(&saved,&joints[1],sizeof(saved))==0);
                captured_camera_inverse_valid=0;
                assert(!addon_chain_solve_room_contact(&chain,&joints[1],&body_chain_collider_states[0],
                    world_pivot,world_end,world_request,room_delta,room_gradient));
                captured_camera_inverse_valid=1;
            }
            for(i=0;i<3;i++) trial[i]=joints[1].sim_offset[i]+delta[i];
            assert(fabsf(physx_vec3_len(trial)-0.1f)<1e-7f);
            addon_chain_contact_predict(&chain,&joints[1],trial,local,desired);
            for(i=0;i<3;i++) assert(fabsf((desired[i]-baseline[i])-achieved[i])<1e-7f);
            chain.limit_angle=0;
            assert(!addon_chain_solve_body_contact(&chain,&joints[1],pivot,baseline,request,delta,achieved,gradient));
        }
    }
    puts("PASS: production output Jacobian across rotated frames, full/legacy angles, inverted roots, scale, fixed limits, and immutable probes");
    puts("PASS: contact-point probe matches endpoint solve, half-lever motion, fixed pivot, and never mutates simulation");
    puts("PASS: production body response separates half-lever contact with correct endpoint prediction across all output fixtures");
    {
        addon_chain_body_manifold_t m={0};
        float n[3]={1,0,0},other[3]={0,1,0},opposite[3]={-1,0,0};
        float p[3]={1,2,3},q[3]={4,5,6},weights[PHYSX_CONTACT_CAPACITY],resolved[3];
        float sum;
        int i;
        addon_chain_body_manifold_store(&m,n,0.001f,p);
        addon_chain_body_manifold_store(&m,n,0.0005f,q);
        assert(m.contacts.count==1 && m.point[0][0]==p[0]);
        addon_chain_body_manifold_store(&m,n,0.002f,q);
        assert(m.contacts.count==1 && m.point[0][0]==q[0]);
        addon_chain_body_manifold_point_requests(&m,weights);
        assert(fabsf(weights[0]-0.002f)<1e-7f);
        addon_chain_body_manifold_store(&m,other,0.001f,p);
        for(i=0;i<2;i++) {
            int j;
            sum=0;
            addon_chain_body_manifold_resolve(&m,resolved);
            addon_chain_body_manifold_point_requests(&m,weights);
            for(j=0;j<m.contacts.count;j++) { assert(weights[j]>=0); sum+=weights[j]; }
            assert(sum<=physx_vec3_len(resolved)+1e-7f);
            addon_chain_body_manifold_store(&m,opposite,0.002f,p);
        }
    }
    puts("PASS: support locations follow deduplication/deeper replacement; corner and opposing requests respect displacement budget");
    puts("PASS: production room contact mapping matches body solve across camera rotation and world translation; rejects missing camera basis");
    {
        float surface[3]={0,0.00026f,0},legacy[3]={0.00303f,0.00026f,0};
        float request[3];
        addon_chain_room_contact_request(surface,legacy,0.4f,1,request);
        assert(request[0]==0 && request[2]==0);
        assert(fabsf(request[1]-0.000104f)<1e-10f);
        addon_chain_room_contact_request(surface,legacy,0.4f,0,request);
        assert(fabsf(request[0]-0.001212f)<1e-9f);
        assert(fabsf(request[1]-0.000104f)<1e-10f);
    }
    puts("PASS: recorded shallow ground contact adds no artificial lateral request to output-aware solve; legacy fallback preserved");
    {
        float previous=0;
        int i;
        for(i=0;i<=1000;i++) {
            float t=0.07f+0.00006f*i;
            float weight=addon_chain_attachment_contact_weight(2,t,1,1);
            assert(weight>=previous && weight<=1);
            assert(weight-previous<0.003f);
            previous=weight;
        }
        assert(addon_chain_attachment_contact_weight(2,0.08f,1,1)==0);
        assert(addon_chain_attachment_contact_weight(2,0.121f,1,1)==1);
        assert(addon_chain_attachment_contact_weight(1,0.5f,1,1)==0);
        assert(addon_chain_attachment_contact_weight(2,0.085f,0,1)==1);
        assert(addon_chain_attachment_contact_weight(2,0.085f,1,0)==1);
        assert(addon_chain_attachment_contact_weight(2,0.085f,1,1)<0.05f);
    }
    puts("PASS: continuous attachment transition, fixed-anchor exclusion, unrelated and non-addon contacts unchanged");
    return 0;
}
'''
generated = build / 'sidecar_collision_test.c'
integration = integration.replace('int main(void) {',
    '#include "chain_contact_test.c"\nint main(void) {\n    test_coherent_chain_contacts();')
generated.write_text(use_production_lengths(integration))
run_test(generated, 'sidecar_collision_test')

liveness = r'''
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
typedef unsigned long DWORD;
typedef struct {
    int ready,scene_liveness_quarantined,scene_liveness_static_camera_samples;
    void *scene_liveness_quarantined_root_raw;
    long scene_liveness_camera_version;
    float scene_liveness_root_view[3];
    DWORD last_scene_liveness_log_tick;
} body_chain_collider_person_state_t;
static int captured_camera_inverse_valid=1;
static long captured_camera_version=1;
static struct { int health_log_ms; } body_chain_collider_cfg={1000};
#define log_line(...) ((void)0)
static int sane_probe_float(float x) { return isfinite(x); }
static float body_collider_distance(const float a[3],const float b[3]) {
    float x=a[0]-b[0],y=a[1]-b[1],z=a[2]-b[2];
    return sqrtf(x*x+y*y+z*z);
}
'''
# Logging is suppressed in this standalone fixture, leaving person unused.
liveness += function('body_chain_collider_scene_liveness(',
                     (root / 'physx_colliders.c').read_text()).replace(
                         '    float delta;', '    float delta; (void)person;')
liveness += r'''
int main(void) {
    body_chain_collider_person_state_t state={0};
    float root[3]={0};
    int object=0,i;
    state.ready=1;
    for(i=0;i<100;i++) {
        captured_camera_version++;
        root[0]+=0.00002977f; /* logged false-despawn displacement */
        assert(body_chain_collider_scene_liveness(&state,"Person02",&object,root,0,1,100+i));
        assert(!state.scene_liveness_quarantined);
    }
    state.scene_liveness_quarantined=1;
    state.scene_liveness_quarantined_root_raw=&object;
    assert(body_chain_collider_scene_liveness(&state,"Person02",&object,root,0,1,300));
    assert(!state.scene_liveness_quarantined); /* stationary-camera recovery */
    /* With visibility unavailable, retain the stale-node fallback. */
    for(i=0;i<3;i++) {
        captured_camera_version++;
        body_chain_collider_scene_liveness(&state,"Person02",&object,root,0,-1,400+i);
    }
    assert(state.scene_liveness_quarantined);
    puts("PASS: explicit live visibility prevents false camera despawn and recovers without camera movement; unknown-visibility fallback preserved");
    return 0;
}
'''
generated = build / 'collider_liveness_test.c'
generated.write_text(liveness)
run_test(generated, 'collider_liveness_test')
