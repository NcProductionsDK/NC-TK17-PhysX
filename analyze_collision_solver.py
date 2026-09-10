"""Offline investigation of contact depth measured before solver integration.

Runs the regular regression suite (including the new coordinated body path),
then characterizes the retained legacy per-joint fallback in eight output
fixtures. The old defects are still expected in that fallback; the main suite
verifies the replacement. Does not build or install the plugin.
"""
from pathlib import Path
import json
import runpy
import subprocess

root = Path(__file__).resolve().parent
suite = runpy.run_path(str(root / 'run_collision_tests.py'))
marker = '''            saved=joints[1];
            assert(addon_chain_solve_body_contact'''
experiment = r'''
            saved=joints[1];
            {
                int direction;
                for(direction=-1;direction<=1;direction+=2) {
                    float advanced[3],current[3],final[3],normal[3],motion[3];
                    float d[3],a[3],g[3],drift,remaining,old_separation,new_separation;
                    int solved;
                    /* Simulate an integration step away from the published
                       pose. Collision geometry/basis remain at that pose. */
                    joints[1]=saved;
                    memcpy(advanced,saved.sim_offset,sizeof(advanced));
                    advanced[0]+=direction*0.0006f;
                    length=physx_vec3_len(advanced);
                    for(i=0;i<3;i++) {
                        advanced[i]*=0.1f/length;
                        normal[i]=request[i]/depth;
                    }
                    memcpy(joints[1].sim_offset,advanced,sizeof(advanced));
                    addon_chain_contact_predict(&chain,&joints[1],advanced,local,current);
                    for(i=0;i<3;i++) motion[i]=current[i]-baseline[i];
                    drift=vec3_dot(motion,normal);
                    solved=addon_chain_solve_body_contact(&chain,&joints[1],pivot,
                        baseline,request,d,a,g);
                    assert(solved);
                    for(i=0;i<3;i++) advanced[i]=joints[1].sim_offset[i]+d[i];
                    addon_chain_contact_predict(&chain,&joints[1],advanced,local,final);
                    for(i=0;i<3;i++) motion[i]=final[i]-baseline[i];
                    old_separation=vec3_dot(motion,normal);
                    /* Experiment: keep the plane goal in the sampled pose,
                       subtract current displacement before asking for more. */
                    remaining=depth-drift;
                    new_separation=drift;
                    if(remaining>0.000001f) {
                        float corrected_request[3];
                        for(i=0;i<3;i++) corrected_request[i]=normal[i]*remaining;
                        assert(addon_chain_solve_body_contact(&chain,&joints[1],pivot,
                            baseline,corrected_request,d,a,g));
                        for(i=0;i<3;i++) advanced[i]=joints[1].sim_offset[i]+d[i];
                        addon_chain_contact_predict(&chain,&joints[1],advanced,local,final);
                        for(i=0;i<3;i++) motion[i]=final[i]-baseline[i];
                        new_separation=vec3_dot(motion,normal);
                    }
                    assert(new_separation>=depth*0.95f);
                    if(direction<0) assert(old_separation<0);
                    else assert(old_separation>new_separation+depth*0.9f);
                    printf("PHASE_CASE {\"pose\":%d,\"direction\":%d,\"requested\":%.9f,"
                        "\"integration_drift\":%.9f,\"current_result\":%.9f,"
                        "\"absolute_goal_result\":%.9f}\n",
                        pose,direction,depth,drift,old_separation,new_separation);
                }
                joints[1]=saved;
                {
                    addon_chain_body_manifold_t manifold={0};
                    float near_point[3],normal[3],far_request[3],d[3],a[3],g[3],near_motion[3];
                    float near_required=depth*0.8f,near_achieved;
                    for(i=0;i<3;i++) {
                        near_point[i]=pivot[i]+0.25f*(baseline[i]-pivot[i]);
                        normal[i]=request[i]/depth;
                        far_request[i]=request[i];
                    }
                    addon_chain_body_manifold_store(&manifold,normal,near_required,near_point);
                    addon_chain_body_manifold_store(&manifold,normal,depth,baseline);
                    assert(manifold.contacts.count==1);
                    assert(addon_chain_solve_body_contact(&chain,&joints[1],pivot,
                        baseline,far_request,d,a,g));
                    addon_chain_contact_point_delta(&chain,&joints[1],pivot,near_point,d,near_motion);
                    near_achieved=vec3_dot(near_motion,normal);
                    assert(near_achieved<near_required*0.4f);
                    printf("LEVER_CASE {\"pose\":%d,\"retained_supports\":%d,"
                        "\"near_required\":%.9f,\"near_achieved\":%.9f}\n",
                        pose,manifold.contacts.count,near_required,near_achieved);
                }
            }
            assert(addon_chain_solve_body_contact'''
fixture = suite['integration']
assert fixture.count(marker) == 1, 'Output fixture changed; review experiment insertion'
fixture = suite['use_production_lengths'](fixture.replace(marker, experiment))
source = suite['build'] / 'collision_phase_analysis.c'
source.write_text(fixture)
exe = source.with_suffix('.exe')
subprocess.run([str(suite['gcc']), '-m32', '-O2', '-Wall', '-Wextra', '-Werror',
                '-Wno-unused-function', '-static-libgcc', '-I', str(root),
                '-o', str(exe), str(source)], env=suite['env'], check=True)
result = subprocess.run([str(exe)], capture_output=True, text=True, check=True)
cases = [json.loads(line.removeprefix('PHASE_CASE '))
         for line in result.stdout.splitlines() if line.startswith('PHASE_CASE ')]
assert len(cases) == 16
lever_cases = [json.loads(line.removeprefix('LEVER_CASE '))
               for line in result.stdout.splitlines() if line.startswith('LEVER_CASE ')]
assert len(lever_cases) == 8
report = suite['build'] / 'collision_phase_analysis.json'
report.write_text(json.dumps({'phase_cases': cases, 'lever_cases': lever_cases}, indent=2) + '\n')
print('LEGACY FALLBACK: reproduced stale-depth response in all 8 output fixtures, inward and outward drift')
print('PASS: isolated absolute-plane request experiment resolves all inward cases and skips already satisfied contacts')
print(json.dumps(cases[:2], indent=2))
print('LEGACY MANIFOLD: reproduced loss of the stricter angular support through normal-only deduplication in all 8 fixtures')
print(json.dumps(lever_cases[:1], indent=2))
print(f'Full experiment results: {report}')
