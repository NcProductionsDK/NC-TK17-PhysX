"""Replay recorded engine poses through the production composed-joint model.

Without arguments, use the small sanitized regression corpus. Pass a pose-audit
game log to replay every complete chain sample. Requires the project's MinGW.
"""
import argparse
import json
import os
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parent


def read_log(path):
    frames = {}
    roots = {}
    for line in path.read_text(errors="replace").splitlines():
        if 'testicle-engine-bones person=' in line and 'joint01_local=' in line:
            person = int(re.search(r'person="Person(\d+)"', line)[1])
            roots[person] = list(map(float, re.search(r'joint01_local=\(([^)]*)', line)[1].split(',')))
        if 'body-contact pose-joint' not in line:
            continue
        match = re.search(r'target="(.*?)" person=(\d+) tick=(\d+) joint=(\d+)', line)
        target, person, tick, joint = match.groups()
        key = (target, person, tick)
        if target == 'testicle' and int(person) not in roots:
            continue
        frame = frames.setdefault(key, {'target': target, 'origin': [0, 0, 0] if target == 'penis' else roots[int(person)], 'joints': {}})
        values = {name: list(map(float, re.search(name+r'=\(([^)]*)', line)[1].split(',')))
                  for name in ('before', 'observed', 'predicted', 'angle_before', 'angle_after', 'output')}
        frame['joints'][int(joint)] = values
    result = []
    for frame in frames.values():
        count = 3 if frame['target'] == 'penis' else 2
        if len(frame['joints']) != count:
            continue
        joints = [frame['joints'][j+1] for j in range(count)]
        result.append(dict(target=frame['target'],
            before=[frame['origin']]+[j['before'] for j in joints],
            observed=[frame['origin']]+[j['observed'] for j in joints],
            old_prediction=[frame['origin']]+[j['predicted'] for j in joints],
            euler_before=[[j['output'][a]-j['angle_after'][a]+j['angle_before'][a] for a in range(3)] for j in joints],
            euler_after=[j['output'] for j in joints]))
    return result


def initializer(value):
    if isinstance(value, list):
        return '{'+','.join(initializer(v) for v in value)+'}'
    return f'{float(value):.9f}f'


def run(records):
    if not records:
        raise ValueError('No complete pose recordings found')
    source = r'''
#include <stdio.h>
#include <stdlib.h>
#include "../physx_body_pose.h"
#define CHECK(x) do { if(!(x)) { fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#x); exit(1); } } while(0)
static float maximum_error=0,maximum_old_error=0;
static void replay(int segments,const float before[4][3],const float observed[4][3],
    const float old_prediction[4][3],const float euler_before[3][3],const float euler_after[3][3])
{
    body_contact_pose_t pose={0},saved;float predicted[4][3];int j,a;
    CHECK(body_pose_fit(before,euler_before,segments,&pose));saved=pose;
    CHECK(body_pose_evaluate(&pose,euler_after,predicted));
    CHECK(!memcmp(&saved,&pose,sizeof(pose)));
    for(j=1;j<=segments;j++) {
        float e=0,old=0;for(a=0;a<3;a++) {
            float d=predicted[j][a]-observed[j][a];e+=d*d;
            d=old_prediction[j][a]-observed[j][a];old+=d*d;
        }
        maximum_error=fmaxf(maximum_error,sqrtf(e));maximum_old_error=fmaxf(maximum_old_error,sqrtf(old));
        CHECK(sqrtf(e)<.00001f);
    }
}
static void invalid_samples(void){
    body_contact_pose_t pose={0},saved={0};
    float points[4][3]={{0,0,0},{0,0,-.1f},{0,0,-.2f},{0,0,-.3f}},euler[3][3]={{0}};
    CHECK(body_pose_fit(points,euler,3,&pose));saved=pose;
    points[1][1]=.05f;CHECK(!body_pose_fit(points,euler,3,&pose));CHECK(!memcmp(&pose,&saved,sizeof(pose)));
    points[1][1]=NAN;CHECK(!body_pose_fit(points,euler,3,&pose));
    points[1][1]=0;euler[0][1]=90;CHECK(!body_pose_fit(points,euler,3,&pose));
    euler[0][1]=0;points[1][2]=0;CHECK(!body_pose_fit(points,euler,3,&pose));
    CHECK(!body_pose_fit(points,euler,4,&pose));
}
int main(void){invalid_samples();
'''
    for record in records:
        segments = len(record['euler_before'])
        source += '{\n'
        for key in ('before', 'observed', 'old_prediction', 'euler_before', 'euler_after'):
            rows = 3 if key.startswith('euler') else 4
            source += f'const float {key}[{rows}][3]={initializer(record[key])};\n'
        source += f'replay({segments},before,observed,old_prediction,euler_before,euler_after);\n}}\n'
    source += f'printf("PASS: {len(records)} recorded engine poses: max composed error=%g; max old error=%g\\n",maximum_error,maximum_old_error);return 0;}}'
    build = ROOT/'build'
    build.mkdir(exist_ok=True)
    path = build/'body_pose_test.c'
    path.write_text(source)
    compiler = Path(r'C:\msys64\mingw32\bin\gcc.exe')
    env = dict(os.environ, PATH=str(compiler.parent)+os.pathsep+os.environ['PATH'])
    exe = build/'body_pose_test.exe'
    subprocess.run([str(compiler), '-m32', '-O2', '-Wall', '-Wextra', '-Werror', '-Wno-unused-function', '-static-libgcc', '-o', str(exe), str(path)], env=env, check=True)
    subprocess.run([str(exe)], env=env, check=True, timeout=30)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log', nargs='?', type=Path)
    args = parser.parse_args()
    run(read_log(args.log) if args.log else json.loads((ROOT/'tests/body_pose_recordings.json').read_text()))
