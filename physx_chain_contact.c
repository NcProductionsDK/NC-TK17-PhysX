/* Included by physx_sidecar.c; pure candidate-pose operations never publish
   engine transforms. Positions are in the frozen wearer's body frame. */
typedef struct {
    int last;
    float live_start[3],live_end[3];
    float offset[32][3];
} physx_chain_contact_pose_t;

static int physx_chain_contact_pose_init(physx_chain_t *chain,int last,DWORD now,
    physx_chain_contact_pose_t *pose)
{
    int i;
    int terminal=last==chain->target_count-1;
    if(last<1 || last>=chain->target_count || last>=32) return 0;
    if(terminal) {
        if(chain->targets[last].contact_terminal_tick!=now) return 0;
    } else if(chain->targets[last+1].contact_pivot_tick!=now) return 0;
    for(i=1;i<=last;i++) {
        physx_target_t *joint=&chain->targets[i];
        if(!joint->addon_simulated_target || joint->contact_basis_tick!=now ||
            joint->contact_pivot_tick!=now || joint->sim_length<=0.0001f) return 0;
        memcpy(pose->offset[i],joint->sim_offset,sizeof(float)*3);
    }
    pose->last=last;
    memcpy(pose->live_start,chain->targets[last].contact_pivot_body,sizeof(float)*3);
    memcpy(pose->live_end,terminal ? chain->targets[last].contact_terminal_body :
        chain->targets[last+1].contact_pivot_body,sizeof(float)*3);
    return 1;
}

static void physx_chain_contact_point(physx_chain_t *chain,
    const physx_chain_contact_pose_t *pose,float t,float point[3])
{
    const float *a=pose->live_start;
    const float *b=pose->live_end;
    int i,j,k;
    for(j=0;j<3;j++) point[j]=a[j]+t*(b[j]-a[j]);
    /* Distal-to-root composition: each joint delta is expressed in its
       original world basis; earlier ancestors then transform the whole result.
       This is exact hierarchical composition for a linear rigid joint chain. */
    for(i=pose->last;i>=1;i--) {
        physx_target_t *joint=&chain->targets[i];
        float local[3],relative[3];
        for(j=0;j<3;j++) {
            local[j]=0;
            for(k=0;k<3;k++) local[j]+=(point[k]-joint->contact_pivot_body[k])*
                joint->contact_live_body_basis[j*3+k];
        }
        addon_chain_contact_predict(chain,joint,pose->offset[i],local,relative);
        for(j=0;j<3;j++) point[j]=joint->contact_pivot_body[j]+relative[j];
    }
}

static int physx_chain_contact_store(physx_contact_set_t *set,float points[][3],
    const float normal[3],float depth,const float point[3])
{
    int i,weakest=0,slot;
    if(depth<=0.000001f) return -1;
    for(i=0;i<set->count;i++) {
        float d[3];
        int j;
        for(j=0;j<3;j++) d[j]=points[i][j]-point[j];
        /* Equal normals at different lever arms are distinct constraints. */
        if(vec3_dot(set->normal[i],normal)>0.9999f && vec3_dot(d,d)<1e-10f) {
            if(depth<=set->penetration[i]) return -1;
            memcpy(set->normal[i],normal,sizeof(float)*3);
            set->penetration[i]=depth;
            return i;
        }
        if(set->penetration[i]<set->penetration[weakest]) weakest=i;
    }
    if(set->count<PHYSX_CONTACT_CAPACITY) slot=set->count++;
    else if(depth>set->penetration[weakest]) slot=weakest;
    else return -1;
    memcpy(set->normal[slot],normal,sizeof(float)*3);
    set->penetration[slot]=depth;
    return slot;
}

static float physx_chain_contact_gradient(physx_chain_t *chain,
    physx_chain_contact_pose_t *pose,float t,const float normal[3],float gradient[32][3])
{
    float norm2=0;
    int i,j,k;
    for(i=1;i<=pose->last;i++) {
        float saved[3],epsilon=chain->targets[i].sim_length*0.001f,radial;
        memcpy(saved,pose->offset[i],sizeof(saved));
        for(j=0;j<3;j++) {
            float plus[3],minus[3],length;
            memcpy(pose->offset[i],saved,sizeof(saved));
            pose->offset[i][j]+=epsilon;
            length=physx_vec3_len(pose->offset[i]);
            for(k=0;k<3;k++) pose->offset[i][k]*=chain->targets[i].sim_length/length;
            physx_chain_contact_point(chain,pose,t,plus);
            memcpy(pose->offset[i],saved,sizeof(saved));
            pose->offset[i][j]-=epsilon;
            length=physx_vec3_len(pose->offset[i]);
            for(k=0;k<3;k++) pose->offset[i][k]*=chain->targets[i].sim_length/length;
            physx_chain_contact_point(chain,pose,t,minus);
            gradient[i][j]=((plus[0]-minus[0])*normal[0]+(plus[1]-minus[1])*normal[1]+
                (plus[2]-minus[2])*normal[2])/(2*epsilon);
        }
        memcpy(pose->offset[i],saved,sizeof(saved));
        radial=vec3_dot(gradient[i],saved)/vec3_dot(saved,saved);
        for(j=0;j<3;j++) {
            gradient[i][j]-=radial*saved[j];
            norm2+=gradient[i][j]*gradient[i][j];
        }
    }
    return norm2;
}

static int physx_chain_contact_solve(physx_chain_t *chain,physx_chain_contact_pose_t *pose,
    const physx_contact_set_t *contacts,const float points[][3],
    const float start[3],const float end[3])
{
    float t[PHYSX_CONTACT_CAPACITY],goal[PHYSX_CONTACT_CAPACITY],segment[3],len2;
    float gradient[32][3],spent[32]={0};
    int i,j,c,iteration,moved=0;
    for(j=0;j<3;j++) segment[j]=end[j]-start[j];
    len2=vec3_dot(segment,segment);
    if(len2<1e-8f) return 0;
    for(c=0;c<contacts->count;c++) {
        float relative[3];
        for(j=0;j<3;j++) relative[j]=points[c][j]-start[j];
        t[c]=physx_clampf(vec3_dot(relative,segment)/len2,0,1);
        goal[c]=vec3_dot(points[c],contacts->normal[c])+0.55f*contacts->penetration[c];
    }
    for(iteration=0;iteration<6;iteration++) {
        int changed=0;
        for(c=0;c<contacts->count;c++) {
            float point[3],residual,norm2,scale=1.0f;
            physx_chain_contact_pose_t trial;
            int attempt;
            physx_chain_contact_point(chain,pose,t[c],point);
            residual=goal[c]-vec3_dot(point,contacts->normal[c]);
            if(residual<=1e-6f) continue;
            norm2=physx_chain_contact_gradient(chain,pose,t[c],contacts->normal[c],gradient);
            if(norm2<1e-6f) continue;
            for(i=1;i<=pose->last;i++) {
                float step=physx_vec3_len(gradient[i])*residual/(norm2+0.0001f);
                float remaining=fmaxf(0,chain->targets[i].sim_length*0.16f-spent[i]);
                float cap=fminf(chain->targets[i].sim_length*0.08f,remaining);
                if(step>cap) scale=fminf(scale,cap/step);
            }
            for(attempt=0;attempt<5 && scale>1e-5f;attempt++,scale*=0.5f) {
                float next;
                trial=*pose;
                for(i=1;i<=pose->last;i++) {
                    float length;
                    for(j=0;j<3;j++) trial.offset[i][j]+=gradient[i][j]*residual*scale/(norm2+0.0001f);
                    length=physx_vec3_len(trial.offset[i]);
                    for(j=0;j<3;j++) trial.offset[i][j]*=chain->targets[i].sim_length/length;
                }
                physx_chain_contact_point(chain,&trial,t[c],point);
                next=goal[c]-vec3_dot(point,contacts->normal[c]);
                if(fabsf(next)<residual && next>=-contacts->penetration[c]*0.05f) {
                    for(i=1;i<=pose->last;i++) {
                        float delta[3];
                        for(j=0;j<3;j++) delta[j]=trial.offset[i][j]-pose->offset[i][j];
                        spent[i]+=physx_vec3_len(delta);
                    }
                    *pose=trial; changed=1; break;
                }
            }
        }
        if(!changed) break;
    }
    for(i=1;i<=pose->last;i++) {
        physx_target_t *joint=&chain->targets[i];
        if(spent[i]>1e-7f) {
            memcpy(joint->sim_offset,pose->offset[i],sizeof(float)*3);
            joint->sim_contact_corrected=1;
            moved++;
        }
    }
    /* Shared normal velocity: a contact sees the SUM of ancestor effects.
       Reproject after friction so later contacts cannot leave an earlier
       normal velocity unchecked. Static supports; moving-body velocity is
       still represented by refreshed geometry, not an explicit surface speed. */
    for(iteration=0;iteration<4;iteration++) for(c=0;c<contacts->count;c++) {
        float norm2=physx_chain_contact_gradient(chain,pose,t[c],contacts->normal[c],gradient);
        float inward=0,point[3];
        physx_chain_contact_point(chain,pose,t[c],point);
        if(vec3_dot(point,contacts->normal[c])-goal[c]>0.002f || norm2<1e-6f) continue;
        for(i=1;i<=pose->last;i++) inward+=vec3_dot(gradient[i],chain->targets[i].sim_velocity);
        if(inward>=0) continue;
        for(i=1;i<=pose->last;i++) {
            physx_target_t *joint=&chain->targets[i];
            float g2=vec3_dot(gradient[i],gradient[i]),radial;
            for(j=0;j<3;j++) joint->sim_velocity[j]-=gradient[i][j]*inward/norm2;
            if(iteration==0 && g2>1e-8f) {
                float tangent[3],along=vec3_dot(joint->sim_velocity,gradient[i])/g2,speed,ratio;
                for(j=0;j<3;j++) tangent[j]=joint->sim_velocity[j]-along*gradient[i][j];
                speed=physx_vec3_len(tangent);
                ratio=speed>1e-8f ? fminf(1,0.25f*(-inward)*sqrtf(g2)/(norm2*speed)) : 0;
                for(j=0;j<3;j++) joint->sim_velocity[j]-=ratio*tangent[j];
            }
            radial=vec3_dot(joint->sim_velocity,joint->sim_offset)/vec3_dot(joint->sim_offset,joint->sim_offset);
            for(j=0;j<3;j++) joint->sim_velocity[j]-=radial*joint->sim_offset[j];
            joint->sim_contact_corrected=1;
        }
    }
    return moved;
}
