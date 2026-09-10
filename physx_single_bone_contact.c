/* Breast/butt only. Engine samples and all collision geometry meet in world
   space; output translation constraints use the actual parent's scaled rows. */
static struct {
    DWORD tick,serial;
    int valid[4][4];
    float world[4][4][3];
} single_bone_published;

static int single_bone_pair_index(int node)
{
    if(node==BODY_COLLIDER_BREAST_L) return 0;
    if(node==BODY_COLLIDER_BREAST_R) return 1;
    if(node==BODY_COLLIDER_BUTT_L) return 2;
    if(node==BODY_COLLIDER_BUTT_R) return 3;
    return -1;
}

static int single_bone_parent_world(const body_chain_collider_person_state_t *owner,
    void *raw,float matrix[9])
{
    float view[9],inverse[9];
    if(!owner->contact_frame_valid || !body_chain_read_mat3_rows(raw,view)) return 0;
    body_chain_mat3_multiply(view,owner->contact_view_to_world,matrix);
    return body_chain_mat3_inverse(matrix,inverse);
}

static void single_bone_center(const float sample[3],const float reference[3],
    const float x[3],const float matrix[9],float out[3])
{
    float delta[3];int a;
    for(a=0;a<3;a++) delta[a]=x[a]-reference[a];
    body_chain_transform_row_vector3(delta,matrix,out);
    for(a=0;a<3;a++) out[a]+=sample[a];
}

static void single_bone_body_contacts(int person,int butt,int side,int scope,
    const float center[3],float radius,const float matrix[9],const float x[3],
    single_bone_contacts_t *contacts)
{
    int p,node,a;
    float strength=physx_clampf(body_chain_collider_cfg.response_strength,0,1);
    if(!body_chain_collider_cfg.enabled || strength<=0 ||
        !(butt?body_chain_collider_cfg.butt_collision_enabled:
                body_chain_collider_cfg.breasts_collision_enabled)) return;
    if(!body_chain_collision_scope_valid(scope)) scope=BODY_CHAIN_COLLISION_SCOPE_FULL_BODY_ALL;
    for(p=0;p<4;p++) {
        body_chain_collider_person_state_t *other=&body_chain_collider_states[p];
        if(!body_chain_collision_scope_all_persons(scope) && p!=person) continue;
        if(!other->ready || !other->basis_valid) continue;
        for(node=0;node<BODY_COLLIDER_NODE_COUNT;node++) {
            float normal[3],distance,depth,target_radius;
            float target_world[3];
            const float *target=target_world;
            int pair=single_bone_pair_index(node);
            if(!other->valid[node] || !paired_bone_physics_collider_node_in_scope(node,scope)) continue;
            if(!body_collision_view_to_world(other,other->view_position[node],target_world)) continue;
            if(p==person && (butt?butt_physics_owner_attachment_node(node):
                                  breasts_physics_owner_attachment_node(node))) continue;
            target_radius=body_chain_collider_visual_radius_for_node(node);
            if(target_radius<=.0001f) continue;
            /* Later single-bone updates see earlier solved pair positions,
               avoiding two full pushes against the same stale pair overlap.
               No global collider or chain geometry is modified. */
            if(pair>=0 && single_bone_published.valid[p][pair])
                target=single_bone_published.world[p][pair];
            for(a=0;a<3;a++) normal[a]=center[a]-target[a];
            distance=physx_vec3_len(normal);
            if(!isfinite(distance)) continue;
            depth=radius+target_radius-distance-body_chain_collider_cfg.collision_slop;
            if(depth<-.0002f) continue;
            if(distance>.00001f) for(a=0;a<3;a++) normal[a]/=distance;
            else {normal[0]=normal[1]=0;normal[2]=side?-1:1;}
            /* Subunit strengths allow gradual recovery; never over-separate
               just because an old profile used strength greater than one. */
            if(depth>0) depth*=strength;
            single_bone_world_plane(contacts,matrix,normal,depth,x);
        }
    }
}

static void single_bone_collect_room(const float center[3],float radius,
    const float matrix[9],const float x[3],single_bone_contacts_t *contacts)
{
    physx_contact_set_t room={0};int i;
    if(!single_bone_room_contacts(center,radius+.0002f,&room)) return;
    for(i=0;i<room.count;i++) single_bone_world_plane(contacts,matrix,
        room.normal[i],room.penetration[i]-.0002f,x);
}

static int single_bone_contact_step(int person,int butt,
    breasts_physics_person_state_t *state,const body_chain_physics_config_t *cfg,
    DWORD now,unsigned int elapsed_ms,const float target[2][3],
    float stiffness,float damping,const float limit[3],float correction[2][3])
{
    body_chain_collider_person_state_t *owner=&body_chain_collider_states[person];
    LONG generation=InterlockedCompareExchange(&named_node_generation,0,0);
    float dt=body_motion_duration(elapsed_ms);
    int steps=body_motion_substeps(dt),side,any=0;
    float largest_residual=0;int total_contacts=0;
    float observed[2][3]={{0}},solved[2][3]={{0}};
    int valid_mask=0;
    if(single_bone_published.tick!=now || single_bone_published.serial!=physx_simulation_serial) {
        memset(&single_bone_published,0,sizeof(single_bone_published));
        single_bone_published.tick=now;single_bone_published.serial=physx_simulation_serial;
    }
    memset(correction,0,sizeof(float)*6);dt/=steps;
    for(side=0;side<2;side++) {
        int node=butt?(side?BODY_COLLIDER_BUTT_R:BODY_COLLIDER_BUTT_L):
                      (side?BODY_COLLIDER_BREAST_R:BODY_COLLIDER_BREAST_L);
        float reference[3],matrix[9],sample[3],lo[3],hi[3];
        float *x=state->bone_translation[side],*v=state->bone_translation_velocity[side];
        float radius=body_chain_collider_visual_radius_for_node(node);
        int a,step,iteration,valid,room=cfg->room_collision_enabled && room_collision_is_enabled();
        int body=body_chain_collider_cfg.enabled &&
            (butt?body_chain_collider_cfg.butt_collision_enabled:body_chain_collider_cfg.breasts_collision_enabled);
        memcpy(reference,x,sizeof(reference));
        valid=owner->ready && owner->basis_valid && owner->valid[node] &&
            radius>.0001f && single_bone_parent_world(owner,state->translation_parent_joint_raw[side],matrix) &&
            body_collision_view_to_world(owner,owner->view_position[node],sample);
        if(valid) {
            memcpy(observed[side],sample,sizeof(sample));valid_mask|=1<<side;
        }
        for(a=0;a<3;a++) {lo[a]=-limit[a];hi[a]=limit[a];}
        if(elapsed_ms>BODY_MOTION_MAX_ELAPSED_MS) memset(v,0,sizeof(float)*3);
        for(step=0;step<steps;step++) {
            single_bone_contacts_t contacts={0};float predicted[3];
            for(a=0;a<3;a++) {
                /* Implicit damping cannot flip velocity with a large damping
                   setting. Contact recovery itself contributes no impulse. */
                v[a]=(v[a]+(target[side][a]-x[a])*stiffness*dt)/(1+damping*dt);
                x[a]+=v[a]*dt;
            }
            memcpy(predicted,x,sizeof(predicted));
            if(valid && room && !step && state->contact_previous_tick[side] &&
                state->contact_previous_generation[side]==generation &&
                state->contact_previous_parent[side]==state->translation_parent_joint_raw[side] &&
                now-state->contact_previous_tick[side]<=120u) {
                float center[3],move[3],push[3];
                single_bone_center(sample,reference,x,matrix,center);
                for(a=0;a<3;a++) move[a]=center[a]-state->contact_previous_world[side][a];
                if(physx_vec3_len(move)<=.750f && room_collision_resolve_swept_sphere(
                    state->contact_previous_world[side],center,radius,push,NULL)) {
                    float len=physx_vec3_len(push);
                    if(len>1e-6f) {
                        for(a=0;a<3;a++) push[a]/=len;
                        single_bone_world_plane(&contacts,matrix,push,len,x);
                    }
                }
            }
            for(iteration=0;iteration<3;iteration++) {
                float center[3],before[3],moved[3];
                if(valid) {
                    single_bone_center(sample,reference,x,matrix,center);
                    if(room) single_bone_collect_room(center,radius,matrix,x,&contacts);
                    if(body) single_bone_body_contacts(person,butt,side,cfg->collision_scope,
                        center,radius,matrix,x,&contacts);
                }
                memcpy(before,x,sizeof(before));
                single_bone_project(&contacts,lo,hi,x);
                for(a=0;a<3;a++) moved[a]=x[a]-before[a];
                if(single_bone_dot(moved,moved)<1e-14f) break;
            }
            single_bone_velocity(&contacts,x,limit,v);
            for(a=0;a<3;a++) correction[side][a]+=x[a]-predicted[a];
            if(contacts.count) any=1;
            total_contacts+=contacts.count;
            largest_residual=fmaxf(largest_residual,sqrtf(single_bone_violation(&contacts,x)));
        }
        /* This is the observed center, not sample + requested correction.
           The next live sample proves what the engine actually published. */
        state->contact_previous_tick[side]=valid && room?now:0;
        if(valid) {
            int pair=single_bone_pair_index(node);
            single_bone_center(sample,reference,x,matrix,single_bone_published.world[person][pair]);
            memcpy(solved[side],single_bone_published.world[person][pair],sizeof(float)*3);
            single_bone_published.valid[person][pair]=1;
        }
        if(valid && room) {
            memcpy(state->contact_previous_world[side],sample,sizeof(sample));
            state->contact_previous_generation[side]=generation;
            state->contact_previous_parent[side]=state->translation_parent_joint_raw[side];
        }
    }
    if((defaults_cfg.debug || any) &&
        (!state->contact_log_tick || now-state->contact_log_tick>=1000u)) {
        state->contact_log_tick=now;
        log_line("single-bone contact system=%s person=%d contacts=%d residual_local=%.6f valid_mask=%d elapsed_ms=%u translation=(%.5f,%.5f,%.5f;%.5f,%.5f,%.5f) correction=(%.5f,%.5f,%.5f;%.5f,%.5f,%.5f) observed_world=(%.5f,%.5f,%.5f;%.5f,%.5f,%.5f) solved_world=(%.5f,%.5f,%.5f;%.5f,%.5f,%.5f)",
            butt?"butt":"breasts",person+1,total_contacts,largest_residual,valid_mask,elapsed_ms,
            state->bone_translation[0][0],state->bone_translation[0][1],state->bone_translation[0][2],
            state->bone_translation[1][0],state->bone_translation[1][1],state->bone_translation[1][2],
            correction[0][0],correction[0][1],correction[0][2],correction[1][0],correction[1][1],correction[1][2],
            observed[0][0],observed[0][1],observed[0][2],observed[1][0],observed[1][1],observed[1][2],
            solved[0][0],solved[0][1],solved[0][2],solved[1][0],solved[1][1],solved[1][2]);
    }
    return any;
}
