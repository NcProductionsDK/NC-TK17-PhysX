/* Included by physx_colliders.c. Joint-space contact solve shared by the
   built-in penis and testicle chains. Corrections change pose, never momentum. */
static int body_contact_predict(const body_chain_physics_config_t *cfg,
    const body_chain_person_state_t *state,const float base[4][3],
    const float correction[3][2],float points[4][3])
{
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

/* Read-only comparison of the last step's predicted geometry with the next
   engine sample. Tests of the solver alone cannot establish this agreement. */
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

static void body_contact_trace_pose(const body_chain_collider_person_state_t *collider,
    body_chain_person_state_t *state,const body_chain_physics_config_t *cfg,
    int testicle,DWORD now,const float observed[4][3],int engine)
{
    float predicted[4][3],error[3];int j;
    int person=(int)(collider-body_chain_collider_states)+1;
    const char *target=testicle?"testicle":"penis";
    if(!(defaults_cfg.debug || body_chain_collider_cfg.diagnostic) ||
       !engine || !state->collision_step_engine_points || !state->collision_step_valid ||
       state->collision_step_tick==now || now-state->collision_step_tick>100u ||
       state->collision_pose_trace_count>=256u ||
       state->collision_prev_max_penetration<.01f) return;
    if(!body_contact_candidate_points(state,cfg,state->collision_step_tick,predicted)) return;
    state->collision_pose_trace_count++;
    body_contact_pose_error(state->collision_step_points,predicted,observed,testicle?2:3,error);
    log_line("body-contact pose-audit target=\"%s\" person=%d tick=%lu previous_tick=%lu sample=%u engine=%d camera_hold=%d penetration=%.6f error=%.7f predicted_move=%.7f observed_move=%.7f h_axis=%d v_axis=%d composed=%d",
        target,person,(unsigned long)now,(unsigned long)state->collision_step_tick,
        state->collision_pose_trace_count,engine,body_chain_camera_pivot_hold_active(now),
        state->collision_prev_max_penetration,error[0],error[1],error[2],
        cfg->horizontal_output_axis,cfg->vertical_output_axis,state->collision_pose_valid);
    for(j=0;j<(testicle?2:3);j++) {
        const float *before=state->collision_step_points[j+1];
        const float *p=predicted[j+1],*o=observed[j+1];
        float output[3]={0};int output_valid=0;
        if(state->joint_raw[j]) {
            if(cfg->output_offset>=0 && ptr_readable((BYTE*)state->joint_raw[j]+cfg->output_offset,sizeof(output))) {
                memcpy(output,(BYTE*)state->joint_raw[j]+cfg->output_offset,sizeof(output));
                output_valid=sane_probe_float(output[0]) && sane_probe_float(output[1]) && sane_probe_float(output[2]);
            }
        }
        log_line("body-contact pose-joint target=\"%s\" person=%d tick=%lu joint=%d before=(%.7f,%.7f,%.7f) predicted=(%.7f,%.7f,%.7f) observed=(%.7f,%.7f,%.7f) angle_before=(%.5f,%.5f,%.5f) angle_after=(%.5f,%.5f,%.5f) output_valid=%d output=(%.5f,%.5f,%.5f)",
            target,person,(unsigned long)now,j+1,before[0],before[1],before[2],p[0],p[1],p[2],o[0],o[1],o[2],
            state->collision_step_angle[j][0],state->collision_step_angle[j][1],state->collision_step_angle[j][2],
            state->angle[j][0],state->angle[j][1],state->angle[j][2],output_valid,output[0],output[1],output[2]);
    }
}

/* Capture once before spring integration. Engine pivot queries during contact
   iterations cannot reflect unpublished candidate rotations. Held points keep
   their existing safety window, but all iterations use the same reference. */
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

static float body_contact_inverse_inertia(const body_chain_person_state_t *state,int joint,int axis)
{
    if(state->dynamics_valid && joint<state->dynamics.segments)
        return state->dynamics.inverse_inertia[joint][axis];
    return 1.0f;
}

static void body_contact_point(const float points[4][3], const body_chain_contact_t *contact,
                                float out[3])
{
    int a;
    for(a=0;a<3;a++) out[a]=points[contact->segment][a]+contact->segment_t*
        (points[contact->segment+1][a]-points[contact->segment][a]);
}

/* Finite differences in the same mapping used for candidate geometry. At a
   joint limit, discard only degrees of freedom that would push farther out. */
static void body_contact_jacobian(const body_chain_physics_config_t *cfg,
    const body_chain_person_state_t *state, const float base[4][3],
    const float correction[3][2], const body_chain_contact_t *contact,
    const float direction[3], float jac[3][2], int respect_limits)
{
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

/* The normal planes are a local approximation to curved colliders. Keep the
   complete chain update within a small angular neighborhood, even when a
   legacy INI permits 45 degrees per joint. This is a numerical trust region,
   independent of contact age, sleep, and velocity. */
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

/* A clamped angle cannot carry velocity farther into its limit. Remove that
   velocity before another contact can redistribute it into a child joint. */
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

/* Preserve an entry side only when history establishes an exterior point.
   An already embedded sample must not reverse the nearest outward normal. */
static int body_contact_orient_from_history(float normal[3],const float previous[3],
    float previous_length,float radius,float slop)
{
    int a;
    if(previous_length<=.0001f || previous_length<fmaxf(.0001f,radius-slop) ||
       vec3_dot(normal,previous)>=0.0f) return 0;
    for(a=0;a<3;a++) normal[a]=-normal[a];
    return 1;
}

/* Individual plane projections can all be rejected by the combined error
   test even though a smaller joint move improves the manifold. Refine the
   accepted pose along the combined gradient, with actual nonlinear error
   checks, joint limits and the existing whole-chain displacement budget. */
static void body_contact_refine_combined(const body_chain_physics_config_t *cfg,
    const body_chain_person_state_t *state,const float base[4][3],
    const body_chain_contact_t *contacts,int count,int segments,
    float cap,float radius,float relaxation,float correction[3][2])
{
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
    float points[4][3];
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
    for(pass=0;pass<iterations;pass++) {
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
    memcpy(correction,best_correction,sizeof(best_correction));
    body_contact_refine_combined(cfg,state,base,contacts,count,segment_count,
        cap,trust_radius,relaxation,correction);
    body_contact_limit_velocity(cfg,state,correction,segment_count);
    /* Project contact velocity across all influencing joints. Positional
       correction/dt is deliberately absent: separation must not cause bounce.
       Free separation is untouched. Repeated passes restore all
       corner-support inequalities. */
    for(pass=0;pass<8;pass++) for(c=0;c<count;c++) {
        float jac[3][2],denom=0,vn=0,lambda;
        body_contact_jacobian(cfg,state,base,correction,&contacts[c],contacts[c].normal,jac,0);
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
