/* Frozen before per-solve velocity Jacobian reuse (2026-09-13). */
static void reference_body_contact_solve(const body_chain_physics_config_t *cfg,
    body_chain_person_state_t *state, const float base[4][3],
    body_chain_contact_t *contacts,int count,int segment_count,float correction[3][2])
{
    COLLISION_PROFILE_SCOPE(profile_solve, CP_BODY_SOLVE);
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
