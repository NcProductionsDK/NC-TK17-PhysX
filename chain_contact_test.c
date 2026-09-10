/* Included in the extracted production sidecar fixture. */
static void test_chain_matmul(const float a[9],const float b[9],float out[9])
{
    int i,j,k;
    for(i=0;i<3;i++) for(j=0;j<3;j++) {
        out[i*3+j]=0;
        for(k=0;k<3;k++) out[i*3+j]+=a[i*3+k]*b[k*3+j];
    }
}

/* Independent root-to-tip FK oracle, using explicit link translations. */
static void test_chain_fk(physx_chain_t *chain,const float offsets[32][3],
    const float anchor[9],float pivots[4][3],float bases[4][9])
{
    int i,j;
    memcpy(bases[0],anchor,sizeof(float)*9);
    memcpy(pivots[1],chain->targets[1].contact_pivot_body,sizeof(float)*3);
    for(i=1;i<=2;i++) {
        float angles[3],rows[9];
        addon_chain_contact_output_angles_at(chain,&chain->targets[i],angles,NULL,NULL,offsets[i]);
        for(j=0;j<3;j++) angles[j]=chain->targets[i].sim_rotation_rest[j]+
            (angles[j]-chain->targets[i].sim_rotation_rest[j])*chain->skinned_matrix_scale;
        physx_contact_rotation_rows(angles,rows);
        test_chain_matmul(rows,bases[i-1],bases[i]);
        for(j=0;j<3;j++) pivots[i+1][j]=pivots[i][j]+0.1f*bases[i][3+j];
    }
}

static void test_chain_snapshot(physx_chain_t *chain,const float anchor[9])
{
    float offsets[32][3]={{0}},pivots[4][3],bases[4][9];
    int i;
    for(i=1;i<=2;i++) memcpy(offsets[i],chain->targets[i].sim_offset,sizeof(float)*3);
    test_chain_fk(chain,offsets,anchor,pivots,bases);
    for(i=1;i<=3;i++) {
        chain->targets[i].contact_pivot_tick=100;
        memcpy(chain->targets[i].contact_pivot_body,pivots[i],sizeof(float)*3);
        if(i<=2) {
            chain->targets[i].contact_basis_tick=100;
            memcpy(chain->targets[i].contact_live_body_basis,bases[i],sizeof(float)*9);
            memcpy(chain->targets[i].contact_parent_body_basis,bases[i-1],sizeof(float)*9);
        }
    }
}

static void test_coherent_chain_contacts(void)
{
    int configuration;
    for(configuration=0;configuration<8;configuration++) {
        physx_target_t joints[4]={0};
        physx_chain_t chain={0};
        physx_chain_contact_pose_t pose;
        float anchor[9],angles[3]={23.0f*configuration,-11.0f*configuration,17.0f*configuration};
        float start[3],end[3],point[3],pivots[4][3],bases[4][9],normal[3],gradient[32][3];
        addon_chain_body_manifold_t manifold={0};
        int i,j;
        chain.targets=joints; chain.target_count=4; chain.addon_chain=1;
        chain.h_axis=0; chain.v_axis=2; chain.root_bend_scale=1.3f;
        chain.skinned_matrix_scale=0.8f; chain.limit_angle=180;
        chain.rotation_solver_full_angle=configuration&1;
        strcpy(chain.name,"tail"); strcpy(chain.parent_name,"anchor");
        strcpy(joints[0].name,"anchor"); strcpy(joints[1].name,"tail");
        physx_contact_rotation_rows(angles,anchor);
        for(i=1;i<=2;i++) {
            joints[i].sim_length=0.1f; joints[i].addon_simulated_target=1;
            joints[i].sim_rest[1]=joints[i].sim_offset[1]=0.1f;
        }
        joints[1].sim_rotation_rest[0]=(configuration&2)?170:0;
        joints[2].sim_rotation_rest[2]=(configuration&4)?35:0;
        test_chain_snapshot(&chain,anchor);
        assert(physx_chain_contact_pose_init(&chain,2,100,&pose));
        /* Candidate ancestor and child both move, sampled bases stay fixed. */
        pose.offset[1][0]=0.003f;
        pose.offset[2][2]=-0.004f;
        for(i=1;i<=2;i++) {
            float length=physx_vec3_len(pose.offset[i]);
            for(j=0;j<3;j++) pose.offset[i][j]*=0.1f/length;
        }
        test_chain_fk(&chain,pose.offset,anchor,pivots,bases);
        physx_chain_contact_point(&chain,&pose,0,start);
        physx_chain_contact_point(&chain,&pose,1,end);
        physx_chain_contact_point(&chain,&pose,0.37f,point);
        for(j=0;j<3;j++) {
            assert(fabsf(start[j]-pivots[2][j])<2e-7f);
            assert(fabsf(end[j]-pivots[3][j])<2e-7f);
            assert(fabsf(point[j]-(0.63f*pivots[2][j]+0.37f*pivots[3][j]))<2e-7f);
            normal[j]=bases[1][j];
        }
        for(i=1;i<=2;i++) memcpy(joints[i].sim_offset,pose.offset[i],sizeof(float)*3);
        /* These contacts are gathered AFTER integration, at candidate geometry.
           Same-normal contacts at different lever arms must both survive. */
        manifold.angular=1;
        addon_chain_body_manifold_store(&manifold,normal,0.00008f,start);
        addon_chain_body_manifold_store(&manifold,normal,0.00010f,end);
        assert(manifold.contacts.count==2);
        addon_chain_body_manifold_store(&manifold,normal,0.00007f,start);
        assert(manifold.contacts.count==2);
        physx_chain_contact_gradient(&chain,&pose,0,normal,gradient);
        for(i=1;i<=2;i++) for(j=0;j<3;j++) joints[i].sim_velocity[j]=-0.01f*gradient[i][j];
        assert(physx_chain_contact_solve(&chain,&pose,&manifold.contacts,manifold.point,start,end)>0);
        for(i=0;i<2;i++) {
            float movement[3],inward=0;
            physx_chain_contact_point(&chain,&pose,(float)i,point);
            for(j=0;j<3;j++) movement[j]=point[j]-manifold.point[i][j];
            assert(vec3_dot(movement,normal)>=0.55f*manifold.contacts.penetration[i]-2e-6f);
            physx_chain_contact_gradient(&chain,&pose,(float)i,normal,gradient);
            for(j=1;j<=2;j++) inward+=vec3_dot(gradient[j],joints[j].sim_velocity);
            assert(inward>-1e-5f);
        }
        for(i=1;i<=2;i++) assert(fabsf(physx_vec3_len(joints[i].sim_offset)-0.1f)<1e-7f);
        /* Outward candidate motion must be visible to a new query, so an
           already-satisfied plane does not reapply its old positive depth. */
        assert(vec3_dot(point,normal)>vec3_dot(end,normal));
        {
            physx_chain_contact_pose_t before=pose;
            addon_chain_body_manifold_t opposed={0};
            float opposite[3],original[32][3];
            physx_chain_contact_point(&chain,&pose,0,start);
            physx_chain_contact_point(&chain,&pose,1,end);
            memcpy(original,pose.offset,sizeof(original));
            for(j=0;j<3;j++) opposite[j]=-normal[j];
            opposed.angular=1;
            addon_chain_body_manifold_store(&opposed,normal,0.02f,end);
            addon_chain_body_manifold_store(&opposed,opposite,0.02f,end);
            physx_chain_contact_solve(&chain,&pose,&opposed.contacts,opposed.point,start,end);
            for(i=1;i<=2;i++) {
                float delta[3];
                for(j=0;j<3;j++) delta[j]=pose.offset[i][j]-original[i][j];
                assert(physx_vec3_sane_limit(pose.offset[i],0.101f));
                assert(physx_vec3_len(delta)<=0.01601f);
            }
            /* One satisfied contact must preserve outward motion. */
            pose=before;
            for(i=1;i<=2;i++) memcpy(joints[i].sim_offset,pose.offset[i],sizeof(float)*3);
            physx_chain_contact_point(&chain,&pose,0,start);
            physx_chain_contact_point(&chain,&pose,1,end);
            physx_chain_contact_gradient(&chain,&pose,1,normal,gradient);
            for(i=1;i<=2;i++) for(j=0;j<3;j++) joints[i].sim_velocity[j]=0.01f*gradient[i][j];
            memset(&opposed,0,sizeof(opposed));
            opposed.contacts.count=1;
            memcpy(opposed.contacts.normal[0],normal,sizeof(normal));
            memcpy(opposed.point[0],end,sizeof(end));
            physx_chain_contact_solve(&chain,&pose,&opposed.contacts,opposed.point,start,end);
            for(i=1;i<=2;i++) for(j=0;j<3;j++)
                assert(fabsf(joints[i].sim_velocity[j]-0.01f*gradient[i][j])<1e-8f);
        }
        joints[2].contact_basis_tick=99;
        assert(!physx_chain_contact_pose_init(&chain,2,100,&pose));
    }
    puts("PASS: composed two-joint pose matches independent FK; candidate contacts retain distinct levers and satisfy both supports with shared normal velocity (8 configurations)");
    puts("PASS: contradictory supports stay bounded and separating contact velocity remains unchanged");
    {
        const int rates[3]={30,60,144};
        int rate;
        for(rate=0;rate<3;rate++) {
            physx_target_t joints[4]={0};
            physx_chain_t chain={0};
            float anchor[9]={1,0,0,0,1,0,0,0,1},dt=1.0f/rates[rate];
            float low=1,high=-1,end[3]={0};
            int frame,i,j,pass;
            chain.targets=joints; chain.target_count=4; chain.addon_chain=1;
            chain.h_axis=0; chain.v_axis=2; chain.root_bend_scale=1;
            chain.skinned_matrix_scale=1; chain.limit_angle=180;
            for(i=1;i<=2;i++) {
                joints[i].sim_length=0.1f; joints[i].addon_simulated_target=1;
                joints[i].sim_rest[1]=joints[i].sim_offset[1]=0.1f;
            }
            for(frame=0;frame<rates[rate]*8;frame++) {
                physx_chain_contact_pose_t pose;
                test_chain_snapshot(&chain,anchor);
                /* Spring-driven harness, using the production candidate/contact
                   functions. Engine animation and child drive are not modeled. */
                for(i=1;i<=2;i++) {
                    float length;
                    for(j=0;j<3;j++) {
                        joints[i].sim_velocity[j]+=(-80*(joints[i].sim_offset[j]-joints[i].sim_rest[j])-
                            4*joints[i].sim_velocity[j])*dt;
                        joints[i].sim_velocity[j]*=fmaxf(0,1-dt);
                        joints[i].sim_offset[j]+=joints[i].sim_velocity[j]*dt;
                    }
                    length=physx_vec3_len(joints[i].sim_offset);
                    for(j=0;j<3;j++) joints[i].sim_offset[j]*=0.1f/length;
                }
                for(pass=0;pass<3;pass++) {
                    addon_chain_body_manifold_t m={0};
                    float start[3],normal[3]={1,0,0};
                    assert(physx_chain_contact_pose_init(&chain,2,100,&pose));
                    physx_chain_contact_point(&chain,&pose,0,start);
                    physx_chain_contact_point(&chain,&pose,1,end);
                    m.angular=1;
                    if(frame<rates[rate]*6) {
                        addon_chain_body_manifold_store(&m,normal,0.002f-start[0],start);
                        addon_chain_body_manifold_store(&m,normal,0.002f-end[0],end);
                        physx_chain_contact_solve(&chain,&pose,&m.contacts,m.point,start,end);
                        physx_chain_contact_point(&chain,&pose,1,end);
                    }
                }
                if(frame>=rates[rate]*4 && frame<rates[rate]*6) {
                    low=fminf(low,end[0]); high=fmaxf(high,end[0]);
                }
                for(i=1;i<=2;i++) assert(fabsf(physx_vec3_len(joints[i].sim_offset)-0.1f)<1e-7f);
            }
            printf("CHAIN_REST hz=%d range=%.9f final_release=%.9f\n",rates[rate],high-low,end[0]);
            assert(high-low<0.00003f);
            assert(low>0.0018f);
            assert(fabsf(end[0])<0.0003f);
            /* Fixed output limits must stay finite and must not invent escape. */
            chain.limit_angle=0;
            test_chain_snapshot(&chain,anchor);
            {
                physx_chain_contact_pose_t pose;
                addon_chain_body_manifold_t m={0};
                float start[3],normal[3]={1,0,0};
                assert(physx_chain_contact_pose_init(&chain,2,100,&pose));
                physx_chain_contact_point(&chain,&pose,0,start);
                physx_chain_contact_point(&chain,&pose,1,end);
                m.angular=1;
                addon_chain_body_manifold_store(&m,normal,0.01f,end);
                assert(physx_chain_contact_solve(&chain,&pose,&m.contacts,m.point,start,end)==0);
            }
        }
    }
    puts("PASS: coupled spring/contact settling and release at 30/60/144 Hz; fixed output limits reject unreachable pushes");
    {
        int mode;
        for(mode=0;mode<8;mode++) {
            physx_target_t joints[4]={0};
            physx_chain_t chain={0};
            physx_chain_contact_pose_t pose,incoming;
            float anchor[9],angles[3]={13.0f*mode,17.0f*mode,-23.0f*mode};
            float start[3],end[3],before[3],expected[3],local[3]={0,0.15f,0},normal[3];
            addon_chain_body_manifold_t m={0};
            int i,j;
            chain.targets=joints; chain.target_count=3; chain.addon_chain=1;
            chain.h_axis=0; chain.v_axis=2; chain.root_bend_scale=1;
            chain.skinned_matrix_scale=0.8f; chain.limit_angle=180;
            chain.rotation_solver_full_angle=mode&1;
            for(i=1;i<=2;i++) {
                joints[i].sim_length=0.1f; joints[i].addon_simulated_target=1;
                joints[i].sim_rest[1]=joints[i].sim_offset[1]=0.1f;
            }
            joints[2].sim_rotation_rest[0]=(mode&2)?170:0;
            physx_contact_rotation_rows(angles,anchor);
            test_chain_snapshot(&chain,anchor);
            for(j=0;j<3;j++) {
                joints[2].contact_terminal_body[j]=joints[2].contact_pivot_body[j]+
                    0.15f*joints[2].contact_live_body_basis[3+j];
                normal[j]=joints[2].contact_live_body_basis[j];
            }
            assert(!physx_chain_contact_pose_init(&chain,2,100,&pose));
            joints[2].contact_terminal_tick=100;
            assert(physx_chain_contact_pose_init(&chain,2,100,&pose));
            assert(physx_chain_contact_pose_init(&chain,1,100,&incoming));
            physx_chain_contact_point(&chain,&pose,1,before);
            /* Last joint rotation moves the virtual end but cannot move its
               own pivot, which was the end of the previously checked segment. */
            pose.offset[2][0]=0.002f;
            {
                float length=physx_vec3_len(pose.offset[2]);
                for(j=0;j<3;j++) pose.offset[2][j]*=0.1f/length;
            }
            physx_chain_contact_point(&chain,&pose,0,start);
            physx_chain_contact_point(&chain,&pose,1,end);
            addon_chain_contact_predict(&chain,&joints[2],pose.offset[2],local,expected);
            for(j=0;j<3;j++) {
                expected[j]+=joints[2].contact_pivot_body[j];
                assert(fabsf(end[j]-expected[j])<2e-7f);
                assert(fabsf(start[j]-joints[2].contact_pivot_body[j])<2e-7f);
                before[j]=end[j]-before[j];
            }
            assert(physx_vec3_len(before)>0.0001f);
            for(i=1;i<=2;i++) memcpy(joints[i].sim_offset,pose.offset[i],sizeof(float)*3);
            /* Hold the supporting upstream joint at its current output limit.
               Terminal collision must be able to use the final joint alone. */
            joints[1].joint_settings_initialized=1;
            m.angular=1;
            addon_chain_body_manifold_store(&m,normal,0.0002f,end);
            assert(physx_chain_contact_solve(&chain,&pose,&m.contacts,m.point,start,end)==1);
            physx_chain_contact_point(&chain,&pose,1,expected);
            for(j=0;j<3;j++) expected[j]-=end[j];
            assert(vec3_dot(expected,normal)>0.000105f);
            assert(joints[2].sim_contact_corrected);
            assert(!joints[1].sim_contact_corrected);
            joints[2].contact_terminal_tick=99;
            assert(!physx_chain_contact_pose_init(&chain,2,100,&pose));
        }
    }
    puts("PASS: virtual terminal segment follows final-joint rotation and separates with upstream joint fixed; absent/stale tip snapshot rejected (8 configurations)");
}
