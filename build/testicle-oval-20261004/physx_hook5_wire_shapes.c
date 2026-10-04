/* Shared collider data -> a single homogeneous line batch. No physics tick,
   bitmap rasterization, screen-radius estimate or per-shape GPU draw call. */
static void physx_wire_set_view_frame(physx_wire_batch *batch,
    const float origin[3], const float x[3], const float y[3], const float z[3])
{
    const float *projection=(const float *)&body_chain_hook5_projection;
    const float *rows[4]={x,y,z,origin}; unsigned row,col;
    for(row=0;row<4;++row) for(col=0;col<4;++col)
        batch->matrix[row*4+col]=rows[row][0]*projection[col]+
            rows[row][1]*projection[4+col]+rows[row][2]*projection[8+col]+
            (row==3?projection[12+col]:0);
}
static void physx_wire_person_frame(physx_wire_batch *batch,
    const body_chain_collider_person_state_t *state)
{
    physx_wire_set_view_frame(batch,state->view_position[BODY_COLLIDER_ROOT],
        state->basis_h,state->basis_v,state->basis_s);
}
static void physx_wire_body_edge(physx_wire_batch *batch,
    const body_chain_collider_person_state_t *state, int start, int end, DWORD color)
{
    if(!state->valid[start] || !state->valid[end] ||
       !body_collider_debug_edge_selected(start,end)) return;
    physx_wire_capsule(batch,state->local_position[start],state->local_position[end],
        body_chain_collider_visual_radius_for_node(start),
        body_chain_collider_visual_radius_for_node(end),color);
}
static void physx_wire_collect_body(physx_wire_batch *batch)
{
    int person,i;
    for(person=0;person<4;++person) {
        body_chain_collider_person_state_t *state=&body_chain_collider_states[person];
        body_chain_person_state_t *chain=&body_chain_person_states[person];
        body_profile_set_active_person_config(person);
        if(!body_chain_collider_cfg.debug_draw || !body_chain_collider_debug_person_active(person)) continue;
        physx_wire_person_frame(batch,state);
        for(i=0;i<BODY_COLLIDER_NODE_COUNT;++i) {
            float axes[3];
            if(!state->valid[i] || i==BODY_COLLIDER_TESTICLES_MID ||
               !body_collider_debug_node_selected(i)) continue;
            if(i>=BODY_COLLIDER_TESTICLES_01 && i<=BODY_COLLIDER_TESTICLES_02 &&
                (!body_chain_collider_cfg.testicle_collision_enabled ||
                 physx_genitals_paused(person))) continue;
            /* Physical target dimensions: the solver separately adds the
               moving chain radius for contact queries. Draw each volume once. */
            if(i==BODY_COLLIDER_STOMACH_01 || i==BODY_COLLIDER_STOMACH_02) {
                if(!state->stomach_points_ready) continue;
                body_chain_collider_visual_stomach_radius_axes(i==BODY_COLLIDER_STOMACH_02,axes);
            } else {
                if(!body_collider_debug_custom_view() &&
                   i<=BODY_COLLIDER_TESTICLES_MID && i<BODY_COLLIDER_TESTICLES_01 &&
                    !body_chain_collider_node_radius_is_oval(i)) continue;
                body_chain_collider_visual_radius_axes_for_node(i,axes);
            }
            physx_wire_ellipsoid(batch,state->local_position[i],axes,body_chain_collider_node_color_d3d(i));
        }
        for(i=0;i<BODY_COLLIDER_EXTRA_EDGE_COUNT;++i) {
            const body_collider_extra_edge_def_t *edge=&body_collider_extra_edges[i];
            physx_wire_body_edge(batch,state,edge->start_node,edge->end_node,edge->d3d_color);
        }
        for(i=0;i<BODY_COLLIDER_LIMB_PAIR_COUNT;++i) {
            int start,end; body_chain_collider_limb_pair_nodes(i,&start,&end);
            physx_wire_body_edge(batch,state,start,end,0xff20ffff);
        }
        if(body_chain_collider_cfg.testicle_collision_enabled && !physx_genitals_paused(person))
            physx_wire_body_edge(batch,state,BODY_COLLIDER_TESTICLES_01,BODY_COLLIDER_TESTICLES_02,0xffff40ff);
        if(body_collider_debug_chain_selected() &&
           body_chain_collider_cfg.penis_collision_enabled && !physx_genitals_paused(person)) {
            float points[4][3]; DWORD now=GetTickCount(); int ready=0,source=0;
            if(state->chain_points_ready && state->chain_points_update_tick &&
                now-state->chain_points_update_tick<=BODY_CHAIN_ENGINE_POINT_STALE_MS) {
                ready=1;
                for(i=0;i<4;++i) { if(!state->chain_point_valid[i]) ready=0;
                    memcpy(points[i],state->chain_local_point[i],sizeof(points[i])); }
            } else if(chain->initialized) {
                ready=body_chain_collision_points_local(state,chain,points,&source,now);
            }
            if(ready) body_chain_penis_offset_points(points, 1.0f);
            if(ready) for(i=0;i<3;++i)
                physx_wire_capsule(batch,points[i],points[i+1],body_chain_penis_radius(),
                    body_chain_penis_radius(),0xffffa000);
        }
    }
    body_profile_set_active_person_config(-1);
}
static void physx_wire_collect_addons(physx_wire_batch *batch)
{
    int i,c,j;
    for(i=0;i<sidecar_count;++i) {
        physx_sidecar_t *sc=&sidecars[i];
        if(!sc->loaded || !sc->enabled) continue;
        for(c=0;c<sc->chain_count;++c) {
            physx_chain_t *chain=&sc->chains[c]; float points[32][3]; int count=0,person=-1;
            if(!chain->addon_chain || !chain->collision_enabled || !chain->collision_debug_draw ||
                chain->target_count<=1) continue;
            if(!addon_chain_collision_points_body_local(sc,chain,points,&count,&person,GetTickCount()) ||
                person<0 || person>=4 || count<2 || count>32) continue;
            if(!body_chain_collider_states[person].basis_valid ||
                !body_chain_collider_states[person].valid[BODY_COLLIDER_ROOT]) continue;
            physx_wire_person_frame(batch,&body_chain_collider_states[person]);
            for(j=0;j+1<count;++j) physx_wire_capsule(batch,points[j],points[j+1],
                chain->collision_radius,chain->collision_radius,chain->collision_debug_color);
        }
    }
}
static void physx_wire_collect_room(physx_wire_batch *batch)
{
    int i,k; float origin[3],zero[3]={0},basis[3][3];
    if(!room_collision_debug_any() || !room_collision_world_to_view_point(zero,origin)) return;
    for(i=0;i<3;++i) {
        float unit[3]={0},view[3]; unit[i]=1;
        if(!room_collision_world_to_view_point(unit,view)) return;
        for(k=0;k<3;++k) basis[i][k]=view[k]-origin[k];
    }
    physx_wire_set_view_frame(batch,origin,basis[0],basis[1],basis[2]);
    for(i=0;i<room_collision_triangle_count;++i) {
        room_collision_triangle_t *tri=&room_collision_triangles[i]; DWORD color=0xffffffffu;
        if(tri->mesh_index>=0 && tri->mesh_index<room_collision_mesh_count)
            color=room_collision_meshes[tri->mesh_index].color;
        for(k=0;k<3;++k) physx_wire_line(batch,tri->v[k],tri->v[(k+1)%3],color);
    }
}
static void physx_wire_collect(physx_wire_batch *batch)
{
    batch->count=0; batch->dropped=0;
    physx_wire_collect_body(batch);
    physx_wire_collect_addons(batch);
    physx_wire_collect_room(batch);
}
