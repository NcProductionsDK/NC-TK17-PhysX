#include "NC-TK17-PhysX.c"
#include <assert.h>
#undef assert
#define assert(ok) do { if(!(ok)) { fprintf(stderr,"FAIL line %d: %s (customizer=%d)\n",__LINE__,#ok,physx_customizer_active); exit(1); } } while(0)

static const float identity[9]={1,0,0,0,1,0,0,0,1};
static const int nodes[2]={BODY_COLLIDER_WRIST_L,BODY_COLLIDER_PALM_L};

static void setup(int owner, int collider)
{
    body_profile_set_active_person_config(-1);
    memset(body_chain_collider_states,0,sizeof(body_chain_collider_states));
    memset(body_chain_collider_projection_cache,0,sizeof(body_chain_collider_projection_cache));
    memset(&single_bone_published,0,sizeof(single_bone_published));
    physx_simulation_serial++;
    defaults_cfg.debug=0;
    body_chain_collider_cfg.enabled=1;
    body_chain_collider_cfg.root_local_offsets=1;
    body_chain_collider_cfg.penis_collision_enabled=1;
    body_chain_collider_cfg.testicle_collision_enabled=1;
    body_chain_collider_cfg.breasts_collision_enabled=1;
    body_chain_collider_cfg.butt_collision_enabled=1;
    body_chain_collider_cfg.response_strength=1;
    body_chain_collider_cfg.response_radius_scale=1;
    body_chain_collider_cfg.response_max_degrees_per_tick=20;
    body_chain_collider_cfg.collision_slop=0;
    body_chain_collider_cfg.collision_iterations=2;
    body_chain_collider_cfg.chain_radius=.02f;
    for(int n=0;n<2;n++) for(int a=0;a<3;a++)
        body_chain_collider_cfg.node_radius[nodes[n]][a]=.04f;
    body_chain_physics_cfg.collision_scope=BODY_CHAIN_COLLISION_SCOPE_FULL_BODY_ALL;
    body_chain_physics_cfg.collision_strength=1;
    body_chain_physics_cfg.horizontal_output_axis=1;
    body_chain_physics_cfg.vertical_output_axis=2;
    body_chain_physics_cfg.room_collision_enabled=0;
    for(int j=0;j<3;j++) for(int a=0;a<3;a++) {
        body_chain_physics_cfg.link_min_angle[j][a]=-90;
        body_chain_physics_cfg.link_max_angle[j][a]=90;
    }
    for(int p=0;p<4;p++) {
        body_chain_collider_person_cfg[p]=body_chain_collider_cfg;
        body_chain_physics_person_cfg[p]=body_chain_physics_cfg;
        testicle_physics_person_cfg[p]=body_chain_physics_cfg;
    }
    body_chain_collider_states[owner].ready=1;
    body_chain_collider_states[owner].basis_valid=1;
    body_chain_collider_person_state_t *s=&body_chain_collider_states[collider];
    s->ready=s->basis_valid=s->contact_frame_valid=1;
    memcpy(s->contact_view_to_world,identity,sizeof(identity));
    body_chain_collider_projection_cache_t *cache=
        &body_chain_collider_projection_cache[owner][collider];
    cache->simulation_serial=physx_simulation_serial;
    cache->active_node_count=2;
    for(int n=0;n<2;n++) {
        int node=nodes[n];
        s->valid[node]=cache->valid[node]=1;
        cache->active_node[n]=(unsigned char)node;
        s->view_position[node][0]=cache->point[node][0]=.04f+n*.04f;
        s->view_position[node][1]=cache->point[node][1]=.02f;
    }
}

static void paired_contacts(int owner,int collider,int butt,int expected)
{
    single_bone_contacts_t contacts={0};
    float center[3]={.05f,0,0},x[3]={0};
    setup(owner,collider);
    single_bone_body_contacts(owner,butt,0,BODY_CHAIN_COLLISION_SCOPE_FULL_BODY_ALL,
        center,.04f,identity,x,x,1,&contacts);
    assert((contacts.count>0)==expected);
}

static void chain_contacts(int owner,int collider,int testicle,int expected)
{
    body_chain_person_state_t state={0};
    float correction[3][2]={{0}};
    setup(owner,collider);
    state.collision_step_valid=1;
    state.collision_step_tick=1000;
    state.collision_step_dt=.016f;
    for(int j=0;j<4;j++) state.collision_step_points[j][0]=j*.05f;
    state.collision_prev_collider_ready[collider]=1;
    body_chain_compute_collider_projection(owner,&state,correction,1000,0,
        NULL,NULL,NULL,testicle);
    assert((state.collision_manifold_contacts>0)==expected);
    if(!expected) assert(!state.collision_prev_collider_ready[collider]);
}

static void addon_contacts(int owner,int collider,int point_query,int expected)
{
    static physx_chain_t chain;
    float start[3]={0},end[3]={.05f,0,0};
    setup(owner,collider);
    memset(&chain,0,sizeof(chain));
    chain.addon_chain=1;
    chain.collision_scope=PHYSX_COLLISION_SCOPE_BODY_ALL;
    chain.collision_radius=.04f;
    chain.target_count=2;
    chain.targets[1].sim_offset[0]=.05f;
    body_chain_collider_states[owner].valid[BODY_COLLIDER_ROOT]=1;
    body_chain_collider_states[collider].valid[BODY_COLLIDER_ROOT]=1;
    addon_body_node_frame_cache_t *cache=&addon_body_node_frame_cache[owner][collider];
    memset(cache,0,sizeof(*cache));
    cache->ready=1;cache->tick=1000;
    for(int n=0;n<2;n++) {
        cache->valid[nodes[n]]=1;
        memcpy(cache->point[nodes[n]],body_chain_collider_states[collider].view_position[nodes[n]],sizeof(float)*3);
    }
    int hit=point_query ? addon_chain_apply_body_point_collision(&chain,&chain.targets[1],
        owner,collider,end,1000,.016f) :
        addon_chain_apply_body_collision(NULL,&chain,&chain.targets[1],
            owner,collider,start,NULL,1,1000,.016f,0);
    assert((hit>0)==expected);
    if(!expected) assert(chain.targets[1].sim_offset[0]==.05f &&
        chain.targets[1].sim_offset[1]==0 && chain.targets[1].sim_offset[2]==0);
}

int main(void)
{
    /* Reused ready/nonzero room colliders must not need a camera movement
       to stop pushing the selected Customizer body. Check every body slot. */
    for(int owner=0;owner<4;owner++) for(int kind=0;kind<2;kind++) {
        int other=(owner+1)%4;
        physx_customizer_active=0;
        paired_contacts(owner,other,kind,1);
        chain_contacts(owner,other,kind,1);
        addon_contacts(owner,other,kind,1);
        physx_customizer_active=1;
        paired_contacts(owner,other,kind,0);
        chain_contacts(owner,other,kind,0);
        addon_contacts(owner,other,kind,0);
        paired_contacts(owner,owner,kind,1);
        chain_contacts(owner,owner,kind,1);
        addon_contacts(owner,owner,kind,1);
        physx_customizer_active=0;
        paired_contacts(owner,other,kind,1);
        chain_contacts(owner,other,kind,1);
        addon_contacts(owner,other,kind,1);
    }
    puts("PASS: Customizer rejects leftover room contacts for both chains, paired systems and add-on point/segment queries; own contacts and room re-entry preserved");
    return 0;
}
