#include "NC-TK17-PhysX.c"
#include <assert.h>

int main(void)
{
    physx_wire_batch batch={0};
    body_chain_collider_person_state_t *state=&body_chain_collider_states[0];
    body_chain_collider_person_state_t saved;
    float target_axes[3],contact_axes[3]; unsigned i,k;
    memset(&body_chain_hook5_projection,0,sizeof(body_chain_hook5_projection));
    body_chain_hook5_projection._11=body_chain_hook5_projection._22=
        body_chain_hook5_projection._33=body_chain_hook5_projection._44=1;
    body_chain_collider_person_cfg[0].enabled=1;
    body_chain_collider_person_cfg[0].debug_draw=1;
    body_chain_collider_person_cfg[0].response_radius_scale=2;
    body_chain_collider_person_cfg[0].chain_radius=0.05f;
    body_chain_collider_person_cfg[0].node_radius[BODY_COLLIDER_BREAST_L][0]=0.1f;
    body_chain_collider_person_cfg[0].node_radius[BODY_COLLIDER_BREAST_L][1]=0.05f;
    body_chain_collider_person_cfg[0].node_radius[BODY_COLLIDER_BREAST_L][2]=0.075f;
    state->valid[BODY_COLLIDER_ROOT]=state->valid[BODY_COLLIDER_BREAST_L]=1;
    state->basis_valid=state->ready=1;
    state->view_position[BODY_COLLIDER_ROOT][2]=0.5f;
    /* Rotated body basis must affect every ring, not only its center. */
    state->basis_h[1]=1; state->basis_v[0]=-1; state->basis_s[2]=1;
    saved=*state;
    body_profile_set_active_person_config(0);
    body_chain_collider_visual_radius_axes_for_node(BODY_COLLIDER_BREAST_L,target_axes);
    body_chain_collider_effective_radius_axes_for_node(BODY_COLLIDER_BREAST_L,contact_axes);
    for(k=0;k<3;++k) assert(fabsf(contact_axes[k]-target_axes[k]-0.05f)<0.000001f);
    body_profile_set_active_person_config(-1);
    physx_wire_collect_body(&batch);
    assert(batch.count==192);
    assert(body_chain_collider_cfg_ptr==&body_chain_collider_global_cfg);
    assert(memcmp(state,&saved,sizeof(saved))==0);
    for(i=0;i<batch.count;++i) {
        const float *p=batch.vertices[i].clip;
        float x=p[1]/target_axes[0],y=-p[0]/target_axes[1],z=(p[2]-0.5f)/target_axes[2];
        assert(fabsf(x*x+y*y+z*z-1)<0.00001f);
    }
    state->scene_liveness_quarantined=1;
    batch.count=0; physx_wire_collect_body(&batch); assert(batch.count==0);
    state->scene_liveness_quarantined=0;
    body_chain_collider_person_cfg[0].debug_draw=0;
    batch.count=0; physx_wire_collect_body(&batch); assert(batch.count==0);
    free(batch.vertices);
    puts("PASS: real collector uses scaled collider axes/body basis, excludes moving-radius inflation, preserves solver state/config and respects liveness/debug flags");
    return 0;
}
