#include "NC-TK17-PhysX.c"
#define CHECK(ok) do { if(!(ok)) { fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#ok); exit(1); } } while(0)

static void configure(const float axes[3])
{
    body_profile_set_active_person_config(-1);
    body_chain_collider_cfg.response_radius_scale=.75f;
    body_chain_collider_cfg.chain_radius=.0275f;
    for(int node=BODY_COLLIDER_TESTICLES_01;node<=BODY_COLLIDER_TESTICLES_MID;node++)
        memcpy(body_chain_collider_cfg.node_radius[node],axes,sizeof(float)*3);
}

static float query(const float p[3],const float a[3],const float b[3],
    float cp[3],float cq[3],float *radius)
{
    float t,u,dist,margin;
    CHECK(body_chain_native_testicle_pair_margin(p,p,a,b,0,&t,&u,cp,cq,&dist,radius,&margin));
    CHECK(isfinite(margin));
    return margin;
}

static void contact_axes(void)
{
    const float oval[3]={.030f,.042f,.045f},zero[3]={0};
    float cp[3],cq[3],radius;
    configure(oval);
    for(int axis=0;axis<3;axis++) {
        float p[3]={0},r=oval[axis]*.75f+.0275f;
        p[axis]=r+.003f;
        CHECK(fabsf(query(p,zero,zero,cp,cq,&radius)-.003f)<.000001f);
        p[axis]=r-.003f;
        CHECK(fabsf(query(p,zero,zero,cp,cq,&radius)+.003f)<.000001f);
    }
    /* Diagonal surface points must lie on the configured ellipsoid and
       supply its gradient, rather than a max-radius sphere or radial normal. */
    float p[3],expected[3],len=0;
    const float unit[3]={.6f,0,.8f};
    for(int k=0;k<3;k++) {
        float r=oval[k]*.75f+.0275f;
        p[k]=unit[k]*r;expected[k]=unit[k]/r;len+=expected[k]*expected[k];
    }
    CHECK(fabsf(query(p,zero,zero,cp,cq,&radius))<.000001f);
    float normal[3];for(int k=0;k<3;k++) normal[k]=cp[k]-cq[k];
    float normal_len=physx_vec3_len(normal);
    for(int k=0;k<3;k++) CHECK(fabsf(normal[k]/normal_len-expected[k]/sqrtf(len))<.00001f);
    /* Only X changes; the largest component stays Z in both configurations. */
    const float wide[3]={.045f,.042f,.045f};
    const float beside[3]={.052f,0,0},a[3]={0,0,-.1f},b[3]={0,0,.1f};
    configure(wide);CHECK(query(beside,a,b,cp,cq,&radius)<0);
    configure(oval);CHECK(query(beside,a,b,cp,cq,&radius)>0);
    CHECK(query(zero,a,b,cp,cq,&radius)<0);
    CHECK(physx_vec3_len(cp)>0 || physx_vec3_len(cq)>.0001f);
    puts("PASS: independent XYZ edits, capsule side/end surfaces, diagonal normals and exact centerline crossings");
}

static void drawing_axes(void)
{
    const float oval[3]={.030f,.042f,.045f};
    physx_wire_batch batch={0};
    body_chain_collider_person_state_t state={0};
    float saved[16],axes[3],max_x=0;
    configure(oval);
    body_chain_collider_cfg.debug_draw_capsules=1;
    body_collider_debug_set_filter("all");
    state.valid[BODY_COLLIDER_TESTICLES_01]=state.valid[BODY_COLLIDER_TESTICLES_02]=1;
    state.local_position[BODY_COLLIDER_TESTICLES_02][2]=.12f;
    batch.matrix[0]=batch.matrix[5]=batch.matrix[10]=batch.matrix[15]=1;
    batch.matrix[14]=.5f;
    memcpy(saved,batch.matrix,sizeof(saved));
    body_chain_collider_visual_radius_axes_for_node(BODY_COLLIDER_TESTICLES_01,axes);
    physx_wire_body_edge(&batch,&state,BODY_COLLIDER_TESTICLES_01,BODY_COLLIDER_TESTICLES_02,0xffff40ff);
    CHECK(batch.count>0);
    CHECK(!memcmp(saved,batch.matrix,sizeof(saved)));
    for(unsigned i=0;i<batch.count;i++) {
        float x=batch.vertices[i].clip[0],y=batch.vertices[i].clip[1];
        float z=batch.vertices[i].clip[2]-.5f;
        float center=fmaxf(0,fminf(.12f,z));
        float surface=x*x/(axes[0]*axes[0])+y*y/(axes[1]*axes[1])+
            (z-center)*(z-center)/(axes[2]*axes[2]);
        CHECK(fabsf(surface-1)<.00002f);
        max_x=fmaxf(max_x,fabsf(x));
    }
    CHECK(fabsf(max_x-.030f*.75f)<.000001f);
    free(batch.vertices);
    /* Legacy D3D/OpenGL connector endpoints remain on the projected ellipses. */
    float p0[2]={0,0},p1[2]={0,10};
    float major[2][2]={{2,0},{2,0}},minor[2][2]={{0,4},{0,4}},offset[2][2];
    CHECK(body_debug_ellipse_connector_offsets(p0,p1,major,minor,offset));
    for(int j=0;j<2;j++) { CHECK(fabsf(fabsf(offset[j][0])-2)<.000001f);CHECK(fabsf(offset[j][1])<.000001f); }
    puts("PASS: production wire capsule matches ellipsoid caps, uses scaled axes, restores draw frame; legacy connectors follow projected ellipse width");
}

int main(void)
{
    contact_axes();drawing_axes();return 0;
}
