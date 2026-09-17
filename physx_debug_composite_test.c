#define WIN32_LEAN_AND_MEAN
#define CINTERFACE
#define COBJMACROS
#include <windows.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "physx_debug_composite.h"
#include "../NC-TK17-Hook5-Extended/external_debug_bridge.h"

static unsigned order;
static void __cdecl regular_draw(ID3D11DeviceContext *c, ID3D11RenderTargetView *t,
    ID3D11DepthStencilView *d, unsigned w, unsigned h)
{ (void)c; (void)t; (void)d; assert(w == 64 && h == 48); assert(order++ == 0); }
static void __cdecl debug_draw(ID3D11DeviceContext *c, ID3D11RenderTargetView *t,
    ID3D11DepthStencilView *d, unsigned w, unsigned h)
{ (void)c; (void)t; (void)d; assert(w == 64 && h == 48); assert(order++ == 1); }

static void check_pixel(ID3D11DeviceContext *c, ID3D11Texture2D *texture,
    ID3D11Texture2D *readback, unsigned x, unsigned y, DWORD expected)
{
    D3D11_MAPPED_SUBRESOURCE mapped;
    DWORD actual;
    ID3D11DeviceContext_CopyResource(c, (ID3D11Resource *)readback, (ID3D11Resource *)texture);
    assert(SUCCEEDED(ID3D11DeviceContext_Map(c, (ID3D11Resource *)readback, 0, D3D11_MAP_READ, 0, &mapped)));
    actual = *(DWORD *)((BYTE *)mapped.pData + y * mapped.RowPitch + x * 4);
    ID3D11DeviceContext_Unmap(c, (ID3D11Resource *)readback, 0);
    if (actual != expected) fprintf(stderr, "pixel %u,%u: %08lx expected %08lx\n", x,y,actual,expected);
    assert(actual == expected);
}


static void identity(physx_wire_batch *b)
{
    memset(b->matrix,0,sizeof(b->matrix));
    b->matrix[0]=b->matrix[5]=b->matrix[10]=b->matrix[15]=1;
    b->count=0; b->dropped=0;
}
static void geometry_test(void)
{
    physx_wire_batch b={0}; unsigned i;
    const float center[3]={0,0,0.5f}, axes[3]={0.2f,0.1f,0.15f};
    const float start[3]={-0.25f,0,0.5f},end[3]={0.25f,0,0.5f};
    identity(&b); physx_wire_ellipsoid(&b,center,axes,0xff123456);
    assert(b.count==3*PHYSX_WIRE_STEPS*2);
    for(i=0;i<b.count;++i) {
        float sum=0; unsigned k;
        for(k=0;k<3;++k) { float q=(b.vertices[i].clip[k]-center[k])/axes[k]; sum+=q*q; }
        assert(fabsf(sum-1)<0.00001f);
        assert(b.vertices[i].rgba==0xff563412);
    }
    identity(&b); physx_wire_capsule(&b,start,end,0.1f,0.2f,0xffffffff);
    assert(b.count>200);
    for(i=0;i<b.count;++i) {
        const float *p=b.vertices[i].clip;
        float t=(p[0]+0.25f)/0.5f, q, radius, distance;
        if(t<0)t=0; if(t>1)t=1;
        q=-0.25f+0.5f*t; radius=0.1f+0.1f*t;
        distance=sqrtf((p[0]-q)*(p[0]-q)+p[1]*p[1]+(p[2]-0.5f)*(p[2]-0.5f));
        assert(fabsf(distance-radius)<0.00001f);
    }
    identity(&b); physx_wire_capsule(&b,center,center,0.1f,0.2f,0xffffffff);
    assert(b.count==3*PHYSX_WIRE_STEPS*2);
    /* Rotate an anisotropic collider 90 degrees: its long axis must rotate. */
    identity(&b); b.matrix[0]=b.matrix[5]=0; b.matrix[1]=1; b.matrix[4]=-1;
    physx_wire_ellipsoid(&b,center,axes,0xffffffff);
    { float max_x=0,max_y=0;
      for(i=0;i<b.count;++i) {
        float x=fabsf(b.vertices[i].clip[0]),y=fabsf(b.vertices[i].clip[1]);
        if(x>max_x)max_x=x; if(y>max_y)max_y=y;
      }
      assert(fabsf(max_x-0.1f)<0.00001f && fabsf(max_y-0.2f)<0.00001f); }
    /* Clip-space W is retained, with no 320-pixel radius limit. */
    identity(&b); b.matrix[15]=0; b.matrix[11]=1;
    { const float a[3]={0,0,0.25f},z[3]={0.2f,0,0.5f};
      physx_wire_line(&b,a,z,0xffffffff);
      assert(b.count==2 && b.vertices[0].clip[3]==0.25f && b.vertices[1].clip[3]==0.5f); }
    identity(&b);
    { const float a[3]={2,0,0.5f},z[3]={3,0,0.5f};
      physx_wire_line(&b,a,z,0xffffffff); assert(b.count==0); }
    free(b.vertices);
}
static void screen_line(physx_wire_batch *b,unsigned width,unsigned height,
    float x0,float y0,float z0,float x1,float y1,float z1,DWORD color)
{
    float a[3]={2*x0/width-1,1-2*y0/height,z0};
    float z[3]={2*x1/width-1,1-2*y1/height,z1};
    physx_wire_line(b,a,z,color);
}
static void render_test(ID3D11Device *device, ID3D11DeviceContext *c,
    physx_debug_surface *surface, unsigned width, unsigned height)
{
    D3D11_TEXTURE2D_DESC desc={0};
    ID3D11Texture2D *target=NULL,*readback=NULL;
    ID3D11RenderTargetView *rtv=NULL,*restored=NULL;
    ID3D11PixelShader *restored_ps=NULL;
    ID3D11RasterizerState *restored_rs=NULL;
    ID3D11Buffer *restored_buffer=NULL,*sentinel=NULL;
    ID3D11InputLayout *restored_layout=NULL;
    D3D11_BUFFER_DESC buffer_desc={0};
    D3D11_VIEWPORT viewport={3,4,17,19,0.2f,0.8f},got;
    D3D11_PRIMITIVE_TOPOLOGY topology;
    UINT count,stride=24,offset=16,got_stride,got_offset;
    const FLOAT blue[4]={0,0,1,1};
    assert(physx_debug_surface_prepare(surface,device,width,height));
    { ID3D11VertexShader *saved=surface->vs;
      assert(physx_debug_surface_prepare(surface,device,width,height)); assert(surface->vs==saved); }
    desc.Width=width; desc.Height=height; desc.MipLevels=1; desc.ArraySize=1;
    desc.Format=DXGI_FORMAT_B8G8R8A8_UNORM; desc.SampleDesc.Count=1;
    desc.BindFlags=D3D11_BIND_RENDER_TARGET;
    assert(SUCCEEDED(ID3D11Device_CreateTexture2D(device,&desc,NULL,&target)));
    assert(SUCCEEDED(ID3D11Device_CreateRenderTargetView(device,(ID3D11Resource *)target,NULL,&rtv)));
    desc.BindFlags=0; desc.Usage=D3D11_USAGE_STAGING; desc.CPUAccessFlags=D3D11_CPU_ACCESS_READ;
    assert(SUCCEEDED(ID3D11Device_CreateTexture2D(device,&desc,NULL,&readback)));
    buffer_desc.ByteWidth=1024; buffer_desc.BindFlags=D3D11_BIND_VERTEX_BUFFER;
    assert(SUCCEEDED(ID3D11Device_CreateBuffer(device,&buffer_desc,NULL,&sentinel)));
    ID3D11DeviceContext_ClearRenderTargetView(c,rtv,blue);
    ID3D11DeviceContext_OMSetRenderTargets(c,1,&rtv,NULL);
    ID3D11DeviceContext_RSSetViewports(c,1,&viewport);
    ID3D11DeviceContext_IASetPrimitiveTopology(c,D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
    ID3D11DeviceContext_IASetVertexBuffers(c,0,1,&sentinel,&stride,&offset);
    ID3D11DeviceContext_IASetInputLayout(c,surface->layout);
    ID3D11DeviceContext_PSSetShader(c,surface->ps,NULL,0);
    ID3D11DeviceContext_RSSetState(c,surface->raster);
    identity(&surface->lines);
    screen_line(&surface->lines,width,height,8.5f,16.5f,0.5f,40.5f,16.5f,0.5f,0xffff0000);
    assert(physx_debug_surface_draw(surface,c,rtv));
    check_pixel(c,target,readback,16,16,0xffff0000u);
    check_pixel(c,target,readback,0,0,0xff0000ffu);
    ID3D11DeviceContext_OMGetRenderTargets(c,1,&restored,NULL);
    assert(restored==rtv); ID3D11RenderTargetView_Release(restored);
    count=1; ID3D11DeviceContext_RSGetViewports(c,&count,&got);
    assert(count==1 && memcmp(&got,&viewport,sizeof(got))==0);
    ID3D11DeviceContext_IAGetPrimitiveTopology(c,&topology);
    assert(topology==D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
    ID3D11DeviceContext_IAGetVertexBuffers(c,0,1,&restored_buffer,&got_stride,&got_offset);
    assert(restored_buffer==sentinel && got_stride==stride && got_offset==offset);
    ID3D11Buffer_Release(restored_buffer);
    ID3D11DeviceContext_IAGetInputLayout(c,&restored_layout);
    assert(restored_layout==surface->layout); ID3D11InputLayout_Release(restored_layout);
    ID3D11DeviceContext_PSGetShader(c,&restored_ps,NULL,NULL);
    assert(restored_ps==surface->ps); ID3D11PixelShader_Release(restored_ps);
    ID3D11DeviceContext_RSGetState(c,&restored_rs);
    assert(restored_rs==surface->raster); ID3D11RasterizerState_Release(restored_rs);
    /* Later UI geometry covers the debug layer. */
    identity(&surface->lines);
    screen_line(&surface->lines,width,height,12.5f,16.5f,0.5f,20.5f,16.5f,0.5f,0xff00ff00);
    assert(physx_debug_surface_draw(surface,c,rtv));
    check_pixel(c,target,readback,16,16,0xff00ff00u);
    check_pixel(c,target,readback,32,16,0xffff0000u);
    /* Near-plane intersection clips only the hidden portion. */
    ID3D11DeviceContext_ClearRenderTargetView(c,rtv,blue);
    identity(&surface->lines);
    screen_line(&surface->lines,width,height,8.5f,16.5f,-0.5f,40.5f,16.5f,0.5f,0xffff0000);
    assert(surface->lines.count==2 && physx_debug_surface_draw(surface,c,rtv));
    check_pixel(c,target,readback,12,16,0xff0000ffu);
    check_pixel(c,target,readback,32,16,0xffff0000u);
    /* Nothing is submitted for an empty or wholly clipped batch. */
    identity(&surface->lines);
    screen_line(&surface->lines,width,height,8.5f,16.5f,-0.5f,40.5f,16.5f,-0.1f,0xffffffff);
    assert(surface->lines.count==0 && !physx_debug_surface_draw(surface,c,rtv));
    ID3D11DeviceContext_ClearState(c); ID3D11Buffer_Release(sentinel);
    ID3D11RenderTargetView_Release(rtv); ID3D11Texture2D_Release(readback); ID3D11Texture2D_Release(target);
}
int main(void)
{
    ID3D11Device *device=NULL;
    ID3D11DeviceContext *context=NULL;
    physx_debug_surface surface={0}; unsigned i;
    geometry_test();
    assert(!get_external_scene_callback() && !get_external_composite_callback());
    external_debug_composite_callback=debug_draw;
    assert(get_external_scene_callback()==external_capture_only);
    order=1; get_external_composite_callback()(NULL,NULL,NULL,64,48); assert(order==2);
    external_scene_callback=regular_draw; external_composite_callback=regular_draw;
    assert(get_external_scene_callback()==regular_draw);
    order=0; get_external_composite_callback()(NULL,NULL,NULL,64,48); assert(order==2);
    external_debug_composite_callback=NULL;
    assert(get_external_composite_callback()==regular_draw);
    external_scene_callback=NULL; external_composite_callback=NULL;
    assert(!get_external_scene_callback() && !get_external_composite_callback());
    for(i=0;i<2;++i) {
        assert(SUCCEEDED(D3D11CreateDevice(NULL,D3D_DRIVER_TYPE_WARP,NULL,0,NULL,0,
            D3D11_SDK_VERSION,&device,NULL,&context)));
        render_test(device,context,&surface,64,48);
        { ID3D11Buffer *saved=surface.buffer;
          render_test(device,context,&surface,96,72); assert(surface.buffer==saved); }
        ID3D11DeviceContext_Release(context); ID3D11Device_Release(device);
    }
    physx_debug_surface_destroy(&surface);
    assert(!surface.device && !surface.buffer && !surface.lines.vertices);
    puts("PASS: ellipsoid/capsule geometry, rotated axes, WARP lines, clipping, draw order, state restore, resource reuse, device replacement and callback coexistence");
    return 0;
}
