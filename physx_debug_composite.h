/* Batched 3D collision wireframes in the completed Hook5 scene. */
#ifndef PHYSX_DEBUG_COMPOSITE_H
#define PHYSX_DEBUG_COMPOSITE_H
#include <d3d11.h>
#include <d3dcompiler.h>
#include "physx_debug_geometry.h"
typedef struct physx_debug_surface {
    UINT width, height, gpu_capacity;
    physx_wire_batch lines;
    ID3D11Device *device;
    ID3D11Buffer *buffer;
    ID3D11InputLayout *layout;
    ID3D11VertexShader *vs;
    ID3D11PixelShader *ps;
    ID3D11RasterizerState *raster;
    ID3D11DepthStencilState *depth;
    ID3D11BlendState *blend;
} physx_debug_surface;
#define PHYSX_COM_RELEASE(p) do { if (p) { IUnknown_Release((IUnknown *)(p)); (p) = NULL; } } while (0)
static void physx_debug_surface_destroy(physx_debug_surface *s)
{
    free(s->lines.vertices);
    PHYSX_COM_RELEASE(s->buffer); PHYSX_COM_RELEASE(s->layout);
    PHYSX_COM_RELEASE(s->vs); PHYSX_COM_RELEASE(s->ps);
    PHYSX_COM_RELEASE(s->raster); PHYSX_COM_RELEASE(s->depth);
    PHYSX_COM_RELEASE(s->blend); PHYSX_COM_RELEASE(s->device);
    memset(s,0,sizeof(*s));
}
static int physx_debug_surface_prepare(physx_debug_surface *s,
    ID3D11Device *device, UINT width, UINT height)
{
    static const char shader[] =
        "struct V {float4 p:SV_Position;float4 c:COLOR;};"
        "V vs(float4 p:POSITION,float4 c:COLOR) {V o;o.p=p;o.c=c;return o;}"
        "float4 ps(V i):SV_Target {return i.c;}";
    typedef HRESULT (WINAPI *compile_t)(LPCVOID,SIZE_T,LPCSTR,
        const D3D_SHADER_MACRO *,ID3DInclude *,LPCSTR,LPCSTR,UINT,UINT,
        ID3DBlob **,ID3DBlob **);
    HMODULE compiler;
    compile_t compile;
    ID3DBlob *vs=NULL,*ps=NULL,*error=NULL;
    D3D11_RASTERIZER_DESC raster={0};
    D3D11_DEPTH_STENCIL_DESC depth={0};
    D3D11_BLEND_DESC blend={0};
    const D3D11_INPUT_ELEMENT_DESC layout[]={
        {"POSITION",0,DXGI_FORMAT_R32G32B32A32_FLOAT,0,0,D3D11_INPUT_PER_VERTEX_DATA,0},
        {"COLOR",0,DXGI_FORMAT_R8G8B8A8_UNORM,0,16,D3D11_INPUT_PER_VERTEX_DATA,0}};
    int ok=0;
    if(!device || !width || !height || width>16384 || height>16384) return 0;
    if(s->device==device) { s->width=width; s->height=height; return 1; }
    physx_debug_surface_destroy(s);
    compiler=LoadLibraryA("d3dcompiler_47.dll");
    if(!compiler) compiler=LoadLibraryA("d3dcompiler_43.dll");
    if(!compiler) return 0;
    compile=(compile_t)GetProcAddress(compiler,"D3DCompile");
    if(!compile) goto done;
    if(FAILED(compile(shader,sizeof(shader)-1,NULL,NULL,NULL,"vs","vs_4_0",
        D3DCOMPILE_OPTIMIZATION_LEVEL3,0,&vs,&error))) goto done;
    PHYSX_COM_RELEASE(error);
    if(FAILED(compile(shader,sizeof(shader)-1,NULL,NULL,NULL,"ps","ps_4_0",
        D3DCOMPILE_OPTIMIZATION_LEVEL3,0,&ps,&error))) goto done;
    if(FAILED(ID3D11Device_CreateVertexShader(device,ID3D10Blob_GetBufferPointer(vs),
        ID3D10Blob_GetBufferSize(vs),NULL,&s->vs))) goto done;
    if(FAILED(ID3D11Device_CreatePixelShader(device,ID3D10Blob_GetBufferPointer(ps),
        ID3D10Blob_GetBufferSize(ps),NULL,&s->ps))) goto done;
    if(FAILED(ID3D11Device_CreateInputLayout(device,layout,2,ID3D10Blob_GetBufferPointer(vs),
        ID3D10Blob_GetBufferSize(vs),&s->layout))) goto done;
    raster.FillMode=D3D11_FILL_SOLID; raster.CullMode=D3D11_CULL_NONE;
    raster.DepthClipEnable=TRUE;
    if(FAILED(ID3D11Device_CreateRasterizerState(device,&raster,&s->raster))) goto done;
    depth.DepthFunc=D3D11_COMPARISON_ALWAYS;
    if(FAILED(ID3D11Device_CreateDepthStencilState(device,&depth,&s->depth))) goto done;
    blend.RenderTarget[0].RenderTargetWriteMask=D3D11_COLOR_WRITE_ENABLE_ALL;
    if(FAILED(ID3D11Device_CreateBlendState(device,&blend,&s->blend))) goto done;
    s->width=width; s->height=height; s->device=device; ID3D11Device_AddRef(device); ok=1;
done:
    PHYSX_COM_RELEASE(error); PHYSX_COM_RELEASE(vs); PHYSX_COM_RELEASE(ps);
    FreeLibrary(compiler); if(!ok) physx_debug_surface_destroy(s);
    return ok;
}

/* Preserve class instances as well as shaders; Hook5 and other extensions
   share this context. Constants, textures, samplers and scissors are untouched. */
#define PHYSX_SHADER_STATE(stage, type) \
    ID3D11##type##Shader *old_##stage = NULL; \
    ID3D11ClassInstance *instances_##stage[256]; UINT count_##stage = 256
#define PHYSX_SHADER_SAVE(stage) \
    ID3D11DeviceContext_##stage##GetShader(c, &old_##stage, instances_##stage, &count_##stage)
#define PHYSX_SHADER_RESTORE(stage) do { \
    ID3D11DeviceContext_##stage##SetShader(c, old_##stage, instances_##stage, count_##stage); \
    PHYSX_COM_RELEASE(old_##stage); \
    for (i = 0; i < count_##stage; ++i) PHYSX_COM_RELEASE(instances_##stage[i]); \
} while (0)

static int physx_debug_surface_draw(physx_debug_surface *s,
    ID3D11DeviceContext *c, ID3D11RenderTargetView *target)
{
    D3D11_MAPPED_SUBRESOURCE mapped;
    ID3D11RenderTargetView *rt[8] = {0};
    ID3D11DepthStencilView *dsv = NULL;
    ID3D11BlendState *blend = NULL;
    ID3D11DepthStencilState *depth = NULL;
    ID3D11RasterizerState *raster = NULL;
    ID3D11InputLayout *layout = NULL;
    ID3D11Buffer *old_buffer = NULL;
    UINT old_stride, old_offset, stride = sizeof(physx_wire_vertex), offset = 0;
    ID3D11Predicate *predicate = NULL;
    ID3D11Buffer *so[4] = {0};
    BOOL predicate_value;
    FLOAT factors[4]; UINT mask, stencil, i, count = 16;
    D3D11_VIEWPORT viewports[16], viewport = {0};
    D3D11_PRIMITIVE_TOPOLOGY topology;
    PHYSX_SHADER_STATE(VS, Vertex);
    PHYSX_SHADER_STATE(PS, Pixel);
    PHYSX_SHADER_STATE(GS, Geometry);
    PHYSX_SHADER_STATE(HS, Hull);
    PHYSX_SHADER_STATE(DS, Domain);
    int stream_output_active = 0;
    if (!c || !target || !s->device || !s->lines.count) return 0;
    /* A debug draw must never append to the engine's stream-output buffers. */
    ID3D11DeviceContext_SOGetTargets(c, 4, so);
    for (i = 0; i < 4; ++i) { if (so[i]) stream_output_active = 1; PHYSX_COM_RELEASE(so[i]); }
    if (stream_output_active) return 0;
    if (s->gpu_capacity < s->lines.count) {
        D3D11_BUFFER_DESC desc = {0}; ID3D11Buffer *buffer = NULL;
        desc.ByteWidth = s->lines.capacity * sizeof(physx_wire_vertex);
        desc.Usage = D3D11_USAGE_DYNAMIC; desc.BindFlags = D3D11_BIND_VERTEX_BUFFER;
        desc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
        if (FAILED(ID3D11Device_CreateBuffer(s->device,&desc,NULL,&buffer))) return 0;
        PHYSX_COM_RELEASE(s->buffer); s->buffer=buffer; s->gpu_capacity=s->lines.capacity;
    }
    if (FAILED(ID3D11DeviceContext_Map(c,(ID3D11Resource *)s->buffer,
        0,D3D11_MAP_WRITE_DISCARD,0,&mapped))) return 0;
    memcpy(mapped.pData,s->lines.vertices,s->lines.count*sizeof(physx_wire_vertex));
    ID3D11DeviceContext_Unmap(c,(ID3D11Resource *)s->buffer,0);
    ID3D11DeviceContext_OMGetRenderTargets(c, 8, rt, &dsv);
    ID3D11DeviceContext_OMGetBlendState(c, &blend, factors, &mask);
    ID3D11DeviceContext_OMGetDepthStencilState(c, &depth, &stencil);
    ID3D11DeviceContext_RSGetState(c, &raster);
    ID3D11DeviceContext_RSGetViewports(c, &count, viewports);
    ID3D11DeviceContext_IAGetInputLayout(c, &layout);
    ID3D11DeviceContext_IAGetPrimitiveTopology(c, &topology);
    ID3D11DeviceContext_IAGetVertexBuffers(c, 0, 1, &old_buffer, &old_stride, &old_offset);
    ID3D11DeviceContext_GetPredication(c, &predicate, &predicate_value);
    PHYSX_SHADER_SAVE(VS); PHYSX_SHADER_SAVE(PS); PHYSX_SHADER_SAVE(GS);
    PHYSX_SHADER_SAVE(HS); PHYSX_SHADER_SAVE(DS);
    viewport.Width = (FLOAT)s->width; viewport.Height = (FLOAT)s->height;
    viewport.MaxDepth = 1;
    ID3D11DeviceContext_OMSetRenderTargetsAndUnorderedAccessViews(c, 1, &target,
        NULL, 0, D3D11_KEEP_UNORDERED_ACCESS_VIEWS, NULL, NULL);
    ID3D11DeviceContext_OMSetBlendState(c, s->blend, NULL, 0xffffffffu);
    ID3D11DeviceContext_OMSetDepthStencilState(c, s->depth, 0);
    ID3D11DeviceContext_RSSetState(c, s->raster);
    ID3D11DeviceContext_RSSetViewports(c, 1, &viewport);
    ID3D11DeviceContext_IASetInputLayout(c, s->layout);
    ID3D11DeviceContext_IASetVertexBuffers(c,0,1,&s->buffer,&stride,&offset);
    ID3D11DeviceContext_IASetPrimitiveTopology(c, D3D11_PRIMITIVE_TOPOLOGY_LINELIST);
    ID3D11DeviceContext_VSSetShader(c, s->vs, NULL, 0);
    ID3D11DeviceContext_PSSetShader(c, s->ps, NULL, 0);
    ID3D11DeviceContext_GSSetShader(c, NULL, NULL, 0);
    ID3D11DeviceContext_HSSetShader(c, NULL, NULL, 0);
    ID3D11DeviceContext_DSSetShader(c, NULL, NULL, 0);
    ID3D11DeviceContext_SetPredication(c, NULL, FALSE);
    ID3D11DeviceContext_Draw(c, s->lines.count, 0);
    /* Use the original RTV count so occupied UAV slots remain legal. */
    for (i = 8; i && !rt[i-1]; --i) {}
    ID3D11DeviceContext_OMSetRenderTargetsAndUnorderedAccessViews(c, i, rt,
        dsv, 0, D3D11_KEEP_UNORDERED_ACCESS_VIEWS, NULL, NULL);
    ID3D11DeviceContext_IASetVertexBuffers(c,0,1,&old_buffer,&old_stride,&old_offset);
    ID3D11DeviceContext_OMSetBlendState(c, blend, factors, mask);
    ID3D11DeviceContext_OMSetDepthStencilState(c, depth, stencil);
    ID3D11DeviceContext_RSSetState(c, raster);
    ID3D11DeviceContext_RSSetViewports(c, count, viewports);
    ID3D11DeviceContext_IASetInputLayout(c, layout);
    ID3D11DeviceContext_IASetPrimitiveTopology(c, topology);
    ID3D11DeviceContext_SetPredication(c, predicate, predicate_value);
    PHYSX_SHADER_RESTORE(VS); PHYSX_SHADER_RESTORE(PS); PHYSX_SHADER_RESTORE(GS);
    PHYSX_SHADER_RESTORE(HS); PHYSX_SHADER_RESTORE(DS);
    for (i = 0; i < 8; ++i) PHYSX_COM_RELEASE(rt[i]);
    PHYSX_COM_RELEASE(dsv); PHYSX_COM_RELEASE(blend); PHYSX_COM_RELEASE(depth);
    PHYSX_COM_RELEASE(raster); PHYSX_COM_RELEASE(layout); PHYSX_COM_RELEASE(old_buffer);
    PHYSX_COM_RELEASE(predicate);
    return 1;
}
#undef PHYSX_SHADER_STATE
#undef PHYSX_SHADER_SAVE
#undef PHYSX_SHADER_RESTORE
#undef PHYSX_COM_RELEASE
#endif
