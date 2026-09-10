typedef struct body_collider_debug_vertex_t {
    float x, y, z, rhw;
    DWORD color;
} body_collider_debug_vertex_t;

typedef void (APIENTRY *physx_gl_begin_t)(GLenum);
typedef void (APIENTRY *physx_gl_end_t)(void);
typedef void (APIENTRY *physx_gl_vertex2f_t)(GLfloat, GLfloat);
typedef void (APIENTRY *physx_gl_color4ub_t)(GLubyte, GLubyte, GLubyte, GLubyte);
typedef void (APIENTRY *physx_gl_get_floatv_t)(GLenum, GLfloat *);
typedef void (APIENTRY *physx_gl_get_integerv_t)(GLenum, GLint *);
typedef void (APIENTRY *physx_gl_push_attrib_t)(GLbitfield);
typedef void (APIENTRY *physx_gl_pop_attrib_t)(void);
typedef void (APIENTRY *physx_gl_disable_t)(GLenum);
typedef void (APIENTRY *physx_gl_matrix_mode_t)(GLenum);
typedef void (APIENTRY *physx_gl_push_matrix_t)(void);
typedef void (APIENTRY *physx_gl_pop_matrix_t)(void);
typedef void (APIENTRY *physx_gl_load_identity_t)(void);
typedef void (APIENTRY *physx_gl_ortho_t)(GLdouble, GLdouble, GLdouble, GLdouble, GLdouble, GLdouble);
typedef void (APIENTRY *physx_gl_line_width_t)(GLfloat);

static physx_gl_begin_t physx_gl_begin;
static physx_gl_end_t physx_gl_end;
static physx_gl_vertex2f_t physx_gl_vertex2f;
static physx_gl_color4ub_t physx_gl_color4ub;
static physx_gl_get_floatv_t physx_gl_get_floatv;
static physx_gl_get_integerv_t physx_gl_get_integerv;
static physx_gl_push_attrib_t physx_gl_push_attrib;
static physx_gl_pop_attrib_t physx_gl_pop_attrib;
static physx_gl_disable_t physx_gl_disable;
static physx_gl_matrix_mode_t physx_gl_matrix_mode;
static physx_gl_push_matrix_t physx_gl_push_matrix;
static physx_gl_pop_matrix_t physx_gl_pop_matrix;
static physx_gl_load_identity_t physx_gl_load_identity;
static physx_gl_ortho_t physx_gl_ortho;
static physx_gl_line_width_t physx_gl_line_width;
static int physx_gl_debug_api_attempted;
static DWORD d3d_chain_draw_log_tick[4];
static DWORD gl_chain_draw_log_tick[4];
static HDC body_chain_gdi_debug_hdc;
static int body_chain_gdi_debug_mode;
static int body_chain_gdi_debug_primitives;
static HWND body_chain_hook5_overlay_hwnd;
static HDC body_chain_hook5_overlay_dc;
static HBITMAP body_chain_hook5_overlay_bitmap;
static HGDIOBJ body_chain_hook5_overlay_old_bitmap;
static int body_chain_hook5_overlay_w;
static int body_chain_hook5_overlay_h;

static COLORREF body_chain_gdi_color(DWORD color)
{
    return RGB((color >> 16) & 0xff, (color >> 8) & 0xff, color & 0xff);
}

static void body_chain_gdi_select_pen(COLORREF color,
                                      HPEN *pen_out,
                                      HGDIOBJ *old_pen_out)
{
    HPEN pen;
    if (pen_out) *pen_out = NULL;
    if (old_pen_out) *old_pen_out = NULL;
    if (!body_chain_gdi_debug_hdc) return;
    pen = CreatePen(PS_SOLID, 2, color);
    if (!pen) return;
    if (pen_out) *pen_out = pen;
    if (old_pen_out) {
        *old_pen_out = SelectObject(body_chain_gdi_debug_hdc, pen);
    } else {
        SelectObject(body_chain_gdi_debug_hdc, pen);
    }
}

static void body_chain_gdi_restore_pen(HPEN pen, HGDIOBJ old_pen)
{
    if (!body_chain_gdi_debug_hdc || !pen) return;
    if (old_pen) SelectObject(body_chain_gdi_debug_hdc, old_pen);
    DeleteObject(pen);
}

static int resolve_body_collider_gl_api(void)
{
    HMODULE gl;
    if (physx_gl_debug_api_attempted) return physx_gl_begin != NULL;
    physx_gl_debug_api_attempted = 1;
    gl = GetModuleHandleA("opengl32.dll");
    if (!gl) return 0;
#define LOAD_GL_DEBUG(NAME, TYPE, SYMBOL) \
    physx_gl_##NAME = (TYPE)GetProcAddress(gl, SYMBOL)
    LOAD_GL_DEBUG(begin, physx_gl_begin_t, "glBegin");
    LOAD_GL_DEBUG(end, physx_gl_end_t, "glEnd");
    LOAD_GL_DEBUG(vertex2f, physx_gl_vertex2f_t, "glVertex2f");
    LOAD_GL_DEBUG(color4ub, physx_gl_color4ub_t, "glColor4ub");
    LOAD_GL_DEBUG(get_floatv, physx_gl_get_floatv_t, "glGetFloatv");
    LOAD_GL_DEBUG(get_integerv, physx_gl_get_integerv_t, "glGetIntegerv");
    LOAD_GL_DEBUG(push_attrib, physx_gl_push_attrib_t, "glPushAttrib");
    LOAD_GL_DEBUG(pop_attrib, physx_gl_pop_attrib_t, "glPopAttrib");
    LOAD_GL_DEBUG(disable, physx_gl_disable_t, "glDisable");
    LOAD_GL_DEBUG(matrix_mode, physx_gl_matrix_mode_t, "glMatrixMode");
    LOAD_GL_DEBUG(push_matrix, physx_gl_push_matrix_t, "glPushMatrix");
    LOAD_GL_DEBUG(pop_matrix, physx_gl_pop_matrix_t, "glPopMatrix");
    LOAD_GL_DEBUG(load_identity, physx_gl_load_identity_t, "glLoadIdentity");
    LOAD_GL_DEBUG(ortho, physx_gl_ortho_t, "glOrtho");
    LOAD_GL_DEBUG(line_width, physx_gl_line_width_t, "glLineWidth");
#undef LOAD_GL_DEBUG
    return physx_gl_begin && physx_gl_end && physx_gl_vertex2f &&
           physx_gl_color4ub && physx_gl_get_floatv &&
           physx_gl_get_integerv && physx_gl_push_attrib &&
           physx_gl_pop_attrib && physx_gl_disable &&
           physx_gl_matrix_mode && physx_gl_push_matrix &&
           physx_gl_pop_matrix && physx_gl_load_identity &&
           physx_gl_ortho && physx_gl_line_width;
}

static int project_d3d8_view_point(IDirect3DDevice8 *device,
                                   const float point[3],
                                   float *screen_x,
                                   float *screen_y)
{
    D3DMATRIX projection;
    D3DVIEWPORT8 viewport;
    float clip_x, clip_y, clip_w;
    if (!device || !point || !screen_x || !screen_y) return 0;
    if (FAILED(IDirect3DDevice8_GetTransform(device, D3DTS_PROJECTION,
                                             &projection)) ||
        FAILED(IDirect3DDevice8_GetViewport(device, &viewport))) {
        return 0;
    }
    clip_x = point[0] * projection._11 + point[1] * projection._21 +
             point[2] * projection._31 + projection._41;
    clip_y = point[0] * projection._12 + point[1] * projection._22 +
             point[2] * projection._32 + projection._42;
    clip_w = point[0] * projection._14 + point[1] * projection._24 +
             point[2] * projection._34 + projection._44;
    /* A nonzero negative W is still mathematically projectable, but it is
       behind the camera. Dividing by it mirrors room geometry onto the
       screen, which previously produced long wireframe lines in the sky. */
    if (!sane_probe_float(clip_w) || clip_w <= 0.00001f) return 0;
    *screen_x = viewport.X + (1.0f + clip_x / clip_w) * viewport.Width * 0.5f;
    *screen_y = viewport.Y + (1.0f - clip_y / clip_w) * viewport.Height * 0.5f;
    return sane_probe_float(*screen_x) && sane_probe_float(*screen_y);
}

static void draw_d3d8_debug_line(IDirect3DDevice8 *device,
                                 float x0, float y0,
                                 float x1, float y1,
                                 DWORD color)
{
    body_collider_debug_vertex_t v[2];
    if (body_chain_gdi_debug_mode && body_chain_gdi_debug_hdc) {
        HPEN pen;
        HGDIOBJ old_pen;
        body_chain_gdi_select_pen(body_chain_gdi_color(color), &pen, &old_pen);
        if (pen) {
            MoveToEx(body_chain_gdi_debug_hdc, (int)(x0 + 0.5f),
                     (int)(y0 + 0.5f), NULL);
            LineTo(body_chain_gdi_debug_hdc, (int)(x1 + 0.5f),
                   (int)(y1 + 0.5f));
            body_chain_gdi_debug_primitives++;
            body_chain_gdi_restore_pen(pen, old_pen);
        }
        return;
    }
    v[0].x = x0; v[0].y = y0; v[0].z = 0.0f; v[0].rhw = 1.0f; v[0].color = color;
    v[1].x = x1; v[1].y = y1; v[1].z = 0.0f; v[1].rhw = 1.0f; v[1].color = color;
    IDirect3DDevice8_DrawPrimitiveUP(device, D3DPT_LINELIST, 1, v, sizeof(v[0]));
}

static void draw_d3d8_debug_circle(IDirect3DDevice8 *device,
                                   float cx, float cy, float radius,
                                   DWORD color)
{
    body_collider_debug_vertex_t v[25];
    int i;
    if (body_chain_gdi_debug_mode && body_chain_gdi_debug_hdc) {
        HPEN pen;
        HGDIOBJ old_pen;
        HGDIOBJ old_brush;
        body_chain_gdi_select_pen(body_chain_gdi_color(color), &pen, &old_pen);
        old_brush = SelectObject(body_chain_gdi_debug_hdc,
                                 GetStockObject(NULL_BRUSH));
        if (pen) {
            Ellipse(body_chain_gdi_debug_hdc,
                    (int)(cx - radius + 0.5f),
                    (int)(cy - radius + 0.5f),
                    (int)(cx + radius + 0.5f),
                    (int)(cy + radius + 0.5f));
            body_chain_gdi_debug_primitives++;
            body_chain_gdi_restore_pen(pen, old_pen);
        }
        if (old_brush) SelectObject(body_chain_gdi_debug_hdc, old_brush);
        return;
    }
    for (i = 0; i <= 24; i++) {
        float a = (float)i / 24.0f * 6.28318530717958647692f;
        v[i].x = cx + (float)cos((double)a) * radius;
        v[i].y = cy + (float)sin((double)a) * radius;
        v[i].z = 0.0f;
        v[i].rhw = 1.0f;
        v[i].color = color;
    }
    IDirect3DDevice8_DrawPrimitiveUP(device, D3DPT_LINESTRIP, 24, v, sizeof(v[0]));
}

static int body_chain_debug_screen_float(float v)
{
    return v == v && v > -1000000000000.0f && v < 1000000000000.0f;
}

static int body_chain_debug_projected_radius_stable(float radius_px)
{
    return body_chain_debug_screen_float(radius_px) &&
           radius_px > 0.0f &&
           radius_px <= 640.0f;
}

static int body_chain_debug_ellipse_from_cov(float a, float b, float c,
                                             float major[2], float minor[2])
{
    float trace;
    float diff;
    float root;
    float lambda_major;
    float lambda_minor;
    float vx, vy;
    float len;
    float major_len;
    float minor_len;
    const float max_axis_len = 320.0f;
    if (!major || !minor) return 0;
    trace = (a + c) * 0.5f;
    diff = (a - c) * 0.5f;
    root = (float)sqrt((double)(diff * diff + b * b));
    lambda_major = trace + root;
    lambda_minor = trace - root;
    if (!body_chain_debug_screen_float(lambda_major) ||
        lambda_major <= 0.25f) return 0;
    if (lambda_minor < 0.0f ||
        !body_chain_debug_screen_float(lambda_minor)) {
        lambda_minor = 0.0f;
    }
    if (physx_absf(b) > 0.00001f) {
        vx = lambda_major - c;
        vy = b;
    } else if (a >= c) {
        vx = 1.0f;
        vy = 0.0f;
    } else {
        vx = 0.0f;
        vy = 1.0f;
    }
    len = (float)sqrt((double)(vx * vx + vy * vy));
    if (len <= 0.00001f || !body_chain_debug_screen_float(len)) {
        vx = 1.0f;
        vy = 0.0f;
        len = 1.0f;
    }
    vx /= len;
    vy /= len;
    major_len = (float)sqrt((double)lambda_major);
    minor_len = (float)sqrt((double)lambda_minor);
    if (!body_chain_debug_projected_radius_stable(major_len)) return 0;
    if (!body_chain_debug_screen_float(major_len) ||
        major_len <= 0.5f) return 0;
    if (!body_chain_debug_screen_float(minor_len) || minor_len < 0.25f) {
        minor_len = 0.25f;
    }
    if (major_len > max_axis_len) {
        float scale = max_axis_len / major_len;
        major_len = max_axis_len;
        minor_len *= scale;
        if (minor_len < 0.25f) minor_len = 0.25f;
    }
    major[0] = vx * major_len;
    major[1] = vy * major_len;
    minor[0] = -vy * minor_len;
    minor[1] = vx * minor_len;
    return 1;
}

static void draw_d3d8_debug_ellipse(IDirect3DDevice8 *device,
                                    float cx, float cy,
                                    const float major[2],
                                    const float minor[2],
                                    DWORD color)
{
    body_collider_debug_vertex_t v[33];
    int i;
    if (!device || !major || !minor) return;
    if (body_chain_gdi_debug_mode && body_chain_gdi_debug_hdc) {
        float extent_x = physx_absf(major[0]) + physx_absf(minor[0]);
        float extent_y = physx_absf(major[1]) + physx_absf(minor[1]);
        HPEN pen;
        HGDIOBJ old_pen;
        HGDIOBJ old_brush;
        if (extent_x < 0.5f) extent_x = 0.5f;
        if (extent_y < 0.5f) extent_y = 0.5f;
        body_chain_gdi_select_pen(body_chain_gdi_color(color), &pen, &old_pen);
        old_brush = SelectObject(body_chain_gdi_debug_hdc,
                                 GetStockObject(NULL_BRUSH));
        if (pen) {
            Ellipse(body_chain_gdi_debug_hdc,
                    (int)(cx - extent_x + 0.5f),
                    (int)(cy - extent_y + 0.5f),
                    (int)(cx + extent_x + 0.5f),
                    (int)(cy + extent_y + 0.5f));
            body_chain_gdi_debug_primitives++;
            body_chain_gdi_restore_pen(pen, old_pen);
        }
        if (old_brush) SelectObject(body_chain_gdi_debug_hdc, old_brush);
        return;
    }
    for (i = 0; i <= 32; i++) {
        float t = (float)i / 32.0f * 6.28318530717958647692f;
        float co = (float)cos((double)t);
        float si = (float)sin((double)t);
        v[i].x = cx + major[0] * co + minor[0] * si;
        v[i].y = cy + major[1] * co + minor[1] * si;
        v[i].z = 0.0f;
        v[i].rhw = 1.0f;
        v[i].color = color;
    }
    IDirect3DDevice8_DrawPrimitiveUP(device, D3DPT_LINESTRIP, 32, v, sizeof(v[0]));
}

static void draw_d3d8_debug_capsule(IDirect3DDevice8 *device,
                                    float x0, float y0,
                                    float x1, float y1,
                                    float radius, DWORD color)
{
    float dx = x1 - x0;
    float dy = y1 - y0;
    float len = (float)sqrt((double)(dx * dx + dy * dy));
    float nx, ny;
    if (len < 0.001f) {
        draw_d3d8_debug_circle(device, x0, y0, radius, color);
        return;
    }
    nx = -dy / len * radius;
    ny = dx / len * radius;
    draw_d3d8_debug_line(device, x0 + nx, y0 + ny, x1 + nx, y1 + ny, color);
    draw_d3d8_debug_line(device, x0 - nx, y0 - ny, x1 - nx, y1 - ny, color);
    draw_d3d8_debug_circle(device, x0, y0, radius, color);
    draw_d3d8_debug_circle(device, x1, y1, radius, color);
}

static void draw_d3d8_debug_tapered_capsule(IDirect3DDevice8 *device,
                                            float x0, float y0, float r0,
                                            float x1, float y1, float r1,
                                            DWORD color)
{
    float dx = x1 - x0;
    float dy = y1 - y0;
    float len = (float)sqrt((double)(dx * dx + dy * dy));
    float nx, ny;
    if (len < 0.001f) {
        draw_d3d8_debug_circle(device, x0, y0, r0 > r1 ? r0 : r1, color);
        return;
    }
    nx = -dy / len;
    ny = dx / len;
    draw_d3d8_debug_line(device,
                         x0 + nx * r0, y0 + ny * r0,
                         x1 + nx * r1, y1 + ny * r1, color);
    draw_d3d8_debug_line(device,
                         x0 - nx * r0, y0 - ny * r0,
                         x1 - nx * r1, y1 - ny * r1, color);
    draw_d3d8_debug_circle(device, x0, y0, r0, color);
    draw_d3d8_debug_circle(device, x1, y1, r1, color);
}

static int body_chain_debug_projected_radius_d3d8(
    IDirect3DDevice8 *device,
    const body_chain_collider_person_state_t *state,
    const float local[3],
    float center_x,
    float center_y,
    float radius,
    float *radius_px)
{
    int axis;
    float max_px = 0.0f;
    if (!device || !state || !local || !radius_px || radius <= 0.0f) return 0;
    for (axis = 0; axis < 3; axis++) {
        float edge_local[3] = { local[0], local[1], local[2] };
        float edge_view[3];
        float draw_edge[3];
        float edge_x, edge_y;
        float dx, dy, dist_px;
        edge_local[axis] += radius;
        if (!body_chain_collider_local_to_view(state, edge_local, edge_view) ||
            !body_collider_draw_view_point(edge_view, draw_edge) ||
            !project_d3d8_view_point(device, draw_edge, &edge_x, &edge_y)) {
            continue;
        }
        dx = edge_x - center_x;
        dy = edge_y - center_y;
        dist_px = (float)sqrt((double)(dx * dx + dy * dy));
        if (sane_probe_float(dist_px) && dist_px > max_px) {
            max_px = dist_px;
        }
    }
    if (!body_chain_debug_projected_radius_stable(max_px)) return 0;
    *radius_px = physx_clampf(max_px, 0.25f, 320.0f);
    return 1;
}

static int body_chain_debug_projected_ellipse_d3d8(
    IDirect3DDevice8 *device,
    const body_chain_collider_person_state_t *state,
    const float local[3],
    const float axes[3],
    float center_x,
    float center_y,
    float major[2],
    float minor[2])
{
    float cov_xx = 0.0f;
    float cov_xy = 0.0f;
    float cov_yy = 0.0f;
    int axis;
    int used = 0;
    if (!device || !state || !local || !axes || !major || !minor) return 0;
    for (axis = 0; axis < 3; axis++) {
        float plus_local[3] = { local[0], local[1], local[2] };
        float minus_local[3] = { local[0], local[1], local[2] };
        float plus_view[3], minus_view[3];
        float draw_plus[3], draw_minus[3];
        float plus_x, plus_y, minus_x, minus_y;
        float step;
        float scale;
        float dx, dy;
        int plus_ok;
        int minus_ok;
        if (axes[axis] <= 0.0f) continue;
        step = physx_clampf(axes[axis] * 0.01f, 0.00025f, 0.002f);
        plus_local[axis] += step;
        minus_local[axis] -= step;
        plus_ok = body_chain_collider_local_to_view(state, plus_local,
                                                     plus_view) &&
                  body_collider_draw_view_point(plus_view, draw_plus) &&
                  project_d3d8_view_point(device, draw_plus,
                                          &plus_x, &plus_y);
        minus_ok = body_chain_collider_local_to_view(state, minus_local,
                                                      minus_view) &&
                   body_collider_draw_view_point(minus_view, draw_minus) &&
                   project_d3d8_view_point(device, draw_minus,
                                           &minus_x, &minus_y);
        if (plus_ok && minus_ok) {
            scale = axes[axis] / (2.0f * step);
            dx = (plus_x - minus_x) * scale;
            dy = (plus_y - minus_y) * scale;
        } else if (plus_ok) {
            scale = axes[axis] / step;
            dx = (plus_x - center_x) * scale;
            dy = (plus_y - center_y) * scale;
        } else if (minus_ok) {
            scale = axes[axis] / step;
            dx = (center_x - minus_x) * scale;
            dy = (center_y - minus_y) * scale;
        } else {
            continue;
        }
        if (!sane_probe_float(dx) || !sane_probe_float(dy)) continue;
        cov_xx += dx * dx;
        cov_xy += dx * dy;
        cov_yy += dy * dy;
        used = 1;
    }
    if (!used) return 0;
    return body_chain_debug_ellipse_from_cov(cov_xx, cov_xy, cov_yy,
                                            major, minor);
}

static int body_chain_collider_debug_person_active(int person_index)
{
    body_chain_collider_person_state_t *state;
    static int unsettled_debug_logged[4];
    if (person_index < 0 || person_index >= 4) return 0;
    if (!body_chain_collider_cfg.enabled) return 0;
    state = &body_chain_collider_states[person_index];
    if (!state->basis_valid ||
        !state->valid[BODY_COLLIDER_ROOT] ||
        state->scene_liveness_engine_invisible ||
        state->scene_liveness_quarantined) {
        return 0;
    }
    if (!state->ready && !unsettled_debug_logged[person_index]) {
        unsettled_debug_logged[person_index] = 1;
        log_line("body-chain-colliders draw using unsettled body-local frame person_index=%d sample_ready=%d chain_points_ready=%d limb_points_ready=%d note=\"debug visuals and add-on sidecar collision can use populated body-local points even while native penis/testicle response waits for full ready promotion\"",
                 person_index + 1,
                 state->sample_ready,
                 state->chain_points_ready,
                 state->limb_points_ready);
    }
    return 1;
}

static void draw_addon_sidecar_colliders_d3d8(IDirect3DDevice8 *device)
{
    int i, c;
    static int addon_d3d_draw_logged;
    if (!device) return;
    for (i = 0; i < sidecar_count; i++) {
        physx_sidecar_t *sc = &sidecars[i];
        if (!sc->loaded || !sc->enabled) continue;
        for (c = 0; c < sc->chain_count; c++) {
            physx_chain_t *chain = &sc->chains[c];
            float local_points[32][3];
            float p[32][2];
            float radius_px[32];
            int point_count = 0;
            int person_index = -1;
            int j;
            if (!chain->addon_chain ||
                !chain->collision_enabled ||
                !chain->collision_debug_draw ||
                chain->target_count <= 1) {
                continue;
            }
            if (!addon_chain_collision_points_body_local(
                    sc, chain, local_points, &point_count,
                    &person_index, GetTickCount())) {
                continue;
            }
            if (person_index < 0 || person_index >= 4) continue;
            for (j = 0; j < point_count; j++) {
                float view[3];
                float draw_point[3];
                if (!body_chain_collider_local_to_view(
                        &body_chain_collider_states[person_index],
                        local_points[j], view) ||
                    !body_collider_draw_view_point(view, draw_point) ||
                    !project_d3d8_view_point(device, draw_point,
                                             &p[j][0], &p[j][1]) ||
                    !body_chain_debug_projected_radius_d3d8(
                        device,
                        &body_chain_collider_states[person_index],
                        local_points[j], p[j][0], p[j][1],
                        chain->collision_radius, &radius_px[j])) {
                    point_count = 0;
                    break;
                }
                radius_px[j] = physx_clampf(radius_px[j], 0.25f, 320.0f);
            }
            if (point_count <= 1) continue;
            for (j = 0; j + 1 < point_count; j++) {
                float radius = radius_px[j] > radius_px[j + 1] ?
                               radius_px[j] : radius_px[j + 1];
                draw_d3d8_debug_capsule(device,
                                        p[j][0], p[j][1],
                                        p[j + 1][0], p[j + 1][1],
                                        radius,
                                        chain->collision_debug_color);
            }
            if (!addon_d3d_draw_logged) {
                addon_d3d_draw_logged = 1;
                log_line("addon sidecar collision debug draw d3d8 active chain=\"%s\" person_index=%d points=%d radius=%.4f color=0x%08lx sidecar=\"%s\" note=\"sidecar collider visuals are independent from body collider debug draw\"",
                         chain->name,
                         person_index + 1,
                         point_count,
                         chain->collision_radius,
                         (unsigned long)chain->collision_debug_color,
                         sc->path);
            }
        }
    }
}

static void addon_sidecar_debug_color_rgb(DWORD color,
                                          unsigned char *r,
                                          unsigned char *g,
                                          unsigned char *b)
{
    if (r) *r = (unsigned char)((color >> 16) & 0xff);
    if (g) *g = (unsigned char)((color >> 8) & 0xff);
    if (b) *b = (unsigned char)(color & 0xff);
}

static int room_collision_world_to_view_point(const float world[3],
                                              float view[3])
{
    const float *m = captured_camera_inverse;
    float delta[3];
    if (!world || !view || !captured_camera_inverse_valid) return 0;
    delta[0] = world[0] - m[12];
    delta[1] = world[1] - m[13];
    delta[2] = world[2] - m[14];
    view[0] = delta[0] * m[0] + delta[1] * m[1] + delta[2] * m[2];
    view[1] = delta[0] * m[4] + delta[1] * m[5] + delta[2] * m[6];
    view[2] = delta[0] * m[8] + delta[1] * m[9] + delta[2] * m[10];
    return sane_probe_float(view[0]) && sane_probe_float(view[1]) &&
           sane_probe_float(view[2]);
}

static void draw_room_collision_d3d8(IDirect3DDevice8 *device)
{
    static int logged;
    int i;
    if (!device || !room_collision_debug_any()) return;
    for (i = 0; i < room_collision_triangle_count; i++) {
        room_collision_triangle_t *tri = &room_collision_triangles[i];
        float screen[3][2];
        int vertex;
        int valid = 1;
        DWORD color = 0xffffffffu;
        if (tri->mesh_index >= 0 &&
            tri->mesh_index < room_collision_mesh_count) {
            color = room_collision_meshes[tri->mesh_index].color;
        }
        for (vertex = 0; vertex < 3; vertex++) {
            float view[3];
            float draw[3];
            if (!room_collision_world_to_view_point(tri->v[vertex], view) ||
                !body_collider_draw_view_point(view, draw) ||
                !project_d3d8_view_point(device, draw,
                                         &screen[vertex][0],
                                         &screen[vertex][1])) {
                valid = 0;
                break;
            }
        }
        if (!valid) continue;
        draw_d3d8_debug_line(device, screen[0][0], screen[0][1],
                             screen[1][0], screen[1][1], color);
        draw_d3d8_debug_line(device, screen[1][0], screen[1][1],
                             screen[2][0], screen[2][1], color);
        draw_d3d8_debug_line(device, screen[2][0], screen[2][1],
                             screen[0][0], screen[0][1], color);
    }
    if (!logged) {
        logged = 1;
        log_line("room collision debug draw d3d8 active meshes=%d triangles=%d note=\"each selected room object uses a stable distinct wireframe color\"",
                 room_collision_mesh_count, room_collision_triangle_count);
    }
}

static void draw_body_chain_colliders_d3d8(IDirect3DDevice8 *device)
{
    DWORD old_z = 0, old_lighting = 0, old_fog = 0;
    DWORD old_zwrite = 0, old_alpha_blend = 0, old_alpha_test = 0;
    DWORD old_src_blend = 0, old_dest_blend = 0, old_cull = 0;
    DWORD old_colorop = 0, old_colorarg1 = 0, old_colorarg2 = 0;
    DWORD old_alphaop = 0, old_alphaarg1 = 0, old_alphaarg2 = 0;
    DWORD old_vertex_shader = 0;
    IDirect3DBaseTexture8 *old_texture = NULL;
    int person_index;
    static int d3d_draw_logged;
    static int d3d_projection_fail_logged;
    int draw_body = body_chain_collider_cfg.enabled &&
                    body_chain_collider_cfg.debug_draw;
    int draw_addon = addon_sidecar_collision_debug_any();
    int draw_room = room_collision_debug_any();
    if (!device || (!draw_body && !draw_addon && !draw_room)) {
        return;
    }
    if (!body_chain_gdi_debug_mode) {
        IDirect3DDevice8_GetRenderState(device, D3DRS_ZENABLE, &old_z);
        IDirect3DDevice8_GetRenderState(device, D3DRS_ZWRITEENABLE, &old_zwrite);
        IDirect3DDevice8_GetRenderState(device, D3DRS_LIGHTING, &old_lighting);
        IDirect3DDevice8_GetRenderState(device, D3DRS_FOGENABLE, &old_fog);
        IDirect3DDevice8_GetRenderState(device, D3DRS_ALPHABLENDENABLE, &old_alpha_blend);
        IDirect3DDevice8_GetRenderState(device, D3DRS_ALPHATESTENABLE, &old_alpha_test);
        IDirect3DDevice8_GetRenderState(device, D3DRS_SRCBLEND, &old_src_blend);
        IDirect3DDevice8_GetRenderState(device, D3DRS_DESTBLEND, &old_dest_blend);
        IDirect3DDevice8_GetRenderState(device, D3DRS_CULLMODE, &old_cull);
        IDirect3DDevice8_GetVertexShader(device, &old_vertex_shader);
        IDirect3DDevice8_GetTexture(device, 0, &old_texture);
        IDirect3DDevice8_GetTextureStageState(device, 0, D3DTSS_COLOROP, &old_colorop);
        IDirect3DDevice8_GetTextureStageState(device, 0, D3DTSS_COLORARG1, &old_colorarg1);
        IDirect3DDevice8_GetTextureStageState(device, 0, D3DTSS_COLORARG2, &old_colorarg2);
        IDirect3DDevice8_GetTextureStageState(device, 0, D3DTSS_ALPHAOP, &old_alphaop);
        IDirect3DDevice8_GetTextureStageState(device, 0, D3DTSS_ALPHAARG1, &old_alphaarg1);
        IDirect3DDevice8_GetTextureStageState(device, 0, D3DTSS_ALPHAARG2, &old_alphaarg2);
        IDirect3DDevice8_SetTexture(device, 0, NULL);
        IDirect3DDevice8_SetRenderState(device, D3DRS_ZENABLE, FALSE);
        IDirect3DDevice8_SetRenderState(device, D3DRS_ZWRITEENABLE, FALSE);
        IDirect3DDevice8_SetRenderState(device, D3DRS_LIGHTING, FALSE);
        IDirect3DDevice8_SetRenderState(device, D3DRS_FOGENABLE, FALSE);
        IDirect3DDevice8_SetRenderState(device, D3DRS_ALPHABLENDENABLE, FALSE);
        IDirect3DDevice8_SetRenderState(device, D3DRS_ALPHATESTENABLE, FALSE);
        IDirect3DDevice8_SetRenderState(device, D3DRS_CULLMODE, D3DCULL_NONE);
        IDirect3DDevice8_SetTextureStageState(device, 0, D3DTSS_COLOROP, D3DTOP_SELECTARG2);
        IDirect3DDevice8_SetTextureStageState(device, 0, D3DTSS_COLORARG2, D3DTA_DIFFUSE);
        IDirect3DDevice8_SetTextureStageState(device, 0, D3DTSS_ALPHAOP, D3DTOP_SELECTARG2);
        IDirect3DDevice8_SetTextureStageState(device, 0, D3DTSS_ALPHAARG2, D3DTA_DIFFUSE);
        IDirect3DDevice8_SetVertexShader(device, D3DFVF_XYZRHW | D3DFVF_DIFFUSE);
    }

    if (draw_room) draw_room_collision_d3d8(device);

    if (draw_body) for (person_index = 0; person_index < 4; person_index++) {
        body_chain_collider_person_state_t *state =
            &body_chain_collider_states[person_index];
        body_chain_person_state_t *chain =
            &body_chain_person_states[person_index];
        float p[BODY_COLLIDER_NODE_COUNT][2];
        float radius_px[BODY_COLLIDER_NODE_COUNT];
        int draw_valid[BODY_COLLIDER_NODE_COUNT];
        float chain_p[4][2];
        float chain_radius_px[4];
        int chain_ready = 0;
        int i;
        body_profile_set_active_person_config(person_index);
        if (!body_chain_collider_debug_person_active(person_index)) {
            body_profile_set_active_person_config(-1);
            continue;
        }
        for (i = 0; i < BODY_COLLIDER_NODE_COUNT; i++) {
            float view[3];
            float draw_point[3];
            float radius = body_chain_collider_visual_radius_for_node(i);
            p[i][0] = 0.0f;
            p[i][1] = 0.0f;
            radius_px[i] = 0.0f;
            draw_valid[i] = 0;
            if (!state->valid[i]) {
                continue;
            }
            if (!body_chain_collider_local_to_view(
                    state, state->local_position[i], view) ||
                !body_collider_draw_view_point(view, draw_point)) {
                continue;
            }
            if (!project_d3d8_view_point(device, draw_point,
                                         &p[i][0], &p[i][1])) {
                if (!d3d_projection_fail_logged) {
                    d3d_projection_fail_logged = 1;
                    log_line("body-chain-colliders draw d3d8 projection-failed person_index=%d node_index=%d stored=(%.5f,%.5f,%.5f) draw=(%.5f,%.5f,%.5f)",
                             person_index,
                             i,
                             view[0],
                             view[1],
                             view[2],
                             draw_point[0],
                             draw_point[1],
                             draw_point[2]);
                }
                continue;
            }
            if (!body_chain_debug_projected_radius_d3d8(
                    device, state, state->local_position[i],
                    p[i][0], p[i][1], radius, &radius_px[i])) {
                continue;
            }
            radius_px[i] = physx_clampf(radius_px[i], 0.25f, 320.0f);
            draw_valid[i] = 1;
        }
        if (!draw_valid[BODY_COLLIDER_ROOT]) {
            body_profile_set_active_person_config(-1);
            continue;
        }
        if (state->basis_valid &&
            body_chain_collider_cfg.penis_collision_enabled) {
            float chain_local[4][3];
            DWORD draw_now = GetTickCount();
            const char *chain_source = "none";
            int chain_points_from_engine = 0;
            int j;
            if (state->chain_points_ready &&
                state->chain_points_update_tick &&
                draw_now - state->chain_points_update_tick <=
                    BODY_CHAIN_ENGINE_POINT_STALE_MS) {
                for (j = 0; j < 4; j++) {
                    if (!state->chain_point_valid[j]) break;
                    chain_local[j][0] = state->chain_local_point[j][0];
                    chain_local[j][1] = state->chain_local_point[j][1];
                    chain_local[j][2] = state->chain_local_point[j][2];
                }
                chain_ready = j == 4;
                if (chain_ready) {
                    chain_source = "state-chain-points";
                    chain_points_from_engine = state->chain_points_fresh ? 1 : 2;
                }
            } else if (chain->initialized) {
                chain_ready = body_chain_collision_points_local(
                    state, chain, chain_local,
                    &chain_points_from_engine, draw_now);
                if (chain_ready) {
                    chain_source =
                        chain_points_from_engine == 1 ?
                            "collision-engine-fresh" :
                        chain_points_from_engine == 2 ?
                            "collision-engine-held" :
                            "collision-simulated";
                }
            }
            if (chain_ready) {
                for (j = 0; j < 4; j++) {
                    float view[3];
                    float draw_point[3];
                    if (!body_chain_collider_local_to_view(
                            state, chain_local[j], view) ||
                        !body_collider_draw_view_point(view, draw_point) ||
                        !project_d3d8_view_point(device, draw_point,
                                                 &chain_p[j][0],
                                                 &chain_p[j][1])) {
                        chain_ready = 0;
                        break;
                    }
                    if (!body_chain_debug_projected_radius_d3d8(
                            device, state, chain_local[j],
                            chain_p[j][0], chain_p[j][1],
                            body_chain_collider_cfg.chain_radius,
                            &chain_radius_px[j])) {
                        chain_ready = 0;
                        break;
                    }
                    chain_radius_px[j] =
                        physx_clampf(chain_radius_px[j], 0.25f, 320.0f);
                }
            }
            if (chain_ready &&
                config_hot_reload_tick &&
                draw_now - config_hot_reload_tick <= 6000u &&
                (!d3d_chain_draw_log_tick[person_index] ||
                 draw_now - d3d_chain_draw_log_tick[person_index] >= 750u)) {
                d3d_chain_draw_log_tick[person_index] = draw_now;
                log_line("body-chain-colliders draw-chain d3d8 person_index=%d source=\"%s\" engine_points=%d screen01=(%.1f,%.1f) screenEnd=(%.1f,%.1f) root_screen=(%.1f,%.1f) camera_version=%ld note=\"hot-reload draw diagnostic only; no collision state changed\"",
                         person_index + 1,
                         chain_source,
                         chain_points_from_engine,
                         chain_p[0][0],
                         chain_p[0][1],
                         chain_p[3][0],
                         chain_p[3][1],
                         p[BODY_COLLIDER_ROOT][0],
                         p[BODY_COLLIDER_ROOT][1],
                         captured_camera_version);
            }
        }
        if (!d3d_draw_logged) {
            d3d_draw_logged = 1;
            log_line("body-chain-colliders draw d3d8 active person_index=%d root_screen=(%.1f,%.1f) root_radius_px=%.1f",
                     person_index,
                     p[BODY_COLLIDER_ROOT][0],
                     p[BODY_COLLIDER_ROOT][1],
                     radius_px[BODY_COLLIDER_ROOT]);
        }
        if (state->stomach_points_ready &&
            draw_valid[BODY_COLLIDER_STOMACH_01] &&
            draw_valid[BODY_COLLIDER_STOMACH_02]) {
            float axes0[3];
            float axes1[3];
            float major[2];
            float minor[2];
            draw_d3d8_debug_line(device,
                p[BODY_COLLIDER_STOMACH_01][0],
                p[BODY_COLLIDER_STOMACH_01][1],
                p[BODY_COLLIDER_STOMACH_02][0],
                p[BODY_COLLIDER_STOMACH_02][1],
                0xff80ff40);
            body_chain_collider_visual_stomach_radius_axes(0, axes0);
            body_chain_collider_visual_stomach_radius_axes(1, axes1);
            if (body_chain_debug_projected_ellipse_d3d8(
                    device, state,
                    state->local_position[BODY_COLLIDER_STOMACH_01],
                    axes0,
                    p[BODY_COLLIDER_STOMACH_01][0],
                    p[BODY_COLLIDER_STOMACH_01][1],
                    major, minor)) {
                draw_d3d8_debug_ellipse(
                    device, p[BODY_COLLIDER_STOMACH_01][0],
                    p[BODY_COLLIDER_STOMACH_01][1],
                    major, minor, 0xff80ff40);
            } else {
                draw_d3d8_debug_circle(device, p[BODY_COLLIDER_STOMACH_01][0],
                                       p[BODY_COLLIDER_STOMACH_01][1],
                                       radius_px[BODY_COLLIDER_STOMACH_01],
                                       0xff80ff40);
            }
            if (body_chain_debug_projected_ellipse_d3d8(
                    device, state,
                    state->local_position[BODY_COLLIDER_STOMACH_02],
                    axes1,
                    p[BODY_COLLIDER_STOMACH_02][0],
                    p[BODY_COLLIDER_STOMACH_02][1],
                    major, minor)) {
                draw_d3d8_debug_ellipse(
                    device, p[BODY_COLLIDER_STOMACH_02][0],
                    p[BODY_COLLIDER_STOMACH_02][1],
                    major, minor, 0xff80ff40);
            } else {
            draw_d3d8_debug_circle(device, p[BODY_COLLIDER_STOMACH_02][0],
                                   p[BODY_COLLIDER_STOMACH_02][1],
                                   radius_px[BODY_COLLIDER_STOMACH_02],
                                   0xff80ff40);
            }
        }
        for (i = 0; i < BODY_COLLIDER_EXTRA_EDGE_COUNT; i++) {
            const body_collider_extra_edge_def_t *edge =
                &body_collider_extra_edges[i];
            if (!state->valid[edge->start_node] ||
                !state->valid[edge->end_node] ||
                !draw_valid[edge->start_node] ||
                !draw_valid[edge->end_node]) {
                continue;
            }
            draw_d3d8_debug_tapered_capsule(
                device,
                p[edge->start_node][0], p[edge->start_node][1],
                radius_px[edge->start_node],
                p[edge->end_node][0], p[edge->end_node][1],
                radius_px[edge->end_node], edge->d3d_color);
        }
        if (draw_valid[BODY_COLLIDER_HIP_L] &&
            draw_valid[BODY_COLLIDER_THIGH_L]) {
            draw_d3d8_debug_tapered_capsule(
                device,
                p[BODY_COLLIDER_HIP_L][0], p[BODY_COLLIDER_HIP_L][1],
                radius_px[BODY_COLLIDER_HIP_L],
                p[BODY_COLLIDER_THIGH_L][0], p[BODY_COLLIDER_THIGH_L][1],
                radius_px[BODY_COLLIDER_THIGH_L], 0xff20ffff);
        }
        if (draw_valid[BODY_COLLIDER_THIGH_L] &&
            draw_valid[BODY_COLLIDER_KNEE_L]) {
            draw_d3d8_debug_tapered_capsule(
                device,
                p[BODY_COLLIDER_THIGH_L][0], p[BODY_COLLIDER_THIGH_L][1],
                radius_px[BODY_COLLIDER_THIGH_L],
                p[BODY_COLLIDER_KNEE_L][0], p[BODY_COLLIDER_KNEE_L][1],
                radius_px[BODY_COLLIDER_KNEE_L], 0xff20ffff);
        }
        if (draw_valid[BODY_COLLIDER_HIP_R] &&
            draw_valid[BODY_COLLIDER_THIGH_R]) {
            draw_d3d8_debug_tapered_capsule(
                device,
                p[BODY_COLLIDER_HIP_R][0], p[BODY_COLLIDER_HIP_R][1],
                radius_px[BODY_COLLIDER_HIP_R],
                p[BODY_COLLIDER_THIGH_R][0], p[BODY_COLLIDER_THIGH_R][1],
                radius_px[BODY_COLLIDER_THIGH_R], 0xff20ffff);
        }
        if (draw_valid[BODY_COLLIDER_THIGH_R] &&
            draw_valid[BODY_COLLIDER_KNEE_R]) {
            draw_d3d8_debug_tapered_capsule(
                device,
                p[BODY_COLLIDER_THIGH_R][0], p[BODY_COLLIDER_THIGH_R][1],
                radius_px[BODY_COLLIDER_THIGH_R],
                p[BODY_COLLIDER_KNEE_R][0], p[BODY_COLLIDER_KNEE_R][1],
                radius_px[BODY_COLLIDER_KNEE_R], 0xff20ffff);
        }
        if (draw_valid[BODY_COLLIDER_HIP_L])
            draw_d3d8_debug_circle(device, p[BODY_COLLIDER_HIP_L][0],
                                   p[BODY_COLLIDER_HIP_L][1],
                                   radius_px[BODY_COLLIDER_HIP_L], 0xff20ffff);
        if (draw_valid[BODY_COLLIDER_HIP_R])
            draw_d3d8_debug_circle(device, p[BODY_COLLIDER_HIP_R][0],
                                   p[BODY_COLLIDER_HIP_R][1],
                                   radius_px[BODY_COLLIDER_HIP_R], 0xff20ffff);
        if (draw_valid[BODY_COLLIDER_THIGH_L])
            draw_d3d8_debug_circle(device, p[BODY_COLLIDER_THIGH_L][0],
                                   p[BODY_COLLIDER_THIGH_L][1],
                                   radius_px[BODY_COLLIDER_THIGH_L], 0xff20ffff);
        if (draw_valid[BODY_COLLIDER_THIGH_R])
            draw_d3d8_debug_circle(device, p[BODY_COLLIDER_THIGH_R][0],
                                   p[BODY_COLLIDER_THIGH_R][1],
                                   radius_px[BODY_COLLIDER_THIGH_R], 0xff20ffff);
        if (draw_valid[BODY_COLLIDER_KNEE_L])
            draw_d3d8_debug_circle(device, p[BODY_COLLIDER_KNEE_L][0],
                                   p[BODY_COLLIDER_KNEE_L][1],
                                   radius_px[BODY_COLLIDER_KNEE_L], 0xff20ffff);
        if (draw_valid[BODY_COLLIDER_KNEE_R])
            draw_d3d8_debug_circle(device, p[BODY_COLLIDER_KNEE_R][0],
                                   p[BODY_COLLIDER_KNEE_R][1],
                                   radius_px[BODY_COLLIDER_KNEE_R], 0xff20ffff);
        for (i = 0; i < BODY_COLLIDER_NODE_COUNT; i++) {
            float axes[3];
            float major[2];
            float minor[2];
            DWORD color;
            if (!state->valid[i]) continue;
            if (i >= BODY_COLLIDER_TESTICLES_01 &&
                i <= BODY_COLLIDER_TESTICLES_MID) {
                continue;
            }
            if (i <= BODY_COLLIDER_TESTICLES_MID &&
                !body_chain_collider_node_radius_is_oval(i)) {
                continue;
            }
            if (!draw_valid[i]) continue;
            body_chain_collider_visual_radius_axes_for_node(i, axes);
            color = body_chain_collider_node_color_d3d(i);
            if (body_chain_debug_projected_ellipse_d3d8(
                    device, state, state->local_position[i], axes,
                    p[i][0], p[i][1], major, minor)) {
                draw_d3d8_debug_ellipse(device, p[i][0], p[i][1],
                                        major, minor, color);
            } else {
                draw_d3d8_debug_circle(device, p[i][0], p[i][1],
                                       radius_px[i], color);
            }
        }
        if (chain_ready && body_chain_collider_cfg.penis_collision_enabled) {
            for (i = 0; i < 3; i++) {
                float r0 = chain_radius_px[i];
                float r1 = chain_radius_px[i + 1];
                float radius = r0 > r1 ? r0 : r1;
                draw_d3d8_debug_capsule(device,
                                        chain_p[i][0], chain_p[i][1],
                                        chain_p[i + 1][0], chain_p[i + 1][1],
                                        radius, 0xffffa000);
            }
        }
        if (body_chain_collider_cfg.testicle_collision_enabled) {
            for (i = BODY_COLLIDER_TESTICLES_01;
                 i <= BODY_COLLIDER_TESTICLES_02;
                 i++) {
                float axes[3];
                float major[2];
                float minor[2];
                if (!draw_valid[i]) continue;
                body_chain_collider_visual_radius_axes_for_node(i, axes);
                if (body_chain_debug_projected_ellipse_d3d8(
                        device, state, state->local_position[i], axes,
                        p[i][0], p[i][1], major, minor)) {
                    draw_d3d8_debug_ellipse(device, p[i][0], p[i][1],
                                            major, minor, 0xffff40ff);
                } else {
                    draw_d3d8_debug_circle(device, p[i][0], p[i][1],
                                           radius_px[i], 0xffff40ff);
                }
            }
            if (draw_valid[BODY_COLLIDER_TESTICLES_01] &&
                draw_valid[BODY_COLLIDER_TESTICLES_02]) {
                draw_d3d8_debug_tapered_capsule(
                    device,
                    p[BODY_COLLIDER_TESTICLES_01][0],
                    p[BODY_COLLIDER_TESTICLES_01][1],
                    radius_px[BODY_COLLIDER_TESTICLES_01],
                    p[BODY_COLLIDER_TESTICLES_02][0],
                    p[BODY_COLLIDER_TESTICLES_02][1],
                    radius_px[BODY_COLLIDER_TESTICLES_02],
                    0xffff40ff);
            }
        }
        body_profile_set_active_person_config(-1);
    }
    body_profile_set_active_person_config(-1);
    if (draw_addon) {
        draw_addon_sidecar_colliders_d3d8(device);
    }

    if (!body_chain_gdi_debug_mode) {
        IDirect3DDevice8_SetVertexShader(device, old_vertex_shader);
        IDirect3DDevice8_SetRenderState(device, D3DRS_ZENABLE, old_z);
        IDirect3DDevice8_SetRenderState(device, D3DRS_ZWRITEENABLE, old_zwrite);
        IDirect3DDevice8_SetRenderState(device, D3DRS_LIGHTING, old_lighting);
        IDirect3DDevice8_SetRenderState(device, D3DRS_FOGENABLE, old_fog);
        IDirect3DDevice8_SetRenderState(device, D3DRS_ALPHABLENDENABLE, old_alpha_blend);
        IDirect3DDevice8_SetRenderState(device, D3DRS_ALPHATESTENABLE, old_alpha_test);
        IDirect3DDevice8_SetRenderState(device, D3DRS_SRCBLEND, old_src_blend);
        IDirect3DDevice8_SetRenderState(device, D3DRS_DESTBLEND, old_dest_blend);
        IDirect3DDevice8_SetRenderState(device, D3DRS_CULLMODE, old_cull);
        IDirect3DDevice8_SetTextureStageState(device, 0, D3DTSS_COLOROP, old_colorop);
        IDirect3DDevice8_SetTextureStageState(device, 0, D3DTSS_COLORARG1, old_colorarg1);
        IDirect3DDevice8_SetTextureStageState(device, 0, D3DTSS_COLORARG2, old_colorarg2);
        IDirect3DDevice8_SetTextureStageState(device, 0, D3DTSS_ALPHAOP, old_alphaop);
        IDirect3DDevice8_SetTextureStageState(device, 0, D3DTSS_ALPHAARG1, old_alphaarg1);
        IDirect3DDevice8_SetTextureStageState(device, 0, D3DTSS_ALPHAARG2, old_alphaarg2);
        IDirect3DDevice8_SetTexture(device, 0, old_texture);
        if (old_texture) IDirect3DBaseTexture8_Release(old_texture);
    }
}

static void draw_body_chain_colliders_gdi_d3d8(IDirect3DDevice8 *device,
                                               HWND hwnd)
{
    HDC hdc;
    int saved;
    static int gdi_draw_logged;
    static int gdi_draw_failed_logged;
    if (!device || !hwnd ||
        !((body_chain_collider_cfg.enabled &&
           body_chain_collider_cfg.debug_draw) ||
          addon_sidecar_collision_debug_any() ||
          room_collision_debug_any())) {
        return;
    }
    hdc = GetDC(hwnd);
    if (!hdc) {
        if (!gdi_draw_failed_logged) {
            gdi_draw_failed_logged = 1;
            log_line("body-chain-colliders draw gdi-hook5 failed hwnd=%p reason=\"GetDC failed\"",
                     hwnd);
        }
        return;
    }
    saved = SaveDC(hdc);
    SetBkMode(hdc, TRANSPARENT);
    body_chain_gdi_debug_hdc = hdc;
    body_chain_gdi_debug_mode = 1;
    body_chain_gdi_debug_primitives = 0;
    draw_body_chain_colliders_d3d8(device);
    body_chain_gdi_debug_mode = 0;
    body_chain_gdi_debug_hdc = NULL;
    if (!gdi_draw_logged && body_chain_gdi_debug_primitives > 0) {
        gdi_draw_logged = 1;
        log_line("body-chain-colliders draw gdi-hook5 active hwnd=%p primitives=%d note=\"window-DC overlay drawn after Hook5 Present; no extra D3D scene is opened\"",
                 hwnd, body_chain_gdi_debug_primitives);
    }
    if (saved) RestoreDC(hdc, saved);
    ReleaseDC(hwnd, hdc);
}

static int body_chain_hook5_overlay_prepare(HWND owner, int width, int height)
{
    HDC screen_dc;
    if (!owner || width <= 0 || height <= 0) return 0;
    if (!body_chain_hook5_overlay_hwnd) {
        body_chain_hook5_overlay_hwnd = CreateWindowExA(
            WS_EX_LAYERED | WS_EX_TRANSPARENT | WS_EX_TOPMOST |
            WS_EX_TOOLWINDOW | WS_EX_NOACTIVATE,
            "STATIC", "NC-TK17-PhysX-ColliderOverlay",
            WS_POPUP, 0, 0, width, height,
            NULL, NULL, self_module, NULL);
        if (!body_chain_hook5_overlay_hwnd) return 0;
    }
    if (body_chain_hook5_overlay_dc &&
        body_chain_hook5_overlay_bitmap &&
        body_chain_hook5_overlay_w == width &&
        body_chain_hook5_overlay_h == height) {
        return 1;
    }
    if (body_chain_hook5_overlay_dc) {
        if (body_chain_hook5_overlay_old_bitmap) {
            SelectObject(body_chain_hook5_overlay_dc,
                         body_chain_hook5_overlay_old_bitmap);
        }
        if (body_chain_hook5_overlay_bitmap) {
            DeleteObject(body_chain_hook5_overlay_bitmap);
        }
        DeleteDC(body_chain_hook5_overlay_dc);
        body_chain_hook5_overlay_dc = NULL;
        body_chain_hook5_overlay_bitmap = NULL;
        body_chain_hook5_overlay_old_bitmap = NULL;
    }
    screen_dc = GetDC(NULL);
    if (!screen_dc) return 0;
    body_chain_hook5_overlay_dc = CreateCompatibleDC(screen_dc);
    body_chain_hook5_overlay_bitmap =
        CreateCompatibleBitmap(screen_dc, width, height);
    ReleaseDC(NULL, screen_dc);
    if (!body_chain_hook5_overlay_dc ||
        !body_chain_hook5_overlay_bitmap) {
        if (body_chain_hook5_overlay_dc) {
            DeleteDC(body_chain_hook5_overlay_dc);
            body_chain_hook5_overlay_dc = NULL;
        }
        if (body_chain_hook5_overlay_bitmap) {
            DeleteObject(body_chain_hook5_overlay_bitmap);
            body_chain_hook5_overlay_bitmap = NULL;
        }
        return 0;
    }
    body_chain_hook5_overlay_old_bitmap =
        SelectObject(body_chain_hook5_overlay_dc,
                     body_chain_hook5_overlay_bitmap);
    body_chain_hook5_overlay_w = width;
    body_chain_hook5_overlay_h = height;
    return 1;
}

static void draw_body_chain_colliders_layered_hook5_d3d8(
    IDirect3DDevice8 *device,
    HWND owner)
{
    RECT client;
    POINT screen_pos = { 0, 0 };
    SIZE size;
    POINT src = { 0, 0 };
    HDC screen_dc;
    RECT fill_rect;
    HBRUSH black_brush;
    int saved;
    static int layered_logged;
    static int layered_failed_logged;

    if (!device || !owner ||
        !((body_chain_collider_cfg.enabled &&
           body_chain_collider_cfg.debug_draw) ||
          addon_sidecar_collision_debug_any() ||
          room_collision_debug_any())) {
        if (body_chain_hook5_overlay_hwnd) {
            ShowWindow(body_chain_hook5_overlay_hwnd, SW_HIDE);
        }
        return;
    }
    if (!GetClientRect(owner, &client)) return;
    size.cx = client.right - client.left;
    size.cy = client.bottom - client.top;
    if (size.cx <= 0 || size.cy <= 0) return;
    ClientToScreen(owner, &screen_pos);
    if (!body_chain_hook5_overlay_prepare(owner, size.cx, size.cy)) {
        if (!layered_failed_logged) {
            layered_failed_logged = 1;
            log_line("body-chain-colliders draw layered-hook5 failed hwnd=%p size=%dx%d reason=\"overlay window or memory bitmap could not be created\"",
                     owner, size.cx, size.cy);
        }
        return;
    }

    fill_rect.left = 0;
    fill_rect.top = 0;
    fill_rect.right = size.cx;
    fill_rect.bottom = size.cy;
    black_brush = CreateSolidBrush(RGB(0, 0, 0));
    if (black_brush) {
        FillRect(body_chain_hook5_overlay_dc, &fill_rect, black_brush);
        DeleteObject(black_brush);
    }
    saved = SaveDC(body_chain_hook5_overlay_dc);
    SetBkMode(body_chain_hook5_overlay_dc, TRANSPARENT);
    body_chain_gdi_debug_hdc = body_chain_hook5_overlay_dc;
    body_chain_gdi_debug_mode = 1;
    body_chain_gdi_debug_primitives = 0;
    draw_body_chain_colliders_d3d8(device);
    body_chain_gdi_debug_mode = 0;
    body_chain_gdi_debug_hdc = NULL;
    if (saved) RestoreDC(body_chain_hook5_overlay_dc, saved);

    if (body_chain_gdi_debug_primitives <= 0) {
        ShowWindow(body_chain_hook5_overlay_hwnd, SW_HIDE);
        return;
    }

    screen_dc = GetDC(NULL);
    if (screen_dc) {
        UpdateLayeredWindow(body_chain_hook5_overlay_hwnd,
                            screen_dc,
                            &screen_pos,
                            &size,
                            body_chain_hook5_overlay_dc,
                            &src,
                            RGB(0, 0, 0),
                            NULL,
                            ULW_COLORKEY);
        ReleaseDC(NULL, screen_dc);
        SetWindowPos(body_chain_hook5_overlay_hwnd,
                     HWND_TOPMOST,
                     screen_pos.x,
                     screen_pos.y,
                     size.cx,
                     size.cy,
                     SWP_NOACTIVATE | SWP_SHOWWINDOW);
        if (!layered_logged) {
            layered_logged = 1;
            log_line("body-chain-colliders draw layered-hook5 active hwnd=%p overlay=%p size=%dx%d primitives=%d note=\"transparent click-through overlay tracks TK17 client area; avoids window-DC flicker after Hook5 Present\"",
                     owner,
                     body_chain_hook5_overlay_hwnd,
                     size.cx,
                     size.cy,
                     body_chain_gdi_debug_primitives);
        }
    }
}

static void destroy_body_chain_hook5_overlay(void)
{
    if (body_chain_hook5_overlay_dc) {
        if (body_chain_hook5_overlay_old_bitmap) {
            SelectObject(body_chain_hook5_overlay_dc,
                         body_chain_hook5_overlay_old_bitmap);
        }
        DeleteDC(body_chain_hook5_overlay_dc);
    }
    if (body_chain_hook5_overlay_bitmap) {
        DeleteObject(body_chain_hook5_overlay_bitmap);
    }
    if (body_chain_hook5_overlay_hwnd) {
        DestroyWindow(body_chain_hook5_overlay_hwnd);
    }
    body_chain_hook5_overlay_hwnd = NULL;
    body_chain_hook5_overlay_dc = NULL;
    body_chain_hook5_overlay_bitmap = NULL;
    body_chain_hook5_overlay_old_bitmap = NULL;
    body_chain_hook5_overlay_w = 0;
    body_chain_hook5_overlay_h = 0;
}

static int project_opengl_view_point(const GLfloat projection[16],
                                     const GLint viewport[4],
                                     const float point[3],
                                     float *screen_x,
                                     float *screen_y)
{
    float clip_x, clip_y, clip_w;
    clip_x = projection[0] * point[0] + projection[4] * point[1] +
             projection[8] * point[2] + projection[12];
    clip_y = projection[1] * point[0] + projection[5] * point[1] +
             projection[9] * point[2] + projection[13];
    clip_w = projection[3] * point[0] + projection[7] * point[1] +
             projection[11] * point[2] + projection[15];
    if (!sane_probe_float(clip_w) || clip_w <= 0.00001f) return 0;
    *screen_x = viewport[0] + (1.0f + clip_x / clip_w) * viewport[2] * 0.5f;
    *screen_y = viewport[1] + (1.0f - clip_y / clip_w) * viewport[3] * 0.5f;
    return sane_probe_float(*screen_x) && sane_probe_float(*screen_y);
}

static int body_chain_debug_projected_radius_opengl(
    const GLfloat projection[16],
    const GLint viewport[4],
    const body_chain_collider_person_state_t *state,
    const float local[3],
    float center_x,
    float center_y,
    float radius,
    float *radius_px)
{
    int axis;
    float max_px = 0.0f;
    if (!projection || !viewport || !state || !local ||
        !radius_px || radius <= 0.0f) return 0;
    for (axis = 0; axis < 3; axis++) {
        float edge_local[3] = { local[0], local[1], local[2] };
        float edge_view[3];
        float draw_edge[3];
        float edge_x, edge_y;
        float dx, dy, dist_px;
        edge_local[axis] += radius;
        if (!body_chain_collider_local_to_view(state, edge_local, edge_view) ||
            !body_collider_draw_view_point(edge_view, draw_edge) ||
            !project_opengl_view_point(projection, viewport, draw_edge,
                                       &edge_x, &edge_y)) {
            continue;
        }
        dx = edge_x - center_x;
        dy = edge_y - center_y;
        dist_px = (float)sqrt((double)(dx * dx + dy * dy));
        if (sane_probe_float(dist_px) && dist_px > max_px) {
            max_px = dist_px;
        }
    }
    if (!body_chain_debug_projected_radius_stable(max_px)) return 0;
    *radius_px = physx_clampf(max_px, 0.25f, 320.0f);
    return 1;
}

static int body_chain_debug_projected_ellipse_opengl(
    const GLfloat projection[16],
    const GLint viewport[4],
    const body_chain_collider_person_state_t *state,
    const float local[3],
    const float axes[3],
    float center_x,
    float center_y,
    float major[2],
    float minor[2])
{
    float cov_xx = 0.0f;
    float cov_xy = 0.0f;
    float cov_yy = 0.0f;
    int axis;
    int used = 0;
    if (!projection || !viewport || !state || !local || !axes ||
        !major || !minor) return 0;
    for (axis = 0; axis < 3; axis++) {
        float plus_local[3] = { local[0], local[1], local[2] };
        float minus_local[3] = { local[0], local[1], local[2] };
        float plus_view[3], minus_view[3];
        float draw_plus[3], draw_minus[3];
        float plus_x, plus_y, minus_x, minus_y;
        float step;
        float scale;
        float dx, dy;
        int plus_ok;
        int minus_ok;
        if (axes[axis] <= 0.0f) continue;
        step = physx_clampf(axes[axis] * 0.01f, 0.00025f, 0.002f);
        plus_local[axis] += step;
        minus_local[axis] -= step;
        plus_ok = body_chain_collider_local_to_view(state, plus_local,
                                                     plus_view) &&
                  body_collider_draw_view_point(plus_view, draw_plus) &&
                  project_opengl_view_point(projection, viewport, draw_plus,
                                            &plus_x, &plus_y);
        minus_ok = body_chain_collider_local_to_view(state, minus_local,
                                                      minus_view) &&
                   body_collider_draw_view_point(minus_view, draw_minus) &&
                   project_opengl_view_point(projection, viewport, draw_minus,
                                             &minus_x, &minus_y);
        if (plus_ok && minus_ok) {
            scale = axes[axis] / (2.0f * step);
            dx = (plus_x - minus_x) * scale;
            dy = (plus_y - minus_y) * scale;
        } else if (plus_ok) {
            scale = axes[axis] / step;
            dx = (plus_x - center_x) * scale;
            dy = (plus_y - center_y) * scale;
        } else if (minus_ok) {
            scale = axes[axis] / step;
            dx = (center_x - minus_x) * scale;
            dy = (center_y - minus_y) * scale;
        } else {
            continue;
        }
        if (!sane_probe_float(dx) || !sane_probe_float(dy)) continue;
        cov_xx += dx * dx;
        cov_xy += dx * dy;
        cov_yy += dy * dy;
        used = 1;
    }
    if (!used) return 0;
    return body_chain_debug_ellipse_from_cov(cov_xx, cov_xy, cov_yy,
                                            major, minor);
}

static void draw_opengl_debug_line(float x0, float y0,
                                   float x1, float y1,
                                   GLubyte r, GLubyte g, GLubyte b)
{
    physx_gl_color4ub(r, g, b, 255);
    physx_gl_begin(GL_LINES);
    physx_gl_vertex2f(x0, y0);
    physx_gl_vertex2f(x1, y1);
    physx_gl_end();
}

static void draw_room_collision_opengl(const GLfloat projection[16],
                                       const GLint viewport[4])
{
    static int logged;
    int i;
    if (!projection || !viewport || !room_collision_debug_any()) return;
    for (i = 0; i < room_collision_triangle_count; i++) {
        room_collision_triangle_t *tri = &room_collision_triangles[i];
        float screen[3][2];
        DWORD color = 0xffffffffu;
        unsigned char r, g, b;
        int vertex;
        int valid = 1;
        if (tri->mesh_index >= 0 &&
            tri->mesh_index < room_collision_mesh_count) {
            color = room_collision_meshes[tri->mesh_index].color;
        }
        addon_sidecar_debug_color_rgb(color, &r, &g, &b);
        for (vertex = 0; vertex < 3; vertex++) {
            float view[3], draw[3];
            if (!room_collision_world_to_view_point(tri->v[vertex], view) ||
                !body_collider_draw_view_point(view, draw) ||
                !project_opengl_view_point(projection, viewport, draw,
                                            &screen[vertex][0],
                                            &screen[vertex][1])) {
                valid = 0;
                break;
            }
        }
        if (!valid) continue;
        draw_opengl_debug_line(screen[0][0], screen[0][1],
                               screen[1][0], screen[1][1], r, g, b);
        draw_opengl_debug_line(screen[1][0], screen[1][1],
                               screen[2][0], screen[2][1], r, g, b);
        draw_opengl_debug_line(screen[2][0], screen[2][1],
                               screen[0][0], screen[0][1], r, g, b);
    }
    if (!logged) {
        logged = 1;
        log_line("room collision debug draw opengl active meshes=%d triangles=%d",
                 room_collision_mesh_count, room_collision_triangle_count);
    }
}

static void draw_opengl_debug_circle(float cx, float cy, float radius,
                                     GLubyte r, GLubyte g, GLubyte b)
{
    int i;
    physx_gl_color4ub(r, g, b, 255);
    physx_gl_begin(GL_LINE_LOOP);
    for (i = 0; i < 24; i++) {
        float a = (float)i / 24.0f * 6.28318530717958647692f;
        physx_gl_vertex2f(cx + (float)cos((double)a) * radius,
                         cy + (float)sin((double)a) * radius);
    }
    physx_gl_end();
}

static void draw_opengl_debug_ellipse(float cx, float cy,
                                      const float major[2],
                                      const float minor[2],
                                      GLubyte r, GLubyte g, GLubyte b)
{
    int i;
    if (!major || !minor) return;
    physx_gl_color4ub(r, g, b, 255);
    physx_gl_begin(GL_LINE_LOOP);
    for (i = 0; i < 32; i++) {
        float t = (float)i / 32.0f * 6.28318530717958647692f;
        float co = (float)cos((double)t);
        float si = (float)sin((double)t);
        physx_gl_vertex2f(cx + major[0] * co + minor[0] * si,
                          cy + major[1] * co + minor[1] * si);
    }
    physx_gl_end();
}

static void draw_opengl_debug_capsule(float x0, float y0,
                                      float x1, float y1,
                                      float radius,
                                      GLubyte r, GLubyte g, GLubyte b)
{
    float dx = x1 - x0;
    float dy = y1 - y0;
    float len = (float)sqrt((double)(dx * dx + dy * dy));
    float nx, ny;
    if (len < 0.001f) {
        draw_opengl_debug_circle(x0, y0, radius, r, g, b);
        return;
    }
    nx = -dy / len * radius;
    ny = dx / len * radius;
    draw_opengl_debug_line(x0 + nx, y0 + ny, x1 + nx, y1 + ny, r, g, b);
    draw_opengl_debug_line(x0 - nx, y0 - ny, x1 - nx, y1 - ny, r, g, b);
    draw_opengl_debug_circle(x0, y0, radius, r, g, b);
    draw_opengl_debug_circle(x1, y1, radius, r, g, b);
}

static void draw_opengl_debug_tapered_capsule(float x0, float y0, float r0,
                                              float x1, float y1, float r1,
                                              GLubyte r, GLubyte g, GLubyte b)
{
    float dx = x1 - x0;
    float dy = y1 - y0;
    float len = (float)sqrt((double)(dx * dx + dy * dy));
    float nx, ny;
    if (len < 0.001f) {
        draw_opengl_debug_circle(x0, y0, r0 > r1 ? r0 : r1, r, g, b);
        return;
    }
    nx = -dy / len;
    ny = dx / len;
    draw_opengl_debug_line(x0 + nx * r0, y0 + ny * r0,
                           x1 + nx * r1, y1 + ny * r1, r, g, b);
    draw_opengl_debug_line(x0 - nx * r0, y0 - ny * r0,
                           x1 - nx * r1, y1 - ny * r1, r, g, b);
    draw_opengl_debug_circle(x0, y0, r0, r, g, b);
    draw_opengl_debug_circle(x1, y1, r1, r, g, b);
}

static void draw_addon_sidecar_colliders_opengl(
    const GLfloat projection[16],
    const GLint viewport[4])
{
    int i, c;
    static int addon_gl_draw_logged;
    if (!projection || !viewport) return;
    for (i = 0; i < sidecar_count; i++) {
        physx_sidecar_t *sc = &sidecars[i];
        if (!sc->loaded || !sc->enabled) continue;
        for (c = 0; c < sc->chain_count; c++) {
            physx_chain_t *chain = &sc->chains[c];
            float local_points[32][3];
            float p[32][2];
            float radius_px[32];
            int point_count = 0;
            int person_index = -1;
            unsigned char r, g, b;
            int j;
            if (!chain->addon_chain ||
                !chain->collision_enabled ||
                !chain->collision_debug_draw ||
                chain->target_count <= 1) {
                continue;
            }
            if (!addon_chain_collision_points_body_local(
                    sc, chain, local_points, &point_count,
                    &person_index, GetTickCount())) {
                continue;
            }
            if (person_index < 0 || person_index >= 4) continue;
            for (j = 0; j < point_count; j++) {
                float view[3];
                float draw_point[3];
                if (!body_chain_collider_local_to_view(
                        &body_chain_collider_states[person_index],
                        local_points[j], view) ||
                    !body_collider_draw_view_point(view, draw_point) ||
                    !project_opengl_view_point(projection, viewport,
                                               draw_point,
                                               &p[j][0], &p[j][1]) ||
                    !body_chain_debug_projected_radius_opengl(
                        projection, viewport,
                        &body_chain_collider_states[person_index],
                        local_points[j], p[j][0], p[j][1],
                        chain->collision_radius, &radius_px[j])) {
                    point_count = 0;
                    break;
                }
                radius_px[j] = physx_clampf(radius_px[j], 0.25f, 320.0f);
            }
            if (point_count <= 1) continue;
            addon_sidecar_debug_color_rgb(chain->collision_debug_color,
                                          &r, &g, &b);
            for (j = 0; j + 1 < point_count; j++) {
                float radius = radius_px[j] > radius_px[j + 1] ?
                               radius_px[j] : radius_px[j + 1];
                draw_opengl_debug_capsule(p[j][0], p[j][1],
                                          p[j + 1][0], p[j + 1][1],
                                          radius, r, g, b);
            }
            if (!addon_gl_draw_logged) {
                addon_gl_draw_logged = 1;
                log_line("addon sidecar collision debug draw opengl active chain=\"%s\" person_index=%d points=%d radius=%.4f color=0x%08lx sidecar=\"%s\" note=\"sidecar collider visuals are independent from body collider debug draw\"",
                         chain->name,
                         person_index + 1,
                         point_count,
                         chain->collision_radius,
                         (unsigned long)chain->collision_debug_color,
                         sc->path);
            }
        }
    }
}

static void draw_body_chain_colliders_opengl(void)
{
    GLfloat projection[16];
    GLint viewport[4];
    GLint old_matrix_mode = GL_MODELVIEW;
    int person_index;
    static int gl_draw_logged;
    static int gl_projection_fail_logged;
    int draw_body = body_chain_collider_cfg.enabled &&
                    body_chain_collider_cfg.debug_draw;
    int draw_addon = addon_sidecar_collision_debug_any();
    int draw_room = room_collision_debug_any();
    if ((!draw_body && !draw_addon && !draw_room) ||
        !resolve_body_collider_gl_api()) {
        return;
    }
    physx_gl_get_floatv(GL_PROJECTION_MATRIX, projection);
    physx_gl_get_integerv(GL_VIEWPORT, viewport);
    physx_gl_get_integerv(GL_MATRIX_MODE, &old_matrix_mode);
    if (viewport[2] <= 0 || viewport[3] <= 0) return;

    physx_gl_push_attrib(GL_ALL_ATTRIB_BITS);
    physx_gl_disable(GL_DEPTH_TEST);
    physx_gl_disable(GL_TEXTURE_2D);
    physx_gl_disable(GL_LIGHTING);
    physx_gl_line_width(2.0f);
    physx_gl_matrix_mode(GL_PROJECTION);
    physx_gl_push_matrix();
    physx_gl_load_identity();
    physx_gl_ortho(0.0, viewport[2], viewport[3], 0.0, -1.0, 1.0);
    physx_gl_matrix_mode(GL_MODELVIEW);
    physx_gl_push_matrix();
    physx_gl_load_identity();

    if (draw_room) draw_room_collision_opengl(projection, viewport);

    if (draw_body) for (person_index = 0; person_index < 4; person_index++) {
        body_chain_collider_person_state_t *state =
            &body_chain_collider_states[person_index];
        body_chain_person_state_t *chain =
            &body_chain_person_states[person_index];
        float p[BODY_COLLIDER_NODE_COUNT][2];
        float radius_px[BODY_COLLIDER_NODE_COUNT];
        int draw_valid[BODY_COLLIDER_NODE_COUNT];
        float chain_p[4][2];
        float chain_radius_px[4];
        int chain_ready = 0;
        int i;
        body_profile_set_active_person_config(person_index);
        if (!body_chain_collider_debug_person_active(person_index)) {
            body_profile_set_active_person_config(-1);
            continue;
        }
        for (i = 0; i < BODY_COLLIDER_NODE_COUNT; i++) {
            float view[3];
            float draw_point[3];
            float radius = body_chain_collider_visual_radius_for_node(i);
            p[i][0] = 0.0f;
            p[i][1] = 0.0f;
            radius_px[i] = 0.0f;
            draw_valid[i] = 0;
            if (!state->valid[i]) {
                continue;
            }
            if (!body_chain_collider_local_to_view(
                    state, state->local_position[i], view) ||
                !body_collider_draw_view_point(view, draw_point)) {
                continue;
            }
            if (!project_opengl_view_point(projection, viewport,
                                           draw_point,
                                           &p[i][0], &p[i][1])) {
                if (!gl_projection_fail_logged) {
                    gl_projection_fail_logged = 1;
                    log_line("body-chain-colliders draw opengl projection-failed person_index=%d node_index=%d stored=(%.5f,%.5f,%.5f) draw=(%.5f,%.5f,%.5f)",
                             person_index,
                             i,
                             view[0],
                             view[1],
                             view[2],
                             draw_point[0],
                             draw_point[1],
                             draw_point[2]);
                }
                continue;
            }
            if (!body_chain_debug_projected_radius_opengl(
                    projection, viewport, state, state->local_position[i],
                    p[i][0], p[i][1], radius, &radius_px[i])) {
                continue;
            }
            radius_px[i] = physx_clampf(radius_px[i], 0.25f, 320.0f);
            draw_valid[i] = 1;
        }
        if (!draw_valid[BODY_COLLIDER_ROOT]) {
            body_profile_set_active_person_config(-1);
            continue;
        }
        if (state->basis_valid &&
            body_chain_collider_cfg.penis_collision_enabled) {
            float chain_local[4][3];
            DWORD draw_now = GetTickCount();
            const char *chain_source = "none";
            int chain_points_from_engine = 0;
            int j;
            if (state->chain_points_ready &&
                state->chain_points_update_tick &&
                draw_now - state->chain_points_update_tick <=
                    BODY_CHAIN_ENGINE_POINT_STALE_MS) {
                for (j = 0; j < 4; j++) {
                    if (!state->chain_point_valid[j]) break;
                    chain_local[j][0] = state->chain_local_point[j][0];
                    chain_local[j][1] = state->chain_local_point[j][1];
                    chain_local[j][2] = state->chain_local_point[j][2];
                }
                chain_ready = j == 4;
                if (chain_ready) {
                    chain_source = "state-chain-points";
                    chain_points_from_engine = state->chain_points_fresh ? 1 : 2;
                }
            } else if (chain->initialized) {
                chain_ready = body_chain_collision_points_local(
                    state, chain, chain_local,
                    &chain_points_from_engine, draw_now);
                if (chain_ready) {
                    chain_source =
                        chain_points_from_engine == 1 ?
                            "collision-engine-fresh" :
                        chain_points_from_engine == 2 ?
                            "collision-engine-held" :
                            "collision-simulated";
                }
            }
            if (chain_ready) {
                for (j = 0; j < 4; j++) {
                    float view[3];
                    float draw_point[3];
                    if (!body_chain_collider_local_to_view(
                            state, chain_local[j], view) ||
                        !body_collider_draw_view_point(view, draw_point) ||
                        !project_opengl_view_point(projection, viewport,
                                                   draw_point,
                                                   &chain_p[j][0],
                                                   &chain_p[j][1])) {
                        chain_ready = 0;
                        break;
                    }
                    if (!body_chain_debug_projected_radius_opengl(
                            projection, viewport, state, chain_local[j],
                            chain_p[j][0], chain_p[j][1],
                            body_chain_collider_cfg.chain_radius,
                            &chain_radius_px[j])) {
                        chain_ready = 0;
                        break;
                    }
                    chain_radius_px[j] =
                        physx_clampf(chain_radius_px[j], 0.25f, 320.0f);
                }
            }
            if (chain_ready &&
                config_hot_reload_tick &&
                draw_now - config_hot_reload_tick <= 6000u &&
                (!gl_chain_draw_log_tick[person_index] ||
                 draw_now - gl_chain_draw_log_tick[person_index] >= 750u)) {
                gl_chain_draw_log_tick[person_index] = draw_now;
                log_line("body-chain-colliders draw-chain opengl person_index=%d source=\"%s\" engine_points=%d screen01=(%.1f,%.1f) screenEnd=(%.1f,%.1f) root_screen=(%.1f,%.1f) camera_version=%ld note=\"hot-reload draw diagnostic only; no collision state changed\"",
                         person_index + 1,
                         chain_source,
                         chain_points_from_engine,
                         chain_p[0][0],
                         chain_p[0][1],
                         chain_p[3][0],
                         chain_p[3][1],
                         p[BODY_COLLIDER_ROOT][0],
                         p[BODY_COLLIDER_ROOT][1],
                         captured_camera_version);
            }
        }
        if (!gl_draw_logged) {
            gl_draw_logged = 1;
            log_line("body-chain-colliders draw opengl active person_index=%d root_screen=(%.1f,%.1f) root_radius_px=%.1f",
                     person_index,
                     p[BODY_COLLIDER_ROOT][0],
                     p[BODY_COLLIDER_ROOT][1],
                     radius_px[BODY_COLLIDER_ROOT]);
        }
        if (state->stomach_points_ready &&
            draw_valid[BODY_COLLIDER_STOMACH_01] &&
            draw_valid[BODY_COLLIDER_STOMACH_02]) {
            float axes0[3];
            float axes1[3];
            float major[2];
            float minor[2];
            draw_opengl_debug_line(
                p[BODY_COLLIDER_STOMACH_01][0],
                p[BODY_COLLIDER_STOMACH_01][1],
                p[BODY_COLLIDER_STOMACH_02][0],
                p[BODY_COLLIDER_STOMACH_02][1],
                128, 255, 64);
            body_chain_collider_visual_stomach_radius_axes(0, axes0);
            body_chain_collider_visual_stomach_radius_axes(1, axes1);
            if (body_chain_debug_projected_ellipse_opengl(
                    projection, viewport, state,
                    state->local_position[BODY_COLLIDER_STOMACH_01],
                    axes0,
                    p[BODY_COLLIDER_STOMACH_01][0],
                    p[BODY_COLLIDER_STOMACH_01][1],
                    major, minor)) {
                draw_opengl_debug_ellipse(p[BODY_COLLIDER_STOMACH_01][0],
                                          p[BODY_COLLIDER_STOMACH_01][1],
                                          major, minor, 128, 255, 64);
            } else {
                draw_opengl_debug_circle(p[BODY_COLLIDER_STOMACH_01][0],
                                         p[BODY_COLLIDER_STOMACH_01][1],
                                         radius_px[BODY_COLLIDER_STOMACH_01],
                                         128, 255, 64);
            }
            if (body_chain_debug_projected_ellipse_opengl(
                    projection, viewport, state,
                    state->local_position[BODY_COLLIDER_STOMACH_02],
                    axes1,
                    p[BODY_COLLIDER_STOMACH_02][0],
                    p[BODY_COLLIDER_STOMACH_02][1],
                    major, minor)) {
                draw_opengl_debug_ellipse(p[BODY_COLLIDER_STOMACH_02][0],
                                          p[BODY_COLLIDER_STOMACH_02][1],
                                          major, minor, 128, 255, 64);
            } else {
                draw_opengl_debug_circle(p[BODY_COLLIDER_STOMACH_02][0],
                                         p[BODY_COLLIDER_STOMACH_02][1],
                                         radius_px[BODY_COLLIDER_STOMACH_02],
                                         128, 255, 64);
            }
        }
        for (i = 0; i < BODY_COLLIDER_EXTRA_EDGE_COUNT; i++) {
            const body_collider_extra_edge_def_t *edge =
                &body_collider_extra_edges[i];
            if (!state->valid[edge->start_node] ||
                !state->valid[edge->end_node] ||
                !draw_valid[edge->start_node] ||
                !draw_valid[edge->end_node]) {
                continue;
            }
            draw_opengl_debug_tapered_capsule(
                p[edge->start_node][0], p[edge->start_node][1],
                radius_px[edge->start_node],
                p[edge->end_node][0], p[edge->end_node][1],
                radius_px[edge->end_node],
                edge->r, edge->g, edge->b);
        }
        if (draw_valid[BODY_COLLIDER_HIP_L] &&
            draw_valid[BODY_COLLIDER_THIGH_L]) {
            draw_opengl_debug_tapered_capsule(
                p[BODY_COLLIDER_HIP_L][0], p[BODY_COLLIDER_HIP_L][1],
                radius_px[BODY_COLLIDER_HIP_L],
                p[BODY_COLLIDER_THIGH_L][0], p[BODY_COLLIDER_THIGH_L][1],
                radius_px[BODY_COLLIDER_THIGH_L], 32, 255, 255);
        }
        if (draw_valid[BODY_COLLIDER_THIGH_L] &&
            draw_valid[BODY_COLLIDER_KNEE_L]) {
            draw_opengl_debug_tapered_capsule(
                p[BODY_COLLIDER_THIGH_L][0], p[BODY_COLLIDER_THIGH_L][1],
                radius_px[BODY_COLLIDER_THIGH_L],
                p[BODY_COLLIDER_KNEE_L][0], p[BODY_COLLIDER_KNEE_L][1],
                radius_px[BODY_COLLIDER_KNEE_L], 32, 255, 255);
        }
        if (draw_valid[BODY_COLLIDER_HIP_R] &&
            draw_valid[BODY_COLLIDER_THIGH_R]) {
            draw_opengl_debug_tapered_capsule(
                p[BODY_COLLIDER_HIP_R][0], p[BODY_COLLIDER_HIP_R][1],
                radius_px[BODY_COLLIDER_HIP_R],
                p[BODY_COLLIDER_THIGH_R][0], p[BODY_COLLIDER_THIGH_R][1],
                radius_px[BODY_COLLIDER_THIGH_R], 32, 255, 255);
        }
        if (draw_valid[BODY_COLLIDER_THIGH_R] &&
            draw_valid[BODY_COLLIDER_KNEE_R]) {
            draw_opengl_debug_tapered_capsule(
                p[BODY_COLLIDER_THIGH_R][0], p[BODY_COLLIDER_THIGH_R][1],
                radius_px[BODY_COLLIDER_THIGH_R],
                p[BODY_COLLIDER_KNEE_R][0], p[BODY_COLLIDER_KNEE_R][1],
                radius_px[BODY_COLLIDER_KNEE_R], 32, 255, 255);
        }
        if (draw_valid[BODY_COLLIDER_HIP_L])
            draw_opengl_debug_circle(p[BODY_COLLIDER_HIP_L][0],
                                     p[BODY_COLLIDER_HIP_L][1],
                                     radius_px[BODY_COLLIDER_HIP_L], 32, 255, 255);
        if (draw_valid[BODY_COLLIDER_HIP_R])
            draw_opengl_debug_circle(p[BODY_COLLIDER_HIP_R][0],
                                     p[BODY_COLLIDER_HIP_R][1],
                                     radius_px[BODY_COLLIDER_HIP_R], 32, 255, 255);
        if (draw_valid[BODY_COLLIDER_THIGH_L])
            draw_opengl_debug_circle(p[BODY_COLLIDER_THIGH_L][0],
                                     p[BODY_COLLIDER_THIGH_L][1],
                                     radius_px[BODY_COLLIDER_THIGH_L], 32, 255, 255);
        if (draw_valid[BODY_COLLIDER_THIGH_R])
            draw_opengl_debug_circle(p[BODY_COLLIDER_THIGH_R][0],
                                     p[BODY_COLLIDER_THIGH_R][1],
                                     radius_px[BODY_COLLIDER_THIGH_R], 32, 255, 255);
        if (draw_valid[BODY_COLLIDER_KNEE_L])
            draw_opengl_debug_circle(p[BODY_COLLIDER_KNEE_L][0],
                                     p[BODY_COLLIDER_KNEE_L][1],
                                     radius_px[BODY_COLLIDER_KNEE_L], 32, 255, 255);
        if (draw_valid[BODY_COLLIDER_KNEE_R])
            draw_opengl_debug_circle(p[BODY_COLLIDER_KNEE_R][0],
                                     p[BODY_COLLIDER_KNEE_R][1],
                                     radius_px[BODY_COLLIDER_KNEE_R], 32, 255, 255);
        for (i = 0; i < BODY_COLLIDER_NODE_COUNT; i++) {
            float axes[3];
            float major[2];
            float minor[2];
            unsigned char r, g, b;
            if (!state->valid[i] || !draw_valid[i]) continue;
            if (i >= BODY_COLLIDER_TESTICLES_01 &&
                i <= BODY_COLLIDER_TESTICLES_MID) {
                continue;
            }
            if (i <= BODY_COLLIDER_TESTICLES_MID &&
                !body_chain_collider_node_radius_is_oval(i)) {
                continue;
            }
            body_chain_collider_visual_radius_axes_for_node(i, axes);
            body_chain_collider_node_color_rgb(i, &r, &g, &b);
            if (body_chain_debug_projected_ellipse_opengl(
                    projection, viewport, state, state->local_position[i],
                    axes, p[i][0], p[i][1], major, minor)) {
                draw_opengl_debug_ellipse(p[i][0], p[i][1],
                                          major, minor, r, g, b);
            } else {
                draw_opengl_debug_circle(p[i][0], p[i][1],
                                         radius_px[i], r, g, b);
            }
        }
        if (chain_ready && body_chain_collider_cfg.penis_collision_enabled) {
            for (i = 0; i < 3; i++) {
                float r0 = chain_radius_px[i];
                float r1 = chain_radius_px[i + 1];
                float radius = r0 > r1 ? r0 : r1;
                draw_opengl_debug_capsule(chain_p[i][0],
                                          chain_p[i][1],
                                          chain_p[i + 1][0],
                                          chain_p[i + 1][1],
                                          radius, 255, 160, 0);
            }
        }
        if (body_chain_collider_cfg.testicle_collision_enabled) {
            for (i = BODY_COLLIDER_TESTICLES_01;
                 i <= BODY_COLLIDER_TESTICLES_02;
                 i++) {
                float axes[3];
                float major[2];
                float minor[2];
                if (!draw_valid[i]) continue;
                body_chain_collider_visual_radius_axes_for_node(i, axes);
                if (body_chain_debug_projected_ellipse_opengl(
                        projection, viewport, state, state->local_position[i],
                        axes, p[i][0], p[i][1], major, minor)) {
                    draw_opengl_debug_ellipse(p[i][0], p[i][1],
                                              major, minor, 255, 64, 255);
                } else {
                    draw_opengl_debug_circle(p[i][0], p[i][1],
                                             radius_px[i], 255, 64, 255);
                }
            }
            if (draw_valid[BODY_COLLIDER_TESTICLES_01] &&
                draw_valid[BODY_COLLIDER_TESTICLES_02]) {
                draw_opengl_debug_tapered_capsule(
                    p[BODY_COLLIDER_TESTICLES_01][0],
                    p[BODY_COLLIDER_TESTICLES_01][1],
                    radius_px[BODY_COLLIDER_TESTICLES_01],
                    p[BODY_COLLIDER_TESTICLES_02][0],
                    p[BODY_COLLIDER_TESTICLES_02][1],
                    radius_px[BODY_COLLIDER_TESTICLES_02],
                    255, 64, 255);
            }
        }
        body_profile_set_active_person_config(-1);
    }
    body_profile_set_active_person_config(-1);
    if (draw_addon) {
        draw_addon_sidecar_colliders_opengl(projection, viewport);
    }

    physx_gl_pop_matrix();
    physx_gl_matrix_mode(GL_PROJECTION);
    physx_gl_pop_matrix();
    physx_gl_matrix_mode((GLenum)old_matrix_mode);
    physx_gl_pop_attrib();
}

