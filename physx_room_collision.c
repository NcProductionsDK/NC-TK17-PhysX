#define ROOM_COLLISION_SIDECAR_COUNT 64
#define ROOM_COLLISION_TEXTURE_COUNT 64
#define ROOM_COLLISION_GRAPH_COUNT 4096
#define ROOM_COLLISION_LEAF_TRIANGLES 8

typedef struct room_collision_sidecar_t {
    char path[MAX_PATH * 4];
    char scene_key[MAX_PATH * 2];
    FILETIME write_time;
    int loaded;
} room_collision_sidecar_t;

typedef struct room_collision_link_t {
    char id[160];
    char link0[160];
    char link1[160];
    char name[160];
} room_collision_link_t;

typedef struct room_collision_mesh_t {
    char geometry_id[160];
    char name[160];
    DWORD color;
    int triangle_start;
    int triangle_count;
} room_collision_mesh_t;

typedef struct room_collision_triangle_t {
    float v[3][3];
    float minv[3];
    float maxv[3];
    float centroid[3];
    int mesh_index;
} room_collision_triangle_t;

typedef struct room_collision_bvh_node_t {
    float minv[3];
    float maxv[3];
    int left;
    int right;
    int start;
    int count;
} room_collision_bvh_node_t;

typedef struct room_collision_config_t {
    int active;
    int enabled;
    int debug_draw;
    int sidecar_index;
    unsigned int generation;
    char path[MAX_PATH * 4];
    char scene_key[MAX_PATH * 2];
    char scene_path[MAX_PATH * 4];
    char textures[ROOM_COLLISION_TEXTURE_COUNT][160];
    int texture_count;
} room_collision_config_t;

static room_collision_sidecar_t
    room_collision_sidecars[ROOM_COLLISION_SIDECAR_COUNT];
static int room_collision_sidecar_count;
static room_collision_config_t room_collision_cfg;
static DWORD room_collision_hot_reload_tick;
static unsigned int room_collision_applied_observation_generation;
static FILETIME room_collision_scene_write_time;
static room_collision_mesh_t *room_collision_meshes;
static int room_collision_mesh_count;
static room_collision_triangle_t *room_collision_triangles;
static int room_collision_triangle_count;
static int room_collision_triangle_capacity;
static room_collision_bvh_node_t *room_collision_bvh;
static int room_collision_bvh_count;
static int room_collision_bvh_capacity;
static int room_collision_sort_axis;

static int room_collision_has_section(const char *path)
{
    char section[4096];
    return path && path[0] &&
        GetPrivateProfileSectionA("collision", section, sizeof(section),
                                  path) > 0;
}

static void room_collision_free_geometry(void)
{
    free(room_collision_meshes);
    free(room_collision_triangles);
    free(room_collision_bvh);
    room_collision_meshes = NULL;
    room_collision_triangles = NULL;
    room_collision_bvh = NULL;
    room_collision_mesh_count = 0;
    room_collision_triangle_count = 0;
    room_collision_triangle_capacity = 0;
    room_collision_bvh_count = 0;
    room_collision_bvh_capacity = 0;
    memset(&room_collision_scene_write_time, 0,
           sizeof(room_collision_scene_write_time));
}

static int room_collision_register_sidecar_a(const char *path)
{
    char scene_key[MAX_PATH * 2];
    int i;
    room_collision_sidecar_t *entry;
    if (!path || !ends_with_i(path, ".physx.ini") ||
        !room_collision_has_section(path) ||
        !room_wind_scene_key_from_path(path, scene_key,
                                       sizeof(scene_key))) {
        return 0;
    }
    for (i = 0; i < room_collision_sidecar_count; i++) {
        if (_stricmp(room_collision_sidecars[i].path, path) == 0) return 1;
    }
    if (room_collision_sidecar_count >= ROOM_COLLISION_SIDECAR_COUNT) {
        log_line("room collision sidecar ignored sidecar=\"%s\" reason=\"registry full\"",
                 path);
        return 0;
    }
    entry = &room_collision_sidecars[room_collision_sidecar_count++];
    memset(entry, 0, sizeof(*entry));
    lstrcpynA(entry->path, path, sizeof(entry->path));
    lstrcpynA(entry->scene_key, scene_key, sizeof(entry->scene_key));
    log_line("room collision sidecar registered scene_key=\"%s\" sidecar=\"%s\"",
             entry->scene_key, entry->path);
    return 1;
}

static void room_collision_trim(char *text)
{
    char *start;
    char *end;
    if (!text) return;
    start = text;
    while (*start == ' ' || *start == '\t' || *start == '\r' ||
           *start == '\n') start++;
    if (start != text) memmove(text, start, strlen(start) + 1);
    end = text + strlen(text);
    while (end > text && (end[-1] == ' ' || end[-1] == '\t' ||
                          end[-1] == '\r' || end[-1] == '\n')) end--;
    *end = 0;
}

static const char *room_collision_basename(const char *path)
{
    const char *a = path ? strrchr(path, '\\') : NULL;
    const char *b = path ? strrchr(path, '/') : NULL;
    if (a && b) return (a > b ? a : b) + 1;
    if (a) return a + 1;
    if (b) return b + 1;
    return path ? path : "";
}

static void room_collision_parse_textures(const char *value)
{
    char buffer[4096];
    char *part;
    room_collision_cfg.texture_count = 0;
    if (!value || !value[0]) return;
    lstrcpynA(buffer, value, sizeof(buffer));
    part = strtok(buffer, ",");
    while (part &&
           room_collision_cfg.texture_count < ROOM_COLLISION_TEXTURE_COUNT) {
        const char *base;
        char *dot;
        room_collision_trim(part);
        base = room_collision_basename(part);
        lstrcpynA(room_collision_cfg.textures
                     [room_collision_cfg.texture_count],
                  base, 160);
        dot = strrchr(room_collision_cfg.textures
                         [room_collision_cfg.texture_count], '.');
        if (dot) *dot = 0;
        room_collision_trim(room_collision_cfg.textures
                                [room_collision_cfg.texture_count]);
        if (room_collision_cfg.textures
                [room_collision_cfg.texture_count][0]) {
            room_collision_cfg.texture_count++;
        }
        part = strtok(NULL, ",");
    }
}

static int room_collision_texture_selected(const char *path)
{
    char base[160];
    char *dot;
    int i;
    lstrcpynA(base, room_collision_basename(path), sizeof(base));
    dot = strrchr(base, '.');
    if (dot) *dot = 0;
    for (i = 0; i < room_collision_cfg.texture_count; i++) {
        if (_stricmp(base, room_collision_cfg.textures[i]) == 0) return 1;
    }
    return 0;
}

static int room_collision_find_scene_path(const char *sidecar,
                                          char *out, size_t outsz)
{
    char dir[MAX_PATH * 4];
    char folder[256];
    char candidate[MAX_PATH * 4];
    char *slash;
    if (!sidecar || !out || !outsz) return 0;
    lstrcpynA(dir, sidecar, sizeof(dir));
    slash = strrchr(dir, '\\');
    if (!slash) return 0;
    *slash = 0;
    slash = strrchr(dir, '\\');
    lstrcpynA(folder, slash ? slash + 1 : dir, sizeof(folder));
    snprintf(candidate, sizeof(candidate), "%s\\%s_core.bs", dir, folder);
    if (GetFileAttributesA(candidate) != INVALID_FILE_ATTRIBUTES) {
        lstrcpynA(out, candidate, (int)outsz);
        return 1;
    }
    snprintf(candidate, sizeof(candidate), "%s\\%s.bs", dir, folder);
    if (GetFileAttributesA(candidate) != INVALID_FILE_ATTRIBUTES) {
        lstrcpynA(out, candidate, (int)outsz);
        return 1;
    }
    return 0;
}

static char *room_collision_read_line(FILE *file, char **buffer,
                                      size_t *capacity)
{
    size_t len = 0;
    int c;
    if (!file || !buffer || !capacity) return NULL;
    if (!*buffer || *capacity < 4096) {
        *capacity = 4096;
        *buffer = (char *)malloc(*capacity);
        if (!*buffer) return NULL;
    }
    while ((c = fgetc(file)) != EOF) {
        if (len + 2 >= *capacity) {
            size_t next = *capacity * 2;
            char *grown = (char *)realloc(*buffer, next);
            if (!grown) return NULL;
            *buffer = grown;
            *capacity = next;
        }
        (*buffer)[len++] = (char)c;
        if (c == '\n') break;
    }
    if (!len && c == EOF) return NULL;
    (*buffer)[len] = 0;
    return *buffer;
}

static int room_collision_extract_local_id(const char *text,
                                           char *out, size_t outsz)
{
    const char *start;
    const char *end;
    size_t len;
    if (!text || !out || !outsz) return 0;
    start = strstr(text, ":local_");
    if (!start) return 0;
    start += 7;
    end = start;
    while (*end && *end != ' ' && *end != '\t' && *end != ';' &&
           *end != '.' && *end != '{' && *end != '}' && *end != '\r' &&
           *end != '\n') end++;
    len = (size_t)(end - start);
    if (!len) return 0;
    if (len >= outsz) len = outsz - 1;
    memcpy(out, start, len);
    out[len] = 0;
    return 1;
}

static int room_collision_extract_quoted(const char *text,
                                         char *out, size_t outsz)
{
    const char *start;
    const char *end;
    size_t len;
    if (!text || !out || !outsz) return 0;
    start = strchr(text, '"');
    if (!start) return 0;
    end = strchr(++start, '"');
    if (!end) return 0;
    len = (size_t)(end - start);
    if (len >= outsz) len = outsz - 1;
    memcpy(out, start, len);
    out[len] = 0;
    return 1;
}

static room_collision_link_t *room_collision_graph_add(
    room_collision_link_t *items, int *count, const char *id)
{
    int i;
    if (!items || !count || !id || !id[0]) return NULL;
    for (i = 0; i < *count; i++) {
        if (_stricmp(items[i].id, id) == 0) return &items[i];
    }
    if (*count >= ROOM_COLLISION_GRAPH_COUNT) return NULL;
    memset(&items[*count], 0, sizeof(items[*count]));
    lstrcpynA(items[*count].id, id, sizeof(items[*count].id));
    return &items[(*count)++];
}

static room_collision_link_t *room_collision_graph_find(
    room_collision_link_t *items, int count, const char *id)
{
    int i;
    if (!items || !id) return NULL;
    for (i = 0; i < count; i++) {
        if (_stricmp(items[i].id, id) == 0) return &items[i];
    }
    return NULL;
}

static DWORD room_collision_color_for_name(const char *name)
{
    static const DWORD palette[] = {
        0xffff4040u, 0xff40ff80u, 0xff4080ffu, 0xffffff40u,
        0xffff40ffu, 0xff40ffffu, 0xffff8040u, 0xff80ff40u,
        0xff8040ffu, 0xff40ffcfu, 0xffff40a0u, 0xffa0ff40u
    };
    unsigned int hash = 2166136261u;
    const unsigned char *p = (const unsigned char *)(name ? name : "");
    while (*p) {
        hash ^= (unsigned int)(unsigned char)tolower(*p++);
        hash *= 16777619u;
    }
    return palette[hash % (sizeof(palette) / sizeof(palette[0]))];
}

static int room_collision_float3(const char **cursor, float out[3])
{
    char *end;
    int axis;
    const char *p = cursor ? *cursor : NULL;
    if (!p || !out) return 0;
    p = strchr(p, '(');
    if (!p) return 0;
    p++;
    for (axis = 0; axis < 3; axis++) {
        while (*p == ' ' || *p == '\t' || *p == ',') p++;
        out[axis] = (float)strtod(p, &end);
        if (end == p) return 0;
        p = end;
    }
    p = strchr(p, ')');
    if (!p) return 0;
    *cursor = p + 1;
    return 1;
}

static int room_collision_parse_int_array(const char *line,
                                          int **values_out, int *count_out)
{
    const char *p = strchr(line, '[');
    int *values = NULL;
    int count = 0;
    int capacity = 0;
    if (!p || !values_out || !count_out) return 0;
    p++;
    while (*p && *p != ']') {
        char *end;
        long value;
        while (*p == ' ' || *p == '\t' || *p == ',' || *p == '\r' ||
               *p == '\n') p++;
        if (*p == ']') break;
        value = strtol(p, &end, 10);
        if (end == p) { p++; continue; }
        if (count >= capacity) {
            int next = capacity ? capacity * 2 : 256;
            int *grown = (int *)realloc(values, sizeof(int) * next);
            if (!grown) { free(values); return 0; }
            values = grown;
            capacity = next;
        }
        values[count++] = (int)value;
        p = end;
    }
    *values_out = values;
    *count_out = count;
    return count > 0;
}

static int room_collision_parse_vertex_array(const char *line,
                                             float (**values_out)[3],
                                             int *count_out)
{
    const char *p = strchr(line, '[');
    float (*values)[3] = NULL;
    int count = 0;
    int capacity = 0;
    if (!p || !values_out || !count_out) return 0;
    p++;
    while ((p = strchr(p, '(')) != NULL) {
        float value[3];
        if (!room_collision_float3(&p, value)) break;
        if (count >= capacity) {
            int next = capacity ? capacity * 2 : 256;
            float (*grown)[3] = (float (*)[3])realloc(
                values, sizeof(float[3]) * next);
            if (!grown) { free(values); return 0; }
            values = grown;
            capacity = next;
        }
        memcpy(values[count++], value, sizeof(value));
    }
    *values_out = values;
    *count_out = count;
    return count > 0;
}

static int room_collision_add_triangle(int mesh_index,
                                       const float a[3], const float b[3],
                                       const float c[3])
{
    room_collision_triangle_t *tri;
    int axis;
    if (room_collision_triangle_count >= room_collision_triangle_capacity) {
        int next = room_collision_triangle_capacity ?
            room_collision_triangle_capacity * 2 : 1024;
        room_collision_triangle_t *grown =
            (room_collision_triangle_t *)realloc(
                room_collision_triangles,
                sizeof(room_collision_triangle_t) * next);
        if (!grown) return 0;
        room_collision_triangles = grown;
        room_collision_triangle_capacity = next;
    }
    tri = &room_collision_triangles[room_collision_triangle_count++];
    memset(tri, 0, sizeof(*tri));
    memcpy(tri->v[0], a, sizeof(float) * 3);
    memcpy(tri->v[1], b, sizeof(float) * 3);
    memcpy(tri->v[2], c, sizeof(float) * 3);
    tri->mesh_index = mesh_index;
    for (axis = 0; axis < 3; axis++) {
        tri->minv[axis] = a[axis];
        if (b[axis] < tri->minv[axis]) tri->minv[axis] = b[axis];
        if (c[axis] < tri->minv[axis]) tri->minv[axis] = c[axis];
        tri->maxv[axis] = a[axis];
        if (b[axis] > tri->maxv[axis]) tri->maxv[axis] = b[axis];
        if (c[axis] > tri->maxv[axis]) tri->maxv[axis] = c[axis];
        tri->centroid[axis] = (a[axis] + b[axis] + c[axis]) / 3.0f;
    }
    return 1;
}

static int room_collision_triangle_compare(const void *left,
                                           const void *right)
{
    const room_collision_triangle_t *a =
        (const room_collision_triangle_t *)left;
    const room_collision_triangle_t *b =
        (const room_collision_triangle_t *)right;
    float d = a->centroid[room_collision_sort_axis] -
              b->centroid[room_collision_sort_axis];
    return d < 0.0f ? -1 : (d > 0.0f ? 1 : 0);
}

static int room_collision_build_bvh_node(int start, int count)
{
    room_collision_bvh_node_t *node;
    float cmin[3], cmax[3];
    int node_index;
    int axis;
    int i;
    if (room_collision_bvh_count >= room_collision_bvh_capacity) {
        int next = room_collision_bvh_capacity ?
            room_collision_bvh_capacity * 2 : 1024;
        room_collision_bvh_node_t *grown =
            (room_collision_bvh_node_t *)realloc(
                room_collision_bvh,
                sizeof(room_collision_bvh_node_t) * next);
        if (!grown) return -1;
        room_collision_bvh = grown;
        room_collision_bvh_capacity = next;
    }
    node_index = room_collision_bvh_count++;
    node = &room_collision_bvh[node_index];
    memset(node, 0, sizeof(*node));
    node->left = node->right = -1;
    node->start = start;
    node->count = count;
    for (axis = 0; axis < 3; axis++) {
        node->minv[axis] = cmin[axis] = 1.0e30f;
        node->maxv[axis] = cmax[axis] = -1.0e30f;
    }
    for (i = start; i < start + count; i++) {
        room_collision_triangle_t *tri = &room_collision_triangles[i];
        for (axis = 0; axis < 3; axis++) {
            if (tri->minv[axis] < node->minv[axis])
                node->minv[axis] = tri->minv[axis];
            if (tri->maxv[axis] > node->maxv[axis])
                node->maxv[axis] = tri->maxv[axis];
            if (tri->centroid[axis] < cmin[axis]) cmin[axis] = tri->centroid[axis];
            if (tri->centroid[axis] > cmax[axis]) cmax[axis] = tri->centroid[axis];
        }
    }
    if (count > ROOM_COLLISION_LEAF_TRIANGLES) {
        float best = cmax[0] - cmin[0];
        room_collision_sort_axis = 0;
        for (axis = 1; axis < 3; axis++) {
            float span = cmax[axis] - cmin[axis];
            if (span > best) { best = span; room_collision_sort_axis = axis; }
        }
        if (best > 0.000001f) {
            int left_count = count / 2;
            int left_index;
            int right_index;
            qsort(room_collision_triangles + start, count,
                  sizeof(room_collision_triangle_t),
                  room_collision_triangle_compare);
            left_index = room_collision_build_bvh_node(start, left_count);
            right_index = room_collision_build_bvh_node(
                start + left_count, count - left_count);
            /* Either recursive call may grow/reallocate the BVH array, so do
               not retain a node pointer across either call. */
            node = &room_collision_bvh[node_index];
            node->left = left_index;
            node->right = right_index;
            if (node->left >= 0 && node->right >= 0) node->count = 0;
        }
    }
    return node_index;
}

static int room_collision_load_geometry(const char *scene_path)
{
    room_collision_link_t *geometries = NULL, *renderers = NULL;
    room_collision_link_t *phongs = NULL, *shader_textures = NULL;
    room_collision_link_t *textures = NULL, *files = NULL;
    int geometry_count = 0, renderer_count = 0, phong_count = 0;
    int shader_texture_count = 0, texture_count = 0, file_count = 0;
    room_collision_link_t *current = NULL;
    enum { RC_RECORD_NONE, RC_GEOM, RC_RENDERER, RC_PHONG,
           RC_SHADER_TEXTURE, RC_TEXTURE, RC_FILE }
        current_type = RC_RECORD_NONE;
    FILE *file = NULL;
    char *line = NULL;
    size_t line_capacity = 0;
    char id[160];
    int i;
    int ok = 0;

    room_collision_free_geometry();
    geometries = (room_collision_link_t *)calloc(
        ROOM_COLLISION_GRAPH_COUNT, sizeof(*geometries));
    renderers = (room_collision_link_t *)calloc(
        ROOM_COLLISION_GRAPH_COUNT, sizeof(*renderers));
    phongs = (room_collision_link_t *)calloc(
        ROOM_COLLISION_GRAPH_COUNT, sizeof(*phongs));
    shader_textures = (room_collision_link_t *)calloc(
        ROOM_COLLISION_GRAPH_COUNT, sizeof(*shader_textures));
    textures = (room_collision_link_t *)calloc(
        ROOM_COLLISION_GRAPH_COUNT, sizeof(*textures));
    files = (room_collision_link_t *)calloc(
        ROOM_COLLISION_GRAPH_COUNT, sizeof(*files));
    if (!geometries || !renderers || !phongs || !shader_textures ||
        !textures || !files) goto cleanup;
    file = fopen(scene_path, "rb");
    if (!file) goto cleanup;
    while (room_collision_read_line(file, &line, &line_capacity)) {
        const char *trimmed = line;
        while (*trimmed == ' ' || *trimmed == '\t') trimmed++;
        current = NULL;
        if (_strnicmp(trimmed, "TPolygonGeometry :local_", 24) == 0 &&
            room_collision_extract_local_id(trimmed, id, sizeof(id))) {
            current_type = RC_GEOM;
            current = room_collision_graph_add(geometries, &geometry_count, id);
        } else if (_strnicmp(trimmed, "RenderShader :local_", 20) == 0 &&
                   room_collision_extract_local_id(trimmed, id, sizeof(id))) {
            current_type = RC_RENDERER;
            current = room_collision_graph_add(renderers, &renderer_count, id);
        } else if (_strnicmp(trimmed, "ShaderPhong :local_", 19) == 0 &&
                   room_collision_extract_local_id(trimmed, id, sizeof(id))) {
            current_type = RC_PHONG;
            current = room_collision_graph_add(phongs, &phong_count, id);
        } else if (_strnicmp(trimmed, "ShaderTexture :local_", 21) == 0 &&
                   room_collision_extract_local_id(trimmed, id, sizeof(id))) {
            current_type = RC_SHADER_TEXTURE;
            current = room_collision_graph_add(
                shader_textures, &shader_texture_count, id);
        } else if (_strnicmp(trimmed, "Texture2D :local_", 17) == 0 &&
                   room_collision_extract_local_id(trimmed, id, sizeof(id))) {
            current_type = RC_TEXTURE;
            current = room_collision_graph_add(textures, &texture_count, id);
        } else if (_strnicmp(trimmed, "FileObject :local_", 18) == 0 &&
                   room_collision_extract_local_id(trimmed, id, sizeof(id))) {
            current_type = RC_FILE;
            current = room_collision_graph_add(files, &file_count, id);
        } else {
            switch (current_type) {
            case RC_GEOM: current = geometry_count ? &geometries[geometry_count - 1] : NULL; break;
            case RC_RENDERER: current = renderer_count ? &renderers[renderer_count - 1] : NULL; break;
            case RC_PHONG: current = phong_count ? &phongs[phong_count - 1] : NULL; break;
            case RC_SHADER_TEXTURE: current = shader_texture_count ? &shader_textures[shader_texture_count - 1] : NULL; break;
            case RC_TEXTURE: current = texture_count ? &textures[texture_count - 1] : NULL; break;
            case RC_FILE: current = file_count ? &files[file_count - 1] : NULL; break;
            default: break;
            }
        }
        if (!current) continue;
        if (current_type == RC_GEOM && strstr(trimmed, "TNode.SNode SPolygonGeometry") &&
            room_collision_extract_local_id(trimmed, id, sizeof(id)))
            lstrcpynA(current->link0, id, sizeof(current->link0));
        else if (current_type == RC_GEOM && strstr(trimmed, "TGeometry.Shader RenderShader") &&
                 room_collision_extract_local_id(trimmed, id, sizeof(id)))
            lstrcpynA(current->link1, id, sizeof(current->link1));
        else if (current_type == RC_RENDERER && strstr(trimmed, "RenderShader.Surface ShaderPhong") &&
                 room_collision_extract_local_id(trimmed, id, sizeof(id)))
            lstrcpynA(current->link0, id, sizeof(current->link0));
        else if (current_type == RC_PHONG && strstr(trimmed, "ShaderPhong.Color ShaderTexture") &&
                 room_collision_extract_local_id(trimmed, id, sizeof(id)))
            lstrcpynA(current->link0, id, sizeof(current->link0));
        else if (current_type == RC_SHADER_TEXTURE && strstr(trimmed, "ShaderTexture.Texture Texture2D") &&
                 room_collision_extract_local_id(trimmed, id, sizeof(id)))
            lstrcpynA(current->link0, id, sizeof(current->link0));
        else if (current_type == RC_TEXTURE && strstr(trimmed, "Texture.FileObject FileObject") &&
                 room_collision_extract_local_id(trimmed, id, sizeof(id)))
            lstrcpynA(current->link0, id, sizeof(current->link0));
        if (strstr(trimmed, "Object.Name"))
            room_collision_extract_quoted(trimmed, current->name, sizeof(current->name));
        if (current_type == RC_FILE && strstr(trimmed, "FileObject.FileName"))
            room_collision_extract_quoted(trimmed, current->name, sizeof(current->name));
        if (strstr(trimmed, "};") ||
            (current_type == RC_FILE && strstr(trimmed, "FileObject.FileName"))) {
            current_type = RC_RECORD_NONE;
        }
    }
    fclose(file);
    file = NULL;

    room_collision_meshes = (room_collision_mesh_t *)calloc(
        geometry_count ? geometry_count : 1, sizeof(*room_collision_meshes));
    if (!room_collision_meshes) goto cleanup;
    for (i = 0; i < geometry_count; i++) {
        room_collision_link_t *renderer = room_collision_graph_find(
            renderers, renderer_count, geometries[i].link1);
        room_collision_link_t *phong = renderer ? room_collision_graph_find(
            phongs, phong_count, renderer->link0) : NULL;
        room_collision_link_t *shader_texture = phong ? room_collision_graph_find(
            shader_textures, shader_texture_count, phong->link0) : NULL;
        room_collision_link_t *texture = shader_texture ? room_collision_graph_find(
            textures, texture_count, shader_texture->link0) : NULL;
        room_collision_link_t *file_link = texture ? room_collision_graph_find(
            files, file_count, texture->link0) : NULL;
        room_collision_mesh_t *mesh;
        if (!file_link || !room_collision_texture_selected(file_link->name) ||
            !geometries[i].link0[0]) continue;
        mesh = &room_collision_meshes[room_collision_mesh_count++];
        lstrcpynA(mesh->geometry_id, geometries[i].link0,
                  sizeof(mesh->geometry_id));
        lstrcpynA(mesh->name,
                  geometries[i].name[0] ? geometries[i].name : geometries[i].id,
                  sizeof(mesh->name));
        mesh->color = room_collision_color_for_name(mesh->name);
    }
    if (!room_collision_mesh_count) {
        log_line("room collision geometry empty scene=\"%s\" textures=%d reason=\"no polygon geometry uses a selected color texture\"",
                 scene_path, room_collision_cfg.texture_count);
        goto cleanup;
    }

    file = fopen(scene_path, "rb");
    if (!file) goto cleanup;
    {
        int active_mesh = -1;
        int *indices = NULL;
        int index_count = 0;
        float (*vertices)[3] = NULL;
        int vertex_count = 0;
        while (room_collision_read_line(file, &line, &line_capacity)) {
            const char *trimmed = line;
            while (*trimmed == ' ' || *trimmed == '\t') trimmed++;
            if (_strnicmp(trimmed, "SPolygonGeometry :local_", 24) == 0 &&
                room_collision_extract_local_id(trimmed, id, sizeof(id))) {
                int mesh_index;
                active_mesh = -1;
                free(indices); indices = NULL; index_count = 0;
                free(vertices); vertices = NULL; vertex_count = 0;
                for (mesh_index = 0; mesh_index < room_collision_mesh_count;
                     mesh_index++) {
                    if (_stricmp(room_collision_meshes[mesh_index].geometry_id,
                                 id) == 0) {
                        active_mesh = mesh_index;
                        room_collision_meshes[mesh_index].triangle_start =
                            room_collision_triangle_count;
                        break;
                    }
                }
                continue;
            }
            if (active_mesh < 0) continue;
            if (strstr(trimmed, "SPolygonGeometry.IndexArray Array_I32")) {
                room_collision_parse_int_array(trimmed, &indices, &index_count);
            } else if (strstr(trimmed, "SPolygonGeometry.VertexArray Array_Vector3f")) {
                room_collision_parse_vertex_array(trimmed, &vertices,
                                                  &vertex_count);
            }
            if (strstr(trimmed, "};")) {
                int index;
                for (index = 0; indices && vertices && index + 2 < index_count;
                     index += 3) {
                    int a = indices[index], b = indices[index + 1],
                        c = indices[index + 2];
                    if (a < 0 || b < 0 || c < 0 ||
                        a >= vertex_count || b >= vertex_count ||
                        c >= vertex_count) continue;
                    if (!room_collision_add_triangle(active_mesh,
                                                     vertices[a], vertices[b],
                                                     vertices[c])) break;
                }
                room_collision_meshes[active_mesh].triangle_count =
                    room_collision_triangle_count -
                    room_collision_meshes[active_mesh].triangle_start;
                free(indices); indices = NULL; index_count = 0;
                free(vertices); vertices = NULL; vertex_count = 0;
                active_mesh = -1;
            }
        }
        free(indices);
        free(vertices);
    }
    fclose(file);
    file = NULL;
    if (!room_collision_triangle_count ||
        room_collision_build_bvh_node(0, room_collision_triangle_count) < 0) {
        goto cleanup;
    }
    get_file_write_time_a(scene_path, &room_collision_scene_write_time);
    log_line("room collision geometry loaded scene=\"%s\" selected_textures=%d meshes=%d triangles=%d bvh_nodes=%d debug_draw=%d",
             scene_path, room_collision_cfg.texture_count,
             room_collision_mesh_count, room_collision_triangle_count,
             room_collision_bvh_count, room_collision_cfg.debug_draw);
    ok = 1;

cleanup:
    if (file) fclose(file);
    free(line);
    free(geometries);
    free(renderers);
    free(phongs);
    free(shader_textures);
    free(textures);
    free(files);
    if (!ok) room_collision_free_geometry();
    return ok;
}

static void room_collision_clear_active(const char *reason)
{
    if (room_collision_cfg.active) {
        log_line("room collision deactivated scene_key=\"%s\" reason=\"%s\"",
                 room_collision_cfg.scene_key,
                 reason ? reason : "room transition");
    }
    room_collision_free_geometry();
    room_collision_cfg.active = 0;
    room_collision_cfg.enabled = 0;
    room_collision_cfg.sidecar_index = -1;
    room_collision_cfg.path[0] = 0;
    room_collision_cfg.scene_key[0] = 0;
    room_collision_cfg.scene_path[0] = 0;
    room_collision_cfg.generation++;
}

static void room_collision_load_entry(int index, const char *reason)
{
    room_collision_sidecar_t *entry;
    FILETIME write_time;
    char texture_list[4096];
    unsigned int generation = room_collision_cfg.generation + 1u;
    if (index < 0 || index >= room_collision_sidecar_count) return;
    entry = &room_collision_sidecars[index];
    if (!get_file_write_time_a(entry->path, &write_time) ||
        !room_collision_has_section(entry->path)) {
        room_collision_clear_active("sidecar missing or [collision] removed");
        return;
    }
    room_collision_free_geometry();
    memset(&room_collision_cfg, 0, sizeof(room_collision_cfg));
    room_collision_cfg.active = 1;
    room_collision_cfg.sidecar_index = index;
    room_collision_cfg.generation = generation ? generation : 1u;
    room_collision_cfg.enabled = profile_bool(
        "collision", "enabled", 1, entry->path);
    room_collision_cfg.debug_draw = profile_bool(
        "collision", "collision_debug_draw", 0, entry->path);
    lstrcpynA(room_collision_cfg.path, entry->path,
              sizeof(room_collision_cfg.path));
    lstrcpynA(room_collision_cfg.scene_key, entry->scene_key,
              sizeof(room_collision_cfg.scene_key));
    GetPrivateProfileStringA("collision", "textures", "", texture_list,
                             sizeof(texture_list), entry->path);
    room_collision_parse_textures(texture_list);
    entry->write_time = write_time;
    entry->loaded = 1;
    if (room_collision_cfg.enabled && room_collision_cfg.texture_count > 0 &&
        room_collision_find_scene_path(entry->path,
                                       room_collision_cfg.scene_path,
                                       sizeof(room_collision_cfg.scene_path))) {
        room_collision_load_geometry(room_collision_cfg.scene_path);
    }
    log_line("room collision %s enabled=%d scene_key=\"%s\" textures=%d meshes=%d triangles=%d debug_draw=%d sidecar=\"%s\"",
             reason ? reason : "loaded", room_collision_cfg.enabled,
             room_collision_cfg.scene_key, room_collision_cfg.texture_count,
             room_collision_mesh_count, room_collision_triangle_count,
             room_collision_cfg.debug_draw, room_collision_cfg.path);
}

static void room_collision_apply_observed_scene(void)
{
    int best = -1;
    int i;
    if (room_collision_applied_observation_generation ==
        room_wind_observation_generation) return;
    room_collision_applied_observation_generation =
        room_wind_observation_generation;
    for (i = 0; i < room_collision_sidecar_count; i++) {
        if (_stricmp(room_collision_sidecars[i].scene_key,
                     room_wind_observed_scene_key) == 0) {
            best = i;
            break;
        }
    }
    if (best >= 0) {
        if (!room_collision_cfg.active ||
            room_collision_cfg.sidecar_index != best) {
            room_collision_load_entry(best, "activated");
        }
    } else if (room_collision_cfg.active) {
        room_collision_clear_active("new room has no [collision] sidecar");
    }
}

static void run_room_collision_hot_reload(DWORD now)
{
    room_collision_sidecar_t *entry;
    FILETIME write_time;
    FILETIME scene_time;
    room_collision_apply_observed_scene();
    if (!room_collision_cfg.active || room_collision_cfg.sidecar_index < 0 ||
        room_collision_cfg.sidecar_index >= room_collision_sidecar_count)
        return;
    if (room_collision_hot_reload_tick &&
        now - room_collision_hot_reload_tick < 750u) return;
    room_collision_hot_reload_tick = now;
    entry = &room_collision_sidecars[room_collision_cfg.sidecar_index];
    if (!get_file_write_time_a(entry->path, &write_time)) {
        room_collision_clear_active("active sidecar removed");
        return;
    }
    if (!entry->loaded || filetime_differs(&entry->write_time, &write_time)) {
        room_collision_load_entry(room_collision_cfg.sidecar_index,
                                  "hot-reloaded");
        return;
    }
    if (room_collision_cfg.scene_path[0] &&
        get_file_write_time_a(room_collision_cfg.scene_path, &scene_time) &&
        filetime_differs(&room_collision_scene_write_time, &scene_time)) {
        room_collision_load_entry(room_collision_cfg.sidecar_index,
                                  "scene-hot-reloaded");
    }
}

static int room_collision_is_enabled(void)
{
    return room_collision_cfg.active && room_collision_cfg.enabled &&
           room_collision_triangle_count > 0 && room_collision_bvh_count > 0;
}

static int room_collision_debug_any(void)
{
    return room_collision_is_enabled() && room_collision_cfg.debug_draw;
}

static float room_collision_aabb_distance_sq(const float point[3],
                                             const float minv[3],
                                             const float maxv[3])
{
    float total = 0.0f;
    int axis;
    for (axis = 0; axis < 3; axis++) {
        float d = point[axis] < minv[axis] ? minv[axis] - point[axis] :
                  (point[axis] > maxv[axis] ? point[axis] - maxv[axis] : 0.0f);
        total += d * d;
    }
    return total;
}

static void room_collision_closest_triangle_point(
    const float p[3], const float a[3], const float b[3], const float c[3],
    float out[3])
{
    float ab[3], ac[3], ap[3], bp[3], cp[3], bc[3];
    float d1, d2, d3, d4, d5, d6, vc, vb, va, v, w, denom;
    int axis;
    for (axis = 0; axis < 3; axis++) {
        ab[axis] = b[axis] - a[axis];
        ac[axis] = c[axis] - a[axis];
        ap[axis] = p[axis] - a[axis];
    }
    d1 = vec3_dot(ab, ap); d2 = vec3_dot(ac, ap);
    if (d1 <= 0.0f && d2 <= 0.0f) { memcpy(out, a, sizeof(float) * 3); return; }
    for (axis = 0; axis < 3; axis++) bp[axis] = p[axis] - b[axis];
    d3 = vec3_dot(ab, bp); d4 = vec3_dot(ac, bp);
    if (d3 >= 0.0f && d4 <= d3) { memcpy(out, b, sizeof(float) * 3); return; }
    vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
        v = d1 / (d1 - d3);
        for (axis = 0; axis < 3; axis++) out[axis] = a[axis] + ab[axis] * v;
        return;
    }
    for (axis = 0; axis < 3; axis++) cp[axis] = p[axis] - c[axis];
    d5 = vec3_dot(ab, cp); d6 = vec3_dot(ac, cp);
    if (d6 >= 0.0f && d5 <= d6) { memcpy(out, c, sizeof(float) * 3); return; }
    vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
        w = d2 / (d2 - d6);
        for (axis = 0; axis < 3; axis++) out[axis] = a[axis] + ac[axis] * w;
        return;
    }
    va = d3 * d6 - d5 * d4;
    if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
        for (axis = 0; axis < 3; axis++) bc[axis] = c[axis] - b[axis];
        w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        for (axis = 0; axis < 3; axis++) out[axis] = b[axis] + bc[axis] * w;
        return;
    }
    denom = 1.0f / (va + vb + vc);
    v = vb * denom; w = vc * denom;
    for (axis = 0; axis < 3; axis++)
        out[axis] = a[axis] + ab[axis] * v + ac[axis] * w;
}

static int room_collision_resolve_sphere(const float center[3], float radius,
                                         float correction[3],
                                         int *mesh_index_out)
{
    physx_contact_set_t contacts = {0};
    int stack[128];
    int stack_count = 0;
    float radius_sq;
    float best_penetration = 0.0f;
    float best_normal[3] = { 0.0f, 0.0f, 0.0f };
    int best_mesh = -1;
    if (correction) memset(correction, 0, sizeof(float) * 3);
    if (!center || !correction || !room_collision_is_enabled() ||
        radius <= 0.000001f) return 0;
    radius_sq = radius * radius;
    stack[stack_count++] = 0;
    while (stack_count > 0) {
        room_collision_bvh_node_t *node =
            &room_collision_bvh[stack[--stack_count]];
        int i;
        if (room_collision_aabb_distance_sq(center, node->minv, node->maxv) >
            radius_sq) continue;
        if (!node->count) {
            if (node->left >= 0 && stack_count < 126) stack[stack_count++] = node->left;
            if (node->right >= 0 && stack_count < 127) stack[stack_count++] = node->right;
            continue;
        }
        for (i = node->start; i < node->start + node->count; i++) {
            room_collision_triangle_t *tri = &room_collision_triangles[i];
            float closest[3], delta[3], dist, penetration;
            room_collision_closest_triangle_point(
                center, tri->v[0], tri->v[1], tri->v[2], closest);
            delta[0] = center[0] - closest[0];
            delta[1] = center[1] - closest[1];
            delta[2] = center[2] - closest[2];
            dist = physx_vec3_len(delta);
            penetration = radius - dist;
            if (penetration <= 0.000001f) continue;
            if (dist > 0.00001f) {
                best_normal[0] = delta[0] / dist;
                best_normal[1] = delta[1] / dist;
                best_normal[2] = delta[2] / dist;
            } else {
                float ab[3], ac[3];
                ab[0] = tri->v[1][0] - tri->v[0][0];
                ab[1] = tri->v[1][1] - tri->v[0][1];
                ab[2] = tri->v[1][2] - tri->v[0][2];
                ac[0] = tri->v[2][0] - tri->v[0][0];
                ac[1] = tri->v[2][1] - tri->v[0][1];
                ac[2] = tri->v[2][2] - tri->v[0][2];
                best_normal[0] = ab[1] * ac[2] - ab[2] * ac[1];
                best_normal[1] = ab[2] * ac[0] - ab[0] * ac[2];
                best_normal[2] = ab[0] * ac[1] - ab[1] * ac[0];
                dist = physx_vec3_len(best_normal);
                if (dist > 0.00001f) {
                    best_normal[0] /= dist; best_normal[1] /= dist;
                    best_normal[2] /= dist;
                } else continue;
                if (vec3_dot(best_normal, physics_environment_cfg.world_gravity) > 0.0f) {
                    best_normal[0] = -best_normal[0];
                    best_normal[1] = -best_normal[1];
                    best_normal[2] = -best_normal[2];
                }
            }
            physx_contact_store(&contacts, best_normal, penetration);
            if (penetration > best_penetration) {
                best_penetration = penetration;
                best_mesh = tri->mesh_index;
            }
        }
    }
    if (best_penetration <= 0.0f) return 0;
    physx_contact_resolve(&contacts, correction);
    {
        float length = physx_vec3_len(correction);
        if (length > 0.100f) {
            int axis;
            for (axis = 0; axis < 3; axis++) correction[axis] *= 0.100f / length;
        }
    }
    if (mesh_index_out) *mesh_index_out = best_mesh;
    return 1;
}

/* Resolve a moving sphere, or a sphere swept along a chain link, against a
   room mesh. The old endpoint-only query could miss a thin floor or wall
   whenever both endpoints landed outside the sphere penetration band. */
static int room_collision_resolve_swept_sphere(
    const float start[3], const float end[3], float radius,
    float correction[3], int *mesh_index_out)
{
    float delta[3];
    float distance;
    float start_correction[3];
    float point[3];
    float hit_correction[3];
    float normal[3];
    float remaining[3];
    float desired_end[3];
    float hit_len;
    float inward;
    float step_length;
    int steps;
    int step;
    int hit_mesh = -1;
    int axis;
    if (correction) memset(correction, 0, sizeof(float) * 3);
    if (!start || !end || !correction || !room_collision_is_enabled() ||
        radius <= 0.000001f) return 0;
    delta[0] = end[0] - start[0];
    delta[1] = end[1] - start[1];
    delta[2] = end[2] - start[2];
    distance = physx_vec3_len(delta);
    if (!sane_probe_float(distance) || distance <= 0.0005f) {
        return room_collision_resolve_sphere(
            end, radius, correction, mesh_index_out);
    }

    /* If the sweep begins inside the surface, an entry side cannot be
       inferred reliably. Resolve the live endpoint normally in that case. */
    if (room_collision_resolve_sphere(
            start, radius, start_correction, NULL)) {
        return room_collision_resolve_sphere(
            end, radius, correction, mesh_index_out);
    }

    step_length = physx_clampf(radius * 0.40f, 0.0025f, 0.0150f);
    steps = (int)ceil((double)(distance / step_length));
    if (steps < 2) steps = 2;
    if (steps > 192) steps = 192;
    for (step = 1; step <= steps; step++) {
        float t = (float)step / (float)steps;
        point[0] = start[0] + delta[0] * t;
        point[1] = start[1] + delta[1] * t;
        point[2] = start[2] + delta[2] * t;
        if (!room_collision_resolve_sphere(
                point, radius, hit_correction, &hit_mesh)) {
            continue;
        }
        hit_len = physx_vec3_len(hit_correction);
        if (hit_len <= 0.000001f) continue;
        normal[0] = hit_correction[0] / hit_len;
        normal[1] = hit_correction[1] / hit_len;
        normal[2] = hit_correction[2] / hit_len;
        for (axis = 0; axis < 3; axis++) {
            remaining[axis] = end[axis] - point[axis];
        }
        inward = vec3_dot(remaining, normal);
        if (inward < 0.0f) {
            for (axis = 0; axis < 3; axis++) {
                remaining[axis] -= normal[axis] * inward;
            }
        }
        for (axis = 0; axis < 3; axis++) {
            desired_end[axis] = point[axis] + hit_correction[axis] +
                                remaining[axis];
            correction[axis] = desired_end[axis] - end[axis];
        }
        if (mesh_index_out) *mesh_index_out = hit_mesh;
        return physx_vec3_len(correction) > 0.000001f;
    }
    return room_collision_resolve_sphere(
        end, radius, correction, mesh_index_out);
}

static int room_collision_world_vector_to_body_local(
    const body_chain_collider_person_state_t *state,
    const float world_vector[3], float local[3])
{
    float view[3];
    const float *m = captured_camera_inverse;
    if (!state || !world_vector || !local || !state->basis_valid ||
        !captured_camera_inverse_valid) return 0;
    view[0] = world_vector[0] * m[0] + world_vector[1] * m[1] +
              world_vector[2] * m[2];
    view[1] = world_vector[0] * m[4] + world_vector[1] * m[5] +
              world_vector[2] * m[6];
    view[2] = world_vector[0] * m[8] + world_vector[1] * m[9] +
              world_vector[2] * m[10];
    return body_collider_view_delta_to_local(
        view, state->basis_h, state->basis_v, state->basis_s, local);
}



static int room_collision_body_local_point_to_world(
    const body_chain_collider_person_state_t *state,
    const float local[3], float world[3])
{
    float view[3];
    return body_chain_collider_local_to_view(state, local, view) &&
           camera_view_to_world_point(view, world);
}

static int body_collision_world_vector_to_local(
    const body_chain_collider_person_state_t *state,
    const float world_vector[3], float local[3])
{
    float view[3];
    if (!state || !world_vector || !local || !state->basis_valid ||
        !state->contact_frame_valid) return 0;
    body_chain_transform_row_vector3(world_vector, state->contact_world_to_view, view);
    return body_collider_view_delta_to_local(
        view, state->basis_h, state->basis_v, state->basis_s, local);
}



static int body_collision_local_point_to_world(
    const body_chain_collider_person_state_t *state,
    const float local[3], float world[3])
{
    float view[3];
    return body_chain_collider_local_to_view(state, local, view) &&
           body_collision_view_to_world(state, view, world);
}

/* Single-bone contacts retain individual room planes for the combined solve. */
static int single_bone_room_contacts(const float center[3], float radius,
    physx_contact_set_t *out)
{
    physx_contact_set_t contacts = {0};
    int stack[128];
    int stack_count = 0;
    float radius_sq;
    float best_penetration = 0.0f;
    float best_normal[3] = { 0.0f, 0.0f, 0.0f };
    if (out) memset(out, 0, sizeof(*out));
    if (!center || !out || !room_collision_is_enabled() ||
        radius <= 0.000001f) return 0;
    radius_sq = radius * radius;
    stack[stack_count++] = 0;
    while (stack_count > 0) {
        room_collision_bvh_node_t *node =
            &room_collision_bvh[stack[--stack_count]];
        int i;
        if (room_collision_aabb_distance_sq(center, node->minv, node->maxv) >
            radius_sq) continue;
        if (!node->count) {
            if (node->left >= 0 && stack_count < 126) stack[stack_count++] = node->left;
            if (node->right >= 0 && stack_count < 127) stack[stack_count++] = node->right;
            continue;
        }
        for (i = node->start; i < node->start + node->count; i++) {
            room_collision_triangle_t *tri = &room_collision_triangles[i];
            float closest[3], delta[3], dist, penetration;
            room_collision_closest_triangle_point(
                center, tri->v[0], tri->v[1], tri->v[2], closest);
            delta[0] = center[0] - closest[0];
            delta[1] = center[1] - closest[1];
            delta[2] = center[2] - closest[2];
            dist = physx_vec3_len(delta);
            penetration = radius - dist;
            if (penetration <= 0.000001f) continue;
            if (dist > 0.00001f) {
                best_normal[0] = delta[0] / dist;
                best_normal[1] = delta[1] / dist;
                best_normal[2] = delta[2] / dist;
            } else {
                float ab[3], ac[3];
                ab[0] = tri->v[1][0] - tri->v[0][0];
                ab[1] = tri->v[1][1] - tri->v[0][1];
                ab[2] = tri->v[1][2] - tri->v[0][2];
                ac[0] = tri->v[2][0] - tri->v[0][0];
                ac[1] = tri->v[2][1] - tri->v[0][1];
                ac[2] = tri->v[2][2] - tri->v[0][2];
                best_normal[0] = ab[1] * ac[2] - ab[2] * ac[1];
                best_normal[1] = ab[2] * ac[0] - ab[0] * ac[2];
                best_normal[2] = ab[0] * ac[1] - ab[1] * ac[0];
                dist = physx_vec3_len(best_normal);
                if (dist > 0.00001f) {
                    best_normal[0] /= dist; best_normal[1] /= dist;
                    best_normal[2] /= dist;
                } else continue;
                if (vec3_dot(best_normal, physics_environment_cfg.world_gravity) > 0.0f) {
                    best_normal[0] = -best_normal[0];
                    best_normal[1] = -best_normal[1];
                    best_normal[2] = -best_normal[2];
                }
            }
            physx_contact_store(&contacts, best_normal, penetration);
            if (penetration > best_penetration) {
                best_penetration = penetration;
            }
        }
    }
    *out=contacts;
    return best_penetration>0;
}
