/* Body-local collision geometry, separate from animation/solver pivots. */
static float body_chain_penis_radius(void)
{
    return body_chain_collider_cfg.penis_radius > 0.0f ?
        body_chain_collider_cfg.penis_radius : body_chain_collider_cfg.chain_radius;
}

static void body_chain_penis_offset_points(float points[4][3], float sign)
{
    int point, axis;
    for (point = 0; point < 4; point++) {
        int joint = point < 3 ? point : 2;
        for (axis = 0; axis < 3; axis++)
            points[point][axis] += sign * body_chain_collider_cfg.penis_fine_offset[joint][axis];
    }
}

static int body_penis_profile_value(const char *key, const char *path, char value[128])
{
    char *comment;
    if (!body_colliders_profile_string(key, "", value, 128, path)) return 0;
    comment = strchr(value, ';');
    if (comment) *comment = 0;
    trim_in_place(value);
    return 1;
}

static void body_penis_profile_radius(const char *key, const char *legacy,
                                      float *radius, const char *path)
{
    char value[128], *end;
    float parsed;
    if (!body_penis_profile_value(key, path, value) &&
        (!legacy || !body_penis_profile_value(legacy, path, value))) return;
    parsed = strtof(value, &end);
    while (*end && isspace((unsigned char)*end)) end++;
    if (end == value || *end || !isfinite(parsed)) {
        log_line("body-collider ignored invalid scalar key=\"%s\" path=\"%s\"", key, path);
        return;
    }
    *radius = physx_clampf(parsed, 0.001f, 0.25f);
}

static void body_penis_profile(const char *path, int overlay)
{
    int joint;
    if (!overlay) {
        body_chain_collider_cfg.chain_radius = 0.018f;
        body_chain_collider_cfg.penis_radius = 0.018f;
        memset(body_chain_collider_cfg.penis_fine_offset, 0,
               sizeof(body_chain_collider_cfg.penis_fine_offset));
    }
    body_penis_profile_radius("penis_radius", "chain_radius",
                              &body_chain_collider_cfg.penis_radius, path);
    body_penis_profile_radius("collision_margin_radius", "chain_radius",
                              &body_chain_collider_cfg.chain_radius, path);
    for (joint = 0; joint < 3; joint++) {
        char key[48], value[128], extra;
        float xyz[3];
        wsprintfA(key, "penis%02d_fine_offset", joint + 1);
        if (!body_penis_profile_value(key, path, value)) continue;
        if ((sscanf(value, "%f , %f , %f %c", &xyz[0], &xyz[1], &xyz[2], &extra) != 3 &&
             sscanf(value, "%f %f %f %c", &xyz[0], &xyz[1], &xyz[2], &extra) != 3) ||
            !isfinite(xyz[0]) || !isfinite(xyz[1]) || !isfinite(xyz[2])) {
            log_line("body-collider ignored invalid offset key=\"%s\" path=\"%s\"", key, path);
            continue;
        }
        memcpy(body_chain_collider_cfg.penis_fine_offset[joint], xyz, sizeof(xyz));
    }
}
