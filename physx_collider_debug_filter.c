/* Visualization selection only. Never used by collision queries or solvers. */
static int body_collider_debug_person; /* 0 = all; 1..4 = Person01..Person04 */
static const char *body_collider_debug_names[BODY_COLLIDER_NODE_COUNT] = {
    [BODY_COLLIDER_ROOT] = "pelvis",
    [BODY_COLLIDER_STOMACH_01] = "spine01",
    [BODY_COLLIDER_STOMACH_02] = "spine02",
    [BODY_COLLIDER_HIP_L] = "left_hip",
    [BODY_COLLIDER_HIP_R] = "right_hip",
    [BODY_COLLIDER_KNEE_L] = "left_knee",
    [BODY_COLLIDER_KNEE_R] = "right_knee",
    [BODY_COLLIDER_THIGH_L] = "left_thigh",
    [BODY_COLLIDER_THIGH_R] = "right_thigh",
    [BODY_COLLIDER_TESTICLES_01] = "testicles01",
    [BODY_COLLIDER_TESTICLES_02] = "testicles02",
    [BODY_COLLIDER_TESTICLES_MID] = "testicles_mid",
    [BODY_COLLIDER_STOMACH_03] = "spine03",
    [BODY_COLLIDER_STOMACH_04] = "spine04",
    [BODY_COLLIDER_NECK_01] = "neck",
    [BODY_COLLIDER_ANKLE_L] = "left_ankle",
    [BODY_COLLIDER_ANKLE_R] = "right_ankle",
    [BODY_COLLIDER_BALL_L] = "left_ball",
    [BODY_COLLIDER_BALL_R] = "right_ball",
    [BODY_COLLIDER_BREAST_L] = "left_breast",
    [BODY_COLLIDER_BREAST_R] = "right_breast",
    [BODY_COLLIDER_HEAD_02] = "head",
    [BODY_COLLIDER_CLAVICLE_L] = "left_clavicle",
    [BODY_COLLIDER_CLAVICLE_R] = "right_clavicle",
    [BODY_COLLIDER_SHOULDER_L] = "left_shoulder",
    [BODY_COLLIDER_SHOULDER_R] = "right_shoulder",
    [BODY_COLLIDER_ELBOW_L] = "left_elbow",
    [BODY_COLLIDER_ELBOW_R] = "right_elbow",
    [BODY_COLLIDER_FOREARM_L] = "left_forearm",
    [BODY_COLLIDER_FOREARM_R] = "right_forearm",
    [BODY_COLLIDER_WRIST_L] = "left_wrist",
    [BODY_COLLIDER_WRIST_R] = "right_wrist",
    [BODY_COLLIDER_PALM_L] = "left_palm",
    [BODY_COLLIDER_PALM_R] = "right_palm",
    [BODY_COLLIDER_FINGER01_L_01] = "left_finger01_01",
    [BODY_COLLIDER_FINGER01_L_02] = "left_finger01_02",
    [BODY_COLLIDER_FINGER01_L_03] = "left_finger01_03",
    [BODY_COLLIDER_FINGER01_L_END] = "left_finger01_end",
    [BODY_COLLIDER_FINGER01_R_01] = "right_finger01_01",
    [BODY_COLLIDER_FINGER01_R_02] = "right_finger01_02",
    [BODY_COLLIDER_FINGER01_R_03] = "right_finger01_03",
    [BODY_COLLIDER_FINGER01_R_END] = "right_finger01_end",
    [BODY_COLLIDER_FINGER02_L_01] = "left_finger02_01",
    [BODY_COLLIDER_FINGER02_L_02] = "left_finger02_02",
    [BODY_COLLIDER_FINGER02_L_03] = "left_finger02_03",
    [BODY_COLLIDER_FINGER02_L_04] = "left_finger02_04",
    [BODY_COLLIDER_FINGER02_L_END] = "left_finger02_end",
    [BODY_COLLIDER_FINGER02_R_01] = "right_finger02_01",
    [BODY_COLLIDER_FINGER02_R_02] = "right_finger02_02",
    [BODY_COLLIDER_FINGER02_R_03] = "right_finger02_03",
    [BODY_COLLIDER_FINGER02_R_04] = "right_finger02_04",
    [BODY_COLLIDER_FINGER02_R_END] = "right_finger02_end",
    [BODY_COLLIDER_FINGER03_L_01] = "left_finger03_01",
    [BODY_COLLIDER_FINGER03_L_02] = "left_finger03_02",
    [BODY_COLLIDER_FINGER03_L_03] = "left_finger03_03",
    [BODY_COLLIDER_FINGER03_L_04] = "left_finger03_04",
    [BODY_COLLIDER_FINGER03_L_END] = "left_finger03_end",
    [BODY_COLLIDER_FINGER03_R_01] = "right_finger03_01",
    [BODY_COLLIDER_FINGER03_R_02] = "right_finger03_02",
    [BODY_COLLIDER_FINGER03_R_03] = "right_finger03_03",
    [BODY_COLLIDER_FINGER03_R_04] = "right_finger03_04",
    [BODY_COLLIDER_FINGER03_R_END] = "right_finger03_end",
    [BODY_COLLIDER_FINGER04_L_01] = "left_finger04_01",
    [BODY_COLLIDER_FINGER04_L_02] = "left_finger04_02",
    [BODY_COLLIDER_FINGER04_L_03] = "left_finger04_03",
    [BODY_COLLIDER_FINGER04_L_04] = "left_finger04_04",
    [BODY_COLLIDER_FINGER04_L_END] = "left_finger04_end",
    [BODY_COLLIDER_FINGER04_R_01] = "right_finger04_01",
    [BODY_COLLIDER_FINGER04_R_02] = "right_finger04_02",
    [BODY_COLLIDER_FINGER04_R_03] = "right_finger04_03",
    [BODY_COLLIDER_FINGER04_R_04] = "right_finger04_04",
    [BODY_COLLIDER_FINGER04_R_END] = "right_finger04_end",
    [BODY_COLLIDER_FINGER05_L_01] = "left_finger05_01",
    [BODY_COLLIDER_FINGER05_L_02] = "left_finger05_02",
    [BODY_COLLIDER_FINGER05_L_03] = "left_finger05_03",
    [BODY_COLLIDER_FINGER05_L_04] = "left_finger05_04",
    [BODY_COLLIDER_FINGER05_L_END] = "left_finger05_end",
    [BODY_COLLIDER_FINGER05_R_01] = "right_finger05_01",
    [BODY_COLLIDER_FINGER05_R_02] = "right_finger05_02",
    [BODY_COLLIDER_FINGER05_R_03] = "right_finger05_03",
    [BODY_COLLIDER_FINGER05_R_04] = "right_finger05_04",
    [BODY_COLLIDER_FINGER05_R_END] = "right_finger05_end",
    [BODY_COLLIDER_BUTT_L] = "left_butt",
    [BODY_COLLIDER_BUTT_R] = "right_butt",
};

static int body_collider_debug_name_matches(const char *filter, const char *name)
{
    const char *unpaired = name;
    size_t length = strlen(filter);
    if (!_strnicmp(name, "left_", 5)) unpaired = name + 5;
    if (!_strnicmp(name, "right_", 6)) unpaired = name + 6;
    if (!_stricmp(filter, name) || !_stricmp(filter, unpaired)) return 1;
    /* Finger settings apply to every joint in the named finger. */
    return (!_strnicmp(filter, name, length) && name[length] == '_') ||
           (!_strnicmp(filter, unpaired, length) && unpaired[length] == '_');
}

static int body_collider_debug_set_filter(const char *value)
{
    char filter[128];
    char *comment;
    int i, matches = 0;
    lstrcpynA(filter, value, sizeof(filter));
    comment = strchr(filter, ';');
    if (comment) *comment = 0;
    trim_in_place(filter);
    if (!filter[0]) lstrcpyA(filter, "all");
    if (!_stricmp(filter, "root")) lstrcpyA(filter, "pelvis");
    if (!_stricmp(filter, "neck01")) lstrcpyA(filter, "neck");
    if (!_stricmp(filter, "head02")) lstrcpyA(filter, "head");
    body_chain_collider_cfg.debug_draw_filtered = _stricmp(filter, "all") != 0;
    body_chain_collider_cfg.debug_draw_chain = !_stricmp(filter, "penis");
    memset(body_chain_collider_cfg.debug_draw_nodes, 0,
           sizeof(body_chain_collider_cfg.debug_draw_nodes));
    if (!body_chain_collider_cfg.debug_draw_filtered) return 1;
    for (i = 0; i < BODY_COLLIDER_NODE_COUNT; i++) {
        int match = body_collider_debug_name_matches(filter, body_collider_debug_names[i]);
        if (!_stricmp(filter, "testicles") &&
            (i == BODY_COLLIDER_TESTICLES_01 || i == BODY_COLLIDER_TESTICLES_02)) match = 1;
        body_chain_collider_cfg.debug_draw_nodes[i] = (unsigned char)match;
        matches += match;
    }
    return matches || body_chain_collider_cfg.debug_draw_chain;
}

static void body_collider_debug_profile(const char *path, int overlay)
{
    char value[128];
    if (!overlay) {
        body_collider_debug_set_filter("all");
        body_chain_collider_cfg.debug_draw_capsules = 1;
        body_collider_debug_person = body_colliders_profile_int("debug_draw_person", 0, path);
        if (body_collider_debug_person < 0 || body_collider_debug_person > 4)
            body_collider_debug_person = 0;
    }
    body_chain_collider_cfg.debug_draw = body_colliders_profile_bool(
        "debug_draw", body_chain_collider_cfg.debug_draw, path);
    body_chain_collider_cfg.debug_draw_capsules = body_colliders_profile_bool(
        "debug_draw_capsules", body_chain_collider_cfg.debug_draw_capsules, path);
    if (body_colliders_profile_string("debug_draw_filter", "", value, sizeof(value), path) &&
        !body_collider_debug_set_filter(value)) {
        log_line("body-collider debug filter unknown value=\"%s\" path=\"%s\" note=\"body debug shapes hidden; collision physics unchanged\"", value, path);
    }
}

static int body_collider_debug_node_selected(int node)
{
    return node >= 0 && node < BODY_COLLIDER_NODE_COUNT &&
        (!body_chain_collider_cfg.debug_draw_filtered || body_chain_collider_cfg.debug_draw_nodes[node]);
}

static int body_collider_debug_edge_selected(int start, int end)
{
    return body_chain_collider_cfg.debug_draw_capsules &&
        (body_collider_debug_node_selected(start) || body_collider_debug_node_selected(end));
}

static int body_collider_debug_chain_selected(void)
{
    return body_chain_collider_cfg.debug_draw_capsules &&
        (!body_chain_collider_cfg.debug_draw_filtered || body_chain_collider_cfg.debug_draw_chain);
}

static int body_collider_debug_custom_view(void)
{
    return body_chain_collider_cfg.debug_draw_filtered || !body_chain_collider_cfg.debug_draw_capsules;
}

static int body_collider_debug_any(void)
{
    int person;
    for (person = 0; person < 4; person++) {
        if (body_collider_debug_person && body_collider_debug_person != person + 1) continue;
        if (body_chain_collider_person_cfg[person].enabled &&
            body_chain_collider_person_cfg[person].debug_draw) return 1;
    }
    return 0;
}
