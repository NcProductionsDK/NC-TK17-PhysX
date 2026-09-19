#include "NC-TK17-PhysX.c"

static void require(int condition, const char *message)
{
    if (!condition) { fprintf(stderr, "FAIL: %s\n", message); exit(1); }
}

static void setup(body_chain_collider_person_state_t *s, int editor)
{
    memset(s, 0, sizeof(*s));
    body_chain_poseeditor_mode_active = editor;
    defaults_cfg.debug = body_chain_collider_cfg.diagnostic = 0;
    body_chain_collider_cfg.root_local_offsets = 1;
    physics_environment_cfg.gravity_probe_motion_epsilon = .005f;
    physics_environment_cfg.gravity_probe_settle_ms = 1500;
    s->basis_valid = s->limb_points_ready = s->chain_points_ready = 1;
    s->active_scope_mask = BODY_CHAIN_COLLIDER_GROUP_FULL_BODY;
    for (int i = 0; i < BODY_COLLIDER_NODE_COUNT; i++) s->valid[i] = 1;
}

static int sample(body_chain_collider_person_state_t *s, DWORD elapsed,
                  int reset, int moving_camera)
{
    float root[3] = {0, 0, 3};
    if (moving_camera) {
        root[0] = sinf(elapsed * .01f);
        root[1] = cosf(elapsed * .013f);
        root[2] += elapsed * .001f;
    }
    /* Small genuine bone movement, expressed in body coordinates. */
    s->local_position[BODY_COLLIDER_HIP_L][0] = sinf(elapsed * .001f) * .01f;
    physx_simulation_serial++;
    return body_chain_collider_update_settle_gate(s, "Person01", 1000 + elapsed,
        1, reset, 1, root);
}

static void editor_startup(int hz, int moving_camera)
{
    body_chain_collider_person_state_t s;
    DWORD step = 1000 / hz, elapsed = 0;
    setup(&s, 1);
    for (; elapsed < BODY_CHAIN_COLLIDER_SETTLE_MS; elapsed += step)
        require(!sample(&s, elapsed, 0, moving_camera), "editor startup interval shortened");
    require(sample(&s, elapsed, 0, moving_camera),
        "coherent body-local collision still waits for view-space root/camera quiet");
    require(s.ready, "collider readiness not published");
    /* Camera motion must not disable already confirmed body colliders. */
    for (int i = 0; i < 30; i++) {
        elapsed += step;
        require(sample(&s, elapsed, 0, 1), "ready collision disabled by camera movement");
    }
    printf("PASS: PoseEditor collision ready after unchanged startup interval at %d Hz, camera_motion=%d\n", hz, moving_camera);
}

static void rejection_and_recovery(void)
{
    body_chain_collider_person_state_t s;
    setup(&s, 1);
    for (DWORD t = 0; t < 640; t += 16) require(!sample(&s, t, 0, 1), "early readiness");
    s.local_position[BODY_COLLIDER_KNEE_L][0] = 1;
    require(!sample(&s, 640, 0, 1), "discontinuous collider accepted");
    require(s.settle_start_tick == 1640, "jump did not restart confirmation");
    for (DWORD t = 656; t < 1290; t += 16) require(!sample(&s, t, 0, 1), "jump released too early");
    require(sample(&s, 1296, 0, 1), "coherent new pose did not recover");
    require(!sample(&s, 1312, 1, 1), "replacement root kept readiness");
    s.valid[BODY_COLLIDER_KNEE_L] = 0;
    require(!sample(&s, 1328, 0, 1), "missing required node accepted");
    s.valid[BODY_COLLIDER_KNEE_L] = 1;
    s.limb_points_ready = 0;
    require(!sample(&s, 1344, 0, 1) && !s.sample_ready, "missing live limbs accepted");
    s.limb_points_ready = 1; s.chain_points_ready = 0;
    require(!sample(&s, 1360, 0, 1), "missing required chain points accepted");
    s.chain_points_ready = 1;
    require(!sample(&s, 1376, 0, 1), "sample regain skipped startup");
    for (DWORD t = 1392; t < 2026; t += 16) require(!sample(&s, t, 0, 1), "sample regain released early");
    require(sample(&s, 2032, 0, 1), "sample regain did not recover during camera movement");
    puts("PASS: jumps, replacement roots, missing nodes/limbs/chains and sample regain retain startup protection");
}

static void legacy_and_runtime(void)
{
    body_chain_collider_person_state_t s;
    setup(&s, 1); s.basis_valid = 0;
    for (DWORD t = 0; t <= 672; t += 16)
        require(!sample(&s, t, 0, 1), "unconfirmed body basis used new startup path");
    setup(&s, 1); body_chain_collider_cfg.root_local_offsets = 0;
    for (DWORD t = 0; t <= 672; t += 16)
        require(!sample(&s, t, 0, 1), "raw scan used new startup path");
    setup(&s, 0);
    require(!sample(&s, 0, 0, 1) && !sample(&s, 16, 0, 1), "runtime startup shortened");
    require(sample(&s, 32, 0, 1), "runtime three-sample readiness changed");
    puts("PASS: unconfirmed/raw sources retain legacy gate; FreeMode readiness unchanged");
}

int main(void)
{
    const int rates[] = {30, 60, 144};
    for (int i = 0; i < 3; i++) for (int camera = 0; camera < 2; camera++)
        editor_startup(rates[i], camera);
    rejection_and_recovery();
    legacy_and_runtime();
    return 0;
}
