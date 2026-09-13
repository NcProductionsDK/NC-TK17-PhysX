#ifndef PHYSX_COLLISION_PROFILE_H
#define PHYSX_COLLISION_PROFILE_H

/* Standalone physics fixtures compile the same algorithms with instrumentation
   erased. The plugin enables this header before including its physics modules. */
#ifndef PHYSX_COLLISION_PROFILE_ENABLED
#define COLLISION_PROFILE_SCOPE(name, phase) ((void)0)
#define COLLISION_PROFILE_END(name) ((void)0)
#define COLLISION_PROFILE_COUNT(counter, amount) ((void)0)
#else

typedef enum collision_profile_phase_t {
    CP_BODY_COLLECTION, CP_BODY_SOLVE, CP_BODY_POSITION, CP_BODY_REFINE,
    CP_BODY_VELOCITY, CP_ADDON_PREPARE_QUERY, CP_ADDON_COHERENT_SOLVE,
    CP_ADDON_VISIBLE_SOLVE, CP_ROOM_SPHERE, CP_ROOM_SWEEP,
    CP_ROOM_SINGLE, CP_PHASE_COUNT
} collision_profile_phase_t;

typedef enum collision_profile_counter_t {
    CP_BODY_CONTACTS, CP_BODY_PASSES, CP_BODY_JACOBIANS, CP_BODY_PREDICTIONS,
    CP_ADDON_CONTACTS, CP_ADDON_PASSES, CP_ADDON_GRADIENTS,
    CP_ROOM_NODES, CP_ROOM_TRIANGLES, CP_COUNTER_COUNT
} collision_profile_counter_t;

static struct {
    int enabled;
    LONGLONG ticks[CP_PHASE_COUNT], maximum[CP_PHASE_COUNT];
    unsigned long long calls[CP_PHASE_COUNT], counts[CP_COUNTER_COUNT];
} collision_profile_state;

typedef struct collision_profile_scope_t {
    LONGLONG start;
    collision_profile_phase_t phase;
} collision_profile_scope_t;

static void collision_profile_clear(void)
{
    memset(&collision_profile_state, 0, sizeof(collision_profile_state));
}

static collision_profile_scope_t collision_profile_begin(collision_profile_phase_t phase)
{
    collision_profile_scope_t scope = {0, phase};
    if (collision_profile_state.enabled) {
        LARGE_INTEGER now;
        QueryPerformanceCounter(&now);
        scope.start = now.QuadPart;
    }
    return scope;
}

static void collision_profile_end(collision_profile_scope_t *scope)
{
    if (scope->start && collision_profile_state.enabled) {
        LARGE_INTEGER now;
        LONGLONG elapsed;
        QueryPerformanceCounter(&now);
        elapsed = now.QuadPart - scope->start;
        if (elapsed >= 0) {
            collision_profile_state.ticks[scope->phase] += elapsed;
            if (elapsed > collision_profile_state.maximum[scope->phase])
                collision_profile_state.maximum[scope->phase] = elapsed;
            collision_profile_state.calls[scope->phase]++;
        }
    }
    scope->start = 0; /* Explicit END and scope cleanup cannot double-count. */
}

/* GCC cleanup accounts for every early return without changing control flow. */
#define COLLISION_PROFILE_SCOPE(name, phase) \
    collision_profile_scope_t name __attribute__((cleanup(collision_profile_end))) = \
        collision_profile_begin(phase)
#define COLLISION_PROFILE_END(name) collision_profile_end(&(name))
#define COLLISION_PROFILE_COUNT(counter, amount) do { \
    if (collision_profile_state.enabled) \
        collision_profile_state.counts[counter] += (unsigned long long)(amount); \
} while (0)

static void collision_profile_report(DWORD window_ms, DWORD frames, LONGLONG frequency)
{
    static const char *names[CP_PHASE_COUNT] = {
        "body_collection", "body_solve", "body_position", "body_refine",
        "body_velocity", "addon_prepare_query", "addon_coherent_solve",
        "addon_visible_solve", "room_sphere", "room_sweep", "room_single"
    };
    static const char *counter_names[CP_COUNTER_COUNT] = {
        "body_contacts", "body_passes", "body_jacobians", "body_predictions",
        "addon_contacts", "addon_passes", "addon_gradients",
        "room_nodes", "room_triangles"
    };
    char line[2048];
    size_t used;
    int i;
    if (!collision_profile_state.enabled || !frames || frequency <= 0) return;
    /* Separate records keep the established performance-profile line intact.
       Maxima are per invocation; averages include all invocations per tick. */
    for (int metric = 0; metric < 3; metric++) {
        used = (size_t)snprintf(line, sizeof(line),
            "window_ms=%lu frames=%lu %s",
            (unsigned long)window_ms, (unsigned long)frames,
            metric == 0 ? "avg_ms" : metric == 1 ? "max_call_ms" : "calls");
        for (i = 0; i < CP_PHASE_COUNT && used < sizeof(line); i++) {
            int written;
            if (metric == 2)
                written = snprintf(line + used, sizeof(line) - used, " %s=%llu",
                    names[i], collision_profile_state.calls[i]);
            else
                written = snprintf(line + used, sizeof(line) - used, " %s=%.6f",
                    names[i], (double)(metric == 0 ? collision_profile_state.ticks[i] :
                    collision_profile_state.maximum[i]) * 1000.0 / (double)frequency /
                    (metric == 0 ? (double)frames : 1.0));
            if (written < 0) break;
            used += (size_t)written;
        }
        log_line("collision profile %s", line);
    }
    used = (size_t)snprintf(line, sizeof(line),
        "window_ms=%lu frames=%lu counts",
        (unsigned long)window_ms, (unsigned long)frames);
    for (i = 0; i < CP_COUNTER_COUNT && used < sizeof(line); i++) {
        int written = snprintf(line + used, sizeof(line) - used, " %s=%llu",
            counter_names[i], collision_profile_state.counts[i]);
        if (written < 0) break;
        used += (size_t)written;
    }
    log_line("collision profile %s", line);
}
#endif
#endif
