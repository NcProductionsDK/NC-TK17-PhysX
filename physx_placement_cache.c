/* Solver resets do not change room placement. Retain only a confirmed room
   reference, never springs, bone samples, contact history or output snapshots.
   A conservative byte comparison of STRS_group's authored transform properties
   (rotation, translation, scale and pivots) avoids trusting the same pointer
   after its placement was edited. Unknown bindings always take the cold path. */
typedef struct body_placement_cache_t {
    void *root, *trs, *source;
    LONG generation;
    /* Shipped STransform::Create allocates 0x120 bytes; its local property
       block begins with scale at +0x60, rotation +0x6c, translation +0x7c,
       and extends through the pivot fields to +0x11c. Exclude its cached
       matrix/version header. Verified against SYS constructors e8cf0/e3460. */
    BYTE properties[0xc0];
    float world_gravity[3];
    gravity_sample_t gravity;
    collision_frame_sample_t collision;
    float local_basis[9];
    int local_basis_valid;
    unsigned int gravity_revision;
    DWORD change_log_tick;
} body_placement_cache_t;

static body_placement_cache_t body_placement_cache[3][4];
static int body_chain_runtime_mode_active(void);

static int body_placement_uniform_basis(float basis[9])
{
    float lengths[3];
    for (int r=0;r<3;r++) {
        lengths[r]=physx_vec3_len(basis+r*3);
        if (!isfinite(lengths[r]) || lengths[r]<.000001f || lengths[r]>8.f) return 0;
    }
    for (int r=0;r<3;r++) {
        if (fabsf(lengths[r]/lengths[0]-1.f)>.0001f) return 0;
        for (int c=0;c<3;c++) basis[r*3+c]/=lengths[r];
    }
    return fabsf(vec3_dot(basis,basis+3))<.0001f &&
        fabsf(vec3_dot(basis,basis+6))<.0001f &&
        fabsf(vec3_dot(basis+3,basis+6))<.0001f;
}

/* STransform caches its camera-independent local matrix at raw+0x18.
   The native matrix consumer at SYS+dac70 checks raw+0x5c: 0 is dirty,
   1 identity, 2 computed. Never evaluate a dirty source from a physics hook.
   Validate the shipped vtable before using this native layout. */
static void *body_placement_source_vtable;
static int body_placement_local_basis(void *source, float basis[9])
{
    BYTE *raw = (BYTE*)source;
    if (!body_placement_source_vtable) {
        BYTE *sys = (BYTE*)GetModuleHandleA("ThriXXX010278-SYS.dll");
        static const BYTE consumer[] = {0x55,0x8b,0xec,0x56,0x8b,0xf1,0x8b,0x46,0x5c};
        if (!sys || !ptr_readable(sys+0xdac70,sizeof(consumer)) ||
            memcmp(sys+0xdac70,consumer,sizeof(consumer)) ||
            !ptr_readable(sys+0x16a310,sizeof(void*)) ||
            *(void**)(sys+0x16a310) != sys+0xdac70) return 0;
        body_placement_source_vtable = sys+0x16a30c;
    }
    if (!ptr_readable(raw,0x60) || *(void**)raw != body_placement_source_vtable) return 0;
    DWORD kind = *(DWORD*)(raw+0x5c);
    if (kind != 1 && kind != 2) return 0;
    for (int r=0;r<3;r++) for (int c=0;c<3;c++)
        basis[r*3+c] = kind == 1 ? (float)(r==c) : *(float*)(raw+0x18+r*16+c*4);
    /* Nonuniform scale/shear needs a different normal transform. Leave it
       on the established camera-quiet acquisition path. */
    return body_placement_uniform_basis(basis);
}

static int body_placement_rebase_gravity(body_placement_cache_t *cache,
                                        const float current[9])
{
    float inverse[9], delta[9], down[3];
    if (!cache->gravity.trusted_valid || !cache->local_basis_valid ||
        !body_chain_mat3_inverse(cache->local_basis,inverse)) return 0;
    body_chain_mat3_multiply(current,inverse,delta);
    for (int r=0;r<3;r++) down[r]=vec3_dot(delta+r*3,cache->gravity.trusted);
    float length=physx_vec3_len(down);
    if (!isfinite(length) || fabsf(length-1.f)>.001f) return 0;
    for (int r=0;r<3;r++) cache->gravity.trusted[r]=down[r]/length;
    cache->gravity.pending_valid=cache->gravity.processed=cache->gravity.accepted=0;
    cache->gravity_revision++;
    return 1;
}

static body_placement_cache_t *body_placement_cache_for(const char *person, void *trs)
{
    char name[256];
    void *root, *source;
    int p;
    LONG generation = InterlockedCompareExchange(&named_node_generation, 0, 0);
    body_placement_cache_t *cache;
    for (p = 0; p < 4; ++p)
        if (person && !strcmp(person, body_chain_person_name(p))) break;
    if (p == 4 || !trs) return NULL;
    cache = &body_placement_cache[body_chain_runtime_mode_active() ?
        (physx_customizer_active ? 2 : 1) : 0][p];
    make_body_runtime_name(name, sizeof(name), person, "root");
    root = resolve_axis_map_raw(name);
    make_body_runtime_name(name, sizeof(name), person, "STRS_group");
    source = resolve_axis_map_raw(name);
    if (!root || !source || !ptr_readable((BYTE*)source + 0x060, sizeof(cache->properties))) {
        if (defaults_cfg.debug && (cache->gravity.trusted_valid || cache->collision.trusted_valid))
            log_line("physics-environment placement-invalidated person=\"%s\" mode=%s customizer=%d reason=unavailable-binding root=%p trs=%p source=%p gravity=%d collision=%d",
                person, body_chain_runtime_mode_active() ? "runtime" : "PoseEditor",
                physx_customizer_active, root, trs, source,
                cache->gravity.trusted_valid, cache->collision.trusted_valid);
        memset(cache, 0, sizeof(*cache));
        return NULL;
    }
    /* The engine has changed properties but has not published the matching
       local matrix yet. Keep the old calibration intact for the next frame. */
    if (body_placement_source_vtable && *(void**)source==body_placement_source_vtable &&
        *(DWORD*)((BYTE*)source+0x5c)==0) return NULL;
    if (cache->root != root || cache->trs != trs || cache->source != source ||
        cache->generation != generation ||
        memcmp(cache->world_gravity, physics_environment_cfg.world_gravity, sizeof(cache->world_gravity))) {
        if (defaults_cfg.debug && cache->root)
            log_line("physics-environment placement-invalidated person=\"%s\" mode=%s customizer=%d reason=binding-generation-or-gravity root=(%p,%p) trs=(%p,%p) source=(%p,%p) generation=(%ld,%ld) world_gravity_changed=%d gravity=%d collision=%d",
                person, body_chain_runtime_mode_active() ? "runtime" : "PoseEditor",
                physx_customizer_active, cache->root, root, cache->trs, trs,
                cache->source, source, (long)cache->generation, (long)generation,
                memcmp(cache->world_gravity, physics_environment_cfg.world_gravity,
                    sizeof(cache->world_gravity)) != 0,
                cache->gravity.trusted_valid, cache->collision.trusted_valid);
        memset(cache, 0, sizeof(*cache));
        cache->root = root; cache->trs = trs; cache->source = source;
        cache->generation = generation;
        memcpy(cache->properties, (BYTE*)source + 0x060, sizeof(cache->properties));
        memcpy(cache->world_gravity, physics_environment_cfg.world_gravity, sizeof(cache->world_gravity));
    } else if (memcmp(cache->properties, (BYTE*)source + 0x060, sizeof(cache->properties))) {
        /* A new pose often moves the person without changing their room
           orientation. Translation cannot change down. Invalidate contact
           placement, but retain direction when ALL other properties match. */
        const BYTE *properties = (BYTE*)source + 0x060;
        int orientation_changed = memcmp(cache->properties, properties, 0x1c) != 0 ||
            memcmp(cache->properties + 0x28, properties + 0x28,
                sizeof(cache->properties) - 0x28) != 0;
        float current[9];
        float live_basis[9];
        int rebased=0;
        /* Rotation/translation may change when a pose places the person.
           Move the confirmed down direction through the local rotation
           change, rather than discarding it and waiting for the camera.
           Scale, rotation order, limits and pivots must remain identical. */
        if (orientation_changed && !memcmp(cache->properties,properties,0x0c) &&
            !memcmp(cache->properties+0x18,properties+0x18,4) &&
            !memcmp(cache->properties+0x28,properties+0x28,sizeof(cache->properties)-0x28) &&
            body_placement_local_basis(source,current) &&
            body_chain_read_mat3_rows(trs,live_basis) && body_placement_uniform_basis(live_basis))
            rebased=body_placement_rebase_gravity(cache,current);
        DWORD now=GetTickCount();
        if (defaults_cfg.debug && (!cache->change_log_tick || now-cache->change_log_tick>=1000u) &&
            ((cache->gravity.trusted_valid && orientation_changed) || cache->collision.trusted_valid)) {
            cache->change_log_tick=now;
            size_t first = 0;
            while (first < sizeof(cache->properties) && cache->properties[first] == properties[first]) first++;
            log_line("physics-environment placement-change person=\"%s\" mode=%s customizer=%d reason=authored-properties first_offset=0x%03x orientation_properties_changed=%d gravity_rebased=%d gravity=%d collision=%d",
                person, body_chain_runtime_mode_active() ? "runtime" : "PoseEditor",
                physx_customizer_active, (unsigned int)(0x60 + first),
                orientation_changed, rebased,
                cache->gravity.trusted_valid, cache->collision.trusted_valid);
        }
        if (orientation_changed && !rebased)
            memset(&cache->gravity, 0, sizeof(cache->gravity));
        memset(&cache->collision, 0, sizeof(cache->collision));
        memcpy(cache->properties, properties, sizeof(cache->properties));
    }
    float live_basis[9];
    cache->local_basis_valid=body_placement_local_basis(source,cache->local_basis) &&
        body_chain_read_mat3_rows(trs,live_basis) && body_placement_uniform_basis(live_basis);
    return cache;
}
