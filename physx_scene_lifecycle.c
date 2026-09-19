/* Called before native room disposal, while no cached engine pointer needs
   to be touched. Readable memory alone is not proof that a room bone lives:
   the allocator can already have reused it for a GUI object. */
static void physx_scene_lifecycle_command(const char *name)
{
    int exiting = !strcmp(name, "Exit");
    int starting = !strcmp(name, "NewGame_Start");
    if (!exiting && !starting && strcmp(name, "Game_Abort")) return;
    if (exiting) physx_shutting_down = 1;
    room_wind_observations_blocked = !starting || physx_shutting_down;
    room_wind_clear_active(name);
    room_wind_observed_scene_key[0] = 0;
    room_wind_observed_package_prefix[0] = 0;
    room_wind_observation_generation++;
    if (!room_wind_observation_generation) room_wind_observation_generation=1;
    room_wind_applied_observation_generation = room_wind_observation_generation;
    room_wind_hot_reload_tick = 0;
    memset(&room_wind_state, 0, sizeof(room_wind_state));
    for (int i = 0; i < sidecar_count; i++) {
        physx_sidecar_t *sc = &sidecars[i];
        if (!sc->room_scene_sidecar) continue;
        sc->addon_scene_active = 0;
        sc->addon_scene_active_tick = 0;
        for (int c = 0; c < sc->chain_count; c++) {
            physx_chain_t *chain = &sc->chains[c];
            chain->addon_scene_visible = 0;
            addon_chain_reset_runtime_state(chain);
            chain->addon_root_settle_until_tick = 0;
        }
    }
    log_line("physics room lifecycle command=%s note=\"wind and room bone bindings retired before native disposal; shutdown=%d\"",
        name, physx_shutting_down);
}
