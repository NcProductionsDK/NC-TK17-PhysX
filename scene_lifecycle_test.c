#include "NC-TK17-PhysX.c"
#define require(ok) do { if(!(ok)) { fprintf(stderr,"FAIL %d: %s\n",__LINE__,#ok); exit(1); } } while(0)

static int lookups,callbacks,writes;
static float last_write[3];
static void *__cdecl forbidden_lookup(const char *name)
{ (void)name; lookups++; return NULL; }
static void __cdecl callback(void) { callbacks++; }
static void THISCALL rotation_write(void *self,const float *value)
{ (void)self; writes++; memcpy(last_write,value,sizeof(last_write)); }

static void activate_wind(void)
{
    room_wind_observe_scene("Luder\\Room\\Breezy",NULL,"test");
    run_room_wind_hot_reload(GetTickCount());
    require(room_wind_cfg.active && room_wind_is_enabled());
    sync_room_sidecar_scene_activity(GetTickCount());
    require(sidecars[0].addon_scene_active);
}

static void retired_room(void)
{
    physx_chain_t *chain=&sidecars[0].chains[0];
    require(!room_wind_cfg.active && !room_wind_is_enabled());
    require(!room_wind_observed_scene_key[0]);
    require(!sidecars[0].addon_scene_active && !chain->addon_scene_visible);
    require(!chain->targets[0].object && !chain->targets[0].s_rotation_base);
    require(!chain->targets[0].sim_initialized);
    require(sidecars[1].addon_scene_active); /* Equipped add-on is unrelated. */
}

int main(void)
{
    char file[MAX_PATH*4];
    require(GetFullPathNameA("wind-test.ini",sizeof(file),file,NULL));
    require(WritePrivateProfileStringA("wind","enabled","true",file));
    require(WritePrivateProfileStringA("wind","strength","0.14",file));
    room_wind_sidecar_count=1;
    lstrcpynA(room_wind_sidecars[0].path,file,sizeof(room_wind_sidecars[0].path));
    lstrcpyA(room_wind_sidecars[0].scene_key,"Luder\\Room\\Breezy");
    physics_environment_cfg.wind_enabled=1;
    sidecar_count=2;
    sidecars[0].loaded=sidecars[0].enabled=sidecars[0].room_scene_sidecar=1;
    sidecars[0].chain_count=1;
    lstrcpynA(sidecars[0].path,file,sizeof(sidecars[0].path));
    sidecars[1].addon_scene_active=1;
    physx_chain_t *chain=&sidecars[0].chains[0];
    chain->addon_chain=chain->simulate=1;
    chain->target_count=1;
    engine_FindObjC=forbidden_lookup;
    for(int repeat=0;repeat<4;repeat++) {
        physx_scene_lifecycle_command("NewGame_Start");
        activate_wind();
        physx_scene_lifecycle_command("Game_Resume");
        require(room_wind_is_enabled()); /* Opening/resuming a menu is not exit. */
        chain->addon_scene_visible=1;
        chain->targets[0].sim_initialized=1;
        /* Disposal must only forget these addresses, never restore through
           them: an old allocation may now be a WComboBox or already freed. */
        chain->targets[0].object=chain->targets[0].raw_object=(void*)1;
        chain->targets[0].s_rotation_base=(void*)1;
        chain->targets[0].addon_visual_pose_valid=1;
        physx_scene_lifecycle_command("Game_Abort");
        retired_room();
        physx_note_room_object_name_a("Luder/Room/Breezy/Breezy.ma");
        run_room_wind_hot_reload(GetTickCount());
        require(!room_wind_is_enabled()); /* Late outgoing names cannot revive it. */
        physx_scene_lifecycle_command("NewGame_Start");
        run_room_wind_hot_reload(GetTickCount());
        require(!room_wind_is_enabled()); /* No incoming room notification either. */
        room_wind_observe_scene("Luder\\Room\\Still",NULL,"test");
        run_room_wind_hot_reload(GetTickCount());
        require(!room_wind_is_enabled());
    }
    require(lookups==0);
    puts("PASS: windy room -> main menu -> room without wind; repeated wind re-entry; unrelated add-ons and menu resume preserved; stale pointers never dereferenced");

    physx_scene_lifecycle_command("NewGame_Start");
    activate_wind();
    physx_scene_lifecycle_command("Exit");
    retired_room();
    physx_tick();
    physx_late_frame_ownership_tick();
    physx_body_chain_apply_post_animation_ownership();
    require(!physx_body_chain_apply_traverse_overlay((void*)1,1));
    require(!physx_addon_apply_traverse_overlay((void*)1,1));
    physx_post_animation_callbacks[0]=(PVOID)callback;
    physx_public_run_post_animation_callbacks();
    require(callbacks==0 && lookups==0);
    require(hook_d3d8_Present(NULL,NULL,NULL,NULL,NULL)==D3DERR_INVALIDCALL);
    require(hook_d3d8_EndScene(NULL)==D3DERR_INVALIDCALL);
    require(!hook_SwapBuffers(NULL));
    physx_hook5_collision_composite((void*)1,(void*)1,NULL,800,600);
    tramp_RuntimeRotationVectorWrite=rotation_write;
    float value[3]={12,23,34};
    hook_RuntimeRotationVectorWrite((void*)1,value);
    require(writes==1 && !memcmp(last_write,value,sizeof(value)));
    require(!addon_constraint_target_owned(&sidecars[1],chain,&chain->targets[0]));
    require(!addon_animation_target_owned(&sidecars[0],chain,&chain->targets[0]));
    room_wind_observe_scene("Luder\\Room\\Breezy",NULL,"late-shutdown-name");
    run_room_wind_hot_reload(GetTickCount());
    require(!room_wind_is_enabled());
    require(DllMain(NULL,DLL_PROCESS_DETACH,NULL));
    require(lookups==0);
    puts("PASS: confirmed quit blocks physics, late ownership, overlays and callbacks; native rotation writes pass through; detach avoids dead engine objects");
    return 0;
}
