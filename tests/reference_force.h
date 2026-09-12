/* Original force assembly, included inside the test reference function. */
if (addon_local_bend) {
                        if (addon_world_gravity_valid) {
                            /* Retain the gravity-only sample for the existing
                               diagnostic report; simulation uses the combined
                               resultant below. */
                            addon_gravity_bend_valid =
                                addon_chain_world_gravity_bend_vector(
                                    chain, target,
                                    addon_world_gravity_drive,
                                    addon_gravity_bend);
                            if (target->gravity_inverted_configured) {
                                addon_gravity_bend_valid =
                                    addon_chain_apply_inverted_gravity_bend(
                                        chain, target,
                                        addon_world_gravity_drive,
                                        1.0f,
                                        addon_gravity_bend);
                            }
                            for (axis = 0; axis < 3; axis++) {
                                addon_force_drive[axis] +=
                                    addon_world_gravity_drive[axis] *
                                    chain->gravity_scale;
                            }
                        }
                        if (addon_world_wind_valid) {
                            addon_wind_strength = room_wind_target_strength(
                                chain, target, now);
                            for (axis = 0; axis < 3; axis++) {
                                addon_force_drive[axis] +=
                                    addon_world_wind_drive[axis] *
                                    addon_wind_strength;
                            }
                        }
                        /* Map and constrain the physical resultant once.
                           Mapping gravity and wind independently and adding
                           the two constrained bends produced incorrect
                           sideways/prone responses. */
                        if (addon_world_gravity_valid ||
                            (addon_world_wind_valid &&
                             physx_absf(addon_wind_strength) > 0.000001f)) {
                            addon_force_bend_valid =
                                addon_chain_world_gravity_bend_vector(
                                    chain, target, addon_force_drive,
                                    addon_force_bend);
                            if (addon_world_gravity_valid &&
                                target->gravity_inverted_configured) {
                                addon_force_bend_valid =
                                    addon_chain_apply_inverted_gravity_bend(
                                        chain, target,
                                        addon_world_gravity_drive,
                                        chain->gravity_scale,
                                        addon_force_bend);
                            }
                        }
                    }
