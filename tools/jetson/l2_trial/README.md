# Held L2 trial launcher

`held_trial.launch.py` is the source-controlled successor of the November 6 metadata-corrected held launcher. It retains wheel-only bringup, corrected lidar overlay, dual-lidar filtering/obstacle inputs, A/B/C controller profiles, speed/steering limits, unknown-space checks, recorder, viewer and 15-minute session cap.

The localization changes are limited to requiring front AMCL throughout, removing the fixed-wall scan-mask process and its bag topics, and preventing automatic saved-pose initialization. A newly reviewed `packet_root` and `body_route_prefix` must be supplied; historical packets are not edited. The selected body-route build must retain all current scan-timing/controller guards. The lidar driver still requires `/home/turtle/nt_lidar_metadata_20261106_v1/install`.

The launcher starts no registration, initial-pose publisher, trial client or movement goal. Follow [the full-map procedure](../../../ros2_ws/src/body_route_controller/FULL_MAP_LOCALIZATION.md) after authorized held startup. A later deployment requires a separately reviewed packet/build; local preparation is not permission to upload or start hardware.
