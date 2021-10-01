# JSK Spot System

System descriptions of JSK Version of Spot.

## System Diagram

TODO

## Systemd services

Currently, there are systemd service below.
Each service will load `/var/lib/robot/config.bash` for robot-specific settings before launching.

- ros related services
  - [roscore.service](./jsk_spot_startup/services/roscore.service)
    - A service to run roscore
    - automatically start when booting
  - [jsk-spot-ros-app-manager.service](./jsk_spot_startup/services/jsk-spot-ros-app-manager.service)
    - A service to run [`jsk_spot_startup app_manager.launch`](./jsk_spot_startup/launch/include/app_manager.launch)
    - automatically start when booting
  - [jsk-spot-ros-behavior-manager.service](./jsk_spot_startup/services/jsk-spot-ros-behavior-manager.service)
    - A service to run [`spot_behavior_manager_demo demo.launch`](./jsk_spot_behaviors/spot_behavior_manager_demo/launch/demo.launch)
    - automatically start when booting
  - [jsk-spot-ros-driver.service](./jsk_spot_startup/services/jsk-spot-ros-driver.service)
    - A service to run [`jsk_spot_startup driver.launch`](./jsk_spot_startup/launch/include/driver.launch)
    - automatically start when booting
  - [jsk-spot-ros-image-processing.service](./jsk_spot_startup/services/jsk-spot-ros-image-processing.service)
    - A service to run [`jsk_spot_startup image_processing.launch`](./jsk_spot_startup/launch/include/image_processing.launch)
    - automatically start when booting
  - [jsk-spot-ros-interaction.service](./jsk_spot_startup/services/jsk-spot-ros-interaction.service)
    - A service to run [`jsk_spot_startup interaction.launch`](./jsk_spot_startup/launch/include/interaction.launch)
    - automatically start when booting
  - [jsk-spot-ros-object-detection.service](./jsk_spot_startup/services/jsk-spot-ros-object-detection.service)
    - A service to run [`jsk_spot_startup object_detection.launch`](./jsk_spot_startup/launch/include/object_detection.launch)
    - automatically start when booting
  - [jsk-spot-ros-peripheral.service](./jsk_spot_startup/services/jsk-spot-ros-peripheral.service)
    - A service to run [`jsk_spot_startup peripheral.launch`](./jsk_spot_startup/launch/include/peripheral.launch)
    - automatically start when booting
  - [jsk-spot-ros-teleop.service](./jsk_spot_startup/services/jsk-spot-ros-teleop.service)
    - A service to run [`jsk_spot_startup teleop.launch`](./jsk_spot_startup/launch/include/teleop.launch)
    - automatically start when booting

- utility services
  - [jsk-spot-utils-network.service](./jsk_spot_startup/services/jsk-spot-utils-network.service)
    - A service to run [`jsk_spot_startup update-network-connection.sh`](./jsk_spot_startup/scripts/update-network-connection.sh)
    - Update network connection so that internet connection will be maintained with ethernet, wifi or LTE.
    - automatically start when booting
  - [jsk-spot-utils-update-workspace.service](./jsk_spot_startup/services/jsk-spot-utils-update-workspace.service)
    - A service to run [`jsk_spot_startup update-workspaces.sh`](./jsk_spot_startup/scripts/update-workspaces.sh)
    - Update `/home/spot/spot_driver_ws`, `/home/spot/spot_coral_ws` and `/home/spot/spot_ws`.
    - start at 6:00 am

