# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project context

This repository is a ROS 2 workspace `src` tree for an Orange Pi-based robot/drone stack. The local WSL checkout is the source of truth during normal development; the remote board workspace is a mirror used for real builds and runtime verification. See `docs/remote-dev.md` for the full WSL/rsync/SSH workflow.

## Common commands

### Local/remote development workflow

```bash
# one-time setup if scripts/remote.env is missing
cp scripts/remote.env.example scripts/remote.env

# preview local -> board sync; do this before first destructive sync
scripts/rsync-to-remote.sh --dry-run

# sync local source to the board
scripts/rsync-to-remote.sh

# sync, then build on the remote board from REMOTE_WORK_DIR
scripts/remote-build.sh

# run REMOTE_RUN_CMD on the board without syncing
scripts/remote-run.sh

# sync, then run REMOTE_RUN_CMD on the board
scripts/remote-run.sh --sync

# open an interactive shell on the board
scripts/remote-shell.sh
```

`/push-remote` is available as a Claude command and runs `scripts/push-remote.sh`, which performs a dry-run and then pushes local source to the board. The rsync push uses `--delete`; keep `REMOTE_SRC_DIR` pointed at the remote `src` directory, not the workspace root or a broad home directory.

### ROS 2 build and test commands

On the board, build from the ROS 2 workspace root, not this `src` directory:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

From a workspace root containing this source tree:

```bash
# build only selected packages
colcon build --symlink-install --packages-select activity_control_pkg pid_control_pkg uart_to_stm32

# run all tests for a package
colcon test --packages-select my_launch
colcon test-result --verbose

# run a single pytest-style test by name/pattern
colcon test --packages-select my_launch --pytest-args -k test_flake8
colcon test-result --verbose

# run lint/test hooks for CMake packages that declare ament_lint_auto
colcon test --packages-select laser_array_pkg my_carto_pkg
```

### Launch and node commands

```bash
# fixed waypoint mission
ros2 launch my_launch fixed_waypoint.launch.py

# warehouse inventory mission
ros2 launch my_launch warehouse_inventory.launch.py

# camera source only
ros2 launch visual_pkg camera_source.launch.py

# mapping/localization stack only
ros2 launch my_carto_pkg fly_carto.launch.py use_rviz:=false

# UART bridge only
ros2 launch uart_to_stm32 uart_to_stm32.launch.py

# position PID only
ros2 launch pid_control_pkg position_pid_controller.launch.py

# laser array ground-height estimator only
ros2 launch laser_array_pkg laser_array_ground.launch.py
```

Mission launch files set `ROS_LOG_DIR` to `mylog/<task_name>/` under the workspace root.

## Architecture overview

The code is organized around reusable low-level capabilities, mission/task nodes, and user-facing launch files.

### Launch composition

`my_launch` is the top-level operator entry point. Mission launch files include lower-level launches and start mission nodes after hardware/localization startup delays:

- `my_carto_pkg/launch/fly_carto.launch.py` starts the BlueSea lidar driver, `robot_state_publisher`, Cartographer, occupancy grid publishing, and optionally RViz.
- `uart_to_stm32/launch/uart_to_stm32.launch.py` starts the STM32 serial bridge.
- `laser_array_pkg/launch/laser_array_ground.launch.py` starts the 64-beam laser-array ground-height estimator.
- `visual_pkg/launch/camera_source.launch.py` starts a reusable camera publisher.
- `pid_control_pkg/launch/position_pid_controller.launch.py` starts the position/visual PID controller.
- `activity_control_pkg` task executables implement mission-specific state machines.

Mission tuning parameters are intentionally kept near the top of launch files (for example `MISSION_STEPS` in `fixed_waypoint.launch.py`, camera topics/devices and visual PID overrides in `warehouse_inventory.launch.py`).

### Main runtime data flow

- Localization/mapping: `bluesea2` + `my_carto_pkg` publish lidar-derived localization, map data, and TF frames. The common control frame pair is `map` -> `laser_link`.
- Mission/task logic: `activity_control_pkg` task nodes publish `/target_position`, `/active_controller`, `/mission_complete`, and auxiliary command topics such as `/arm_command`, `/magnet_command`, and `/signal_command`.
- Position control: `pid_control_pkg` subscribes to `/target_position`, `/height`, TF, and optional visual fine-control inputs; it publishes `/target_velocity` as a `Float32MultiArray` velocity command.
- STM32 bridge: `uart_to_stm32` receives `/target_velocity`, aux commands, mission-complete signals, barcode/on-pillar status, and height-source inputs; it writes protocol frames to the STM32 and publishes `/height`, `/height_raw`, `/is_st_ready`, and `/mission_step`.
- Height sensing: `laser_array_pkg` publishes `/laser_array/ground_height`, `/laser_array/min_range`, `/laser_array/obstacle_below`, `/laser_array/obstacle_height`, and `/laser_array/raw_percentile`. `uart_to_stm32` can choose `serial_raw` or `laser_ground` as the mission height source.
- Vision: `visual_pkg` only publishes raw camera frames. Task-specific vision lives in task packages/nodes, such as QR/barcode handling in `activity_control_pkg` or circle detection in `circle_detector`.

### Package responsibilities

- `activity_control_pkg`: C++ mission framework. `mission_core` provides `WaypointNavigator` (target publishing, reach/hold checks, controller activation), `AuxController` (arm/magnet/signal command publishing), `SensorCache` (height and laser cache), and `ImageCache` (latest camera frame). `src/tasks/` contains mission nodes such as fixed waypoint, warehouse inventory, barcode reader, and side-camera viewer.
- `my_launch`: User-facing launch package and small tooling such as `analyze_plant_tuning`. Add new mission launch files here rather than making operators compose low-level packages manually.
- `pid_control_pkg`: C++ position controller. Converts target pose + current TF/height into `/target_velocity`; also has visual takeover/fine-data PID paths for camera-assisted alignment.
- `uart_to_stm32`: C++ serial protocol bridge to STM32. Keep STM32 frame/protocol handling here; do not put mission flow logic in this package.
- `visual_pkg`: Python OpenCV camera source. It retries camera open/read failures and publishes `sensor_msgs/Image` to a configurable topic.
- `laser_array_pkg`: C++ serial reader and ground/obstacle estimator for the 64-beam laser array.
- `my_carto_pkg`: Cartographer configuration, URDF, RViz config, and launch wrapper around the BlueSea lidar driver.
- `bluesea2`: Vendor BlueSea ROS 2 lidar driver; keep changes minimal and vendor-aware.
- `circle_detector` and `pillar_detector_pkg`: task/perception support packages for shape/pillar detection.
- `serial_comm`: reusable serial communication library used by `uart_to_stm32`.

## Adding a new mission

Follow the existing mission split instead of modifying low-level control packages:

1. Add a task node under `activity_control_pkg/src/tasks/` and use `WaypointNavigator`, `AuxController`, `SensorCache`, and/or `ImageCache` as needed.
2. Register the executable in `activity_control_pkg/CMakeLists.txt` and install it.
3. Add a launch file under `my_launch/launch/` that includes the common localization, UART, height, camera, and PID launches needed by that mission.
4. Put frequently tuned mission parameters at the top of the launch file.

Do not change `pid_control_pkg`, `uart_to_stm32`, or `visual_pkg` just to encode a new competition/task sequence; those packages are shared infrastructure.
