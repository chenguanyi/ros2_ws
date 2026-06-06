# D Task Ground Station Interface

This document describes the ROS 2 contract between the drone and the ground
station for D task warehouse inventory.

The drone side is `warehouse_inventory_task_node`. If the ground station does
not publish a mode command before STM32 ready, the drone defaults to
requirement 1 inventory. Requirement 2 is selected before STM32 ready with
`/d_task/mode = 1`.

## Sequences

### Requirement 1: default inventory

```text
drone launch starts
drone waits for /is_st_ready while accepting optional /d_task/mode commands
no mode command arrives before /is_st_ready
drone starts inventory route after /is_st_ready and valid flight state
drone publishes scan results during flight
drone publishes /mission_complete after landing
```

Ground station subscriptions used in this mode:

- `/warehouse_inventory/route`
- `/warehouse_inventory/scan_result`
- `/warehouse_inventory/all_results`
- `/warehouse_inventory/query_result`
- `/mission_complete`

### Requirement 2: target inventory

```text
ground station publishes /d_task/mode = 1 before /is_st_ready
drone waits for side-camera QR recognition
drone publishes /d_task/qr_id and /warehouse_inventory/target_id
ground station plans route from the QR id and its current inventory table
ground station publishes /d_task/route before /is_st_ready
drone validates route JSON and waits for /is_st_ready
after /is_st_ready and valid flight state, drone takes off, flies the route, scans target, returns, lands
drone publishes /mission_complete
```

After `/is_st_ready` has been received, new `/d_task/mode` and `/d_task/route`
messages are ignored. The ignore reason is reported on `/d_task/status`.

## Topics

| Topic | Direction | Type | QoS suggestion | Meaning |
| --- | --- | --- | --- | --- |
| `/d_task/mode` | ground -> drone | `std_msgs/UInt8` | reliable, depth 10 | `1` selects requirement 2 before `/is_st_ready`. `0` keeps requirement 1. Other values are ignored. |
| `/d_task/route` | ground -> drone | `std_msgs/String` | reliable, depth 10 | Requirement 2 route JSON. Must be sent after receiving the QR id and before `/is_st_ready`. |
| `/is_st_ready` | STM32 bridge -> drone | `std_msgs/UInt8` | reliable, transient local, depth 1 | `1` closes the mode/route selection window and allows the selected mission to start when ready. |
| `/d_task/qr_id` | drone -> ground | `std_msgs/Int32` | reliable, transient local, depth 1 | QR id recognized by the drone before requirement 2 takeoff. Positive integer only. |
| `/d_task/status` | drone -> ground | `std_msgs/String` | reliable, transient local, depth 1 | Drone state JSON for UI/debugging. |
| `/warehouse_inventory/target_id` | drone -> ground | `std_msgs/Int32` | reliable, transient local, depth 1 | Compatibility copy of `/d_task/qr_id`. |
| `/warehouse_inventory/route` | drone -> ground | `std_msgs/String` | reliable, transient local, depth 1 | Route JSON currently loaded by the drone, including auto-added return/landing display steps. |
| `/warehouse_inventory/scan_result` | drone -> ground | `std_msgs/String` | reliable, transient local, depth 1 | One accepted inventory/target scan result. |
| `/warehouse_inventory/all_results` | drone -> ground | `std_msgs/String` | reliable, transient local, depth 1 | Final requirement 1 inventory table. |
| `/warehouse_inventory/query_result` | drone -> ground | `std_msgs/String` | reliable, transient local, depth 1 | Response to existing `/warehouse_inventory/query`. |
| `/mission_complete` | drone -> ground | `std_msgs/Empty` | reliable, depth 10 | Mission has completed and the drone has landed. |

## `/d_task/route` JSON

The message payload is a UTF-8 JSON string:

```json
{
  "steps": [
    {
      "kind": "scan",
      "scan": true,
      "coord": "A1",
      "x_cm": 15.0,
      "y_cm": 75.0,
      "z_cm": 140.0,
      "yaw_deg": 90.0
    }
  ]
}
```

Validation rules:

- Root must be an object.
- `steps` must be a non-empty array.
- Every step must contain numeric finite `x_cm`, `y_cm`, `z_cm`, and `yaw_deg`.
- `scan` is optional and defaults to `false`, but at least one step must set
  `scan: true`.
- `kind` is optional and defaults to `scan` for scan steps, otherwise `transit`.
- `coord` is optional but should be set for the target scan point, for example
  `A1`.

Units and frames:

- `x_cm`, `y_cm`, `z_cm`: centimeters.
- `yaw_deg`: degrees.
- Coordinates are in the drone `map` frame.
- The drone appends its normal return and landing steps after the received
  route. The ground station should send only the route from takeoff/cruise to
  the target scan point unless a custom pre-target path is needed.

## `/d_task/status` JSON

Example:

```json
{
  "mode": "target",
  "mode_value": 1,
  "phase": "WAIT_GROUND_ROUTE",
  "locked": false,
  "st_ready": false,
  "target_id": 7,
  "route_valid": false,
  "route_loaded": false,
  "active_steps": 0,
  "current_step_index": 0,
  "note": "waiting_ground_route"
}
```

Fields:

- `mode`: `inventory` or `target`.
- `mode_value`: `0` for requirement 1, `1` for requirement 2.
- `phase`: internal drone phase, useful for UI and debugging.
- `locked`: `true` after takeoff has started; mode/route updates are ignored.
- `st_ready`: `true` after the drone receives `/is_st_ready = 1`; mode/route
  updates are ignored from this point.
- `target_id`: QR id recognized before requirement 2.
- `route_valid`: latest `/d_task/route` parsed successfully.
- `route_loaded`: parsed route has been loaded into the mission state machine.
- `active_steps`: number of active route steps.
- `current_step_index`: current active route index.
- `note`: short reason for the latest state publication.

## Manual ROS 2 Test Commands

Start the drone stack:

```bash
ros2 launch my_launch warehouse_inventory.launch.py
```

Default requirement 1:

```bash
ros2 topic echo /d_task/status
ros2 topic pub --once /is_st_ready std_msgs/msg/UInt8 "{data: 1}"
```

Select requirement 2 before STM32 ready:

```bash
ros2 topic pub --once /d_task/mode std_msgs/msg/UInt8 "{data: 1}"
```

Simulate the side camera recognizing QR id 7:

```bash
ros2 topic pub --once /warehouse_inventory/barcode_value std_msgs/msg/Int32 "{data: 7}"
ros2 topic echo /d_task/qr_id
```

Publish a target route:

```bash
ros2 topic pub --once /d_task/route std_msgs/msg/String "{data: '{\"steps\":[{\"kind\":\"scan\",\"scan\":true,\"coord\":\"A1\",\"x_cm\":15.0,\"y_cm\":75.0,\"z_cm\":140.0,\"yaw_deg\":90.0}]}'}"
```

Release the selected mission after mode, QR, and route are ready:

```bash
ros2 topic pub --once /is_st_ready std_msgs/msg/UInt8 "{data: 1}"
```

Watch route and scan feedback:

```bash
ros2 topic echo /warehouse_inventory/route
ros2 topic echo /warehouse_inventory/scan_result
ros2 topic echo /mission_complete
```

## Network Notes

For two machines on the same LAN, both sides need:

- Same `ROS_DOMAIN_ID`.
- Matching topic names and message types.
- DDS discovery and multicast allowed by the network/firewall.

Before flight, verify both directions:

```bash
ros2 topic echo /d_task/qr_id
ros2 topic echo /d_task/route
```
