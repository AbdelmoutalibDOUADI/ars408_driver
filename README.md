# ARS408 ROS2 Dual Radar Driver

ROS2 driver for the Continental ARS408-21 radar sensor, extended to support **two radar units operating simultaneously on the same CAN bus**.

This work is based on the original [TIER IV implementation](https://github.com/tier4/ars408_driver) and was developed during an internship at **UNISA-DIEM** (Università degli Studi di Salerno), under the supervision of Prof. Diego Gragnaniello.

---

## What was changed from the original driver

The original TIER IV driver supports only one radar unit with hardcoded CAN IDs. The following modifications were made to support two radars simultaneously:

| File | Change |
|------|--------|
| `ars408_constants.hpp` | CAN ID constants renamed to `_BASE` (e.g. `OBJ_STATUS_BASE`) |
| `ars408_driver.hpp` | Added `sensor_id_` attribute and dynamic CAN ID variables |
| `ars408_driver.cpp` | Added `Init(sensor_id)` function + replaced `switch/case` with `if/else if` |
| `ars408_ros_node.cpp` | Added `sensor_id` ROS2 parameter with validation (0–7) |
| `config/radar_front.yaml` | Configuration file for radar front (SensorID=0) |
| `config/radar_rear.yaml` | Configuration file for radar rear (SensorID=1) |
| `launch/dual_radar.launch.xml` | Launch file to run both radars simultaneously |

---

## How it works

The Continental ARS408-21 supports up to 8 sensors on the same CAN bus using the **SensorID** mechanism. Each radar can be assigned a unique SensorID (0 to 7), which shifts all its CAN message IDs by an offset:

```
MsgId = MsgId_BASE + SensorId * 0x10
```

### Example with two radars

| Message | SensorID=0 (radar front) | SensorID=1 (radar rear) |
|---------|--------------------------|-------------------------|
| RadarState | 0x201 | 0x211 |
| Obj_Status | 0x60A | 0x61A |
| Obj_General | 0x60B | 0x61B |
| Obj_Quality | 0x60C | 0x61C |
| Obj_Extended | 0x60D | 0x61D |

> **Exception:** The relay control message `0x8` always keeps the same ID regardless of SensorID (Continental documentation, section 7.6).

Each driver instance reads its `sensor_id` from a YAML configuration file, computes the correct CAN IDs at startup via `Init()`, and filters incoming CAN frames accordingly. This ensures full isolation between the two radar data streams.

---

## Prerequisites

- ROS2 Humble (Ubuntu 22.04)
- `ros-humble-can-msgs`
- `ros-humble-radar-msgs`
- `ros-humble-ros2-socketcan`
- A CAN interface (e.g. `can0`) connected to the radar(s)

Install dependencies:

```bash
cd ~/ros2_ws_ars408
rosdep install --from-paths src --ignore-src -r -y
```

---

## Build

```bash
cd ~/ros2_ws_ars408
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## Configuration

Each radar is configured via a YAML file in the `config/` directory.

### `config/radar_front.yaml` (SensorID = 0)

```yaml
/**:
  ros__parameters:
    sensor_id: 0
    output_frame: "radar_front"
    publish_radar_track: true
    publish_radar_scan: false
    sequential_publish: false
    size_x: 1.8
    size_y: 1.8
```

### `config/radar_rear.yaml` (SensorID = 1)

```yaml
/**:
  ros__parameters:
    sensor_id: 1
    output_frame: "radar_rear"
    publish_radar_track: true
    publish_radar_scan: false
    sequential_publish: false
    size_x: 1.8
    size_y: 1.8
```

### Parameter description

| Parameter | Type | Description |
|-----------|------|-------------|
| `sensor_id` | int (0–7) | Radar SensorID — determines the CAN ID offset |
| `output_frame` | string | Frame ID used in published ROS2 message headers |
| `publish_radar_track` | bool | Publish `RadarTracks` on `~/output/objects` |
| `publish_radar_scan` | bool | Publish `RadarScan` on `~/output/scan` |
| `sequential_publish` | bool | If true, publish each object as soon as received |
| `size_x` / `size_y` | double | Default object size in meters (used when radar does not provide dimensions) |

---

## Running two radars simultaneously

### Step 1 — Set up the CAN interface

```bash
sudo ip link set can0 up type can bitrate 500000
```

### Step 2 — Launch both radars

```bash
ros2 launch pe_ars408_ros dual_radar.launch.xml
```

This launches:
- One `socketcan` receiver on `can0` (shared by both radars)
- One driver instance for radar front (`namespace: radar_front`, `sensor_id: 0`)
- One driver instance for radar rear (`namespace: radar_rear`, `sensor_id: 1`)

### Step 3 — Verify the output topics

```bash
ros2 topic list
```

You should see:

```
/radar_front/output/objects
/radar_rear/output/objects
```

To verify data isolation (no cross-contamination):

```bash
ros2 topic echo /radar_front/output/objects
ros2 topic echo /radar_rear/output/objects
```

---

## Architecture

```
CAN bus (can0)
      │
socketcan_bridge
      │ /sensing/radar/can_tx
      │
      ├──────────────────────────────────┐
      │                                  │
ARS408 Node (radar_front)        ARS408 Node (radar_rear)
sensor_id = 0                    sensor_id = 1
Listens: 0x60A, 0x60B...         Listens: 0x61A, 0x61B...
      │                                  │
/radar_front/output/objects      /radar_rear/output/objects
```

---

## Assumptions

- Both radars must be pre-configured with different SensorIDs before connecting to the CAN bus. SensorID can be set via the `RadarCfg` message (CAN ID `0x200`) with `RadarCfg_SensorID_valid = 1` and `RadarCfg_StoreInNVM = 1` to persist across power cycles.
- The CAN bus bitrate is **500 kbit/s** as specified in the Continental ARS408-21 documentation.
- The driver assumes the radar is configured to output **objects** (`RadarCfg_OutputType = 0x1`). Cluster output is not handled by this driver.
- `PointCloud2` output is not natively published. If required for visualization or downstream processing, a dedicated conversion node should be used.

---

## References

- Continental ARS408-21 Technical Documentation V1.8 (October 2017)
- [TIER IV original driver](https://github.com/tier4/ars408_driver)
- ROS2 Humble documentation

---

## Author

**Abdelmoutalib Douadi**  
4th Year Engineering Student — Mechatronics and Embedded Systems (MeSE)  
ESIX Normandie — Université de Caen Normandie  
Intern at UNISA-DIEM, Fisciano (Italy) — 2026
