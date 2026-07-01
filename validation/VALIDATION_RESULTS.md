# ARS408 Radar Driver — Validation Results
**Date:** 2026-07-01  
**Platform:** MiviaCar (MIVIA Lab, UNISA)  
**Driver:** `pe_ars408_ros` (AbdelmoutalibDOUADI/ars408_driver)  
**Sensor:** Continental ARS408-21 77 GHz  
**ROS2:** Humble | **Autoware:** Universe v1.8.0  

> ⚠️ **Test scope:** FRONT radar only (SensorID=0)  
> Rear radar (SensorID=1) physically not connected during this session.

---

## 1. Test Environment

| Component | Details |
|-----------|---------|
| Vehicle | MiviaCar autonomous platform |
| Radar tested | Continental ARS408-21 **FRONT** (SensorID=0) |
| Radar NOT tested | Continental ARS408-21 REAR (SensorID=1) — not connected |
| CAN Interface | `can1` (hardware, radar bus) |
| Vehicle CAN | `can0` (PIX Hooke vehicle bus — separate) |
| LiDAR | Ouster OS1 (cross-validation only) |
| Stack | Autoware Universe v1.8.0 (ROS2 Humble) |
| Visualization | RViz2 + Foxglove Studio |

### Why front radar only?
The rear radar (SensorID=1) uses CAN IDs `0x61A`–`0x61D` (offset by `SensorID × 0x10`).
No such frames were observed in `candump can1` during the test session,
confirming the rear radar was not physically active.

```
Front SensorID=0 → CAN IDs: 0x60A, 0x60B, 0x60C, 0x60D  ✅ tested
Rear  SensorID=1 → CAN IDs: 0x61A, 0x61B, 0x61C, 0x61D  🔜 pending
```

---

## 2. Pipeline Architecture

```
ARS408 Radar FRONT (77 GHz, SensorID=0)
        │
        │ CAN frames: 0x60A (status) 0x60B (general)
        │             0x60C (quality) 0x60D (extended)
        ▼
    can1  (hardware CAN bus — radar only)
        │
        ▼
radar_can_receiver  [lifecycle node, manually activated]
        │  /radar_can_bus  [can_msgs/msg/Frame @ ~300 Hz]
        ▼
    ars408_node  (pe_ars408_ros)
        │
        ├── /objects_raw      [radar_msgs/msg/RadarTracks @ 13.6 Hz]
        └── /scan             [radar_msgs/msg/RadarScan]
                │
                ▼
    radar_pointcloud_node
                │
                └── /radar_pointcloud  [sensor_msgs/msg/PointCloud2 @ 13.6 Hz]
                                │
                                ▼
                    RViz2 / Foxglove Studio

NOTE: Autoware uses can0 for PIX Hooke vehicle bus (separate from radar).
      A dedicated socket_can_receiver must be launched for can1.
```

---

## 3. Launch Procedure (Manual — MiviaCar)

### Step 1 — Start radar CAN receiver (lifecycle node)
```bash
ros2 run ros2_socketcan socket_can_receiver_node_exe \
  --ros-args \
  -r __node:=radar_can_receiver \
  -p interface:=can1 \
  -r /from_can_bus:=/radar_can_bus

ros2 lifecycle set /radar_can_receiver configure
ros2 lifecycle set /radar_can_receiver activate
```

### Step 2 — Start ARS408 driver node
```bash
ros2 run pe_ars408_ros pe_ars408_node \
  --ros-args \
  -r __node:=ars408_node \
  -r /ars408_node/input/frame:=/radar_can_bus \
  -r /ars408_node/output/objects:=/objects_raw \
  -r /ars408_node/output/scan:=/scan
```

### Step 3 — Start PointCloud2 converter
```bash
ros2 run pe_ars408_ros radar_pointcloud_node \
  --ros-args \
  -r /radar_pointcloud_node/input/tracks:=/objects_raw \
  -r /radar_pointcloud_node/output/pointcloud:=/radar_pointcloud
```

> 💡 Use `scripts/launch_radar_miviaCar.sh` to run all steps automatically.

---

## 4. Validation Results

### 4.1 CAN Bus Communication ✅
```
candump can1 | head -5
  can1  60A   [4]  09 EE 99 10   ← Object_0_Status header (9 objects)
  can1  60B   [8]  05 4E 5C 03 80 20 06 66  ← Object general info
  can1  60B   [8]  10 4E CC 02 80 20 01 8F
  can1  60B   [8]  1B 4F 24 01 80 20 01 7B
  can1  60C   [7]  3D 84 A3 3A 02 20 E8     ← Object quality
```
**Result:** ARS408 CAN frames correctly received on `can1` ✅

### 4.2 Topic Publication Rate ✅
| Topic | Rate | Expected | Status |
|-------|------|----------|--------|
| `/radar_can_bus` | 300 Hz | ~300 Hz | ✅ |
| `/objects_raw` | **13.6 Hz** | 13–17 Hz | ✅ |
| `/radar_pointcloud` | 13.6 Hz | 13–17 Hz | ✅ |

### 4.3 Object Detection — Sample Data ✅
```yaml
# 6 objects detected in single frame
tracks:
- position: {x: 10.6m, y:  0.8m}  size: {4.4×1.8m}  class: CAR (32001)
- position: {x: 16.8m, y:  0.2m}  size: {2.2×2.4m}  class: CAR (32001)
- position: {x:  5.8m, y:  0.8m}  size: {0.6×0.2m}  class: UNKNOWN (32000)
- position: {x:  4.2m, y:  1.2m}  size: {1.0×1.0m}  class: UNKNOWN (32000)
- position: {x:  8.6m, y: -1.0m}  size: {1.0×1.0m}  class: UNKNOWN (32000)
- position: {x:  1.4m, y:  1.0m}  size: {1.4×1.0m}  class: UNKNOWN (32000)
```
**Result:** 6 objects detected, range 1.4m–16.8m ✅

### 4.4 RViz2 Visualization ✅
See screenshots in `validation/results/`:

| Screenshot | Description |
|-----------|-------------|
| `rviz_radar_pointcloud.png` | Radar PointCloud2 alone |
| `rviz_radar_lidar_combined.png` | Radar + LiDAR combined |
| `rviz_radar_lidar_final.png` | Final view — radar objects on LiDAR scene |
| `foxglove_topics.png` | Foxglove Studio connected to MiviaCar |

---

## 5. TF Frame Transformation

### Problem
Each sensor publishes in its own coordinate frame:
```
LiDAR (Ouster)  → frame: os_sensor_top
Radar (ARS408)  → frame: radar_front_link
```
RViz2 needs a single **Fixed Frame** to display all data together.
Switching Fixed Frame to `os_sensor_top` hides the radar and vice versa.

### Solution — Virtual parent frame `base_link`

```
              base_link  (virtual common parent)
             /          \
            /              \
    os_sensor_top      radar_front_link
          |                    |
     os_lidar_top           (radar data)
     os_imu
```

### Commands used
```bash
# Connect radar frame to base_link
ros2 run tf2_ros static_transform_publisher \
  0 0 0 0 0 0 base_link radar_front_link

# Connect LiDAR frame to base_link
ros2 run tf2_ros static_transform_publisher \
  0 0 0 0 0 0 base_link os_sensor_top
```

### Parameters explained
```
0  0  0  → translation X Y Z = 0 m  (no offset)
0  0  0  → rotation roll pitch yaw = 0° (no rotation)
```

> ⚠️ **Important:** Values `0 0 0 0 0 0` are used for testing only.  
> In production, real **extrinsic calibration** values must be used  
> (measured physical offsets of each sensor on MiviaCar).

### Result in RViz2
With `Fixed Frame = base_link`:
- LiDAR dense point cloud ✅
- Radar PointCloud2 (colored squares) ✅  
- Both displayed simultaneously ✅
- Radar detections visually consistent with LiDAR obstacles ✅

---

## 6. Known Issues & Next Steps

| Item | Status | Action |
|------|--------|--------|
| Front radar (SensorID=0) validated | ✅ Done | — |
| Rear radar (SensorID=1) | 🔜 Pending | Connect rear radar, test `dual_radar.launch.xml` |
| Lifecycle node activation manual | ⚠️ | Integrate in launch file |
| TF calibration (zeros used) | ⚠️ | Measure real sensor offsets on MiviaCar |
| Foxglove `radar_msgs` not rendered | ⚠️ | Use `/radar_pointcloud` topic instead |
| Autoware integration `/sensing/radar` | 🔜 | Remap `/objects_raw` → Autoware perception |

---

## 7. ROS2 Bag Info
```
File: radar_lidar_recording_0.db3
Duration: 87.8s
Topics:
  /objects_raw                      [radar_msgs/msg/RadarTracks]   1191 msgs
  /radar_pointcloud                 [sensor_msgs/msg/PointCloud2]  1191 msgs
  /sensing/lidar/top/ouster/points  [sensor_msgs/msg/PointCloud2]  ~870 msgs
  /radar_can_bus                    [can_msgs/msg/Frame]
  /sensing/radar/can_tx             [can_msgs/msg/Frame]
  /tf                               [tf2_msgs/msg/TFMessage]      14008 msgs
  /tf_static                        [tf2_msgs/msg/TFMessage]          3 msgs
```

---

*Validation: Abdelmoutalib DOUADI — MIVIA Lab, UNISA*  
*Supervisor: Alessio*  
*Next session: Douadi (MiviaCar PC) — rear radar + Autoware integration*
