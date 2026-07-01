# ARS408 Radar Driver — Validation Results
**Date:** 2026-07-01  
**Platform:** MiviaCar (MIVIA Lab, UNISA)  
**Driver:** `pe_ars408_ros` (AbdelmoutalibDOUADI/ars408_driver)  
**Sensor:** Continental ARS408-21 77 GHz  
**ROS2:** Humble  

---

## 1. Test Environment

| Component | Details |
|-----------|---------|
| Vehicle | MiviaCar autonomous platform |
| Radar | Continental ARS408-21 (front, SensorID=0) |
| CAN Interface | `can1` (hardware) |
| LiDAR | Ouster OS1 (for cross-validation) |
| Stack | Autoware Universe v1.8.0 (ROS2 Humble) |
| Visualization | RViz2 + Foxglove Studio |

---

## 2. Pipeline Architecture

```
ARS408 Radar (77 GHz)
        │
        │ CAN frames (0x60A, 0x60B, 0x60C, 0x60D)
        ▼
    can1 (hardware CAN bus)
        │
        ▼
socket_can_receiver_node  (lifecycle node, activated)
        │ /radar_can_bus  [can_msgs/msg/Frame @ 300 Hz]
        ▼
    pe_ars408_node
        │
        ├── /objects_raw      [radar_msgs/msg/RadarTracks @ 13.6 Hz]
        └── /scan             [radar_msgs/msg/RadarScan]
                │
                ▼
    radar_pointcloud_node
                │
                └── /radar_pointcloud  [sensor_msgs/msg/PointCloud2]
```

**Key discovery:** Autoware uses `can0` for vehicle (PIX Hooke), radar is on `can1`.  
A dedicated `socket_can_receiver` lifecycle node must be launched separately for `can1`.

---

## 3. Launch Procedure

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

---

## 4. Validation Results

### 4.1 CAN Bus Communication ✅
```
candump can1 | head -5
  can1  60A   [4]  09 EE 99 10   ← Object_0_Status header
  can1  60B   [8]  05 4E 5C 03 80 20 06 66  ← Object detected
  can1  60B   [8]  10 4E CC 02 80 20 01 8F
  can1  60B   [8]  1B 4F 24 01 80 20 01 7B
  can1  60C   [7]  3D 84 A3 3A 02 20 E8     ← Object quality
```
**Result:** ARS408 CAN frames correctly received on `can1` ✅

### 4.2 Topic Publication Rate ✅
```
/radar_can_bus    →  300 Hz   (raw CAN frames)
/objects_raw      →  13.6 Hz  (radar tracks, nominal: 13-17 Hz)
/radar_pointcloud →  13.6 Hz  (PointCloud2)
```
**Result:** Publication rate within ARS408 specification (13-17 Hz) ✅

### 4.3 Object Detection — Sample Data ✅
```yaml
tracks:
- position: {x: 10.6m, y: 0.8m, z: 0.0}
  size: {x: 4.4m, y: 1.8m}
  classification: 32001  # CAR
- position: {x: 16.8m, y: 0.2m, z: 0.0}
  size: {x: 2.2m, y: 2.4m}
  classification: 32001  # CAR
- position: {x: 5.8m, y: 0.8m, z: 0.0}
  size: {x: 0.6m, y: 0.2m}
  classification: 32000  # UNKNOWN
- position: {x: 4.2m, y: 1.2m, z: 0.0}
  classification: 32000
- position: {x: 8.6m, y: -1.0m, z: 0.0}
  classification: 32000
- position: {x: 1.4m, y: 1.0m, z: 0.0}
  classification: 32000
```
**Result:** 6 objects detected at distances 1.4m–16.8m ✅

### 4.4 RViz2 Visualization — Radar + LiDAR ✅
Both `/radar_pointcloud` and `/sensing/lidar/top/ouster/points` displayed  
simultaneously in RViz2 with a common `base_link` frame.  
Radar detections visually consistent with LiDAR point cloud obstacles. ✅

---

## 5. TF Frame Transformation Explanation

### Problem
Each sensor publishes in its own coordinate frame:
- LiDAR (Ouster) → `os_sensor_top`
- Radar (ARS408) → `radar_front_link`

RViz2 requires a single Fixed Frame to display all data together.

### Solution — Static TF Publisher
A virtual parent frame `base_link` was introduced:

```
         base_link  (virtual parent)
         /         \
        /             \
os_sensor_top    radar_front_link
      |                  |
   LiDAR              Radar
      |                  |
os_lidar_top          (data)
os_imu
```

### Commands
```bash
# Connect radar to base_link
ros2 run tf2_ros static_transform_publisher \
  0 0 0 0 0 0 base_link radar_front_link

# Connect LiDAR to base_link
ros2 run tf2_ros static_transform_publisher \
  0 0 0 0 0 0 base_link os_sensor_top
```

### Parameters: `0 0 0 0 0 0`
```
x=0  y=0  z=0    → no translation (same position)
r=0  p=0  y=0    → no rotation (same orientation)
```
> **Note:** In production, real extrinsic calibration values (measured  
> physical offsets between sensors on MiviaCar) should replace these zeros.

### Result
With `Fixed Frame = base_link` in RViz2:
- LiDAR point cloud renders correctly ✅
- Radar PointCloud2 renders correctly ✅
- Both displayed simultaneously ✅

---

## 6. Known Issues & Next Steps

| Issue | Status | Next Action |
|-------|--------|-------------|
| `can0` vs `can1` bus separation | ✅ Solved | Document in launch file |
| Lifecycle node activation manual | ⚠️ | Automate in launch file |
| TF calibration (zeros used) | ⚠️ | Measure real sensor offsets on MiviaCar |
| Dual radar (rear SensorID=1) | 🔜 | Test with `dual_radar.launch.xml` |
| Foxglove `radar_msgs` support | ⚠️ | Use PointCloud2 topic instead |

---

## 7. ROS2 Bag Recording
A bag file was recorded for offline analysis:
```
Topics recorded:
  /objects_raw                          [radar_msgs/msg/RadarTracks]
  /radar_pointcloud                     [sensor_msgs/msg/PointCloud2]
  /sensing/lidar/top/ouster/points      [sensor_msgs/msg/PointCloud2]
  /radar_can_bus                        [can_msgs/msg/Frame]
  /sensing/radar/can_tx                 [can_msgs/msg/Frame]
  /tf                                   [tf2_msgs/msg/TFMessage]
  /tf_static                            [tf2_msgs/msg/TFMessage]
Duration: ~88s
Size: ~367 MB
```

---

*Validation performed by Abdelmoutalib DOUADI — MIVIA Lab, UNISA*  
*Supervisor: Alessio*
