# ARS408 Radar Driver — Validation Results

**Date:** 2026-07-01
**Platform:** MiviaCar — MIVIA Lab, UNISA
**Driver:** pe_ars408_ros (AbdelmoutalibDOUADI/ars408_driver)
**Sensor:** Continental ARS408-21 77 GHz FMCW Radar
**ROS2 Distribution:** Humble
**Autoware Version:** Universe v1.8.0

> **Test scope:** Front radar only (SensorID=0).
> The rear radar (SensorID=1) was not physically connected during this session.

---

## 1. Test Environment

| Component         | Details                                      |
|-------------------|----------------------------------------------|
| Vehicle           | MiviaCar autonomous platform                 |
| Radar (tested)    | Continental ARS408-21 FRONT — SensorID=0     |
| Radar (pending)   | Continental ARS408-21 REAR — SensorID=1      |
| CAN interface     | can1 — dedicated radar bus                   |
| Vehicle CAN       | can0 — PIX Hooke vehicle bus (separate)      |
| LiDAR             | Ouster OS1 — used for cross-validation       |
| Autoware stack    | Universe v1.8.0, ROS2 Humble                 |
| Visualization     | RViz2                                        |

### Why front radar only?

The rear radar (SensorID=1) uses CAN IDs derived from the following formula:

    MsgId = BASE + SensorID x 0x10

    Front SensorID=0 :  0x60A  0x60B  0x60C  0x60D   (tested)
    Rear  SensorID=1 :  0x61A  0x61B  0x61C  0x61D   (not detected in candump)

No frames with IDs 0x61A–0x61D were observed during the session,
confirming the rear radar was not active on can1.

---

## 2. System Architecture

The following diagram shows the complete data flow from the physical radar
to the RViz2 visualization.

```
+---------------------------+
|  ARS408-21 FRONT Radar    |
|  77 GHz — SensorID=0      |
+---------------------------+
             |
             | CAN frames
             | 0x60A  Object_0_Status
             | 0x60B  Object_1_General
             | 0x60C  Object_2_Quality
             | 0x60D  Object_3_Extended
             v
+---------------------------+
|   can1  (hardware)        |
|   radar dedicated bus     |
+---------------------------+
             |
             v
+----------------------------------+
|  socket_can_receiver_node        |
|  (lifecycle node — activated)    |
|  interface: can1                 |
+----------------------------------+
             |
             | /radar_can_bus
             | [can_msgs/msg/Frame]
             | ~300 Hz
             v
+----------------------------------+
|  pe_ars408_node                  |
|  CAN decoding + track parsing    |
+----------------------------------+
             |
             +---> /objects_raw
             |     [radar_msgs/msg/RadarTracks]
             |     13.6 Hz
             |
             +---> /scan
                   [radar_msgs/msg/RadarScan]
                        |
                        v
             +----------------------------------+
             |  radar_pointcloud_node           |
             |  RadarTracks -> PointCloud2      |
             +----------------------------------+
                        |
                        | /radar_pointcloud
                        | [sensor_msgs/msg/PointCloud2]
                        | 13.6 Hz
                        v
             +---------------------------+
             |   RViz2                   |
             |   Fixed Frame: base_link  |
             +---------------------------+
```

**Important note on CAN bus separation:**
Autoware uses can0 for the PIX Hooke vehicle interface.
The radar operates on can1 as a separate bus.
A dedicated socket_can_receiver must be launched independently for can1,
separate from the Autoware CAN stack.

---

## 3. Launch Procedure

### Step 1 — Start the radar CAN receiver (lifecycle node)

```bash
ros2 run ros2_socketcan socket_can_receiver_node_exe \
  --ros-args \
  -r __node:=radar_can_receiver \
  -p interface:=can1 \
  -r /from_can_bus:=/radar_can_bus

ros2 lifecycle set /radar_can_receiver configure
ros2 lifecycle set /radar_can_receiver activate
```

The lifecycle node must be explicitly configured and activated.
Without this step, /radar_can_bus is not published.

### Step 2 — Start the ARS408 driver node

```bash
ros2 run pe_ars408_ros pe_ars408_node \
  --ros-args \
  -r __node:=ars408_node \
  -r /ars408_node/input/frame:=/radar_can_bus \
  -r /ars408_node/output/objects:=/objects_raw \
  -r /ars408_node/output/scan:=/scan
```

### Step 3 — Start the PointCloud2 converter

```bash
ros2 run pe_ars408_ros radar_pointcloud_node \
  --ros-args \
  -r /radar_pointcloud_node/input/tracks:=/objects_raw \
  -r /radar_pointcloud_node/output/pointcloud:=/radar_pointcloud
```

A one-shot launch script integrating all three steps is available
at scripts/launch_radar_miviaCar.sh.

---

## 4. Validation Results

### 4.1 CAN Bus Communication

Raw CAN frames observed on can1:

```
candump can1 | head -5

  can1  60A   [4]  09 EE 99 10
  can1  60B   [8]  05 4E 5C 03 80 20 06 66
  can1  60B   [8]  10 4E CC 02 80 20 01 8F
  can1  60B   [8]  1B 4F 24 01 80 20 01 7B
  can1  60C   [7]  3D 84 A3 3A 02 20 E8
```

Frame 0x60A (first byte = 0x09) indicates 9 objects detected in that cycle.
Result: ARS408 CAN frames correctly received and identified on can1.

### 4.2 Topic Publication Rate

| Topic              | Measured rate | Expected rate | Result |
|--------------------|---------------|---------------|--------|
| /radar_can_bus     | 300 Hz        | ~300 Hz       | Pass   |
| /objects_raw       | 13.6 Hz       | 13 – 17 Hz    | Pass   |
| /radar_pointcloud  | 13.6 Hz       | 13 – 17 Hz    | Pass   |

The publication rate of 13.6 Hz is within the ARS408-21 specification.

### 4.3 Object Detection — Sample Frame

```
Frame timestamp: 2026-07-01 12:51:31 UTC

Object   Position X   Position Y   Size (m)    Classification
------   ----------   ----------   --------    --------------
  1        10.6 m       +0.8 m    4.4 x 1.8   CAR  (32001)
  2        16.8 m       +0.2 m    2.2 x 2.4   CAR  (32001)
  3         5.8 m       +0.8 m    0.6 x 0.2   UNKNOWN (32000)
  4         4.2 m       +1.2 m    1.0 x 1.0   UNKNOWN (32000)
  5         8.6 m       -1.0 m    1.0 x 1.0   UNKNOWN (32000)
  6         1.4 m       +1.0 m    1.4 x 1.0   UNKNOWN (32000)

Detection range: 1.4 m to 16.8 m
Number of objects: 6
```

### 4.4 RViz2 Visualization

Two PointCloud2 displays were configured simultaneously in RViz2:

```
Display 1 — LiDAR
  Topic     : /sensing/lidar/top/ouster/points
  Type      : sensor_msgs/msg/PointCloud2
  Style     : Flat Squares
  Size (m)  : 0.01
  Color     : Intensity (rainbow)

Display 2 — Radar
  Topic     : /radar_pointcloud
  Type      : sensor_msgs/msg/PointCloud2
  Style     : Flat Squares
  Size (m)  : 0.5
  Color     : Intensity (rainbow)
```

Screenshots are available in validation/results/:

```
rviz_radar_pointcloud.png      Radar PointCloud2 displayed alone
rviz_radar_lidar_combined.png  Radar and LiDAR displayed together
rviz_radar_lidar_final.png     Final view — radar objects on LiDAR scene
```

The radar detections (large colored squares) are visually consistent
with the obstacles visible in the LiDAR point cloud.

---

## 5. TF Frame Transformation

### 5.1 Problem Statement

Each sensor on MiviaCar publishes data in its own coordinate frame:

```
LiDAR (Ouster OS1)   -->  frame_id: os_sensor_top
Radar (ARS408 front) -->  frame_id: radar_front_link
```

RViz2 requires a single Fixed Frame to display all data in the same scene.
Without a common parent frame, switching the Fixed Frame between
os_sensor_top and radar_front_link causes the other sensor to disappear.

### 5.2 Solution — Static Transform Publisher

A virtual parent frame named base_link was introduced.
Two static transforms connect each sensor frame to this common parent.

```
                  base_link
                 (virtual parent)
                /               \
               /                 \
        [T1: identity]      [T2: identity]
             /                       \
      os_sensor_top           radar_front_link
            |                        |
       os_lidar_top              (radar data)
       os_imu
```

Transform T1 and T2 are both identity transforms (no translation,
no rotation), used here for testing purposes only.

### 5.3 Commands

```bash
# T1 — connect LiDAR frame to base_link
ros2 run tf2_ros static_transform_publisher \
  0 0 0 0 0 0 base_link os_sensor_top

# T2 — connect radar frame to base_link
ros2 run tf2_ros static_transform_publisher \
  0 0 0 0 0 0 base_link radar_front_link
```

Parameter order:

```
x  y  z  roll  pitch  yaw  parent_frame  child_frame

0  0  0    0     0     0   base_link     os_sensor_top
^  ^  ^    ^     ^     ^
|  |  |    |     |     |
translation (m)  rotation (rad)
```

### 5.4 RViz2 Configuration

```
Fixed Frame : base_link
              |
              +-- Display 1 : /sensing/lidar/top/ouster/points
              +-- Display 2 : /radar_pointcloud
```

With this configuration, both sensors are rendered in the same scene.

### 5.5 Important Note on Calibration

The identity transform (0 0 0 0 0 0) assumes that both sensors
are co-located at the origin with identical orientation.
This is not physically accurate on MiviaCar.

For production use, the transform parameters must be replaced
with real extrinsic calibration values:
- Translation: measured X Y Z offset between sensor mounting positions
- Rotation: measured roll pitch yaw difference in sensor orientations

Extrinsic calibration is planned as a future task.

---

## 6. Known Issues and Next Steps

| Item                               | Status  | Action required                          |
|------------------------------------|---------|------------------------------------------|
| Front radar (SensorID=0)           | Done    | —                                        |
| Rear radar (SensorID=1)            | Pending | Connect hardware, test dual_radar.launch |
| Lifecycle node activation          | Manual  | Integrate configure+activate in launch   |
| Extrinsic calibration (TF)         | Pending | Measure real sensor offsets on MiviaCar  |
| Autoware perception integration    | Pending | Remap /objects_raw to Autoware pipeline  |

---

## 7. ROS2 Bag Recording

A bag file was recorded during the validation session for offline analysis.

```
File     : radar_lidar_recording_0.db3
Duration : 87.8 s

Topic                                  Type                          Messages
-----                                  ----                          --------
/objects_raw                           radar_msgs/msg/RadarTracks      1191
/radar_pointcloud                      sensor_msgs/msg/PointCloud2     1191
/sensing/lidar/top/ouster/points       sensor_msgs/msg/PointCloud2      ~870
/radar_can_bus                         can_msgs/msg/Frame              ~26000
/sensing/radar/can_tx                  can_msgs/msg/Frame              ~26000
/tf                                    tf2_msgs/msg/TFMessage          14008
/tf_static                             tf2_msgs/msg/TFMessage              3
```

---

Validation performed by: Abdelmoutalib DOUADI
Laboratory: MIVIA Lab, UNISA
Supervisor: Alessio
Next session: Rear radar validation + Autoware perception integration
