# Real-Time Radar Demo — RViz2 Visualization on MiviaCar

This guide describes the complete procedure to run the ARS408-21 front radar
in real time on MiviaCar and visualize its detections in RViz2, together
with the Ouster LiDAR point cloud for cross-validation.

**Prerequisites**

| Requirement            | Details                                          |
|------------------------|--------------------------------------------------|
| Access                 | SSH access to the MiviaCar PC                    |
| Radar                  | ARS408-21 front (SensorID=0) powered on can1     |
| Autoware               | Running (occupies can0 — vehicle bus)            |
| Workspace              | ~/abdel_ws built and sourced                     |
| RViz2                  | On MiviaCar PC or on a remote machine (Docker)   |

---

## 1. Overview

```
Terminal 1          Terminal 2          Terminal 3          Terminal 4
----------          ----------          ----------          ----------
CAN receiver        ARS408 driver       PointCloud2         TF publishers
(lifecycle)         node                converter           + RViz2
     |                   |                   |                   |
     v                   v                   v                   v
/radar_can_bus --> /objects_raw ------> /radar_pointcloud --> RViz2 display
   300 Hz             13.6 Hz               13.6 Hz
```

Each terminal is an SSH session to the MiviaCar PC:

```bash
ssh miviaware@<MIVIACAR_IP>
```

In every terminal, source the environment first:

```bash
source /opt/ros/humble/setup.bash
source ~/abdel_ws/install/setup.bash
```

---

## 2. Step-by-Step Procedure

### Terminal 1 — CAN receiver (lifecycle node)

The radar is connected to can1. Autoware already occupies can0 for the
PIX Hooke vehicle bus, so a dedicated receiver is required for the radar.

```bash
ros2 run ros2_socketcan socket_can_receiver_node_exe \
  --ros-args \
  -r __node:=radar_can_receiver \
  -p interface:=can1 \
  -r /from_can_bus:=/radar_can_bus
```

The receiver is a managed lifecycle node. It stays inactive until it is
explicitly configured and activated. In a second terminal (or the same
one after backgrounding with &):

```bash
ros2 lifecycle set /radar_can_receiver configure
ros2 lifecycle set /radar_can_receiver activate
```

Verification:

```bash
ros2 topic hz /radar_can_bus
# expected: ~300 Hz
```

### Terminal 2 — ARS408 driver node

Decodes the raw CAN frames (0x60A - 0x60D) into radar tracks.

```bash
ros2 run pe_ars408_ros pe_ars408_node \
  --ros-args \
  -r __node:=ars408_node \
  -r /ars408_node/input/frame:=/radar_can_bus \
  -r /ars408_node/output/objects:=/objects_raw \
  -r /ars408_node/output/scan:=/scan
```

Verification:

```bash
ros2 topic hz /objects_raw
# expected: 13 - 17 Hz (nominal ARS408 cycle)

ros2 topic echo /objects_raw --once
# expected: list of tracks with position, size, classification
```

### Terminal 3 — PointCloud2 converter

Converts RadarTracks into PointCloud2 so that RViz2 can render the
detections natively.

```bash
ros2 run pe_ars408_ros radar_pointcloud_node \
  --ros-args \
  -r /radar_pointcloud_node/input/tracks:=/objects_raw \
  -r /radar_pointcloud_node/output/pointcloud:=/radar_pointcloud
```

Verification:

```bash
ros2 topic hz /radar_pointcloud
# expected: same rate as /objects_raw
```

Alternatively, terminals 1 to 3 can be replaced by the single script:

```bash
bash scripts/launch_radar_miviaCar.sh
```

### Terminal 4 — TF transforms and RViz2

#### 4.1 Why TF transforms are required

Each sensor publishes in its own coordinate frame:

```
LiDAR (Ouster OS1)    frame_id: os_sensor_top
Radar (ARS408 front)  frame_id: radar_front_link
```

RViz2 renders all displays relative to a single Fixed Frame. Without a
transform chain connecting both frames, only one sensor can be shown at
a time. The solution is to introduce a common parent frame (base_link)
and connect each sensor frame to it with a static transform:

```
                base_link
               /         \
     [T1: identity]   [T2: identity]
             /               \
     os_sensor_top     radar_front_link
          |                   |
     LiDAR data           radar data
```

#### 4.2 Publish the static transforms

```bash
# T1 - LiDAR frame -> base_link
ros2 run tf2_ros static_transform_publisher \
  0 0 0 0 0 0 base_link os_sensor_top &

# T2 - radar frame -> base_link
ros2 run tf2_ros static_transform_publisher \
  0 0 0 0 0 0 base_link radar_front_link &
```

Argument order:

```
x  y  z  roll  pitch  yaw  parent_frame  child_frame
```

The six zeros form an identity transform (no translation, no rotation).
This is acceptable for a functional demo. For metric accuracy, replace
the zeros with the real extrinsic calibration of each sensor, i.e. the
measured mounting offsets on the vehicle.

#### 4.3 Launch RViz2

```bash
rviz2
```

If RViz2 must run on a remote laptop instead of the MiviaCar PC, replay
a recorded bag inside the Docker image (see docs/VALIDATION_RESULTS.md,
section 8) — X11 forwarding over SSH is unreliable for OpenGL
applications such as RViz2.

---

## 3. RViz2 Configuration

```
Global Options
  Fixed Frame : base_link

Display 1 — LiDAR point cloud
  Add > By topic > /sensing/lidar/top/ouster/points > PointCloud2
  Style       : Flat Squares
  Size (m)    : 0.01
  Color Transformer : Intensity
  Use rainbow : enabled

Display 2 — Radar detections
  Add > By topic > /radar_pointcloud > PointCloud2
  Style       : Flat Squares
  Size (m)    : 0.5
  Color Transformer : Intensity
  Use rainbow : enabled
```

The size difference (0.01 m vs 0.5 m) makes radar detections appear as
large colored squares over the dense LiDAR cloud, which makes the
comparison immediate.

Save the configuration for future sessions:

```
File > Save Config As > radar_lidar_demo.rviz
```

Expected result — see docs/images/rviz_radar_lidar.png:
radar objects (large squares) aligned with the obstacles visible in the
LiDAR point cloud.

---

## 4. Verification Checklist

| Check                                   | Command                                  | Expected           |
|-----------------------------------------|-------------------------------------------|--------------------|
| Radar frames on can1                    | candump can1 \| head                      | IDs 60A 60B 60C    |
| CAN topic active                        | ros2 topic hz /radar_can_bus              | ~300 Hz            |
| Tracks published                        | ros2 topic hz /objects_raw                | 13 - 17 Hz         |
| PointCloud published                    | ros2 topic hz /radar_pointcloud           | 13 - 17 Hz         |
| Single publisher per topic              | ros2 topic info /objects_raw              | Publisher count: 1 |
| TF chain complete                       | ros2 run tf2_tools view_frames            | base_link parent   |
| RViz2 status                            | Displays panel                            | Status: Ok         |

---

## 5. Common Issues

| Symptom                                  | Cause                                        | Fix                                                    |
|-------------------------------------------|----------------------------------------------|--------------------------------------------------------|
| /radar_can_bus not published              | Lifecycle node not activated                 | ros2 lifecycle set ... configure, then activate         |
| CAN Receive Timeout in receiver log       | Another receiver already reads can1          | Kill the duplicate node (ps aux \| grep socket_can)     |
| Publisher count: 2 on /objects_raw        | Driver launched twice                        | Kill the duplicate pe_ars408_node                       |
| Radar squares invisible in RViz2          | Wrong Fixed Frame or missing TF              | Set Fixed Frame to base_link, publish both transforms   |
| Node subscribes but publishes nothing     | Remapping used ~ instead of absolute names   | Use /ars408_node/input/frame:=... (absolute)            |
| RViz2 fails over SSH -X                   | OpenGL over X11 forwarding                   | Replay a bag locally in Docker instead                  |
