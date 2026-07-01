#!/bin/bash
# ============================================================
# ARS408 Radar Launch Script — MiviaCar (FRONT radar, SensorID=0)
# 
# Usage:
#   cd ~/abdel_ws
#   source /opt/ros/humble/setup.bash
#   source install/setup.bash
#   bash scripts/launch_radar_miviaCar.sh
#
# NOTE: Autoware must already be running (uses can0 for vehicle).
#       This script uses can1 (dedicated radar CAN bus).
# ============================================================

set -e

echo "================================================"
echo " ARS408 Radar Pipeline — MiviaCar"
echo " Sensor: FRONT (SensorID=0) | Interface: can1"
echo "================================================"
echo ""

# Check can1 is up
if ! ip link show can1 &>/dev/null; then
    echo "[ERROR] can1 interface not found!"
    echo "  Run: sudo ip link set up can1 type can bitrate 500000"
    exit 1
fi
echo "[✓] can1 interface found"

# Check CAN data is coming
echo "[INFO] Checking radar CAN frames on can1..."
FRAMES=$(timeout 2 candump can1 2>/dev/null | grep -c "60[ABCD]" || true)
if [ "$FRAMES" -eq 0 ]; then
    echo "[WARN] No ARS408 frames detected on can1 — is radar powered?"
else
    echo "[✓] $FRAMES ARS408 frames detected"
fi
echo ""

# Step 1 — socket_can_receiver lifecycle node
echo "[1/4] Starting radar_can_receiver on can1..."
ros2 run ros2_socketcan socket_can_receiver_node_exe \
  --ros-args \
  -r __node:=radar_can_receiver \
  -p interface:=can1 \
  -r /from_can_bus:=/radar_can_bus &
RECV_PID=$!
sleep 1.5

echo "[2/4] Activating lifecycle node..."
ros2 lifecycle set /radar_can_receiver configure
ros2 lifecycle set /radar_can_receiver activate
echo "[✓] radar_can_receiver active"
echo ""

# Step 2 — ARS408 driver
echo "[3/4] Starting ars408_node..."
ros2 run pe_ars408_ros pe_ars408_node \
  --ros-args \
  -r __node:=ars408_node \
  -r /ars408_node/input/frame:=/radar_can_bus \
  -r /ars408_node/output/objects:=/objects_raw \
  -r /ars408_node/output/scan:=/scan &
ARS_PID=$!
sleep 1
echo "[✓] ars408_node started"
echo ""

# Step 3 — PointCloud2 converter
echo "[4/4] Starting radar_pointcloud_node..."
ros2 run pe_ars408_ros radar_pointcloud_node \
  --ros-args \
  -r /radar_pointcloud_node/input/tracks:=/objects_raw \
  -r /radar_pointcloud_node/output/pointcloud:=/radar_pointcloud &
PC_PID=$!
sleep 1
echo "[✓] radar_pointcloud_node started"
echo ""

echo "================================================"
echo " ✅ Radar pipeline running!"
echo ""
echo " Topics:"
echo "   /radar_can_bus      → raw CAN (~300 Hz)"
echo "   /objects_raw        → RadarTracks (~13.6 Hz)"
echo "   /radar_pointcloud   → PointCloud2 (~13.6 Hz)"
echo ""
echo " Verify: ros2 topic hz /objects_raw"
echo ""
echo " Visualize in RViz2:"
echo "   ros2 run tf2_ros static_transform_publisher \\"
echo "     0 0 0 0 0 0 base_link radar_front_link"
echo "   ros2 run tf2_ros static_transform_publisher \\"
echo "     0 0 0 0 0 0 base_link os_sensor_top"
echo "   rviz2  (Fixed Frame: base_link)"
echo "================================================"
echo ""
echo "Press Ctrl+C to stop all nodes"

# Wait and cleanup on exit
trap "echo 'Stopping...'; kill $RECV_PID $ARS_PID $PC_PID 2>/dev/null" EXIT
wait
