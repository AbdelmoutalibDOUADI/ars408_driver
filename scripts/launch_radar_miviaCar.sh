#!/bin/bash
# ============================================================
# ARS408 Radar Launch Script for MiviaCar
# Usage: bash launch_radar_miviaCar.sh
# ============================================================

source /opt/ros/humble/setup.bash
source ~/abdel_ws/install/setup.bash

echo "[1/3] Starting radar CAN receiver on can1..."
ros2 run ros2_socketcan socket_can_receiver_node_exe \
  --ros-args \
  -r __node:=radar_can_receiver \
  -p interface:=can1 \
  -r /from_can_bus:=/radar_can_bus &

sleep 1

echo "[2/3] Activating lifecycle node..."
ros2 lifecycle set /radar_can_receiver configure
ros2 lifecycle set /radar_can_receiver activate

sleep 1

echo "[3/3] Starting ARS408 driver node..."
ros2 run pe_ars408_ros pe_ars408_node \
  --ros-args \
  -r __node:=ars408_node \
  -r /ars408_node/input/frame:=/radar_can_bus \
  -r /ars408_node/output/objects:=/objects_raw \
  -r /ars408_node/output/scan:=/scan &

sleep 1

echo "[4/4] Starting PointCloud2 converter..."
ros2 run pe_ars408_ros radar_pointcloud_node \
  --ros-args \
  -r /radar_pointcloud_node/input/tracks:=/objects_raw \
  -r /radar_pointcloud_node/output/pointcloud:=/radar_pointcloud &

echo ""
echo "✅ Radar pipeline started!"
echo "Topics:"
echo "  /objects_raw       → radar tracks (13.6 Hz)"
echo "  /radar_pointcloud  → PointCloud2"
echo ""
echo "To verify: ros2 topic hz /objects_raw"
