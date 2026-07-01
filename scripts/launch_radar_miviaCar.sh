#!/bin/bash
# =============================================================================
# Script   : launch_radar_miviaCar.sh
# Purpose  : Launch the ARS408-21 radar pipeline on MiviaCar
# Sensor   : Continental ARS408-21 FRONT (SensorID=0)
# Interface: can1 (dedicated radar CAN bus)
# Platform : MiviaCar — MIVIA Lab, UNISA
# ROS2     : Humble
#
# Prerequisites:
#   - Autoware Universe v1.8.0 running (uses can0 for vehicle)
#   - can1 interface up and radar powered
#   - ROS2 Humble sourced
#   - pe_ars408_ros workspace sourced
#
# Usage:
#   cd ~/abdel_ws
#   source /opt/ros/humble/setup.bash
#   source install/setup.bash
#   bash scripts/launch_radar_miviaCar.sh
#
# Topics published:
#   /radar_can_bus     [can_msgs/msg/Frame]         ~300 Hz
#   /objects_raw       [radar_msgs/msg/RadarTracks]  ~13.6 Hz
#   /scan              [radar_msgs/msg/RadarScan]    ~13.6 Hz
#   /radar_pointcloud  [sensor_msgs/msg/PointCloud2] ~13.6 Hz
#
# RViz2 visualization (run separately after this script):
#   ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link radar_front_link
#   ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link os_sensor_top
#   rviz2   (Fixed Frame: base_link)
# =============================================================================

set -euo pipefail

# -----------------------------------------------------------------------------
# Configuration
# -----------------------------------------------------------------------------
CAN_INTERFACE="can1"
RADAR_NODE_NAME="ars408_node"
RECEIVER_NODE_NAME="radar_can_receiver"
RADAR_CAN_TOPIC="/radar_can_bus"
OBJECTS_TOPIC="/objects_raw"
SCAN_TOPIC="/scan"
POINTCLOUD_TOPIC="/radar_pointcloud"
LIFECYCLE_TIMEOUT=3   # seconds to wait before lifecycle activation

# -----------------------------------------------------------------------------
# Logging helpers
# -----------------------------------------------------------------------------
log_info()  { echo "[INFO]  $*"; }
log_ok()    { echo "[OK]    $*"; }
log_warn()  { echo "[WARN]  $*"; }
log_error() { echo "[ERROR] $*"; }

# -----------------------------------------------------------------------------
# Preflight checks
# -----------------------------------------------------------------------------
echo ""
echo "============================================================"
echo "  ARS408-21 Radar Pipeline — MiviaCar"
echo "  Sensor : FRONT (SensorID=0)"
echo "  Bus    : ${CAN_INTERFACE}"
echo "============================================================"
echo ""

# Check: can1 interface exists
if ! ip link show "${CAN_INTERFACE}" &>/dev/null; then
    log_error "CAN interface '${CAN_INTERFACE}' not found."
    log_error "Bring it up with:"
    log_error "  sudo ip link set ${CAN_INTERFACE} up type can bitrate 500000"
    exit 1
fi
log_ok "CAN interface '${CAN_INTERFACE}' is available."

# Check: ARS408 frames present on can1
log_info "Checking for ARS408 CAN frames on ${CAN_INTERFACE} (2s timeout)..."
FRAME_COUNT=$(timeout 2 candump "${CAN_INTERFACE}" 2>/dev/null \
              | grep -c "60[ABCD]" || true)
if [ "${FRAME_COUNT}" -eq 0 ]; then
    log_warn "No ARS408 frames detected on ${CAN_INTERFACE}."
    log_warn "Verify that the front radar is powered and connected."
else
    log_ok "${FRAME_COUNT} ARS408 frames detected on ${CAN_INTERFACE}."
fi

# Check: ROS2 environment sourced
if ! command -v ros2 &>/dev/null; then
    log_error "ros2 command not found. Source your ROS2 Humble setup first:"
    log_error "  source /opt/ros/humble/setup.bash"
    exit 1
fi
log_ok "ROS2 environment is sourced."

# Check: pe_ars408_ros package available
if ! ros2 pkg list 2>/dev/null | grep -q "pe_ars408_ros"; then
    log_error "Package 'pe_ars408_ros' not found. Source your workspace:"
    log_error "  source ~/abdel_ws/install/setup.bash"
    exit 1
fi
log_ok "Package 'pe_ars408_ros' found."
echo ""

# -----------------------------------------------------------------------------
# Cleanup handler
# -----------------------------------------------------------------------------
PIDS=()
cleanup() {
    echo ""
    log_info "Shutting down all radar nodes..."
    for pid in "${PIDS[@]}"; do
        kill "${pid}" 2>/dev/null || true
    done
    log_info "Radar pipeline stopped."
}
trap cleanup EXIT INT TERM

# -----------------------------------------------------------------------------
# Step 1 — Start socket_can_receiver (lifecycle node) on can1
# -----------------------------------------------------------------------------
log_info "[1/4] Starting socket_can_receiver on ${CAN_INTERFACE}..."

ros2 run ros2_socketcan socket_can_receiver_node_exe \
    --ros-args \
    -r __node:="${RECEIVER_NODE_NAME}" \
    -p interface:="${CAN_INTERFACE}" \
    -r /from_can_bus:="${RADAR_CAN_TOPIC}" &
PIDS+=($!)

log_info "Waiting ${LIFECYCLE_TIMEOUT}s for node to initialize..."
sleep "${LIFECYCLE_TIMEOUT}"

# -----------------------------------------------------------------------------
# Step 2 — Activate the lifecycle node
# -----------------------------------------------------------------------------
log_info "[2/4] Activating lifecycle node '${RECEIVER_NODE_NAME}'..."

ros2 lifecycle set "/${RECEIVER_NODE_NAME}" configure
ros2 lifecycle set "/${RECEIVER_NODE_NAME}" activate

log_ok "Lifecycle node '${RECEIVER_NODE_NAME}' is active."
log_ok "Publishing CAN frames on: ${RADAR_CAN_TOPIC}"
echo ""

# -----------------------------------------------------------------------------
# Step 3 — Start ARS408 driver node
# -----------------------------------------------------------------------------
log_info "[3/4] Starting ARS408 driver node..."

ros2 run pe_ars408_ros pe_ars408_node \
    --ros-args \
    -r __node:="${RADAR_NODE_NAME}" \
    -r /"${RADAR_NODE_NAME}"/input/frame:="${RADAR_CAN_TOPIC}" \
    -r /"${RADAR_NODE_NAME}"/output/objects:="${OBJECTS_TOPIC}" \
    -r /"${RADAR_NODE_NAME}"/output/scan:="${SCAN_TOPIC}" &
PIDS+=($!)
sleep 1

log_ok "ARS408 driver node started."
log_ok "Publishing tracks on  : ${OBJECTS_TOPIC}"
log_ok "Publishing scan on    : ${SCAN_TOPIC}"
echo ""

# -----------------------------------------------------------------------------
# Step 4 — Start PointCloud2 converter
# -----------------------------------------------------------------------------
log_info "[4/4] Starting PointCloud2 converter node..."

ros2 run pe_ars408_ros radar_pointcloud_node \
    --ros-args \
    -r /radar_pointcloud_node/input/tracks:="${OBJECTS_TOPIC}" \
    -r /radar_pointcloud_node/output/pointcloud:="${POINTCLOUD_TOPIC}" &
PIDS+=($!)
sleep 1

log_ok "PointCloud2 converter started."
log_ok "Publishing pointcloud on: ${POINTCLOUD_TOPIC}"
echo ""

# -----------------------------------------------------------------------------
# Summary
# -----------------------------------------------------------------------------
echo "============================================================"
echo "  Radar pipeline is running."
echo ""
echo "  Topic                  Type                      Rate"
echo "  -------                ----                      ----"
echo "  ${RADAR_CAN_TOPIC}    can_msgs/msg/Frame        ~300 Hz"
echo "  ${OBJECTS_TOPIC}          radar_msgs/msg/RadarTracks ~13.6 Hz"
echo "  ${SCAN_TOPIC}              radar_msgs/msg/RadarScan   ~13.6 Hz"
echo "  ${POINTCLOUD_TOPIC}   sensor_msgs/msg/PointCloud2 ~13.6 Hz"
echo ""
echo "  Verification:"
echo "    ros2 topic hz ${OBJECTS_TOPIC}"
echo "    ros2 topic echo ${OBJECTS_TOPIC} --once"
echo ""
echo "  RViz2 visualization (run in a separate terminal):"
echo "    ros2 run tf2_ros static_transform_publisher \\"
echo "      0 0 0 0 0 0 base_link radar_front_link"
echo "    ros2 run tf2_ros static_transform_publisher \\"
echo "      0 0 0 0 0 0 base_link os_sensor_top"
echo "    rviz2   (Fixed Frame: base_link)"
echo ""
echo "  Press Ctrl+C to stop all nodes."
echo "============================================================"
echo ""

# Keep script alive until Ctrl+C
wait
