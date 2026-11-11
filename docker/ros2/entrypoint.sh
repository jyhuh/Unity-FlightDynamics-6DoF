#!/usr/bin/env bash
# Licensed under the Apache License, Version 2.0.
set -e

source /opt/ros/humble/setup.bash
if [[ -f /ros2_ws/install/setup.bash ]]; then
  source /ros2_ws/install/setup.bash
fi

UXRCE_PORT="${UXRCE_DDS_PORT:-8888}"
ACTION="${1:-agent}"

echo "======================================================================"
echo " [Container: ros2] ROS 2 Humble + Micro-XRCE-DDS Agent"
echo "  -> Listening for PX4 SITL uXRCE-DDS on UDP port : ${UXRCE_PORT}"
echo "  -> ROS_DOMAIN_ID                                : ${ROS_DOMAIN_ID:-0}"
echo "======================================================================"

case "$ACTION" in
  agent)
    exec env LD_LIBRARY_PATH="/opt/uxrce/lib:${LD_LIBRARY_PATH:-}" /opt/uxrce/bin/MicroXRCEAgent udp4 -p "$UXRCE_PORT"
    ;;

  offboard)
    python3 /ros2_ws/unity_px4_offboard_node.py "${@:2}"
    ;;

  *)
    exec "$@"
    ;;
esac
