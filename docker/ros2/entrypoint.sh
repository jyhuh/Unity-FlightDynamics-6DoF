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

  nav2)
    pkill -f "[u]nity_px4_offboard_node.py|[u]nity_nav2_px4_bridge.py|[n]av2_drone.launch.py|[r]viz2|[c]ontroller_server|[p]lanner_server|[b]t_navigator|[b]ehavior_server|[s]moother_server|[w]aypoint_follower|[v]elocity_smoother|[l]ifecycle_manager|Xvfb :1|x11vnc|websockify" >/dev/null 2>&1 || true
    sleep 0.5
    export DISPLAY=:1
    Xvfb :1 -screen 0 1440x900x24 >/tmp/xvfb.log 2>&1 &
    sleep 1
    fluxbox >/tmp/fluxbox.log 2>&1 &
    x11vnc -display :1 -nopw -listen 0.0.0.0 -rfbport 5900 -forever -shared >/tmp/x11vnc.log 2>&1 &
    websockify --web=/usr/share/novnc 6080 localhost:5900 >/tmp/novnc.log 2>&1 &
    echo "[ros2-nav2] noVNC RViz2 available at http://localhost:6080/vnc.html"
    exec ros2 launch /ros2_ws/nav2_drone.launch.py
    ;;

  *)
    exec "$@"
    ;;
esac
