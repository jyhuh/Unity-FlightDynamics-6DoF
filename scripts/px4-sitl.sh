#!/usr/bin/env bash
# Licensed under the Apache License, Version 2.0.
# Non-Docker Container Stack Manager for PX4-Autopilot SITL + ROS 2 Humble
# Supports:
#   1. Colima (containerd + nerdctl, Apple Virtualization.framework + virtiofs)
#   2. Apple `container` CLI (https://github.com/apple/container native micro-VMs)
set -euo pipefail

export PATH="/opt/homebrew/bin:/usr/local/bin:$PATH"

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4_DIR="${PX4_DIR:-$ROOT_DIR/PX4-Autopilot}"
CONFIG_DIR="$ROOT_DIR/docker/config"
COMPOSE_FILE="$ROOT_DIR/docker-compose.yml"
COLIMA_PROFILE="${COLIMA_PROFILE:-flightdynamics}"

ACTION="${1:-help}"
MODEL="${2:-iris}"

mkdir -p "$CONFIG_DIR"

detect_backend() {
  local pref="${CONTAINER_BACKEND:-auto}"
  if [[ "$pref" == "colima" || "$pref" == "apple" ]]; then
    echo "$pref"
    return
  fi

  # Prefer Colima if its 'flightdynamics' containerd VM is already running
  if command -v colima >/dev/null 2>&1; then
    if colima status --profile "$COLIMA_PROFILE" 2>&1 | grep -qi "is running"; then
      echo "colima"
      return
    fi
  fi

  if command -v container >/dev/null 2>&1; then
    echo "apple"
  elif command -v colima >/dev/null 2>&1; then
    echo "colima"
  else
    echo "colima"
  fi
}

BACKEND="$(detect_backend)"

echo "======================================================================"
echo " FlightDynamics 6-DOF: Non-Docker Container Stack (Backend: ${BACKEND})"
echo "======================================================================"

ensure_px4_repo() {
  if [[ -d "$PX4_DIR/.git" ]]; then
    echo "[PX4 SITL] PX4-Autopilot repository ready at: $PX4_DIR"
  else
    echo "[PX4 SITL] Cloning PX4-Autopilot (v1.15.0) into $PX4_DIR..."
    git clone --depth 1 --branch v1.15.0 --recursive https://github.com/PX4/PX4-Autopilot.git "$PX4_DIR"
  fi
}

# ==============================================================================
# Backend 1: Colima (containerd runtime + colima nerdctl — Zero Docker)
# ==============================================================================
ensure_colima_containerd() {
  if ! command -v colima >/dev/null 2>&1; then
    echo "[Error] Colima is not installed. Run: brew install colima"
    exit 1
  fi

  local status_out=""
  status_out="$(colima status --profile "$COLIMA_PROFILE" 2>&1 || true)"
  if ! echo "$status_out" | grep -qi "is running"; then
    echo "[Colima] Starting dedicated '$COLIMA_PROFILE' VM with containerd runtime (Apple Virtualization.framework + virtiofs)..."
    colima start --profile "$COLIMA_PROFILE" \
      --runtime containerd \
      --vm-type vz \
      --mount-type virtiofs \
      --cpu 4 \
      --memory 8
  elif ! echo "$status_out" | grep -qi "runtime: containerd"; then
    echo "[Colima] Profile '$COLIMA_PROFILE' was running without containerd runtime. Restarting with '--runtime containerd'..."
    colima stop --profile "$COLIMA_PROFILE"
    colima start --profile "$COLIMA_PROFILE" \
      --runtime containerd \
      --vm-type vz \
      --mount-type virtiofs \
      --cpu 4 \
      --memory 8
  fi

  # Ensure containerd runtime service inside Colima VM is active and accessible
  if ! colima --profile "$COLIMA_PROFILE" nerdctl -- version >/dev/null 2>&1; then
    echo "[Colima] containerd runtime service inactive inside VM. Reviving systemd containerd.service..."
    colima ssh --profile "$COLIMA_PROFILE" -- sudo systemctl restart containerd || true
    colima ssh --profile "$COLIMA_PROFILE" -- sudo systemctl enable containerd || true
    sleep 1
  fi
}

nerdctl_cmd() {
  colima --profile "$COLIMA_PROFILE" nerdctl -- "$@"
}

colima_get_container_ip() {
  local cname="$1"
  nerdctl_cmd exec "$cname" bash -c "hostname -I | awk '{print \$1}'" 2>/dev/null | tr -d '\r\n ' || true
}

colima_cleanup_containers() {
  nerdctl_cmd rm -f unity-px4-sitl unity-ros2-dds unitydrone-px4-sitl-1 unitydrone-ros2-1 >/dev/null 2>&1 || true
}

# ==============================================================================
# Backend 2: Apple `container` CLI (macOS native Virtualization.framework)
# ==============================================================================
ensure_apple_container() {
  if ! command -v container >/dev/null 2>&1; then
    echo "[Apple Container] 'container' CLI not found. Installing via Homebrew Cask..."
    brew install --cask container
  fi
  container system start >/dev/null 2>&1 || true
}

apple_get_container_ip() {
  local cname="$1"
  container exec "$cname" bash -c "hostname -I | awk '{print \$1}'" 2>/dev/null | tr -d '\r\n ' || true
}

# ==============================================================================
# Commands
# ==============================================================================
case "$ACTION" in
  clone)
    ensure_px4_repo
    ;;

  setup|build)
    ensure_px4_repo
    if [[ "$BACKEND" == "apple" ]]; then
      ensure_apple_container
      echo "[Apple Container] Building 'flightdynamics6dof/ros2-humble:latest'..."
      container build -t flightdynamics6dof/ros2-humble:latest -f "$ROOT_DIR/docker/ros2/Dockerfile" "$ROOT_DIR/docker/ros2"

      echo "[Apple Container] Building 'flightdynamics6dof/px4-sitl:latest'..."
      container build -t flightdynamics6dof/px4-sitl:latest -f "$ROOT_DIR/docker/px4-sitl/Dockerfile" "$ROOT_DIR/docker/px4-sitl"

      echo "[Apple Container] Compiling PX4 SITL binary inside Linux container..."
      container run --rm \
        --volume "$PX4_DIR:/workspace/PX4-Autopilot" \
        flightdynamics6dof/px4-sitl:latest build
    else
      ensure_colima_containerd
      colima_cleanup_containers
      echo "[Colima containerd] Building 'flightdynamics6dof/ros2-humble:latest'..."
      nerdctl_cmd build -t flightdynamics6dof/ros2-humble:latest -f "$ROOT_DIR/docker/ros2/Dockerfile" "$ROOT_DIR/docker/ros2"

      echo "[Colima containerd] Building 'flightdynamics6dof/px4-sitl:latest'..."
      nerdctl_cmd build -t flightdynamics6dof/px4-sitl:latest -f "$ROOT_DIR/docker/px4-sitl/Dockerfile" "$ROOT_DIR/docker/px4-sitl"

      echo "[Colima containerd] Compiling PX4 SITL binary inside Linux container..."
      nerdctl_cmd run --rm \
        -v "$PX4_DIR:/workspace/PX4-Autopilot" \
        flightdynamics6dof/px4-sitl:latest build
    fi
    echo "[Success] Setup complete! Run './scripts/px4-sitl.sh up' to start both containers."
    ;;

  ensure)
    ensure_px4_repo
    if [[ "$BACKEND" == "apple" ]]; then
      ensure_apple_container
      if container ps 2>/dev/null | grep -q "unity-px4-sitl"; then
        echo "[Apple Container] 'unity-px4-sitl' and 'unity-ros2-dds' are already running (keeping live QGC/ROS2 connection)."
        exit 0
      fi
    else
      ensure_colima_containerd
      if nerdctl_cmd ps 2>/dev/null | grep -q "unity-px4-sitl"; then
        echo "[Colima containerd] 'unity-px4-sitl' and 'unity-ros2-dds' are already running (keeping live QGC/ROS2 connection)."
        exit 0
      fi
    fi
    exec "$0" up "$MODEL"
    ;;

  up|run)
    ensure_px4_repo
    if [[ "$BACKEND" == "apple" ]]; then
      ensure_apple_container
      container stop unity-px4-sitl unity-ros2-dds >/dev/null 2>&1 || true
      container rm unity-px4-sitl unity-ros2-dds >/dev/null 2>&1 || true

      echo "[Apple Container] Starting 'unity-ros2-dds' (ROS 2 Humble + Micro-XRCE-DDS Agent)..."
      container run -d --name unity-ros2-dds \
        flightdynamics6dof/ros2-humble:latest

      sleep 1
      ROS2_IP="$(apple_get_container_ip unity-ros2-dds)"
      echo "[Apple Container] 'unity-ros2-dds' running at IP: ${ROS2_IP:-auto}"

      echo "[Apple Container] Starting 'unity-px4-sitl' (Model: $MODEL -> macOS Unity TCP 4560)..."
      container run -d --name unity-px4-sitl \
        --env "PX4_SIM_MODEL=$MODEL" \
        --env "PX4_SIMULATOR=none" \
        --env "PX4_UXRCE_DDS_HOST=${ROS2_IP:-127.0.0.1}" \
        --env "PX4_UXRCE_DDS_PORT=8888" \
        --volume "$PX4_DIR:/workspace/PX4-Autopilot" \
        --volume "$CONFIG_DIR:/workspace/config" \
        flightdynamics6dof/px4-sitl:latest
    else
      ensure_colima_containerd
      colima_cleanup_containers

      echo "[Colima containerd] Starting 'unity-ros2-dds' (ROS 2 Humble + Micro-XRCE-DDS Agent)..."
      nerdctl_cmd run -d --name unity-ros2-dds \
        --network host \
        -e ROS_DOMAIN_ID=0 \
        -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
        -e UXRCE_AGENT_PORT=8888 \
        flightdynamics6dof/ros2-humble:latest

      echo "[Colima containerd] Starting 'unity-px4-sitl' (Model: $MODEL -> macOS Unity TCP 4560 & QGC UDP 14550)..."
      nerdctl_cmd run -d --name unity-px4-sitl \
        --network host \
        -e "PX4_SIM_MODEL=$MODEL" \
        -e "PX4_SIMULATOR=none" \
        -e "PX4_SIM_HOSTNAME=host.lima.internal" \
        -e "PX4_UXRCE_DDS_HOST=127.0.0.1" \
        -e "PX4_UXRCE_DDS_PORT=8888" \
        -v "$PX4_DIR:/workspace/PX4-Autopilot" \
        -v "$CONFIG_DIR:/workspace/config" \
        flightdynamics6dof/px4-sitl:latest
    fi

    echo ""
    echo "[Running] Both containers are active on ${BACKEND}:"
    echo "  1. unity-px4-sitl -> Connects to macOS Unity TCP server (:4560)"
    echo "  2. unity-ros2-dds -> MicroXRCEAgent on UDP :8888 (bridging /fmu/in/* & /fmu/out/*)"
    ;;

  down|stop)
    if [[ "$BACKEND" == "apple" ]]; then
      echo "[Apple Container] Stopping 'unity-px4-sitl' and 'unity-ros2-dds'..."
      container stop unity-px4-sitl unity-ros2-dds >/dev/null 2>&1 || true
      container rm unity-px4-sitl unity-ros2-dds >/dev/null 2>&1 || true
    else
      echo "[Colima containerd] Stopping 'unity-px4-sitl' and 'unity-ros2-dds' containers..."
      colima_cleanup_containers
      echo "[Success] Both containers stopped (Colima VM is kept warm for fast restarts)."
      echo "  -> To completely stop the Colima VM profile: ./scripts/px4-sitl.sh vm-stop"
    fi
    ;;

  restart)
    echo "[Restarting] Stopping and restarting containers for model '${MODEL}'..."
    "$0" down
    "$0" up "$MODEL"
    ;;

  ps|status)
    echo "[Container Status (${BACKEND})]"
    if [[ "$BACKEND" == "apple" ]]; then
      container ps -a 2>&1 || true
    else
      nerdctl_cmd ps -a 2>&1 || true
    fi
    ;;

  status-info)
    if [[ "$BACKEND" == "apple" ]]; then
      if command -v container >/dev/null 2>&1; then
        echo "VM_STATUS=RUNNING"
        echo "CONTAINERD_STATUS=ACTIVE"
      else
        echo "VM_STATUS=STOPPED"
        echo "CONTAINERD_STATUS=INACTIVE"
      fi
      container ps -a --format "{{.Names}}|{{.Status}}|{{.Image}}|{{.Ports}}" 2>/dev/null || true
    else
      if colima status --profile "$COLIMA_PROFILE" 2>&1 | grep -qi "is running"; then
        echo "VM_STATUS=RUNNING"
        if colima --profile "$COLIMA_PROFILE" nerdctl -- version >/dev/null 2>&1; then
          echo "CONTAINERD_STATUS=ACTIVE"
        else
          echo "CONTAINERD_STATUS=DEGRADED"
        fi
        colima nerdctl -p "$COLIMA_PROFILE" -- ps -a --format "{{.Names}}|{{.Status}}|{{.Image}}|{{.Ports}}" 2>/dev/null || true
      else
        echo "VM_STATUS=STOPPED"
        echo "CONTAINERD_STATUS=INACTIVE"
      fi
    fi
    ;;

  vm-start)
    if [[ "$BACKEND" == "apple" ]]; then
      ensure_apple_container
      echo "[Apple Container] Apple virtualization system ready."
    else
      ensure_colima_containerd
      echo "[Colima containerd] '${COLIMA_PROFILE}' VM and containerd runtime are running and healthy."
    fi
    ;;

  vm-status)
    if [[ "$BACKEND" == "apple" ]]; then
      echo "[Apple Container Status]"
      container ps 2>&1 || true
    else
      echo "[Colima Profile Status]"
      colima status --profile "$COLIMA_PROFILE" 2>&1 || true
      echo ""
      echo "[VM containerd Service Status]"
      colima ssh --profile "$COLIMA_PROFILE" -- systemctl status containerd --no-pager 2>&1 | head -n 12 || true
      echo ""
      echo "[Running Containers via nerdctl]"
      nerdctl_cmd ps -a 2>&1 || true
    fi
    ;;

  vm-repair)
    if [[ "$BACKEND" == "apple" ]]; then
      ensure_apple_container
      echo "[Apple Container] Repaired system runtime."
    else
      echo "[Colima containerd] Repairing and restarting containerd inside '${COLIMA_PROFILE}' VM..."
      ensure_colima_containerd
      colima ssh --profile "$COLIMA_PROFILE" -- sudo systemctl daemon-reload || true
      colima ssh --profile "$COLIMA_PROFILE" -- sudo systemctl restart containerd || true
      colima ssh --profile "$COLIMA_PROFILE" -- sudo systemctl enable containerd || true
      sleep 1
      if colima --profile "$COLIMA_PROFILE" nerdctl -- version >/dev/null 2>&1; then
        echo "[Success] containerd service successfully repaired and operational!"
      else
        echo "[Warning] containerd service restarted, but check status with ./scripts/px4-sitl.sh vm-status"
      fi
    fi
    ;;

  vm-stop|shutdown)
    if [[ "$BACKEND" == "apple" ]]; then
      echo "[Apple Container] Stopping containers..."
      container stop unity-px4-sitl unity-ros2-dds >/dev/null 2>&1 || true
      container rm unity-px4-sitl unity-ros2-dds >/dev/null 2>&1 || true
    else
      echo "[Colima containerd] Stopping containers and completely shutting down '${COLIMA_PROFILE}' VM..."
      colima_cleanup_containers
      colima stop --profile "$COLIMA_PROFILE" || true
      echo "[Success] Colima profile '${COLIMA_PROFILE}' VM is now completely stopped."
    fi
    ;;

  logs)
    if [[ "$BACKEND" == "apple" ]]; then
      container logs -f unity-px4-sitl
    else
      nerdctl_cmd logs -f unity-px4-sitl
    fi
    ;;

  pxh)
    CMD_STR="${*:2}"
    if [[ -z "$CMD_STR" ]]; then
      echo "Usage: ./scripts/px4-sitl.sh pxh <command>  (e.g., ./scripts/px4-sitl.sh pxh commander takeoff)"
      exit 1
    fi
    echo "[pxh>] Sending 'px4-${CMD_STR}' to unity-px4-sitl (${BACKEND})..."
    if [[ "$BACKEND" == "apple" ]]; then
      container exec unity-px4-sitl bash -lc "/workspace/PX4-Autopilot/build/px4_sitl_container/bin/px4-${CMD_STR}"
    else
      nerdctl_cmd exec unity-px4-sitl bash -lc "/workspace/PX4-Autopilot/build/px4_sitl_container/bin/px4-${CMD_STR}"
    fi
    ;;

  ros2-topics)
    echo "[ROS 2] Listing active uORB / DDS topics in unity-ros2-dds (${BACKEND})..."
    if [[ "$BACKEND" == "apple" ]]; then
      container exec unity-ros2-dds bash -lc "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && ros2 topic list"
    else
      nerdctl_cmd exec unity-ros2-dds bash -lc "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && ros2 topic list"
    fi
    ;;

  ros2-offboard)
    echo "[ROS 2] Starting Autonomous Offboard Trajectory Node inside unity-ros2-dds (${BACKEND})..."
    if [[ "$BACKEND" == "apple" ]]; then
      container exec unity-ros2-dds bash -lc "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && python3 /ros2_ws/unity_px4_offboard_node.py ${*:2}"
    else
      nerdctl_cmd exec unity-ros2-dds bash -lc "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && python3 /ros2_ws/unity_px4_offboard_node.py ${*:2}"
    fi
    ;;

  ros2-shell)
    if [[ "$BACKEND" == "apple" ]]; then
      container exec -it unity-ros2-dds bash
    else
      nerdctl_cmd exec -it unity-ros2-dds bash
    fi
    ;;

  *)
    echo "Usage: [CONTAINER_BACKEND=colima|apple] ./scripts/px4-sitl.sh <command> [args]"
    echo ""
    echo "Non-Docker Container Commands (Colima containerd/nerdctl or Apple 'container'):"
    echo "  setup                  Start Colima(containerd) or Apple 'container', build images & compile PX4 SITL"
    echo "  up [iris|plane]        Start 'unity-px4-sitl' + 'unity-ros2-dds' containers"
    echo "  down                   Stop and remove both containers (keeps Colima VM warm)"
    echo "  vm-stop                Completely shut down both containers and the Colima VM profile"
    echo "  logs                   Tail live logs from PX4 SITL / ROS 2"
    echo "  pxh <cmd>              Send a PX4 shell command (e.g., './scripts/px4-sitl.sh pxh commander takeoff')"
    echo "  ros2-topics            List active ROS 2 /fmu/in/* and /fmu/out/* topics"
    echo "  ros2-offboard          Run the ROS 2 Offboard orbit controller node"
    echo "  ros2-shell             Open an interactive bash shell inside the ROS 2 Humble container"
    ;;
esac
