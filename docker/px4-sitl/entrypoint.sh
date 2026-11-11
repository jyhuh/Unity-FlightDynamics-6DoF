#!/usr/bin/env bash
# Licensed under the Apache License, Version 2.0.
# Container 1 Entrypoint: Works with Colima (containerd/nerdctl) and Apple `container` (Virtualization.framework)
set -euo pipefail

PX4_DIR="${PX4_DIR:-/workspace/PX4-Autopilot}"
BUILD_DIR="${PX4_BUILD_DIR:-$PX4_DIR/build/px4_sitl_container}"
RAW_SIM_MODEL="${PX4_SIM_MODEL:-iris}"
PX4_UXRCE_DDS_HOST="${PX4_UXRCE_DDS_HOST:-ros2}"
PX4_UXRCE_DDS_PORT="${PX4_UXRCE_DDS_PORT:-8888}"
ACTION="${1:-run}"

# Map friendly airframe names to PX4 v1.15 ROMFS init.d-posix/airframes/* filenames
normalize_px4_sim_model() {
  local m="$1"
  case "$m" in
    iris|quad|multirotor)
      echo "none_iris"
      ;;
    plane|fixedwing)
      echo "gazebo-classic_plane"
      ;;
    vtol|standard_vtol)
      echo "gazebo-classic_standard_vtol"
      ;;
    *)
      echo "$m"
      ;;
  esac
}

PX4_SIM_MODEL="$(normalize_px4_sim_model "$RAW_SIM_MODEL")"

# ------------------------------------------------------------------------------
# 1. Resolve macOS Host IP (for Unity TCP :4560 and QGroundControl UDP :14550)
#    Priority:
#      a) Explicit PX4_SIM_HOST_ADDR environment variable
#      b) Colima / Lima host alias (host.lima.internal -> 192.168.5.2)
#      c) Explicit PX4_SIM_HOSTNAME if resolvable
#      d) Apple `container` / vmnet default gateway (e.g., 192.168.64.1)
# ------------------------------------------------------------------------------
resolve_ipv4() {
  local target="$1"
  getent ahostsv4 "$target" 2>/dev/null | awk '{print $1; exit}' || true
}

resolve_macos_host_ip() {
  if [[ -n "${PX4_SIM_HOST_ADDR:-}" ]]; then
    echo "$PX4_SIM_HOST_ADDR"
    return
  fi

  local ip=""
  for candidate in "${PX4_SIM_HOSTNAME:-}" "host.lima.internal" "host.containers.internal" "host.docker.internal"; do
    if [[ -n "$candidate" && "$candidate" != "auto" ]]; then
      ip="$(resolve_ipv4 "$candidate")"
      if [[ -n "$ip" ]]; then
        echo "$ip"
        return
      fi
    fi
  done

  # Fallback for Apple `container` (macOS Virtualization.framework vmnet gateway is the macOS host)
  ip="$(ip route show default 2>/dev/null | awk '/default/ {print $3; exit}')"
  echo "${ip:-192.168.64.1}"
}

# ------------------------------------------------------------------------------
# 2. Resolve ROS 2 Container IPv4 (PX4 uxrce_dds_client -h requires numeric IPv4)
# ------------------------------------------------------------------------------
resolve_ros2_dds_ip() {
  if [[ "$PX4_UXRCE_DDS_HOST" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
    echo "$PX4_UXRCE_DDS_HOST"
    return
  fi

  local ip=""
  for attempt in {1..15}; do
    ip="$(resolve_ipv4 "$PX4_UXRCE_DDS_HOST")"
    if [[ -n "$ip" ]]; then
      echo "$ip"
      return
    fi
    sleep 0.5
  done

  echo "127.0.0.1"
}

MACOS_HOST_IP="$(resolve_macos_host_ip)"
ROS2_DDS_IP="$(resolve_ros2_dds_ip)"

echo "======================================================================"
echo " [Container: px4-sitl] PX4-Autopilot v1.15 SITL (Colima / Apple Container)"
echo "  -> macOS Unity 6-DOF Host IP  : ${MACOS_HOST_IP}:4560 (TCP Lockstep)"
echo "  -> ROS 2 uXRCE-DDS Agent IP   : ${ROS2_DDS_IP}:${PX4_UXRCE_DDS_PORT} (UDP)"
echo "  -> QGroundControl Telemetry   : ${MACOS_HOST_IP}:14550 (UDP)"
echo "  -> Airframe Model             : ${PX4_SIM_MODEL}"
echo "======================================================================"

ensure_px4_repo() {
  git config --global --add safe.directory "*" || true
  if [[ ! -d "$PX4_DIR/.git" ]]; then
    echo "[px4-sitl] Cloning PX4-Autopilot (v1.15.0) into $PX4_DIR..."
    git clone --depth 1 --branch v1.15.0 --recursive https://github.com/PX4/PX4-Autopilot.git "$PX4_DIR"
  fi
}

build_px4_sitl() {
  ensure_px4_repo
  if [[ ! -f "$BUILD_DIR/build.ninja" ]]; then
    echo "[px4-sitl] Configuring px4_sitl_default inside Linux container ($BUILD_DIR)..."
    mkdir -p "$BUILD_DIR"
    cmake -S "$PX4_DIR" -B "$BUILD_DIR" -G Ninja -DCONFIG=px4_sitl_default
  fi

  if [[ ! -x "$BUILD_DIR/bin/px4" || "$ACTION" == "build" ]]; then
    echo "[px4-sitl] Compiling px4_sitl_default inside Linux container ($BUILD_DIR)..."
    cmake --build "$BUILD_DIR"
    echo "[px4-sitl] Build finished: $BUILD_DIR/bin/px4"
  else
    echo "[px4-sitl] Using cached Linux PX4 SITL binary: $BUILD_DIR/bin/px4"
  fi
}

patch_container_init_scripts() {
  local RC_S="$BUILD_DIR/etc/init.d-posix/rcS"
  local RC_PARAMS="$BUILD_DIR/etc/init.d-posix/px4-rc.params"
  local RC_MAVLINK="$BUILD_DIR/etc/init.d-posix/px4-rc.mavlink"

  # 1. Point uxrce_dds_client to the resolved numeric IPv4 of the ROS 2 container
  if [[ -f "$RC_S" ]]; then
    sed -i "s|uxrce_dds_client start -t udp -h .* -p|uxrce_dds_client start -t udp -h ${ROS2_DDS_IP} -p|g" "$RC_S"
  fi

  # 2. Clean any custom CA parameter overrides from px4-rc.params so PX4 uses valid airframe defaults (e.g. 10016_none_iris)
  if [[ -f "$RC_PARAMS" ]]; then
    sed -i '/# BEGIN_UNITY_STAGE1_CA_PARAMS/,/# END_UNITY_STAGE1_CA_PARAMS/d' "$RC_PARAMS"
  fi

  # 3. Forward QGroundControl GCS MAVLink stream to macOS host IP (${MACOS_HOST_IP}:14550)
  if [[ -f "$RC_MAVLINK" ]]; then
    sed -i "s|mavlink start -x -u \$udp_gcs_port_local -r 4000000 -f.*|mavlink start -x -u \$udp_gcs_port_local -r 4000000 -f -t ${MACOS_HOST_IP} -o 14550|g" "$RC_MAVLINK"
  fi
}

case "$ACTION" in
  build)
    build_px4_sitl
    ;;

  run)
    build_px4_sitl
    patch_container_init_scripts

    ROOTFS_DIR="$BUILD_DIR/tmp/rootfs"
    ETC_DIR="$BUILD_DIR/etc"
    mkdir -p "$ROOTFS_DIR"

    cd "$ROOTFS_DIR"
    export PX4_SIM_MODEL="$PX4_SIM_MODEL"
    export PX4_SIMULATOR="none"
    unset PX4_SIM_HOSTNAME
    export PX4_SIM_HOST_ADDR="$MACOS_HOST_IP"

    echo "[px4-sitl] Connecting to Unity TCP server at ${MACOS_HOST_IP}:4560 (PX4_SIM_MODEL=${PX4_SIM_MODEL})..."
    exec "$BUILD_DIR/bin/px4" -d "$ETC_DIR" -w "$ROOTFS_DIR" -s etc/init.d-posix/rcS
    ;;

  *)
    exec "$@"
    ;;
esac
