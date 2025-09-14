#!/usr/bin/env bash
# -u can break sourcing ROS setup files due to unbound envs; we toggle it around source commands.
set -eo pipefail

# Environment
ROS_IP=${ROS_IP:-0.0.0.0}
ROS_TCP_PORT=${ROS_TCP_PORT:-10000}
CONTROLLER_NAME=${CONTROLLER_NAME:-joint_trajectory_controller}
PREVIEW_TOPIC=${PREVIEW_TOPIC:-/trajectory_preview}
COMMAND_TOPIC=${COMMAND_TOPIC:-/joint_command}

# Setup ROS env (tolerate unbound vars inside setup files)
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  set -u 2>/dev/null || true
fi

# cd to workspace root (ros2-ar4-ws)
cd "$(dirname "$0")/.."

# Source workspace overlay if available; try a quick build if missing
if [ ! -f "install/setup.bash" ]; then
  echo "[run_server_and_bridge] Workspace not built. Attempting quick build (rosdep+colcon)..."
  set +u
  rosdep update || true
  rosdep install --from-paths . --ignore-src -r -y || true
  set -u 2>/dev/null || true
  colcon build --symlink-install || true
fi

if [ -f "install/setup.bash" ]; then
  set +u
  # shellcheck disable=SC1091
  source install/setup.bash
  set -u 2>/dev/null || true
else
  echo "[run_server_and_bridge] Warning: install/setup.bash not found; proceeding without overlay."
fi

echo "Starting ROS TCP server on ${ROS_IP}:${ROS_TCP_PORT}..."

# Start server in background and ensure cleanup on exit
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=${ROS_IP} -p ROS_TCP_PORT:=${ROS_TCP_PORT} &
server_pid=$!
trap 'kill ${server_pid} 2>/dev/null || true' EXIT INT TERM

sleep 0.5

echo "Starting MoveIt↔Unity bridge (controller=${CONTROLLER_NAME})..."
exec python3 src/moveit_bridge.py --ros-args \
  -p controller_name:=${CONTROLLER_NAME} \
  -p preview_topic:=${PREVIEW_TOPIC} \
  -p command_topic:=${COMMAND_TOPIC}
