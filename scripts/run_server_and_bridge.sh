#!/usr/bin/env bash
set -euo pipefail

# Environment
ROS_IP=${ROS_IP:-0.0.0.0}
ROS_TCP_PORT=${ROS_TCP_PORT:-10000}
CONTROLLER_NAME=${CONTROLLER_NAME:-joint_trajectory_controller}
PREVIEW_TOPIC=${PREVIEW_TOPIC:-/trajectory_preview}
COMMAND_TOPIC=${COMMAND_TOPIC:-/joint_command}

# Setup ROS env
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
  source /opt/ros/jazzy/setup.bash
fi

# cd to workspace root (ros2-ar4-ws)
cd "$(dirname "$0")/.."

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

