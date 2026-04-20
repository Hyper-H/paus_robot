#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="/root/projects/ag-repro"
LOG_DIR="${PROJECT_ROOT}/runtime_logs_clean"

mkdir -p "${LOG_DIR}"

source /opt/ros/humble/setup.bash
source "${PROJECT_ROOT}/ros2_ws/install/setup.bash"

pkill -f 'ag_marker_ros2/image_receiver_node' || true
pkill -f 'ag_marker_ros2/marker_pose_node' || true
pkill -f 'ag_marker_ros2/target_transform_node' || true
pkill -f 'ag_marker_ros2/fairino_control_node' || true

sleep 1

nohup bash -lc "source /opt/ros/humble/setup.bash && source ${PROJECT_ROOT}/ros2_ws/install/setup.bash && ros2 run ag_marker_ros2 image_receiver_node" \
  >"${LOG_DIR}/image_receiver.out" 2>"${LOG_DIR}/image_receiver.err" < /dev/null &

nohup bash -lc "source /opt/ros/humble/setup.bash && source ${PROJECT_ROOT}/ros2_ws/install/setup.bash && ros2 run ag_marker_ros2 marker_pose_node --ros-args -p camera_config_path:=/mnt/d/Projects/auto_arm/camera_config/camera.yaml" \
  >"${LOG_DIR}/marker_pose.out" 2>"${LOG_DIR}/marker_pose.err" < /dev/null &

nohup bash -lc "source /opt/ros/humble/setup.bash && source ${PROJECT_ROOT}/ros2_ws/install/setup.bash && ros2 run ag_marker_ros2 target_transform_node --ros-args -p extrinsics_path:=${PROJECT_ROOT}/configs/extrinsics.yaml -p config_path:=${PROJECT_ROOT}/configs/default.yaml" \
  >"${LOG_DIR}/target_transform.out" 2>"${LOG_DIR}/target_transform.err" < /dev/null &

nohup bash -lc "source /opt/ros/humble/setup.bash && source ${PROJECT_ROOT}/ros2_ws/install/setup.bash && ros2 run ag_marker_ros2 fairino_control_node --ros-args -p config_path:=${PROJECT_ROOT}/configs/default.yaml" \
  >"${LOG_DIR}/fairino_control.out" 2>"${LOG_DIR}/fairino_control.err" < /dev/null &

sleep 3

echo "Started marker stack. Logs in ${LOG_DIR}"
ros2 node list
