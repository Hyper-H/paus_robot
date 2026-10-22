#!/usr/bin/env bash
set -eo pipefail
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ! -f "$task_root/install/setup.bash" ]]; then
  echo "Build paus_interfaces, paus_ultrasound_ros2, paus_ui and paus_bringup first." >&2
  exit 1
fi
source /opt/ros/humble/setup.bash
source "$HOME/miniconda3/etc/profile.d/conda.sh"
conda activate paus_robot
# Include the dependency workspaces recorded by colcon for partial builds.
source "$task_root/install/setup.bash"
exec ros2 launch paus_bringup ui.launch.py \
  start_image_receiver:=false start_camera_bridge:=false start_calibration_node:=false \
  execute_motion:=false enable_ultrasound:=true \
  ultrasound_listen_host:="${1:-127.0.0.1}" \
  ultrasound_listen_port:="${2:-55001}" ui_port:="${3:-8088}"
