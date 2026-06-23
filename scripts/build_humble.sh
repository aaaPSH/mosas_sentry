#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

livox_dir="src/driver/livox_ros_driver2"
if [ -f "$livox_dir/package_ROS2.xml" ]; then
  cp -f "$livox_dir/package_ROS2.xml" "$livox_dir/package.xml"
fi

if [ -f /opt/ros/humble/setup.bash ]; then
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  set -u
fi

colcon build --symlink-install \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DROS_EDITION=ROS2 \
    -DDISTRO_ROS=humble
