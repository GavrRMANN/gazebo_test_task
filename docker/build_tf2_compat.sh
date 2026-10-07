#!/usr/bin/env bash
set -euo pipefail

task_dir=$(mktemp -d /tmp/tf2-compat.XXXXXX)
trap 'rm -rf "$task_dir"' EXIT
curl -fsSL https://github.com/ros2/geometry2/archive/refs/tags/0.25.22.tar.gz \
    -o "$task_dir/geometry2.tar.gz"
echo "c26c15a5fc430928741e4c774057f142f2b70484b2fd898dab3e1dca4ee5cdc6  $task_dir/geometry2.tar.gz" \
    | sha256sum -c -
mkdir "$task_dir/src"
tar -xzf "$task_dir/geometry2.tar.gz" -C "$task_dir/src" --strip-components=1
set +u
source /opt/ros/humble/setup.bash
set -u
cd "$task_dir/src"
colcon build --packages-select tf2_ros --packages-ignore tf2 tf2_msgs \
    --merge-install --install-base /opt/tf2_compat \
    --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
