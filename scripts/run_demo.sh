#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
cd /workspace
colcon build --packages-up-to autonomy_bringup --symlink-install
source /workspace/install/setup.bash
source /opt/tf2_compat/setup.bash

for attempt in {1..60}; do
    if python3 -c 'import socket; s = socket.socket(socket.AF_UNIX); s.connect("/tmp/.X11-unix/X1"); s.close()' 2>/dev/null; then
        exec ros2 launch autonomy_bringup demo.launch.py "$@"
    fi
    sleep 0.2
done
echo "X display :1 is unavailable" >&2
exit 1
