#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/humble/setup.bash"
source "/home/colcon_ws/install/setup.bash"
sysctl -w net.core.rmem_max=2147483647
exec "$@"
