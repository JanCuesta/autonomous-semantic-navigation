# Source in EVERY laptop terminal used with turtlebot05.
source /opt/ros/jazzy/setup.bash
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/install/setup.bash"
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER="192.168.77.1:11811"
export ROS_SUPER_CLIENT=TRUE
