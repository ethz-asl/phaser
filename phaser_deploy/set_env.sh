#!/usr/bin/env bash

. /usr/home/ws/devel/setup.bash
host_ip=$(ip route | awk '/default/ {print $3}')
container_ip=$(hostname -I | awk '{gsub(/^ +| +$/,"")} {print $0}')
export ROS_MASTER_URI=http://${host_ip}:11311
export ROS_IP=${container_ip}
exec "$@"
