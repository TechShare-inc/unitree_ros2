#!/bin/bash
echo "Start publishing the camera image"
exec > /tmp/unitree_ros2_start_camera.log 2>&1
source /opt/ros/foxy/setup.bash
source /home/unitree/unitree_ws/install/setup.bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
# Loop until the specified address is reachable
while ! ping -c 1 192.168.123.161 &> /dev/null; do
    echo "Waiting for 192.168.123.161 to become reachable..."
    sleep 1
done


ros2 run go2_legged_real go2_front_camera
