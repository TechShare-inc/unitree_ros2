#!/bin/bash
echo "Start publishing the camera image"
source /opt/ros/foxy/setup.bash
source $HOME/unitree_ws/install/setup.bash

# Loop until the specified address is reachable
while ! ping -c 1 192.168.123.161 &> /dev/null; do
    echo "Waiting for 192.168.123.161 to become reachable..."
    sleep 1
done


ros2 run go2_legged_real go2_front_camera
