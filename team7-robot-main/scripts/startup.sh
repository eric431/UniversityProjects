#!/bin/bash

source /home/ubuntu/.bashrc
cd /home/ubuntu/team7-robot/ros-webpage-testing
sudo python -m SimpleHTTPAuthServer 80 team7:password &
roscore &
sleep 5
roslaunch rosbridge_server rosbridge_websocket.launch &
roslaunch open_manipulator_controller open_manipulator_controller.launch use_platform:=false &
rosrun cmd_runner cmd_runner_server &
rosrun raspi_battery_monitor raspi_battery_monitor &
rosrun ltc2943_battery_monitor ltc2943_battery_monitor &
