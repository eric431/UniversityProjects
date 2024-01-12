#!/bin/bash

source /home/ubuntu/.bashrc
script_name=`basename "$0"`

cd /home/ubuntu/catkin_ws/src/open_manipulator

branch=$(git branch | sed -n -e 's/^\* \(.*\)/\1/p')

if [ "${script_name}" == "rotortool" ] && [ "${branch}" = "master" ]; then
    echo "rotortool is desired and branch is master, quitting..."
    exit 0
elif [ "${script_name}" == "rotortool" ] && [ "${branch}" != "master" ]; then
    echo "rotortool is desired and branch is not master, changing to master..."
    git checkout master
elif [ "${script_name}" == "notool" ] && [ "${branch}" = "team7-notool" ]; then
    echo "notool is desired and branch is team7-notool, quitting..."
    exit 0
elif [ "${script_name}" == "notool" ] && [ "${branch}" != "team7-notool" ]; then
    echo "rotortool is desired and branch is not team7-notool, changing to team7-notool..."
    git checkout team7-notool
fi

cd /home/ubuntu/catkin_ws
catkin_make

rosnode kill /open_manipulator_controller

roslaunch open_manipulator_controller open_manipulator_controller.launch &

exit 0