#!/bin/sh

xterm -e "roslaunch toycar robot.launch" &
sleep 2
xterm -e "roslaunch toycar controller.launch" &
sleep 2
xterm -e "roslaunch toycar slam_hector.launch" &
sleep 2
