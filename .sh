#!/bin/bash

gnome-terminal --tab -- bash -c "roscore; exec bash"

gnome-terminal --tab -- bash -c "rosrun stage_ros stageros ~/ir_workspace/src/socspioneer/data/meeting.world; exec bash"

gnome-terminal --tab -- bash -c "rosrun gmapping slam_gmapping scan:=base_scan; exec bash"

gnome-terminal --tab -- bash -c "rosrun rviz rviz; exec bash"

gnome-terminal --tab -- bash -c "roslaunch socspioneer keyboard_teleop.launch; exec bash"

gonme-terminal --tab -- bash -c "roslaunch pf_localisation pf.py; exec bash"

read -p "Press Enter to close the script"
