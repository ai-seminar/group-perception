#!/bin/bash
roscore &
roslaunch openni_launch openni.launch load_driver:=false &
rosbag play --clock kiba_pointcloud.bag --loop
