#!/bin/bash
apt update
apt upgrade -y
roslaunch ur_robotiq_bringup app.launch use_rapyuta.io:=true sim:=true