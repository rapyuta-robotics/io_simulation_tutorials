#!/bin/bash
apt update
apt upgrade -y
roslaunch ur_robotiq_bringup sim.launch use_rapyuta.io:=true