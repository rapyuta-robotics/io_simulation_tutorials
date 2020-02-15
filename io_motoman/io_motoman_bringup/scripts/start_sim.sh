#!/bin/bash
apt update
apt upgrade -y
roslaunch io_motoman_bringup hc10_sim.launch use_rapyuta.io:=true