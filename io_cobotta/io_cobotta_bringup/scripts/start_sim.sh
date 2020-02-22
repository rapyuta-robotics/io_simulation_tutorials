#!/bin/bash
apt update
apt upgrade -y
roslaunch io_motoman_bringup cobotta_sim.launch use_rapyuta.io:=true