#!/bin/bash
apt update
apt upgrade -y
roslaunch io_cobotta_bringup cobotta_sim.launch use_rapyuta.io:=true