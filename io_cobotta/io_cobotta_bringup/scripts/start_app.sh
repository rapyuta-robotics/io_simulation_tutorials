#!/bin/bash
apt update
apt upgrade -y
roslaunch io_cobotta_bringup cobotta_app.launch use_rapyuta.io:=true sim:=true