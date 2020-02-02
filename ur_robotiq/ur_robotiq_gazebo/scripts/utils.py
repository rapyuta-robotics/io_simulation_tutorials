#!/usr/bin/env python
import os
from subprocess import Popen, PIPE
import rospy
import rospkg
import roslaunch
from std_msgs.msg import String

robot_name=''
rospack = rospkg.RosPack()
path = rospack.get_path('ur_robotiq_gazebo')

def get_robot_name():
	if rospy.get_param('/use_rapyuta_io', False) :	
		def peers_cb(msg):
			global robot_name 
			robot_name = msg.data.split(',')[0]
			rospy.loginfo('Robot name: '+robot_name)
			if not robot_name:
				rospy.logfatal('Robot name can nott be empty')
				exit(-1)
		global robot_name
		rospy.Subscriber('/rapyuta_io_peers', String, peers_cb)
		rospy.wait_for_message('/rapyuta_io_peers', String)
		while(not robot_name):
			rospy.loginfo('Waiting rapytua_io_peers.......')
			rospy.sleep(1)

	else :
		 robot_name = rospy.get_param('~robot_name')

	return robot_name


def parse_args(robot_name):
	# sanity check
	if not robot_name:
		rospy.logwarn('robot name is empty. robot name become hitachi_forklift')

	return (' robot_name:='+robot_name) if robot_name != '' else '' 

def launch_spawn_params(robot_name):
	uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
	roslaunch.configure_logging(uuid)

	cli_args = [path+'/launch/ur_robotiq_gazebo_spawn_params.launch' , parse_args(robot_name)]
	launch_files = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], cli_args[1:])]
	rospy.logfatal(cli_args)
	parent = roslaunch.parent.ROSLaunchParent(uuid, launch_files)
	parent.start()


# def launch_spawner(robot_name, x=0, y=0, yaw=0,):
# 	robot_name_arg = parse_args(robot_name)

# 	# spawner
# 	args_str = "roslaunch ur_robotiq_gazebo ur_robotiq_gazebo_spawner.launch" + \
# 				namespace_arg + robot_name_arg + forklift_model_arg + simplified_arg + \
# 				' x_pos:=' + str(x) + \
# 				' y_pos:=' + str(y) + \
# 				' yaw:=' + str(yaw)

# 	p = Popen(args_str.split(), stdout=PIPE, shell=False)
# 	rospy.loginfo('executed cmd: ' + args_str)
# 	return p
