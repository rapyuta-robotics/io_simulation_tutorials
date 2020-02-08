#!/usr/bin/env python
import os
import sys
import rospy

if __name__ == '__main__':
	if(len(sys.argv)<1):
		rospy.logerr('you need to provide robot_name')
		exit()
	
	# Initialize ROS node
	rospy.init_node("add_namespace_to_controller_list")
	rospy.loginfo("add_namespace_to_controller_list node started")
	# add namespace to controller_lsit param
	robot_name = sys.argv[1]
	controller_param = rospy.get_param('/move_group/controller_list')
	for i in range(len(controller_param)):
		controller_param[i]['name'] = os.path.join(robot_name, controller_param[i]['name'])

	rospy.set_param('/move_group/controller_list', controller_param)

	rospy.loginfo("add_namespace_to_controller_list node finished")
	rospy.loginfo(controller_param)
