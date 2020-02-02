#!/usr/bin/env python

import rospy
from ur_robotiq_gazebo.srv import LoadParam, LoadParamResponse
from utils import launch_spawn_params

def loadParamCb(req):
	rospy.loginfo('load common param request received.')
	launch_spawn_params(req.robot_name)
	rospy.logfatal(req.robot_name)
	return LoadParamResponse(True)

if __name__ == "__main__":
	rospy.init_node('load_common_param_server')
	s = rospy.Service('load_common_param', LoadParam, loadParamCb)
	rospy.loginfo('Ready to load param server.')
	rospy.spin()
	