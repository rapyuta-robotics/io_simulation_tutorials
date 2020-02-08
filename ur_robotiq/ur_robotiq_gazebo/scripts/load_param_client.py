#!/usr/bin/env python
import rospy
from utils import get_robot_name
from ur_robotiq_gazebo.srv import LoadParam, LoadParamRequest

# Initialize ROS node
rospy.init_node("load_client")
rospy.loginfo("Load client node started")

rospy.wait_for_service('load_common_param')
client = rospy.ServiceProxy('load_common_param', LoadParam)

req = LoadParamRequest()
req.robot_name = get_robot_name()
rospy.loginfo(req)

res = client(req)
if res.success:
    rospy.logerr('load param succeeded')
else:
    rospy.logerr('load param failed')