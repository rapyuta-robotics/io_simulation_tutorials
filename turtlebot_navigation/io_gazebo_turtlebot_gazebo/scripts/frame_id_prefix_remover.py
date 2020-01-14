#!/usr/bin/env python
import sys
import rospy
from roslib.message import get_message_class
from gazebo_msgs.srv import GetWorldProperties

def separate_prefix(namespace):
    namespace_list = namespace.split('/')
    if len(namespace_list)>1:
        prefix = namespace_list[0]
        remaining = namespace[len(prefix):]
    else:
        prefix = ''
        remaining = namespace

    return prefix, remaining

class FrameidPrefixremover(object):
    def __init__(self, in_topic, out_topic):
        self._model_list = []
        self._sub = rospy.Subscriber(in_topic, rospy.AnyMsg, self.cb)
        rospy.wait_for_service('/gazebo/get_world_properties')
        self._out_topic = out_topic

    def cb(self, msg):
        if 'header' in msg._connection_header['message_definition']:
            msg_name = msg._connection_header['type']
            msg_class = get_message_class(msg_name)
            msg = msg_class().deserialize(msg._buff)

            #remove prefix if prefix is model_name
            prefix, remaining = separate_prefix(msg.header.frame_id)
            if prefix in self._model_list:
                msg.header.frame_id = remaining
                pub = rospy.Publisher(self._out_topic, msg_class, queue_size=1)
                pub.publish(msg)
 
    def get_model_list(self):
        client = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
        res = client.call()
        self._model_list = res.model_names

    def spin(self):
        rate = rospy.Rate(1, reset=True)
        while not rospy.is_shutdown():
            self.get_model_list()
            rate.sleep()


if __name__ == '__main__':
    # Initialize ROS node
    if(len(sys.argv)<1):
        rospy.logerr('you need to provide input and output topic name to remove prefix from header.frame_id')
        exit()

    rospy.init_node("frame_id_prefix_remover")
    node = FrameidPrefixremover(sys.argv[1], sys.argv[2])
    node.spin()