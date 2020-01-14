#!/usr/bin/env python
import rospy
from tf2_msgs.msg import TFMessage
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

class TfPrefixSeparator(object):
    def __init__(self):
        # self._freq = rospy.get_param('~/freq', 100)
        self._model_list = []
        self._sub = rospy.Subscriber('/tf', TFMessage, self.cb)
        self._pubs = dict()
        rospy.wait_for_service('/gazebo/get_world_properties')

    def cb(self, msg):
        tfs = dict()
        for t in msg.transforms:
            prefix_parent, remaining_parent = separate_prefix(t.header.frame_id)
            prefix_child, remaining_child = separate_prefix(t.child_frame_id)

            # case 0 : both parent and child have same prefix. e.g transform among robot links
            if prefix_parent in self._pubs and prefix_parent == prefix_child:
                t.header.frame_id = remaining_parent
                t.child_frame_id = remaining_child
                if prefix_parent in tfs:
                    tfs[prefix_parent].transforms.append(t)
                else:
                    tfs[prefix_parent] = TFMessage()
                    tfs[prefix_parent].transforms.append(t)

            # case 1 : prent don't have prefix. e.g. transform from grounnd to robot.
            elif prefix_child in self._pubs and prefix_parent != prefix_child:
                t.child_frame_id = remaining_child
                if prefix_child in tfs:
                    tfs[prefix_child].transforms.append(t)
                else:
                    tfs[prefix_child] = TFMessage()
                    tfs[prefix_child].transforms.append(t)

            # case 2 : two robots are connected to same object?  
            # todo

        for model in tfs:
            self._pubs[model].publish(tfs[model])

    def get_model_list(self):
        client = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
        res = client.call()
        for model in res.model_names:
            self._pubs[model] =  rospy.Publisher('/'+model+'/tf',TFMessage, queue_size=1)

    def spin(self):
        rate = rospy.Rate(1, reset=True)
        while not rospy.is_shutdown():
            self.get_model_list()
            rate.sleep()


if __name__ == '__main__':
    # Initialize ROS node
    rospy.init_node("gazebo_model_state_separator")
    node = TfPrefixSeparator()
    node.spin()