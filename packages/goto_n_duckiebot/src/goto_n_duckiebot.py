#!/usr/bin/env python

import cv2
import numpy as np
import os
import rospy

from duckietown import DTROS
from std_msgs.msg import Int32MultiArray
from duckietown_msgs.msg import WheelsCmdStamped
from std_msgs.msg import String


class GoToNDuckiebotNode(DTROS):

    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(GoToNDuckiebotNode, self).__init__(node_name=node_name)
        self.veh_name = rospy.get_namespace().strip("/")
        print ("all gucci")
        rospy.Subscriber("/autobot22/movement_commands", Int32MultiArray, self.callback)
        rospy.spin()
        print("I left init")



    def callback(self,data):
        print(data)
        #rospy.loginfo("I heard %s", data.data)



if __name__ == '__main__':
    # Initialize the node
    goto_duckiebot_node = GoToNDuckiebotNode(node_name='goto_n_duckiebot')
    # Keep it spinning to keep the node alive
    rospy.spin()