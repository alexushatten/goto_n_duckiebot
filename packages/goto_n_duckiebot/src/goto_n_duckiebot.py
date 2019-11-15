#!/usr/bin/env python

import cv2
import numpy as np
import os
import rospy

from duckietown import DTROS
from std_msgs.msg import Int32MultiArray
from duckietown_msgs.msg import WheelsCmdStamped
from std_msgs.msg import String
from duckietown_msgs.msg import BoolStamped

class GoToNDuckiebotNode(DTROS):

    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(GoToNDuckiebotNode, self).__init__(node_name=node_name)
        self.veh_name = rospy.get_namespace().strip("/")
        print ("all gucci")
        #Init subscriber
        self.sub_message_from_server = rospy.Subscriber("/autobot22/movement_commands", Int32MultiArray, self.servermsgCB)
        #Init publications
        self.pub_override_cmd = rospy.Publisher("~/autobot21/joy_mapper_node/joystick_override", BoolStamped, queue_size=10)
        self.pub_wheels_cmd = rospy.Publisher("~/autobot21/wheels_driver_node/wheels_cmd", WheelsCmdStamped, queue_size=10)
        self.pub_turn_cmd = rospy.Publisher("~/autobot21/ninja/cmd", Int32MultiArray, queue_size=10)

        #initialized the turn commands
        commands = []

        commands = [0,0,0,0,0,0,0,0,0]

        self.publish_turns(commands)
    
    def override_bot(self, turn_msg):
        rate = rospy.Rate(1) # 1hz
        override_msg = BoolStamped()   	    
        while not rospy.is_shutdown():
            override_msg.data = False
            self.pub_override_cmd.publish(override_msg)
            self.pub_turn_cmd.publish(turn_msg)
            rate.sleep()

    def publish_turns (self, commands):
        pum_msg=Int32MultiArray(data=commands)
        self.override_bot(pum_msg)

    def servermsgCB(self,commands):
        print(commands.data)
        if not self.commands:
            self.commands = commands.data
            #Publish all turns
            self.publish_turns()
        #rospy.loginfo("I heard %s", data.data)

    def onShutdown(self):
        """Shutdown procedure.

        Publishes a zero velocity command at shutdown."""

        # MAKE SURE THAT THE LAST WHEEL COMMAND YOU PUBLISH IS ZERO,
        # OTHERWISE YOUR DUCKIEBOT WILL CONTINUE MOVING AFTER
        # THE NODE IS STOPPED

        # PUT YOUR CODE HERE
        override_msg = BoolStamped()	    
        override_msg.data = True

        self.pub_override_cmd.publish(override_msg)
        #Stop wheelcommand
        wheel_msg = WheelsCmdStamped()	    
        wheel_msg.vel_left = 0
        wheel_msg.vel_right = 0

        self.pub_wheels_cmd.publish(wheel_msg)

        super(GoToNDuckiebotNode, self).onShutdown()



if __name__ == '__main__':
    # Initialize the node
    goto_duckiebot_node = GoToNDuckiebotNode(node_name='goto_n_duckiebot')
    # Keep it spinning to keep the node alive
    rospy.spin()