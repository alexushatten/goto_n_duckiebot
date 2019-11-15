#!/usr/bin/env python

import cv2
import numpy as np
import os
import rospy
import math

from duckietown import DTROS
from std_msgs.msg import Int32MultiArray
from duckietown_msgs.msg import WheelsCmdStamped
from std_msgs.msg import String, Int16
from duckietown_msgs.msg import FSMState, AprilTagsWithInfos, BoolStamped, TurnIDandType

class GoToNDuckiebotNode(DTROS):

    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(GoToNDuckiebotNode, self).__init__(node_name=node_name)
        self.veh_name = rospy.get_namespace().strip("/")
        print ("all gucci")
        #Init server subscriber
        self.sub_message_from_server = rospy.Subscriber("/autobot22/movement_commands", Int32MultiArray, self.servermsgCB)
        #Init publications
        self.pub_override_cmd = rospy.Publisher("goto_n_duckiebot/start_override", BoolStamped, queue_size=10)
        self.pub_wheels_cmd = rospy.Publisher("~/autobot21/wheels_driver_node/wheels_cmd", WheelsCmdStamped, queue_size=10)
        self.pub_turn_type = rospy.Publisher("~turn_type",Int16, queue_size=1, latch=True)
        self.pub_id_and_type = rospy.Publisher("~turn_id_and_type",TurnIDandType, queue_size=1, latch=True)

        #initialized the turn commands
        self.commands = [0,0,0,0,0,0,0,0,0]

        print ("subscribing")
        self.sub_topic_mode = rospy.Subscriber("~mode", FSMState, self.cbMode, queue_size=1)
        #self.fsm_mode = None #TODO what is this?
        self.sub_topic_tag = rospy.Subscriber("~tag", AprilTagsWithInfos, self.cbTag, queue_size=1)
        print ("what")
        self.override_bot()

    def override_bot(self):
        override_msg = BoolStamped()   	    
        override_msg.data = False
        rospy.sleep(1)
        self.pub_override_cmd.publish(override_msg)

    def servermsgCB(self,command):
        if not self.commands:
            commands = command.data
            self.process_msg(commands)
            self.rate.sleep()


    def cbMode(self, mode_msg):
        #print mode_msg
        self.fsm_mode = mode_msg.state
        if(self.fsm_mode != mode_msg.INTERSECTION_CONTROL):
            self.turn_type = -1
            self.pub_turn_type.publish(self.turn_type)
            #rospy.loginfo("Turn type now: %i" %(self.turn_type))

    def cbTag(self, tag_msgs):
        if self.fsm_mode == "INTERSECTION_CONTROL" or self.fsm_mode == "INTERSECTION_COORDINATION" or self.fsm_mode == "INTERSECTION_PLANNING":
            #loop through list of april tags

            # filter out the nearest apriltag
            dis_min = 999
            idx_min = -1
            for idx, taginfo in enumerate(tag_msgs.infos):
                if(taginfo.tag_type == taginfo.SIGN):
                    tag_det = (tag_msgs.detections)[idx]
                    pos = tag_det.pose.pose.position
                    distance = math.sqrt(pos.x**2 + pos.y**2 + pos.z**2)
                    if distance < dis_min:
                        dis_min = distance
                        idx_min = idx

            if idx_min != -1:
                chosenTurn = self.commands[0]
                self.turn_type = chosenTurn
                self.pub_turn_type.publish(self.turn_type)

                id_and_type_msg = TurnIDandType()
                id_and_type_msg.tag_id = taginfo.id
                id_and_type_msg.turn_type = self.turn_type
                self.pub_id_and_type.publish(id_and_type_msg)

                #rospy.loginfo("possible turns %s." %(availableTurns))
                #rospy.loginfo("Turn type now: %i" %(self.turn_type))




    def onShutdown(self):
        """Shutdown procedure.

        Publishes a zero velocity command at shutdown."""
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