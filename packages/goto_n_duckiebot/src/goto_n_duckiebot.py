#!/usr/bin/env python

import cv2
import numpy as np
import os
import rospy
import math

from duckietown import DTROS
from std_msgs.msg import Int32MultiArray, Float32MultiArray
from duckietown_msgs.msg import WheelsCmdStamped
from std_msgs.msg import String, Int16
from duckietown_msgs.msg import FSMState, AprilTagsWithInfos, BoolStamped, TurnIDandType

class GoToNDuckiebotNode(DTROS):

    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(GoToNDuckiebotNode, self).__init__(node_name=node_name)
        self.veh_name = rospy.get_namespace().strip("/")
        print ("all gucci")
        self.turn_type = -1
        #Init server subscriber
        self.sub_message_from_server = rospy.Subscriber("~movement_commands", Int32MultiArray, self.servermsgCB)
        self.sub_watchtower_delta = rospy.Subscriber("~positional_diff", Float32MultiArray, self.precisionCB)
        #Init publications
        self.pub_override_cmd = rospy.Publisher("~joystick_override", BoolStamped, queue_size=10)
        self.pub_wheels_cmd = rospy.Publisher("~wheels_cmd", WheelsCmdStamped, queue_size=10)
        self.pub_turn_type = rospy.Publisher("~turn_type",Int16, queue_size=1, latch=True)
        self.pub_id_and_type = rospy.Publisher("~turn_id_and_type",TurnIDandType, queue_size=1, latch=True)

        #Init gain of wheels
        rospy.set_param('/{}/kinematics_node/gain'.format(self.veh_name), 0.9)

        #Init deltas
        self.delta_x = 100
        self.delta_y = 100
        #Init thresholds
        self.thresh_x = 0.10
        self.thresh_y = 0.10
        self.start_precision = False


        #initialized the turn commands
        self.commands = []
        self.previous_intersection_tag = -1

        self.sub_topic_mode = rospy.Subscriber("~mode", FSMState, self.cbMode, queue_size=1)
        #self.fsm_mode = None #TODO what is this?
        self.sub_topic_tag = rospy.Subscriber("~tag", AprilTagsWithInfos, self.cbTag, queue_size=1)
        

    def override_bot(self):
        override_msg = BoolStamped()   	    
        override_msg.data = False
        rospy.sleep(1)
        self.pub_override_cmd.publish(override_msg)
    
    def servermsgCB (self, data):
        print ("got message")
        for i in data.data:
            self.commands.append(i)
        print (self.commands)
        self.override_bot()
    
    def precisionCB(self, delta):
        self.delta_x = delta.data[0]
        self.delta_y = delta.data[1]
        if self.start_precision == True:
            if (self.delta_x < self.thresh_x) and (self.delta_y < self.thresh_y):
                    self.stop_navigation()
                    print("arrived at location")
                    self.keep_driving = False


    def cbMode(self, mode_msg):
        #print mode_msg
        self.fsm_mode = mode_msg.state
            #rospy.loginfo("Turn type now: %i" %(self.turn_type))


    def cbTag(self, tag_msgs):
        if self.fsm_mode == "INTERSECTION_CONTROL" or self.fsm_mode == "INTERSECTION_COORDINATION" or self.fsm_mode == "INTERSECTION_PLANNING":
            #loop through list of april tags
            if self.commands:
                while self.commands[0] == 3 or self.commands[0] == 4:
                    self.commands.pop(0)
                    if not self.commands:
                        break
    
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
                taginfo = (tag_msgs.infos)[idx_min]
                if self.commands:
                    chosenTurn = self.commands[0]
                else:
                    chosenTurn = 1
                self.turn_type = chosenTurn
                id_and_type_msg = TurnIDandType()
                id_and_type_msg.tag_id = taginfo.id

                id_and_type_msg.turn_type = self.turn_type
                if self.previous_intersection_tag != taginfo.id:
                    self.pub_turn_type.publish(self.turn_type)
                    self.pub_id_and_type.publish(id_and_type_msg)
                    if self.commands:
                        self.commands.pop(0)
                    self.previous_intersection_tag = taginfo.id
                    if 0 not in self.commands and 1 not in self.commands and 2 not in self.commands:
                        rospy.loginfo("Turn type now: %i" %(self.turn_type))
                        self.start_precision = True
                    
                    #rospy.loginfo("possible turns %s." %(availableTurns))
                    rospy.loginfo("Turn type now: %i" %(self.turn_type))

    def stop_navigation (self):
        override_msg = BoolStamped()	    
        override_msg.data = True
        self.pub_override_cmd.publish(override_msg)
        
        wheel_msg = WheelsCmdStamped()	    
        wheel_msg.vel_left = 0
        wheel_msg.vel_right = 0
        self.pub_wheels_cmd.publish(wheel_msg)
        

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