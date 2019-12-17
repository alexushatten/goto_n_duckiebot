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
    """
    GoTo-N-Duckiebot Node

    This node implements the Goto-N package on the duckiebots.
    The node is to be be run on all duckiebots in the Robotarium that need to be reset. 

    Args:
        GotoNDuckiebotNode (self)

    Subscriber:
        ~movement_commands (:obj: 'Int32MultiArray): Subscribes to the movement commands created by the 
            global planner in order to get to the duckiebot to the desired position
        ~positional_difference (:obj: 'Float32MultiArray): Subscribes to the positional difference commands
            output by a seperate accuracy node running on the server (Difference between current and desired
            position)
        ~mode (:obj: 'duckietown_msgs.msg.FSMState): Subscribes FSM state

    Publisher:
        ~wheels_cmd (:obj:`duckietown_msgs.msg.WheelsCmdStamped`): The
            wheel commands that the motors will execute
        ~joystick_override (:obj:`duckietown_msgs.msg.BoolStamped`): Published the Boolean that 
            will allow us to override the joystick if necessary
        ~turn_type (:obj:`duckietown_msgs.msg.Int16`): The publisher that will allow us to 
            give turn type commands for when the duckiebot arrives at the next intersection 
            (these are the commands outputted by the ~movement_commands subscriber)
        ~turn_id_and_type (:obj:`duckietown_msgs.msg.TurnIDandType`): The publisher that allows us to 
            send the turn type and ID to the indefinite navigation/intersection navigation node. 
        ~arrival_msg (:obj:`duckietown_msgs.msg.BoolStamped`): This Publisher is used to send a 
            Boolean to activate the final precision node the duckiebot has performed the last turn.  
    
    Additional Info:
        - Delta X/Y: initialize the deltas to a very high value (arbitrary)
        - Threshold X/Y: The desired accuracy threshold for the final precision (in cm)

    Credit:
        Some functions in this code is taken and modified from the random_april_tag_turns_node to fit our needs from the dt-core image on github 
    """

    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(GoToNDuckiebotNode, self).__init__(node_name=node_name)
        self.veh_name = rospy.get_namespace().strip("/")
        self.turn_type = -1
        self.start_precision = False
        
        #Init server subscriber
        self.sub_message_from_server = rospy.Subscriber("~movement_commands", Int32MultiArray, self.servermsgCB)
        self.sub_watchtower_delta = rospy.Subscriber("~positional_diff", Float32MultiArray, self.precisionCB)
        self.sub_topic_mode = rospy.Subscriber("~mode", FSMState, self.cbMode, queue_size=1)

        #Init publications
        self.pub_override_cmd = rospy.Publisher("~joystick_override", BoolStamped, queue_size=10)
        self.pub_wheels_cmd = rospy.Publisher("~wheels_cmd", WheelsCmdStamped, queue_size=10)
        self.pub_turn_type = rospy.Publisher("~turn_type",Int16, queue_size=1, latch=True)
        self.pub_id_and_type = rospy.Publisher("~turn_id_and_type",TurnIDandType, queue_size=1, latch=True)
        self.pub_arrival_msgs = rospy.Publisher("~arrival_msg",BoolStamped, queue_size=1, latch=True)

        #Initialize the 
        self.string_commands= ["Left", "Straight", "Right"]

        #Init gain of wheels
        rospy.set_param('/{}/kinematics_node/gain'.format(self.veh_name), 0.9)

        #Init deltas
        self.delta_x = 100
        self.delta_y = 100
        
        #Init thresholds
        self.thresh_x = 0.10
        self.thresh_y = 0.10

        #initialized the turn commands
        self.commands = []
        self.previous_intersection_tag = -1

        rospy.loginfo("Initialized")

    def override_bot(self):
         """Used to Override the Duckiebot commands

         When called, the function will start indefinite navigation.
         It publishes a boolean value that allows us to override indefinite navigation. 

        Args:

        Returns:
            Publishes a Boolean Value
        """

        override_msg = BoolStamped()   	    
        override_msg.data = False
        rospy.sleep(1)
        
        #Publish Bool
        self.pub_override_cmd.publish(override_msg)
    
    def servermsgCB (self, data):
         """This function receives the intersection commands from the server and 
         turns them into a commands list to be used in a later function.

         When the ~movement_commands subscriber receives a waypoint message from the server, 
         the function prints a prompt to let the user know that it has. It then creates the commands 
         list that containes the sequential action commands for the duckiebot at the intersections.

        Args:
            data (:obj:'Int32MultiArray): Message from server with the waypoint commands
                0 means go left at the next intersection
                1 means go straight at the next intersection
                2 means go right at the next intersection
        """

        print ("I have received the message from the Global Planner")

        #create movement commands list
        for i in data.data:
            self.commands.append(i)
        
        print (self.commands)
        self.override_bot()
    
    def precisionCB(self, delta):
         """Starts the robot specific finally accuracy to get to the termination positoin

        When the function is called, it reads in the delta x/y (difference between the current
        and desired x/y positions). If the duckiebot has completed its final intersection, the function 
        will continue to drive the duckiebot in the lane until the desired threshold values are reached. 
        Once it reaches the desired position, it publishes an arrival message.

        Args:
            delta_x (:obj:'float'): The difference between the current and desired x-position. 
            delta_y (:obj:'float'): The difference between the current and desired y-position. 

        Returns:
            Publishes Arrival Message packed in an 'arrival_msg'. 
        """
        self.delta_x = delta.data[0]
        self.delta_y = delta.data[1]
        
        #check if the final intersection has been completed
        if self.start_precision == True:

            #continue driving until duckiebot is in desired threshold
            if (self.delta_x < self.thresh_x) and (self.delta_y < self.thresh_y):
                    self.stop_navigation()
                    arrived_msg = BoolStamped()	    
                    arrived_msg.data = True
                    self.pub_arrival_msgs.publish(arrived_msg)
                    print("I have arrived at the desired location.")

    def cbMode(self, mode_msg):
         """Sets the FSM mode

        This function sets the FSM mode to intersection control when it reaches an intersection

        Args:
            mode_msg(:obj:'FSMState'): The input determines what mode the FSM node is in

        Returns:
            Publishes the turn_type to the ~turn_type publisher. 
        """

        self.fsm_mode = mode_msg.state
        if(self.fsm_mode != mode_msg.INTERSECTION_CONTROL):
            self.turn_type = -1
            self.pub_turn_type.publish(self.turn_type)

    def cbTag(self, tag_msgs):
         """Reads the April tag and understand that it is located at an intersection.

        This function was taken from the dt-core package and slightly adapted to fit the Goto-N needs
        
        Args:
            Takes the commands from the server at each intersetion.

        Returns:
            Publishes desired turn to other nodes running on the bot.
        """
        
        if self.fsm_mode == "INTERSECTION_CONTROL" or self.fsm_mode == "INTERSECTION_COORDINATION" or self.fsm_mode == "INTERSECTION_PLANNING":
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
                    self.stop_navigation()
                    not_arrived_msg = BoolStamped()	    
                    not_arrived_msg.data = False
                    self.pub_arrival_msgs.publish(not_arrived_msg)
                    print("could not find termination position, will restart planner on server")
                    chosenTurn = -1
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
                        self.start_precision = True
                        rospy.loginfo(self.turn_type)

    def stop_navigation (self):
         """Stops the Duckiebot movements

        When this function is called, the duckiebot should come to a stop. This is achieved by

        Returns:
            Publish message to override current commands
            Publish wheel_msg
        """

        override_msg = BoolStamped()	    
        override_msg.data = True
        self.pub_override_cmd.publish(override_msg)
        
        wheel_msg = WheelsCmdStamped()	    
        wheel_msg.vel_left = 0
        wheel_msg.vel_right = 0
        self.pub_wheels_cmd.publish(wheel_msg)
        
    def onShutdown(self):
        """Shutdown procedure.

        Publishes a zero velocity command at shutdown.
        """

        override_msg = BoolStamped()	    
        override_msg.data = True

        #Publish Override Command
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