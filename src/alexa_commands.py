#!/usr/bin/env python

#Author: Roger Pi Roig

import rospy
import numpy as np
import math
import copy

#Topics
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped, Pose, Point
from tf2_msgs.msg import TFMessage
from tf import TransformListener

from std_msgs.msg import String

from robot_map.msg import Detection
from robot_map.msg import Object, ObjectArray

#Actions
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# Services
from std_srvs.srv import Empty, EmptyResponse
from robot_map.srv import SemanticGoal, SemanticGoalResponse
from robot_map.srv import SemanticChange, SemanticChangeResponse


#from Path import Path
#from Furniture import Furniture

# -----------------------------------------------------------------------------------#
# Semantic Map
# -----------------------------------------------------------------------------------#
class alexa_command_transf:
    def __init__(self):


        rospy.wait_for_service('/semantic_map/goal_to_region')
        self.goto_region_subscriber = rospy.Subscriber("/tiago/room_target", String, self.go_to_callback)
        print("init")

    def go_to_callback(self,msg):
        print("go to "+str(msg.data))
        try:
            send_region_goal = rospy.ServiceProxy('/semantic_map/goal_to_region', SemanticGoal)

            send_region_goal(msg.data.replace(" ","_"))
            return #send_region_goal.success
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

# Find element by attribute in array, None if no element
#next((x for x in test_list if x.value == value), None)
#-------------------------------- MAIN ---------------------------------------------#
if __name__ == '__main__':

    # ROS initializzation
    rospy.init_node('alexa_to_map', anonymous=False)

    node = alexa_command_transf()
    rospy.spin()

    #r = rospy.Rate(10)
    #while not rospy.is_shutdown():
    #    r.sleep()
    #    node.publish()

    #Using timer instead of rate for more accurate timing response (inside Semantic Map)
