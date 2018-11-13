#!/usr/bin/env python

#Author: Roger Pi Roig

import numpy as np
import math
import copy

#Topics
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped, Pose, Point
from tf2_msgs.msg import TFMessage

from robot_map.msg import Detection

#Actions
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# Services
from std_srvs.srv import Empty, EmptyResponse
from robot_map.srv import SemanticGoal, SemanticGoalResponse

# Others
from tf import TransformListener




# -----------------------------------------------------------------------------------#
# Landmark represents an object
# -----------------------------------------------------------------------------------#
class Landmark:
    def __init__(self,id,x,y,z,room = "Undefined",on_top = None):
        self.id = id
        self.x = x
        self.y = y
        self.z = z
        self.room = room # string
        self.on_top = on_top #string

        self.width = None #TODO #ASsume is squared
        self.height = None #TODO
    #-------- ADD information to the landmark ----------#
    def set_room(self,room):
       self.room = room

    def set_position(self,x,y,z):
        self.x = x
        self.y = y
        self.z = z

    def set_place(self,on_top):
        self.on_top = on_top

    #^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^#

    def send_goal(self,client):

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = self.x
        goal.target_pose.pose.position.y = self.y
        goal.target_pose.pose.orientation.w = 1.0

        print("Goal to Waypoint "+str(self.id))
        client.send_goal(goal)
        wait = client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            print("Goal finished")
            return client.get_result()

    def get_marker(self):

        #Sphere as position
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.scale.x = 0.08
        marker.scale.y = 0.08
        marker.scale.z = 0.08
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        marker.pose.position.x = self.x
        marker.pose.position.y = self.y
        marker.pose.position.z = self.z
        marker.ns = self.id
        marker.id = 0

        #Show text above
        text = Marker()
        text.header.frame_id = "map"
        text.type =Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD

        text.scale.z = 0.2

        text.color.a = 1.0
        text.color.r = 0.8
        text.pose.position.x = self.x
        text.pose.position.y = self.y
        text.pose.position.z = self.z + 0.1
        text.ns = self.id+"_name"
        text.text = self.id
        text.id = 1

        #So the markers gets deleted from RVIZ if they are not published
        marker.lifetime.secs = 2
        text.lifetime.secs = 2

        return marker, text
