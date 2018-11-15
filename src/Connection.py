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
# Connection between rooms
# -----------------------------------------------------------------------------------#
class Connection:
    def __init__(self,id,rooms,corner1,corner2,door = False,open = True):
        self.id = id
        self.rooms = rooms
        self.door = door
        self.open = open
        self.corner1 = corner1
        self.corner2 = corner2

    def send_goal(self,client):
        #TODO
        print("TODO")

    def set_status(self,open):
        self.open = open

    def get_status(self):
        return self.open




    def get_marker(self):
        if not self.door:
            return None,None
        #Sphere as position
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = marker.CYLINDER
        marker.action = marker.ADD

        dx = (self.corner1[0]-self.corner2[0])
        dy = (self.corner1[1]-self.corner2[1])
        dist = math.sqrt(dx**2+dy**2)

        marker.scale.x = dist
        marker.scale.y = dist
        marker.scale.z = 1

        marker.pose.position.x = (self.corner1[0] + self.corner2[0])/2
        marker.pose.position.y = (self.corner1[1] + self.corner2[1])/2
        marker.pose.position.z = 0.5

        marker.color.a = 0.4
        if self.open:
            marker.color.g = 1.0
        else:
            marker.color.r = 1.0
        if self.open == 2:
            marker.color.b = 1.0
        marker.ns = self.id



        #Show text above
        text = Marker()
        text.header.frame_id = "map"
        text.type =Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD

        text.scale.z = 0.4

        text.color.a = 1.0

        text.pose.position.x = (self.corner1[0] + self.corner2[0])/2
        text.pose.position.y = (self.corner1[1] + self.corner2[1])/2
        text.pose.position.z = 0.2
        text.ns = self.id+"_name"
        text.text = self.id
        text.id = 1

        marker.lifetime.secs = 2
        text.lifetime.secs = 2

        return marker, text




































