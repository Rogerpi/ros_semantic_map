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

from Landmark import Landmark



# -----------------------------------------------------------------------------------#
# Static Path
# -----------------------------------------------------------------------------------#
class Path:
    def __init__(self,id,waypoints = [], frame_id = "map_align"):
        self.id = id
        self.waypoints = waypoints
        self.frame_id = frame_id


    def append(self,waypoint):
        self.waypoints.append(waypoint)

    def execute(self,client,max_attempts = 5):
        for waypoint in self.waypoints:
            success = False
            attempts = 0
            while( not success and attempts < max_attempts):
                resp = waypoint.send_goal(client)
                success = True #TODO
                attempts += 1



    def get_marker(self):

        #Sphere as position
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD
        marker.scale.x = 0.04

        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        marker.ns = self.id
        marker.points = []

        for waypoint in self.waypoints:
            marker.points.append(copy.deepcopy(waypoint.get_point()))

        #Show text above
        text = Marker()
        text.header.frame_id = self.frame_id
        text.type =Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD

        text.scale.z = 0.4

        text.color.a = 1.0
        text.color.g = 0.5

        text.pose.position.x = marker.points[0].x
        text.pose.position.y = marker.points[0].y
        text.pose.position.z = 0.2
        text.ns = self.id+"_name"
        text.text = self.id
        text.id = 1

        return marker, text
