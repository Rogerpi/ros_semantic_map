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

from Waypoint import Waypoint
# -----------------------------------------------------------------------------------#
# Connection between rooms
# -----------------------------------------------------------------------------------#
class Connection:
    def __init__(self,id,rooms,waypoint1, waypoint2, open = True, frame_id = "map_align"):
        self.id = id
        self.rooms = rooms
        self.open = open
        self.position = [(waypoint1[0] + waypoint2[0])/2,(waypoint1[1] + waypoint2[1])/2]
        self.waypoint1_pose = waypoint1
        self.waypoint2_pose = waypoint2
        self.frame_id = frame_id
        self.waypoint1 = Waypoint(self.id+"_1",self.waypoint1_pose[0],self.waypoint1_pose[1],self.waypoint1_pose[2],self.rooms[0],self.frame_id)
        self.waypoint2 = Waypoint(self.id+"_2",self.waypoint2_pose[0],self.waypoint2_pose[1],self.waypoint2_pose[2],self.rooms[1],self.frame_id)
        self.waypoint1_b = Waypoint(self.id+"_1",self.waypoint1_pose[0],self.waypoint1_pose[1],self.waypoint1_pose[2]-3.14,self.rooms[0],self.frame_id)
        self.waypoint2_b = Waypoint(self.id+"_2",self.waypoint2_pose[0],self.waypoint2_pose[1],self.waypoint2_pose[2]-3.14,self.rooms[1],self.frame_id)


    def send_goal(self,client):
        #TODO
        print("TODO")

    def get_first_waypoint(self,forward = True):
        if forward:
            return self.waypoint1
        else:
            return self.waypoint2_b

    def get_last_waypoint(self,forward = True):
        if forward:
            return self.waypoint2
        else:
            return self.waypoint1_b

    def set_status(self,open):
        self.open = open

    def get_status(self):
        return self.open




    def get_marker(self):

        #Sphere as position
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.type = marker.CYLINDER
        marker.action = marker.ADD

        #dx = (self.corner1[0]-self.corner2[0])
        #dy = (self.corner1[1]-self.corner2[1])
        #dist = math.sqrt(dx**2+dy**2)

        marker.scale.x = 0.9#dist
        marker.scale.y = 0.9#dist
        marker.scale.z = 1

        marker.pose.position.x = self.position[0]
        marker.pose.position.y = self.position[1]
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
        text.header.frame_id = self.frame_id
        text.type =Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD

        text.scale.z = 0.4

        text.color.a = 1.0

        text.pose.position.x = self.position[0]
        text.pose.position.y = self.position[1]
        text.pose.position.z = 0.2
        text.ns = self.id+"_name"
        text.text = self.id
        text.id = 1

        marker.lifetime.secs = 2
        text.lifetime.secs = 2

        return marker, text




































