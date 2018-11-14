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
from robot_map.msg import Object

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
    def __init__(self,id,pose,room = "" , mapped=False, seen=False, furniture = "" ):
        self.id = id

        self.mapped = mapped
        self.seen = seen
        #If detected

        self.pose = []
        self.room = ''
        self.furniture = ''
        self.mapped_pose = ''
        self.mapped_room = ''
        self.mapped_furniture = ''

        if self.seen:
            self.pose = pose
            self.room = room # string or object?
            self.furniture = furniture #string or object?

        if self.mapped:
            self.mapped_pose = pose
            self.mapped_room = room
            self.mapped_furniture = furniture

        #GUESS

        self.expected_rooms = [] #Names or objects?
        #self.expected_room_names = []
        self.expected_furniture = []


        #Object information

        self.width = None #TODO #Assume is squared
        self.height = None #TODO

        self.frame_id = "/map" #TODO configurable?

    #-------- ADD information to the landmark ----------#
    def set_room(self,room):
       self.room = room

    def set_position(self,pose):
        self.pose = pose
        self.seen = True

    def set_place(self,furniture):
        self.furniture = furniture

    #def set_seen(self,seen):
    #    self.seen = seen

    def set_expected_rooms(self,rooms):
        self.expected_rooms = rooms

    def set_expected_furniture(self,expected_furniture):
        self.expected_furniture = expected_furniture

    #---------------- GET Information ----------------#
    #   Strings or Numbers, no objects
    def get_position(self):
        return self.pose

    def get_room(self):
        return self.room #If string TODO

    def get_furniture(self):
        return self.furniture

    def get_expected_furniture(self):
        return self.expected_furniture

    def get_expected_rooms(self): # return string list
        return self.expected_rooms

    #^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^#

    def send_goal(self,client): #TODO n attempts?

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = self.pose[0]
        goal.target_pose.pose.position.y = self.pose[1]
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



    def get_seen_marker(self,r=1.0,g=0.0,b=0.0,a=1.0):
        #Sphere as position
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.scale.x = 0.08
        marker.scale.y = 0.08
        marker.scale.z = 0.08

        marker.color.a = a
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b


        marker.pose.position.x = self.pose[0]
        marker.pose.position.y = self.pose[1]
        marker.pose.position.z = self.pose[2]
        marker.ns = self.id
        marker.id = 0

        #Show text above
        text = Marker()
        text.header.frame_id = self.frame_id
        text.type =Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD

        text.scale.z = 0.2

        text.color.a = a
        text.color.r = r
        text.color.g = g
        text.color.b = b

        text.pose.position.x = self.pose[0]
        text.pose.position.y = self.pose[1]
        text.pose.position.z = self.pose[2] + 0.1
        text.ns = self.id+"_name"
        text.text = "DETECTED "+self.id

        text.id = 1

        #So the markers gets deleted from RVIZ if they are not published
        marker.lifetime.secs = 2
        text.lifetime.secs = 2

        return marker, text

    def get_map_marker(self,r=0.0,g=0.0,b=1.0,a=1.0):
        #Sphere as position
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.scale.x = 0.08
        marker.scale.y = 0.08
        marker.scale.z = 0.08

        marker.color.a = a
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b


        marker.pose.position.x = self.mapped_pose[0]
        marker.pose.position.y = self.mapped_pose[1]
        marker.pose.position.z = self.mapped_pose[2]
        marker.ns = self.id
        marker.id = 0

        #Show text above
        text = Marker()
        text.header.frame_id = self.frame_id
        text.type =Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD

        text.scale.z = 0.2

        text.color.a = a
        text.color.r = r
        text.color.g = g
        text.color.b = b

        text.pose.position.x = self.mapped_pose[0]
        text.pose.position.y = self.mapped_pose[1]
        text.pose.position.z = self.mapped_pose[2] + 0.1
        text.ns = self.id+"_name"
        text.text = "MAP "+self.id

        text.id = 1

        #So the markers gets deleted from RVIZ if they are not published
        marker.lifetime.secs = 2
        text.lifetime.secs = 2

        return marker, text
