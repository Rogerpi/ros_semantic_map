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
# Regions (Rooms)
# -----------------------------------------------------------------------------------#
class Region:
    def __init__(self,id,points,inside=None,frame_id = "map_align"):
        self.id = id
        self.points = points
        self.inside = inside # Region



    def send_goal(self,client):

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = sum(p.x for p in self.points)/4
        goal.target_pose.pose.position.y = sum(p.y for p in self.points)/4
        goal.target_pose.pose.orientation.w = 1.0

        print("Goal to Region "+str(self.id))
        client.send_goal(goal)
        wait = client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            print("Goal finished")
            return client.get_result()

        self.frame_id = frame_id

    def get_marker(self):

        #Sphere as position
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD
        marker.scale.x = 0.08

        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        marker.ns = self.id
        marker.points = []

        for i in range (5):
            marker.points.append(copy.deepcopy(self.points[i%4]))

        #Show text above
        text = Marker()
        text.header.frame_id = self.frame_id
        text.type =Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD

        text.scale.z = 0.4

        text.color.a = 1.0
        text.color.g = 0.5

        text.pose.position.x = sum(p.x for p in self.points)/4
        text.pose.position.y = sum(p.y for p in self.points)/4
        text.pose.position.z = 0.2
        text.ns = self.id+"_name"
        text.text = self.id
        text.id = 1

        return marker, text

