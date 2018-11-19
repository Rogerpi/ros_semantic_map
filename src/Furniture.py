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

# Transformations
from tf.transformations import euler_from_quaternion, quaternion_from_euler



# -----------------------------------------------------------------------------------#
# Furniture
# -----------------------------------------------------------------------------------#
class Furniture:
    def __init__(self,id,pose,yaw,size,room = "",mapped = False, seen=False,static=False, frame_id = "map_align"):
        self.id = id
        self.seen = seen
        self.mapped = mapped
        self.static = static
        self.mapped_orientation = []
        self.mapped_pose = []
        self.mapped_room = ''
        self.pose = []
        self.orientation = []
        self.room = ''
        #If detected
        if self.seen:
            self.pose = [0.0,0.0,0.0]
            self.pose[0:2] = pose[0:2] #Z Assumtion, all furniture stands on the floor, z is ommited

            self.orientation = quaternion_from_euler(0.0, 0.0, yaw)
            self.room = room # string or object?
        if self.mapped:
            self.mapped_pose = [0.0,0.0,0.0]
            self.mapped_pose[0:2] = pose[0:2] #Z Assumtion, all furniture stands on the floor, z is ommited

            self.mapped_orientation = quaternion_from_euler(0.0, 0.0, yaw)
            self.mapped_room = room # string or object?




        #GUESS

        self.expected_rooms = [] #Names or objects?
        #self.expected_room_names = []

        #Object information

        self.size = size

        self.frame_id = frame_id #TODO configurable?

    #-------- ADD information to the landmark ----------#
    def set_room(self,room):
       self.room = room

    def set_position(self,x,y):
        self.pose = [x,y,0.0]

    def set_seen(self,seen):
        self.seen = seen

    def set_expected_rooms(self,rooms):
        self.expected_rooms = rooms

    def set_size(self,size):
        self.size = size



    def set_mapped(self):
        if self.seen:
            self.mapped = True
            self.seen = False

            self.mapped_pose = self.pose
            self.mapped_room = self.room
            self.mapped_orientatin = self.orientation

            #Reset current
            self.pose = []
            #self.orientation = [] For now, lets keep it...
            self.room = ''
            self.furniture = ''

    #---------------- GET Information ----------------#

    def get_position(self):
        return self.pose

    def get_size(self):
        return self.size

    def get_room(self):
        return self.room #If string TODO

    def get_expected_rooms(self): # return string list
        return self.expected_rooms

    def has_changed(self):
        if self.mapped and self.seen:
            if self.room != self.mapped_room:
                return 1
            else:
                return 0
        else:
            return 2


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


    def get_status_response(self):
        response = SemanticChangeResponse()

        response.mapped = self.mapped
        response.detected = self.seen

        if self.mapped and not self.seen: #
            response.mapped_position = self.mapped_pose
            response.mapped_room = self.mapped_room
            #response.mapped_furniture = self.mapped_furniture
            response.moved = 2
        elif not self.mapped and self.seen: #
            response.current_position = self.pose
            response.current_room = self.room
            #response.current_furniture = self.furniture
            response.moved = 2
        elif self.mapped and self.seen: #Check if moved
            response.mapped_position = self.mapped_pose
            response.mapped_room = self.mapped_room
            #response.mapped_furniture = self.mapped_furniture

            response.current_position = self.pose
            response.current_room = self.room
            #response.current_furniture = self.furniture

            if  response.current_room != response.mapped_room:
                response.moved = 1
            else:
                response.moved = 0
        else: #Just for saving expected values, never mapped, never seen
            response.moved = 2

        return response

    def get_seen_marker(self,r=1.0,g=0.0,b=0.0,a=1.0):
        if not self.seen:
            return None, None
        #Sphere as position
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.scale.x = self.size[0]
        marker.scale.y = self.size[1]
        marker.scale.z = self.size[2]

        marker.color.a = a
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b


        marker.pose.position.x = self.pose[0]
        marker.pose.position.y = self.pose[1]
        marker.pose.position.z = self.self.size[2]/2

        marker.pose.orientation.x = self.orientation[0]
        marker.pose.orientation.y = self.orientation[1]
        marker.pose.orientation.z = self.orientation[2]
        marker.pose.orientation.w = self.orientation[3]

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
        if not self.mapped:
            return None,None
        #Sphere as position
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.scale.x = self.size[0]
        marker.scale.y = self.size[1]
        marker.scale.z = self.size[2]

        marker.color.a = a
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b


        marker.pose.position.x = self.mapped_pose[0]
        marker.pose.position.y = self.mapped_pose[1]
        marker.pose.position.z = self.size[2]/2

        marker.pose.orientation.x = self.mapped_orientation[0]
        marker.pose.orientation.y = self.mapped_orientation[1]
        marker.pose.orientation.z = self.mapped_orientation[2]
        marker.pose.orientation.w = self.mapped_orientation[3]

        marker.ns = self.id
        marker.id = 2

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

        text.id = 3

        #So the markers gets deleted from RVIZ if they are not published
        marker.lifetime.secs = 2
        text.lifetime.secs = 2

        return marker, text



    def get_object_msg(self):
        obj = Object()
        obj.header.frame_id = self.frame_id
        obj.header.stamp = rospy.Time.now()
        obj.type = "Furniture"
        obj.name = self.id
        obj.seen = self.seen
        obj.mapped = self.mapped
        obj.static = self.static
        if self.seen:
            obj.current_position.position.x = self.pose[0]
            obj.current_position.position.y = self.pose[1]
            obj.current_room = self.room
        if self.mapped:
            obj.mapped_position.position.x = self.mapped_pose[0]
            obj.mapped_position.position.y = self.mapped_pose[1]
            obj.mapped_room = self.mapped_room
        obj.changed = self.has_changed()

        return obj
