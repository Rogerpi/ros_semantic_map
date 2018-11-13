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
from Region import Region
from Waypoint import Waypoint

#from Path import Path
#from Furniture import Furniture

# -----------------------------------------------------------------------------------#
# Semantic Map
# -----------------------------------------------------------------------------------#
class Semantic_Map:
     def __init__(self):

         self.current_map = [] #Landmark
         self.last_map = [] # Landmark

         self.regions = [] # Regions
         self.waypoints = [] # Waypoints

         #Publishers
         self.regions_pub = rospy.Publisher('/semantic_map/regions', MarkerArray , queue_size=10)
         self.waypoints_pub = rospy.Publisher('/semantic_map/waypoints', MarkerArray , queue_size=10)
         self.landmarks_pub = rospy.Publisher('/semantic_map/landmarks', MarkerArray , queue_size=10)
         self.last_landmarks_pub = rospy.Publisher('/semantic_map/previous_landmarks', MarkerArray , queue_size=10)

         #Subscribers
         self.landmark_detection_sub = rospy.Subscriber('/semantic_map/landmark_detection',Detection,self.add_landmark)

         #Actions
         self.goal_client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
         self.goal_client.wait_for_server()
         print("Move Base Goal Action Server Initialized")

         #Services
         self.load_from_param_srv = rospy.Service('/semantic_map/load_from_param', Empty, self.load_from_param)
         self.finish_mapping_srv = rospy.Service('/semantic_map/save_map', Empty, self.save_map)
         self.goal_to_waypoint_srv = rospy.Service('/semantic_map/goal_to_waypoint', SemanticGoal, self.go_to_waypoint)
         self.goal_to_landmark_srv = rospy.Service('/semantic_map/goal_to_landmark', SemanticGoal, self.go_to_landmark)
         self.goal_to_region_srv   = rospy.Service('/semantic_map/goal_to_region'  , SemanticGoal, self.go_to_region)
         print("Service Server Initialized")

         self.vis_timer = rospy.Timer(rospy.Duration(1), self.publish_markers)

         print("Ready!")

#--------- ADD TO MAP --------------------------#

     def add_landmark(self,detection):
         selected = next((x for x in self.current_map if x.id == detection.id), None)
         if selected == None: #NEW LANDMARK
             print("New Landmark "+detection.id+" Added!")
             landmark = Landmark(detection.id,detection.pose.position.x,detection.pose.position.y,detection.pose.position.z)
             self.current_map.append(copy.deepcopy(landmark))
         else:
             print("Landmark "+detection.id+" seen again")
             print("Seting new position (Current Approach)")
             selected.set_position(detection.pose.position.x,detection.pose.position.y,detection.pose.position.z)


     def save_map(self,call):
         print("Save Map...")
         print("Overwritting last map...")
         del self.last_map[:]
         self.last_map = copy.deepcopy(self.current_map)
         print("Deleting current map...")
         del self.current_map[:]
         print("Done!")

         return EmptyResponse()


#--------- Goals to ----------------------------#
     def go_to_waypoint(self,call):
         selected = next((x for x in self.waypoints if x.id == call.goal_name), None)
         response = SemanticGoalResponse()
         if selected == None:
             response.success = False
         else:
             selected.send_goal(self.goal_client)
             response.success = True
         return response

     def go_to_landmark(self,call):
         selected = next((x for x in self.current_map if x.id == call.goal_name), None)
         response = SemanticGoalResponse()
         if selected == None:
             response.success = False
         else:
             selected.send_goal(self.goal_client)
             response.success = True
         return response

     def go_to_region(self,call):
         selected = next((x for x in self.regions if x.id == call.goal_name), None)
         response = SemanticGoalResponse()
         if selected == None:
             response.success = False
         else:
             selected.send_goal(self.goal_client)
             response.success = True
         return response
#^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^#

#--------- Load from param server --------------#
     def load_from_param(self,call):
         print("Loading from param server")
         self.get_waypoints("/waypoints")
         self.get_regions("/rooms")
         return EmptyResponse()


     def get_waypoints(self,name):
         waypoints = rospy.get_param(name)
         for i in range(len(waypoints)):
             position = waypoints[i].get("pose")
             waypoint = Waypoint( waypoints[i].get("name"),position[0],position[1])
             self.waypoints.append(copy.deepcopy(waypoint))


     def get_regions(self,name):
         regions = rospy.get_param(name)
         for i in range(len(regions)):

             position1 = regions[i].get("pose1")
             position2 = regions[i].get("pose2")
             position3 = regions[i].get("pose3")
             position4 = regions[i].get("pose4")

             positions = []

             p1 = Point()
             p1.x = position1[0]
             p1.y = position1[1]
             p1.z = 0.0


             p2 = Point()
             p2.x = position2[0]
             p2.y = position2[1]
             p2.z = 0.0

             p3 = Point()
             p3.x = position3[0]
             p3.y = position3[1]
             p3.z = 0.0

             p4 = Point()
             p4.x = position4[0]
             p4.y = position4[1]
             p4.z = 0.0

             positions.append(p1)
             positions.append(p2)
             positions.append(p3)
             positions.append(p4)

             region = Region( regions[i].get("name"),positions)

             self.regions.append(copy.deepcopy(region))


#^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^#

#--------- Publish Markers for Visualization ---#
     def publish_regions(self):
         ma = MarkerArray()
         for i in range(len(self.regions)):
             bbox, text = self.regions[i].get_marker()
             ma.markers.append(copy.deepcopy(bbox))
             ma.markers.append(copy.deepcopy(text))
         self.regions_pub.publish(ma)


     def publish_waypoints(self):
         ma = MarkerArray()
         for i in range(len(self.waypoints)):
             bbox, text = self.waypoints[i].get_marker()
             ma.markers.append(copy.deepcopy(bbox))
             ma.markers.append(copy.deepcopy(text))
         self.waypoints_pub.publish(ma)

     def publish_landmarks(self):
         #current map
         ma = MarkerArray()
         for i in range(len(self.current_map)):
             bbox, text = self.current_map[i].get_marker()
             ma.markers.append(copy.deepcopy(bbox))
             ma.markers.append(copy.deepcopy(text))
         self.landmarks_pub.publish(ma)

         #previous map
         ma2 = MarkerArray()
         for i in range(len(self.last_map)):
             bbox, text = self.last_map[i].get_marker()
             ma2.markers.append(copy.deepcopy(bbox))
             ma2.markers.append(copy.deepcopy(text))

             #visualization purposes
             ma2.markers[-2].color.r = 0.0
             ma2.markers[-2].color.b = 1.0
             ma2.markers[-2].ns = ".last_"+ma2.markers[-2].ns
             ma2.markers[-1].color.r = 0.0
             ma2.markers[-1].color.b = 1.0
             ma2.markers[-1].ns = ".last_"+ma2.markers[-1].ns

         self.last_landmarks_pub.publish(ma2)

     def publish_markers(self,event):
         if not self.vis_timer.is_alive():
             self.vis_timer.shutdown()
         else:
             self.publish_regions()
             self.publish_waypoints()
             self.publish_landmarks()

#^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^#


# Find element by attribute in array, None if no element
#next((x for x in test_list if x.value == value), None)
#-------------------------------- MAIN ---------------------------------------------#
if __name__ == '__main__':

    # ROS initializzation
    rospy.init_node('semantic_map', anonymous=False)

    node = Semantic_Map()
    rospy.spin()

    #r = rospy.Rate(10)
    #while not rospy.is_shutdown():
    #    r.sleep()
    #    node.publish()

    #Using timer instead of rate for more accurate timing response (inside Semantic Map)
