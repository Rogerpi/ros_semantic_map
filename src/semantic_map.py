#!/usr/bin/env python

#Author: Roger Pi Roig

import rospy
import numpy as np
import math
import copy
import sys
#Topics
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped, Pose, Point
from tf2_msgs.msg import TFMessage
from tf import TransformListener

from robot_map.msg import Detection
from robot_map.msg import Object, ObjectArray
from robot_map.msg import DoorStatus, DoorStatusArray

#Actions
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# Services
from std_srvs.srv import Empty, EmptyResponse
from robot_map.srv import SemanticGoal, SemanticGoalResponse
from robot_map.srv import SemanticChange, SemanticChangeResponse

# Others
from tf import TransformListener


from Landmark import Landmark
from Region import Region
from Waypoint import Waypoint
from Furniture import Furniture
from Connection import Connection
from Path import Path

from std_srvs.srv import Empty, EmptyResponse

from FreePointFinder import FreePointFinder


#from Path import Path
#from Furniture import Furniture

# -----------------------------------------------------------------------------------#
# Semantic Map
# -----------------------------------------------------------------------------------#
class Semantic_Map:
     def __init__(self,frame_id = "map"):

         self.frame_id = frame_id #TODO initialize objects with the frame_id
         self.current_map = [] #Landmark

         self.current_furniture = [] # Furniture


         self.regions = [] # Regions
         self.waypoints = [] # Waypoints

         self.connections = [] # Doors

         self.paths = [] # Paths

         self.tf_listener = TransformListener()


         #Publishers
         self.regions_pub = rospy.Publisher('/semantic_map/viz/regions', MarkerArray , queue_size=10)
         self.waypoints_pub = rospy.Publisher('/semantic_map/viz/waypoints', MarkerArray , queue_size=10)
         self.landmarks_pub = rospy.Publisher('/semantic_map/viz/landmarks', MarkerArray , queue_size=10)
         self.furniture_pub = rospy.Publisher('/semantic_map/viz/furniture', MarkerArray , queue_size=10)
         self.connections_pub = rospy.Publisher('/semantic_map/viz/connections', MarkerArray , queue_size=10)
         self.paths_pub = rospy.Publisher('/semantic_map/viz/paths',MarkerArray,queue_size = 10)

         self.semantic_pub = rospy.Publisher('/semantic_map/map',ObjectArray,queue_size = 10)
         self.doors_pub = rospy.Publisher('/semantic_map/doors_map',DoorStatusArray,queue_size = 10)

         ##Subscribers
         self.landmark_detection_sub = rospy.Subscriber('/semantic_map/landmark_detection',Detection,self.add_landmark)

         ##Actions
         self.goal_client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
         self.goal_client.wait_for_server()
         print("Move Base Goal Action Server Initialized")

         ##Services
         #Load from YAML
         self.load_from_param_srv = rospy.Service('/semantic_map/load_from_param', Empty, self.load_from_param_srv)

         #Mapping
         self.finish_mapping_srv = rospy.Service('/semantic_map/save_map', Empty, self.save_map)

         #Navigation
         self.goal_to_waypoint_srv = rospy.Service('/semantic_map/goal_to_waypoint', SemanticGoal, self.go_to_waypoint)
         self.goal_to_landmark_srv = rospy.Service('/semantic_map/goal_to_landmark', SemanticGoal, self.go_to_landmark)
         self.goal_to_region_srv   = rospy.Service('/semantic_map/goal_to_region'  , SemanticGoal, self.go_to_region)
         self.follow_path_srv      = rospy.Service('/semantic_map/follow_path'  , SemanticGoal, self.follow_path)

         #Knowledge
         self.check_object_status_srv   = rospy.Service('/semantic_map/check_object_status'  , SemanticChange, self.check_object_status)

         print("Service Server Initialized")



         self.load_from_param()

         self.pub_data_timer = rospy.Timer(rospy.Duration(1), self.publish_data)

         print("Ready!")

#--------- ADD TO MAP --------------------------#

     def add_landmark(self,detection_msg):

         if detection_msg.header.frame_id != self.frame_id:
             print("Transform detection to map frame")
             p = Pose()
             p = copy.deepcopy(detection_msg.pose)
             pose_msg = PoseStamped()
             pose_msg.pose = p
             pose_msg.header.frame_id = detection_msg.header.frame_id

             object_world = self.tf_listener.transformPose(self.frame_id,pose_msg)

             detection = copy.deepcopy(detection_msg)
             detection.pose = object_world.pose
         else:
             detection = detection_msg

         selected = next((x for x in self.current_map if x.id == detection.id), None)
         if selected == None: #NEW LANDMARK
             print("New Landmark "+detection.id+" Added!")
             landmark = Landmark(detection.id,[detection.pose.position.x,detection.pose.position.y,detection.pose.position.z],mapped=False,seen=True)
             self.current_map.append(copy.deepcopy(landmark))
         else:
             print("Landmark "+detection.id+" seen again")
             print("Seting new position (Current Approach)")
             selected.set_position([detection.pose.position.x,detection.pose.position.y,detection.pose.position.z])


     def save_map(self,call):
         print("Save Map...")
         for i in range(len(self.current_map)):
             self.current_map[i].set_mapped()
         print("Done!")
         return EmptyResponse()

     def delete_map(self):
         del self.current_map[:] #Landmark
         del self.current_furniture[:] # Furniture
         del self.regions[:] # Regions
         del self.waypoints[:] # Waypoints
         del self.connections[:] # Doors
         del self.paths[:] # Paths


     def export_map(self,call):
         pass #TODO



# --------- DETECT CHANGES ---------------------#

     def check_object_status(self,call):

         landmark =  next((x for x in self.current_map if x.id == call.object_id), None)
         #MOVED: 0 No, 1 Yes, 2 Unknown
         if  landmark == None: # The landmark doesn't exist
             response = SemanticChangeResponse()
             response.mapped = False
             response.detected = False
             return response
         else:
             return landmark.get_status_response()



#--------- Goals to ----------------------------#
     def go_to_waypoint(self,call):
         selected = next((x for x in self.waypoints if x.id == call.goal_name), None)
         response = SemanticGoalResponse()
         if selected == None:
             response.success = False
         else:
             clear_costmap = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
             clear_costmap()
             selected.send_goal(self.goal_client)
             response.success = True
         return response

     def go_to_landmark(self,call):
         selected = next((x for x in self.current_map if x.id == call.goal_name), None)
         response = SemanticGoalResponse()
         if selected == None:
             response.success = False
         else:
             clear_costmap = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
             clear_costmap()
             print("object pose: "+str(selected.mapped_pose))
             '''
             fpf = FreePointFinder('/home/roger/tiago_ws/src/robot_map/src/map.yaml')
             #Transform to map
             p = Pose()
             p.position.x = selected.mapped_pose[0]
             p.position.y = selected.mapped_pose[1]
             pose_msg = PoseStamped()
             pose_msg.pose = p
             pose_msg.header.frame_id = selected.frame_id

             object_world = self.tf_listener.transformPose("map",pose_msg)

             x = object_world.pose.position.x
             y = object_world.pose.position.y

             print('Point ', 'is' if fpf.is_free(x,t) else 'is not', 'free')
             print('Closest free point is', fpf.closest_free_point(x,y))
             '''
             selected.send_goal(self.goal_client)
             response.success = True
         return response

     def go_to_region(self,call):
         selected = next((x for x in self.regions if x.id == call.goal_name), None)
         response = SemanticGoalResponse()
         if selected == None:
             response.success = False
         else:
             clear_costmap = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
             clear_costmap()
             selected.send_goal(self.goal_client)
             response.success = True
         return response

     def follow_path(self,call):
         selected = next((x for x in self.paths if x.id == call.goal_name), None)
         print(selected.id)
         response = SemanticGoalResponse()
         if selected == None:
             response.success = False
         else:
             clear_costmap = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
             clear_costmap()
             selected.execute(self.goal_client)
             response.success = True
         return response
#^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^#

#--------- Load from param server --------------#
     def load_from_param(self):
         print("Deleting previous data...")
         self.delete_map()
         print("Loading from param server")
         self.get_waypoints("/waypoints")
         self.get_regions("/rooms")
         self.get_landmarks("/objects")
         self.get_furniture("/furniture")
         self.get_connections("/connections")
         self.get_paths("/paths")


     def load_from_param_srv(self,call):
         self.load_from_param()
         return EmptyResponse()

     def get_waypoints(self,name):
         waypoints = rospy.get_param(name)
         for i in range(len(waypoints)):
             position = waypoints[i].get("pose")
             waypoint = Waypoint( waypoints[i].get("name"),position[0],position[1],0.0)
             self.waypoints.append(copy.deepcopy(waypoint))


     def get_regions(self,name):
         regions = rospy.get_param(name)
         for i in range(len(regions)):

             position1 = regions[i].get("pose1")
             position2 = regions[i].get("pose2")
             position3 = regions[i].get("pose3")
             position4 = regions[i].get("pose4")
             goal = regions[i].get("goal")

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

             region = Region( regions[i].get("name"),positions,goal)

             self.regions.append(copy.deepcopy(region))



     def get_landmarks(self,name):
         landmarks = rospy.get_param(name)
         for i in range(len(landmarks)):
             id = landmarks[i].get("name")
             mapped = landmarks[i].get("mapped")
             seen = landmarks[i].get("seen")
             pose = landmarks[i].get("pose")
             room = landmarks[i].get("room")
             furniture = landmarks[i].get("furniture")
             exp_furniture = landmarks[i].get("expected_furniture")
             exp_rooms = landmarks[i].get("expected_rooms")

             landmark = Landmark(id,pose,room,mapped,seen,furniture)
             landmark.set_expected_rooms(exp_rooms)
             landmark.set_expected_furniture(exp_furniture)

             self.current_map.append(copy.deepcopy(landmark))


     def get_furniture(self,name):
         furnitures = rospy.get_param(name)
         for i in range(len(furnitures)):
             id = furnitures[i].get("name")
             mapped = furnitures[i].get("mapped")
             seen = furnitures[i].get("seen")
             static = furnitures[i].get("static")
             pose = furnitures[i].get("pose")
             orientation = furnitures[i].get("orientation")
             room = furnitures[i].get("room")
             size = furnitures[i].get("size")

             object = Furniture(id,pose,orientation,size,room,mapped,seen,static)


             self.current_furniture.append(copy.deepcopy(object))


     def get_connections(self,name):
         connections = rospy.get_param(name)
         for i in range(len(connections)):
             waypoint1 = connections[i].get("waypoint1")
             waypoint2 = connections[i].get("waypoint2")
             #door = connections[i].get("door")
             open = connections[i].get("open",2)
             rooms = connections[i].get("rooms")
             #position = connections[i].get("position")
             con = Connection( connections[i].get("name"),rooms,waypoint1,waypoint2,open)
             self.connections.append(copy.deepcopy(con))
         print("connections_set")

     def get_paths(self,name):
         paths = rospy.get_param(name)
         for path_item in paths:
             path = None
             path = Path(path_item.get("name"),[])
             wp_list = path_item.get("poses")
             for wp in wp_list:

                is_door = wp.get("type") == 0
                name = wp.get("name")
                if is_door:

                    move_forward = wp.get("forward")
                    door = next((d for d in self.connections if d.id == name ), None)
                    waypoint1 = door.get_first_waypoint(move_forward)
                    waypoint2 = door.get_last_waypoint(move_forward)
                    path.append(waypoint1)
                    path.append(waypoint2)

                else:
                    wayp = next((w for w in self.waypoints if w.id == name ), None)
                    path.append(wayp)

             self.paths.append(copy.deepcopy(path))

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


     def publish_paths(self):
         ma = MarkerArray()
         for i in range(len(self.paths)):

             bbox, text = self.paths[i].get_marker()
             ma.markers.append(copy.deepcopy(bbox))
             ma.markers.append(copy.deepcopy(text))
         self.paths_pub.publish(ma)

     def publish_landmarks(self):
         #current map
         ma = MarkerArray()
         for i in range(len(self.current_map)):
             bbox, text = self.current_map[i].get_map_marker()
             if bbox != None:
                 ma.markers.append(copy.deepcopy(bbox))
                 ma.markers.append(copy.deepcopy(text))
             bbox, text = self.current_map[i].get_seen_marker()
             if bbox != None:
                 ma.markers.append(copy.deepcopy(bbox))
                 ma.markers.append(copy.deepcopy(text))     
         self.landmarks_pub.publish(ma)

     def publish_furniture(self):
         #current map
         ma = MarkerArray()
         for i in range(len(self.current_furniture)):
             bbox, text = self.current_furniture[i].get_map_marker()
             if bbox != None:
                 ma.markers.append(copy.deepcopy(bbox))
                 ma.markers.append(copy.deepcopy(text))
             bbox, text = self.current_furniture[i].get_seen_marker()
             if bbox != None:
                 ma.markers.append(copy.deepcopy(bbox))
                 ma.markers.append(copy.deepcopy(text))
         self.furniture_pub.publish(ma)

     def publish_connections(self):
         ma = MarkerArray()
         for i in range(len(self.connections)):
             bbox, text = self.connections[i].get_marker()
             if bbox != None:
                 ma.markers.append(copy.deepcopy(bbox))
                 ma.markers.append(copy.deepcopy(text))
         self.connections_pub.publish(ma)


     def publish_map(self):
         oa = ObjectArray()
         da = DoorStatusArray()
         for i in range(len(self.current_map)):
             obj = self.current_map[i].get_object_msg()
             oa.objects.append(copy.deepcopy(obj))
         for i in range(len(self.current_furniture)):
             obj = self.current_furniture[i].get_object_msg()
             oa.objects.append(copy.deepcopy(obj))
         self.semantic_pub.publish(oa)
         for i in range(len(self.connections)):
             door = self.connections[i].get_door_msg()
             da.doors.append(copy.deepcopy(door))
         self.doors_pub.publish(da)



     def publish_data(self,event):

         if not self.pub_data_timer.is_alive():

             self.pub_data_timer.shutdown()
         else:

             self.publish_regions()
             self.publish_waypoints()
             self.publish_landmarks()
             self.publish_furniture()
             self.publish_connections()
             self.publish_paths()

             self.publish_map() #TODO: Finish

#^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^#


# Find element by attribute in array, None if no element
#next((x for x in test_list if x.value == value), None)
#-------------------------------- MAIN ---------------------------------------------#
if __name__ == '__main__':

    # ROS initializzation
    rospy.init_node('semantic_map', anonymous=False)


    node = Semantic_Map(sys.argv[1])
    rospy.spin()

    #r = rospy.Rate(10)
    #while not rospy.is_shutdown():
    #    r.sleep()
    #    node.publish()

    #Using timer instead of rate for more accurate timing response (inside Semantic Map)
