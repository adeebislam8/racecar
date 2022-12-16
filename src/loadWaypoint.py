#!/usr/bin/env python

# read points from path.text and publish as markers

import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Header
import numpy as np
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseActionGoal
from actionlib_msgs.msg import GoalID

class loadWaypoint:
    def __init__(self):
        # self.path_pub = rospy.Publisher('/waypoint', MarkerArray, queue_size=10)
        self.marker_pub = rospy.Publisher('waypoint_markers', Marker, queue_size=10)
        self.odom_sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.odomCallback)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)
        self.cancel_msg = GoalID()
        self.cancel_msg.id = "cancel"
        self.path = Path()
        self.path.header = Header()
        self.path.header.frame_id = "map"
        self.path.poses = []
        self.path_file = open("path.txt", "r")
        self.path_array = MarkerArray()
        self.path_array.markers = []
        self.ego_odom = Odometry()
        self.ego_odom.pose.pose.position.x = 0
        self.ego_odom.pose.pose.position.y = 0
        self.ego_odom.pose.pose.position.z = 0
        self.ego_odom.pose.pose.orientation.x = 0
        self.ego_odom.pose.pose.orientation.y = 0
        self.ego_odom.pose.pose.orientation.z = 0
        self.ego_odom.pose.pose.orientation.w = 1
        self.car_start = 0



        # read path from file
        for line in self.path_file:
            line = line.strip()
            line = line.split(" ")
            pose = PoseStamped()
            pose.pose.position.x = float(line[0])
            pose.pose.position.y = float(line[1])
            pose.pose.position.z = 0
            pose.pose.orientation.x = float(line[2])
            pose.pose.orientation.y = float(line[3])
            pose.pose.orientation.z = float(line[4])
            pose.pose.orientation.w = float(line[5])
            print(pose.pose.position)
            self.path.poses.append(pose)

        # post process self.path.posses to make it sparse
        self.path.poses = self.path.poses[::100]
        self.x_list = []
        self.y_list = []
        for i in range(len(self.path.poses)):
            self.x_list.append(self.path.poses[i].pose.position.x)
            self.y_list.append(self.path.poses[i].pose.position.y)

        

        # self.path.poses = self.path.poses[::10]
    # Find nearest point

    def odomCallback(self, msg):
        self.ego_odom.pose.pose.position.x = msg.pose.pose.position.x
        self.ego_odom.pose.pose.position.y = msg.pose.pose.position.y
        self.ego_odom.pose.pose.position.z = msg.pose.pose.position.z
        self.ego_odom.pose.pose.orientation.x = msg.pose.pose.orientation.x
        self.ego_odom.pose.pose.orientation.y = msg.pose.pose.orientation.y
        self.ego_odom.pose.pose.orientation.z = msg.pose.pose.orientation.z
        self.ego_odom.pose.pose.orientation.w = msg.pose.pose.orientation.w
        # print("odomCallback", self.ego_odom.pose.pose.position.x, self.ego_odom.pose.pose.position.y)


    def publish2DNavigationGoal(self, goal):
        # Publish 2D Navigation Goal
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = goal.pose.position.x
        msg.pose.position.y = goal.pose.position.y
        msg.pose.position.z = 0
        msg.pose.orientation.x = goal.pose.orientation.x
        msg.pose.orientation.y = goal.pose.orientation.y
        msg.pose.orientation.z = goal.pose.orientation.z
        msg.pose.orientation.w = goal.pose.orientation.w
        # print("publish2DNavigationGoal", msg.pose.position.x, msg.pose.position.y, msg.pose.orientation.w)
        # print(msg)
        self.goal_pub.publish(msg)




    def find_nearest_point(self,ego_x, ego_y, x_list, y_list):
        """
        TODO 2.
        Find the nearest distance(near_dist) and its index(near_ind) w.r.t. current position (ego_x,y) and given trajectory (x_list,y_list).
            - Use 'calc_dist' function to calculate distance.
            - Use np.argmin for finding the index whose value is minimum.
        """


        near_ind = -1
        near_dist = float('inf')

        for i in range(len(x_list)):
            temp_dist = self.calc_dist(x_list[i], y_list[i], ego_x, ego_y)
            if temp_dist < near_dist:
                near_dist = temp_dist
                near_ind = i
                
        return near_dist, near_ind
    # Calculate distance
    def calc_dist(self,tx, ty, ix, iy):
        return math.sqrt( (tx-ix)**2 + (ty-iy)**2 )

    def publishPath(self):
        # Publish wpt as MarkerArray
        msg_wpts = Marker()
        msg_wpts.header.frame_id= "/map"
        msg_wpts.header.stamp= rospy.Time.now()
        msg_wpts.ns= "spheres"
        msg_wpts.action= Marker.ADD
        msg_wpts.pose.orientation.w= 1.0

        msg_wpts.id = 0
        msg_wpts.type = Marker.SPHERE_LIST

        # POINTS markers use x and y scale for width/height respectively
        msg_wpts.scale.x = 0.1
        msg_wpts.scale.y = 0.1
        msg_wpts.scale.z = 0.1

        # Points are green
        msg_wpts.color.a = 1.0      # Don't forget to set the alpha!
        msg_wpts.color.r = 0/255.
        msg_wpts.color.g = 255/255.
        msg_wpts.color.b = 255/255.

        msg_point_list = []

        for i in range(len(self.path.poses)):
            msg_point = Point()
            msg_point.x = self.path.poses[i].pose.position.x
            msg_point.y = self.path.poses[i].pose.position.y

            msg_point_list.append(msg_point)

        msg_wpts.points = msg_point_list
        self.marker_pub.publish(msg_wpts)

        # rate.sleep()


            # publish path as markers
            # for i in range(len(self.path.poses)):
            #     marker = Marker()
            #     marker.header = self.path.header
            #     marker.ns = "waypoint"
            #     marker.id = i
            #     marker.type = marker.SPHERE
            #     marker.action = marker.ADD
            #     marker.pose = self.path.poses[i].pose
            #     marker.scale.x = 0.5
            #     marker.scale.y = 0.5
            #     marker.scale.z = 0.5
            #     marker.color.a = 1.0
            #     marker.color.r = 1.0
            #     marker.color.g = 0.0
            #     marker.color.b = 0.0
            #     self.path_array.markers.append(marker)

            # print("Printing path...")
            # self.path_pub.publish(self.path_array)

if __name__ == '__main__':
    rospy.init_node('loadWaypoint', anonymous=True)
    loadWaypoint = loadWaypoint()
    while not rospy.is_shutdown(): 
        loadWaypoint.publishPath()
        near_dist, near_idx = loadWaypoint.find_nearest_point(loadWaypoint.ego_odom.pose.pose.position.x, loadWaypoint.ego_odom.pose.pose.position.y, loadWaypoint.x_list, loadWaypoint.y_list)
        print("near_dist, ",near_dist, " near_idx, ",near_idx)
        # print("car_start ", loadWaypoint.car_start)
        pointahead = math.atan2(loadWaypoint.path.poses[near_idx].pose.position.y - loadWaypoint.ego_odom.pose.pose.position.y, loadWaypoint.path.poses[near_idx].pose.position.x - loadWaypoint.ego_odom.pose.pose.position.x)
        print("pointahead ", pointahead)
        # if near_dist > 0.0:
        if near_idx + 2 >= len(loadWaypoint.x_list):
            near_idx = near_idx - len(loadWaypoint.x_list)
            
        elif near_idx < 0:
            near_idx = len(loadWaypoint.x_list) + near_idx - 1

        if not loadWaypoint.car_start:
            loadWaypoint.publish2DNavigationGoal(loadWaypoint.path.poses[near_idx])
            loadWaypoint.car_start = True
        else:
            if near_dist < 0.3:
                loadWaypoint.publish2DNavigationGoal(loadWaypoint.path.poses[near_idx+1])


    rospy.spin()
