#!/usr/bin/env python

# subscribe to /trajectory and save it to a file
import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Header

# import navGoal
# import 
import numpy as np
import atexit


class saveHectorPath:
    def __init__(self):
        # self.path_sub = rospy.Subscriber('/trajectory', Path, self.pathCallback)
        # self.path = Path()
        # self.path.header = Header()
        # self.path.header.frame_id = "map"
        # self.path.poses = []
        self.path_file = open("path.txt", "w")

        self.navGoal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.navGoalCallback)
        self.navGoal = PoseStamped()
        self.navGoal.header = Header()
        self.navGoal.header.frame_id = "map"
        self.navGoal.pose.position = Point()
        self.navGoal.pose.orientation = Quaternion()
        self.navGoal.pose.position.x = 0
        self.navGoal.pose.position.y = 0
        self.navGoal.pose.position.z = 0
        self.navGoal.pose.orientation.x = 0
        self.navGoal.pose.orientation.y = 0
        self.navGoal.pose.orientation.z = 0
        self.navGoal.pose.orientation.w = 1

        self.navGoalList = []

    def navGoalCallback(self, msg):
        self.navGoalList.append(msg)
        print("navGoalList: " + str(self.navGoalList))

    def saveNavGoalList(self):
        for pose in self.navGoalList:
            self.path_file.write(str(pose.pose.position.x) + " " + str(pose.pose.position.y) + " " +  str(pose.pose.orientation.x) + " " + str(pose.pose.orientation.y) + " " +str(pose.pose.orientation.z) + " " +str(pose.pose.orientation.w) + "\n"  ) 

    def pathCallback(self, msg):
        # save path to file
        self.path = msg
        # self.path_pub.publish(self.path)
        for pose in self.path.poses:
            self.path_file.write(str(pose.pose.position.x) + " " + str(pose.pose.position.y) + " " +  str(pose.pose.orientation.x) + " " + str(pose.pose.orientation.y) + " " +str(pose.pose.orientation.z) + " " +str(pose.pose.orientation.w) + "\n"  ) 

if __name__ == '__main__':
    rospy.init_node('saveHectorPath', anonymous=True)
    saveHectorPath = saveHectorPath()
    atexit.register(saveHectorPath.saveNavGoalList)
    rospy.spin()

