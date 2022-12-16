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
import numpy as np



class saveHectorPath:
    def __init__(self):
        self.path_sub = rospy.Subscriber('/trajectory', Path, self.pathCallback)
        self.path = Path()
        self.path.header = Header()
        self.path.header.frame_id = "map"
        self.path.poses = []
        self.path_file = open("path.txt", "w")

    def pathCallback(self, msg):
        # save path to file
        self.path = msg
        # self.path_pub.publish(self.path)
        for pose in self.path.poses:
            self.path_file.write(str(pose.pose.position.x) + " " + str(pose.pose.position.y) + " " +  str(pose.pose.orientation.x) + " " + str(pose.pose.orientation.y) + " " +str(pose.pose.orientation.z) + " " +str(pose.pose.orientation.w) + "\n"  ) 

if __name__ == '__main__':
    rospy.init_node('saveHectorPath', anonymous=True)
    saveHectorPath = saveHectorPath()
    rospy.spin()

