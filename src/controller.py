#!/usr/bin/env python
# from email.quoprimime import header_decode
import numpy as np
import os
import pandas as pd
import math


import roslib
import rospy
import rospkg

from std_msgs.msg import Int16
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStampedWithCovariance, PoseStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Path
from std_msgs.msg import Header

import tf
from tf.transformations import euler_from_quaternion


class WaypointFollower():
    def __init__(self):
        # ROS init
        rospy.init_node('wpt_follwer')
        self.rate = rospy.Rate(100.0)

        # Params
        self.target_speed = 10/3.6
        self.MAX_STEER    = np.deg2rad(17.75)

        # vehicle state
        self.ego_x   = 0
        self.ego_y   = 0
        self.ego_yaw = 0
        self.ego_vx  = 0



        


        self.wpt_look_ahead = 0   # [index]
        self.total_crosstrack_error = 0
        self.total_yaw_error = 0
        self.total_velocity_error = 0
        self.lap = 0
        # Pub/Sub
        self.pub_command = rospy.Publisher('/control', AckermannDriveStamped, queue_size=5)
        self.sub_amcl_pose    = rospy.Subscriber('/amcl_pose', PoseStampedWithCovariance, self.callback_amcl_pose)
        self.sub_imu    = rospy.Subscriber('/imu', Imu, self.callback_imu)
        # self.sub_waypoints    = rospy.Subscriber('/waypoint_markers', Marker, self.callback_waypoints)

        self.time = rospy.get_rostime()

        # self.steer = 0;
    def callback_amcl_pose(self, msg):
        self.ego_x = msg.pose.pose.position.x
        self.ego_y = msg.pose.pose.position.y
        self.ego_yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)

    def callback_imu(self, msg):
        self.ego_ax = msg.linear_acceleration.x
        self.ego_yaw_imu = self.quaternion_to_yaw(msg.orientation)
        old_time = self.time
        self.time = rospy.get_rostime()
        self.ego_vx = self.ego_vx + self.ego_ax * (self.time - old_time).to_sec()

    def callback_waypoints(self, msg):
        self.x_list = []
        self.y_list = []
        for i in range(len(msg.points)):
            self.x_list.append(msg.points[i].x)
            self.y_list.append(msg.points[i].y)




    # Calculate distance
    def calc_dist(self,tx, ty, ix, iy):
        return math.sqrt( (tx-ix)**2 + (ty-iy)**2 )

    # Normalize angle [-pi, +pi]
    def normalize_angle(self, angle):
        if angle > math.pi:
            norm_angle = angle - 2*math.pi
        elif angle < -math.pi:
            norm_angle = angle + 2*math.pi
        else:
            norm_angle = angle
        return norm_angle

    # Global2Local
    def global2local(self, ego_x, ego_y, ego_yaw, x_list, y_list):
        """
        TODO 1.
        Transform from global to local coordinate w.r.t. ego vehicle's current pose (x, y, yaw).
            - x_list, y_list               : global coordinate trajectory.
            - ego_x, ego_y, ego_yaw        : ego vehicle's current pose.
            - output_x_list, output_y_list : transformed local coordinate trajectory.
        """

        
        """
        
        xEgo = delX * cos(egoYaw) + dely * sin(egoYaw);
        yEgo = delY * cos(egoYaw) - delx * sin(egoYaw);

        """
        delX = x_list - ego_x
        delY = y_list - ego_y

        output_x_list = delX * np.cos(ego_yaw) + delY * np.sin(ego_yaw)
        output_y_list = delY * np.cos(ego_yaw) - delX * np.sin(ego_yaw)
        # print ("\n x_list ", x_list)
        # print ("\n outputX", output_x_list)
        # print("ego_x ", ego_x)
        # print("ego_y ", ego_y)
        return output_x_list, output_y_list

    # Find nearest point
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
                
        print("near_dist, ",near_dist, " near_ind, ",near_ind)
        return near_dist, near_ind

    # Calculate Error
    def calc_error(self,ego_x, ego_y, ego_yaw, x_list, y_list, wpt_look_ahead=0):
        """
        TODO 3.
        1. Transform from global to local coordinate trajectory.
        2. Find the nearest waypoint.
        3. Set lookahead waypoint.
            - (hint) use the index of the nearest waypoint (near_ind) from 'find_nearest_point'.
            - (hint) consider that the next index of the terminal waypoint is 0, which is not terminal_index + 1.
        4. Calculate errors
            - error_yaw (yaw error)
                : (hint) use math.atan2 for calculating angle btw lookahead and next waypoints.
                : (hint) consider that the next index of the terminal waypoint is 0, which is not terminal_index + 1.
            - error_y (crosstrack error)
                : (hint) y value of the lookahead point in local coordinate waypoint trajectory.
        """

        # 1. Global to Local coordinate
        local_x_list, local_y_list = self.global2local(ego_x, ego_y, ego_yaw, x_list, y_list)

        # 2. Find the nearest waypoint
        _, near_ind = self.find_nearest_point(ego_x, ego_y, x_list, y_list)

        # 3. Set lookahead waypoint (index of waypoint trajectory)
        lookahead_wpt_ind = 10
        # print("local_x_list ",local_x_list,"\nlocal_y_list ", local_y_list)
        # 4. Calculate errors

        wp_del_x = 0 
        wp_del_y = 0
        
        # if near_ind == (len(x_list) - 1):
        #     self.lap = self.lap + 1

        if near_ind + lookahead_wpt_ind/2 >= len(x_list):
            idx = near_ind + lookahead_wpt_ind/2 - len(x_list)
            wp_del_x = x_list[idx] - x_list[near_ind - lookahead_wpt_ind/2]
            wp_del_y = y_list[idx] - y_list[near_ind - lookahead_wpt_ind/2]

        elif near_ind - lookahead_wpt_ind/2 < 0:
            idx = len(x_list) + near_ind - lookahead_wpt_ind/2
            wp_del_x = x_list[near_ind + lookahead_wpt_ind/2] - x_list[idx]
            wp_del_y = y_list[near_ind + lookahead_wpt_ind/2] - y_list[idx]

        else:
            wp_del_x = x_list[near_ind + lookahead_wpt_ind/2] - x_list[near_ind - lookahead_wpt_ind/2]
            wp_del_y = y_list[near_ind + lookahead_wpt_ind/2] - y_list[near_ind - lookahead_wpt_ind/2]

        if wp_del_x == 0:
            if wp_del_y > 0:
                heading_wp =  math.pi/2
            else:
                heading_wp = -math.pi/2
        else:
            heading_wp = math.atan2(wp_del_y,wp_del_x) 
        error_yaw = ego_yaw - heading_wp        

        print("egoYaw ", ego_yaw, " heading yaw ", heading_wp)
        error_yaw = self.normalize_angle(error_yaw) # Normalize angle to [-pi, +pi]
        error_y   = -local_y_list[near_ind] 
        

        return error_y, error_yaw

    # Controller
    def steer_control(self, error_y, error_yaw, ego_vx):
        """
        TODO 4.
        Implement a steering controller (PID controller or Pure pursuit or Stanley method).
        You can use not only error_y, error_yaw, but also other input arguments for this controller if you want.
        """
        K1 = 0.3
        K2 = 0.6        

        # IMPLEMENTING STANLEY CONTROLLER
            #   STEER = YAW_ERROR + ARCTAN2(K*CROSSTRACK_ERROR/LONGITUDINAL_SPEED)
        # print("steer control: ", self.steer)
        # Control limit
        steer = -(K1*error_yaw + math.atan2(K2*error_y,ego_vx))
        steer = np.clip(steer, -self.MAX_STEER, self.MAX_STEER)

        return steer
    # def steer_control_PID(self, error_yaw):
        
    #     Kp = 0.3
    #     Ki = 0.6
    #     Kd = 0.2
    #     # print("steer control: ", self.steer)
    #     # Control limit
    #     self.steer = self.steer + Kp*error_yaw + Ki*self.integral + Kd*(error_yaw - self.prev_error)
    #     self.integral = self.integral + error_yaw
    #     self.prev_error = error_yaw
    #     self.steer = np.clip(self.steer, -self.MAX_STEER, self.MAX_STEER)

    #     return self.steer

        
    def speed_control(self, error_v):
        """
        TODO 5.
        Implement a speed controller (PID controller).
        You can use not only error_v, but also other input arguments for this controller if you want.
        """
        K = 1.0
        throttle = K*error_v
                
        return throttle

    def brake(self):
        msg = AckermannDriveStamped()
        msg.drive.acceleration = -1.0
        self.pub_command.publish(msg)
                
        # return throttle
    def publish_command(self, steer, accel):
        """
        Publish command as AckermannDriveStamped
        ref: http://docs.ros.org/en/jade/api/ackermann_msgs/html/msg/AckermannDriveStamped.html
        """
        msg = AckermannDriveStamped()
        msg.drive.steering_angle = steer / np.deg2rad(17.75)
        msg.drive.acceleration = accel
        self.pub_command.publish(msg)

def main():
    # Load Waypoint
    # rospack = rospkg.RosPack()
    # WPT_CSV_PATH = rospack.get_path('waypoint_follower') + "/wpt_data/wpt_data_dense.csv"
    # csv_data = pd.read_csv(WPT_CSV_PATH, sep=',', header=None)
    # wpts_x = csv_data.values[:,0]
    # wpts_y = csv_data.values[:,1]

    wpts_x = []
    wpts_y = []

        # self.path = Path()
    # self.path.header = Header()
    # self.path.header.frame_id = "map"
    # self.path.poses = []
    path_file = open("path.txt", "r")
    # self.path_array = MarkerArray()
    # self.path_array.markers = []

    # read path from file
    for line in path_file:
        line = line.strip()
        line = line.split(" ")
        # pose = PoseStamped()
        wpts_x.append(float(line[0]))
        wpts_y.append(float(line[1]))


    print("loaded wpt :", wpts_x.shape, wpts_y.shape)
    start_x = wpts_x[0]
    start_y = wpts_y[0]
    prev_flag = 0
    curr_flag = 0
    # Define controller
    wpt_control = WaypointFollower()

    while not rospy.is_shutdown():
        # Get current state

        ego_x = wpt_control.ego_x
        ego_y = wpt_control.ego_y
        ego_yaw = wpt_control.ego_yaw
        ego_vx = wpt_control.ego_vx
        if wpt_control.calc_dist(ego_x,ego_y,start_x, start_y) < 1:
            prev_flag = curr_flag
            curr_flag = 0
        else:
            prev_flag = curr_flag
            curr_flag = 1
            
        if curr_flag:
            if not prev_flag:
                wpt_control.lap += 1 
        # print("main ego_x ", ego_x, " main ego_y ", ego_y)
        # Lateral error calculation (cross-track error, yaw error)
        error_y, error_yaw = wpt_control.calc_error(ego_x, ego_y, ego_yaw, wpts_x, wpts_y, wpt_look_ahead=wpt_control.wpt_look_ahead)
        wpt_control.total_crosstrack_error = wpt_control.total_crosstrack_error + abs(error_y)

        # Longitudinal error calculation (speed error)
        error_v = wpt_control.target_speed - ego_vx
        wpt_control.total_velocity_error = wpt_control.total_velocity_error + abs(error_v)

        # Control
        steer_cmd = wpt_control.steer_control(error_y, error_yaw, ego_vx)
        throttle_cmd = wpt_control.speed_control(error_v)

        # Publish command
        wpt_control.publish_command(steer_cmd, throttle_cmd)

        rospy.loginfo("Commands: (steer=%.3f, accel=%.3f). Errors: (CrossTrackError=%.3f, YawError=%.3f, SpeedError=%.3f)." %(steer_cmd, throttle_cmd, error_y, error_yaw, error_v))
        rospy.loginfo("Total error: (cross-track=%.3f, velocity=%.3f)." %(wpt_control.total_crosstrack_error, wpt_control.total_velocity_error))
        rospy.loginfo("Lap: %d)." %(wpt_control.lap))

        wpt_control.rate.sleep()
        if wpt_control.lap == 3:
            rospy.loginfo("Total average error over two laps: (cross-track=%.3f, velocity=%.3f)." %(wpt_control.total_crosstrack_error/(wpt_control.lap - 1), wpt_control.total_velocity_error/(wpt_control.lap - 1)))
            while (ego_vx > 0):
                wpt_control.brake()
            break


if __name__ == '__main__':
    main()