#!/usr/bin/env python

# convert cmd_vel to pwm
import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16

class cmdVelToPWM:
    def __init__(self):
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmdVelCallback)
        self.throttle_pwm_pub = rospy.Publisher('/auto_cmd/throttle', Int16, queue_size=10)
        self.steer_pwm_pub = rospy.Publisher('/auto_cmd/steer', Int16, queue_size=10)
        self.throttle_pwm = 0
        self.steer_pwm = 0
        self.throttle_center = 1500
        self.steer_center = 1500
        self.throttle_max = 1700
        self.throttle_min = 1300
        self.steer_max = 1700
        self.steer_min = 1300


    def cmdVelCallback(self, msg):
        # convert cmd_vel to pwm
        self.throttle_pwm = self.throttle_center + int(msg.linear.x * 100)
        self.steer_pwm = self.steer_center + int(msg.angular.z * 50)


        # publish pwm
        self.throttle_pwm_pub.publish(min(self.throttle_max, max(self.throttle_min,self.throttle_pwm)))
        self.steer_pwm_pub.publish(min(self.steer_max, max(self.steer_min,self.steer_pwm)))

    def emergencyStop(self):
        # convert cmd_vel to pwm
        # min(1900, max(1100, autoSteer)
        self.throttle_pwm = self.throttle_center
        self.steer_pwm = self.steer_center

        # publish pwm
        self.throttle_pwm_pub.publish(self.throttle_pwm)
        self.steer_pwm_pub.publish(self.steer_pwm)

    # emergency stop when node is killed
    def __del__(self):
        self.emergencyStop()

if __name__ == '__main__':
    rospy.init_node('cmdVelToPWM', anonymous=True)
    cmdVelToPWM = cmdVelToPWM()
    rospy.spin()

