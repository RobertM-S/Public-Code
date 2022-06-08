#!/usr/bin/env python
import rospy
import numpy as np
import math
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from math import sqrt


class Safety(object):


    def __init__(self):

        # setup subscribers and publishers

        self.odom_sub = rospy.Subscriber(
            '/odom', Odometry, self.odom_callback, queue_size=1)

        self.scan_sub = rospy.Subscriber(
            '/scan', LaserScan, self.scan_callback, queue_size=1)

        self.Ackermann_Brake_pub = rospy.Publisher(
            '/brake', AckermannDriveStamped, queue_size=10)

        self.Brake_Bool_pub = rospy.Publisher(
            '/brake_bool', Bool, queue_size=10)

    def odom_callback(self, odom_msg):
        #sets the x, y and z movement values from the odometry to velocity
        self.velocity = odom_msg.twist.twist.linear


    def scan_callback(self, scan_msg):
    #Publishes to the brake and AckermannDriveStamped topic if the car is about to crash
        ranges = np.array(scan_msg.ranges)
        TTC = 100

        #loops through all ranges and based on the angular velocity calculates time to collision
        for i in range(0, len(scan_msg.ranges)):

            if ((~np.isnan(ranges[i])) and (~np.isinf(ranges[i]))):
                assert ranges[i] != np.inf
                assert ranges[i] != np.nan
                distance = scan_msg.ranges[i]
                angle = scan_msg.angle_min + scan_msg.angle_increment * i
                angular_velocity = math.cos(
                    angle) * self.velocity.x + math.sin(angle) * self.velocity.y
                if angular_velocity > 0:
                    if (~np.isinf(distance/angular_velocity)):
                    	if((distance/angular_velocity) < TTC):
                        	TTC = distance/angular_velocity
                        	assert TTC > 0

        #if TTC is above set value, set speed to 0 and apply brakes
        if TTC <= 0.35:
            Ackermann_Msg = AckermannDriveStamped()
            Ackermann_Msg.header.stamp = rospy.Time.now()
            Ackermann_Msg.header.frame_id = ''
            Ackermann_Msg.drive.steering_angle = 0
            Ackermann_Msg.drive.speed = 0
            self.Ackermann_Brake_pub.publish(Ackermann_Msg)
            
            Bool_Msg = Bool()
            Bool_Msg.data = True
            self.Brake_Bool_pub.publish(Bool_Msg)
            
        else:
            Bool_Msg = Bool()
            Bool_Msg.data = False
            self.Brake_Bool_pub.publish(Bool_Msg)



def main():
    rospy.init_node('safety_node', anonymous=True)
    sn = Safety()
    rospy.spin()


if __name__ == '__main__':
    main()
