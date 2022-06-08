#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

#PID CONTROL PARAMS
kp = 1.3
kd = 0.5
ki = 0.01
servo_offset = 0.0
prev_error = 0.0
error = 0.0
integral = 0.0

#WALL FOLLOW PARAMS
ANGLE_RANGE = 270 # Hokuyo 10LX has 270 degrees scan
DESIRED_DISTANCE_RIGHT = 1 # meters
DESIRED_DISTANCE_LEFT = 0.55
VELOCITY = 2.00 # meters per second
CAR_LENGTH = 0.50 # Traxxas Rally is 20 inches or 0.5 meters

class WallFollow:

    def __init__(self):
        #initiates Subscribers and Publishers
        lidarscan_topic = '/scan'
        drive_topic = '/nav'

        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback, queue_size=1)

        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1)



    def pid_control(self, ProjectedError, velocity, angle, error):
    #PID controller that calculates smooth turning angle into central line and publishes to AckermannDriveStamped
        global prev_error
        global kp
        global ki
        global kd
        DesiredAngle = kp * ProjectedError + kd * (error/ProjectedError) * ProjectedError
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = ''
        drive_msg.drive.steering_angle = DesiredAngle
        drive_msg.drive.speed = 2

        prev_error = ProjectedError
        self.drive_pub.publish(drive_msg)

    def lidar_callback(self, data):
    #Calculates current facing angle away from desired line and deviation from line at current and future points
        offset = 0.03
        ranges = np.array(data.ranges)

        real_ranges = ranges[np.logical_and(
            ~np.isnan(ranges), ~np.isinf(ranges))]

        a_angle = 140
        b_angle = 90


        a_dist = data.ranges[round((a_angle / 180.0 * math.pi)/data.angle_increment)]
        b_dist = data.ranges[round((b_angle / 180.0 * math.pi)/data.angle_increment)]
        if (~np.isnan(a_dist) and ~np.isinf(a_dist)) and (~np.isnan(b_dist), ~np.isinf(b_dist)):
            #assert ~np.isnan(a_dist) or ~np.isnan(b_dist) or ~np.isinf(a_dist) or ~np.isinf(b_dist)
            angle = math.degrees(math.atan((a_dist * math.cos(math.radians(a_angle - b_angle)) - b_dist) / (a_dist * math.sin(math.radians(a_angle - b_angle)))))
            #assert round(math.degrees(math.atan((1 * math.cos(math.radians(30 - 60)) - 1.5) / (1 * math.sin(math.radians(30 - 60)))))) == 52
            Distance = b_dist * math.cos(math.radians(angle))
            #assert round(2 * math.cos(math.radians(180))) == -2
            error = DESIRED_DISTANCE_RIGHT - Distance
            ProjectedError = DESIRED_DISTANCE_RIGHT - (Distance + 0.75 * math.sin(math.radians(angle)))
            self.pid_control(ProjectedError, VELOCITY, angle, error)
        else:
            pass

def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    wf = WallFollow()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)