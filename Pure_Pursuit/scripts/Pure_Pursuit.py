#!/usr/bin/env python
import sys
import math
import numpy as np
import csv
import os, rospkg
import copy

import rospy
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

# TODO: import ROS msg types and libraries

class PurePursuit(object):
    """
    The class that handles pure pursuit.
    """
    def __init__(self):
        # TODO: create ROS subscribers and publishers.
        odom_topic = '/odom'
        drive_topic = '/nav'
        marker_topic = 'visualization_marker_array'
        lookahead_topic = 'visualization_marker'

        self.markerArray = MarkerArray()
        self.count = 0
        self.Lookahead = 1

        rospack = rospkg.RosPack()
        cwd = os.getcwd()
        print(cwd)
        #file = open('/home/robert/robert_ws/src/Pure_Pursuit/scripts/levine_blocked.csv', 'r')
        #print(file)
        with open(os.path.join(rospack.get_path("Pure_Pursuit"), "scripts", "levine_blocked.csv"), 'r') as csvfile:
            cvsreader = csv.reader(csvfile)
            self.waypointx = []
            self.waypointy = []
            self.waypointhead = []
            for row in cvsreader:
                self.waypointx.append(row[0])
                self.waypointy.append(row[1])
                self.waypointhead.append(row[2])

        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.odom_callback, queue_size=1)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1)
        self.marked_pub = rospy.Publisher(marker_topic, MarkerArray, queue_size = 10000)
        self.lookahead_pub = rospy.Publisher(lookahead_topic, Marker, queue_size = 2)


        #print(self.waypointhead)





    #def map_waypoint(self):


        #markerArray = MarkerArray()
        #marker = Marker()
        #point = Point()
        #marker.header.stamp = rospy.Time()
        #marker.header.frame_id = "map"
        #marker.id = 0
        #marker.type = self.marker.POINTS
        #marker.action = self.marker.ADD
        #marker.pose.position.x = 0.0
        #marker.pose.position.y = 0.0
        #marker.pose.position.z = 0.0
        #marker.pose.orientation.x = 0.0
        #marker.pose.orientation.y = 0.0
        #marker.pose.orientation.z = 0.0
        #marker.pose.orientation.w = 1.0
        #marker.scale.x = 1
        #marker.scale.y = 0.1
        #marker.scale.z = 0.1
        #marker.color.a = 1.0  # Don't forget to set the alpha!
        #marker.color.r = 0.0
        #marker.color.g = 1.0
        #marker.color.b = 0.0
        #for i in range(len(waypointx)):
        #    self.marker.id = i
            #self.marker.pose.position.x = 1.2#waypointx[i]
            #self.marker.pose.position.y = 0.5#waypointy[i]
            #self.marker.pose.position.z = -3.2#waypointhead[i]
        #    self.point.x = waypointx[i]
        #    self.point.y = waypointy[i]
        #    self.point.z = 0.0
        #    self.marker.points.insert(len(self.marker.points), self.point)
            #print(self.marker)
            #self.markerArray.markers.append(self.marker)
        #print(self.marker.points)
        #self.marked_pub.publish(self.marker)
        #id = 0
        #for m in self.marker.markers:
        #    m.id = id
        #    id += 1
        #print(self.markerArray)
        #print(self.markerArray)
        #self.marked_pub.publish(markerArray)
        #self.marked_pub.publish(marker)


    def get_Closest_Waypoint(self, odometry):
        #print(odometry.x, odometry.y)
        closest = math.inf
        closest_index = 0
        for n in self.markerArray.markers:
            magnitude = math.sqrt((odometry.x - n.pose.position.x)**2 + (odometry.y - n.pose.position.y)**2)
            if magnitude < closest:
                closest = magnitude
                closest_index = n
        return closest_index, magnitude

    def get_Lookahead_Waypoint(self, waypoint_id, waypoint_magnitude, odometry):
        minimum = 0
        minimum_index = 0
        for i in self.markerArray.markers:
            if i.id >= waypoint_id:
        #for i in range(waypoint_id, len(self.waypointx)):
                magnitude = math.sqrt((odometry.x - i.pose.position.x)**2 + (odometry.y - i.pose.position.y)**2)
                if magnitude >= self.Lookahead:
                    return i
        for i in self.markerArray.markers:
            magnitude = math.sqrt((odometry.x - i.pose.position.x)**2 + (odometry.y - i.pose.position.y)**2)
            if magnitude >= self.Lookahead:
                return i





    def get_Heading(self, orientation, odometry, lookahead_point):
        heading = math.atan2((2*(orientation.w * orientation.z + orientation.x * orientation.y)), (1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)))
        magnitude = math.sqrt((odometry.x - lookahead_point.pose.position.x)**2 + (odometry.y - lookahead_point.pose.position.y)**2)
        angle_to_lookahead = math.atan2(lookahead_point.pose.position.y - odometry.y, lookahead_point.pose.position.x - odometry.x)
        parallel_distance = magnitude * math.sin(angle_to_lookahead - heading)
        return parallel_distance, magnitude


    def odom_callback(self, pose_msg):
        while self.count < len(self.waypointx):

            marker = Marker()
            marker.header.frame_id = "map"
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = (float(self.waypointx[self.count]))
            marker.pose.position.y = (float(self.waypointy[self.count]))
            marker.pose.position.z = 0.0


            self.markerArray.markers.append(marker)

            id = 0
            for m in self.markerArray.markers:
               m.id = id
               id += 1

           # Publish the MarkerArray
            self.marked_pub.publish(self.markerArray)
            #print('still here')
            self.count += 1
            if self.count + 1 == len(self.waypointx):
                rospy.sleep(0.1)
                self.marked_pub.publish(self.markerArray)
                rospy.sleep(0.1)

        close_waypoint, close_magnitude = self.get_Closest_Waypoint(pose_msg.pose.pose.position)
        lookahead_Waypoint = self.get_Lookahead_Waypoint(close_waypoint.id, close_magnitude, pose_msg.pose.pose.position)
        #for i in self.markerArray.markers:
        #    if lookahead_Waypoint_id + 1 == i.id:
        #        lookahead_Waypoint = i
        temp_close_id = close_waypoint.id
        close_waypoint.scale.x = 0.4
        close_waypoint.scale.y = 0.4
        close_waypoint.color.g = 1.0
        close_waypoint.id = 1
        self.lookahead_pub.publish(close_waypoint)
        close_waypoint.id = temp_close_id

        temp_lookahead_id = lookahead_Waypoint.id
        lookahead_Waypoint.scale.x = 0.4
        lookahead_Waypoint.scale.y = 0.4
        lookahead_Waypoint.color.b = 1.0
        lookahead_Waypoint.id = 0
        self.lookahead_pub.publish(lookahead_Waypoint)
        lookahead_Waypoint.id = temp_lookahead_id

        steering_angle, angle_magnitude = self.get_Heading(pose_msg.pose.pose.orientation, pose_msg.pose.pose.position, lookahead_Waypoint)

        angle = 2*steering_angle/angle_magnitude**2
        #print(angle)

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = ''
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = 2.5
        self.drive_pub.publish(drive_msg)
        # pose_msg.pose.pose.orientation.x 1 = north

        #print(len(MarkerArray))
        pass
        # TODO: find the current waypoint to track using methods mentioned in lecture

        # TODO: transform goal point to vehicle frame of reference

        # TODO: calculate curvature/steering angle

        # TODO: publish drive message, don't forget to limit the steering angle between -0.4189 and 0.4189 radians


def main(args):
    rospy.init_node('pure_pursuit_node', anonymous=True)
    pp = PurePursuit()
    rospy.sleep(0.1)
    rospy.spin()
if __name__ == '__main__':
    main(sys.argv)
