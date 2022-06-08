#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

BubbleLimit = 350

class reactive_follow_gap:
    def __init__(self):
        #Topics & Subscriptions,Publishers
        lidarscan_topic = '/scan'
        drive_topic = '/nav'

        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback, queue_size=1)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1)
    
    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """

        #print(np.amin(ranges), "minimum before change")

        for i in range(180):
            ranges[i] = 0
            ranges[len(ranges)-i-1] = 0

        #print(np.amin(ranges), "minimum after change")

        for i in range(len(ranges)):
            if ranges[i] >= 5:
                ranges[i] = 5

        return ranges

    def bubble(self, proc_ranges):

        #value = -math.inf
        #index = 0
        #for i in range(len(proc_ranges)):
        #    if proc_ranges[i] > index and proc_ranges[i] != math.inf:
        #        value = proc_ranges[i]
        #        index = i
        #print(index)

        lowest = np.where(proc_ranges ==np.min(proc_ranges[np.nonzero(proc_ranges)]))

        lowest = math.inf
        lowest_index = 0

        for i in range(len(proc_ranges)):
            if proc_ranges[i] < lowest:
                lowest = proc_ranges[i]
                lowest_index = i

        for i in range(lowest_index - BubbleLimit, lowest_index + BubbleLimit):
            proc_ranges[i] = 0

        #wrap_index = 0
        #wrap_iterator = 0
        #check = 0
        #for j in range(BubbleLimit):
        #    #print(j)
        #    if index+j > len(proc_ranges)-1:
        #        proc_ranges[wrap_index+wrap_iterator] = 0
        #        proc_ranges[index-j] = 0
        #        check+=1
        #    elif index-j < 0:
        #        proc_ranges[len(proc_ranges)-1-wrap_iterator] = 0
        #        proc_ranges[index+j] = 0
        #        check+=1
        #    else:
        #        proc_ranges[index+j] = 0
        #        proc_ranges[index-j] = 0
        #        check+=1
        #check
        #assert check == BubbleLimit
        #for i in range(len(proc_ranges)):
        #    print(proc_ranges[i])
        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """

        max_subset = []
        current_max_subset = []
        end_index = 0
        index = [0,0]

        for n in range(len(free_space_ranges)):
            if free_space_ranges[n] > 0:
                current_max_subset.append(free_space_ranges[n])
            else:
                if len(current_max_subset) > len(max_subset):
                    max_subset = current_max_subset
                    end_index = n
                current_max_subset = []
        start_index = end_index - len(max_subset)
        index = [start_index, len(max_subset)]
        return index

        #max_gap=[0,0,0]
        #ranges = list(free_space_ranges)
        #ranges.append(0)
        #for i in range(len(ranges)):
        #    print(ranges[i])
        #count = 0
        #longest = 0
        #for i in range(len(ranges)-1):
        #    if ranges[i] != 0:
        #        count += 1
        #    else:
        #        if count > longest:
        #            max_gap[1] = count
        #            longest = count
        #        count = 0
        #print(longest)
        #max_gap = [max_gap[1]-longest, max_gap[1], longest]
        #print(max_gap)

            #print(i)
            #j = i
            #iterator = 0
            #while free_space_ranges[i] > 0 and i < len(free_space_ranges)-1:
            #    iterator += 1
            #    i += 1
            #    if iterator > max_gap[2]:
            #        max_gap = [j, i, i-j+1]

        #print(max_gap)
        #return max_gap
    
    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	Naive: Choose the furthest point within ranges and go there
        """

        m = max(ranges)
        consecutive = [i for i, j in enumerate(ranges) if j == m]
        middle = round(len(consecutive)/2)
        middle = consecutive[0] + middle
        return middle
        #value = -math.inf
        #index = 0
        #print(start_i, end_i)
        #for i in range(start_i, end_i):
        #    if value < ranges[i] and ranges[i] != math.inf:
        #        value = ranges[i]
        #        index = i


        #return index

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = np.array(data.ranges)
        proc_ranges = self.preprocess_lidar(ranges)
        proceed_ranges = self.bubble(proc_ranges)
        max_gap = self.find_max_gap(proceed_ranges)
        best_point = self.find_best_point(max_gap[0], max_gap[1], proceed_ranges)
        scan_difference = len(proceed_ranges)/2 - best_point
        assert len(proceed_ranges)/2 - -90 == 630
        degree_difference = scan_difference*math.degrees(data.angle_increment)
        #print(degree_difference)
        radian_difference = degree_difference* math.pi/180

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = ''
        drive_msg.drive.steering_angle = -radian_difference
        #print(radian_difference)
        if degree_difference > 20 or degree_difference < -20:
            drive_msg.drive.speed = 2
        elif degree_difference > 10 or degree_difference < -10:
            drive_msg.drive.speed = 2
        else:
            drive_msg.drive.speed = 2
                #print('here')
        self.drive_pub.publish(drive_msg)

        #Find closest point to LiDAR

        #Eliminate all points inside 'bubble' (set them to zero) 

        #Find max length gap 

        #Find the best point in the gap 

        #Publish Drive message

def main(args):
    rospy.init_node("FollowGap_node", anonymous=True)
    rfgs = reactive_follow_gap()
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
