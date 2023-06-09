#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from std_msgs.msg import Float32

# Car parameters ----------------------------
# The distance between the front and
# rear axle of the racecar (wheelbase)
L = 0.3302 #meters
# The width of racecar
W = 0.2032 #meters
car_length = 0.5

max_steering_angle = 0.4189

# The distance from the center of the
# rear axis (base_link) to the lidar
scan_distance_to_base_link = 0.275 # meters

# LiDAR parameters --------------------------
LASER_SCAN_COUNT = 1080 #From right(-45o) to left(225o) 1080 measurements
MAX_POSITIVE_ANGLE = 225
MAX_NEGATIVE_ANGLE = 45

# Algoritmh parameters ----------------------
velocity = 0.1 #m/s
angle_range = 1.22 #rad -> 70 degrees
# angle_diff and length_limit for the triangle
angle_limit = 0.7 #rad -> 45 degrees
length_limit = 0.5 #meters 
left_wall_dist = 0.50 #meters

# Notes -------------------------------------
# Vehicle moves on the x axis towards the positive values.
# The parking space is located on the left side of the car.
# The y values are positive to the left side of the car.


class FindParking:
    def __init__(self):
        #Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/nav'
        odom_topic = '/odom'
        
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10)
        rospy.sleep(rospy.Duration(2.0))
        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback)
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.odom_callback)
        self.got_parking = False
        self.parked = False
        self.drive_msg = AckermannDriveStamped()
        self.drive_msg.header.frame_id = "laser"
        
    def get_range(self, msg, angle):
        # msg: single message from topic /scan
        # angle: between -45 to 225 degrees, where 0 degrees is directly to the right
        # Outputs length in meters to object with angle in lidar scan field of view
        angle_range_2 = MAX_POSITIVE_ANGLE + MAX_NEGATIVE_ANGLE
        factor = LASER_SCAN_COUNT/angle_range_2
        new_angle = angle + MAX_NEGATIVE_ANGLE #Get angle from [-negative_limit,postive_limit] to [0,positive+negative]
        index = round( (new_angle)*factor )
        return msg.ranges[index]
    
    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.orientation = msg.pose.pose.orientation.z
        self.drive_msg.header.stamp = rospy.Time.now()
        self.drive_pub.publish(self.drive_msg)
        
    def run(self):
        rospy.loginfo("Moving Car Forward")
        self.drive_msg.drive.speed = velocity
        self.drive_msg.drive.steering_angle = 0.0

    def lidar_callback(self, msg):
        if not self.got_parking:
            reversed_ranges = reversed(msg.ranges)
            angle_min = -0.785 # rad -> -45 degrees
            angle_increment = msg.angle_increment
            angles = []
            ranges = []
            for idx, item in enumerate(reversed_ranges):
                angle = angle_min + idx * angle_increment
                #print(angle)
                if (angle > 0.0 and angle < angle_range):
                    # perpendicular distance between car and measurement
                    perp = math.cos(angle)*item
                    if (perp < left_wall_dist):
                        angles.append(angle)
                        ranges.append(item)
            
            for i in range(1,len(angles)):
                ang_diff = angles[i] - angles[i-1]
                if (ang_diff > angle_limit and ranges[i] > length_limit):
                    self.got_parking = True
                    
                    self.drive_msg.drive.speed = 0.0
                    self.drive_msg.drive.steering_angle = 0.0
                    
                    rospy.loginfo("Parking found")

                    dx = ranges[i] * math.sin(angles[i])
                    dy = ranges[i] * math.cos(angles[i])
                    # The midle point of the back bumper of the front vehicle parked
                    self.point_x = self.x + dx + scan_distance_to_base_link
                    self.point_y = self.y + dy - W/2
                    string = "Point detected: x "+str(self.point_x)+" y "+str(self.point_y)
                    rospy.loginfo(string)
        if self.got_parking and not self.parked:
            while self.x < self.point_x + 0.1:
                self.drive_msg.drive.speed = velocity
                self.drive_msg.drive.steering_angle = 0.0
            while self.y < self.point_y - W/2 - 0.07:
                self.drive_msg.drive.speed = -velocity
                self.drive_msg.drive.steering_angle = max_steering_angle
            while self.orientation < 0:
                self.drive_msg.drive.speed = -velocity
                self.drive_msg.drive.steering_angle = -max_steering_angle
            while self.x < self.point_x - L - 0.2:
                self.drive_msg.drive.speed = velocity
                self.drive_msg.drive.steering_angle = 0.0
            self.drive_msg.drive.speed = 0.0
            self.drive_msg.drive.steering_angle = 0.0
            self.parked = True
            

                

def main(args):
    rospy.init_node('find_parking')
    rospy.loginfo("Node Started")
    find_parking = FindParking()
    find_parking.run()
    rate = rospy.Rate(10) # 10 hz
    while not rospy.is_shutdown():
            rate.sleep()


if __name__=='__main__':
	main(sys.argv)

