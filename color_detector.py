# -*- coding: utf-8 -*-
"""
Created on Tue Feb  4 18:43:18 2020

@author: user
"""

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cmvision.msg import Blobs, Blob

from sensor_msgs.msg import PointCloud2
import point_cloud2


class color_detector():
    def __init__(self):
        rospy.init_node("color_detector")
        rospy.Subscriber('blobs', Blobs, self.blob_callback)
        rospy.Subscriber('point_cloud', PointCloud2, self.set_cmd_vel)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
        
        # Set the shutdown function (stop the robot)
        rospy.on_shutdown(self.shutdown)
        self.stop = False
        
        self.num_blobs_threshold = 300
        self.point_cloud_threshold = 4000
        
    def blob_callback(self, blobs):
        num_blobs = 0
        for blob in blobs.blobs:
            if blob.red == 255 and blob.green == 0 and blob.blue == 0:
                num_blobs += 1
        if num_blobs > self.num_blobs_threshold and not self.stop:
            rospy.loginfo("Color detected.")
            self.move_forward(0.1)
        else:
            rospy.loginfo("Color not detected.")
            self.look_around(0.1)
    
    def move_forward(self, linear_speed):
        move_cmd = Twist()
        move_cmd.linear.x = linear_speed
        # Publish the movement command
        while not rospy.is_shutdown():
            self.cmd_vel_pub.publish(move_cmd)
        
    def look_around(self, angular_speed):
        move_cmd = Twist()
        move_cmd.angular.z = angular_speed
        # Publish the movement command
        while not rospy.is_shutdown():
            self.cmd_vel_pub.publish(move_cmd)
        
    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)
        
    def set_cmd_vel(self, msg):
        # Initialize the centroid coordinates point count
        x = y = z = n = 0
        
        # Read in the x, y, z coordinates of all points in the cloud
        for point in point_cloud2.read_points(msg, skip_nans=True):
            pt_x = point[0]
            pt_y = point[1]
            pt_z = point[2]
            
            x += pt_x
            y += pt_y
            z += pt_z
            n += 1
                
        # Stop the robot by default
        move_cmd = Twist()
        
        rospy.loginfo(n)
        
        # stop if number of points is large enough
        if n > self.point_cloud_threshold:
            self.stop = True
            move_cmd.linear.x = 0
            move_cmd.angular.z = 0
                        
        # Publish the movement command
        self.cmd_vel_pub.publish(move_cmd)
        
        
        
if __name__ == '__main__':
    try:
        color_detector()
        rospy.spin()
    except:
        rospy.loginfo('Nothing happend...')
