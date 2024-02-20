#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
import sensor_msgs.point_cloud2 as pc2

def pointcloud_callback(data):
    # Process the received PointCloud2 data
    pc_data = pc2.read_points(data)
    list1= []
    list2 = []
    for p in pc_data:
        list1.append(p[0:3])
        list2.append(p[3])
    print("list1",list1[1])
    print("list2",list2)
        
        
    

def pointcloud_subscriber():
    rospy.init_node('pointcloud_subscriber_node', anonymous=True)
    rospy.Subscriber('/labeled_objects', PointCloud2, pointcloud_callback)
    rospy.spin()

if __name__ == '__main__':
    pointcloud_subscriber()
