#!/usr/bin/env python
# import the necessary packages
import rospy
import argparse
import math
import cv2
from std_msgs.msg import Header, Empty
import sensor_msgs.point_cloud2 as pcl2
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
camera_feed = cv2.VideoCapture(0)

def callback(msg):
	global scaled_polygon_pcl,pub
	pub.publish(scaled_polygon_pcl)

def main_program():
	global scaled_polygon_pcl,pub
	rospy.init_node('tb_heightimg2pointcloud_node')
	pub = rospy.Publisher('/tb_pc2_full', PointCloud2, queue_size=1)
	image = cv2.imread(rospy.get_param('~image_path', '/home/nuc/mapimages/map_to_publish2.png'))
	pointlist = []
	for c in range((len(image[0])-1)):
		for r in range((len(image))):
			x = (c - len(image[0])/2)
			y = (len(image)/2 - r)
			z = int(round((float(image[r][c][0]) / 255.0)*25.0))
			#for z in range(image[r][c][0]/5):
			pointlist.append([x,y,z])
	header = Header()
	header.frame_id = 'map'
	scaled_polygon_pcl = pcl2.create_cloud_xyz32(header, pointlist)
	rospy.Timer(rospy.Duration(1), callback)
	rospy.spin()
if __name__ == '__main__':
	try:
		main_program()
	except rospy.ROSInterruptException:
		pass
