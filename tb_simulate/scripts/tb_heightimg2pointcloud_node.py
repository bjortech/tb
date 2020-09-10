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
	par_pathstring = rospy.get_param('~image_path', '/home/nuc/mapimages/d2.png')
	pub = rospy.Publisher('/tb_pc2_full', PointCloud2, queue_size=1)
	par_zmax = rospy.get_param('~image_elevationmax',50.0)
	par_zres = rospy.get_param('~pointcloud_zresolution',3.0)
	image = cv2.imread(par_pathstring)
	pointlist = []
	for c in range((len(image[0])-1)):
		for r in range((len(image))):
			x = (c - len(image[0])/2)
			y = (len(image)/2 - r)
			z = (image[r][c][0])
			z = int(round((float(image[r][c][0]) / 255.0)*par_zmax))
			altatpnt = int(round((float(image[r][c][0]) / 255.0)*par_zmax))
			for z in range(altatpnt/int(round(par_zres))):
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
