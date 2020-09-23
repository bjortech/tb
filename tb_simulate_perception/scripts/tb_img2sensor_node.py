#!/usr/bin/env python
# import the necessary packages
import rospy
import argparse
import math
import cv2
from std_msgs.msg import Header, Empty, Float64
import sensor_msgs.point_cloud2 as pcl2
import rospy
from sensor_msgs.msg import PointCloud2
import geometry_msgs.msg
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
camera_feed = cv2.VideoCapture(0)

def y2r(y,npx):
  return (npx/2 - y)
def x2c(x,npx):
  return (x + npx/2)
def r2y(r,npx):
  return npx/2 - r
def c2x(c,npx):
  return (c - npx/2)

def get_shortest(target_hdng,actual_hdng):
  a = target_hdng - actual_hdng;
  if(a > math.pi):
	  a -= math.pi*2;
  elif(a < -math.pi):
	  a += math.pi*2;
  return a;

def callback(msg):
	global scaled_polygon_pcl,pub,par_zmax,par_rad,par_dhdng,par_dincl,listener,radius,image
	try:
		(trans,rot) = listener.lookupTransform('/map', '/base_stabilized', rospy.Time(0))
		(roll, pitch, yaw) = euler_from_quaternion(rot)
		pointlist = []
		px = int(round(trans[0]))
		py = int(round(trans[1]))
		px0 = max(px - radius,-len(image[0])/2)
		py0 = max(py - radius,-len(image)/2)
		px1 = min(px + radius,len(image[0])/2)
		py1 = min(py + radius,len(image)/2)
		xrange = px1 - px0
		yrange = py1 - py0
		for yn in range(yrange):
			for xn in range(xrange):
				y = py0 + yn
				x = px0 + xn
				c = x2c(x,len(image[0]))
				r = y2r(y,len(image))
				dx = x - px
				dy = y - py
				dst = math.sqrt(dx*dx+dy*dy)
				hdng = math.atan2(dy,dx);
				dhdng = get_shortest(hdng,yaw)
				if dhdng < 0:
					dhdng = dhdng * -1.0
				if(dhdng < par_dhdng):
					altatpnt = int(round((float(image[r][c][0]) / 255.0)*par_zmax))
					for z in range(altatpnt):
						incl  = math.atan2(z - trans[2],dst)
						dincl = get_shortest(incl,pitch)
						if dincl < 0:
							dincl = dincl * -1.0
						#print("incl: " + str(incl) + " dincl: " + str(dincl))
						if(dincl < par_dincl):
							pointlist.append([x,y,z])
							#print("altatpnt: " + str(altatpnt) + " hdng: " + str(hdng) + " dhdng: " + str(dhdng) + " yaw: " + str(yaw))
		header = Header()
		header.frame_id = 'map'
		header.stamp = rospy.Time.now()
		scaled_polygon_pcl = pcl2.create_cloud_xyz32(header, pointlist)
		pub.publish(scaled_polygon_pcl)
	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
  		print("Something else went wrong")

def main_program():
	global scaled_polygon_pcl,pub,par_zmax,par_rad,par_dhdng,par_dincl,listener,radius,image
	rospy.init_node('tb_img2sensor_node')
	par_pathstring = rospy.get_param('~image_path', '/home/nuc/mapimages/map_to_publish2.png')
	par_zmax = rospy.get_param('~image_elevationmax',25.0)
	par_rad  = rospy.get_param('~radius_surroundcloud',30.0)
	par_dhdng  = rospy.get_param('~limit_dhdng',1.5)
	par_dincl  = rospy.get_param('~limit_dincl',0.50)
	pub = rospy.Publisher('/velodyne_points', PointCloud2, queue_size=1)
	listener = tf.TransformListener()
	radius = int(round(par_rad))
	image = cv2.imread(par_pathstring)
	rospy.Timer(rospy.Duration(1), callback)
	rospy.spin()


if __name__ == '__main__':
	try:
		main_program()
	except rospy.ROSInterruptException:
		pass
