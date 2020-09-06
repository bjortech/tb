#!/usr/bin/env python
from subprocess import (PIPE, Popen)
from sys import stdout, stdin, stderr
import cv2
import time
import sys
import rospy
from std_msgs.msg import String

proc_list = []
def invoke(command):
	proc = Popen(command, shell=True, stdin=stdin, stdout=stdout, stderr=stderr)
	proc_list.append(proc)

def callback(msg):
	invoke(msg.data)
	
def main_program():
	rospy.init_node('tb_invoke_node')
	s1 = rospy.Subscriber('/tb_invoke', String, callback)
	rospy.spin()

if __name__ == '__main__':
	try:
		main_program()
	except rospy.ROSInterruptException:
		pass
