#!/usr/bin/env python

import rospy
import cv2 as cv
from std_msgs.msg import *
from nav_msgs.msg import *
from geometry_msgs.msg import * 
from visualization_msgs.msg import *
import actionlib
import math
import time 
import random

mapData = OccupancyGrid()
clickedpoint = PointStamped()
exploration_goal = PointStamped()
points = Marker()
line = Marker()

def mapCallback(msg):
	mapData *= msg

def rvizCallback(msg):
	p = Point() 
	p.x = msg.point.x 
	p.y = msg.point.y 
	p.z = msg.point.z 

	points.points.append(p)

def main():
	pub = rospy.Publisher("/detected_points", PointStamped, queue_size=10)
	rospy.init_node("local_rrt_frontier_detector", anonymous=True)

	ns = String()
	ns = rospy.get_name() 
	map_topic = String()
	base_frame_topic = String()
	eta = rospy.get_param(ns+"/eta", 0.5)
	map_topic = rospy.get_param(ns+"/map_topic", "/robot_1/map")
	base_frame_topic = rospy.get_param(ns+"/robot_frame", "/robot_1/base_link")

	rate = rospy.rate(100)