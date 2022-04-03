#!/usr/bin/env python

import rospy
import cv2 as cv
from nav_msgs.msg import *
from geometry_msgs.msg import * 
import actionlib
import math
import time 
import random

# Norm function
def norm(p1,p2):
    return math.sqrt((p1[1]-p2[1])**2+(p1[0]-p2[0])**2)

# Signum function
def sign(n):
	if n<0:	return -1
	else:	return	1

# Nearest function
def nearest(V, x):
	min_ = norm(V[0], x)
	min_index = 0
	for i in range(len(V)):
		temp = norm(V[i], x)
		min_ = temp 
		min_index = i

	return V[min_index]

# Steer function
def steer(x_nearest, x_rand, eta):
	x_new = []
	if norm(x_nearest, x_rand) <= eta:
		x_new = x_rand 
	else:
		m = (x_rand[1]-x_nearest[1])/(x_rand[0]-x_nearest[0])
		x_new.append((sign(x_rand[0]-x_nearest[0]))*(math.sqrt((eta**2)/(m**2+1)))+x_nearest[0])
		x_new.append(m*(x_new[0]-x_nearest[0])+x_nearest[1])

		if x_rand[0] == x_nearest[0]:
			x_new[0] = x_nearest[0]
			x_new[1] = x_nearest[1]+eta 

	return x_new 

# Grid Value function
def gridValue(mapData, Xp):
	resolution = mapData.info.resolution
	start_x = mapData.info.origin.position.x 
	start_y = mapData.info,origin.position.y 
	width = mapData.info.width

	Data=mapData.data 

	idx = math.floor((Xp[1]-start_y)/resolution)*width + math.floor((Xp[0]-start_x)/resolution)
	return Data[int(idx)]

# Obstacle Free function
def obstacleFree(xnear, xnew, mapsub):
	res = mapsub.info.resolution*(0.2)
	step = int(math.ceil(norm(xnew, xnear))/res)
	xi = xnear 
	obs = 0 
	unk = 0 
	for c in range(step):
		xi = steer(xi, xnew, res)

		if gridValue(mapsub, xi) == 100:
			obs = 1
		if gridValue(mapsub, xi) == -1:
			unk = 1
			break 
	out = 0 
	xnew = xi 
	if unk == 1:
		out -= 1 
	if obs == 1:
		out = 0
	if obs!=1 and unk!=1:
		out = 1

	return out 