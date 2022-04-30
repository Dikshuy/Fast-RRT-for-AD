from __future__ import division
import yaml
from shapely.geometry import Point, Polygon, LineString, box
from environment import Environment

from planner import SamplingBasedPathPlanner

environment = Environment('simple.yaml')
bounds = (-2, -3, 12, 8)
start_pose = (0, 0)
goal_region = Polygon([(10,5), (10,6), (11,6), (11,5)])
object_radius = 0.3
steer_distance = 0.3
num_iterations = 10000
resolution = 3
drawResults = True
runForFullIterations = False
isLazy = True

sbpp = SamplingBasedPathPlanner()
path, v, e = sbpp.RRTStar(environment, bounds, start_pose, goal_region, object_radius, steer_distance, num_iterations, resolution, runForFullIterations, drawResults)
print(path)