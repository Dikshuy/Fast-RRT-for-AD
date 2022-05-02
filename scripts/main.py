from algos import algo
from drawer import draw_results
from shapely.geometry import Point, Polygon, LineString, box
from environment import Environment
import time
import yaml

class Solver():
    def __init__(self):
        self.algos = algo()

    def RRT(self, environment, bounds, start_pose, goal_region, object_radius, steer_distance, num_iterations, resolution=3, runForFullIterations=False, drawResults=False):
        start_time = time.time()
        path, V, E = self.algos.RRT(environment, bounds, start_pose, goal_region, object_radius, steer_distance, num_iterations, resolution, runForFullIterations)
        elapsed_time = time.time() - start_time
        if path and drawResults:
            draw_results(path, V, E, environment, bounds, object_radius, resolution, start_pose, goal_region, elapsed_time)
        return path, V, E

    def fastRRT(self, environment, bounds, start_pose, goal_region, object_radius, steer_distance, num_iterations, resolution=3, runForFullIterations=False, drawResults=False):
        start_time = time.time()
        path, V, E = self.algos.fastRRT(environment, bounds, start_pose, goal_region, object_radius, steer_distance, num_iterations, resolution, runForFullIterations)
        elapsed_time = time.time() - start_time
        if path and drawResults:
            draw_results(path, V, E, environment, bounds, object_radius, resolution, start_pose, goal_region, elapsed_time)
        return path, V, E

if __name__=="__main__":
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

	solver = Solver()
	path, v, e = solver.RRT(environment, bounds, start_pose, goal_region, object_radius, steer_distance, num_iterations, resolution, runForFullIterations, drawResults)
	print(path)
	path, v, e = solver.fastRRT(environment, bounds, start_pose, goal_region, object_radius, steer_distance, num_iterations, resolution, runForFullIterations, drawResults)
	print(path)
