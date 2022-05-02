from shapely.geometry import Point, Polygon, LineString, box
from algos import algo
from plot import results
from environment import Environment
import time
import yaml

class Solver():
    def __init__(self):
        self.algos = algo()

    def RRT(self, environment, bounds, start, goal_region, obj_radius, steer_distance, iterations, resolution=3, runForFullIterations=False):
        start_time = time.time()
        path, V, E = self.algos.RRT(environment, bounds, start, goal_region, obj_radius, steer_distance, iterations, resolution, runForFullIterations)
        elapsed_time = time.time() - start_time
        if path:
            results(path, V, E, environment, bounds, obj_radius, resolution, start, goal_region, elapsed_time)
        return path, V, E

    def fastRRT(self, environment, bounds, start, goal_region, obj_radius, steer_distance, iterations, resolution=3, runForFullIterations=False):
        start_time = time.time()
        path, V, E = self.algos.fastRRT(environment, bounds, start, goal_region, obj_radius, steer_distance, iterations, resolution, runForFullIterations)
        elapsed_time = time.time() - start_time
        if path:
            results(path, V, E, environment, bounds, obj_radius, resolution, start, goal_region, elapsed_time)
        return path, V, E

if __name__=="__main__":
	environment = Environment('obs.yaml')
	bounds = (-2, -3, 12, 8)
	start = (0, 0)
	goal_region = Polygon([(10,5), (10,6), (11,6), (11,5)])
	obj_radius = 0.3
	steer_distance = 0.3
	iterations = 10000
	resolution = 3
	drawResults = True
	runForFullIterations = False

	solver = Solver()
	# path, v, e = solver.RRT(environment, bounds, start, goal_region, obj_radius, steer_distance, iterations, resolution, runForFullIterations)
	# print(path)
	path, v, e = solver.fastRRT(environment, bounds, start, goal_region, obj_radius, steer_distance, iterations, resolution, runForFullIterations)
	print(path)
