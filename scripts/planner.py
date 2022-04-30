from fastRRT import FastRRT
from drawer import draw_results
import time

class SamplingBasedPathPlanner():
    def __init__(self):
        """
        The planner contains two objects. One for planning using RRT algorithms and another for using a PRM planner.
        """
        self.RRTFamilySolver = FastRRT()

    def RRTStar(self, environment, bounds, start_pose, goal_region, object_radius, steer_distance, num_iterations, resolution=3, runForFullIterations=False, drawResults=False):
        start_time = time.time()
        path, V, E = self.RRTFamilySolver.path(environment, bounds, start_pose, goal_region, object_radius, steer_distance, num_iterations, resolution, runForFullIterations, RRT_Flavour="RRT*")
        elapsed_time = time.time() - start_time
        if path and drawResults:
            draw_results("RRT*", path, V, E, environment, bounds, object_radius, resolution, start_pose, goal_region, elapsed_time)
        return path, V, E