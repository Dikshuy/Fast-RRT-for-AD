from shapely.geometry import Point, LineString
import numpy as np
import random 
import math 

class algo():
	def initialize(self, environment, bounds, start_pose, goal_region, object_radius, steer_distance, num_iterations, resolution, runForFullIterations):
		self.env = environment
		self.obstacles = environment.obstacles
		self.bounds = bounds
		self.minx, self.miny, self.maxx, self.maxy = bounds
		self.start_pose = start_pose
		self.goal_region = goal_region
		self.obj_radius = object_radius
		self.N = num_iterations
		self.resolution = resolution
		self.steer_distance = steer_distance
		self.V = set()
		self.E = set()
		self.child_to_parent_dict = dict() #key = child, value = parent
		self.runForFullIterations = runForFullIterations
		self.goal_pose = (goal_region.centroid.coords[0])

	def update(curr_pos, new_pos, client):
	    wp = MoveXYGoal()
	    wp.pose_dest.x = new_pos[0]
	    wp.pose_dest.y = new_pos[1]
	    wp.pose_dest.theta = math.atan2((new_pos[1]-curr_pos[1]),(new_pos[0]-curr_pos[0]))
	    #send waypoint to turtlebot3 via move_xy server
	    client.send_goal(wp)
	    client.wait_for_result()

	    #getting updated robot location
	    result = client.get_result()

	    #write to output file (replacing the part below)
	    print(result.pose_final.x, result.pose_final.y, result.pose_final.theta)
	    return [result.pose_final.x, result.pose_final.y]

	def RRT(self, environment, bounds, start_pose, goal_region, object_radius, steer_distance, num_iterations, resolution, runForFullIterations):
		self.env = environment

		self.initialize(environment, bounds, start_pose, goal_region, object_radius, steer_distance, num_iterations, resolution, runForFullIterations)

		x0, y0 = start_pose
		x1, y1 = goal_region.centroid.coords[0]    # goal is considered as the centroid of goal polygon
		start = (x0, y0)
		goal = (x1, y1)

		if start == goal:                               # same starting and end pos
			path = [start, goal]
			self.V.union([start, goal])
			self.E.union([(start, goal)])

		else:
			path = []
			path_length = float('inf')
			tree_size = 0
			path_size = 0
			self.V.add(self.start_pose)
			goal_centroid = self.get_centroid(self.goal_region)

			for i in range(self.N):
				if(random.random()>=0.4): 
					random_point = goal_centroid
				else:
					random_point = self.get_collision_free_random_point()

				nearest_point = self.find_nearest_point(random_point)
				new_point = self.steer(nearest_point, random_point)
				if self.isEdgeCollisionFree(nearest_point, new_point):
					self.V.add(new_point)
					self.E.add((nearest_point, new_point))
					self.child_to_parent_dict[new_point] = nearest_point
					if self.isAtGoalRegion(new_point):
						if not self.runForFullIterations: # If not running for full iterations, terminate as soon as a path is found.
							path, tree_size, path_size, path_length = self.find_path(self.start_pose, new_point)
							break
						else: # If running for full iterations, we return the shortest path found.
							tmp_path, tmp_tree_size, tmp_path_size, tmp_path_length = self.find_path(self.start_pose, new_point)
							if tmp_path_length < path_length:
								path_length = tmp_path_length
								path = tmp_path
								tree_size = tmp_tree_size
								path_size = tmp_path_size

			return path, self.V, self.E

	def RRT_star(self, environment, bounds, start_pose, goal_region, object_radius, steer_distance, num_iterations, resolution, runForFullIterations):
		self.env = environment
		rospy.init_node('test', anonymous= True)
	    client = actionlib.SimpleActionClient('move_xy', MoveXYAction)
	    client.wait_for_server()
		self.initialize(environment, bounds, start_pose, goal_region, object_radius, steer_distance, num_iterations, resolution, runForFullIterations)

		x0, y0 = start_pose
		x1, y1 = goal_region.centroid.coords[0]    # goal is considered as the centroid of goal polygon
		start = (x0, y0)
		goal = (x1, y1)

		if start == goal:                               # same starting and end pos
			path = [start, goal]
			self.V.union([start, goal])
			self.E.union([(start, goal)])
		elif self.isEdgeCollisionFree(start, goal):		# rush towards the goal if direct path available
			path = [start, goal]
			self.V.union([start, goal])
			self.E.union([(start, goal)])
		else:
			path = []
			path_length = float('inf')
			tree_size = 0
			path_size = 0
			self.V.add(self.start_pose)
			goal_centroid = self.get_centroid(self.goal_region)

			for i in range(self.N):
				if(random.random()>=1.95):				# choose a random number and rush towards the goal if it's greater than certain lambda value
					random_point = goal_centroid
				else:
					random_point = self.get_collision_free_random_point()

				nearest_point = self.find_nearest_point(random_point)
				new_point = self.steer(nearest_point, random_point)
				update(nearest_point, new_point, client)

				if self.isEdgeCollisionFree(nearest_point, new_point):
					nearest_set = self.find_nearest_set(new_point)
					min_point = self.find_min_point(nearest_set, nearest_point, new_point)
					self.V.add(new_point)
					self.E.add((min_point, new_point))     				
					self.child_to_parent_dict[new_point] = min_point 		# set the parent
					self.check(nearest_set, min_point, new_point)
					if self.isAtGoalRegion(new_point):
						if not self.runForFullIterations:
							path, tree_size, path_size, path_length = self.find_path(self.start_pose, new_point)
							break
						else:
							tmp_path, tmp_tree_size, tmp_path_size, tmp_path_length = self.find_path(self.start_pose, new_point)
							if tmp_path_length < path_length:
								path_length = tmp_path_length
								path = tmp_path
								tree_size = tmp_tree_size
								path_size = tmp_path_size

			return path, self.V, self.E

	def dist(self, point1, point2):
		return math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)

	def ball_radii(self):
		unit_ball_volume = math.pi
		n = len(self.V)
		dimensions = 2.0
		gamma = (2**dimensions)*(1.0 + 1.0/dimensions) * (self.maxx - self.minx) * (self.maxy - self.miny)
		ball_radius = min(((gamma/unit_ball_volume) * math.log(n) / n)**(1.0/dimensions), self.steer_distance)
		return ball_radius

	def isOutOfBounds(self, point):
		if((point[0] - self.obj_radius) < self.minx):
			return True
		if((point[1] - self.obj_radius) < self.miny):
			return True
		if((point[0] + self.obj_radius) > self.maxx):
			return True
		if((point[1] + self.obj_radius) > self.maxy):
			return True
		return False

	def isEdgeCollisionFree(self, point1, point2):
		if self.isOutOfBounds(point2):
			return False
		line = LineString([point1, point2])
		expanded_line = line.buffer(self.obj_radius, self.resolution)
		for obstacle in self.obstacles:
			if expanded_line.intersects(obstacle):
				return False
		return True

	def find_min_point(self, nearest_set, nearest_point, new_point):
		min_point = nearest_point
		min_cost = self.cost(nearest_point) + self.dist(nearest_point, new_point)
		for vertex in nearest_set:
			if self.isEdgeCollisionFree(vertex, new_point):
				temp_cost = self.cost(vertex) + self.dist(vertex, new_point)
				if temp_cost < min_cost:
					min_point = vertex
					min_cost = temp_cost
		return min_point

	def find_nearest_set(self, new_point):
		points = set()
		ball_radius = self.ball_radii()
		for vertex in self.V:
			euc_dist = self.dist(new_point, vertex)
			if euc_dist < ball_radius:
				points.add(vertex)
		return points

	def cost(self, vertex):
		path, tree_size, path_size, path_length = self.find_path(self.start_pose, vertex)
		return path_length

	def get_random_point(self):
		x = self.minx + random.random() * (self.maxx - self.minx)
		y = self.miny + random.random() * (self.maxy - self.miny)
		return (x, y)

	def isPointCollisionFree(self, point):
		for obstacle in self.obstacles:
			if obstacle.contains(point):
				return False
		return True

	def get_collision_free_random_point(self):
		while True:
			point = self.get_random_point()
			next_point = Point(point).buffer(self.obj_radius, self.resolution)
			if self.isPointCollisionFree(next_point):
				return point

	def find_nearest_point(self, random_point):
		closest_point = None
		min_dist = float('inf')
		for vertex in self.V:
			euc_dist = self.dist(random_point, vertex)
			if euc_dist < min_dist:
				min_dist = euc_dist
				closest_point = vertex
		return closest_point

	def steer(self, from_point, to_point):
		fromPoint_buffered = Point(from_point).buffer(self.obj_radius, self.resolution)
		toPoint_buffered = Point(to_point).buffer(self.obj_radius, self.resolution)
		if fromPoint_buffered.distance(toPoint_buffered) < self.steer_distance:
			return to_point
		else:
			from_x, from_y = from_point
			to_x, to_y = to_point
			theta = math.atan2(to_y - from_y, to_x- from_x)
			new_point = (from_x + self.steer_distance * math.cos(theta), from_y + self.steer_distance * math.sin(theta))
			return new_point

	def isAtGoalRegion(self, point):
		buffered_point = Point(point).buffer(self.obj_radius, self.resolution)
		intersection = buffered_point.intersection(self.goal_region)
		inGoal = intersection.area / buffered_point.area
		return inGoal >= 0.5

	def find_path(self, start_point, end_point):
		path = [end_point]
		tree_size, path_size, path_length = len(self.V), 1, 0
		current_node = end_point
		previous_node = None
		target_node = start_point
		while current_node != target_node:
			parent = self.child_to_parent_dict[current_node]
			path.append(parent)
			previous_node = current_node
			current_node = parent
			path_length += self.dist(current_node, previous_node)
			path_size += 1
		path.reverse()
		return path, tree_size, path_size, path_length

	def get_centroid(self, region):
		centroid = region.centroid.wkt
		filtered_vals = centroid[centroid.find("(")+1:centroid.find(")")]
		filtered_x = filtered_vals[0:filtered_vals.find(" ")]
		filtered_y = filtered_vals[filtered_vals.find(" ") + 1: -1]
		(x,y) = (float(filtered_x), float(filtered_y))
		return (x,y)

	def check(self, nearest_set, min_point, new_point):
		# discards edges in the nearest_set that lead to a longer path than going through the new_point first
		for vertex in nearest_set - set([min_point]):
			if self.isEdgeCollisionFree(vertex, new_point):
				if self.cost(vertex) > self.cost(new_point) + self.dist(vertex, new_point):
					parent_point = self.child_to_parent_dict[vertex]
					self.E.discard((parent_point, vertex))
					self.E.discard((vertex, parent_point))
					self.E.add((new_point, vertex))
					self.child_to_parent_dict[vertex] = new_point