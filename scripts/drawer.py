from shapely.geometry import Point, Polygon, LineString, box
from environment import Environment, plot_environment, plot_line, plot_poly
from math import sqrt
import matplotlib.pyplot as plt

def draw_results(algo_name, path, V, E, env, bounds, object_radius, resolution, start_pose, goal_region, elapsed_time):
    graph_size = len(V)
    path_size = len(path)
    # Calculate path length
    path_length = 0.0
    for i in range(len(path)-1):
        path_length += euclidian_dist(path[i], path[i+1])

    # Create title with descriptive information based on environment, path length, and elapsed_time
    title = algo_name + " Obstacles. Path Size: " + str(path_size) + "\n Path Length: " + str(path_length) + "\n Runtime(s)= " + str(elapsed_time)

    # Plot environment
    env_plot = plot_environment(env, bounds)
    # Add title
    env_plot.set_title(title)
    # Plot goal
    plot_poly(env_plot, goal_region, 'green')
    # Plot start
    buffered_start_vertex = Point(start_pose).buffer(object_radius, resolution)
    plot_poly(env_plot, buffered_start_vertex, 'red')

    # Plot Edges explored by ploting lines between each edge
    for edge in E:
        line = LineString([edge[0], edge[1]])
        plot_line(env_plot, line)

    # Plot path
    plot_path(env_plot, path, object_radius)


def euclidian_dist(point1, point2):
    return sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)


def plot_path(env_plot, path, object_radius):
    # Plots path by taking an enviroment plot and ploting in red the edges that form part of the path
    line = LineString(path)
    x, y = line.xy
    env_plot.plot(x, y, color='red', linewidth=3, solid_capstyle='round', zorder=1)
    # plt.show()