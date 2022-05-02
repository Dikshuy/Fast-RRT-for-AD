from shapely.geometry import Point, Polygon, LineString, box
from environment import plot_environment, plot_line, plot_poly
import math
import matplotlib.pyplot as plt

def results(path, V, E, env, bounds, object_radius, resolution, start_pose, goal_region, elapsed_time):
    graph_size = len(V)
    path_size = len(path)
    path_length = 0.0
    for i in range(len(path)-1):
        path_length += dist(path[i], path[i+1])

    title = "Path Length: " + str(round(path_length,3)) + "\n Runtime(s)= " + str(round(elapsed_time,3))

    env_plot = plot_environment(env, bounds)
    env_plot.set_title(title)
    plot_poly(env_plot, goal_region, 'green')
    buffered_start_vertex = Point(start_pose).buffer(object_radius, resolution)
    plot_poly(env_plot, buffered_start_vertex, 'red')

    for edge in E:
        line = LineString([edge[0], edge[1]])
        plot_line(env_plot, line)

    line = LineString(path)
    x, y = line.xy
    env_plot.plot(x, y, color='red', linewidth=3, solid_capstyle='round', zorder=1)
    plt.show()

def dist(pt1, pt2):
    return math.sqrt((pt2[0] - pt1[0])**2 + (pt2[1] - pt1[1])**2)