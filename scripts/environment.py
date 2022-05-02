import yaml
import math
import shapely.geometry as geom
from shapely import affinity
import itertools
import matplotlib.pyplot as plt
from descartes import PolygonPatch


def plot_environment(env, bounds=None, figsize=None):
    if bounds is None and env.bounds:
        minx, miny, maxx, maxy = env.bounds
    elif bounds:
        minx, miny, maxx, maxy = bounds
    else:
        minx, miny, maxx, maxy = (-10,-5,10,5)

    max_width, max_height = 12, 5.5
    if figsize is None:
        width, height = max_width, (maxy-miny)*max_width/(maxx-minx)
        if height > 5:
            width, height = (maxx-minx)*max_height/(maxy-miny), max_height
        figsize = (width, height)
    f = plt.figure(figsize=figsize)
    
    ax = f.add_subplot(111)
    for i, obs in enumerate(env.obstacles):
        patch = PolygonPatch(obs, fc='blue', ec='blue', alpha=0.5, zorder=20)
        ax.add_patch(patch)

    plt.xlim([minx, maxx])
    plt.ylim([miny, maxy])
    ax.set_aspect('equal', adjustable='box')
    return ax

def plot_line(ax, line):
    x, y = line.xy
    ax.plot(x, y, color='gray', linewidth=1, solid_capstyle='round', zorder=1)

def plot_poly(ax, poly, color, alpha=1.0, zorder=1):
    patch = PolygonPatch(poly, fc=color, ec="black", alpha=alpha, zorder=zorder)
    ax.add_patch(patch)

class Environment:
    def __init__(self, yaml_file=None, bounds=None):
        self.yaml_file = yaml_file
        self.environment_loaded = False
        self.obstacles = []
        self.obstacles_map = {}
        self.bounds = bounds
        if not yaml_file is None:
            if self.load_from_yaml_file(yaml_file):
                if bounds is None:
                    self.calculate_scene_dimensions()
                self.environment_loaded = True

    def bounds(self):
        return self.bounds

    def add_obstacles(self, obstacles):
        self.obstacles = self.obstacles + obstacles
        self.calculate_scene_dimensions()

    def calculate_scene_dimensions(self):
        points = []
        for elem in self.obstacles:
            points = points + list(elem.boundary.coords)

        mp = geom.MultiPoint(points)
        self.bounds = mp.bounds

    def load_from_yaml_file(self, yaml_file):
        f = open(yaml_file)
        self.data = yaml.safe_load(f)
        f.close()
        return self.parse_yaml_data(self.data)

    def parse_yaml_data(self, data):
        if 'environment' in data:
            env = data['environment']
            self.parse_yaml_obstacles(env['obstacles'])
            return True
        else:
            return False

    def parse_yaml_obstacles(self, obstacles):
        self.obstacles = []
        self.obstacles_map = {}
        for name, description in obstacles.items():
            if description['shape'] == 'polygon':
                parsed = self.parse_polygon(name, description)
            else:
                raise Exception("not a valid obstacle")
            self.obstacles.append(parsed)
            self.obstacles_map[name] = parsed
        self.expanded_obstacles = [obs.buffer(0.75/2, resolution=2) for obs in self.obstacles]

    def parse_polygon(self, name, description):
        _points = description['corners']
        for points in itertools.permutations(_points):
            polygon = geom.Polygon(points)
            polygon.name = name
            if polygon.is_valid:
                return polygon
    