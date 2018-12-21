import numpy as np
import matplotlib.pyplot as plt
from grid import create_grid_and_edges
import numpy.linalg as LA
from graph_search import a_star
import networkx as nx

nx.__version__

plt.rcParams['figure.figsize'] = 12, 12


def heuristic(n1, n2):
    h = LA.norm(np.array(n2) - np.array(n1))
    return h


# 1. Find the closest point in the graph to our current location,
# same thing for the goal location.
def closest_point(graph, current_point):
    d_min = 1000000
    _closest_point = None
    for point in graph.nodes:
        d = LA.norm(np.array(point) - np.array(current_point))
        if d < d_min:
            _closest_point = point
            d_min = d
    return _closest_point


def find_path(data, grid_start, grid_goal, drone_altitude, safety_distance):
    # This is now the routine using Voronoi
    grid, edges = create_grid_and_edges(data, drone_altitude, safety_distance)

    g = nx.Graph()

    for p1, p2 in edges:
        dist = LA.norm(np.array(p2) - np.array(p1))
        g.add_edge(p1, p2, weight=dist)

    start_closest = closest_point(g, grid_start)
    print(grid_start, start_closest)
    goal_closest = closest_point(g, grid_goal)
    print(grid_goal, goal_closest)

    # 2. Compute the path from start to goal using A* algorithm
    path, cost = a_star(g, heuristic, start_closest, goal_closest)

    # insert actual start and goal positions
    if len(path) > 0:
        path.insert(0, grid_start)
        path.append(grid_goal)

    # convert double coordinates to integers
    _path = []
    for n, e in path:
        _path.append((int(n), int(e)))

    return grid, _path, cost


def draw_path(grid, path):
    # Plot the path
    plt.imshow(grid, origin='lower', cmap='Greys')

    for i in range(len(path) - 1):
        p1 = path[i]
        p2 = path[i + 1]
        plt.plot([p1[1], p2[1]], [p1[0], p2[0]], 'r-')

    plt.plot(path[0][1], path[0][0], 'rx', markersize=12)
    plt.plot(path[-1][1], path[-1][0], 'gx', markersize=12)

    plt.xlabel('EAST', fontsize=20)
    plt.ylabel('NORTH', fontsize=20)
    plt.show()
