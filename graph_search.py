import numpy as np
from queue import PriorityQueue
from planning_utils import create_grid_and_edges
import numpy.linalg as LA
import networkx as nx


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
    _, edges = create_grid_and_edges(data, drone_altitude, safety_distance)

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

    return _path, cost


def a_star(graph, h, start, goal):
    path = []
    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False

    while not queue.empty():
        item = queue.get()
        current_node = item[1]
        if current_node == start:
            current_cost = 0.0
        else:
            current_cost = branch[current_node][0]

        if current_node == goal:
            print('Found a path.')
            found = True
            break
        else:
            for next_node in graph[current_node]:
                # get the tuple representation
                cost = graph.edges[current_node, next_node]['weight']

                branch_cost = current_cost + cost
                queue_cost = branch_cost + h(next_node, goal)

                if next_node not in visited:
                    visited.add(next_node)
                    branch[next_node] = (branch_cost, current_node)
                    queue.put((queue_cost, next_node))

    if found:
        # retrace steps
        n = goal
        path_cost = branch[n][0]
        path.append(goal)
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************')
    return path[::-1], path_cost
