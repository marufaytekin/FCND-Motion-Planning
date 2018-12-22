import argparse
import time
from udacidrone.connection import MavlinkConnection
from motion_planning import *
from grid_search import a_star, heuristic
from planning_utils import prune_path


def find_path(grid, grid_start, grid_goal):
    path, path_cost = a_star(grid, heuristic, grid_start, grid_goal)
    path = prune_path(path)
    return path, path_cost


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    # Read in obstacle map
    data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)

    #goal = (-122.397745, 37.793837, 0)
    goal = (-122.399563, 37.795926, 0)

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), threaded=False, send_rate=5, timeout=600)
    wp = []
    drone = MotionPlanning(conn, data, find_path, goal, wp)

    (err, wp) = drone.start()
    if err:
        print(err)
        print(wp)
        drone.stop()
        time.sleep(10)
        print("restarting connection...")
        conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), threaded=False, send_rate=5, timeout=600)
        drone = MotionPlanning(conn, data, find_path, goal, wp)
        time.sleep(1)
        drone.start()
