import argparse
import time
from graph_search import find_path as city_search_find_path
from udacidrone.connection import MavlinkConnection
from motion_planning import *


def find_path(_, grid_start, grid_goal):
    path, path_cost = city_search_find_path(data, grid_start, grid_goal, 5, 3)
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
