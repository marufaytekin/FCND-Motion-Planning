import _thread
import msgpack
from enum import Enum, auto
from udacidrone import Drone
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import *
from planning_utils import create_grid, set_heading, draw_path


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection: object, data, func_find_path, goal, wp=[]) -> object:
        super().__init__(connection)
        self.data = data
        self.goal = goal
        self.find_path = func_find_path
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = wp
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed and len(self.waypoints) == 0:
                    self.plan_path()
                else:
                    self.send_waypoints()
                    self.flight_state = States.PLANNING
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2],
                          self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE

        # DONE: read lat0, lon0 from colliders into floating point values
        filename = 'colliders.csv'
        with open(filename) as f:
            line = f.readline()
            (_, lat0, _, lon0) = line.replace(",", "").split()
        print(lat0, lon0)

        # DONE: set home position to (lon0, lat0, 0)
        self.set_home_position(float(lon0), float(lat0), 0.0)

        # DONE: retrieve current global position
        curr_global_pos = [self._longitude, self._latitude, self._altitude]

        # DONE: convert to current local position using global_to_local()
        current_local_position = global_to_local(curr_global_pos, self.global_home)
        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))

        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(self.data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))

        # Define starting point on the grid (this is just grid center)
        # current local position should be starting point.
        # grid_start = (-north_offset, -east_offset)
        start_north = int(current_local_position[0])
        start_east = int(current_local_position[1])
        print("north_start:", start_north, "easth_start:", start_east)

        # DONE: convert start position to current position rather than map center
        grid_start = ((start_north + -north_offset), (start_east + -east_offset))
        print("grid_start:", grid_start)

        # Set goal as some arbitrary position on the grid
        # grid_goal = (-north_offset + 10, -east_offset + 10)

        # DONE: adapt to set goal as latitude / longitude position and convert
        # goal_north, goal_east, goal_alt = global_to_local(grid_goal, self.global_home)

        (goal_lon, goal_lat, goal_alt) = self.goal

        goal_global_position = [goal_lon, goal_lat, goal_alt]
        goal_local_position = global_to_local(goal_global_position, self.global_home)
        (goal_north, goal_east) = (int(goal_local_position[0]), int(goal_local_position[1]))
        grid_goal = (int(np.ceil(goal_north - north_offset)), int(np.ceil(goal_east - east_offset)))
        print("grid_goal:", grid_goal)

        # Run A* to find a path from start to goal
        # DONE: add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here)
        print('Local Start and Goal: ', grid_start, grid_goal)
        path, path_cost = self.find_path(grid, grid_start, grid_goal)
        print("path length: ", len(path), "path cost: ", path_cost)

        # draw path
        _thread.start_new_thread(draw_path, (grid, path,))

        # Convert path to way points
        waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in path]

        # Add heading commands to waypoints
        waypoints = set_heading(waypoints)

        # Set self.waypoints
        self.waypoints = waypoints

        # DONE: send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        try:
            self.connection.start()
            self.stop_log()
        except ConnectionResetError as err:
            return err, self.waypoints

        # Only required if they do threaded
        # while self.in_mission:
        #    pass
