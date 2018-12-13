# Project: 3D Motion Planning

## Explain the Starter Code

### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`
 
The way points in `backyard_flyer_solution.py` are manually set in `calculate_box()` function. On the other hand 
`motion_planning.py` uses a-star search algorthm to find the shortest path to the goal. The main features of 
`motion_planning.py` are as follows:

* The state machine implemented on `motion_planning.py` adds an extra state called `PLANNING` after after `ARMING` and before 
`TAKEOFF` state.
* After the completion of `ARMING` state, `plan_path()` function is called to find a path to target and the drone enters in 
`PLANNING` state. `path_plan()` function is working as follows:
    - Sets the flight state to `PLANNING`. (115)
    - Sets the target altitude and safety distance. (117-118)
    - Loads the data file and creates the grid by `executing create_grid()` function in `planning_utils.py`.(133-136)
    - Sets the `grid_start` and `grid_goal` points. (139, 143)
    - Runs the a-star search in `planning_utils.py` file. (150). I would like to highlight two points of a-star search as 
follows. a-star uses four actions as the valid actions. The valid actions are `WEST`, `EAST`, `SOUTH` and `NORTH`. 
Each valid action costs the same. The second point is that a-star uses a `heuristic` function which calculates the cost
 as the distance to the goal.
    - It returns a path with the cost to the goal.
    - The path is converted way points. (155)
    - It sets the waypoints and calls `send_waypoints()` function. (157-159)
* Once `plan_path()` function completes execution `state_callback` function initiates `takeoff_transition` at line 69.
* Once `take_off_transition` completes, it triggers `waypoint_transition()` in `local_position_callback()` function. (45)
* `Waypoint_transition` function sets the flight state to `WAYPOINT` and stes the target position to the first waypoint. 
Then it executes `cmd_position` command to send the drone to the target waypoint. (86-90)
* `local_position_callback` function calls `waypoint_transition` for all way points in `self.waypoints` list. (49)
* Once all way points are reached `local_position_callback` sets the flight state to `LANDING` and calls 
`landing_transition` function. (51)

## Implementing Your Path Planning Algorithm

### 1. Set your global home position

This is implemented at line 129.

### 2. Set your current local position

Implemented at line 136.

### 3. Set grid start position from local position

Implemented at lines 150-156.

### 4. Set grid goal position from geodetic coords

Implemented at lines 164-168.

### 5. Modify A* to include diagonal motion

Added diagonal motions to `a_star()` function in `planning_utils.py`. Diagonal motions are called  NORTH_WEST, NORTH_EAST, 
SOUTH_WEST, SOUTH_EAST and they cost `sqrt(2)` 


### 6. Cull waypoints 

I implemented `prune_path()` function in `planning_utils.py`. It uses collinearity test to prune the path of unnecessary waypoints.
`prune_path` calls `collinearity_check` function and passes three connected points to it. Then `collinearity_check` function
 calculates the determinant of the passed points. Determinant gives the area of these points when they are connected. 
 If the area is 0 that means these points are on the same line. We use a small number epsilon to tolerate small differences. 
 This way we can consider almost straight lines as one straight line.

## 7. Extra Step: Add heading commands to waypoints

This is implemente din line 197. `set_heading` function from `planning_utils` is called to calculate a unique heading 
for each way point. It calculates the heading based on relative position to the current position.
 

## Executing the flight

### 1. Does it work?
Yes, it works!

## Goal 1

I manually flew the drone and found the first goal. The first goal point is as follows:
```python
goal1 = (-122.397745, 37.793837, 0)
(goal_lon, goal_lat, goal_alt) = goal1
```

It creates an initial path of 151 way points with cost of 160.76955262170057. As the next step it prunes 
the path down to 42 way points. The pruned path on the map is shown in Fig 1.
![path 2](path1.png?raw=true "Fig 1: Path for goal1")


You can see the flight video here:

[![Goal 1 flight](http://img.youtube.com/vi/LlIOmUREkAo/0.jpg)](http://www.youtube.com/watch?v=LlIOmUREkAo)


The execution log is as follows:
```Searching for a path ...
37.792480 -122.397450
global home [-122.39745   37.79248    0.     ], position [-1.22397450e+02  3.77924795e+01 -6.80000000e-02], local position [-0.04889394  0.0040308   0.06843797]
North offset = -316, east offset = -445
north_start: 0 easth_start: 0
grid_start: (316, 445)
grid_goal: (466, 419)
Local Start and Goal:  (316, 445) (466, 419)
Found a path.
path length:  151 path cost:  160.76955262170057
pruning the path...
pruned path length:  42
the first 10 waypoints:  [(316, 445), (384, 445), (385, 444), (412, 444), (413, 443), (414, 443), (415, 442), (416, 442), (417, 441), (418, 441)]
Sending waypoints to simulator ...
takeoff transition
```

## Goal 2

I manually flew the drone and found the second goal. Second goal point is as follows:
```python
goal2 = (-122.399563, 37.795926, 0)
(goal_lon, goal_lat) = goal2
```

It creates a path with 454 waypoints then prunes the path down to 47. The path on the map is shown in Fig 2.
![path 2](path2.png?raw=true "Fig 2: Path for goal2")

When I send the waypoints to simulator it throws an exception as follows:

```
Searching for a path ...
global home [-122.3974533   37.7924804    0.       ], position [-122.3974534   37.7924795    0.257    ], local position [-0.09947606 -0.01336324 -0.25823599]
North offset = -316, east offset = -445
north_start: 0 easth_start: 0
grid_start: (316, 445)
grid_goal: (697, 257)
Found a path.
path length:  454 path cost:  510.161471607488
pruning the path...
pruned path length:  47
the first 10 waypoints: [(316, 445), (340, 421), (345, 426), (395, 426), (460, 361), (454, 355), (454, 295), (455, 294), (505, 294), (515, 284)]
Sending waypoints to simulator ...
Traceback (most recent call last):
  File "/Users/maruf.aytekin/miniconda2/envs/fcnd/lib/python3.6/site-packages/udacidrone/connection/connection.py", line 88, in notify_message_listeners
    fn(name, msg)
  File "/Users/maruf.aytekin/miniconda2/envs/fcnd/lib/python3.6/site-packages/udacidrone/drone.py", line 117, in on_message_receive
    if (((msg.time - self._message_time) > 0.0)):
AttributeError: 'int' object has no attribute 'time'
```
I tried to find different goals but ran into this error each time. 
