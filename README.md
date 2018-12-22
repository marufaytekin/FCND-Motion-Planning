# 3D Motion Planning


## Grid Based Implementation

`motion_planning_grid_based.py` implements grid based version of path planning algorithm. 

### Goal 1

I manually flew the drone and found the first goal. The first goal point is as follows:
```python
goal1 = (-122.397745, 37.793837, 0)
(goal_lon, goal_lat, goal_alt) = goal1
```

It creates an initial path of 151 way points with cost of 160.76955262170057. As the next step it prunes 
the path down to 42 way points. The pruned path on the map is shown in the following figure:

![path 2](path1.png?raw=true "Fig 1: Path for goal1")


You can see the flight video here:

[![3D Motion Planning Demo Flight - Goal1](http://img.youtube.com/vi/kas6BiCqDcc/0.jpg)](http://www.youtube.com/watch?v=kas6BiCqDcc)


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

### Goal 2

I manually flew the drone and found the second goal. Second goal point is as follows:
```python
goal2 = (-122.399563, 37.795926, 0)
(goal_lon, goal_lat) = goal2
```

It creates a path with 454 waypoints then prunes the path down to 47. The pruned path on the map is shown in the following figure:
 
![path 2](path2.png?raw=true "Fig 2: Path for goal2")

You can see the flight video here:

[![3D Motion Planning Demo Flight - Goal2](http://img.youtube.com/vi/CDBEfLeUeEg/0.jpg)](http://www.youtube.com/watch?v=CDBEfLeUeEg)



## Graph Based Implementation

`motion_planning_graph.py` implements graph version of path planning algorithm based on Voronoi diagram.
It finds following path:
 
![path 3](path3.png?raw=true "Fig 3: Path for goal2")

You can see the flight video here:

[![3D Graph Based Motion Planning Demo Flight - Goal2](http://img.youtube.com/vi/mzxtqZ8l4f8/0.jpg)](http://www.youtube.com/watch?v=mzxtqZ8l4f8)
