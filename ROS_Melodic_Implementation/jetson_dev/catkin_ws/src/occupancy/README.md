# Occupancy Mapping

- This package uses the cameras to create a 2D occupancy map. The map can be saved and loaded later.
- The occupancy map is published by line 100 in occupancy_node.cpp:
```
ros::Publisher pub = n.advertise<nav_msgs::OccupancyGrid>("occupancy", 1);
```
- To see more about this message:  http://docs.ros.org/jade/api/nav_msgs/html/msg/OccupancyGrid.html