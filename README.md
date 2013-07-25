map_view
========

### Features:

- View, open, and save ROS maps (OccupancyGrid)

- Add nodes and edges to maps

- Automatically generate PRM graphs on top of maps

- Create and publish 2D pose estimates and 2D nav goals

- Display robot pose and orientation in real-time (amcl_pose)

### Notes:

For ROS Groovy, you need to replace your Navigation stack with the one located at http://github.com/jonbinney/navigation/tree/catkinized-groovy-devel. Otherwise map_view will not be able to access move_base.

