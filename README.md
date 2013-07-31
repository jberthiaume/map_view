map_view
========

###Features:

- View, open, and save ROS maps (OccupancyGrid)

- Add nodes and edges to maps

- Automatically generate PRM graphs on top of maps

- Create and publish 2D pose estimates and 2D nav goals

- Display robot pose and orientation in real-time (amcl_pose)

###Usage:

Use the following command to run the program:
>rosrun map_view gui.py

###Notes:

For ROS Groovy, the default Navigation stack should be replaced by the one located at http://github.com/jonbinney/navigation/tree/catkinized-groovy-devel. Otherwise map_view will not be able to access move_base and the program will not run.

