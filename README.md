#MapView
######A map viewer and editor for ROS

###Features:

- View, open, and save ROS maps (OccupancyGrid)

- Add nodes and edges to maps

- Automatically generate PRMs on top of a map

- Create and publish 2D pose estimates and 2D nav goals

- Display robot pose and orientation in real-time (from amcl_pose)

- Integrated with [node_traveller](https://github.com/uobirlab/node_traveller) to display live route info and status when performing a tour of a map

###Usage:

Use the following command to run the program:
>rosrun map_view gui.py

###Notes:

- For ROS Groovy, the default Navigation stack should be replaced by the [catkinized version](http://github.com/jonbinney/navigation/tree/catkinized-groovy-devel). Otherwise MapView will not be able to access move_base and the program will not run.

- The HELP file contains instructions for using the map editing tools, as well as a list of useful hotkeys

======
_Questions? jberthiaume4(at)gmail.com_
