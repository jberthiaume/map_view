#MapView Help

##_Tools_

####Map Tools
__Node Creation Tool__: Left-clicking on the map creates a node. Left-clicking on an existing node or edge selects/deselects it.

__Edge Creation Tool__: Left-clicking on an existing node starts an edge from that node. Moving the cursor to another node and left-clicking that node creates between the two nodes. Left-clicking in open space (not on an existing node) creates a new node at that location.

__Box Selection Tool__: Left-clicking and dragging draws a box on the screen. All nodes and edges within the box are selected.

__2D Pose Estimate__: Left-clicking gives the position, then dragging the arrow gives the orientation. Releasing the cursor publishes the pose estimate to _/initialpose_. Right-clicking cancels the operation without publishing.

__2D Nav Goal__: Left-clicking gives the position, then dragging the arrow gives the orientation. Releasing the cursor publishes the nav goal to _/move_base/goal_. Right-clicking cancels the operation without publishing.

======
####Action Buttons
__Zoom to Fit__: Zooms such that all known areas of the map are on the screen

__Clear__: If a route is currently being displayed, this erases route indicators and edge status indicators from the map. Otherwise, this erases all graph data (nodes and edges).

##_Hotkeys_

####File Operations
__Ctrl+O__: Open a map file

__Ctrl+S__: Save map

__Ctrl+W__: Close current map

__Ctrl+Q__: Exit MapView

======
####Selection
__Ctrl+A__: Select all

__Ctrl+D__: Deselect all

__Ctrl+Shift+E__: Select all edges

__Ctrl+Shift+N__: Select all nodes

======
####Map Actions
__Ctrl+E__: Create edges between the selected nodes

__Ctrl+K__: Automatically connect the selected nodes to their closest neighbours

__Ctrl+R__: Create a node at the robot's location

__Delete__: Delete all selected nodes and edges

__Arrow Keys__: Move the selected nodes

======
####Settings
__Alt+B__: Toggle obstacle display on/off

__Alt+C__: Toggle console output on/off

__Alt+K__: Toggle automatic edge creation on/off

======

_Questions? jberthiaume4(at)gmail.com_
