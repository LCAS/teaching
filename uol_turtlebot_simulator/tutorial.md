# The Turtlebot Simulator - A Brief Tutorial

## Empty world and keyboard teleop
The following has largely been taken from [the first official tutorial](http://wiki.ros.org/turtlebot_simulator/Tutorials/hydro/Explore%20the%20Gazebo%20world).

### Start-up
* Empty world with keyboard teleop: 
 * Simulator: `roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(rospack find turtlebot_gazebo)/worlds/empty.world`
 * Keyboard teleop: `roslaunch kobuki_keyop keyop.launch`

### Control

```
Reading from keyboard
---------------------------
Forward/back arrows : linear velocity incr/decr.
Right/left arrows : angular velocity incr/decr.
Spacebar : reset linear/angular velocities.
d : disable motors.
e : enable motors.
q : quit.
```

### Visualising what the robot sees

* Run rviz: `roslaunch turtlebot_rviz_launchers view_robot.launch`
* Show the image:
 * Tick the box next to `image`
 * Expand the `image` node and select `/camera/rgb/image_raw` as the `Topic`
* Enable `Laser Scan`
* Show the registered pointcloud:
 * Tick the box next to `Registered PointCloud`
 * Expand the `Regesitered PointCloud` node and select `/camera/depth/points` as the `Topic`
* Adding objects to see something:
 * In Gazebo, select a cube, speher, or cylinder and drop it with a mouse click infront of the robot
 * Go back to rviz and observe what you can see
 
## Comp Lab C and rviz

### Start-up
* Comp Lab C and rviz: 
 * Simulator: `roslaunch uol_turtlebot_simulator labc.launch`
 * Rviz: `roslaunch uol_turtlebot_simulator view_navigation.launch`

### Control
* Use the 2 "2D Nav Goal" buttons in the top panel to send the robots around.
 * Click the button
 * Click and hold the left mouse button somewhere on the map. An arrow should appear.
 * While holding the left mouse button you can oreint the arrow which determines the final orientation of the robot when arriving at the designated target.
 * Let the left mouse button go to send the goal to the robot.
* Use the keyboard teleop
 * Run the teleop in a new terminal specifing which robot you want to control: `roslaunch uol_turtlebot_simulator keyop.launch robot_name:=<robot_name>`. Where `<robot_name>` can be `turtlebot_1` or `turtlebot_2`.
 * The actual control mechanism is printed on the screen:
 ```
Reading from keyboard
---------------------------
Forward/back arrows : linear velocity incr/decr.
Right/left arrows : angular velocity incr/decr.
Spacebar : reset linear/angular velocities.
d : disable motors.
e : enable motors.
q : quit.
 ```

### Visualisations
In the left panel you can see three groups of visualisations:

* Global: This turns on the map and the robot models. Untick to turn this off.
* Turtlebot_1: This turns on the global and local costmap, the global and local path, the image, the amcl point cloud, and the laser scan of turtlebot 1.
* Turtlebot_2: This turns on the global and local costmap, the global and local path, the image, the amcl point cloud, and the laser scan of turtlebot 2.

All of these groups can be expanded to turn on/off the seperate visualisations.
