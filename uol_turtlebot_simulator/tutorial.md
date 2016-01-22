# The Turtlebot Simulator - A Brief Tutorial

## Empty world and keyboard teleop
The following has largely been taken from [the first official tutorial](http://wiki.ros.org/turtlebot_simulator/Tutorials/hydro/Explore%20the%20Gazebo%20world).

### Start-up
_For each of the following commands, open a new terminal and then execute it._

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

### Adding things to the simulation

In order to see something with our robot's sensors, we want to add some obstacles:

**Simple geometric objects**
* In the top left menu bar you can find a cube, sphere, and cylinder object.
* Click on one of them and then click in the simulation environment where you want to place it.

**More advanced objects**
* On the left hand side of the Simulator window, click on the `Insert` tab
* Expand the node that starts with `http://gazebosim.org/...`. This might take a few moments to download all the models form the database.
* Click on one of the object names and place it the simulator environment.

_Have a play with the environment and create some interesting arrangements._

### Visualising what the robot sees

* In a new terminal, run rviz: `roslaunch turtlebot_rviz_launchers view_robot.launch`
* Show the image:
 * Tick the box next to `image`
 * Expand the `image` node and select `/camera/rgb/image_raw` as the `Topic`
* Enable `Laser Scan`
* Show the registered pointcloud:
 * Tick the box next to `Registered PointCloud`
 * Expand the `Regesitered PointCloud` node and select `/camera/depth/points` as the `Topic`
* Adding objects to see something:
 * If you haven't already done so, in Gazebo, select a cube, speher, or cylinder and drop it with a mouse click infront of the robot
 * Go back to rviz and observe what you can see

_Drive the robot through your environment using the teleop from above and see how it perceives the world in rviz._
 
## Comp Lab C and rviz

_If you still have the simulator running from above example, close everything you started by closing the GUI window and/or pressing `Ctrl+C` in the terminal you started it in._

### Start-up
_For each of the following commands, open a new terminal and then execute it._

* Comp Lab C and rviz: 
 * Simulator: `roslaunch uol_turtlebot_simulator labc.launch`
 * Rviz: `roslaunch uol_turtlebot_simulator view_navigation.launch`
* Trouble shooting:
 * If, after starting the simulator, you get output like this:

 ```
Warning [gazebo.cc:215] Waited 1seconds for namespaces.
Error [gazebo.cc:220] Waited 11 seconds for namespaces. Giving up.
Error [Node.cc:90] No namespace found
 ```

 on the terminal, it because Gazbo is trying to download missing models. Please give it a minute or two and if nothing happens, please ask a demonstrator.
 * If the simulation does not want to start properly, try launching these two files instead, each in a separate terminal:
 
 ```
roslaunch uol_turtlebot_simulator gazebo.launch
roslaunch uol_turtlebot_simulator robots.launch
 ```

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
