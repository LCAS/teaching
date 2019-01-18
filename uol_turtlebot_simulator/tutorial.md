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
 
