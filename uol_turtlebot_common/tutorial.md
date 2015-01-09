# The Turtlebot Simulator - A Brief Tutorial

## Empty world and keyboard teleop
The following has largely been taken from (the first official tutorial)[http://wiki.ros.org/turtlebot_simulator/Tutorials/hydro/Explore%20the%20Gazebo%20world].

### Installation
Ubuntu packages:
* Turtlebot specific tools: `sudo apt-get install ros-hydro-turtlebot-apps ros-hydro-turtlebot-rviz-launchers`
* Turtlebot simulator: `sudo apt-get install ros-hydro-turtlebot-simulator`

### Start-up
* Empty world with keyboard teleop: 
 * Simulator: `roslaunch turtlebot_gazebo turtlebot_empty_world.launch`
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
* Adding objects to see something:
 * In Gazebo, select a cube, speher, or cylinder and drop it with a mouse click infront of the robot
 * Go back to rviz and observe what you can see
