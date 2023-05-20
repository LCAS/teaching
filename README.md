LCAS Teaching Repo
========

A repository for all the teaching stuff (mainly CMP3103M)



## Some useful launch commands

### Nav2

Run a full navigation stack (requires additional packages), see https://navigation.ros.org/getting_started/index.html

```
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False world:=/opt/ros/lcas_addons/install/uol_turtlebot_simulator/share/uol_turtlebot_simulator/worlds/object-search-training.world slam:=True autostart:=True x_pose:=0.0 y_pose:=0.0
```

