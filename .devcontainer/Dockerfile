FROM lcas.lincoln.ac.uk/lcas/ros:humble

RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
    && apt-get -y upgrade \
    && apt-get -y install ros-humble-desktop ros-humble-turtlebot3-gazebo ros-humble-turtlebot3-navigation2 \
        python3-pip ros-humble-turtlesim ros-humble-vision-opencv bash-completion python3-colcon-common-extensions \
        ros-humble-cv-bridge ros-humble-rosbridge-suite ros-humble-turtlebot3-teleop nano syslog-ng

### Install ngrok1
RUN curl -L -o /usr/local/bin/ngrok1 'https://github.com/marc-hanheide/network-scripts/blob/master/bin/ngrok1.lnx?raw=true' \
    && chmod +x /usr/local/bin/ngrok1


RUN echo "source /usr/share/gazebo/setup.bash" >> $HOME/.bashrc \
    && echo "source /opt/ros/humble/setup.bash" >> $HOME/.bashrc