FROM lcas.lincoln.ac.uk/lcas/ros:humble

RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
    && apt-get -y upgrade \
    && apt-get -y install ros-humble-desktop ros-humble-turtlebot3-gazebo ros-humble-turtlebot3-navigation2 \
        python3-pip ros-humble-turtlesim ros-humble-vision-opencv bash-completion python3-colcon-common-extensions \
        ros-humble-cv-bridge ros-humble-rosbridge-suite ros-humble-turtlebot3-teleop nano syslog-ng


RUN useradd -rm -d /home/lcas -s /bin/bash -g root -G sudo -u 1001 lcas
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

RUN echo "source /usr/share/gazebo/setup.bash" >> /home/lcas/.bashrc \
    && echo "source /opt/ros/humble/setup.bash" >> /home/lcas/.bashrc
