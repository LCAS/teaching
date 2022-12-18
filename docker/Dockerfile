FROM dorowu/ubuntu-desktop-lxde-vnc:jammy-arm64
LABEL maintainer="Tiryoh<tiryoh@gmail.com>"

RUN apt-get update -q && \
    apt-get upgrade -yq && \
    DEBIAN_FRONTEND=noninteractive apt-get install -yq wget curl git build-essential vim sudo lsb-release locales bash-completion tzdata gosu && \
    rm -rf /var/lib/apt/lists/*
RUN useradd --create-home --home-dir /home/ubuntu --shell /bin/bash --user-group --groups adm,sudo ubuntu && \
    echo ubuntu:ubuntu | chpasswd && \
    echo "ubuntu ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
COPY ./ros-noetic-desktop.sh /ros-noetic-desktop.sh
RUN mkdir -p /tmp/ros_setup_scripts_ubuntu && mv /ros-noetic-desktop.sh /tmp/ros_setup_scripts_ubuntu/ && \
    gosu ubuntu /tmp/ros_setup_scripts_ubuntu/ros-noetic-desktop.sh && \
    rm -rf /var/lib/apt/lists/*
ENV USER ubuntu


FROM tiryoh/ros-desktop-vnc:humble
LABEL maintainer="L-CAS<mhanheide@lincoln.ac.uk>"
LABEL maintainer="L-CAS<25921584@lincoln.ac.uk>"

# Add repos
## LCAS repo (not available in noetic yet)
# RUN curl https://raw.githubusercontent.com/LCAS/rosdistro/master/lcas-rosdistro-setup.sh | bash -
## Gazebo
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

# Installing dependencies (map-server has some weird conflicts with libsdl)
RUN apt-get update; apt-get -y upgrade; \
    apt-get install -y ros-noetic-desktop-full ros-noetic-map-server openssh-server

# Creating thorvald_ws
RUN mkdir -p /opt/lcas/thorvald_ws/src
ADD Thorvald /opt/lcas/thorvald_ws/src/Thorvald
RUN /bin/bash -c ". /opt/ros/noetic/setup.bash && \
                  rosdep install --from-paths /opt/lcas/thorvald_ws/src -i -y && \
                  cd /opt/lcas/thorvald_ws && catkin_make install && rm -rf /opt/lcas/thorvald_ws/src/Thorvald && \
                  echo 'source /opt/lcas/thorvald_ws/install/setup.bash' >> ~/.bashrc"

# Creating ros_ws
RUN mkdir -p /opt/lcas/ros_ws/src
RUN /bin/bash -c ". /opt/lcas/thorvald_ws/install/setup.bash && cd /opt/lcas/ros_ws && catkin_make && \
                  echo 'source /opt/lcas/ros_ws/devel/setup.bash' >> ~/.bashrc"

# copying bacchus repo
RUN cd /opt/lcas/ros_ws/src && \
    git clone -b master --recursive https://github.com/LCAS/CMP9767M.git && \
    git clone -b teaching_noetic --recursive https://github.com/LCAS/bacchus_lcas.git

# Installing dependecies
RUN /bin/bash -c ".  /opt/lcas/thorvald_ws/install/setup.bash && \
                  cd /opt/lcas/ros_ws/src && \
                  git clone https://github.com/LCAS/depth_sensors.git && \
                  git clone https://github.com/LCAS/topological_navigation.git && \
                  git clone https://github.com/GT-RAIL/robot_pose_publisher.git && \
                  apt-get install -y ros-noetic-dwa-local-planner ros-noetic-teb-local-planner \
                  ros-noetic-velodyne-description ros-noetic-ira-laser-tools ros-noetic-move-base && \
                  apt-get -y clean"
RUN /bin/bash -c ". /opt/lcas/thorvald_ws/install/setup.bash; cd /opt/lcas/ros_ws; catkin_make -DCMAKE_CXX_STANDARD=17"

RUN cd /tmp && curl -fOL https://github.com/coder/code-server/releases/download/v4.8.3/code-server_4.8.3_amd64.deb && dpkg -i code-server_4.8.3_amd64.deb && rm code-server_4.8.3_amd64.deb && /usr/bin/code-server --install-extension ms-python.python && /usr/bin/code-server --install-extension pijar.ros-snippets && \
    wget -O code.deb 'https://code.visualstudio.com/sha/download?build=stable&os=linux-deb-x64' && dpkg -i code.deb
RUN su -c '/usr/bin/code --install-extension ms-python.python && /usr/bin/code --install-extension pijar.ros-snippets' ubuntu

RUN bash -c 'echo -e "[supervisord]\nredirect_stderr=true\nstopsignal=QUIT\nautorestart=true\ndirectory=/root\n\n[program:codeserver]\ndirectory=/home/ubuntu\ncommand=/usr/bin/code-server --auth none --bind-addr 0.0.0.0:8888\nuser=ubuntu\nenvironment=DISPLAY=:1,HOME=/home/ubuntu,USER=ubuntu,PYTHONPATH=/opt/lcas/ros_ws/devel/lib/python3/dist-packages:/opt/lcas/thorvald_ws/install/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages" > /etc/supervisor/conf.d/codeserver.conf'

