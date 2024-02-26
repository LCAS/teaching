FROM lcas.lincoln.ac.uk/lcas/ros:humble

RUN echo "deb [trusted=yes] https://download.eclipse.org/zenoh/debian-repo/ /" | tee -a /etc/apt/sources.list

RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
    && apt-get -y upgrade \
    && apt-get -y install --no-install-recommends ros-humble-desktop ros-humble-turtlebot3-gazebo ros-humble-turtlebot3-navigation2 \
        python3-pip ros-humble-turtlesim ros-humble-vision-opencv bash-completion python3-colcon-common-extensions \
        ros-humble-cv-bridge ros-humble-rosbridge-suite ros-humble-turtlebot3-teleop nano syslog-ng \
        build-essential \
        cmake \
        git \
        libssl-dev \
        libusb-1.0-0-dev \
        pkg-config \
        libgtk-3-dev \
        libglfw3-dev \
        libgl1-mesa-dev \
        libglu1-mesa-dev \
        libglvnd0 \
        libglx0 \
        libegl1 \
        libxext6 \
        libx11-6 \
        curl \  
        libusb-1.0* \
        udev \
        apt-transport-https \
        ca-certificates \
        curl \
        swig \
        software-properties-common \
        python3-pip \
        ros-humble-simulation \
        ros-humble-rmw-cyclonedds-cpp \
        ros-humble-rviz2* \
        ros-humble-cartographer* \
        ros-humble-nav* \
        ros-humble-teleop-twist-keyboard \
        ros-humble-joint-state-publisher* \
        ros-humble-robot-state-publisher* \ 
        ros-humble-xacro \
        ros-humble-imu-tools \
        libgflags-dev \
        nlohmann-json3-dev \
        ros-humble-image-* \
        wget \
        openvpn \
        python3 \
        cmake g++ pip \
        libasio-dev \
        libtinyxml2-dev \
        libssl-dev \
        python3-sphinx \
        libyaml-cpp-dev \
        ros-humble-cyclonedds \
        ros-humble-zenoh-bridge-dds \
        llvm-dev libclang-dev

RUN useradd -rm -d /home/lcas -s /bin/bash -g root -G sudo -u 1001 lcas
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

RUN echo "source /usr/share/gazebo/setup.bash" >> /home/lcas/.bashrc \
    && echo "source /opt/ros/humble/setup.bash" >> /home/lcas/.bashrc

RUN mkdir -p /opt/lcas/extensions 
WORKDIR /opt/lcas/extensions 

RUN git clone https://github.com/YDLIDAR/YDLidar-SDK.git &&\
    mkdir -p YDLidar-SDK/build && \
    cd YDLidar-SDK/build &&\
    cmake ..&&\
    make &&\
    make install &&\
    cd .. &&\
    pip install . &&\
    cd .. && rm -r YDLidar-SDK 
RUN wget -c https://github.com/google/glog/archive/refs/tags/v0.6.0.tar.gz  -O glog-0.6.0.tar.gz &&\
    tar -xzvf glog-0.6.0.tar.gz &&\
    cd glog-0.6.0 &&\
    mkdir build && cd build &&\
    cmake .. && make -j4 &&\
    make install &&\
    ldconfig &&\
    cd ../.. && rm -r glog-*
RUN wget -c https://github.com/Neargye/magic_enum/archive/refs/tags/v0.8.0.tar.gz -O  magic_enum-0.8.0.tar.gz &&\
    tar -xzvf magic_enum-0.8.0.tar.gz &&\
    cd magic_enum-0.8.0 &&\
    mkdir build && cd build &&\
    cmake .. && make -j4 &&\
    make install &&\
    ldconfig &&\
    cd ../.. && rm -r magic_enum*
RUN git clone https://github.com/libuvc/libuvc.git &&\
    cd libuvc &&\
    mkdir build && cd build &&\
    cmake .. && make -j4 &&\
    make install &&\
    ldconfig  &&\
    cd ../.. && rm -r libuvc*

RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y 
RUN git clone https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds.git
WORKDIR /opt/lcas/extensions/zenoh-plugin-ros2dds
RUN bash -c "source '$HOME/.cargo/env'; cargo build --release -p zenoh-bridge-ros2dds"
RUN install target/release/zenoh-bridge-ros2dds /usr/local/bin/


# add commit history to bust cache if and when needed.
ADD "https://api.github.com/repos/LCAS/limo_ros2/commits?per_page=1&sha=humble" .lcas_addons.limo_ros2.commits
ADD "https://api.github.com/repos/LCAS/teaching/commits?per_page=1&sha=2324-devel" .lcas_addons.teaching.commits

COPY *repos *.sh /tmp/.devcontainer/
RUN bash /tmp/.devcontainer/install.sh
RUN cat /tmp/.devcontainer/setup-router.sh >> /home/lcas/.bashrc

RUN mkdir -p /home/lcas/ws/src && ln -s /workspaces /home/lcas/ws/src/workspaces
WORKDIR /home/lcas/ws
