ARG BASE_IMAGE=lcas.lincoln.ac.uk/lcas/ros:jammy-humble-cuda12.2-opengl

FROM ${BASE_IMAGE} as base

USER root

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -qq -y --no-install-recommends \
    git \
    python3-pip \
    python3-rosdep \
    python3-matplotlib \
    python3-pandas \
    ros-humble-rviz2 \
    ros-humble-gazebo-plugins \
    ros-humble-nav2-* \
    ros-humble-tf-transformations \
    ros-humble-rqt* \
    ros-humble-find-object-2d \
    zenoh-bridge-ros2dds

# get the source tree and analyse it for its package.xml only
FROM base as sourcefilter
COPY ./src /tmp/src/_workspace/src
COPY ./.devcontainer/*repos /tmp/.devcontainer/
RUN cd /tmp/src && vcs import < /tmp/.devcontainer/lcas.repos && \
    vcs pull

# remove everything that isn't package.xml
RUN find /tmp/src -type f \! -name "package.xml" -print | xargs rm -rf

# install all dependencies listed in the package.xml
FROM base as depinstaller
# copy the reduced source tree (only package.xml) from previous stage
COPY --from=sourcefilter /tmp/src /tmp/src
RUN rosdep update && apt-get update
RUN rosdep install --from-paths /tmp/src --ignore-src -r -y && rm -rf /tmp/src

FROM depinstaller as depbuilder
# add commit history to bust cache if and when needed.
ADD "https://api.github.com/repos/LCAS/limo_ros2/commits?per_page=1&sha=humble" .lcas_addons.limo_ros2.commits
COPY .devcontainer/*repos .devcontainer/*.sh /tmp/.devcontainer/

RUN source /opt/ros/humble/setup.bash; \
    apt update; \
    rosdep --rosdistro=humble update; \
    mkdir -p /opt/ros/lcas/src; \
    cd /opt/ros/lcas/src; \
    vcs import < /tmp/.devcontainer/lcas.repos; \
    vcs pull; \
    rosdep install --from-paths . -r -i -y; \
    cd /opt/ros/lcas
RUN cd /opt/ros/lcas; colcon build --packages-up-to limo_gazebosim

FROM depbuilder as devcontainer
# add user ros with sudo rights if it doesn't exist
RUN if ! id ros; then \
        useradd -ms /bin/bash ros && echo "ros:ros" | chpasswd && adduser ros sudo; \
        echo "ALL ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers; \
        echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc; \
    else echo "user ros already exists"; \
    fi
RUN echo "source /opt/ros/lcas/install/setup.bash" >> ~ros/.bashrc

FROM devcontainer as compiled

COPY ./src /opt/ros/lcas/src/teaching-code/src
RUN cd /opt/ros/lcas && colcon build --packages-up-to uol_tidybot
RUN echo "source /opt/ros/lcas/install/setup.bash" >> ~/.bashrc

USER ros