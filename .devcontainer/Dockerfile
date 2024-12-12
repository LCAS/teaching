ARG BASE_IMAGE=lcas.lincoln.ac.uk/lcas/ros:jammy-humble-cuda12.2-opengl

FROM ${BASE_IMAGE} as base

# making the standard global variables available for target-specific builds
ARG TARGETARCH

USER root

ENV DEBIAN_FRONTEND=noninteractive


RUN echo "building for ${TARGETARCH}"
RUN if [ "${TARGETARCH}" = "arm64" ]; then \
    add-apt-repository -s -y ppa:openrobotics/gazebo11-non-amd64; \
    fi

RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -qq -y --no-install-recommends \
        git \
        python3-pip \
        python3-rosdep \
        python3-matplotlib \
        python3-pandas && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# get the source tree and analyse it for its package.xml only
FROM base as sourcefilter
COPY ./src /tmp/src/_workspace/src
COPY ./.devcontainer/*repos /tmp/.devcontainer/
RUN cd /tmp/src && vcs import < /tmp/.devcontainer/lcas.repos
RUN if [ "${TARGETARCH}" = "arm64" ]; then \
        cd /tmp/src; \
        vcs import < /tmp/.devcontainer/gazebo_ros_pkgs.repos; \
    fi
RUN cd /tmp/src && vcs pull

# remove everything that isn't package.xml
RUN find /tmp/src -type f \! -name "package.xml" -print | xargs rm -rf

# install all dependencies listed in the package.xml
FROM base as depinstaller
# copy the reduced source tree (only package.xml) from previous stage
COPY --from=sourcefilter /tmp/src /tmp/src
RUN rosdep update --rosdistro=${ROS_DISTRO} && apt-get update
RUN rosdep install --from-paths /tmp/src --ignore-src -r -y && rm -rf /tmp/src && apt-get clean && rm -rf /var/lib/apt/lists/* /tmp/src

FROM depinstaller as depbuilder
COPY .devcontainer/*repos .devcontainer/*.sh /tmp/.devcontainer/

# get the source tree and build it (include gazebo only for arm64 platform as it is not available for arm64)
# see https://github.com/gazebosim/gazebo-classic/issues/3236
RUN mkdir -p /opt/ros/lcas/src && \
    cd /opt/ros/lcas/src && \
    vcs import < /tmp/.devcontainer/lcas.repos
RUN  if [ "${TARGETARCH}" = "arm64" ]; then cd /opt/ros/lcas/src; vcs import < /tmp/.devcontainer/gazebo_ros_pkgs.repos; fi

RUN . /opt/ros/humble/setup.sh && \
    apt update && \
    rosdep --rosdistro=${ROS_DISTRO} update && \
    cd /opt/ros/lcas/src && \
    vcs pull && \
    rosdep install --from-paths . -i -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# build the workspace but only until limo_gazebosim to avoid building the hardware specific packages
RUN cd /opt/ros/lcas; colcon build --packages-up-to limo_gazebosim; \
    rm -rf /opt/ros/lcas/src/ /opt/ros/lcas/build/ /opt/ros/lcas/log/

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
RUN . /opt/ros/lcas/install/setup.sh && \
    apt update && \
    rosdep --rosdistro=${ROS_DISTRO} update && \
    rosdep install --from-paths /opt/ros/lcas/src/teaching-code/src --ignore-src -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN cd /opt/ros/lcas && colcon build --packages-up-to uol_tidybot_control && \
    rm -rf /opt/ros/lcas/src/ /opt/ros/lcas/build/ /opt/ros/lcas/log/

RUN echo "source /opt/ros/lcas/install/setup.bash" >> ~/.bashrc
RUN find /opt/ros/lcas/ 

USER ros
ENV SHELL=/bin/bash
