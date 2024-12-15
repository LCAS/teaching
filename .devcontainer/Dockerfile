ARG BASE_IMAGE=lcas.lincoln.ac.uk/lcas/limo_platform_amd64:1

FROM ${BASE_IMAGE} as base

# making the standard global variables available for target-specific builds
ARG TARGETARCH

USER root

# get the source tree and analyse it for its package.xml only
FROM base as sourcefilter
RUN mkdir -p /tmp/src/
COPY ./src/*/package.xml /tmp/src/_workspace/src/_pkgs_xmls
COPY ./src/*/*/package.xml /tmp/src/_workspace/src/_pkgs_xmls
COPY ./src/*/*/*/package.xml /tmp/src/_workspace/src/_pkgs_xmls
COPY ./src/*/*/*/*/package.xml /tmp/src/_workspace/src/_pkgs_xmls

# remove everything that isn't package.xml
RUN find /tmp/src -type f \! -name "package.xml" -print | xargs rm -rf

# install all dependencies listed in the package.xml
FROM base as devcontainer
# copy the reduced source tree (only package.xml) from previous stage
COPY --from=sourcefilter /tmp/src /tmp/src
RUN rosdep update --rosdistro=${ROS_DISTRO} && apt-get update
RUN . /opt/ros/lcas/install/setup.sh && rosdep install --from-paths /tmp/src --ignore-src -y && rm -rf /tmp/src && apt-get clean && rm -rf /var/lib/apt/lists/* /tmp/src


FROM devcontainer as compiled

COPY ./src /opt/ros/lcas/src/workspace/src
RUN . /opt/ros/lcas/install/setup.sh && \
    apt update && \
    rosdep --rosdistro=${ROS_DISTRO} update && \
    rosdep install --from-paths /opt/ros/lcas/src/workspace/src --ignore-src -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN cd /opt/ros/lcas && colcon build  && \
    rm -rf /opt/ros/lcas/src/ /opt/ros/lcas/build/ /opt/ros/lcas/log/

USER ros
WORKDIR /home/ros
ENV SHELL=/bin/bash
