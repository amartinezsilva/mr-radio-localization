# Standalone image: the localization stack only, without the PX4/Gazebo
# simulator -- for running uwb_localization against your own rosbags (or
# the real-world dataset) without downloading or building UWBPX4Sim.
# For the full SITL setup, see Dockerfile.sim (built FROM the UWBPX4Sim
# image) -- `./setup.sh --sim`.
#
# The localization dependencies themselves are installed by
# install_localization_deps.sh, shared with Dockerfile.sim so both images
# always get the same set.
FROM osrf/ros:jazzy-desktop

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y sudo nano git

# Non-root user, same name/uid as the SITL image (UWBPX4Sim's base image
# creates `jazzy` the same way -- uid 1001, since Ubuntu 24.04 images
# already ship an `ubuntu` user at 1000), so setup.sh, the bind mounts and
# the docs behave identically for both images.
RUN useradd -ms /bin/bash jazzy && \
    echo "jazzy:jazzy" | chpasswd && \
    usermod -aG sudo,dialout,plugdev jazzy

ENV HOME="/home/jazzy"
ARG ROS_WS=/home/jazzy/ros2_ws

COPY install_localization_deps.sh /usr/local/bin/install_localization_deps.sh
RUN chmod +x /usr/local/bin/install_localization_deps.sh && \
    install_localization_deps.sh system ${ROS_WS}

# eliko_messages (from eliko_ros): the UWB range message type. In the SITL
# image it comes with the UWBPX4Sim base image instead.
RUN git clone https://github.com/robotics-upo/eliko_ros.git ${ROS_WS}/src/eliko_ros

# Docker only invalidates a `RUN git clone` layer when this instruction's own
# text changes, not when the remote it clones actually moves -- setup.sh
# passes a fresh CACHEBUST every build so this clone (and what follows) is
# redone without invalidating the slower layers above.
ARG CACHEBUST=1
RUN git clone https://github.com/amartinezsilva/mr-radio-localization ${ROS_WS}/src/mr-radio-localization

RUN install_localization_deps.sh rosdep ${ROS_WS}

# BUILD_TESTING=OFF: the tests aren't run in the image, and ars548_driver's
# own unit tests don't compile against its current jazzy branch.
RUN cd ${ROS_WS} && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF && \
    chown -R jazzy:jazzy /home/jazzy

# Interactive shells (menu option "Open a shell", `docker exec ... bash`)
# come up with the workspace already sourced.
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/jazzy/.bashrc && \
    echo "source ${ROS_WS}/install/setup.bash" >> /home/jazzy/.bashrc

COPY localization-lib.sh /localization-lib.sh
COPY entrypoint-standalone.sh /entrypoint.sh
RUN chmod +x /localization-lib.sh /entrypoint.sh

USER jazzy
WORKDIR /home/jazzy

ENTRYPOINT ["/entrypoint.sh"]
CMD ["menu"]
