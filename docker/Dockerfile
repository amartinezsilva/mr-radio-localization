# This is a custom-made image for simulating the new version of VRX
# With ROS2, Ubuntu 24.04 and new Gazebo
FROM osrf/ros:jazzy-desktop-full

# Install IGN Gazebo Harmonic
# fuse3 provides fusermount3 for running QGC; python3-flask backs the setup GUI (gui/app.py)
RUN apt-get update && sudo apt-get install lsb-release wget gnupg nano inetutils-ping tmux fuse3 python3-flask -y
RUN lsb_release -cs
RUN dpkg --print-architecture
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt-get update && apt-cache search gz- && apt-get install gz-harmonic -y

RUN apt install ros-jazzy-ros-gz-interfaces ros-jazzy-xacro ros-jazzy-joy-teleop ros-jazzy-gps-msgs -y


RUN apt install libspdlog-dev libceres-dev libeigen3-dev libomp-dev -y    

RUN apt install git build-essential cmake genromfs python3-pip python3-venv -y

# Install PX4 repository
RUN git clone https://github.com/PX4/PX4-Autopilot.git --recursive
# Set PX4 working directory
RUN cd PX4-Autopilot/Tools/setup && ./ubuntu.sh --break-system-packages

# Clone and compile small_gicp
RUN cd /root/ && git clone https://github.com/koide3/small_gicp
RUN cd /root/small_gicp && mkdir build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release && make -j4 && make install

# Install the Micro XRCE-DDS Agent
RUN cd /root/ && git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
RUN cd /root/Micro-XRCE-DDS-Agent && mkdir build && cd build && cmake .. && make -j4 && make install && ldconfig /usr/local/lib/

# Docker only invalidates a `RUN git clone` layer when this instruction's own
# text changes, not when the remote it clones actually moves -- so a normal
# cached rebuild would silently keep serving a stale checkout even after new
# commits land upstream. CACHEBUST forces a fresh clone on every build that
# passes a new value (see setup.sh, which passes the current timestamp)
# without invalidating the expensive earlier layers (base image, PX4-Autopilot
# clone/setup, small_gicp, Micro-XRCE-DDS-Agent) above it.
ARG CACHEBUST=1
RUN cd && mkdir -p ros2_ws/src && cd ros2_ws/src && git clone https://github.com/amartinezsilva/mr-radio-localization --recursive
SHELL ["/bin/bash", "-c"]
RUN cd $HOME/ros2_ws/src && git clone https://github.com/PX4/px4_msgs.git && cd ~/ros2_ws
RUN sudo apt-get install ros-jazzy-sophus -y
RUN cd $HOME/ros2_ws/src && git clone https://github.com/robotics-upo/eliko_ros.git && cd ~/ros2_ws
RUN cd $HOME/ros2_ws/src && git clone https://github.com/PX4/px4_ros_com && cd ~/ros2_ws

# By default the DockerFile uses the root as user.
# We will add a custom user so that we work in user space by default
# And then switch to that user
# Source: https://www.docker.com/blog/understanding-the-docker-user-instruction/
RUN useradd -ms /bin/bash jazzy
# Sometimes it is good to add a password so that we can use sudo to install things
# To this end, uncomment the following line, remove extra ### and change the password 
RUN echo "jazzy:jazzy" | chpasswd  
#### Remove #### to change the password
RUN adduser jazzy sudo & adduser jazzy dialout & adduser jazzy plugdev & echo Hi
RUN usermod -aG sudo jazzy

# Set up QGroundControl
RUN mkdir -p /home/jazzy/Desktop && \
    wget -O /home/jazzy/Desktop/QGroundControl-x86_64.AppImage \
      https://github.com/mavlink/qgroundcontrol/releases/latest/download/QGroundControl-x86_64.AppImage && \
    chmod +x /home/jazzy/Desktop/QGroundControl-x86_64.AppImage

# Everything above was cloned while running as root. ros2_ws landed under
# /root (via `cd` with no argument), while PX4-Autopilot landed at
# /PX4-Autopilot (the base image's default WORKDIR is / , not /root, and
# that clone has no `cd` in front of it) -- both outside jazzy's home,
# /root also being a directory jazzy cannot even traverse. Move them into
# jazzy's home (created by useradd -m above) and hand over ownership
# *before* building, so colcon's --symlink-install symlinks get created
# with their final /home/jazzy/... absolute paths instead of stale ones
# that would dangle once moved.
RUN mv /root/ros2_ws /home/jazzy/ros2_ws && \
    mv /PX4-Autopilot /home/jazzy/PX4-Autopilot && \
    chown -R jazzy:jazzy /home/jazzy

RUN cd /home/jazzy/ros2_ws && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install && \
    chown -R jazzy:jazzy /home/jazzy/ros2_ws

# Guided entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

USER jazzy

# We will use the home directory as work directory
# So that the docker starts at HOME!
WORKDIR "/home/jazzy"
ENV HOME="/home/jazzy"
ENV PX4_DIR="/home/jazzy/PX4-Autopilot"

ENTRYPOINT ["/entrypoint.sh"]
CMD ["menu"]