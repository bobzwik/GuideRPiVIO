```
sudo apt update
sudo apt upgrade -y
```

```
cd ~
sudo apt install curl -y
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
```

```
sudo groupadd docker
sudo usermod -aG docker $USER
sudo reboot
```

```
sudo apt install wget gpg
wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
sudo install -D -o root -g root -m 644 packages.microsoft.gpg /etc/apt/keyrings/packages.microsoft.gpg
echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" |sudo tee /etc/apt/sources.list.d/vscode.list > /dev/null
rm -f packages.microsoft.gpg
```

```
sudo apt install apt-transport-https
sudo apt update
sudo apt install code
```

```
mkdir -p ~/ros2_ws/src/RPi_GCS
cd ~/ros2_ws/src/RPi_GCS
```

```
code .
xhost +
```

Click bottom left. Open new dev container. ROS ijnek. Additional options. humble. desktop-full
Dockerfile:
```
FROM osrf/ros:humble-desktop-full

# Add ubuntu user with same UID and GID as your host system, if it doesn't already exist
# Since Ubuntu 24.04, a non-root user is created by default with the name vscode and UID=1000
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN if ! id -u $USER_UID >/dev/null 2>&1; then \
        groupadd --gid $USER_GID $USERNAME && \
        useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME; \
    fi
# Add sudo support for the non-root user
RUN apt-get update && \
    apt-get install -y sudo && \
    echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

# Switch from root to user
USER $USERNAME

# Add user to video group to allow access to webcam
RUN sudo usermod --append --groups video $USERNAME

# Update all packages
RUN sudo apt update && sudo apt upgrade -y

# Install Git
RUN sudo apt install -y git

# Rosdep update
RUN rosdep update

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

################################
## ADD ANY CUSTOM SETUP BELOW ##
################################

SHELL ["/bin/bash", "-c"]

ARG WS=/home/$USERNAME/ros2_ws
ARG SRC=$WS/src

RUN sudo apt-get update && sudo apt-get install -y \
    git \
    wget \
    nano

WORKDIR $SRC
RUN git config --global --add safe.directory $SRC/ardupilot && \
    git clone --depth 1 --recurse-submodules https://github.com/ArduPilot/ardupilot.git

WORKDIR $SRC/ardupilot/
# RUN sudo apt-get update && \
#     sudo apt-get install -y \
#     sudo \
#     lsb-release \
#     tzdata

RUN USER=$USERNAME Tools/environment_install/install-prereqs-ubuntu.sh -y && \
    . /home/$USERNAME/.profile



RUN  sudo rm -rf /var/lib/apt/lists/*

WORKDIR $SRC
RUN git clone --depth 1 --recurse-submodules --branch humble https://github.com/micro-ROS/micro-ROS-Agent.git && \
    git clone --depth 1 --recurse-submodules --branch ros2 https://github.com/ArduPilot/ardupilot_gazebo && \
    git clone --depth 1 --recurse-submodules https://github.com/ArduPilot/ardupilot_gz.git && \
    git clone --depth 1 --recurse-submodules https://github.com/ArduPilot/SITL_Models.git && \
    git clone --depth 1 --recurse-submodules --branch humble https://github.com/ros/sdformat_urdf.git && \
    git clone --depth 1 --recurse-submodules --branch humble https://github.com/gazebosim/ros_gz.git

RUN sudo apt-get update && \
    sudo apt-get install -y default-jre && \
    git clone --depth 1 --recurse-submodules https://github.com/ardupilot/Micro-XRCE-DDS-Gen.git
WORKDIR $SRC/Micro-XRCE-DDS-Gen/
RUN ./gradlew assemble 
ENV PATH "$PATH:$SRC/Micro-XRCE-DDS-Gen/scripts"

# WORKDIR $SRC/ardupilot/
# RUN ./waf distclean && \
#     ./waf configure --board sitl --enable-dds

WORKDIR $WS

RUN sudo apt-get update && \
    sudo apt-get upgrade -y && \
    sudo apt-get install -y \
    curl \
    lsb-release \
    gnupg

RUN sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN sudo apt-get update && \
    sudo apt-get install -y \
    gz-harmonic
    # ros-humble-ros-gzharmonic

ENV GZ_VERSION=harmonic

RUN sudo apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    MAKEFLAGS='-j1' colcon build --packages-up-to ardupilot_gz_bringup
```
devcontainer.json:
```
{
  "name": "humble desktop-full",
  "dockerFile": "Dockerfile",
  "runArgs": [
    "--privileged",
    "--network=host"
  ],
  "workspaceMount": "source=${localEnv:HOME}${localEnv:USERPROFILE}/ros2_ws/src/RPi_GCS,target=/home/ubuntu/ros2_ws/src/RPi_GCS,type=bind",
  "workspaceFolder": "/home/ubuntu/ros2_ws",
  "mounts": [
    "source=${localEnv:HOME}${localEnv:USERPROFILE}/.bash_history,target=/home/vscode/.bash_history,type=bind"
  ]
}
```
