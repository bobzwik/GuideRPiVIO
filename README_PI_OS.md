# GuideRPiVIO

NOTE: The following guide uses hardware flow-control for the serial connection between the FC and the RPi. I thought this was necessary for a 12.5 MBaud datarate, but it turns out that hardware flow-control isn't needed, at least for the hardware I have (H743 processor). I also did not do extensive testing without hardware flow-control, but I plan to if I decide to switch to a flight controller that doesn't have RTS and CTS pins.

## 1 - Wiring

```
Matek ~~~ RPi4
Rx7   ==> GPIO 14 (Pin 8)
Tx7   ==> GPIO 15 (Pin 10)
Rts7  ==> GPIO 16 (Pin 36)
Cts7  ==> GPIO 17 (Pin 11)
Gnd   ==> Gnd
```

## 2 - Raspberry Pi 4 Setup

### 2.1 - Ubuntu and Pi Setup
Flash SD card with Raspberry Pi OS Bookworm (64 bit) using [Raspberry Pi Imager](https://www.raspberrypi.com/software/). This guide should work for both Desktop or Server versions of Pi OS, but if you're going Server, it would be best to configure your networking before going too far in the setup process, to allow yourself to `ssh` into the RPi4 and copy/paste the commands from a PC with a web explorer. 
```
sudo apt update
sudo rpi-eeprom-update -a
sudo reboot
sudo apt update
sudo apt upgrade -y
sudo reboot
```
Edit `sudo nano /boot/firmware/config.txt`. At the bottom of the file, add the lines:
```
enable_uart=1
init_uart_clock=200000000
dtoverlay=disable-bt
dtoverlay=uart0,ctsrts
gpio=16-17=a3
```
**Explanation:** 
RPi 4 has 2 types of UART
* PL011 (better)
* Mini-UART (reduced features)

We disable Bluetooth (which was on a PL011) to move the primary UART from a Mini-UART to the PL011 ([more info here](https://www.raspberrypi.com/documentation/computers/configuration.html#configure-uarts)). We also specify that UART0 will use hardware flow control (`ctsrts`) using GPIO 16 and 17. And we speed up the UART clock to 200 MHz to allow for a 12.5 MBaud connection. UART0 will be available on `/dev/ttyAMA0` or `/dev/serial0`.

Then save `/boot/firmware/config.txt` with these changes.

To make sure the Primary UART (UART0) is unoccupied, disable the Bluetooth controller and the serial Linux console permanently.
```
sudo systemctl disable hciuart
sudo systemctl disable serial-getty@ttyAMA0.service
```
To make sure to disable the serial Linux console, edit the file 
```
sudo nano /boot/firmware/cmdline.txt
```
Remove the `console=serial0,115200` or similar entry, leaving the rest of the line intact.

And then add yourself into the group `dialout` to be allowed to use the UART ports
```
sudo usermod -a -G dialout $USER
sudo reboot
```
**Optional: CPU overclock**
For those who need a little extra from the CPU. You can add these lines to `/boot/firmware/config.txt`. Choose either of the 2 sets of 3 parameters.
```
#over_voltage=6
#arm_freq=2000
#gpu_freq=700
over_voltage=8
arm_freq=2250
gpu_freq=750
```
This will not improve DDS or PPP. It's only in case you plan on running CPU intensive algorithms alongside DDS.

For the OV9281 camera, in `/boot/firmware/config.txt`, change `camera-auto-detect=1` to `camera-auto-detect=0`. And add
```
dtoverlay=ov9281
```

### 2.2 - PPPD
Most likely, the version by default on Bookworm is version `2.4.9` (you can check with `pppd --version`). While you can still use version `2.4.9`, it will not be able to achieve the fastest 12.5M baudrate. You will need version `2.5.1`.
```
cd ~
sudo apt update && sudo apt install curl build-essential libssl-dev autogen autoconf libtool -y
curl -L https://github.com/ppp-project/ppp/archive/refs/tags/ppp-2.5.1.tar.gz | tar -xz
cd ~/ppp-ppp-2.5.1
autoreconf -i 
./configure 
make 
sudo make install
```
Andrew Tridgell has a [video](https://youtu.be/VGy6eOf7Uqk?si=zquWZOHEGXQJZNzV) showcasing the uses of PPP.

### 2.3 - Ardupilot
Install the Ardupilot development environment ([more info here](https://ardupilot.org/dev/docs/building-setup-linux.html)).
```
cd ~
sudo apt install git -y
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
sudo reboot
```

### 2.4 - Micro-XRCE-DDS-Gen
Install Micro-XRCE-DDS-Gen. This is needed when building the Ardupilot firmware with DDS enabled.
```
cd ~
sudo apt install default-jre -y
git clone --recurse-submodules https://github.com/ardupilot/Micro-XRCE-DDS-Gen.git
cd Micro-XRCE-DDS-Gen/
./gradlew assemble
echo "export PATH=\$PATH:$PWD/scripts" >> ~/.bashrc
sudo reboot
```

### 2.5 - mavp2p
MAVproxy can be used to connect to the flight controller from the Pi (MAVproxy is installed with Ardupilot's prereq script). MAVproxy can also forward the connection to a different network, to allow another computer on that network to connect to the flight controller. However, MAVproxy is CPU heavy if that is the only use for it.

Instead, we can install `mavp2p`, a [flexible and efficient Mavlink proxy / bridge / router](https://github.com/bluenviron/mavp2p) to route the Mavlink connection to a different network.

First, choose a version of `mavp2p` (the latest, most likely) from the [releases page](https://github.com/bluenviron/mavp2p/releases). Then download the appropriate `tar.gz` (for a Pi4 running a 64bit OS, it is `arm64v8`) using:
```
cd ~
wget https://github.com/bluenviron/mavp2p/releases/download/v1.3.1/mavp2p_v1.3.1_linux_arm64v8.tar.gz
```
Replacing both `v1.3.1` by the version of your choice.

Then:
```
tar xzf mavp2p_v1.3.1_linux_arm64v8.tar.gz
sudo mv mavp2p /usr/local/sbin
```

### 2.6 - cpp_imu_sub
```
mkdir -p ~/ros2_docker/ros2_ws/src && cd ~/ros2_docker/ros2_ws/src
git clone https://github.com/bobzwik/cpp_imu_sub.git
```

### 2.7 - Install Docker
```
cd ~
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
```
Post-installation steps
```
sudo groupadd docker
sudo usermod -aG docker $USER
sudo reboot
```
### 2.8 - Create Docker Image
Create Dockerfile
```
cd ~/ros2_docker
nano Dockerfile
```
And paste in the following script:
```
# Use the ROS Humble base image
FROM ros:humble-ros-base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-c"]

# Upgrade everything
RUN apt-get update && apt-get upgrade -y

# Install necessary dependencies
RUN apt-get install -y \
    python3-vcstool \
    python3-rosdep \
    python3-pip \
    build-essential \
    v4l-utils \
    git \
    wget
RUN python3 -m pip install colcon-meson jinja2 ninja ply

# Download the .repos and clone packages
WORKDIR /ros2_ws/src
RUN wget https://raw.githubusercontent.com/bobzwik/GuideRPiVIO/main/ros2.repos && \
    vcs import --recursive < ros2.repos

# Install pigpio (needed for cpp_imu_sub)
WORKDIR /
RUN git clone https://github.com/joan2937/pigpio && \
    cd pigpio && \
    make && \
    make install

# Copy other ROS packages from host
COPY ros2_ws /ros2_ws/

# Download libcamera and camera_ros
WORKDIR /ros2_ws/src
RUN git clone https://github.com/raspberrypi/libcamera.git && \
    git clone https://github.com/christianrauch/camera_ros.git
# Use latest libcamera tag with "vX.X.X" format
WORKDIR /ros2_ws/src/libcamera
RUN git fetch --tags && \
    latestTag=$(git tag --list 'v*.*.*' | grep -E '^v[0-9]+\.[0-9]+\.[0-9]+$' | sort -V | tail -n 1) && \
    git checkout $latestTag

# Install dependencies for the workspace 
WORKDIR /ros2_ws
RUN apt-get update && \
    rosdep update && \
    rosdep install --rosdistro ${ROS_DISTRO} --from-paths src --ignore-src -y --skip-keys pigpio --skip-keys libcamera

# Build the workspace
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build

# Source the ROS files and ROS workspace build files, for every opened shell
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /etc/bash.bashrc

# Copy entrypoint script into the image
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

RUN rm -rf /var/lib/apt/lists/*

# Set the default entrypoint
ENTRYPOINT ["/entrypoint.sh"]

# Default command (can be overridden)
CMD ["bash"]
```
Create entrypoint.sh
```
nano entrypoint.sh
```
And paste in the following script:
```
#!/bin/bash
set -e

# Source ROS 2 and workspace setup files
source /opt/ros/${ROS_DISTRO}/setup.bash
source /ros2_ws/install/setup.bash

# Start pigpio daemon
sudo  pigpiod

# Execute the passed command
exec "$@"
```
Now build the docker image (takes about 530 seconds)
```
cd ~/ros2_docker
docker build -t ros2_rpi4 .
```
And run the image!
```
docker run -it --rm --network=host --privileged --name ros2_cont ros2_rpi4
```
Enter the docker with another terminal:
```
docker exec -it ros2_cont bash
```

## 3 - Flight Controller Setup
If using a brand new flight controller that doesn't have Ardupilot pre-installed, you have to flash Ardupilot with bootloader. I recommend using [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html#get-software). This step doesn't have to be performed on the RPi 4, I prefer using a **Windows PC**. Download the latest or stable firmware for your board (for example, for the Matek H743, [from here](https://firmware.ardupilot.org/Copter/stable/MatekH743/)). Make sure you download the `_with_bl.hex` file. Then follow [this guide](https://www.mateksys.com/?p=6905) or [this video tutorial](https://www.youtube.com/watch?v=utdvxuuekSE).

## 4 - Build Ardupilot Firmware
Once your flight controller is ready, plug it via USB to your RPi 4. Now back on your RPi 4, build the Ardupilot firmware with DDS and PPP enabled and upload it to your FC. When doing `./waf configure`, specify the correct board.
```
cd ~/ardupilot
./waf configure --board MatekH743 --enable-DDS --enable-PPP
./waf copter --upload
```
This will take some time.

## 5 - Ardupilot Parameters
Now you must set the correct parameters in Ardupilot. You will have to reboot and/or refresh the parameters multiples times, as you will be enabling multiple "enable" parameters. You can use Mavproxy on your RPi4 to edit the parameters (`mavproxy.py --console` will detect the flight controller connected via USB), or connect your flight controller to a PC with Mission Planner. 

### 5.1 - Serial Port Parameters:
```
SERIAL1_PROTOCOL,48
SERIAL1_BAUD,12500000
BRD_SER1_RTSCTS,1
```
This enable PPP on SERIAL1, maxes out the baudrate (allowing for fast networking and fast log download using the webserver (faster than USB)), and also enables flow control.

### 5.2 - PPP Parameters:
```
NET_ENABLE,1
NET_OPTIONS,0
NET_P1_IP0,192
NET_P1_IP1,168
NET_P1_IP2,13
NET_P1_IP3,16
NET_P1_PORT,14550
NET_P1_PROTOCOL,2
NET_P1_TYPE,1
NET_P2_TYPE,0
NET_P3_TYPE,0
NET_P4_TYPE,0
```
This enables Mavlink over PPP. `NET_P1_TYPE,1` sets the port to use UDP and enables the IP address parameters. The address `192.168.13.16` is the destination address, which will need to be specified by `pppd` on the RPi4. You can choose a random value for `NET_P1_IP3`, as long as it is not the same as another device on your network. 

### 5.3 - DDS Parameters:
```
DDS_DOMAIN_ID,0
DDS_ENABLE,1
DDS_IP0,192
DDS_IP1,168
DDS_IP2,13
DDS_IP3,16
DDS_UDP_PORT,2019
DDS_MAX_RETRY,0
```
This sets the DDS destination to the same address at the one specified by the `NET_*` parameters, but to port `2019` instead.

### 5.4 - Webserver Parameters:
The webserver is done via a lua script. Therefore, first enable scripting:
```
SCR_ENABLE,1
SCR_VM_I_COUNT,1000000
```
And then download the file [net_webserver.lua](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Scripting/applets/net_webserver.lua) from Ardupilot's Github. There is also a [readme](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Scripting/applets/net_webserver.md). Andrew Tridgell has a good [video tutorial](https://youtu.be/bN6iDP4Zjzg?si=96L6GpZp2XLA-xA4) for the webserver.

Using Mission Planner's MAVftp tool, upload the `net_webserver.lua` file to the flight controller's `APM/scripts/` folder. Then reboot and change the following parameters:


```
WEB_ENABLE,1
WEB_BIND_PORT,8080
```

## 6 - Run
You're now ready to use DDS and PPP! 
Power the RPi4 and the flight controller, open a terminal and run:
```
sudo pppd /dev/ttyAMA0 12500000 192.168.13.16:192.168.13.65 crtscts debug noauth nodetach local proxyarp ktune
```
This takes the incoming stream from `/dev/ttyAMA0` (at a 12.5M Baudrate) destined for `192.168.13.16` (the local IP of `pppd`) and defines the remote (the flight controller) as `192.168.13.65`. You can change `65` for anything you want.

Then, run the docker container.
```
docker run -it --rm --network=host --privileged --name ros2_cont ros2_rpi4
```
And start the `micro_ros_agent`, listening on port 2019 (specified in the `DDS_*` parameters). No need to source your repo (`source install/setup.bash`) as it is already sourced in your `/etc/bash.bashrc` (as per the Dockerfile).

```
ros2 run micro_ros_agent micro_ros_agent udp4 --port 2019
```
You should see the `micro_ros_agent` acknowledge the reception of the DDS topics. You can now view the available topics with
```
ros2 topic list -v
Published topics:
 * /ap/airspeed [geometry_msgs/msg/Vector3] 1 publisher
 * /ap/battery [sensor_msgs/msg/BatteryState] 1 publisher
 * /ap/clock [rosgraph_msgs/msg/Clock] 1 publisher
 * /ap/geopose/filtered [geographic_msgs/msg/GeoPoseStamped] 1 publisher
 * /ap/gps_global_origin/filtered [geographic_msgs/msg/GeoPointStamped] 1 publisher
 * /ap/imu/experimental/data [sensor_msgs/msg/Imu] 1 publisher
 * /ap/navsat [sensor_msgs/msg/NavSatFix] 1 publisher
 * /ap/pose/filtered [geometry_msgs/msg/PoseStamped] 1 publisher
 * /ap/tf_static [tf2_msgs/msg/TFMessage] 1 publisher
 * /ap/time [builtin_interfaces/msg/Time] 1 publisher
 * /ap/twist/filtered [geometry_msgs/msg/TwistStamped] 1 publisher
 * /parameter_events [rcl_interfaces/msg/ParameterEvent] 1 publisher
 * /rosout [rcl_interfaces/msg/Log] 1 publisher

Subscribed topics:
 * /ap/cmd_gps_pose [ardupilot_msgs/msg/GlobalPosition] 1 subscriber
 * /ap/cmd_vel [geometry_msgs/msg/TwistStamped] 1 subscriber
 * /ap/joy [sensor_msgs/msg/Joy] 1 subscriber
 * /ap/tf [tf2_msgs/msg/TFMessage] 1 subscriber
```
View the frequency of a topic with
```
ros2 topic hz /ap/time
```
View the contents of a topic with
```
ros2 topic echo /ap/imu/experimental/data
```
You can also start Mavproxy on the host (not in the Docker container)
```
mavproxy.py --master=udp:192.168.13.16:14550
mavproxy.py --master=udp:192.168.13.16:14550 --out=udpbcast:192.168.1.255:14550
mavproxy.py --master=udp:192.168.13.16:14550 --out=udpbcast:192.168.8.255:14550
```
Or preferably `mavp2p`, which opens mavlink endpoints and connects them.
```
mavp2p udps:192.168.13.16:14550 udpb:192.168.1.255:14550
mavp2p --streamreq-disable udps:192.168.13.16:14550 udpb:192.168.1.255:14550
```
Example if another local endpoint is required:
```
mavp2p --streamreq-disable udps:192.168.13.16:14550 udpb:192.168.1.255:14550 udps:0.0.0.0:14551
```

## 7 - Updating Repos and Dependencies
On the host:
```
cd ~/ardupilot
git pull --recurse-submodules
```
```
cd ~/Micro-XRCE-DDS-Gen
git pull --recurse-submodules
./gradlew assemble
```
For updating the repos inside the docker, it is best to rebuild the image from scratch.
```
docker build --no-cache -t ros2_rpi4 .
```

## 8 - Forward Webserver to Other PC

Edit `/etc/sysctl.conf` and uncomment
```
net.ipv4.ip_forward=1
```
Create routing tables
```
sudo iptables -P FORWARD ACCEPT
sudo iptables -t nat -A PREROUTING -p tcp --dport 8080 -j DNAT --to-destination 192.168.13.65:8080
sudo iptables -t nat -A POSTROUTING -d 192.168.13.65 -j MASQUERADE
```
Use `iptables-persistent` to keep routing permanent between reboots
```
sudo apt install iptables-persistent
sudo netfilter-persistent save
```
Now, the webserver is accessible from any device on the same network as the Pi, at the address `<Pi address on network>:8080`.
Port 8080 is necessary instead of 80, since Docker requires port 80 for accessing the internet when building images.

## 9 - Switch to Preferred Wifi Automatically
```
sudo apt install network-manager
sudo nmcli connection add type wifi ifname "*" con-name ROS2_Network ssid ROS2_Network
sudo nmcli connection modify ROS2_Network wifi-sec.key-mgmt wpa-psk wifi-sec.psk "po5i7ion!"
sudo nmcli connection modify ROS2_Network connection.autoconnect yes
sudo nmcli connection modify ROS2_Network connection.autoconnect-priority 10
nmcli connection show ROS2_Network
```
On your `ROS2_Network` network router, you can view the Pi's IP address or reserve an address for it. In my case, I reserved 192.168.8.100.

Create shell script
```
sudo nano /usr/local/bin/preferred_wifi_switch.sh
```
And add
```
#!/bin/bash

# Define the preferred Wi-Fi network
PREFERRED="ROS2_Network"
DEBUG=true
dbg() {
    if [ "$DEBUG" = true ]; then
        echo "$1"
    fi
}

while true; do
    # Get the name of the currently active connection
    CURRENT=$(nmcli -g NAME,DEVICE con show --active | grep "$(nmcli -t -f DEVICE,TYPE device | grep ':wifi' | cut -d: -f1)" | cut -d: -f1)

    # Check if the current network is not the preferred network
    if [ "$CURRENT" != "$PREFERRED" ]; then
        dbg "Currently connected to $CURRENT"

        # Check if the preferred network is available
        if nmcli -g SSID device wifi list | grep -Fqwx "$PREFERRED"; then
            dbg "Attempting switch to $PREFERRED"
            nmcli c up "$PREFERRED"
        fi
    fi

    # Wait 20 seconds before checking again
    sleep 20
done
```
Make it executable
```
sudo chmod +x /usr/local/bin/preferred_wifi_switch.sh
```

Create service file
```
sudo nano /etc/systemd/system/wifi-switch.service
```
And add
```
[Unit]
Description=Wi-Fi Auto Switch Service
After=network.target

[Service]
ExecStart=/usr/local/bin/preferred_wifi_switch.sh
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```
```
sudo systemctl enable wifi-switch.service
sudo systemctl start wifi-switch.service
```

## 10 - Run PPPD at Boot
Create `pppd` parameter file
```
sudo nano /etc/ppp/ip_config.txt
```
And add
```
# /etc/ppp/ip_config.txt
LOCAL_IP=192.168.13.16
REMOTE_IP=192.168.13.65
BAUD=12500000
```
Create shell script that runs `pppd`
```
sudo nano /usr/local/bin/start_pppd.sh
```
And add
```
#!/bin/bash

# Read the IPs from the configuration file
source /etc/ppp/ip_config.txt

# Start pppd with the IPs 
# (pppd is in /usr/local/sbin/ if 2.5.1 manually compiled, otherwise in /usr/sbin/)
/usr/local/sbin/pppd /dev/ttyAMA0 $BAUD $LOCAL_IP:$REMOTE_IP crtscts debug noauth nodetach local proxyarp ktune
```
Make it executable
```
sudo chmod +x /usr/local/bin/start_pppd.sh
```
Make `pppd` service file
```
sudo nano /etc/systemd/system/pppd.service
```
And add
```
[Unit]
Description=PPP Daemon Service
After=network.target

[Service]
Type=simple
ExecStart=/usr/local/bin/start_pppd.sh
ExecStop=/bin/kill $MAINPID
Restart=on-failure
RestartSec=5s

[Install]
WantedBy=multi-user.target
```
Enable and start service
```
sudo systemctl daemon-reload
sudo systemctl enable pppd.service
sudo systemctl start pppd.service
```
Other commands:
```
sudo systemctl status pppd.service
sudo systemctl restart pppd.service
sudo systemctl stop pppd.service
sudo systemctl disable pppd.service
```

## 11 - Github SSH Key
```
ssh-keygen -t ed25519 -C "john.bobzwik@gmail.com"
eval "$(ssh-agent -s)"
ssh-add ~/.ssh/id_ed25519
```
Then copy output of
```
cat ~/.ssh/id_ed25519.pub
```
