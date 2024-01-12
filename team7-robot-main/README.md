# team7-robot


## Prerequisites

To use the ROS packages in `catkin_ws`, you'll need ROS and wiringPi installed. 

**How to install ROS Melodic on Raspberry Pi Ubuntu 18.04:**

- Download Ubuntu 18.04 for Raspberry Pi https://cdimage.ubuntu.com/releases/18.04/release/ubuntu-18.04.5-preinstalled-server-armhf+raspi4.img.xz
- Download Balena Etcher https://www.balena.io/etcher/
- Flash the downloaded image onto the desired SD card using Balena Etcher
- Follow [ROS installation instructions](http://wiki.ros.org/Installation/Ubuntu)

```
sudo systemctl disable apt-daily.service
sudo systemctl disable apt-daily.timer
sudo systemctl disable apt-daily-upgrade.timer
sudo systemctl disable apt-daily-upgrade.service
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -
sudo apt update
sudo apt upgrade
sudo apt install -y ros-melodic-ros-base
sudo apt install -y python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
```

**How to install wiringPi on Raspberry Pi Ubuntu 18.04:**

```
sudo apt-get purge wiringpi
hash -r
git clone https://github.com/WiringPi/WiringPi.git
cd WiringPi
git pull origin
./build
sudo apt install rpi.gpio-common
sudo adduser "${USER}" dialout
sudo reboot
```

## Installation

- Create and build a local catkin workspace:

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```
- Copy this repo's `catkin_ws/src` to your local `catkin_ws/src`

```
catkin_make
source devel/setup.bash
```

Now you should be able to use the battery monitors.

## Usage

`ltc2943_battery_monitor` creates a publisher node that publishes a `battery_level` message to the `ltc2943_battery_level` topic.

Start the node with 

```
rosrun ltc2943_battery_monitor ltc2943_battery_monitor
```

`raspi_battery_monitor` creates a publisher that publishes a `battery_level` message to the `raspi_battery_level` topic.

Start the node with

```
rosrun raspi_battery_monitor raspi_battery_monitor
```
