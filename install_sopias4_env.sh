#!/usr/bin/env bash
set -eu
echo "Installing ROS2"

ROS_DISTRO=humble
ROS_DOMAIN_ID=1
INSTALL_PACKAGE=desktop
TARGET_OS=jammy
USERNAME=ros2
WORKSPACE=/home/ros2/sopias4_ws

# Check OS version
if ! which lsb_release > /dev/null ; then
	sudo apt-get update
	sudo apt-get install -y curl lsb-release
fi

if [[ "$(lsb_release -sc)" == "$TARGET_OS" ]]; then
	echo "OS Check Passed"
else
	printf '\033[33m%s\033[m\n' "=================================================="
	printf '\033[33m%s\033[m\n' "ERROR: This OS (version: $(lsb_release -sc)) is not supported"
	printf '\033[33m%s\033[m\n' "=================================================="
	exit 1
fi

if ! dpkg --print-architecture | grep -q 64; then
	printf '\033[33m%s\033[m\n' "=================================================="
	printf '\033[33m%s\033[m\n' "ERROR: This architecture ($(dpkg --print-architecture)) is not supported"
	printf '\033[33m%s\033[m\n' "See https://www.ros.org/reps/rep-2000.html"
	printf '\033[33m%s\033[m\n' "=================================================="
	exit 1
fi

# systemd and udev-related packages needs to be updated
# ref: https://github.com/ros2/ros2_documentation/pull/2581
# ref: https://itsfoss.com/apt-get-upgrade-vs-dist-upgrade/
# ref: https://penpen-dev.com/blog/upgrade-tigai/
sudo apt-get update && sudo apt upgrade -y

# Install
sudo apt-get update
sudo apt-get install -y software-properties-common
sudo add-apt-repository -y universe
sudo apt-get install -y curl gnupg2 lsb-release build-essential

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt-get update
sudo apt-get install -y ros-$ROS_DISTRO-$INSTALL_PACKAGE
sudo apt-get install -y python3-argcomplete python3-colcon-clean
sudo apt-get install -y python3-colcon-common-extensions
sudo apt-get install -y python3-rosdep python3-vcstool # https://index.ros.org/doc/ros2/Installation/Linux-Development-Setup/
[ -e /etc/ros/rosdep/sources.list.d/20-default.list ] ||
sudo rosdep init
rosdep update

set +u
source /opt/ros/$ROS_DISTRO/setup.bash

######################################################################
# --------------------- Installing packages & Dependencies ------------------------- 
######################################################################
echo "Installing necessary packages"
# Install general apps, tools, dependencies etc.
apt-get upgrade -y 
apt install -y wget python3-pip doxygen figlet qttools5-dev-tools qttools5-dev python3-mock 
apt install -y libgl1-mesa-glx libgl1-mesa-dri mesa-utils chrony 
    
# Install Turtlebot4 dependencies
sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' 
wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add - && apt update  
apt install -y ros-$ROS_DISTRO-turtlebot4-description ros-$ROS_DISTRO-turtlebot4-msgs ros-$ROS_DISTRO-turtlebot4-navigation 
apt install -y ros-$ROS_DISTRO-teleop-twist-keyboard ros-$ROS_DISTRO-turtlebot4-simulator
apt install -y ros-$ROS_DISTRO-domain-bridge ignition-fortress gazebo ros-dev-tools

# Install rosdoc2
if cd /home/rosdoc2 ; then git pull; else git clone https://github.com/ros-infrastructure/rosdoc2 ; fi
cd $WORKSPACE
pip3 install -r  ./requirements.txt
# Update ROS-Dep
rosdep install --from-paths src --ignore-src  -y 
colcon build 

# Setup ntp server as source for time synchronization
echo "Setup NTP server/client"
cp $WORKSPACE/chrony.conf /etc/chrony/chrony.conf
systemctl restart chronyd

######################################################################
# ---------------------------------- Setup bash rc ------------------------------------- 
######################################################################
echo "Setup .bashrc"
# Happy Easter
grep -F 'figlet "Linux is great"' /home/$USERNAME/.bashrc ||
echo 'figlet "Linux is great"' >> /home/$USERNAME/.bashrc 
# Automatic sourcing of ROS2
grep -F "source /opt/ros/$ROS_DISTRO/setup.bash" /home/$USERNAME/.bashrc ||
echo "source /opt/ros/$ROS_DISTRO/setup.bash" >>  /home/$USERNAME/.bashrc
grep -F "source $WORKSPACE/install/setup.bash" /home/$USERNAME/.bashrc ||
echo "source $WORKSPACE/install/setup.bash" >>  /home/$USERNAME/.bashrc

# Setup some aliases
grep -F 'alias sopias4-application="ros2 run sopias4_application gui"' /home/$USERNAME/.bashrc ||
echo 'alias sopias4-application="ros2 run sopias4_application gui"' >>  /home/$USERNAME/.bashrc

grep -F 'alias sopias4-fleetbroker="ros2 run sopias4_fleetbroker gui.py"'  /home/$USERNAME/.bashrc ||
echo 'alias sopias4-fleetbroker="ros2 run sopias4_fleetbroker gui.py"' >>  /home/$USERNAME/.bashrc

grep -F 'alias sopias4-testsystem-planner="ros2 launch sopias4_framework bringup_test_system_planner.launch.py"'  /home/$USERNAME/.bashrc ||
echo 'alias sopias4-testsystem-planner="ros2 launch sopias4_framework bringup_test_system_planner.launch.py"' >>  /home/$USERNAME/.bashrc

grep -F 'alias sopias4-testrequest-planner="ros2 service call /send_test_request std_srvs/srv/Empty"' /home/$USERNAME/.bashrc ||
echo 'alias sopias4-testrequest-planner="ros2 service call /send_test_request std_srvs/srv/Empty"' >>  /home/$USERNAME/.bashrc

grep -F 'export DISPLAY=:0' /home/$USERNAME/.bashrc ||
echo 'export DISPLAY=:0' >>  /home/$USERNAME/.bashrc

grep -F 'export LIBGL_ALWAYS_SOFTWARE=1' /home/$USERNAME/.bashrc ||
echo 'export LIBGL_ALWAYS_SOFTWARE=1' >>  /home/$USERNAME/.bashrc

grep -F 'export LIBGL_ALWAYS_INDIRECT=0' /home/$USERNAME/.bashrc ||
echo 'export LIBGL_ALWAYS_INDIRECT=0' >>  /home/$USERNAME/.bashrc

grep -F "export ROS_DOMAIN_ID=$ROS_DOMAIN_ID" /home/$USERNAME/.bashrc ||
echo  "export ROS_DOMAIN_ID=$ROS_DOMAIN_ID"  >>  /home/$USERNAME/.bashrc

grep -F "export ROS_LOG_DIR=$WORKSPACE/log" /home/$USERNAME/.bashrc ||
echo  "export ROS_LOG_DIR=$WORKSPACE/log"  >>  /home/$USERNAME/.bashrc

grep -F "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" /home/$USERNAME/.bashrc ||
echo  "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp"  >>  /home/$USERNAME/.bashrc

echo "Setting permissions for workspace"
chown -R $USERNAME $WORKSPACE

echo "Installation complete. Open new terminal for the changes to take effect"