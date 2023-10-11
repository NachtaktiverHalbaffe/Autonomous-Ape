ARG ROS_DISTRO=humble

FROM ros:${ROS_DISTRO}
ARG USERNAME=ros2
ARG WORKSPACE
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
	&&useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
	# [Optional] Add sudo support. Omit if you don't need to install software after connecting.
	&& apt-get update \
	&& apt-get install -y sudo \
	&& echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
	&& chmod 0440 /etc/sudoers.d/$USERNAME

######################################################################
# --------------------- Installing packages & Dependencies ------------------------- 
######################################################################

# Install general apps, tools, dependencies etc.
RUN sudo apt-get update && apt-get upgrade -y --with-new-pkgs \ 
	&& sudo apt-get install -y\
	wget\ 
	python3-pip \
	doxygen\ 
	figlet\
	qttools5-dev-tools\
	qttools5-dev\
	python3-mock \ 
	libgl1-mesa-glx \
	libgl1-mesa-dri \
	mesa-utils  \
	chrony

# Install ROS2 dependencies 
ARG RTI_NC_LICENSE_ACCEPTED=yes
RUN apt-get update && apt-get install -y --no-install-recommends\
	ros-${ROS_DISTRO}-rmw-cyclonedds-cpp\
	ros-${ROS_DISTRO}-rmw-fastrtps-cpp \
	ros-${ROS_DISTRO}-rmw-connextdds 

# Install Turtlebot4 dependencies
RUN sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' \
	&& wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - \
	&& apt-get update  && apt-get install -y \
	ros-${ROS_DISTRO}-turtlebot4-description \
	ros-${ROS_DISTRO}-turtlebot4-msgs \ 
	ros-${ROS_DISTRO}-turtlebot4-navigation \ 
	ros-${ROS_DISTRO}-teleop-twist-keyboard \
	ros-${ROS_DISTRO}-turtlebot4-simulator\ 
	ros-${ROS_DISTRO}-domain-bridge \
	ignition-fortress \
	gazebo \
	ros-dev-tools

# Install rosdoc2
RUN cd /home && git clone https://github.com/ros-infrastructure/rosdoc2 

# Install Pip3 deps from requirements.txt file
COPY ./requirements.txt /tmp/pip-tmp/requirements.txt
RUN pip3 install  -r /tmp/pip-tmp/requirements.txt \
	&& rm -rf /tmp/pip-tmp

######################################################################
# ------------------------------- Setup Environment ---------------------------------- 
######################################################################

# Setup right RMW implementation 
ENV RMW_IMPLEMENTATION rmw_fastrtps_cpp
# ENV RMW_IMPLEMENTATION rmw_cyclonedds_cpp
ENV RCYCLONEDDS_URI ${WORKSPACE}/cyclonedds_pc.xml
RUN echo 'net.core.rmem_max=2147483647' >> /etc/sysctl.d/10-cyclone-max.conf

# Setup directory where log files are stored
ENV ROS_LOG_DIR ${WORKSPACE}/log

# For nvidia cards working right in docker
ENV NVIDIA_VISIBLE_DEVICES \
	${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
	${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}
ENV QT_X11_NO_MITSHM 1

ENV SHELL /bin/bash

######################################################################
# ---------------------------------- Setup bash rc ------------------------------------- 
######################################################################
# Happy Easter
RUN echo 'figlet "Linux is great" ' >> /home/${USERNAME}/.bashrc 
# Automatic sourcing of ROS2
RUN echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >>  /home/${USERNAME}/.bashrc
RUN echo "source ${WORKSPACE}/install/setup.bash" >>  /home/${USERNAME}/.bashrc
# Setup ntp server as source for time synchronization
COPY ./chrony.conf /etc/chrony/chrony.conf
EXPOSE 123/udp
COPY postCreate.sh /bin/postCreate.sh 
COPY postStart.sh /bin/postStart.sh
# Setup some aliases
RUN echo 'alias sopias4-application="ros2 run sopias4_application gui"' >>  /home/${USERNAME}/.bashrc
RUN echo 'alias sopias4-fleetbroker="ros2 run sopias4_fleetbroker gui.py"' >>  /home/${USERNAME}/.bashrc
RUN echo 'alias sopias4-testsystem-planner="ros2 launch sopias4_framework bringup_test_system_planner.launch.py"' >>  /home/${USERNAME}/.bashrc
RUN echo 'alias sopias4-testrequest-planner="ros2 service call /send_test_request std_srvs/srv/Empty"' >>  /home/${USERNAME}/.bashrc

######################################################################
# ------------------Install external deps of ROS2 packages --------------------------
######################################################################
# Copy workspace into container
WORKDIR ${WORKSPACE}/src
COPY ./ ./src
# Install ros dependencies
WORKDIR ${WORKSPACE}
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
	&& apt-get update -y \
	&& rosdep update \
	&& rosdep install --from-paths src --ignore-src  -y 
# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME

CMD ["/bin/bash"]