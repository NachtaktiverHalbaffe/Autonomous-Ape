FROM ros:humble
ARG USERNAME= ros
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

# Install apps etc. outside ros dependencies
RUN --mount=target=/var/lib/apt/lists,type=cache,sharing=locked \
	--mount=target=/var/cache/apt,type=cache,sharing=locked \
	rm -f /etc/apt/apt.conf.d/docker-clean\
	&& sudo apt-get update && sudo apt-get install wget \
	&& apt-get update && apt-get upgrade -y \
	&& apt-get install -y --no-install-recommends python3-pip \
	doxygen figlet qttools5-dev-tools qttools5-dev python3-mock \ 
	libgl1-mesa-glx libgl1-mesa-dri mesa-utils 

# Install ros dependencies 
ARG RTI_NC_LICENSE_ACCEPTED=yes
RUN --mount=target=/var/lib/apt/lists,type=cache,sharing=locked \
	--mount=target=/var/cache/apt,type=cache,sharing=locked \
	rm -f /etc/apt/apt.conf.d/docker-clean\
	&& apt-get update && apt-get install -y --no-install-recommends\
	ros-humble-rmw-cyclonedds-cpp ros-humble-rmw-fastrtps-cpp \
	ros-humble-rmw-connextdds 

# Install turtlebot4 dependencies
RUN --mount=target=/var/lib/apt/lists,type=cache,sharing=locked \
	--mount=target=/var/cache/apt,type=cache,sharing=locked \
	rm -f /etc/apt/apt.conf.d/docker-clean\
	&& sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' \
	&& wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - \
	&& apt-get update  && apt-get install -y \
	ros-humble-turtlebot4-description \
	ros-humble-turtlebot4-msgs \ 
	ros-humble-turtlebot4-navigation \ 
	ros-humble-teleop-twist-keyboard \
	ros-humble-turtlebot4-simulator ignition-fortress \
	ros-dev-tools gazebo 

# Install rosdoc2
RUN cd /home && git clone https://github.com/ros-infrastructure/rosdoc2 

# Install pip3 deps
COPY ./requirements.txt /tmp/pip-tmp/requirements.txt
RUN pip3 install  -r /tmp/pip-tmp/requirements.txt \
	&& rm -rf /tmp/pip-tmp

######################################################################
# ------------------------------- Setup Environment ---------------------------------- 
######################################################################

# Setup right RMW implementation 
ENV RMW_IMPLEMENTATION rmw_fastrtps_cpp
# ENV FASTRTPS_DEFAULT_PROFILES_FILE home/ws/.devcontainer/fastrtps-profile.xml
# ENV RMW_IMPLEMENTATION rmw_cyclonedds_cpp
ENV RCYCLONEDDS_URI home/ws/.devcontainer/cyclonedds_pc.xml
RUN echo 'net.core.rmem_max=2147483647' >> /etc/sysctl.d/10-cyclone-max.conf

# Setup directory where log files are stored
ENV ROS_LOG_DIR /home/ws/log

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
RUN echo 'source /opt/ros/humble/setup.bash' >>  /home/${USERNAME}/.bashrc
RUN echo "source ${WORKSPACE}/install/setup.bash" >>  /home/${USERNAME}/.bashrc
# Setup some aliases
RUN echo 'alias sopias4-application="ros2 run sopias4_application gui.py"' >>  /home/${USERNAME}/.bashrc
RUN echo 'alias sopias4-mapserver="ros2 run sopias4_map_server gui.py"' >>  /home/${USERNAME}/.bashrc

# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME

CMD ["/bin/bash"]
