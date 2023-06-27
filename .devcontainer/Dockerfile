FROM ros:humble
ARG USERNAME=nachtaktiverhalbaffe
ARG WORKSPACE
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
	&& useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
	# [Optional] Add sudo support. Omit if you don't need to install software after connecting.
	&& apt-get update \
	&& apt-get install -y sudo \
	&& echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
	&& chmod 0440 /etc/sudoers.d/$USERNAME

# Install apps etc. outside ros dependencies
RUN --mount=target=/var/lib/apt/lists,type=cache,sharing=locked \
	--mount=target=/var/cache/apt,type=cache,sharing=locked \
	rm -f /etc/apt/apt.conf.d/docker-clean\
	&& sudo apt-get update && sudo apt-get install wget \
	&& sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' \
	&& wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - \
	&& apt-get update && apt-get upgrade -y \
	&& apt-get install -y --no-install-recommends python3-pip \
	doxygen \
	figlet \
	qttools5-dev-tools 	qttools5-dev \
	python3-mock \ 
	libgl1-mesa-glx libgl1-mesa-dri mesa-utils

# Install turtlebot4 dependencies
RUN --mount=target=/var/lib/apt/lists,type=cache,sharing=locked \
	--mount=target=/var/cache/apt,type=cache,sharing=locked \
	rm -f /etc/apt/apt.conf.d/docker-clean\
	&& apt-get install -y --no-install-recommends \
	ros-humble-turtlebot4-description \
	ros-humble-turtlebot4-msgs \ 
	ros-humble-turtlebot4-navigation \ 
	ros-humble-turtlebot4-node \
	ros-humble-turtlebot4-simulator ignition-fortress \
	ros-dev-tools gazebo ros-humble-aws-robomaker-small-warehouse-world

# Install rosdoc2
RUN cd /home && git clone https://github.com/ros-infrastructure/rosdoc2 \
	&& pip3 install -e /home/rosdoc2 

ENV NVIDIA_VISIBLE_DEVICES \
	${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
	${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}
ENV QT_X11_NO_MITSHM 1

ENV SHELL /bin/bash

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************

# Happy Easter
RUN echo 'figlet "Linux is great" ' >> /home/nachtaktiverhalbaffe/.bashrc 
# Automatic sourcing of ROS2
RUN echo 'source /opt/ros/humble/setup.bash' >>  /home/nachtaktiverhalbaffe/.bashrc
RUN echo "source ${WORKSPACE}/install/setup.bash" >>  /home/nachtaktiverhalbaffe/.bashrc

# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME

CMD ["/bin/bash"]
