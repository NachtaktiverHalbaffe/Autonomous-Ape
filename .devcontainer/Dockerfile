FROM ros:humble
ARG USERNAME=nachtaktiverhalbaffe
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
	&& useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
	#
	# [Optional] Add sudo support. Omit if you don't need to install software after connecting.
	&& apt-get update \
	&& apt-get install -y sudo \
	&& echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
	&& chmod 0440 /etc/sudoers.d/$USERNAME
# Comment below command if buildx is not installed or enabled
RUN --mount=target=/var/lib/apt/lists,type=cache,sharing=locked \
	--mount=target=/var/cache/apt,type=cache,sharing=locked \
	rm -f /etc/apt/apt.conf.d/docker-clean\
	&& apt-get update && apt-get upgrade -y \
	&& apt-get install -y --no-install-recommends python3-pip \
	doxygen \
	# Install turtlebot4 dependencies
	ros-humble-turtlebot4-description \
	ros-humble-turtlebot4-msgs \ 
	ros-humble-turtlebot4-navigation \ 
	ros-humble-turtlebot4-node \
	ros-humble-turtlebot4-simulator
# Install rosdoc2
RUN cd /home && git clone https://github.com/ros-infrastructure/rosdoc2 \
	&& pip3 install -e /home/rosdoc2
ENV SHELL /bin/bash

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************

# [Optional] Set the default user. Omit if you want to keep the default as root.
RUN echo 'source /opt/ros/humble/setup.bash' >> /home/.bashrc
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/.bashrc

USER $USERNAME

CMD ["/bin/bash"]
