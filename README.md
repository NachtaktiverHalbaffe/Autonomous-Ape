# Workspace Overview
There are three ROS2 packages within the workspace:
-  [Sopias4 Framework](#sopias4-framework): The Framework which includes all Tools which you need to develop the Sopias4-Application. This package doesn't need to be modified by you
-  [Sopias4 Map-Server](#sopias4-map-server): The central map server which also includes the multi robot coordinator. Under normal circumstances, this package doesn't need to be modified and must only be run on one instance
-  [Sopias4 Application](#sopias4-application): This is the main  package where you build your application. Here you utilize the Sopias4 Framework to build the neccessary parts for navigation

In the following subsections the components are introduced in more details.

## Sopias4 Framework
TODO Short overview of Framework

More detailed information can be found in the README.md of the corresponding package

## Sopias4 Map-Server
TODO Short Overview of map server

More detailed information can be found in the README.md of the corresponding package

## Sopias4 Application
TODO Short task description of application

More detailed information can be found in the README.md of the corresponding package

# Installation
## Running in Docker using Dev Containers (Visual Studio Code)
Using Visual Studio Code and Docker Containers will enable you to run your favorite ROS 2 Distribution without the necessity to change your operating system or use a virtual machine. This workspace provides a nearly finished configuration, but if you want to replicate it by your own (with a little different configuration) a tutorial can be found here: https://docs.ros.org/en/iron/How-To-Guides/Setup-ROS-2-with-VSCode-and-Docker-Container.html

If you want to use the pre-configured Dev Container, then the configuration is found in `.devcontainer/`. This also installs all required packages automatically on setup. However, a few steps are still needed:
1. Install `docker`  (follow the guide of your corresponding Linux distribution) **and** `docker-buildx` for build proper build caching  (If buildx isn't available/wished, then remove lines 15-18 from `.devcontainer/Dockerfile` )
2. Add your current linux user which is used inside the container to the docker group:
    ```bash
    sudo groupadd docker
    sudo usermod -aG docker $USER
    newgrp docker
    ```
3. Modify `.devcontainer/devcontainer.json`: Replace `nachtaktiverhalbaffe` with your Linux username. If you do not know your username, you can find it by running `echo $USERNAME` in the terminal
4. Repeat last step in `.devcontainer/Dockerfile`
5. Open Command Palette (either use `View->Command Palette...` or `Ctrl+Shift+P`), then search and/or run command `Dev Containers: (Re-)build and Reopen in Container`

## Run directly on host
Follow https://docs.ros.org/en/iron/Installation.html. Remind that the guides in this repository where written for Linux (Ubuntu), so if you are using another OS you may need to adapt a few commands.

You also need to install these packages to run the prototype:
- [Turtlebot4 Packages](https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_common.html)
- python3-pip
- If you want to use the Gazebo simulation: [Turtlebot4 Simulator](https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_simulator.html)
- If documentation generation is wanted: Install [rosdoc2](https://github.com/ros-infrastructure/rosdoc2):
    1. Clone repository and cd into it
    2. Run `pip3 install -e .`

# Run the stack 
TODO 

#  Development
TODO