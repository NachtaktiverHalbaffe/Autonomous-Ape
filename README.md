# Sopias4   <!-- omit in toc -->
This is a workspace repository for Turtlebot4 implementation of the laboratory course for software engineering on the IAS of the University of Stuttgart.

## Table of Contents   <!-- omit in toc -->
- [Workspace Overview](#workspace-overview)
  - [Sopias4 Application](#sopias4-application)
  - [Sopias4 Framework](#sopias4-framework)
  - [Sopias4 Map-Server](#sopias4-map-server)
- [Installation](#installation)
  - [Setting up ROS2 workspace](#setting-up-ros2-workspace)
  - [Installing ROS2 and necesary applications](#installing-ros2-and-necesary-applications)
    - [Recommended: Running in Docker using Dev Containers (Visual Studio Code)](#recommended-running-in-docker-using-dev-containers-visual-studio-code)
    - [Run directly on host](#run-directly-on-host)
- [Running the system](#running-the-system)
- [License](#license)

# Workspace Overview
There are three ROS2 packages within the workspace:
-  [Sopias4 Application](#sopias4-application): This is the main  package where you build your application. Here you utilize the Sopias4 Framework to build the neccessary parts for navigation
-  [Sopias4 Framework](#sopias4-framework): The Framework which includes all Tools which you need to develop the Sopias4-Application. This package doesn't need to be modified by you
-  [Sopias4 Map-Server](#sopias4-map-server): The central map server which also includes the multi robot coordinator. Under normal circumstances, this package doesn't need to be modified and must only be run on one instance

In the following subsections the components are introduced in more details.

## Sopias4 Application
This is the main application which the user uses to control the system and is the main goal of development. In this package you have to provide 4 elemental parts:
- Navigation2 plugins for autonomous operation, mainly path planning
- A GUI which is designed graphically with QT Designer and converted and interacted with PyQT5 (Use GUINode from Sopias4 Framework for this purpose)
- A technical interface between the user/GUI and the underlying ROS2 Nodes
- Also following ROS2 packages are used and need to be parameterized: SLAM Toolbox, Navigation2 and AMCL

More detailed information can be found in the README.md of the corresponding package

## Sopias4 Framework
This package extends the ROS2 framework with additional tools, definitions and classes specific for this Sopias4 project. It provides all important message, service and action definitions, so that the ROS2 interfaces are standardized.

It also provides all necessary parts for the Python bridges for the Navigation2 package because it's plugins can originally be only written in C++, so we have to utilize ROS2 services and actions to enable Python usage for these plugins.

It also provides basic ROS2 Nodes which can be extended for the Sopias4 usecases. In specific, it provides a basic node for the gui which can be used to develop the Pyqt5 plugin. The other nodes are the corresponding Python nodes for the Navigation2 plugins in which the Python implementation of the plugins can be done.

It also provide various tools provided in a library manner which are loosely organized. Read the docs for a complete list and details.

More detailed information can be found in the README.md of the corresponding package

## Sopias4 Map-Server
This package provides a central map server and a central identity provider. It provides a static map to all connected robots, prohibits namespace conflicts and collects/provides the states of all robots. This package don't need to be modified an can be used as it is. Make sure only one instance is running on the whole network.

More detailed information can be found in the README.md of the corresponding package

# Installation
This splits up into setting up a ROS2 workspace for the Sopias4 project and installing ROS2. It's recommended to first setup the workspace and then setting up ROS2, because the workspace contains necessary configuration files if you want to use the Dev Containers and Docker for easy setup. 

## Setting up ROS2 workspace
Clone this repository. When using Linux, it is recommended to clone it into the home directory of your linux user so it matches with the commands in this guide.
```Bash
# If git isn't installed
sudo apt update && apt install git

cd /home/<your linux user>
git clone https://github.tik.uni-stuttgart.de/IAS/sopias4_ws # Make sure the URL is right, can vary 
```
After you have installed ROS2, you can run an initial build of this workspace:
```Bash
cd <path to your workspace>
colcon build
```

## Installing ROS2 and necesary applications 
### Recommended: Running in Docker using Dev Containers (Visual Studio Code)
Using Visual Studio Code and Docker Containers will enable you to run your favorite ROS 2 Distribution without the necessity to change your operating system or use a virtual machine. This workspace provides a nearly finished configuration, but if you want to replicate it by your own (with a little different configuration) a tutorial can be found here: https://docs.ros.org/en/iron/How-To-Guides/Setup-ROS-2-with-VSCode-and-Docker-Container.html

If you want to use the pre-configured Dev Container, then the configuration is found in `.devcontainer/`. This also installs all required packages automatically on setup. However, a few steps are still needed:
1. Install `docker`  **and** `docker-buildx` for build proper build caching  
    - For Ubuntu:
        ```Bash
        sudo apt-get update
        sudo apt-get install ca-certificates curl gnupg

        sudo install -m 0755 -d /etc/apt/keyrings
        curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
        sudo chmod a+r /etc/apt/keyrings/docker.gpg

        echo \
        "deb [arch="$(dpkg --print-architecture)" signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
        "$(. /etc/os-release && echo "$VERSION_CODENAME")" stable" | \
        sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

        sudo apt-get update
        sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
        ```
        You can verify your install by running
        ```bash
        sudo docker run hello-world
        ```
    - For other Linux distros or OS search for the corresponding guides
2. Add your current linux user which is used inside the container to the docker group:
    ```bash
    sudo groupadd docker
    sudo usermod -aG docker $USER
    newgrp docker
    ```
3. Modify `.devcontainer/devcontainer.json`: Replace `nachtaktiverhalbaffe` with your Linux username. If you do not know your username, you can find it by running `echo $USERNAME` in the terminal
4. Repeat last step in `.devcontainer/Dockerfile`
5. Open Command Palette (either use `View->Command Palette...` or `Ctrl+Shift+P`), then search and/or run command `Dev Containers: (Re-)build and Reopen in Container`

### Run directly on host
Follow https://docs.ros.org/en/iron/Installation.html and use the humble release. Remind that the guides in this repository where written for Linux (Ubuntu), so if you are using another OS you may need to adapt a few commands.

You also need to install these packages to run the prototype:
- [Turtlebot4 Packages](https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_common.html)
- python3-pip
- If you want to use the Gazebo simulation: [Turtlebot4 Simulator](https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_simulator.html)
- If documentation generation is wanted: Install [rosdoc2](https://github.com/ros-infrastructure/rosdoc2):
    1. Clone repository and cd into it
    2. Run `pip3 install -e .`

 You have to source ROS2 and the workspace everytime you open a terminal and your IDE needs to be started from a sourced terminal so you get all linter and syntax highlighting capabilities. For this you have to run these commands in the opened terminal:
```bash
source /opt/ros/humble/setup.bash
source source <path to ros workspace>/install/setup.bash
```
If you want this done automatically each time, when run this commands once:
```bash
echo "source /opt/ros/humble/setup.bash" >>/home/<your linux user>/.bashrc
echo "if [ -f <path to ros workspace>/install/setup.bash ]; then source <path to ros workspace>/install/setup.bash; fi" >> /home/<your linux user>/.bashrc
```

# Running the system
You can run the stack via launchfiles. These are located in the corresponding `launch/` directories. In the readme's of Sopias4 Application and Sopias4 Map-Server you can get more details of the provides launchfiles of these packages. Following, a short description of the most important launchfiles are given:
- `graphical_operation`: Used when operated with GUI. In the GUI you can launch the other neccessary parts
- `cli_operation`: Used when operated with the terminal. Note: You have to interact with the system via the corresponding services over the terminal
- `map_server`: Launches the Sopias4 Map-Server

<!-- TODO Add necessary launchfiles (not all) if new comes up -->

#  License
Copyright 2023 Institute of Industrial Automation and Software Engineering, University of Stuttgart

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.