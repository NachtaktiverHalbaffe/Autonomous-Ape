# Autonomous Ape  <!-- omit in toc -->
This was my master thesis where a model process for a autonomous navigation was implemented. The main goal was to provide a model process which can be used in research and teaching. The Turtlebot4 is used. Because this process is used in a lab course where students rebuild this by their own, all names were changed to Autonomous Ape so student cant find this as a solution online easily.

## Table of Contents   <!-- omit in toc -->
- [Workspace Overview](#workspace-overview)
  - [Autonomous Ape Application](#autonomous-ape-applicatio)
  - [Autonomous Ape Framework](#autonomous-ape-framework)
  - [Autonomous Ape Fleetbroker](#autonomous-ape-fleetbroker)
  - [Autonomous Ape Messages](#autonomous-ape-messages)
- [Documentation and Guides](#documentation-and-guides)
- [License](#license)

# Workspace Overview
There are four ROS2 packages within the workspace (the other ones are just dependencies which aren't published in the online repositories of ROS):
-  [Autonomous Ape Application](#autonomous-ape-application): This is the main  package where you build your application. Here you utilize the Autonomous Ape Framework to build the necessary parts for navigation
-  [Autonomous Ape Framework](#autonomous-ape-framework): The Framework which includes all Tools which you need to develop the Autonomous Ape-Application. This package doesn't need to be modified by you
-  [Autonomous Ape Fleetbroker](#autonomous-ape-fleetbroker): The central map server which also includes the multi robot coordinator. Under normal circumstances, this package doesn't need to be modified and must only be run on one instance
-  [Autonomous Ape Messages](#autonomous-ape-messages): A package containing all custom ROS2 interface definitions

In the following subsections the components are introduced in more details.

## Autonomous Ape Application
This is the main application which the user uses to control the system and is the main goal of development. In this package you have to provide 4 elemental parts:
- Navigation2 plugins for autonomous operation, mainly path planning
- A GUI which is designed graphically with QT Designer and converted and interacted with PyQT5 (Use GUINode from Sopias4 Framework for this purpose)
- A technical interface between the user/GUI and the underlying ROS2 Nodes
- Also following ROS2 packages are used and need to be parameterized: SLAM Toolbox, Navigation2 and AMCL

More detailed information can be found in the README.md of the corresponding package

## Autonomous Ape Framework
This package extends the ROS2 framework with additional tools, definitions and classes specific for this Sopias4 project. 

It provides all necessary parts for the Python bridges for the Navigation2 package because it's plugins can originally be only written in C++, so we have to utilize ROS2 services and actions to enable Python usage for these plugins.

It also provides basic ROS2 Nodes which can be extended for the Sopias4 usecases. In specific, it provides a basic node for the gui which can be used to develop the PyQt5 plugin. The other nodes are the corresponding Python nodes for the Navigation2 plugins in which the Python implementation of the plugins can be done.

It also provide various tools provided in a library manner which are loosely organized. Read the docs for a complete list and details.

More detailed information can be found in the README.md of the corresponding package

## Autonomous Ape Fleetbroker
This package provides a central map server and a central identity provider. It provides a static map to all connected robots, prohibits namespace conflicts and collects/provides the states of all robots. This package don't need to be modified an can be used as it is. Make sure only one instance is running on the whole network.

More detailed information can be found in the README.md of the corresponding package

## Autonomous Ape Messages
This package provides all important message, service and action definitions, so that the ROS2 interfaces are standardized. Also, the standard ROS2 message and action definitions are used. A overview for the standard ROS2 interfaces is given in this [Github repository](https://github.com/ros2/common_interfaces). 

More detailed information can be found in the README.md of the corresponding package

# Documentation and Guides
The documentation and useful guides are provided inside the Autonomous Ape Framework package as HTML which can be opened locally by any modern browser. It is provided as an ZIP package on the Releases section on Github. It also contains the installation guide. 

Otherwise it can be generated from source. For this purpose run the `generate_docs.py` script inside of `sopias4_framework.tools.scripts`. Make sure you are running inside the Dev container/WSL to have all necessary dependencies installed.

To open it, locate and open the `index.html` file.

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
