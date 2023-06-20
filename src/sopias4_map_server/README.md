# Sopias4 Map Server  <!-- omit in toc -->
This package provides a central map server and a central identity provider. It provides a static map to all connected robots, prohibits namespace conflicts and collects/provides the states of all robots. This package don't need to be modified an can be used as it is. Make sure only one instance is running on the whole network.

## Table of contents  <!-- omit in toc -->
- [Overview](#overview)
- [Running](#running)
- [License](#license)

# Overview
This package consists of two nodes: [map_server](https://github.com/ros-planning/navigation2/blob/main/nav2_map_server/README.md) from the ROS2 Ecosystem (Navigation2 to be specific) with the necessary configuration and Multi-Robot-Coordinator. The map-server provides a map over the map_server node to all Turtlebots 4 and can save and load maps during runtime over services. Additionally the map is provided.

The Multi-Robot-Coordinator is provided by the multi_robot_coordinator node. The lab course environment is using a multi roboter scenario, where up to 3 Turtlebots run in parallel. To avoid naming conflicts in the ROS2 topics, services and actions each Turtlebot uses it's namespace which is added as an prefix to all corresponding topics etc.. As a consequence, all namespaces must be unique and also be known to each other robot if they want to interact with each other. The multi_robot_coordinator ensures this uniqueness by letting all Turtlebot register it's namespaces before they start their technical system. In addition, each Turtlebot provides it's position, planned global path and ip adress to this node so each Turtlebot can utilize this information. This is used by e.g. the Costmaps of the navigation stack to mark the Turtlebots as obstacles and their paths as avoidable, but passable regions in the map.


# Running
The necessary parts can be run utilizing ROS2 launchfiles. Thus, this node can only be launched over the terminal. It can be launchend by running the command `ros2 run sopias4_map_server <launchfile> <launchfile arguments>` in the terminal. The available launch files are presented in more detail below.

First, `map_server` launches the map_server and multi_robot_coordinator node. It's the main launch file and usually you don't need the others.

<!-- TODO add other launchfiles if necessary -->

# License
<!-- TODO Licensing -->