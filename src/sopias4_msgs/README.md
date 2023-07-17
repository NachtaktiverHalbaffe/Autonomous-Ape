# Sopias4 Messages
This package contains the message, services and actions definitions. As a result, this package defines the custom, eventdriven communication interfaces of Sopias4. On top the standard ROS2 messages, services and actions are used. Look athttps://github.com/ros2/common_interfaces to get more information about them.

# Overview
Here. a overview of each type of interfaces is given. Note that the services specific to the Python bridges are missing here because you dont need to use them.

## Messages
The message definitions are located in `msgs/`. There you can read the details of each implementation. 

| **Message type** | **Recommended topic name** | **Brief description**                                                                  |
| ---------------- | -------------------------- | -------------------------------------------------------------------------------------- |
| Robot            | -                          | The virtual identity of the Turtlebot which are necessary/helpful for the other robots |

## Services
The service definitions are located in `srv/`. There you can read the details of each implementation. Note that the services specific to the Python bridges are missing here because you dont need to use them.

| **Service name**             | **Service**         | **Brief description**                                                                                                                        |
| ---------------------------- | ------------------- | -------------------------------------------------------------------------------------------------------------------------------------------- |
| register_namespace           | RegistryRequest     | Register a Turtlebot on the Sopias4 Map-Server                                                                                               |
| unregister_namespace         | RegisteryRequest    | Unregisters a existing namespace in the Sopias4 Map-Server                                                                                   |
| get_robots                   | GetRobots           | Get a list of all registered Turtlebots                                                                                                      |
| get_robot_identity           | GetRobotIdentity    | Get one specific Turtlebot                                                                                                                   |
| get_namespaces               | GetNamespaces       | Get a list of all registered Namespaces. Can also be achieved with GetRobots, but there you have more overload due to more given information |
| set_robot_path               | SetRobotPath        | Sets the planned  global path of an Turtlebot on the Sopias4 Map server so the robots can use them for their own planning                    |
| /<*namespace*>/launch        | LaunchTurtlebot     | Connects the Sopias4 Application with the Turtlebots and starts all necessary nodes                                                          |
| /<*namespace*>/stop          | EmptyWithStatusCode | Disconnects the Sopias4 Application with the Turtlebots and stops all necessary nodes                                                        |
| /<*namespace*>/start_mapping | EmptyWithStatusCode | Starts the Mapping process                                                                                                                   |
| /<*namespace*>/stop_mapping  | StopMapping         | Stops the mapping process                                                                                                                    |
| /<*namespace*>/show_dialog   | ShowDialog          | Shows a dialog in the gui with which the user can interacted. Used to give user feedback and/or to interact with the user                    |
| /<*namespace*>/drive         | Drive               | Sends a drive command which the turtlebot executes until a new drive command is send                                                         |

## Actions
The action definitions are located in `actions/`. Note that the actions specific to the Python bridges are missing here because you dont need to use them.

## Services and Actions specific to PluginBridges
This is only needed if you want to create your own clients for your PluginBridges. Here, the services and actions are listed which you need to implement to your PluginBridge Client to make the bridges work. The general approach is to write a C++ plugin which utilizes services or actions in the necessary methods to offload the work to the client nodes which are implemented as nodes with the programming language of your choice. All neccessary C++ Plugin Bridges are written, but only for Planner and layer plugin are reference implementations given and thus the bridges tested, so expect some bugs when using the other plugin bridges.

When using the plugin bridge, it is important to provide an parameter file which gives the corresponding implementation name space. This is important if you operate more than one plugin of the same type at the same time, because then the implementation name space is important to correctly identifiy each bridge.

### Services
| **Service**                                              | **Service type**        | **Plugin Bridge**          | **Overrides Method**        | **Descriptiom**                                                                                                 |
| -------------------------------------------------------- | ----------------------- | -------------------------- | --------------------------- | --------------------------------------------------------------------------------------------------------------- |
| /<*namespace*>/<*plugin_name*>/update_costs              | UpdateCosts             | Layer Plugin PyBridge      | update_costs()              | Implementation pending                                                                                          |
| /<*namespace*>/<*plugin_name*>/create_plan               | CreatePlan              | Planner Plugin PyBridge    | create_plan()               | Implementation pending                                                                                          |
| /<*namespace*>/<*plugin_name*>/compute_velocity_commands | ComputeVelocityCommands | Controller Plugin PyBridge | compute_velocity_commands() | Implementation Pending                                                                                          |
| -                                                        | -                       | Navigator Plugin PyBridge  | get_default_bt_filepath()   | Implementation not useful, because every bridge needs own Action definition which cant be dynamically loaded in |
| -                                                        | -                       | Navigator Plugin PyBridge  | goal_received()             | Implementation not useful, because every bridge needs own Action definition which cant be dynamically loaded in |
| -                                                        | -                       | Navigator Plugin PyBridge  | goal_completed()            | Implementation not useful, because every bridge needs own Action definition which cant be dynamically loaded in |
| -                                                        | -                       | Navigator Plugin PyBridge  | get_name()                  | Implementation not useful, because every bridge needs own Action definition which cant be dynamically loaded in |
| -                                                        | -                       | BT Plugin PyBridge         | provide_ports()             | Implementation not useful, because behaviour plugin not implemented                                             |
| -                                                        | -                       | Behavior Plugin PyBridge   | on_run()                    | Implementation not useful, because every bridge needs own Action definition which cant be dynamically loaded in |

### Actions


# License
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