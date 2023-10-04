# ROS2 messages & services
Here. a overview of each type of interfaces is given. Also, the standard ROS2 message and action definitions are used. A overview for the standard ROS2 interfaces is given in this [Github repository](https://github.com/ros2/common_interfaces).  Note that the services specific to the Python bridges are missing here because you don't need to use them.

## Message types
The message definitions are located in `msgs/` in the `sopias4_msgs` package. There you can read the details of each implementation. 

| **Message type** | **Brief description**                                                                  |
| ---------------- | -------------------------------------------------------------------------------------- |
| Robot            | The virtual identity of the Turtlebot which are necessary/helpful for the other robots |
| RobotStates      | A list of Robot                                                                        |

## Overview of most important topics
| **Topic**                    | **Message type** | **Description**                                                                                                                                                          |
| ---------------------------- | ---------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| /robot_states                | RobotStates      | The state of all registered robots in the system e.g. position                                                                                                           |
| /<*namespace*>/is_navigating | Bool             | If the Turtlebot is currently navigating. Its not precise, it only checks if the turtlebot has a global plan published and if the turtlebot is near the end of this path |


## Services
The service definitions are located in `srv/` in the `sopias4_msgs` package. There you can read the details of each implementation. Note that the services specific to the Python bridges are missing here because you don't need to use them.

| **Service name**                 | **Service**         | **Brief description**                                                                                                                        |
| -------------------------------- | ------------------- | -------------------------------------------------------------------------------------------------------------------------------------------- |
| register_namespace               | RegistryRequest     | Register a Turtlebot on the Sopias4 Map-Server                                                                                               |
| unregister_namespace             | RegistryRequest     | Unregisters a existing namespace in the Sopias4 Map-Server                                                                                   |
| get_robots                       | GetRobots           | Get a list of all registered Turtlebot's                                                                                                     |
| get_robot_identity               | GetRobotIdentity    | Get one specific Turtlebot                                                                                                                   |
| get_namespaces                   | GetNamespaces       | Get a list of all registered Namespaces. Can also be achieved with GetRobots, but there you have more overload due to more given information |
| /<*namespace*>/launch_nav2_stack | LaunchNav2Stack     | Connects the Sopias4 Application with the Turtlebot's and starts all necessary nodes                                                         |
| /<*namespace*>/stop              | EmptyWithStatusCode | Disconnects the Sopias4 Application with the Turtlebot's and stops all necessary nodes                                                       |
| /<*namespace*>/start_mapping     | EmptyWithStatusCode | Starts the Mapping process                                                                                                                   |
| /<*namespace*>/stop_mapping      | StopMapping         | Stops the mapping process                                                                                                                    |
| /<*namespace*>/show_dialog       | ShowDialog          | Shows a dialog in the gui with which the user can interacted. Used to give user feedback and/or to interact with the user                    |
| /<*namespace*>/drive             | Drive               | Sends a drive command which the turtlebot executes until a new drive command is send                                                         |

## Services and Actions specific to PluginBridges
This is only needed if you want to create your own clients for your PluginBridges. Here, the services and actions are listed which you need to implement to your PluginBridge Client to make the bridges work. The general approach is to write a C++ plugin which utilizes services or actions in the necessary methods to offload the work to the client nodes which are implemented as nodes with the programming language of your choice. All necessary C++ Plugin Bridges are written, but only for Planner and layer plugin are reference implementations given and thus the bridges tested, so expect some bugs when using the other plugin bridges.

When using the plugin bridge, it is important to provide an parameter file which gives the corresponding implementation name space. This is important if you operate more than one plugin of the same type at the same time, because then the implementation name space is important to correctly identify each bridge.

### Services
| **Service**                                              | **Service type**        | **Plugin Bridge**          | **Overrides Method**        | **Description**        |
| -------------------------------------------------------- | ----------------------- | -------------------------- | --------------------------- | ---------------------- |
| /<*namespace*>/<*plugin_name*>/update_costs              | UpdateCosts             | Layer Plugin PyBridge      | update_costs()              | Implementation pending |
| /<*namespace*>/<*plugin_name*>/create_plan               | CreatePlan              | Planner Plugin PyBridge    | create_plan()               | Implementation pending |
| /<*namespace*>/<*plugin_name*>/compute_velocity_commands | ComputeVelocityCommands | Controller Plugin PyBridge | compute_velocity_commands() | Implementation Pending |
