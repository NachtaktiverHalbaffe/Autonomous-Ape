# Sopias4 Framework <!-- omit in toc -->
This package extends the ROS2 framework with additional tools, definitions and classes specific for this Sopias4 project. 

## Table of Contents <!-- omit in toc -->
- [Overview](#overview)
- [Usage](#usage)
  - [Plugin Bridges](#plugin-bridges)
  - [Nodes](#nodes)
  - [Launch files](#launch-files)
  - [Tools](#tools)
    - [Development tools](#development-tools)
- [License](#license)

# Overview
This package extends the ROS2 framework with additional tools, definitions and classes specific for this Sopias4 project. 

It provides all necessary parts for the plugin bridges for the Navigation2 package because it's plugins can originally be only written in C++, so we have to utilize ROS2 services and actions to enable Python usage for these plugins.

It also provides basic ROS2 Nodes which can be extended for the Sopias4 usecases. In specific, it provides a basic node for the gui which can be used to develop the Pyqt5 plugin. The other nodes are the corresponding Python nodes for the Navigation2 plugins in which the Python implementation of the plugins can be done.

It also provide various tools provided in a library manner which are loosely organized. Read the docs for a complete list and details.

# Usage
## Plugin Bridges
When using a specific Python Bridge, it has to be configured in the Navigation2 configuration:
<!-- TODO provide example configuration -->

It uses ROS2 Services and Actions for bridging. The Nodes package provides Python plugin nodes which already implemented these Actions and Services and you only need to override the plugin-specific implementation methods which you want to extend e.g. the path planning. Following, a table with the available Python Bridges is given. Note that the PyPlugin counterparts are included in the nodes module.

| **Plugin Bridge**          | **Corresponding Python node** | **Methods to override**                        | **Quality Scale**          |
| -------------------------- | ----------------------------- | ---------------------------------------------- | -------------------------- |
| Layer Plugin PyBridge      | Layer PyPlugin                | update_costs()                                 | Implementend Plugin-Bridge |
| Planner Plugin PyBridge    | Planner PyPlugin              | create_plan()                                  | Implementend Plugin-Bridge |
| Controller Plugin PyBridge | Controller PyPlugin           | compute_velocity_commands(), set_speed_limit() | Implementend Plugin-Bridge |

<!-- TODO provide example configuration of plugins  -->
## Nodes
As mentioned in the [PluginBridges section](#pluginbridges) the Python Plugin Nodes are included in this module and there you can read for further implementation details. 

Also there a base GUI Node is provided which you can use to build your own PyQT GUI. Following, the usage for this is described.  
For generating a GUI, create a UI file with QT Designer with which you can design the GUI graphically. For this purpose, you can run `designer` in the terminal to open QT-Designer from the Dev-Container environment, otherwise you can install it on your host OS and use it there. After that, save the UI-File in the assets folder in Sopias4 Application and run the script `python3 generate_ui.py -i <path to ui file> -o <path to directory where the generated Python class should be saved>` which converts it to a Python class. The script is located in `sopias4_framework/tools/scripts` in this ROS2-package. After that, you can inherit from the GUI Node and do the necessary steps:

```Python
class YourImplementationClass(GUINode):

    def __init__(self) ->None:
        # The script should name the Python object this way, but can vary. Doublecheck the class name
        # within the generated Python file to be sure
        self.ui: Ui_MainWindow # To enable auto-completion in IDE
        ui_file = Ui_MainWindow() 
        super().__init(self, ui_file)

    def connect_callbacks(self):
        # You need to overrride this method. Connect your UI elements with the callacks here.
        # The ui elements are in self.ui field of the GUINode class
        self.ui.example_button.clicked.connect(lambda: Thread(target=self.__foobar).start())

    def set_default_values(self):
        # You need to override this method. Set default values or default choices of your ui elements here
        self.ui.example_textbox.setText("Hello World")
    
    def set_initial_enabled_elements(self):
        # Disable the desired elements here
        self.ui.pushButton_example.setEnabled(False)

    def connect_labels_to_subscriptions(self):
        # CReate and connect subscriptions here
        gui_logger = GuiLogger(self.ui.textEdit)
        self.node.create_subscription(Log, "/rosout", gui_logger.add_log_msg, 10)
        
    def __foobar():
        print("Hello World!")
```

Following methods needs to be overriden (look further into documentation for more details):
  - `connect_callbacks()`
  - `set_default_values()`
  - `set_initial_enabled_elements()`
  - `connect_labels_to_subscriptions()`

It also has builtin methods which you can use as callback function for certain tasks in Sopias4 (look further into documentation for more details):
  - `register_namespace()`: Register namespace in Sopias4 Map-Server
  - `launch_robot()`: Start all the nodes in Sopias4 Application which are interacting with the Turtlebot
  - `stop_robot()`: Stops all the nodes in Sopias4 Application which are interacting with the Turtlebot
  - `start_mapping()`: Starts the mapping process
  - `stop_mapping()`: Stops the mapping process and saves the map on the Sopias4 Map-Server
  - `drive()`: Let the robot execute a specific drive command

It provides following ROS2-services:
  - `show_dialog`: Show a dialog with the users. Also interaction options can be specified

## Launch files
The necessary parts can be run utilizing ROS2 launchfiles. It can be launchend by running the command `ros2 launch sopias4_framework <launchfile> <launchfile arguments>` in the terminal. The launch arguments are provided as keyword-named values e.g. `namespace:="ape"`. A launch command could look like the following: `ros2 launch sopias4_framework bringup_turtlebot.launch.py namespace:=ape use_simulation:=true`. The node `robot_manager` provides services to launch specific nodes. These services also utilize this launch files under the hood. 

Sopias4-Framework provides following launch-files which can be uses:
- `nav2.launch.py` launches the Navigation2 navigation stack. It has following launch arguments:
    - `use_sim_time` (bool): If the simulation time from gazebo should be used (necessary if Turtlebot 4 simulation is used). Defaults to false
    - `namespace` (string): The namespace under which the navigation stack should be launched. Needed in multi roboter environment. Defaults to ""
    - `params_file` (string): Full path to the configuration file of the Navigation2 stack.  Defaults to the default config file
- `amcl.launch.py` launches the AMCL node. It has following launch arguments:
    - `use_sim_time` (bool): If the simulation time from gazebo should be used (necessary if Turtlebot 4 simulation is used). Defaults to false
    - `namespace` (string): The namespace under which the node should be launched. Needed in multi roboter environment. Defaults to ""
    - `params_file` (string): Full path to the configuration file of AMCL.  Defaults to the default config file
    - `use_respawn` (bool): Whether to respawn if a node crashes. Defaults to false
- `rviz.launch.py` launches RViz2 to visualize the system. It has following launch arguments
    - `use_sim_time` (bool): If the simulation time from gazebo should be used (necessary if Turtlebot 4 simulation is used). Defaults to false
    - `namespace` (string): The namespace under which Rviz2 should be launched. Needed in multi roboter environment. Defaults to ""
    - `params_file` (string): Full path to the configuration file of the Rviz2 visualization. Defaults to the default config file
    - `use_description` (bool): If turtlebot4 description should also be launched. Defaults to false
- `bringup_turtlebot.launch.py` launches the before mentioned launch files => Connects to the robot. It has the following launch arguments:
    - `use_simulation` (bool): If the Gazebo Ignition simulation of the Turtlebot4 should be used. Defaults to false
    - `namespace` (string): The namespace under which Rviz2 should be launched. Needed in multi roboter environment. Defaults to ""
- `bringup_slam.launch.py` lauches the SLAM nodes for the mapping process. It has the following launch arguments:
    - `use_sim_time` (bool): If the simulation time from gazebo should be used (necessary if Turtlebot 4 simulation is used). Defaults to false
    - `namespace` (string): The namespace under which Rviz2 should be launched. Needed in multi roboter environment. Defaults to ""
    - `sync` (bool): If synchronous SLAM should be used. Defaults to true
    - `params` (string): Full path to the configuration file of the SLAM node. Defaults to the default config file
## Tools
This module provides miscellaneous scripts, functions, helpers etc. which are loosely organized and doesn't fit in the other modules. These are loosely organized by their purpose. Following, a brief overlook is given.

### Development tools
These tools are mainly Python Scripts which should support the development process. It provides help for the development and arent used in production code. These are thje provided tools:
- `generate_docs`: It generates the documentation for the Sopias4 Framework by parsing the comments of the code e.g. Docstrings. Usage: `python3 generate_docs.py -p  <path to ROS2-package>`
- `generate_ui`: It generates the Python class from a given Ui file for the Sopias4 Framework with the help of PyQt5. Usage: `generate_ui.py -i <inputfile> -o <path of outputfile> -n <name of outputfile>`
<!-- TODO Keep up to date -->

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