# Sopias4 Framework <!-- omit in toc -->
This package extends the ROS2 framework with additional tools, definitions and classes specific for this Sopias4 project. 

## Table of Contents <!-- omit in toc -->
- [Overview](#overview)
- [Usage](#usage)
  - [PyBridges](#pybridges)
  - [Nodes](#nodes)
    - [GUI-Node](#gui-node)
  - [Msgs](#msgs)
  - [Tools](#tools)
    - [Development tools](#development-tools)
- [License](#license)

# Overview
It provides all important message, service and action definitions, so that the ROS2 interfaces are standardized.

It also provides all necessary parts for the Python bridges for the Navigation2 package because it's plugins can originally be only written in C++, so we have to utilize ROS2 services and actions to enable Python usage for these plugins.

It also provides basic ROS2 Nodes which can be extended for the Sopias4 usecases. In specific, it provides a basic node for the gui which can be used to develop the Pyqt5 plugin. The other nodes are the corresponding Python nodes for the Navigation2 plugins in which the Python implementation of the plugins can be done.

It also provide various tools provided in a library manner which are loosely organized. Read the docs for a complete list and details.

# Usage
## PyBridges
When using a specific Python Bridge, it has to be configured in the Navigation2 configuration:
<!-- TODO provide example configuration -->

It uses ROS2 Services and Actions for bridging. The Nodes package provides Python plugin nodes which already implemented these Actions and Services and you only need to override the plugin-specific implementation methods which you want to extend e.g. the path planning. Following, a table with the available Python Bridges is given. Note that the PyPlugin counterparts are included in the nodes module.

| **Python Bridge**          | **Corresponding Python node** | **Methods to override**                                                  | **Quality Scale**      |
| -------------------------- | ----------------------------- | ------------------------------------------------------------------------ | ---------------------- |
| Layer Plugin PyBridge      | Layer PyPlugin                | update_costs()                                                           | Implementation pending |
| Planner Plugin PyBridge    | Planner PyPlugin              | create_plan()                                                            | Implementation pending |
| Controller Plugin PyBridge | Controller PyPlugin           | compute_velocity_commands(), set_speed_limit()                           | Implementation Pending |
| Navigator Plugin PyBridge  | Naviagtor PyPlugin            | get_default_bt_filepath(), goal_received(), goal_completed(), get_name() | Implementation Pending |
| BT Plugin PyBridge         | BT Action PyPlugin            | provide_ports()                                                          | Implementation pending |
| Behavior Plugin PyBridge   | Behavior PyPlugin             | on_run()                                                                 | Implementation pending |


## Nodes
As mentioned in the [PyBridges section](#pybridges) the Python Plugin Nodes are included in this module and there you can read for further implementation details. Also there a base GUI Node is provided which you can use to build your own PyQT GUI. Following, the usage for this is described.

### GUI-Node
For generating a GUI, create a UI file with QT Designer with which you can design the GUI graphically. For this purpose, you can run `designer` in the terminal to open QT-Designer from the Dev-Container environment, otherwise you can install it on your host OS and use it there. After that, save the UI-File in the assets folder in Sopias4 Application and run the script `python generate_ui.py -i <path to ui file> -o <path to directory where the generated Python class should be saved>` which converts it to a Python class. The script is located in `sopias4_framework/tools/scripts` in this ROS2-package. After that, you can inherit from the GUI Node and do the necessary steps:
```Python
class YourImplementationClass(GUINode):

  def __init__(self) ->None:
    # The script should name the Python object this way, but can vary. Doublecheck the class name
    # within the generated Python file to be sure
    ui_file = Ui_MainWindow() 
    super().__init(self, ui_file)

  def connect_callbacks(self):
    # You need to overrride this method. Connect your UI elements with the callacks here.
    # The ui elements are in self.ui field of the GUINode class
    self.ui.example_button.clicked.connect(self.__foobar)
  
  def set_default_values(self):
      # You need to override this method. Set default values or default choices of your ui elements here
      self.ui.example_textbox.setText("Hello World")

  def __foobar():
    print("Hello World!")
```

## Msgs
This package defines all messages, service and action definitions in addition to the ones which are available from ROS2.  The message definitions are located in `msgs/`, the service definitions in `srv/` and the action definitions under `action`. There you can read the details of each implementation. Following, only a brief overview is given. Note that the services and actions specific to the Python bridges are missing here because you dont need to use them.

**Messages**
| **Message type** | **Recommended topic name** | **Brief description**                                                                  |
| ---------------- | -------------------------- | -------------------------------------------------------------------------------------- |
| Robot            | -                          | The virtual identity of the Turtlebot which are necessary/helpful for the other robots |

**Services**
| **Service**      | **Recommended service name** | **Brief description**                                                                                                                        |
| ---------------- | ---------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------- |
| Register         | register                     | Register a Turtlebot on the Sopias4 Map-Server                                                                                               |
| GetRobots        | get_robots                   | Get a list of all registered Turtlebots                                                                                                      |
| GetRobotIdentity | get_robot_identity           | Get one specific Turtlebot                                                                                                                   |
| GetNamespaces    | get_namespaces               | Get a list of all registered Namespaces. Can also be achieved with GetRobots, but there you have more overload due to more given information |
| Connect          | connect                      | Connects the Sopias4 Application with the Turtlebots and starts all necessary nodes                                                          |
| Disconnect       | disconnect                   | Disconnects the Sopias4 Application with the Turtlebots and stops all necessary nodes                                                        |
| StartMapping     | start_mapping                | Starts the Mapping process                                                                                                                   |
| StopMapping      | stop_mapping                 | Stops the mapping process                                                                                                                    |


## Tools
This module provides miscellaneous scripts, functions, helpers etc. which are loosely organized and doesn't fit in the other modules. These are loosely organized by their purpose. Following, a brief overlook is given.

### Development tools
These tools are mainly Python Scripts which should support the development process. It provides help for the development and arent used in production code. These are thje provided tools:
- `generate_docs`: It generates the documentation for the Sopias4 Framework by parsing the comments of the code e.g. Docstrings
- `generate_ui`: It generates the Python class from a given Ui file for the Sopias4 Framework with the help of PyQt5
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