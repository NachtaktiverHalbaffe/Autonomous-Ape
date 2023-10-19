# Running Sopias4
The Sopias4 system consists of Sopias4 Fleetbroker which should be run centrally one time and Sopias4 Application which is run as many times as individual Turtlebot's are running. 

There are three ways of running a these applications:
1. Using launch files from the terminal
2. Using alias commands in the terminal
3. Using (and programming) a GUI

No matter which of these methods is used, there are some steps that must be done for properly running the Sopias4 system:
1. The selected namespace must be registered. For this purpose, Sopias4 Fleetbroker must be running and the `register_namespace` service must be called. If this isn't done, the information of the robot isn't shared with other robots and namespace collisions could happen
2. If the namespace is successfully registered, the Robot Manager (and the GUI node) must be started in the right namespace
3. After that, everything can be started normally with the services which the robot manager offers. If you start nodes etc. without the robot manager, then make sure that they run in the right namespace

Following, a overview and corresponding information for each method for running the applications is given.

## Launch files
Launch files compose multiple Nodes and other launch files together which then can be launched together with one command. Here only the usage  is explained. If you want to create your own launch file, look at corresponding tutorials e.g. [https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Creating-Launch-Files.html](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Creating-Launch-Files.html).

For running a launch file, use the command `ros2 launch <ROS2 package> <launchfile> <launchfile arguments>`  in the terminal, e.g. `ros2 launch sopias4_framework bringup_nav2_stack.launch.py namespace:=/turtle1`. Following, a overview with the most important is given:

| **Launch file**                       | **ROS2 package**    | **Description**                                                                                                                  | **Arguments**                                                                                                                                                                                                                                                                                                    |
| ------------------------------------- | ------------------- | -------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| bringup_nav2_stack.launch.py          | sopias4_framework   | Launches the complete autonomous navigation stack (Navigation2, localization and visualization)                                  | `use_simulation`: If a Gazebo simulation  should be launched (deprecated) <br> `namespace`: The namespace of the  Turtlebot which should  be controlled <br> `log_level`: The logging level of all nodes                                                                                                         |
| bringup_test_system_planner.py.launch | sopias4_framework   | Launches a test system which sends test requests to a running PlannerPyPlugin to test it without the complete Nav2 Stack running | `plugin_name`: The name of the planner plugin which should be tested <br> `namespace`: The namespace of the  Turtlebot which should  be controlled <br> `log_level`: The logging level of all nodes                                                                                                              |
| bringup_slam.launch.py                | sopias4_framework   | Launches the mapping process                                                                                                     | Nothing important, just arguments for advanced usage                                                                                                                                                                                                                                                             |
| bringup_fleetbroker.launch.py         | sopias4_fleetbroker | Launches all central components of Sopias4 Fleetbroker                                                                           | `use_domain_bridge`: If Sopias4 DomainBridge  should be launched which bridges communication between different ROS domain IDs <br> `params_file`: Path to custom map_server configuration if if custom one should be used <br> `map`: The path to the static map (YAML file) if custom should be used at startup |

There are also launch files which are part of the above mentioned and can be used to launch parts of them individually:
| **Launch file**      | **ROS2 package**    | **Part of launchfile**        | **Description**                                             | **Arguments**                                               |
| -------------------- | ------------------- | ----------------------------- | ----------------------------------------------------------- | ----------------------------------------------------------- |
| nav2.launch.py       | sopias4_framework   | bringup_nav2_stack.launch.py  | Launches Navigation2                                        | `params_file`: The full path to the nav2 configuration file |
| amcl.py.launch       | sopias4_framework   | bringup_nav2_stack.launch.py  | Launches the localization                                   | Nothing important, just arguments for advanced usage        |
| rviz.launch.py       | sopias4_framework   | bringup_nav2_stack.launch.py  | Launches the visualization                                  | Nothing important, just arguments for advanced usage        |
| map_server.launch.py | sopias4_fleetbroker | bringup_fleetbroker.launch.py | Launches the central map server which serves the static map | `params_file`: Path to custom Rviz configuration file       |

If you want all arguments for a specific launch file, then you can run  `ros2 launch <ROS2 package> <launchfile> --show-args` 

## Using alias command in terminal
If you are using the development container/Docker image, then alias (shortcuts for these commands) for the most common used launch files and applications are implemented. Following a overview is given:

| **Alias command**           | **Description**                                                       | **Corresponding terminal command**                                    |
| --------------------------- | --------------------------------------------------------------------- | --------------------------------------------------------------------- |
| sopias4-application         | Launches Sopias4 Application (GUI)                                    | `ros2 run sopias4_application gui`                                    |
| sopias4-fleetbroker         | Launches Sopias4 Fleetbroker (GUI)                                    | `ros2 run sopias4_fleetbroker gui.py`                                 |
| sopias4-testsystem-planner  | Launches a test system to test PlannerPyPlugin                        | `ros2 launch sopias4_framework bringup_test_system_planner.launch.py` |
| sopias4-testrequest-planner | Sends a service request to the PlannerPyPlugin which should be tested | `ros2 service call /send_test_request std_srvs/srv/Empty`             |

## Using (and programming) a GUI
When you program a GUI, then you have essentially two ways: Using the `GUINode` or using the `LaunchService`, both provided by the Sopias4 Framework. For detailed information look at the documentation of these classes. Here only a short introduction is given.

### Using GUINode
The GUINode provides two ways of launching nodes: You can load the Python Nodes into the executor and run it or you can use the provided, builtin function for launching the selected launch files. The last ones can be found in the corresponding documentation of the GUINode class.

For loading nodes into the path, there is one restriction: The node class must be programmed in Python and importable as a Python class. Thus this only works for individual nodes and not for launch files. The methods for this are named `load_ros2_node` or `unload_ros2_node` if you want to stop it.

### Using the LaunchService
This class is provided by the Sopias4-Framework which can run each node and launch file which is installed on the system. It basically runs the corresponding terminal commands in a own shell process.

## Run nodes from a specific ROS Domain ID
Sometimes you want to run your ROS systems in another ROS Domain ID which isnt set by default. For this purpose, you can run `export ROS_DOMAIN_ID=<your desired id>` inside the terminal and all following nodes which are launched from that terminal session are run inside this domain. Note: The change only applies to the used terminal session. If you open a new terminal session, then you have to rerun this step.