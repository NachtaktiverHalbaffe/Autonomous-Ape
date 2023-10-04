# Using the PluginBridges
In Navigation 2 the plugins must be written in C++ because it utilizes the pluginlib which is specific for that language. However, in some usecases you want to write your plugins in another language. For this purpose the PluginBridges are provided.

It provides a plugin which you can configure into Navigation2 and utilizes ROS2 services which calls an external service which runs a implementation of the desired algorithm. This service can be run from another node with the programming language for your choice. However, these can only be utilized where the PluginBridge can be written in a general matter which is only possible for a costmap layer, the planner and the controller. All other plugins would require different C++ code for each PluginBridge for each different external plugin implementation.

## General usage
Each plugin needs a PluginBridge configured in Navigation 2 and an external running ROS2 node which runs this service. As a result, you have to make sure that this node is started somehow an running additional to the Nav2-stack.  


## Writing your own plugin implementations
For Python, there is already abstract classes called PyPlugins given in this framework which implements these things so you only need to implement your own algorithm. For this purpose you only need to inherit from these classes and override the needed functions.

For a plugin implementation in your specific language there are following conditions which needs to be met:
- There must be a ROS2 client library available, installed and running
- The ROS2 client library must be able to do: Running a node, running a service, generate service classes from service definitions
- https://docs.ros.org/en/rolling/Concepts/Basic/About-Client-Libraries.html lists a set of known client libraries and languages

If your programming language fulfill these conditions, then you have to implement the service for the corresponding service which runs under the right service name. In order to fulfill this, the services must be named the same on the PluginBridge and the plugin implementation. This is done by utilizing the plugin_name because the name is constructed after the following scheme: `/<namespace>/<plugin_name>/<service type in snake case>`.  You can look into the documentation of these PluginsBridge to get the specific naming convention for each specific bridge.

## PlannerBridge
The planner is responsible for planning the global path. Thus a path finding algorithm must be implemented.

For using the PluginBridge for this, it can be configured by adding following to your Navigation 2 configuration:
```YAML
planner_server:
    ros__parameters:
        plugins: ["GridBased"]
        GridBased:
            plugin: "plugin_bridges/PlannerBridge"
            plugin_name: "Astar"
            save_costmaps: True
            save_path: "/home/ws/"
```
You can set `save_costmaps` to true to save the sent service requests to JSON files under the path `save_path`. These can be used to get testing data for the [GlobalPlannerServer](#heading_target) which enables testing your planner implementation without needing the actual robot and the whole nav2 stack running.

In Python you can use the PlannerPyPlugin to implement the service. For this purpose, the `plugin_name` from the Navigation 2 configuration must match the one from the constructor. For further details, look into the documentation for this class. A implementation could look like the following:
```Python
class Astar(PlannerPyPlugin):

    def __init__(self, namespace: str | None = None) -> None:
        super().__init__(
            node_name="planner_astar", plugin_name="astar"
        ) if namespace is None else super().__init__(
            node_name="planner_astar", plugin_name="astar", namespace=namespace
        )
        self.enable_caching()
        self.get_logger().info("Started node")
        self.get_logger().set_level(30)

    def generate_path(
        self,
        start: Tuple[int, int],
        goal: Tuple[int, int],
        costmap: PyCostmap2D,
        goal_tolerance: float = 0.2,
    ) -> list[Tuple[int, int]]:
    # The algorithm here

def main(args=None):
    # Initialize node context
    rclpy.init(args=args)
    # Create ROS2 Node
    ros2_node = Astar()
    # Run node
    rclpy.spin(ros2_node)
    # Cleanup
    ros2_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```
As you can see, the PlannerPyPlugin has additionally a caching feature which returns a sliced version of the last planned path if it's suitable for the requested start and goal. This reduces oscillation in the planned paths and reduces computation time because the planning algorithm must only run when a complete new route is requested. It is enabled by default, but can be disabled. For further details, look into the documentation of the PlannerPyPlugin.

If you want to create your own plugin implementation, the service must be named under the following scheme: `/<namespace>/<plugin_name>/create_plan`. The corresponding service definition can be found in `sopias4_msgs/srv/CreatePlan.srv`.

(heading-target)=
### GlobalPlannerTestServer
This node enabled the offline testing of the planner implementation without a need of running the actual robot or the whole Navigation2 stack. For this purpose recorded service requests are loaded from a JSON file and then can be send by the ROS2 service `/send_test_request`.

The launch file `bringup_test_system_planner.launch.py` can be used to launch this test-system and a RViz visualization. The only needed launch-argument is `plugin_name:=<name of the tested plugin>`. The launch command could look like the following: `ros2 launch sopias4_framework bringup_test_system_planner.launch.py plugin_name:=astar`. In the development container this could also be launched with the shortcut `sopias4-testsystem-planner plugin_name:=astar`. The node to be tested still need to be started manually.  

To send a test request simply call the service  with `ros2 service call /send_test_request std_srvs/srv/Empty`. In the development container this is also shortcut with `sopias4-testrequest-planner`.

## LayerBridge
A costmap layer provide one layer to the costmap. In the background, all layers are merged together by choosing the maximum value from all layers for a specific cell. Thus, only high costs of the layer have the potential to be represented in the final costmap and low costs are tending to get discarded when other layers have higher costs there. 

For using the PluginBridge for this, it can be configured by adding following to your Navigation 2 configuration (for global costmap simply change `local_costmap` to `global_costmap`):
```YAML
local_costmap:
    local_costmap:
        ros__parameters:
            plugins: [robot_layer]
            robot_layer:
                plugin: plugin_bridges/LayerPlugin
                plugin_name: "robot_layer"
```

In Python you can use the LayerPyPlugin to implement the service. For this purpose, the `plugin_name` from the Navigation 2 configuration must match the one from the constructor. For further details, look into the documentation for this class.  A implementation could look like the following:
```Python
class RobotLayer(LayerPyPlugin):

    def __init__(self, namespace: str | None = None) -> None:
        super().__init__(
            node_name="robot_layer_node", plugin_name="robot_layer"
        ) if namespace is None else super().__init__(
            node_name="robot_layer_node",
            plugin_name="robot_layer",
            namespace=namespace,
        )

    def update_costs(
        self, min_i: int, min_j: int, max_i: int, max_j: int, costmap: PyCostmap2D
    ) -> PyCostmap2D:
    # Algorithm here

def main(args=None):
    # Initialize node context
    rclpy.init(args=args)
    # Create ROS2 Node
    ros2_node = RobotLayer()
    # Run node
    rclpy.spin(ros2_node)
    # Cleanup
    ros2_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```

If you want to create your own plugin implementation, the service must be named under the following scheme: `/<namespace>/<plugin_name>/update_costs`. The corresponding service definition can be found in `sopias4_msgs/srv/UpdateCosts.srv`.

## ControllerBridge
The controller is responsible for generating the actual steering commands so the robot follows the planned path from the planner.

For using the PluginBridge for this, it can be configured by adding following to your Navigation 2 configuration:
```YAML
controller_server:
    ros__parameters:
        controller_plugins: ["FollowPath"]
        FollowPath:
            plugin:  "plugin_bridges/ControllerBridge"
            plugin_name: "rotating_straightline"
```

In Python you can use the ControllerPyPlugin to implement the service. For this purpose, the `plugin_name` from the Navigation 2 configuration must match the one from the constructor. For further details, look into the documentation for this class. A implementation could look like the following:
```Python
class RotationStraightController(ControllerPyPlugin):

    def __init__(self, namespace: str | None = None) -> None:
        super().__init__(
            node_name="rotation_straight_controller_node", plugin_name="rotation_straight_controller"
        ) if namespace is None else super().__init__(
            node_name="rotation_straight_controller_node",
            plugin_name="rotation_straight_controller",
            namespace=namespace,
        )

    def compute_velocity_command(
        self,
        goal_pose: Pose,
        current_pos: PoseStamped,
        current_vel: Twist,
        costmap: PyCostmap2D,
    ) -> TwistStamped:
    # Algorithm here

def main(args=None):
    # Initialize node context
    rclpy.init(args=args)
    # Create ROS2 Node
    ros2_node = RotationStraightController()
    # Run node
    rclpy.spin(ros2_node)
    # Cleanup
    ros2_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```

If you want to create your own plugin implementation, the service must be named under the following scheme: `/<namespace>/<plugin_name>/compute_velocity_commands`. The corresponding service definition can be found in `sopias4_msgs/srv/ComputeVelocityCommands.srv`.

