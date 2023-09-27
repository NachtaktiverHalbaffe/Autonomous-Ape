#!/usr/bin/env python3
import json
import os

import rclpy
from ament_index_python import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.costmap_2d import PyCostmap2D
from nav_msgs.msg import OccupancyGrid, Path
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.client import Client
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.service import Service
from sopias4_framework.tools.ros2 import node_tools
from std_srvs.srv import Empty

from sopias4_msgs.srv import CreatePlan


class GlobalPlannerTestServer(Node):
    """
    A node for testing PlannerPyPlugins. It loads data from a JSON file and creates a test service requests which then can be send to
    the PlannerPyPlugin. If a response is received, it then publishes it to the `/<namespace>/global_plan` topic.

    Can be used together with the launch file `bringup_test_system_planner.launch.py` where also RViz is launched to visualize the planned path
    and the map which is used for testing. To send a test request, simple call `ros2 service call /send_test_request std_srvs/srv/Empty`from the terminal
    or call the ROS2 service `/send_test_request` otherwise. You may call this service once to see the complete visualization in Rviz.

    It provides following services:
        - /<namespace>/send_test_request: Trigger to send a test request to the PlannerPyPlugin

    It has following paramteters:
        - test_data_path: Full path to the JSON file which holds the test data
    """

    def __init__(
        self,
        node_name: str = "layer_pyplugin",
        namespace: str | None = None,
    ) -> None:
        # --- Setup ROS node ---
        super().__init__(node_name) if namespace is None else super().__init__(node_name, namespace=namespace)  # type: ignore

        # -- Load map metadata ---
        self.declare_parameter(
            "test_data_path",
            os.path.join(
                get_package_share_directory("sopias4_framework"),
                "assets",
                "global_costmaps",
                "global_costmap.json",
            ),
            ParameterDescriptor(description="Path to the JSON file where all data"),
        )
        self.declare_parameter(
            "plugin_name",
            "abstract_plugin",
            ParameterDescriptor(
                description="The plugin name of the planner plugin which should be tested"
            ),
        )
        plugin_name: str = (
            self.get_parameter("plugin_name").get_parameter_value().string_value
        )
        test_data_path: str = (
            self.get_parameter("test_data_path").get_parameter_value().string_value
        )

        # Service client which sends the request to the planner
        self.__service_client_node: Node = rclpy.create_node("_global_planner_testserver_service_clients")  # type: ignore
        self.__global_planner_sclient_createplan: Client = (
            self.__service_client_node.create_client(
                CreatePlan,
                f"{self.get_namespace()}/{plugin_name}/create_plan"
                if self.get_namespace() != "/"
                else f"/{plugin_name}/create_plan",
            )
        )
        # Service which triggers a request to be send
        self.__test_service: Service = self.create_service(
            Empty, "send_test_request", self.send_test_request
        )
        # Publisher which publishes the plan from the planner to a topic
        self.__path_pub: Publisher = self.create_publisher(Path, "global_plan", 10)
        self.__costmap_pub: Publisher = self.create_publisher(
            OccupancyGrid, "global_costmap", 10
        )

        loaded_params: dict = dict()
        with open(test_data_path) as json_data:
            loaded_params = json.load(json_data)

        self.costmap: OccupancyGrid = OccupancyGrid()
        self.costmap.data = loaded_params["costmap"]["data"]
        self.costmap.header.frame_id = loaded_params["costmap"]["header"]["frame_id"]
        self.costmap.info.height = loaded_params["costmap"]["info"]["height"]
        self.costmap.info.width = loaded_params["costmap"]["info"]["width"]
        self.costmap.info.resolution = loaded_params["costmap"]["info"]["resolution"]
        self.costmap.info.origin.position.x = loaded_params["costmap"]["info"][
            "origin"
        ]["position"]["x"]
        self.costmap.info.origin.position.y = loaded_params["costmap"]["info"][
            "origin"
        ]["position"]["y"]
        self.costmap.info.origin.orientation.y = loaded_params["costmap"]["info"][
            "origin"
        ]["orientation"]["y"]
        self.costmap.info.origin.orientation.w = loaded_params["costmap"]["info"][
            "origin"
        ]["orientation"]["w"]

        self.start: PoseStamped = PoseStamped()
        self.start.header.frame_id = loaded_params["start"]["header"]["frame_id"]
        self.start.pose.position.x = loaded_params["start"]["pose"]["position"]["x"]
        self.start.pose.position.y = loaded_params["start"]["pose"]["position"]["y"]
        self.start.pose.orientation.y = loaded_params["start"]["pose"]["orientation"][
            "y"
        ]
        self.start.pose.orientation.w = loaded_params["start"]["pose"]["orientation"][
            "w"
        ]

        self.goal: PoseStamped = PoseStamped()
        self.goal.header.frame_id = loaded_params["goal"]["header"]["frame_id"]
        self.goal.pose.position.x = loaded_params["goal"]["pose"]["position"]["x"]
        self.goal.pose.position.y = loaded_params["goal"]["pose"]["position"]["y"]
        self.goal.pose.orientation.y = loaded_params["goal"]["pose"]["orientation"]["y"]
        self.goal.pose.orientation.w = loaded_params["goal"]["pose"]["orientation"]["w"]

        self.send_test_request()

    def send_test_request(
        self,
        _: Empty.Request = Empty.Request(),
        response_data: Empty.Response = Empty.Response(),
    ):
        """
        Send a test request to the PlannerPyPlugin. When it finished, it publishes the planned path to\
        the topic /global_plan
        """
        self.__costmap_pub.publish(self.costmap)
        req = CreatePlan.Request()
        req.start = self.start
        req.goal = self.goal
        req.costmap = self.costmap

        response: CreatePlan.Response | None = node_tools.call_service(
            self.__global_planner_sclient_createplan,
            req,
            self.__service_client_node,
            timeout_sec=120,
        )

        if response is None:
            self.get_logger().error("Planner didn't return response")
        else:
            self.__path_pub.publish(response.global_path)

        return response_data


def main(args=None):
    """
    :meta private:
    """
    # Initialize node context
    rclpy.init(args=args)
    # Create ROS2 Node
    ros2_node = GlobalPlannerTestServer()
    # Run node
    rclpy.spin(ros2_node)
    # Cleanup
    ros2_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
