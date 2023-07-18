#!/usr/bin/env python3
import abc
from threading import Thread
from typing import Tuple

from geometry_msgs.msg import Pose
from nav2_simple_commander.costmap_2d import PyCostmap2D
from nav_msgs.msg import Path
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.service import Service
from sopias4_framework.tools.ros2 import costmap_tools

from sopias4_msgs.srv import CreatePlan


class PlannerPyPlugin(Node):
    """
    This class is a base planner plugin class with which you can write your own Navigation2 planner. This class
    utilizes the planner plugin bridge so the planner can be implemented in python. This class handles all the service
    stuff under the hood, so the developer only has to implement the `generate_path()` method.

    The `generate_path()` method gives the start and goal pose and the current costmap. The method should then return the
    generated path as an list of x,y-coordinates in the costmap. For this purpose, a suitable path finding algorithm should be implemented
    there.

    Also, the plugin bridge itself must be configurated in the Navigation2 Stack. For this purpose, make shure to configure the PlannerBridge
    as a planner plugin and give it the same plugin name as this class

    Attributes:
        costmap (nav2_simple_commander.costmap_2d.PyCostmap2D): The current costmap which contains the costs of each region. Low costs means\
                                                                                                                    free regions and higher costs that there may be an obstacle or other reasons why \
                                                                                                                    the robot should avoid this region
    """

    def __init__(self, node_name, plugin_name, namespace: str | None = None) -> None:
        if namespace is None:
            super().__init__(node_name)  # type: ignore
        else:
            super().__init__(node_name, namespace=namespace)  # type: ignore

        # Service server
        self.__plugin_bridge_server: Service = self.create_service(
            CreatePlan, f"{plugin_name}/create_plan", self.__create_plan_callback
        )

        self.costmap: PyCostmap2D
        # Let node spin itself
        # self.__executor = MultiThreadedExecutor()
        # self.__executor.add_node(self)
        # self.__spin_node_thread = Thread(target=self.__executor.spin)
        # self.__spin_node_thread.start()

    def __create_plan_callback(
        self, request: CreatePlan.Request, response: CreatePlan.Response
    ) -> CreatePlan.Response:
        """
        Callback when a service requests to create a global path. It's basically the ROS2 service server stuff
        necessary to work with the PlannerBridge. The path finding algorithm itself should be implemented
        in `generate_path()`
        """
        self.get_logger().info("Got request to generate a global path from the Planner")
        start = costmap_tools.pose_2_map(request.start, PyCostmap2D(request.costmap))
        goal = costmap_tools.pose_2_map(request.start, PyCostmap2D(request.costmap))
        self.costmap = PyCostmap2D(request.costmap)

        pixel_path: list[Tuple[int, int]] = self.generate_path(start=start, goal=goal)

        self.get_logger().debug(
            "Found shortest path in costmap domain. Transforming it into map domain"
        )
        path: Path = Path()
        for node in pixel_path:
            path_node = Pose()
            path_node = costmap_tools.map_2_pose(
                node[0], node[1], PyCostmap2D(request.costmap)
            )
            path.poses.append(path_node)  # type: ignore

        response.global_path = path
        self.get_logger().info("Shortest global path to goal successfully found")
        return response

    @abc.abstractmethod
    def generate_path(
        self, start: Tuple[int, int], goal: Tuple[int, int]
    ) -> list[Tuple[int, int]]:
        """
        Here the path finding algorithm should be implemented

        Args:
            start (tuple(int, int)): The position from which the path should start as an x,y-coordinate in the costmap
            goal (tuple(int, int)): The position in which the path should end as an x,y-coordinate in the costmap

        Returns:
            list(tuple(int,int)): The generated path as a list of x,y-coordinates in the costmap
        """
