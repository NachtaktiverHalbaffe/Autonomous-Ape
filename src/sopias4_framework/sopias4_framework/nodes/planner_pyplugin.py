#!/usr/bin/env python3
import abc
from typing import Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.costmap_2d import PyCostmap2D
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.service import Service
from sopias4_framework.tools.ros2 import costmap_tools

from sopias4_msgs.srv import CreatePlan


class PlannerPyPlugin(Node):
    """
    This class is a base planner plugin class with which you can write your own Navigation2 planner. This class
    utilizes the planner plugin bridge so the planner can be implemented in python. This class handles all the service
    stuff under the hood, so the developer only has to implement the `generate_path()` method.

    The `generate_path()` method gives the start & goal pose and the current costmap. The method should then return the
    generated path as an list of x,y-coordinates in the costmap. For this purpose, a suitable path finding algorithm should be implemented
    there.

    Also, the plugin bridge itself must be configurated in the Navigation2 Stack. For this purpose, make shure to configure the PlannerBridge
    as a planner plugin and give it the same plugin name as this class

    Attributes:
        costmap (nav2_simple_commander.costmap_2d.PyCostmap2D): The current costmap which contains the costs of each region. Low costs means\
                                                                                                                    free regions and higher costs that there may be an obstacle or other reasons why \
                                                                                                                    the robot should avoid this region
        goal_tolerance (float, optional): The tolerance distance in meters to the goal under which the algorithm considers its goal reached. Defaults to 0.2
    """

    def __init__(
        self,
        node_name: str,
        plugin_name: str,
        namespace: str | None = None,
        goal_tolerance: float = 0.2,
    ) -> None:
        super().__init__(node_name) if namespace is None else super().__init__(node_name, namespace=namespace)  # type: ignore

        # Service server
        self.__plugin_bridge_server: Service = self.create_service(
            CreatePlan, f"{plugin_name}/create_plan", self.__create_plan_callback
        )

        self.costmap: PyCostmap2D
        self.goal_tolerance = 0.2

    def __create_plan_callback(
        self, request: CreatePlan.Request, response: CreatePlan.Response
    ) -> CreatePlan.Response:
        """
        Callback when a service requests to create a global path. It's basically the ROS2 service server stuff
        necessary to work with the PlannerBridge. The path finding algorithm itself should be implemented
        in `generate_path()`
        """
        self.get_logger().debug(
            "Got request to generate a global path from the Planner"
        )
        start = costmap_tools.pose_2_costmap(
            request.start, PyCostmap2D(request.costmap)
        )
        goal = costmap_tools.pose_2_costmap(request.goal, PyCostmap2D(request.costmap))
        self.costmap = PyCostmap2D(request.costmap)

        self.get_logger().debug(
            f"Generating path in costmap domain from {start} to {goal}"
        )

        pixel_path: list[Tuple[int, int]] = self.generate_path(
            start=start, goal=goal, costmap=self.costmap, goal_tolerance=0.2
        )

        self.get_logger().debug(
            "Found shortest path in costmap domain. Transforming it into map domain"
        )

        path: Path = Path()
        path.header.frame_id = self.costmap.global_frame_id
        for node in pixel_path:
            path_node = PoseStamped()
            path_node.header.frame_id = self.costmap.global_frame_id
            path_node.pose = costmap_tools.costmap_2_pose(
                node[0], node[1], PyCostmap2D(request.costmap)
            )
            path.poses.append(path_node)  # type: ignore

        response.global_path = path
        self.get_logger().info("Shortest global path to goal successfully found")
        return response

    @abc.abstractmethod
    def generate_path(
        self,
        start: Tuple[int, int],
        goal: Tuple[int, int],
        costmap: PyCostmap2D,
        goal_tolerance: float = 0.2,
    ) -> list[Tuple[int, int]]:
        """
        Here the path finding algorithm should be implemented. Some tips for implementing the algorithm:
        1. Use python sets and dicts instead of lists. They are way faster when a specific value is searched, inserted or updated
        2. Be careful with loops when iterating through datasets. When applying first tip, there are often ways to directly update, \
            searching or updating values in datasets instead of iterating through them
        3. The module costmap_tools from sopias4_framework.tools.ros2 package has useful tools for interacting with the given costmap. \

        Args:
            start (tuple(int, int)): The position from which the path should start as an x,y-coordinate in the costmap
            goal (tuple(int, int)): The position in which the path should end as an x,y-coordinate in the costmap
            costmap (nav2_simple_commander.costmap_2d.PyCostmap2D): The global costmap in which the path should be computed
            goal_tolerance (float, optional): The tolerance distance in meters to the goal under which the algorithm considers its goal reached. Defaults to 0.2

        Returns:
            list(tuple(int,int)): The generated path as a list of x,y-coordinates in the costmap
        """
