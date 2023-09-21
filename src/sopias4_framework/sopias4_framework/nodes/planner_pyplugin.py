#!/usr/bin/env python3
import abc
import math
from typing import Tuple

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
    as a planner plugin and give it the same plugin name as this class. The configuration inside the yaml-configuration should look something like that:

    .. highlight:: yaml
    .. code-block:: yaml

            planner_server:
                ros__parameters:
                    plugins: ["GridBased"]
                    GridBased:
                        plugin: "plugin_bridges/PlannerBridge"
                        plugin_name: "Astar"

    Attributes:
        costmap (nav2_simple_commander.costmap_2d.PyCostmap2D): The current costmap which contains the costs of each region. Low costs means\
                                                                                                                    free regions and higher costs that there may be an obstacle or other reasons why \
                                                                                                                    the robot should avoid this region
        goal_tolerance (float, optional): The tolerance distance in meters to the goal under which the algorithm considers its goal reached. Defaults to 0.2
        caching_tolerance (float, optional): The tolerance distance in meters between the start given by a service request and any node in the cached path under\
                                                                    which the cached_path will be reused instead of being recomputed. Defaults to 0.4
        caching_enabled (bool, optional): If enabled, it will check if the cached path is feasible for the service request and returns the cached path, otherwise it\
                                                                  computes a new one. It should reduce oscillation between different global paths when the planner server wants it to regenerated\
                                                                  on a regular basis. Defaults to True (enabled) 
    """

    def __init__(
        self,
        node_name: str,
        plugin_name: str,
        namespace: str | None = None,
        goal_tolerance: float = 0.2,
        caching_tolerance: float = 0.4,
        caching_enabled: bool = True,
    ) -> None:
        super().__init__(node_name) if namespace is None else super().__init__(node_name, namespace=namespace)  # type: ignore

        # Service server
        self.__plugin_bridge_server: Service = self.create_service(
            CreatePlan, f"{plugin_name}/create_plan", self.__create_plan_callback
        )

        self.costmap: PyCostmap2D
        self.goal_tolerance: float = goal_tolerance
        self.caching_tolerance: float = caching_tolerance
        self.__caching_enabled: bool = caching_enabled
        self.__cached_path: list[Tuple[int, int]] = list()

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

        if self.is_caching_enabled() and self.__check_feasability_of_cached_path(
            start, goal
        ):
            self.get_logger().debug("Using cached path")
            pixel_path: list[Tuple[int, int]] = self.__generate_sliced_cached_path(
                start=start
            )
        else:
            self.get_logger().debug("Generating new path")
            pixel_path: list[Tuple[int, int]] = self.generate_path(
                start=start, goal=goal, costmap=self.costmap, goal_tolerance=0.2
            )
            self.__cached_path = pixel_path

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

    def enable_caching(self):
        """
        Enables the caching of generated paths. If enabled, it will check if the cached path is feasible for the service request and returns\
        the cached path, otherwise it computes a new one. It should reduce oscillation between different global paths when the planner \
        server wants it to regenerated on a regular basis 
        """
        self.__caching_enabled = True
        self.get_logger().info("Caching of planned paths enabled")

    def disable_caching(self):
        """
        Disables the caching of generated paths. If disabled, every path is generated from scratch on every service request
        """
        self.__caching_enabled = False
        self.get_logger().info("Caching of planned paths disabled")

    def is_caching_enabled(self) -> bool:
        """
        Check if caching of generated paths is enabled

        Returns:
            bool: True if caching is enabled
        """
        return self.__caching_enabled

    def __check_feasability_of_cached_path(
        self, start: Tuple[int, int], goal: Tuple[int, int]
    ) -> bool:
        """
        Checks if cached path is feasible for service request. For feasibility, following has to be fullfilled:
        1. Last node of cached path is within the tolerance distance to the goal of the service request
        2. One of the nodes is within the tolerance distance of the start of the service request
        3. No one of the nodes of the cached path is within a lethal obstacle of the new passed global costmap of the service request

        Args:
            start (tuple(int, int)): The position from which the path should start as an x,y-coordinate in the costmap
            goal (tuple(int, int)): The position in which the path should end as an x,y-coordinate in the costmap

        Returns:
            bool: If cached path is feasible
        """
        if len(self.__cached_path) == 0:
            return False
        # Check if last node of cached path isnt within the tolerance distance to the goal of the service request
        if (
            costmap_tools.euclidian_distance_map_domain(
                start=self.__cached_path[-1], goal=goal, costmap=self.costmap
            )
            > self.goal_tolerance
        ):
            return False

        start_is_feasible: bool = False
        for node in self.__cached_path:
            # Check if none of the nodes is within the tolerance distance of the start of the service request
            if (
                costmap_tools.euclidian_distance_map_domain(
                    start=node, goal=start, costmap=self.costmap
                )
                <= self.caching_tolerance
            ):
                start_is_feasible = True
            # Check if one of the node is within lethal obstacles
            if self.costmap.getCostXY(node[0], node[1]) >= costmap_tools.LETHAL_COST:
                return False

        return start_is_feasible

    def __generate_sliced_cached_path(
        self, start: Tuple[int, int]
    ) -> list[Tuple[int, int]]:
        index_start: int = 0
        shortest_dist: float = math.inf

        for index, node in enumerate(self.__cached_path):
            curr_dist: float = costmap_tools.euclidian_distance_map_domain(
                start=node, goal=start, costmap=self.costmap
            )
            if curr_dist <= shortest_dist:
                shortest_dist = curr_dist
                index_start = index

        return self.__cached_path[index_start::]
