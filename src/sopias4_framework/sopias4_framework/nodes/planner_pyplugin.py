#!/usr/bin/env python3
import abc
from threading import Thread

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.costmap_2d import PyCostmap2D
from nav_msgs.msg import Path
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.service import Service

from sopias4_msgs.srv import CreatePlan


class PlannerPyPlugin(Node):
    """
    This class is a base planner plugin class with which you can write your own Navigation2 planner. This class
    utilizes the planner plugin bridge so the planner can be implemented in python. This class handles all the service
    stuff under the hood, so the developer only has to implement the `generate_path()` method.

    The `generate_path()` method gives the start and goal pose and the current costmap. The method should then return the
    generated path as an nav_msgs/Path message. For this purpose, a suitable path finding algorithm should be implemented
    there.

    Also, the plugin bridge itself must be configurated in the Navigation2 Stack. For this purpose, make shure to configure the PlannerBridge
    as a planner plugin and give it the same plugin name as this class

    """

    def __init__(self, plugin_name, namespace: str | None = None) -> None:
        if namespace is None:
            super().__init__(plugin_name)  # type: ignore
        else:
            super().__init__(plugin_name, namespace=namespace)  # type: ignore

        # Service server
        self.__plugin_bridge_server: Service = self.create_service(
            CreatePlan, f"{plugin_name}/create_plan", self.__create_plan_callback
        )

        # Let node spin itself
        self.__executor = MultiThreadedExecutor()
        self.__executor.add_node(self)
        self.__spin_node_thread = Thread(target=self.__executor.spin)
        self.__spin_node_thread.start()

    def __create_plan_callback(
        self, request: CreatePlan.Request, response: CreatePlan.Response
    ) -> CreatePlan.Response:
        path: Path = self.generate_path(
            start=request.start, goal=request.goal, costmap=PyCostmap2D(request.costmap)
        )

        response.global_path = path
        return response

    @abc.abstractmethod
    def generate_path(
        self, start: PoseStamped, goal: PoseStamped, costmap: PyCostmap2D
    ) -> Path:
        """
        Here the path finding algorithm should be implemented

        Args:
            start (PoseStamped): The position from which the path should start
            goal (PoseStamped): The position in which the path should end
            costmap (Costmap): The current costmap which contains the costs of each region. Low costs means free regions and higher\
                                                costs that there may be an obstacle or other reasons why the robot should avoid this region
        
        Returns:
            Path: The generated path
        """
