#!/usr/bin/env python3
import abc
from threading import Thread

import sopias4_framework.tools.ros2
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.costmap_2d import PyCostmap2D
from nav_msgs.msg import Path
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.service import Service
from sopias4_framework.tools.ros2.costmap_tools import pycostmap2d_2_occupancygrid

from sopias4_msgs.srv import UpdateCosts


class LayerPyPlugin(Node):
    """
    This class is a base layer plugin class with which you can write your own Navigation2 costmap layer. This class
    utilizes the layer plugin bridge so the layer can be implemented in python. This class handles all the service
    stuff under the hood, so the developer only has to implement the `update_costs()` method.

    The `update_costs()` method takes the window frame which should be updated in form of a minimum and maximum coordinate. It also gives
    the costmap itself which should be updated. The method should then return the updated costmap

    Also, the plugin bridge itself must be configurated in the Navigation2 Stack. For this purpose, make shure to configure the LayerBridge
    as a layer plugin and give it the same plugin name as this class.
    """

    def __init__(
        self, node_name: str, plugin_name: str, namespace: str | None = None
    ) -> None:
        if namespace is None:
            super().__init__(node_name)  # type: ignore
        else:
            super().__init__(node_name, namespace=namespace)  # type: ignore

        # Service server
        self.__plugin_bridge_server: Service = self.create_service(
            UpdateCosts, f"{plugin_name}/update_costs", self.__update_costs_callback
        )

        # Let node spin itself
        self.__executor = MultiThreadedExecutor()
        self.__executor.add_node(self)
        self.__spin_node_thread = Thread(target=self.__executor.spin)
        self.__spin_node_thread.start()

    def __update_costs_callback(
        self, request: UpdateCosts.Request, response: UpdateCosts.Response
    ) -> UpdateCosts.Response:
        """
        Callback function which executes when the update_costs service is called
        """
        updated_costmap: PyCostmap2D = self.update_costs(
            min_i=request.min_i,
            min_j=request.min_j,
            max_i=request.max_i,
            max_j=request.max_j,
            costmap=PyCostmap2D(request.current_costmap),
        )
        response.updated_costmap = pycostmap2d_2_occupancygrid(updated_costmap)
        return response

    @abc.abstractmethod
    def update_costs(
        self, min_i: int, min_j: int, max_i: int, max_j: int, costmap: PyCostmap2D
    ) -> PyCostmap2D:
        """
        Here the costmap should be updated. Only update the region inside the window, specified by the min_* and max_* \
        arguments, to save computational time.

        Args:
            min_i (int): The minimum x-index of the update window
            min_j (int): The minimum y-index of the update window
            max_i (int): The maximum x-index of the update window
            max_j (int): The maximum y-index of the update window
            costmap(nav2_simplecommander.costmap_2d.PyCostmap2D): The costmap that should be updated

        Returns:
            nav2_simplecommander.costmap_2d.PyCostmap2D: The updated costmap 
        """
