#!/usr/bin/env python3
import abc
from threading import Thread

import rclpy
import sopias4_framework.tools.ros2
from nav2_simple_commander.costmap_2d import PyCostmap2D
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
    as a layer plugin and give it the same plugin name as this class. The configuration inside the yaml-configuration should look something like that:

    .. highlight:: yaml
    .. code-block:: yaml

        local_costmap:
            local_costmap:
                ros__parameters:
                    plugins: [robot_layer]
                    robot_layer:
                        plugin: plugin_bridges/LayerPlugin
                        plugin_name: "robot_layer"
    """

    def __init__(
        self,
        node_name: str = "layer_pyplugin",
        plugin_name: str = "abstract_plugin",
        namespace: str | None = None,
    ) -> None:
        super().__init__(node_name) if namespace is None else super().__init__(node_name, namespace=namespace)  # type: ignore

        # Service
        self.__plugin_bridge_server: Service = self.create_service(
            UpdateCosts, f"{plugin_name}/update_costs", self.__update_costs_callback
        )

    def __update_costs_callback(
        self, request: UpdateCosts.Request, response: UpdateCosts.Response
    ) -> UpdateCosts.Response:
        """
        Callback function which executes when the update_costs service is called
        """
        self.get_logger().debug(
            "Got request to update costs in robot layer",
            throttle_duration_sec=2,
        )
        updated_costmap: PyCostmap2D = self.update_costs(
            min_i=request.min_i,
            min_j=request.min_j,
            max_i=request.max_i,
            max_j=request.max_j,
            costmap=PyCostmap2D(request.current_costmap),
        )
        self.get_logger().debug(
            "Updated costmap, returning costmap to service requester",
            throttle_duration_sec=2,
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

    def destroy_node(self):
        self.get_logger().info("Shutting down node")
        super().destroy_node()
