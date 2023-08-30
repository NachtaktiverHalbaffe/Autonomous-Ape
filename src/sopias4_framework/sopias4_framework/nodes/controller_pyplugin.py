#!/usr/bin/env python3
import abc
from threading import Thread

import rclpy
import sopias4_framework.tools.ros2
from geometry_msgs.msg import Pose, PoseStamped, Twist, TwistStamped
from nav2_simple_commander.costmap_2d import PyCostmap2D
from rclpy.node import Node
from rclpy.service import Service
from sopias4_framework.tools.ros2.costmap_tools import pycostmap2d_2_occupancygrid

from sopias4_msgs.srv import ComputeVelocityCommands


class ControllerPyPlugin(Node):
    """
    This class is a base layer plugin class with which you can write your own Navigation2 controller plugin. This class
    utilizes the layer plugin bridge so the controller can be implemented in python. This class handles all the service
    stuff under the hood, so the developer only has to implement the `compute_velocity_commands()` method.

    The `compute_velocity_commands()` takes the local costmap, the targeted position and the current position&velocity and should
    return a velocity command which steers the robot towards the goal pose.

    Also, the plugin bridge itself must be configurated in the Navigation2 Stack. For this purpose, make shure to configure the ControllerBridge
    as a layer plugin and give it the same plugin name as this class. The configuration inside the yaml-configuration should look something like that:

    .. highlight:: yaml
    .. code-block:: yaml

            controller_server:
            ros__parameters:
                controller_plugins: ["FollowPath"]
                FollowPath:
                plugin:  "plugin_bridges/ControllerBridge"
                plugin_name: "Example"

    """

    def __init__(
        self,
        node_name: str = "controller_pyplugin",
        plugin_name: str = "abstract_plugin",
        namespace: str | None = None,
    ) -> None:
        super().__init__(node_name) if namespace is None else super().__init__(node_name, namespace=namespace)  # type: ignore

        # Service
        self.__plugin_bridge_server: Service = self.create_service(
            ComputeVelocityCommands,
            f"{plugin_name}/compute_velocity_commands",
            self.__compute_vel_cmd_callback,
        )

    def __compute_vel_cmd_callback(
        self,
        request: ComputeVelocityCommands.Request,
        response: ComputeVelocityCommands.Response,
    ) -> ComputeVelocityCommands.Response:
        """
        Callback function which executes when the update_costs service is called
        """
        self.get_logger().debug(
            "Got request to compute velocity commands",
            throttle_duration_sec=2,
        )
        request.current_vel_cmd
        computed_cmd: TwistStamped = self.compute_velocity_command(
            goal_pose=request.goal_pose,
            current_pos=request.current_pose,
            current_vel=request.current_vel_cmd,
            costmap=PyCostmap2D(request.local_costmap),
        )
        self.get_logger().debug(
            "Updated costmap, returning costmap to service requester",
            throttle_duration_sec=2,
        )

        response.cmd_vel = computed_cmd
        return response

    @abc.abstractmethod
    def compute_velocity_command(
        self,
        goal_pose: Pose,
        current_pos: PoseStamped,
        current_vel: Twist,
        costmap: PyCostmap2D,
    ) -> TwistStamped:
        """
        Here the velocity command should be computed

        Args:
            goal_pose (Pose): The position to which the robot should drive next
            current_pos (PoseStamped): Current position of the robot in the map domain
            current_vel (Twist): The current velocity of the robot

        Returns:
            TwistStamped: The computed velocity command which the robot should execute next
        """

    def destroy_node(self):
        self.get_logger().info("Shutting down node")
        self.__plugin_bridge_server.destroy()
        super().destroy_node()
