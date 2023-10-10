#!/usr/bin/env python3
import abc
from threading import Thread
from typing import Tuple

import numpy as np
import tf2_geometry_msgs
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from nav2_simple_commander.costmap_2d import PyCostmap2D
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from rclpy.service import Service
from rclpy.time import Time
from sopias4_framework.tools.ros2 import costmap_tools
from sopias4_framework.tools.ros2.costmap_tools import pycostmap2d_2_occupancygrid
from tf2_geometry_msgs import Pose, PoseStamped, PoseWithCovarianceStamped
from tf2_msgs.msg import TFMessage
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

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

    COST_NO_INFORMATION: np.uint8 = np.uint8(255)
    """We treat unknown space as space which we dont want drive to. So these have the highest cost, even higher than lethal obstacles"""
    COST_LETHAL_OBSTACLE: np.uint8 = np.uint8(254)
    """"Lethal" cost means that there is an actual (workspace) obstacle in a cell. So if the robot's center were in that cell, the robot would obviously be in collision"""
    COST_INSCRIBED_INFLATED_OBSTACLE: np.uint8 = np.uint8(253)
    """"Inscribed" cost means that a cell is less than the robot's inscribed radius away from an actual obstacle. So the robot is certainly in collision with some obstacle if the robot center is in a cell that is at or above the inscribed cost."""
    COST_MAX_NON_OBSTACLE: np.uint8 = np.uint8(252)
    """Maximum cost until before the cost represents a obstacle"""
    COST_FREE_SPACE: np.uint8 = np.uint8(0)
    """Cost which indicates that there is nothing that should keep the robot from going there."""

    def __init__(
        self,
        node_name: str = "layer_pyplugin",
        plugin_name: str = "abstract_plugin",
        namespace: str | None = None,
    ) -> None:
        super().__init__(node_name) if namespace is None else super().__init__(node_name, namespace=namespace)  # type: ignore
        self.get_logger().info(f"Initializing {plugin_name}")
        # Service
        self.__plugin_bridge_server: Service = self.create_service(
            UpdateCosts, f"{plugin_name}/update_costs", self.__update_costs_callback
        )

        self.get_logger().info("Initializing tf buffer")

        self.tf_buffer: Buffer = Buffer()
        self.__tf_listener = TransformListener(self.tf_buffer, self)

        # Transformlistener doesnt hear on namespaced tf topics => recreate subscriptions with namespaced topics
        self.get_logger().info("Changing tf topics to namespaced ones")
        self.__tf_listener.unregister()
        qos = QoSProfile(
            depth=100,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        static_qos = QoSProfile(
            depth=100,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.__tf_listener.tf_sub = self.create_subscription(
            TFMessage,
            "tf",
            self.__tf_listener.callback,
            qos,
            callback_group=self.__tf_listener.group,
        )
        self.__tf_listener.tf_static_sub = self.create_subscription(
            TFMessage,
            "tf_static",
            self.__tf_listener.static_callback,
            static_qos,
            callback_group=self.__tf_listener.group,
        )
        self.get_logger().info(f"Started {plugin_name}")

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
        arguments, to save computational time. Also, make sure to work in the right transformation frame. There also framesafe\
        conversions between poses in the map domain [m] to coordinates in the costmap [cells] in this class which can be used\
        for this purpose

        Args:
            min_i (int): The minimum x-index of the update window
            min_j (int): The minimum y-index of the update window
            max_i (int): The maximum x-index of the update window
            max_j (int): The maximum y-index of the update window
            costmap(nav2_simplecommander.costmap_2d.PyCostmap2D): The costmap that should be updated

        Returns:
            nav2_simplecommander.costmap_2d.PyCostmap2D: The updated costmap 
        """

    def transform_pose(self, pose, target_frame: str, timeout_milliseconds: int = 0):
        """
        Transform a pose into a source frame

        Args:
            pose: The position that should be transformed
            target_frame (str): The frame into which the pose should be transformed
            timeout_milliseconds (int, optional): The time after which the transformation should time out

        Returns:
            Same type as pose: The transformed pose
        """
        return self.tf_buffer.transform(
            pose,
            target_frame=target_frame,
            timeout=Duration(nanoseconds=timeout_milliseconds * 1000),
        )

    def pose_with_covariance_stamped_to_costmap_framesafe(
        self, pose: PoseWithCovarianceStamped, costmap: PyCostmap2D
    ) -> Tuple[int, int]:
        """
        Converts a PoseWithCovarianceStamped into x,y-coordinates from the costmap and applies a transformation if\
        the pose and the costmap are in a different transformation frame

        Args:
            pose (PoseWithCovarianceStamped): The pose which should be converted into coordinates
            costmap (nav2_simplecommander.costmap_2d.PyCostmap2D): The costmap in where the x,y-coordinates should be
        
        Returns:
            Tuple(int,int): The converted x,y-coordinate in the right frame
        """
        if pose.header.frame_id == costmap.getGlobalFrameID():
            return costmap_tools.pose_2_costmap(pose.pose.pose, costmap)
        else:
            try:
                transform = self.tf_buffer.lookup_transform(
                    costmap.getGlobalFrameID(), pose.header.frame_id, Time()
                )
                pose_transformed = (
                    tf2_geometry_msgs.do_transform_pose_with_covariance_stamped(
                        pose, transform
                    )
                )
                # pose_transformed = self.transform_pose(pose, costmap.getGlobalFrameID())
                return costmap_tools.pose_2_costmap(
                    pose_transformed.pose.pose, costmap  # type: ignore
                )
            except Exception as e:
                raise e

    def pose_to_costmap_framesafe(
        self, pose: Pose, costmap: PyCostmap2D, source_frame: str
    ) -> Tuple[int, int]:
        """
        Converts a pose into x,y-coordinates from the costmap and applies a transformation if\
        the pose and the costmap are in a different transformation frame

        Args:
            pose (Pose): The pose which should be converted into coordinates
            costmap (nav2_simplecommander.costmap_2d.PyCostmap2D): The costmap in where the x,y-coordinates should be
            source_frame (str): The transformation frame in which the pose is
        
        Returns:
            Tuple(int,int): The converted x,y-coordinate in the right frame
        """
        if source_frame == costmap.getGlobalFrameID():
            return costmap_tools.pose_2_costmap(pose, costmap)
        else:
            try:
                transform = self.tf_buffer.lookup_transform(
                    costmap.getGlobalFrameID(), source_frame, Time()
                )
                pose_transformed = tf2_geometry_msgs.do_transform_pose(pose, transform)
                # pose_transformed = self.transform_pose(pose, costmap.getGlobalFrameID())
                return costmap_tools.pose_2_costmap(pose_transformed.pose, costmap)  # type: ignore
            except Exception as e:
                raise e

    def pose_stamped_to_costmap_framesafe(
        self, pose: PoseStamped, costmap: PyCostmap2D
    ) -> Tuple[int, int]:
        """
        Converts a PoseStamped into x,y-coordinates from the costmap and applies a transformation if\
        the pose and the costmap are in a different transformation frame

        Args:
            pose (PoseStamped): The pose which should be converted into coordinates
            costmap (nav2_simplecommander.costmap_2d.PyCostmap2D): The costmap in where the x,y-coordinates should be
        
        Returns:
            Tuple(int,int): The converted x,y-coordinate in the right frame
        """
        if pose.header.frame_id == costmap.getGlobalFrameID():
            return costmap_tools.pose_2_costmap(pose, costmap)
        else:
            try:
                transform = self.tf_buffer.lookup_transform(
                    costmap.getGlobalFrameID(), pose.header.frame_id, Time()
                )
                pose_transformed = tf2_geometry_msgs.do_transform_pose_stamped(
                    pose, transform
                )
                # pose_transformed = self.transform_pose(pose, costmap.getGlobalFrameID())
                return costmap_tools.pose_2_costmap(pose_transformed.pose, costmap)  # type: ignore
            except Exception as e:
                raise e

    def destroy_node(self):
        self.get_logger().info("Shutting down node")
        super().destroy_node()
