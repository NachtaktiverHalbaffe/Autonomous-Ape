#!/usr/bin/env python3

import math
from typing import Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.costmap_2d import PyCostmap2D
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from scipy import ndimage
from skimage import draw
from sopias4_framework.nodes.layer_pyplugin import LayerPyPlugin
from sopias4_framework.tools.ros2 import costmap_tools

from sopias4_msgs.msg import Robot, RobotStates


class PathLayer(LayerPyPlugin):
    """
    A costmap layer into which the footprint of all known global navigation paths of all robots are set as moderate costs. This layer
    is already implemented and only needs to be configured to be run in the Navigation 2 stack. For this purpose, make sure that this node
    is running and add following to your costmap configuration inside `nav2.yaml` according to this example:

    .. highlight:: yaml
    .. code-block:: yaml

        local_costmap:
            local_costmap:
                ros__parameters:
                    plugins: [robot_layer]
                    robot_layer:
                        plugin: plugin_bridges/LayerPlugin
                        plugin_name: "path_layer"

    Attributes:
        COST_PATH(np.uint8): The cost the robot should have. Usually moderate cost because we want the roboter\
                                             to cross an path if cost of another way gets too big
        ROBOT_RADIUS (float): The radius of the robots footprint. It is assumed that the robot has an circular footprint (Turtlebot 4)
    """

    def __init__(self, namespace: str | None = None) -> None:
        if namespace is None:
            super().__init__(node_name="path_layer_node", plugin_name="path_layer")
        else:
            super().__init__(
                node_name="path_layer_node",
                plugin_name="path_layer",
                namespace=namespace,
            )
        self.COST_PATH: np.uint8 = np.uint8(142)
        self.ROBOT_RADIUS: float = 0.15

        self.robot_paths: list[Path] = list()

        # Create own sub node for service clients so they can spin independently
        self.__sub_robot_states = self.create_subscription(
            RobotStates,
            "/robot_states",
            self.__update_robot_states,
            QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE,
                depth=5,
            ),
        )
        self.get_logger().info("Started node")

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
        # Set cost of all pixels in costmap to zero, because only the costs calculated by this layer should be included in the map.
        # In the plugin bridge, all layers get combined so the cleared data doesn't get lost if it is up to date
        costmap.costmap.fill(np.uint8(0))

        self.get_logger().debug(
            "Path layer is inserting planned paths of other robots as moderate costs",
            throttle_duration_sec=2,
        )

        # Calculate how thick the pixels need to be inflated to meet size of robot
        inflation_distance_pxl: int = math.ceil(
            self.ROBOT_RADIUS * (1 / costmap.getResolution())
        )
        if inflation_distance_pxl < 1:
            inflation_distance_pxl = 1

        for path in self.robot_paths:
            # Ignore if robot hasnt a planned path
            if len(path.poses) == 0:
                continue
            try:
                last_node: Tuple[int, int] = self.pose_to_costmap_framesafe(path.poses[0], costmap)  # type: ignore
                # Check if central point is in the costmap itselfs
                x_out_of_bounds: bool = (
                    last_node[0] >= costmap.getSizeInCellsX() or last_node[0] < 0
                )
                y_out_of_bounds: bool = (
                    last_node[1] >= costmap.getSizeInCellsY() or last_node[1] < 0
                )
                if y_out_of_bounds or x_out_of_bounds:
                    continue
            except Exception as e:
                continue

            for node in path.poses[1::]:  # type: ignore
                try:
                    current_node = self.pose_stamped_to_costmap_framesafe(node, costmap)
                except Exception as e:
                    continue
                # Basically a Bresenhams line algorithm implementation
                rr, cc = draw.line(
                    last_node[0], last_node[1], current_node[0], current_node[1]
                )
                for x in rr:
                    for y in cc:
                        # Set cost of path
                        if costmap.getIndex(x, y) < len(costmap.costmap):
                            costmap.setCost(x, y, self.COST_PATH)

                        # Inflate this pixel so path is as thick as the robot
                        for y_infl in range(
                            max(0, int(y) - inflation_distance_pxl),
                            min(
                                costmap.getSizeInCellsY(),
                                int(y) + inflation_distance_pxl,
                            ),
                        ):
                            for x_infl in range(
                                max(0, int(x) - inflation_distance_pxl),
                                min(
                                    costmap.getSizeInCellsX(),
                                    int(x) + inflation_distance_pxl,
                                ),
                            ):
                                if costmap.getIndex(x_infl, y_infl) < len(
                                    costmap.costmap
                                ):
                                    costmap.setCost(x_infl, y_infl, self.COST_PATH)

                last_node = current_node

        return costmap

    def __update_robot_states(self, msg: RobotStates) -> None:
        """
        Callback function for the robot states subscriber. It basically updates the positions of other roboters
        """
        # Clear last known positions
        self.robot_paths.clear()

        # Add new position of robots to list
        for robot in msg.robot_states:
            # Check if robotstate is the robot itself or if robot isn't navigating => ignore these paths
            if robot.name_space == self.get_namespace() or not robot.is_navigating:
                continue

            if len(robot.nav_path.poses) == 0:
                continue

            self.robot_paths.append(robot.nav_path)


def main(args=None):
    """
    :meta private:
    """
    # Initialize node context
    rclpy.init(args=args)
    # Create ROS2 Node
    ros2_node = PathLayer()
    # Run node
    rclpy.spin(ros2_node)
    # Cleanup
    ros2_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
