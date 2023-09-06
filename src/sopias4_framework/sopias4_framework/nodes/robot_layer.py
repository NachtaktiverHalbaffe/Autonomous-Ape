#!/usr/bin/env python3
import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_simple_commander.costmap_2d import PyCostmap2D
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sopias4_framework.nodes.layer_pyplugin import LayerPyPlugin
from sopias4_framework.tools.ros2 import costmap_tools

from sopias4_msgs.msg import Robot, RobotStates


class RobotLayer(LayerPyPlugin):
    """
    A costmap layer into which the footprint of all known robots in the environment are set as the costs of an lethal obstacle. This layer
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
                        plugin_name: "robot_layer"

    Attributes:
        COST_ROBOTS (np.uint8): The cost the robot should have. Usually maximum cost because we want the roboter as an lethal obstacle
        ROBOT_RADIUS (float): The radius of the robots footprint. It is assumed that the robot has an circular footprint (Turtlebot 4)
    """

    def __init__(self, namespace: str | None = None) -> None:
        super().__init__(
            node_name="robot_layer_node", plugin_name="robot_layer"
        ) if namespace is None else super().__init__(
            node_name="robot_layer_node",
            plugin_name="robot_layer",
            namespace=namespace,
        )

        self.COST_ROBOTS: np.uint8 = np.uint8(254)
        self.ROBOT_RADIUS: float = 0.20

        self.robot_positions: list[PoseWithCovarianceStamped] = list()

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
        On the current position of the robot a circle is expanded until the robot radius is reached. Inside this circle the costs are set\
        to costs of lethal obstacles

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
        self.get_logger().debug(
            "Robot layer is inserting robot positions as lethal obstacles",
            throttle_duration_sec=2,
        )
        costmap.costmap.fill(np.uint8(0))
        costmap.global_frame_id = "map"

        radius_pixel: int = int(self.ROBOT_RADIUS * (1 / costmap.getResolution()))

        costmap = self.__draw_circle_simple(
            radius_pixel, min_i, max_i, min_j, max_j, costmap
        )

        return costmap

    def __draw_circle_simple(
        self,
        radius_pixel: int,
        min_i: int,
        min_j: int,
        max_i: int,
        max_j: int,
        costmap: PyCostmap2D,
    ) -> PyCostmap2D:
        # Iterate through all known robot positions
        for position in self.robot_positions:
            central_point = costmap_tools.pose_2_costmap(position.pose.pose, costmap)

            # The footprint is a circle => Only check if pixel in one quarter is within robot radius and
            # mirror setting costs to all 4 quadrants if pixel is within robot radius
            for x_offset in range(radius_pixel + 1):
                for y_offset in range(radius_pixel + 1):
                    # Check upper right quadrant
                    if (
                        costmap_tools.euclidian_distance_map_domain(
                            central_point,
                            (central_point[0] + x_offset, central_point[1] + y_offset),
                            costmap,
                        )
                        < self.ROBOT_RADIUS
                    ):
                        if costmap.getIndex(
                            central_point[0] + x_offset, central_point[0] + 1
                        ) > len(costmap.costmap):
                            continue
                        # Update costs in all 4 quadrants if pixel is within robot radius
                        # Upper right quadrant
                        costmap.setCost(
                            central_point[0] + x_offset,
                            central_point[1] + y_offset,
                            self.COST_ROBOTS,
                        )
                        # Upper left quadrant
                        costmap.setCost(
                            central_point[0] - x_offset,
                            central_point[1] + y_offset,
                            self.COST_ROBOTS,
                        )
                        #  Lower right quadrant
                        costmap.setCost(
                            central_point[0] + x_offset,
                            central_point[1] - y_offset,
                            self.COST_ROBOTS,
                        )
                        # Lower left quadrant
                        costmap.setCost(
                            central_point[0] - x_offset,
                            central_point[1] - y_offset,
                            self.COST_ROBOTS,
                        )

        return costmap

    def __draw_circle_opimized(
        self,
        radius_pixel: int,
        min_i: int,
        min_j: int,
        max_i: int,
        max_j: int,
        costmap: PyCostmap2D,
    ) -> PyCostmap2D:
        # Getting a generic circle
        y_indices, x_indices = np.ogrid[
            -radius_pixel : radius_pixel + 1, -radius_pixel : radius_pixel + 1
        ]
        # Convert it to indices in a 1d array which the costmap uses in the background
        in_circle = x_indices**2 + y_indices**2 <= radius_pixel**2
        indices_in_circle = np.where(in_circle)

        for position in self.robot_positions:
            central_point = costmap_tools.pose_2_costmap(position.pose.pose, costmap)
            # Calculate indices in the costmap where the circle is projected into
            indices_in_1d = (
                central_point[1] + indices_in_circle[0]
            ) * costmap.getSizeInCellsX() + (central_point[0] + indices_in_circle[1])
            # Cut all values off which are outside the update window
            indices_in_update_window = np.where(
                (central_point[0] + indices_in_circle[1] >= min_i)
                & (central_point[0] + indices_in_circle[1] < max_i)
                & (central_point[1] + indices_in_circle[0] >= min_j)
                & (central_point[1] + indices_in_circle[0] < max_j)
            )
            # Set cost
            costmap.costmap[indices_in_1d] = self.COST_ROBOTS

        return costmap

    def __update_robot_states(self, msg: RobotStates) -> None:
        """
        Callback function for the robot states subscriber. It basically updates the positions of other roboters
        """
        # Clear last known positions
        self.robot_positions.clear()
        # Add new position of robots to list
        for robot in msg.robot_states:
            if robot.name_space != self.get_namespace():
                self.robot_positions.append(robot.pose)


def main(args=None):
    """
    Start the node. It basically initializes the ROS2 context and creates a instance of Astar planner
    :meta private:
    """
    # Initialize node context
    rclpy.init(args=args)
    # Create ROS2 Node
    ros2_node = RobotLayer()
    # Run node
    rclpy.spin(ros2_node)
    # Cleanup
    ros2_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
