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
    A costmap layer into which the footprint of all known robots in the environment are set as an cost.

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
        self.get_logger().set_level(30)

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
        # TODO If neccessary only update within the specified boundary window
        # Set cost of all pixels in costmap to zero, because only the costs calculated by this layer should be included in the map.
        # In the plugin bridge, all layers get combined so the cleared data doesn't get lost if it is up to date
        self.get_logger().debug(
            "Robot layer is inserting robot positions as lethal obstacles",
            throttle_duration_sec=2,
        )
        costmap.costmap.fill(np.uint8(0))

        # Iterate through all known robot positions
        for position in self.robot_positions:
            central_point = costmap_tools.pose_2_costmap(position.pose.pose, costmap)
            radius_pixel: int = int(self.ROBOT_RADIUS * (1 / costmap.getResolution()))

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
