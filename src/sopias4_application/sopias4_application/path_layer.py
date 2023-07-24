#!/usr/bin/env python3

from typing import Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.costmap_2d import PyCostmap2D
from nav_msgs.msg import Path
from scipy import ndimage
from skimage import draw
from sopias4_framework.nodes.layer_pyplugin import LayerPyPlugin
from sopias4_framework.tools.ros2 import costmap_tools

from sopias4_msgs.msg import RobotStates


class PathLayer(LayerPyPlugin):
    """
    A costmap layer into which the footprint of all known global navigation paths of all robots are set as an cost.

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
        self.COST_PATH: np.uint8 = np.uint8(100)
        self.ROBOT_RADIUS: float = 0.25

        self.robot_paths: list[Path]

        # Create own sub node for service clients so they can spin independently
        self.__sub_robot_states = self.create_subscription(
            RobotStates, "robot_states", self.__update_robot_states, 10
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
        # TODO If neccessary only update within the specified boundary window
        # Set cost of all pixels in costmap to zero, because only the costs calculated by this layer should be included in the map.
        # In the plugin bridge, all layers get combined so the cleared data doesn't get lost if it is up to date
        costmap.costmap.fill(0)
        # Convert 1d array costmap data to a 2d grid because it makes things easier and copy it into a new numpy array
        costmap_grid = costmap_tools.costmap_2_grid(costmap)

        # --- Set costs for path itself which is only 1 pixel wide ---
        # For each path of each registered robot
        for path in self.robot_paths:
            last_node: Tuple[int, int] = costmap_tools.pose_2_costmap(
                path.poses[0], costmap  # type: ignore
            )
            for node in path.poses[1::]:  # type: ignore
                current_node = costmap_tools.pose_2_costmap(node, costmap)
                # Basically a Bresenhams line algorithm implementation
                rr, cc = draw.line(
                    last_node[0], last_node[1], current_node[0], current_node[1]
                )
                costmap_grid[rr, cc] = self.COST_PATH
                last_node = current_node

        # --- Inflate the path so it is as thick as needed ---
        # Create a copy of the input array to avoid modifying the original
        inflated_arr = np.copy(costmap_grid)
        # Get the dimensions of the array
        rows, cols = costmap_grid.shape
        # Convert the distance space into pixel space 
        inflation_distance_pxl:int = int(self.ROBOT_RADIUS* (1/costmap.getResolution()))
        if inflation_distance_pxl <1:
            inflation_distance_pxl = 1
        
        # Iterate through the array
        for row in range(rows):
            for col in range(cols):
                if costmap_grid[row, col] == self.COST_PATH:
                    # Iterate through the neighboring cells within the inflation_distance
                    for i in range(max(0, row - inflation_distance_pxl), min(rows, row + inflation_distance_pxl + 1)):
                        for j in range(max(0, col - inflation_distance_pxl), min(cols, col + inflation_distance_pxl+ 1)):
                            # Check if the cell is within the inflation_distance
                            if costmap_tools.euclidian_distance_pixel_domain((row, col),( i, j)) <=  inflation_distance_pxl:
                                # Update the cell value to the target_value
                                inflated_arr[i, j] = self.COST_PATH
        
        # Write grid-based costmap data back into 1d data array of costmap
        costmap.costmap = costmap_tools.grid_2_costmap(costmap_grid)

        return costmap

    def __update_robot_states(self, msg: RobotStates) -> None:
        """
        Callback function for the robot states subscriber. It basically updates the positions of other roboters
        """
        # Clear last known positions
        self.robot_paths.clear()

        # Add new position of robots to list
        for robot in msg.robot_states:
            self.robot_paths.append(robot.nav_path)


def main(args=None):
    """
    Start the node. It basically initializes the ROS2 context and creates a instance of Astar planner
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
