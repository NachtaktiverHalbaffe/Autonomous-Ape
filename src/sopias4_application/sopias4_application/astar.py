#!/usr/bin/env python3

from math import sqrt
from typing import Tuple

import rclpy
from nav2_simple_commander.costmap_2d import PyCostmap2D
from sopias4_framework.nodes.planner_pyplugin import PlannerPyPlugin
from sopias4_framework.tools.ros2 import costmap_tools


class Astar(PlannerPyPlugin):
    """
    An A* path finding algorithm. It is an improved version of Dijkstras Algorithm. Look at https://www.youtube.com/watch?v=A60q6dcoCjw \
    for an good explanation of the algorithm. This is a self contained node
    """

    def __init__(self) -> None:
        super().__init__(node_name="planner_astar", plugin_name="astar")
        self.get_logger().info("Started node")

    def generate_path(
        self, start: Tuple[int, int], goal: Tuple[int, int]
    ) -> list[Tuple[int, int]]:
        """
        Here the path finding algorithm should be implemented

        Args:
            start (tuple(int, int)): The position from which the path should start as an x,y-coordinate in the costmap
            goal (tuple(int, int)): The position in which the path should end as an x,y-coordinate in the costmap
            costmap (nav2_simple_commander.costmap_2d.PyCostmap2D): The current costmap which contains the costs of each region. Low costs means free regions and higher\
                                                costs that there may be an obstacle or other reasons why the robot should avoid this region
        
        Returns:
            list(tuple(int,int)): The generated path as a list of x,y-coordinates in the costmap
        """
        ###################
        # --- Initialization ----
        ###################
        # The global path
        path = []
        self.get_logger().info("Using A* algorithm")
        # A list of all nodes, which are open to being processed. Each node must be a tuple with first element being a
        # tuple with the x,y-coordinate of the node and the second element it's cost
        list_open_canditates: list = []
        # A list of all nodes which are already processed
        list_processed: set = set()

        # Append start node to
        list_open_canditates.append(
            (start, costmap_tools.euclidian_distance(start, goal, self.costmap))
        )
        # Dict for mapping children to parent
        parents = dict()

        path_found: bool = False
        self.get_logger().debug("Initialized A* algorithm")
        #######################
        # ------ A* execution ------
        #######################
        while len(list_open_canditates) != 0:
            # --- Search node which gets processed this iteration ---
            # Sort open list according to the lowest cost (second element of each sublist)
            list_open_canditates.sort(key=lambda x: x[1])
            # Take first element of the open candidates because it has the lowest costs
            current_node = list_open_canditates.pop(0)[0]

            # If current_node is the goal, finish the algorithm execution
            if current_node == goal:
                self.get_logger().debug("A* found goal node")
                path_found = True
                break

            # --- Process neighbors of nodes ---
            # Get neighbors
            neighbors: list[
                Tuple[Tuple[int, int], float]
            ] = costmap_tools.find_neighbors(current_node, self.costmap)
            for neighbor_node, cost in neighbors:
                # Skip node if already processed
                if neighbor_node in list_processed:
                    continue

                # Add heuristic (euclidian distance) to costs
                final_cost = cost + costmap_tools.euclidian_distance(
                    neighbor_node, goal, self.costmap
                )

                # Check if neighbor is in canditates list
                is_already_canditate: bool = False
                for idx, element in enumerate(list_open_canditates):
                    # Neighbors is already an candidate
                    if element[0] == neighbor_node:
                        is_already_canditate = True
                        # Update cost of canditate if current costs are lower
                        if final_cost < element[1]:
                            list_open_canditates[idx] = (element[0], final_cost)
                            parents[neighbor_node] = current_node
                        break

                # Append node to list of canditates if not already on there
                if not is_already_canditate:
                    list_open_canditates.append((neighbor_node, final_cost))
                    parents[neighbor_node] = current_node

            # --- Add node to list of processed nodes so it doesn't get visited --
            list_processed.add(current_node)

        ######################
        # --- Post-processing ---
        ######################
        self.get_logger().debug(
            "A* done traversing nodes in list of open candiatets. Beginning post-processing"
        )

        if not path_found:
            self.get_logger().warning("A* couldn't find path")
            return path

        # Reconstruct path by working backwards from target
        if path_found:
            node = goal
            path.append(node)
            while node != start:
                path.append(node)
                node = parents[node]

        path = path[::-1]

        self.get_logger().info("A* finished generating a path")
        return path

    def destroy_node(self):
        self.get_logger().info("Shutting down node")
        self.__plugin_bridge_server.destroy()
        super().destroy_node()


def main(args=None):
    """
    Start the node. It basically initializes the ROS2 context and creates a instance of Astar planner
    :meta private:
    """
    # Initialize node context
    rclpy.init(args=args)
    # Create ROS2 Node
    ros2_node = Astar()
    # Run node
    rclpy.spin(ros2_node)
    # Cleanup
    ros2_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
