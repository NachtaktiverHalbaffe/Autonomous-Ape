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

    def __init__(self, namespace: str | None = None) -> None:
        if namespace is None:
            super().__init__(node_name="planner_astar", plugin_name="astar")
        else:
            super().__init__(
                node_name="planner_astar", plugin_name="astar", namespace=namespace
            )
        self.get_logger().set_level(10)
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
        start_index: int = self.costmap.getIndex(start[0], start[1])
        goal_index: int = self.costmap.getIndex(goal[0], goal[1])

        if start_index >= len(self.costmap.costmap) or goal_index > len(
            self.costmap.costmap
        ):
            raise IndexError("Specified start or goal arent in the given costmap")

        # A list of all nodes, which are open to being processed. Each node must be a tuple with first element being a
        # tuple with the x,y-coordinate of the node and the second element it's cost
        list_open_canditates: list = []
        # A list of all nodes which are already processed
        list_processed: set = set()

        # Append start node to
        list_open_canditates.append(
            (
                start_index,
                costmap_tools.euclidian_distance_pixel_domain(
                    start_index, goal, self.costmap
                ),
            )
        )
        # Dict for mapping children to parent
        parents = dict()
        # Dict containing the costs of the processed nodes. These are the normal costs like in dijkstra without the heuristic applied
        natural_costs: dict = dict()
        natural_costs[start_index] = 0
        # Final costs which are natural_costs + heuristic costs
        final_costs: dict = dict()
        final_costs[start_index] = costmap_tools.euclidian_distance_pixel_domain(
            start_index, goal_index, self.costmap
        )

        path_found: bool = False
        self.get_logger().debug("Initialized A* algorithm")
        #######################
        # ------ A* execution ------
        #######################
        while len(list_open_canditates) != 0:
            # --- Search node which gets processed this iteration ---
            self.get_logger().debug("Running new iteration", throttle_duration_sec=1)
            # Sort open list according to the lowest cost (second element of each sublist)
            list_open_canditates.sort(key=lambda x: x[1])
            # Take first element of the open candidates because it has the lowest costs
            current_index: int = list_open_canditates.pop(0)[0]
            self.get_logger().debug(
                f"Current distance to goal: {costmap_tools.euclidian_distance_pixel_domain(current_index, goal_index, self.costmap)}",
                throttle_duration_sec=1,
            )

            # If current_node is the goal, finish the algorithm execution
            if current_index == goal_index:
                self.get_logger().debug("A* found goal node")
                path_found = True
                break

            # --- Process neighbors of nodes ---
            # Get neighbors
            neighbors: list[Tuple[int, float]] = costmap_tools.find_neighbors_index(
                current_index, self.costmap
            )
            for neighbor_index, cost in neighbors:
                # Skip node if already processed
                if neighbor_index in list_processed:
                    continue

                # Calculate natural costs of current neighbor node like in dijkstra
                n_cost = natural_costs[current_index] + cost
                # Add heuristic (euclidian distance) to costs
                f_cost = n_cost + costmap_tools.euclidian_distance_pixel_domain(
                    neighbor_index, goal, self.costmap
                )

                # Check if neighbor is in canditates list
                is_already_canditate: bool = False
                for idx, element in enumerate(list_open_canditates):
                    # Neighbors is already an candidate
                    if element[0] == neighbor_index:
                        # Update cost of canditate if current costs are lower
                        if f_cost < final_costs[neighbor_index]:
                            natural_costs[neighbor_index] = n_cost
                            final_costs[neighbor_index] = f_cost
                            parents[neighbor_index] = current_index
                            list_open_canditates[idx] = (neighbor_index, f_cost)
                        is_already_canditate = True
                        break

                if not is_already_canditate:
                    # Append node to list of canditates if not already on there
                    natural_costs[neighbor_index] = n_cost
                    final_costs[neighbor_index] = f_cost
                    parents[neighbor_index] = current_index
                    list_open_canditates.append((neighbor_index, f_cost))

            # --- Add node to list of processed nodes so it doesn't get visited --
            list_processed.add(current_index)

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
            node_index = goal_index
            path.append(goal)
            while node_index != start_index:
                node_cor = costmap_tools.index_2_costmap(
                    index=node_index, costmap=self.costmap
                )
                path.append(node_cor)
                node_index = parents[node_index]

        path = path[::-1]

        self.get_logger().info("A* finished generating a path")
        return path


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
