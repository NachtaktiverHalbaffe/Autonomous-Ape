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
        super().__init__(
            node_name="planner_astar", plugin_name="astar"
        ) if namespace is None else super().__init__(
            node_name="planner_astar", plugin_name="astar", namespace=namespace
        )

        self.get_logger().info("Started node")
        self.get_logger().set_level(30)

    def generate_path(
        self, start: Tuple[int, int], goal: Tuple[int, int], goal_tolerance: float = 0.2
    ) -> list[Tuple[int, int]]:
        """
        Here the path finding algorithm should be implemented. Some tips for implementing the algorithm:
        1. Use python sets and dicts instead of lists. They are way faster when a specific value is searched, inserted or updated
        2. Be careful with loops when iterating through datasets. When applying first tip, there are often ways to directly update, \
            searching or updating values in datasets instead of iterating through them
        3. The module costmap_tools from sopias4_framework.tools.ros2 package has useful tools for interacting with the given costmap. \
            Remember that the costmap is stored in self.costmap of this class

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
        open_canditates: set = set()
        # A list of all nodes which are already processed
        list_processed: set = set()

        # Append start node to
        open_canditates.add(
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
        while len(open_canditates) != 0:
            # --- Search node which gets processed this iteration ---
            # Get current node which is the one which has the minimal costs in the set of open canditates
            current = min(open_canditates, key=lambda x: x[1])
            current_index: int = current[0]
            open_canditates.remove(current)

            # If current_node is the goal, finish the algorithm execution
            if (
                costmap_tools.euclidian_distance_map_domain(
                    current_index, goal_index, self.costmap
                )
                <= goal_tolerance
            ):
                self.get_logger().debug("A* found goal node")
                if goal_index not in parents.keys():
                    parents[goal_index] = current_index
                path_found = True
                break

            # --- Process neighbors of nodes ---
            # Get neighbors
            neighbors: list[Tuple[int, float]] = costmap_tools.find_neighbors_index(
                current_index, self.costmap, step_size=1
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

                # We can simply check if the neighbor has already costs
                # associated with it to see if neighbor was visited already or not
                if final_costs.get(neighbor_index) is None:
                    # neighbor node is first visited
                    natural_costs[neighbor_index] = n_cost
                    final_costs[neighbor_index] = f_cost
                    parents[neighbor_index] = current_index
                    open_canditates.add((neighbor_index, f_cost))
                else:
                    # neighbor node was already visited# => update cost if cheaper
                    if f_cost < final_costs[neighbor_index]:
                        open_canditates.remove(
                            (neighbor_index, final_costs[neighbor_index])
                        )
                        open_canditates.add((neighbor_index, f_cost))
                        # Update costs
                        natural_costs[neighbor_index] = n_cost
                        final_costs[neighbor_index] = f_cost
                        parents[neighbor_index] = current_index

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
