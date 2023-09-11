import math
from typing import Tuple

from nav2_simple_commander.costmap_2d import PyCostmap2D
from sopias4_framework.tools.ros2 import costmap_tools


class NeighborFinder:
    def __init__(
        self,
        costmap: PyCostmap2D,
        step_size: int = 1,
        weight_distance_costs: float = 1.0,
        weight_costmap_costs: float = 1.0,
    ) -> None:
        self.costmap: PyCostmap2D = costmap
        self.LETHAL_COST = 250
        self.step_size: int = step_size
        self.weight_distance_costs: float = weight_distance_costs
        self.weight_costmap_costs: float = weight_costmap_costs

    def find_neighbors_coordinates(
        self,
        node: Tuple[int, int],
    ) -> list[Tuple[Tuple[int, int], float]]:
        """
        Identifies neighbor nodes inspecting the 8 adjacent neighbors and is working woth coordinates. Checks if neighbor is inside the map boundaries\
        and if is not an obstacle according to a threshold. The costs of a neighbor is the sum of the distance between the node and the \
        found neighbor and the cost of the pixel which the neighbor represents.

        Args:
            node (tuple(int,int)): The node as an x,y-coordinate in the costmap for which the neighbors should be found
            costmap (nav2_simplecommander.costmap_2d.PyCostmap2D): The costmap in which it should be searched.
            step_size (int, optional): The step size to the next neighbor in pixel. Defaults to 1
            weight_distance_costs (float, optional): The weighting how much the distance cost should be added to the total cost of the neighbor \
                                                                                i.e. total_cost=weight_distance_costs * distance_cost + sum(other_weighted_costs). Defaults to 1.0
            weight_costmap_costs (float, optional): The weighting how much the costs from the costmap should be added to the total cost of the neighbor \
                                                                                i.e. total_cost=weight_costmap_costs * distance_cost + sum(other_weighted_costs). Defaults to 1.0
        Returns:
            list(tuple(tuple(int,int), float)): A list with valid neighbour nodes as a tuple with [x,y-coordinates, step_cost] pairs
        """
        neighbors = []

        upper_neighbor = (node[0], node[1] + self.step_size)
        if self.__check_constrains(node_to_check=upper_neighbor):
            neighbors.append(
                (
                    upper_neighbor,
                    self.__calculate_cost(
                        node_to_calculate=upper_neighbor,
                    ),
                )
            )

        left_neighbor = (node[0] - self.step_size, node[1])
        if self.__check_constrains(node_to_check=left_neighbor):
            neighbors.append(
                (
                    left_neighbor,
                    self.__calculate_cost(
                        node_to_calculate=left_neighbor,
                    ),
                )
            )

        upper_left_neighbor = (node[0] - self.step_size, node[1] + self.step_size)
        if self.__check_constrains(node_to_check=upper_left_neighbor):
            neighbors.append((upper_left_neighbor, upper_left_neighbor))

        upper_right_neighbor = (node[0] + self.step_size, node[1] + self.step_size)
        if self.__check_constrains(node_to_check=upper_right_neighbor):
            neighbors.append(
                (
                    upper_right_neighbor,
                    self.__calculate_cost(
                        node_to_calculate=upper_right_neighbor,
                    ),
                )
            )

        right_neighbor = (node[0] + self.step_size, node[1])
        if self.__check_constrains(node_to_check=right_neighbor):
            neighbors.append(
                (
                    right_neighbor,
                    self.__calculate_cost(
                        node_to_calculate=right_neighbor,
                    ),
                )
            )

        lower_left_neighbor = (node[0] - self.step_size, node[1] - self.step_size)
        if self.__check_constrains(node_to_check=lower_left_neighbor):
            neighbors.append(
                (
                    lower_left_neighbor,
                    self.__calculate_cost(
                        node_to_calculate=lower_left_neighbor,
                    ),
                )
            )

        lower_neighbor = (node[0], node[1] - self.step_size)
        if self.__check_constrains(
            node_to_check=lower_neighbor,
        ):
            neighbors.append(
                (
                    lower_neighbor,
                    self.__calculate_cost(
                        node_to_calculate=lower_neighbor,
                    ),
                )
            )

        lower_right_neighbor = (node[0] + self.step_size, node[1] - self.step_size)
        if self.__check_constrains(node_to_check=lower_right_neighbor):
            neighbors.append(
                (
                    lower_right_neighbor,
                    self.__calculate_cost(
                        node_to_calculate=lower_right_neighbor,
                    ),
                )
            )

        return neighbors

    def find_neighbors_index(
        self,
        node_index: int,
    ) -> list[Tuple[int, float]]:
        """
        Identifies neighbor nodes inspecting the 8 adjacent neighbors and is working with index in 1d costmap array directly. Checks if neighbor is inside the map boundaries\
        and if is not an obstacle according to a threshold. The costs of a neighbor is the sum of the distance between the node and the \
        found neighbor and the cost of the pixel which the neighbor represents.

        Args:
            node (int): The node as an index in the 1d costmap array for which the neighbors should be found
            costmap (nav2_simplecommander.costmap_2d.PyCostmap2D): The costmap in which it should be searched
            step_size (int, optional): The step size to the next neighbor in pixel. Defaults to 1
            weight_distance_costs (float, optional): The weighting how much the distance cost should be added to the total cost of the neighbor \
                                                                                i.e. total_cost=weight_distance_costs * distance_cost + sum(other_weighted_costs). Defaults to 1.0
            weight_costmap_costs (float, optional): The weighting how much the costs from the costmap should be added to the total cost of the neighbor \
                                                                                i.e. total_cost=weight_costmap_costs * distance_cost + sum(other_weighted_costs). Defaults to 1.0
        Returns:
            list(tuple(int, float)): A list with valid neighbour nodes as a tuple with [index, step_cost] pairs
        """
        neighbors = []
        width: int = self.costmap.getSizeInCellsX()
        height: int = self.costmap.getSizeInCellsY()

        upper_neighbor: int = node_index - width * self.step_size
        if self.__check_constrains(
            node_to_check=upper_neighbor,
            additional_constraints=[upper_neighbor > 0],
        ):
            neighbors.append(
                (
                    upper_neighbor,
                    self.__calculate_cost(
                        node_to_calculate=upper_neighbor,
                    ),
                )
            )

        left_neighbor: int = node_index - self.step_size
        if self.__check_constrains(
            node_to_check=left_neighbor,
            additional_constraints=[left_neighbor % width > 0],
        ):
            neighbors.append(
                (
                    left_neighbor,
                    self.__calculate_cost(
                        node_to_calculate=left_neighbor,
                    ),
                )
            )

        upper_left_neighbor: int = node_index - width * self.step_size + self.step_size
        if self.__check_constrains(
            node_to_check=upper_left_neighbor,
            additional_constraints=[
                upper_left_neighbor > 0,
                upper_left_neighbor % width > 0,
            ],
        ):
            neighbors.append(
                (
                    upper_left_neighbor,
                    self.__calculate_cost(
                        node_to_calculate=upper_left_neighbor,
                        is_diagonal=True,
                    ),
                )
            )

        upper_right_neighbor: int = node_index - width * self.step_size - self.step_size
        if self.__check_constrains(
            node_to_check=upper_right_neighbor,
            additional_constraints=[
                upper_right_neighbor > 0,
                (upper_right_neighbor) % width != width - 1,
            ],
        ):
            neighbors.append(
                (
                    upper_right_neighbor,
                    self.__calculate_cost(
                        node_to_calculate=upper_right_neighbor,
                        is_diagonal=True,
                    ),
                )
            )

        right_neighbor: int = node_index + self.step_size
        if self.__check_constrains(
            node_to_check=right_neighbor,
            additional_constraints=[right_neighbor % width != (width + 1)],
        ):
            neighbors.append(
                (
                    right_neighbor,
                    self.__calculate_cost(
                        node_to_calculate=right_neighbor,
                    ),
                )
            )

        lower_left_neighbor: int = node_index + width - self.step_size
        if self.__check_constrains(
            node_to_check=lower_left_neighbor,
            additional_constraints=[
                lower_left_neighbor < height * width,
                lower_left_neighbor % width != 0,
            ],
        ):
            neighbors.append(
                (
                    lower_left_neighbor,
                    self.__calculate_cost(
                        node_to_calculate=lower_left_neighbor,
                        is_diagonal=True,
                    ),
                )
            )

        lower_neighbor: int = node_index + width * self.step_size
        if self.__check_constrains(
            node_to_check=lower_neighbor,
            additional_constraints=[lower_neighbor < width * height],
        ):
            neighbors.append(
                (
                    lower_neighbor,
                    self.__calculate_cost(
                        node_to_calculate=lower_neighbor,
                    ),
                )
            )

        lower_right_neighbor: int = node_index + width * self.step_size + self.step_size
        if self.__check_constrains(
            node_to_check=lower_right_neighbor,
            additional_constraints=[
                lower_right_neighbor <= height * width,
                lower_right_neighbor % width != (width - 1),
            ],
        ):
            neighbors.append(
                (
                    lower_right_neighbor,
                    self.__calculate_cost(
                        node_to_calculate=lower_right_neighbor,
                        is_diagonal=True,
                    ),
                )
            )

        return neighbors

    def __check_constrains(
        self,
        node_to_check: Tuple[int, int] | int,
        additional_constraints: list[bool] = [True],
    ) -> bool:
        """
        Checks if constraints of nodes are meeting the constraints for being added as an neighbor

        General approach:
        1. Check if new coordinate is out of costmap
        2. Check if new coordinate is lethal obstacle
        3. Check if all additional constraints are met if given. They are chained together in AND-logic

        Args:
            node_to_check(tuple(int, int) or int): The node for which the constraints should be checked in either coordinates or index of costmap array (technically the same)
            additional_constraints (list(bool), optional): List of additional constraints that can be checked. All of them need to be true \
                                                                                        in order to fulfill constraints (AND-logic). Remember that you can pass expressions inside\
                                                                                        of them as long as they return bool e.g. `number < threshold` could be passed
        Returns:
            bool: If constraints are fulfilled
        """
        if type(node_to_check) == Tuple[int, int]:
            is_index_above_lower_bound: bool = (
                node_to_check[0] >= 0 and node_to_check[0] >= 0
            )
            is_index_under_upper_bound: bool = (
                node_to_check[0] < self.costmap.getSizeInCellsX()
                and node_to_check[1] < self.costmap.getSizeInCellsY()
            )
            is_index_in_arraybounds: bool = (
                self.costmap.getIndex(node_to_check[0], node_to_check[1])
                <= len(self.costmap.costmap) - 1
            )
            # Conversion needed because it returns bool_ as data type and not bool
            # Also is_index_in_arraybounds must be true before performing check, otherwise a IndexError could be thrown
            if is_index_in_arraybounds:
                is_node_lethal: bool = bool(
                    self.costmap.getCostXY(node_to_check[0], node_to_check[1])
                    >= self.LETHAL_COST
                )
            else:
                return False
        elif type(node_to_check) == int:
            is_index_above_lower_bound: bool = node_to_check >= 0
            is_index_under_upper_bound: bool = node_to_check < len(self.costmap.costmap)
            is_index_in_arraybounds: bool = is_index_under_upper_bound
            # Conversion needed because it returns bool_ as data type and not bool.
            # Also is_index_in_arraybounds must be true before performing check, otherwise a IndexError could be thrown
            if is_index_in_arraybounds:
                is_node_lethal: bool = bool(
                    self.costmap.getCostIdx(node_to_check) >= self.LETHAL_COST
                )
            else:
                return False
        else:
            return False
        # print(additional_constraints)
        # print(not False in additional_constraints)
        if (
            is_index_above_lower_bound
            and is_index_under_upper_bound
            and is_index_in_arraybounds
            and not is_node_lethal
            and not False in additional_constraints
        ):
            return True

        return False

    def __calculate_cost(
        self,
        node_to_calculate: int | Tuple[int, int],
        is_diagonal: bool = False,
    ) -> float:
        """
        Calculate the cost of a given node. It respect the step costs an the cost from the costmap which are factors like areas that should be avoided etc.

        Args:
            node_to_calculate(tuple(int, int) or int): The node for which the constraints should be checked in either coordinate or index notation
            costmap (nav2_simplecommander.costmap_2d.PyCostmap2D): The costmap from which the costs should be taken
            is_diagonal (bool) = If the node is a diagonal neighbor
            step_size (int: The step size to the next neighbor in pixel
            weight_distance_costs (float, optional): The weighting how much the distance cost should be added to the total cost of the neighbor \
                                                                                i.e. total_cost=weight_distance_costs * distance_cost + sum(other_weighted_costs)
            weight_costmap_costs (float, optional): The weighting how much the costs from the costmap should be added to the total cost of the neighbor \
                                                                                i.e. total_cost=weight_costmap_costs * distance_cost + sum(other_weighted_costs)
        Returns:
            bool: If constraints are fulfilled
        """
        # length of diagonal = length of one side by the square root of 2 (1.41421)
        diagonal_step_cost: float = (
            self.step_size * 1.41421 * self.weight_distance_costs
        )
        step_cost = self.step_size * self.weight_distance_costs
        if type(node_to_calculate) == int:
            if is_diagonal:
                cost: float = (
                    diagonal_step_cost
                    + float(self.costmap.getCostIdx(node_to_calculate))
                    * self.weight_costmap_costs
                )
            else:
                cost: float = (
                    step_cost
                    + float(self.costmap.getCostIdx(node_to_calculate))
                    * self.weight_costmap_costs
                )
        elif type(node_to_calculate) == Tuple[int, int]:
            if is_diagonal:
                cost: float = (
                    diagonal_step_cost
                    + float(
                        self.costmap.getCostXY(
                            node_to_calculate[0], node_to_calculate[1]
                        )
                    )
                    * self.weight_costmap_costs
                )
            else:
                cost: float = (
                    step_cost
                    + float(
                        self.costmap.getCostXY(
                            node_to_calculate[0], node_to_calculate[1]
                        )
                    )
                    * self.weight_costmap_costs
                )
        else:
            cost = math.inf

        return cost

    def euclidian_distance_map_domain(
        self,
        start: Tuple[int, int] | int,
        goal: Tuple[int, int] | int,
    ) -> float:
        """
        Calculates the euclidian distance between two coordinates in the costmap space.  For this purpose,\
        it calculates the distance in the costmap domain (pixels) and them multiplies it with the resolution\
        of the map to get the distance in meters.

        Args:
            start (tuple(int, int) or int): The start of the distance as an x,y-coordinate or and index in the costmap
            goal (tuple(int, int)): The goal of the distance as an x,y-coordinate in the costmap
            costmap (nav2_simplecommander.costmap_2d.PyCostmap2D): The costmap in which the distance is calculated

        Returns:
            float: The euclidian distance in meters
        """

        return (
            self.euclidian_distance_pixel_domain(start, goal)
            * self.costmap.getResolution()
        )

    def euclidian_distance_pixel_domain(
        self, start: Tuple[int, int] | int, goal: Tuple[int, int] | int
    ) -> float:
        """
        Calculates the euclidian distance between two coordinates in the costmap space.  For this purpose,\
        it calculates the distance in the costmap domain (pixels) and them multiplies it with the resolution\
        of the map to get the distance in meters.

        Args:
            start (tuple(int, int) or int): The start of the distance as an x,y-coordinate or index in the costmap
            goal (tuple(int, int)): The goal of the distance as an x,y-coordinate in the costmap
            costmap (nav2_simplecommander.costmap_2d.PyCostmap2D): The costmap in which the distance is calculated

        Returns:
            float: The euclidian distance in meters
        """
        if type(start) == int:
            start = costmap_tools.index_2_costmap(start, self.costmap)
        if type(goal) == int:
            goal = costmap_tools.index_2_costmap(goal, self.costmap)

        return math.sqrt((start[0] - goal[0]) ** 2 + (start[1] - goal[1]) ** 2)  # type: ignore
