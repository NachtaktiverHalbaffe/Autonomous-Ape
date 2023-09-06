"""
This contains multiple helpful functions to interact with a PyCostmap2D which is used in the Python implementations
of the Navigation2 plugins. Most of them a wrapper around the own functions of PyCostmap2 and only apply more appropriate
types to work with. Also the built-in functions of PyCostmap2D can often be helpful too.
"""

import math
from math import sqrt
from typing import Tuple

import numpy as np
from geometry_msgs.msg import Pose, PoseStamped
from nav2_simple_commander.costmap_2d import PyCostmap2D
from nav_msgs.msg import OccupancyGrid

LETHAL_COST = 250


def costmap_2_pose(x: int, y: int, costmap: PyCostmap2D) -> Pose:
    """
    Converts a PyCostmap2D map coordinate to an pose in the map frame by applying a transformation

    Args:
        x (int): The x-index of the cell that should be converted
        y (int): The y-index of the cell zhat should be converted
        costmap (PyCostmap2D): The costmap in which the map is located

    Returns:
        Pose: The pose of the cell in the map frame
    """
    pose: Pose = Pose()
    pose.position.x, pose.position.y = costmap.mapToWorld(x, y)

    return pose


def pose_2_costmap(pose: Pose | PoseStamped, costmap: PyCostmap2D) -> Tuple[int, int]:
    """
    Converts a pose from the global map frame to the indexes of a costmap map

    Args:
        pose (Pose): The pose that should be converted
        costmap(PyCostmap2D): The costmap in which the cell should be located

    Returns:
        int: The x-index of the cell
        int: The y-index of the cell
    """
    x: int = 0
    y: int = 0

    if type(pose) == Pose:
        x, y = costmap.worldToMap(pose.position.x, pose.position.y)
    elif type(pose) == PoseStamped:
        pose.header.frame_id = costmap.global_frame_id
        x, y = costmap.worldToMap(pose.pose.position.x, pose.pose.position.y)

    return x, y


def index_2_costmap(index: int, costmap: PyCostmap2D) -> Tuple[int, int]:
    """
    Converts a index from a costmap 1d array from to coodinates oin the costmap domain

    Args:
        index (int): The index that should be converted
        costmap(PyCostmap2D): The costmap in which the cell should be located

    Returns:
        int: The x-index of the cell
        int: The y-index of the cell
    """
    x: int = index % costmap.getSizeInCellsX()
    y: int = math.floor(index / costmap.getSizeInCellsX())

    return x, y


def costmap_2_grid(costmap: PyCostmap2D) -> np.ndarray:
    """
    Reshapes/creates a grid (2D array) from the costmap which is a 1D array

    Args:
        costmap (PyCostmap2D):  The costmap from which the 2D array should be generated

    Returns:
        np.ndarray: A 2-dimensional array of the costmap data
    """
    array2D = costmap.costmap.reshape(
        costmap.getSizeInCellsY(), costmap.getSizeInCellsX()
    )

    return array2D


def grid_2_costmap(grid: np.ndarray) -> np.ndarray:
    """
    Reshapes/flattes a grid (2d-array) to an array which the costmap uses internally for its costmap data

    Args:
        grid (np.ndarray): The grid which should be flattened

    Returns:
        np.ndarray: A 1-dimenionsal array
    """
    return grid.flatten()


def pycostmap2d_2_occupancygrid(pycostmap: PyCostmap2D) -> OccupancyGrid:
    """ 
    Converts a PyCostmap2D instance to a Occupancygrid message. Pycost2d is used internally by the 
    plugins and OccupancyGrid is used for the event and service communication

    Args:
        pycostmap (nav2_simplecommander.costmap_2d.PyCostmap2D): The PyCostmap2D which should be \
                                                                                                                       converted
    
    Returns:
        nav_msgs.msg.OccupancyGrid: The generated OccupancyGrid message
    """
    occ_grid = OccupancyGrid()
    # Convert np.uint8 array to list with ints in range 0 to 100
    SCALE_FACTOR = 100 / 255
    converted_array = np.round(pycostmap.costmap.astype(float) * SCALE_FACTOR).astype(
        int
    )
    occ_grid.data = converted_array.tolist()

    # occ_grid.data = [0] * len(pycostmap.costmap)
    # for i in range(len(occ_grid.data)):
    #     occ_grid.data[i]
    occ_grid.info.height = pycostmap.getSizeInCellsY()
    occ_grid.info.width = pycostmap.getSizeInCellsX()
    occ_grid.info.resolution = pycostmap.getOriginX()
    occ_grid.info.origin.position.x = pycostmap.getOriginX()
    occ_grid.info.origin.position.y = pycostmap.getOriginY()
    occ_grid.info.origin.position.z = 0.0
    occ_grid.info.origin.orientation.w = 0.0
    occ_grid.header.frame_id = pycostmap.getGlobalFrameID()

    return occ_grid


def find_neighbors_coordinates(
    node: Tuple[int, int],
    costmap: PyCostmap2D,
    step_size: int = 1,
    weight_distance_costs: float = 1.0,
    weight_costmap_costs: float = 1.0,
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

    upper_neighbor = (node[0], node[1] + step_size)
    if __check_constrains(node_to_check=upper_neighbor, costmap=costmap):
        neighbors.append(
            (
                upper_neighbor,
                __calculate_cost(
                    node_to_calculate=upper_neighbor,
                    costmap=costmap,
                    step_size=step_size,
                    weight_costmap_costs=weight_costmap_costs,
                    weight_distance_costs=weight_distance_costs,
                ),
            )
        )

    left_neighbor = (node[0] - step_size, node[1])
    if __check_constrains(node_to_check=left_neighbor, costmap=costmap):
        neighbors.append(
            (
                left_neighbor,
                __calculate_cost(
                    node_to_calculate=left_neighbor,
                    costmap=costmap,
                    step_size=step_size,
                    weight_costmap_costs=weight_costmap_costs,
                    weight_distance_costs=weight_distance_costs,
                ),
            )
        )

    upper_left_neighbor = (node[0] - step_size, node[1] + step_size)
    if __check_constrains(node_to_check=upper_left_neighbor, costmap=costmap):
        neighbors.append((upper_left_neighbor, upper_left_neighbor))

    upper_right_neighbor = (node[0] + step_size, node[1] + step_size)
    if __check_constrains(node_to_check=upper_right_neighbor, costmap=costmap):
        neighbors.append(
            (
                upper_right_neighbor,
                __calculate_cost(
                    node_to_calculate=upper_right_neighbor,
                    costmap=costmap,
                    step_size=step_size,
                    weight_costmap_costs=weight_costmap_costs,
                    weight_distance_costs=weight_distance_costs,
                ),
            )
        )

    right_neighbor = (node[0] + step_size, node[1])
    if __check_constrains(node_to_check=right_neighbor, costmap=costmap):
        neighbors.append(
            (
                right_neighbor,
                __calculate_cost(
                    node_to_calculate=right_neighbor,
                    costmap=costmap,
                    step_size=step_size,
                    weight_costmap_costs=weight_costmap_costs,
                    weight_distance_costs=weight_distance_costs,
                ),
            )
        )

    lower_left_neighbor = (node[0] - step_size, node[1] - step_size)
    if __check_constrains(node_to_check=lower_left_neighbor, costmap=costmap):
        neighbors.append(
            (
                lower_left_neighbor,
                __calculate_cost(
                    node_to_calculate=lower_left_neighbor,
                    costmap=costmap,
                    step_size=step_size,
                    weight_costmap_costs=weight_costmap_costs,
                    weight_distance_costs=weight_distance_costs,
                ),
            )
        )

    lower_neighbor = (node[0], node[1] - step_size)
    if __check_constrains(node_to_check=lower_neighbor, costmap=costmap):
        neighbors.append(
            (
                lower_neighbor,
                __calculate_cost(
                    node_to_calculate=lower_neighbor,
                    costmap=costmap,
                    step_size=step_size,
                    weight_costmap_costs=weight_costmap_costs,
                    weight_distance_costs=weight_distance_costs,
                ),
            )
        )

    lower_right_neighbor = (node[0] + step_size, node[1] - step_size)
    if __check_constrains(node_to_check=lower_right_neighbor, costmap=costmap):
        neighbors.append(
            (
                lower_right_neighbor,
                __calculate_cost(
                    node_to_calculate=lower_right_neighbor,
                    costmap=costmap,
                    step_size=step_size,
                    weight_costmap_costs=weight_costmap_costs,
                    weight_distance_costs=weight_distance_costs,
                ),
            )
        )

    return neighbors


def find_neighbors_index(
    node_index: int,
    costmap: PyCostmap2D,
    step_size: int = 1,
    weight_distance_costs: float = 1.0,
    weight_costmap_costs: float = 1.0,
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
    width: int = costmap.getSizeInCellsX()
    height: int = costmap.getSizeInCellsY()

    upper_neighbor: int = node_index - width * step_size
    if __check_constrains(
        node_to_check=upper_neighbor,
        costmap=costmap,
        additional_constraints=[upper_neighbor > 0],
    ):
        neighbors.append(
            (
                upper_neighbor,
                __calculate_cost(
                    node_to_calculate=upper_neighbor,
                    costmap=costmap,
                    step_size=step_size,
                    weight_costmap_costs=weight_costmap_costs,
                    weight_distance_costs=weight_distance_costs,
                ),
            )
        )

    left_neighbor: int = node_index - step_size
    if __check_constrains(
        node_to_check=left_neighbor,
        costmap=costmap,
        additional_constraints=[left_neighbor % width > 0],
    ):
        neighbors.append(
            (
                left_neighbor,
                __calculate_cost(
                    node_to_calculate=left_neighbor,
                    costmap=costmap,
                    step_size=step_size,
                    weight_costmap_costs=weight_costmap_costs,
                    weight_distance_costs=weight_distance_costs,
                ),
            )
        )

    upper_left_neighbor: int = node_index - width * step_size + step_size
    if __check_constrains(
        node_to_check=upper_left_neighbor,
        costmap=costmap,
        additional_constraints=[
            upper_left_neighbor > 0,
            upper_left_neighbor % width > 0,
        ],
    ):
        neighbors.append(
            (
                upper_left_neighbor,
                __calculate_cost(
                    node_to_calculate=upper_left_neighbor,
                    costmap=costmap,
                    is_diagonal=True,
                    step_size=step_size,
                    weight_costmap_costs=weight_costmap_costs,
                    weight_distance_costs=weight_distance_costs,
                ),
            )
        )

    upper_right_neighbor: int = node_index - width * step_size - step_size
    if __check_constrains(
        node_to_check=upper_right_neighbor,
        costmap=costmap,
        additional_constraints=[
            upper_right_neighbor > 0,
            (upper_right_neighbor) % width != width - 1,
        ],
    ):
        neighbors.append(
            (
                upper_right_neighbor,
                __calculate_cost(
                    node_to_calculate=upper_right_neighbor,
                    costmap=costmap,
                    is_diagonal=True,
                    step_size=step_size,
                    weight_costmap_costs=weight_costmap_costs,
                    weight_distance_costs=weight_distance_costs,
                ),
            )
        )

    right_neighbor: int = node_index + step_size
    if __check_constrains(
        node_to_check=right_neighbor,
        costmap=costmap,
        additional_constraints=[right_neighbor % width != (width + 1)],
    ):
        neighbors.append(
            (
                right_neighbor,
                __calculate_cost(
                    node_to_calculate=right_neighbor,
                    costmap=costmap,
                    step_size=step_size,
                    weight_costmap_costs=weight_costmap_costs,
                    weight_distance_costs=weight_distance_costs,
                ),
            )
        )

    lower_left_neighbor: int = node_index + width - step_size
    if __check_constrains(
        node_to_check=lower_left_neighbor,
        costmap=costmap,
        additional_constraints=[
            lower_left_neighbor < height * width,
            lower_left_neighbor % width != 0,
        ],
    ):
        neighbors.append(
            (
                lower_left_neighbor,
                __calculate_cost(
                    node_to_calculate=lower_left_neighbor,
                    costmap=costmap,
                    is_diagonal=True,
                    step_size=step_size,
                    weight_costmap_costs=weight_costmap_costs,
                    weight_distance_costs=weight_distance_costs,
                ),
            )
        )

    lower_neighbor: int = node_index + width * step_size
    if __check_constrains(
        node_to_check=lower_neighbor,
        costmap=costmap,
        additional_constraints=[lower_neighbor < width * height],
    ):
        neighbors.append(
            (
                lower_neighbor,
                __calculate_cost(
                    node_to_calculate=lower_neighbor,
                    costmap=costmap,
                    step_size=step_size,
                    weight_costmap_costs=weight_costmap_costs,
                    weight_distance_costs=weight_distance_costs,
                ),
            )
        )

    lower_right_neighbor: int = node_index + width * step_size + step_size
    if __check_constrains(
        node_to_check=lower_right_neighbor,
        costmap=costmap,
        additional_constraints=[
            lower_right_neighbor <= height * width,
            lower_right_neighbor % width != (width - 1),
        ],
    ):
        neighbors.append(
            (
                lower_right_neighbor,
                __calculate_cost(
                    node_to_calculate=lower_right_neighbor,
                    costmap=costmap,
                    is_diagonal=True,
                    step_size=step_size,
                    weight_costmap_costs=weight_costmap_costs,
                    weight_distance_costs=weight_distance_costs,
                ),
            )
        )

    return neighbors


def __check_constrains(
    node_to_check: Tuple[int, int] | int,
    costmap: PyCostmap2D,
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
        costmap (nav2_simplecommander.costmap_2d.PyCostmap2D): The costmap in which it should be searched.
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
            node_to_check[0] < costmap.getSizeInCellsX()
            and node_to_check[1] < costmap.getSizeInCellsY()
        )
        is_index_in_arraybounds: bool = (
            costmap.getIndex(node_to_check[0], node_to_check[1])
            <= len(costmap.costmap) - 1
        )
        # Conversion needed because it returns bool_ as data type and not bool
        # Also is_index_in_arraybounds must be true before performing check, otherwise a IndexError could be thrown
        if is_index_in_arraybounds:
            is_node_lethal: bool = bool(
                costmap.getCostXY(node_to_check[0], node_to_check[1]) >= LETHAL_COST
            )
        else:
            return False
    elif type(node_to_check) == int:
        is_index_above_lower_bound: bool = node_to_check >= 0
        is_index_under_upper_bound: bool = node_to_check < len(costmap.costmap)
        is_index_in_arraybounds: bool = is_index_under_upper_bound
        # Conversion needed because it returns bool_ as data type and not bool.
        # Also is_index_in_arraybounds must be true before performing check, otherwise a IndexError could be thrown
        if is_index_in_arraybounds:
            is_node_lethal: bool = bool(
                costmap.getCostIdx(node_to_check) >= LETHAL_COST
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
    node_to_calculate: int | Tuple[int, int],
    costmap: PyCostmap2D,
    step_size: int,
    weight_distance_costs: float,
    weight_costmap_costs: float,
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
    diagonal_step_cost: float = step_size * 1.41421 * weight_distance_costs
    step_cost = step_size * weight_distance_costs
    if type(node_to_calculate) == int:
        if is_diagonal:
            cost: float = (
                diagonal_step_cost
                + float(costmap.getCostIdx(node_to_calculate)) * weight_costmap_costs
            )
        else:
            cost: float = (
                step_cost
                + float(costmap.getCostIdx(node_to_calculate)) * weight_costmap_costs
            )
    elif type(node_to_calculate) == Tuple[int, int]:
        if is_diagonal:
            cost: float = (
                diagonal_step_cost
                + float(costmap.getCostXY(node_to_calculate[0], node_to_calculate[1]))
                * weight_costmap_costs
            )
        else:
            cost: float = (
                step_cost
                + float(costmap.getCostXY(node_to_calculate[0], node_to_calculate[1]))
                * weight_costmap_costs
            )
    else:
        cost = math.inf

    return cost


def euclidian_distance_map_domain(
    start: Tuple[int, int] | int, goal: Tuple[int, int] | int, costmap: PyCostmap2D
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
        euclidian_distance_pixel_domain(start, goal, costmap) * costmap.getResolution()
    )


def euclidian_distance_pixel_domain(
    start: Tuple[int, int] | int, goal: Tuple[int, int] | int, costmap: PyCostmap2D
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
        start = index_2_costmap(start, costmap)
    if type(goal) == int:
        goal = index_2_costmap(goal, costmap)

    return sqrt((start[0] - goal[0]) ** 2 + (start[1] - goal[1]) ** 2)  # type: ignore


if __name__ == "__main__":
    print("This is only an library, dont execute this Python script at it's own")
