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

LETHAL_COST = 254


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
    # pose.position.x = x * costmap.getResolution() + costmap.getOriginX()
    # pose.position.y = y * costmap.getResolution() + costmap.getOriginY()
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
    # x: int = int(round((pose.position.x - costmap.origin_x) / costmap.getResolution()))
    # y: int = int(round((pose.position.y - costmap.origin_y) / costmap.getResolution()))
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
    # x: int = int(round((pose.position.x - costmap.origin_x) / costmap.getResolution()))
    # y: int = int(round((pose.position.y - costmap.origin_y) / costmap.getResolution()))
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
    array2D = np.array(costmap.costmap, dtype=np.int8).reshape(
        costmap.size_y, costmap.size_x
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
    occ_grid.data = pycostmap.costmap
    occ_grid.info.height = pycostmap.size_y
    occ_grid.info.width = pycostmap.size_x
    occ_grid.info.resolution = pycostmap.resolution
    occ_grid.info.origin.position.x = pycostmap.origin_x
    occ_grid.info.origin.position.y = pycostmap.origin_y

    return occ_grid


def find_neighbors_coordinates(
    node: Tuple[int, int], costmap: PyCostmap2D
) -> list[Tuple[Tuple[int, int], float]]:
    """
    Identifies neighbor nodes inspecting the 8 adjacent neighbors and is working woth coordinates. Checks if neighbor is inside the map boundaries\
    and if is not an obstacle according to a threshold. The costs of a neighbor is the sum of the distance between the node and the \
    found neighbor and the cost of the pixel which the neighbor represents.

    Args:
        node (tuple(int,int)): The node as an x,y-coordinate in the costmap for which the neighbors should be found
        costmap (nav2_simplecommander.costmap_2d.PyCostmap2D): The costmap in which it should be searched.
    Returns:
         list(tuple(tuple(int,int), float)): A list with valid neighbour nodes as a tuple with [x,y-coordinates, step_cost] pairs
    """
    neighbors = []

    upper_neighbor = (node[0], node[1] + 1)
    if __check_constrains(node_to_check=upper_neighbor, costmap=costmap):
        neighbors.append(
            (
                upper_neighbor,
                __calculate_cost(node_to_calculate=upper_neighbor, costmap=costmap),
            )
        )

    left_neighbor = (node[0] - 1, node[1])
    if __check_constrains(node_to_check=left_neighbor, costmap=costmap):
        neighbors.append(
            (
                left_neighbor,
                __calculate_cost(node_to_calculate=left_neighbor, costmap=costmap),
            )
        )

    upper_left_neighbor = (node[0] - 1, node[1] + 1)
    if __check_constrains(node_to_check=upper_left_neighbor, costmap=costmap):
        neighbors.append((upper_left_neighbor, upper_left_neighbor))

    upper_right_neighbor = (node[0] + 1, node[1] + 1)
    if __check_constrains(node_to_check=upper_right_neighbor, costmap=costmap):
        neighbors.append(
            (
                upper_right_neighbor,
                __calculate_cost(
                    node_to_calculate=upper_right_neighbor, costmap=costmap
                ),
            )
        )

    right_neighbor = (node[0] + 1, node[1])
    if __check_constrains(node_to_check=right_neighbor, costmap=costmap):
        neighbors.append(
            (
                right_neighbor,
                __calculate_cost(node_to_calculate=right_neighbor, costmap=costmap),
            )
        )

    lower_left_neighbor = (node[0] - 1, node[1] - 1)
    if __check_constrains(node_to_check=lower_left_neighbor, costmap=costmap):
        neighbors.append(
            (
                lower_left_neighbor,
                __calculate_cost(
                    node_to_calculate=lower_left_neighbor, costmap=costmap
                ),
            )
        )

    lower_neighbor = (node[0], node[1] - 1)
    if __check_constrains(node_to_check=lower_neighbor, costmap=costmap):
        neighbors.append(
            (
                lower_neighbor,
                __calculate_cost(node_to_calculate=lower_neighbor, costmap=costmap),
            )
        )

    lower_right_neighbor = (node[0] + 1, node[1] - 1)
    if __check_constrains(node_to_check=lower_right_neighbor, costmap=costmap):
        neighbors.append(
            (
                lower_right_neighbor,
                __calculate_cost(
                    node_to_calculate=lower_right_neighbor, costmap=costmap
                ),
            )
        )

    return neighbors


def find_neighbors_index(
    node_index: int, costmap: PyCostmap2D
) -> list[Tuple[int, float]]:
    """
    Identifies neighbor nodes inspecting the 8 adjacent neighbors and is working with index in 1d costmap array directly. Checks if neighbor is inside the map boundaries\
    and if is not an obstacle according to a threshold. The costs of a neighbor is the sum of the distance between the node and the \
    found neighbor and the cost of the pixel which the neighbor represents.

    Args:
        node (int): The node as an index in the 1d costmap array for which the neighbors should be found
        costmap (nav2_simplecommander.costmap_2d.PyCostmap2D): The costmap in which it should be searched.
    Returns:
         list(tuple(int, float)): A list with valid neighbour nodes as a tuple with [index, step_cost] pairs
    """
    neighbors = []
    width: int = costmap.getSizeInCellsX()
    height: int = costmap.getSizeInCellsY()

    upper_neighbor = node_index - width
    if upper_neighbor > 0 and costmap.getCostIdx(upper_neighbor) < LETHAL_COST:
        neighbors.append(
            (
                upper_neighbor,
                __calculate_cost(node_to_calculate=upper_neighbor, costmap=costmap),
            )
        )

    left_neighbor = node_index - 1
    if left_neighbor % width > 0 and costmap.getCostIdx(left_neighbor) < LETHAL_COST:
        neighbors.append(
            (
                left_neighbor,
                __calculate_cost(node_to_calculate=left_neighbor, costmap=costmap),
            )
        )

    upper_left_neighbor = node_index - width + 1
    if (
        upper_left_neighbor > 0
        and upper_left_neighbor % width > 0
        and costmap.getCostIdx(upper_left_neighbor) < LETHAL_COST
    ):
        neighbors.append(
            (
                upper_left_neighbor,
                __calculate_cost(
                    node_to_calculate=upper_left_neighbor,
                    costmap=costmap,
                    is_diagonal=True,
                ),
            )
        )

    upper_right_neighbor = node_index - width - 1
    if (
        upper_right_neighbor > 0
        and (upper_right_neighbor) % width != width - 1
        and costmap.getCostIdx(upper_right_neighbor) < LETHAL_COST
    ):
        neighbors.append(
            (
                upper_right_neighbor,
                __calculate_cost(
                    node_to_calculate=upper_right_neighbor,
                    costmap=costmap,
                    is_diagonal=True,
                ),
            )
        )

    right_neighbor = node_index + 1
    if right_neighbor % width != (width + 1) and right_neighbor < LETHAL_COST:
        neighbors.append(
            (
                right_neighbor,
                __calculate_cost(node_to_calculate=right_neighbor, costmap=costmap),
            )
        )

    lower_left_neighbor = node_index + width - 1
    if (
        lower_left_neighbor < height * width
        and lower_left_neighbor % width != 0
        and costmap.getCostIdx(lower_left_neighbor) < LETHAL_COST
    ):
        neighbors.append(
            (
                lower_left_neighbor,
                __calculate_cost(
                    node_to_calculate=lower_left_neighbor,
                    costmap=costmap,
                    is_diagonal=True,
                ),
            )
        )

    lower_neighbor = node_index + width
    if lower_neighbor < width * height and costmap.getCostIdx(lower_neighbor):
        neighbors.append(
            (
                lower_neighbor,
                __calculate_cost(node_to_calculate=lower_neighbor, costmap=costmap),
            )
        )

    lower_right_neighbor = node_index + width + 1
    if (
        (lower_right_neighbor) <= height * width
        and lower_right_neighbor % width != (width - 1)
        and costmap.getCostIdx(lower_right_neighbor) < LETHAL_COST
    ):
        neighbors.append(
            (
                lower_right_neighbor,
                __calculate_cost(
                    node_to_calculate=lower_right_neighbor,
                    costmap=costmap,
                    is_diagonal=True,
                ),
            )
        )

    return neighbors


def __check_constrains(node_to_check: Tuple[int, int], costmap: PyCostmap2D) -> bool:
    """
    Checks if constraints of nodes are meeting the constraints for being added as an neighbor

    General approach:
    1. Check if new coordinate is out of costmap
    2. Check if new coordinate is lethal obstacle

    Args:
        node_to_check(tuple(int, int)): The node for which the constraints should be checked
        costmap (nav2_simplecommander.costmap_2d.PyCostmap2D): The costmap in which it should be searched.
    Returns:
         bool: If constraints are fulfilled
    """
    is_index_above_lower_bound: bool = node_to_check[0] > 0 and node_to_check[0] > 0
    is_index_under_upper_bound: bool = (
        node_to_check[0] < costmap.getSizeInCellsX()
        and node_to_check[1] < costmap.getSizeInCellsY()
    )
    is_index_in_arraybounds: bool = (
        costmap.getIndex(node_to_check[0], node_to_check[1]) <= len(costmap.costmap) - 1
    )

    if (
        is_index_above_lower_bound
        and is_index_under_upper_bound
        and is_index_in_arraybounds
    ):
        if costmap.getCostXY(node_to_check[0], node_to_check[1]) < LETHAL_COST:
            return True

    return False


def __calculate_cost(
    node_to_calculate: int | Tuple[int, int],
    costmap: PyCostmap2D,
    is_diagonal: bool = False,
) -> float:
    """
    Calculate the cost of a given node. It respect the step costs an the cost from the costmap which are factors like areas that should be avoided etc.

    Args:
        node_to_calculate(tuple(int, int) or int): The node for which the constraints should be checked in either coordinate or index notation
        costmap (nav2_simplecommander.costmap_2d.PyCostmap2D): The costmap from which the costs should be taken
        is_diagonal (bool) = If the node is a diagonal neighbor
    Returns:
         bool: If constraints are fulfilled
    """
    # length of diagonal = length of one side by the square root of 2 (1.41421)
    diagonal_step_cost: float = costmap.getResolution() * 1.41421
    if type(node_to_calculate) == int:
        if is_diagonal:
            step_cost: float = diagonal_step_cost + float(
                costmap.getCostIdx(node_to_calculate) / 255
            )
        else:
            step_cost: float = costmap.getResolution() + float(
                costmap.getCostIdx(node_to_calculate) / 255
            )
    elif type(node_to_calculate) == Tuple[int, int]:
        if is_diagonal:
            step_cost: float = diagonal_step_cost + float(
                costmap.getCostXY(node_to_calculate[0], node_to_calculate[1]) / 255
            )
        else:
            step_cost: float = costmap.getResolution() + float(
                costmap.getCostXY(node_to_calculate[0], node_to_calculate[1]) / 255
            )
    else:
        step_cost = 0.0

    return step_cost


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
