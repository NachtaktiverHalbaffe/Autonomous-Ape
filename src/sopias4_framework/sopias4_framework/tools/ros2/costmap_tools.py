"""
This contains multiple helpful functions to interact with a PyCostmap2D which is used in the Python implementations
of the Navigation2 plugins. Most of them a wrapper around the own functions of PyCostmap2 and only apply more appropriate
types to work with. Also the built-in functions of PyCostmap2D can often be helpful too.
"""

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
        x, y = costmap.worldToMap(pose.pose.position.x, pose.pose.position.y)

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


def find_neighbors(
    node: Tuple[int, int], costmap: PyCostmap2D
) -> list[Tuple[Tuple[int, int], float]]:
    """
    Identifies neighbor nodes inspecting the 8 adjacent neighbors. Checks if neighbor is inside the map boundaries\
    and if is not an obstacle according to a threshold. The costs of a neighbor is the sum of the distance between the node and the \
    found neighbor and the cost of the pixel which the neighbor represents.

    Args:
        node (tuple(int,int)): The node as an x,y-coordinate in the costmap for which the neighbors should be found
        costmap (nav2_simplecommander.costmap_2d.PyCostmap2D): The costmap in which it should be searched.
    Returns:
         list(tuple(tuple(int,int), float)): A list with valid neighbour nodes as a tuple with [x,y-coordinates, step_cost] pairs
    """
    neighbors = []
    # length of diagonal = length of one side by the square root of 2 (1.41421)
    diagonal_step_cost = costmap.getResolution() * 1.41421
    # threshold value used to reject neighbor nodes as they are considered as obstacles [1-254]

    # General approach:
    # 1. Calculate new coordinate
    # 2. Check if new coordinate is out of costmap
    # 3. Check if new coordinate is lethal obstacle
    # 4. If checks are successful, the calculate costs in meters
    upper = (node[0], node[1] + 1)
    if upper[1] <= costmap.getSizeInCellsY():
        if costmap.getCostXY(upper[0], upper[1]) < LETHAL_COST:
            step_cost = (
                costmap.getResolution() + costmap.getCostXY(upper[0], upper[1]) / 255
            )
            neighbors.append((upper, step_cost))

    left = (node[0] - 1, node[1])
    if left[0] >= 0:
        if costmap.getCostXY(left[0], left[1]) < LETHAL_COST:
            step_cost = (
                costmap.getResolution() + costmap.getCostXY(left[0], left[1]) / 255
            )
            neighbors.append((left, step_cost))

    upper_left = (node[0] - 1, node[1] + 1)
    if upper_left[0] >= 0 and upper_left[1] <= costmap.getSizeInCellsY():
        if costmap.getCostXY(upper_left[0], upper_left[1]) < LETHAL_COST:
            step_cost = (
                diagonal_step_cost
                + costmap.getCostXY(upper_left[0], upper_left[1]) / 255
            )
            neighbors.append((upper_left, step_cost))

    upper_right = (node[0] + 1, node[1] + 1)
    if (
        upper_right[0] <= costmap.getSizeInCellsX()
        and upper_right[1] <= costmap.getSizeInCellsY()
    ):
        if costmap.getCostXY(upper_right[0], upper_right[1]) < LETHAL_COST:
            step_cost = (
                diagonal_step_cost
                + costmap.getCostXY(upper_right[0], upper_right[1]) / 255
            )
            neighbors.append((upper_right, step_cost))

    right = (node[0] + 1, node[1])
    if right[0] <= costmap.getSizeInCellsX():
        if costmap.getCostXY(right[0], right[1]) < LETHAL_COST:
            step_cost = (
                costmap.getResolution() + costmap.getCostXY(right[0], right[1]) / 255
            )
            neighbors.append((right, step_cost))

    lower_left = (node[0] - 1, node[1] - 1)
    if lower_left[0] >= 0 and lower_left[1] >= 0:
        if costmap.getCostXY(lower_left[0], lower_left[1]) < LETHAL_COST:
            step_cost = (
                diagonal_step_cost
                + costmap.getCostXY(lower_left[0], lower_left[1]) / 255
            )
            neighbors.append((lower_left, step_cost))

    lower = (node[0], node[1] - 1)
    if lower[1] >= 0:
        if costmap.getCostXY(lower[0], lower[1]) < LETHAL_COST:
            step_cost = (
                costmap.getResolution() + costmap.getCostXY(lower[0], lower[1]) / 255
            )
            neighbors.append((lower, step_cost))

    lower_right = (node[0] + 1, node[1] - 1)
    if lower_right[0] <= costmap.getSizeInCellsX() and lower_right[1] >= 0:
        if costmap.getCostXY(lower_right[0], lower_right[1]) < LETHAL_COST:
            step_cost = (
                diagonal_step_cost
                + costmap.getCostXY(lower_right[0], lower_right[1]) / 255
            )
            neighbors.append((lower_right, step_cost))

    return neighbors


def euclidian_distance_map_domain(
    start: Tuple[int, int], goal: Tuple[int, int], costmap: PyCostmap2D
) -> float:
    """
    Calculates the euclidian distance between two coordinates in the costmap space.  For this purpose,\
    it calculates the distance in the costmap domain (pixels) and them multiplies it with the resolution\
    of the map to get the distance in meters.

    Args:
        start (tuple(int, int)): The start of the distance as an x,y-coordinate in the costmap
        goal (tuple(int, int)): The goal of the distance as an x,y-coordinate in the costmap
        costmap (nav2_simplecommander.costmap_2d.PyCostmap2D): The costmap in which the distance is calculated

    Returns:
        float: The euclidian distance in meters
    """
    return (
        sqrt((start[0] - goal[0]) ** 2 + (start[1] - goal[1]) ** 2)
        * costmap.getResolution()
    )


def euclidian_distance_pixel_domain(
    start: Tuple[int, int], goal: Tuple[int, int]
) -> float:
    """
    Calculates the euclidian distance between two coordinates in the costmap space.  For this purpose,\
    it calculates the distance in the costmap domain (pixels) and them multiplies it with the resolution\
    of the map to get the distance in meters.

    Args:
        start (tuple(int, int)): The start of the distance as an x,y-coordinate in the costmap
        goal (tuple(int, int)): The goal of the distance as an x,y-coordinate in the costmap

    Returns:
        float: The euclidian distance in meters
    """
    return sqrt((start[0] - goal[0]) ** 2 + (start[1] - goal[1]) ** 2)


if __name__ == "__main__":
    print("This is only an library, dont execute this Python script at it's own")
