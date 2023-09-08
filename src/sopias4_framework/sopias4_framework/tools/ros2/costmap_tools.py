"""
This contains multiple helpful functions to interact with a PyCostmap2D which is used in the Python implementations
of the Navigation2 plugins. Most of them a wrapper around the own functions of PyCostmap2 and only apply more appropriate
types to work with. Also the built-in functions of PyCostmap2D can often be helpful too.
"""

import math
from math import sqrt
from typing import Tuple

import numpy as np
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
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
