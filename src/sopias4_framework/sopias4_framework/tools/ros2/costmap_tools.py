"""
This contains multiple helpful functions to interact with a PyCostmap2D which is used in the Python implementations
of the Navigation2 plugins. Most of them a wrapper around the own functions of PyCostmap2 and only apply more appropriate
types to work with. Also the built-in functions of PyCostmap2D can often be helpful too.
"""

from typing import Tuple

import numpy as np
from geometry_msgs.msg import Pose
from nav2_simple_commander.costmap_2d import PyCostmap2D
from nav_msgs.msg import OccupancyGrid


def map_2_pose(x: int, y: int, costmap: PyCostmap2D) -> Pose:
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


def pose_2_map(pose: Pose, costmap: PyCostmap2D) -> Tuple[int, int]:
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
    x, y = costmap.worldToMap(pose.position.x, pose.position.y)

    return x, y


def costmap_2_2darray(costmap: PyCostmap2D) -> np.ndarray:
    """
    Reshapes/creates a 2D array from the costmap which is a 1D array

    Args:
        costmap (PyCostmap2D):  The costmap from which the 2D array should be generated

    Returns:
        np.ndarray: A 2-dimensional array of the costmap data
    """
    array2D = np.array(costmap.costmap, dtype=np.int8).reshape(
        costmap.size_y, costmap.size_x
    )

    return array2D


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
