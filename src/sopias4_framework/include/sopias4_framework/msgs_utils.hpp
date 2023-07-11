#ifndef MSGS_UTILS_HPP_
#define MSGS_UTILS_HPP_

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace sopias4_framework::tools
{
    /**
     * @brief Converts a nav2 costmap to a nav2 costmap message
     * @param costmap A pointer to the costmap which should be converted to an message
     * @returns A nav2_msgs/CostMap message
     */
    nav_msgs::msg::OccupancyGrid costmap_2_costmap_msg(nav2_costmap_2d::Costmap2D *costmap);

    /**
     * @brief Updates a costmap with the new cost values from a costmap message
     * @param costmap_msg A pointer to the costmap message from which the updated values will be taken
     * @param costmap_array A pointer to the costmap array which costs should be updated
     * @returns
     */
    void update_costmap_with_msg(nav_msgs::msg::OccupancyGrid *costmap_msg, unsigned char* costmap_array);
}
#endif // PLUGIN_BRIDGE_PLANNER__STRAIGHT_LINE_PLANNER_HPP_