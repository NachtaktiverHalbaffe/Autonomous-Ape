#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "sopias4_framework/msgs_utils.hpp"
#include <iostream>
#include <cmath>

namespace sopias4_framework::tools
{
  nav_msgs::msg::OccupancyGrid costmap_2_costmap_msg(nav2_costmap_2d::Costmap2D *costmap)
  {
    nav_msgs::msg::OccupancyGrid costmap_msg = nav_msgs::msg::OccupancyGrid();
    // Map metadata
    costmap_msg.info.resolution = costmap->getResolution();
    costmap_msg.info.width = costmap->getSizeInCellsX();
    costmap_msg.info.height = costmap->getSizeInCellsY();
    costmap_msg.info.origin.position.x = costmap->getOriginX();
    costmap_msg.info.origin.position.y = costmap->getOriginY();
    costmap_msg.info.origin.position.z = 0;
    costmap_msg.info.origin.orientation.w = 1.0;

    // Build data
    costmap_msg.data.resize(costmap_msg.info.width * costmap_msg.info.height);
    unsigned char *data = costmap->getCharMap();
    for (unsigned int i = 0; i < costmap_msg.data.size(); i++)
    {
      costmap_msg.data[i] = data[i];
    }

    return costmap_msg;
  }

  nav_msgs::msg::OccupancyGrid costmap_2_costmap_msg(nav2_costmap_2d::Costmap2D *costmap, std::string frame_id)
  {

    nav_msgs::msg::OccupancyGrid costmap_msg = sopias4_framework::tools::costmap_2_costmap_msg(costmap);
    costmap_msg.header.frame_id = frame_id;

    return costmap_msg;
  }

  void set_costmap_cost(nav_msgs::msg::OccupancyGrid *costmap_msg, unsigned char *costmap_array, unsigned int *x, unsigned *y)
  {
    int index = static_cast<int>(nav2_costmap_2d::Costmap2D(*costmap_msg).getIndex(*x, *y));
    unsigned char old_cost = costmap_array[index];
    // Cost from message is between 0 and 100 => scale to range 0 to 254
    unsigned char new_cost = static_cast<unsigned char>(costmap_msg->data[index]) * 254 / 100;
    if (new_cost == 255)
    {
      // 255 is undefined, so we want to avoid it all all costs
      new_cost = 254;
    }
    costmap_array[index] = std::max(old_cost, new_cost);
  }

  void update_costmap_with_msg(nav_msgs::msg::OccupancyGrid *costmap_msg, unsigned char *costmap_array)
  {
    for (long unsigned int i = 0; i < costmap_msg->data.size(); ++i)
    {
      // Calculate index
      // int x = i / costmap_msg->info.width;
      // int y = i - (y * costmap_msg->info.width);
      // Set cost

      unsigned char old_cost = costmap_array[i];
      unsigned char new_cost = static_cast<unsigned char>(costmap_msg->data[i]) * 254 / 100;
      costmap_array[i] = std::max(old_cost, new_cost);
    }
  }

  void update_costmap_with_msg_within_bounds(nav_msgs::msg::OccupancyGrid *costmap_msg, unsigned char *costmap_array, int min_i, int min_j, int max_i, int max_j)
  {
    for (int j = min_j; j < max_j; j++)
    {
      for (int i = min_i; i < max_i; i++)
      {

        int index = static_cast<int>(nav2_costmap_2d::Costmap2D(*costmap_msg).getIndex(i, j));
        unsigned char old_cost = costmap_array[index];
        // Cost from message is between 0 and 100 => scale to range 0 to 254
        unsigned char new_cost = static_cast<unsigned char>(costmap_msg->data[index]) * 254 / 100;
        if (new_cost == 255)
        {
          // 255 is undefined, so we want to avoid it all all costs
          new_cost = 254;
        }
        costmap_array[index] = std::max(old_cost, new_cost);
      }
    }
  }
}
