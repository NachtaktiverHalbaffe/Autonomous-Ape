#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "sopias4_framework/msgs_utils.hpp"

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
    costmap_msg.info.origin.position.y = costmap -> getOriginY();
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

  void update_costmap_with_msg(nav_msgs::msg::OccupancyGrid *costmap_msg, unsigned char* costmap_array)
  {
    for (int i = 0; i < costmap_msg->data.size(); ++i)
    {
      // Calculate index
      // int x = i / costmap_msg->info.width;
      // int y = i - (y * costmap_msg->info.width);
      // Set cost
      costmap_array[i] = costmap_msg ->data[i];
    }
  }
}