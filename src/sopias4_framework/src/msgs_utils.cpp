#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "sopias4_framework/msgs_utils.hpp"

namespace sopias4_framework::tools
{
  nav2_msgs::msg::Costmap costmap_2_costmap_msg(nav2_costmap_2d::Costmap2D *costmap)
  {
    nav2_msgs::msg::Costmap costmap_msg = nav2_msgs::msg::Costmap();
    // Metadata
    costmap_msg.metadata.layer = "master";
    costmap_msg.metadata.resolution = costmap->getResolution();
    costmap_msg.metadata.size_x = costmap->getSizeInCellsX();
    costmap_msg.metadata.size_y = costmap->getSizeInCellsY();
    double wx, wy;
    costmap->mapToWorld(0, 0, wy, wy);
    costmap_msg.metadata.origin.position.x = wx - costmap->getResolution() / 2;
    costmap_msg.metadata.origin.position.y = wy - costmap->getResolution() / 2;
    costmap_msg.metadata.origin.position.z = 0;
    costmap_msg.metadata.origin.orientation.w = 1.0;
    // Build data
    costmap_msg.data.resize(costmap_msg.metadata.size_x * costmap_msg.metadata.size_y);
    unsigned char *data = costmap->getCharMap();
    for (unsigned int i = 0; i < costmap_msg.data.size(); i++)
    {
      costmap_msg.data[i] = data[i];
    }

    return costmap_msg;
  }

  void update_costmap_with_msg(nav2_msgs::msg::Costmap *costmap_msg, unsigned char* costmap_array)
  {
    for (int i = 0; i < costmap_msg->data.size(); ++i)
    {
      // Calculate index
      // int x = i / costmap_msg->metadata.size_x;
      // int y = i - (y * costmap_msg->metadata.size_x);
      // Set cost
      costmap_array[i] = costmap_msg ->data[i];
    }
  }
}