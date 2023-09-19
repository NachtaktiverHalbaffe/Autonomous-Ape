#include "sopias4_framework/plugin_bridge_layer.hpp"
#include "sopias4_framework/msgs_utils.hpp"
#include "sopias4_msgs/srv/update_costs.hpp"

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"
#include "nav2_util/node_utils.hpp"
#include <iostream>

using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace plugin_bridges
{

    LayerBridge::LayerBridge() : last_min_x_(-std::numeric_limits<double>::min()),
                                 last_min_y_(-std::numeric_limits<double>::min()),
                                 last_max_x_(std::numeric_limits<double>::max()),
                                 last_max_y_(std::numeric_limits<double>::max())
    {
    }

    // This method is called at the end of plugin initialization.
    // It contains ROS parameter(s) declaration, service clients setup and initialization
    // of need_recalculation_ variable.
    void
    LayerBridge::onInitialize()
    {
        auto node = node_.lock();
        declareParameter("enabled", rclcpp::ParameterValue(true));
        node->get_parameter(name_ + "." + "enabled", enabled_);
        declareParameter("plugin_name", rclcpp::ParameterValue("local_layer"));
        node->get_parameter(name_ + ".plugin_name", plugin_name_);

        // Service client node
        service_node_ = std::make_shared<rclcpp::Node>("_planner_bridge_service_node_" + plugin_name_);

        client_ = service_node_->create_client<sopias4_msgs::srv::UpdateCosts>(plugin_name_ + "/update_costs");

        need_recalculation_ = false;
        current_ = true;
    }

    // The method is called to ask the plugin: which area of costmap it needs to update.
    // Inside this method window bounds are re-calculated if need_recalculation_ is true
    // and updated independently on its value.
    void
    LayerBridge::updateBounds(
        double robot_x, double robot_y, double /*robot_yaw*/, double *min_x,
        double *min_y, double *max_x, double *max_y)
    {
        std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
        if (layered_costmap_->isRolling())
        {
            updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
        }

        if (need_recalculation_)
        {
            last_min_x_ = *min_x;
            last_min_y_ = *min_y;
            last_max_x_ = *max_x;
            last_max_y_ = *max_y;
            *min_x = -std::numeric_limits<double>::max();
            *min_y = -std::numeric_limits<double>::max();
            *max_x = std::numeric_limits<double>::max();
            *max_y = std::numeric_limits<double>::max();
            need_recalculation_ = false;
        }
        else
        {
            double tmp_min_x = last_min_x_;
            double tmp_min_y = last_min_y_;
            double tmp_max_x = last_max_x_;
            double tmp_max_y = last_max_y_;
            last_min_x_ = *min_x;
            last_min_y_ = *min_y;
            last_max_x_ = *max_x;
            last_max_y_ = *max_y;
            *min_x = std::min(tmp_min_x, *min_x);
            *min_y = std::min(tmp_min_y, *min_y);
            *max_x = std::max(tmp_max_x, *max_x);
            *max_y = std::max(tmp_max_y, *max_y);
        }
    }

    // The method is called when footprint was changed.
    // Here it just resets need_recalculation_ variable.
    void
    LayerBridge::onFootprintChanged()
    {
        need_recalculation_ = true;

        RCLCPP_DEBUG(rclcpp::get_logger(
                         "nav2_costmap_2d"),
                     "GradientLayer::onFootprintChanged(): num footprint points: %lu",
                     layered_costmap_->getFootprint().size());
    }

    // The method is called when costmap recalculation is required.
    // It updates the costmap within its window bounds.
    void
    LayerBridge::updateCosts(
        nav2_costmap_2d::Costmap2D &master_grid, int min_i, int min_j,
        int max_i,
        int max_j)
    {
        std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
        if (!enabled_)
        {
            return;
        }
        if (!client_->service_is_ready())
        {
            RCLCPP_WARN(service_node_->get_logger(), "Service isn't online, cant update %s layer", plugin_name_.c_str());
            return;
        }

        // master_grid - is a resulting costmap combined from all layers.
        // By using this pointer all layers will be overwritten!
        // To work with costmap layer and merge it with other costmap layers,
        // please use costmap_ pointer instead (this is pointer to current
        // costmap layer grid) and then call one of updates methods:
        // - updateWithAddition()
        // - updateWithMax()
        // - updateWithOverwrite()
        // - updateWithTrueOverwrite()
        // In this case using master_array pointer is equal to modifying local costmap_
        // pointer and then calling updateWithTrueOverwrite():

        // Create the service message
        auto request = std::make_shared<sopias4_msgs::srv::UpdateCosts::Request>();
        request->min_i = min_i;
        request->min_j = min_j;
        request->max_i = max_i;
        request->max_j = max_j;
        request->current_costmap = sopias4_framework::tools::costmap_2_costmap_msg(&master_grid, layered_costmap_->getGlobalFrameID());

        // Send request and receive response
        auto future = client_->async_send_request(request);
        auto return_code = rclcpp::spin_until_future_complete(service_node_, future);

        if (return_code == rclcpp::FutureReturnCode::SUCCESS)
        {
            nav_msgs::msg::OccupancyGrid received_map = future.get()->updated_costmap;
            sopias4_framework::tools::update_costmap_with_msg_within_bounds(&received_map, master_grid.getCharMap(), min_i, min_j, max_i, max_j);
        }

        current_ = true;
    }
    void LayerBridge::reset()
    {
        resetMaps();
        current_ = false;
        need_recalculation_ = true;
    }

} // namespace plugin_bridges

// This is the macro allowing a plugin_bridges::LayerBridge class
// to be registered in order to be dynamically loadable of base type nav2_costmap_2d::Layer.
// Usually places in the end of cpp-file where the loadable class written.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(plugin_bridges::LayerBridge, nav2_costmap_2d::Layer)