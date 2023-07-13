#include "sopias4_framework/plugin_bridge_layer.hpp"
#include "sopias4_framework/msgs_utils.hpp"
#include "sopias4_msgs/srv/update_costs.hpp"

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"
#include "nav2_util/node_utils.hpp"

using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace plugin_bridges
{

    LayerBridge::LayerBridge() : last_min_x_(-std::numeric_limits<float>::max()),
                                 last_min_y_(-std::numeric_limits<float>::max()),
                                 last_max_x_(std::numeric_limits<float>::max()),
                                 last_max_y_(std::numeric_limits<float>::max())
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
        node->get_parameter(name_ + "plugin_name", plugin_name_);

        client_ = node->create_client<sopias4_msgs::srv::UpdateCosts>(plugin_name_ + "/update_costs");

        need_recalculation_ = false;
        current_ = true;
    }

    // The method is called to ask the plugin: which area of costmap it needs to update.
    // Inside this method window bounds are re-calculated if need_recalculation_ is true
    // and updated independently on its value.
    void
    LayerBridge::updateBounds(
        double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double *min_x,
        double *min_y, double *max_x, double *max_y)
    {
        if (need_recalculation_)
        {
            last_min_x_ = *min_x;
            last_min_y_ = *min_y;
            last_max_x_ = *max_x;
            last_max_y_ = *max_y;
            *min_x = -std::numeric_limits<float>::max();
            *min_y = -std::numeric_limits<float>::max();
            *max_x = std::numeric_limits<float>::max();
            *max_y = std::numeric_limits<float>::max();
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
    // Inside this method the costmap gradient is generated and is writing directly
    // to the resulting costmap master_grid without any merging with previous layers.
    void
    LayerBridge::updateCosts(
        nav2_costmap_2d::Costmap2D &master_grid, int min_i, int min_j,
        int max_i,
        int max_j)
    {
        if (!enabled_)
        {
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
        unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();

        // {min_i, min_j} - {max_i, max_j} - are update-window coordinates.
        // These variables are used to update the costmap only within this window
        // avoiding the updates of whole area.
        //
        // Fixing window coordinates with map size if necessary.
        min_i = std::max(0, min_i);
        min_j = std::max(0, min_j);
        max_i = std::min(static_cast<int>(size_x), max_i);
        max_j = std::min(static_cast<int>(size_y), max_j);

        // Create the service message
        auto request = std::make_shared<sopias4_msgs::srv::UpdateCosts::Request>();
        request->min_i = min_i;
        request->min_j = min_j;
        request->max_i = max_i;
        request->max_j = max_j;
        request->current_costmap = sopias4_framework::tools::costmap_2_costmap_msg(&master_grid);

        // Send request and receive response
        auto future = client_->async_send_request(request);
        auto node = node_.lock();
        auto return_code = rclcpp::spin_until_future_complete(node, future);

        if (return_code == rclcpp::FutureReturnCode::SUCCESS)
        {
            sopias4_framework::tools::update_costmap_with_msg(&future.get()->updated_costmap, costmap_);
            updateWithMax(master_grid, min_i, min_j, max_i, max_j);
        }
    }

} // namespace plugin_bridges

// This is the macro allowing a plugin_bridges::LayerBridge class
// to be registered in order to be dynamically loadable of base type nav2_costmap_2d::Layer.
// Usually places in the end of cpp-file where the loadable class written.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(plugin_bridges::LayerBridge, nav2_costmap_2d::CostmapLayer)