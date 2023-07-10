#ifndef PLUGIN_BRIDGE_LAYER_HPP_
#define PLUGIN_BRIDGE_LAYER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "sopias4_msgs/srv/update_costs.hpp"

namespace plugin_bridges
{
    class LayerBridge : public nav2_costmap_2d::CostmapLayer
    {
    public:
        LayerBridge();

        virtual void onInitialize();
        virtual void updateBounds(
            double robot_x, double robot_y, double robot_yaw, double *min_x,
            double *min_y,
            double *max_x,
            double *max_y);
        virtual void updateCosts(
            nav2_costmap_2d::Costmap2D &master_grid,
            int min_i, int min_j, int max_i, int max_j);

        virtual void reset()
        {
            return;
        }

        virtual void onFootprintChanged();

        virtual bool isClearable() { return false; }

    private:
        double last_min_x_, last_min_y_, last_max_x_, last_max_y_;

        // Indicates that the entire gradient should be recalculated next time.
        bool need_recalculation_;

        std::string plugin_name_;

        rclcpp::Client<sopias4_msgs::srv::UpdateCosts>::SharedPtr client_;
    };

} // namespace plugin_bridge

#endif // PLUGIN_BRIDGE_LAYER_HPP_