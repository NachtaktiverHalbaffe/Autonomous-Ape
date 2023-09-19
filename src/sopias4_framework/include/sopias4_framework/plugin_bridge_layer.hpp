#ifndef PLUGIN_BRIDGE_LAYER_HPP_
#define PLUGIN_BRIDGE_LAYER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "sopias4_msgs/srv/update_costs.hpp"

namespace plugin_bridges
{
    /**
     * @class LayerBridge
     * @brief  A plugin bridge that allows to write Nav2 layer plugins in another programming language than C++
     * 
     * This class shouldn't need any modification if you plan to use the plugin bridge. However, if you plan to write your own
     * C++ plugin without this bridge, then this documentation and code can be taken as a refernce (additionally the official documentation
     * at https://navigation.ros.org/plugin_tutorials/docs/writing_new_costmap2d_plugin.html is good).
     *
     * If you plan to use this Plugin bridge, then you have to do the following steps:
     *      1. Configure your Navigation 2 configuration to use this bridge as an costmap layer:           
     *            \code{.yaml}
                        local_costmap:
                            local_costmap:
                                ros__parameters:
                                    plugins: [robot_layer]
                                    robot_layer:
                                        plugin: plugin_bridges/LayerPlugin
                                        plugin_name: "robot_layer"
     *             \endcode
     *      2. Implement a ROS2 service which has to fulfill following things:
     *          - The service name must be /<namespace>/<plugin_name>/update_costs (Reminder: ROS applies namespace automatically to service name if node is launched in corresponding namespace)
     *          - The service must return a nav_msgs/OccupancyGrid instance where the planned path is given back as an response. Look at service definition sopias4_msgs/srv/UpdateCosts.srv
     *          - If using Python: The LayerPyPlugin class from this framework does this already under the hood, so you can inherit from this class
     *          - A service can be implemented in any programming language, as long as following conditions are met:
     *              - There must be a ROS2 client library available, installed and running
     *              - The ROS2 client library must be able to do: Running a node, running a service, generate service classes from service definitons
     *              - https://docs.ros.org/en/rolling/Concepts/Basic/About-Client-Libraries.html lists a set of known client libraries and languages
     *          - Make sure that the node which serves your service is running
     */
    class LayerBridge : public nav2_costmap_2d::CostmapLayer
    {
    public:
        /**
         * @brief LayerBridge constructor
         */
        LayerBridge();

        /**
         * @brief Initialization process of layer on startup
         */
        virtual void onInitialize();

        /**
         * @brief Update the bounds of the master costmap by this layer's update dimensions
         * @param robot_x X pose of robot
         * @param robot_y Y pose of robot
         * @param robot_yaw Robot orientation
         * @param min_x X min map coord of the window to update
         * @param min_y Y min map coord of the window to update
         * @param max_x X max map coord of the window to update
         * @param max_y Y max map coord of the window to update
         */
        virtual void updateBounds(
            double robot_x, double robot_y, double robot_yaw, double *min_x,
            double *min_y,
            double *max_x,
            double *max_y);

        /**
         * @brief Update the costs in the layer and update costmap with all layers
         *
         * Here the service client is implemented which has must have the name /<namespace>/<plugin_name>/update_costs.
         * This service client calls the implementation, written in the desired programming language, which must be run as an service server in his own node.
         *
         * @param master_grid The master costmap grid to update
         * @param min_x X min map coord of the window to update
         * @param min_y Y min map coord of the window to update
         * @param max_x X max map coord of the window to update
         * @param max_y Y max map coord of the window to update
         */
        virtual void updateCosts(
            nav2_costmap_2d::Costmap2D &master_grid,
            int min_i, int min_j, int max_i, int max_j);

        /**
         * @brief Reset this costmap
         */
        virtual void reset();

        /**
         * The method is called when footprint was changed.
         * Here it just resets need_recalculation_ variable.
         */
        virtual void onFootprintChanged();

        /**
         * @brief If clearing operations should be processed on this layer or not
         */
        virtual bool isClearable() { return true; }

    private:
        double last_min_x_, last_min_y_, last_max_x_, last_max_y_, origin_x, origin_y;

        /**
         * Indicates that the entire layer should be recalculated next time.
         */
        bool need_recalculation_;
        /**
         * Name of the plugin bridge. Needed to configure the service name of the bridge correctly.
         * Should match with the bridge implementation
         */
        std::string plugin_name_;
        /**
         * Service client which bridges the pdate_costs() method to a bridge implementation
         */
        rclcpp::Client<sopias4_msgs::srv::UpdateCosts>::SharedPtr client_;
        /**
         * A pointer to the underlying ROS2 node which runs the service client
         */
        rclcpp::Node::SharedPtr service_node_;
    };

} // namespace plugin_bridge

#endif // PLUGIN_BRIDGE_LAYER_HPP_