#ifndef PLUGIN_BRIDGE_PLANNER__PLUGIN_BRIDGE_PLANNER_HPP_
#define PLUGIN_BRIDGE_PLANNER__PLUGIN_BRIDGE_PLANNER_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "sopias4_msgs/srv/create_plan.hpp"

namespace plugin_bridges
{

    /**
     * @class PlannerBridge
     * @brief A plugin bridge that allows to write Nav2 planner plugins in another programming language than C++
     */
    class PlannerBridge : public nav2_core::GlobalPlanner
    {
    public:
        /**
         * @brief Constructor
        */
        PlannerBridge() = default;
        /**
         * @brief Destructor
        */
        ~PlannerBridge() = default;

        /**
         * Method is called at when planner server enters on_configure state. Ideally this methods should perform 
         * declarations of ROS parameters and initialization of planner’s member variables.
         * 
         * @param  parent pointer to user's node
         * @param  name The name of this planner
         * @param  tf A pointer to a TF buffer
         * @param  costmap_ros A pointer to the costmap
         */
        void configure(
            const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
            std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
            std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

        /**
         * @brief Method to cleanup resources used on shutdown.
         */
        void cleanup() override;

        /**
         * @brief Method to active planner and any threads involved in execution.
         */
        void activate() override;

        /**
         * @brief Method to deactive planner and any threads involved in execution.
         */
        void deactivate() override;

        /**
         * @brief Method create the plan from a starting and ending goal.
         * 
         * Here the service client is implemented which has must have the name /<namespace>/<plugin_name>/create_plan.
         * This service client calls the implementation, written in the desired programming language, which must be run as an service server in his own node.
         * It is presumed that the global plan is already set.
         * 
         * @param start The starting pose of the robot
         * @param goal  The goal pose of the robot
         * @return The sequence of poses to get from start to goal, if any
         */
        nav_msgs::msg::Path createPlan(
            const geometry_msgs::msg::PoseStamped &start,
            const geometry_msgs::msg::PoseStamped &goal) override;

    private:
        /**
         * A pointer to the transformation tree buffer
        */
        std::shared_ptr<tf2_ros::Buffer> tf_;

        /**
         * A pointer to the underlying ROS2 node
        */
        nav2_util::LifecycleNode::SharedPtr node_;

        /**
         * Global Costmap
        */
        nav2_costmap_2d::Costmap2D *costmap_;

        /**
         * The global frame of the costmap
        */
        std::string global_frame_;
        /**
         * The internal name of the plugin
        */
        std::string  name_;
        /**
         * Name of the plugin bridge. Needed to configure the service name of the bridge correctly.
         * Should match with the bridge implementation
         */
        std::string plugin_name_; 
        /**
         * Service client which bridges the create_plan() method to a bridge implementation
         */
        rclcpp::Client<sopias4_msgs::srv::CreatePlan>::SharedPtr client_;
    };

} // namespace plugin_bridge

#endif // PLUGIN_BRIDGE_PLANNER__STRAIGHT_LINE_PLANNER_HPP_