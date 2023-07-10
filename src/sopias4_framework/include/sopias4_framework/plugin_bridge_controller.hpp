#ifndef PLUGIN_BRIDGE_CONTROLLER__PLUGIN_BRIDGE_CONTROLLER_HPP_
#define PLUGIN_BRIDGE_CONTROLLER__PLUGIN_BRIDGE_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <memory>

#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "sopias4_msgs/srv/compute_velocity_commands.hpp"

namespace plugin_bridges
{

    class ControllerBridge : public nav2_core::Controller
    {
    public:
        ControllerBridge() = default;
        ~ControllerBridge() override = default;

        void configure(
            const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
            std::string name, const std::shared_ptr<tf2_ros::Buffer> &tf,
            const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> &costmap_ros);

        void cleanup() override;
        void activate() override;
        void deactivate() override;

        geometry_msgs::msg::TwistStamped computeVelocityCommands(
            const geometry_msgs::msg::PoseStamped &pose,
            const geometry_msgs::msg::Twist &velocity) override;

        void setPlan(const nav_msgs::msg::Path &path) override;

    protected:
        nav_msgs::msg::Path transformGlobalPlan(const geometry_msgs::msg::PoseStamped &pose);

        bool transformPose(
            const std::shared_ptr<tf2_ros::Buffer> tf,
            const std::string frame,
            const geometry_msgs::msg::PoseStamped &in_pose,
            geometry_msgs::msg::PoseStamped &out_pose,
            const rclcpp::Duration &transform_tolerance) const;

        rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
        std::shared_ptr<tf2_ros::Buffer> tf_;
        std::string name_;
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
        rclcpp::Logger logger_{rclcpp::get_logger("ControllerBridge")};
        rclcpp::Clock::SharedPtr clock_;

        double desired_linear_vel_;
        double lookahead_dist_;
        double max_angular_vel_;
        std::string plugin_name_;
        rclcpp::Duration transform_tolerance_{0, 0};

        nav_msgs::msg::Path global_plan_;
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> global_pub_;

        rclcpp::Client<sopias4_msgs::srv::ComputeVelocityCommands>::SharedPtr client_compute_vel_;
    };

} // namespace plugin_bridges

#endif // PLUGIN_BRIDGE_CONTROLLER__PLUGIN_BRIDGE_CONTROLLER_HPP_