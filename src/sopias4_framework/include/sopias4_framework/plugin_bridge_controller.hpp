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
    /**
     * @class ControllerBridge
     * @brief A plugin bridge that allows to write Nav2 controller plugins in another programming language than C++
     * 
     * This class shouldn't need any modification if you plan to use the plugin bridge. However, if you plan to write your own
     * C++ plugin without this bridge, then this documentation and code can be taken as a refernce (additionally the official documentation
     * at https://navigation.ros.org/plugin_tutorials/docs/writing_new_costmap2d_plugin.html is good).
     *
     * If you plan to use this Plugin bridge, then you have to do the following steps:
     *      1. Configure your Navigation 2 configuration to use this bridge as an costmap layer:        
     *            \code{.yaml}
                        controller_server:
                        ros__parameters:
                            controller_plugins: ["FollowPath"]
                            FollowPath:
                                plugin:  "plugin_bridges/ControllerBridge"
                                plugin_name: "Example"
     *             \endcode
     *      2. Implement a ROS2 service which has to fulfill following things:
     *          - The service name must be /<namespace>/<plugin_name>/compute_velocity_commands (Reminder: ROS applies namespace automatically to service name if node is launched in corresponding namespace)
     *          - The service must return a geometry_msgs/TwistStamped instance where the planned path is given back as an response. Look at service definition sopias4_msgs/drv/ComputeVelocityCommands.srv
     *          - If using Python: The ControllerPyPlugin class from this framework does this already under the hood, so you can inherit from this class
     *          - A service can be implemented in any programming language, as long as following conditions are met:
     *              - There must be a ROS2 client library available, installed and running
     *              - The ROS2 client library must be able to do: Running a node, running a service, generate service classes from service definitons
     *              - https://docs.ros.org/en/rolling/Concepts/Basic/About-Client-Libraries.html lists a set of known client libraries and languages
     *          - Make sure that the node which serves your service is running
     */
    class ControllerBridge : public nav2_core::Controller
    {
    public:
        /**
         * @brief Controller Bridge constructor
         */
        ControllerBridge() = default;
        /**
         * @brief Controller Bridge destructor
        */
        ~ControllerBridge() override = default;

        /**
         * @param  parent pointer to user's node
         * @param  costmap_ros A pointer to the costmap
         */
        void configure(
            const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
            std::string name, const std::shared_ptr<tf2_ros::Buffer> &tf,
            const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> &costmap_ros);

        /**
         * @brief Method to cleanup resources.
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
         * @brief Controller computeVelocityCommands - calculates the best command given the current pose and velocity.
         *
         * Here the service client is implemented which has must have the name /<namespace>/<plugin_name>/compute_velocity_commands.
         * This service client calls the implementation, written in the desired programming language, which must be run as an service server in his own node.
         * It is presumed that the global plan is already set.
         *
         * @param pose Current robot pose
         * @param velocity Current robot velocity
         * @return The best command for the robot to drive
         */
        geometry_msgs::msg::TwistStamped computeVelocityCommands(
            const geometry_msgs::msg::PoseStamped &pose,
            const geometry_msgs::msg::Twist &velocity) override;

        /**
         * @brief local setPlan - Sets the global plan
         * @param path The global plan
         */
        void setPlan(const nav_msgs::msg::Path &path) override;

    protected:
        nav_msgs::msg::Path transformGlobalPlan(const geometry_msgs::msg::PoseStamped &pose);

        bool transformPose(
            const std::shared_ptr<tf2_ros::Buffer> tf,
            const std::string frame,
            const geometry_msgs::msg::PoseStamped &in_pose,
            geometry_msgs::msg::PoseStamped &out_pose,
            const rclcpp::Duration &transform_tolerance) const;

        /**
         * A pointer to the underlying ROS2 node
         */
        rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
        /**
         * A pointer to the transformation tree
         */
        std::shared_ptr<tf2_ros::Buffer> tf_;
        /**
         * Name of the controller
         */
        std::string name_;
        /**
         * A pointer to the local costmap
         */
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
        rclcpp::Logger logger_{rclcpp::get_logger("ControllerBridge")};
        /**
         * A pointer to the clock
         */
        rclcpp::Clock::SharedPtr clock_;

        /**
         * Name of the plugin bridge. Needed to configure the service name of the bridge correctly.
         * Should match with the bridge implementation
         */
        std::string plugin_name_;
        rclcpp::Duration transform_tolerance_{0, 0};

        /**
         * The global path from the planner which the controller tries to follow
         */
        nav_msgs::msg::Path global_plan_;
        /**
         * Publisher which publishes the global plan of the controller
         */
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> global_pub_;

        /**
         * Service client which bridges the compute_velocity_commands() method to a bridge implementation
         */
        rclcpp::Client<sopias4_msgs::srv::ComputeVelocityCommands>::SharedPtr client_compute_vel_;
        /**
         * A pointer to the underlying ROS2 node which runs the service client
         */
        rclcpp::Node::SharedPtr service_node_;
    };

} // namespace plugin_bridges

#endif // PLUGIN_BRIDGE_CONTROLLER__PLUGIN_BRIDGE_CONTROLLER_HPP_