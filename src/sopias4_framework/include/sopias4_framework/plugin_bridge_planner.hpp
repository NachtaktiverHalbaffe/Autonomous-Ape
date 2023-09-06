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
#include <cmath>
#include <string>
#include <memory>
#include <iomanip>
#include <iostream>
#include <ctime>
#include <sstream>
#include "nav2_util/node_utils.hpp"
#include "sopias4_framework/plugin_bridge_planner.hpp"
#include "sopias4_framework/msgs_utils.hpp"
#include "sopias4_msgs/srv/create_plan.hpp"
#include "rclcpp/rclcpp.hpp"
#include <jsoncpp/json/value.h>
#include <jsoncpp/json/json.h>
#include <jsoncpp/json/writer.h>
#include <fstream>
#include <string>
#include "yaml-cpp/yaml.h"
namespace plugin_bridges
{
    /**
     * @class PlannerBridge
     * @brief A plugin bridge that allows to write Nav2 planner plugins in another programming language than C++
     *
     * This class shouldn't need any modification if you plan to use the plugin bridge. However, if you plan to write your own
     * C++ plugin without this bridge, then this documentation and code can be taken as a refernce (additionally the official documentation
     * at https://navigation.ros.org/plugin_tutorials/docs/writing_new_nav2planner_plugin.html is good).
     *
     * If you plan to use this Plugin bridge, then you have to do the following steps:
     *      1. Configure your Navigation 2 configuration to use this bridge as an planner:
     *            \code{.yaml}
     *             planner_server:
                        ros__parameters:
                            plugins: ["GridBased"]
                            GridBased:
                                plugin: "plugin_bridges/PlannerBridge"
                                plugin_name: "Astar"
                                save_costmaps: True
                                save_path: "/home/ws/"
     *             \endcode
     *      2. Implement a ROS2 service which has to fulfill following things:
     *          - The service name must be /<namespace>/<plugin_name>/create_plan (Reminder: ROS applies namespace automatically to service name if node is launched in corresponding namespace)
     *          - The service must return a nav_msgs/Path instance where the planned path is given back as an response. Look at service definition sopias4_msgs/drv/CreatePlan.srv
     *          - If using Python: The PlannerPyPlugin class from this framework does this already under the hood, so you can inherit from this class
     *          - A service can be implemented in any programming language, as long as following conditions are met:
     *              - There must be a ROS2 client library available, installed and running
     *              - The ROS2 client library must be able to do: Running a node, running a service, generate service classes from service definitons
     *              - https://docs.ros.org/en/rolling/Concepts/Basic/About-Client-Libraries.html lists a set of known client libraries and languages
     *          - Make sure that the node which serves your service is running
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
         * @brief Method to create the plan from a starting and ending goal.
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

        /**
         * @brief Records the request which is sent to the Plugin implementation to a JSON file
         * 
         * This could be useful for recording data requests and reusing them later in a offline test environment
         * 
         * @param request The request which should be recorded
         * @param save_path Full path to which the JSON file should be saved
         * @param costmap The global costmap which should be saved as a pgm
        */
        void save_request(sopias4_msgs::srv::CreatePlan::Request::SharedPtr request, std::string save_path, nav2_costmap_2d::Costmap2D *costmap);

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
        std::string name_;
        /**
         * Name of the plugin bridge. Needed to configure the service name of the bridge correctly.
         * Should match with the bridge implementation
         */
        std::string plugin_name_;
        /**
         * If the global costmaps which are sent to the planner should be saved. It is useful to gather
         * costmap which then can be used to as mocking data
        */
       bool save_costmaps_;
       /**
        * The path to where the costmap should be saved
       */
       std::string save_path_;
        /**
         * Service client which bridges the create_plan() method to a bridge implementation
         */
        rclcpp::Client<sopias4_msgs::srv::CreatePlan>::SharedPtr client_;
        /**
         * A pointer to the underlying ROS2 node which runs the service client
         */
        rclcpp::Node::SharedPtr service_node_;

    };

} // namespace plugin_bridge

#endif // PLUGIN_BRIDGE_PLANNER__STRAIGHT_LINE_PLANNER_HPP_