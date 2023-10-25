#ifndef MULTI_ROBOT_COORDINATOR__MULTI_ROBOT_COORDINATOR_HPP_
#define MULTI_ROBOT_COORDINATOR__MULTI_ROBOT_COORDINATOR_HPP_

#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sopias4_msgs/srv/get_namespaces.hpp"
#include "sopias4_msgs/srv/get_robots.hpp"
#include "sopias4_msgs/srv/get_robot_identity.hpp"
#include "sopias4_msgs/srv/registry_service.hpp"
#include "sopias4_msgs/msg/robot.hpp"
#include "sopias4_msgs/msg/robot_states.hpp"

namespace sopias4_fleetbroker
{
    /**
     * @class Costmap2D
     * @brief A 2D costmap provides a mapping between points in the world and their associated "costs".
     */
    class MultiRobotCoordinator : public rclcpp::Node
    {

    public:
        explicit MultiRobotCoordinator(const std::string &nodeName);

    private:
        /**
         * All registered Namespaces
        */
        std::vector<std::string> registered_namespaces = {};
        /**
         * The states of all registered robots
        */
        std::vector<sopias4_msgs::msg::Robot> robot_states = {};
        /**
         * Pointer to all slam posesubscription so they dont get disallocated and can be referenced. Each pointer is mapped to its namespace as a key
         */
        std::map<std::string, rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr> pose_slam_subscribers_;
        /**
         * Pointer to amcl pose slam subscription so they dont get disallocated and can be referenced. Each pointer is mapped to its namespace as a key
         */
        std::map<std::string, rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr> pose_amcl_subscribers_;
        /**
         * Pointer to all global plan subscription so they dont get disallocated and can be referenced. Each pointer is mapped to its namespace as a key
         */
        std::map<std::string, rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr> plan_subscribers_;
        /**
         * Pointer to all navigation state subscription so they dont get disallocated and can be referenced. Each pointer is mapped to its namespace as a key
         */
        std::map<std::string, rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr> nav_state_subscribers_;
        /**
         * Pointer to all veloctiy subscription so they dont get disallocated and can be referenced. Each pointer is mapped to its namespace as a key
         */
        std::map<std::string, rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr> vel_subscribers_manual;
        std::map<std::string, rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr> vel_subscribers_nav;
        std::map<std::string, rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr> vel_subscribers_teleop;
        /**
         * The ROS2 logger of this node
        */
        rclcpp::Logger logger = this->get_logger();
        /**
         * A ROS2 service to get all registered namespace.
        */
        rclcpp::Service<sopias4_msgs::srv::GetNamespaces>::SharedPtr get_namespaces_service;
        /**
         * A ROS2 service to get the virtual representation (state) of a specific robot
        */
        rclcpp::Service<sopias4_msgs::srv::GetRobotIdentity>::SharedPtr get_robot_identity_service;
        /**
         * A ROS2 service to get all virtual representations (state) of all registered robots
        */
        rclcpp::Service<sopias4_msgs::srv::GetRobots>::SharedPtr get_robots_service;
        /**
         * A ROS2 service to register a namespace.  Also a state for this robot should be created inside this service
        */
        rclcpp::Service<sopias4_msgs::srv::RegistryService>::SharedPtr register_service;
        /**
         * A ROS2 service to unregister a registered namespace. Also the state for the corresponding robot should be deleted in this
        */
        rclcpp::Service<sopias4_msgs::srv::RegistryService>::SharedPtr unregister_service;
        /**
         * A ROS2 publisher which publishes all the states of the robots when one of the states changed
        */
        rclcpp::Publisher<sopias4_msgs::msg::RobotStates>::SharedPtr publisher_robot_states_;

        /**
         * @brief Callback function which gets executed when the /register_namespace service gets called
         *
         * Additionally to registering the namespace also a state for the registered robot is created and the multi robot coordinator subscribes to the relevant topics
         * to update the state properly
         *
         * @param request Data from the service request which includes the namespace which should be registered
         * @param response Response into which the response data is written which includes the statuscode
         */
        void register_callback(const sopias4_msgs::srv::RegistryService::Request::SharedPtr request, sopias4_msgs::srv::RegistryService::Response::SharedPtr response);

        /**
         * @brief Callback function which gets executed when the /get_namespace service gets called. 
         *
         * It returns all registered namespaces
         *
         * @param response Response into which the response data is written which includes a list of all registered namespaces
         */
        void get_namespace_callback(const sopias4_msgs::srv::GetNamespaces::Request::SharedPtr, sopias4_msgs::srv::GetNamespaces::Response::SharedPtr response);

        /**
         * @brief Callback function which gets executed when the /get_robot_identity service gets called
         *
         * It returns the state of the selected robot
         *
         * @param request Data from the service request which includes the namespace of the robot
         * @param response Response into which the response data is written which includes a list of RobotStates
         */
        void get_robot_identity_callback(const sopias4_msgs::srv::GetRobotIdentity::Request::SharedPtr request, sopias4_msgs::srv::GetRobotIdentity::Response::SharedPtr response);

        /**
         * @brief Callback function which gets executed when the /get_robots service gets called
         *
         * It returns a list of all states of all registered robots. These information can also be gained by the /robot_states topic where 
         * this information is also provided 
         *
         * @param response Response into which the response data is written which includes a list of all robot states
         */
        void get_robots_callback(const sopias4_msgs::srv::GetRobots::Request::SharedPtr, sopias4_msgs::srv::GetRobots::Response::SharedPtr response);

        /**
         * @brief Callback function which gets executed when the /unregister_namespace service gets called
         *
         * Additionally to unregistering the namespace also a state for the registered robot is deleted and the multi robot coordinator unsubscribes 
         * from all the relevant topics 
         *
         * @param request Data from the service request which includes the namespace which should be unregistered
         * @param response Response into which the response data is written which includes the statuscode of the operation
         */
        void unregister_callback(const sopias4_msgs::srv::RegistryService::Request::SharedPtr request, sopias4_msgs::srv::RegistryService::Response::SharedPtr response);

        /**
         * @brief Subscriber callback function to which the MultiRobotCoordinator subscribes when a namespace is registered to track the global plan of the robot
         * 
         * @param path The global path
         * @param name_space The namespace of the robot to which this path belongs
        */
        void path_sub_callback(const nav_msgs::msg::Path::SharedPtr path, std::string name_space);

        /**
         * @brief Subscriber callback function to which the MultiRobotCoordinator subscribes when a namespace is registered to track the pose of the robot.
         * Can be used both for slam and amcl callbacks
         * 
         * @param pose The position of the robot
         * @param name_space The namespace of the robot to which this positions belongs
         * 
        */
        void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose, const std::string name_space);

        /**
         * @brief Subscriber callback function to which the MultiRobotCoordinator subscribes when a namespace is registered to track the velocity of the robot.
         * 
         * @param vel The velocity from the robot
         * @param name_space The namespace of the robot to which this positions belongs
         * 
        */
        void velocity_callback(const geometry_msgs::msg::Twist::SharedPtr vel, const std::string name_space);

        /**
         * @brief Subscriber callback function to which the MultiRobotCoordinator subscribes when a namespace is registered to track the navigation state of the robot.
         *
         * @param is_navigating The navigation state of the robot
         * @param name_space The namespace of the robot to which the navigations state belongs
         */
        void nav_state_sub_callback(const std_msgs::msg::Bool::SharedPtr is_navigating, const std::string name_space);

        /**
         * @brief Publishes the states of all registered robots to the /robot_states topic
        */
        void publish_robot_states();
    };
} // namespace sopias4_fleetbroker

#endif // MULTI_ROBOT_COORDINATOR__MULTI_ROBOT_COORDINATOR_HPP_