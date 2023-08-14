#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "sopias4_msgs/srv/get_namespaces.hpp"
#include "sopias4_msgs/srv/get_robots.hpp"
#include "sopias4_msgs/srv/get_robot_identity.hpp"
#include "sopias4_msgs/srv/registry_service.hpp"
#include "sopias4_msgs/srv/set_robot_path.hpp"
#include "sopias4_msgs/msg/robot.hpp"


namespace sopias4_map_server {
    /**
     * @class Costmap2D
     * @brief A 2D costmap provides a mapping between points in the world and their associated "costs".
     */
    class MultiRobotCoordinator : public rclcpp:: Node {

    public:
        explicit MultiRobotCoordinator(const std::string & nodeName) ;

    private:
        // All registered namespaces
        std::vector<std::string> registered_namespaces = {};
        std::vector<sopias4_framework::msg::Robot> robot_states = {};

        rclcpp::Service<sopias4_framework::srv::GetNamespaces>::SharedPtr get_namespaces_service;
        rclcpp::Service<sopias4_framework::srv::GetRobotIdentity>::SharedPtr get_robot_identity_service;
        rclcpp::Service<sopias4_framework::srv::GetRobots>::SharedPtr get_robots_service;
        rclcpp::Service<sopias4_framework::srv::Register>::SharedPtr register_service;

        /**
         * @brief Callback function which gets executed when the get_namespace  service gets called
            @param request Data from the service request
            @param response Response into which the response data is written
        */
        void get_namespace_callback(const sopias4_framework::srv::GetNamespaces::Request::SharedPtr request, sopias4_framework::srv::GetNamespaces::Response::SharedPtr response );

        void get_robot_identity_callback(const sopias4_framework::srv::GetRobotIdentity::Request::SharedPtr request, sopias4_framework::srv::GetRobotIdentity::Response::SharedPtr response);

        void get_robots_callback(const sopias4_framework::srv::GetRobots::Request::SharedPtr request, sopias4_framework::srv::GetRobots::Response::SharedPtr response);

        void register_callback(const sopias4_framework::srv::Register::Request::SharedPtr request, sopias4_framework::srv::Register::Response::SharedPtr response);
    };
}