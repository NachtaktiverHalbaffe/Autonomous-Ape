#include <memory>
#include <vector>
#include <algorithm>
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rmw/validate_namespace.h"
#include "sopias4_framework/srv/get_namespaces.hpp"
#include "sopias4_framework/srv/get_robots.hpp"
#include "sopias4_framework/srv/get_robot_identity.hpp"
#include "sopias4_framework/srv/register.hpp"
#include "sopias4_framework/srv/unregister.hpp"
#include "sopias4_framework/srv/set_robot_path.hpp"
#include "sopias4_framework/msg/robot.hpp"



class MultiRobotCoordinator : public rclcpp:: Node {
public:
    explicit MultiRobotCoordinator(const std::string & nodeName) : Node(nodeName){
            // TODO Add service for unregistering, adding path
            get_namespaces_service = this -> create_service<sopias4_framework::srv::GetNamespaces>("get_namespaces", std::bind(&MultiRobotCoordinator::get_namespace_callback, this, std::placeholders::_1, std::placeholders::_2));
            get_robot_identity_service= this -> create_service<sopias4_framework::srv::GetRobotIdentity>("get_robot_identity", std::bind(&MultiRobotCoordinator::get_robot_identity_callback, this,std::placeholders::_1, std::placeholders::_2));
            get_robots_service = this -> create_service<sopias4_framework::srv::GetRobots>("get_robots", std::bind(&MultiRobotCoordinator::get_robots_callback, this, std::placeholders::_1, std::placeholders::_2) );
            register_service = this -> create_service<sopias4_framework::srv::Register>("register", std::bind(&MultiRobotCoordinator::register_callback, this, std::placeholders::_1, std::placeholders::_2));
            unregister_service = this -> create_service<sopias4_framework::srv::Unregister>("unregister", std::bind(&MultiRobotCoordinator::unregister_callback, this, std::placeholders::_1, std::placeholders::_2));
            set_robot_path_service = this -> create_service<sopias4_framework::srv::SetRobotPath>("set_robot_path", std::bind(&MultiRobotCoordinator::set_robot_path_callback, this, std::placeholders::_1, std::placeholders::_2));
    }
private:
    // All registered namespaces
    std::vector<std::string> registered_namespaces = {};
    std::vector<sopias4_framework::msg::Robot> robot_states = {};

    // Services
    rclcpp::Service<sopias4_framework::srv::GetNamespaces>::SharedPtr get_namespaces_service;
    rclcpp::Service<sopias4_framework::srv::GetRobotIdentity>::SharedPtr get_robot_identity_service;
    rclcpp::Service<sopias4_framework::srv::GetRobots>::SharedPtr get_robots_service;
    rclcpp::Service<sopias4_framework::srv::Register>::SharedPtr register_service;
    rclcpp::Service<sopias4_framework::srv::Unregister>::SharedPtr unregister_service;
    rclcpp::Service<sopias4_framework::srv::SetRobotPath>::SharedPtr set_robot_path_service;

    void get_namespace_callback(const sopias4_framework::srv::GetNamespaces::Request::SharedPtr request, sopias4_framework::srv::GetNamespaces::Response::SharedPtr response ){
        response -> name_spaces = registered_namespaces;
        return;
    }

    void get_robot_identity_callback(const sopias4_framework::srv::GetRobotIdentity::Request::SharedPtr request, sopias4_framework::srv::GetRobotIdentity::Response::SharedPtr response){
        // Search through all robot states
        for(auto element = robot_states.begin(); element != robot_states.end();++element){
            // Robot is identified by namespace
            if( element->name_space == request -> name_space){
                response ->robot = *element;
                break;
            }
        }
        return;
    }

    void get_robots_callback(const sopias4_framework::srv::GetRobots::Request::SharedPtr request, sopias4_framework::srv::GetRobots::Response::SharedPtr response){
        response->robots = robot_states;
        return;
    }

    void register_callback(const sopias4_framework::srv::Register::Request::SharedPtr request, sopias4_framework::srv::Register::Response::SharedPtr response){
            // --- Validate namespace ---
            int validation_result;
            size_t invalid_index;
            
            rmw_ret_t rmw_ret =
                rmw_validate_namespace(request->namespace_canditate.c_str(), &validation_result, &invalid_index);

            // Check if validation itself failed
            if (rmw_ret != RMW_RET_OK) {
                if (rmw_ret == RMW_RET_INVALID_ARGUMENT) {
                    rclcpp::exceptions::throw_from_rcl_error(RCL_RET_INVALID_ARGUMENT, "failed to validate subnode namespace");
                }
                    rclcpp::exceptions::throw_from_rcl_error(RCL_RET_ERROR, "failed to validate subnode namespace");
            }
            // Check result from validatiom
            if (validation_result != RMW_NAMESPACE_VALID) {
                response -> statuscode = sopias4_framework::srv::Register::Response::ILLEGAL_NAMESPACE_ERROR;
                return;
            }

            // --- Check if namespace is already registered --
            if (std::find(registered_namespaces.begin(), registered_namespaces.end(), request->namespace_canditate) != registered_namespaces.end()) {
                response -> statuscode = sopias4_framework::srv::Register::Response::COLLISION_ERROR;
                return;
            }

            // --- Register namespace ---
            registered_namespaces.push_back(request->namespace_canditate);
            response -> statuscode = sopias4_framework::srv::Register::Response::SUCCESS;

            // Use a callback factory to pass a second argument to callback function (known bug in ros2, so this workaround is needed)
            std::function<void(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)> callback_fcn = std::bind(&MultiRobotCoordinator::set_pose_callback, this , std::placeholders::_1, request->namespace_canditate);

            this -> create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(request->namespace_canditate+ std::string("/amcl_pose"), 10, callback_fcn);
            this -> create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(request->namespace_canditate+ std::string("/pose"), 10, callback_fcn);

            return;
    }

    void unregister_callback(const sopias4_framework::srv::Unregister::Request::SharedPtr request, sopias4_framework::srv::Unregister::Response::SharedPtr response){
        response -> statuscode = sopias4_framework::srv::Unregister::Response::UNKNOWN_ERROR;
         // --- Delete namespace --
        for(auto element = registered_namespaces.begin(); element != registered_namespaces.end(); ++element){
            response -> statuscode = sopias4_framework::srv::Unregister::Response::NS_NOT_FOUND;
            if( *element == request -> name_space){
                registered_namespaces.erase(element);
                response -> statuscode = sopias4_framework::srv::Unregister::Response::SUCCESS;
                break;
            }
        }

        // --- Delete state of robot ---
        for(auto element = robot_states.begin(); element != robot_states.end(); ++element){
            if( element -> name_space == request -> name_space){
                robot_states.erase(element);
                response -> statuscode = sopias4_framework::srv::Unregister::Response::SUCCESS;
                break;
            }
        }

        return;
    }

    void set_robot_path_callback(const sopias4_framework::srv::SetRobotPath::Request::SharedPtr request, sopias4_framework::srv::SetRobotPath::Response::SharedPtr response){
        for(auto element = robot_states.begin(); element != robot_states.end(); ++element){
            response -> statuscode = sopias4_framework::srv::SetRobotPath::Response::ROBOT_NOT_FOUND;
            if( element -> name_space == request -> name_space){
                element ->nav_path = request -> path;
                response -> statuscode = sopias4_framework::srv::SetRobotPath::Response::SUCCESS;
                break;
            }
        }
        return;
    }

    void set_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose, std::string name_space){
        for(auto element = robot_states.begin(); element != robot_states.end(); ++element){
            if(element -> name_space == name_space){
                element ->pose= *pose;
                return;
            }
        }
    }        
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);

    // Create node 
    auto node = std::make_shared<MultiRobotCoordinator>("multi_robot_coordinator");

    // Run node
    rclcpp::spin(node);

    // Shutdown node
    rclcpp::shutdown();

    return 0;
}