#include "rclcpp/rclcpp.hpp"
#include "domain_bridge/domain_bridge.hpp"
#include "sopias4_msgs/srv/registry_service.hpp"
#include "nav2_msgs/srv/save_map.hpp"

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
    
    // --- Configure bridge --- 
    RCLCPP_INFO(rclcpp::get_logger("sopias4_domain_bridge_logger"),"Configuring Sopias4 DomainBridge");
    domain_bridge::DomainBridgeOptions options = domain_bridge::DomainBridgeOptions();
    options = options.name("sopias4_domain_bridge");
    domain_bridge::DomainBridgeConfig config = domain_bridge::DomainBridgeConfig();
    config.options = options;
	domain_bridge::DomainBridge domain_bridge = domain_bridge::DomainBridge(config);
    
	// --- Bridge topics ---
	RCLCPP_INFO(rclcpp::get_logger("sopias4_domain_bridge_logger"),"Adding topics which should be bridged");
    // map from fleetbroker
	domain_bridge.bridge_topic("/map","nav_msgs/msg/OccupancyGrid",0,1);
	domain_bridge.bridge_topic("/map","nav_msgs/msg/OccupancyGrid",0,2);
	domain_bridge.bridge_topic("/map","nav_msgs/msg/OccupancyGrid",0,3);
	// States of robots
	domain_bridge.bridge_topic("/robot_states","sopias4_msgs/msg/RobotStates",0,1);
	domain_bridge.bridge_topic("/robot_states","sopias4_msgs/msg/RobotStates",0,2);
	domain_bridge.bridge_topic("/robot_states","sopias4_msgs/msg/RobotStates",0,3);
	// amcl_pose
	domain_bridge.bridge_topic("/turtle1/amcl_pose","geometry_msgs/msg/PoseWithCovarianceStamped",1,0);
	domain_bridge.bridge_topic("/turtle2/amcl_pose","geometry_msgs/msg/PoseWithCovarianceStamped",2,0);
	domain_bridge.bridge_topic("/turtle3/amcl_pose","geometry_msgs/msg/PoseWithCovarianceStamped",3,0);
	// slam pose
	domain_bridge.bridge_topic("/turtle1/pose","geometry_msgs/msg/PoseWithCovarianceStamped",1,0);
	domain_bridge.bridge_topic("/turtle2/pose","geometry_msgs/msg/PoseWithCovarianceStamped",2,0);
	domain_bridge.bridge_topic("/turtle3/pose","geometry_msgs/msg/PoseWithCovarianceStamped",3,0);
	// planned global path
	domain_bridge.bridge_topic("/turtle1/plan","nav_msgs/msg/Path",1,0);
	domain_bridge.bridge_topic("/turtle2/plan","nav_msgs/msg/Path",2,0);
	domain_bridge.bridge_topic("/turtle3/plan","nav_msgs/msg/Path",3,0);
	// Is navigating topic
	domain_bridge.bridge_topic("/turtle1/is_navigating","std_msgs/msg/Bool",1,0);
	domain_bridge.bridge_topic("/turtle2/is_navigating","std_msgs/msg/Bool",2,0);
	domain_bridge.bridge_topic("/turtle3/is_navigating","std_msgs/msg/Bool",3,0);
	// Map from slam
	domain_bridge.bridge_topic("/turtle1/map","nav_msgs/msg/OccupancyGrid",1,0);
	domain_bridge.bridge_topic("/turtle2/map","nav_msgs/msg/OccupancyGrid",2,0);
	domain_bridge.bridge_topic("/turtle3/map","nav_msgs/msg/OccupancyGrid",3,0);
	// velocity from manual steering
	domain_bridge.bridge_topic("/turtle1/cmd_vel","geometry_msgs/msg/Twist",1,0);
	domain_bridge.bridge_topic("/turtle2/cmd_vel","geometry_msgs/msg/Twist",2,0);
	domain_bridge.bridge_topic("/turtle3/cmd_vel","geometry_msgs/msg/Twist",3,0);
	// velocity from nav2
	domain_bridge.bridge_topic("/turtle1/cmd_vel_nav","geometry_msgs/msg/Twist",1,0);
	domain_bridge.bridge_topic("/turtle2/cmd_vel_nav","geometry_msgs/msg/Twist",2,0);
	domain_bridge.bridge_topic("/turtle3/cmd_vel_nav","geometry_msgs/msg/Twist",3,0);
	// velocity from nav2
	domain_bridge.bridge_topic("/turtle1/cmd_vel_teleop","geometry_msgs/msg/Twist",1,0);
	domain_bridge.bridge_topic("/turtle2/cmd_vel_teleop","geometry_msgs/msg/Twist",2,0);
	domain_bridge.bridge_topic("/turtle3/cmd_vel_teleop","geometry_msgs/msg/Twist",3,0);
	// --- Bridge services ---
    RCLCPP_INFO(rclcpp::get_logger("sopias4_domain_bridge_logger"),"Adding services which should be bridged");
    // Register namespace service
	domain_bridge.bridge_service<sopias4_msgs::srv::RegistryService>("/register_namespace",0,1);
	domain_bridge.bridge_service<sopias4_msgs::srv::RegistryService>("/register_namespace",0,2);
	domain_bridge.bridge_service<sopias4_msgs::srv::RegistryService>("/register_namespace",0,3);
    // Unregister namespace service 
	domain_bridge.bridge_service<sopias4_msgs::srv::RegistryService>("/unregister_namespace",0,1);
	domain_bridge.bridge_service<sopias4_msgs::srv::RegistryService>("/unregister_namespace",0,2);
	domain_bridge.bridge_service<sopias4_msgs::srv::RegistryService>("/unregister_namespace",0,3);
    // Save map service
	domain_bridge.bridge_service<nav2_msgs::srv::SaveMap>("/map_saver/save_map",0,1);
	domain_bridge.bridge_service<nav2_msgs::srv::SaveMap>("/map_saver/save_map",0,2);
	domain_bridge.bridge_service<nav2_msgs::srv::SaveMap>("/map_saver/save_map",0,3);

	// Run node
    RCLCPP_INFO(rclcpp::get_logger("sopias4_domain_bridge_logger"),"Adding domain_bridge to executor");
    rclcpp::executors::MultiThreadedExecutor executor=  rclcpp::executors::MultiThreadedExecutor();
    RCLCPP_INFO(rclcpp::get_logger("sopias4_domain_bridge_logger"),"Sopias4 DomainBridge started successfully");
    domain_bridge.add_to_executor(executor);
	executor.spin();

	// Shutdown node	
	rclcpp::shutdown();

	return 0;
}