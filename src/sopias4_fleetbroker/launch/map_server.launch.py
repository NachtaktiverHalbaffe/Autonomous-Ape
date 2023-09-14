import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

SOPIAS4_FLEETBROKER_PATH = get_package_share_directory("sopias4_fleetbroker")
ARGUMENTS = [
    DeclareLaunchArgument(
        "map",
        default_value=os.path.join(
            SOPIAS4_FLEETBROKER_PATH, "maps", "default_map.yaml"
        ),
        description="Full path to map yaml file to load",
    ),
    DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(
            SOPIAS4_FLEETBROKER_PATH, "config", "map_server.yaml"
        ),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    ),
    DeclareLaunchArgument(
        "use_respawn",
        default_value="false",
        description="Whether to respawn if a node crashes",
    ),
    DeclareLaunchArgument("log_level", default_value="info", description="log level"),
    DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the nav2 stack",
    ),
]


def generate_launch_description():
    # Get the launch directory

    map_yaml_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")
    autostart = LaunchConfiguration("autostart")
    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={},
        convert_types=True,
    )
    lifecycle_nodes = ["map_server", "map_saver"]

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    map_server_empty = Node(
        condition=LaunchConfigurationEquals("map", ""),
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[
            configured_params,
            {"topic_name": "map"},
            {"frame_id": "map"},
        ],
        arguments=["--ros-args", "--log-level", log_level],
        remappings=remappings,
    )

    map_server_map_loaded = Node(
        condition=LaunchConfigurationNotEquals("map", ""),
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        respawn=use_respawn,
        parameters=[
            configured_params,
            {"yaml_filename": map_yaml_file},
            {"topic_name": "map"},
            {"frame_id": "map"},
        ],
        respawn_delay=2.0,
        arguments=["--ros-args", "--log-level", log_level],
        remappings=remappings,
    )

    map_saver = Node(
        package="nav2_map_server",
        executable="map_saver_server",
        output="screen",
        emulate_tty=True,  # https://github.com/ros2/launch/issues/188
        parameters=[
            {"save_map_timeout": 2.0},
            {"free_thresh_default": 0.25},
            {"occupied_thresh_default": 0.65},
        ],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_fleetbroker",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[{"autostart": autostart}, {"node_names": lifecycle_nodes}],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Add the actions to launch all of the localiztion nodes
    ld.add_action(map_server_empty)
    ld.add_action(map_server_map_loaded)
    ld.add_action(map_saver)
    ld.add_action(lifecycle_manager)

    return ld
