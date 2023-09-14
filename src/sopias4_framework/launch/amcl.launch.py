from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import (
    IfCondition,
    LaunchConfigurationEquals,
    LaunchConfigurationNotEquals,
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from nav2_common.launch import RewrittenYaml

ARGUMENTS = [
    DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        choices=["true", "false"],
        description="Use sim time",
    ),
    DeclareLaunchArgument("namespace", default_value="", description="Robot namespace"),
    DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn if a node crashes",
    ),
    DeclareLaunchArgument("log_level", default_value="WARN", description="log level"),
    DeclareLaunchArgument(
        "use_composition",
        default_value="True",
        description="Whether to use composed bringup",
    ),
    DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the localization stack",
    ),
    DeclareLaunchArgument(
        "container_name",
        default_value="amcl_container",
        description="the name of conatiner that nodes will load in if use composition",
    ),
    DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution(
            [
                get_package_share_directory("sopias4_framework"),
                "config",
                "amcl.yaml",
            ]
        ),
        description="AMCL parameters",
    ),
]


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    params_file = LaunchConfiguration("params_file")
    use_composition = LaunchConfiguration("use_composition")
    autostart = LaunchConfiguration("autostart")
    log_level = LaunchConfiguration("log_level")
    use_respawn = LaunchConfiguration("use_respawn")
    container_name = LaunchConfiguration("container_name")
    container_name_full = (namespace, "/", container_name)

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static"), ("map", "/map")]

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites={},
        convert_types=True,
    )

    amcl_container = GroupAction(
        condition=IfCondition(use_composition),
        actions=[
            Node(
                condition=LaunchConfigurationNotEquals("namespace", ""),
                name=container_name,
                package="rclcpp_components",
                executable="component_container_isolated",
                parameters=[{"autostart": autostart}],
                arguments=[
                    "--ros-args",
                    "--log-level",
                    [
                        PythonExpression(['"', namespace, '"[1::]']),
                        ".amcl:=",
                        log_level,
                    ],
                ],
                remappings=remappings,
                namespace=namespace,
                output="screen",
            ),
            Node(
                condition=LaunchConfigurationEquals("namespace", ""),
                name=container_name,
                package="rclcpp_components",
                executable="component_container_isolated",
                parameters=[{"autostart": autostart}],
                arguments=[
                    "--ros-args",
                    "--log-level",
                    ["amcl:=", log_level],
                ],
                remappings=remappings,
                namespace=namespace,
                output="screen",
            ),
        ],
    )

    amcl = GroupAction(
        condition=IfCondition(PythonExpression(["not ", use_composition])),
        actions=[
            Node(
                package="nav2_amcl",
                executable="amcl",
                name="amcl",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
                namespace=namespace,
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_localization",
                output="screen",
                namespace=namespace,
                arguments=["--ros-args", "--log-level", log_level],
                parameters=[
                    {"autostart": autostart},
                    {"node_names": ["amcl"]},
                ],
            ),
        ],
    )

    amcl_composable = GroupAction(
        condition=IfCondition(use_composition),
        actions=[
            LoadComposableNodes(
                target_container=container_name_full,
                composable_node_descriptions=[
                    ComposableNode(
                        package="nav2_amcl",
                        plugin="nav2_amcl::AmclNode",
                        name="amcl",
                        parameters=[configured_params],
                        remappings=remappings,
                        namespace=namespace,
                    ),
                    ComposableNode(
                        package="nav2_lifecycle_manager",
                        plugin="nav2_lifecycle_manager::LifecycleManager",
                        name="lifecycle_manager_localization",
                        namespace=namespace,
                        parameters=[{"autostart": autostart, "node_names": ["amcl"]}],
                    ),
                ],
            ),
        ],
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(amcl_container)
    ld.add_action(amcl)
    ld.add_action(amcl_composable)
    return ld
