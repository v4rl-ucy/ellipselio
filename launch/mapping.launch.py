import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition

from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    package_path = get_package_share_directory('ellipse_livo')
    default_config_path = os.path.join(package_path, 'config')
    default_rviz_config_path = os.path.join(
        package_path, 'rviz', 'ellipselivo.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path')
    config_file = LaunchConfiguration('config_file')
    rviz_use = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='Yaml config file path'
    )
    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file', default_value='mid360.yaml',
        description='Config file'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Use RViz to monitor results'
    )
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_cfg', default_value=default_rviz_config_path,
        description='RViz config file path'
    )

    ellipse_livo_node = ComposableNode(
        package='ellipse_livo',
        plugin='ellipselivo::MappingNode',
        name='ellipse_livo_node',
        parameters=[PathJoinSubstitution([config_path, config_file]),
                    {'use_sim_time': use_sim_time}],
        extra_arguments=[{'use_intra_process_comms': True}],
    )
    ellipse_livo_container = ComposableNodeContainer(
        namespace='',
        package='rclcpp_components',
        name='ellipse_livo_container',
        executable='component_container_mt',
        composable_node_descriptions=[ellipse_livo_node],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz_use),
        output='log'
    )
    odom_tf_node = Node(
        package = "tf2_ros", 
        executable = "static_transform_publisher",
        arguments = "-0.010 0.005 0.080 0.001 -0.001 0.701 0.713 odom_vilens odom_ellipselivo".split(' ')
    )
    base_tf_node = Node(
        package = "tf2_ros", 
        executable = "static_transform_publisher",
        arguments = "-0.005 -0.010 -0.080 -0.001 0.001 -0.701 0.713 imu_ellipselivo base_ellipselivo".split(' ')
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(declare_config_file_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)

    ld.add_action(ellipse_livo_container)
    ld.add_action(rviz_node)
    ld.add_action(odom_tf_node)
    ld.add_action(base_tf_node)

    return ld
