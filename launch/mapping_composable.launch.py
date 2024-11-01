import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition

from launch_ros.actions import ComposableNodeContainer, Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    package_path = get_package_share_directory('fast_lio')
    default_config_path = os.path.join(package_path, 'config')
    default_rviz_config_path = os.path.join(
        package_path, 'rviz', 'fastlio.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path')
    config_file = LaunchConfiguration('config_file')
    rviz_use = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')
    container_name = LaunchConfiguration('container_name')

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
    container_name_arg = DeclareLaunchArgument(
        name='container_name', 
        default_value="osprey_container", 
        description="container name") 

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
        arguments = "-0.010 0.005 0.080 0.001 -0.001 0.701 0.713 odom_vilens odom_fastlio".split(' ')
    )
    base_tf_node = Node(
        package = "tf2_ros", 
        executable = "static_transform_publisher",
        arguments = "-0.005 -0.010 -0.080 -0.001 0.001 -0.701 0.713 imu_fastlio base_fastlio".split(' ')
    )
    ouster_tf_node = Node(
        package = "tf2_ros", 
        executable = "static_transform_publisher",
        arguments = "0 0 0 0 0 0 1 imu_fastlio os1_imu".split(' ')
    )
    rooster_tf_node = Node(
        package = "tf2_ros", 
        executable = "static_transform_publisher",
        arguments = "-0.006253 0.011775 0.028535 0 0 1 0 os1_imu os1_lidar".split(' ')
    )
    camera_tf_node = Node(
        package = "tf2_ros", 
        executable = "static_transform_publisher",
        arguments = "0.07796081 0.02937116 -0.01774574 -0.6490941 0.2612981 -0.2760486 0.6589365 os1_imu rs_cam1_optical".split(' ')
    )
    base_tf_node = Node(
        package = "tf2_ros", 
        executable = "static_transform_publisher",
        arguments = "0 0 0 0.5 -0.5 0.5 0.5 rs_cam1_optical rs_cam1_base".split(' ')
    )
    odom_cam_tf_node = Node(
        package = "tf2_ros", 
        executable = "static_transform_publisher",
        arguments = "0.07796081 0.02937116 -0.01774574 -0.6490941 0.2612981 -0.2760486 0.6589365 odom_fastlio odom_cam1_optical".split(' ')
    )
    odom_base_tf_node = Node(
        package = "tf2_ros", 
        executable = "static_transform_publisher",
        arguments = "0 0 0 0.5 -0.5 0.5 0.5 odom_cam1_optical odom_cam1_base".split(' ')
    )

    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping_node',
        name='fast_lio',
        parameters=[PathJoinSubstitution([config_path, config_file]),
                    {'use_sim_time': use_sim_time}],
        output='screen',
        prefix=['gdbserver localhost:3000'],
    )

    fast_lio_comp = ComposableNode(
        package='fast_lio',
        plugin='fastlio::LaserMappingNode',
        name='fast_lio',
        parameters=[PathJoinSubstitution([config_path, config_file]),
                    {'use_sim_time': use_sim_time}],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    composable_node = LoadComposableNodes(
        target_container=container_name,
        composable_node_descriptions=[
            fast_lio_comp,
        ],
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(declare_config_file_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)
    ld.add_action(container_name_arg)
    ld.add_action(rviz_node)
    ld.add_action(ouster_tf_node)
    ld.add_action(rooster_tf_node)
    ld.add_action(camera_tf_node)
    ld.add_action(base_tf_node)
    ld.add_action(odom_cam_tf_node)
    ld.add_action(odom_base_tf_node)
    # ld.add_action(odom_tf_node)
    # ld.add_action(base_tf_node)
    ld.add_action(composable_node)
    # ld.add_action(fast_lio_node)

    return ld
