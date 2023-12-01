import os
from ament_index_python.packages import get_package_share_directory
import launch
from launch_ros.actions import Node

def generate_launch_description():

    pkg_share = get_package_share_directory('bluerov_ctrl')
    rviz_config_file = os.path.join(pkg_share, 'rviz/rviz.rviz')

    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time')

    use_sim_time_launch_arg = launch.actions.DeclareLaunchArgument(
        name='use_sim_time',
        default_value=str('false'),
        description='Use simulation(Gazebo) clock if true')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='RVIZ',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # without the following transform rviz spams warnings that the map frame does not exist before the simulation is
    # started
    static_world_map_tf = Node(package="tf2_ros",
                               executable="static_transform_publisher",
                               arguments=["0", "0", "0", "0", "0", "0","world", "map"],
                               parameters=[{'use_sim_time': use_sim_time}])

    rviz_mesh_publisher = Node(package='hippo_common',
                                                  executable='rviz_mesh_publisher',
                                                  parameters=[{'use_sim_time': use_sim_time}]
                                                  )

    return launch.LaunchDescription([
        use_sim_time_launch_arg,
        rviz_node,
        rviz_mesh_publisher,
        static_world_map_tf
    ])