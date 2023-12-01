import os
from ament_index_python.packages import get_package_share_directory
import launch
from launch_ros.actions import Node

def generate_launch_description():

    pkg_share = get_package_share_directory('uvms_visualization')
    rviz_config_file = os.path.join(pkg_share, 'rviz/rviz_bag_config.rviz')

    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time')

    use_sim_time_launch_arg = launch.actions.DeclareLaunchArgument(
        name='use_sim_time',
        default_value=str('False'),
        description='Use simulation(Gazebo) clock if true')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='RVIZ',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return launch.LaunchDescription([
        use_sim_time_launch_arg,
        rviz_node,
    ])