from ament_index_python.packages import get_package_share_path
import launch
import launch_ros


def generate_launch_description():
    package_path = get_package_share_path('bluerov_ctrl')
    config_file = launch.substitutions.LaunchConfiguration('config_file')
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time')
    vehicle_name = launch.substitutions.LaunchConfiguration('vehicle_name')

    use_sim_time_launch_arg = launch.actions.DeclareLaunchArgument(
        name='use_sim_time',
        default_value=str(False),
        description='decide if sim time should be used for nodes'
    )
    vehicle_name_launch_arg = launch.actions.DeclareLaunchArgument(
        name='vehicle_name',
        default_value='bluerov',
        description='used for node namespace'
    )

    config_file_launch_arg = launch.actions.DeclareLaunchArgument(
        name='config_file',
        description='config file with control parameters'
    )

    control_node = launch_ros.actions.Node(package='bluerov_ctrl',
                                           executable='pose_control_module_node',
                                           namespace=vehicle_name,
                                           parameters=[{'use_sim_time': use_sim_time}, config_file],
                                           output='screen'
                                           )

    return launch.LaunchDescription([
        use_sim_time_launch_arg,
        vehicle_name_launch_arg,
        config_file_launch_arg,
        control_node
    ])
