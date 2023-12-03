from ament_index_python.packages import get_package_share_path
import launch
import launch_ros


def generate_launch_description():
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time')
    vehicle_name = launch.substitutions.LaunchConfiguration('vehicle_name')
    visualization_modules = launch.substitutions.LaunchConfiguration('visualization_modules')

    use_sim_time_launch_arg = launch.actions.DeclareLaunchArgument(
        name='use_sim_time',
        default_value=str('false'),
        description='Use simulation(Gazebo) clock if true')
    vehicle_name_launch_arg = launch.actions.DeclareLaunchArgument(
        name='vehicle_name',
        default_value='uvms',
        description='Vehicle name used as namespace'
    )
    visualization_modules_launch_arg = launch.actions.DeclareLaunchArgument(
        name='visualization_modules',
        default_value=str([2, 3, 4, 5]),
        description='Vehicle name used as namespace'
    )

    return launch.LaunchDescription([
        use_sim_time_launch_arg,
        vehicle_name_launch_arg,
        visualization_modules_launch_arg,
        launch_ros.actions.Node(package='uvms_visualization',
                                executable='visualization_node',
                                namespace=vehicle_name,
                                parameters=[{'use_sim_time': use_sim_time,
                                             'vehicle_name': vehicle_name,
                                             'visualization_modules': visualization_modules}])
    ])
