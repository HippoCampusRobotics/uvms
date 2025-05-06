from ament_index_python.packages import get_package_share_path
import launch
import launch_ros


def generate_launch_description():
    package_path = get_package_share_path('uvms_kinematic_ctrl')
    config_file_path = str(package_path / 'config/planner_params.yaml')

    vehicle_name = launch.substitutions.LaunchConfiguration('vehicle_name')

    vehicle_name_launch_arg = launch.actions.DeclareLaunchArgument(
        name='vehicle_name',
        default_value='klopsi00',
        description='used for node namespace'
    )
    
    planner_node = launch_ros.actions.Node(package='uvms_kinematic_ctrl',
                                           executable='uvms_planner_node',
                                           namespace=vehicle_name,
                                           parameters=[config_file_path,
                                                       ],
                                           output='screen'
                                           )

    return launch.LaunchDescription([
        vehicle_name_launch_arg,
        planner_node
    ])
