from ament_index_python.packages import get_package_share_path
import launch
import launch_ros


def generate_launch_description():
    package_path = get_package_share_path('uvms_kinematic_ctrl')
    # config_file_path = str(package_path / 'config/planner_params.yaml')

    vehicle_name = launch.substitutions.LaunchConfiguration('vehicle_name')
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time')

    vehicle_name_launch_arg = launch.actions.DeclareLaunchArgument(
        name='vehicle_name',
        default_value='klopsi00',
        description='used for node namespace'
    )

    use_sim_time_launch_arg = launch.actions.DeclareLaunchArgument(
        name='use_sim_time',
        default_value=str(False),
        description='decide if sim time should be used for nodes'
    )

    
    evaluation_node = launch_ros.actions.Node(package='uvms_kinematic_ctrl',
                                              executable='pap_evaluation_node',
                                              namespace=vehicle_name,
                                              parameters=[{'use_sim_time': use_sim_time},
                                                        #   config_file_path,
                                                         ],
                                              output='screen'
                                              )
                                 
    return launch.LaunchDescription([
        vehicle_name_launch_arg,
        use_sim_time_launch_arg,
        evaluation_node
    ])
