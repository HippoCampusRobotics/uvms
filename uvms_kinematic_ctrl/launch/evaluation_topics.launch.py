import launch
import launch_ros


def generate_launch_description():
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
                                              parameters=[{'use_sim_time': use_sim_time},],
                                              output='screen'
                                              )
                                 
    return launch.LaunchDescription([
        vehicle_name_launch_arg,
        use_sim_time_launch_arg,
        evaluation_node
    ])
