from ament_index_python.packages import get_package_share_path
import launch
import launch_ros

def generate_launch_description():
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time')
    vehicle_name = launch.substitutions.LaunchConfiguration('vehicle_name')


    estimation_config_path = str(get_package_share_path('bluerov_estimation')/"config")
    noise_param_file = estimation_config_path + "/kf_config.yaml"
    model_param_file = str(get_package_share_path('bluerov_ctrl')/"config/model_params.yaml")


    use_sim_time_launch_arg = launch.actions.DeclareLaunchArgument(
        name='use_sim_time',
        default_value=str('false'),
        description='Use simulation(Gazebo) clock if true')


    vehicle_name_launch_arg = launch.actions.DeclareLaunchArgument(
        name='vehicle_name',
        default_value='bluerov',
        description='Vehicle name used as namespace'
    )


    return launch.LaunchDescription([
        use_sim_time_launch_arg,
        vehicle_name_launch_arg,
        launch_ros.actions.Node(package='bluerov_estimation',
                                executable='estimation_node',
                                namespace=vehicle_name,
                                parameters=[{'use_sim_time': use_sim_time},
                                            noise_param_file, model_param_file],
                                output='screen',
                                )
    ])