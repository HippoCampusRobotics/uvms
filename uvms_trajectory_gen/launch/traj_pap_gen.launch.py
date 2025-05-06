from ament_index_python.packages import get_package_share_path
import launch
import launch_ros


def generate_launch_description():
    package_path = str(get_package_share_path('uvms_trajectory_gen') / "config")
    motion_param_file = package_path + "/traj_params.yaml"
    vehicle_name = launch.substitutions.LaunchConfiguration('vehicle_name')
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time')

    use_sim_time_launch_arg = launch.actions.DeclareLaunchArgument(
        name='use_sim_time',
        default_value=str('false'),
        description='Use simulation(Gazebo) clock if true')

    vehicle_name_launch_arg = launch.actions.DeclareLaunchArgument(
        name='vehicle_name',
        default_value='klopsi00',
        description='Vehicle name used as namespace'
    )

    node = launch_ros.actions.Node(package='uvms_trajectory_gen',
                                   executable='uvms_traj_gen_pick_place_node',
                                   namespace=vehicle_name,
                                   parameters=[{'use_sim_time': use_sim_time},
                                               motion_param_file],
                                   output='screen')


    return launch.LaunchDescription([
        use_sim_time_launch_arg,
        vehicle_name_launch_arg,
        node
    ])
