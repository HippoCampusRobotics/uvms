from ament_index_python.packages import get_package_share_path
import launch
import launch_ros

def generate_launch_description():
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time')
    vehicle_name = launch.substitutions.LaunchConfiguration('vehicle_name')
    trajectory_type = launch.substitutions.LaunchConfiguration('trajectory_type')

    package_path = get_package_share_path("bluerov_trajectory_gen")
    config_file_path = str(package_path / "config/traj_params.yaml")

    use_sim_time_launch_arg = launch.actions.DeclareLaunchArgument(
        name='use_sim_time',
        default_value=str(False),
        description="Decides if sim time is used"
    )
    vehicle_name_launch_arg = launch.actions.DeclareLaunchArgument(
        name='vehicle_name',
        default_value='klopsi00',
        description='vehicle name used for namespace'
    )

    trajectory_type_launch_arg = launch.actions.DeclareLaunchArgument(
        name='trajectory_type',
        default_value='2',
        description='vehicle name used for namespace'
    )

    motion_planning_node = launch_ros.actions.Node(package='bluerov_trajectory_gen',
                                                   executable='trajectory_gen_node',
                                                    namespace=vehicle_name,
                                                   parameters=[{'trajectory_type': trajectory_type,
                                                       'use_sim_time' : use_sim_time},
                                                               config_file_path],
                                                   output='screen'
                                                   )
    return launch.LaunchDescription([
        use_sim_time_launch_arg,
        vehicle_name_launch_arg,
        trajectory_type_launch_arg,
        motion_planning_node
    ])

