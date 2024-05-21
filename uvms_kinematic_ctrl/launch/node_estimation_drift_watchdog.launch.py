import launch
import launch_ros
from ament_index_python.packages import get_package_share_path


def generate_launch_description():
    package_path = get_package_share_path('uvms_kinematic_ctrl')
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time')
    vehicle_name = launch.substitutions.LaunchConfiguration('vehicle_name')
    config_file = str(package_path / 'config/watchdog_params.yaml')

    use_sim_time_launch_arg = launch.actions.DeclareLaunchArgument(
        name='use_sim_time',
        default_value=str(False),
        description='decide if sim time should be used for nodes',
    )
    vehicle_name_launch_arg = launch.actions.DeclareLaunchArgument(
        name='vehicle_name',
        default_value='uvms',
        description='used for node namespace',
    )

    control_node = launch_ros.actions.Node(
        package='uvms_kinematic_ctrl',
        executable='estimation_drift_watchdog_node',
        namespace=vehicle_name,
        parameters=[{'use_sim_time': use_sim_time}, config_file],
        output='screen',
    )

    return launch.LaunchDescription(
        [use_sim_time_launch_arg, vehicle_name_launch_arg, control_node]
    )
