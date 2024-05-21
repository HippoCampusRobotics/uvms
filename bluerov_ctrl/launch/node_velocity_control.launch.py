import launch
import launch_ros
from ament_index_python.packages import get_package_share_path


def generate_launch_description():
    config_file = launch.substitutions.LaunchConfiguration('config_file')
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time')
    vehicle_name = launch.substitutions.LaunchConfiguration('vehicle_name')
    controller_type = launch.substitutions.LaunchConfiguration(
        'controller_type'
    )
    model_params = str(
        get_package_share_path('bluerov_ctrl') / 'config/model_params.yaml'
    )

    use_sim_time_launch_arg = launch.actions.DeclareLaunchArgument(
        name='use_sim_time',
        default_value=str(False),
        description='decide if sim time should be used for nodes',
    )
    vehicle_name_launch_arg = launch.actions.DeclareLaunchArgument(
        name='vehicle_name',
        default_value='bluerov',
        description='used for node namespace',
    )
    controller_type_launch_arg = launch.actions.DeclareLaunchArgument(
        name='controller_type',
        default_value='2',
        description='used for node namespace',
    )

    config_file_launch_arg = launch.actions.DeclareLaunchArgument(
        name='config_file', description='config file with control parameters'
    )

    control_node = launch_ros.actions.Node(
        package='bluerov_ctrl',
        executable='velocity_control_node',
        namespace=vehicle_name,
        parameters=[
            {'use_sim_time': use_sim_time, 'controller_type': controller_type},
            config_file,
            model_params,
        ],
        output='screen',
    )

    return launch.LaunchDescription(
        [
            use_sim_time_launch_arg,
            vehicle_name_launch_arg,
            controller_type_launch_arg,
            config_file_launch_arg,
            control_node,
        ]
    )
