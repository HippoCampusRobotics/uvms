from ament_index_python.packages import get_package_share_path
import launch
import launch_ros


def generate_launch_description():
    vehicle_name = launch.substitutions.LaunchConfiguration('vehicle_name')

    vehicle_name_launch_arg = launch.actions.DeclareLaunchArgument(
        name='vehicle_name',
        default_value='klopsi00',
        description='used for node namespace'
    )
    
    px4_bridge = launch_ros.actions.Node(package='visual_localization',
                                         executable='px4_bridge',
                                         namespace=vehicle_name,
                                         name='px4_bridge',
                                        #  parameters=[args],
                                         output='screen',
                                         emulate_tty=True,
                                         # remappings=[('odometry', 'inertial_odometry'),],
                                         )

    return launch.LaunchDescription([
        vehicle_name_launch_arg,
        px4_bridge
    ])
