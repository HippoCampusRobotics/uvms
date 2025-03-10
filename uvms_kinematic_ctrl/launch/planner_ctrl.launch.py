from ament_index_python.packages import get_package_share_path
import launch
import launch_ros


def generate_launch_description():
    package_path = get_package_share_path('uvms_kinematic_ctrl')
    config_file_path = str(package_path / 'config/planner_params.yaml')

    vehicle_name = launch.substitutions.LaunchConfiguration('vehicle_name')
    # offset_distance = launch.substitutions.LaunchConfiguration('offset_distance')
    # number_test_rounds = launch.substitutions.LaunchConfiguration('number_test_rounds')

    vehicle_name_launch_arg = launch.actions.DeclareLaunchArgument(
        name='vehicle_name',
        default_value='klopsi00',
        description='used for node namespace'
    )

    # offset_dist_launch_arg = launch.actions.DeclareLaunchArgument(
    #     name='offset_distance',
    #     default_value=str(0.1),
    #     description='offset to the final pose of the object or placement'
    # )

    # number_test_rounds_launch_arg = launch.actions.DeclareLaunchArgument(
    #     name='number_test_rounds',
    #     default_value=str(1),
    #     description='the pick and place test will be repeated by this number of times'
    # )

    
    planner_node = launch_ros.actions.Node(package='uvms_kinematic_ctrl',
                                           executable='uvms_planner_node',
                                           namespace=vehicle_name,
                                           parameters=[config_file_path,
                                                       ],
                                           output='screen'
                                           )
    # {'offset_distance': offset_distance,
    #                                                     'number_test_rounds': number_test_rounds},

    return launch.LaunchDescription([
        vehicle_name_launch_arg,
        # offset_dist_launch_arg,
        # number_test_rounds_launch_arg,
        planner_node
    ])
