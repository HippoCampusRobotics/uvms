from ament_index_python.packages import get_package_share_path
import launch
import launch_ros


def generate_launch_description():
    alpha_sim_path = get_package_share_path('alpha_sim')



    package_path = get_package_share_path('uvms_sim')
    # use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time')
    object_name = launch.substitutions.LaunchConfiguration('object_name')
    # simulate_kinematics = launch.substitutions.LaunchConfiguration('simulate_kinematics')


    model_path = str(alpha_sim_path / 'models/object/urdf/cylinder.urdf.xacro')
    # tf_tree_model_path = str(package_path / 'models/urdf/uvms_rviz.urdf.xacro')

    # use_sim_time_launch_arg = launch.actions.DeclareLaunchArgument(
    #     name='use_sim_time',
    #     default_value=str(True),
    #     description='decides if system time or simulated time is used'
    # )
    # vehicle_name_launch_arg = launch.actions.DeclareLaunchArgument(
    #     name='vehicle_name',
    #     default_value='uvms',
    #     description='vehicle name used as namespace '
    # )

    # simulate_kinematics_launch_arg = launch.actions.DeclareLaunchArgument(
    #     name='simulate_kinematics',
    #     default_value=str(False),
    #     description='decide if gazebo simulation should be executed on torque or velocity level'
    # )

    # state_publisher = launch_ros.actions.Node(package='robot_state_publisher',
    #                                           executable='robot_state_publisher',
    #                                           name='robot_state_publisher',
    #                                           output='screen',
    #                                           parameters=[{'use_sim_time': use_sim_time,
    #                                                        'robot_description': launch_ros.descriptions.ParameterValue(
    #                                                            launch.substitutions.Command(
    #                                                                ['xacro ', tf_tree_model_path,
    #                                                                 ' ', 'vehicle_name:=', vehicle_name]),
    #                                                            value_type=str)}])  # pi: 3.14159265359
    object_description = launch.substitutions.LaunchConfiguration(
        'object_description',
        default=launch.substitutions.Command([
            'ros2 run hippo_sim create_robot_description.py ', '--input ',
            model_path
        ]))

    description = {'object_description': object_description}

    spawner = launch_ros.actions.Node(package='hippo_sim',
                                      executable='spawn',
                                      parameters=[description],
                                      arguments=[
                                          '--param',
                                          'object_description',
                                          '--remove_on_exit',
                                          'true',
                                          '--x',
                                          '1.0',
                                          '--y',
                                          '2.5',
                                          '--z',
                                          '-0.5',
                                      ])

    # bridge = launch_ros.actions.Node(package='uvms_sim',
    #                                  executable='bridge',
    #                                  parameters=[{'use_sim_time': use_sim_time,
    #                                               'update_frequency': 50.0,
    #                                               'simulate_kinematics': simulate_kinematics}],
    #                                  output='screen')

    spawn_group = launch.actions.GroupAction([
        launch_ros.actions.PushRosNamespace(
            object_name),
        # state_publisher,
        spawner,
        # bridge
        ])

    # launch_path = str(
    #     get_package_share_path('hippo_common') /
    #     'launch/tf_publisher_hippo.launch.py')
    # launch_source = launch.launch_description_sources.PythonLaunchDescriptionSource(launch_path)
    # tf_publisher_vehicle = launch.actions.IncludeLaunchDescription(
    #     launch_source,
    #     launch_arguments={
    #         'vehicle_name': vehicle_name,
    #         'use_sim_time': use_sim_time,
    #     }.items(),
    # )

    return launch.LaunchDescription([
        # use_sim_time_launch_arg,
        # vehicle_name_launch_arg,
        # simulate_kinematics_launch_arg,
        spawn_group,
        # tf_publisher_vehicle
    ])
