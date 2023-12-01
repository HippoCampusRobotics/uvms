from ament_index_python.packages import get_package_share_path
import launch
import launch_ros

def generate_launch_description():
    package_path = get_package_share_path("uvms_visualization")
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time')
    namespace = "bag"
    vehicle_namespace = "/klopsi00"
    visualization_modules = launch.substitutions.LaunchConfiguration('visualization_modules')

    use_sim_time_launch_arg = launch.actions.DeclareLaunchArgument(
        name='use_sim_time',
        default_value=str('false'),
        description='Use simulation(Gazebo) clock if true')
    visualization_modules_launch_arg = launch.actions.DeclareLaunchArgument(
        name='visualization_modules',
        default_value=str([1, 11, 3, 10]),
        description='Vehicle name used as namespace'
    )
    rviz = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            str(package_path / 'launch/rviz_bag_config.launch.py')),
        launch_arguments={
            'use_sim_time' : use_sim_time
        }.items()
    )

    visualization_topics = ["/pose_endeffector", "/traj_setpoint", "/odometry", "/velocity_setpoint", "/thrust_setpoint", "/traj_setpoint_uvms"]
    print([( "/" + namespace + topic, vehicle_namespace + topic) for topic in visualization_topics])
    return launch.LaunchDescription([
        use_sim_time_launch_arg,
        visualization_modules_launch_arg,
        rviz,
        launch_ros.actions.Node(package='uvms_visualization',
                                executable='visualization_node',
                                namespace=namespace,
                                remappings=[('/' + namespace + topic, vehicle_namespace + topic) for topic in visualization_topics],
                                parameters=[{'use_sim_time': use_sim_time,
                                             'vehicle_name': namespace,
                                             'visualization_modules': visualization_modules}])
    ])