import launch
import launch_ros
from ament_index_python.packages import get_package_share_path
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    sim_package_path = get_package_share_path('hippo_sim')
    low_level_ctrl_package_path = get_package_share_path('hippo_control')
    mixer_path = str(low_level_ctrl_package_path /
                     "launch/node_actuator_mixer_bluerov.launch.py")
    mixer_config_file_path = str(
        low_level_ctrl_package_path /
        ('config/actuator_mixer_bluerov_advanced.yaml'))

    ctrl_package_path = get_package_share_path('bluerov_ctrl')
    velocity_ctrl_path = str(ctrl_package_path /
                             "launch/node_velocity_control.launch.py")
    ctrl_config_path = str(ctrl_package_path / "config/ctrl_params_real.yaml")
    pose_ctrl_module_path = str(ctrl_package_path /
                                "launch/node_pose_control_module.launch.py")
    estimation_watchdog_path = str(
        ctrl_package_path / "launch/node_estimation_drift_watchdog.launch.py")

    estimation_package_path = get_package_share_path('bluerov_estimation')
    estimation_path = str(estimation_package_path /
                          "launch/estimation.launch.py")
    visualization_package_path = get_package_share_path('uvms_visualization')
    visualization_path = str(visualization_package_path /
                             "launch/visualization.launch.py")
    rviz_path = str(ctrl_package_path / "launch/rviz.launch.py")
    model_path_rviz = str(sim_package_path /
                          'models/bluerov/urdf/bluerov_rviz.xacro')

    vehicle_name = 'klopsi00'
    use_sim_time = False
    launch_files = []

    launch_files.append(
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                visualization_path),
            launch_arguments={
                'visualization_modules': "[2, 3, 4, 5, 6, 9]",
                'use_sim_time': str(use_sim_time),
                'vehicle_name': vehicle_name
            }.items()))
    launch_files.append(
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                velocity_ctrl_path),
            launch_arguments={
                'use_sim_time': str(use_sim_time),
                'controller_type': str(4),
                'vehicle_name': vehicle_name,
                'config_file': ctrl_config_path
            }.items()))
    launch_files.append(
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                pose_ctrl_module_path),
            launch_arguments={
                'use_sim_time': str(use_sim_time),
                'vehicle_name': vehicle_name,
                'config_file': ctrl_config_path
            }.items()))
    launch_files.append(
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                estimation_watchdog_path),
            launch_arguments={
                'use_sim_time': str(use_sim_time),
                'vehicle_name': vehicle_name
            }.items()))

    launch_files.append(
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                rviz_path),
            launch_arguments={
                'use_sim_time': str(use_sim_time),
            }.items()))
    launch_files.append(
        launch_ros.actions.Node(package='robot_state_publisher',
                                executable='robot_state_publisher',
                                name='robot_state_publisher',
                                namespace=vehicle_name,
                                output='screen',
                                parameters=[{
                                    'use_sim_time':
                                    use_sim_time,
                                    'robot_description':
                                    launch_ros.descriptions.ParameterValue(
                                        launch.substitutions.Command([
                                            'xacro ', model_path_rviz,
                                            " vehicle_name:=", vehicle_name
                                        ]),
                                        value_type=str)
                                }]))  # pi: 3.14159265359

    launch_path = str(
        get_package_share_path('hippo_common') /
        'launch/tf_publisher_hippo.launch.py')
    launch_source = PythonLaunchDescriptionSource(launch_path)
    launch_files.append(
        launch.actions.IncludeLaunchDescription(
            launch_source,
            launch_arguments={
                'vehicle_name': vehicle_name,
                'use_sim_time': str(use_sim_time),
            }.items(),
        ))

    launch_files.append(
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                mixer_path),
            launch_arguments={
                'use_sim_time': str(use_sim_time),
                'vehicle_name': vehicle_name,
                'mixer_path': mixer_config_file_path
            }.items()))

    launch_files.append(
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                estimation_path),
            launch_arguments={
                'use_sim_time': str(use_sim_time),
                'vehicle_name': vehicle_name
            }.items()))

    return launch.LaunchDescription([
        *launch_files,
    ])
