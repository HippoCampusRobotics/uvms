from ament_index_python.packages import get_package_share_path
import launch
import launch_ros


def generate_launch_description():
    package_path = get_package_share_path('uvms_sim')
    hippo_sim_path = get_package_share_path('hippo_sim')
    alpha_ctrl_path = get_package_share_path('alpha_ctrl')
    alpha_model_path = get_package_share_path('alpha_model')

    vehicle_name = 'klopsi00'
    simulate_kinematics = False
    use_sim_time = True
    use_hydro = True

    start_gazebo = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            str(hippo_sim_path / 'launch/start_gazebo.launch.py')))

    spawn_uvms = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            str(package_path / 'launch/spawn_uvms.launch.py')),
        launch_arguments={'vehicle_name': vehicle_name,
                          'use_sim_time': str(use_sim_time),
                          'simulate_kinematics': str(simulate_kinematics)}.items()
    )

    alpha_sim_interface = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            str(alpha_ctrl_path / 'launch/simulation_velocity_control.launch.py')),
        launch_arguments={
            'vehicle_name': vehicle_name,
            'base_tf_file': str(alpha_model_path / 'config/alpha_base_tf_params_bluerov.yaml'),
            'use_hydro': str(use_hydro),
            'update_base_ref': str(True),
            'is_sim' : str(use_sim_time)}.items())

    bluerov_estimation = launch_ros.actions.Node(package='hippo_sim',
                                                 executable='fake_state_estimator',
                                                 namespace=vehicle_name,
                                                 parameters=[{'use_sim_time' : use_sim_time}],
                                                 name='bluerov_state_estimator',
                                                 output='screen')

    return launch.LaunchDescription([
        start_gazebo,
        spawn_uvms,
        alpha_sim_interface,
        bluerov_estimation
    ])