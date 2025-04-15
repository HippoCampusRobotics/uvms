from ament_index_python.packages import get_package_share_path
import launch
import launch_ros


def generate_launch_description():
    alpha_model_path = get_package_share_path('alpha_model')
    alpha_estimation_path = get_package_share_path('alpha_estimation')
    bluerov_ctrl_path = get_package_share_path('bluerov_ctrl')
    bluerov_low_level_ctrl_path = get_package_share_path('hippo_control')
    bluerov_acceleration_estimation_path = get_package_share_path('bluerov_estimation')
    mixer_path = str(bluerov_low_level_ctrl_path / "launch/node_actuator_mixer_bluerov.launch.py")
    mixer_config_file_path = str(bluerov_low_level_ctrl_path / ('config/actuator_mixer_bluerov_advanced.yaml'))

    alpha_ctrl_path = get_package_share_path('alpha_ctrl')

    uvms_kin_ctrl_path = get_package_share_path('uvms_kinematic_ctrl')
    estimation_watchdog_path = str(uvms_kin_ctrl_path / "launch/node_estimation_drift_watchdog.launch.py")

    uvms_trajectory_gen_path = get_package_share_path('uvms_trajectory_gen')

    vehicle_name = 'klopsi00'
    use_sim_time = True
    use_hydro = True

    alpha_estimation = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            str(alpha_estimation_path / 'launch/estimation.launch.py')),
        launch_arguments={
            'vehicle_name': vehicle_name,
            'use_hydro': str(use_hydro),
            'use_sim_time': str(use_sim_time)}.items())

    alpha_force_torque_calc = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            str(alpha_model_path / 'launch/dyn_calc.launch.py')),
        launch_arguments={
            'vehicle_name': vehicle_name,
            'use_hydrodynamics': str(use_hydro),
            'base_tf_file': str(alpha_model_path / 'config/alpha_base_tf_params_bluerov.yaml'),
            'moving_base': str(True),
            'use_sim_time': str(use_sim_time)}.items())

    bluerov_acceleration_estimation = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            str(bluerov_acceleration_estimation_path / 'launch/estimation.launch.py')
        ),
        launch_arguments={
            'use_sim_time': str(use_sim_time),
            'vehicle_name': vehicle_name
        }.items()
    )

    bluerov_ctrl = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            str(bluerov_ctrl_path / 'launch/node_velocity_control.launch.py')
        ),
        launch_arguments={
            'use_sim_time': str(use_sim_time),
            'vehicle_name': vehicle_name,
            'controller_type': str(4),
            'config_file': str(bluerov_ctrl_path / 'config/ctrl_params_sim_uvms.yaml'),
        }.items()
    )

    estimation_drift_watchdog = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            estimation_watchdog_path
        ),
        launch_arguments={
            'use_sim_time': str(use_sim_time),
            'vehicle_name': vehicle_name,
        }.items()
    )

    bluerov_mixer = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            mixer_path),
        launch_arguments=dict(use_sim_time=str(use_sim_time),
                              vehicle_name=vehicle_name,
                              mixer_path=mixer_config_file_path).items()
    )

    uvms_kinematic_ctrl = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            str(uvms_kin_ctrl_path / 'launch/uvms_pap_kin_ctrl.launch.py')),
        launch_arguments={
            'vehicle_name': vehicle_name,
            'use_sim_time': str(use_sim_time)}.items()
    )

    uvms_trajectory_gen = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            str(uvms_trajectory_gen_path / 'launch/traj_pap_gen.launch.py')),
        launch_arguments={
            'vehicle_name': vehicle_name,
            'use_sim_time': str(use_sim_time)}.items()
    )

    uvms_visualization = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            str(get_package_share_path('uvms_visualization') / 'launch/visualization.launch.py')),
        launch_arguments={
            'visualization_modules': "[1, 2, 4, 5, 7, 8, 9]",
            'vehicle_name': vehicle_name,
            'use_sim_time': str(use_sim_time)}.items()
    )

    rviz = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            str(uvms_kin_ctrl_path / 'launch/rviz.launch.py')),
        launch_arguments={
            'use_sim_time': str(use_sim_time)
        }.items()
    )

    gripper = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            str(alpha_ctrl_path / 'launch/gripper_control.launch.py')),
        launch_arguments={
            'vehicle_name': vehicle_name,
            'use_sim_time': str(use_sim_time)}.items()
    )

    velocity_command = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            str(alpha_ctrl_path / 'launch/velocity_command.launch.py')),
        launch_arguments={
            'vehicle_name': vehicle_name,
            'use_sim_time': str(use_sim_time)}.items()
    )

    pick_and_place_planner = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            str(uvms_kin_ctrl_path / 'launch/planner_ctrl.launch.py')),
        launch_arguments={
            'vehicle_name': vehicle_name,}.items()
    )

    return launch.LaunchDescription([
        alpha_estimation,
        alpha_force_torque_calc,
        bluerov_acceleration_estimation,
        bluerov_ctrl,
        bluerov_mixer,
        uvms_kinematic_ctrl,
        estimation_drift_watchdog,
        uvms_trajectory_gen,
        uvms_visualization,
        rviz,
        gripper,
        velocity_command,
        pick_and_place_planner
    ])
