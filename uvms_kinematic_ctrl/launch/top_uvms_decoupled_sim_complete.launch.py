import launch
from ament_index_python.packages import get_package_share_path


def generate_launch_description():
    alpha_model_path = get_package_share_path('alpha_model')
    alpha_estimation_path = get_package_share_path('alpha_estimation')
    bluerov_ctrl_path = get_package_share_path('bluerov_ctrl')
    bluerov_low_level_ctrl_path = get_package_share_path('hippo_control')
    bluerov_acceleration_estimation_path = get_package_share_path(
        'bluerov_estimation'
    )
    mixer_path = str(
        bluerov_low_level_ctrl_path
        / 'launch/node_actuator_mixer_bluerov.launch.py'
    )
    mixer_config_file_path = str(
        bluerov_low_level_ctrl_path
        / ('config/actuator_mixer_bluerov_advanced.yaml')
    )
    estimation_watchdog_path = str(
        bluerov_ctrl_path / 'launch/node_estimation_drift_watchdog.launch.py'
    )
    uvms_kin_ctrl_path = get_package_share_path('uvms_kinematic_ctrl')
    bluerov_traj_gen_path = get_package_share_path('bluerov_trajectory_gen')
    alpha_traj_gen_path = get_package_share_path('alpha_trajectory_gen')
    alpha_ctrl_path = get_package_share_path('alpha_ctrl')

    vehicle_name = 'klopsi00'
    use_sim_time = True
    use_hydro = True
    use_joint_space_traj = True

    eef_traj_gen = launch.actions.GroupAction(
        condition=launch.conditions.UnlessCondition(str(use_joint_space_traj)),
        actions=[
            launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.PythonLaunchDescriptionSource(
                    str(
                        alpha_traj_gen_path
                        / 'launch/eef_traj_gen_coupled.launch.py'
                    )
                ),
                launch_arguments={
                    'trajectory_type': str(1),
                    'vehicle_name': vehicle_name,
                    'use_sim_time': str(use_sim_time),
                }.items(),
            )
        ],
    )

    joint_traj_gen = launch.actions.GroupAction(
        condition=launch.conditions.IfCondition(str(use_joint_space_traj)),
        actions=[
            launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.PythonLaunchDescriptionSource(
                    str(
                        alpha_traj_gen_path
                        / 'launch/joint_traj_gen_coupled.launch.py'
                    )
                ),
                launch_arguments={
                    'trajectory_type': str(2),
                    'vehicle_name': vehicle_name,
                    'use_sim_time': str(use_sim_time),
                }.items(),
            )
        ],
    )

    alpha_traj_gen = [eef_traj_gen, joint_traj_gen]

    alpha_eef_control = launch.actions.GroupAction(
        condition=launch.conditions.UnlessCondition(str(use_joint_space_traj)),
        actions=[
            launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.PythonLaunchDescriptionSource(
                    str(alpha_ctrl_path / 'launch/eef_control.launch.py')
                ),
                launch_arguments={
                    'vehicle_name': vehicle_name,
                    'is_sim': str(use_sim_time),
                }.items(),
            )
        ],
    )

    alpha_joint_control = launch.actions.GroupAction(
        condition=launch.conditions.IfCondition(str(use_joint_space_traj)),
        actions=[
            launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.PythonLaunchDescriptionSource(
                    str(alpha_ctrl_path / 'launch/joint_control.launch.py')
                ),
                launch_arguments={
                    'vehicle_name': vehicle_name,
                    'is_sim': str(use_sim_time),
                }.items(),
            )
        ],
    )
    alpha_control = [alpha_eef_control, alpha_joint_control]

    bluerov_traj_gen = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            str(bluerov_traj_gen_path / 'launch/node_trajectory_gen.launch.py')
        ),
        launch_arguments={
            'trajectory_type': str(5),
            'vehicle_name': vehicle_name,
            'use_sim_time': str(use_sim_time),
        }.items(),
    )

    alpha_estimation = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            str(alpha_estimation_path / 'launch/estimation.launch.py')
        ),
        launch_arguments={
            'vehicle_name': vehicle_name,
            'use_hydro': str(use_hydro),
            'use_sim_time': str(use_sim_time),
        }.items(),
    )

    alpha_force_torque_calc = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            str(alpha_model_path / 'launch/dyn_calc.launch.py')
        ),
        launch_arguments={
            'vehicle_name': vehicle_name,
            'use_hydrodynamics': str(use_hydro),
            'base_tf_file': str(
                alpha_model_path / 'config/alpha_base_tf_params_bluerov.yaml'
            ),
            'moving_base': str(True),
            'use_sim_time': str(use_sim_time),
        }.items(),
    )

    bluerov_acceleration_estimation = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            str(
                bluerov_acceleration_estimation_path
                / 'launch/estimation.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': str(use_sim_time),
            'vehicle_name': vehicle_name,
        }.items(),
    )

    bluerov_pose_ctrl = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            str(bluerov_ctrl_path / 'launch/node_pose_control_module.launch.py')
        ),
        launch_arguments={
            'use_sim_time': str(use_sim_time),
            'vehicle_name': vehicle_name,
            'config_file': str(
                bluerov_ctrl_path / 'config/ctrl_params_sim.yaml'
            ),
        }.items(),
    )

    bluerov_velocity_ctrl = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            str(bluerov_ctrl_path / 'launch/node_velocity_control.launch.py')
        ),
        launch_arguments={
            'use_sim_time': str(use_sim_time),
            'vehicle_name': vehicle_name,
            'controller_type': str(4),
            'config_file': str(
                bluerov_ctrl_path / 'config/ctrl_params_sim.yaml'
            ),
        }.items(),
    )

    estimation_drift_watchdog = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            estimation_watchdog_path
        ),
        launch_arguments={
            'use_sim_time': str(use_sim_time),
            'vehicle_name': vehicle_name,
        }.items(),
    )

    bluerov_mixer = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            mixer_path
        ),
        launch_arguments={
            'use_sim_time': str(use_sim_time),
            'vehicle_name': vehicle_name,
            'mixer_path': mixer_config_file_path,
        }.items(),
    )

    uvms_visualization = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            str(
                get_package_share_path('uvms_visualization')
                / 'launch/visualization.launch.py'
            )
        ),
        launch_arguments={
            'visualization_modules': '[2, 3, 4, 5, 6, 9]',
            'vehicle_name': vehicle_name,
            'use_sim_time': str(use_sim_time),
        }.items(),
    )

    rviz = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            str(uvms_kin_ctrl_path / 'launch/rviz.launch.py')
        ),
        launch_arguments={'use_sim_time': str(use_sim_time)}.items(),
    )

    return launch.LaunchDescription(
        [
            *alpha_traj_gen,
            *alpha_control,
            bluerov_traj_gen,
            alpha_estimation,
            alpha_force_torque_calc,
            bluerov_acceleration_estimation,
            bluerov_pose_ctrl,
            bluerov_velocity_ctrl,
            bluerov_mixer,
            estimation_drift_watchdog,
            uvms_visualization,
            rviz,
        ]
    )
