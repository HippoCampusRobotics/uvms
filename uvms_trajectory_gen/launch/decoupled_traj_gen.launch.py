from ament_index_python.packages import get_package_share_path
import launch
import launch_ros


def generate_launch_description():
    bluerov_traj_gen_path = get_package_share_path('bluerov_trajectory_gen')
    alpha_traj_gen_path = get_package_share_path('alpha_trajectory_gen')


    vehicle_name = 'klopsi00'
    use_sim_time = False
    use_joint_space_traj = True

    eef_traj_gen = launch.actions.GroupAction(condition=launch.conditions.UnlessCondition(str(use_joint_space_traj)),
                                              actions=[
                                                  launch.actions.IncludeLaunchDescription(
                                                      launch.launch_description_sources.PythonLaunchDescriptionSource(
                                                          str(alpha_traj_gen_path/ 'launch/eef_traj_gen_coupled.launch.py')),
                                                      launch_arguments = {
                                                          'trajectory_type': str(1),
                                                          'vehicle_name': vehicle_name,
                                                          'use_sim_time' : str(use_sim_time)}.items())
                                              ])

    joint_traj_gen = launch.actions.GroupAction(condition=launch.conditions.IfCondition(str(use_joint_space_traj)),
                                                actions=[
                                                    launch.actions.IncludeLaunchDescription(
                                                        launch.launch_description_sources.PythonLaunchDescriptionSource(
                                                            str(alpha_traj_gen_path/ 'launch/joint_traj_gen_coupled.launch.py')),
                                                        launch_arguments = {
                                                            'trajectory_type': str(2),
                                                            'vehicle_name': vehicle_name,
                                                            'use_sim_time' : str(use_sim_time)}.items())
                                                ])

    alpha_traj_gen = [eef_traj_gen, joint_traj_gen]


    bluerov_traj_gen = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            str(bluerov_traj_gen_path / "launch/node_trajectory_gen.launch.py")),
        launch_arguments={
            'trajectory_type' : str(5),
            'vehicle_name': vehicle_name,
            'use_sim_time' : str(use_sim_time)}.items())

    return launch.LaunchDescription([
        *alpha_traj_gen,
        bluerov_traj_gen,
    ])