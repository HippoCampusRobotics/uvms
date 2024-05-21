from ament_index_python.packages import get_package_share_path
from hippo_common import launch_helper
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def declare_launch_args(launch_description: LaunchDescription):
    launch_helper.declare_vehicle_name_and_sim_time(
        launch_description=launch_description,
        use_sim_time_default='true',
    )

    action = DeclareLaunchArgument('simulate_kinematics', default_value='false')
    launch_description.add_action(action)


def include_gazebo():
    pkg_path = get_package_share_path('hippo_sim')
    path = str(pkg_path / 'launch/start_gazebo.launch.py')
    source = PythonLaunchDescriptionSource(path)
    return IncludeLaunchDescription(source)


def include_spawn_uvms():
    pkg_path = get_package_share_path('uvms_sim')
    path = str(pkg_path / 'launch/spawn_uvms.launch.py')
    launch_args = launch_helper.LaunchArgsDict()
    launch_args.add_vehicle_name_and_sim_time()
    launch_args.add('simulate_kinematics')
    source = PythonLaunchDescriptionSource(path)
    return IncludeLaunchDescription(
        source, launch_arguments=launch_args.items()
    )


def include_alpha_sim_interface():
    pkg_path = get_package_share_path('alpha_ctrl')
    path = str(pkg_path / 'launch/simulation_velocity_control.launch.py')
    source = PythonLaunchDescriptionSource(path)

    args = launch_helper.LaunchArgsDict()
    args.add_vehicle_name_and_sim_time()
    # TODO: verify/clarify that/why we do not use_sim_time here
    args['is_sim'] = args['use_sim_time']
    args['use_hydro'] = 'true'
    args['update_base_ref'] = 'true'
    tf_file = str(
        get_package_share_path('alpha_model')
        / 'config/alpha_base_tf_params_bluerov.yaml'
    )
    args['base_tf_file'] = tf_file

    return IncludeLaunchDescription(source, launch_arguments=args.items())


def add_bluerov_state_estimator():
    args = launch_helper.LaunchArgsDict()
    args.add_vehicle_name_and_sim_time()
    return Node(
        package='hippo_sim',
        executable='fake_state_estimator',
        namespace=args['vehicle_name'],
        parameters=[args],
        name='bluerov_state_estimator',
        output='screen',
    )


def generate_launch_description():
    launch_description = LaunchDescription()
    declare_launch_args(launch_description=launch_description)

    actions = [
        include_gazebo(),
        include_spawn_uvms(),
        include_alpha_sim_interface(),
        add_bluerov_state_estimator(),
    ]

    for action in actions:
        launch_description.add_action(action)

    return launch_description
