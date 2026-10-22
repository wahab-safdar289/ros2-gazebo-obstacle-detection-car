import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    Shutdown,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


WORLD_NAME = 'obstacle_world'   # must match <world name="..."> in obstacle_world.sdf
ROBOT_NAME = 'car'


def after_success(target, next_actions, step_name, next_name):
    """Run next_actions only if `target` exited with code 0; otherwise shut everything down."""

    def _callback(event, context):
        if event.returncode == 0:
            return [LogInfo(msg=f'[bringup] {step_name}: OK -> starting {next_name}')] + next_actions
        return [
            LogInfo(msg=f'[bringup] {step_name}: FAILED (exit code {event.returncode}). '
                        f'{next_name} will NOT be started. Shutting down.'),
            Shutdown(reason=f'{step_name} failed'),
        ]

    return RegisterEventHandler(OnProcessExit(target_action=target, on_exit=_callback))


def generate_launch_description():
    car_description_dir = get_package_share_directory('car_description')
    car_gazebo_dir = get_package_share_directory('car_gazebo')
    car_control_dir = get_package_share_directory('car_control')
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')

    xacro_file = os.path.join(car_description_dir, 'urdf', 'car.urdf.xacro')
    bridge_config = os.path.join(car_description_dir, 'config', 'gz_bridge.yaml')
    controllers_yaml = os.path.join(car_description_dir, 'config', 'controllers.yaml')
    rviz_config = os.path.join(car_description_dir, 'rviz', 'car.rviz')
    default_world = os.path.join(car_gazebo_dir, 'worlds', 'obstacle_world.sdf')

    # ---------------- Launch arguments ----------------
    args = [
        DeclareLaunchArgument('world', default_value=default_world,
                              description='Path to the SDF world file'),
        DeclareLaunchArgument('x', default_value='0.0', description='Spawn x [m]'),
        DeclareLaunchArgument('y', default_value='0.0', description='Spawn y [m]'),
        DeclareLaunchArgument('z', default_value='0.05', description='Spawn z [m]'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn yaw [rad]'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz'),
        DeclareLaunchArgument('start_control', default_value='true',
                              description='Start safety_gate + obstacle_avoider. '
                                          'Set false to test the car without autonomy.'),
        DeclareLaunchArgument('gz_flags', default_value='-r -v 2',
                              description='Flags for gz sim, e.g. "-r -s -v 2" for no GUI'),
    ]

    # ---------------- Step 1: Gazebo with the obstacle world ----------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': [LaunchConfiguration('gz_flags'), TextSubstitution(text=' '),
                        LaunchConfiguration('world')],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # ---------------- Step 2: clock + sensor bridge ----------------
    # Started early so /clock exists before any use_sim_time node needs it.
    # Sensor topics simply appear once the robot is spawned.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config, 'use_sim_time': True}],
        output='screen',
    )

    # ---------------- Step 3: robot_state_publisher ----------------
    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', xacro_file]),
        value_type=str,
    )
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen',
    )

    # ---------------- Step 4: spawn the car in Gazebo ----------------
    spawn_car = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', WORLD_NAME,
            '-name', ROBOT_NAME,
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-Y', LaunchConfiguration('yaw'),
        ],
        output='screen',
    )
    # Small delay so Gazebo's world is up and robot_description has been published.
    delayed_spawn = TimerAction(period=4.0, actions=[spawn_car])

    # ---------------- Step 5: joint_state_broadcaster ----------------
    # Long timeouts: the controller manager lives inside Gazebo and answers slowly
    # while Gazebo is busy starting its sensors (software rendering in WSL).
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '120',
                   '--service-call-timeout', '120',
                   '--switch-timeout', '120'],
        output='screen',
    )

    # ---------------- Step 6: diff_drive_controller ----------------
    # --param-file hands the controller its parameters explicitly (wheel_separation etc.)
    diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller',
                   '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '120',
                   '--service-call-timeout', '120',
                   '--switch-timeout', '120',
                   '--param-file', controllers_yaml],
        output='screen',
    )

    # ---------------- Step 7: RViz + safety gate / obstacle avoider ----------------
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )

    # control.launch.py starts safety_gate first, then obstacle_avoider 1 s later
    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(car_control_dir, 'launch', 'control.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items(),
        condition=IfCondition(LaunchConfiguration('start_control')),
    )
    # Brief pause after the controller is active before anything can command it.
    delayed_control = TimerAction(period=1.0, actions=[control])

    # ---------------- Sequencing (each step starts only if the previous one succeeded) ----------------
    spawn_done = after_success(
        spawn_car, [joint_state_broadcaster],
        'spawn car', 'joint_state_broadcaster')

    jsb_done = after_success(
        joint_state_broadcaster, [diff_drive_controller],
        'joint_state_broadcaster', 'diff_drive_controller')

    ddc_done = after_success(
        diff_drive_controller, [rviz, delayed_control],
        'diff_drive_controller', 'RViz and control nodes')

    return LaunchDescription(args + [
        LogInfo(msg='[bringup] Step 1-3: Gazebo, bridge, robot_state_publisher'),
        gazebo,
        bridge,
        robot_state_publisher,
        LogInfo(msg='[bringup] Step 4: spawning car in 4 s'),
        delayed_spawn,
        spawn_done,
        jsb_done,
        ddc_done,
    ])
