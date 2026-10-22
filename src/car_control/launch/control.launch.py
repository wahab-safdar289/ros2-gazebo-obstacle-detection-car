import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
    TimerAction,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    # Resolve the launch arguments NOW, into plain Python values.
    # Actions started later by a TimerAction run outside this file's argument scope,
    # so they must not use LaunchConfiguration themselves.
    params_file = LaunchConfiguration('params_file').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).strip().lower() in (
        'true', '1', 'yes')

    if not os.path.isfile(params_file):
        raise RuntimeError(f'[control] params file not found: {params_file}')

    safety_gate = Node(
        package='car_control',
        executable='safety_gate',
        name='safety_gate',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        output='screen',
        emulate_tty=True,
    )

    obstacle_avoider = Node(
        package='car_control',
        executable='obstacle_avoider',
        name='obstacle_avoider',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        output='screen',
        emulate_tty=True,
    )

    # 1. safety_gate starts first.
    # 2. Once its process is up, wait 1 s so it is already subscribed and publishing zeros.
    # 3. Only then start obstacle_avoider, so nothing ever produces motion commands
    #    while the safety layer is missing.
    start_avoider_after_gate = RegisterEventHandler(
        OnProcessStart(
            target_action=safety_gate,
            on_start=[
                LogInfo(msg='[control] safety_gate is up -> starting obstacle_avoider in 1 s'),
                TimerAction(period=1.0, actions=[obstacle_avoider]),
            ],
        )
    )

    # If the safety gate dies, do not keep driving without it: shut everything down.
    shutdown_if_gate_dies = RegisterEventHandler(
        OnProcessExit(
            target_action=safety_gate,
            on_exit=[
                LogInfo(msg='[control] safety_gate exited -> shutting down all nodes'),
                Shutdown(reason='safety_gate exited'),
            ],
        )
    )

    # A dead avoider means the car is no longer driving itself: stop the whole launch too.
    shutdown_if_avoider_dies = RegisterEventHandler(
        OnProcessExit(
            target_action=obstacle_avoider,
            on_exit=[
                LogInfo(msg='[control] obstacle_avoider exited -> shutting down all nodes'),
                Shutdown(reason='obstacle_avoider exited'),
            ],
        )
    )

    return [
        LogInfo(msg='[control] starting safety_gate first'),
        start_avoider_after_gate,
        shutdown_if_gate_dies,
        shutdown_if_avoider_dies,
        safety_gate,
    ]


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('car_control'), 'config', 'car_control_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the Gazebo clock (/clock) instead of the wall clock'),
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='YAML file with the safety_gate and obstacle_avoider parameters'),
        OpaqueFunction(function=launch_setup),
    ])
