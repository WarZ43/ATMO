from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='atmo',
            executable='rl_controller_sim',
            name='rl_controller_sim',
            output='screen',
        ),
        ExecuteProcess(
            cmd=[
                'ros2',
                'bag',
                'record',
                '/fmu/in/actuator_motors',
                '/fmu/out/vehicle_local_position_groundtruth',
                '/fmu/out/vehicle_attitude_groundtruth',
                '/fmu/out/vehicle_angular_velocity_groundtruth',
                '/atmo/rl/policy_action',
            ],
            output='screen',
        ),
    ])
