from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import AndSubstitution, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    command_hardware = LaunchConfiguration("command_hardware")
    drive_hardware = LaunchConfiguration("drive_hardware")
    route = LaunchConfiguration("route")
    hardware_mode = LaunchConfiguration("hardware_mode")
    action_test = LaunchConfiguration("action_test")
    action_sign = LaunchConfiguration("action_sign")
    action_magnitude = LaunchConfiguration("action_magnitude")
    rotor_baseline = LaunchConfiguration("rotor_baseline")
    action_test_duration = LaunchConfiguration("action_test_duration")
    kill_test_passed = LaunchConfiguration("kill_test_passed")
    record = LaunchConfiguration("record")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "record",
                default_value="true",
                description=(
                    "Record a rosbag from this launch. Set false when the session "
                    "script runs its own recorder, so the bag is not written twice "
                    "and can be SIGINTed and waited for on shutdown."
                ),
            ),
            DeclareLaunchArgument(
                "route",
                default_value="landing",
                description="Combined policy route: takeoff or landing",
            ),
            DeclareLaunchArgument(
                "hardware_mode",
                default_value="policy",
                description="policy, ground, shadow, sensor_test, or action_test",
            ),
            DeclareLaunchArgument(
                "action_test",
                default_value="lift",
                description="lift, roll, pitch, yaw, tilt, drive, or turn",
            ),
            DeclareLaunchArgument("action_sign", default_value="positive"),
            DeclareLaunchArgument("action_magnitude", default_value="0.1"),
            DeclareLaunchArgument("rotor_baseline", default_value="-0.8"),
            DeclareLaunchArgument("action_test_duration", default_value="5.0"),
            DeclareLaunchArgument(
                "kill_test_passed",
                default_value="false",
                description="Required for roll, pitch, and yaw after the lift/kill test passes",
            ),
            DeclareLaunchArgument(
                "command_hardware",
                default_value="true",
                description="Launch the physical tilt and drive command nodes",
            ),
            DeclareLaunchArgument(
                "drive_hardware",
                default_value="false",
                description=(
                    "Launch the drive (wheel) node. DEFAULT FALSE since "
                    "2026-08-17: the drive RoboClaw is dead (regen through the "
                    "12V regulator -- see the README). Set true only "
                    "after a replacement board is installed BEHIND the battery "
                    "bypass diode."
                ),
            ),
            Node(
                package="atmo",
                executable="rl_controller_hardware",
                name="rl_controller_hardware",
                output="screen",
                additional_env={
                    "ATMO_RL_ROUTE": route,
                    "ATMO_RL_HARDWARE_MODE": hardware_mode,
                    "ATMO_RL_ACTION_TEST": action_test,
                    "ATMO_RL_ACTION_SIGN": action_sign,
                    "ATMO_RL_ACTION_MAGNITUDE": action_magnitude,
                    "ATMO_RL_ROTOR_BASELINE": rotor_baseline,
                    "ATMO_RL_ACTION_TEST_DURATION": action_test_duration,
                    "ATMO_RL_KILL_TEST_PASSED": kill_test_passed,
                },
            ),
            Node(
                package="atmo",
                executable="tilt_controller_hardware",
                name="tilt_controller_hardware",
                output="log",
                condition=IfCondition(command_hardware),
            ),
            Node(
                package="atmo",
                executable="drive_controller_hardware",
                name="drive_controller_hardware",
                output="log",
                # Both switches must be true: the profile wants command nodes AND
                # the drive board is declared alive (see the drive_hardware arg).
                condition=IfCondition(AndSubstitution(command_hardware, drive_hardware)),
            ),
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "bag",
                    "record",
                    "/fmu/in/actuator_motors",
                    "/fmu/in/vehicle_visual_odometry",
                    "/fmu/out/battery_status_v1",
                    "/fmu/out/estimator_status_flags",
                    "/fmu/out/failsafe_flags",
                    "/fmu/out/input_rc",
                    "/fmu/out/vehicle_command_ack",
                    "/fmu/out/vehicle_control_mode",
                    "/fmu/out/vehicle_odometry",
                    "/fmu/out/vehicle_status_v1",
                    "/tilt_vel",
                    "/drive_vel",
                    "/atmo/rl/manual_override",
                    "/atmo/groundtruth_odom",
                    "/atmo/rl/policy_action",
                    "/atmo/rl/observation_state",
                ],
                output="screen",
                condition=IfCondition(record),
            ),
        ]
    )
