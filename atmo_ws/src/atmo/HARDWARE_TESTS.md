# Hardware action and connectivity tests

Build and source the workspace before every test. Action tests do not move hardware at
launch. Start with both Offboard and RL switches low, raise both, lower only RL, then
raise RL to begin the five-second test. Lower either switch to stop early.

## Connectivity

This mode creates no command publishers. Move the unpowered vehicle by hand and check
the reported message counts, ages, PX4 NED pose and velocity, tilt angle, RC gates, and
PX4 state.

```bash
ros2 launch atmo rl_control.launch.py hardware_mode:=sensor_test command_hardware:=false
```

## First rotor and kill-switch test

Remove all propellers and secure the vehicle. The lift test uses a low raw collective
baseline. Activate the physical kill switch while the motors are turning and confirm
that all four stop. Lower both gates after the test.

```bash
ros2 launch atmo rl_control.launch.py hardware_mode:=action_test \
  action_test:=lift action_sign:=positive action_magnitude:=0.1 \
  rotor_baseline:=-0.8 command_hardware:=false
```

Do not run roll, pitch, or yaw until the lift/kill test passes. After it passes, declare
that result explicitly on those launches:

```bash
ros2 launch atmo rl_control.launch.py hardware_mode:=action_test \
  action_test:=roll action_sign:=positive action_magnitude:=0.1 \
  rotor_baseline:=-0.8 kill_test_passed:=true command_hardware:=false
```

Repeat with `roll`, `pitch`, and `yaw`, and with both action signs.

## Tilt, drive, and turn

These tests never arm PX4 or request flight offboard mode. Keep
`command_hardware:=true` so the physical wheel and tilt controllers are launched.

```bash
ros2 launch atmo rl_control.launch.py hardware_mode:=action_test \
  action_test:=tilt action_sign:=positive action_magnitude:=1.0

ros2 launch atmo rl_control.launch.py hardware_mode:=action_test \
  action_test:=drive action_sign:=positive action_magnitude:=0.1

ros2 launch atmo rl_control.launch.py hardware_mode:=action_test \
  action_test:=turn action_sign:=positive action_magnitude:=0.1
```

Repeat each test with `action_sign:=negative`. Every launch records the existing PX4,
actuator, tilt, drive, and manual-override topics in its rosbag.
