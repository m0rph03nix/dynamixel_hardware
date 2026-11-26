# dynamixel_hardware

A [`ros2_control`](https://github.com/ros-controls/ros2_control) `SystemInterface` plugin for [ROBOTIS Dynamixel](https://emanual.robotis.com/docs/en/dxl/) servos.
Supports Protocol 1.0 devices (AX-12A, AX-18A, RX/MX series) and Protocol 2.0, position and velocity control, runtime mode switching, and a stub mode for testing without hardware.

## Supported hardware

Any servo supported by [`dynamixel_workbench_toolbox`](https://github.com/ROBOTIS-GIT/dynamixel-workbench). Protocol 1.0 velocity control uses wheel mode via `Moving_Speed` rather than `Goal_Velocity`.

| Series | Protocol | Position | Velocity |
|--------|----------|----------|----------|
| AX-12A / AX-18A | 1.0 | 0–300° | wheel mode |
| RX / MX (P1 mode) | 1.0 | yes | wheel mode |
| MX / XM / XH (P2 mode) | 2.0 | yes | `Goal_Velocity` |

On AX-12A, switching to velocity control sets `CW_Angle_Limit` and `CCW_Angle_Limit` to `0` (wheel mode). Position control is unavailable while in that mode. The plugin switches modes automatically when the active controller changes.

## Setup

```bash
cd <your_ws>
git clone https://github.com/m0rph03nix/dynamixel_hardware.git src/dynamixel_hardware
vcs import src < src/dynamixel_hardware/dynamixel_control.repos
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Configuration

### Hardware-level parameters

```xml
<ros2_control name="my_robot" type="system">
  <hardware>
    <plugin>dynamixel_hardware/DynamixelHardware</plugin>
    <param name="usb_port">/dev/ttyUSB0</param>
    <param name="baud_rate">1000000</param>  <!-- AX-12A factory default -->
    <!-- <param name="use_stub">true</param> -->
  </hardware>
  ...
</ros2_control>
```

`use_stub: true` disables all serial communication — commands are echoed back as states. Useful for simulation-free integration testing.

### Wheel mode on Protocol 1.0 servos (AX, RX, MX)

In wheel mode, Protocol 1.0 servos have several limitations that the plugin corrects, so that the `velocity` interface really behaves as a velocity:

- **`Moving_Speed` sets a motor power, not a speed.** The actual speed depends on friction and load: on an AX-12A, a command of 0.86 rad/s gave about 0.3 rad/s. The plugin closes the loop with a PI controller on the measured speed (output = command + Kp·error + integral). The integral is reset when the command is zero.
- **`Present_Speed` overestimates the real speed** by 5 to 9 % on the AX-12A (measured against the time of a full wheel turn). The speed is therefore computed from the position sensor and filtered over about 0.1 s.
- **The position sensor covers 300° out of 360°.** In the remaining 60°, the position reads 0, 1023 or a frozen value that jitters by a few steps, and the speed reading is wrong. The plugin detects the dead zone (out-of-range, jumping or frozen position, implausible speed), uses the command as speed estimate while the wheel crosses it (0 when stopped), and leaves it only when the wheel progresses again from the expected edge.
- **The raw position wraps at each turn.** The wheel position is continuous and relative: it starts at 0 when entering wheel mode, as in simulation, so `diff_drive_controller` does not count the absolute sensor angle as a motion. Outside the dead zone, it follows the sensor exactly; after a dead zone, it is resynchronised on the sensor, modulo one turn, using the turn closest to the estimate. The sensor is trusted only once the wheel has been seen moving, since a wheel stopped in the dead zone may read a wrong value.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `velocity_pi` | `true` | `false` to send the command as is (open loop) |
| `velocity_kp` | `1.0` | Proportional gain |
| `velocity_ki` | `4.0` | Integral gain (1/s) |
| `velocity_integral_limit` | `3.0` | Bound of the integral term (rad/s) |

Measured on a simplebot with two AX-12A at 12 V, starting with the wheels at any position, including inside the dead zone: 0.113 to 0.117 m for 0.12 m commanded in a straight line (heading error below 0.03 rad), and 9.74 to 9.95 rad for 10 rad of rotation in place.

### Joint parameters

| Parameter | Description |
|-----------|-------------|
| `id` | Dynamixel ID (required) |
| `invert` | `true` to negate command and state signs — for motors mounted in mirror |
| any control-table name | Written to the servo at init via `itemWrite` (e.g. `Return_Delay_Time`) |

### Interfaces

Each joint exposes `position` and `velocity` command interfaces, and `position`, `velocity`, `effort` state interfaces. Only one command type can be active at a time — the plugin switches the servo's control mode accordingly.

## Examples

### Differential drive — AX-12A

The right motor faces the opposite direction — its positive rotation moves the wheel backward. `invert: true` corrects for this mounting convention at the hardware level.

```xml
<ros2_control name="DynamixelHardware" type="system">
  <hardware>
    <plugin>dynamixel_hardware/DynamixelHardware</plugin>
    <param name="usb_port">/dev/ttyACM0</param>
    <param name="baud_rate">1000000</param>
  </hardware>

  <joint name="left_wheel_joint">
    <param name="id">1</param>
    <command_interface name="velocity"/>
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>

  <joint name="right_wheel_joint">
    <param name="id">2</param>
    <param name="invert">true</param>
    <command_interface name="velocity"/>
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
</ros2_control>
```

controllers.yaml:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 30
    diff_cont:
      type: diff_drive_controller/DiffDriveController
    joint_broad:
      type: joint_state_broadcaster/JointStateBroadcaster

diff_cont:
  ros__parameters:
    left_wheel_names:  ['left_wheel_joint']
    right_wheel_names: ['right_wheel_joint']
    wheel_separation: 0.184
    wheel_radius: 0.033
```

ROS 2 Jazzy publishes `TwistStamped` on `cmd_vel` by default:

```bash
ros2 topic pub /diff_cont/cmd_vel geometry_msgs/msg/TwistStamped \
  '{header: {stamp: {sec: 0}}, twist: {linear: {x: 0.1}, angular: {z: 0.0}}}'
```

### Multi-joint position control

```xml
<ros2_control name="DynamixelHardware" type="system">
  <hardware>
    <plugin>dynamixel_hardware/DynamixelHardware</plugin>
    <param name="usb_port">/dev/ttyUSB0</param>
    <param name="baud_rate">57600</param>
  </hardware>

  <joint name="joint1">
    <param name="id">1</param>
    <command_interface name="position"/>
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>

  <joint name="joint2">
    <param name="id">2</param>
    <command_interface name="position"/>
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
</ros2_control>
```

```bash
ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "{
    trajectory: {
      joint_names: [joint1, joint2],
      points: [
        { positions: [0.5, -0.5], time_from_start: { sec: 2 } },
        { positions: [0.0,  0.0], time_from_start: { sec: 4 } }
      ]
    }
  }"
```

### Stub mode via xacro argument

```xml
<xacro:arg name="use_stub" default="false"/>
<param name="use_stub">$(arg use_stub)</param>
```

```bash
ros2 launch my_bringup bringup.launch.py use_stub:=true
```

## Diagnostics

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 control list_hardware_components
```


