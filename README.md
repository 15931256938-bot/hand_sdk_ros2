# hand_sdk_ros2

Two executables built on top of the `hand_sdk` C++ SDK:

| Binary | What it is |
|---|---|
| `hand_sdk_ros2_node` | ROS 2 node: publishes joint states, subscribes to joint/gain commands |
| `hand_sdk_app` | Qt5 GUI: step-by-step bring-up, live readings, manual pose commands |

The rest of this README covers the GUI. It does not use rclcpp, so it runs with
or without a ROS environment.

---

## `hand_sdk_ros2_node` interface

| Direction | Name | Type |
|---|---|---|
| out | `/<side>/joint_states` | `sensor_msgs/JointState` — q [rad], dq [rad/s], tau [N·m], 100 Hz |
| in | `/<side>/joint_command` | `sensor_msgs/JointState` — q, dq, tau_ff; written to the bus on arrival |
| in | `/<side>/set_gains` | `hand_sdk_ros2/SetGains` — force-mode position loop kp/kd |

`<side>` is `left` or `right`.

```bash
ros2 service call /left/set_gains hand_sdk_ros2/srv/SetGains "{kp: 3.0, kd: 0.5}"
```

Gains apply to the whole hand, and the response carries the device's per-joint
readback. Control mode and per-joint gains are **not** on ROS: mode is a bring-up
concern the node handles itself, and from ROS the hand is one unit. Both are
available to C++ callers — `Api::serviceMode()` and `Api::serviceGains()` take a
`service::JointMask` (default: every joint), so the GUI or a C++ tool can re-tune
or disable a single motor.

---

## Build

```bash
cd ~/Documents/ros2_dev_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select hand_sdk_ros2
```

## 1. Grant the binary EtherCAT permissions

EtherCAT needs a raw socket and the cyclic thread needs real-time scheduling.
Per the SDK README, capabilities go on the **final executable**:

```bash
sudo setcap cap_net_raw,cap_sys_nice+ep install/hand_sdk_ros2/lib/hand_sdk_ros2/hand_sdk_app
```

Check with `getcap <binary>`.

> colcon overwrites the installed binary on every build, which **wipes the
> capabilities**. Re-run `setcap` after each `colcon build`.

## 2. Bring the EtherCAT NIC up

The SDK's default `interface_name` is `enp132s0`. Confirm it exists and is up:

```bash
ip -br link                          # look for enp132s0
sudo ip link set enp132s0 up
```

## 3. Run it

```bash
source install/setup.bash
ros2 run hand_sdk_ros2 hand_sdk_app
```

Or directly, bypassing ROS entirely:

```bash
./install/hand_sdk_ros2/lib/hand_sdk_ros2/hand_sdk_app
```

If Qt misbehaves — typically `could not load the Qt platform plugin "xcb"` — run
it from a shell with conda deactivated. Conda envs ship their own Qt libraries
that can shadow the system Qt5 this binary links against.

## 4. Use the tabs in order

### Tab 1 — 连接 (bring-up)

Bring-up is split into one button per SDK service call, so each step can be run
and inspected on its own. Press them in order and watch each result label:

| Step | Button | SDK call | Result label |
|---|---|---|---|
| 1 | `connect` | `Api::connect()`, starts 50 Hz polling | `left=yes right=... dc=armed` |
| 2 | `set com` | `serviceComm(kStartBroadcast)` | `broadcast on` |
| 3 | `set ctrl` | `serviceMode(kForceControl, true)` | `force-ctrl n/20 enabled` |
| 4 | `mc_state` | `serviceMcState()` | `ready 20/20` |
| 5 | `set kpkd` | `serviceGains(kp, kd)` | device readback of kp/kd |

kp/kd are prefilled with `3.0` / `3.0`. Every call targets whichever hands
`connect()` reported. Pressing `connect` again disconnects.

### Tab 2 — 播放

Not implemented.

### Tab 3 — 调试界面 (debug)

- Live readings at 50 Hz: `readpos0-19` (deg), `readvel0-19` (deg/s),
  `readtau0-19` (N·m), and `error0-19` driven from `HandState::error_bits`.
- `发送调试指令` reads `command_pos0-19` as **degrees**, converts to radians, and
  sends them to all present hands.
- `回到零位` resets the fields to `0.0` and sends a zero-position command.
- `当前json` dumps the current fields as a pose, for pasting into a task list:

```json
{
    "position": [0, 0, ...],
    "unit": "deg"
}
```

> **Safety**: the two send buttons command the hands to hold those positions
> immediately after control is enabled in step 3. Keep the hands free to move on
> a first run.

## Shutdown

The window X and the 关闭 button both call `Api::disconnect()`, as does the
destructor. There is no separate close/end call in the SDK — `disconnect()` is
it, and it is safe if triggered twice.

## If connect fails

| Message | Cause |
|---|---|
| `ecx_init failed: could not open interface` | missing `cap_net_raw`, or `enp132s0` is down |
| no hands reported | hand unpowered, or cabled to a different NIC |

---

## Source layout

| File | Contents |
|---|---|
| `src/hand_sdk_app.hpp` | `HandAppWindow` declaration, grouped by tab |
| `src/hand_sdk_app.cpp` | definitions plus `main()` |
| `src/hand_sdk_app_ui.cpp` | uic output; a header despite the extension, so it is `#include`d rather than compiled separately |
| `src/hand_sdk_ros2_node.cpp` | the ROS 2 node |

Known limitations: the GUI uses `hand_sdk::Config{}` defaults (interface
`enp132s0`), and the readings show the left hand when present, since the UI has
no hand selector.
