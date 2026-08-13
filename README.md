# hand_sdk_ros2

Two independent executables built on top of the `hand_sdk` C++ SDK. They do not
talk to each other, and only one of them can hold the bus at a time — run one or
the other, never both.

| Binary | What it is | ROS at runtime |
|---|---|---|
| [`hand_sdk_ros2_node`](#hand_sdk_ros2_node--the-ros-2-node) | ROS 2 node: publishes joint states, takes joint/gain commands, carries a diagnostic board | **yes** — links rclcpp |
| [`hand_sdk_app`](#hand_sdk_app--the-standalone-qt-gui) | Qt5 GUI: step-by-step bring-up, live readings, manual pose commands | **no** — links only Qt5 and the SDK |

The app is ROS-free by construction: no source file in it includes rclcpp, and
`ldd` on the built binary shows zero ROS libraries. It lives in this package
because it shares the SDK and the `.ui` tooling, and it is *built* with colcon
like everything else here — but nothing about running it needs a ROS
environment, a daemon, or a sourced overlay.

Instructions are in three parts: **[common setup](#common-setup)** applies to
both, then one section per binary. Start with the common setup either way.

---

# Common setup

## 1. Build

```bash
cd ~/Documents/ros2_dev_ws                  # the workspace root, always
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to hand_sdk_ros2
```

`--packages-up-to` rather than `--packages-select`, so that `hand_sdk_cpp` is
rebuilt **and reinstalled** first. Both binaries compile against the *installed*
SDK headers under `install/hand_sdk_cpp/include/`, so selecting only this package
after an SDK edit compiles the new sources against the old headers — which shows
up as "no member named ..." for something you can plainly see in the file, or
worse, as a silent ODR mismatch.

Run it from the workspace root. colcon writes `build/`, `install/` and `log/`
into whatever directory it is invoked from, so running it inside a package
scatters a second workspace there.

## 2. Grant EtherCAT permissions

EtherCAT needs a raw socket and the cyclic thread needs real-time scheduling.
Per the SDK README, capabilities go on the **final executable** — both of them:

```bash
cd ~/Documents/ros2_dev_ws
sudo setcap cap_net_raw,cap_sys_nice+ep install/hand_sdk_ros2/lib/hand_sdk_ros2/hand_sdk_ros2_node
sudo setcap cap_net_raw,cap_sys_nice+ep install/hand_sdk_ros2/lib/hand_sdk_ros2/hand_sdk_app
```

Check with `getcap <binary>`.

> colcon overwrites the installed binaries on every build, which **wipes the
> capabilities**. Re-run both `setcap` lines after each `colcon build`.

## 3. Bring the EtherCAT NIC up

The SDK's default `interface_name` is `enp132s0`. Confirm it exists and is up:

```bash
ip -br link                          # look for enp132s0
sudo ip link set enp132s0 up
```

## If connecting fails

| Message | Cause |
|---|---|
| `ecx_init failed: could not open interface` | missing `cap_net_raw`, or `enp132s0` is down |
| no hands reported | hand unpowered, or cabled to a different NIC |
| `could not load the Qt platform plugin "xcb"` | conda's Qt shadowing the system Qt5 — run from a shell with conda deactivated |

---

# `hand_sdk_ros2_node` — the ROS 2 node

## Run

```bash
cd ~/Documents/ros2_dev_ws
source install/setup.bash
ros2 run hand_sdk_ros2 hand_sdk_ros2_node                              # board comes up
ros2 run hand_sdk_ros2 hand_sdk_ros2_node --ros-args -p board:=false   # headless
```

Bring-up runs inside the node's constructor — mc-state wait, start broadcast,
seed the hold pose from the measured state, enable force control, apply
`initial_kp`/`initial_kd`. Those lines go to the **ROS console only**; the
board's log pane starts afterwards and shows only what you do from the board.

Parameters: `interface_name`, `left_hand_name`, `right_hand_name`,
`publish_rate_hz` (100), `initial_kp` / `initial_kd` (3.0), `bringup_timeout_sec`
(5.0), `board` (true).

Qt owns the main thread and ROS spins beside it. Closing the board window stops
the node; Ctrl-C stops both.

## Topics and services

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
readback. `success` is the SDK's two-step verdict, not just "the message went
out": it is false unless every joint came back running the requested gains, and
`message` then names the joints that did not and what they returned instead. A
call that reports false is worth reading rather than retrying blind.

Control mode and per-joint gains are **not** on ROS: mode is a bring-up concern
the node handles itself, and from ROS the hand is one unit. Both are reachable
from C++ — `Api::serviceMode()` and `Api::serviceGains()` take a
`service::JointMask` (default: every joint) — and from the board below, which is
where you go to disable one motor.

## The board

The node brings up a diagnostic window with it. It is not a second front end for
driving the hand: it is there to see what is being published and to take a
faulted motor out of control.

**states tab** — a 5×4 grid per hand, showing whichever kind of state you last
asked for (`render_state_type` says which):

| Button | Sends | Shows |
|---|---|---|
| `button_mc_state` | cmd 12, read-only | `standby` / `active` / `FAULT`, with decoded fault bits, red if faulted |
| `button_ctrl_state` | cmd 11 with the ignore word in every slot — asks without writing | `1` enabled / `0` disabled, red when 0 |
| `ctrl_cmd_enable` / `ctrl_cmd_disable` | cmd 11 for the motor picked by `ctrl_finger_index` + `ctrl_joint_index` | nothing in the grid — a one-line verdict in `ctrl_cmd_result`, green ok / red failed |

The loop mode is not in the cells — everything is force control unless someone
is debugging — so a joint reporting three-loop is called out in the log instead.

The two write buttons leave the grid alone on purpose: it shows the state you
last *asked* for, and a write silently repainting it would blur "what I read"
with "what I just did". Press `ctrl_state` to see the write reflected in the
grid. Their verdict goes to a `ctrl_cmd_result` label, which the board creates
under the buttons because the `.ui` has none — add one with that object name and
the board uses yours instead.

Enable/disable addresses **one motor**: the other nineteen slots carry the
ignore word, so they keep running untouched. Each call is a ~10 ms bus window on
the GUI thread, and it fails cleanly if a ROS `set_gains` happens to own the
service channel at that moment.

> **Safety**: disabling a motor drops it to standby — it stops holding and that
> finger goes limp. Re-enabling picks up whatever the command holder last had,
> so make sure a sane pose is being commanded before you re-enable.

**readings tab** — the same samples the node publishes, so the screen and
`joint_states` never disagree. Positions and velocities in **degrees** (the wire
is radians), torque in N·m, and `error0-19` as 1/0 per motor. The node publishes
at 100 Hz; the board *samples* the newest cached snapshot at 10 Hz, which is a
reading rate rather than a control rate. Nothing queues, so it cannot fall
behind — but a fault bit that sets and clears inside 100 ms is never drawn.

Those error bits come from `HandState::error_bits`, which rides in every
`readState()` and has nowhere to go in a `JointState` — the board is currently
the only place they surface. Note the SDK's own caveat: `notes.md` lists
"whether the error bits are 1=fault" as assumed rather than confirmed on
hardware. Nothing acts on them automatically for that reason; the board shows
them and you decide.

The readings grid has no side selector, so it shows the left hand when present,
otherwise the right.

---

# `hand_sdk_app` — the standalone Qt GUI

No ROS involved. Do the [common setup](#common-setup) first, then run the binary
directly:

```bash
cd ~/Documents/ros2_dev_ws
./install/hand_sdk_ros2/lib/hand_sdk_ros2/hand_sdk_app
```

`ros2 run hand_sdk_ros2 hand_sdk_app` also works if you happen to have a sourced
overlay, but it buys nothing — there is no node, no topic and no parameter here.

The app talks to the bus itself, so the node must not be running at the same
time.

## Tab 1 — 连接 (bring-up)

Bring-up is split into one button per SDK service call, so each step can be run
and inspected on its own. Press them in order and watch each result label:

| Step | Button | SDK call | What its label reports |
|---|---|---|---|
| 1 | `connect` | `Api::connect()`, starts 50 Hz polling | `left=yes right=no dc=armed` |
| 2 | `mc_state` | `serviceMcState()` | `ready 20/20`, or the faulted joints with their codes — and the 5×4 grid is filled in |
| 3 | `set com` | `serviceComm(kStartBroadcast)` **+ hold-pose seed** | per hand, `broadcast on, seeded` |
| 4 | `set ctrl` | `serviceMode(kForceControl, true)` | `mode: n/20 joints took it`, naming any that refused |
| 5 | `set kpkd` | `serviceGains(kp, kd)` | `gains: n/20 joints took it`, naming any that refused |

kp/kd are prefilled with `3.0` / `3.0`. Every call targets whichever hands
`connect()` reported, and every joint of them. Pressing `connect` again
disconnects.

Steps are independent: nothing is gated on an earlier step having passed, so a
failed step can be retried, or skipped if you know why it failed.

**`set com` does two things.** After starting broadcast it reads the current
pose and sends it back as the standing command, so that `set ctrl` enables
motors onto the pose they are already in rather than onto whatever the device
last had. Seeding runs on every present hand regardless of how broadcast went,
but a hand only *passes* the step if it both started and was seeded — the label
always states both halves (`broadcast on, NOT seeded`). Seeding also mirrors the
measured pose into the tab-3 command boxes.

### OK / PARTIAL / FAILED

Each label is prefixed with a verdict and coloured green / amber / red:

| Verdict | Meaning |
|---|---|
| `OK` | every motor addressed did what it was told |
| `PARTIAL` | some did — amber, because the next step is still worth pressing |
| `FAILED` | none did, or the call never reached the bus |

The count is over every motor on the bus, not per hand, so on a two-hand rig one
healthy hand and one dead one reads PARTIAL rather than FAILED.

For `set com`, `set ctrl` and `set kpkd` the wording comes straight from the SDK,
which verifies each service reply per joint (see the SDK README, "write services
are verified per joint"). A refusal names the joint and what came back:

```
PARTIAL | left: mode: 18/20 joints took it | pass: 0-17 | fail: j18 mode=0 en=0 (wanted mode=0 en=1), j19 ...
```

## Tab 2 — 播放

Not implemented.

## Tab 3 — 调试界面 (debug)

- Live readings at 50 Hz: `readpos0-19` (deg), `readvel0-19` (deg/s),
  `readtau0-19` (N·m), and `error0-19` as `1`/`0` per motor from
  `HandState::error_bits`.
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

Every send carries the same three things per joint: the position from the box,
a commanded velocity of `kCommandVelocity` (0.1 rad/s, one constant at the top
of `hand_sdk_app.cpp`), and a zero feedforward torque. The device closes
`tau = kp(q_d − q) + kd(dq_d − dq) + tau_ff`, so `dq_d` is not cosmetic — with a
standing target it settles the joint `kd·dq_d/kp` past the commanded angle.

**All 20 boxes are sent, every time.** There is no per-joint opt-out: the cyclic
PDO carries q/dq/tau for all 21 slots in every frame, so a partial pose is not
expressible — `0xFFFF` "no action" is a *service channel* feature, not a data
path one. A box you never touched still commands its displayed value. If any box
is blank or not a number the send is refused outright, naming the box in the log.

That makes the order matter: after `set com` the boxes hold the measured pose, so
editing three of them moves three joints. Pressing send *before* `set com` sends
the startup `0.0` in all twenty and drives the whole hand to zero.

> **Safety**: the send buttons command the hands to hold those positions
> immediately once control is enabled in step 4. Keep the hands free to move on
> a first run.

## Shutdown

The window X and the 关闭 button both call `Api::disconnect()`, as does the
destructor. There is no separate close/end call in the SDK — `disconnect()` is
it, and it is safe if triggered twice.

---

# Source layout

| File | Belongs to | Contents |
|---|---|---|
| `src/hand_sdk_ros2_node.cpp` | node | the ROS 2 node, and its `main()` |
| `src/hand_sdk_node_board.hpp/.cpp` | node | the node's diagnostic board |
| `src/hand_sdk_node_board_ui.cpp` | node | uic output for the board; `#include`d, not compiled separately |
| `src/hand_sdk_app.hpp` | app | `HandAppWindow` declaration, grouped by tab |
| `src/hand_sdk_app.cpp` | app | definitions plus `main()` |
| `src/hand_sdk_app_ui.cpp` | app | uic output; a header despite the extension, so it is `#include`d |
| `srv/SetGains.srv` | node | the one ROS service type this package generates |

Known limitations: the app uses `hand_sdk::Config{}` defaults (interface
`enp132s0`) with no way to change them from the UI, and both readings grids show
the left hand when present, since neither UI has a hand selector.
