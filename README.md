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

Bring-up runs inside the node's constructor and stops short of enabling
anything: mc-state wait, start broadcast, seed the hold pose from the measured
state. Those lines go to the **ROS console only**; the board's log pane starts
afterwards and shows only what you do from the board.

**The node comes up broadcasting and holding, not enabled.** The device zeroes
itself wherever it happens to be at power-up, so until a joint has been
zero-calibrated its `q` is measured from an arbitrary boot pose — enabling force
control before that would hold the joint against a pose in a frame the
calibration is about to replace. Zero calibration drives the motor looking for
its stop, so it is not something a node constructor should start on its own
either. The rest is pressed from the board, in this order:

```
zero_calibrate -> calibrate_state (until 2 ok) -> seed current pos -> set gains -> ctrl_cmd_enable
```

`set com all` is the retry for the one step bring-up does take, if it failed.

Parameters: `interface_name`, `left_hand_name`, `right_hand_name`,
`publish_rate_hz` (100), `initial_kp` / `initial_kd` (3.0 — no longer applied at
bring-up; the default the board's gains control starts from),
`bringup_timeout_sec` (5.0), `board` (true).

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

Control mode and per-joint gains are **not** on ROS: from ROS the hand is one
unit, and enabling it is a deliberate act that has to follow zero calibration.
Both are reachable from C++ — `Api::serviceMode()` and `Api::serviceGains()`
take a `service::JointMask` (default: every joint) — and from the board below,
which is where you go to enable the hand in the first place, or to disable one
motor.

## The board

The node brings up a diagnostic window with it. It is not a second front end for
driving the hand: it is there to see what is being published and to take a
faulted motor out of control.

**states tab** — two 5×4 grids per hand. The upper one is text, and shows
whichever of the three kinds of state you last asked for — mc-state, control
state, or gains (`render_state_type` says which). The
lower one is checkboxes: **which motors the next write is aimed at.**

| Button | Sends | Shows |
|---|---|---|
| `button_mc_state` | cmd 12, read-only | text grid: `standby` / `active` / `FAULT`, with decoded fault bits, red if faulted |
| `button_ctrl_state` | cmd 11 with the ignore word in every slot — asks without writing | text grid: `1` enabled / `0` disabled, red when 0 |
| `read_gains` | cmd 14 read, every joint | text grid: `kp/kd` per joint, red when both are zero |
| `ctrl_cmd_enable` / `ctrl_cmd_disable` | cmd 11 to **every ticked joint**, then a fresh `ctrl_state` read | text grid refilled from the read; verdict in `ctrl_cmd_result` |
| `set_com_all_button` | cmd 4 start-broadcast, both hands | verdict only — cmd 4 has no mask, so the ticks play no part |
| `zero_calibrate` | cmd 10 write to **every ticked joint** | verdict only — nothing in either grid moves |
| `calibrate_state` | cmd 10 read, **every joint** | checkbox *captions*: `0 none` / `1 busy` / `2 ok` / `-1 FAIL` |
| `setGains` | cmd 14 write to **every ticked joint** from `kp_input` / `kd_input`, then a fresh `read_gains` | text grid refilled from the read; verdict in `ctrl_cmd_result` |
| `seed current pos` | nothing on the service channel — re-latches the standing command from the measured pose | verdict only |
| `selectAll` / `clearAll` | nothing | ticks all / no boxes; captions untouched |

The loop mode is not in the cells — everything is force control unless someone
is debugging — so a joint reporting three-loop is called out in the log instead.

### Ticks are a selection, never a readout

Nothing the device says ever moves a tick. Only a click, `selectAll`, or
`clearAll` does. That is what makes the boxes safe to fire a write against: what
is ticked is exactly what somebody chose, not something a read left behind.

The captions are the other half of each box and move independently —
`calibrate_state` fills all forty from cmd 10, and `selectAll` / `clearAll`
leave them alone. So a box reading `2 ok` **unticked** is the normal resting
state of a calibrated joint you are not currently writing to.

`zero_calibrate` touches neither. It only asks the routine to *start*, and that
routine runs for seconds past the 10 ms service window — press `calibrate_state`
to see where it got to.

### seed_pos

Re-latches the standing command from the pose the hand is measured at, so
force control holds where the joint currently is rather than wherever the
command holder was last left. It stands until something else writes a command —
a `joint_command` message, or another press of this.

It goes through the node (`BoardSource::seedFromState`) rather than calling
`sendCommand()` from the board. The node re-sends its own command holder on
every `joint_command` it receives, so a command the board wrote straight to the
bus would be overwritten by the next message.

Whole hand, not the ticked joints: a command frame carries all twenty-one slots
and the data path has no "no action" word, so there is no such thing as seeding
a subset. The ticks belong to the service writes.

Worth pressing after a `zero_calibrate` finishes: calibration redefines that
joint's origin, so a command latched beforehand refers to an origin the device
has since dropped.

### Gains

`read_gains` fills the text grid with `kp/kd` per joint, red when both are zero
— a joint on zero gains is enabled but holding nothing, which looks exactly like
a failed enable until you read the gains and see why.

`setGains` writes `kp_input` / `kd_input` to **the ticked joints only**, then
re-reads all twenty for the grid (same reason as enable/disable, below). Both
values are s16en11 on the wire, so `|kp|` and `|kd|` must be under 16 — past
that the value saturates at the INT16 rail instead of wrapping, and the device
would quietly run a gain nobody asked for. Out-of-range input is refused rather
than clamped.

Gains want setting **before** `ctrl_cmd_enable`, not after: a joint that comes up
on the device's power-on defaults may be holding with whatever those are.

### Reading back a masked write

Enable/disable addresses only the ticked motors; the rest carry the ignore word
and keep running untouched. **The reply to a masked write can only be trusted
for the joints it addressed.** The SDK already judges it that way — `checkJoints`
verifies the request's mask and nothing else, so `.msg` reads `mode: 4/4 joints
took it` for four ticked joints, not `4/20`.

Reading the *other* slots out of that reply is the trap. A device that echoes the
ignore word back for a slot it was told to skip sends `0xFFFF`, and cmd 11
decodes that as:

```
addressed   j0 : mode=0x00 enabled=1
unaddressed j1 : mode=0xFF enabled=1   <-- decoded from 0xFFFF
```

— an untouched joint reading as **enabled**, because bit 8 of `0xFFFF` is set.
So the board does not paint the grid from the write at all: after the write it
runs a real `ctrl_state` query (cmd 11, all-ignore, which asks about every joint
properly) and fills all twenty cells from that. Two ~10 ms windows instead of
one, and no slot on screen is a guess.

Short version, for your own code: **judge the joints you addressed, read the
whole hand with a query.**

Each call is a ~10 ms bus window on the GUI thread, and it fails cleanly if a ROS
`set_gains` happens to own the service channel at that moment.

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

**Seed last.** Seeding latches the pose the device reports *now*, in whatever
frame it is currently using, so it is always right at the moment it is taken.
What it cannot survive is the frame moving afterwards: zero-calibrating a joint
redefines that joint's origin, so a pose latched beforehand refers to an origin
the device has since dropped, and `set ctrl` would drive the joint by the whole
difference. Press `set com` again after any calibration and before `set ctrl` —
it is re-pressable for exactly this.

### The buttons are independent

This is a bench tool for bringing a hand up a joint or a finger at a time, so
each button makes its own service call and nothing else. `connect` is the only
prerequisite any of them has: nothing is gated on an earlier step having passed,
nothing checks anything else's state first, and a step that failed can simply be
retried or skipped. On a rig with four joints wired up that is the point — the
sixteen that are not there fail their part of every step, and the four that are
there still come up.

Two consequences worth knowing:

- **`set ctrl` reads PARTIAL (amber) on a partial rig, by design.** It counts
  against all 20 active slots, so four working joints report `4/20`. The label
  beside the button carries the SDK's per-joint verdict — `mode: 4/20 joints
  took it | pass: 0-3 | fail: j4 mode=0 en=0 (wanted mode=0 en=1) ...` — so
  which joints came up is read off directly rather than inferred from the
  colour.
- **Nothing stops you calibrating a joint that is currently being held.** The
  position loop and the calibration routine will fight over the motor. That is
  the operator's call, not the window's.

### Zero-point calibration

Below the mc-state grid sits a second 5x4 grid of checkboxes per hand,
`calib_l_00`..`calib_r_43`, and a `calib_refresh` button. These drive cmd 10 and
are independent of the numbered bring-up steps.

**Ticking a box** calls `serviceZeroCalibrate()` on *that joint alone* — the
other nineteen get the device's "no action" word and keep running as they were.
The tick only stands if the write came back OK; a failed write puts the box back
to unticked and captions it `ERR`, with the reason in the tooltip. Either way a
line goes to the log:

```
[14:02:11.331] zero-cal left j7: ok, calibration started (zero-cal | not calibrated: 0-19)
[14:02:19.774] zero-cal right j3: FAILED -- right hand not present
```

A tick therefore means *the device accepted the request*, not *the joint is
calibrated* — the routine runs for seconds past the 10 ms service window.

**Unticking sends nothing.** Only ticking ever puts anything on the wire — the
device has no way to undo a calibration. The box is simply left empty and a line
says so. Between refreshes a tick is therefore only ever something you put there;
`calib_refresh` is what repaints the grid from the device.

**`calib_refresh`** calls `serviceZeroCalibrateState()` once per present hand
and repaints all forty boxes from the reply. One rule decides every tick it
paints:

> **ticked if and only if the joint reported state `2` (succeeded).**

`0`, `1` and `-1` all come back **unticked** — including a joint you ticked by
hand a moment ago that has not finished, or that failed. The caption carries the
state code and a word for it either way.

| Caption | `pos_zero_calibration_state` |
|---|---|
| `-` | not read yet, or the connection was dropped |
| `0 none` | never calibrated |
| `1 busy` | calibrating |
| `2 ok` | succeeded — the box is ticked |
| `-1 FAIL` | failed |
| `ERR` | the call itself failed; the tooltip says why |

The spelled-out name is in each box's tooltip. Boxes are cleared before the read,
so a hand that is not there — or one whose read fails — cannot leave a stale tick
standing, and they are cleared again on disconnect: the device's state resets to
`0` on power-up, so a tick must not outlive the connection that earned it.

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
