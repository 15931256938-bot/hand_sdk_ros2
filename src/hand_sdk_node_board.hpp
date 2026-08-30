#pragma once

#include <hand_sdk.hpp>

#include <QCheckBox>
#include <QLabel>
#include <QLineEdit>
#include <QMainWindow>
#include <QString>
#include <QTimer>

// uic output; kept with a .cpp extension in this package, so it is included
// rather than compiled as its own translation unit.
#include "hand_sdk_node_board_ui.cpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

// The board shows the 20 active joints; the SDK's 21st slot is reserved.
constexpr std::size_t kNumBoardJoints = 20;

// How a step went. A step that reached some of the motors but not all of them
// is reported as kPartial rather than failed: on a hand with one motor powered
// you still want to carry on to the next step and drive that motor. Nothing
// gates on this -- it is what the log line and the result label say, not a lock.
enum class Verdict
{
    kFail,
    kPartial,
    kPass,
};

// One published state sample, as the node last put on joint_states. The board
// renders exactly what was published rather than reading the bus itself, so
// what you see on screen is what a subscriber saw.
struct BoardSnapshot
{
    bool valid{false};
    std::array<float, kNumBoardJoints> q{};    // rad
    std::array<float, kNumBoardJoints> dq{};   // rad/s
    std::array<float, kNumBoardJoints> tau{};  // N*m
    uint32_t error_bits{0};                    // bit i = motor i, 1 = fault
};

// What the board needs from the node. An interface rather than the node type so
// the board does not have to know about rclcpp at all.
//
// The bus is brought up from the board and nowhere else -- the node starts
// disconnected, so the window is always there to press `connect` in, whatever
// the hardware is doing. connectBus(), disconnectBus() and api() are for the Qt
// thread only; the node's own threads reach the SDK through their own handle
// and keep working while this one is null.
class BoardSource
{
public:
    // What connect() got: the SDK's own message, plus what it found on the bus.
    struct Connection
    {
        bool success{false};
        std::string msg{};
        bool has_left{false};
        bool has_right{false};
        bool dc_active{false};
    };

    virtual ~BoardSource() = default;

    virtual Connection connectBus() = 0;
    // Returns the closing summary -- where the slaves ended in the EtherCAT
    // state machine and how the cycling went. That is only readable at
    // disconnect, so it is handed back rather than dropped.
    virtual std::string disconnectBus() = 0;

    // Null until connectBus() has succeeded; null again after disconnectBus().
    virtual hand_sdk::Api* api() = 0;

    virtual bool present(hand_sdk::service::Hand hand) const = 0;
    virtual BoardSnapshot snapshot(hand_sdk::service::Hand hand) const = 0;

    // Re-latch the standing command from the measured pose. This has to go
    // through the node rather than being a sendCommand() from the board: the
    // node re-sends its own command holder on every joint_command it receives,
    // so anything the board wrote straight to the bus would be overwritten by
    // the next message. Returns false if the hand is absent or no state came
    // back to seed from.
    virtual bool seedFromState(hand_sdk::service::Hand hand) = 0;
};

// Diagnostic front end for hand_sdk_ros2_node, and the only place the bus is
// driven by hand. The node itself does one thing -- joint_command in,
// joint_states out -- so everything that is not that lives here: connecting,
// opening communication, zero calibration, gains, and turning force control on
// and off. What the node is publishing shows on the second tab.
//
// It runs on the Qt main thread while ROS spins on its own. The only shared
// state is the node's snapshot (mutex-guarded) and the SDK Api, whose calls are
// documented as safe from any thread.
class HandNodeBoard final : public QMainWindow
{
public:
    // The gains fields start from the node's initial_kp / initial_kd, so one
    // parameter file sets what the board offers and what a script would send.
    HandNodeBoard(BoardSource& source, double initial_kp, double initial_kd);

private:
    // --- setup -------------------------------------------------------------
    void cacheWidgets();
    void wireButtons();

    // --- tab 1: the bus, one step per button -------------------------------
    void onConnect();        // button_connect:  connect, or drop the bus again
    void onComm();           // button_com:      cmd 4 start broadcast, then seed
    void onSeedPos();        // seed_pos:        the seeding half of button_com, alone
    void onSetEnable(bool enable);  // ctrl_cmd_enable / ctrl_cmd_disable: cmd 11
    void onMcState();        // button_mc_state: cmd 12 read, per-joint state + fault
    void onCalibrateState(); // calibrate_state: cmd 10 read, every joint
    void onReadGains();      // read_gains:      cmd 14 read, every joint
    void onZeroCalibrate();  // zero_calibrate:  cmd 10 write, ticked joints only
    void onSetGains();       // set_gains:       cmd 14 write, ticked joints only

    // Latch the measured pose as the standing command on each of `hands`, and
    // return the ones that took it. `detail` gets the fragment the caller puts
    // at the end of its log line.
    std::vector<hand_sdk::service::Hand> seedHands(
        const std::vector<hand_sdk::service::Hand>& hands, QString& detail);

    // --- the tick table ----------------------------------------------------
    //
    // The ticks are a selection and nothing else: which motors the next write
    // is aimed at. Clicking one sends nothing, and nothing the device says ever
    // moves one -- only a click, or selectAll / clearAll. That is what makes
    // them safe to press a write against, since what is ticked is exactly what
    // somebody chose.
    //
    // Only zero_calibrate and setGains read them. Enable/disable go to every
    // active joint, the way the app's `set ctrl` does: turning the hand on is
    // not a per-motor operation.
    void setAllChecks(bool on);   // select_all / clear_all
    hand_sdk::service::JointMask checkedMask(hand_sdk::service::Hand hand) const;

    // --- the state grid ----------------------------------------------------
    //
    // One 5x4 grid per hand, showing whichever of the three reads was asked for
    // last: mc-state, zero-calibration state, or gains. label_40 is the caption
    // that says which.
    void setStateKind(const QString& kind);
    void renderMcState(hand_sdk::service::Hand hand,
                       const hand_sdk::service::McStatusArray& status);
    void renderCalibState(hand_sdk::service::Hand hand,
                          const hand_sdk::service::ZeroCalStateArray& states);
    void renderGains(hand_sdk::service::Hand hand,
                     const hand_sdk::service::JointGainsArray& gains);
    void clearStateColumn(hand_sdk::service::Hand hand, const QString& text);
    void clearStateGrid(const QString& text);
    // Cells keep the joint-in-finger prefix the .ui ships them with, so a
    // column stays readable when what it is showing changes.
    void setCell(hand_sdk::service::Hand hand, std::size_t joint,
                 const QString& text, bool bad, const QString& tip = QString());

    // --- tab 2: what the node is publishing --------------------------------
    void onRefresh();

    // --- helpers -----------------------------------------------------------
    void appendLog(const QString& text);
    // The one-line verdict beside the control buttons. It is small, so it gets
    // a word and a colour; `detail` is what goes in the log and the tooltip.
    void setCtrlResult(Verdict verdict, const QString& text, const QString& detail);

    // Every button but `connect` needs a bus with a hand on it. Refused in one
    // place, logged the same way each time.
    bool requireConnected(const QString& what);
    std::vector<hand_sdk::service::Hand> presentHands() const;

    std::array<QLabel*, kNumBoardJoints>& stateLabels(hand_sdk::service::Hand hand);
    std::array<QCheckBox*, kNumBoardJoints>& checkBoxes(hand_sdk::service::Hand hand);
    const std::array<QCheckBox*, kNumBoardJoints>& checkBoxes(hand_sdk::service::Hand hand) const;

    // Runs a read service on every hand the bus reported, logging each result
    // and blanking the column of a hand that is not there.
    template <typename Call>
    void forEachHand(const QString& what, Call call);

    Ui::MainWindow ui_;
    BoardSource& source_;
    QTimer* refresh_timer_{nullptr};

    std::array<QLabel*, kNumBoardJoints> pos_labels_{};
    std::array<QLabel*, kNumBoardJoints> vel_labels_{};
    std::array<QLabel*, kNumBoardJoints> tau_labels_{};
    std::array<QLabel*, kNumBoardJoints> err_labels_{};
    std::array<QLabel*, kNumBoardJoints> state_left_{};
    std::array<QLabel*, kNumBoardJoints> state_right_{};
    std::array<QCheckBox*, kNumBoardJoints> check_left_{};
    std::array<QCheckBox*, kNumBoardJoints> check_right_{};
};
