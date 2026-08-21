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

// The board shows the 20 active joints; the SDK's 21st slot is reserved.
constexpr std::size_t kNumBoardJoints = 20;

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
class BoardSource
{
public:
    virtual ~BoardSource() = default;
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

// Diagnostic front end for hand_sdk_ros2_node: watch what is being published,
// see which motors have faulted, and take one of them out of control.
//
// It runs on the Qt main thread while ROS spins on its own. The only shared
// state is the node's snapshot (mutex-guarded) and the SDK Api, which is
// documented as safe to call from any thread.
class HandNodeBoard final : public QMainWindow
{
public:
    explicit HandNodeBoard(BoardSource& source);

private:
    // --- setup -------------------------------------------------------------
    void cacheWidgets();
    void ensureCtrlResultLabel();
    void wireButtons();

    // --- states tab --------------------------------------------------------
    void onMcState();     // button_mc_state:   cmd 12, per-joint mc state + fault
    void onCtrlState();   // button_ctrl_state: cmd 11 all-ignore, reports mode/enable
    void onSetEnable(bool enable);  // ctrl_cmd_enable / ctrl_cmd_disable
    void onSetComAll();      // set_com_all_button: cmd 4, start broadcast
    void onZeroCalibrate();  // zero_calibrate:  cmd 10 write, ticked joints only
    void onCalibrateState(); // calibrate_state: cmd 10 read, every joint
    void onSeedPos();        // seed_pos: re-latch the hold pose from measured
    void onReadGains();      // pushButton "read_gains": cmd 14 read, every joint
    void onSetGains();       // set_gains: cmd 14 write, ticked joints only

    // --- the tick table ----------------------------------------------------
    //
    // The ticks are a selection and nothing else: which motors the next write
    // is aimed at. Nothing the device says ever moves one -- only a click, or
    // select_all / clear_all. That is what makes them safe to press a write
    // against, since what is ticked is exactly what somebody chose.
    //
    // The captions are the other half and move independently: `calibrate state`
    // fills all forty from cmd 10, and select_all / clear_all leave them alone.
    void setAllChecks(bool on);   // select_all / clear_all
    hand_sdk::service::JointMask checkedMask(hand_sdk::service::Hand hand) const;
    void setCheckCaptions(hand_sdk::service::Hand hand, const QString& text, const QString& tip);

    // One-line verdict for the enable/disable buttons. They do not touch the
    // state grid: that shows the last state you asked for, and a write silently
    // repainting it would blur "what I read" with "what I just did".
    void setCtrlResult(bool ok, const QString& text);

    void renderMcState(hand_sdk::service::Hand hand,
                       const hand_sdk::service::McStatusArray& status);
    void renderCtrlState(hand_sdk::service::Hand hand,
                         const hand_sdk::service::ModeStatusArray& status);
    void renderGains(hand_sdk::service::Hand hand,
                     const hand_sdk::service::JointGainsArray& gains);
    void clearStateColumn(hand_sdk::service::Hand hand, const QString& text);

    // --- readings tab ------------------------------------------------------
    void onRefresh();

    // --- helpers -----------------------------------------------------------
    void appendLog(const QString& text);
    std::array<QLabel*, kNumBoardJoints>& stateLabels(hand_sdk::service::Hand hand);
    std::array<QCheckBox*, kNumBoardJoints>& checkBoxes(hand_sdk::service::Hand hand);
    const std::array<QCheckBox*, kNumBoardJoints>& checkBoxes(hand_sdk::service::Hand hand) const;

    // Runs a service call on every hand the bus reported, logging each result.
    template <typename Call>
    void forEachHand(const QString& what, Call call);

    Ui::MainWindow ui_;
    BoardSource& source_;
    QTimer* refresh_timer_{nullptr};
    QLabel* ctrl_result_{nullptr};

    std::array<QLabel*, kNumBoardJoints> pos_labels_{};
    std::array<QLabel*, kNumBoardJoints> vel_labels_{};
    std::array<QLabel*, kNumBoardJoints> tau_labels_{};
    std::array<QLabel*, kNumBoardJoints> err_labels_{};
    std::array<QLabel*, kNumBoardJoints> state_left_{};
    std::array<QLabel*, kNumBoardJoints> state_right_{};
    std::array<QCheckBox*, kNumBoardJoints> check_left_{};
    std::array<QCheckBox*, kNumBoardJoints> check_right_{};
};
