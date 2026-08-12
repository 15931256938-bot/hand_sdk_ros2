#pragma once

#include <hand_sdk.hpp>

#include <QLabel>
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

    // One-line verdict for the enable/disable buttons. They do not touch the
    // state grid: that shows the last state you asked for, and a write silently
    // repainting it would blur "what I read" with "what I just did".
    void setCtrlResult(bool ok, const QString& text);
    static QString jointName(std::size_t joint);

    // Which motor the finger/joint combos are pointing at.
    std::size_t selectedJoint() const;

    void renderMcState(hand_sdk::service::Hand hand,
                       const hand_sdk::service::McStatusArray& status);
    void renderCtrlState(hand_sdk::service::Hand hand,
                         const hand_sdk::service::ModeStatusArray& status);
    void clearStateColumn(hand_sdk::service::Hand hand, const QString& text);

    // --- readings tab ------------------------------------------------------
    void onRefresh();

    // --- helpers -----------------------------------------------------------
    void appendLog(const QString& text);
    std::array<QLabel*, kNumBoardJoints>& stateLabels(hand_sdk::service::Hand hand);

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
};
