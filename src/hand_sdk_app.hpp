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
#include "hand_sdk_app_ui.cpp"

#include <array>
#include <cstddef>
#include <memory>
#include <vector>

// The UI exposes the 20 active joints; the SDK's 21st slot is reserved.
constexpr std::size_t kNumUiJoints = 20;

// How a bring-up step went. A step that reached some of the motors but not all
// of them is reported as kPartial rather than failed: on a hand with one motor
// powered you still want to carry on to the next step and drive that motor.
// Nothing gates on this -- it is what the result label says, not a lock.
enum class Verdict
{
    kFail,
    kPartial,
    kPass,
};

// Qt front end for the hand SDK. Bring-up is deliberately split into one
// button per service call so each step can be run and inspected on its own.
class HandAppWindow final : public QMainWindow
{
public:
    HandAppWindow();
    ~HandAppWindow() override;

protected:
    void closeEvent(QCloseEvent* event) override;

private:
    // --- Setup ---------------------------------------------------------------

    void cacheJointWidgets();
    void wireButtons();
    void wireCalibBoxes();

    // --- Shared helpers ------------------------------------------------------

    void appendLog(const QString& text);
    void setResult(QLabel* label, Verdict verdict, const QString& text);

    // The hands the bus actually reported; every service call targets these.
    std::vector<hand_sdk::service::Hand> presentHands() const;
    // `result` may be null for the controls that have no result label of their
    // own; the log line is written either way.
    bool requireConnected(QLabel* result);

    // --- Tab 1: bring-up, one step per button --------------------------------

    void onConnect();   // button_connect  -> result_connect
    void onComm();      // button_com      -> result_com

    // Latch the measured pose as the standing command, run as part of onComm()
    // so that it is always in place before onCtrl() enables force control.
    //
    // Runs on whatever hands it is given, independent of how the broadcast
    // step went, and returns the ones it actually seeded. A hand counts as
    // having passed the comm step only when it both started and comes back in
    // this list, so what the result line claims and what the device is holding
    // cannot drift apart. `detail` gets the fragment shown at the end of that
    // line.
    std::vector<hand_sdk::service::Hand> seedCommandFromState(
        const std::vector<hand_sdk::service::Hand>& hands, QString& detail);

    void onCtrl();      // button_ctrl     -> result_ctrl
    void onMcState();   // button_mc_state -> result_mc_state
    void onGains();     // button_kpkd     -> result_kpkd

    // --- Tab 1: zero-point calibration (cmd 10) ------------------------------

    // One checkbox per joint per hand. calib_refresh paints the grid from the
    // device: ticked == the joint last reported pos_zero_calibration_state 2,
    // with 0, 1 and -1 all unticked, so a freshly refreshed grid can be scanned
    // for what is actually zeroed.
    //
    // Ticking one box starts that joint's calibration and nothing else's.
    // Unticking sends nothing -- there is no "un-calibrate" on the device --
    // and leaves the box empty; between refreshes a tick is therefore only ever
    // something the operator put there, never something the device claimed.
    //
    // The device zeroes itself at power-up and this state resets to 0 with it,
    // so every session needs its own calibration and no tick may outlive the
    // connection that earned it (see shutdownSdk()).
    void onCalibClicked(hand_sdk::service::Hand hand, std::size_t joint, bool checked);
    void onCalibRefresh();   // calib_refresh: re-read every joint's state
    void resetCalibBoxes();

    // Paint one box and remember the tick, so a later untick has something
    // truthful to spring back to.
    void paintCalibBox(hand_sdk::service::Hand hand, std::size_t joint,
                       hand_sdk::service::ZeroCalState state, bool ticked);
    // Clear a hand's whole column: no tick, `text` as the caption, `tip` as the
    // reason. Used when there is nothing to show or the read failed.
    void blankCalibBoxes(hand_sdk::service::Hand hand, const QString& text, const QString& tip);

    std::array<QCheckBox*, kNumUiJoints>& calibBoxes(hand_sdk::service::Hand hand);
    std::array<bool, kNumUiJoints>& calibTicks(hand_sdk::service::Hand hand);
    bool handPresent(hand_sdk::service::Hand hand) const;

    // --- Tab 3: live readings and manual commands ----------------------------

    void onPoll();         // 50 Hz: readpos/readvel/readtau/error labels
    void onSendFields();   // send_command_from_fields
    void onSendZero();     // send_zero_command_2
    void onDumpJson();     // button_readjson

    // Fields are degrees; the SDK takes radians.
    bool readFieldsAsRadians(std::array<float, kNumUiJoints>& out);
    void sendPositions(const std::array<float, kNumUiJoints>& positions_rad, const QString& label);

    // --- Teardown ------------------------------------------------------------

    // Runs from the quit button, the window close, and the destructor.
    void shutdownSdk();

    Ui::MainWindow ui_;
    // Kept rather than built at connect time: the write services verify their
    // replies against config_.service_tolerance, and the per-joint lists on
    // screen have to be drawn with the same number the SDK judged by.
    hand_sdk::Config config_{};
    std::unique_ptr<hand_sdk::Api> api_;
    hand_sdk::HandsStatus hands_{};
    QTimer* poll_timer_{nullptr};
    std::array<QLabel*, kNumUiJoints> pos_labels_{};
    std::array<QLabel*, kNumUiJoints> vel_labels_{};
    std::array<QLabel*, kNumUiJoints> tau_labels_{};
    std::array<QLabel*, kNumUiJoints> err_labels_{};
    std::array<QLabel*, kNumUiJoints> mc_left_labels_{};
    std::array<QLabel*, kNumUiJoints> mc_right_labels_{};
    std::array<QLineEdit*, kNumUiJoints> cmd_fields_{};
    std::array<QCheckBox*, kNumUiJoints> calib_left_boxes_{};
    std::array<QCheckBox*, kNumUiJoints> calib_right_boxes_{};
    // What each box should be showing, kept beside the widgets so that a user
    // untick can be undone without going back to the bus for an answer we
    // already have.
    std::array<bool, kNumUiJoints> calib_left_ticks_{};
    std::array<bool, kNumUiJoints> calib_right_ticks_{};
};
