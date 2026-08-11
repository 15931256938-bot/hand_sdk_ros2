#pragma once

#include <hand_sdk.hpp>

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

    // --- Shared helpers ------------------------------------------------------

    void appendLog(const QString& text);
    void setResult(QLabel* label, bool ok, const QString& text);

    // The hands the bus actually reported; every service call targets these.
    std::vector<hand_sdk::service::Hand> presentHands() const;
    bool requireConnected(QLabel* result);

    // --- Tab 1: bring-up, one step per button --------------------------------

    void onConnect();   // button_connect  -> result_connect
    void onComm();      // button_com      -> result_com

    // Latch the measured pose as the standing command, run as part of onComm()
    // so that it is always in place before onCtrl() enables force control.
    // `detail` gets the fragment shown in the comm result line.
    bool seedCommandFromState(QString& detail);

    void onCtrl();      // button_ctrl     -> result_ctrl
    void onMcState();   // button_mc_state -> result_mc_state
    void onGains();     // button_kpkd     -> result_kpkd

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
};
