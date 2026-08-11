#include "hand_sdk_app.hpp"

#include <QApplication>
#include <QCloseEvent>
#include <QDateTime>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QPushButton>
#include <QStringList>
#include <QTextEdit>

#include <chrono>
#include <cmath>
#include <thread>

namespace
{

constexpr double kRadToDeg = 180.0 / M_PI;
constexpr double kDegToRad = M_PI / 180.0;

QString mcStateName(hand_sdk::service::McState state)
{
    switch (state) {
    case hand_sdk::service::McState::kNone:    return QStringLiteral("none");
    case hand_sdk::service::McState::kInit:    return QStringLiteral("init");
    case hand_sdk::service::McState::kStandby: return QStringLiteral("standby");
    case hand_sdk::service::McState::kActive:  return QStringLiteral("active");
    case hand_sdk::service::McState::kFault:   return QStringLiteral("fault");
    }
    return QStringLiteral("unknown");
}

QString handName(hand_sdk::service::Hand hand)
{
    return hand == hand_sdk::service::Hand::kLeft ? QStringLiteral("left")
                                                  : QStringLiteral("right");
}

QString mcStateLabel(hand_sdk::service::McState state)
{
    switch (state) {
    case hand_sdk::service::McState::kNone:    return QStringLiteral("None");
    case hand_sdk::service::McState::kInit:    return QStringLiteral("Init");
    case hand_sdk::service::McState::kStandby: return QStringLiteral("Standby");
    case hand_sdk::service::McState::kActive:  return QStringLiteral("Active");
    case hand_sdk::service::McState::kFault:   return QStringLiteral("Fault");
    }
    return QStringLiteral("Unknown");
}

QString mcFaultNames(uint32_t code)
{
    using namespace hand_sdk::service;

    if (code == fault::kNone) {
        return QStringLiteral("None");
    }

    QStringList names;
    if ((code & fault::kOverCurrent) != 0u) {
        names << QStringLiteral("OverCurrent");
    }
    if ((code & fault::kOverVoltage) != 0u) {
        names << QStringLiteral("OverVoltage");
    }
    if ((code & fault::kUnderVoltage) != 0u) {
        names << QStringLiteral("UnderVoltage");
    }
    if ((code & fault::kStall) != 0u) {
        names << QStringLiteral("Stall");
    }
    if ((code & fault::kOverTemp) != 0u) {
        names << QStringLiteral("OverTemp");
    }
    if ((code & fault::kCurrentZero) != 0u) {
        names << QStringLiteral("CurrentZero");
    }
    if ((code & fault::kPosNotSync) != 0u) {
        names << QStringLiteral("PosNotSync");
    }
    if ((code & fault::kOverSpeed) != 0u) {
        names << QStringLiteral("OverSpeed");
    }
    if ((code & fault::kHardware) != 0u) {
        names << QStringLiteral("Hardware");
    }

    constexpr uint32_t kKnownMask =
        fault::kOverCurrent | fault::kOverVoltage | fault::kUnderVoltage |
        fault::kStall | fault::kOverTemp | fault::kCurrentZero |
        fault::kPosNotSync | fault::kOverSpeed | fault::kHardware;

    const uint32_t unknown = code & ~kKnownMask;
    if (unknown != 0u) {
        names << QStringLiteral("Unknown(0x%1)").arg(unknown, 0, 16);
    }

    return names.join(QStringLiteral("|"));
}

void resetMcLabels(const std::array<QLabel*, kNumUiJoints>& labels)
{
    for (std::size_t i = 0; i < kNumUiJoints; ++i) {
        if (labels[i] != nullptr) {
            labels[i]->setText(QStringLiteral("%1: ").arg(static_cast<int>(i % 4)));
        }
    }
}

void renderMcLabels(const std::array<QLabel*, kNumUiJoints>& labels,
                    const hand_sdk::service::McStatusArray& states)
{
    for (std::size_t i = 0; i < kNumUiJoints; ++i) {
        if (labels[i] == nullptr) {
            continue;
        }

        const int joint_in_finger = static_cast<int>(i % 4);
        const auto state = states[i].state;
        if (static_cast<uint8_t>(state) == 0u) {
            labels[i]->setText(QStringLiteral("%1: ").arg(joint_in_finger));
            continue;
        }
        if (state == hand_sdk::service::McState::kFault) {
            labels[i]->setText(QStringLiteral("%1: Fault: %2")
                                   .arg(joint_in_finger)
                                   .arg(mcFaultNames(states[i].fault_code)));
        } else {
            labels[i]->setText(QStringLiteral("%1: %2")
                                   .arg(joint_in_finger)
                                   .arg(mcStateLabel(state)));
        }
    }
}

} // namespace

HandAppWindow::HandAppWindow()
{
    ui_.setupUi(this);
    cacheJointWidgets();
    wireButtons();

    ui_.kp_input->setText(QStringLiteral("3.0"));
    ui_.kd_input->setText(QStringLiteral("3.0"));

    poll_timer_ = new QTimer(this);
    poll_timer_->setInterval(20);  // 50 Hz
    connect(poll_timer_, &QTimer::timeout, this, [this]() { onPoll(); });

    setResult(ui_.result_connect, false, QStringLiteral("not connected"));
}

HandAppWindow::~HandAppWindow()
{
    shutdownSdk();
}

void HandAppWindow::closeEvent(QCloseEvent* event)
{
    shutdownSdk();
    event->accept();
}

void HandAppWindow::cacheJointWidgets()
{
    for (std::size_t i = 0; i < kNumUiJoints; ++i) {
        const auto index = static_cast<int>(i);
        const auto finger = static_cast<int>(i / 4);
        const auto joint  = static_cast<int>(i % 4);

        pos_labels_[i] = findChild<QLabel*>(QStringLiteral("readpos%1").arg(index));
        vel_labels_[i] = findChild<QLabel*>(QStringLiteral("readvel%1").arg(index));
        tau_labels_[i] = findChild<QLabel*>(QStringLiteral("readtau%1").arg(index));
        err_labels_[i] = findChild<QLabel*>(QStringLiteral("error%1").arg(index));
        mc_left_labels_[i] =
            findChild<QLabel*>(QStringLiteral("state_l_%1%2").arg(finger).arg(joint));
        mc_right_labels_[i] =
            findChild<QLabel*>(QStringLiteral("state_r_%1%2").arg(finger).arg(joint));
        cmd_fields_[i] = findChild<QLineEdit*>(QStringLiteral("command_pos%1").arg(index));
        if (cmd_fields_[i] != nullptr) {
            cmd_fields_[i]->setText(QStringLiteral("0.0"));
        }
    }

    resetMcLabels(mc_left_labels_);
    resetMcLabels(mc_right_labels_);
}

void HandAppWindow::wireButtons()
{
    connect(ui_.button_connect, &QPushButton::clicked, this, [this]() { onConnect(); });
    connect(ui_.button_com, &QPushButton::clicked, this, [this]() { onComm(); });
    connect(ui_.button_ctrl, &QPushButton::clicked, this, [this]() { onCtrl(); });
    connect(ui_.button_mc_state, &QPushButton::clicked, this, [this]() { onMcState(); });
    connect(ui_.button_kpkd, &QPushButton::clicked, this, [this]() { onGains(); });

    connect(ui_.send_command_from_fields, &QPushButton::clicked, this,
            [this]() { onSendFields(); });
    connect(ui_.send_zero_command_2, &QPushButton::clicked, this, [this]() { onSendZero(); });
    connect(ui_.button_readjson, &QPushButton::clicked, this, [this]() { onDumpJson(); });

    connect(ui_.quit, &QPushButton::clicked, this, [this]() { close(); });
}

void HandAppWindow::appendLog(const QString& text)
{
    const auto stamp = QDateTime::currentDateTime().toString(QStringLiteral("HH:mm:ss.zzz"));
    ui_.log->append(QStringLiteral("[%1] %2").arg(stamp, text));
}

void HandAppWindow::setResult(QLabel* label, bool ok, const QString& text)
{
    label->setText(text);
    label->setStyleSheet(ok ? QStringLiteral("color: green;") : QStringLiteral("color: red;"));
}

std::vector<hand_sdk::service::Hand> HandAppWindow::presentHands() const
{
    std::vector<hand_sdk::service::Hand> hands;
    if (hands_.has_left) {
        hands.push_back(hand_sdk::service::Hand::kLeft);
    }
    if (hands_.has_right) {
        hands.push_back(hand_sdk::service::Hand::kRight);
    }
    return hands;
}

bool HandAppWindow::requireConnected(QLabel* result)
{
    if (api_ && (hands_.has_left || hands_.has_right)) {
        return true;
    }
    setResult(result, false, QStringLiteral("connect first"));
    appendLog(QStringLiteral("rejected: not connected"));
    return false;
}

void HandAppWindow::onConnect()
{
    if (api_) {
        shutdownSdk();
        setResult(ui_.result_connect, false, QStringLiteral("disconnected"));
        appendLog(QStringLiteral("disconnected"));
        return;
    }

    api_ = std::make_unique<hand_sdk::Api>(hand_sdk::Config{});
    const auto connected = api_->connect();
    if (!connected.success) {
        api_.reset();
        setResult(ui_.result_connect, false, QString::fromStdString(connected.msg));
        appendLog(QStringLiteral("connect failed: %1").arg(QString::fromStdString(connected.msg)));
        return;
    }

    hands_ = connected.value;
    const auto summary = QStringLiteral("left=%1 right=%2 dc=%3")
                             .arg(hands_.has_left ? QStringLiteral("yes") : QStringLiteral("no"),
                                  hands_.has_right ? QStringLiteral("yes") : QStringLiteral("no"),
                                  api_->dcActive() ? QStringLiteral("armed") : QStringLiteral("off"));
    setResult(ui_.result_connect, true, summary);
    appendLog(QStringLiteral("connected: %1").arg(summary));
    poll_timer_->start();
}

void HandAppWindow::onComm()
{
    if (!requireConnected(ui_.result_com)) {
        return;
    }

    QStringList parts;
    bool all_ok = true;
    for (const auto hand : presentHands()) {
        const auto result = api_->serviceComm(hand, hand_sdk::service::ComCmd::kStartBroadcast);
        all_ok = all_ok && result.success;
        parts << QStringLiteral("%1: %2").arg(
            handName(hand),
            result.success ? QStringLiteral("broadcast on")
                           : QString::fromStdString(result.msg));
    }

    QString seed_detail;
    all_ok = seedCommandFromState(seed_detail) && all_ok;
    parts << seed_detail;

    const auto text = parts.join(QStringLiteral(" | "));
    setResult(ui_.result_com, all_ok, text);
    appendLog(QStringLiteral("comm: %1").arg(text));
}

bool HandAppWindow::seedCommandFromState(QString& detail)
{
    using namespace std::chrono_literals;

    // The bus has been cycling since onConnect(), so the first read normally
    // succeeds; the retries only cover a torn read or a just-started cycle.
    hand_sdk::frames::DualHandState state{};
    bool have_state = false;
    for (int attempt = 0; attempt < 40; ++attempt) {
        const auto read = api_->readState();
        if (read.success && (read.value.has_left || read.value.has_right)) {
            state = read.value;
            have_state = true;
            break;
        }
        std::this_thread::sleep_for(5ms);
    }

    if (!have_state) {
        detail = QStringLiteral("seed: no state to hold");
        return false;
    }

    // Per hand, not the shared pose sendPositions() uses: the two hands are in
    // different poses, and seeding one from the other's would be a lurch.
    hand_sdk::frames::DualHandCommand command;
    if (hands_.has_left && state.has_left) {
        command.has_left = true;
        command.left.q = state.left.q;  // dq / tau stay zero
    }
    if (hands_.has_right && state.has_right) {
        command.has_right = true;
        command.right.q = state.right.q;
    }

    if (!command.has_left && !command.has_right) {
        detail = QStringLiteral("seed: present hand missing from state");
        return false;
    }

    const auto sent = api_->sendCommand(command);
    if (!sent.success) {
        detail = QStringLiteral("seed: %1").arg(QString::fromStdString(sent.msg));
        return false;
    }

    // Mirror the latched pose into the command fields, so the first "send
    // fields" of a tuning session continues from the hold pose rather than
    // jumping to whatever was left in the boxes.
    const auto& shown = command.has_left ? command.left : command.right;
    for (std::size_t i = 0; i < kNumUiJoints; ++i) {
        if (cmd_fields_[i] != nullptr) {
            cmd_fields_[i]->setText(
                QString::number(static_cast<double>(shown.q[i]) * kRadToDeg, 'f', 2));
        }
    }

    detail = QStringLiteral("hold pose seeded");
    return true;
}

void HandAppWindow::onCtrl()
{
    if (!requireConnected(ui_.result_ctrl)) {
        return;
    }

    QStringList parts;
    bool all_ok = true;
    for (const auto hand : presentHands()) {
        const auto result = api_->serviceMode(hand, hand_sdk::service::LoopMode::kForceControl, true);
        all_ok = all_ok && result.success;
        if (!result.success) {
            parts << QStringLiteral("%1: %2").arg(handName(hand),
                                                  QString::fromStdString(result.msg));
            continue;
        }

        int enabled = 0;
        for (std::size_t i = 0; i < kNumUiJoints; ++i) {
            if (result.value[i].enabled) {
                ++enabled;
            }
        }
        parts << QStringLiteral("%1: force-ctrl %2/%3 enabled")
                     .arg(handName(hand))
                     .arg(enabled)
                     .arg(static_cast<int>(kNumUiJoints));
    }

    const auto text = parts.join(QStringLiteral(" | "));
    setResult(ui_.result_ctrl, all_ok, text);
    appendLog(QStringLiteral("ctrl: %1").arg(text));
}

void HandAppWindow::onMcState()
{
    if (!requireConnected(ui_.result_mc_state)) {
        return;
    }

    resetMcLabels(mc_left_labels_);
    resetMcLabels(mc_right_labels_);

    QStringList parts;
    bool all_ok = true;
    for (const auto hand : presentHands()) {
        const auto result = api_->serviceMcState(hand);
        all_ok = all_ok && result.success;
        if (!result.success) {
            parts << QStringLiteral("%1: %2").arg(handName(hand),
                                                  QString::fromStdString(result.msg));
            continue;
        }

        if (hand == hand_sdk::service::Hand::kLeft) {
            renderMcLabels(mc_left_labels_, result.value);
        } else {
            renderMcLabels(mc_right_labels_, result.value);
        }

        int ready = 0;
        QStringList faults;
        for (std::size_t i = 0; i < kNumUiJoints; ++i) {
            const auto& joint = result.value[i];
            if (joint.state == hand_sdk::service::McState::kStandby ||
                joint.state == hand_sdk::service::McState::kActive) {
                ++ready;
            } else {
                faults << QStringLiteral("j%1=%2(0x%3)")
                              .arg(static_cast<int>(i))
                              .arg(mcStateName(joint.state))
                              .arg(joint.fault_code, 0, 16);
            }
        }

        const bool hand_ok = ready == static_cast<int>(kNumUiJoints);
        all_ok = all_ok && hand_ok;
        parts << QStringLiteral("%1: ready %2/%3%4")
                     .arg(handName(hand))
                     .arg(ready)
                     .arg(static_cast<int>(kNumUiJoints))
                     .arg(faults.isEmpty()
                              ? QString()
                              : QStringLiteral(" [%1]").arg(faults.join(QLatin1Char(','))));
    }

    const auto text = parts.join(QStringLiteral(" | "));
    setResult(ui_.result_mc_state, all_ok, text);
    appendLog(QStringLiteral("mc-state: %1").arg(text));
}

void HandAppWindow::onGains()
{
    if (!requireConnected(ui_.result_kpkd)) {
        return;
    }

    bool kp_ok = false;
    bool kd_ok = false;
    const double kp = ui_.kp_input->text().toDouble(&kp_ok);
    const double kd = ui_.kd_input->text().toDouble(&kd_ok);
    if (!kp_ok || !kd_ok) {
        setResult(ui_.result_kpkd, false, QStringLiteral("kp/kd must be numbers"));
        return;
    }

    QStringList parts;
    bool all_ok = true;
    for (const auto hand : presentHands()) {
        const auto result = api_->serviceGains(hand, static_cast<float>(kp), static_cast<float>(kd));
        all_ok = all_ok && result.success;
        parts << (result.success
                      ? QStringLiteral("%1: kp=%2 kd=%3")
                            .arg(handName(hand))
                            .arg(static_cast<double>(result.value[0].kp), 0, 'f', 3)
                            .arg(static_cast<double>(result.value[0].kd), 0, 'f', 3)
                      : QStringLiteral("%1: %2").arg(handName(hand),
                                                     QString::fromStdString(result.msg)));
    }

    const auto text = parts.join(QStringLiteral(" | "));
    setResult(ui_.result_kpkd, all_ok, text);
    appendLog(QStringLiteral("gains: %1").arg(text));
}

void HandAppWindow::onPoll()
{
    if (!api_) {
        return;
    }

    const auto state = api_->readState();
    if (!state.success) {
        return;
    }

    const bool use_left = state.value.has_left;
    if (!use_left && !state.value.has_right) {
        return;
    }

    const auto& hand_state = use_left ? state.value.left : state.value.right;
    for (std::size_t i = 0; i < kNumUiJoints; ++i) {
        if (pos_labels_[i] != nullptr) {
            pos_labels_[i]->setText(
                QString::number(static_cast<double>(hand_state.q[i]) * kRadToDeg, 'f', 2));
        }
        if (vel_labels_[i] != nullptr) {
            vel_labels_[i]->setText(
                QString::number(static_cast<double>(hand_state.dq[i]) * kRadToDeg, 'f', 2));
        }
        if (tau_labels_[i] != nullptr) {
            tau_labels_[i]->setText(
                QString::number(static_cast<double>(hand_state.tau[i]), 'f', 3));
        }
        if (err_labels_[i] != nullptr) {
            const bool faulted = (hand_state.error_bits & (1u << i)) != 0u;
            err_labels_[i]->setText(faulted ? QStringLiteral("ERR") : QString());
            err_labels_[i]->setStyleSheet(faulted ? QStringLiteral("color: red;") : QString());
        }
    }
}

bool HandAppWindow::readFieldsAsRadians(std::array<float, kNumUiJoints>& out)
{
    for (std::size_t i = 0; i < kNumUiJoints; ++i) {
        if (cmd_fields_[i] == nullptr) {
            return false;
        }
        bool ok = false;
        const double degrees = cmd_fields_[i]->text().toDouble(&ok);
        if (!ok) {
            appendLog(QStringLiteral("command_pos%1 is not a number").arg(static_cast<int>(i)));
            return false;
        }
        out[i] = static_cast<float>(degrees * kDegToRad);
    }
    return true;
}

void HandAppWindow::sendPositions(const std::array<float, kNumUiJoints>& positions_rad,
                                  const QString& label)
{
    hand_sdk::frames::DualHandCommand command;
    hand_sdk::frames::HandCommand hand_command{};
    for (std::size_t i = 0; i < kNumUiJoints; ++i) {
        hand_command.q[i] = positions_rad[i];
    }

    if (hands_.has_left) {
        command.has_left = true;
        command.left = hand_command;
    }
    if (hands_.has_right) {
        command.has_right = true;
        command.right = hand_command;
    }

    const auto result = api_->sendCommand(command);
    appendLog(result.success
                  ? QStringLiteral("%1 sent").arg(label)
                  : QStringLiteral("%1 failed: %2").arg(label, QString::fromStdString(result.msg)));
}

void HandAppWindow::onSendFields()
{
    if (!requireConnected(ui_.result_connect)) {
        return;
    }

    std::array<float, kNumUiJoints> positions{};
    if (!readFieldsAsRadians(positions)) {
        return;
    }
    sendPositions(positions, QStringLiteral("field command"));
}

void HandAppWindow::onSendZero()
{
    if (!requireConnected(ui_.result_connect)) {
        return;
    }

    for (auto* field : cmd_fields_) {
        if (field != nullptr) {
            field->setText(QStringLiteral("0.0"));
        }
    }
    sendPositions(std::array<float, kNumUiJoints>{}, QStringLiteral("zero command"));
}

void HandAppWindow::onDumpJson()
{
    QJsonArray positions;
    for (std::size_t i = 0; i < kNumUiJoints; ++i) {
        if (cmd_fields_[i] == nullptr) {
            continue;
        }
        bool ok = false;
        const double degrees = cmd_fields_[i]->text().toDouble(&ok);
        positions.append(ok ? degrees : 0.0);
    }

    QJsonObject pose;
    pose.insert(QStringLiteral("unit"), QStringLiteral("deg"));
    pose.insert(QStringLiteral("position"), positions);

    const QJsonDocument document(pose);
    ui_.print_readjson->setPlainText(QString::fromUtf8(document.toJson(QJsonDocument::Indented)));
    appendLog(QStringLiteral("pose json generated"));
}

void HandAppWindow::shutdownSdk()
{
    if (poll_timer_ != nullptr) {
        poll_timer_->stop();
    }
    if (api_) {
        api_->disconnect();
        api_.reset();
    }
    hands_ = hand_sdk::HandsStatus{};
}

int main(int argc, char** argv)
{
    QApplication app(argc, argv);

    HandAppWindow window;
    window.show();

    return app.exec();
}
