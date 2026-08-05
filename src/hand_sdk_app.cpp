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

#include <cmath>

namespace
{

constexpr double kRadToDeg = 180.0 / M_PI;
constexpr double kDegToRad = M_PI / 180.0;

QString mcStateName(hand_sdk::service::McState state)
{
    switch (state) {
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
        pos_labels_[i] = findChild<QLabel*>(QStringLiteral("readpos%1").arg(index));
        vel_labels_[i] = findChild<QLabel*>(QStringLiteral("readvel%1").arg(index));
        tau_labels_[i] = findChild<QLabel*>(QStringLiteral("readtau%1").arg(index));
        err_labels_[i] = findChild<QLabel*>(QStringLiteral("error%1").arg(index));
        cmd_fields_[i] = findChild<QLineEdit*>(QStringLiteral("command_pos%1").arg(index));
        if (cmd_fields_[i] != nullptr) {
            cmd_fields_[i]->setText(QStringLiteral("0.0"));
        }
    }
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

    const auto text = parts.join(QStringLiteral(" | "));
    setResult(ui_.result_com, all_ok, text);
    appendLog(QStringLiteral("comm: %1").arg(text));
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
