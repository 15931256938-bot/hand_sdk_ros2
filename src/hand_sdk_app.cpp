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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

namespace
{

constexpr double kRadToDeg = 180.0 / M_PI;
constexpr double kDegToRad = M_PI / 180.0;

// Every position request from the readings tab carries this commanded velocity
// and a zero feedforward torque. The device closes
//   tau = kp*(q_d - q) + kd*(dq_d - dq) + tau_ff
// so a dq_d of zero asks the motor to brake against its own motion for the
// whole move. Same number, and the same reasoning, as hand_control_lib's
// motion.default_speed. One knob: change it here.
constexpr float kCommandVelocity = 0.1f;   // rad/s
constexpr float kCommandTorqueFf = 0.0f;   // N*m

// How close a gain readback has to be to count as set lives in
// hand_sdk::Config::service_tolerance -- the SDK judges by it, so this screen
// does too.

// How many of a hand's motors answered, as a verdict: all of them, some of
// them, or none.
Verdict tally(int reached, int total)
{
    if (total <= 0) {
        return Verdict::kFail;   // nothing was even attempted
    }
    if (reached >= total) {
        return Verdict::kPass;
    }
    return reached > 0 ? Verdict::kPartial : Verdict::kFail;
}

bool contains(const std::vector<hand_sdk::service::Hand>& hands, hand_sdk::service::Hand hand)
{
    return std::find(hands.begin(), hands.end(), hand) != hands.end();
}

QString verdictTag(Verdict verdict)
{
    switch (verdict) {
    case Verdict::kPass:    return QStringLiteral("OK");
    case Verdict::kPartial: return QStringLiteral("PARTIAL");
    case Verdict::kFail:    return QStringLiteral("FAILED");
    }
    return QStringLiteral("?");
}

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

    // Not a verdict -- nothing has been tried yet.
    ui_.result_connect->setText(QStringLiteral("not connected"));
    ui_.result_connect->setStyleSheet(QStringLiteral("color: gray;"));
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

void HandAppWindow::setResult(QLabel* label, Verdict verdict, const QString& text)
{
    label->setText(text);
    switch (verdict) {
    case Verdict::kPass:
        label->setStyleSheet(QStringLiteral("color: green;"));
        break;
    case Verdict::kPartial:
        // Amber, not red: the step did something, and the next one is worth
        // pressing.
        label->setStyleSheet(QStringLiteral("color: #b8860b;"));
        break;
    case Verdict::kFail:
        label->setStyleSheet(QStringLiteral("color: red;"));
        break;
    }
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
    setResult(result, Verdict::kFail, QStringLiteral("connect first"));
    appendLog(QStringLiteral("rejected: not connected"));
    return false;
}

void HandAppWindow::onConnect()
{
    if (api_) {
        shutdownSdk();
        ui_.result_connect->setText(QStringLiteral("disconnected"));
        ui_.result_connect->setStyleSheet(QStringLiteral("color: gray;"));
        appendLog(QStringLiteral("disconnected"));
        return;
    }

    api_ = std::make_unique<hand_sdk::Api>(config_);
    const auto connected = api_->connect();
    if (!connected.success) {
        api_.reset();
        setResult(ui_.result_connect, Verdict::kFail, QString::fromStdString(connected.msg));
        appendLog(QStringLiteral("connect failed: %1").arg(QString::fromStdString(connected.msg)));
        return;
    }

    hands_ = connected.value;
    const auto summary = QStringLiteral("left=%1 right=%2 dc=%3")
                             .arg(hands_.has_left ? QStringLiteral("yes") : QStringLiteral("no"),
                                  hands_.has_right ? QStringLiteral("yes") : QStringLiteral("no"),
                                  api_->dcActive() ? QStringLiteral("armed") : QStringLiteral("off"));
    // One hand on the bus is a normal rig, not a half-failure, so this stays a
    // plain pass whenever the bus answered at all.
    setResult(ui_.result_connect, Verdict::kPass, summary);
    appendLog(QStringLiteral("connected: %1").arg(summary));
    poll_timer_->start();
}

void HandAppWindow::onComm()
{
    if (!requireConnected(ui_.result_com)) {
        return;
    }

    const auto hands = presentHands();
    std::vector<hand_sdk::service::Hand> started;
    std::vector<QString> notes(hands.size());

    for (std::size_t i = 0; i < hands.size(); ++i) {
        // The SDK's message either way: on a pass it says how many motors
        // echoed the command, on a failure which ones did not.
        const auto result = api_->serviceComm(hands[i], hand_sdk::service::ComCmd::kStartBroadcast);
        if (result.success) {
            started.push_back(hands[i]);
        }
        notes[i] = QString::fromStdString(result.msg);
    }

    // Seeding runs on every present hand, whatever the broadcast step did: a
    // hand that is already listening from an earlier run still needs a pose
    // latched, and getting one costs nothing if it is not.
    QString seed_detail;
    const auto seeded = seedCommandFromState(hands, seed_detail);

    int passed = 0;
    QStringList parts;
    for (std::size_t i = 0; i < hands.size(); ++i) {
        const bool did_start = contains(started, hands[i]);
        const bool did_seed = contains(seeded, hands[i]);
        if (did_start && did_seed) {
            ++passed;
        }
        // Both halves always shown, so "broadcast on, NOT seeded" and "timed
        // out, seeded" are told apart at a glance. Only a hand with both counts
        // as passed.
        parts << QStringLiteral("%1: %2, %3")
                     .arg(handName(hands[i]), notes[i],
                          did_seed ? QStringLiteral("seeded") : QStringLiteral("NOT seeded"));
    }

    // One hand broadcasting and holding its pose is enough to go on with; that
    // hand can be driven while the other is looked at.
    const auto verdict = tally(passed, static_cast<int>(hands.size()));
    const auto text = QStringLiteral("%1 | %2 | %3")
                          .arg(verdictTag(verdict), parts.join(QStringLiteral(" | ")), seed_detail);
    setResult(ui_.result_com, verdict, text);
    appendLog(QStringLiteral("comm: %1").arg(text));
}

std::vector<hand_sdk::service::Hand> HandAppWindow::seedCommandFromState(
    const std::vector<hand_sdk::service::Hand>& hands, QString& detail)
{
    using namespace std::chrono_literals;

    if (hands.empty()) {
        detail = QStringLiteral("seed: skipped, no hand to seed");
        return {};
    }

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
        return {};
    }

    // Per hand, not the shared pose sendPositions() uses: the two hands are in
    // different poses, and seeding one from the other's would be a lurch.
    hand_sdk::frames::DualHandCommand command;
    std::vector<hand_sdk::service::Hand> seeded;
    for (const auto hand : hands) {
        if (hand == hand_sdk::service::Hand::kLeft && state.has_left) {
            command.has_left = true;
            command.left.q = state.left.q;  // dq / tau stay zero
            seeded.push_back(hand);
        } else if (hand == hand_sdk::service::Hand::kRight && state.has_right) {
            command.has_right = true;
            command.right.q = state.right.q;
            seeded.push_back(hand);
        }
    }

    if (seeded.empty()) {
        detail = QStringLiteral("seed: hand missing from state");
        return {};
    }

    // One write covers both sides, so a failure here un-seeds every hand in it
    // rather than leaving some of them claimed.
    const auto sent = api_->sendCommand(command);
    if (!sent.success) {
        detail = QStringLiteral("seed: %1").arg(QString::fromStdString(sent.msg));
        return {};
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

    QStringList names;
    for (const auto hand : seeded) {
        names << handName(hand);
    }
    detail = QStringLiteral("hold pose seeded (%1)").arg(names.join(QLatin1Char(',')));
    return seeded;
}

void HandAppWindow::onCtrl()
{
    if (!requireConnected(ui_.result_ctrl)) {
        return;
    }

    const auto hands = presentHands();
    QStringList parts;
    int enabled_total = 0;

    for (const auto hand : hands) {
        const auto result = api_->serviceMode(hand, hand_sdk::service::LoopMode::kForceControl,
                                              true, hand_sdk::service::JointMask::active());

        for (std::size_t i = 0; i < hand_sdk::frames::kNumActiveJoints; ++i) {
            if (hand_sdk::service::took(result.value[i],
                                        hand_sdk::service::LoopMode::kForceControl, true)) {
                ++enabled_total;
            }
        }
        parts << QStringLiteral("%1: %2").arg(handName(hand),
                                              QString::fromStdString(result.msg));
    }

    const auto verdict =
        tally(enabled_total, static_cast<int>(hands.size() * hand_sdk::frames::kNumActiveJoints));
    const auto text = QStringLiteral("%1 | %2").arg(verdictTag(verdict), parts.join(QStringLiteral(" | ")));
    setResult(ui_.result_ctrl, verdict, text);
    appendLog(QStringLiteral("ctrl: %1").arg(text));
}

void HandAppWindow::onMcState()
{
    if (!requireConnected(ui_.result_mc_state)) {
        return;
    }

    resetMcLabels(mc_left_labels_);
    resetMcLabels(mc_right_labels_);

    const auto hands = presentHands();
    QStringList parts;
    int ready_total = 0;

    for (const auto hand : hands) {
        const auto result = api_->serviceMcState(hand);
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

        ready_total += ready;
        parts << QStringLiteral("%1: ready %2/%3%4")
                     .arg(handName(hand))
                     .arg(ready)
                     .arg(static_cast<int>(kNumUiJoints))
                     .arg(faults.isEmpty()
                              ? QString()
                              : QStringLiteral(" [%1]").arg(faults.join(QLatin1Char(','))));
    }

    // One motor in standby is enough to carry on with: the rest of the bring-up
    // will simply do nothing for the motors that are not there.
    const auto verdict = tally(ready_total, static_cast<int>(hands.size() * kNumUiJoints));
    const auto text = QStringLiteral("%1 | %2").arg(verdictTag(verdict), parts.join(QStringLiteral(" | ")));
    setResult(ui_.result_mc_state, verdict, text);
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
        setResult(ui_.result_kpkd, Verdict::kFail, QStringLiteral("kp/kd must be numbers"));
        return;
    }

    const auto hands = presentHands();
    QStringList parts;
    int set_total = 0;

    for (const auto hand : hands) {
        const auto result = api_->serviceGains(hand, static_cast<float>(kp), static_cast<float>(kd),
                                               hand_sdk::service::JointMask::active());

        // Counted with the SDK's own tolerance, so the verdict on screen and the
        // one in the message are the same judgement.
        for (std::size_t i = 0; i < hand_sdk::frames::kNumActiveJoints; ++i) {
            if (hand_sdk::service::took(result.value[i], static_cast<float>(kp),
                                        static_cast<float>(kd), config_.service_tolerance)) {
                ++set_total;
            }
        }
        parts << QStringLiteral("%1: %2").arg(handName(hand),
                                              QString::fromStdString(result.msg));
    }

    const auto verdict =
        tally(set_total, static_cast<int>(hands.size() * hand_sdk::frames::kNumActiveJoints));
    const auto text = QStringLiteral("%1 | %2").arg(verdictTag(verdict), parts.join(QStringLiteral(" | ")));
    setResult(ui_.result_kpkd, verdict, text);
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
            // 1 faulted, 0 healthy -- the same 1/0 the node's board shows, so
            // an empty cell always means "not read", never "no fault".
            const bool faulted = (hand_state.error_bits & (1u << i)) != 0u;
            err_labels_[i]->setText(faulted ? QStringLiteral("1") : QStringLiteral("0"));
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
        hand_command.dq[i] = kCommandVelocity;
        hand_command.tau[i] = kCommandTorqueFf;
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
                  ? QStringLiteral("%1 sent (dq=%2 deg/s, tau_ff=%3)")
                        .arg(label)
                        .arg(static_cast<double>(kCommandVelocity) * kRadToDeg, 0, 'f', 1)
                        .arg(static_cast<double>(kCommandTorqueFf), 0, 'f', 3)
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
