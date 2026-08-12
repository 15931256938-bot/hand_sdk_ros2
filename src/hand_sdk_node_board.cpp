#include "hand_sdk_node_board.hpp"

#include <QComboBox>
#include <QDateTime>
#include <QPushButton>
#include <QStringList>
#include <QTextEdit>

#include <cmath>

namespace
{
// 10Hz
constexpr int kRefreshMs = 100;

const char* mcStateName(hand_sdk::service::McState state)
{
    switch (state) {
    case hand_sdk::service::McState::kNone:    return "none";
    case hand_sdk::service::McState::kInit:    return "init";
    case hand_sdk::service::McState::kStandby: return "standby";
    case hand_sdk::service::McState::kActive:  return "active";
    case hand_sdk::service::McState::kFault:   return "FAULT";
    }
    return "?";
}

// Only the bits the vendor documents; anything else shows as its raw value.
QString faultNames(uint32_t code)
{
    if (code == hand_sdk::service::fault::kNone) {
        return {};
    }
    struct Bit { uint32_t mask; const char* name; };
    static const Bit kBits[] = {
        {hand_sdk::service::fault::kOverCurrent,  "over-current"},
        {hand_sdk::service::fault::kOverVoltage,  "over-voltage"},
        {hand_sdk::service::fault::kUnderVoltage, "under-voltage"},
        {hand_sdk::service::fault::kStall,        "stall"},
        {hand_sdk::service::fault::kOverTemp,     "over-temp"},
        {hand_sdk::service::fault::kCurrentZero,  "current-zero"},
        {hand_sdk::service::fault::kPosNotSync,   "pos-not-sync"},
        {hand_sdk::service::fault::kOverSpeed,    "over-speed"},
        {hand_sdk::service::fault::kHardware,     "hardware"},
    };

    QStringList names;
    uint32_t seen = 0;
    for (const auto& bit : kBits) {
        if (code & bit.mask) {
            names << bit.name;
            seen |= bit.mask;
        }
    }
    if (const uint32_t rest = code & ~seen) {
        names << QString("0x%1").arg(rest, 0, 16);
    }
    return names.join(",");
}

const char* handLabel(hand_sdk::service::Hand hand)
{
    return hand == hand_sdk::service::Hand::kLeft ? "left" : "right";
}
} // namespace

HandNodeBoard::HandNodeBoard(BoardSource& source)
    : source_(source)
{
    ui_.setupUi(this);
    cacheWidgets();
    ensureCtrlResultLabel();
    wireButtons();

    ui_.render_state_type->setText("--");
    clearStateColumn(hand_sdk::service::Hand::kLeft, "-");
    clearStateColumn(hand_sdk::service::Hand::kRight, "-");

    refresh_timer_ = new QTimer(this);
    refresh_timer_->setInterval(kRefreshMs);
    connect(refresh_timer_, &QTimer::timeout, this, [this]() { onRefresh(); });
    refresh_timer_->start();

    appendLog(QString("board up: left=%1 right=%2")
                  .arg(source_.present(hand_sdk::service::Hand::kLeft) ? "yes" : "no")
                  .arg(source_.present(hand_sdk::service::Hand::kRight) ? "yes" : "no"));
}

// --- setup -----------------------------------------------------------------

void HandNodeBoard::cacheWidgets()
{
    // The readings grid is flat (readpos0..19); the states grid is per finger
    // and joint (state_l_FJ), so it is walked as finger * 4 + joint -- the same
    // index the joint names and the SDK's JointMask use.
    const auto label = [this](const char* format, int index) {
        return findChild<QLabel*>(QString::asprintf(format, index));
    };

    for (std::size_t i = 0; i < kNumBoardJoints; ++i) {
        const int n = static_cast<int>(i);
        pos_labels_[i] = label("readpos%d", n);
        vel_labels_[i] = label("readvel%d", n);
        tau_labels_[i] = label("readtau%d", n);
        err_labels_[i] = label("error%d", n);

        const int finger = n / 4;
        const int joint = n % 4;
        state_left_[i] = findChild<QLabel*>(QString::asprintf("state_l_%d%d", finger, joint));
        state_right_[i] = findChild<QLabel*>(QString::asprintf("state_r_%d%d", finger, joint));
    }
}

// The .ui has no result label for the enable/disable buttons, so one is made
// here -- but only if the .ui has not grown one since. Name it
// `ctrl_cmd_result` there and this picks it up instead of adding a second.
void HandNodeBoard::ensureCtrlResultLabel()
{
    ctrl_result_ = findChild<QLabel*>("ctrl_cmd_result");
    if (ctrl_result_ != nullptr) {
        return;
    }

    ctrl_result_ = new QLabel(ui_.tab);
    ctrl_result_->setObjectName("ctrl_cmd_result");
    // Directly under ctrl_cmd_disable, which sits at (610, 140, 151, 31).
    ctrl_result_->setGeometry(QRect(610, 178, 360, 25));
    ctrl_result_->setText("--");
    ctrl_result_->show();
}

void HandNodeBoard::wireButtons()
{
    connect(ui_.button_mc_state, &QPushButton::clicked, this, [this]() { onMcState(); });
    connect(ui_.button_ctrl_state, &QPushButton::clicked, this, [this]() { onCtrlState(); });
    connect(ui_.ctrl_cmd_enable, &QPushButton::clicked, this, [this]() { onSetEnable(true); });
    connect(ui_.ctrl_cmd_disable, &QPushButton::clicked, this, [this]() { onSetEnable(false); });
    connect(ui_.quit, &QPushButton::clicked, this, [this]() { close(); });
}

// --- states tab -------------------------------------------------------------

template <typename Call>
void HandNodeBoard::forEachHand(const QString& what, Call call)
{
    using hand_sdk::service::Hand;

    bool any = false;
    for (const auto hand : {Hand::kLeft, Hand::kRight}) {
        if (!source_.present(hand)) {
            clearStateColumn(hand, "-");
            continue;
        }
        any = true;
        // ~10 ms on the GUI thread: one bus window, below the blink threshold.
        // It can still fail outright if a ROS set_gains owns the channel, which
        // is reported rather than retried.
        const QString error = call(hand);
        if (error.isEmpty()) {
            appendLog(QString("%1 %2: ok").arg(handLabel(hand), what));
        } else {
            clearStateColumn(hand, "err");
            appendLog(QString("%1 %2 FAILED: %3").arg(handLabel(hand), what, error));
        }
    }
    if (!any) {
        appendLog(QString("%1: no hands present").arg(what));
    }
}

void HandNodeBoard::onMcState()
{
    ui_.render_state_type->setText("mc_state");
    forEachHand("mc_state", [this](hand_sdk::service::Hand hand) {
        const auto result = source_.api()->serviceMcState(hand);
        if (!result.success) {
            return QString::fromStdString(result.msg);
        }
        renderMcState(hand, result.value);
        return QString();
    });
}

void HandNodeBoard::onCtrlState()
{
    ui_.render_state_type->setText("ctrl_state");
    forEachHand("ctrl_state", [this](hand_sdk::service::Hand hand) {
        const auto result = source_.api()->serviceModeQuery(hand);
        if (!result.success) {
            return QString::fromStdString(result.msg);
        }
        renderCtrlState(hand, result.value);
        return QString();
    });
}

void HandNodeBoard::onSetEnable(bool enable)
{
    using hand_sdk::service::Hand;

    const std::size_t joint = selectedJoint();
    const auto mask = hand_sdk::service::JointMask::single(joint);
    const QString what = QString("%1 %2").arg(enable ? "enable" : "disable", jointName(joint));

    // Deliberately leaves the table alone. The table shows the last state you
    // *asked* for with mc_state / ctrl_state; a write that quietly repainted it
    // would blur the two. Press ctrl_state to see the result in the grid.
    QStringList done;
    QStringList failed;
    for (const auto hand : {Hand::kLeft, Hand::kRight}) {
        if (!source_.present(hand)) {
            continue;
        }
        // Only the selected motor is addressed; the other nineteen get the
        // device's "no action" word and keep running exactly as they were.
        const auto result = source_.api()->serviceMode(
            hand, hand_sdk::service::LoopMode::kForceControl, enable, mask);
        if (result.success) {
            done << handLabel(hand);
        } else {
            failed << QString("%1: %2").arg(handLabel(hand), QString::fromStdString(result.msg));
        }
    }

    if (done.isEmpty() && failed.isEmpty()) {
        setCtrlResult(false, QString("%1: no hands present").arg(what));
        return;
    }
    if (failed.isEmpty()) {
        setCtrlResult(true, QString("%1: ok (%2)").arg(what, done.join(", ")));
        return;
    }
    setCtrlResult(false, QString("%1: FAILED %2").arg(what, failed.join("; ")));
}

void HandNodeBoard::setCtrlResult(bool ok, const QString& text)
{
    if (ctrl_result_ != nullptr) {
        ctrl_result_->setText(text);
        ctrl_result_->setStyleSheet(ok ? "color: green;" : "color: red;");
    }
    appendLog(text);
}

QString HandNodeBoard::jointName(std::size_t joint)
{
    return QString("finger%1_joint%2").arg(joint / 4).arg(joint % 4);
}

std::size_t HandNodeBoard::selectedJoint() const
{
    const int finger = ui_.ctrl_finger_index->currentIndex();
    const int joint = ui_.ctrl_joint_index->currentIndex();
    return static_cast<std::size_t>(std::max(finger, 0) * 4 + std::max(joint, 0));
}

std::array<QLabel*, kNumBoardJoints>& HandNodeBoard::stateLabels(hand_sdk::service::Hand hand)
{
    return hand == hand_sdk::service::Hand::kLeft ? state_left_ : state_right_;
}

void HandNodeBoard::renderMcState(hand_sdk::service::Hand hand,
                                  const hand_sdk::service::McStatusArray& status)
{
    auto& labels = stateLabels(hand);
    for (std::size_t i = 0; i < kNumBoardJoints; ++i) {
        if (labels[i] == nullptr) {
            continue;
        }
        const auto& joint = status[i];
        const QString faults = faultNames(joint.fault_code);
        const bool bad = joint.state == hand_sdk::service::McState::kFault || !faults.isEmpty();

        labels[i]->setText(faults.isEmpty()
                               ? QString(mcStateName(joint.state))
                               : QString("%1 (%2)").arg(mcStateName(joint.state), faults));
        labels[i]->setStyleSheet(bad ? "color: red;" : "");
    }
}

void HandNodeBoard::renderCtrlState(hand_sdk::service::Hand hand,
                                    const hand_sdk::service::ModeStatusArray& status)
{
    auto& labels = stateLabels(hand);
    QStringList odd_mode;

    for (std::size_t i = 0; i < kNumBoardJoints; ++i) {
        if (labels[i] == nullptr) {
            continue;
        }
        const auto& joint = status[i];
        // 1 enabled, 0 disabled. Off is not a fault, but it is the thing this
        // screen exists to spot, so it gets the colour.
        labels[i]->setText(joint.enabled ? "1" : "0");
        labels[i]->setStyleSheet(joint.enabled ? "" : "color: red;");

        if (joint.mode == hand_sdk::service::LoopMode::kThreeLoop) {
            odd_mode << QString::number(i);
        }
    }

    // The loop mode is not in the cells any more, so say it here rather than
    // drop it: everything should be force control unless someone is debugging.
    if (!odd_mode.isEmpty()) {
        appendLog(QString("%1: joints %2 are in three-loop mode, not force control")
                      .arg(handLabel(hand), odd_mode.join(",")));
    }
}

void HandNodeBoard::clearStateColumn(hand_sdk::service::Hand hand, const QString& text)
{
    for (auto* label : stateLabels(hand)) {
        if (label != nullptr) {
            label->setText(text);
            label->setStyleSheet("");
        }
    }
}

// --- readings tab -----------------------------------------------------------

void HandNodeBoard::onRefresh()
{
    using hand_sdk::service::Hand;

    // One set of readings, one hand: the grid has no side selector. Left when
    // it is on the bus, otherwise right.
    const Hand hand = source_.present(Hand::kLeft) ? Hand::kLeft : Hand::kRight;
    const BoardSnapshot snapshot = source_.snapshot(hand);

    for (std::size_t i = 0; i < kNumBoardJoints; ++i) {
        if (!snapshot.valid) {
            if (pos_labels_[i] != nullptr) pos_labels_[i]->setText("-");
            if (vel_labels_[i] != nullptr) vel_labels_[i]->setText("-");
            if (tau_labels_[i] != nullptr) tau_labels_[i]->setText("-");
            if (err_labels_[i] != nullptr) err_labels_[i]->setText("-");
            continue;
        }

        // Degrees on screen, radians on the wire.
        if (pos_labels_[i] != nullptr) {
            pos_labels_[i]->setText(QString::number(snapshot.q[i] * 180.0 / M_PI, 'f', 1));
        }
        if (vel_labels_[i] != nullptr) {
            vel_labels_[i]->setText(QString::number(snapshot.dq[i] * 180.0 / M_PI, 'f', 1));
        }
        if (tau_labels_[i] != nullptr) {
            tau_labels_[i]->setText(QString::number(snapshot.tau[i], 'f', 3));
        }
        if (err_labels_[i] != nullptr) {
            const bool faulted = (snapshot.error_bits >> i) & 1u;
            err_labels_[i]->setText(faulted ? "1" : "0");
            err_labels_[i]->setStyleSheet(faulted ? "color: red;" : "");
        }
    }
}

// --- helpers ----------------------------------------------------------------

void HandNodeBoard::appendLog(const QString& text)
{
    ui_.log->append(QDateTime::currentDateTime().toString("HH:mm:ss ") + text);
}
