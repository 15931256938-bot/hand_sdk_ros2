#include "hand_sdk_node_board.hpp"

#include <QCheckBox>
#include <QLineEdit>
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

// pos_zero_calibration_state as a checkbox caption: the raw code the device
// reported, then a word for it. Ten columns of these sit side by side, so the
// caption stays short and the spelled-out name goes in the tooltip.
QString zeroCalTag(hand_sdk::service::ZeroCalState state)
{
    switch (state) {
    case hand_sdk::service::ZeroCalState::kFailed:        return QStringLiteral("-1 FAIL");
    case hand_sdk::service::ZeroCalState::kNotCalibrated: return QStringLiteral("0 none");
    case hand_sdk::service::ZeroCalState::kInProgress:    return QStringLiteral("1 busy");
    case hand_sdk::service::ZeroCalState::kSucceeded:     return QStringLiteral("2 ok");
    }
    // A code outside the vendor table: shown raw rather than rounded to the
    // nearest known one.
    return QString("%1 ?").arg(static_cast<int>(state));
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
        check_left_[i] = findChild<QCheckBox*>(QString::asprintf("check_l_%d%d", finger, joint));
        check_right_[i] = findChild<QCheckBox*>(QString::asprintf("check_r_%d%d", finger, joint));
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
    connect(ui_.set_com_all_button, &QPushButton::clicked, this, [this]() { onSetComAll(); });
    connect(ui_.zero_calibrate, &QPushButton::clicked, this, [this]() { onZeroCalibrate(); });
    connect(ui_.calibrate_state, &QPushButton::clicked, this, [this]() { onCalibrateState(); });
    connect(ui_.seed_pos, &QPushButton::clicked, this, [this]() { onSeedPos(); });
    // The read-gains button kept uic's default name in the .ui.
    connect(ui_.pushButton, &QPushButton::clicked, this, [this]() { onReadGains(); });
    connect(ui_.set_gains, &QPushButton::clicked, this, [this]() { onSetGains(); });

    // Selection only. The boxes carry no handler of their own -- nothing the
    // device says is allowed to move a tick, so there is nothing to react to.
    connect(ui_.select_all, &QPushButton::clicked, this, [this]() { setAllChecks(true); });
    connect(ui_.clear_all, &QPushButton::clicked, this, [this]() { setAllChecks(false); });

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

// --- the tick table ---------------------------------------------------------

std::array<QCheckBox*, kNumBoardJoints>& HandNodeBoard::checkBoxes(hand_sdk::service::Hand hand)
{
    return hand == hand_sdk::service::Hand::kLeft ? check_left_ : check_right_;
}

const std::array<QCheckBox*, kNumBoardJoints>& HandNodeBoard::checkBoxes(
    hand_sdk::service::Hand hand) const
{
    return hand == hand_sdk::service::Hand::kLeft ? check_left_ : check_right_;
}

// Captions deliberately untouched: they are what `calibrate state` last read,
// and re-selecting which motors to write to does not make that stale.
void HandNodeBoard::setAllChecks(bool on)
{
    using hand_sdk::service::Hand;
    for (const auto hand : {Hand::kLeft, Hand::kRight}) {
        for (auto* box : checkBoxes(hand)) {
            if (box != nullptr) {
                box->setChecked(on);
            }
        }
    }
    appendLog(on ? QString("selected all joints") : QString("cleared all joints"));
}

hand_sdk::service::JointMask HandNodeBoard::checkedMask(hand_sdk::service::Hand hand) const
{
    hand_sdk::service::JointMask mask;
    const auto& boxes = checkBoxes(hand);
    for (std::size_t i = 0; i < kNumBoardJoints; ++i) {
        if (boxes[i] != nullptr && boxes[i]->isChecked()) {
            mask.set(i);
        }
    }
    return mask;
}

void HandNodeBoard::setCheckCaptions(hand_sdk::service::Hand hand,
                                     const QString& text, const QString& tip)
{
    for (auto* box : checkBoxes(hand)) {
        if (box != nullptr) {
            box->setText(text);
            box->setToolTip(tip);
        }
    }
}

// --- writes against the ticked joints ---------------------------------------

void HandNodeBoard::onSetEnable(bool enable)
{
    using hand_sdk::service::Hand;

    const QString what = enable ? QStringLiteral("enable") : QStringLiteral("disable");

    QStringList done;
    QStringList failed;
    int written = 0;

    for (const auto hand : {Hand::kLeft, Hand::kRight}) {
        if (!source_.present(hand)) {
            continue;
        }

        const auto mask = checkedMask(hand);
        if (!mask.any()) {
            failed << QString("%1: nothing ticked").arg(handLabel(hand));
            continue;
        }

        // Only the ticked motors are addressed; the rest get the device's "no
        // action" word and keep running exactly as they were.
        const auto result = source_.api()->serviceMode(
            hand, hand_sdk::service::LoopMode::kForceControl, enable, mask);
        ++written;

        appendLog(QString("%1 %2 %3 joints: %4")
                      .arg(handLabel(hand), what)
                      .arg(static_cast<int>(mask.count()))
                      .arg(QString::fromStdString(result.msg)));

        if (result.success) {
            done << QString("%1 %2j").arg(handLabel(hand)).arg(static_cast<int>(mask.count()));
        } else {
            failed << QString("%1: %2").arg(handLabel(hand), QString::fromStdString(result.msg));
        }
    }

    if (done.isEmpty() && failed.isEmpty()) {
        setCtrlResult(false, QString("%1: no hands present").arg(what));
        return;
    }

    // The cells are refilled from a *fresh read*, not from the write's own
    // reply. A masked write only answers for the joints it addressed: the rest
    // were sent the ignore word, and a slot that echoes 0xFFFF back decodes as
    // mode 0xFF with the enable bit set -- which would paint an untouched joint
    // as enabled. onCtrlState() asks for all twenty properly.
    if (written > 0) {
        onCtrlState();
    }

    if (failed.isEmpty()) {
        setCtrlResult(true, QString("%1: ok (%2)").arg(what, done.join(", ")));
        return;
    }
    setCtrlResult(false, QString("%1: %2%3")
                             .arg(what, failed.join("; "),
                                  done.isEmpty() ? QString() : QString(" | ok: %1").arg(done.join(", "))));
}

void HandNodeBoard::onSetComAll()
{
    using hand_sdk::service::Hand;

    QStringList done;
    QStringList failed;
    for (const auto hand : {Hand::kLeft, Hand::kRight}) {
        if (!source_.present(hand)) {
            continue;
        }
        // cmd 4 is broadcast to every joint by construction -- there is no mask
        // on it, so the ticks play no part here.
        const auto result = source_.api()->serviceComm(hand, hand_sdk::service::ComCmd::kStartBroadcast);
        appendLog(QString("%1 set com: %2").arg(handLabel(hand), QString::fromStdString(result.msg)));
        if (result.success) {
            done << handLabel(hand);
        } else {
            failed << QString("%1: %2").arg(handLabel(hand), QString::fromStdString(result.msg));
        }
    }

    if (done.isEmpty() && failed.isEmpty()) {
        setCtrlResult(false, QStringLiteral("set com: no hands present"));
        return;
    }
    if (failed.isEmpty()) {
        setCtrlResult(true, QString("set com: ok (%1)").arg(done.join(", ")));
        return;
    }
    setCtrlResult(false, QString("set com: %1").arg(failed.join("; ")));
}

void HandNodeBoard::onZeroCalibrate()
{
    using hand_sdk::service::Hand;

    QStringList done;
    QStringList failed;

    for (const auto hand : {Hand::kLeft, Hand::kRight}) {
        if (!source_.present(hand)) {
            continue;
        }

        const auto mask = checkedMask(hand);
        if (!mask.any()) {
            failed << QString("%1: nothing ticked").arg(handLabel(hand));
            continue;
        }

        const auto result = source_.api()->serviceZeroCalibrate(
            hand, hand_sdk::service::ZeroCalDir::kSameAsEncoder, mask);

        appendLog(QString("%1 zero-cal %2 joints: %3")
                      .arg(handLabel(hand))
                      .arg(static_cast<int>(mask.count()))
                      .arg(QString::fromStdString(result.msg)));

        if (result.success) {
            done << QString("%1 %2").arg(handLabel(hand)).arg(static_cast<int>(mask.count()));
        } else {
            failed << QString("%1: %2").arg(handLabel(hand), QString::fromStdString(result.msg));
        }
    }

    // Neither the ticks nor the captions move. The ticks are the selection that
    // was just written to, and the caption is the *state*, which this call has
    // only asked to start changing -- the routine runs for seconds past the
    // service window. Press `calibrate state` to see where it got to.
    if (done.isEmpty() && failed.isEmpty()) {
        setCtrlResult(false, QStringLiteral("zero-cal: no hands present"));
        return;
    }
    if (failed.isEmpty()) {
        setCtrlResult(true, QString("zero-cal: started (%1)").arg(done.join(", ")));
        return;
    }
    setCtrlResult(false, QString("zero-cal: %1").arg(failed.join("; ")));
}

void HandNodeBoard::onSeedPos()
{
    using hand_sdk::service::Hand;

    QStringList done;
    QStringList failed;

    for (const auto hand : {Hand::kLeft, Hand::kRight}) {
        if (!source_.present(hand)) {
            continue;
        }
        // Whole hand, not the ticked joints: a command frame carries all
        // twenty-one slots and there is no "no action" word on the data path,
        // so there is no such thing as seeding a subset. The ticks belong to
        // the service writes.
        if (source_.seedFromState(hand)) {
            done << handLabel(hand);
        } else {
            failed << handLabel(hand);
        }
    }

    if (done.isEmpty() && failed.isEmpty()) {
        setCtrlResult(false, QStringLiteral("seed: no hands present"));
        return;
    }
    if (failed.isEmpty()) {
        const auto text = QString("seed: hold pose latched (%1)").arg(done.join(", "));
        setCtrlResult(true, text);
        appendLog(text);
        return;
    }
    const auto text = QString("seed: FAILED %1%2")
                          .arg(failed.join(", "),
                               done.isEmpty() ? QString() : QString(" | ok: %1").arg(done.join(", ")));
    setCtrlResult(false, text);
    appendLog(text);
}

void HandNodeBoard::onReadGains()
{
    ui_.render_state_type->setText("gains kp/kd");
    forEachHand("gains", [this](hand_sdk::service::Hand hand) {
        const auto result = source_.api()->serviceGainsQuery(hand);
        if (!result.success) {
            return QString::fromStdString(result.msg);
        }
        renderGains(hand, result.value);
        return QString();
    });
}

void HandNodeBoard::onSetGains()
{
    using hand_sdk::service::Hand;

    bool kp_ok = false;
    bool kd_ok = false;
    const double kp = ui_.kp_input->text().toDouble(&kp_ok);
    const double kd = ui_.kd_input->text().toDouble(&kd_ok);
    if (!kp_ok || !kd_ok) {
        setCtrlResult(false, QStringLiteral("gains: kp and kd must be numbers"));
        return;
    }

    // s16en11 on the wire: at or past +-16 the value saturates at the INT16
    // rail instead of wrapping, so the device would quietly run a gain nobody
    // asked for. Refused rather than clamped -- a silently different gain is
    // worse than a button that did nothing.
    constexpr double kWireLimit = 16.0;
    if (std::fabs(kp) >= kWireLimit || std::fabs(kd) >= kWireLimit) {
        setCtrlResult(false, QString("gains: |kp| and |kd| must be < %1 (s16en11 wire range)")
                                 .arg(kWireLimit));
        return;
    }

    QStringList done;
    QStringList failed;
    int written = 0;

    for (const auto hand : {Hand::kLeft, Hand::kRight}) {
        if (!source_.present(hand)) {
            continue;
        }

        const auto mask = checkedMask(hand);
        if (!mask.any()) {
            failed << QString("%1: nothing ticked").arg(handLabel(hand));
            continue;
        }

        const auto result = source_.api()->serviceGains(
            hand, static_cast<float>(kp), static_cast<float>(kd), mask);
        ++written;

        appendLog(QString("%1 gains kp=%2 kd=%3 on %4 joints: %5")
                      .arg(handLabel(hand))
                      .arg(kp, 0, 'f', 3)
                      .arg(kd, 0, 'f', 3)
                      .arg(static_cast<int>(mask.count()))
                      .arg(QString::fromStdString(result.msg)));

        if (result.success) {
            done << QString("%1 %2j").arg(handLabel(hand)).arg(static_cast<int>(mask.count()));
        } else {
            failed << QString("%1: %2").arg(handLabel(hand), QString::fromStdString(result.msg));
        }
    }

    if (done.isEmpty() && failed.isEmpty()) {
        setCtrlResult(false, QStringLiteral("gains: no hands present"));
        return;
    }

    // Same rule as enable/disable: a masked write only answers for the joints
    // it addressed, so the grid is refilled from a real read of all twenty.
    if (written > 0) {
        onReadGains();
    }

    if (failed.isEmpty()) {
        setCtrlResult(true, QString("gains kp=%1 kd=%2: ok (%3)")
                                .arg(kp, 0, 'f', 3).arg(kd, 0, 'f', 3).arg(done.join(", ")));
        return;
    }
    setCtrlResult(false, QString("gains: %1").arg(failed.join("; ")));
}

void HandNodeBoard::onCalibrateState()
{
    using hand_sdk::service::Hand;

    QStringList parts;
    bool any = false;

    for (const auto hand : {Hand::kLeft, Hand::kRight}) {
        if (!source_.present(hand)) {
            setCheckCaptions(hand, QStringLiteral("-"), QStringLiteral("hand not present"));
            continue;
        }
        any = true;

        // Reads every joint, ticked or not: this is the state of the hand, not
        // a report on the selection.
        const auto result = source_.api()->serviceZeroCalibrateState(hand);
        if (!result.success) {
            const auto why = QString::fromStdString(result.msg);
            setCheckCaptions(hand, QStringLiteral("ERR"), why);
            parts << QString("%1: %2").arg(handLabel(hand), why);
            appendLog(QString("%1 calib state FAILED: %2").arg(handLabel(hand), why));
            continue;
        }

        // Captions only. A tick means "this motor is selected", never "this
        // motor is calibrated", so a read has no business moving one.
        auto& boxes = checkBoxes(hand);
        for (std::size_t i = 0; i < kNumBoardJoints; ++i) {
            if (boxes[i] == nullptr) {
                continue;
            }
            boxes[i]->setText(zeroCalTag(result.value[i]));
            boxes[i]->setToolTip(
                QString::fromUtf8(hand_sdk::service::stateName(result.value[i])));
        }

        parts << QString("%1: %2").arg(handLabel(hand), QString::fromStdString(result.msg));
        appendLog(QString("%1 calib state: %2")
                      .arg(handLabel(hand), QString::fromStdString(result.msg)));
    }

    if (!any) {
        setCtrlResult(false, QStringLiteral("calib state: no hands present"));
        return;
    }
    setCtrlResult(true, QString("calib state | %1").arg(parts.join(QStringLiteral(" | "))));
}

void HandNodeBoard::setCtrlResult(bool ok, const QString& text)
{
    if (ctrl_result_ != nullptr) {
        ctrl_result_->setText(text);
        ctrl_result_->setStyleSheet(ok ? "color: green;" : "color: red;");
    }
    appendLog(text);
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

// The third thing the state grid can be showing. Both numbers in one cell:
// they are set together and only make sense read together.
void HandNodeBoard::renderGains(hand_sdk::service::Hand hand,
                                const hand_sdk::service::JointGainsArray& gains)
{
    auto& labels = stateLabels(hand);
    for (std::size_t i = 0; i < kNumBoardJoints; ++i) {
        if (labels[i] == nullptr) {
            continue;
        }
        const auto& joint = gains[i];
        labels[i]->setText(QString("%1/%2")
                               .arg(joint.kp, 0, 'f', 2)
                               .arg(joint.kd, 0, 'f', 2));
        // A joint running zero gains is enabled but holding nothing -- it looks
        // like a failed enable and is the thing worth spotting here, so it gets
        // the colour the other two renders use for "not what you want".
        labels[i]->setStyleSheet((joint.kp == 0.0f && joint.kd == 0.0f) ? "color: red;" : "");
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
