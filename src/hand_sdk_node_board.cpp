#include "hand_sdk_node_board.hpp"

#include <QCheckBox>
#include <QDateTime>
#include <QLineEdit>
#include <QPushButton>
#include <QStringList>
#include <QTextEdit>

#include <algorithm>
#include <cmath>

namespace
{

constexpr double kRadToDeg = 180.0 / M_PI;

// The readings tab redraws from the node's last published sample. 10 Hz: fast
// enough to watch a motor move, slow enough that the grid is readable.
constexpr int kRefreshMs = 100;

// s16en11 on the wire: at or past +-16 the value saturates at the INT16 rail
// instead of wrapping, so the device would quietly run a gain nobody asked for.
// Refused rather than clamped -- a silently different gain is worse than a
// button that did nothing.
constexpr double kGainWireLimit = 16.0;

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

QString handName(hand_sdk::service::Hand hand)
{
    return hand == hand_sdk::service::Hand::kLeft ? QStringLiteral("left")
                                                  : QStringLiteral("right");
}

QString mcStateName(hand_sdk::service::McState state)
{
    switch (state) {
    case hand_sdk::service::McState::kNone:    return QStringLiteral("none");
    case hand_sdk::service::McState::kInit:    return QStringLiteral("init");
    case hand_sdk::service::McState::kStandby: return QStringLiteral("standby");
    case hand_sdk::service::McState::kActive:  return QStringLiteral("active");
    case hand_sdk::service::McState::kFault:   return QStringLiteral("FAULT");
    }
    return QStringLiteral("unknown");
}

// Only the bits the vendor documents; anything else shows as its raw value.
QString faultNames(uint32_t code)
{
    using namespace hand_sdk::service;

    if (code == fault::kNone) {
        return {};
    }

    struct Bit { uint32_t mask; const char* name; };
    static const Bit kBits[] = {
        {fault::kOverCurrent,  "over-current"},
        {fault::kOverVoltage,  "over-voltage"},
        {fault::kUnderVoltage, "under-voltage"},
        {fault::kStall,        "stall"},
        {fault::kOverTemp,     "over-temp"},
        {fault::kCurrentZero,  "current-zero"},
        {fault::kPosNotSync,   "pos-not-sync"},
        {fault::kOverSpeed,    "over-speed"},
        {fault::kHardware,     "hardware"},
    };

    QStringList names;
    uint32_t seen = 0;
    for (const auto& bit : kBits) {
        if ((code & bit.mask) != 0u) {
            names << QString::fromLatin1(bit.name);
            seen |= bit.mask;
        }
    }
    if (const uint32_t rest = code & ~seen) {
        names << QStringLiteral("0x%1").arg(rest, 0, 16);
    }
    return names.join(QLatin1Char(','));
}

// pos_zero_calibration_state as a grid cell: the raw code the device reported,
// then a word for it. Ten columns of these sit side by side, so the cell stays
// short and the spelled-out name goes in the tooltip.
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
    return QStringLiteral("%1 ?").arg(static_cast<int>(state));
}

} // namespace

HandNodeBoard::HandNodeBoard(BoardSource& source, double initial_kp, double initial_kd)
    : source_(source)
{
    ui_.setupUi(this);
    cacheWidgets();
    wireButtons();

    ui_.kp_input->setText(QString::number(initial_kp, 'g', 6));
    ui_.kd_input->setText(QString::number(initial_kd, 'g', 6));

    // Not a verdict -- nothing has been tried yet, and nothing can be until the
    // bus is up.
    setStateKind(QStringLiteral("--"));
    clearStateGrid(QStringLiteral("-"));
    ui_.ctrl_cmd_result->setText(QStringLiteral("--"));
    ui_.ctrl_cmd_result->setStyleSheet(QStringLiteral("color: gray;"));

    refresh_timer_ = new QTimer(this);
    refresh_timer_->setInterval(kRefreshMs);
    connect(refresh_timer_, &QTimer::timeout, this, [this]() { onRefresh(); });
    refresh_timer_->start();

    appendLog(QStringLiteral("board up, bus down -- press connect"));
}

// --- setup ------------------------------------------------------------------

void HandNodeBoard::cacheWidgets()
{
    // The readings grid is flat (readpos0..19); the states grid is per finger
    // and joint (state_l_FJ, calib_l_FJ), so it is walked as finger * 4 + joint
    // -- the same index the joint names and the SDK's JointMask use, and the
    // same mapping hand_sdk_app uses for the identical grid.
    for (std::size_t i = 0; i < kNumBoardJoints; ++i) {
        const int index = static_cast<int>(i);
        const int finger = index / 4;
        const int joint = index % 4;

        pos_labels_[i] = findChild<QLabel*>(QStringLiteral("readpos%1").arg(index));
        vel_labels_[i] = findChild<QLabel*>(QStringLiteral("readvel%1").arg(index));
        tau_labels_[i] = findChild<QLabel*>(QStringLiteral("readtau%1").arg(index));
        err_labels_[i] = findChild<QLabel*>(QStringLiteral("error%1").arg(index));

        state_left_[i] =
            findChild<QLabel*>(QStringLiteral("state_l_%1%2").arg(finger).arg(joint));
        state_right_[i] =
            findChild<QLabel*>(QStringLiteral("state_r_%1%2").arg(finger).arg(joint));
        check_left_[i] =
            findChild<QCheckBox*>(QStringLiteral("calib_l_%1%2").arg(finger).arg(joint));
        check_right_[i] =
            findChild<QCheckBox*>(QStringLiteral("calib_r_%1%2").arg(finger).arg(joint));
    }
}

void HandNodeBoard::wireButtons()
{
    connect(ui_.button_connect, &QPushButton::clicked, this, [this]() { onConnect(); });
    connect(ui_.button_com, &QPushButton::clicked, this, [this]() { onComm(); });
    connect(ui_.seed_pos, &QPushButton::clicked, this, [this]() { onSeedPos(); });
    connect(ui_.ctrl_cmd_enable, &QPushButton::clicked, this, [this]() { onSetEnable(true); });
    connect(ui_.ctrl_cmd_disable, &QPushButton::clicked, this, [this]() { onSetEnable(false); });

    connect(ui_.button_mc_state, &QPushButton::clicked, this, [this]() { onMcState(); });
    connect(ui_.calibrate_state, &QPushButton::clicked, this, [this]() { onCalibrateState(); });
    // The read-gains button kept uic's default name in the .ui.
    connect(ui_.pushButton, &QPushButton::clicked, this, [this]() { onReadGains(); });

    connect(ui_.zero_calibrate, &QPushButton::clicked, this, [this]() { onZeroCalibrate(); });
    connect(ui_.set_gains, &QPushButton::clicked, this, [this]() { onSetGains(); });

    // Selection only. The boxes carry no handler of their own -- clicking one
    // puts nothing on the wire, so there is nothing to react to.
    connect(ui_.select_all, &QPushButton::clicked, this, [this]() { setAllChecks(true); });
    connect(ui_.clear_all, &QPushButton::clicked, this, [this]() { setAllChecks(false); });

    connect(ui_.quit, &QPushButton::clicked, this, [this]() { close(); });
}

// --- helpers ----------------------------------------------------------------

void HandNodeBoard::appendLog(const QString& text)
{
    const auto stamp = QDateTime::currentDateTime().toString(QStringLiteral("HH:mm:ss.zzz"));
    ui_.log->append(QStringLiteral("[%1] %2").arg(stamp, text));
}

void HandNodeBoard::setCtrlResult(Verdict verdict, const QString& text, const QString& detail)
{
    ui_.ctrl_cmd_result->setText(text);
    ui_.ctrl_cmd_result->setToolTip(detail);
    switch (verdict) {
    case Verdict::kPass:
        ui_.ctrl_cmd_result->setStyleSheet(QStringLiteral("color: green;"));
        break;
    case Verdict::kPartial:
        // Amber, not red: the step did something, and the next one is worth
        // pressing.
        ui_.ctrl_cmd_result->setStyleSheet(QStringLiteral("color: #b8860b;"));
        break;
    case Verdict::kFail:
        ui_.ctrl_cmd_result->setStyleSheet(QStringLiteral("color: red;"));
        break;
    }
    appendLog(detail);
}

bool HandNodeBoard::requireConnected(const QString& what)
{
    if (source_.api() != nullptr &&
        (source_.present(hand_sdk::service::Hand::kLeft) ||
         source_.present(hand_sdk::service::Hand::kRight))) {
        return true;
    }
    appendLog(QStringLiteral("%1 rejected: not connected").arg(what));
    return false;
}

std::vector<hand_sdk::service::Hand> HandNodeBoard::presentHands() const
{
    using hand_sdk::service::Hand;

    std::vector<Hand> hands;
    for (const auto hand : {Hand::kLeft, Hand::kRight}) {
        if (source_.present(hand)) {
            hands.push_back(hand);
        }
    }
    return hands;
}

// --- tab 1: the bus ---------------------------------------------------------

void HandNodeBoard::onConnect()
{
    if (source_.api() != nullptr) {
        const auto summary = QString::fromStdString(source_.disconnectBus());
        ui_.button_connect->setText(QStringLiteral("connect"));
        setStateKind(QStringLiteral("--"));
        clearStateGrid(QStringLiteral("-"));
        ui_.ctrl_cmd_result->setText(QStringLiteral("--"));
        ui_.ctrl_cmd_result->setStyleSheet(QStringLiteral("color: gray;"));
        appendLog(QStringLiteral("disconnected | %1").arg(summary));
        return;
    }

    const auto connected = source_.connectBus();
    if (!connected.success) {
        appendLog(QStringLiteral("connect failed: %1")
                      .arg(QString::fromStdString(connected.msg)));
        return;
    }

    // One hand on the bus is a normal rig, not a half-failure, so this is a
    // plain pass whenever the bus answered at all.
    const auto summary =
        QStringLiteral("left=%1 right=%2 dc=%3")
            .arg(connected.has_left ? QStringLiteral("yes") : QStringLiteral("no"),
                 connected.has_right ? QStringLiteral("yes") : QStringLiteral("no"),
                 connected.dc_active ? QStringLiteral("armed") : QStringLiteral("off"));
    ui_.button_connect->setText(QStringLiteral("disconnect"));
    clearStateGrid(QStringLiteral("-"));
    appendLog(QStringLiteral("connected: %1").arg(summary));

    if (!connected.has_left && !connected.has_right) {
        appendLog(QStringLiteral("bus is up but reported no hands -- check power and cabling"));
    }
}

void HandNodeBoard::onComm()
{
    if (!requireConnected(QStringLiteral("set com all"))) {
        return;
    }

    const auto hands = presentHands();
    std::vector<hand_sdk::service::Hand> started;
    std::vector<QString> notes(hands.size());

    for (std::size_t i = 0; i < hands.size(); ++i) {
        // cmd 4 is broadcast to every joint by construction -- there is no mask
        // on it, so the ticks play no part here. The SDK's message either way:
        // on a pass it says how many motors echoed the command, on a failure
        // which ones did not.
        const auto result =
            source_.api()->serviceComm(hands[i], hand_sdk::service::ComCmd::kStartBroadcast);
        if (result.success) {
            started.push_back(hands[i]);
        }
        notes[i] = QString::fromStdString(result.msg);
    }

    // Seeding runs on every present hand, whatever the broadcast step did, and
    // whatever any joint's calibration state is: seeding latches the pose the
    // device is reporting *now*, in whatever frame it is currently using, so it
    // is always correct at the moment it is taken.
    //
    // What it cannot survive is the frame moving afterwards. Zero-calibrating a
    // joint redefines its origin, so a pose latched before that calibration
    // refers to an origin the device has since dropped. The rule is therefore
    // "seed last", not "seed only calibrated joints" -- press `seed current
    // pos` again after any calibration and before enabling control.
    QString seed_detail;
    const auto seeded = seedHands(hands, seed_detail);

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
    setCtrlResult(verdict, verdictTag(verdict),
                  QStringLiteral("set com: %1 | %2 | %3")
                      .arg(verdictTag(verdict), parts.join(QStringLiteral(" | ")), seed_detail));
}

std::vector<hand_sdk::service::Hand> HandNodeBoard::seedHands(
    const std::vector<hand_sdk::service::Hand>& hands, QString& detail)
{
    if (hands.empty()) {
        detail = QStringLiteral("seed: skipped, no hand to seed");
        return {};
    }

    // Whole hand, not the ticked joints: a command frame carries all twenty-one
    // slots and there is no "no action" word on the data path, so there is no
    // such thing as seeding a subset. The ticks belong to the service writes.
    std::vector<hand_sdk::service::Hand> seeded;
    for (const auto hand : hands) {
        if (source_.seedFromState(hand)) {
            seeded.push_back(hand);
        }
    }

    if (seeded.empty()) {
        detail = QStringLiteral("seed: no state to hold");
        return {};
    }

    QStringList names;
    for (const auto hand : seeded) {
        names << handName(hand);
    }
    detail = QStringLiteral("hold pose seeded (%1)").arg(names.join(QLatin1Char(',')));
    return seeded;
}

void HandNodeBoard::onSeedPos()
{
    if (!requireConnected(QStringLiteral("seed current pos"))) {
        return;
    }

    const auto hands = presentHands();
    QString detail;
    const auto seeded = seedHands(hands, detail);

    const auto verdict = tally(static_cast<int>(seeded.size()), static_cast<int>(hands.size()));
    setCtrlResult(verdict, verdictTag(verdict), QStringLiteral("seed: %1").arg(detail));
}

void HandNodeBoard::onSetEnable(bool enable)
{
    const QString what = enable ? QStringLiteral("enable ctrl") : QStringLiteral("disable ctrl");
    if (!requireConnected(what)) {
        return;
    }

    const auto hands = presentHands();
    QStringList parts;
    int took_total = 0;

    for (const auto hand : hands) {
        // Every active joint, not the ticked ones: turning the hand on is not a
        // per-motor operation, and this is the same call the app's `set ctrl`
        // makes. The ticks are for zero_calibrate and setGains.
        const auto result =
            source_.api()->serviceMode(hand, hand_sdk::service::LoopMode::kForceControl, enable,
                                       hand_sdk::service::JointMask::active());

        for (std::size_t i = 0; i < hand_sdk::frames::kNumActiveJoints; ++i) {
            if (hand_sdk::service::took(result.value[i],
                                        hand_sdk::service::LoopMode::kForceControl, enable)) {
                ++took_total;
            }
        }
        parts << QStringLiteral("%1: %2").arg(handName(hand),
                                              QString::fromStdString(result.msg));
    }

    const auto verdict =
        tally(took_total, static_cast<int>(hands.size() * hand_sdk::frames::kNumActiveJoints));
    // The state grid is deliberately not repainted here: it shows the last
    // thing you asked to *read*, and a write quietly refilling it would blur
    // "what I read" with "what I just did". Press mc_state for the new state.
    setCtrlResult(verdict,
                  enable ? QStringLiteral("enabled") : QStringLiteral("disabled"),
                  QStringLiteral("%1: %2 | %3")
                      .arg(what, verdictTag(verdict), parts.join(QStringLiteral(" | "))));
}

// --- tab 1: the three reads that share the state grid ------------------------

template <typename Call>
void HandNodeBoard::forEachHand(const QString& what, Call call)
{
    using hand_sdk::service::Hand;

    QStringList parts;
    int ok = 0;
    int tried = 0;

    for (const auto hand : {Hand::kLeft, Hand::kRight}) {
        if (!source_.present(hand)) {
            clearStateColumn(hand, QStringLiteral("-"));
            continue;
        }
        ++tried;

        // ~10 ms on the GUI thread: one bus window, below the blink threshold.
        // It can still fail outright if a ROS set_gains owns the service
        // channel, which is reported rather than retried.
        const QString error = call(hand);
        if (error.isEmpty()) {
            ++ok;
            parts << QStringLiteral("%1: ok").arg(handName(hand));
        } else {
            clearStateColumn(hand, QStringLiteral("err"));
            parts << QStringLiteral("%1: %2").arg(handName(hand), error);
        }
    }

    const auto verdict = tally(ok, tried);
    setCtrlResult(verdict, verdictTag(verdict),
                  QStringLiteral("%1: %2 | %3")
                      .arg(what, verdictTag(verdict), parts.join(QStringLiteral(" | "))));
}

void HandNodeBoard::onMcState()
{
    if (!requireConnected(QStringLiteral("mc_state"))) {
        return;
    }

    setStateKind(QStringLiteral("mc_state"));
    forEachHand(QStringLiteral("mc_state"), [this](hand_sdk::service::Hand hand) {
        const auto result = source_.api()->serviceMcState(hand);
        if (!result.success) {
            return QString::fromStdString(result.msg);
        }
        renderMcState(hand, result.value);
        return QString();
    });
}

void HandNodeBoard::onCalibrateState()
{
    if (!requireConnected(QStringLiteral("calibrate state"))) {
        return;
    }

    // The read half of cmd 10, and the only one of the pair that touches the
    // screen: it reads every joint, ticked or not, because this is the state of
    // the hand and not a report on the selection.
    setStateKind(QStringLiteral("calib state"));
    forEachHand(QStringLiteral("calibrate state"), [this](hand_sdk::service::Hand hand) {
        const auto result = source_.api()->serviceZeroCalibrateState(hand);
        if (!result.success) {
            return QString::fromStdString(result.msg);
        }
        renderCalibState(hand, result.value);
        return QString();
    });
}

void HandNodeBoard::onReadGains()
{
    if (!requireConnected(QStringLiteral("read_gains"))) {
        return;
    }

    setStateKind(QStringLiteral("gains kp/kd"));
    forEachHand(QStringLiteral("read gains"), [this](hand_sdk::service::Hand hand) {
        const auto result = source_.api()->serviceGainsQuery(hand);
        if (!result.success) {
            return QString::fromStdString(result.msg);
        }
        renderGains(hand, result.value);
        return QString();
    });
}

// --- tab 1: the two writes aimed at the ticked joints ------------------------

void HandNodeBoard::onZeroCalibrate()
{
    if (!requireConnected(QStringLiteral("zero calibrate"))) {
        return;
    }

    QStringList parts;
    int done = 0;
    int tried = 0;

    for (const auto hand : presentHands()) {
        const auto mask = checkedMask(hand);
        if (!mask.any()) {
            parts << QStringLiteral("%1: nothing ticked").arg(handName(hand));
            continue;
        }
        ++tried;

        // No interlock against force control being enabled on these joints.
        // This is a bench control panel, so each button does its own service
        // call and nothing else. Calibrating a joint that is currently being
        // held will have the loop and the routine fighting over the motor; that
        // is the operator's call to make, not this window's.
        const auto result = source_.api()->serviceZeroCalibrate(
            hand, hand_sdk::service::ZeroCalDir::kSameAsEncoder, mask);
        if (result.success) {
            ++done;
        }
        parts << QStringLiteral("%1: %2 joints, %3")
                     .arg(handName(hand))
                     .arg(static_cast<int>(mask.count()))
                     .arg(QString::fromStdString(result.msg));
    }

    // Neither the ticks nor the grid move. The ticks are the selection that was
    // just written to, and the state is something this call has only asked to
    // start changing -- the routine runs for seconds past the service window.
    // Press `calibrate state` to see where it got to.
    const auto verdict = tally(done, tried);
    setCtrlResult(verdict, verdictTag(verdict),
                  QStringLiteral("zero-cal: %1 | %2")
                      .arg(verdictTag(verdict), parts.join(QStringLiteral(" | "))));
}

void HandNodeBoard::onSetGains()
{
    if (!requireConnected(QStringLiteral("setGains"))) {
        return;
    }

    bool kp_ok = false;
    bool kd_ok = false;
    const double kp = ui_.kp_input->text().toDouble(&kp_ok);
    const double kd = ui_.kd_input->text().toDouble(&kd_ok);
    if (!kp_ok || !kd_ok) {
        setCtrlResult(Verdict::kFail, QStringLiteral("bad kp/kd"),
                      QStringLiteral("gains: kp and kd must be numbers"));
        return;
    }
    if (std::fabs(kp) >= kGainWireLimit || std::fabs(kd) >= kGainWireLimit) {
        setCtrlResult(Verdict::kFail, QStringLiteral("out of range"),
                      QStringLiteral("gains: |kp| and |kd| must be < %1 (s16en11 wire range)")
                          .arg(kGainWireLimit));
        return;
    }

    QStringList parts;
    int done = 0;
    int tried = 0;
    bool wrote = false;

    for (const auto hand : presentHands()) {
        const auto mask = checkedMask(hand);
        if (!mask.any()) {
            parts << QStringLiteral("%1: nothing ticked").arg(handName(hand));
            continue;
        }
        ++tried;

        const auto result = source_.api()->serviceGains(hand, static_cast<float>(kp),
                                                        static_cast<float>(kd), mask);
        if (result.success) {
            ++done;
        }
        wrote = true;
        parts << QStringLiteral("%1: %2 joints, %3")
                     .arg(handName(hand))
                     .arg(static_cast<int>(mask.count()))
                     .arg(QString::fromStdString(result.msg));
    }

    const auto verdict = tally(done, tried);
    const auto detail = QStringLiteral("gains kp=%1 kd=%2: %3 | %4")
                            .arg(kp, 0, 'f', 3)
                            .arg(kd, 0, 'f', 3)
                            .arg(verdictTag(verdict), parts.join(QStringLiteral(" | ")));

    // Refilled from a *fresh read*, not from the write's own reply: a masked
    // write only answers for the joints it addressed, and the rest were sent
    // the ignore word. onReadGains() asks for all twenty properly -- and it
    // overwrites this call's verdict label, so the detail goes in first.
    appendLog(detail);
    if (wrote) {
        onReadGains();
        return;
    }
    setCtrlResult(verdict, verdictTag(verdict), detail);
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

void HandNodeBoard::setAllChecks(bool on)
{
    using hand_sdk::service::Hand;

    // The boxes are a selection the operator owns, so this moves ticks on both
    // hands whether or not either is on the bus.
    for (const auto hand : {Hand::kLeft, Hand::kRight}) {
        for (auto* box : checkBoxes(hand)) {
            if (box != nullptr) {
                box->setChecked(on);
            }
        }
    }
    appendLog(on ? QStringLiteral("selected all joints")
                 : QStringLiteral("cleared all joints"));
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

// --- the state grid ---------------------------------------------------------

void HandNodeBoard::setStateKind(const QString& kind)
{
    ui_.label_40->setText(kind);
}

std::array<QLabel*, kNumBoardJoints>& HandNodeBoard::stateLabels(hand_sdk::service::Hand hand)
{
    return hand == hand_sdk::service::Hand::kLeft ? state_left_ : state_right_;
}

void HandNodeBoard::setCell(hand_sdk::service::Hand hand, std::size_t joint,
                            const QString& text, bool bad, const QString& tip)
{
    auto* label = stateLabels(hand)[joint];
    if (label == nullptr) {
        return;
    }
    label->setText(QStringLiteral("%1: %2").arg(static_cast<int>(joint % 4)).arg(text));
    label->setStyleSheet(bad ? QStringLiteral("color: red;") : QString());
    label->setToolTip(tip);
}

void HandNodeBoard::renderMcState(hand_sdk::service::Hand hand,
                                  const hand_sdk::service::McStatusArray& status)
{
    for (std::size_t i = 0; i < kNumBoardJoints; ++i) {
        const auto& joint = status[i];
        const QString faults = faultNames(joint.fault_code);
        const bool bad = joint.state == hand_sdk::service::McState::kFault || !faults.isEmpty();
        setCell(hand, i,
                faults.isEmpty() ? mcStateName(joint.state)
                                 : QStringLiteral("%1 (%2)").arg(mcStateName(joint.state), faults),
                bad);
    }
}

void HandNodeBoard::renderCalibState(hand_sdk::service::Hand hand,
                                     const hand_sdk::service::ZeroCalStateArray& states)
{
    for (std::size_t i = 0; i < kNumBoardJoints; ++i) {
        const auto state = states[i];
        // Anything but "succeeded" is worth spotting: a joint that has not been
        // zeroed is measuring from an arbitrary boot pose.
        setCell(hand, i, zeroCalTag(state),
                state != hand_sdk::service::ZeroCalState::kSucceeded,
                QString::fromUtf8(hand_sdk::service::stateName(state)));
    }
}

void HandNodeBoard::renderGains(hand_sdk::service::Hand hand,
                                const hand_sdk::service::JointGainsArray& gains)
{
    for (std::size_t i = 0; i < kNumBoardJoints; ++i) {
        const auto& joint = gains[i];
        // Both numbers in one cell: they are set together and only make sense
        // read together. A joint running zero gains is enabled but holding
        // nothing -- it looks like a failed enable, so it gets the colour.
        setCell(hand, i,
                QStringLiteral("%1/%2").arg(joint.kp, 0, 'f', 2).arg(joint.kd, 0, 'f', 2),
                joint.kp == 0.0f && joint.kd == 0.0f);
    }
}

void HandNodeBoard::clearStateColumn(hand_sdk::service::Hand hand, const QString& text)
{
    for (std::size_t i = 0; i < kNumBoardJoints; ++i) {
        setCell(hand, i, text, false);
    }
}

void HandNodeBoard::clearStateGrid(const QString& text)
{
    using hand_sdk::service::Hand;
    for (const auto hand : {Hand::kLeft, Hand::kRight}) {
        clearStateColumn(hand, text);
    }
}

// --- tab 2: what the node is publishing --------------------------------------

void HandNodeBoard::onRefresh()
{
    using hand_sdk::service::Hand;

    // One set of readings, one hand: the grid has no side selector. Left when
    // it is on the bus, otherwise right.
    const Hand hand = source_.present(Hand::kLeft) ? Hand::kLeft : Hand::kRight;
    const BoardSnapshot snapshot = source_.snapshot(hand);

    for (std::size_t i = 0; i < kNumBoardJoints; ++i) {
        if (!snapshot.valid) {
            // Nothing has been published for this hand -- the bus is down, or
            // it is up and not yet cycling. A dash rather than a stale number.
            if (pos_labels_[i] != nullptr) pos_labels_[i]->setText(QStringLiteral("-"));
            if (vel_labels_[i] != nullptr) vel_labels_[i]->setText(QStringLiteral("-"));
            if (tau_labels_[i] != nullptr) tau_labels_[i]->setText(QStringLiteral("-"));
            if (err_labels_[i] != nullptr) {
                err_labels_[i]->setText(QStringLiteral("-"));
                err_labels_[i]->setStyleSheet(QString());
            }
            continue;
        }

        // Degrees on screen, radians on the wire.
        if (pos_labels_[i] != nullptr) {
            pos_labels_[i]->setText(
                QString::number(static_cast<double>(snapshot.q[i]) * kRadToDeg, 'f', 2));
        }
        if (vel_labels_[i] != nullptr) {
            vel_labels_[i]->setText(
                QString::number(static_cast<double>(snapshot.dq[i]) * kRadToDeg, 'f', 2));
        }
        if (tau_labels_[i] != nullptr) {
            tau_labels_[i]->setText(
                QString::number(static_cast<double>(snapshot.tau[i]), 'f', 3));
        }
        if (err_labels_[i] != nullptr) {
            // 1 faulted, 0 healthy -- the same 1/0 hand_sdk_app shows, so an
            // empty cell always means "not read", never "no fault".
            const bool faulted = (snapshot.error_bits & (1u << i)) != 0u;
            err_labels_[i]->setText(faulted ? QStringLiteral("1") : QStringLiteral("0"));
            err_labels_[i]->setStyleSheet(faulted ? QStringLiteral("color: red;") : QString());
        }
    }
}
