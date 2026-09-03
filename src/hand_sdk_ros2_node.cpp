#include <hand_sdk.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <hand_sdk_ros2/srv/set_gains.hpp>

#include "hand_sdk_node_board.hpp"

#include <QApplication>

#include <array>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace
{

using namespace std::chrono_literals;

constexpr std::size_t kNumSdkJoints = hand_sdk::frames::kNumJoints;
constexpr std::size_t kNumRosJoints = kNumSdkJoints - 1;

// Loop gains ride the service channel as s16en11 fixed point, so |gain| < 16.
constexpr float kGainWireLimit = 16.0f;

// ROS exposes only the 20 active joints. The SDK keeps one reserved slot at
// the end of its 21-element EtherCAT array for future use.
const std::array<std::string, kNumRosJoints> kJointNames = {
    "finger0_joint0",
    "finger0_joint1",
    "finger0_joint2",
    "finger0_joint3",
    "finger1_joint0",
    "finger1_joint1",
    "finger1_joint2",
    "finger1_joint3",
    "finger2_joint0",
    "finger2_joint1",
    "finger2_joint2",
    "finger2_joint3",
    "finger3_joint0",
    "finger3_joint1",
    "finger3_joint2",
    "finger3_joint3",
    "finger4_joint0",
    "finger4_joint1",
    "finger4_joint2",
    "finger4_joint3",
};

std::vector<std::string> toVector(const std::array<std::string, kNumRosJoints>& names)
{
    return std::vector<std::string>(names.begin(), names.end());
}

std::vector<double> toDoubleVector(const std::array<float, kNumSdkJoints>& values)
{
    std::vector<double> result;
    result.reserve(kNumRosJoints);
    for (std::size_t i = 0; i < kNumRosJoints; ++i) {
        result.push_back(static_cast<double>(values[i]));
    }
    return result;
}

std::string describeSlave(const hand_sdk::SlaveStatus& status)
{
    if (!status.present) {
        return "absent";
    }
    std::ostringstream text;
    text << status.state_text << " (AL 0x" << std::hex << status.al_state
         << ", code 0x" << status.al_status_code << std::dec << ")";
    if (!status.error_text.empty()) {
        text << " -- " << status.error_text;
    }
    return text.str();
}

using SetGains = hand_sdk_ros2::srv::SetGains;

struct SideState
{
    // Read from the publish timer, the service callbacks and the Qt thread;
    // written by connect/disconnect on the Qt thread.
    std::atomic<bool> present{false};
    hand_sdk::service::Hand hand{hand_sdk::service::Hand::kLeft};
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr command_sub;
    rclcpp::Service<SetGains>::SharedPtr gains_srv;
    hand_sdk::frames::HandCommand command{};
};

} // namespace

// The ROS half of the hand: joint commands in, joint states out, and nothing
// else. Everything that is not the data path -- bringing the bus up, opening
// communication, zero calibration, gains, enabling force control -- is pressed
// from the board, which is why this constructor no longer touches the hardware.
//
// The node therefore starts with no bus at all. Publishers, subscriptions and
// services are all up from the first instant; they simply have nothing to talk
// to until somebody presses `connect`, and say so rather than failing silently.
class HandSdkRos2Node final : public rclcpp::Node, public BoardSource
{
public:
    HandSdkRos2Node()
        : rclcpp::Node("hand_sdk_ros2")
    {
        declare_parameter<std::string>("interface_name", "enp133s0f1");
        declare_parameter<std::string>("left_hand_name", "hand_200_L");
        declare_parameter<std::string>("right_hand_name", "hand_200_R");
        declare_parameter<double>("publish_rate_hz", 100.0);
        // What the board's kp/kd fields start from. Not applied to the device
        // by anything here: gains are a deliberate write, pressed from the
        // board or asked for over set_gains.
        declare_parameter<double>("initial_kp", 1.0);
        declare_parameter<double>("initial_kd", 0.005);
        // How long seed_pos waits for a state sample to latch. Tenths of a
        // second: the bus publishes at 1 kHz once it is cycling, so this only
        // covers a just-started cycle.
        declare_parameter<double>("seed_timeout_sec", 0.5);

        config_.interface_name = get_parameter("interface_name").as_string();
        config_.left_hand_name = get_parameter("left_hand_name").as_string();
        config_.right_hand_name = get_parameter("right_hand_name").as_string();

        left_.hand = hand_sdk::service::Hand::kLeft;
        right_.hand = hand_sdk::service::Hand::kRight;

        joint_name_index_.reserve(kNumRosJoints);
        for (std::size_t i = 0; i < kNumRosJoints; ++i) {
            joint_name_index_.emplace(kJointNames[i], i);
        }

        left_.state_pub = create_publisher<sensor_msgs::msg::JointState>("left/joint_states", 10);
        right_.state_pub = create_publisher<sensor_msgs::msg::JointState>("right/joint_states", 10);

        left_.command_sub = create_subscription<sensor_msgs::msg::JointState>(
            "left/joint_command", 10,
            [this](sensor_msgs::msg::JointState::SharedPtr msg) {
                onJointCommand(hand_sdk::service::Hand::kLeft, *msg);
            });
        right_.command_sub = create_subscription<sensor_msgs::msg::JointState>(
            "right/joint_command", 10,
            [this](sensor_msgs::msg::JointState::SharedPtr msg) {
                onJointCommand(hand_sdk::service::Hand::kRight, *msg);
            });

        // serviceGains() is a blocking bus round-trip (~10 ms, 100 ms timeout).
        // Left in the default (mutually exclusive) callback group it would stall
        // the state timer and joint_command subscription for that long, so the
        // services get their own group and main() spins multi-threaded. Api is
        // documented as safe to call from any thread.
        service_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        left_.gains_srv = create_service<SetGains>(
            "left/set_gains",
            [this](const SetGains::Request::SharedPtr request, SetGains::Response::SharedPtr response) {
                onSetGains(hand_sdk::service::Hand::kLeft, "left", *request, *response);
            },
            rclcpp::ServicesQoS(), service_group_);
        right_.gains_srv = create_service<SetGains>(
            "right/set_gains",
            [this](const SetGains::Request::SharedPtr request, SetGains::Response::SharedPtr response) {
                onSetGains(hand_sdk::service::Hand::kRight, "right", *request, *response);
            },
            rclcpp::ServicesQoS(), service_group_);

        const double publish_rate_hz = get_parameter("publish_rate_hz").as_double();
        const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / publish_rate_hz));
        publish_timer_ = create_wall_timer(period, [this]() { publishStates(); });

        RCLCPP_INFO(get_logger(),
                    "up, bus down. Nothing is published until the board connects.");
    }

    ~HandSdkRos2Node() override
    {
        disconnectBus();
    }

    double initialKp() const { return get_parameter("initial_kp").as_double(); }
    double initialKd() const { return get_parameter("initial_kd").as_double(); }

    // --- BoardSource: the bus, as the board drives it ------------------------
    //
    // connectBus(), disconnectBus() and api() run on the Qt thread and are the
    // only writers of api_. Everything else takes its own handle out of it
    // under the lock (bus()), so a call already in flight keeps its Api alive
    // even if the board drops the bus underneath it.

    Connection connectBus() override
    {
        Connection result;
        if (bus() != nullptr) {
            result.msg = "already connected";
            return result;
        }

        auto api = std::make_shared<hand_sdk::Api>(config_);
        const auto connected = api->connect();
        if (!connected.success) {
            result.msg = connected.msg;
            RCLCPP_WARN(get_logger(), "connect failed: %s", connected.msg.c_str());
            return result;
        }

        left_.present.store(connected.value.has_left);
        right_.present.store(connected.value.has_right);
        clearSnapshots();

        {
            std::lock_guard<std::mutex> lock(api_mutex_);
            api_ = std::move(api);
        }

        result.success = true;
        result.msg = connected.msg;
        result.has_left = connected.value.has_left;
        result.has_right = connected.value.has_right;
        result.dc_active = bus()->dcActive();

        RCLCPP_INFO(get_logger(), "connected: left=%s right=%s dc=%s",
                    result.has_left ? "yes" : "no",
                    result.has_right ? "yes" : "no",
                    result.dc_active ? "armed" : "off");
        return result;
    }

    std::string disconnectBus() override
    {
        std::shared_ptr<hand_sdk::Api> api;
        {
            std::lock_guard<std::mutex> lock(api_mutex_);
            api.swap(api_);
        }
        if (!api) {
            return "not connected";
        }

        // Taken before the disconnect, and the slave status after it:
        // disconnect() re-reads the AL status, so this is the one place either
        // side of the node can say what state the slaves ended in. A run whose
        // exchanges were failing ends somewhere other than OP.
        const auto stats = api->cycleStats();
        api->disconnect();

        std::ostringstream summary;
        summary << "bus at disconnect: left " << describeSlave(api->leftStatus())
                << " | right " << describeSlave(api->rightStatus())
                << " | cycles " << stats.cycles
                << " | exchange failures " << stats.exchange_failures
                << " | phase " << (stats.phase_error_ns / 1000) << " us";

        left_.present.store(false);
        right_.present.store(false);
        clearSnapshots();

        RCLCPP_INFO(get_logger(), "%s", summary.str().c_str());
        return summary.str();
    }

    hand_sdk::Api* api() override { return bus().get(); }

    bool present(hand_sdk::service::Hand hand) const override
    {
        return sideFor(hand).present.load();
    }

    BoardSnapshot snapshot(hand_sdk::service::Hand hand) const override
    {
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        return hand == hand_sdk::service::Hand::kLeft ? left_snapshot_ : right_snapshot_;
    }

    // The board's seed_pos, and the seeding half of its `set com all`. Runs on
    // the Qt thread: readState() is safe from anywhere, and the command itself
    // is written under command_mutex_ like every other writer of it.
    bool seedFromState(hand_sdk::service::Hand hand) override
    {
        auto& side = sideFor(hand);
        if (!side.present.load()) {
            return false;
        }
        const char* label = hand == hand_sdk::service::Hand::kLeft ? "left" : "right";
        return seedCommandFromState(side, label);
    }

private:
    std::shared_ptr<hand_sdk::Api> bus() const
    {
        std::lock_guard<std::mutex> lock(api_mutex_);
        return api_;
    }

    SideState& sideFor(hand_sdk::service::Hand hand)
    {
        return (hand == hand_sdk::service::Hand::kLeft) ? left_ : right_;
    }

    const SideState& sideFor(hand_sdk::service::Hand hand) const
    {
        return (hand == hand_sdk::service::Hand::kLeft) ? left_ : right_;
    }

    void clearSnapshots()
    {
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        left_snapshot_ = BoardSnapshot{};
        right_snapshot_ = BoardSnapshot{};
    }

    void sendLatestCommand(const std::shared_ptr<hand_sdk::Api>& api)
    {
        if (!api) {
            return;
        }

        hand_sdk::frames::DualHandCommand command;
        {
            std::lock_guard<std::mutex> lock(command_mutex_);
            if (left_.present.load()) {
                command.has_left = true;
                command.left = left_.command;
            }
            if (right_.present.load()) {
                command.has_right = true;
                command.right = right_.command;
            }
        }

        const auto result = api->sendCommand(command);
        if (!result.success) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "sendCommand failed: %s", result.msg.c_str());
        }
    }

    void onJointCommand(hand_sdk::service::Hand hand, const sensor_msgs::msg::JointState& msg)
    {
        // Latched whether or not the bus is up, so a publisher that starts
        // before the operator presses connect is not lost: the standing command
        // is already what it asked for by the time the wire exists.
        {
            std::lock_guard<std::mutex> lock(command_mutex_);
            updateCommandFromJointState(msg, sideFor(hand).command);
        }

        const auto api = bus();
        if (!api) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "joint_command received while the bus is down -- latched, "
                                 "not sent. Press connect on the board.");
            return;
        }
        sendLatestCommand(api);
    }

    void onSetGains(hand_sdk::service::Hand hand,
                    const char* label,
                    const SetGains::Request& request,
                    SetGains::Response& response)
    {
        const auto api = bus();
        if (!api) {
            response.success = false;
            response.message = "bus is down -- connect from the board first";
            RCLCPP_WARN(get_logger(), "%s set_gains: %s", label, response.message.c_str());
            return;
        }

        if (!sideFor(hand).present.load()) {
            response.success = false;
            response.message = std::string(label) + " hand is not present";
            RCLCPP_WARN(get_logger(), "%s", response.message.c_str());
            return;
        }

        // s16en11 on the wire: anything at or past +-16 saturates at the INT16
        // rail instead of wrapping, so the device would silently take a value
        // the caller never asked for.
        if (std::abs(request.kp) >= kGainWireLimit || std::abs(request.kd) >= kGainWireLimit) {
            RCLCPP_WARN(get_logger(),
                        "%s gains kp=%.3f kd=%.3f exceed the s16en11 range (+-%.1f) and will saturate",
                        label, request.kp, request.kd, kGainWireLimit);
        }

        // Hand-wide, unlike the board's masked write: from ROS the hand is one
        // unit (see srv/SetGains.srv).
        const auto result = api->serviceGains(hand, request.kp, request.kd);
        response.success = result.success;
        response.message = result.msg;

        if (!result.success) {
            RCLCPP_WARN(get_logger(), "%s serviceGains failed: %s", label, result.msg.c_str());
            return;
        }

        response.kp_readback.reserve(kNumRosJoints);
        response.kd_readback.reserve(kNumRosJoints);
        for (std::size_t i = 0; i < kNumRosJoints; ++i) {
            response.kp_readback.push_back(result.value[i].kp);
            response.kd_readback.push_back(result.value[i].kd);
        }

        RCLCPP_INFO(get_logger(), "%s gains set: kp=%.3f kd=%.3f (readback kp=%.3f kd=%.3f)",
                    label, request.kp, request.kd,
                    result.value[0].kp, result.value[0].kd);
    }

    void publishStates()
    {
        const auto api = bus();
        if (!api) {
            return;   // no bus, nothing to publish, and nothing worth warning about
        }

        const auto state = api->readState();
        if (!state.success) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "readState failed: %s", state.msg.c_str());
            return;
        }

        const auto stamp = now();
        if (left_.present.load() && state.value.has_left) {
            publishSide(left_.state_pub, stamp, state.value.left);
            cacheSnapshot(left_snapshot_, state.value.left);
        }
        if (right_.present.load() && state.value.has_right) {
            publishSide(right_.state_pub, stamp, state.value.right);
            cacheSnapshot(right_snapshot_, state.value.right);
        }
    }

    // Kept alongside the publish so the board shows exactly what subscribers
    // got, error bits included -- those ride in every readState() but have no
    // room in a JointState, so this is the only place they surface.
    void cacheSnapshot(BoardSnapshot& target, const hand_sdk::frames::HandState& state)
    {
        BoardSnapshot fresh;
        fresh.valid = true;
        for (std::size_t i = 0; i < kNumRosJoints; ++i) {
            fresh.q[i] = state.q[i];
            fresh.dq[i] = state.dq[i];
            fresh.tau[i] = state.tau[i];
        }
        fresh.error_bits = state.error_bits;

        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        target = fresh;
    }

    void publishSide(const rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr& publisher,
                     const rclcpp::Time& stamp,
                     const hand_sdk::frames::HandState& state)
    {
        sensor_msgs::msg::JointState msg;
        msg.header.stamp = stamp;
        msg.name = jointNames_;
        msg.position = toDoubleVector(state.q);
        msg.velocity = toDoubleVector(state.dq);
        msg.effort = toDoubleVector(state.tau);
        publisher->publish(msg);
    }

    void updateCommandFromJointState(const sensor_msgs::msg::JointState& msg,
                                     hand_sdk::frames::HandCommand& command)
    {
        const bool direct_order = msg.name.empty() || msg.name.size() == kNumRosJoints;

        if (direct_order) {
            copyByIndex(msg.position, command.q);
            copyByIndex(msg.velocity, command.dq);
            copyByIndex(msg.effort, command.tau);
            return;
        }

        for (std::size_t msg_i = 0; msg_i < msg.name.size(); ++msg_i) {
            const auto iter = joint_name_index_.find(msg.name[msg_i]);
            if (iter == joint_name_index_.end()) {
                continue;
            }

            const std::size_t joint_i = iter->second;
            if (msg_i < msg.position.size()) {
                command.q[joint_i] = static_cast<float>(msg.position[msg_i]);
            }
            if (msg_i < msg.velocity.size()) {
                command.dq[joint_i] = static_cast<float>(msg.velocity[msg_i]);
            }
            if (msg_i < msg.effort.size()) {
                command.tau[joint_i] = static_cast<float>(msg.effort[msg_i]);
            }
        }
    }

    template <typename T>
    void copyByIndex(const std::vector<double>& source, std::array<T, kNumSdkJoints>& target)
    {
        const std::size_t count = std::min(source.size(), kNumRosJoints);
        for (std::size_t i = 0; i < count; ++i) {
            target[i] = static_cast<T>(source[i]);
        }
    }

    // Latch the pose the hand is measured at as the standing command, and put
    // it on the wire. The board asks for this after opening communication and
    // after any zero calibration -- a calibration redefines a joint's origin,
    // so a pose latched before it refers to an origin the device has dropped.
    bool seedCommandFromState(SideState& side, const char* label)
    {
        const auto api = bus();
        if (!api) {
            return false;
        }

        const double timeout_sec = get_parameter("seed_timeout_sec").as_double();
        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                  std::chrono::duration<double>(timeout_sec));

        // The bus has been cycling since connect, so the first read normally
        // succeeds; the retries only cover a torn read or a just-started cycle.
        while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
            const auto state = api->readState();
            const bool is_left = side.hand == hand_sdk::service::Hand::kLeft;
            const bool has_side = state.success &&
                                  (is_left ? state.value.has_left : state.value.has_right);

            if (has_side) {
                const auto& measured = is_left ? state.value.left : state.value.right;
                {
                    std::lock_guard<std::mutex> lock(command_mutex_);
                    side.command.q = measured.q;
                    side.command.dq.fill(0.0f);
                    side.command.tau.fill(0.0f);
                }
                sendLatestCommand(api);
                RCLCPP_INFO(get_logger(), "%s hold pose seeded from measured state", label);
                return true;
            }

            std::this_thread::sleep_for(5ms);
        }

        RCLCPP_WARN(get_logger(), "%s no state to seed the hold pose from", label);
        return false;
    }

    hand_sdk::Config config_{};

    // Written only by connectBus()/disconnectBus() on the Qt thread; every
    // other thread copies the handle out under the lock before using it.
    mutable std::mutex api_mutex_;
    std::shared_ptr<hand_sdk::Api> api_;

    SideState left_;
    SideState right_;
    rclcpp::CallbackGroup::SharedPtr service_group_;
    std::mutex command_mutex_;

    // Written by the publish timer, read by the Qt thread.
    mutable std::mutex snapshot_mutex_;
    BoardSnapshot left_snapshot_;
    BoardSnapshot right_snapshot_;
    std::unordered_map<std::string, std::size_t> joint_name_index_;
    std::vector<std::string> jointNames_ = toVector(kJointNames);
    rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<HandSdkRos2Node>();
        // Multi-threaded so a blocking set_gains round-trip cannot stall the
        // state timer or the joint_command path; those stay serialized with
        // each other in the node's default callback group.
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(node);

        // Qt owns the main thread -- it is the only thread it will build
        // widgets on -- so ROS spins beside it. The board comes up every run,
        // whatever the hardware is doing, because it is the only way to open
        // the bus. Closing the window ends the node; Ctrl-C ends both, since
        // the executor thread drops out of spin() on shutdown and quits the Qt
        // loop.
        QApplication app(argc, argv);
        HandNodeBoard board(*node, node->initialKp(), node->initialKd());
        board.show();

        std::thread ros_thread([&executor, &app]() {
            executor.spin();
            QMetaObject::invokeMethod(&app, "quit", Qt::QueuedConnection);
        });

        const int status = app.exec();

        executor.cancel();
        ros_thread.join();
        rclcpp::shutdown();
        return status;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("hand_sdk_ros2"), "%s", e.what());
        rclcpp::shutdown();
        return 1;
    }
}
