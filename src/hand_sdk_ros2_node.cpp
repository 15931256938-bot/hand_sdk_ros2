#include <hand_sdk.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <hand_sdk_ros2/srv/set_gains.hpp>

#include "hand_sdk_node_board.hpp"

#include <QApplication>

#include <array>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <string>
#include <stdexcept>
#include <thread>
#include <unordered_map>
#include <utility>
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
        const auto value = values[i];
        result.push_back(static_cast<double>(value));
    }
    return result;
}

using SetGains = hand_sdk_ros2::srv::SetGains;

struct SideState
{
    bool present{false};
    hand_sdk::service::Hand hand{hand_sdk::service::Hand::kLeft};
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr command_sub;
    rclcpp::Service<SetGains>::SharedPtr gains_srv;
    hand_sdk::frames::HandCommand command{};
};

} // namespace

class HandSdkRos2Node final : public rclcpp::Node, public BoardSource
{
public:
    HandSdkRos2Node()
        : rclcpp::Node("hand_sdk_ros2")
    {
        declare_parameter<std::string>("interface_name", "enp132s0");
        declare_parameter<std::string>("left_hand_name", "hand_200_L");
        declare_parameter<std::string>("right_hand_name", "hand_200_R");
        declare_parameter<double>("publish_rate_hz", 100.0);
        // Not applied at bring-up any more -- nothing is enabled unattended, so
        // there are no gains to set yet. Kept as the default the board's gains
        // control starts from.
        declare_parameter<double>("initial_kp", 1.0);
        declare_parameter<double>("initial_kd", 0.005);
        declare_parameter<double>("bringup_timeout_sec", 5.0);
        // The diagnostic board comes up with the node. Turn it off for headless
        // runs -- Qt needs a display and will fail hard without one.
        declare_parameter<bool>("board", true);

        hand_sdk::Config config;
        config.interface_name = get_parameter("interface_name").as_string();
        config.left_hand_name = get_parameter("left_hand_name").as_string();
        config.right_hand_name = get_parameter("right_hand_name").as_string();

        api_ = std::make_unique<hand_sdk::Api>(config);

        const auto connected = api_->connect();
        if (!connected.success) {
            throw std::runtime_error("hand_sdk connect failed: " + connected.msg);
        }

        left_.present = connected.value.has_left;
        left_.hand = hand_sdk::service::Hand::kLeft;
        right_.present = connected.value.has_right;
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

        RCLCPP_INFO(get_logger(), "connected: left=%s right=%s dc=%s",
                    left_.present ? "yes" : "no",
                    right_.present ? "yes" : "no",
                    api_->dcActive() ? "armed" : "off");

        if (left_.present) {
            bringUpSide(left_, "left");
        }
        if (right_.present) {
            bringUpSide(right_, "right");
        }

        if (!left_.present && !right_.present) {
            throw std::runtime_error("hand_sdk connect reported no hands present");
        }
    }

    ~HandSdkRos2Node() override
    {
        if (api_) {
            api_->disconnect();
        }
    }

    bool boardEnabled() const { return get_parameter("board").as_bool(); }

    // --- BoardSource: what the diagnostic board reads --------------------

    hand_sdk::Api* api() override { return api_.get(); }

    bool present(hand_sdk::service::Hand hand) const override
    {
        return sideFor(hand).present;
    }

    BoardSnapshot snapshot(hand_sdk::service::Hand hand) const override
    {
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        return hand == hand_sdk::service::Hand::kLeft ? left_snapshot_ : right_snapshot_;
    }

    // The board's seed_pos button. Runs the same step bring-up does, on the Qt
    // thread: readState() is safe from anywhere, and the command itself is
    // written under command_mutex_ like every other writer of it.
    bool seedFromState(hand_sdk::service::Hand hand) override
    {
        auto& side = sideFor(hand);
        if (!side.present) {
            return false;
        }
        const char* label = hand == hand_sdk::service::Hand::kLeft ? "left" : "right";
        return seedCommandFromState(side, label);
    }

private:
    SideState& sideFor(hand_sdk::service::Hand hand)
    {
        return (hand == hand_sdk::service::Hand::kLeft) ? left_ : right_;
    }

    const SideState& sideFor(hand_sdk::service::Hand hand) const
    {
        return (hand == hand_sdk::service::Hand::kLeft) ? left_ : right_;
    }

    void sendLatestCommand()
    {
        hand_sdk::frames::DualHandCommand command;

        {
            std::lock_guard<std::mutex> lock(command_mutex_);
            if (left_.present) {
                command.has_left = true;
                command.left = left_.command;
            }
            if (right_.present) {
                command.has_right = true;
                command.right = right_.command;
            }
        }

        const auto result = api_->sendCommand(command);
        if (!result.success) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "sendCommand failed: %s", result.msg.c_str());
        }
    }

    void onJointCommand(hand_sdk::service::Hand hand, const sensor_msgs::msg::JointState& msg)
    {
        {
            std::lock_guard<std::mutex> lock(command_mutex_);
            auto& command = sideFor(hand).command;
            updateCommandFromJointState(msg, command);
        }

        sendLatestCommand();
    }

    void onSetGains(hand_sdk::service::Hand hand,
                    const char* label,
                    const SetGains::Request& request,
                    SetGains::Response& response)
    {
        if (!sideFor(hand).present) {
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

        const auto result = api_->serviceGains(hand, request.kp, request.kd);
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
        const auto state = api_->readState();
        if (!state.success) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "readState failed: %s", state.msg.c_str());
            return;
        }

        const auto stamp = now();
        if (left_.present && state.value.has_left) {
            publishSide(left_.state_pub, stamp, state.value.left);
            cacheSnapshot(left_snapshot_, state.value.left);
        }
        if (right_.present && state.value.has_right) {
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

    bool waitForMcReady(hand_sdk::service::Hand hand, const char* label)
    {
        const double timeout_sec = get_parameter("bringup_timeout_sec").as_double();
        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                  std::chrono::duration<double>(timeout_sec));

        while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
            const auto mc = api_->serviceMcState(hand);
            if (!mc.success) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                     "%s mc-state query failed: %s",
                                     label, mc.msg.c_str());
                std::this_thread::sleep_for(100ms);
                continue;
            }

            bool ready = true;
            for (const auto& joint : mc.value) {
                if (joint.state != hand_sdk::service::McState::kStandby &&
                    joint.state != hand_sdk::service::McState::kActive) {
                    ready = false;
                    break;
                }
            }

            if (ready) {
                RCLCPP_INFO(get_logger(), "%s mc-state ready", label);
                return true;
            }

            std::this_thread::sleep_for(100ms);
        }

        RCLCPP_WARN(get_logger(), "%s mc-state did not reach standby|active before timeout", label);
        return false;
    }

    bool seedCommandFromState(SideState& side, const char* label)
    {
        const double timeout_sec = get_parameter("bringup_timeout_sec").as_double();
        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                  std::chrono::duration<double>(timeout_sec));

        while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
            const auto state = api_->readState();
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
                sendLatestCommand();
                RCLCPP_INFO(get_logger(), "%s hold pose seeded from measured state", label);
                return true;
            }

            std::this_thread::sleep_for(10ms);
        }

        RCLCPP_WARN(get_logger(), "%s no state to seed the hold pose from", label);
        return false;
    }

    // Bring-up stops short of enabling anything.
    //
    // The device zeroes itself wherever it happens to be at power-up, so until
    // a joint has been zero-calibrated its q is measured from an arbitrary boot
    // pose. Enabling force control before that would hold the joint against a
    // pose in a frame the calibration is about to replace. Zero calibration is
    // a deliberate, per-joint operation -- it drives the motor looking for its
    // stop -- so it is not something a node constructor should be starting on
    // its own either.
    //
    // What is left is everything that is safe unattended: get the motors
    // talking, and latch the pose they are already in so that whoever enables
    // control has something sane standing. Force control and gains are pressed
    // from the board once the joints are calibrated (`zero_calibrate` ->
    // `seed current pos` -> enable), which is also why `set com all` is still
    // there: if this fails, that is the retry.
    bool bringUpSide(SideState& side, const char* label)
    {
        using hand_sdk::service::ComCmd;

        waitForMcReady(side.hand, label);

        const auto comm = api_->serviceComm(side.hand, ComCmd::kStartBroadcast);
        if (!comm.success) {
            RCLCPP_WARN(get_logger(), "%s start broadcast failed: %s", label, comm.msg.c_str());
            return false;
        }

        if (!seedCommandFromState(side, label)) {
            return false;
        }

        RCLCPP_INFO(get_logger(),
                    "%s ready: broadcasting and holding its measured pose. Not enabled -- "
                    "zero-calibrate, re-seed, then enable control from the board.", label);
        return true;
    }

    std::unique_ptr<hand_sdk::Api> api_;
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

        if (!node->boardEnabled()) {
            executor.spin();
            rclcpp::shutdown();
            return 0;
        }

        // Qt owns the main thread -- it is the only thread it will build
        // widgets on -- so ROS spins beside it. Closing the window ends the
        // node; Ctrl-C ends both, since the executor thread drops out of spin()
        // on shutdown and quits the Qt loop.
        QApplication app(argc, argv);
        HandNodeBoard board(*node);
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