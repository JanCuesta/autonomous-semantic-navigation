#include <chrono>
#include <cmath>
#include <exception>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;
using SteadyClock = std::chrono::steady_clock;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
using NavigateClient = rclcpp_action::Client<NavigateToPose>;
using ListControllers = controller_manager_msgs::srv::ListControllers;

// Mission outcomes and active phases. Readiness is tracked separately.
enum class MissionState
{
    IDLE,
    PREPARING,
    NAVIGATING,
    SUCCEEDED,
    FAILED,
    CANCELLED
};

class MissionManager : public rclcpp::Node
{
public:
    MissionManager() : Node("mission_manager")
    {
        // Parameters -------------------------------------------------------
        goal_timeout_ = this->declare_parameter<double>("goal_timeout", 180.0);
        max_attempts_ = this->declare_parameter<int>("max_attempts", 3);
        odom_timeout_ = this->declare_parameter<double>("odom_timeout", 1.0);

        if (goal_timeout_ <= 0.0 || odom_timeout_ <= 0.0 || max_attempts_ < 1) {
            throw std::invalid_argument("goal_timeout and odom_timeout must be positive; max_attempts >= 1");
        }

        // Publishers -------------------------------------------------------
        state_pub_ = this->create_publisher<std_msgs::msg::String>("/mission/state", 10);

        // Subscribers ------------------------------------------------------
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mission/goal", 10,
            std::bind(&MissionManager::goal_callback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&MissionManager::odom_callback, this, std::placeholders::_1));

        // Nav2 AMCL in our tested simulation publishes TRANSIENT_LOCAL.
        // Recheck this QoS on the physical robot before using it there.
        auto amcl_qos = rclcpp::QoS(10).reliable().transient_local();
        amcl_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/amcl_pose", amcl_qos,
            std::bind(&MissionManager::amcl_callback, this, std::placeholders::_1));

        // Action and service clients --------------------------------------
        nav_client_ = rclcpp_action::create_client<NavigateToPose>(
            this, "/navigate_to_pose");
        controller_client_ = this->create_client<ListControllers>(
            "/controller_manager/list_controllers");

        // Service servers --------------------------------------------------
        cancel_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/mission/cancel",
            std::bind(&MissionManager::cancel_callback, this,
                      std::placeholders::_1, std::placeholders::_2));

        // Periodic readiness and per-attempt timeout checks ---------------
        timer_ready_ = this->create_wall_timer(500ms, [this]() {
            check_readiness();
            navigation_metrics();
        });

        publish_state();
        RCLCPP_INFO(this->get_logger(), "Mission Manager started.");
        print_status();
    }

private:
    // ---------------------------------------------------------------------
    // Mission state and reporting
    // ---------------------------------------------------------------------
    static const char *state_to_string(MissionState state)
    {
        switch (state) {
            case MissionState::IDLE: return "IDLE";
            case MissionState::PREPARING: return "PREPARING";
            case MissionState::NAVIGATING: return "NAVIGATING";
            case MissionState::SUCCEEDED: return "SUCCEEDED";
            case MissionState::FAILED: return "FAILED";
            case MissionState::CANCELLED: return "CANCELLED";
        }
        return "UNKNOWN";
    }

    void publish_state()
    {
        std_msgs::msg::String msg;
        msg.data = state_to_string(state_);
        state_pub_->publish(msg);
    }

    void set_state(MissionState next_state)
    {
        if (next_state == state_) {
            return;
        }
        state_ = next_state;
        publish_state();
        print_status();
    }

    void print_status()
    {
        RCLCPP_INFO(this->get_logger(), "STATUS: %s", readiness_ ? "READY" : "NOT READY");
        if (!readiness_) {
            RCLCPP_INFO(this->get_logger(), "Missing:");
            if (!odom_ready_) RCLCPP_INFO(this->get_logger(), "  - odom");
            if (!localization_ready_) RCLCPP_INFO(this->get_logger(), "  - amcl");
            if (!nav2_ready_) RCLCPP_INFO(this->get_logger(), "  - nav2");
            if (!controller_ready_) RCLCPP_INFO(this->get_logger(), "  - controller");
        }
        RCLCPP_INFO(this->get_logger(), "Mission state: %s", state_to_string(state_));
    }

    void print_mission_summary(const char *reason)
    {
        const double total_time = std::chrono::duration<double>(
            SteadyClock::now() - mission_start_time_).count();

        RCLCPP_INFO(
            this->get_logger(),
            "\n========== MISSION SUMMARY ==========\n"
            "Final state: %s\n"
            "Goal: (%.2f, %.2f) m\n"
            "Total time: %.1f s (wall clock)\n"
            "Attempts: %d/%d\n"
            "Total Nav2 recoveries: %d\n"
            "Reason: %s\n"
            "=====================================",
            state_to_string(state_),
            current_goal_.pose.position.x,
            current_goal_.pose.position.y,
            total_time,
            current_attempt_,
            max_attempts_,
            total_recoveries_,
            reason);
    }

    void finish_mission(MissionState result_state, const char *reason)
    {
        mission_active_ = false;
        set_state(result_state);
        print_mission_summary(reason);
        cancel_requested_ = false;
    }

    // ---------------------------------------------------------------------
    // Subscribers
    // ---------------------------------------------------------------------
    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (mission_active_) {
            RCLCPP_WARN(this->get_logger(),
                        "Mission already in progress. New goal rejected.");
            return;
        }
        if (!readiness_) {
            RCLCPP_WARN(this->get_logger(),
                        "Mission system is NOT READY. Goal was not sent.");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Goal received: x=%.2f, y=%.2f",
                    msg->pose.position.x, msg->pose.position.y);

        // Reset mission-level values only once, not on each retry.
        mission_active_ = true;
        cancel_requested_ = false;
        current_attempt_ = 1;
        total_recoveries_ = 0;
        mission_start_time_ = SteadyClock::now();
        current_goal_ = *msg;

        set_state(MissionState::PREPARING);
        send_navigation_goal(current_goal_);
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        odom_received_ = true;
        last_odom_ = SteadyClock::now();
        const double linear = std::abs(msg->twist.twist.linear.x);
        const double angular = std::abs(msg->twist.twist.angular.z);
        moving_ = linear > 0.01 || angular > 0.02;
    }

    void amcl_callback(
        const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr)
    {
        localization_received_ = true;
        last_amcl_ = SteadyClock::now();
    }

    // ---------------------------------------------------------------------
    // Nav2 goal and feedback
    // ---------------------------------------------------------------------
    void send_navigation_goal(const geometry_msgs::msg::PoseStamped &goal_pose)
    {
        NavigateToPose::Goal goal;
        goal.pose = goal_pose;
        current_goal_ = goal_pose;

        // Attempt-level values are reset for each new Nav2 action goal.
        distance_remaining_ = 0.0;
        navigation_time_ = 0.0;
        recovery_count_ = 0;
        goal_timed_out_ = false;
        attempt_start_time_ = SteadyClock::now();

        NavigateClient::SendGoalOptions options;

        options.goal_response_callback = [this](
            GoalHandleNavigateToPose::SharedPtr handle)
        {
            if (!handle) {
                RCLCPP_ERROR(this->get_logger(), "Navigation goal rejected.");
                finish_mission(MissionState::FAILED, "GOAL_REJECTED");
                return;
            }
            active_goal_handle_ = handle;
            RCLCPP_INFO(this->get_logger(), "Navigation goal accepted.");
            set_state(MissionState::NAVIGATING);
        };

        options.result_callback = std::bind(
            &MissionManager::result_callback, this, std::placeholders::_1);

        options.feedback_callback = [this](
            GoalHandleNavigateToPose::SharedPtr handle,
            const std::shared_ptr<const NavigateToPose::Feedback> feedback)
        {
            // Ignore late feedback from an earlier attempt.
            if (!active_goal_handle_ || !handle ||
                handle->get_goal_id() != active_goal_handle_->get_goal_id()) {
                return;
            }
            distance_remaining_ = feedback->distance_remaining;
            navigation_time_ = feedback->navigation_time.sec +
                               feedback->navigation_time.nanosec * 1e-9;
            recovery_count_ = static_cast<int>(feedback->number_of_recoveries);
        };

        try {
            nav_client_->async_send_goal(goal, options);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Could not send Nav2 goal: %s", e.what());
            finish_mission(MissionState::FAILED, "SEND_GOAL_ERROR");
        }
    }

    void result_callback(const GoalHandleNavigateToPose::WrappedResult &result)
    {
        if (!active_goal_handle_ ||
            result.goal_id != active_goal_handle_->get_goal_id()) {
            RCLCPP_WARN(this->get_logger(),
                        "Ignoring result from non-active navigation goal.");
            return;
        }

        active_goal_handle_.reset();
        total_recoveries_ += recovery_count_;

        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                finish_mission(MissionState::SUCCEEDED, "GOAL_REACHED");
                break;

            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_WARN(this->get_logger(), "Navigation aborted. Attempt %d/%d",
                            current_attempt_, max_attempts_);
                if (result.result) {
                    RCLCPP_WARN(this->get_logger(), "Nav2 error %u: %s",
                                static_cast<unsigned>(result.result->error_code),
                                result.result->error_msg.c_str());
                }

                // Never retry an operator-cancelled or timed-out goal.
                if (!cancel_requested_ && !goal_timed_out_ && readiness_ &&
                    current_attempt_ < max_attempts_) {
                    ++current_attempt_;
                    RCLCPP_INFO(this->get_logger(), "Retrying navigation. Attempt %d/%d",
                                current_attempt_, max_attempts_);
                    set_state(MissionState::PREPARING);
                    send_navigation_goal(current_goal_);
                } else if (cancel_requested_) {
                    finish_mission(MissionState::FAILED, "ABORTED_AFTER_CANCEL_REQUEST");
                } else if (goal_timed_out_) {
                    finish_mission(MissionState::FAILED, "GOAL_TIMEOUT");
                } else {
                    finish_mission(MissionState::FAILED, "NAV2_ABORTED");
                }
                break;

            case rclcpp_action::ResultCode::CANCELED:
                if (goal_timed_out_) {
                    finish_mission(MissionState::FAILED, "GOAL_TIMEOUT");
                } else if (cancel_requested_) {
                    RCLCPP_INFO(this->get_logger(), "Navigation cancellation CONFIRMED.");
                    finish_mission(MissionState::CANCELLED, "OPERATOR_CANCEL");
                } else {
                    finish_mission(MissionState::CANCELLED, "NAV2_CANCELED");
                }
                break;

            default:
                finish_mission(MissionState::FAILED, "UNKNOWN_RESULT");
                break;
        }
    }

    // ---------------------------------------------------------------------
    // Cancellation service
    // ---------------------------------------------------------------------
    void cancel_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (!mission_active_ || state_ != MissionState::NAVIGATING ||
            !active_goal_handle_) {
            response->success = false;
            response->message = "No active navigation goal to cancel.";
            return;
        }
        if (cancel_requested_ || goal_timed_out_) {
            response->success = false;
            response->message = "Cancellation or timeout is already pending.";
            return;
        }

        const auto goal_id = active_goal_handle_->get_goal_id();
        cancel_requested_ = true;  // Prevent a retry while awaiting Nav2.
        RCLCPP_INFO(this->get_logger(), "Requesting navigation cancellation...");

        try {
            nav_client_->async_cancel_goal(
                active_goal_handle_,
                [this, goal_id](NavigateClient::CancelResponse::SharedPtr cancel_response) {
                    if (!active_goal_handle_ ||
                        active_goal_handle_->get_goal_id() != goal_id) {
                        return;  // Late response from a completed goal.
                    }
                    if (cancel_response->return_code ==
                            NavigateClient::CancelResponse::ERROR_NONE &&
                        !cancel_response->goals_canceling.empty()) {
                        RCLCPP_INFO(this->get_logger(),
                                    "Nav2 accepted cancellation request.");
                    } else {
                        cancel_requested_ = false;
                        RCLCPP_WARN(this->get_logger(),
                                    "Nav2 rejected cancellation. Code: %d",
                                    static_cast<int>(cancel_response->return_code));
                    }
                });
            response->success = true;
            response->message =
                "Cancellation request submitted. Awaiting Nav2 result.";
        } catch (const std::exception &e) {
            cancel_requested_ = false;
            response->success = false;
            response->message = "Failed to submit cancellation request.";
            RCLCPP_ERROR(this->get_logger(), "Cancellation error: %s", e.what());
        }
    }

    // ---------------------------------------------------------------------
    // Readiness and periodic navigation metrics
    // ---------------------------------------------------------------------
    void check_controller()
    {
        if (!controller_client_->service_is_ready()) {
            controller_active_ = false;
            return;
        }
        if (controller_request_pending_) {
            return;
        }

        controller_request_pending_ = true;
        auto request = std::make_shared<ListControllers::Request>();
        controller_client_->async_send_request(
            request, [this](rclcpp::Client<ListControllers>::SharedFuture future) {
                controller_request_pending_ = false;
                controller_active_ = false;
                try {
                    for (const auto &controller : future.get()->controller) {
                        if (controller.name == "diff_drive_controller" &&
                            controller.state == "active") {
                            controller_active_ = true;
                            break;
                        }
                    }
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(this->get_logger(),
                                 "Controller readiness request failed: %s", e.what());
                }
            });
    }

    void check_readiness()
    {
        check_controller();

        const auto now = SteadyClock::now();
        const bool odom_publisher_present = this->count_publishers("/odom") > 0;
        const bool amcl_publisher_present = this->count_publishers("/amcl_pose") > 0;

        if (!odom_publisher_present) odom_received_ = false;
        if (!amcl_publisher_present) localization_received_ = false;

        const bool odom_fresh = odom_received_ &&
            std::chrono::duration<double>(now - last_odom_).count() < odom_timeout_;

        const bool new_odom_ready = odom_publisher_present && odom_fresh;
        // Initial AMCL estimate plus live publisher; not a rigorous localization-health test.
        const bool new_localization_ready =
            amcl_publisher_present && localization_received_;
        const bool new_nav2_ready = nav_client_->action_server_is_ready();
        const bool new_controller_ready = controller_active_;
        const bool new_readiness = new_odom_ready && new_localization_ready &&
                                   new_nav2_ready && new_controller_ready;

        const bool changed = (new_odom_ready != odom_ready_) ||
                             (new_localization_ready != localization_ready_) ||
                             (new_nav2_ready != nav2_ready_) ||
                             (new_controller_ready != controller_ready_) ||
                             (new_readiness != readiness_);

        odom_ready_ = new_odom_ready;
        localization_ready_ = new_localization_ready;
        nav2_ready_ = new_nav2_ready;
        controller_ready_ = new_controller_ready;
        readiness_ = new_readiness;
        if (changed) print_status();
    }

    void navigation_metrics()
    {
        if (state_ != MissionState::NAVIGATING) {
            return;
        }

        RCLCPP_INFO(this->get_logger(),
                    "\nGoal: x=%.2f m, y=%.2f m\n"
                    "Distance remaining: %.2f m\n"
                    "Navigation time: %.1f s\n"
                    "Motion: %s\n"
                    "Recoveries: %d\n"
                    "Current attempt: %d/%d",
                    current_goal_.pose.position.x,
                    current_goal_.pose.position.y,
                    distance_remaining_, navigation_time_,
                    moving_ ? "MOVING" : "STATIONARY",
                    recovery_count_, current_attempt_, max_attempts_);

        // Use a steady clock: this timeout still runs if Nav2 feedback stops.
        const double elapsed = std::chrono::duration<double>(
            SteadyClock::now() - attempt_start_time_).count();
        if (elapsed >= goal_timeout_ && !goal_timed_out_ &&
            !cancel_requested_ && active_goal_handle_) {
            goal_timed_out_ = true;
            RCLCPP_WARN(this->get_logger(),
                        "Goal timeout (%.1f wall seconds). Cancelling Nav2.",
                        goal_timeout_);
            nav_client_->async_cancel_goal(active_goal_handle_);
        }
    }

    // ---------------------------------------------------------------------
    // ROS interfaces (grouped by role)
    // ---------------------------------------------------------------------
    // Publishers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;

    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;

    // Action and service clients
    NavigateClient::SharedPtr nav_client_;
    rclcpp::Client<ListControllers>::SharedPtr controller_client_;

    // Service servers and timers
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_service_;
    rclcpp::TimerBase::SharedPtr timer_ready_;

    // ---------------------------------------------------------------------
    // Parameters
    // ---------------------------------------------------------------------
    double goal_timeout_{180.0};       // Per attempt; wall-clock seconds.
    double odom_timeout_{1.0};         // Wall-clock seconds.
    int max_attempts_{3};              // Initial attempt + up to 2 retries.

    // ---------------------------------------------------------------------
    // Mission state and cached measurements
    // ---------------------------------------------------------------------
    MissionState state_{MissionState::IDLE};
    bool mission_active_{false};
    bool cancel_requested_{false};
    bool goal_timed_out_{false};

    bool readiness_{false};
    bool odom_received_{false};
    bool localization_received_{false};
    bool odom_ready_{false};
    bool localization_ready_{false};
    bool nav2_ready_{false};
    bool controller_ready_{false};
    bool controller_active_{false};
    bool controller_request_pending_{false};
    bool moving_{false};

    int current_attempt_{1};
    int recovery_count_{0};
    int total_recoveries_{0};
    double distance_remaining_{0.0};
    double navigation_time_{0.0};

    geometry_msgs::msg::PoseStamped current_goal_;
    GoalHandleNavigateToPose::SharedPtr active_goal_handle_;

    SteadyClock::time_point mission_start_time_{};
    SteadyClock::time_point attempt_start_time_{};
    SteadyClock::time_point last_odom_{};
    SteadyClock::time_point last_amcl_{};
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MissionManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
