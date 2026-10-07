#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include "std_msgs/msg/string.hpp"

#include "controller_manager_msgs/srv/list_controllers.hpp"

using namespace std::chrono_literals;

using NavigateToPose = nav2_msgs::action::NavigateToPose;

using GoalHandleNavigateToPose =
    rclcpp_action::ClientGoalHandle<NavigateToPose>;

// ---------------------------------------------------------
// MISSION STATES
// ---------------------------------------------------------

enum class MissionState
{
    IDLE,
    PREPARING,
    NAVIGATING,
    SUCCEEDED,
    FAILED,
    CANCELLED
};

// ---------------------------------------------------------
// MISSION MANAGER NODE
// ---------------------------------------------------------

class MissionManager : public rclcpp::Node
{
public:
    MissionManager()
        : Node("mission_manager")
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Mission Manager started.");

        // -------------------------------------------------
        // STATE PUBLISHER
        // -------------------------------------------------

        state_pub_ =
            this->create_publisher<std_msgs::msg::String>(
                "/mission/state",
                10);

        // -------------------------------------------------
        // GOAL SUBSCRIBER
        // -------------------------------------------------

        goal_sub_ =
            this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/mission/goal",
                10,
                std::bind(
                    &MissionManager::goal_callback,
                    this,
                    std::placeholders::_1));

        // -------------------------------------------------
        // NAV2 ACTION CLIENT
        // -------------------------------------------------

        nav_client_ =
            rclcpp_action::create_client<NavigateToPose>(
                this,
                "/navigate_to_pose");

        // -------------------------------------------------
        // CONTROLLER MANAGER CLIENT
        // -------------------------------------------------

        controller_client_ =
            this->create_client<
                controller_manager_msgs::srv::ListControllers>(
                "/controller_manager/list_controllers");

        // -------------------------------------------------
        // ODOM SUBSCRIBER
        // -------------------------------------------------

        odom_sub_ =
            this->create_subscription<nav_msgs::msg::Odometry>(
                "/odom",
                10,
                std::bind(
                    &MissionManager::odom_callback,
                    this,
                    std::placeholders::_1));

        // -------------------------------------------------
        // AMCL SUBSCRIBER
        // -------------------------------------------------

        amcl_sub_ =
            this->create_subscription<
                geometry_msgs::msg::PoseWithCovarianceStamped>(
                "/amcl_pose",
                10,
                std::bind(
                    &MissionManager::amcl_callback,
                    this,
                    std::placeholders::_1));

        // -------------------------------------------------
        // READINESS + NAVIGATION METRICS TIMER
        // -------------------------------------------------

        timer_ready_ =
            this->create_wall_timer(
                2s,
                [this]()
                {
                    check_readiness();
                    navigation_metrics();
                });

        // Publish initial mission state
        std_msgs::msg::String msg;
        msg.data = state_to_string(state_);
        state_pub_->publish(msg);

        print_status();
    }

private:
    // ---------------------------------------------------------
    // NAVIGATION METRICS
    // ---------------------------------------------------------

    void navigation_metrics()
    {
        if (state_ != MissionState::NAVIGATING)
        {
            return;
        }

        RCLCPP_INFO(
            this->get_logger(),
            "\nGoal: x=%.2f m, y=%.2f m\n"
            "Distance remaining: %.2f m\n"
            "Navigation time: %.1f s\n"
            "Motion: %s\n"
            "Recoveries: %d",
            current_goal_.pose.position.x,
            current_goal_.pose.position.y,
            distance_remaining_,
            navigation_time_,
            moving_ ? "MOVING" : "STATIONARY",
            recovery_count_);
    }

    // ---------------------------------------------------------
    // STATUS PRINTING
    // ---------------------------------------------------------

    void print_status()
    {
        RCLCPP_INFO(
            this->get_logger(),
            "\nSTATUS: %s",
            readiness_ ? "READY" : "NOT READY");

        if (!readiness_)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "Missing:");

            if (!odom_ready_)
            {
                RCLCPP_INFO(
                    this->get_logger(),
                    "  - odom");
            }

            if (!localization_ready_)
            {
                RCLCPP_INFO(
                    this->get_logger(),
                    "  - amcl");
            }

            if (!nav2_ready_)
            {
                RCLCPP_INFO(
                    this->get_logger(),
                    "  - nav2");
            }

            if (!controller_ready_)
            {
                RCLCPP_INFO(
                    this->get_logger(),
                    "  - controller");
            }
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Mission state: %s",
            state_to_string(state_).c_str());
    }

    // ---------------------------------------------------------
    // GOAL CALLBACK
    // ---------------------------------------------------------

    void goal_callback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Goal received: x=%.2f, y=%.2f",
            msg->pose.position.x,
            msg->pose.position.y);

        // Temporary behavior until readiness_timeout is implemented.
        if (!readiness_)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Mission system is NOT READY. Goal was not sent.");

            return;
        }

        set_state(MissionState::PREPARING);

        send_navigation_goal(*msg);
    }

    // ---------------------------------------------------------
    // ODOM CALLBACK
    // ---------------------------------------------------------

    void odom_callback(
        const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        odom_received_ = true;

        double linear =
            std::abs(msg->twist.twist.linear.x);

        double angular =
            std::abs(msg->twist.twist.angular.z);

        moving_ =
            linear > 0.01 ||
            angular > 0.02;
    }

    // ---------------------------------------------------------
    // AMCL CALLBACK
    // ---------------------------------------------------------

    void amcl_callback(
        const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr)
    {
        localization_received_ = true;
    }

    // ---------------------------------------------------------
    // CONTROLLER CHECK
    // ---------------------------------------------------------

    void check_controller()
    {
        if (!controller_client_->service_is_ready())
        {
            controller_active_ = false;
            return;
        }

        if (controller_request_pending_)
        {
            return;
        }

        controller_request_pending_ = true;

        auto request =
            std::make_shared<
                controller_manager_msgs::srv::ListControllers::Request>();

        controller_client_->async_send_request(
            request,
            [this](
                rclcpp::Client<
                    controller_manager_msgs::srv::ListControllers>::SharedFuture future)
            {
                controller_request_pending_ = false;
                controller_active_ = false;

                auto response = future.get();

                for (const auto &controller : response->controller)
                {
                    if (
                        controller.name == "diff_drive_controller" &&
                        controller.state == "active")
                    {
                        controller_active_ = true;
                        break;
                    }
                }
            });
    }

    // ---------------------------------------------------------
    // SEND GOAL TO NAV2
    // ---------------------------------------------------------

    void send_navigation_goal(
        const geometry_msgs::msg::PoseStamped &goal_pose)
    {
        NavigateToPose::Goal goal;

        goal.pose = goal_pose;

        current_goal_ = goal.pose;

        distance_remaining_ = 0.0;
        navigation_time_ = 0.0;
        recovery_count_ = 0;
        moving_ = false;

        auto send_goal_options =
            rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

        // -------------------------------------------------
        // GOAL ACCEPT / REJECT
        // -------------------------------------------------

        send_goal_options.goal_response_callback =
            [this](
                GoalHandleNavigateToPose::SharedPtr goal_handle)
        {
            if (!goal_handle)
            {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Navigation goal rejected.");

                set_state(MissionState::FAILED);
                return;
            }

            RCLCPP_INFO(
                this->get_logger(),
                "Navigation goal accepted.");

            set_state(MissionState::NAVIGATING);
        };

        // -------------------------------------------------
        // FINAL RESULT
        // -------------------------------------------------

        send_goal_options.result_callback =
            std::bind(
                &MissionManager::result_callback,
                this,
                std::placeholders::_1);

        // -------------------------------------------------
        // NAVIGATION FEEDBACK
        // -------------------------------------------------

        send_goal_options.feedback_callback =
            [this](
                GoalHandleNavigateToPose::SharedPtr,
                const std::shared_ptr<
                    const NavigateToPose::Feedback>
                    feedback)
        {
            distance_remaining_ =
                feedback->distance_remaining;

            navigation_time_ =
                feedback->navigation_time.sec +
                feedback->navigation_time.nanosec * 1e-9;

            int recoveries =
                static_cast<int>(
                    feedback->number_of_recoveries);

            if (recoveries != recovery_count_)
            {
                recovery_count_ = recoveries;
            }
        };

        nav_client_->async_send_goal(
            goal,
            send_goal_options);
    }

    // ---------------------------------------------------------
    // NAVIGATION RESULT CALLBACK
    // ---------------------------------------------------------

    void result_callback(
        const GoalHandleNavigateToPose::WrappedResult &result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:

            set_state(MissionState::SUCCEEDED);
            break;

        case rclcpp_action::ResultCode::ABORTED:

            set_state(MissionState::FAILED);
            break;

        case rclcpp_action::ResultCode::CANCELED:

            set_state(MissionState::CANCELLED);
            break;

        default:

            set_state(MissionState::FAILED);
            break;
        }
    }

    // ---------------------------------------------------------
    // STATE -> STRING
    // ---------------------------------------------------------

    std::string state_to_string(MissionState state)
    {
        switch (state)
        {
        case MissionState::IDLE:
            return "IDLE";

        case MissionState::PREPARING:
            return "PREPARING";

        case MissionState::NAVIGATING:
            return "NAVIGATING";

        case MissionState::SUCCEEDED:
            return "SUCCEEDED";

        case MissionState::FAILED:
            return "FAILED";

        case MissionState::CANCELLED:
            return "CANCELLED";
        }

        return "UNKNOWN";
    }

    // ---------------------------------------------------------
    // CHANGE STATE
    // ---------------------------------------------------------

    void set_state(MissionState new_state)
    {
        if (state_ == new_state)
        {
            return;
        }

        state_ = new_state;

        std_msgs::msg::String msg;
        msg.data = state_to_string(state_);

        state_pub_->publish(msg);

        print_status();
    }

    // ---------------------------------------------------------
    // READINESS CHECK
    // ---------------------------------------------------------

    void check_readiness()
    {
        check_controller();

        // Check that publishers currently exist.
        bool odom_publisher_present =
            this->count_publishers("/odom") > 0;

        bool amcl_publisher_present =
            this->count_publishers("/amcl_pose") > 0;

        // If publisher disappears, require a new message
        // when it comes back.
        if (!odom_publisher_present)
        {
            odom_received_ = false;
        }

        if (!amcl_publisher_present)
        {
            localization_received_ = false;
        }

        bool new_odom_ready =
            odom_publisher_present &&
            odom_received_;

        bool new_localization_ready =
            amcl_publisher_present &&
            localization_received_;

        bool new_nav2_ready =
            nav_client_->action_server_is_ready();

        bool new_controller_ready =
            controller_active_;

        bool new_readiness =
            new_odom_ready &&
            new_localization_ready &&
            new_nav2_ready &&
            new_controller_ready;

        // Check whether any visible readiness condition changed.
        bool changed =
            new_odom_ready != odom_ready_ ||
            new_localization_ready != localization_ready_ ||
            new_nav2_ready != nav2_ready_ ||
            new_controller_ready != controller_ready_ ||
            new_readiness != readiness_;

        odom_ready_ =
            new_odom_ready;

        localization_ready_ =
            new_localization_ready;

        nav2_ready_ =
            new_nav2_ready;

        controller_ready_ =
            new_controller_ready;

        readiness_ =
            new_readiness;

        if (changed)
        {
            print_status();
        }
    }

    // ---------------------------------------------------------
    // ROS INTERFACES
    // ---------------------------------------------------------

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
        state_pub_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
        goal_sub_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
        odom_sub_;

    rclcpp::Subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
        amcl_sub_;

    rclcpp_action::Client<NavigateToPose>::SharedPtr
        nav_client_;

    rclcpp::Client<
        controller_manager_msgs::srv::ListControllers>::SharedPtr
        controller_client_;

    // ---------------------------------------------------------
    // TIMER
    // ---------------------------------------------------------

    rclcpp::TimerBase::SharedPtr
        timer_ready_;

    // ---------------------------------------------------------
    // INTERNAL STATE
    // ---------------------------------------------------------

    MissionState state_{MissionState::IDLE};

    bool odom_received_{false};
    bool localization_received_{false};

    bool controller_active_{false};
    bool controller_request_pending_{false};

    bool readiness_{false};

    bool odom_ready_{false};
    bool localization_ready_{false};
    bool nav2_ready_{false};
    bool controller_ready_{false};

    double distance_remaining_{0.0};
    double navigation_time_{0.0};

    int recovery_count_{0};

    bool moving_{false};

    geometry_msgs::msg::PoseStamped
        current_goal_;
};

// ---------------------------------------------------------
// MAIN
// ---------------------------------------------------------

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<MissionManager>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}