#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class MissionManager : public rclcpp::Node
{
public:
    MissionManager()
    : Node("mission_manager")
    {
        this->declare_parameter<double>("goal_x", 0.0);
        this->declare_parameter<double>("goal_y", 0.0);
        this->declare_parameter<double>("goal_yaw", 0.0);

        timer_ = this->create_wall_timer(
            2s,
            std::bind(&MissionManager::status_callback, this)
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Mission Manager started."
        );
    }

private:
    void status_callback()
    {
        double goal_x = this->get_parameter("goal_x").as_double();
        double goal_y = this->get_parameter("goal_y").as_double();
        double goal_yaw = this->get_parameter("goal_yaw").as_double();

        RCLCPP_INFO(
            this->get_logger(),
            "Current navigation goal: x=%.2f, y=%.2f, yaw=%.2f",
            goal_x,
            goal_y,
            goal_yaw
        );
    }

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MissionManager>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}