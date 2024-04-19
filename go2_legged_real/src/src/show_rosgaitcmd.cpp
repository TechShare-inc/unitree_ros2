
#include <unistd.h>
#include <cmath>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"
// please respqct the order here
#include "unitree_interfaces/msg/gait_cmd.hpp"   
#include "unitree_go/msg/low_state.hpp"
#include "unitree_api/msg/request.hpp"
#include "common/ros2_sport_client.h"
#include "geometry_msgs/msg/twist.hpp"
#include "unitree_go/msg/wireless_controller.hpp"
#include "techshare_ros_pkg2/msg/controller_msg.hpp"
#include "techshare_ros_pkg2/srv/change_drive_mode.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <std_msgs/msg/int8.hpp>
#include <chrono>
class GaitCmdListener : public rclcpp::Node
{
public:
    GaitCmdListener() : Node("gaitcmd_listener")
    {
        subscription_ = this->create_subscription<unitree_interfaces::msg::GaitCmd>(
            "rosgaitcmd", 10, std::bind(&GaitCmdListener::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const unitree_interfaces::msg::GaitCmd::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received GaitCmd:");
        RCLCPP_INFO(this->get_logger(), "  Mode: %u", msg->mode);
        RCLCPP_INFO(this->get_logger(), "  Gait Type: %u", msg->gait_type);
        RCLCPP_INFO(this->get_logger(), "  Speed Level: %u", msg->speed_level);
        RCLCPP_INFO(this->get_logger(), "  Foot Raise Height: %f", msg->foot_raise_height);
        RCLCPP_INFO(this->get_logger(), "  Body Height: %f", msg->body_height);
        RCLCPP_INFO(this->get_logger(), "  Position: [%f, %f]", msg->position[0], msg->position[1]);
        RCLCPP_INFO(this->get_logger(), "  Euler Angles: [%f, %f, %f]", msg->euler[0], msg->euler[1], msg->euler[2]);
        RCLCPP_INFO(this->get_logger(), "  Velocity: [%f, %f]", msg->velocity[0], msg->velocity[1]);
        RCLCPP_INFO(this->get_logger(), "  Yaw Speed: %f", msg->yaw_speed);
    }

    rclcpp::Subscription<unitree_interfaces::msg::GaitCmd>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GaitCmdListener>());
    rclcpp::shutdown();
    return 0;
}
