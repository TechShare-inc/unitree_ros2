
#include <unistd.h>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"

#include "unitree_api/msg/request.hpp"
#include "common/ros2_sport_client.h"
#include "geometry_msgs/msg/twist.hpp"
#include "unitree_go/msg//wireless_controller.hpp"


using std::placeholders::_1;
// Create a soprt_request class for soprt commond request
class soprt_request : public rclcpp::Node
{
public:
    soprt_request() : Node("req_sender")
    {
        // // the state_suber is set to subscribe "sportmodestate" topic
        // state_suber = this->create_subscription<unitree_go::msg::SportModeState>(
        //     "sportmodestate", 10, std::bind(&soprt_request::state_callback, this, _1));
        this->declare_parameter<std::string>("cmd_vel_topic", "go2_cmd_vel");
                // 2. Retrieve the parameter value
        std::string cmd_vel_topic;
        this->get_parameter("cmd_vel_topic", cmd_vel_topic);
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            cmd_vel_topic, 1, std::bind(&soprt_request::cmdVelCallback, this, std::placeholders::_1));
        // the cmd_puber is set to subscribe "/wirelesscontroller" topic
        wireless_sub_ = this->create_subscription<unitree_go::msg::WirelessController>(
            "/wirelesscontroller", 10, std::bind(&soprt_request::wirelessControllerCallback, this, _1));
        // the req_puber is set to subscribe "/api/sport/request" topic with dt
        req_puber = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);
        // timer_ = this->create_wall_timer(std::chrono::milliseconds(int(dt * 1000)), std::bind(&soprt_request::timer_callback, this));

        t = -1; // Runing time count
    };

private:
    void cmdVelCallback(geometry_msgs::msg::Twist::SharedPtr msg)
    {
        
        sport_req.Move(req, msg->linear.x, msg->linear.y, msg->angular.z);
        req_puber->publish(req);
    }

    void wirelessControllerCallback(unitree_go::msg::WirelessController::SharedPtr data)
    {
        // lx: Left joystick x value
        // ly: Left joystick y value
        // rx: Right joystick x value
        // ry: Right joystick y value
        // keys value

        RCLCPP_INFO(this->get_logger(), "Wireless controller -- lx: %f; ly: %f; rx: %f; ry: %f; key value: %d",
                    data->lx, data->ly, data->rx, data->ry, data->keys);
    }

    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr state_suber;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<unitree_go::msg::WirelessController>::SharedPtr wireless_sub_;

    rclcpp::TimerBase::SharedPtr timer_; // ROS2 timer
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_puber;

    unitree_api::msg::Request req; // Unitree Go2 ROS2 request message
    SportClient sport_req;

    double t; // runing time count
    double dt = 0.002; //control time step

    double px0 = 0;  // initial x position
    double py0 = 0;  // initial y position
    double yaw0 = 0; // initial yaw angle
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv); // Initialize rclcpp
    rclcpp::TimerBase::SharedPtr timer_; // Create a timer callback object to send sport request in time intervals

    rclcpp::spin(std::make_shared<soprt_request>()); //Run ROS2 node

    rclcpp::shutdown();
    return 0;
}


