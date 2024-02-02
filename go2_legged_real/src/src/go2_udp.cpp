
#include <unistd.h>
#include <cmath>
#include <chrono>
#include "rclcpp/rclcpp.hpp"

// please respqct the order here
#include "unitree_go/msg/low_state.hpp"
#include "unitree_go/msg/imu_state.hpp"
#include "unitree_go/msg/motor_state.hpp"
#include "unitree_api/msg/request.hpp"
#include "common/ros2_sport_client.h"
#include "geometry_msgs/msg/twist.hpp"
#include "unitree_go/msg/wireless_controller.hpp"
#include "techshare_ros_pkg2/msg/controller_msg.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"
#include <chrono>
#define INFO_IMU 0        // Set 1 to info IMU states
#define INFO_MOTOR 0      // Set 1 to info motor states
#define INFO_FOOT_FORCE 0 // Set 1 to info foot force states
#define INFO_BATTERY 0    // Set 1 to info battery states

#define HIGH_FREQ 1 // Set 1 to subscribe to low states with high frequencies (500Hz)

using std::placeholders::_1;
// Create a GO2UDP class for soprt commond request
class GO2UDP : public rclcpp::Node
{
public:
    GO2UDP() : Node("go2_udp"), fixed_stand(true),remotelyControlled(false), damped(false)
    {
        this->declare_parameter<std::string>("cmd_vel_topic", "go2_cmd_vel");
                // 2. Retrieve the parameter value
        std::string cmd_vel_topic;
        this->get_parameter("cmd_vel_topic", cmd_vel_topic);
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            cmd_vel_topic, 1, std::bind(&GO2UDP::cmdVelCallback, this, std::placeholders::_1));
        remote_controller_sub_ = this->create_subscription<techshare_ros_pkg2::msg::ControllerMsg>(
            "controller_status", 1, std::bind(&GO2UDP::remoteControllerCallback, this, std::placeholders::_1));
        // the cmd_puber is set to subscribe "/wirelesscontroller" topic
        wireless_sub_ = this->create_subscription<unitree_go::msg::WirelessController>(
            "/wirelesscontroller", 10, std::bind(&GO2UDP::wirelessControllerCallback, this, _1));
        // the req_puber is set to subscribe "/api/sport/request" topic with dt
        req_puber = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);

        low_state_sub_ = this->create_subscription<unitree_go::msg::LowState>(
            "lowstate", 1, std::bind(&GO2UDP::lowStateCallback, this, _1));
        t = -1; // Runing time count
                // make a fake covarance here
        last_message_time_ = this->get_clock()->now();
        makeFakeCovariance(imu_msg);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&GO2UDP::timerCallback, this)
        );
    };

private:
      void lowStateCallback(unitree_go::msg::LowState::SharedPtr data)
    {
        imu = data->imu_state;
        publishImuData();
    if (INFO_IMU)
    {
      // Info IMU states
      // RPY euler angle(ZYX order respected to body frame)
      // Quaternion
      // Gyroscope (raw data)
      // Accelerometer (raw data)
      

      RCLCPP_INFO(this->get_logger(), "\033[33mEuler angle -- roll: %f; pitch: %f; yaw: %f\033[0m", imu.rpy[0], imu.rpy[1], imu.rpy[2]);
    //   RCLCPP_INFO(this->get_logger(), "\033[33mQuaternion -- qw: %f; qx: %f; qy: %f; qz: %f\033[0m",
    //               imu.quaternion[0], imu.quaternion[1], imu.quaternion[2], imu.quaternion[3]);
    //   RCLCPP_INFO(this->get_logger(), "\033[1;33mGyroscope -- wx: %f; wy: %f; wz: %f\033[0m", imu.gyroscope[0], imu.gyroscope[1], imu.gyroscope[2]);
    //   RCLCPP_INFO(this->get_logger(), "\033[1;33mAccelerometer -- ax: %f; ay: %f; az: %f\033[0m",
    //               imu.accelerometer[0], imu.accelerometer[1], imu.accelerometer[2]);
    }

    if (INFO_MOTOR)
    {
      // Info motor states
      // q: angluar (rad)
      // dq: angluar velocity (rad/s)
      // ddq: angluar acceleration (rad/(s^2))
      // tau_est: Estimated external torque

      for (int i = 0; i < 12; i++)
      {
        motor[i] = data->motor_state[i];
        RCLCPP_INFO(this->get_logger(), "Motor state -- num: %d; q: %f; dq: %f; ddq: %f; tau: %f",
                    i, motor[i].q, motor[i].dq, motor[i].ddq, motor[i].tau_est);
      }
    }

    if (INFO_FOOT_FORCE)
    {
      // Info foot force value (int not true value)
      for (int i = 0; i < 4; i++)
      {
        foot_force[i] = data->foot_force[i];
        foot_force_est[i] = data->foot_force_est[i];
      }

      RCLCPP_INFO(this->get_logger(), "Foot force -- foot0: %d; foot1: %d; foot2: %d; foot3: %d",
                  foot_force[0], foot_force[1], foot_force[2], foot_force[3]);
      RCLCPP_INFO(this->get_logger(), "Estimated foot force -- foot0: %d; foot1: %d; foot2: %d; foot3: %d",
                  foot_force_est[0], foot_force_est[1], foot_force_est[2], foot_force_est[3]);
    }

    if (INFO_BATTERY)
    {
      // Info battery states
      // battery current
      // battery voltage
      battery_current = data->power_a;
      battery_voltage = data->power_v;

      RCLCPP_INFO(this->get_logger(), "Battery state -- current: %f; voltage: %f", battery_current, battery_voltage);
    }
  }

    void dampFunction()
    {
        RCLCPP_INFO(this->get_logger(), "\033[1;33m----->Damp\033[0m");
        sport_req.Damp(req);
        req_puber->publish(req);
        delay_timer_->cancel();
    }

    void timerCallback() {
        // Check time since last message
        auto now = this->get_clock()->now();
        if ((now - last_message_time_).seconds() >= 1.0) {
            remotelyControlled = false;
            RCLCPP_INFO(this->get_logger(), "No message received for more than 1 second. Setting remotelyControlled to false.");
        }
    }

    void remoteControllerCallback(const techshare_ros_pkg2::msg::ControllerMsg::SharedPtr msg){
        remotelyControlled = true;
        last_message_time_ = this->get_clock()->now();
        bool action = true;
        unitree_api::msg::Request req_; // Unitree Go2 ROS2 request message
        /*
        Gait enumeration value, with values ranging from 0 to 4, 
        where 0 is idle, 1 is trot, 2 is trot running, 
        3 is forward climbing mode, and 4 is reverse climbing mode
        */

        if (msg->start && msg->right){
            RCLCPP_INFO(this->get_logger(), "\033[1;33m----->forward climbing mode\033[0m");
            sport_req.SwitchGait(req_, 3);
        }else if (msg->start && msg->left){
            RCLCPP_INFO(this->get_logger(), "\033[1;33m----->reverse climbing mode\033[0m");
            sport_req.SwitchGait(req_, 4);
        }else if (msg->l2 && msg->a && !damped){
            RCLCPP_INFO(this->get_logger(), "\033[1;33m----->StandDown mode\033[0m");
            sport_req.StandDown(req_);
        }else if (msg->l2 && msg->a && damped){
            RCLCPP_INFO(this->get_logger(), "\033[1;33m----->StandUp mode\033[0m");
            sport_req.StandUp(req_);
        }else if (msg->l2 && msg->b){
            RCLCPP_INFO(this->get_logger(), "\033[1;33m----->Damp mode\033[0m");
            sport_req.Damp(req_);
            damped = true;
            fixed_stand = true;
        } else if (msg->start){
            RCLCPP_INFO(this->get_logger(), "\033[1;33m----->BalanceStand mode\033[0m");
            sport_req.SwitchGait(req_, 1);
            damped = false;
            // sport_req.BalanceStand(req_);
        }else{
            action = false;
        }
        if(action)
            req_puber->publish(req_);



    }


    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (msg->linear.z != 0.0 && !remotelyControlled){
            // stop the move
            // sport_req.StopMove(req);
            // req_puber->publish(req);

            if (msg->linear.z < 0){
                RCLCPP_INFO(this->get_logger(), "\033[1;36m----->Stand down\033[0m");
                sport_req.StandDown(req);
                                // Set up the timer for 2 seconds delay
                delay_timer_ = this->create_wall_timer(
                    std::chrono::seconds(2), 
                    std::bind(&GO2UDP::dampFunction, this));
            }else {
                RCLCPP_INFO(this->get_logger(), "\033[1;34m----->Stand up\033[0m");
                sport_req.StandUp(req);
            }
            req_puber->publish(req);
            fixed_stand = true;
        }else{
            if (fixed_stand){
                sport_req.BalanceStand(req);
                req_puber->publish(req);
                fixed_stand = false;
            }
            RCLCPP_INFO(this->get_logger(), "\033[1;32m----->Moving\033[0m");
            sport_req.Move(req, msg->linear.x, msg->linear.y, msg->angular.z);
            req_puber->publish(req);
        }
    }
    void makeFakeCovariance(sensor_msgs::msg::Imu& imu_msg_){
        // Set the orientation covariance matrix
        imu_msg_.orientation_covariance[1] = 0.0;
        imu_msg_.orientation_covariance[2] = 0.0;
        imu_msg_.orientation_covariance[3] = 0.0;
        imu_msg_.orientation_covariance[4] = 0.0207;
        imu_msg_.orientation_covariance[0] = 0.0479;
        imu_msg_.orientation_covariance[5] = 0.0;
        imu_msg_.orientation_covariance[6] = 0.0;
        imu_msg_.orientation_covariance[7] = 0.0;
        imu_msg_.orientation_covariance[8] = 0.0041;

        // Set the linear acceleration covariance matrix
        imu_msg_.linear_acceleration_covariance[0] = 0.0364;
        imu_msg_.linear_acceleration_covariance[1] = 0.0;
        imu_msg_.linear_acceleration_covariance[2] = 0.0;
        imu_msg_.linear_acceleration_covariance[3] = 0.0;
        imu_msg_.linear_acceleration_covariance[4] = 0.0048;
        imu_msg_.linear_acceleration_covariance[5] = 0.0;
        imu_msg_.linear_acceleration_covariance[6] = 0.0;
        imu_msg_.linear_acceleration_covariance[7] = 0.0;
        imu_msg_.linear_acceleration_covariance[8] = 0.0796;

        // Set the angular velocity covariance matrix
        imu_msg_.angular_velocity_covariance[0] = 0.0663;
        imu_msg_.angular_velocity_covariance[1] = 0.0;
        imu_msg_.angular_velocity_covariance[2] = 0.0;
        imu_msg_.angular_velocity_covariance[3] = 0.0;
        imu_msg_.angular_velocity_covariance[4] = 0.1453;
        imu_msg_.angular_velocity_covariance[5] = 0.0;
        imu_msg_.angular_velocity_covariance[6] = 0.0;
        imu_msg_.angular_velocity_covariance[7] = 0.0;
        imu_msg_.angular_velocity_covariance[8] = 0.0378;

    }
    void publishImuData()
    {
        // Create and populate an IMU message
        
        imu_msg.header.stamp = this->now();
        imu_msg.header.frame_id = "imu_link";

        // Orientation data (assuming you have this data)
        imu_msg.orientation.x = imu.quaternion[0];
        imu_msg.orientation.y = imu.quaternion[1];
        imu_msg.orientation.z = imu.quaternion[2];
        imu_msg.orientation.w = imu.quaternion[3];

        // Angular velocity data
        imu_msg.angular_velocity.x = imu.gyroscope[0];
        imu_msg.angular_velocity.y = imu.gyroscope[1];
        imu_msg.angular_velocity.z = imu.gyroscope[2];

        // Linear acceleration data
        imu_msg.linear_acceleration.x = imu.accelerometer[0];
        imu_msg.linear_acceleration.y = imu.accelerometer[1];
        imu_msg.linear_acceleration.z = imu.accelerometer[2];

        // Publish the IMU message
        imu_pub_->publish(imu_msg);
    }



    void wirelessControllerCallback(const unitree_go::msg::WirelessController::SharedPtr data)
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
    // Create the suber  to receive low state of robot
    rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr low_state_sub_;
    rclcpp::Subscription<techshare_ros_pkg2::msg::ControllerMsg>::SharedPtr remote_controller_sub_;



    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_puber;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    unitree_go::msg::IMUState imu;         // Unitree go2 IMU message
    unitree_go::msg::MotorState motor[12]; // Unitree go2 motor state message
    int16_t foot_force[4];                 // External contact force value (int)
    int16_t foot_force_est[4];             // Estimated  external contact force value (int)
    float battery_voltage;                 // Battery voltage
    float battery_current;                 // Battery current
    bool fixed_stand;
    bool remotelyControlled, damped;
    rclcpp::Time last_message_time_;
    rclcpp::TimerBase::SharedPtr timer_, delay_timer_; // ROS2 timer
    sensor_msgs::msg::Imu imu_msg;
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
    rclcpp::spin(std::make_shared<GO2UDP>()); //Run ROS2 node
    rclcpp::shutdown();
    return 0;
}


