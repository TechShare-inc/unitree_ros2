
#include <unistd.h>
#include <cmath>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"
// please respqct the order here
#include "unitree_go/msg/low_state.hpp"
#include "unitree_interfaces/msg/gait_cmd.hpp"   
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
#include <std_msgs/msg/string.hpp>
#include "std_srvs/srv/trigger.hpp"
#define INFO_IMU 0        // Set 1 to info IMU states
#define INFO_MOTOR 0      // Set 1 to info motor states
#define INFO_FOOT_FORCE 0 // Set 1 to info foot force states
#define INFO_BATTERY 0    // Set 1 to info battery states

#define HIGH_FREQ 1 // Set 1 to subscribe to low states with high frequencies (500Hz)
using std::placeholders::_1;
// Create a B2DDS class for soprt commond request
enum KeyValue
{
    START = 4,
    L1_A = 258,
    L2_R2 = 48,
    L1_Y = 2050
};
class B2DDS : public rclcpp::Node
{
public:
    B2DDS() : Node("go2_dds")
    {
        this->declare_parameter<std::string>("cmd_vel_topic", "b2_cmd_vel");
                // 2. Retrieve the parameter value
        std::string cmd_vel_topic;
        this->get_parameter("cmd_vel_topic", cmd_vel_topic);
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            cmd_vel_topic, 1, std::bind(&B2DDS::cmdVelCallback, this, std::placeholders::_1));
        remote_controller_sub_ = this->create_subscription<techshare_ros_pkg2::msg::ControllerMsg>(
            "controller_status", 1, std::bind(&B2DDS::remoteControllerCallback, this, std::placeholders::_1));
        // the cmd_puber is set to subscribe "/wirelesscontroller" topic
        wireless_sub_ = this->create_subscription<unitree_go::msg::WirelessController>(
            "/wirelesscontroller", 10, std::bind(&B2DDS::wirelessControllerCallback, this, _1));
        low_state_sub_ = this->create_subscription<unitree_go::msg::LowState>(
            "/lowstate", 1, std::bind(&B2DDS::lowStateCallback, this, _1));
        sportmode_state_sub_ = this->create_subscription<unitree_go::msg::SportModeState>(
            "/lf/sportmodestate", 1, std::bind(&B2DDS::sportmodeStateCallback, this, _1));
        rosgaitcmd_sub_ = this->create_subscription<unitree_interfaces::msg::GaitCmd>(
            "rosgaitcmd", 10, std::bind(&B2DDS::rosgaitcmdCallback, this, std::placeholders::_1));
        key_value_pub_ = this->create_publisher<std_msgs::msg::String>("remote_toweb_cmd",10);
        kill_all_client_ = this->create_client<std_srvs::srv::Trigger>("killall");

        // the req_puber is set to subscribe "/api/sport/request" topic with dt
        req_puber = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);
    };

private:
    // void driving_mode_cb(const std_msgs::msg::Int8::SharedPtr msg){
    //     driving_mode = msg->data;
    // }

    void sportmodeStateCallback(const unitree_go::msg::SportModeState::SharedPtr msg)
    {
        /*
        class TerrainType(Enum):

            #posture
            BALANCED = 1
            NORMAL = 3
            LAYDOWN = 5
            FIXEDSTAND = 6
            DUMP = 7

            #special mode
            CLIMBUP = 103
            CLIMBDOWN = 104
            SLOPEUP = 203
            SLOPEDOWN = 205
        the above enum represents a drivining mode index in HALNA SYSTEM
        */
        rosgaitstate = *msg;
        gait_type_ = msg->gait_type;
        RCLCPP_INFO(
            this->get_logger(),
            "\033[1;33mMode: %d | gait_type: %d\033[0m",
            msg->mode,
            msg->gait_type // Use the function to get the string representation
        );

        if (msg->mode == 7)//DUMP
        {
            stand = false;
            driving_mode = 7;
        }else if (msg->mode == 6) // FIXED STAND
        {
            driving_mode = 6;  
        }else if (msg->mode == 1) //BALANCED STAND
        {
            driving_mode = 1;
        }else if (msg->mode == 3) //NORMAL
        {
            driving_mode = 3;
        }
        if ((msg->mode == 1 || msg->mode == 3) && msg->gait_type == 3){ //climb up
            driving_mode = 103;
        }else if ((msg->mode == 1 || msg->mode == 3) && msg->gait_type == 4){ //climb down
            driving_mode = 104;
        }
        // if (driving_mode != prev_driving_mode){
        //     RCLCPP_INFO(this->get_logger(), "\033[1;33m----->Changing the mode from %d to %d\033[0m", prev_driving_mode, driving_mode);
        //     {
        //         if(drivemode_srv_client_flag){
        //             auto message_request = std::make_shared<techshare_ros_pkg2::srv::ChangeDriveMode::Request>();
        //             message_request->mode = driving_mode;
        //             change_drivemode_srv_client_->async_send_request(message_request);
        //         }
        //         prev_driving_mode = driving_mode;
        //     }
        // }

    }
      void lowStateCallback(unitree_go::msg::LowState::SharedPtr data)
    {
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

    // void send_request(int& value) {
    //     auto request = std::make_shared<techshare_ros_pkg2::srv::ChangeDriveMode::Request>();
    //     request->mode = value;

    //     // Wait for the service to be available. You can specify a timeout duration.
    //     if (change_drivemode_srv_client_->wait_for_service(std::chrono::seconds(5))) {
    //         // Service is available, send the request.
    //         auto future_result = change_drivemode_srv_client_->async_send_request(request,
    //             [](rclcpp::Client<techshare_ros_pkg2::srv::ChangeDriveMode>::SharedFuture returned_future) {
    //                 // This lambda function is the callback and will be called once the service response is received.
    //                 // You can process the response here.
    //                 auto response = returned_future.get();
    //                 // For example, log the response (this is just a placeholder, adapt it to your actual response type)
    //                 RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received response: %d", response->success);
    //             });
    //     } else {
    //         // Service not available after waiting.
    //         RCLCPP_ERROR(this->get_logger(), "Service change_driving_mode not available after waiting");
    //     }
    // }


    void remoteControllerCallback(const techshare_ros_pkg2::msg::ControllerMsg::SharedPtr msg){
        std::lock_guard<std::mutex> lock(mutex_); 
        remotelyControlled = true;
        last_message_time_ = this->get_clock()->now();
        bool action = true;
        unitree_api::msg::Request req_; // Unitree Go2 ROS2 request message
        /*
        Gait enumeration value, with values ranging from 0 to 4, 
        where 0 is idle, 1 is trot, 2 is trot running, 
        3 is forward climbing mode, and 4 is reverse climbing mode
        */

        bool continuousGaitFlag= false;
        if (msg->start && msg->right){
            RCLCPP_INFO(this->get_logger(), "\033[1;33m----->forward climbing mode\033[0m");
            sport_req.SwitchGait(req_, 3);
            continuousGaitFlag = true;
        }else if (msg->start && msg->left){
            RCLCPP_INFO(this->get_logger(), "\033[1;33m----->reverse climbing mode\033[0m");
            sport_req.SwitchGait(req_, 4);
            continuousGaitFlag = true;
        }else if (msg->l2 && msg->a && (driving_mode != 5 && driving_mode !=7 ) && stand){
            RCLCPP_INFO(this->get_logger(), "\033[1;33m----->StandDown mode\033[0m");
            sport_req.StandDown(req_);
        }else if (msg->l2 && msg->a){
            RCLCPP_INFO(this->get_logger(), "\033[1;33m----->StandUp mode\033[0m");
            sport_req.StandUp(req_);
        }else if (msg->l2 && msg->b && driving_mode == 7){
            RCLCPP_INFO(this->get_logger(), "\033[1;33m----->Damp mode\033[0m");
            sport_req.Damp(req_);
            stand = false;
        } else if (msg->start){
            RCLCPP_INFO(this->get_logger(), "\033[1;33m----->BalanceStand mode\033[0m");
            sport_req.SwitchGait(req_, 1);
            stand = true;
        }else{
            action = false;
        }
        if(action){
            req_puber->publish(req_);
            if (continuousGaitFlag){
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                sport_req.ContinuousGait(req_, continuousGaitFlag);
            }
        }




    }


    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_); 
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
                    std::bind(&B2DDS::dampFunction, this));
            }else {
                RCLCPP_INFO(this->get_logger(), "\033[1;34m----->Stand up\033[0m");
                sport_req.StandUp(req);
            }
            req_puber->publish(req);
        }else{
            stand = true;
            if (driving_mode == 6){
                sport_req.BalanceStand(req);
                req_puber->publish(req);
            }
            RCLCPP_INFO(this->get_logger(), "\033[1;32m----->Moving\033[0m");
            sport_req.Move(req, msg->linear.x, msg->linear.y, msg->angular.z);
            req_puber->publish(req);
        }
    }
    void rosgaitcmdCallback(const unitree_interfaces::msg::GaitCmd::SharedPtr msg){
        static unitree_api::msg::Request req_; // Unitree Go2 ROS2 request message
        if (std::abs(msg->velocity[0]) < 1e-9 &&
            std::abs(msg->velocity[1]) < 1e-9 &&
            std::abs(msg->yaw_speed) < 1e-9)
        {
            return;
        }
        if (driving_mode == 6){
            sport_req.BalanceStand(req_);
            req_puber->publish(req_);
            return;
        }
    }
    void wirelessControllerCallback(const unitree_go::msg::WirelessController::SharedPtr data)
    {
        static bool l2_r2 = false;
        static bool start = true;
        static bool l1_a = false;
        static bool l1_y = false;
        static std_msgs::msg::String ss;
        uint16_t keyValue = data->keys;
        if(keyValue == L2_R2 && !l2_r2){
            l2_r2 = true;
            //call killall
            auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
            kill_all_client_->async_send_request(request);
            RCLCPP_INFO(this->get_logger(), "\033[1;32mKey value L2+R2(call killall service) has been  pressed.\033[0m");
        }else if (keyValue == L1_A && !l1_a){
            //call key add point
            l1_a = true;
            start = false;
            RCLCPP_INFO(this->get_logger(), "\033[1;32mKey value L1+A(add a point) has been  pressed.\033[0m");
            ss.data = "L1A";
            key_value_pub_->publish(ss);
        }else if (keyValue == L1_Y && !l1_y){
            //call key add point
            l1_y = true;
            start = false;
            RCLCPP_INFO(this->get_logger(), "\033[1;32mKey value L1+Y(save_graph) has been  pressed.\033[0m");
            ss.data = "L1Y";
            key_value_pub_->publish(ss);
        }else if (keyValue == START){
            start = true;
            l1_a = false;
            l1_y = false;
            l2_r2 = false;
            RCLCPP_INFO(this->get_logger(), "\033[1;32mKey value START has been pressed.\033[0m");

        }
        RCLCPP_INFO(this->get_logger(), "Wireless controller -- lx: %f; ly: %f; rx: %f; ry: %f; key value: %d",
                    data->lx, data->ly, data->rx, data->ry, data->keys);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<unitree_go::msg::WirelessController>::SharedPtr wireless_sub_;
    rclcpp::Subscription<unitree_interfaces::msg::GaitCmd>::SharedPtr rosgaitcmd_sub_;

    // Create the suber  to receive low state of robot
    rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr low_state_sub_;
    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr sportmode_state_sub_;
    rclcpp::Subscription<techshare_ros_pkg2::msg::ControllerMsg>::SharedPtr remote_controller_sub_;
    rclcpp::Client<techshare_ros_pkg2::srv::ChangeDriveMode>::SharedPtr change_drivemode_srv_client_;
    // rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr driving_mode_sub_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr key_value_pub_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr kill_all_client_;

    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_puber;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    unitree_go::msg::IMUState imu;         // Unitree go2 IMU message
    unitree_go::msg::MotorState motor[12]; // Unitree go2 motor state message
    int16_t foot_force[4];                 // External contact force value (int)
    int16_t foot_force_est[4];             // Estimated  external contact force value (int)
    float battery_voltage;                 // Battery voltage
    float battery_current;                 // Battery current
    bool fixed_stand;
    bool remotelyControlled, stand;
    bool imu_msg_flag, dog_odom_flag, drivemode_srv_client_flag;
    int driving_mode = 0;
    int prev_driving_mode = 0;
    rclcpp::Time last_message_time_;
    rclcpp::TimerBase::SharedPtr timer_, delay_timer_ ,rawDataTimer; // ROS2 timer
    unitree_api::msg::Request req; // Unitree Go2 ROS2 request message
    unitree_go::msg::SportModeState rosgaitstate;
    SportClient sport_req;
    std::mutex mutex_;
    //link names
    std::string robotMovement;
    int gait_type_ =0;
    double t; // runing time count
    double dt = 0.002; //control time step

    double px0 = 0;  // initial x position
    double py0 = 0;  // initial y position
    double yaw0 = 0; // initial yaw angle
    std::once_flag flag;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv); // Initialize rclcpp
    rclcpp::spin(std::make_shared<B2DDS>()); //Run ROS2 node
    rclcpp::shutdown();
    return 0;
}

