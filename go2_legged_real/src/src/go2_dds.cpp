
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
#include "techshare_ros_pkg2/srv/sdk_client.hpp"
#include <techshare_ros_pkg2/srv/zoom.hpp>
#include <techshare_ros_pkg2/srv/pan_tilt.hpp>
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/string.hpp>
#include "std_srvs/srv/trigger.hpp"
#include <chrono>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <semaphore.h>
#include <cstring>
#include <errno.h>
#include <iostream>
#include <ctime>
#include <iomanip>
#define INFO_IMU 0        // Set 1 to info IMU states
#define INFO_MOTOR 0      // Set 1 to info motor states
#define INFO_FOOT_FORCE 0 // Set 1 to info foot force states
#define INFO_BATTERY 0    // Set 1 to info battery states

#define HIGH_FREQ 1 // Set 1 to subscribe to low states with high frequencies (500Hz)
using std::placeholders::_1;
// Create a GO2DDS class for soprt commond request

enum KeyValue
{
    START = 4,
    L1_A = 258,
    L1_X = 1026,
    L2_R2 = 48,
    L1_Y = 2050,
    UP = 4096,
    RIGHT = 8192,
    LEFT = 32768,
    DOWN = 16384,
    R1R2 = 17,
    L1L2 = 34
};

struct SDK_CLIENT_DATA {
    char client_name[256]; // Fixed size array for client name
    float params[10];      // Fixed size array for parameters
    sem_t semaphore;
};


class GO2DDS : public rclcpp::Node
{
public:
    GO2DDS() : Node("go2_dds"), fixed_stand(true),remotelyControlled(false), stand(false),imu_msg_flag(false), dog_odom_flag(false),drivemode_srv_client_flag(false)
    {

        //for ipc ------------------------
        // Try to open existing shared memory
        shm_fd_ = shm_open("/sdk_client_shm", O_RDWR, 0666);
        if (shm_fd_ == -1) {
            // Shared memory doesn't exist yet, create it
            RCLCPP_INFO(this->get_logger(), "Creating shared memory...");
            shm_fd_ = shm_open("/sdk_client_shm", O_CREAT | O_RDWR, 0666);
            ftruncate(shm_fd_, sizeof(SDK_CLIENT_DATA));

            // Map the shared memory into the process
            shared_memory_ = static_cast<SDK_CLIENT_DATA*>(mmap(0, sizeof(SDK_CLIENT_DATA), PROT_WRITE, MAP_SHARED, shm_fd_, 0));

            if (shared_memory_ == MAP_FAILED) {
                RCLCPP_ERROR(this->get_logger(), "Failed to map shared memory");
                return;
            }

            // Initialize the semaphore
            sem_init(&shared_memory_->semaphore, 1, 0); // Shared between processes
            RCLCPP_INFO(this->get_logger(), "Shared memory and semaphore initialized.");
        } else {
            // Shared memory exists, just map it
            shared_memory_ = static_cast<SDK_CLIENT_DATA*>(mmap(0, sizeof(SDK_CLIENT_DATA), PROT_WRITE, MAP_SHARED, shm_fd_, 0));
            if (shared_memory_ == MAP_FAILED) {
                RCLCPP_ERROR(this->get_logger(), "Failed to map shared memory");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Shared memory mapped.");
        }       

        this->declare_parameter("imuFrame", "imu_link");
        this->declare_parameter("odomFrame", "odom");
        this->declare_parameter("imuTopic", "imu/data");
        this->declare_parameter("robotFrame", "base_link");
        this->declare_parameter("robotOdomTopic", "dog_odom");
        this->get_parameter("robotOdomTopic", robotOdometry);
        this->get_parameter("imuTopic", imuTopic);
        this->get_parameter("imuFrame", imuFrame);
        this->get_parameter("odomFrame", odomFrame);
        this->get_parameter("robotFrame", robotFrame);

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "msg_cmd_vel", 1, std::bind(&GO2DDS::cmdVelCallback, this, std::placeholders::_1));
        remote_controller_sub_ = this->create_subscription<techshare_ros_pkg2::msg::ControllerMsg>(
            "controller_status", 1, std::bind(&GO2DDS::remoteControllerCallback, this, std::placeholders::_1));
        // the cmd_puber is set to subscribe "/wirelesscontroller" topic
        wireless_sub_ = this->create_subscription<unitree_go::msg::WirelessController>(
            "/wirelesscontroller", 10, std::bind(&GO2DDS::wirelessControllerCallback, this, _1));
        rosgaitcmd_sub_ = this->create_subscription<unitree_interfaces::msg::GaitCmd>(
            "rosgaitcmd", 10, std::bind(&GO2DDS::rosgaitcmdCallback, this, std::placeholders::_1));

        sportmode_state_sub_ = this->create_subscription<unitree_go::msg::SportModeState>(
            "/sportmodestate", 1, std::bind(&GO2DDS::sportmodeStateCallback, this, _1));
        low_state_sub_ = this->create_subscription<unitree_go::msg::LowState>(
            "/lowstate", 1, std::bind(&GO2DDS::lowStateCallback, this, _1));

        // the req_puber is set to subscribe "/api/sport/request" topic with dt
        req_puber = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
        dog_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>(robotOdometry,1);
        key_value_pub_ = this->create_publisher<std_msgs::msg::String>("remote_toweb_cmd",10);
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("go2_cmd_vel",10);
        change_drivemode_srv_client_ = this->create_client<techshare_ros_pkg2::srv::ChangeDriveMode>("change_driving_mode");
        kill_all_client_ = this->create_client<std_srvs::srv::Trigger>("killall");

        go2_sdk_service_ = this->create_service<techshare_ros_pkg2::srv::SdkClient>(
            "unitree_sdk_client", std::bind(&GO2DDS::handle_client, this, std::placeholders::_1, std::placeholders::_2));

        last_message_time_ = this->get_clock()->now();
        zoom_level = 1.0;
        pan_tilt_req = std::make_shared<techshare_ros_pkg2::srv::PanTilt::Request>();
        pan_tilt_cli_ = this->create_client<techshare_ros_pkg2::srv::PanTilt>("pan_tilt");
        zoom_cli_ = this->create_client<techshare_ros_pkg2::srv::Zoom>("zoom");


        double pub_rate = 400.0f;
        double T = 1.0 / pub_rate * 1000.f;   
        auto time = std::chrono::duration<long, std::ratio<1, 1000>>(int(T));
        rawDataTimer = this->create_wall_timer(
            time,
            std::bind(&GO2DDS::rawDataPubCallback, this)
        );
        if (change_drivemode_srv_client_->wait_for_service(std::chrono::seconds(5))){
            drivemode_srv_client_flag = true;
        }
        if(isTimeInRange()){
            lightControl = true;
            light_level = 10;
        }
    };

    ~GO2DDS(){
        // Clean up
        munmap(shared_memory_, sizeof(SDK_CLIENT_DATA));
        close(shm_fd_);
        RCLCPP_INFO(this->get_logger(), "\033[1;33mClean up the memory\033[0m");

    }
private:
 
    void handle_client(const std::shared_ptr<techshare_ros_pkg2::srv::SdkClient::Request> request,
                          std::shared_ptr<techshare_ros_pkg2::srv::SdkClient::Response> response)
    {
        handleCommand(request->client_name, request->params);
        response->res = 1;
    }

    bool isTimeInRange() {
        // Get current time in UTC
        auto now = std::chrono::system_clock::now();
        std::time_t now_c = std::chrono::system_clock::to_time_t(now);

        // Convert to local time
        std::tm *local_time = std::gmtime(&now_c);

        // Convert to JST (UTC + 9 hours)
        local_time->tm_hour += 9;
        if (local_time->tm_hour >= 24) {
            local_time->tm_hour -= 24;
            local_time->tm_mday += 1;
        }

        // Check if the current time is between 15:00 - 0:00 or 0:00 - 6:00
        int hour = local_time->tm_hour;
        if ((hour >= 15 && hour < 24) || (hour >= 0 && hour < 6)) {
            return true;
        } else {
            return false;
        }
    }



    void handle_ptz(int cmd){
        switch (static_cast<KeyValue>(cmd)) {
            case RIGHT:{ // right
                pan_tilt_req->direction = "right";
                pan_tilt_req->speed = 100;
                pan_tilt_req->degrees = 5;
                pan_tilt_cli_->async_send_request(pan_tilt_req);
                break;
            }
            case LEFT:{ // left
                pan_tilt_req->direction = "left";
                pan_tilt_req->speed = 100;
                pan_tilt_req->degrees = 5;
                pan_tilt_cli_->async_send_request(pan_tilt_req);
                break;
            }
            case UP:{ // up
                pan_tilt_req->direction = "up";
                pan_tilt_req->speed = 100;
                pan_tilt_req->degrees = 5;
                pan_tilt_cli_->async_send_request(pan_tilt_req);
                break;
            }
            case DOWN:{ // down
                pan_tilt_req->direction = "down";
                pan_tilt_req->speed = 100;
                pan_tilt_req->degrees = 5;
                pan_tilt_cli_->async_send_request(pan_tilt_req);
                break;
            }
            case R1R2:{ // zoom in
                if (zoom_level < 30) zoom_level++;
                zoom_req->target = zoom_level;
                zoom_cli_->async_send_request(zoom_req);
                break;
            }
            case L1L2:{ // zoom out
                if (zoom_level > 1) zoom_level--;
                zoom_req->target = zoom_level;
                zoom_cli_->async_send_request(zoom_req);
                break;
            }
        }

    }


    void handleCommand(const std::string& client_name, const std::array<float, 10>& params) {
        strncpy(shared_memory_->client_name, client_name.c_str(), sizeof(shared_memory_->client_name) - 1);
        std::copy(params.begin(), params.end(), shared_memory_->params);
        sem_post(&shared_memory_->semaphore);
    }

    void rawDataPubCallback(){
        // Publish the IMU message
        if (imu_msg_flag && dog_odom_flag){
            // makeFakeCovariance(imu_msg); // this is called only once
            imu_pub_->publish(imu_msg);
            dog_odom_pub->publish(dog_odom);
        }
    }

    void lowStateCallback(const unitree_go::msg::LowState::SharedPtr msg){
        imu = msg->imu_state;   
        getImuData();
    }

    void sportmodeStateCallback(const unitree_go::msg::SportModeState::SharedPtr msg)
    {

        rosgaitstate = *msg;
        getOdom();
        
        gait_type_ = msg->gait_type;
        driving_mode = msg->mode;
        if(lightControl){
            static float params[10];
            if (msg->mode != 7 && !lightOn){
                lightOn = true;
                params[0] = light_level;
                std::array<float, 10> params_array;
                std::copy(std::begin(params), std::end(params), params_array.begin());
                
                handleCommand("vui_client", params_array);
            }else if (msg->mode == 7 && lightOn){
                lightOn = false;
                params[0] = 0;
                std::array<float, 10> params_array;
                std::copy(std::begin(params), std::end(params), params_array.begin());
                handleCommand("vui_client", params_array);
            }
        }



        if (msg->mode == 7)//DUMP
        {
            stand = false;
        }else if (msg->mode == 9 || msg->mode == 6){
            stand = true;
        }

    }

    void dampFunction()
    {
        RCLCPP_INFO(this->get_logger(), "\033[1;33m----->Damp\033[0m");
        sport_req.Damp(req);
        req_puber->publish(req);
        delay_timer_->cancel();
    }
    void resetIgnoreCmd()
    {
        ignore_cmd_flag = false;
        ignore_cmd_timer_->cancel();
    }



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
        /*
        class TerrainType(Enum):

            #posture
            BALANCED = 1
            NORMAL = 3 
            LAYDOWN = 5
            FIXEDSTAND = 6
            DUMP = 7
            ------ for new version
            new ai mode = 9
            old ai mode = 18

        the above enum represents a drivining mode index in HALNA SYSTEM
        */
        if (ignore_cmd_flag) return;

        bool continuousGaitFlag= false;
        if (msg->start && msg->right){
            RCLCPP_INFO(this->get_logger(), "\033[1;33m----->forward climbing mode\033[0m");
            sport_req.SwitchGait(req_, 3);
            continuousGaitFlag = true;
        }else if (msg->start && msg->left){
            RCLCPP_INFO(this->get_logger(), "\033[1;33m----->reverse climbing mode\033[0m");
            sport_req.SwitchGait(req_, 4);
            continuousGaitFlag = true;
        }else if (msg->l2 && msg->a && (driving_mode != 5 && driving_mode !=7 )){
            RCLCPP_INFO(this->get_logger(), "\033[1;33m----->StandDown mode\033[0m");
            sport_req.StandDown(req_);
            ignore_cmd_flag = true;
            ignore_cmd_timer_ = this->create_wall_timer(
                    std::chrono::seconds(2), 
                    std::bind(&GO2DDS::resetIgnoreCmd, this));
        }else if (msg->l2 && msg->a && (driving_mode == 5 || driving_mode ==7 )){
            RCLCPP_INFO(this->get_logger(), "\033[1;33m----->StandUp mode\033[0m");
            sport_req.StandUp(req_);
            ignore_cmd_flag = true;
            ignore_cmd_timer_ = this->create_wall_timer(
                    std::chrono::seconds(2), 
                    std::bind(&GO2DDS::resetIgnoreCmd, this));
        }else if (msg->l2 && msg->b && driving_mode == 7){
            RCLCPP_INFO(this->get_logger(), "\033[1;33m----->Damp mode\033[0m");
            sport_req.Damp(req_);
        } else if (msg->start){
            RCLCPP_INFO(this->get_logger(), "\033[1;33m----->BalanceStand mode\033[0m");
            sport_req.SwitchGait(req_, 1);
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
        go2_cmd_vel_msg = *msg;
        if (abs(msg->linear.z) >0.1){

            if (msg->linear.z < 0){
                RCLCPP_INFO(this->get_logger(), "\033[1;36m----->Stand down\033[0m");
                sport_req.StandDown(req);
                // Set up the timer for 2 seconds delay
                delay_timer_ = this->create_wall_timer(
                    std::chrono::seconds(2), 
                    std::bind(&GO2DDS::dampFunction, this));
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
            go2_cmd_vel_msg.linear.x = msg->linear.x;
            go2_cmd_vel_msg.linear.y = msg->linear.y;
            go2_cmd_vel_msg.angular.z = msg->angular.z;
            cmd_vel_pub_->publish(go2_cmd_vel_msg);
        }
    }
    void makeFakeCovariance(sensor_msgs::msg::Imu& imu_msg_){
        std::call_once(flag, [&imu_msg_]() {
            
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
            std::cout << "\033[1;33mDone setting imu fake covariance\033[0m" << std::endl;
        });
    }
    void getImuData()
    {
        // Create and populate an IMU message
         
        imu_msg.header.stamp = this->now();
        imu_msg.header.frame_id = imuFrame;
    
        // Orientation data (assuming you have this data)
        imu_msg.orientation.w = imu.quaternion[0];
        imu_msg.orientation.x = imu.quaternion[1];
        imu_msg.orientation.y = imu.quaternion[2];
        imu_msg.orientation.z = imu.quaternion[3];

        // Angular velocity data
        imu_msg.angular_velocity.x = imu.gyroscope[0];
        imu_msg.angular_velocity.y = imu.gyroscope[1];
        imu_msg.angular_velocity.z = imu.gyroscope[2];

        // Linear acceleration data
        imu_msg.linear_acceleration.x = imu.accelerometer[0];
        imu_msg.linear_acceleration.y = imu.accelerometer[1];
        imu_msg.linear_acceleration.z = imu.accelerometer[2];

        imu_msg_flag = true;
    }
    void getOdom(){
            // Quadruped robot odometer    
        dog_odom.header.frame_id = odomFrame;
        dog_odom.child_frame_id = robotFrame;
        dog_odom.header.stamp = this->now();

        //position
        dog_odom.pose.pose.position.x = rosgaitstate.position[0];
        dog_odom.pose.pose.position.y = rosgaitstate.position[1];
        dog_odom.pose.pose.position.z = rosgaitstate.position[2];
        //orientation
        dog_odom.pose.pose.orientation.w = rosgaitstate.imu_state.quaternion[0];
        dog_odom.pose.pose.orientation.x = rosgaitstate.imu_state.quaternion[1];
        dog_odom.pose.pose.orientation.y = rosgaitstate.imu_state.quaternion[2];
        dog_odom.pose.pose.orientation.z = rosgaitstate.imu_state.quaternion[3];
        //linear
        dog_odom.twist.twist.linear.x = rosgaitstate.velocity[0];
        dog_odom.twist.twist.linear.y = rosgaitstate.velocity[1];
        dog_odom.twist.twist.linear.z = rosgaitstate.velocity[2];
        //angular
        dog_odom.twist.twist.angular.x = rosgaitstate.imu_state.gyroscope[0];
        dog_odom.twist.twist.angular.y = rosgaitstate.imu_state.gyroscope[1];
        dog_odom.twist.twist.angular.z = rosgaitstate.imu_state.gyroscope[2];

        dog_odom_flag = true;
    }

    void rosgaitcmdCallback(const unitree_interfaces::msg::GaitCmd::SharedPtr msg){
        std::lock_guard<std::mutex> lock(mutex_); 

        static unitree_api::msg::Request req_; // Unitree Go2 ROS2 request message
        if (gait_type_ !=msg->gait_type && msg->gait_type !=0){
            sport_req.SwitchGait(req_, msg->gait_type);
            req_puber->publish(req_);
        }
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
        float x_vel= msg->velocity[0];
        float y_vel = msg->velocity[1];
        float yaw_vel = msg->yaw_speed;
        go2_cmd_vel_msg.linear.x = x_vel;
        go2_cmd_vel_msg.linear.y = y_vel;
        go2_cmd_vel_msg.angular.z = yaw_vel;
        cmd_vel_pub_->publish(go2_cmd_vel_msg);
        sport_req.Move(req_, x_vel, y_vel, yaw_vel);
        req_puber->publish(req_);
        
    }


    void wirelessControllerCallback(const unitree_go::msg::WirelessController::SharedPtr data)
    {
        static bool l2_r2 = false;
        static bool start = true;
        static bool l1_a = false;
        static bool l1_y = false;
        static bool l1_x = false;
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
        }else if (keyValue == L1_X && !l1_x){
            //call key add point
            l1_x = true;
            start = false;
            RCLCPP_INFO(this->get_logger(), "\033[1;32mKey value L1+UP(add a point) has been  pressed.\033[0m");
            ss.data = "L1X";
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
            l1_x = false;
            RCLCPP_INFO(this->get_logger(), "\033[1;32mKey value START has been pressed.\033[0m");
        }else{
            handle_ptz(keyValue);
        }
        RCLCPP_INFO(this->get_logger(), "Wireless controller -- lx: %f; ly: %f; rx: %f; ry: %f; key value: %d",
                    data->lx, data->ly, data->rx, data->ry, data->keys);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<unitree_go::msg::WirelessController>::SharedPtr wireless_sub_;
    rclcpp::Subscription<unitree_interfaces::msg::GaitCmd>::SharedPtr rosgaitcmd_sub_;
    // Create the suber  to receive low state of robot
    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr sportmode_state_sub_;
    rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr low_state_sub_;
    rclcpp::Subscription<techshare_ros_pkg2::msg::ControllerMsg>::SharedPtr remote_controller_sub_;
    rclcpp::Client<techshare_ros_pkg2::srv::ChangeDriveMode>::SharedPtr change_drivemode_srv_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr kill_all_client_;
    rclcpp::Service<techshare_ros_pkg2::srv::SdkClient>::SharedPtr go2_sdk_service_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr dog_odom_pub; // Publishes odom to ROS
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_; // Publishes odom to ROS

    rclcpp::Client<techshare_ros_pkg2::srv::PanTilt>::SharedPtr pan_tilt_cli_;
    rclcpp::Client<techshare_ros_pkg2::srv::Zoom>::SharedPtr zoom_cli_;
    nav_msgs::msg::Odometry dog_odom;  // odom data
    geometry_msgs::msg::Twist go2_cmd_vel_msg;  // odom data
    geometry_msgs::msg::Twist zero_twist;  // odom data

    sensor_msgs::msg::Imu imu_msg;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr key_value_pub_;

    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_puber;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    unitree_go::msg::IMUState imu;         // Unitree go2 IMU message
    float battery_voltage;                 // Battery voltage
    float battery_current;                 // Battery current
    bool fixed_stand;
    bool ignore_cmd_flag = false;
    bool remotelyControlled, stand;
    bool imu_msg_flag, dog_odom_flag, drivemode_srv_client_flag;
    int driving_mode = 0;
    int prev_driving_mode = 0;
    rclcpp::Time last_message_time_;
    rclcpp::TimerBase::SharedPtr timer_, delay_timer_ ,rawDataTimer, ignore_cmd_timer_; // ROS2 timer
    unitree_api::msg::Request req; // Unitree Go2 ROS2 request message
    unitree_go::msg::SportModeState rosgaitstate;
    std::shared_ptr<techshare_ros_pkg2::srv::PanTilt::Request> pan_tilt_req;
    std::shared_ptr<techshare_ros_pkg2::srv::Zoom::Request> zoom_req;
    SportClient sport_req;
    std::mutex mutex_;
    //link names
    std::string robotOdometry;
    std::string imuFrame;
    std::string robotMovement;
    std::string odomFrame;
    std::string robotFrame;
    std::string imuTopic;
    int gait_type_ =0;
    double t; // runing time count
    double dt = 0.002; //control time step

    double px0 = 0;  // initial x position
    double py0 = 0;  // initial y position
    double yaw0 = 0; // initial yaw angle
    std::once_flag flag;
    int zoom_level;
    //for ipc
    int light_level =0;
    bool lightOn = false;
    bool lightControl = false;
    int shm_fd_;
    SDK_CLIENT_DATA* shared_memory_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv); // Initialize rclcpp
    rclcpp::spin(std::make_shared<GO2DDS>()); //Run ROS2 node
    rclcpp::shutdown();
    return 0;
}


