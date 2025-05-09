// dog_dds_common.hpp
// -----------------------------------------------------------------------------
//  ONE‑STOP HEADER FOR ALL SHARED LOGIC BETWEEN "B2DDS" AND "GO2DDS" NODES.
//  Only **two** things are left for the concrete nodes:
//      • override `onRemoteController()`  (differences are just a few if‑else)
//      • override `onCmdVel()`            (slightly different StandUp/Down logic)
//  Everything else – IMU/odom relay, PTZ handling, shared‑memory IPC, key
//  mapping, timers, sport‑mode reactions – lives exactly ONCE in this file.
// -----------------------------------------------------------------------------
#pragma once

/* ==============================  STANDARD ================================= */
#include <array>
#include <chrono>
#include <cstring>
#include <mutex>
#include <string_view>
#include <thread>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <semaphore.h>  // sem_t, sem_init, sem_post
#include <unistd.h>     // close
#include <ctime>
#include <iomanip>
#include <errno.h>
#include <cmath>
/* ================================  ROS 2  ================================= */
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int8.hpp"

/* =======================  UNITREE + TECHSHARE MSGS ======================== */
#include "unitree_go/msg/sport_mode_state.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "unitree_go/msg/wireless_controller.hpp"
#include "unitree_api/msg/request.hpp"
#include "techshare_ros_pkg2/msg/controller_msg.hpp"
#include "techshare_ros_pkg2/srv/pan_tilt.hpp"
#include "techshare_ros_pkg2/srv/zoom.hpp"
#include "techshare_ros_pkg2/srv/change_drive_mode.hpp"
#include "techshare_ros_pkg2/srv/sdk_client.hpp"
#include "unitree_interfaces/msg/gait_cmd.hpp"   
#include "std_srvs/srv/trigger.hpp"

#include "common/ros2_sport_client.h"   // Unitree helper

/* ============================  TRAITS TABLE  ============================== */
enum class DogModel { B2, GO2, GO2W };

template<DogModel> struct DogTraits;             // primary ‑‑ undefined

template<> struct DogTraits<DogModel::B2>
{
    static constexpr std::string_view node_name      = "b2_dds";
    static constexpr std::string_view cmd_vel_topic  = "b2_cmd_vel";
    static constexpr bool publish_rawdata           = false; // no 500 Hz relay
};

template<> struct DogTraits<DogModel::GO2>
{
    static constexpr std::string_view node_name      = "go2_dds";
    static constexpr std::string_view cmd_vel_topic  = "go2_cmd_vel";
    static constexpr bool publish_rawdata           = true;  // IMU+odom relay
};

template<> struct DogTraits<DogModel::GO2W>
{
    static constexpr std::string_view node_name      = "go2w_dds";
    static constexpr std::string_view cmd_vel_topic  = "go2w_cmd_vel";
    static constexpr bool publish_rawdata           = true;  // IMU+odom relay
};

/* ============================  KEY CODES  ================================= */
enum KeyValue : uint16_t
{
    START  = 4,
    L1_START = 6,
    L1_A   = 258,
    L1_X   = 1026,
    L2_R2  = 48,
    L1_Y   = 2050,
    L1_UP  = 4098,
    L1_B   = 514,
    UP     = 4096,
    RIGHT  = 8192,
    LEFT   = 32768,
    DOWN   = 16384,
    R1R2   = 17,
    L1L2   = 34
};
enum class KEY_ACTION : uint16_t
{
    SAVE_MAP = 23
};

/* =====================  SHARED‑MEM STRUCT FOR IPC  ======================== */
struct SDK_CLIENT_DATA
{
    char  client_name[256]{};
    float params[10]{};
    sem_t semaphore{};
};
struct CmdVelData {
    double linear_x;
    double linear_y;
    double angular_z;
    sem_t semaphore;
};
/* ============================  BASE CLASS  ================================ */

template<DogModel Model>
class DogDDSBase : public rclcpp::Node
{
    protected:
        using Traits      = DogTraits<Model>;
        using Twist       = geometry_msgs::msg::Twist;
        using ImuMsg      = sensor_msgs::msg::Imu;
        using OdomMsg     = nav_msgs::msg::Odometry;
        using CtrlMsg     = techshare_ros_pkg2::msg::ControllerMsg;
        using LowStateMsg = unitree_go::msg::LowState;
        using SportState  = unitree_go::msg::SportModeState;
        using GaitCmdMsg  = unitree_interfaces::msg::GaitCmd;

        /* === services === */
        using PanTiltSrv  = techshare_ros_pkg2::srv::PanTilt;
        using ZoomSrv     = techshare_ros_pkg2::srv::Zoom;
        using TriggerSrv  = std_srvs::srv::Trigger;
        using SDKSrv = techshare_ros_pkg2::srv::SdkClient;

public:
    DogDDSBase() : rclcpp::Node(std::string(Traits::node_name))
    {
        declareParams();
        readParams();
        initSharedMemory();
        createPubsSubs();
        createServices();
        createClients();
        setupTimers();
        buildStaticRequests();
        RCLCPP_INFO(get_logger(), "🚀 %s initialised", Traits::node_name.data());
    }

    ~DogDDSBase() override
    {
        munmap(shared_mem_, sizeof(SDK_CLIENT_DATA));
        close(shm_fd_);
        RCLCPP_INFO(get_logger(), "Shared memory cleaned up");
    }

protected:
    /* ----- pure virtual hooks the concrete node must override ------------- */
    virtual void onRemoteController(const CtrlMsg&) = 0;
    virtual void onSportState(const SportState&) = 0;
    virtual void onCmdVel(const Twist&)             = 0;
    virtual void onGaitCmd(const GaitCmdMsg &)      = 0;
    virtual void onTick500Hz()             {}             // optional high-rate work

    /* ----- helpers for derived classes ------------------------------------ */

    void postIPC(const std::string &name, const std::array<float,10>& p)
    {
        std::strncpy(shared_mem_->client_name, name.c_str(), sizeof(shared_mem_->client_name)-1);
        std::copy(p.begin(), p.end(), shared_mem_->params);
        sem_post(&shared_mem_->semaphore);
    }

    void ptzHandle(int key)
    {
        switch(static_cast<KeyValue>(key))
        {
            case RIGHT: ptz("right");           break;
            case LEFT:  ptz("left");            break;
            case UP:    ptz("up");              break;
            case DOWN:  ptz("down");            break;
            case R1R2:  zoom(+1);                 break;
            case L1L2:  zoom(-1);                 break;
            default: /* ignore */ break;
        }
    }

    /* ----- night‑time helper --------------------------------------------- */
    static bool isNightTimeJST()
    {
        using clk = std::chrono::system_clock;
        auto tt   = clk::to_time_t(clk::now());
        std::tm tm{ *std::gmtime(&tt) };
        tm.tm_hour = (tm.tm_hour + 9) % 24;                       // UTC→JST
        return (tm.tm_hour >= 15 || tm.tm_hour < 6);
    }


    void fillOdomMsg()
    {
        odom_msg_.header.stamp       = now();
        odom_msg_.header.frame_id    = odom_frame_;
        odom_msg_.child_frame_id     = base_frame_;
        odom_msg_.pose.pose.position.x = sport_state_.position[0];
        odom_msg_.pose.pose.position.y = sport_state_.position[1];
        odom_msg_.pose.pose.position.z = sport_state_.position[2];
        odom_msg_.pose.pose.orientation.w = sport_state_.imu_state.quaternion[0];
        odom_msg_.pose.pose.orientation.x = sport_state_.imu_state.quaternion[1];
        odom_msg_.pose.pose.orientation.y = sport_state_.imu_state.quaternion[2];
        odom_msg_.pose.pose.orientation.z = sport_state_.imu_state.quaternion[3];
        odom_msg_.twist.twist.linear.x    = sport_state_.velocity[0];
        odom_msg_.twist.twist.linear.y    = sport_state_.velocity[1];
        odom_msg_.twist.twist.linear.z    = sport_state_.velocity[2];
        odom_msg_.twist.twist.angular.x   = sport_state_.imu_state.gyroscope[0];
        odom_msg_.twist.twist.angular.y   = sport_state_.imu_state.gyroscope[1];
        odom_msg_.twist.twist.angular.z   = sport_state_.imu_state.gyroscope[2];
        odom_ready_ = true;
    }

    /* ----- exposed publishers (some nodes push custom twists) ------------- */
    rclcpp::Publisher<Twist>::SharedPtr      cmd_vel_pub_;
    rclcpp::Publisher<OdomMsg>::SharedPtr    odom_pub_;
    rclcpp::Publisher<ImuMsg>::SharedPtr     imu_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr key_pub_;
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_pub_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr navigation_state_pub_;


    /* ---- in DogDDSBase (protected section) ---------------------------- */
    rclcpp::Subscription<Twist>::SharedPtr              cmd_vel_sub_;
    rclcpp::Subscription<CtrlMsg>::SharedPtr            controller_sub_;
    rclcpp::Subscription<unitree_go::msg::WirelessController>::SharedPtr wl_ctrl_sub_;
    rclcpp::Subscription<LowStateMsg>::SharedPtr        low_state_sub_;
    rclcpp::Subscription<SportState>::SharedPtr         sport_state_sub_;
    rclcpp::Subscription<GaitCmdMsg>::SharedPtr         gait_sub_;

    /* ----- sport client & reusable requests ------------------------------- */
    SportClient               sport_client_;
    unitree_api::msg::Request old_gait_req_{};   // switcher (common)
    unitree_api::msg::Request new_gait_req_{};   // switcher (common)

    /* ----- state used by shared callbacks --------------------------------- */
    ImuMsg            imu_msg_{};
    OdomMsg           odom_msg_{};
    unitree_go::msg::IMUState  imu_state_{};   // raw from LowState
    geometry_msgs::msg::Twist cmd_vel_msg_;  // odom data
    SportState        sport_state_{};

    bool  imu_ready_   = false;
    bool  odom_ready_  = false;
    int   driving_mode_ = 0;

private:
    /* =====================  MEMBER DATA =================================== */
    int                 shm_fd_      = -1;
    SDK_CLIENT_DATA    *shared_mem_  = nullptr;
    int                 zoom_level_  = 1;

    // parameters
    std::string imu_frame_   {"imu_link"};
    std::string odom_frame_  {"odom"};
    std::string base_frame_  {"base_link"};

    // timers
    rclcpp::TimerBase::SharedPtr relay_timer_;

    // clients
    rclcpp::Client<PanTiltSrv>::SharedPtr pan_tilt_cli_;
    rclcpp::Client<ZoomSrv>::SharedPtr    zoom_cli_;
    rclcpp::Client<TriggerSrv>::SharedPtr killall_cli_;
    rclcpp::Client<TriggerSrv>::SharedPtr save_map_cli_;

    // services
    rclcpp::Service<techshare_ros_pkg2::srv::SdkClient>::SharedPtr sdk_service_;
    /* =====================  PARAM + INIT HELPERS ========================= */
    void declareParams()
    {
        this->declare_parameter("imuFrame", imu_frame_);
        this->declare_parameter("odomFrame", odom_frame_);
        this->declare_parameter("robotFrame", base_frame_);
    }

    void readParams()
    {
        this->get_parameter("imuFrame",  imu_frame_);
        this->get_parameter("odomFrame", odom_frame_);
        this->get_parameter("robotFrame", base_frame_);
    }

    void initSharedMemory()
    {
        shm_fd_ = shm_open("/sdk_client_shm", O_RDWR, 0666);
        if (shm_fd_ == -1)
        {
            RCLCPP_INFO(get_logger(), "Creating shared memory: /sdk_client_shm");
            shm_fd_ = shm_open("/sdk_client_shm", O_CREAT|O_RDWR, 0666);
            ftruncate(shm_fd_, sizeof(SDK_CLIENT_DATA));
        }
        shared_mem_ = static_cast<SDK_CLIENT_DATA*>(
            mmap(nullptr, sizeof(SDK_CLIENT_DATA), PROT_WRITE, MAP_SHARED, shm_fd_, 0));
        if (shared_mem_ == MAP_FAILED)
            throw std::runtime_error("mmap() failed for shared memory");
        sem_init(&shared_mem_->semaphore, 1, 0);
    }

    /* =======================  PUB/SUB + CLIENTS =========================== */
    void createPubsSubs()
    {
        using namespace std::placeholders;

        cmd_vel_pub_ = this->create_publisher<Twist>(std::string(Traits::cmd_vel_topic), 10);
        imu_pub_     = this->create_publisher<ImuMsg>("imu/data", 10);
        odom_pub_    = this->create_publisher<OdomMsg>("dog_odom", 1);
        key_pub_     = this->create_publisher<std_msgs::msg::String>("remote_toweb_cmd",10);
        req_pub_     = this->create_publisher<unitree_api::msg::Request>("/api/sport/request",10);
        navigation_state_pub_ = this->create_publisher<std_msgs::msg::Int8>("navigation_state", 1);

        /* ––––– subscriptions ––––– */
        cmd_vel_sub_ = this->create_subscription<Twist>("msg_cmd_vel", 1,
            [this](Twist::SharedPtr m){ onCmdVel(*m); });

        controller_sub_ = this->create_subscription<CtrlMsg>("controller_status", 1,
            [this](CtrlMsg::SharedPtr m){ onRemoteController(*m); });

        wl_ctrl_sub_ = this->create_subscription<unitree_go::msg::WirelessController>(
            "/wirelesscontroller", 10,
            [this](unitree_go::msg::WirelessController::SharedPtr m)
            {
                this->handleWireless(*m);
            });

        low_state_sub_ =  this->create_subscription<LowStateMsg>("/lowstate", 1,
            [this](LowStateMsg::SharedPtr m){ this->lowStateCB(*m); });

        sport_state_sub_ =  this->create_subscription<SportState>("/sportmodestate", 1,
            [this](SportState::SharedPtr m){ onSportState(*m); });

        gait_sub_ = this->create_subscription<GaitCmdMsg>("rosgaitcmd", 10,
            [this](GaitCmdMsg::SharedPtr m){ this->onGaitCmd(*m); });
    }

    void createClients()
    {
        pan_tilt_cli_ = this->create_client<PanTiltSrv>("pan_tilt");
        zoom_cli_     = this->create_client<ZoomSrv>("zoom");
        killall_cli_  = this->create_client<TriggerSrv>("killall");
        save_map_cli_  = this->create_client<TriggerSrv>("lio/save_map");
    }
    
    void createServices()
    {
        sdk_service_ = this->create_service<SDKSrv>(
            "unitree_sdk_client",
            /* lambda keeps code local to the base class */
            [this](const std::shared_ptr<SDKSrv::Request>  req,
                   std::shared_ptr<SDKSrv::Response>       res)
            {
                postIPC(req->client_name, req->params); // write to shared-mem
                res->res = 1;                           // ack
            });

    }

    void setupTimers()
    {
        if constexpr (Traits::publish_rawdata)
        {
            relay_timer_ = this->create_wall_timer(std::chrono::milliseconds(2),
                [this]()
                {
                    onTick500Hz();                                // << new
                    if (imu_ready_ && odom_ready_)
                    {
                        imu_pub_->publish(imu_msg_);
                        odom_pub_->publish(odom_msg_);
                    }
                });
        }
    }


    void buildStaticRequests()
    {
        old_gait_req_.header.identity.id      = 271801251;  // arbitrary
        old_gait_req_.header.identity.api_id  = 1049;       // switcher
        old_gait_req_.header.lease.id         = 0;
        old_gait_req_.header.policy.priority  = 0;
        old_gait_req_.header.policy.noreply   = false;
        old_gait_req_.parameter               = "{\"data\":true}"; // true → old‑AI

        new_gait_req_.header.identity.id      = 271801251;  // arbitrary
        new_gait_req_.header.identity.api_id  = 1049;       // switcher
        new_gait_req_.header.lease.id         = 0;
        new_gait_req_.header.policy.priority  = 0;
        new_gait_req_.header.policy.noreply   = false;
        new_gait_req_.parameter               = "{\"data\":false}"; // true → old‑AI



    }

    /* =======================  INTERNAL CALLBACKS ========================== */
    void lowStateCB(const LowStateMsg &msg)
    {
        imu_state_ = msg.imu_state;
        fillImuMsg();
    }

    void fillImuMsg()
    {
        imu_msg_.header.stamp    = now();
        imu_msg_.header.frame_id = imu_frame_;
        imu_msg_.orientation.w   = imu_state_.quaternion[0];
        imu_msg_.orientation.x   = imu_state_.quaternion[1];
        imu_msg_.orientation.y   = imu_state_.quaternion[2];
        imu_msg_.orientation.z   = imu_state_.quaternion[3];
        imu_msg_.angular_velocity.x = imu_state_.gyroscope[0];
        imu_msg_.angular_velocity.y = imu_state_.gyroscope[1];
        imu_msg_.angular_velocity.z = imu_state_.gyroscope[2];
        imu_msg_.linear_acceleration.x = imu_state_.accelerometer[0];
        imu_msg_.linear_acceleration.y = imu_state_.accelerometer[1];
        imu_msg_.linear_acceleration.z = imu_state_.accelerometer[2];
        imu_ready_ = true;
    }




    /* ============  WIRELESS CONTROLLER KEY‑MAPPING (shared)  ============== */
    void handleWireless(const unitree_go::msg::WirelessController &wc)
    {
        static bool l2r2=false,l1a=false,l1x=false,l1y=false,l1up=false,l1b=false,l1start=false;
        std_msgs::msg::String ss;
        switch(static_cast<KeyValue>(wc.keys))
        {
            case L2_R2:
                if(!l2r2){ l2r2=true; killall_cli_->async_send_request(std::make_shared<TriggerSrv::Request>()); }
                break;
            case L1_A:
                if(!l1a){ l1a=true; ss.data="L1A"; key_pub_->publish(ss);} break;

            case L1_X:
                if(!l1x){ l1x=true; ss.data="L1X"; key_pub_->publish(ss);} break;
            case L1_Y:
                if(!l1y){ l1y=true; ss.data="L1Y"; key_pub_->publish(ss);} break;
            case L1_UP:
                if(!l1up){ l1up=true; ss.data="L1UP";key_pub_->publish(ss);} break;
            case L1_B:
                if(!l1b){ l1b=true; ss.data="L1B"; key_pub_->publish(ss);} break;
            case L1_START:
                if(!l1start){ l1start=true; 
                    save_map_cli_->async_send_request(std::make_shared<TriggerSrv::Request>()); 
                    std_msgs::msg::Int8 navigation_state_msg;
                    navigation_state_msg.data = static_cast<int>(KEY_ACTION::SAVE_MAP);
                    navigation_state_pub_->publish(navigation_state_msg);
                }
                break;
            case START:
                l2r2=l1a=l1x=l1y=l1up=l1b=false; break;
            default:
                ptzHandle(wc.keys); break;
        }
    }

    /* ========================  INTERNAL HELPERS =========================== */
    void ptz(const std::string &dir)
    {
        auto req = std::make_shared<PanTiltSrv::Request>();
        req->direction = dir;
        req->speed     = 100;
        req->degrees   = 5;
        pan_tilt_cli_->async_send_request(req);
    }

    void zoom(int delta)
    {
        zoom_level_ = std::clamp(zoom_level_ + delta, 1, 30);
        auto req = std::make_shared<ZoomSrv::Request>();
        req->target = zoom_level_;
        zoom_cli_->async_send_request(req);
    }
};

/* ===========================================================================
   End of dog_dds_common.hpp                                                  
   ======================================================================= */
