// go2w_dds.cpp --------------------------------------------------------------
#include "dog_dds_common.hpp"
#include <sys/mman.h>
#include <fcntl.h>
#include <semaphore.h>
#include <thread>
#include <chrono> 
using namespace std::chrono_literals; 
enum Mode{
    NORMAL_MODE = 1,
    GENERAL_STAIR_MODE = 2,
    WALL_CLIMB_MODE = 3,
    HIGH_SPEED_MODE = 5,
    AI_MODE = 9,
};

class GO2WDDS : public DogDDSBase<DogModel::GO2W>
{
public:
    GO2WDDS() : DogDDSBase<DogModel::GO2W>()
    {
        initIpcBlocks();                 // the two extra shared-memory areas
        temporal_sportmodestate_pub = this->create_publisher<unitree_go::msg::SportModeState>("/sportmodestate", 10);

        ipc_timer_ = create_wall_timer(                      // 300 ms poll
                      300ms, std::bind(&GO2WDDS::pollIpc,this));
    }
    ~GO2WDDS() override
    {
        munmap(cmd_shm_, sizeof(CmdVelData));
        munmap(state_shm_,sizeof(SDK_CLIENT_DATA));
        close(cmd_fd_);  close(state_fd_);
    }

private:
    /* ======== overrides of the three hooks we now have ============ */

    // 1)  game-pad
    void onRemoteController(const CtrlMsg &msg) override
    {
        std::scoped_lock lk(mutex_);
        if (ignore_cmd_) return;

        bool continuous_gait = false;
        bool do_publish      = true;
        unitree_api::msg::Request req;

        // if (msg.start && msg.right)
        // {
        //     RCLCPP_INFO(get_logger(), "→ forward climbing (ignored for B2)");
        //     continuous_gait = true;         // kept for symmetry, but not sent
        //     do_publish      = false;
        // }
        // else if (msg.start && msg.left)
        // {
        //     RCLCPP_INFO(get_logger(), "→ reverse climbing (ignored for B2)");
        //     continuous_gait = true;
        //     do_publish      = false;
        // }
        if (msg.l2 && msg.a && (driving_mode_!=5 && driving_mode_!=7))
        {
            RCLCPP_INFO(get_logger(), "→ StandDown");
            sport_client_.StandDown(req);
            startIgnoreTimer();
        }
        else if (msg.l2 && msg.a && (driving_mode_==5 || driving_mode_==7))
        {
            RCLCPP_INFO(get_logger(), "→ StandUp");
            sport_client_.StandUp(req);
            startIgnoreTimer();
        }
        else if (msg.l2 && msg.b && driving_mode_==7)
        {
            RCLCPP_INFO(get_logger(), "→ Damp");
            sport_client_.Damp(req);
        }
        else if (msg.start)
        {
            RCLCPP_INFO(get_logger(), "→ BalanceStand");
            sport_client_.BalanceStand(req);
        }
        else
        {
            do_publish = false;    // no action detected
        }

        if(do_publish)
        {
            req_pub_->publish(req);
            if(continuous_gait)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                sport_client_.ContinuousGait(req, continuous_gait);
            }
        }
    }

    // 2)  velocity from upper layer
    void onCmdVel(const Twist &msg) override
    {
        std::scoped_lock lk(mutex_);
        static unitree_api::msg::Request req_; // Unitree Go2 ROS2 request message

        cmd_vel_msg_ = msg;
        if (abs(msg.linear.z) >0.1){
            if (msg.linear.z < 0){
                RCLCPP_INFO(this->get_logger(), "\033[1;36m----->Stand down\033[0m");
                changeMode(req_, NORMAL_MODE);
                req_pub_->publish(req_);
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                sport_client_.StandUp(req_);
                req_pub_->publish(req_);
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                sport_client_.StandDown(req_);
                // Set up the timer for 2 seconds delay
                delay_timer_ = this->create_wall_timer(std::chrono::seconds(2),
                    [this]() { unitree_api::msg::Request r; sport_client_.Damp(r); req_pub_->publish(r); delay_timer_->cancel(); });
            }else {
                RCLCPP_INFO(this->get_logger(), "\033[1;34m----->Stand up\033[0m");
                sport_client_.StandUp(req_);
            }
            req_pub_->publish(req_);
        }else{
            if (checkMode()) return;
            RCLCPP_INFO(this->get_logger(), "\033[1;32m----->Moving\033[0m");
            cmd_vel_pub_->publish(cmd_vel_msg_);
            publishMove();       // new: wake up PC side
        }
    }


    void changeMode(unitree_api::msg::Request& req_local, int mode){

            /*

            //stair mode
            ---
            header:
            identity:
                id: 1829516185
                api_id: 1011
            lease:
                id: 0
            policy:
                priority: 0
                noreply: false
            parameter: '{"data":1}'
            binary: []
            ---

            //wall climb mode
            ---
            header:
            identity:
                id: 1829555010
                api_id: 1011
            lease:
                id: 0
            policy:
                priority: 0
                noreply: false
            parameter: '{"data":0}'
            binary: []
            ---

            //high speed
            header:
            identity:
                id: 1829599783
                api_id: 1015
            lease:
                id: 0
            policy:
                priority: 0
                noreply: false
            parameter: '{"data":1}'
            binary: []
            ---

            */

        //special meesag to change new ai mode to old one
        req_local.header.identity.id = std::rand() % 100001;;  //maybe it is okay to be random int?

        // Set lease ID
        req_local.header.lease.id = 0;
        // Set policy values
        req_local.header.policy.priority = 0;
        req_local.header.policy.noreply = false;
        // Set parameter as a JSON string

        // Set binary as an empty array
        req_local.binary.clear();



        if (mode == AI_MODE){ //classic ai for go2
            req_local.header.identity.api_id = 1049;  //this is the switcher 
            req_local.parameter = "{\"data\":true}";  //if data is false then it becomes new ai mode
        }else if (mode == GENERAL_STAIR_MODE){
            req_local.header.identity.api_id = 1011;  //this is the switcher 
            req_local.parameter = "{\"data\":1}";  //if data is false then it becomes new ai mode
        }else if (mode == WALL_CLIMB_MODE){
            req_local.header.identity.api_id = 1011;  //this is the switcher 
            req_local.parameter = "{\"data\":2}";  //if data is false then it becomes new ai mode
        }else if (mode == HIGH_SPEED_MODE){
            req_local.header.identity.api_id = 1015;  //this is the switcher 
            req_local.parameter = "{\"data\":1}";  //if data is false then it becomes new ai mode
        }else if (mode == NORMAL_MODE){
            req_local.header.identity.api_id = 1011;  //this is the switcher 
            req_local.parameter = "{\"data\":0}";  //if data is false then it becomes new ai mode
        }

    }

    bool checkMode(){
        static unitree_api::msg::Request req_; // Unitree Go2 ROS2 request message
        if (driving_mode_ == 6){
            // sport_req.BalanceStand(req);
            changeMode(req_, NORMAL_MODE);
            req_pub_->publish(req_);
            RCLCPP_INFO(this->get_logger(), "\033[1;34m----->Changing to the balance\033[0m");
            return true;
        } else if (driving_mode_ == 9){
            changeMode(req_, AI_MODE);
            req_pub_->publish(req_);
            return true;
        }
        return false;
    }
    // 3)  gait-command from web
    void onGaitCmd(const GaitCmdMsg &msg) override
    {
        std::scoped_lock lk(mutex_);

        if (checkMode()) return;
        static unitree_api::msg::Request req_; // Unitree Go2 ROS2 request message
        if ((gait_type_+1) != msg.gait_type && gait_type_ < 4){
            changeMode(req_, msg.gait_type);
            RCLCPP_INFO(this->get_logger(), "\033[1;34m----->Changing the gait type to %d\033[0m", msg.gait_type);
            req_pub_->publish(req_);
            return;
        }
        if (std::abs(msg.velocity[0]) < 1e-9 &&
            std::abs(msg.velocity[1]) < 1e-9 &&
            std::abs(msg.yaw_speed) < 1e-9)
        {
            return;
        }
        // if (checkMode()) return;
        float x_vel= msg.velocity[0];
        float y_vel = msg.velocity[1];
        float yaw_vel = msg.yaw_speed;
        cmd_vel_msg_.linear.x = x_vel;
        cmd_vel_msg_.linear.y = y_vel;
        cmd_vel_msg_.angular.z = yaw_vel;
        cmd_vel_pub_->publish(cmd_vel_msg_);
        publishMove();
    }

    void publishMove(){
        // Write cmd_vel data to shared memory
        cmd_shm_->linear_x = cmd_vel_msg_.linear.x;
        cmd_shm_->linear_y = cmd_vel_msg_.linear.y;
        cmd_shm_->angular_z = cmd_vel_msg_.angular.z;
        // Post the semaphore to signal the other process
        sem_post(&cmd_shm_->sem);
    }

    // 4) optional 500 Hz tick – pack raw IMU to IPC if you want
    void onTick500Hz() override
    {
        /* high-rate work, if any (leave empty otherwise) */
    }

    /* =======  shared-memory helpers (private)  ==================== */

    struct CmdVelData { double linear_x,linear_y,angular_z; sem_t sem; };

    void initIpcBlocks()
    {
        cmd_fd_   = openOrCreate("/cmd_vel_shm",   sizeof(CmdVelData));
        state_fd_ = openOrCreate("/high_state_shm",sizeof(SDK_CLIENT_DATA));

        cmd_shm_   = static_cast<CmdVelData *>
                        (mmap(nullptr,sizeof(CmdVelData),PROT_WRITE,MAP_SHARED,cmd_fd_,0));
        state_shm_ = static_cast<SDK_CLIENT_DATA *>
                        (mmap(nullptr,sizeof(SDK_CLIENT_DATA),PROT_WRITE,MAP_SHARED,state_fd_,0));

        sem_init(&cmd_shm_->sem,   1, 0);
        sem_init(&state_shm_->semaphore, 1, 0);
    }

    int openOrCreate(const char *name,size_t sz)
    {
        int fd = shm_open(name,O_RDWR,0666);
        if(fd==-1){ fd = shm_open(name,O_CREAT|O_RDWR,0666); ftruncate(fd,sz); }
        return fd;
    }

    void postCmdVelIpc()
    {
        cmd_shm_->linear_x = last_twist_.linear.x;
        cmd_shm_->linear_y = last_twist_.linear.y;
        cmd_shm_->angular_z= last_twist_.angular.z;
        sem_post(&cmd_shm_->sem);
    }

    void pollIpc()                        // called every 0.3 s
    {
        if(sem_trywait(&state_shm_->semaphore)==0)          // new state
        {
            driving_mode_ = static_cast<int>(state_shm_->params[0]);
            gait_type_    = static_cast<int>(state_shm_->params[1]);
            float progress = state_shm_->params[2];
            if (!odom_ready_) odom_ready_=true; 
            unitree_go::msg::SportModeState temp_msg;
            temp_msg.gait_type = gait_type_;
            temp_msg.mode = driving_mode_;
            temp_msg.progress = static_cast<int>(state_shm_->params[2]);
            temporal_sportmodestate_pub->publish(temp_msg);
            
            sem_post(&state_shm_->semaphore);  // release
        }
    }

    /* 2‑second ignore window after posture change ----------------------- */
    void startIgnoreTimer()
    {
        ignore_cmd_ = true;
        ignore_timer_ = this->create_wall_timer(std::chrono::seconds(2),
                         [this]() { ignore_cmd_ = false; ignore_timer_->cancel(); });
    }

    /* ============= members ============== */
    CmdVelData           *cmd_shm_   {nullptr};
    SDK_CLIENT_DATA      *state_shm_ {nullptr};
    int                   cmd_fd_  {-1},  state_fd_{-1};
    rclcpp::TimerBase::SharedPtr ipc_timer_;
    Twist                 last_twist_;
    int gait_type_ =0;
    bool                            ignore_cmd_   = false;
    rclcpp::TimerBase::SharedPtr    ignore_timer_;
    rclcpp::TimerBase::SharedPtr    delay_timer_;
    rclcpp::Publisher<unitree_go::msg::SportModeState>::SharedPtr temporal_sportmodestate_pub;
    std::mutex                      mutex_;
};

/* ---- main ---- */
int main(int argc,char **argv)
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<GO2WDDS>());
    rclcpp::shutdown();
}
