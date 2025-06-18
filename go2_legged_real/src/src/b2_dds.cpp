// b2_dds.cpp
// -----------------------------------------------------------------------------
//  Concrete implementation of the B2 node, re‑using all shared logic from
//  `dog_dds_common.hpp` and providing only the B2‑specific behaviour for
//  remote‑controller keys and cmd_vel handling.
// -----------------------------------------------------------------------------
#include "dog_dds_common.hpp"
#include <thread>
#include <fcntl.h>      // O_RDWR, O_CREAT
#include <sys/mman.h>   // shm_open, mmap, munmap, PROT_WRITE, MAP_SHARED
#include <sys/stat.h>   // mode constants
#include <unistd.h>     // close
#include <fcntl.h>      // O_RDWR, O_CREAT
#include <sys/mman.h>   // shm_open, mmap, munmap, PROT_WRITE, MAP_SHARED
#include <sys/stat.h>   // mode constants
#include <semaphore.h>  // sem_t, sem_init, sem_post
#include <unistd.h>     // close

class B2DDS : public DogDDSBase<DogModel::B2>
{
public:
    B2DDS() : DogDDSBase<DogModel::B2>() {}

protected:
    /* ------------------------------------------------------------------ */
    /*  Controller (game‑pad) callback – B2 flavour                       */
    /* ------------------------------------------------------------------ */

    void onSportState(const SportState &msg)
    {
        sport_state_  = msg;
        driving_mode_ = msg.mode;
        // publish old‑API switch if needed
        // if(driving_mode_ == 9) req_pub_->publish(old_gait_req_);
        fillOdomMsg();
    }

    void onRemoteController(const CtrlMsg &msg) override
    {
        std::scoped_lock lk(mutex_);
        if (ignore_cmd_) return;

        bool continuous_gait = false;
        bool do_publish      = true;
        unitree_api::msg::Request req;

        if (msg.start && msg.right)
        {
            RCLCPP_INFO(get_logger(), "→ forward climbing (ignored for B2)");
            continuous_gait = true;         // kept for symmetry, but not sent
            do_publish      = false;
        }
        else if (msg.start && msg.left)
        {
            RCLCPP_INFO(get_logger(), "→ reverse climbing (ignored for B2)");
            continuous_gait = true;
            do_publish      = false;
        }
        else if (msg.l2 && msg.a && (driving_mode_!=5 && driving_mode_!=7))
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

    /* ------------------------------------------------------------------ */
    /*  Cmd‑vel from higher layer                                         */
    /* ------------------------------------------------------------------ */
    void onCmdVel(const Twist &msg) override
    {
        std::scoped_lock lk(mutex_);
        unitree_api::msg::Request req;

        if (std::abs(msg.linear.z) > 0.1)
        {
            if (msg.linear.z < 0)
            {
                RCLCPP_INFO(get_logger(), "StandDown + Damp");
                sport_client_.StandDown(req);
                req_pub_->publish(req);
                // damp after 2 s
                delay_timer_ = this->create_wall_timer(std::chrono::seconds(2),
                    [this]() { unitree_api::msg::Request r; sport_client_.Damp(r); req_pub_->publish(r); delay_timer_->cancel(); });
                return;
            }
            else
            {
                RCLCPP_INFO(get_logger(), "StandUp");
                sport_client_.StandUp(req);
                req_pub_->publish(req);
                return;
            }
        }

        // regular move
        if (driving_mode_ == 6)
        {
            sport_client_.BalanceStand(req);   // ensure balanced first
            req_pub_->publish(req);
        }
        // else if (driving_mode_ == 9)           // switch to old‑AI if needed
        // {
        //     req_pub_->publish(old_gait_req_);
        //     return;
        // }

        RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 1000, "Moving … x:%f y:%f z:%f",
                              msg.linear.x, msg.linear.y, msg.angular.z);
        sport_client_.Move(req, msg.linear.x, msg.linear.y, msg.angular.z);
        req_pub_->publish(req);
        cmd_vel_pub_->publish(msg);            // echo out on b2_cmd_vel
    }


    void onGaitCmd(const GaitCmdMsg &msg) override
    {
        std::scoped_lock lk(mutex_);

        static unitree_api::msg::Request req_; // Unitree Go2 ROS2 request message
        if (driving_mode_ == 6){
            sport_client_.BalanceStand(req_);
            req_pub_->publish(req_);
        }


        if (msg.gait_type == 2  && driving_mode_ != 9)
        {
            req_pub_->publish(new_gait_req_);
            return;
        }else if (msg.gait_type != 2 && driving_mode_ == 9)
        {
            req_pub_->publish(old_gait_req_);
            return;
        }

        if (std::abs(msg.velocity[0]) < 1e-9 &&
            std::abs(msg.velocity[1]) < 1e-9 &&
            std::abs(msg.yaw_speed) < 1e-9)
        {
            return;
        }

        float x_vel= msg.velocity[0];
        float y_vel = msg.velocity[1];
        float yaw_vel = msg.yaw_speed;
        cmd_vel_msg_.linear.x = x_vel;
        cmd_vel_msg_.linear.y = y_vel;
        cmd_vel_msg_.angular.z = yaw_vel;
        cmd_vel_pub_->publish(cmd_vel_msg_);
    }

private:
    /* 2‑second ignore window after posture change ----------------------- */
    void startIgnoreTimer()
    {
        ignore_cmd_ = true;
        ignore_timer_ = this->create_wall_timer(std::chrono::seconds(2),
                         [this]() { ignore_cmd_ = false; ignore_timer_->cancel(); });
    }

    /* --------------------- internal state ------------------------------ */
    std::mutex                      mutex_;
    bool                            ignore_cmd_   = false;
    rclcpp::TimerBase::SharedPtr    ignore_timer_;
    rclcpp::TimerBase::SharedPtr    delay_timer_;
};

/* --------------------------------  main  ---------------------------------- */
int main(int argc,char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<B2DDS>());
    rclcpp::shutdown();
    return 0;
}
