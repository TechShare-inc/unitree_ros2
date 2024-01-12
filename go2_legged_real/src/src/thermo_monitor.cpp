/**
 * This example demonstrates how to use ROS2 to receive low states of unitree go2 robot
 **/
#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "unitree_go/msg/imu_state.hpp"
#include "unitree_go/msg/motor_state.hpp"
#include "unitree_go/msg/stat.hpp"
#include <iostream>
#include <fstream>
#include <string>
#include <thread>
#include <chrono>
#include <unistd.h>
#include <mutex>

#define INFO_CPU 0        // Set 1 to info IMU states
#define INFO_MOTOR 0      // Set 1 to info motor states
#define INFO_BATTERY 0    // Set 1 to info battery states

using std::placeholders::_1;

class thrmo_monitor : public rclcpp::Node
{
public:
  thrmo_monitor() : Node("thrmo_monitor")
  {
    // suber is set to subscribe "/lowcmd" or  "lf/lowstate" (low frequencies) topic

    // The suber  callback function is bind to low_state_suber::topic_callback
    low_state_sub_ = this->create_subscription<unitree_go::msg::LowState>(
        "lf/lowstate", 10, std::bind(&thrmo_monitor::topic_callback, this, _1));
    stat_pub_ = this->create_publisher<unitree_go::msg::Stat>(
        "unitree_stat", 10
    );
    timer_ = this->create_wall_timer(
            std::chrono::seconds(1), // 0.1 seconds
            std::bind(&thrmo_monitor::timer_callback, this));

  }
    ~thrmo_monitor() 
  {
    // Destructor code
    // Clean up code if necessary
    RCLCPP_INFO(this->get_logger(), "Destroying thermo_monitor");
  }

private:

  void timer_callback(){
    std::lock_guard<std::mutex> lock(stats_mutex_);
    stat_pub_->publish(stat);
  }

float get_cpu_temp() {
    std::ifstream file("/sys/class/thermal/thermal_zone0/temp");
    if (!file.is_open()) {
        std::cerr << "Failed to open temperature file." << std::endl;
        return -1;
    }
    
    float temp;
    file >> temp;
    file.close();

    return temp / 1000; // Convert from millidegrees
}

float get_cpu_usage() {
    static size_t prev_idle_time = 0, prev_total_time = 0;
    std::ifstream file("/proc/stat");
    if (!file.is_open()) {
        std::cerr << "Failed to open stat file." << std::endl;
        return -1;
    }

    std::string line;
    getline(file, line);
    file.close();

    size_t idle, total, del_total, del_idle;
    
    std::istringstream iss(line);
    std::string cpu;
    size_t user, nice, system, idle_now, iowait, irq, softirq, steal, guest, guest_nice;
    iss >> cpu >> user >> nice >> system >> idle_now >> iowait >> irq >> softirq >> steal >> guest >> guest_nice;

    idle = idle_now + iowait;
    total = user + nice + system + idle + irq + softirq + steal;

    del_idle = idle - prev_idle_time;
    del_total = total - prev_total_time;

    prev_idle_time = idle;
    prev_total_time = total;

    if (del_total == 0) {
        return 0; // To avoid division by zero
    }

    return 100.0 * (1.0 - (static_cast<float>(del_idle) / del_total));
}

    void update_motor_temp(int motor_index, int temp) {
        switch (motor_index) {
            case 0: stat.fl_0 = temp; break;
            case 1: stat.fl_1 = temp; break;
            case 2: stat.fl_2 = temp; break;
            case 3: stat.fr_0 = temp; break;
            case 4: stat.fr_1 = temp; break;
            case 5: stat.fr_2 = temp; break;
            case 6: stat.rl_0 = temp; break;
            case 7: stat.rl_1 = temp; break;
            case 8: stat.rl_2 = temp; break;
            case 9: stat.rr_0 = temp; break;
            case 10: stat.rr_1 = temp; break;
            case 11: stat.rr_2 = temp; break;
            default: 
                RCLCPP_WARN(this->get_logger(), "Invalid motor index: %d", motor_index);
            
            if (INFO_MOTOR)
            {
                RCLCPP_INFO(this->get_logger(), "Motor state -- %s: temperature: %d", motor_names[motor_index].c_str(), temp);
            }
        }
    }


  void topic_callback(unitree_go::msg::LowState::SharedPtr data)
  {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    stat.cpu_temp = get_cpu_temp();
    stat.cpu_usage = get_cpu_usage();
    if (INFO_CPU)
    {
        RCLCPP_INFO(this->get_logger(), "CPU Temperature: %d   CPU Usage: %f", stat.cpu_temp, stat.cpu_usage); 
    }


      // Info motor states
      // q: angluar (rad)
      // dq: angluar velocity (rad/s)
      // ddq: angluar acceleration (rad/(s^2))
      // tau_est: Estimated external torque

      


      // FR_0 -> 0 , FR_1 -> 1  , FR_2 -> 2   The motor control sequence is currently only 12 motors, which will be retained later.
      // FL_0 -> 3 , FL_1 -> 4  , FL_2 -> 5
      // RR_0 -> 6 , RR_1 -> 7  , RR_2 -> 8
      // RL_0 -> 9 , RL_1 -> 10 , RL_2 -> 11




    for (int i = 0; i < 12; i++)
    {
    // Debug print
    update_motor_temp(i, data->motor_state[i].temperature);        
    }
          // Info battery states
      // battery current
      // battery voltage
    stat.current = data->power_a;
    stat.voltage = data->power_v;
    stat.soc = data->bms_state.soc;

    if (INFO_BATTERY)
    {


      RCLCPP_INFO(this->get_logger(), "Battery state -- current: %f; voltage: %f; soc: %d", stat.current, stat.voltage, stat.soc);
    }
    

  }

  // Create the suber  to receive low state of robot
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr low_state_sub_;
  rclcpp::Publisher<unitree_go::msg::Stat>::SharedPtr stat_pub_;

  unitree_go::msg::IMUState imu;         // Unitree go2 IMU message
  unitree_go::msg::MotorState motor[12]; // Unitree go2 motor state message
  unitree_go::msg::Stat stat;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mutex stats_mutex_;
  std::string motor_names[12] = {"FR_0", "FR_1", "FR_2", 
                                "FL_0", "FL_1", "FL_2", 
                                "RR_0", "RR_1", "RR_2", 
                                "RL_0", "RL_1", "RL_2" 
                                };
//    int motor_tempratures[12] = {stat.fl_0, stat.fl_1, stat.fl_2,
//                             stat.fr_0, stat.fr_1, stat.fr_2,
//                             stat.rl_0, stat.rl_1, stat.rl_2,
//                             stat.rr_0, stat.rr_1, stat.rr_2
//    };

};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);                          // Initialize rclcpp
  rclcpp::spin(std::make_shared<thrmo_monitor>()); // Run ROS2 node which is make share with low_state_suber class
  rclcpp::shutdown();
  return 0;
}