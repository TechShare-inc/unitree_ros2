#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
#include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"
#include <cstring>
#include <iostream>
#include "std_srvs/srv/trigger.hpp"

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree::robot::b2;

class MotionController
{
public:
    void InitMotionSwitcherClient()
    {
        msc.SetTimeout(10.0f); 
        msc.Init();
    }


    int queryMotionStatus() {
        std::string robotForm,motionName;
        int motionStatus;
        msc.CheckMode(robotForm,motionName);
        std::cout << "Current robotform: " << (robotForm == "0" ? "WALK MODE" : "WHEEL MODE") << std::endl;
        if(motionName.empty())
        {
            std::cout<<"sport_mode or ai_sport is deactivate "<<std::endl;
            motionStatus = 0;
        }
        else
        {
            std::cout << (motionName == "normal" ? "sport_mode is activate" : "ai_sport is activate") << std::endl;
            motionStatus = 1;
        }
        return motionStatus;
    }

    void SelectMode(const std::string &mode) {
        msc.SelectMode(mode);
    }

    bool checkSilent() {
        bool isSilent = false;
        msc.GetSilent(isSilent);
        return isSilent;
    }

    int32_t setSilent(bool silent) {
        return msc.SetSilent(silent);
    }

    void ReleaseMode() {
        msc.ReleaseMode();
    }
private:
    MotionSwitcherClient msc;
};




// class MotionController
// {
// public:
//     explicit MotionController(){}
//     ~MotionController(){}

//     void InitMotionSwitcherClient();
//     int queryMotionStatus();
//     void ReleaseMode();
//     void SelectMode(const std::string& mode);
//     bool checkSilent();
//     int32_t setSilent(bool flag);

 
// private:

   
// };



// void MotionController::

// int MotionController::queryMotionStatus()
// {
//     std::string robotForm,motionName;
//     int motionStatus;
//     msc.CheckMode(robotForm,motionName);
//     std::cout << "Current robotform: " << (robotForm == "0" ? "WALK MODE" : "WHEEL MODE") << std::endl;
//     if(motionName.empty())
//     {
//         std::cout<<"sport_mode or ai_sport is deactivate "<<std::endl;
//         motionStatus = 0;
//     }
//     else
//     {
//         std::cout << (motionName == "normal" ? "sport_mode is activate" : "ai_sport is activate") << std::endl;
//         motionStatus = 1;
//     }
//     return motionStatus;
// }

// void MotionController::SelectMode(const std::string& mode){
    
// }

// void MotionController::ReleaseMode()
// {
//     msc.ReleaseMode(); 
// }

// bool MotionController::checkSilent(){
//     bool isSilent = false;
//     msc.GetSilent(isSilent);
//     return isSilent;
// }

// int32_t MotionController::setSilent(bool flag){
//     return msc.SetSilent(flag);
// }
class MotionTriggerNode : public rclcpp::Node
{
public:
    MotionTriggerNode() : Node("motion_trigger_node")
    {
        trigger_service_ = this->create_service<std_srvs::srv::Trigger>(
            "trigger_motion", std::bind(&MotionTriggerNode::handle_trigger, this, std::placeholders::_1, std::placeholders::_2));
    }
private:
    void handle_trigger(const std_srvs::srv::Trigger::Request::SharedPtr request,
                        std_srvs::srv::Trigger::Response::SharedPtr response)
    {
        // Trigger the motion sequence
        custom.queryMotionStatus();

        // Assume the mode is provided via another method or is a fixed string
        std::string mode = "ai";  // Example mode, you can modify how this is handled
        std::cout << "THE CHOSEN WALKING MODE IS " << mode << std::endl;

        if (mode == "normal" || mode == "ai") {
            custom.SelectMode(mode);
        } else {
            std::cout << "YOU have to choose the walking mode either normal or ai" << std::endl;
            response->success = false;
            response->message = "Invalid walking mode.";
            return;
        }

        bool silent_flag = custom.checkSilent();

        if (!silent_flag) {
            custom.setSilent(true);
            response->message = "Robot was not in silent mode, set to silent now.";
        } else {
            std::cout << "This robot is already in silent mode" << std::endl;
            response->message = "Robot is already in silent mode.";
        }

        custom.ReleaseMode();
        response->success = true;
    }

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_service_;
    MotionController custom; // Assuming this is your class handling motion control
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotionTriggerNode>());
    rclcpp::shutdown();
    return 0;
}


// int main(int argc, const char** argv)
// {
//     if (argc < 3)
//     {
//         std::cout << "Usage: " << argv[0] << " networkInterface| " << argv[1] << " WALK MODE|" << argv[2] << std::endl;
//         exit(-1); 
//     }


//     // std::cin.ignore();
//     std::cout << "THE NETWORKE INTERFACE IS " << argv[1] << std::endl;
//     ChannelFactory::Instance()->Init(0, argv[1]);

//     Custom custom;
//     custom.InitMotionSwitcherClient();
//     custom.queryMotionStatus();
//     std::cout << "THE CHOSEN WALKING MODE IS " << argv[2] << std::endl;
//     if (strcmp(argv[2], "normal") == 0 || strcmp(argv[2], "ai") == 0){
//         custom.SelectMode(argv[2]);
//     }else{
//         std::cout << "YOU have to choose the walking mode either normal or ai" << std::endl;
//     }
//     bool silent_flag = custom.checkSilent();

//     if (!silent_flag){
//         custom.setSilent(true);
//     }else{
//         std::cout << "This robot is already in silent mode" << std::endl;

//     }
//     custom.ReleaseMode();
//     return 0;
// }
