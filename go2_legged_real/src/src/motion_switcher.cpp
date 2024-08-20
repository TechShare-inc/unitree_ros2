    #include <iostream>
    #include <stdio.h>
    #include <stdint.h>
    #include <math.h>
    #include "rclcpp/rclcpp.hpp"
    #include <unitree/robot/channel/channel_publisher.hpp>
    #include <unitree/robot/channel/channel_subscriber.hpp>
    #include <unitree/common/time/time_tool.hpp>
    #include "unitree/idl/go2/SportModeCmd_.hpp"
    // #include "std_msgs/msg/string.hpp"
    // #include <unitree/robot/go2/sport/sport_client.hpp>
    #include <cstring>
    #include "std_srvs/srv/trigger.hpp"
    #include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>



    using namespace unitree::common;
    using namespace unitree::robot;
    using namespace unitree::robot::b2;

    #define TOPIC "rt/motion_switcher"
    class MotionSwitcher{

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

    void MessageHandler(const void* message)
    {
        const unitree_go::msg::dds_::SportModeCmd_* pmsg = (const unitree_go::msg::dds_::SportModeCmd_*)message;
        MotionSwitcher ms;
        ms.InitMotionSwitcherClient();
        ms.queryMotionStatus();

        // Assume the mode is provided via another method or is a fixed string
        std::string mode = "ai";  // Example mode, you can modify how this is handled
        std::cout << "THE CHOSEN WALKING MODE IS " << mode << std::endl;

        if (mode == "normal" || mode == "ai") {
            ms.SelectMode(mode);
        } else {
            std::cout << "YOU have to choose the walking mode either normal or ai" << std::endl;
            return;
        }

        bool silent_flag = ms.checkSilent();

        if (!silent_flag) {
            ms.setSilent(true);
        } else {
            std::cout << "This robot is already in silent mode" << std::endl;
        }

    }

    int main(int argc, char *argv[])
    {
        std::string interface = "enp87s0";
        std::cout << "Initializing ChannelFactory..." << std::endl;
        ChannelFactory::Instance()->Init(0, interface);
        std::cout << "ChannelFactory initialized." << std::endl;
        // ChannelFactory::Instance()->Init(0);
        ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeCmd_> subscriber = ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeCmd_>  \
            (new ChannelSubscriber<unitree_go::msg::dds_::SportModeCmd_>(TOPIC));
        subscriber->InitChannel(std::bind(MessageHandler, std::placeholders::_1), 1);   
        while (true)
        {
            sleep(1);
        }

        return 0;
    }

