/**
 * This example demonstrates how to use ROS2 to receive low states of unitree go2 robot
 **/
#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/go2_front_video_data.hpp"
#include <opencv2/opencv.hpp>
#include <vector>
#include <thread>
#include <iostream>
using std::placeholders::_1;

class video_stream : public rclcpp::Node
{
public:
  video_stream() : Node("video_stream")
  {

    // The suber  callback function is bind to low_state_suber::topic_callback
    // video_sub_ = this->create_subscription<unitree_go::msg::Go2FrontVideoData>(
    //     "frontvideostream", 10, std::bind(&video_stream::video_callback, this, _1));        
    // timer_ = this->create_wall_timer(
    //         std::chrono::milliseconds(100), // 0.1 seconds
    //         std::bind(&video_stream::timer_callback, this));
    worker_thread_ = std::thread(&video_stream::show_image, this);

  }

    ~video_stream(){
              // Signal the thread to stop
        running_ = false;

        // Wait for the thread to finish before exiting the destructor
        if (worker_thread_.joinable()) {
            worker_thread_.join();
        }      
    }



private:
    void show_image() {
        RCLCPP_INFO(this->get_logger(), "Started the show image"); 
        cv::VideoCapture cap("udpsrc address=230.1.1.1 port=1720 multicast-iface=eth0 ! application/x-rtp, media=video, encoding-name=H264 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! video/x-raw,width=1280,height=720,format=BGR ! appsink drop=1", 
        cv::CAP_GSTREAMER);
        if (!cap.isOpened()) {
            std::cerr <<"VideoCapture not opened"<< std::endl;
            RCLCPP_ERROR(this->get_logger(), "VideoCapture not opened"); 

            exit(-1);
        }
        RCLCPP_INFO(this->get_logger(), "Opend the cap"); 
        while (running_ && rclcpp::ok()) {
          cv::Mat frame;
          

            cap.read(frame);

            cv::imshow("receiver", frame);

            if (cv::waitKey(3) == 27) {
                break;
            }
        
        }
        cap.release();
    }
    // void timer_callback() {
    //     // Actions to perform on each timer tick

    // }


  // Create the suber  to receive low state of robot
  rclcpp::TimerBase::SharedPtr timer_;
  cv::Mat frame;
  std::thread worker_thread_;
  bool running_ = true;
};


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);                          // Initialize rclcpp
  rclcpp::spin(std::make_shared<video_stream>()); // Run ROS2 node which is make share with low_state_suber class
  rclcpp::shutdown();
  return 0;
}