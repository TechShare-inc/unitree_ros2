/**
 * This example demonstrates how to use ROS2 to receive low states of unitree go2 robot
 **/
#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/go2_front_video_data.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
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
    
    // Publishers for raw and compressed images
      // image_transport_ = std::make_shared<image_transport::ImageTransport>(this->shared_from_this());
      // raw_image_pub_ = image_transport_->advertise("image_raw", 1);
      compressed_image_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("/front_camera/image_raw/compressed", 1);
      raw_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/front_camera/image_raw", 1);


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
        cv::VideoCapture cap("udpsrc address=230.1.1.1 port=1720 multicast-iface=enp86s0 ! application/x-rtp, media=video, encoding-name=H264 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! video/x-raw,width=1280,height=720,format=BGR ! appsink drop=1", 
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

            // Publish raw image


            // cv::imshow("receiver", frame);
            if(!frame.empty())
              publish_image(frame);
            if (cv::waitKey(3) == 27) {
                break;
            }
        }
        cap.release();
    }
    void publish_image(const cv::Mat &original_frame) {
        // Resize frame to 640x480
        cv::Mat resized_frame;
        cv::resize(original_frame, resized_frame, cv::Size(640, 480));

        // Convert to ROS image message for raw image
        sensor_msgs::msg::Image::SharedPtr image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", resized_frame).toImageMsg();
        RCLCPP_INFO(this->get_logger(), "ROS Image encoding: %s", image_msg->encoding.c_str());
        // sensor_msgs::msg::Image raw_image_msg;
        // raw_image_msg.header = image_msg->header;
        image_msg->header.stamp = this->now();
        image_msg->header.frame_id = "camera_link";
        // raw_image_msg.data = image_msg->data;
        raw_image_pub_->publish(*image_msg.get());

        // // Compress in H264 format
        // std::vector<uchar> buffer;
        // std::vector<int> compression_params = {
        //     cv::IMWRITE_JPEG_QUALITY, 95,  // You can adjust the quality
        //     cv::IMWRITE_JPEG_PROGRESSIVE, 1,
        //     cv::IMWRITE_JPEG_OPTIMIZE, 1
        // };

        // // Encode the frame
        // cv::imencode(".jpg", resized_frame, buffer, compression_params);

        // // Create compressed image message
        // sensor_msgs::msg::CompressedImage compressed_image_msg;
        // compressed_image_msg.header = image_msg->header;
        // compressed_image_msg.header.frame_id = "camera_link";
        // compressed_image_msg.format = "jpeg"; // Set format to jpeg
        // compressed_image_msg.data = std::move(buffer);
        // compressed_image_pub_->publish(compressed_image_msg);
        // Compress and publish compressed image


        sensor_msgs::msg::CompressedImage compressed_image_msg;
        cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", resized_frame).toCompressedImageMsg(compressed_image_msg);
        compressed_image_msg.header.stamp = this->now();
        compressed_image_msg.header.frame_id = "camera_link";
        compressed_image_msg.format = "jpeg"; // You can choose other formats like png
        compressed_image_pub_->publish(compressed_image_msg);
    }

  // Create the suber  to receive low state of robot
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_image_pub_;
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