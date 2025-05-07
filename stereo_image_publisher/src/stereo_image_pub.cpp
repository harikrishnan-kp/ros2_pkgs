// ROS2 node that captures images from two USB cameras using OpenCV and
// publishes image and camera info of each camera with synchronized timestamps(same time stamp)
// we should provide camera caliberation data as yaml file
// output image type bgr8 
//
// Ref: https://github.com/ros-drivers/usb_cam
//      https://github.com/klintan/ros2_usb_camera
//      https://github.com/clydemcqueen/opencv_cam
// 
// TODO:
//      make node parmaters- yaml file path,camera devices,frame rate,resolution,camera frame, topics
//      read camera name from yaml file
//      work even if yaml file is not provided
//      compressed image transport
//      publish stereo cam info (stereo calibration data)

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

class DualCameraPublisher : public rclcpp::Node
{
public:
    DualCameraPublisher()
        : Node("dual_camera_publisher"),
        camera1_info_manager_(std::make_shared<camera_info_manager::CameraInfoManager>(this, "c270_hd_webcam", "file:///home/hari/hk/tasks/stereovision/c270_hd_webcam.yaml")),
        camera2_info_manager_(std::make_shared<camera_info_manager::CameraInfoManager>(this, "c270_hd_webcam", "file:///home/hari/hk/tasks/stereovision/c270_hd_webcam.yaml"))
    {
        // Open the left and right camera devices
        cap_left_.open(0);  // left cam - /dev/video0
        cap_right_.open(2); // right cam - /dev/video2
        
        if (!cap_left_.isOpened() || !cap_right_.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open one or both camera devices.");
            rclcpp::shutdown();
            return;
        }
        
        // Set camera properties if needed (e.g., resolution)
        cap_left_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        cap_left_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
        cap_right_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        cap_right_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
        
        // Initialize publishers for left and right camera images
        left_img_pub = this->create_publisher<sensor_msgs::msg::Image>("/left/image_raw", 10);
        right_img_pub = this->create_publisher<sensor_msgs::msg::Image>("/right/image_raw", 10);
        info_pub1 = this->create_publisher<sensor_msgs::msg::CameraInfo>("/left/camera_info", 10);
        info_pub2 = this->create_publisher<sensor_msgs::msg::CameraInfo>("/right/camera_info", 10);

        // Create a timer to capture and publish images at a fixed rate
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33), // Approximately 30 FPS
            std::bind(&DualCameraPublisher::capture_and_publish, this));
    }

private:
    void capture_and_publish()
    {
        cv::Mat frame_left, frame_right;

        // Capture frames from both cameras
        cap_left_ >> frame_left;
        cap_right_ >> frame_right;

        if (frame_left.empty() || frame_right.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Empty frame captured from one or both cameras.");
            return;
        }

        // Get the current ROS time
        rclcpp::Time timestamp = this->now();

        // Convert OpenCV images to ROS Image messages
        auto left_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame_left).toImageMsg();
        auto right_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame_right).toImageMsg();
        // Set the same timestamp for both images
        left_msg->header.stamp = timestamp;
        right_msg->header.stamp = timestamp;
        // Optionally set frame IDs
        left_msg->header.frame_id = "camera_left_frame";
        right_msg->header.frame_id = "camera_right_frame";
        
        // Get CameraInfo messages
        auto camera_info1 = camera1_info_manager_->getCameraInfo();
        auto camera_info2 = camera2_info_manager_->getCameraInfo();
        camera_info1.header.stamp = timestamp;
        camera_info2.header.stamp = timestamp;
        camera_info1.header.frame_id = "camera_left_frame";
        camera_info2.header.frame_id = "camera_right_frame";
        

        // Publish the messages
        left_img_pub->publish(*left_msg);
        right_img_pub->publish(*right_msg);
        info_pub1->publish(camera_info1);
        info_pub2->publish(camera_info2);
    }

 
    cv::VideoCapture cap_left_;
    cv::VideoCapture cap_right_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub1, info_pub2;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_img_pub, right_img_pub;
    std::shared_ptr<camera_info_manager::CameraInfoManager> camera1_info_manager_, camera2_info_manager_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DualCameraPublisher>());
    rclcpp::shutdown();
    return 0;
}
