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
        : Node("dual_camera_publisher")
    {
        // Default value for parameters
        std::string default_left_yaml = "";
        std::string default_right_yaml = "";
        std::string default_left_frame = "camera_left";
        std::string default_right_frame = "camera_right";
        int default_video_device_left = 0;  // -/dev/video0
        int default_video_device_right = 2; // -/dev/video2
        int default_publish_rate = 25;      // 25 FPS

        // Declare parameters
        this->declare_parameter<std::string>("left_yaml_path", default_left_yaml);
        this->declare_parameter<std::string>("right_yaml_path", default_right_yaml);
        this->declare_parameter<std::string>("left_frame_id", default_left_frame);
        this->declare_parameter<std::string>("right_frame_id", default_right_frame);
        this->declare_parameter<int>("video_device_left", default_video_device_left);   
        this->declare_parameter<int>("video_device_right", default_video_device_right);
        this->declare_parameter<int>("publish_rate", default_publish_rate);  

        // Get parameters
        this->get_parameter("left_yaml_path", yaml_path_left);
        this->get_parameter("right_yaml_path", yaml_path_right);
        this->get_parameter("left_frame_id", frame_id_left);
        this->get_parameter("right_frame_id", frame_id_right);
        this->get_parameter("video_device_left", left_video_device);
        this->get_parameter("video_device_right", right_video_device);
        this->get_parameter("publish_rate", fps);

        std::string left_uri = yaml_path_left.empty() ? "" : "file://" + yaml_path_left;
        std::string right_uri = yaml_path_right.empty() ? "" : "file://" + yaml_path_right;
        camera1_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, "narrow_stereo/left", left_uri);
        camera2_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, "narrow_stereo/right", right_uri);

        // Open the left and right camera devices
        cap_left_.open(left_video_device);
        cap_right_.open(right_video_device);

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
        milli_time = 1000/fps;
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(milli_time), 
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

        // Get the current ROS time to set same time stamp for messages
        rclcpp::Time timestamp = this->now();

        // Setup ROS Image messages
        auto left_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame_left).toImageMsg();
        auto right_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame_right).toImageMsg();
        left_msg->header.stamp = timestamp;
        right_msg->header.stamp = timestamp;
        left_msg->header.frame_id = frame_id_left;
        right_msg->header.frame_id = frame_id_right;

        // Get CameraInfo messages
        auto camera_info1 = camera1_info_manager_->getCameraInfo();
        auto camera_info2 = camera2_info_manager_->getCameraInfo();
        camera_info1.header.stamp = timestamp;
        camera_info2.header.stamp = timestamp;
        camera_info1.header.frame_id = frame_id_left;
        camera_info2.header.frame_id = frame_id_right;

        // Publish the messages
        left_img_pub->publish(*left_msg);
        right_img_pub->publish(*right_msg);
        info_pub1->publish(camera_info1);
        info_pub2->publish(camera_info2);
    }

    std::string yaml_path_left, yaml_path_right, frame_id_left, frame_id_right;
    int left_video_device, right_video_device, fps, milli_time;
    cv::VideoCapture cap_left_, cap_right_;
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
