# stereo image publisher
This ROS 2 node captures images from two USB cameras using OpenCV (optionally integrated with GStreamer) and publishes both the image and corresponding camera information for each camera. Key features include:
- Image Encoding: bgr8 
- Synchronized Timestamps: Images from both cameras are published with identical timestamps
- Accepts camera calibration data provided in a YAML file, adhering to the standard ROS camera calibration format. This calibration data is used to populate the sensor_msgs/CameraInfo messages for each camera
- Utilizes OpenCV's cv::VideoCapture for image acquisition. If GStreamer is available and configured, it can be employed to enhance performance and provide greater control over camera parameters

## Published Topics
 * **left/image_raw** (sensor_msgs::msg::Image): ROS Image message of your input file.
 * **left/camera_info** (sensor_msgs::msg::CameraInfo): CameraInfo published along with Image.
 * **right/image_raw** (sensor_msgs::msg::Image): ROS Image message of your input file.
 * **right/camera_info** (sensor_msgs::msg::CameraInfo): CameraInfo published along with Image.

## Parameters
- **left_frame_id** (string, default: "camera_left") Frame id inserted in published image and camera_info.
- **right_frame_id** (string, default: "camera_right") Frame id inserted in published image and camera_info.
- **left_yaml_path** (string, default: ""): Path to left bcamera info.  (~/.ros/camera_info/narrow_stereo/left.yaml)
- **right_yaml_path** (string, default: ""): Path to right camera info.
- **video_device_left** (int, default: 0): Video device used (/dev/video0)
- **video_device_right** ((int, default: 2)) Video device used (/dev/video2)
- **publish_rate** (int, default: 25): Rate to publish image (hz).


- **filename** (string, default: ""): Name of image file to be published.
- **field_of_view** (double, default: 0): Camera field of view (deg) used to calculate focal length for camera info topic.
 

## example usage
Run node
```
ros2 run stereo_image_publisher stereo_image_pub
```
parameter setting and topic remaping
```
ros2 run stereo_image_publisher stereo_image_pub \
  --ros-args \
  -p left_yaml_path:=/home/user/calib/left.yaml \
  -p right_yaml_path:=/home/user/calib/right.yaml \
  -p left_frame_id:=left_cam_new \
  -p right_frame_id:=right_cam_new \
  -p video_device_left:=1 \
  -p video_device_right:=4 \
  -p publish_rate:=30 \
  --remap /left/image_raw:=/stereo/left/image_raw \
  --remap /right/image_raw:=/stereo/right/image_raw \
  --remap /left/camera_info:=/stereo/left/camera_info \
  --remap /right/camera_info:=/stereo/right/camera_info
```
## TODO
- Enable setting parameters: yaml file paths, camera devices, camera frame, frame rate. (finished)
- Ensure the node operates even if the YAML file(calibration data) is absent. (finished)
- Enable setting parameters: resolution
- functionality to read the camera name from the YAML file
- publish stereo cam info (stereo calibration data)
- Integrate compressed_image_transport for efficient image data handling.
- provide example format for camera calibration files
- provide link to camera calibration in documenation
- next version
    - camera calibration (individual and stereo)
    - image rectification
- verify publish_rate parameter setup is correct

## Issues 
- image publish rate control mechanism is not working as expected, not getting required result when tested with `ros2 topic hz /<topic name>`

## Ref: 
- https://github.com/ros-drivers/usb_cam
- https://github.com/klintan/ros2_usb_camera
- https://github.com/clydemcqueen/opencv_cam