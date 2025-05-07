# stereo image publisher
- ROS2 node that captures images from two USB cameras using OpenCV and publishes image and camera info of each camera with synchronized timestamps(same time stamp)
- we should provide camera caliberation data as yaml file
- output image type bgr8 

## Ref: 
- https://github.com/ros-drivers/usb_cam
- https://github.com/klintan/ros2_usb_camera
- https://github.com/clydemcqueen/opencv_cam
 
## TODO
- Enable setting parameters: yaml file path,camera devices,frame rate,resolution,camera frame, topics
- functionality to read the camera name from the YAML file
- publish stereo cam info (stereo calibration data)
- Ensure the node operates even if the YAML file(calibration data) is absent.
- Integrate compressed_image_transport for efficient image data handling.
- provide example format for camera calibration files
- provide link to camera calibration
- next version
    - camera calibration (individual and stereo)
    - image rectification