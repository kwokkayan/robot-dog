# ROS Humble packages

## File Structure

- `imu_tools`: provides madgwick filter to fuse IMU sensor data into an orientation
- `isaac_ros_common`: provides common packages and utilities in Isaac ROS
- `isaac_ros_image_segmentation`: provides image segmentation packages
- `isaac_ros_nvblox`: provides Nvblox, a 3D scene reconstruction package
- `isaac_ros_object_detection`: provides object detection packages (e.g., YOLO)
- `m-explore`: provides frontier exploration and map merging packages
- `pose_estimation_filter`: provides human pose estimation package for dynamic object following
- `realsense-ros`: provides ROS wrapper for Intel RealSense camera

## How to run

- Base mode: `ros2 launch nvblox_examples_bringup realsense_nav2_example.launch.py`
- Run frontier exploration node: `ros2 launch explore_lite explore.launch.py`
- Run dynamic object following node: `ros2 run pose_estimation_filter evaluator`
  - Please see [isaac_ros_unet - Quickstart](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_image_segmentation/isaac_ros_unet/index.html#quickstart) to configure and start the image segmentation node
