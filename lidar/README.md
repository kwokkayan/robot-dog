# LIDAR ros node
## Information
1. [ros driver](https://github.com/YDLIDAR/ydlidar_ros_driver)
2. [lidar model](https://www.ydlidar.com/products/view/5.html): X4. Use the corresponding ros launch file to test.
3. The driver is already installed on SBC under home directory of `user`.
## Tasks
1. Test connection between SBC and ros master on remote host (docker).
2. Integrate with champ framework. X4 is supported out-of-the-box. [reference](https://github.com/chvmp/champ/wiki/Hardware-Integration#5-light-detection-and-ranging-sensor-lidar)
## Notes
1. The latest ydlidar_ros_driver is not built correctly. Please use the commit hash (b2c53bf151577104e9e3dc102eb784d163363d71).
2. The default sample rate 9K would cause timeout issue due to unkown cause as described in the [issue](https://github.com/YDLIDAR/YDLidar-SDK/issues/18). Using a sample rate (e.g. 8K) which is a factor of the baudrate (128K) seems to be resolving the issue.

