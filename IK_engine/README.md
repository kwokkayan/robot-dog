# IK_engine
## File Structure
- `src/champ*`: CHAMP files, refer to [here](https://github.com/chvmp/champ) for instructions.
- `src/champ_setup_assistant`: refer to [here](https://github.com/chvmp/champ_setup_assistant) for instructions.
- `src/champ_teleop`: refer [here](https://github.com/chvmp/champ_teleop) for instructions.
- `src/esp32_ros_confollers`: `ros_control` package for esp32 hardware.
- `src/realsense2_description`: URDF and meshs for Intel realsense cameras.
- `src/test_config`: main config for robot:
    - `urdf`: URDF files and meshes for the robot
    - `config`: configuration files for gait, pid, ...
    - `launch`: launch files
## How to run
- Joint calibration mode: `roslaunch test_config bringup.launch rviz:=true hardware_connected:=true publish_joint_control:=false calibrate_joints:=true balance_mode:=false`
- Hardware connected without balance mode: `roslaunch test_config bringup.launch hardware_connected:=true balance_mode:=false`
- Hardware connected with balance mode: `roslaunch test_config bringup.launch hardware_connected:=true`
- Hardware connected (TCP protocol) with balance mode: `roslaunch test_config bringup.launch hardware_connected:=true ros_serial_tcp:=true`
- Hardware connected with custom baud rate and device file: `roslaunch test_config bringup.launch hardware_connected:=true baud_rate:=1000000 device_file:=/dev/ttyUSB0`
- Joystick control:`roslaunch champ_teleop teleop.launch joy:=true`
- Refer to CHAMP for other commands
## Configuration
- Balance Mode pid: go to `src/test_config/config/pid/pid.yaml`
- Gait config: go to `src/test_config/config/gait/gait.yaml`