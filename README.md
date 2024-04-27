# 23/24 COMP4601 Final Year Project

## Group information

- group: fyp23005
- title: Intelligent robot design and implementation

## File Structure

- `3D_model`: STL and blender files
- `docker`: docker files for development and production environments
- `workspaces/esp32_champ_dog`: new firmware for champ
- `workspaces/newviz`: React web application for live visualizations
- `workspaces/ros/noetic`: ROS1 packages including CHAMP
- `workspaces/ros/humble`: ROS2 packages including CV utilities

## Requirements

1. `docker`, `docker-compose`, and [`docker-buildx-plugin`](https://github.com/docker/buildx)
2. Alternatively, ROS melodic to run `workspaces/ros/noetic` and ROS humble to run others.
3. `nodejs` > 21.0.0
4. Hardware requirements:
   - ESP32 microcontroller
   - MPU-9250
   - PCA9685 PWM controller
   - 12 PWM servos (MG996R, MG958, TD81XXG series...)
   - Nvidia Jetson Nano
   - Intel Realsense D435i

## How to run

### Firmware

1. Use PlatformIO to open `esp32_champ_dog`
2. Build and flash the firmware to `esp32`
3. Refer to `README.md` in `esp32_champ_dog` for further information.

### Development: CHAMP with ROS Melodic

1. Configure `docker-compose.yml`:
   - Web GUI Resoulution is set in `docker-compose.yml`. Please refer to [base image](https://hub.docker.com/r/dorowu/ubuntu-desktop-lxde-vnc/) for detailed configuration.
   - Pass device files to the container in `docker-compose.yml` under `device`:
     - joystick: e.g. `/dev/input/js0:/dev/input/js`
     - rfcomm socket: e.g. `/dev/rfcomm1:/dev/ttyUSB0`
     - usb socket: e.g. `/dev/ttyUSB0:/dev/ttyUSB0`
   - The following ports are opened:
     - 22: ssh
     - 5900: ?
     - 6080: vnc
     - 9090: websocket
     - 11311, 11411: ros serial TCP socket
   - Uncomment `bind volumne` for live development
2. Run `docker-compose up -d`.
   - Use `http://localhost:6080` to access the web GUI.
   - CHAMP and other packages is installed and built under `/IK_engine`.
3. Refer to `README.md` in `IK_engine` for further information.

### Development: React webapp

1. `cd workspaces/newviz`
2. `npm i`: install dependencies
3. Configure `.env`:
   - VITE_ROS_PACKAGE=public
   - VITE_URDF_PATH=public/quadruped.urdf
   - VITE_WS_URL=`websocket url for middleware`
   - VITE_CAMERA_STREAM_URL=`http url for video stream`
4. `npm run dev`: start web server
5. Refer to `README.md` in `workspaces/newviz` for further information.

### Production environments

Start the docker container for running ROS noetic packages

```bash
$ cd docker/
$ ./build.sh noetic
$ ./run.sh noetic
> roslaunch test_config bringup.launch
```

Start the docker container for running ROS1-to-ROS2 bridge

```bash
$ cd docker/
$ ./build.sh bridge
$ ./run.sh bridge
> ros2 run ros1_bridge dynamic_bridge 2>/dev/null
```

Start the docker container for running ROS humble packages (Note that the building process of ROS packages are done after starting the container as host CUDA library are required)

```bash
$ cd docker/
$ ./build.sh humble
$ ./run.sh humble
> source /opt/ros/humble/setup.bash && colcon build --symlink-install
> source install/setup.bash && ros2 launch nvblox_examples_bringup realsense_nav2_example.launch.py
```

Start the docker container for running the web dashboard (Please refer to the `README.md` in `workspaces/newviz` for getting a Firebase API key)

```bash
$ vim workspaces/newviz/.env # To configure VITE_FIREBASE_API_KEY
$ cd docker/
$ ./build.sh newviz
$ docker run --rm -p 127.0.0.1:8080:80 ros_custom-newviz-$(uname -m)
```

Then, you can access the web dashboard at `http://127.0.0.1:8080/`.
