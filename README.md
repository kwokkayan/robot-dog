# 23/24 COMP4601 Final Year Project
## Group information
- group: fyp23005
- title: Intelligent robot design and implementation
## File Structure
- `esp32_champ_dog`: new firmware for champ
- `3D_model`: STL and blender files
- `docker`: docker files for development and production environments
- `IK_engine`: ROS1 packages including CHAMP
- `newviz`: React web application for live vizualizations
## Requirements
1. `docker` and `docker-compose`
2. Alternatively, ROS melodic to run `IK_engine` and ROS humble to run others.
3. `nodejs` > 21.0.0
4. Hardware requirements:
    - ESP32 microcontroller
    - MPU-9250
    - PCA9685 PWM controller
    - 12 PWM servos (MG996R, MG958, TD81XXG series...)
    - Nvidia Jetson Nano
    - Intel Realsense D435i
## How to run
### Install submodules
- `git submodule update --init` 
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
1. `cd newviz`
2. `npm i`: install dependencies
3. Configure `.env`:
    - VITE_ROS_PACKAGE=public
    - VITE_URDF_PATH=public/quadruped.urdf
    - VITE_WS_URL=`websocket url for middleware`
    - VITE_CAMERA_STREAM_URL=`http url for video stream`
4. `npm run dev`: start web server
5. Refer to `README.md` in `newviz` for further information. 
### Production environments
#### TODO