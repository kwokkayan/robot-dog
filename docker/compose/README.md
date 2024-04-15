# ROS Docker Compose
## Overview
This compose currently contains the following images:
1. ros-melodic:
    - base image: `dorowu/ubuntu-desktop-lxde-vnc:bionic-lxqt` (Ubuntu 18)
    - description: Installs ROS melodic and champ on Ubuntu 18. Provides a web GUI through lxde and vnc.
## Usage
1. ros-melodic:
    - Use `http://localhost:6080` to access the web GUI.
    - Champ is installed and built under `/champ`. Use the docker volume `champ` to interact with it from the host machine.
    - Web GUI Resoulution is set in `docker-compose.yml`. Please refer to [base image](https://hub.docker.com/r/dorowu/ubuntu-desktop-lxde-vnc/) for detailed configuration.
    - Pass device files to the container in `docker-compose.yml` under `device`:
        - joystick: `/dev/input/js0:/dev/input/js`
        - rfcomm socket: `/dev/rfcomm1:/dev/ttyUSB0`
        - usb socket: `/dev/ttyUSB0:/dev/ttyUSB0`
    - The following ports are opened:
        - 22: ssh
        - 5900: ?
        - 6080: vnc
        - 9090: websocket
        - 11311, 11411: ros serial TCP socket
## References
1. [champ](https://github.com/chvmp/champ)
2. [ubuntu desktop image](https://hub.docker.com/r/dorowu/ubuntu-desktop-lxde-vnc/)
3. [ROS melodic installation guide](https://wiki.ros.org/melodic/Installation/Ubuntu)
