FROM dorowu/ubuntu-desktop-lxde-vnc:bionic-lxqt AS install_ros
SHELL ["/bin/bash", "-c"]
# Link ROS repositories to apt
RUN apt update && apt install gpg-agent -y
RUN sh -c \
  'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > \
  /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
  | apt-key add -
# Install ROS and update bash profile
RUN apt update && \
  apt upgrade -y && \
  apt install ros-melodic-desktop-full -y && \
  echo "source /opt/ros/melodic/setup.bash" >> /etc/bash.bashrc
# Install other dependencies
RUN source /etc/bash.bashrc && \
  apt install -y \
  ros-melodic-catkin \
  python-catkin-tools \
  python-rosdep \
  python-rosinstall\ 
  python-rosinstall-generator\ 
  python-wstool\ 
  build-essential\
  ros-melodic-rosbridge-suite &&\
  rosdep init 

FROM install_ros AS install_openssh
ARG OS_USER root
ARG OS_PASS 123
RUN apt-get install -y openssh-server
RUN echo PermitRootLogin=yes >> /etc/ssh/sshd_config
RUN echo $OS_USER:$OS_PASS | chpasswd

FROM install_openssh AS install_ik_engine_dep
# Copy local files
ARG IK_ENGINE_PATH ./IK_engine
COPY ${IK_ENGINE_PATH} /IK_engine
# Build champ and update bash profile
WORKDIR /IK_engine
RUN rosdep update --include-eol-distros && \
  rosdep install --from-paths src --ignore-src -r -y --rosdistro melodic

FROM install_ik_engine_dep AS build_ik_engine
WORKDIR /IK_engine
RUN source /opt/ros/melodic/setup.bash && catkin_make
RUN echo "source /IK_engine/devel/setup.bash" >> /etc/bash.bashrc
RUN apt autoremove -y