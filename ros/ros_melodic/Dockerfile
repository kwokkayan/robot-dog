FROM dorowu/ubuntu-desktop-lxde-vnc:bionic-lxqt AS install_ros
SHELL ["/bin/bash", "-c"]
RUN apt update && apt install gpg-agent -y
RUN sh -c \
  'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > \
  /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
  | apt-key add -
RUN apt update && \
  apt upgrade -y && \
  apt install ros-melodic-desktop-full -y && \
  echo "source /opt/ros/melodic/setup.bash" >> /etc/bash.bashrc
RUN source /etc/bash.bashrc && \
  apt install -y \
  ros-melodic-catkin \
  python-catkin-tools \
  python-rosdep \
  python-rosinstall\ 
  python-rosinstall-generator\ 
  python-wstool\ 
  build-essential &&\
  rosdep init 

FROM install_ros AS build_champ
WORKDIR /champ/src
RUN git clone --recursive https://github.com/chvmp/champ
RUN git clone https://github.com/chvmp/champ_teleop
WORKDIR /champ
RUN rosdep update --include-eol-distros && \
  rosdep install --from-paths src --ignore-src -r -y --rosdistro melodic
RUN source /opt/ros/melodic/setup.bash && catkin_make
RUN echo "source /champ/devel/setup.bash" >> /etc/bash.bashrc
RUN apt autoremove -y