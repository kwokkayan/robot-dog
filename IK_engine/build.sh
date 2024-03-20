rosdep update --include-eol-distros && \
rosdep install --from-paths src --ignore-src -r -y --rosdistro melodic
source /opt/ros/melodic/setup.bash && catkin_make
echo "source /IK_engine/devel/setup.bash" >> /etc/bash.bashrc
apt autoremove -y