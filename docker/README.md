# Configure the Jetson Orin Nano devkit

## Isaac ROS RealSense Setup

Clone repos

```
cd ${ISAAC_ROS_WS}/src
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
git clone https://github.com/IntelRealSense/realsense-ros.git -b 4.51.1
```

Configure dev container

```
cd ${ISAAC_ROS_WS}/src/isaac_ros_common/scripts && \
touch .isaac_ros_common-config && \
echo CONFIG_IMAGE_KEY=ros2_humble.realsense > .isaac_ros_common-config
```

## Nvblox with human reconstruction

In host, run

```
sudo sysctl -w net.core.rmem_max=8388608 net.core.rmem_default=8388608
``` 

Clone repo

```
git clone --recurse-submodules https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox.git && \
    cd isaac_ros_nvblox && git lfs pull
```

Otherwise, camera_link frame would not be found

```
cd ${ISAAC_ROS_WS}/src/isaac_ros_nvblox/nvblox_examples/realsense_splitter && \
  git update-index --assume-unchanged COLCON_IGNORE && \
  rm COLCON_IGNORE
```


In container,

```
cd /workspaces/isaac_ros-dev/ && \
    rosdep install -i -r --from-paths src --rosdistro humble -y --skip-keys "libopencv-dev libopencv-contrib-dev libopencv-imgproc-dev python-opencv python3-opencv nvblox"
```

Install package dependencies

```
sudo apt-get install -y ros-humble-isaac-ros-stereo-image-proc ros-humble-isaac-ros-visual-slam
```

### Human reconstruction

Clone the repo

```
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_segmentation.git
```

```
sudo apt-get install -y ros-humble-isaac-ros-unet ros-humble-isaac-ros-triton ros-humble-isaac-ros-dnn-image-encoder
```

Download the "PeopleSemSegNet ShuffleSeg" ETLT fil

```
mkdir -p /workspaces/isaac_ros-dev/models/peoplesemsegnet_shuffleseg/1 && \
  cd /workspaces/isaac_ros-dev/models/peoplesemsegnet_shuffleseg && \
  wget https://api.ngc.nvidia.com/v2/models/nvidia/tao/peoplesemsegnet/versions/deployable_shuffleseg_unet_v1.0/files/peoplesemsegnet_shuffleseg_etlt.etlt && \
  wget https://api.ngc.nvidia.com/v2/models/nvidia/tao/peoplesemsegnet/versions/deployable_shuffleseg_unet_v1.0/files/peoplesemsegnet_shuffleseg_cache.txt
```

Convert the ETLT file to TensorRT plan file

```
/opt/nvidia/tao/tao-converter -k tlt_encode -d 3,544,960 -p input_2:0,1x3x544x960,1x3x544x960,1x3x544x960 -t int8 -c peoplesemsegnet_shuffleseg_cache.txt -e /workspaces/isaac_ros-dev/models/peoplesemsegnet_shuffleseg/1/model.plan -o argmax_1 peoplesemsegnet_shuffleseg_etlt.etlt
```

Copy the sample Triton config file

```
cp /workspaces/isaac_ros-dev/src/isaac_ros_image_segmentation/resources/peoplesemsegnet_shuffleseg_config.pbtxt /workspaces/isaac_ros-dev/models/peoplesemsegnet_shuffleseg/config.pbtxt
```
