# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from launch.conditions import UnlessCondition


def generate_launch_description():

    # Config file
    config_file = os.path.join(
        get_package_share_directory('nvblox_examples_bringup'),
        'config', 'sensors', 'realsense.yaml')

    # Option to attach the nodes to a shared component container for speed ups through intra process communication.
    # Make sure to set the 'component_container_name' to the name of the component container you want to attach to.
    attach_to_shared_component_container_arg = LaunchConfiguration('attach_to_shared_component_container', default=False)
    component_container_name_arg = LaunchConfiguration('component_container_name', default='realsense_container')

    # If we do not attach to a shared component container we have to create our own container.
    realsense_container = Node(
        name=component_container_name_arg,
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        condition=UnlessCondition(attach_to_shared_component_container_arg)
    )

    load_composable_nodes = LoadComposableNodes(
        target_container=component_container_name_arg,
        composable_node_descriptions=[
            # RealSense splitter node
            ComposableNode(
                namespace="camera",
                name='realsense_splitter_node',
                package='realsense_splitter',
                plugin='nvblox::RealsenseSplitterNode',
                parameters=[{
                    'input_qos': 'SENSOR_DATA',
                    'output_qos': 'SENSOR_DATA'
                }],
                remappings=[('input/infra_1', '/camera/infra1/image_rect_raw'),
                            ('input/infra_1_metadata', '/camera/infra1/metadata'),
                            ('input/infra_2', '/camera/infra2/image_rect_raw'),
                            ('input/infra_2_metadata', '/camera/infra2/metadata'),
                            ('input/depth', '/camera/depth/image_rect_raw'),
                            ('input/depth_metadata', '/camera/depth/metadata'),
                            ('input/pointcloud', '/camera/depth/color/points'),
                            ('input/pointcloud_metadata',
                             '/camera/depth/metadata'),
                            ]),
            # Node Factory
            ComposableNode(
                namespace="camera",
                package='realsense2_camera',
                plugin='realsense2_camera::RealSenseNodeFactory',
                parameters=[config_file])])

    return LaunchDescription([realsense_container, load_composable_nodes])
