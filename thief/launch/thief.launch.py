# Copyright 2024 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    thief_cmd = Node(
        package='thief',
        executable='thief',
        name='thief',
        output='screen',
        parameters=[
            {'use_sim_time': True}
        ])

    yolo_launch_file = os.path.join(
        FindPackageShare('yolo_bringup').find('yolo_bringup'), 'launch', 'yolo.launch.py')

    yolo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(yolo_launch_file),
        launch_arguments={
            'model': 'yolov8m-seg.pt',
            'input_image_topic': '/rgb/image_raw'
        }.items()
    )

    ld = LaunchDescription()
    ld.add_action(thief_cmd)
    ld.add_action(yolo_cmd)
    return ld
