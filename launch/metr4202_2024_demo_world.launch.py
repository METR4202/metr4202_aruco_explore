#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
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
#
# Authors: Joep Tool

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, EnvironmentVariable


def generate_launch_description():
    package_dir = get_package_share_directory('metr4202_aruco_explore')
    launch_file_dir = os.path.join(package_dir, 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-1.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    resource_path = os.path.join(package_dir, 'resource')
    if  'GAZEBO_RESOURCE_PATH' in os.environ:
        resource_path = [EnvironmentVariable('GAZEBO_RESOURCE_PATH'), ':' + resource_path]
    else:
    	assert os.path.isdir('/usr/share/gazebo-11'), 'Gazebo not found, set GAZEBO_RESOURCE_PATH environment variable'
    	resource_path = ['/usr/share/gazebo-11:' + resource_path]

    model_path = os.path.join(package_dir, 'models')
    if  'GAZEBO_MODEL_PATH' in os.environ:
        model_path = [EnvironmentVariable('GAZEBO_MODEL_PATH') , ':' + model_path]

    print(f"Resourcepath: {resource_path}")

    world = os.path.join(
        get_package_share_directory('metr4202_aruco_explore'),
        'worlds',
        'metr4202_2024_final_demo.world'
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    ld = LaunchDescription([
        SetEnvironmentVariable(name='GAZEBO_RESOURCE_PATH', value=resource_path),
        SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=model_path),
        gzserver_cmd,
        gzclient_cmd,
        robot_state_publisher_cmd,
        spawn_turtlebot_cmd])

    return ld
