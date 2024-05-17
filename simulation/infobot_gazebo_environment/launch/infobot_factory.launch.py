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
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    launch_file_dir = os.path.join(
        get_package_share_directory("infobot_gazebo_environment"), "launch"
    )
    os.environ["GAZEBO_MODEL_PATH"] = (
        os.environ["GAZEBO_MODEL_PATH"]
        + os.path.join(
            get_package_share_directory("infobot_gazebo_environment"), "models"
        )
        + ":"
        + os.path.join(get_package_share_directory("infobot_agent"), "models")
    )
    print(os.environ["GAZEBO_MODEL_PATH"])
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    world = os.path.join(
        get_package_share_directory("infobot_gazebo_environment"),
        "worlds",
        "infobot_factory.world",
    )
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gzserver.launch.py")
        ),
        launch_arguments={
            "world": world,
            "use_sim_time": use_sim_time,
            "verbose": "true",
            "pause": "false",
        }.items(),
    )

    ld = LaunchDescription()
    ld.add_action(gzserver_cmd)

    try:
        value = os.environ["SIM_ENV"]  # "DEV" , "CICD" or empty
        print("SIM_ENV=" + value)
    except KeyError:
        print("SIM_ENV not found, set to DEV")
        os.environ["SIM_ENV"] = "DEV"

    if os.environ["SIM_ENV"] != "CICD":
        gzclient_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, "launch", "gzclient.launch.py")
            ),
            launch_arguments={"use_sim_time": use_sim_time, "verbose": "true"}.items(),
        )
        ld.add_action(gzclient_cmd)

    return ld
