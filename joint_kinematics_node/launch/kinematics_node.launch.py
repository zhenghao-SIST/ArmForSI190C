#!/usr/bin/env python
#-*- coding: utf-8 -*-
# Author: Zhenghao Li
# Email: lizhenghao@shanghaitech.edu.cn
# Institute: SIST
# Date: 2025-04-29

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os
params=[os.path.join(get_package_share_directory('joint_kinematics_node'), 'config', 'dh_params.yaml')]


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joint_kinematics_node',
            executable='joint_kinematics_node',
            name='kinematics_node',
            parameters=[params],
            output='screen'
        )
    ])

