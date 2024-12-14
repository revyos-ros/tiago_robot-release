# Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
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
from dataclasses import dataclass
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction, DeclareLaunchArgument, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration
from launch.actions import GroupAction
from launch import LaunchContext
from launch_pal.arg_utils import read_launch_argument
from launch_pal.arg_utils import LaunchArgumentsBase
from controller_manager.launch_utils import generate_load_controller_launch_description
from launch_pal.param_utils import parse_parametric_yaml
from tiago_description.launch_arguments import TiagoArgs


@dataclass(frozen=True)
class LaunchArguments(LaunchArgumentsBase):

    arm_motor_model: DeclareLaunchArgument = TiagoArgs.arm_motor_model
    end_effector: DeclareLaunchArgument = TiagoArgs.end_effector


def declare_actions(launch_description: LaunchDescription, launch_args: LaunchArguments):

    launch_description.add_action(OpaqueFunction(function=setup_gravity_controller_configuration))

    gravity_compensation_controller = GroupAction(
        [
            generate_load_controller_launch_description(
                controller_name="gravity_compensation_controller",
                controller_params_file=LaunchConfiguration("controller_config"),
                extra_spawner_args=["--inactive"],
            )
        ],
    )
    launch_description.add_action(gravity_compensation_controller)

    return


def setup_gravity_controller_configuration(context: LaunchContext):

    arm_motor_model = read_launch_argument('arm_motor_model', context)
    end_effector = read_launch_argument('end_effector', context)

    # To be removed when the gravity compensation controller is updated
    if (end_effector == "no-end-effector"):
        ee_tip_link = "arm_tool_link"
    elif (end_effector == "pal-gripper"):
        ee_tip_link = "gripper_link"
    elif (end_effector == "pal-hey5"):
        ee_tip_link = "hand_palm_link"
    elif (end_effector in ["robotiq-2f-85", "robotiq-2f-140"]):
        ee_tip_link = "gripper_base_link"
    else:
        ee_tip_link = "arm_tool_link"
    remappings = {"EE_TIP_LINK": ee_tip_link}

    param_file = os.path.join(get_package_share_directory(
        'tiago_controller_configuration'), "config", "gravity_compensation_controller_" +
        arm_motor_model + ".yaml")

    parsed_yaml = parse_parametric_yaml(source_files=[param_file], param_rewrites=remappings)

    return [SetLaunchConfiguration('controller_config', parsed_yaml)]


def generate_launch_description():

    # Create the launch description and populate
    ld = LaunchDescription()
    launch_arguments = LaunchArguments()

    launch_arguments.add_to_launch_description(ld)

    declare_actions(ld, launch_arguments)

    return ld
