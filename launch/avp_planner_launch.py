import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import yaml


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "RIN",
        default_value="SRP1",
        description="Robot identification number RIN to choose between setups -- possible values: [SRP1, IQ159-1]",
        )
    )
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])


def launch_setup(context, *args, **kwargs):

    # load config file
    motion_config_path = os.path.join(
        get_package_share_directory('avp_planner'),
        'config',
        f'moveit_py.yaml'
    )

    with open(motion_config_path, 'r') as config_file:
        motion_config = yaml.safe_load(config_file)

    # Initialize arguments
    ros2_control_hardware_type = motion_config['ros2_control_hardware_type']
    robot_ip = motion_config['robot_ip']
    reverse_ip = motion_config['reverse_ip']
    ur_type = motion_config['ur_type']

    # Define file paths for RTDE input and output recipes
    # Will be forwared to urXX.urdf.xacro, need to be set here for the robot to work
    input_recipe_filename = PathJoinSubstitution(
        [FindPackageShare("ur_robot_driver"), "resources", "rtde_input_recipe.txt"]
    )

    output_recipe_filename = PathJoinSubstitution(
        [FindPackageShare("ur_robot_driver"), "resources", "rtde_output_recipe.txt"]
    )

    # Set UR controller for motion execution
    moveit_controllers = 'joint_trajectory_controller.yaml'

    # Build MoveIt configuration for the UR robot, used for MoveIt and controlling the robot
    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur10e", package_name="moveit_config_ur")
        .robot_description(
            file_path=f"urdf/{ur_type}.urdf.xacro",
            mappings={
                "ros2_control_hardware_type": ros2_control_hardware_type,
                "robot_ip": robot_ip,
                "input_recipe_filename": input_recipe_filename,
                "output_recipe_filename": output_recipe_filename,
                "reverse_ip": reverse_ip,
            },
        )
        .joint_limits(file_path=f"config/{ur_type}/joint_limits_{motion_config['motion_core_node']['pose_config']}.yaml")
        .robot_description_semantic(file_path=f"srdf/{ur_type}.srdf.xacro")
        .trajectory_execution(file_path=f"config/{moveit_controllers}")
        .moveit_cpp(file_path="config/motion_planning.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # Start motion and environment nodes
    avp_planner_node = Node(
        package="avp_planner",
        executable="plan_publisher",
        name="avp_planner",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            motion_config['motion_core_node'],
            {"use_sim_time": True},
            {"use_fake_hardware":True},
        ]
    )

    nodes_to_start = [TimerAction(
        period=1.0,
        actions=[
        avp_planner_node
        ]
    )
    ]

    return nodes_to_start


# # Copyright (c) 2021 PickNik, Inc.
# #
# # Redistribution and use in source and binary forms, with or without
# # modification, are permitted provided that the following conditions are met:
# #
# #    * Redistributions of source code must retain the above copyright
# #      notice, this list of conditions and the following disclaimer.
# #
# #    * Redistributions in binary form must reproduce the above copyright
# #      notice, this list of conditions and the following disclaimer in the
# #      documentation and/or other materials provided with the distribution.
# #
# #    * Neither the name of the {copyright_holder} nor the names of its
# #      contributors may be used to endorse or promote products derived from
# #      this software without specific prior written permission.
# #
# # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# # AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# # IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# # ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# # LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# # CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# # SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# # INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# # CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# # ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# # POSSIBILITY OF SUCH DAMAGE.

# #
# # Author: Denis Stogl

# import os

# from pathlib import Path
# from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare
# from ur_moveit_config.launch_common import load_yaml

# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, OpaqueFunction
# from launch.conditions import IfCondition
# from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution


# def launch_setup(context, *args, **kwargs):
#     # Initialize Arguments
#     ur_type = LaunchConfiguration("ur_type")
#     use_mock_hardware = LaunchConfiguration("use_mock_hardware")
#     safety_limits = LaunchConfiguration("safety_limits")
#     safety_pos_margin = LaunchConfiguration("safety_pos_margin")
#     safety_k_position = LaunchConfiguration("safety_k_position")
#     # General arguments
#     description_package = LaunchConfiguration("description_package")
#     description_file = LaunchConfiguration("description_file")
#     moveit_config_package = LaunchConfiguration("moveit_config_package")
#     moveit_joint_limits_file = LaunchConfiguration("moveit_joint_limits_file")
#     moveit_config_file = LaunchConfiguration("moveit_config_file")
#     warehouse_sqlite_path = LaunchConfiguration("warehouse_sqlite_path")
#     prefix = LaunchConfiguration("prefix")
#     use_sim_time = LaunchConfiguration("use_sim_time")
#     launch_rviz = LaunchConfiguration("launch_rviz")
#     launch_servo = LaunchConfiguration("launch_servo")

#     joint_limit_params = PathJoinSubstitution(
#         [FindPackageShare(description_package), "config", ur_type, "joint_limits.yaml"]
#     )
#     kinematics_params = PathJoinSubstitution(
#         [FindPackageShare(description_package), "config", ur_type, "default_kinematics.yaml"]
#     )
#     physical_params = PathJoinSubstitution(
#         [FindPackageShare(description_package), "config", ur_type, "physical_parameters.yaml"]
#     )
#     visual_params = PathJoinSubstitution(
#         [FindPackageShare(description_package), "config", ur_type, "visual_parameters.yaml"]
#     )

#     robot_description_content = Command(
#         [
#             PathJoinSubstitution([FindExecutable(name="xacro")]),
#             " ",
#             PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
#             " ",
#             "robot_ip:=xxx.yyy.zzz.www",
#             " ",
#             "joint_limit_params:=",
#             joint_limit_params,
#             " ",
#             "kinematics_params:=",
#             kinematics_params,
#             " ",
#             "physical_params:=",
#             physical_params,
#             " ",
#             "visual_params:=",
#             visual_params,
#             " ",
#             "safety_limits:=",
#             safety_limits,
#             " ",
#             "safety_pos_margin:=",
#             safety_pos_margin,
#             " ",
#             "safety_k_position:=",
#             safety_k_position,
#             " ",
#             "name:=",
#             "ur",
#             " ",
#             "ur_type:=",
#             ur_type,
#             " ",
#             "script_filename:=ros_control.urscript",
#             " ",
#             "input_recipe_filename:=rtde_input_recipe.txt",
#             " ",
#             "output_recipe_filename:=rtde_output_recipe.txt",
#             " ",
#             "prefix:=",
#             prefix,
#             " ",
#         ]
#     )
#     robot_description = {"robot_description": robot_description_content}

#     # MoveIt Configuration
#     robot_description_semantic_content = Command(
#         [
#             PathJoinSubstitution([FindExecutable(name="xacro")]),
#             " ",
#             PathJoinSubstitution(
#                 [FindPackageShare(moveit_config_package), "srdf", moveit_config_file]
#             ),
#             " ",
#             "name:=",
#             # Also ur_type parameter could be used but then the planning group names in yaml
#             # configs has to be updated!
#             "ur",
#             " ",
#             "prefix:=",
#             prefix,
#             " ",
#         ]
#     )
#     robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}

#     robot_description_kinematics = PathJoinSubstitution(
#         [FindPackageShare(moveit_config_package), "config", "kinematics.yaml"]
#     )

#     robot_description_planning = {
#         "robot_description_planning": load_yaml(
#             str(moveit_config_package.perform(context)),
#             os.path.join("config", str(moveit_joint_limits_file.perform(context))),
#         )
#     }

#     # Planning Configuration
#     ompl_planning_pipeline_config = {
#         "move_group": {
#             "planning_plugin": "ompl_interface/OMPLPlanner",
#             "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
#             "start_state_max_bounds_error": 0.1,
#         }
#     }
#     ompl_planning_yaml = load_yaml("ur_moveit_config", "config/ompl_planning.yaml")
#     ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

#     # Trajectory Execution Configuration
#     controllers_yaml = load_yaml("ur_moveit_config", "config/controllers.yaml")
#     # the scaled_joint_trajectory_controller does not work on fake hardware
#     change_controllers = context.perform_substitution(use_mock_hardware)
#     if change_controllers == "true":
#         controllers_yaml["scaled_joint_trajectory_controller"]["default"] = False
#         controllers_yaml["joint_trajectory_controller"]["default"] = True

#     moveit_controllers = {
#         "moveit_simple_controller_manager": controllers_yaml,
#         "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
#     }

#     trajectory_execution = {
#         "moveit_manage_controllers": True,
#         "trajectory_execution.allowed_execution_duration_scaling": 1.2,
#         "trajectory_execution.allowed_goal_duration_margin": 0.5,
#         "trajectory_execution.allowed_start_tolerance": 0.01,
#     }

#     planning_scene_monitor_parameters = {
#         "publish_planning_scene": True,
#         "publish_geometry_updates": True,
#         "publish_state_updates": True,
#         "publish_transforms_updates": True,
#     }

#     warehouse_ros_config = {
#         "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
#         "warehouse_host": warehouse_sqlite_path,
#     }

#     # Start the actual move_group node/action server
#     move_group_node = Node(
#         package="moveit_ros_move_group",
#         executable="move_group",
#         output="screen",
#         parameters=[
#             robot_description,
#             robot_description_semantic,
#             robot_description_kinematics,
#             robot_description_planning,
#             ompl_planning_pipeline_config,
#             trajectory_execution,
#             moveit_controllers,
#             planning_scene_monitor_parameters,
#             {"use_sim_time": use_sim_time},
#             warehouse_ros_config,

#             #Publish the descriptions for moveitpy
#             {"publish_robot_description": True},
#             {"publish_robot_description_semantic": True},
#         ],
#     )

#     # rviz with moveit configuration
#     rviz_config_file = PathJoinSubstitution(
#         [FindPackageShare(moveit_config_package), "rviz", "view_robot.rviz"]
#     )
#     rviz_node = Node(
#         package="rviz2",
#         condition=IfCondition(launch_rviz),
#         executable="rviz2",
#         name="rviz2_moveit",
#         output="log",
#         arguments=["-d", rviz_config_file],
#         parameters=[
#             robot_description,
#             robot_description_semantic,
#             ompl_planning_pipeline_config,
#             robot_description_kinematics,
#             robot_description_planning,
#             warehouse_ros_config,
#         ],
#     )

#     # Servo node for realtime control
#     servo_yaml = load_yaml("ur_moveit_config", "config/ur_servo.yaml")
#     servo_params = {"moveit_servo": servo_yaml}
#     servo_node = Node(
#         package="moveit_servo",
#         condition=IfCondition(launch_servo),
#         executable="servo_node_main",
#         parameters=[
#             servo_params,
#             robot_description,
#             robot_description_semantic,
#             robot_description_kinematics,
#         ],
#         output="screen",
#     )

#     # Start setting up the required parameters for the moveit_py node 
#     moveit_py_yaml = load_yaml("moveit2_tutorials", "config/motion_planning_python_api_tutorial.yaml")
#     ur_ompl_planning_pipeline_config = {
#         "ompl": {
#             "planning_plugin": "ompl_interface/OMPLPlanner",
#             "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
#             "start_state_max_bounds_error": 0.1, 
#             "planner_id": "RRTConnectkConfigDefault"
#         }
#     }    
#     ur_ompl_planning_pipeline_config["ompl"].update(ompl_planning_yaml)

#     # Not tested with CHOMP and Pilz, just added this to be able to use the original moveit2_tutorials yaml file.
#     ur_chomp_planning_pipeline_config = {
#         "chomp": {
#             "planning_plugin": "chomp_interface/CHOMPPlanner",
#             "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints",
#             "start_state_max_bounds_error": 0.1,
#         }
#     }

#     ur_pilz_planning_pipeline_config = {
#         "pilz_industrial_motion_planner": {
#             "planning_plugin": "pilz_industrial_motion_planner/CommandPlanner",
#             "request_adapters": "default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints",
#             "default_planner_config": "PTP",
#             "planner_id": "PTP",
#         }
#     }

#     pilz_cartesian_limits_yaml = load_yaml("moveit2_tutorials", "config/pilz_cartesian_limits.yaml")
#     robot_description_planning["robot_description_planning"].update(pilz_cartesian_limits_yaml)
#     moveit_py_yaml["plan_request_params"].update({"planner_id": "RRTConnectkConfigDefault", "planning_time": 1.0, "planning_pipeline": "ompl"})
#     # To initialize the planning with Pilz comment upper line and uncomment following line:
#     # moveit_py_yaml["plan_request_params"].update({"planner_id": "PTP", "planning_time": 1.0, "planning_pipeline": "pilz_industrial_motion_planner"})

#     moveit_args_not_concatenated = [
#         moveit_py_yaml,
#         robot_description,
#         robot_description_semantic,
#         {"robot_description_kinematics": load_yaml("moveit2_tutorials", "config/ur_kinematics.yaml")},
#         ur_ompl_planning_pipeline_config,
#         ur_chomp_planning_pipeline_config,
#         ur_pilz_planning_pipeline_config,
#         robot_description_planning,
#         moveit_controllers,
#         planning_scene_monitor_parameters,
#         trajectory_execution,
#         {
#             "publish_robot_description": True,
#             "publish_robot_description_semantic": True,
#             "publish_geometry_updates": True,
#             "publish_state_updates": True,
#             "publish_transforms_updates": True,
#         },
#     ]

#     # Concatenate all dictionaries together, else moveitpy won't read all parameters
#     moveit_args = dict()
#     for d in moveit_args_not_concatenated:
#         moveit_args.update(d)

#     moveit_py_node = Node(
#         name="moveit_py",
#         package="avp_planner",
#         executable="avp_planner_member_function.py",
#         output="both",
#         parameters=[moveit_args]        
#     )

#     nodes_to_start = [move_group_node, rviz_node, servo_node, moveit_py_node]

#     return nodes_to_start


# def generate_launch_description():
#     declared_arguments = []
#     # UR specific arguments
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "ur_type",
#             description="Type/series of used UR robot.",
#             default_value="ur10e",
#             choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e"],
#         )
#     )
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "use_mock_hardware",
#             default_value="false",
#             description="Indicate whether robot is running with mock hardware mirroring command to its states.",
#         )
#     )
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "safety_limits",
#             default_value="true",
#             description="Enables the safety limits controller if true.",
#         )
#     )
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "safety_pos_margin",
#             default_value="0.15",
#             description="The margin to lower and upper limits in the safety controller.",
#         )
#     )
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "safety_k_position",
#             default_value="20",
#             description="k-position factor in the safety controller.",
#         )
#     )
#     # General arguments
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "description_package",
#             default_value="ur_description",
#             description="Description package with robot URDF/XACRO files. Usually the argument \
#         is not set, it enables use of a custom description.",
#         )
#     )
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "description_file",
#             default_value="ur.urdf.xacro",
#             description="URDF/XACRO description file with the robot.",
#         )
#     )
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "moveit_config_package",
#             default_value="ur_moveit_config",
#             description="MoveIt config package with robot SRDF/XACRO files. Usually the argument \
#         is not set, it enables use of a custom moveit config.",
#         )
#     )
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "moveit_config_file",
#             default_value="ur.srdf.xacro",
#             description="MoveIt SRDF/XACRO description file with the robot.",
#         )
#     )
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "moveit_joint_limits_file",
#             default_value="joint_limits.yaml",
#             description="MoveIt joint limits that augment or override the values from the URDF robot_description.",
#         )
#     )
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "warehouse_sqlite_path",
#             default_value=os.path.expanduser("~/.ros/warehouse_ros.sqlite"),
#             description="Path where the warehouse database should be stored",
#         )
#     )
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "use_sim_time",
#             default_value="false",
#             description="Make MoveIt to use simulation time. This is needed for the trajectory planing in simulation.",
#         )
#     )
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "prefix",
#             default_value='""',
#             description="Prefix of the joint names, useful for \
#         multi-robot setup. If changed than also joint names in the controllers' configuration \
#         have to be updated.",
#         )
#     )
#     declared_arguments.append(
#         DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
#     )
#     declared_arguments.append(
#         DeclareLaunchArgument("launch_servo", default_value="true", description="Launch Servo?")
#     )

#     return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])    