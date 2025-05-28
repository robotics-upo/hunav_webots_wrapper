#!/usr/bin/env python

# Copyright 1996-2023 Cyberbotics Ltd.
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

"""Launch Webots and the controller."""

import os
import launch
from launch.substitutions import LaunchConfiguration
from launch.actions import (IncludeLaunchDescription, SetEnvironmentVariable, 
                            DeclareLaunchArgument, ExecuteProcess, Shutdown, 
                            RegisterEventHandler, TimerAction, LogInfo)
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import IncludeLaunchDescription
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection
from launch.substitutions import TextSubstitution, PythonExpression



def get_robot_nodes(*args):

    package_dir = get_package_share_directory('hunav_webots_wrapper')

    use_rviz = LaunchConfiguration('rviz', default=True)
    use_nav = LaunchConfiguration('navigation', default=True)
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    # ROS control spawners
    controller_manager_timeout = ['--controller-manager-timeout', '500']
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''
    diffdrive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['diffdrive_controller'] + controller_manager_timeout,
    )
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['joint_state_broadcaster'] + controller_manager_timeout,
    )
    ros_control_spawners = [diffdrive_controller_spawner, joint_state_broadcaster_spawner]

    robot_description_path = os.path.join(package_dir, 'resource', 'tiago_webots.urdf')
    ros2_control_params = os.path.join(package_dir, 'resource', 'ros2_control.yml')
    use_twist_stamped = 'ROS_DISTRO' in os.environ and (os.environ['ROS_DISTRO'] in ['rolling', 'jazzy'])
    if use_twist_stamped:
        mappings = [('/diffdrive_controller/cmd_vel', '/cmd_vel'), ('/diffdrive_controller/odom', '/odom')]
    else:
        mappings = [('/diffdrive_controller/cmd_vel_unstamped', '/cmd_vel'), ('/diffdrive_controller/odom', '/odom')]
    tiago_driver = WebotsController(
        robot_name='Tiago_Lite',
        parameters=[
            {'robot_description': robot_description_path,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
            ros2_control_params
        ],
        remappings=mappings,
        respawn=True
    )

    # RViz
    rviz_config = os.path.join(get_package_share_directory('webots_ros2_tiago'), 'resource', 'default.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['--display-config=' + rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=launch.conditions.IfCondition(use_rviz)
    )

    # Navigation
    navigation_nodes = []
    nav2_params_file = 'nav2_params.yaml'
    wrapper_dir = get_package_share_directory('hunav_webots_wrapper') 
    my_map = os.path.join(wrapper_dir, 'resource', 'maps', 'office_map.yaml')
    my_params = os.path.join(wrapper_dir, 'resource', nav2_params_file)

    if 'nav2_bringup' in get_packages_with_prefixes():
        navigation_nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')),
            launch_arguments=[
                ('map',my_map),
                ('params_file', my_params),
                ('use_sim_time', use_sim_time),
            ],
            condition=launch.conditions.IfCondition(use_nav)))

     # Wait for the simulation to be ready to start RViz, the navigation and spawners
    waiting_nodes = WaitForControllerConnection(
        target_driver=tiago_driver,
        nodes_to_start=[rviz] + navigation_nodes + ros_control_spawners
    )

    return [
        tiago_driver,
        waiting_nodes,
    ]


def generate_launch_description():
    
    package_dir = get_package_share_directory('hunav_webots_wrapper')

    # World generation parameters
    world_file_name = LaunchConfiguration('world')
    urdf_folder_name = LaunchConfiguration('urdf_folder')
    robot_name = LaunchConfiguration('robot_name')
    global_frame = LaunchConfiguration('global_frame_to_publish')
    use_navgoal = LaunchConfiguration('use_navgoal_to_start')
    navgoal_topic = LaunchConfiguration('navgoal_topic')
    ignore_models = LaunchConfiguration('ignore_models')
    navigation = LaunchConfiguration('navigation')

    # Robot parameters
    namespace = LaunchConfiguration('robot_namespace')

     # agent configuration file
    agent_conf_file = PathJoinSubstitution([
        FindPackageShare('hunav_webots_wrapper'),
        'config',
        LaunchConfiguration('configuration_file')
    ])

    # Read the yaml file and load the parameters
    hunav_loader_node = Node(
        package='hunav_agent_manager',
        executable='hunav_loader',
        output='screen',
        parameters=[agent_conf_file]
        #arguments=['--ros-args', '--params-file', conf_file]
    )

    # urdf folder path
    resource_folder = PathJoinSubstitution([
        FindPackageShare('hunav_webots_wrapper'),
        urdf_folder_name   
        ])
    
    # the node looks for the base_world file in the directory 'worlds'
    # of the package hunav_gazebo_plugin direclty. So we do not need to 
    # indicate the path
    hunav_webots_worldgen_node = Node(
        package='hunav_webots_wrapper',
        executable='hunav_webots_world_generator',
        output='screen',
        parameters=[{'base_world': world_file_name},
        {'urdf_path' : resource_folder},
        {'robot_name': robot_name},
        {'global_frame_to_publish': global_frame},
        {'use_navgoal_to_start': use_navgoal},
        {'navgoal_topic': navgoal_topic},
        {'ignore_models': ignore_models}]
        #arguments=['--ros-args', '--params-file', conf_file]
    )

    ordered_launch_event = RegisterEventHandler(
        OnProcessStart(
            target_action=hunav_loader_node,
            on_start=[
                LogInfo(msg='HunNavLoader started, launching HuNav_Gazebo_world_generator after 2 seconds...'),
                TimerAction(
                    period=2.0,
                    actions=[hunav_webots_worldgen_node],
                )
            ]
        )
    )

    hunav_manager_node = Node(
        package='hunav_agent_manager',
        executable='hunav_agent_manager',
        name='hunav_agent_manager',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )


    metrics_file = PathJoinSubstitution([
    FindPackageShare('hunav_evaluator'),
    'config',
    LaunchConfiguration('metrics_file')
    ])

    # hunav_evaluator node
    hunav_evaluator_node = Node(
        package='hunav_evaluator',
        executable='hunav_evaluator_node',
        output='screen',
        parameters=[metrics_file]
    )
    
    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', 'temp', world_file_name]),
        ros2_supervisor=True
    )

    webots_launch_event = RegisterEventHandler(
        OnProcessStart(
            target_action=hunav_webots_worldgen_node,
            on_start=[
                LogInfo(msg='GenerateWorld started, launching Webots after 2 seconds...'),
                TimerAction(
                    period=2.0,
                    actions=[webots,webots._supervisor]
                )
            ]
        )
    )


    static_tf_node = Node(
        package = "tf2_ros", 
        executable = "static_transform_publisher",
        output='screen',
        arguments = ['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        condition=UnlessCondition(navigation)
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': '<robot name=""><link name=""/></robot>'
        }],
    )

    footprint_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
    )


    #Param declaration
    declare_agents_conf_file = DeclareLaunchArgument(
        'configuration_file', default_value='agents_office.yaml',
        description='Specify configuration file name in the cofig directory'
    )
    declare_metrics_conf_file = DeclareLaunchArgument(
        'metrics_file', default_value='metrics.yaml',
        description='Specify the name of the metrics configuration file in the cofig directory'
    )
    declare_arg_world = DeclareLaunchArgument(
        'world', default_value='office.wbt',
        description='Specify world file name'
    )

    declare_urdf_folder = DeclareLaunchArgument(
        'urdf_folder', default_value='resource',
        description='Specify urdf folder name'
    )

    declare_robot_name = DeclareLaunchArgument(
        'robot_name', default_value='Tiago_Lite',
        description='Specify the name of the robot model'
    )
    declare_frame_to_publish = DeclareLaunchArgument(
        'global_frame_to_publish', default_value='map',
        description='Name of the global frame in which the position of the agents are provided'
    )
    declare_use_navgoal = DeclareLaunchArgument(
        'use_navgoal_to_start', default_value='False',
        description='Whether to start the agents movements when a navigation goal is received or not'
    )
    declare_navgoal_topic = DeclareLaunchArgument(
        'navgoal_topic', default_value='goal_pose',
        description='Name of the topic in which navigation goal for the robot will be published'
    )
    declare_navigation = DeclareLaunchArgument(
        'navigation', default_value='True',
        description='If launch the Tiago navigation system'
    )
    declare_ignore_models = DeclareLaunchArgument(
        'ignore_models', default_value='floor(1),floor,manhole,manhole(0),manhole(1),manhole(2)',
        description='list of Gazebo models that the agents should ignore as obstacles as the ground_plane. Indicate the models with a blank space between them'
    )
    declare_arg_verbose = DeclareLaunchArgument(
        'verbose', default_value='False',
        description='Set "true" to increase messages written to terminal.'
    )
    declare_arg_namespace = DeclareLaunchArgument('robot_namespace', default_value='',
            description='The type of robot')
    
    ld = LaunchDescription()

    # Declare the launch arguments
    ld.add_action(declare_agents_conf_file)
    ld.add_action(declare_metrics_conf_file)
    ld.add_action(declare_arg_world)
    ld.add_action(declare_urdf_folder)
    ld.add_action(declare_robot_name)
    ld.add_action(declare_frame_to_publish)
    ld.add_action(declare_use_navgoal)
    ld.add_action(declare_navgoal_topic)
    ld.add_action(declare_ignore_models)
    ld.add_action(declare_navigation)
    ld.add_action(declare_arg_verbose)
    ld.add_action(declare_arg_namespace)

    # Generate the world with the agents
    # launch hunav_loader and the WorldGenerator a few seconds later
    ld.add_action(hunav_loader_node)
    ld.add_action(ordered_launch_event)

    # hunav behavior manager node
    ld.add_action(hunav_manager_node)
    # hunav evaluator
    ld.add_action(hunav_evaluator_node)

    ld.add_action(static_tf_node)

    # simulator launch event
    ld.add_action(webots_launch_event)
    
    ld.add_action(robot_state_publisher)
    ld.add_action(footprint_publisher)

    #pull out the robot nodes and add them
    tiago_driver, waiting_nodes = get_robot_nodes()
    ld.add_action(tiago_driver)
    ld.add_action(waiting_nodes)

    # This event handler respawns the robot nodes on simulation reset (supervisor process ends).
    reset_handler = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=webots._supervisor,
            on_exit=get_robot_nodes
        )
    )

    # If the folder does not exist, log an error.
    resource_folder = os.path.join(package_dir, "resource")

    if not os.path.isdir(resource_folder):
        print("Resource folder not found:", resource_folder)
    
    # List all URDF files in that folder.
    urdf_files = [f for f in os.listdir(resource_folder) if f.endswith('.urdf')]

    # Create a list to hold all WebotsController actions.
    agent_controllers = []
    for urdf_file in urdf_files:
        # Derive the robot name by stripping the .urdf extension.
        agent_name = os.path.splitext(urdf_file)[0]
        urdf_full_path = os.path.join(resource_folder, urdf_file)
        
        # Create a WebotsController node for this robot.
        controller = WebotsController(
            robot_name=agent_name,
            parameters=[{'robot_description': urdf_full_path}],
            respawn=True
        )
        agent_controllers.append(controller)

    #Launch the agents controllers
    agents_launch_event = RegisterEventHandler(
        OnProcessStart(
            target_action=tiago_driver,
            on_start=[
                LogInfo(msg='Robot ready, launching agent controllers...'),
                TimerAction(
                    period=5.0,
                    actions=agent_controllers
                )
            ]
        )
    )
    
    ld.add_action(agents_launch_event)

    ld.add_action(# This action will kill all nodes once the Webots simulation has exited
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[
                    launch.actions.UnregisterEventHandler(
                        event_handler=reset_handler.event_handler
                    ),
                    launch.actions.EmitEvent(event=launch.events.Shutdown())
                ],
            )
        )
    )
    ld.add_action(reset_handler)
    
    return ld