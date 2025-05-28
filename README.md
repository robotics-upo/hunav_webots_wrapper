# hunav_webots_wrapper

**This version is still WIP**

This is a **ROS2 wrapper** to use [**HuNavSim**](https://github.com/robotics-upo/hunav_sim) with the **Webots** simulator (tested with ROS2 Humble and Webots R2025a). We provide three different simulation scenes (an office, a factory, and a hall) with different agent configurations.

![](media/factory_tiago1.png)

## Main components

This wrapper contains two ROS2 nodes:

* The *hunav_webots_world_generator* node reads configuration parameters from HuNavSim (*hunav_loader* module) and populates a base world (.wbt file) with the agents in the desired configuration. Three different agent configurations files (.yaml) are provided in the config folder.
* The *HuNavPlugin* node is the Webots plugin implementation that interacts with HuNavSim to read and control the movement of the human agents.


## Dependencies

* Ubuntu 22.04 LTS
* ROS2 Humble and Webots Simulator: please read the ROS2 Official [Documentation](https://docs.ros.org/en/humble/Installation.html) and the specific [Webots](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Installation-Ubuntu.html) section to set them up.
* The HuNavSim: https://github.com/robotics-upo/hunav_sim
* The ROS2 package *webots_ros2*: if you completed point 2 you should already have it installed. If not, please follow the instructions [here](https://github.com/cyberbotics/webots_ros2/wiki/Getting-Started) to install it.
* The Navigation 2 package, which is used to control the robot. You can install it with this command:
 ```sh 
    sudo apt install ros-$(ROS_DISTRO)-nav2-bringup
  ```

This package incorporates the [TIAGo](https://github.com/cyberbotics/webots_ros2/wiki/Example-TIAGo) robot by PAL Robotics in the simulation, which is included as part of the *webots_ros2* package.

To use the package, clone it to you ROS2 workspace and compile it with the standard colcon tool:

 ```sh     
    colcon build --packages-select hunav_webots_wrapper
  ```

## Plugin parameters

Plugin instances are generated automatically at launch and assigned to each one of the agents. This is done by creating a URDF file per agent that declares the plugin and allows the Webots ROS2 driver to launch it and connect to the specified target.  Here we provide a code snippet for one of the URDF files: 

```html
<?xml version="1.0" encoding="utf-8"?>
<robot name="agent1">
    <webots>
        <plugin type="hunav_plugin::HuNavPlugin">
            <agent_name>agent1</agent_name>
            <agent_id>1</agent_id>
            <skin_device_name>Robert</skin_device_name>
            <overseer_name>agent4</overseer_name>
            <robot_name>Tiago_Lite</robot_name>
            <global_frame_to_publish>map</global_frame_to_publish>
            <use_navgoal_to_start>false</use_navgoal_to_start>
            <navgoal_topic>goal_pose</navgoal_topic>
            <ignore_models>floor(1),floor,manhole,manhole(0),manhole(1),manhole(2)</ignore_models>
        </plugin>
    </webots>
</robot>
```

* ```agent_name```: the name of the agent that is assigned to this plugin instance.
* ```agent_id```: id of the agent assigned to the plugin instance.
* ```skin_device_name```: name of the skin assigned to this agent. Used to animate the skeletons of the agents.
* ```overseer_name```: name of the agent which is randomly assigned to update the state of all agents. This is because each agent must host a plugin instance, but only one of them is able to manage the updates. 
* ```robot_name```: name of the model corresponding to the robot that will be spawned.
* ```global_frame_ẗo_publish```: coordinate frame in which agent positions are provided.
* ```use_navgoal_to_start```: Boolean to indicate whether the plugin must wait for the robot to receive a navigation goal to start the movement of the human agents
* ```navgoal_topic```: Name  of the topic where the robot navigation goal is expected. The message must be type *geometry_msgs/msg/PoseStamped*. This parameter is used only if the parameter ```use_navgoal_to_start``` is True. 
* ```ignore_models```: List of the models than must be ignored by the agents for collision handling. These are usually the floor models and similar (manholes, carpets).


## Usage example

This package provides three different scenarios already set up with three different pre-configurations of human agents: 
`office`, `factory` and `hall`. You can find their respective configuration files in the <em>config</em> folder. Feel free to modify them to simulate different behaviors, number of agents, etc. For example, to launch the `factory` scenario, you can run in your terminal:

 ```sh 
    ros2 launch hunav_webots_wrapper hunavsim_launch_factory.py
  ```

This will cause Webots to launch the simulated environment with the specified agents. An rviz window will pop up too. You can send goals to the robot using the standard Nav2 Goal tool. 

## Troubleshooting

The simulator may take a while to load after a clean build, which can result in HuNavSim not properly loading the agents. If this happens, close the simulator with Ctrl+C and launch it again.
