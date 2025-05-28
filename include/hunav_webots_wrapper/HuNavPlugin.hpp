#ifndef WEBOTS_ROS2_PLUGIN_EXAMPLE_HPP
#define WEBOTS_ROS2_PLUGIN_EXAMPLE_HPP

// C++
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <vector>
#include <cstdio>


//Webots
#include "webots_ros2_driver/PluginInterface.hpp"
#include "webots_ros2_driver/WebotsNode.hpp"

// ROS
#include "hunav_msgs/msg/agent.hpp"
#include "hunav_msgs/msg/agents.hpp"
#include "hunav_msgs/srv/compute_agents.hpp"
#include "hunav_msgs/srv/compute_agent.hpp"
#include "hunav_msgs/srv/move_agent.hpp"
#include "hunav_msgs/srv/get_agents.hpp"
#include "hunav_msgs/srv/reset_agents.hpp"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "geometry_msgs/msg/twist.hpp"

namespace hunav_plugin {

class HuNavPluginPrivate;

class HuNavPlugin : public webots_ros2_driver::PluginInterface {
public:

  /// \brief Constructor
  HuNavPlugin();

  /// \brief Destructor
  virtual ~HuNavPlugin();

  void step() override;
  void init(webots_ros2_driver::WebotsNode *node,
            std::unordered_map<std::string, std::string> &parameters) override;

private:
  std::unique_ptr<HuNavPluginPrivate> hnav_;

};
} // namespace hunav_plugin
#endif