/***********************************************************************/
/**                                                                    */
/** HuNavPlugin.cpp                                                 */
/**                                                                    */
/** Copyright (c) 2022, Service Robotics Lab (SRL).                    */
/**                     http://robotics.upo.es                         */
/**                                                                    */
/** All rights reserved.                                               */
/**                                                                    */
/** Authors:                                                           */
/** Andrés Martínez Silva (maintainer)                                    */
/** email: amarsil1@upo.es                                             */
/**                                                                    */
/** This software may be modified and distributed under the terms      */
/** of the MIT license. See the LICENSE file for details.              */
/**                                                                    */
/**                                                                    */
/***********************************************************************/

#include "hunav_webots_wrapper/HuNavPlugin.hpp"

#include "rclcpp/rclcpp.hpp"
#include <cstdio>
#include <functional>
#include <memory>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/supervisor.h>  // for supervisor functions
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <webots/bvh_util.h>

#include <webots/skin.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

static constexpr double SURPRISED_YAW_OFFSET = M_PI_2;  // +90°

using namespace std::chrono_literals;

namespace hunav_plugin {


class HuNavPluginPrivate{

  public:

    webots_ros2_driver::WebotsNode* node_; // Pointer to the node

    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    /// \brief ros service to update the actor states
    rclcpp::Client<hunav_msgs::srv::ComputeAgents>::SharedPtr rosSrvComputeAgentsClient;

    /// \brief ros service to update an actor state
    rclcpp::Client<hunav_msgs::srv::ComputeAgent>::SharedPtr rosSrvComputeAgentClient;

    /// \brief ros service to get the initial agents setup
    rclcpp::Client<hunav_msgs::srv::GetAgents>::SharedPtr rosSrvGetAgentsClient;

    /// \brief ros service to reset the agents in the hunav_manager
    rclcpp::Client<hunav_msgs::srv::ResetAgents>::SharedPtr rosSrvResetClient;

    /// \brief ros service to move an agent
    rclcpp::Client<hunav_msgs::srv::MoveAgent>::SharedPtr rosSrvMoveAgentClient;

    /// \brief ros service to reset the agents in the hunav_manager
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub;

    /// \brief Helper function to collect the pedestrians data from Gazebo
    bool GetPedestrians(const double &dt);
    /// \brief Helper function to collect the robot data from Gazebo
    bool GetRobot(const double &dt);
    /// \brief Helper function to collect obtacles from Gazebo
    void HandleObstacles();

    bool Reset();

    void collectCollisionPrimitives(WbNodeRef node, std::vector<WbNodeRef>& out);
      /// Computes the closest point on a (“box‐aligned”) bounding volume in Webots.
    std::vector<Eigen::Vector3d> GetClosestPointOnBoundingBox(const Eigen::Vector3d& point, WbNodeRef modelNode);
    void UpdateWebotsPedestrians(const hunav_msgs::msg::Agents& _agents, const double &dt);

    /// \brief Helper function to initialize the agents at the initial step
    void InitializeAgents();
    /// \brief Helper function to initialize the robot the initial step
    bool InitializeRobot();
    inline double normalizeAngle(double a)
    {
      double value = a;
      while (value <= -M_PI)
        value += 2 * M_PI;
      while (value > M_PI)
        value -= 2 * M_PI;
      return value;
    }

    //Functions to handle animations
    void loadAnimations(const std::string& base_path,
                        const std::vector<std::string>& anim_names);

    std::string chooseAnimation(const hunav_msgs::msg::Agent& agent);
    void manageAnimations();

    rclcpp::Time rostime;
    double lastUpdate = wb_robot_get_time();
    double lastAgentUpdate = wb_robot_get_time();
    double startTime = wb_robot_get_time();
    bool computeAgentReady = false;
    bool reset;

    rclcpp::Time agents_request_start;

    /// \brief the robot as a agent msg
    hunav_msgs::msg::Agent robotAgent;
    hunav_msgs::msg::Agent init_robotAgent;
    /// \brief vector of pedestrians detected -used only by overseer.
    std::vector<hunav_msgs::msg::Agent> pedestrians, init_pedestrians;
    hunav_msgs::msg::Agent passiveAgent; //used for the rest of the agents

    // Last updated poses from HuNavSim
    hunav_msgs::msg::Agents last_service_agents;
    bool have_last_service = false;
    double lastOverseerUpdate = wb_robot_get_time();

    std::string robotName;
    std::string agentName;
    int agentId;
    std::string overseerName;
    std::string globalFrame;

    bool waitForGoal;
    bool goalReceived;
    std::string goalTopic;

    /// \brief List of models to ignore. Used for vector field
    std::vector<std::string> ignoreModels;

    // Reference to the robot node as a supervisor.
    WbNodeRef robot_node = NULL;

    geometry_msgs::msg::Pose new_goal;

    // This is to animate the agents
    std::string skinDeviceName = "Robert";
    WbDeviceTag skinDevice = 0;

    std::string currentAnim = "walk.bvh";
    std::unordered_map<std::string, WbuBvhMotion> motions;
    std::unordered_map<std::string, std::vector<int>> motionBoneMaps;

    std::shared_future<hunav_msgs::srv::ComputeAgent::Response::SharedPtr> agent_future;
    std::shared_future<hunav_msgs::srv::ComputeAgents::Response::SharedPtr> agents_future;

};


// /////////////////////////////////////////////////
HuNavPlugin::HuNavPlugin() : hnav_(std::make_unique<HuNavPluginPrivate>())
{
  RCLCPP_INFO(rclcpp::get_logger("HuNavPlugin"), "HuNavPlugin constructed – plugin loaded");
}

HuNavPlugin::~HuNavPlugin()
{
}


void HuNavPlugin::init(
    webots_ros2_driver::WebotsNode *node,
    std::unordered_map<std::string, std::string> &parameters) {

  hnav_->node_ = node;  // Store the node pointer for later spinning
  // --- Read parameters from unordered_map (instead of SDF elements) ---
  if (parameters.find("agent_name") != parameters.end()) {
    hnav_->agentName = parameters["agent_name"];
    RCLCPP_INFO(node->get_logger(), "Parameter robot_name: %s", hnav_->agentName.c_str());
  } else {
    hnav_->agentName = "default_agent";
    RCLCPP_INFO(node->get_logger(), "Parameter robot_name not provided, using default: default_agent");
  }

  // --- Read parameters from unordered_map (instead of SDF elements) ---
  if (parameters.find("agent_id") != parameters.end()) {
    hnav_->agentId = std::stoi(parameters["agent_id"]);
    RCLCPP_INFO(node->get_logger(), "Parameter robot_id: %d", hnav_->agentId);
  } else {
    hnav_->agentId = 0;
    RCLCPP_INFO(node->get_logger(), "Parameter agent_id not provided, using default: 0");
  }

  // --- Read parameters from unordered_map (instead of SDF elements) ---
  if (parameters.find("overseer_name") != parameters.end()) {
    hnav_->overseerName = parameters["overseer_name"];
    RCLCPP_INFO(node->get_logger(), "Parameter robot_name: %s", hnav_->overseerName.c_str());
  } else {
    hnav_->overseerName = "default_overseer";
    RCLCPP_INFO(node->get_logger(), "Parameter robot_name not provided, using default: default_agent");
  }

  if (parameters.find("skin_device_name") != parameters.end()) {
    hnav_->skinDeviceName = parameters["skin_device_name"];
    RCLCPP_INFO(node->get_logger(), "Parameter skin_device_name: %s", hnav_->skinDeviceName.c_str());
  } else {
    hnav_->skinDeviceName = "Robert";
    RCLCPP_INFO(node->get_logger(), "Parameter skin_device_name not provided, using default: Robert");
  }

  if (parameters.find("robot_name") != parameters.end()) {
    hnav_->robotName = parameters["robot_name"];
    RCLCPP_INFO(node->get_logger(), "Parameter robot_name: %s", hnav_->robotName.c_str());
  } else {
    hnav_->robotName = "default_robot";
    RCLCPP_INFO(node->get_logger(), "Parameter robot_name not provided, using default: default_agent");
  }

  if (parameters.find("global_frame_to_publish") != parameters.end()) {
    hnav_->globalFrame = parameters["global_frame_to_publish"];
    RCLCPP_INFO(node->get_logger(), "Parameter global_frame_to_publish: %s", hnav_->globalFrame.c_str());
  } else {
    hnav_->globalFrame = "map";
    RCLCPP_INFO(node->get_logger(), "Parameter global_frame_to_publish not provided, using default: map");
  }

  if (parameters.find("use_navgoal_to_start") != parameters.end()) {
    // Assuming parameter is either "true" or "false"
    hnav_->waitForGoal = (parameters["use_navgoal_to_start"] == "true");
    RCLCPP_INFO(node->get_logger(), "Parameter use_navgoal_to_start: %s", 
                hnav_->waitForGoal ? "true" : "false");
  } else {
    hnav_->waitForGoal = false;
    RCLCPP_INFO(node->get_logger(), "Parameter use_navgoal_to_start not provided, using default: false");
  }

  if (hnav_->waitForGoal) {
    hnav_->goalReceived = false;
    if (parameters.find("navgoal_topic") != parameters.end()) {
      hnav_->goalTopic = parameters["navgoal_topic"];
      RCLCPP_INFO(node->get_logger(), "Parameter navgoal_topic: %s", hnav_->goalTopic.c_str());
    } else {
      hnav_->goalTopic = "goal_pose";
      RCLCPP_INFO(node->get_logger(), "Parameter navgoal_topic not provided, using default: goal_pose");
    }
  } else {
    hnav_->goalReceived = true;
  }

  if (parameters.find("ignore_models") != parameters.end()) {
    // Assume ignore_models is given as comma-separated names, e.g., "model1,model2"
    std::string modelsStr = parameters["ignore_models"];
    RCLCPP_INFO(node->get_logger(), "Parameter ignore_models: %s", modelsStr.c_str());
    std::istringstream iss(modelsStr);
    std::string model;
    while (std::getline(iss, model, ',')) {
      // Optionally, remove whitespace from model
      hnav_->ignoreModels.push_back(model);
      RCLCPP_INFO(node->get_logger(), "Ignoring model: %s", model.c_str());
    }
  } else {
    RCLCPP_INFO(node->get_logger(), "No ignore_models parameter provided");
  }


  if (hnav_->waitForGoal)
  {
    hnav_->goal_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        hnav_->goalTopic, 1, std::bind(&HuNavPluginPrivate::goalCallback, hnav_.get(), std::placeholders::_1));
  }

  hnav_->rosSrvComputeAgentsClient = node->create_client<hunav_msgs::srv::ComputeAgents>("compute_agents");
  hnav_->rosSrvGetAgentsClient = node->create_client<hunav_msgs::srv::GetAgents>("get_agents");
  hnav_->rosSrvResetClient = node->create_client<hunav_msgs::srv::ResetAgents>("reset_agents");
  hnav_->rosSrvComputeAgentClient = node->create_client<hunav_msgs::srv::ComputeAgent>("compute_agent");
  hnav_->rosSrvMoveAgentClient = node->create_client<hunav_msgs::srv::MoveAgent>("move_agent");

  if(hnav_->agentName == hnav_->overseerName){
    hnav_->InitializeAgents();
    hnav_->HandleObstacles();
  }

  //initialize animations

  RCLCPP_INFO(hnav_->node_->get_logger(), "Loading animations for agent %s", hnav_->agentName.c_str());

   // 1) Load all available animations
  std::vector<std::string> animations = { "walk.bvh",           "69_02_walk_forward.bvh",   "137_28-normal_wait.bvh",
                                       "142_01-walk_childist.bvh", "07_04-slow_walk.bvh",      "02_01-walk.bvh",
                                       "142_17-walk_scared.bvh",   "17_01-walk_with_anger.bvh", "141_20-waiting.bvh", "141_16-wave_hello.bvh"
                                       "jump.bvh", "normal_wait_mod.bvh" };
  // if (parameters.find("motion_file") != parameters.end())
  //   motion_name = parameters["motion_file"];

  std::string pkg_share = ament_index_cpp::get_package_share_directory("hunav_webots_wrapper");
  std::string bvh_dir = pkg_share + "/resource/motions";
  hnav_->skinDevice = wb_robot_get_device(hnav_->skinDeviceName.c_str());
  hnav_->loadAnimations(bvh_dir, animations);

  hnav_->startTime = wb_robot_get_time();

}

void HuNavPluginPrivate::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  new_goal = msg->pose;
  RCLCPP_INFO(node_->get_logger(), "Received new goal");
  goalReceived = true;
}

bool HuNavPluginPrivate::InitializeRobot(){

  RCLCPP_INFO(this->node_->get_logger(), "Initializing robot...");
  WbNodeRef node = wb_supervisor_node_get_from_def(this->robotName.c_str());

  if (!node)
    {
      // Log error and return false if the node is not found.
      RCLCPP_ERROR(this->node_->get_logger(), "Robot model %s not found!!!!", this->robotName.c_str());
      return false;
    }

      // Obtain the translation field and read the current position.
  WbFieldRef trans_field = wb_supervisor_node_get_field(node, "translation");
  const double* newPos = wb_supervisor_field_get_sf_vec3f(trans_field);
  double xf = newPos[0]; 
  double yf = newPos[1];

  // Obtain the rotation field and read the current orientation.
  WbFieldRef rot_field = wb_supervisor_node_get_field(node, "rotation");
  const double* rot = wb_supervisor_field_get_sf_rotation(rot_field);
  // Webots returns the rotation as an axis-angle vector.
  // Assuming the rotation axis is (0, 1, 0) (i.e. around the vertical axis),

  // 2) Build a quaternion:
  tf2::Quaternion q;
  q.setRotation({ rot[0], rot[1], rot[2] }, rot[3]);

  // 3) Pull out just the yaw component in (–π…π]:
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  yaw = normalizeAngle(yaw);

  // 4) Write back a *pure* Z‐axis rotation by that yaw:
  const double newRot[4] = { 0.0, 0.0, 1.0, yaw };
  wb_supervisor_field_set_sf_rotation(rot_field, newRot);
  yaw = newRot[3];
  //double yaw = rot[3];
  //Get velocities from simulator
  const double *vel = wb_supervisor_node_get_velocity(node);
  // vel[0..2] = linear x,y,z; vel[3..5] = angular x,y,z
  double vx = vel[0];
  double vy = vel[1];
  double linearVelocity = std::hypot(vx, vy);
  double anvel = vel[5];  // yaw rate

  robotAgent.position.position.x = xf;
  robotAgent.position.position.y = yf;
  robotAgent.yaw = yaw;
  robotAgent.velocity.linear.x = vx;
  robotAgent.velocity.linear.y = vy;
  robotAgent.linear_vel = linearVelocity;
  robotAgent.velocity.angular.z = anvel;
  robotAgent.angular_vel = anvel;

  robotAgent.id = wb_supervisor_node_get_id(node);
  robotAgent.type = hunav_msgs::msg::Agent::ROBOT;
  robotAgent.behavior.type = hunav_msgs::msg::AgentBehavior::BEH_REGULAR;
  robotAgent.behavior.state = hunav_msgs::msg::AgentBehavior::BEH_NO_ACTIVE;
  robotAgent.name = robotName;
  robotAgent.group_id = -1;
  robotAgent.radius = 0.8;

  init_robotAgent = robotAgent;

  return true;

}

void HuNavPluginPrivate::InitializeAgents()
{
  RCLCPP_INFO(node_->get_logger(), "Initializing agents...");

  if(!InitializeRobot()){
      RCLCPP_WARN(node_->get_logger(), "Could not initialize robot!");
  }

  // Fill the service request
  auto request = std::make_shared<hunav_msgs::srv::GetAgents::Request>();
  request->empty = 0;

  // Wait for the service to be available
  while (!rosSrvGetAgentsClient->wait_for_service(1s))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(this->node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_WARN(this->node_->get_logger(), "service /get_agents not available, waiting again...");
  }

  // Call the service
  auto result = rosSrvGetAgentsClient->async_send_request(request);

  // Spin the node until the future is complete or a timeout occurs.
  rclcpp::FutureReturnCode status =
    rclcpp::spin_until_future_complete(node_->get_node_base_interface(), result, std::chrono::milliseconds(200));

  if (status == rclcpp::FutureReturnCode::SUCCESS)
  {
    // Initialize the actors
    // const hunav_msgs::msg::Agents &agents = result.get()->agents;
    auto res = *result.get();
    const hunav_msgs::msg::Agents agents = res.agents;

    RCLCPP_INFO(node_->get_logger(), "Received %i agents from service /get_agents", (int)agents.agents.size());
    this->pedestrians.clear();

    for (auto agent : agents.agents)
    {
      hunav_msgs::msg::Agent ag = agent;

      // Attempt to obtain the supervisor node for this agent using its name.
      WbNodeRef agent_node = wb_supervisor_node_get_from_def(agent.name.c_str());
      if (agent_node == NULL)
      {
        RCLCPP_ERROR(this->node_->get_logger(), "Supervisor node for agent '%s' not found!", ag.name.c_str());
        continue;
      }

      // Get translation field
      WbFieldRef trans_field = wb_supervisor_node_get_field(agent_node, "translation");
      // Get the rotation field; it expects an axis–angle [axis_x, axis_y, axis_z, angle].
      WbFieldRef rot_field = wb_supervisor_node_get_field(agent_node, "rotation");

      // --- Update Translation Field ---
      const double newPos[3] = { 
        agent.position.position.x, 
        agent.position.position.y, 
        agent.position.position.z, 
      };
      
      if(this->agentName == this->overseerName) wb_supervisor_field_set_sf_vec3f(trans_field, newPos);

      // --- Update Rotation Field ---
      
      tf2::Quaternion q;
      tf2::fromMsg(ag.position.orientation, q);

      // 2) Extract yaw in (–π…π]:
      double roll, pitch, yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
      yaw = normalizeAngle(yaw);

      // 4) Push that to Webots around Z:
      const double newRot[4] = { 0.0, 0.0, 1.0, yaw };

      if(this->agentName == this->overseerName) wb_supervisor_field_set_sf_rotation(rot_field, newRot);

      //Update the values to build the pedestrian vector
      ag.position.position.x = newPos[0];
      ag.position.position.y = newPos[1];
      // rebuild quaternion from axis‐angle
      tf2::Quaternion q2;
      tf2::Vector3 axis2(newRot[0], newRot[1], newRot[2]);
      q2.setRotation(axis2, newRot[3]);
      ag.position.orientation = tf2::toMsg(q2);
      ag.yaw = yaw;

      //Obtain velocities and update them
      const double *vel = wb_supervisor_node_get_velocity(agent_node);
      ag.desired_velocity = agent.desired_velocity;
      ag.velocity.linear.x = vel[0];
      ag.velocity.linear.y = vel[1];
      ag.linear_vel = std::hypot(ag.velocity.linear.x, ag.velocity.linear.y);
      ag.velocity.angular.z = vel[5];
      ag.angular_vel = std::fabs(ag.velocity.angular.z);

      // Log the updated fields for this agent.
      RCLCPP_INFO(node_->get_logger(),
                  "Updated agent: %s | pos: (%.2f, %.2f, %.2f) | rot: (%.2f, %.2f, %.2f, %.2f)",
                  ag.name.c_str(), 
                  newPos[0], newPos[1], newPos[2],
                  newRot[0], newRot[1], newRot[2], newRot[3]);

      // Store agent information for later use.
      this->pedestrians.push_back(ag);
      this->init_pedestrians.push_back(ag);
    }

    RCLCPP_INFO(this->node_->get_logger(), "Agents successfully initialized! Got %d pedestrians", this->pedestrians.size());

  }

  else
  {
    RCLCPP_ERROR(this->node_->get_logger(), "Failed to call service get_agents");
  }
}

bool HuNavPluginPrivate::GetRobot(const double &dt){

  WbNodeRef node = wb_supervisor_node_get_from_def(this->robotName.c_str());

  if (!node)
    {
      // Log error and return false if the node is not found.
      RCLCPP_ERROR(this->node_->get_logger(), "Robot model %s not found!!!!", this->robotName.c_str());
      return false;
    }

  // Obtain the translation field and read the current position.
  WbFieldRef trans_field = wb_supervisor_node_get_field(node, "translation");
  const double* newPos = wb_supervisor_field_get_sf_vec3f(trans_field);
  double xf = newPos[0]; 
  double yf = newPos[1];

  // 2) rotation
  const double *R = wb_supervisor_node_get_orientation(node);
  double yaw = std::atan2(R[3], R[0]);
  yaw = normalizeAngle(yaw);
  //Update velocities reading from the simulator
  const double *vel = wb_supervisor_node_get_velocity(node);
  // vel[0..2] = linear x,y,z; vel[3..5] = angular x,y,z
  double vx = vel[0];
  double vy = vel[1];
  double linearVelocity = std::hypot(vx, vy);
  double anvel = vel[5];  // yaw rate

  // //Update velocities manually
  // Compute the differences with respect to the previously stored pose.
  // Note: pedestrians[i].position.position.x and .y are expected to store the last known horizontal position.
  // double xi = this->robotAgent.position.position.x;
  // double yi = this->robotAgent.position.position.y;
  // double dist = sqrt((xf - xi) * (xf - xi) + (yf - yi) * (yf - yi));
  // double linearVelocity = dist / dt;
  // double anvel = normalizeAngle(yaw - this->robotAgent.yaw) / dt;
  // double vx = (xf - xi) / dt;
  // double vy = (yf - yi) / dt;

  robotAgent.position.position.x = xf;
  robotAgent.position.position.y = yf;
  robotAgent.yaw = yaw;
  robotAgent.velocity.linear.x = vx;
  robotAgent.velocity.linear.y = vy;
  robotAgent.linear_vel = linearVelocity;
  robotAgent.velocity.angular.z = anvel;
  robotAgent.angular_vel = anvel;

  // RCLCPP_INFO(node_->get_logger(),
  // "Read robot pose: %s | pos: (%.2f, %.2f, %.2f) | linvel: (%.2f) | angvel: (%.2f)",
  // robotName.c_str(), 
  // newPos[0], newPos[1], newPos[2],
  // linearVelocity, anvel);

  return true;

}
bool HuNavPluginPrivate::GetPedestrians(const double &dt)
{
  // Loop through all pedestrian agents in the vector.
  for (size_t i = 0; i < this->pedestrians.size(); ++i)
  {
    auto &ped = this->pedestrians[i];
    // Get the supervisor node using the DEF name of the pedestrian.
    WbNodeRef node = wb_supervisor_node_get_from_def(ped.name.c_str());
    if (!node)
    {
      // Log error and return false if the node is not found.
      RCLCPP_ERROR(node_->get_logger(), "Pedestrian model %s not found!!!!", ped.name.c_str());
      return false;
    }

    // Obtain the translation field and read the current position.
    WbFieldRef trans_field = wb_supervisor_node_get_field(node, "translation");
    const double* newPos = wb_supervisor_field_get_sf_vec3f(trans_field);
    double xf = newPos[0]; 
    double yf = newPos[1];

    // 2) rotation
    const double *R = wb_supervisor_node_get_orientation(node);
    double yaw_f = std::atan2(R[3], R[0]);
    yaw_f = normalizeAngle(yaw_f);

    // apply correction when the animation is looking +90 degrees
    if (ped.behavior.type == hunav_msgs::msg::AgentBehavior::BEH_SURPRISED &&
        ped.behavior.state != hunav_msgs::msg::AgentBehavior::BEH_NO_ACTIVE) {
      yaw_f = normalizeAngle(yaw_f - SURPRISED_YAW_OFFSET);
    }

    // Compute the differences with respect to the previously stored pose.
    // Note: pedestrians[i].position.position.x and .y are expected to store the last known horizontal position.

    //Update velocities reading from the simulator
    const double *vel = wb_supervisor_node_get_velocity(node);
    // vel[0..2] = linear x,y,z; vel[3..5] = angular x,y,z
    double vx = vel[0];
    double vy = vel[1];
    double linearVelocity = std::hypot(vx, vy);
    double anvel = vel[5];  // yaw rate

    double xi = ped.position.position.x;
    double yi = ped.position.position.y;
    double yaw_i = ped.yaw;

    // // // // //Update velocities manually
    // double dist = sqrt((xf - xi) * (xf - xi) + (yf - yi) * (yf - yi));
    // double linearVelocity = dist / dt;
    // double anvel = normalizeAngle(yaw_f - ped.yaw) / dt;
    // double vx = (xf - xi) / dt;
    // double vy = (yf - yi) / dt;

    // //Log the updated fields for this agent.
    RCLCPP_DEBUG(node_->get_logger(),
    "Read pose: %s | pos: (%.2f, %.2f, %.2f) | yaw: (%.2f) | linvel: (%.2f) | angvel: (%.2f)",
    pedestrians[i].name.c_str(), 
    newPos[0], newPos[1], newPos[2],
    yaw_f, linearVelocity, anvel);

    // Update the pedestrian’s velocity and angular velocity fields.
    ped.velocity.linear.x = vx;
    ped.velocity.linear.y = vy;
    ped.linear_vel = linearVelocity;
    ped.velocity.angular.z = anvel;
    ped.angular_vel = anvel;

    // Update the pose in the agent message.
    // Here we store the new horizontal coordinates.
    ped.position.position.x = xf;
    ped.position.position.y = yf;
    // Update the stored yaw value.
    ped.yaw = yaw_f;
    // Construct a quaternion from the yaw angle.
    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY(0, 0, yaw_f);
    ped.position.orientation = tf2::toMsg(myQuaternion);

  }

  // Clear the reset flag after processing.
  reset = false;

  return true;
}


void HuNavPluginPrivate::UpdateWebotsPedestrians(const hunav_msgs::msg::Agents& _agents, const double &dt){

   // If the robot navigation goal has not been received, log and return.
   if (goalReceived == false)
   {
     RCLCPP_INFO(node_->get_logger(), "HuNavPlugin. Waiting to receive the robot navigation goal...");
     return;
   }
 
   // Loop over each agent received in the service response.
   for (const auto &a : _agents.agents)
   {
     // Use the agent name (DEF name) to lookup its supervisor node.
     WbNodeRef agent_node = wb_supervisor_node_get_from_def(a.name.c_str());
     if (agent_node == NULL)
     {
       RCLCPP_ERROR(node_->get_logger(), "Supervisor node for agent '%s' not found!", a.name.c_str());
       continue;
     }

     // find matching pedestrian
    auto it = std::find_if(pedestrians.begin(), pedestrians.end(),
                           [&](auto &p){ return p.id == a.id; });
    if (it == pedestrians.end()) continue;
    auto &ped = *it;
 
     // --- Update the Translation Field ---
     // The "translation" field is of type SFVec3f.
     WbFieldRef trans_field = wb_supervisor_node_get_field(agent_node, "translation");
     const double newPos[3] = { 
       a.position.position.x, 
       a.position.position.y, 
       a.position.position.z,
     };
 
    // --- Update the Rotation Field ---
    // The "rotation" field is of type SFVec4f and expects [axis_x, axis_y, axis_z, angle].
    WbFieldRef rot_field = wb_supervisor_node_get_field(agent_node, "rotation");
    tf2::Quaternion q;
    tf2::fromMsg(a.position.orientation, q);
    //Extract yaw in (–π…π]:
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    yaw = normalizeAngle(yaw);

    //Push that to Webots around Z:
    const double newRot[4] = { 0.0, 0.0, 1.0, yaw };

    // Check distance between desired and actual pose
    double dx = a.position.position.x - ped.position.position.x;
    double dy = a.position.position.y - ped.position.position.y;
    double dyaw = normalizeAngle(a.yaw - ped.yaw);

    // Set absolute velocity on the node in world coordinates (kinematics based)
    // Format: [vx, vy, vz, wx, wy, wz]
    double vx = a.velocity.linear.x;
    double vy = a.velocity.linear.y;
    double yaw_rate = a.velocity.angular.z;
    
    wb_supervisor_field_set_sf_vec3f(trans_field, newPos);
    //The surprised animation look 90 degrees right, we must correct it --only-- in the simulator
    if (ped.behavior.type == hunav_msgs::msg::AgentBehavior::BEH_SURPRISED &&
    ped.behavior.state != hunav_msgs::msg::AgentBehavior::BEH_NO_ACTIVE){
      vx = dx;
      vy = dy;
      yaw_rate = 0.0;
      double desired_yaw = normalizeAngle(a.yaw + SURPRISED_YAW_OFFSET);
      const double correctedRot[4] = { 0.0, 0.0, 1.0, desired_yaw };
      wb_supervisor_field_set_sf_rotation(rot_field, correctedRot);
    }
    else{
        wb_supervisor_field_set_sf_rotation(rot_field, newRot);
    }

    double vel[6] = {
      vx,
      vy,
      0.0,                // No vertical velocity
      0.0,                // No angular x
      0.0,                // No angular y
      yaw_rate
    };
    wb_supervisor_node_set_velocity(agent_node, vel);

    //Log updated state
    RCLCPP_DEBUG(node_->get_logger(),
      "[→To Webots] Agent '%s' id=%d vel=(%.3f,%.3f,%.3f,%.3f,%.3f,%.3f)",
      a.name.c_str(), a.id,
      vel[0], vel[1], vel[2], vel[3], vel[4], vel[5]
    );

    int index = -1;
    for (unsigned int i = 0; i < this->pedestrians.size(); i++)
    {
      if (a.id == this->pedestrians[i].id)
      {
        this->pedestrians[i].goals.clear();
        this->pedestrians[i].goals = a.goals;

        // update behavior state
        if (a.behavior.state != this->pedestrians[i].behavior.state)
        {
          this->pedestrians[i].behavior.state = a.behavior.state;
          index = i;
          break;
        }
      }
    }
  }
}


// Helper to convert node‐type enum to a human‐readable string
static const char *nodeTypeName(WbNodeType t) {
  switch (t) {
    case WB_NODE_NO_NODE:      return "NONE";
    case WB_NODE_GROUP:     return "GROUP";
    case WB_NODE_TRANSFORM: return "TRANSFORM";
    case WB_NODE_SHAPE:     return "SHAPE";
    case WB_NODE_BOX:       return "BOX";
    case WB_NODE_CYLINDER:  return "CYLINDER";
    case WB_NODE_SPHERE:    return "SPHERE";
    case WB_NODE_CAPSULE:   return "CAPSULE";
    case WB_NODE_PLANE:     return "PLANE";
    case WB_NODE_SOLID:     return "SOLID";
    // … add any others you care about …
    default:                return "OTHER";
  }
}


static bool isPrimitiveNode(WbNodeRef node) {
  switch (wb_supervisor_node_get_type(node)) {
    case WB_NODE_BOX:
    case WB_NODE_CYLINDER:
    case WB_NODE_SPHERE:
    case WB_NODE_CAPSULE:
    case WB_NODE_PLANE:
      return true;
    default:
      return false;
  }
}



void HuNavPluginPrivate::collectCollisionPrimitives(WbNodeRef node, std::vector<WbNodeRef>& out) {

  if (!node)
    return;

  const WbNodeType type = wb_supervisor_node_get_type(node);

  // 1) If it’s already a Box/Cylinder/Sphere/Plane/…
  if (isPrimitiveNode(node)) {
    out.push_back(node);
    return;
  }

  // 2) If it’s a Shape, grab its geometry sub-node:
  if (type == WB_NODE_SHAPE) {
    WbFieldRef geomField = wb_supervisor_node_get_field(node, "geometry");
    if (geomField && wb_supervisor_field_get_type(geomField) == WB_SF_NODE) {
      WbNodeRef geom = wb_supervisor_field_get_sf_node(geomField);
      if (geom && isPrimitiveNode(geom)) {
        out.push_back(geom);
        return;
      }
    }
    return;
  }

  // 3) If it’s a Solid, first try its boundingObject:
  if (type == WB_NODE_SOLID) {
    WbFieldRef bbField = wb_supervisor_node_is_proto(node)
      ? wb_supervisor_node_get_base_node_field(node, "boundingObject")
      : wb_supervisor_node_get_field(node, "boundingObject");

    if (bbField && wb_supervisor_field_get_type(bbField) == WB_SF_NODE) {
      if (WbNodeRef bbNode = wb_supervisor_field_get_sf_node(bbField)) {
        collectCollisionPrimitives(bbNode, out);
        return;
      }
    }

    // 4) no explicit boundingObject: only walk *Solid* children
    WbFieldRef children = wb_supervisor_node_is_proto(node)
      ? wb_supervisor_node_get_base_node_field(node, "children")
      : wb_supervisor_node_get_field(node, "children");
    if (!children)
      return;
    const int count = wb_supervisor_field_get_count(children);
    for (int i = 0; i < count; ++i) {
      WbNodeRef child = wb_supervisor_field_get_mf_node(children, i);
      if (wb_supervisor_node_get_type(child) == WB_NODE_SOLID)
        collectCollisionPrimitives(child, out);
    }
  }
}

std::vector<Eigen::Vector3d> HuNavPluginPrivate::GetClosestPointOnBoundingBox(
    const Eigen::Vector3d& point,
    WbNodeRef modelNode) 
{
  std::vector<Eigen::Vector3d> cps;

  // 1) Get model's world-space center and orientation
  const double *pos = wb_supervisor_node_get_position(modelNode);
  const double *rot = wb_supervisor_node_get_orientation(modelNode);  // 3x3 rotation matrix

  Eigen::Vector3d center(pos[0], pos[1], pos[2]);
  Eigen::Matrix3d R;
  R << rot[0], rot[1], rot[2],
       rot[3], rot[4], rot[5],
       rot[6], rot[7], rot[8];

  // 2) Transform point to local frame
  Eigen::Vector3d localPoint = R.transpose() * (point - center);

  // 3) Get primitives
  std::vector<WbNodeRef> prims;
  collectCollisionPrimitives(modelNode, prims);
  if (prims.empty())
    return {};

  for (auto prim : prims) {
    WbNodeType type = wb_supervisor_node_get_type(prim);

    if (type == WB_NODE_BOX) {
      const double *sz = wb_supervisor_field_get_sf_vec3f(
                           wb_supervisor_node_get_field(prim, "size"));
      Eigen::Vector3d h(sz[0] / 2.0, sz[1] / 2.0, sz[2] / 2.0);

      Eigen::Vector3d localClamped;
      localClamped.x() = std::clamp(localPoint.x(), -h.x(), h.x());
      localClamped.y() = std::clamp(localPoint.y(), -h.y(), h.y());
      localClamped.z() = std::clamp(localPoint.z(), -h.z(), h.z());

      // Back to world frame
      cps.push_back(R * localClamped + center);

    } else if (type == WB_NODE_CYLINDER || type == WB_NODE_CAPSULE) {
      double r  = wb_supervisor_field_get_sf_float(
                    wb_supervisor_node_get_field(prim, "radius"));
      double hh = wb_supervisor_field_get_sf_float(
                    wb_supervisor_node_get_field(prim, "height")) * 0.5;

      double clampedY = std::clamp(localPoint.y(), -hh, hh);
      Eigen::Vector2d xz(localPoint.x(), localPoint.z());
      double d = xz.norm();
      Eigen::Vector2d xzOnSurface = (d > 1e-6) ? (xz * (r / d)) : Eigen::Vector2d(r, 0);

      Eigen::Vector3d localClamped(xzOnSurface.x(), clampedY, xzOnSurface.y());
      cps.push_back(R * localClamped + center);

    } else if (type == WB_NODE_SPHERE) {
      double r = wb_supervisor_field_get_sf_float(
                   wb_supervisor_node_get_field(prim, "radius"));
      Eigen::Vector3d dir = localPoint;
      double dist = dir.norm();

      Eigen::Vector3d localClamped;
      if (dist > 1e-6)
        localClamped = dir * (r / dist);
      else
        localClamped = Eigen::Vector3d(r, 0, 0);

      cps.push_back(R * localClamped + center);

    } else if (type == WB_NODE_PLANE) {
      WbFieldRef sizeField = wb_supervisor_node_get_field(prim, "size");
      const double *sz = wb_supervisor_field_get_sf_vec2f(sizeField);
      double halfX = sz[0] / 2.0;
      double halfZ = sz[1] / 2.0;

      double cx = std::clamp(localPoint.x(), -halfX, halfX);
      double cz = std::clamp(localPoint.z(), -halfZ, halfZ);
      double cy = 0.0;  // assuming plane is flat on Y=0 in local space

      Eigen::Vector3d localClamped(cx, cy, cz);
      cps.push_back(R * localClamped + center);
    }
  }

  return cps;
}


void HuNavPluginPrivate::HandleObstacles(){

  // 1) Get the root of the scene tree
  WbNodeRef root = wb_supervisor_node_get_root();     
  WbFieldRef worldChildren = wb_supervisor_node_get_field(root, "children");
  int modelCount = wb_supervisor_field_get_count(worldChildren); 

  for (size_t i = 0; i < this->pedestrians.size(); ++i)
  {
    auto &ped = this->pedestrians[i];
    // RCLCPP_INFO(node_->get_logger(),
    // "Handling obstacles for pedestrian %d", i);

    // World‐space actor position from your stored Agent msg
    Eigen::Vector3d actorPos{
      ped.position.position.x,
      ped.position.position.y,
      ped.position.position.z
    };
    double minDist = 5.0;
    ped.closest_obs.clear();

    for (int m = 0; m < modelCount; ++m) {

      WbNodeRef mdl = wb_supervisor_field_get_mf_node(worldChildren, m);
      if (!mdl) continue;
    
      WbFieldRef nameField = wb_supervisor_node_get_field(mdl, "name");
      if(!nameField) continue;
      const char *modelName = wb_supervisor_field_get_sf_string(nameField);  
      if (!modelName) continue;

      // inside your m‐loop, after fetching modelName...
      if ( std::find(ignoreModels.begin(), ignoreModels.end(), modelName) != ignoreModels.end()
      || std::any_of(pedestrians.begin(), pedestrians.end(),
                    [&](auto &o){ return o.name == modelName; }) ){
            
            // RCLCPP_WARN(node_->get_logger(), "Ignoring model: %s", modelName);
            continue;
      }

      // RCLCPP_WARN(node_->get_logger(), "Evaluating Model: %s", modelName);

      const auto cps = GetClosestPointOnBoundingBox(actorPos, mdl);
      for (auto &cp : cps) {
        double d = (cp - actorPos).norm();
        if (d > 0.0 && d < minDist) {
          minDist = d;
          geometry_msgs::msg::Point p;
          p.x = cp.x(); p.y = cp.y(); p.z = cp.z();
          ped.closest_obs.push_back(p);
          RCLCPP_DEBUG(node_->get_logger(),
          " Pedestrian %d closest obstacle on '%s' at [%.2f, %.2f, %.2f]",
          i, modelName, p.x, p.y, p.z);
        }
      }
      
    }

    // RCLCPP_INFO(node_->get_logger(),
    //   "Handled obstacles for pedestrian %d", i);
  }
}


bool HuNavPluginPrivate::Reset(){
  
  RCLCPP_INFO(node_->get_logger(), "\n\n---------World reset---------\n");

  wb_supervisor_simulation_reset_physics();

  // Restore agents' positions in Webots
  for (const auto& agent : this->init_pedestrians) {
    WbNodeRef agent_node = wb_supervisor_node_get_from_def(agent.name.c_str());
    if (!agent_node) {
      RCLCPP_WARN(node_->get_logger(), "Agent node '%s' not found in supervisor!", agent.name.c_str());
      continue;
    }

    // Update position
    WbFieldRef trans_field = wb_supervisor_node_get_field(agent_node, "translation");
    const double newPos[3] = {
      agent.position.position.x,
      agent.position.position.y,
      agent.position.position.z
    };
    wb_supervisor_field_set_sf_vec3f(trans_field, newPos);

    // Update orientation
    tf2::Quaternion q;
    tf2::fromMsg(agent.position.orientation, q);
    // 2) Extract yaw in (–π…π]:
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    yaw = normalizeAngle(yaw);

    // 4) Push that to Webots around Z:
    const double newRot[4] = { 0.0, 0.0, 1.0, yaw };

    WbFieldRef rot_field = wb_supervisor_node_get_field(agent_node, "rotation");
    wb_supervisor_field_set_sf_rotation(rot_field, newRot);
  }

  // Update local runtime agents state
  this->pedestrians = this->init_pedestrians;

  // Wait for the reset service
  rclcpp::Time start_time = this->node_->now();
  rclcpp::Duration timeout = rclcpp::Duration::from_seconds(5.0);
  while (!rosSrvResetClient->wait_for_service(1s)) {
    if ((this->node_->now() - start_time) > timeout) {
      RCLCPP_ERROR(this->node_->get_logger(), "Timeout while waiting for service /reset_agents");
      return false;
    }
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->node_->get_logger(), "Node interrupted while waiting for service /reset_agents");
      return false;
    }
    RCLCPP_WARN(this->node_->get_logger(), "Service /reset_agents not available, waiting...");
  }

  // Prepare request
  auto request = std::make_shared<hunav_msgs::srv::ResetAgents::Request>();
  request->current_agents.agents = init_pedestrians;
  request->current_agents.header.frame_id = globalFrame;
  request->current_agents.header.stamp = rclcpp::Time(static_cast<uint64_t>(wb_robot_get_time() * 1e9));;
  request->robot = init_robotAgent;

  // Call service
  auto result = rosSrvResetClient->async_send_request(request);
  rclcpp::FutureReturnCode status =
    rclcpp::spin_until_future_complete(node_->get_node_base_interface(), result, std::chrono::milliseconds(500));

  if (status == rclcpp::FutureReturnCode::SUCCESS && result.get()->ok) {
    RCLCPP_INFO(this->node_->get_logger(), "World successfully reset.");
    return true;
  } else {
    RCLCPP_ERROR(this->node_->get_logger(), "Failed to reset agents via /reset_agents");
    return false;
  }
}

void HuNavPluginPrivate::loadAnimations(const std::string& base_path,
                                        const std::vector<std::string>& anim_names) {

  for (const auto& anim_name : anim_names) {
    std::string full_path = base_path + "/" + anim_name;
    WbuBvhMotion motion = wbu_bvh_read_file(full_path.c_str());
    if (!motion) {
      RCLCPP_WARN(node_->get_logger(), "Failed to load BVH: %s", full_path.c_str());
      continue;
    }
    // Optionally scale motion (20 is empirically good for CMU BVH)
    wbu_bvh_set_scale(motion, 20);
    this->motions[anim_name] = motion;

    // For the agent skinDevice, build bone maps
    std::vector<int> boneMap;
    int boneCount = wb_skin_get_bone_count(this->skinDevice);
    int jointCount = wbu_bvh_get_joint_count(motion);
    boneMap.resize(boneCount, -1);

    for (int i = 0; i < boneCount; ++i) {
      const char* boneName = wb_skin_get_bone_name(skinDevice, i);
      for (int j = 0; j < jointCount; ++j) {
        if (strcmp(boneName, wbu_bvh_get_joint_name(motion, j)) == 0) {
          boneMap[i] = j;
          break;
        }
      }
      if (boneMap[i] >= 0) {
        auto globalTp = wb_skin_get_bone_orientation(skinDevice, i, true);
        wbu_bvh_set_model_t_pose(motion, globalTp, boneMap[i], true);
        auto localTp = wb_skin_get_bone_orientation(skinDevice, i, false);
        wbu_bvh_set_model_t_pose(motion, localTp, boneMap[i], false);
      }
    }

    this->motionBoneMaps[anim_name] = boneMap;
  }

  RCLCPP_INFO(node_->get_logger(), "Successfully loaded animations");
}


std::string HuNavPluginPrivate::chooseAnimation(const hunav_msgs::msg::Agent& agent) {

  // These are the available animations!
  // std::vector<std::string> animations = { "walk",           "69_02_walk_forward.bvh",   "137_28-normal_wait.bvh",
  //                                      "142_01-walk_childist.bvh", "07_04-slow_walk.bvh",      "02_01-walk.bvh",
  //                                      "142_17-walk_scared.bvh",   "17_01-walk_with_anger.bvh" };

  // // Set idle animation if agent velocities are near zero
  // double lin_vel = agent.linear_vel;
  // if (agent.linear_vel < 1e-6) {
  //     return "137_28-normal_wait.bvh";
  // }

  //Choose inactive animations
  if (agent.behavior.state == hunav_msgs::msg::AgentBehavior::BEH_NO_ACTIVE){
      return "walk.bvh";
  }
  //Choose active animations
  else{
    switch (agent.behavior.type) {
      case hunav_msgs::msg::AgentBehavior::BEH_REGULAR:
            return "walk.bvh";
      case hunav_msgs::msg::AgentBehavior::BEH_IMPASSIVE:
            return "69_02_walk_forward.bvh";
      case hunav_msgs::msg::AgentBehavior::BEH_SCARED:
            return "142_17-walk_scared.bvh";
      case hunav_msgs::msg::AgentBehavior::BEH_CURIOUS:
            return "07_04-slow_walk.bvh";
      case hunav_msgs::msg::AgentBehavior::BEH_THREATENING:
            return "17_01-walk_with_anger.bvh";
      case hunav_msgs::msg::AgentBehavior::BEH_SURPRISED:
            return "137_28-normal_wait.bvh";
      default:
        return "walk.bvh";
    }
  }

}

void HuNavPluginPrivate::manageAnimations() {
  
  const hunav_msgs::msg::Agent* agent = nullptr;
  if(this->agentName == this->overseerName){
    // Overseer determines which agent to use from list of pedestrians
    auto pedIt = std::find_if(this->pedestrians.begin(), this->pedestrians.end(),
                              [&](const auto& p) { return p.name == this->agentName; });
    if (pedIt == this->pedestrians.end())
      return;
    agent = &(*pedIt);
  }
  //The rest of the agents just use the one received by the compute_agent service
  else agent = &this->passiveAgent;

  //Choose the animation based on pedestrian state
  std::string selectedAnim = chooseAnimation(*agent);

  // Log only if the animation has changed
  if (selectedAnim != this->currentAnim) {
    RCLCPP_INFO(node_->get_logger(), "Agent %s: switching animation from %s to %s",
                this->agentName.c_str(),
                this->currentAnim.empty() ? "NONE" : this->currentAnim.c_str(),
                selectedAnim.c_str());
    currentAnim = selectedAnim;
  }

  auto itMotion = motions.find(selectedAnim);
  auto itBones = motionBoneMaps.find(selectedAnim);
  if (itMotion == motions.end() || itBones == motionBoneMaps.end())
    return;

  WbuBvhMotion& motion = itMotion->second;
  const std::vector<int>& boneMap = itBones->second;

  int boneCount = wb_skin_get_bone_count(this->skinDevice);
  for (int i = 0; i < boneCount; ++i) {
    int j = boneMap[i];
    if (j < 0) continue;
    const double* rot = wbu_bvh_get_joint_rotation(motion, j);
    wb_skin_set_bone_orientation(this->skinDevice, i, rot, false);
  }

  int steps = 4;
  if(this->currentAnim.c_str() != "137_28-normal_wait.bvh")
    // Adjust step count: 2 to 5 depending on velocity (linear_vel)
    steps = std::clamp(static_cast<int>(1 + 3 * agent->linear_vel), 1, 6);

  for (int k = 0; k < steps; ++k){
    wbu_bvh_step(motion);
    //The first frame is always a T-pose, so we skip it
    int frame = wbu_bvh_get_frame_index(motion);
    if (frame == 0) {
      wbu_bvh_step(motion);
    }
  }
}

void HuNavPlugin::step() {

  double now = wb_robot_get_time();
  double dt = now - hnav_->lastUpdate;

  /********************************UPDATE ALL OF THE AGENTS *********************************/
  if(hnav_->agentName == hnav_->overseerName){
    
    double overseer_dt = now - hnav_->lastOverseerUpdate;
    if (!hnav_->GetRobot(dt)){
        RCLCPP_ERROR(hnav_->node_->get_logger(), "Could not get robot!");
        return;
      }
    if (!hnav_->GetPedestrians(dt)){
        RCLCPP_ERROR(hnav_->node_->get_logger(), "Could not get pedestrians!");
        return;
      }

    hnav_->HandleObstacles();

    // If no request is in progress, send one
    if (!hnav_->agents_future.valid() && hnav_->rosSrvComputeAgentsClient->service_is_ready()) {
      auto request = std::make_shared<hunav_msgs::srv::ComputeAgents::Request>();
      hunav_msgs::msg::Agents agents;
      agents.header.frame_id = hnav_->globalFrame;
      agents.header.stamp = rclcpp::Time(static_cast<uint64_t>(now * 1e9));;
      agents.agents = hnav_->pedestrians;

      request->robot = hnav_->robotAgent;
      request->current_agents = agents;

      hnav_->agents_request_start = hnav_->node_->now();  // New member variable
      hnav_->agents_future = hnav_->rosSrvComputeAgentsClient->async_send_request(request);

      //***********If service does not respond, interpolate positions **********/
      if (hnav_->have_last_service) {

        // Build an interp message from the *current* Webots readings...
        hunav_msgs::msg::Agents interp;
        interp.header = hnav_->last_service_agents.header;  // keep frame_id, stamp
        interp.agents.reserve(hnav_->pedestrians.size());

        for (auto &ped : hnav_->pedestrians) {
          // start from the *current* Webots‐read pose:
          auto it = std::find_if(
          hnav_->last_service_agents.agents.begin(),
          hnav_->last_service_agents.agents.end(),
          [&](auto &x){ return x.id == ped.id; });

          if (it == hnav_->last_service_agents.agents.end()) continue;
          hunav_msgs::msg::Agent a = *it;  // copy all fields *including* last.velocities

          // 3) dead-reckon forward in time using *last* velocities
          a.position.position.x += it->velocity.linear.x  * dt;
          a.position.position.y += it->velocity.linear.y  * dt;
          a.yaw += it->velocity.angular.z * dt;
          a.yaw = hnav_->normalizeAngle(a.yaw);
          tf2::Quaternion q; q.setRPY(0,0,a.yaw);
          a.position.orientation = tf2::toMsg(q);
          interp.agents.push_back(a);

          double yaw_deg = a.yaw * 180.0/M_PI;
          RCLCPP_DEBUG(hnav_->node_->get_logger(),
            "[InterpolationAgents→] [%2d] %s  pos=(%.3f, %.3f, %.3f)  yaw=%.1f°  lin_vel=%.3f, ang_vel=%.3f",
            a.id,
            a.name.c_str(),
            a.position.position.x,
            a.position.position.y,
            a.position.position.z,
            yaw_deg,
            a.linear_vel,
            a.angular_vel);
        }

        hnav_->UpdateWebotsPedestrians(interp, dt); 
      }
    } 
    
    // 2) If the request finishes, update stored agent list
    if (hnav_->agents_future.valid() && hnav_->agents_future.wait_for(0ms) == std::future_status::ready) {

      auto response = hnav_->agents_future.get();
      // rclcpp::Duration duration = hnav_->node_->now() - hnav_->agents_request_start;
      // RCLCPP_INFO(hnav_->node_->get_logger(), "compute_agents response took %.3f ms",
      //     duration.seconds() * 1000.0);

      // === LOG THE RAW SERVICE VALUES ===
      for (const auto &a : response->updated_agents.agents) {
        double yaw_deg = a.yaw * 180.0/M_PI;
        RCLCPP_DEBUG(hnav_->node_->get_logger(),
          "[ComputeAgents→] [%2d] %s  pos=(%.3f, %.3f, %.3f)  yaw=%.1f°  lin_vel=%.3f, ang_vel=%.3f",
          a.id,
          a.name.c_str(),
          a.position.position.x,
          a.position.position.y,
          a.position.position.z,
          yaw_deg,
          a.linear_vel,
          a.angular_vel);
      }
      
      hnav_->UpdateWebotsPedestrians(response->updated_agents, overseer_dt);
      hnav_->last_service_agents = response->updated_agents;
      hnav_->have_last_service = true;
      hnav_->agents_future = {};
      hnav_->lastOverseerUpdate = wb_robot_get_time();
    }

  }

  /********************************Rest of agents update their own animations/behaviours*********************************/
  else{

      if (!hnav_->rosSrvComputeAgentClient->service_is_ready()) {
        RCLCPP_WARN(hnav_->node_->get_logger(), "Service /compute_agent not available for agent %s", hnav_->agentName.c_str());
        return;
      }

      // Wait a bit for overseer to call compute_agents first (move_agent does not work otherwise)
      if (!hnav_->computeAgentReady) {
        if ((now - hnav_->startTime) >= 1.0) {
          // RCLCPP_INFO(hnav_->node_->get_logger(), "[%s] Initial delay complete. Starting /compute_agent updates.", hnav_->agentName.c_str());
          hnav_->computeAgentReady = true;
          hnav_->lastAgentUpdate = now; 
        } else return;
      }

      if (!hnav_->agent_future.valid()) {
      auto request = std::make_shared<hunav_msgs::srv::ComputeAgent::Request>();
      request->id = hnav_->agentId;
      hnav_->agent_future = hnav_->rosSrvComputeAgentClient->async_send_request(request);
      } 
      else {
        auto status = hnav_->agent_future.wait_for(std::chrono::milliseconds(0));
        if (status == std::future_status::ready) {
          auto response = hnav_->agent_future.get();
          hnav_->passiveAgent = response->updated_agent;
          // Reset for next request
          hnav_->agent_future = std::shared_future<hunav_msgs::srv::ComputeAgent::Response::SharedPtr>();
          hnav_->lastAgentUpdate = now; 
        }
      }
  }

  //Apply animations to each agent
  hnav_->manageAnimations();
  hnav_->lastUpdate = wb_robot_get_time();

}
} // namespace hunav_plugin

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(hunav_plugin::HuNavPlugin,
                       webots_ros2_driver::PluginInterface)