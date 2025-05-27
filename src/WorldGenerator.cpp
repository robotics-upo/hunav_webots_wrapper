/***********************************************************************/
/**                                                                    */
/** WorldGenerator.cpp                                                 */
/**                                                                    */
/** Copyright (c) 2022, Service Robotics Lab (SRL).                    */
/**                     http://robotics.upo.es                         */
/**                                                                    */
/** All rights reserved.                                               */
/**                                                                    */
/** Authors:                                                           */
/** Noé Pérez-Higueras (maintainer)                                    */
/** email: noeperez@upo.es                                             */
/**                                                                    */
/** This software may be modified and distributed under the terms      */
/** of the MIT license. See the LICENSE file for details.              */
/**                                                                    */
/**                                                                    */
/***********************************************************************/

#include "hunav_webots_wrapper/WorldGenerator.hpp"
#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <random>    // for std::random_device, std::mt19937, std::uniform_int_distribution
#include <filesystem>
namespace fs = std::filesystem;


using namespace tinyxml2;

namespace hunav
{

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

WorldGenerator::WorldGenerator() : Node("hunav_webots_world_generator")
{
  // fill with the names of agent parameters
  params_ = { ".id",
              ".skin",
              ".behavior.type",
              ".behavior.configuration",
              ".behavior.duration",
              ".behavior.once",
              ".behavior.vel",
              ".behavior.dist",
              ".behavior.social_force_factor",
              ".behavior.goal_force_factor",
              ".behavior.obstacle_force_factor",
              ".behavior.other_force_factor",
              ".group_id",
              ".max_vel",
              ".radius",
              ".init_pose.x",
              ".init_pose.y",
              ".init_pose.z",
              ".init_pose.h",
              ".goal_radius",
              ".cyclic_goals",
              ".goals" };
  // names of the goal parameters
  goal_params_ = { ".x", ".y", ".h" };

  agents_srv_ = this->create_service<hunav_msgs::srv::GetAgents>(
      std::string("get_agents"), std::bind(&hunav::WorldGenerator::getAgentsService, this, _1, _2));
  // agents_srv_ = this->create_service<hunav_msgs::srv::GetAgents>(
  //    std::string("get_agents"), &hunav::WorldGenerator::getAgentsService);

  // Read the plugin parameters
  readPluginParams();
  // Read the agents parameters
  readAgentParams();
  // // Generate the world
  appendAgentsToWorldFile();
  generateAgentURDFFiles();
}

WorldGenerator::~WorldGenerator()
{
}

void WorldGenerator::readPluginParams()
{
  std::string package_shared_dir, urdf_shared_dir, world_shared_dir;
  try {
    package_shared_dir =
        ament_index_cpp::get_package_share_directory("hunav_webots_wrapper");

  } catch (ament_index_cpp::PackageNotFoundError) {
    RCLCPP_ERROR(this->get_logger(),
                 "Package hunav_gazebo_wrapper not found in dir: %s!!!",
                 package_shared_dir.c_str());
  }
  urdf_shared_dir = package_shared_dir + "/resource/";
  world_shared_dir = package_shared_dir + "/worlds/";

  // Plugin parameters
  std::string base_world_name = this->declare_parameter<std::string>("base_world", std::string("factory.wbt"));
  urdf_folder_ = this->declare_parameter<std::string>("urdf_folder", std::string(urdf_shared_dir));
  plug_use_gazebo_obs_ = this->declare_parameter<bool>("use_gazebo_obs", false);
  plug_robot_name_ = this->declare_parameter<std::string>("robot_name", std::string("robot"));
  RCLCPP_INFO(this->get_logger(), "Robot name: %s", plug_robot_name_.c_str());
  plug_global_frame_ = this->declare_parameter<std::string>("global_frame_to_publish", std::string("map"));
  plug_use_navgoal_to_start_ = this->declare_parameter<bool>("use_navgoal_to_start", false);
  plug_navgoal_topic_ = this->declare_parameter<std::string>("navgoal_topic", std::string("goal_pose"));
  this->declare_parameter<std::string>("ignore_models", std::string("model_ign"));
  rclcpp::Parameter ig_models = this->get_parameter("ignore_models");
  std::string models = ig_models.as_string();
  // RCLCPP_INFO(this->get_logger(), "Ignore_models string: %s", models.c_str());
  const char delim = ' ';
  tokenize(models, delim, plug_ignore_models_);
  for (std::string st : plug_ignore_models_)
  {
    RCLCPP_INFO(this->get_logger(), "Ignore_model: %s", st.c_str());
  }

  // Create the temp directory if it doesn't exist
  fs::path temp_dir = world_shared_dir + "temp";
  if (!fs::exists(temp_dir)) {
    try {
      fs::create_directories(temp_dir);
      RCLCPP_INFO(this->get_logger(), "Created temp directory: %s", temp_dir.c_str());
    } catch (const fs::filesystem_error& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create temp directory: %s", e.what());
      return;
    }
  }

  // Define temporary output path in the same directory
  base_world_path_ = world_shared_dir + base_world_name;
  std::string temp_world = temp_dir.string() + "/" + base_world_name;

  try {
      fs::copy_file(base_world_path_, temp_world, fs::copy_options::overwrite_existing);
      RCLCPP_INFO(this->get_logger(), "Copied base world to: %s", temp_world.c_str());
  } catch (const fs::filesystem_error& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to copy world file: %s", e.what());
      return;
  }

  // Set base_world_ to point to the temp file, not the original
  base_world_path_ = temp_world;
}

void WorldGenerator::readAgentParams()
{
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this, "hunav_loader");
  while (!parameters_client->wait_for_service(1s))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      rclcpp::shutdown();
    }
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }
  RCLCPP_INFO(this->get_logger(), "Reading parameters...");
  auto parameters = parameters_client->get_parameters({ "map", "agents" });

  std::string map = parameters[0].value_to_string();

  std::cout << "map parameter: " << map << std::endl;
  std::cout << "agent names: " << parameters[1].value_to_string() << std::endl << std::endl;

  //   for (auto &parameter : parameters) {
  //     std::cout << "\nParameter name: " << parameter.get_name() << std::endl;
  //     std::cout << "Parameter value (" << parameter.get_type_name()
  //               << "): " << parameter.value_to_string() << std::endl;
  //   }

  auto agent_names = parameters[1].as_string_array();
  for (std::string an : agent_names)
  {
    std::cout << "agent name: " << an << std::endl;
    std::vector<std::string> agent_params = params_;
    for (unsigned int i = 0; i < params_.size(); i++)
    {
      agent_params[i] = an + agent_params[i];
      // std::cout << "agent_params " << i << ": " << agent_params[i] <<
      // std::endl;
    }
    auto aparams = parameters_client->get_parameters(agent_params);

    // std::cout << "aparams: " << aparams << std::endl;
    hunav_msgs::msg::Agent a;
    a.name = an;
    // std::cout << "aparams[0]: " << aparams[0] << std::endl;
    a.id = aparams[0].as_int();
    // std::cout << "id: " << a.id << std::endl;
    a.type = hunav_msgs::msg::Agent::PERSON;
    // std::cout << "aparams[1]: " << aparams[1] << std::endl;
    a.skin = aparams[1].as_int();
    // std::cout << "skin: " << a.skin << std::endl;

    // behavior
    a.behavior.type = aparams[2].as_int();
    a.behavior.configuration = aparams[3].as_int();
    a.behavior.duration = aparams[4].as_double();
    a.behavior.once = aparams[5].as_bool();
    a.behavior.vel = aparams[6].as_double();
    a.behavior.dist = aparams[7].as_double();
    a.behavior.social_force_factor = aparams[8].as_double();
    a.behavior.goal_force_factor = aparams[9].as_double();
    a.behavior.obstacle_force_factor = aparams[10].as_double();
    a.behavior.other_force_factor = aparams[11].as_double();

    // std::cout << "aparams[12]: " << aparams[12] << std::endl;
    a.group_id = aparams[12].as_int();
    // std::cout << "group_id: " << a.group_id << std::endl;
    // std::cout << "aparams[13]: " << aparams[13] << std::endl;
    a.desired_velocity = aparams[13].as_double();
    // std::cout << "desired vel: " << a.desired_velocity << std::endl;
    // std::cout << "aparams[14]: " << aparams[14] << std::endl;
    a.radius = aparams[14].as_double();
    // std::cout << "radius: " << a.radius << std::endl;

    // init pose
    a.position.position.x = aparams[15].as_double();
    a.position.position.y = aparams[16].as_double();
    a.position.position.z = aparams[17].as_double();
    a.yaw = aparams[18].as_double();
    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY(0, 0, aparams[9].as_double());
    a.position.orientation = tf2::toMsg(myQuaternion);
    a.goal_radius = aparams[19].as_double();
    a.cyclic_goals = aparams[20].as_bool();

    std::cout << "id: " << a.id << " skin:" << (int)a.skin << " group_id:" << (int)a.group_id
              << " max_vel:" << a.desired_velocity << " radius:" << a.radius << std::endl
              << " initpose.x:" << a.position.position.x << " initpose.y:" << a.position.position.y << std::endl;
    std::cout << "Behavior:" << std::endl;
    std::cout << "type:" << (int)a.behavior.type << " configuration:" << (int)a.behavior.configuration
              << " duration:" << a.behavior.duration << " once:" << a.behavior.once << " vel:" << a.behavior.vel
              << " dist:" << a.behavior.dist << " goal_force_factor:" << a.behavior.goal_force_factor
              << " obstacle_force_factor:" << a.behavior.obstacle_force_factor
              << " social_force_factor:" << a.behavior.social_force_factor
              << " other_force_factor:" << a.behavior.other_force_factor << std::endl;

    auto goal_names = aparams[21].as_string_array();
    for (std::string goal : goal_names)
    {
      std::vector<std::string> gnames = goal_params_;
      for (unsigned int i = 0; i < goal_params_.size(); i++)
      {
        gnames[i] = an + "." + goal + goal_params_[i];
      }
      auto gparams = parameters_client->get_parameters({ gnames });
      geometry_msgs::msg::Pose p;
      p.position.x = gparams[0].as_double();
      p.position.y = gparams[1].as_double();
      tf2::Quaternion quat;
      quat.setRPY(0, 0, gparams[2].as_double());
      p.orientation = tf2::toMsg(quat);
      a.goals.push_back(p);
      std::cout << "goal: " << goal << " x:" << p.position.x << " y:" << p.position.y << std::endl;
    }

    agents_.agents.push_back(a);
  }
}

void WorldGenerator::getAgentsService(const std::shared_ptr<hunav_msgs::srv::GetAgents::Request> request,
                                      std::shared_ptr<hunav_msgs::srv::GetAgents::Response> response)
{
  int r = request->empty;
  response->agents = agents_;
  std::cout << "Sending " << agents_.agents.size() << " agents to agent_manager" << std::endl;
  std::cout << "Shutting down WorldGenerator..." << std::endl;
  // do not turn off the node because in webots we must answer all the nodes
  //rclcpp::shutdown();
}

// Helper function to generate a Webots Robot node (agent) definition in .wbt format.
std::string WorldGenerator::generateAgentWBT(const hunav_msgs::msg::Agent &agent) {
  std::ostringstream oss;
  
  // Use agent.name as the DEF identifier and the robot name.
  std::string defName = agent.name;

  std::string skinName;
  switch(agent.skin) {
    case 0: skinName = "Robert"; break;
    case 1: skinName = "Sandra"; break;
    case 2: skinName = "Sophia"; break;
    case 3: skinName = "Anthony"; break;
    default: skinName = "Robert"; break;
  }
  
  // Start the Robot node with a top-level translation from the agent position.
  oss << "DEF " << defName << " Robot {\n";
  oss << "  translation " << agent.position.position.x << " " 
      << agent.position.position.y << " " << agent.position.position.z << "\n";
  oss << "  children [\n";
  
  // CharacterSkin child with dynamic skin based on agent.skin.
  oss << "    CharacterSkin {\n";
  oss << "      name \"" << skinName << "\"\n";
  oss << "      model \"" << skinName << "\"\n";
  oss << "    }\n";

  // Agent body geometry defined with a Transform node.
  oss << "    Transform {\n";
  oss << "      translation 0 0 0.9\n";
  oss << "      children [\n";
  oss << "        Shape {\n";
  oss << "          appearance PBRAppearance {\n";
  oss << "            baseColor 0 0 0\n";
  oss << "            transparency 1\n";  // Fully transparent (invisible)
  oss << "            roughness 0\n";
  oss << "            metalness 0\n";
  oss << "          }\n";
  oss << "          geometry DEF AGENTBODY Cylinder {\n";
  oss << "            height 1.78\n";
  oss << "            radius 0.35\n";
  oss << "          }\n";
  oss << "        }\n";
  oss << "      ]\n";
  oss << "    }\n";
  
  // BoundingObject using the previously defined AGENTBODY.
  oss << "  ]\n";
  oss << "  boundingObject Transform {\n";
  oss << "    translation 0 0 0.9\n";
  oss << "    children [\n";
  oss << "      USE AGENTBODY\n";
  oss << "    ]\n";
  oss << "  }\n";
    
  // default physics.
  // oss << "  physics Physics { }\n";

  // // --- Physics for a ~75 kg human ---
  // oss << "  physics Physics {\n";
  // oss << "    density       -1       # ignore density, use mass\n";
  // oss << "    mass          75       # kg\n";
  // oss << "    centerOfMass  [ 0 0 0.9 ]\n";
  // oss << "    inertiaMatrix [\n";
  // oss << "      1e6 1e6 1.0\n";  // Ixx Iyy Izz — > 0
  // oss << "      0.0  0.0  0.0\n";   // Ixy Ixz Iyz — ok for symmetry
  // oss << "    ]\n";
  // oss << "    damping Damping {\n";
  // oss << "      linear  0.5\n";
  // oss << "      angular 2.0\n";
  // oss << "    }\n";
  // oss << "  }\n";

  // controller.
  oss << "  controller \"<extern>\"\n";
  // name.
  oss << "  name \"" << defName << "\"\n";
  // supervisor flag.
  oss << "  supervisor TRUE\n";
  oss << "}\n";
  
  return oss.str();
}


bool WorldGenerator::appendAgentsToWorldFile() {

  
  const char* path = base_world_path_.c_str();
  // Read the entire base world file into a string.
  std::ifstream inFile(path);
  if (!inFile) {
    RCLCPP_ERROR(this->get_logger(), "\nCould not open world file: %s", base_world_path_.c_str());
  return false;
  }

  RCLCPP_INFO(this->get_logger(), "Opening world file: %s", base_world_path_.c_str());

  std::stringstream buffer;
  buffer << inFile.rdbuf();
  std::string worldText = buffer.str();
  inFile.close();

  // Generate the agent definitions.
  std::ostringstream agentSegments;
  for (const auto &agent : agents_.agents) {
    RCLCPP_INFO(this->get_logger(), "Adding agent: %s", agent.name.c_str());
    agentSegments << generateAgentWBT(agent) << "\n";
  }

  // Append the agent definitions to the world text.
  worldText += "\n" + agentSegments.str();

  // Write the new world text back to the same file.
  std::ofstream outFile(path);
  if (!outFile) {
    RCLCPP_ERROR(this->get_logger(), "Failed to write to world file: %s", base_world_path_.c_str());
    return false;
  }
  outFile << worldText;
  outFile.close();

  RCLCPP_INFO(this->get_logger(), "Closing world file: %s", base_world_path_.c_str());

  return true;
}


// Generates a URDF document for a given agent.
std::string WorldGenerator::populateAgentURDF(const hunav_msgs::msg::Agent &agent, const std::string &overseer_name) {
  
  
  std::string skinName;
  switch(agent.skin) {
    case 0: skinName = "Robert"; break;
    case 1: skinName = "Sandra"; break;
    case 2: skinName = "Sophia"; break;
    case 3: skinName = "Anthony"; break;
    default: skinName = "Robert"; break;
  }
  
  XMLDocument doc;

  // Create the XML declaration.
  XMLDeclaration* decl = doc.NewDeclaration("xml version=\"1.0\" encoding=\"utf-8\"");
  doc.InsertFirstChild(decl);

  // Create the root element <robot> and set its "name" attribute using the agent's name.
  XMLElement* robotElem = doc.NewElement("robot");
  robotElem->SetAttribute("name", agent.name.c_str());
  doc.InsertEndChild(robotElem);

  // Create the <webots> element.
  XMLElement* webotsElem = doc.NewElement("webots");
  robotElem->InsertEndChild(webotsElem);

  // Create the <plugin> element and set its type.
  XMLElement* pluginElem = doc.NewElement("plugin");
  pluginElem->SetAttribute("type", "hunav_plugin::HuNavPlugin");
  webotsElem->InsertEndChild(pluginElem);
  
  // Add <agent_name> element.
  XMLElement* agentNameElem = doc.NewElement("agent_name");
  agentNameElem->SetText(agent.name.c_str());
  pluginElem->InsertEndChild(agentNameElem);

  // Add <agent_id> element.
  XMLElement* agentIdElem = doc.NewElement("agent_id");
  agentIdElem->SetText(agent.id);
  pluginElem->InsertEndChild(agentIdElem);

  // Add <agent_name> element.
  XMLElement* skinNameElem = doc.NewElement("skin_device_name");
  skinNameElem->SetText(skinName.c_str());
  pluginElem->InsertEndChild(skinNameElem);

  // Add <overseer_name> element.
  XMLElement* overseerNameElem = doc.NewElement("overseer_name");
  overseerNameElem->SetText(overseer_name.c_str());
  pluginElem->InsertEndChild(overseerNameElem);

  // Add <robot_name> element.
  XMLElement* robotNameElem = doc.NewElement("robot_name");
  robotNameElem->SetText(plug_robot_name_.c_str());
  pluginElem->InsertEndChild(robotNameElem);

  // Add <global_frame_to_publish> element.
  XMLElement* globalFrameElem = doc.NewElement("global_frame_to_publish");
  globalFrameElem->SetText(plug_global_frame_.c_str());  // Hard-coded here; adjust as needed.
  pluginElem->InsertEndChild(globalFrameElem);

  // Add <use_navgoal_to_start> element.
  XMLElement* useNavGoalElem = doc.NewElement("use_navgoal_to_start");
  std::string navgoal_str = plug_use_navgoal_to_start_ ? "true" : "false";
  useNavGoalElem->SetText(navgoal_str.c_str());
  pluginElem->InsertEndChild(useNavGoalElem);

  // Add <navgoal_topic> element.
  XMLElement* navGoalElem = doc.NewElement("navgoal_topic");
  // The topic could be constructed from agent.name, e.g. "my_agent/goal_pose"
  navGoalElem->SetText(plug_navgoal_topic_.c_str());
  pluginElem->InsertEndChild(navGoalElem);

  // Add <ignore_models> element.
  XMLElement* ignoreModelsElem = doc.NewElement("ignore_models");
  // Join the vector of strings into a single string.
  std::string ignoreModelsStr;
  for (size_t i = 0; i < plug_ignore_models_.size(); ++i) {
    ignoreModelsStr += plug_ignore_models_[i];
    if (i != plug_ignore_models_.size() - 1) {
      ignoreModelsStr += " ";
    }
  }
  ignoreModelsElem->SetText(ignoreModelsStr.c_str());
  pluginElem->InsertEndChild(ignoreModelsElem);

  // // Add the <controller> element to specify the external controller.
  // XMLElement* controllerElem = doc.NewElement("controller");
  // controllerElem->SetText("<extern>");
  // webotsElem->InsertEndChild(controllerElem);

  // Convert the document to a string.
  XMLPrinter printer;
  doc.Print(&printer);
  return std::string(printer.CStr());
}


// Example function to write URDF files for all agents in a vector.
bool WorldGenerator::generateAgentURDFFiles() {

  // 1) pick exactly one agent at random
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<size_t> dist(0, agents_.agents.size() - 1);
  const auto &selected = agents_.agents[dist(gen)];

  for (const auto &agent : agents_.agents) {

    std::string fileName = urdf_folder_ + agent.name + ".urdf";
    RCLCPP_INFO(this->get_logger(), "Opening URDF file: %s", fileName.c_str());

    std::string urdfContent = populateAgentURDF(agent, selected.name);

    std::ofstream outFile(fileName);
    if (!outFile) {
      // Log error (use your ROS logging here or std::cerr)
      RCLCPP_INFO(this->get_logger(), "Failed to open URDF file: %s", fileName.c_str());
      return false;
    }
    outFile << urdfContent;
    outFile.close();
  }
  return true;
}

}  // namespace hunav

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<hunav::WorldGenerator>());

  rclcpp::shutdown();
  return 0;
}
