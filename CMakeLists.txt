cmake_minimum_required(VERSION 3.5)
project(hunav_webots_wrapper)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Besides the package specific dependencies we also need the `pluginlib` and `webots_ros2_driver`
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(webots_ros2_driver REQUIRED)
find_package(hunav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)

# Get WEBOTS installation folder "/home/user/.ros/webotsR2025a/webots"
set(WEBOTS_HOME $ENV{WEBOTS_HOME})

 # STEP 1: pull in Webots’ controller headers
 include_directories(
   ${WEBOTS_HOME}/include/controller/c      # C API headers
   ${WEBOTS_HOME}/include/controller/cpp    # C++ API headers
   ${WEBOTS_HOME}/include/webots            # <webots/bvh_util.h>, skin.h, etc.
 )

  # — NOW: bring in the **BVH util** sample sources ——
  set(BVH_UTIL_DIR "${WEBOTS_HOME}/projects/humans/skin_animated_humans/libraries/bvh_util")
  include_directories(
    ${BVH_UTIL_DIR}/include       # for <webots/bvh_util.h>
  )
  file(GLOB BVH_UTIL_SOURCES
    ${BVH_UTIL_DIR}/src/*.c
    ${BVH_UTIL_DIR}/src/*.cpp     # if any C++ helpers exist
  )

# Export the plugin configuration file
pluginlib_export_plugin_description_file(webots_ros2_driver hunav_plugin.xml)

include_directories(
  include
)

include_directories(SYSTEM
  ${TINYXML2_INCLUDE_DIR}
)

link_directories(
  ${TINYXML2_LIBRARY_DIRS}
)


add_executable(hunav_webots_world_generator
  src/WorldGenerator.cpp
)
ament_target_dependencies(hunav_webots_world_generator
  rclcpp
  geometry_msgs
  sensor_msgs
  hunav_msgs
  tf2_geometry_msgs
  tf2
  tf2_ros
  tinyxml2_vendor
  TinyXML2
)

install(TARGETS
  hunav_webots_world_generator
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY include/
  DESTINATION include
)

# HunavPlugin library
add_library(
  ${PROJECT_NAME}
  SHARED
  src/HuNavPlugin.cpp
)
target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)
ament_target_dependencies(
  ${PROJECT_NAME}
  pluginlib
  rclcpp
  webots_ros2_driver
  hunav_msgs
  geometry_msgs
  tf2 
  tf2_geometry_msgs
)

 # **compile the BVH util sources into your plugin**:
 target_sources(${PROJECT_NAME}
   PRIVATE
     ${BVH_UTIL_SOURCES}
 )

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
# Install additional directories.
install(DIRECTORY
  launch
  resource
  worlds
  scenarios
  DESTINATION share/${PROJECT_NAME}/
)
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_package()