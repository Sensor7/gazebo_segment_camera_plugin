cmake_minimum_required(VERSION 3.8)
project(object_camera_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(stihl_sim_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(gazebo REQUIRED)
find_package(cv_bridge REQUIRED)

# Note adding new plugins
# To add a new plugin please add a new library for every plugin. This enables us to refrence the
# library in the urdf file without having ambiguity between plugins. So just copy the build procedure
# for one of the existing packages. Also do not forget to export the library (at the end of the file).

# build lib for object_camera_plugin
add_library(object_camera_plugin src/object_camera_plugin.cpp)
add_library(object_camera_plugin::object_camera_plugin ALIAS object_camera_plugin)
target_compile_features(object_camera_plugin PUBLIC c_std_99 cxx_std_17) # Require C99 and C++17
target_include_directories(object_camera_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  object_camera_plugin
  "rclcpp"
  "gazebo_ros"
  "std_msgs"
  "sensor_msgs"
  "stihl_sim_msgs"
  "geometry_msgs"
  "tf2"
  "gazebo"
  "OpenCV"
  "visualization_msgs"
  "cv_bridge"
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(object_camera_plugin PRIVATE "OBJECT_CAMERA_PLUGIN_BUILDING_LIBRARY")
install(
  TARGETS object_camera_plugin
  EXPORT export_${PROJECT_NAME}
  DESTINATION share/${PROJECT_NAME}
)

# install(DIRECTORY models
# DESTINATION share/${PROJECT_NAME}
# FILES_MATCHING PATTERN "*"
# PATTERN ".*" EXCLUDE)
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)

  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)

# Do not forget to export your added library
ament_export_libraries(
  object_camera_plugin
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
