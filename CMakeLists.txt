cmake_minimum_required(VERSION 3.5)
project(controller_plugins)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(navigation_lite REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)


add_library(controller_plugins 
  src/pure_pursuit_controller.cpp
  src/vhf_plus_controller.cpp
  src/histogram.cpp
  src/utils.cpp)
target_include_directories(controller_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(controller_plugins 
  Eigen3::Eigen)    
ament_target_dependencies( 
  controller_plugins
  "navigation_lite"
  "pluginlib"
  "tf2" 
  "tf2_ros" 
  "tf2_msgs"
  "tf2_geometry_msgs"
  "octomap"
  "octomap_msgs"
  "geometry_msgs"
  "nav_msgs"
)

pluginlib_export_plugin_description_file(navigation_lite controller_plugins.xml)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS controller_plugins
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  controller_plugins
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
