cmake_minimum_required(VERSION 3.8)
project(me_nav2_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_behaviors REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)

# Build SmartRecovery behavior plugin
add_library(me_nav2_bringup_plugins SHARED
  plugins/smart_recovery.cpp
  plugins/stall_aware_behaviors.cpp
)

target_include_directories(me_nav2_bringup_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/plugins>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(me_nav2_bringup_plugins
  rclcpp
  nav2_core
  nav2_behaviors
  nav2_msgs
  nav2_util
  nav2_costmap_2d
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  tf2
  tf2_geometry_msgs
  pluginlib
)

# Install plugin library
install(TARGETS me_nav2_bringup_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install plugin description
pluginlib_export_plugin_description_file(nav2_core plugins/behavior_plugin.xml)

install(DIRECTORY
  launch
  config
  behavior_trees
  pcd
  map
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/../../scripts/record_waypoints.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME record_waypoints
)

install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/../../scripts/exploration_ui.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME exploration_ui
)

install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/../../scripts/exploration_mode.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME exploration_mode
)

install(FILES README_BUNKER_MINI2.md
  DESTINATION share/${PROJECT_NAME}
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

  find_package(ament_cmake_gtest REQUIRED)
  find_package(behaviortree_cpp_v3 REQUIRED)
  ament_add_gtest(test_exploration_recovery_tree test/test_exploration_recovery_tree.cpp)
  ament_target_dependencies(test_exploration_recovery_tree behaviortree_cpp_v3)
  target_compile_definitions(test_exploration_recovery_tree PRIVATE
    RECOVERY_TREE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/behavior_trees/navigate_w_replanning_and_exploration_recovery.xml")
  ament_add_gtest(test_recovery_progress test/test_recovery_progress.cpp)
  target_include_directories(test_recovery_progress PRIVATE plugins)
  ament_add_gtest(test_escape_trajectory test/test_escape_trajectory.cpp)
  target_include_directories(test_escape_trajectory PRIVATE plugins)
  ament_add_gtest(test_escape_grid test/test_escape_grid.cpp)
  target_include_directories(test_escape_grid PRIVATE plugins)
endif()

ament_package()
