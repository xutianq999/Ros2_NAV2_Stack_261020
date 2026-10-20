cmake_minimum_required(VERSION 3.11)
project(global_small_gicp_relocalization)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
# add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
find_package(OpenMP REQUIRED)
find_package(small_gicp QUIET)
ament_auto_find_build_dependencies()

if(NOT small_gicp_FOUND)
  # Not found, fallback to FetchContent
  message(STATUS
    "System-installed small_gicp not found. Attempting to fetch small_gicp via FetchContent..."
  )

  include(FetchContent)
  fetchcontent_declare(
    small_gicp
    GIT_REPOSITORY https://github.com/koide3/small_gicp.git
    GIT_TAG "master"
  )

  fetchcontent_makeavailable(small_gicp)
endif()

###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

# find_package 找到的 small_gicp 导出带命名空间的 target (small_gicp::small_gicp)，
# FetchContent 拉取的源码 target 为 small_gicp，两者择一链接才能继承头文件路径
if(TARGET small_gicp::small_gicp)
  set(SMALL_GICP_TARGET small_gicp::small_gicp)
else()
  set(SMALL_GICP_TARGET small_gicp)
endif()

target_link_libraries(${PROJECT_NAME}
  OpenMP::OpenMP_CXX
  ${SMALL_GICP_TARGET}
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN global_small_gicp_relocalization::SmallGicpRelocalizationNode
  EXECUTABLE ${PROJECT_NAME}_node
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_SOURCE_DIR}/.clang-format")
  set(ament_cmake_clang_tidy_CONFIG_FILE "${CMAKE_SOURCE_DIR}/.clang-tidy")
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

#############
## Install ##
#############

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
