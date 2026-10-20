cmake_minimum_required(VERSION 3.11)
project(global_relocalization)

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
find_package(ament_index_cpp REQUIRED)
find_package(OpenMP REQUIRED)
find_package(small_gicp QUIET)

# An interrupted FetchContent install can leave a small_gicp config file behind
# without its headers. Do not accept that incomplete package as a dependency.
if(small_gicp_FOUND AND TARGET small_gicp::small_gicp)
  get_target_property(
    SMALL_GICP_INCLUDE_DIRS small_gicp::small_gicp INTERFACE_INCLUDE_DIRECTORIES)
  set(SMALL_GICP_PCL_REGISTRATION_HEADER_FOUND FALSE)
  foreach(SMALL_GICP_INCLUDE_DIR IN LISTS SMALL_GICP_INCLUDE_DIRS)
    if(EXISTS "${SMALL_GICP_INCLUDE_DIR}/small_gicp/pcl/pcl_registration.hpp")
      set(SMALL_GICP_PCL_REGISTRATION_HEADER_FOUND TRUE)
      break()
    endif()
  endforeach()
  if(NOT SMALL_GICP_PCL_REGISTRATION_HEADER_FOUND)
    message(WARNING "Ignoring incomplete small_gicp installation; using FetchContent instead.")
    set(small_gicp_FOUND FALSE)
  endif()
endif()

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

target_link_libraries(${PROJECT_NAME}
  OpenMP::OpenMP_CXX
  small_gicp::small_gicp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN small_gicp_relocalization::SmallGicpRelocalizationNode
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

ament_auto_package()
