cmake_minimum_required(VERSION 3.5)

project(gazebo_maliput)

# Skip if Ignition not present
find_package(ignition-gazebo3 QUIET)
if(NOT ignition-gazebo3_FOUND)
  message(WARNING "Ignition Gazebo 3 not found, proceeding without that simulator.")
  return()
endif()

find_package(ament_cmake REQUIRED)
# find_package(ignition_gazebo6 REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY
    ign
    launch
    models
    worlds
  DESTINATION
    share/${PROJECT_NAME}/
)

add_subdirectory(plugin/pure_pursuit_maliput)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.sh.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

ament_package()
