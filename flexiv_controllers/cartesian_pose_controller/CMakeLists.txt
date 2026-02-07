cmake_minimum_required(VERSION 3.5)
project(cartesian_pose_controller LANGUAGES CXX)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  builtin_interfaces
  controller_interface
  geometry_msgs
  generate_parameter_library
  hardware_interface
  pluginlib
  rclcpp_lifecycle
  rcutils
)

generate_parameter_library(cartesian_pose_controller_parameters
  src/cartesian_pose_controller_parameters.yaml
)

add_library(${PROJECT_NAME}
  SHARED
  src/cartesian_pose_controller.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE include)

target_link_libraries(${PROJECT_NAME} PUBLIC
  cartesian_pose_controller_parameters
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(controller_interface cartesian_pose_controller.xml)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS
    ${PROJECT_NAME}
    cartesian_pose_controller_parameters
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
