cmake_minimum_required(VERSION 3.8)
project(agent_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)

## C++ executable
#add_executable(my_cpp_node src/my_cpp_node.cpp)
#target_include_directories(my_cpp_node PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>)
#target_link_libraries(my_cpp_node ${rclcpp_LIBRARIES})
#
#install(TARGETS my_cpp_node
#  DESTINATION lib/${PROJECT_NAME})

# For Python
find_package(rclpy REQUIRED)
#find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  agent_control/consensus.py 
  agent_control/LF_formation.py 
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY agent_control launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
