cmake_minimum_required(VERSION 3.8)
project(my_cpp_pkg)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# create an executable
add_executable(cpp_node src/my_first_node.cpp) # executable (not file or node)
ament_target_dependencies(cpp_node rclcpp) #executable and rclcpp library
# not a standard CMAKE declaration, specific to ament build system
# If you just compile two lines above, you will not be able to run the node so install.

# install executable and place inside the workspace (lib folder)
install(TARGETS
  cpp_node
  DESTINATION lib/${PROJECT_NAME} # PROJECT_NAME is name of the package or my_cpp_pkg
)

ament_package()
