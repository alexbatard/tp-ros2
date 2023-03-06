cmake_minimum_required(VERSION 3.8)
project(tp1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(nodeA src/main_nodeA.cpp)
ament_target_dependencies(nodeA rclcpp std_msgs)
target_link_libraries(nodeA "${cpp_typesupport_target}")
target_include_directories(nodeA PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)

add_executable(nodeB src/main_nodeB.cpp)
ament_target_dependencies(nodeB rclcpp std_msgs)
target_link_libraries(nodeB "${cpp_typesupport_target}")
target_include_directories(nodeB PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)

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

install(TARGETS nodeA
DESTINATION lib/${PROJECT_NAME})

install(TARGETS nodeB
DESTINATION lib/${PROJECT_NAME})

ament_package()
