cmake_minimum_required(VERSION 3.8)
project(assi2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)

include_directories(
  include
  )

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

#Try1
#target_include_directories()

#Try2
#target_link_libraries(cmd_publisher
#  ${rclcpp_LIBRARIES}
#)

#Try3



add_executable(cmd_publisher src/cmd_publisher.cpp)
ament_target_dependencies(cmd_publisher rclcpp std_msgs geometry_msgs turtlesim)
install(TARGETS
  cmd_publisher
  DESTINATION lib/${PROJECT_NAME})

add_executable(pub src/pub.cpp)
ament_target_dependencies(pub rclcpp std_msgs geometry_msgs turtlesim)
install(TARGETS
  pub
  DESTINATION lib/${PROJECT_NAME})


# install header files
install(
  DIRECTORY include/
  DESTINATION include/
)

#install launch file
install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
