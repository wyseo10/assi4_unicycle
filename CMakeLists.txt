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

include_directories(${PROJECT_SOURCE_DIR}/include)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(pub src/pub.cpp src/cmd_publisher.cpp)
ament_target_dependencies(pub rclcpp std_msgs geometry_msgs turtlesim)
install(TARGETS
  pub
  DESTINATION lib/${PROJECT_NAME})

#install launch file
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
