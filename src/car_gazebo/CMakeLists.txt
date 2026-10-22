cmake_minimum_required(VERSION 3.8)
project(car_gazebo)

find_package(ament_cmake REQUIRED)

# Install the world and launch files into share/car_gazebo
install(
  DIRECTORY worlds launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
