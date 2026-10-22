cmake_minimum_required(VERSION 3.8)
project(car_description)

find_package(ament_cmake REQUIRED)

# Install model, controller/bridge configs and RViz layout into share/car_description
install(
  DIRECTORY urdf config rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
