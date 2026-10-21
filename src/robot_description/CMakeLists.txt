cmake_minimum_required(VERSION 3.8)
project(robot_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY training_bot bruno
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
