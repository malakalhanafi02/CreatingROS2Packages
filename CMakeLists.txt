cmake_minimum_required(VERSION 3.5)
project(robot_arm_package)

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)

install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
