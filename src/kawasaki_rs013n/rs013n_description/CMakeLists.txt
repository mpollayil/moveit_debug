cmake_minimum_required(VERSION 3.10.2)
project(rs013n_description)
find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY config launch world meshes urdf DESTINATION share/${PROJECT_NAME})