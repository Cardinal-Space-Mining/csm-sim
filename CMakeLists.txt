cmake_minimum_required(VERSION 3.5)
project(csm_sim)

find_package(ament_cmake REQUIRED)

install(
    DIRECTORY description launch
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
