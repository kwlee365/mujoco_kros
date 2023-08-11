cmake_minimum_required(VERSION 3.0.2)
project(panda_controller)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  mujoco_ros
  eigen_conversions
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
add_library(${PROJECT_NAME}_lib
  src/controller.cpp
  src/mujoco_interface.cpp
)

add_executable(${PROJECT_NAME} src/main.cpp)

target_link_libraries(${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
  -lrbdl
  -lrbdl_urdfreader
)


target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
  )

add_dependencies(panda_controller_lib ${catkin_EXPORTED_TARGETS})
add_dependencies(panda_controller ${catkin_EXPORTED_TARGETS})