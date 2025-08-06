cmake_policy(SET CMP0048 NEW)
project(ur5_ik VERSION 0.0.1)
cmake_minimum_required(VERSION 3.0.2)

add_compile_options(-std=c++17)


find_package(catkin REQUIRED
COMPONENTS
  roslib
  roscpp
  control_msgs
)
find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    roslib
    roscpp
    control_msgs
    DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

file(GLOB SRC_FILES src/*.cpp)
add_executable(${PROJECT_NAME}_node ${SRC_FILES})

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
