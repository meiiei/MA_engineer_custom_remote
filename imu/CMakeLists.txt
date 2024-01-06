cmake_minimum_required(VERSION 3.0.2)
project(imu)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Setting Cmake epxort command on
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#definitions
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

find_package(Eigen3 REQUIRED)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
            roscpp 
            std_msgs
            tf 
            sensor_msgs
            geometry_msgs 
            message_generation)

find_package(serial REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${serial_INCLUDE_DIRS}
)

# add_message_files(
#   FILES
#   bm.msg
# )

# generate_messages(
#   DEPENDENCIES
#   geometry_msgs
#   sensor_msgs
# )

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp tf std_msgs sensor_msgs message_runtime 
  DEPENDS EIGEN3 
)

add_executable(${PROJECT_NAME}1_pub_node src/imu1_pub.cpp)
add_executable(${PROJECT_NAME}2_pub_node src/imu2_pub.cpp)
add_executable(${PROJECT_NAME}3_pub_node src/imu3_pub.cpp)


target_link_libraries(${PROJECT_NAME}1_pub_node ${catkin_LIBRARIES} ${serial_LIBRARIES})
target_link_libraries(${PROJECT_NAME}2_pub_node ${catkin_LIBRARIES} ${serial_LIBRARIES})
target_link_libraries(${PROJECT_NAME}3_pub_node ${catkin_LIBRARIES} ${serial_LIBRARIES})

