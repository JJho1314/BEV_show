cmake_minimum_required(VERSION 3.0.2)
project(point_cloud)

add_compile_options(-std=c++17)
set(CMAKE_BUILD_TYPE "Release")

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(PCL 1.8 REQUIRED)
find_package(OpenCV 3 REQUIRED)
find_package(Eigen3 REQUIRED)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_conversions     
  pcl_ros
  geometry_msgs
  laser_geometry
  sensor_msgs
  message_generation
  roslaunch
  cv_bridge
  image_transport
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES point_cloud
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})
add_executable(point_cloud src/test.cpp)

target_link_libraries(point_cloud ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${EIGEN3_LIBRARIES} ${catkin_LIBRARIES})
