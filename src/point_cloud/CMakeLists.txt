cmake_minimum_required(VERSION 3.0.2)
project(point_cloud)

add_compile_options(-std=c++17)
set(CMAKE_BUILD_TYPE "Release")

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(PCL  1.9 REQUIRED)
find_package(OpenCV 4  REQUIRED)
find_package(Eigen3 REQUIRED)
find_package( OpenMP REQUIRED)
if(OPENMP_FOUND)
    message("OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_conversions 
  autoware_msgs    
  pcl_ros
  geometry_msgs
  laser_geometry
  sensor_msgs
  message_generation
  roslaunch
  cv_bridge
  image_transport
  jsk_rviz_plugins
  jsk_recognition_msgs
)

generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
        sensor_msgs
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
  include/h264_decoder
  ${catkin_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})
add_executable(point_cloud src/main.cpp src/test.cpp src/PracticalSocket.cpp src/h264_decoder.cpp)

target_link_libraries(point_cloud ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${EIGEN3_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(point_cloud ${OpenCV_LIBRARIES}
avformat avdevice  avcodec avutil  avfilter postproc  swresample swscale)