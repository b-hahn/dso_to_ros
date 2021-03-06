cmake_minimum_required(VERSION 2.4.6)
project(dso_to_ros)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
  message_filters
  pcl_catkin
  libpointmatcher_ros
)

set(DSO_PATH $ENV{DSO_PATH})
message("\n\n---- LOOKING FOR DSO at\n\"${DSO_PATH}\"")


find_package(Pangolin 0.2 REQUIRED)
message("\n\n---- FOUND Pangolin Headers at\n\"${Pangolin_INCLUDE_DIRS}\"")
message("---- FOUND Pangolin Libs at\n\"${Pangolin_LIBRARIES}\"")

find_package(OpenCV REQUIRED)
message("\n\n---- FOUND OpenCV Headers at\n\"${OpenCV_INCLUDE_DIRS}\"")
message("---- FOUND OpenCV Libs at\n\"${OpenCV_LIBS}\"")

find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS system thread)
find_library(DSO_LIBRARY dso ${DSO_PATH}/build/lib)
# TODO: find_package(segmatch) includes entire ROS/src folder along with
# wrong versions of Eigen - fix that, remove hard-coded path workaround

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    roscpp
    sensor_msgs
    geometry_msgs
    cv_bridge
    image_transport
    message_filters
    pcl_catkin
    libpointmatcher_ros
)

###########
## Build ##
###########

set(CMAKE_CXX_FLAGS
   "${SSE_FLAGS} -O2 -g -std=c++0x -march=native -fno-omit-frame-pointer"
)

include_directories(
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/include
  ${DSO_PATH}/src
  ${DSO_PATH}/thirdparty/Sophus
  ${Pangolin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/../segmap/segmatch/include  # TODO: avoid using hardcoded path here
)

## Declare a C++ executable
add_executable(dso_live src/main.cpp)

target_link_libraries(dso_live
    ${DSO_LIBRARY}
    ${Pangolin_LIBRARIES}
    ${OpenCV_LIBS}
    ${catkin_LIBRARIES})
