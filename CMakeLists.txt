cmake_minimum_required(VERSION 3.14.0)
project(slam2ref)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")
add_compile_options(-Wpedantic -Wall -Wextra -O3 -g)

#set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "/opt/ros/noetic")
#
#
#set(CATKIN_PACKAGES
#tf
#roscpp
#rospy
#cv_bridge
#pcl_conversions
#std_msgs
#sensor_msgs
#geometry_msgs
#nav_msgs
#message_generation)
#
#find_package(catkin REQUIRED COMPONENTS ${CATKIN_PACKAGES} QUIET)

find_package(OpenMP REQUIRED QUIET)
find_package(PCL REQUIRED QUIET)
find_package(OpenCV 4.2.0 REQUIRED QUIET)
find_package(GTSAM REQUIRED QUIET)
find_package(yaml-cpp REQUIRED) # MV to load parameters without ROS -> better for development
find_package(Open3D REQUIRED)
message(STATUS "Found Open3D: ${Open3D_VERSION} (found version \"${Open3D_VERSION}\")")

#catkin_package(
#  INCLUDE_DIRS include
#  DEPENDS PCL GTSAM
#
#  CATKIN_DEPENDS
#  std_msgs
#  nav_msgs
#  geometry_msgs
#  sensor_msgs
#  message_runtime
#  message_generation
#)

# include directories
include_directories(
	include
#  ${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
	${GTSAM_INCLUDE_DIR}
	${Open3D_INCLUDE_DIRS}
)



# link directories
link_directories(
  include
	${PCL_LIBRARY_DIRS}
  	${OpenCV_LIBRARY_DIRS}
  	${GTSAM_LIBRARY_DIRS}
	${Open3D_LIBRARY_DIRS}
)

###########
## Build ##
###########

# SLAM2REF
file(GLOB_RECURSE SLAM2REF_SRCS src/*.cpp)
file(GLOB_RECURSE SLAM2REF_HEADERS include/*.h)


# Add executable
add_executable(${PROJECT_NAME} ${SLAM2REF_SRCS} ${SLAM2REF_HEADERS})


#add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_compile_options(${PROJECT_NAME} PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(${PROJECT_NAME}
#  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${OpenMP_CXX_FLAGS}
  ${Open3D_LIBRARIES}
  gtsam
  stdc++fs
  yaml-cpp
)
