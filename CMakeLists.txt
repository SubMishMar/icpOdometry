cmake_minimum_required(VERSION 2.8.3)
project(icpOdometry)

find_package(libpointmatcher CONFIG)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  #libpointmatcher
  libpointmatcher_ros
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
  tf2
  tf_conversions
)

find_package(libpointmatcher CONFIG)

catkin_package(
  CATKIN_DEPENDS geometry_msgs libpointmatcher libpointmatcher_ros nav_msgs roscpp rospy std_msgs tf tf2 tf_conversions
)

set(CMAKE_BUILD_TYPE RelWithDebInfo)

if(CMAKE_COMPILER_IS_GNUCXX)
	execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
	if (GCC_VERSION VERSION_GREATER 4.4 OR GCC_VERSION VERSION_EQUAL 4.4)
		add_definitions(-std=gnu++0x)
	else (GCC_VERSION VERSION_GREATER 4.4 OR GCC_VERSION VERSION_EQUAL 4.4)
		message(SEND_ERROR "You need GCC version 4.4 or greater to compile this library")
	endif (GCC_VERSION VERSION_GREATER 4.4 OR GCC_VERSION VERSION_EQUAL 4.4)
else(CMAKE_COMPILER_IS_GNUCXX)
	message(SEND_ERROR "You need partial C++0x support for N2351, N2672, N1984. Currently this build toolchain supports only GCC >= 4.4. If your compiler supports these specifications, please send us a patch.")
endif(CMAKE_COMPILER_IS_GNUCXX)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_definitions(-DHAVE_YAML_CPP)

include_directories(include ${catkin_INCLUDE_DIRS} ${libpointmatcher_INCLUDE_DIRS})

add_executable(icpOdometry src/icpOdometry_node.cpp)
#add_dependencies(icpOdometry ${PROJECT_NAME}_gencpp)
target_link_libraries(icpOdometry ${catkin_LIBRARIES} ${libpointmatcher_LIBRARIES})
