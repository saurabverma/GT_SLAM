cmake_minimum_required(VERSION 2.8.3)
project(blam_slam)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosbag
#  measurement_synchronizer
  #point_cloud_filter
  #point_cloud_odometry
  laser_loop_closure
  point_cloud_localization
  point_cloud_mapper
  geometry_utils
  parameter_utils
  pcl_ros
  pcl_conversions
)

find_package(PCL REQUIRED)
#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}
#  ${PROJECT_SOURCE_DIR}/src )
#find_package(LuaGoogle REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    rosbag
#    measurement_synchronizer
    #point_cloud_filter
    #point_cloud_odometry
    laser_loop_closure
    point_cloud_localization
    point_cloud_mapper
    geometry_utils
    parameter_utils
    pcl_ros
    pcl_conversions
)

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if (COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
  message(FATAL_ERROR "${CMAKE_CXX_COMPILER} doesn't provide c++11 support.")
endif()

include_directories(
  include
  include/dataFramework
  ${LUA_INCLUDE_DIR}
	  ${CERES_INCLUDE_DIRS}
	  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}
  	${PCL_INCLUDE_DIRS}
  )

include_directories(include ${catkin_INCLUDE_DIRS} /usr/include/lua5.2)
link_directories(${catkin_LIBRARY_DIRS})
add_library(${PROJECT_NAME} src/LOAM_loopClose.cc)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_library(LOAM_loopClose_node src/LOAM_loopClose.cc)
target_link_libraries(LOAM_loopClose_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(LOAM_loopClose_offline src/LOAM_loopClose_offline.cc src/proto_stream.cc src/rigid_transform.cc src/trajectory.pb.cc src/transform.cc src/transform_interpolation_buffer.cc src/lua_parameter_dictionary.cc src/transform.pb.cc src/time_conversion.cc src/time.cc)
target_link_libraries(LOAM_loopClose_offline
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${LUA_LIBRARIES}
  glog
  gflags
  protobuf
  /usr/lib/x86_64-linux-gnu/liblua5.2.so.0
)
