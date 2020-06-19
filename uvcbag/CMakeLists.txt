cmake_minimum_required(VERSION 2.8.3)
project(uvcbag)

add_compile_options(-std=c++11)
find_package(catkin REQUIRED rosbag roscpp std_msgs geometry_msgs actionlib_msgs cartographer_ros_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES uvcbag
  CATKIN_DEPENDS 
  DEPENDS system_lib
)
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${rosbag_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/uvcbag.cpp)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} rosbag)
 target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
 )

 install(TARGETS ${PROJECT_NAME}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
