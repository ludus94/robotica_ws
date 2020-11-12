cmake_minimum_required(VERSION 3.1.3)
project(fanuc_movit_config)

find_package(catkin REQUIRED COMPONENTS
  rosconsole
  roscpp
  tf2_ros
  tf2
  geometry_msgs
  std_msgs
)


catkin_package()
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
add_executable(Listener Listener/Listener.cpp)
target_link_libraries(Listener
   ${catkin_LIBRARIES}
 )
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
