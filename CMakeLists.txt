cmake_minimum_required(VERSION 2.8.6)

project(conman_barrett)

find_package(catkin REQUIRED
  COMPONENTS 
  rtt_ros 
  rtt_rosparam
  rtt_ros_tools
  rtt_sensor_msgs
  kdl_urdf_tools
  control_toolbox)

find_package(OROCOS-RTT REQUIRED)
find_package(Orocos-KDL REQUIRED)
find_package(Eigen REQUIRED)

include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

orocos_use_package(conman)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${USE_OROCOS_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${Orocos-KDL_INCLUDE_DIRS})
link_directories(
  ${catkin_LIBRARY_DIRS}
  ${Orocos-KDL_LIBRARY_DIRS}
  ${USE_OROCOS_LIBRARY_DIRS})

#orocos_component(${PROJECT_NAME}
  #src/conman_barrett.cpp
  #)

#target_link_libraries(
  #${PROJECT_NAME}
  #${USE_OROCOS_LIBRARIES} 
  #${catkin_LIBRARIES})

orocos_generate_package()
