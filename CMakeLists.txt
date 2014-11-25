cmake_minimum_required(VERSION 2.8.6)

project(lcsr_barrett)

find_package(catkin REQUIRED COMPONENTS rtt_ros kdl_urdf_tools control_toolbox)

find_package(cmake_modules REQUIRED)

find_package(OROCOS-RTT REQUIRED)
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )
find_package(orocos_kdl REQUIRED)
find_package(Eigen REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${USE_OROCOS_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS})
link_directories(
  ${catkin_LIBRARY_DIRS}
  ${orocos_kdl_LIBRARY_DIRS}
  ${USE_OROCOS_LIBRARY_DIRS})

#orocos_component(${PROJECT_NAME}
  #src/lcsr_barrett.cpp
  #)

#target_link_libraries(
  #${PROJECT_NAME}
  #${USE_OROCOS_LIBRARIES} 
  #${catkin_LIBRARIES})

orocos_generate_package()
