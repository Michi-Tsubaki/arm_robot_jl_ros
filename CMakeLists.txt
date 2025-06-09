cmake_minimum_required(VERSION 3.5)
project(arm_robot_jl_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  rospack
  cv_bridge
  catkin_virtualenv
)

catkin_generate_virtualenv(
  PYTHON_INTERPRETER python3
  INPUT_REQUIREMENTS requirements.txt
  USE_SYSTEM_PACKAGES TRUE
  ISOLATE_REQUIREMENTS TRUE
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    sensor_msgs
    rospack
    cv_bridge
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(CODE "
  execute_process(
    COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/scripts/precompile_julia.sh
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
")

install(PROGRAMS
  src/simple_controller.jl
  src/simple_simulator.jl
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
