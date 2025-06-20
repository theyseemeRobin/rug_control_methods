cmake_minimum_required(VERSION 3.0.2)
project(rug_panda_planners)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  rospy
  moveit_commander
)
catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    rospy
)

# Ensure Python scripts are executable
file(GLOB_RECURSE PYTHON_SCRIPTS RELATIVE ${CMAKE_SOURCE_DIR} "scripts/*.py")
foreach(script ${PYTHON_SCRIPTS})
    execute_process(COMMAND chmod +x ${CMAKE_SOURCE_DIR}/${script})
endforeach()