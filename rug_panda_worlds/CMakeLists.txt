cmake_minimum_required(VERSION 3.4)
project(rug_panda_worlds)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  plotjuggler
  gazebo_dev
  roscpp
  gazebo_ros_control
  boost_sml
  urdf
  franka_gazebo
)

list_insert_in_workspace_order(catkin_INCLUDE_DIRS ${Franka_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS
  plotjuggler
  roscpp
  gazebo_ros_control
  boost_sml
  urdf
  franka_gazebo
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

include_directories(include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
# Install world files
install(DIRECTORY world/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/world
)

# Install models
install(DIRECTORY models/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models
)

## Tools
include(${CMAKE_CURRENT_LIST_DIR}/../cmake/ClangTools.cmake OPTIONAL
  RESULT_VARIABLE CLANG_TOOLS
  )
if(CLANG_TOOLS)
  file(GLOB_RECURSE SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
  file(GLOB_RECURSE HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h
    )
  add_format_target(franka_gazebo FILES ${SOURCES} ${HEADERS})
  add_tidy_target(franka_gazebo
    FILES ${SOURCES}
    DEPENDS franka_hw_sim
    )
endif()
