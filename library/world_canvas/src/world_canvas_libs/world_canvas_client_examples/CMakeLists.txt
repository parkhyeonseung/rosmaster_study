cmake_minimum_required(VERSION 2.8.3)
project(world_canvas_client_examples)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  unique_id
  uuid_msgs
  nav_msgs
  yocs_msgs
  world_canvas_msgs
  world_canvas_client_cpp
)

catkin_package(
  CATKIN_DEPENDS roscpp unique_id uuid_msgs nav_msgs yocs_msgs world_canvas_msgs world_canvas_client_cpp
)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
include_directories(${catkin_INCLUDE_DIRS})

add_executable(test_annotation_collection src/test_annotation_collection.cpp)
target_link_libraries(test_annotation_collection ${catkin_LIBRARIES})

install(TARGETS test_annotation_collection DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
