cmake_minimum_required(VERSION 2.8.3)
project(image_cropping)

find_package(OpenCV 3.2.0 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_msgs
  message_generation
)

add_service_files(
   FILES
   image_crop_srv.srv
)

generate_messages(
  DEPENDENCIES
  sensor_msgs   
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES image_cropping
  CATKIN_DEPENDS cv_bridge image_transport roscpp rospy sensor_msgs std_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(client_node src/client_node.cpp)
target_link_libraries(client_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(client_node ${catkin_EXPORTED_TARGETS})

add_executable(server_node src/server_node.cpp)
target_link_libraries(server_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(server_node ${catkin_EXPORTED_TARGETS})