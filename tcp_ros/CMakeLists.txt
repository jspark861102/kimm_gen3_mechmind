cmake_minimum_required(VERSION 2.8.3)
project(tcp_ros)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  message_generation
  std_msgs
)

add_message_files(
  FILES
  Position.msg
  GetResult.msg
  Recipe.msg
  Status.msg
  Trigger.msg
  Feedback.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include 
  CATKIN_DEPENDS message_runtime std_msgs
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_executable(tcp_server_node
  src/tcp_server.cpp
  src/parser.cpp
  src/coordinate.cpp
)
add_dependencies(tcp_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tcp_server_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})


add_executable(tcp_client_node
  src/tcp_client.cpp
  src/parser.cpp
  src/coordinate.cpp
)
add_dependencies(tcp_client_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tcp_client_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(
  DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  TARGETS tcp_server_node tcp_client_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
