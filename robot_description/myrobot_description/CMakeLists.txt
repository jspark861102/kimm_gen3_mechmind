cmake_minimum_required(VERSION 3.0.2)
project(myrobot_description)

find_package(catkin REQUIRED COMPONENTS)
  
catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

