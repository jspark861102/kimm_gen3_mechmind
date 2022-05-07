cmake_minimum_required(VERSION 3.0.2)
project(kimm_gen3_pnp)

add_compile_options(-std=c++17 -Ofast)
add_definitions(-D_OS_UNIX)

INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/python.cmake)
INCLUDE(cmake/ide.cmake)
INCLUDE(cmake/apple.cmake)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
set(KORTEX_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/../thirdparty/include")
set(KORTEX_LIBRARY_DIR "${PROJECT_SOURCE_DIR}/../thirdparty/lib")
LIST( APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}" )

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  geometry_msgs
  mujoco_ros
  hardware_interface
  dynamic_reconfigure    
  pluginlib
  realtime_tools    
)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp  
    geometry_msgs
    mujoco_ros    
    hardware_interface
    dynamic_reconfigure
    pluginlib
    realtime_tools
    controller_interface                
  DEPENDS system_lib
)

###########
## Build ##
###########
link_directories(${Boost_LIBRARY_DIR})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

## library
# SET(${PROJECT_NAME}_HEADERS  
  
# )
# SET(${PROJECT_NAME}_SOURCES  
  
# )

### include kinova kortex library
set(THRIDPARTY_LIB ${THRIDPARTY_LIB} ${KORTEX_LIBRARY_DIR}/release/libKortexApiCpp.a)
set(THRIDPARTY_INC ${THRIDPARTY_INC} ${KORTEX_INCLUDE_DIRS} ${KORTEX_INCLUDE_DIRS}/client ${KORTEX_INCLUDE_DIRS}/common ${KORTEX_INCLUDE_DIRS}/messages ${KORTEX_INCLUDE_DIRS}/client_stubs ${KORTEX_INCLUDE_DIRS}/google)

# ADD_SOURCE_GROUP(${PROJECT_NAME}_HEADERS)
# ADD_SOURCE_GROUP(${PROJECT_NAME}_SOURCES)
# ADD_LIBRARY(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_HEADERS})
# TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} SYSTEM PUBLIC $<INSTALL_INTERFACE:include> ${EIGEN3_INCLUDE_DIR} ${THRIDPARTY_INC})
# TARGET_LINK_LIBRARIES(${PROJECT_NAME} PUBLIC ${THRIDPARTY_LIB} -lpthread)

### exe 
add_executable(kimm_gen3_pnp_v1 src/kimm_gen3_pnp_v1.cpp)
# target_link_libraries(kimm_gen3_pnp_v1 ${PROJECT_NAME} ${catkin_LIBRARIES})
# TARGET_INCLUDE_DIRECTORIES(kimm_gen3_pnp_v1 SYSTEM PUBLIC $<INSTALL_INTERFACE:include>)
target_link_libraries(kimm_gen3_pnp_v1 ${catkin_LIBRARIES} ${THRIDPARTY_LIB})
TARGET_INCLUDE_DIRECTORIES(kimm_gen3_pnp_v1 SYSTEM PUBLIC $<INSTALL_INTERFACE:include> ${EIGEN3_INCLUDE_DIR} ${THRIDPARTY_INC})

#############
## Install ##
#############
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


