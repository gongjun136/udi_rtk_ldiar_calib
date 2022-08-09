cmake_minimum_required(VERSION 3.0.2)
project(lidar_imu_calib)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -msse4.2")
set(CMAKE_BUILD_TYPE "RELEASE")
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_ros
#  ndt_omp
  message_generation
  geometry_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   Vel.msg
   PosVel.msg
#   Message1.msg
#   Message2.msg
 )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )
generate_messages(DEPENDENCIES
        std_msgs
        geometry_msgs
#        sensor_msgs
#        nav_msgs
        )
set(ALL_TARGET_LIBRARIES "")

include(cmake/PCL.cmake)
include(cmake/glog.cmake)
include(cmake/global_defination.cmake)
include(cmake/YAML.cmake)
include(cmake/geographic.cmake)
include(cmake/ceres.cmake)

include_directories(include ${catkin_INCLUDE_DIRS})



################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lidar_imu_calib
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time)

set(PANGOLIN_DIR "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/build-pangolin")
find_package(Pangolin REQUIRED)

#add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/Kontiki)

# Set link libraries used by all binaries
#第三方库
list(APPEND thirdparty_libraries
        ${Boost_LIBRARIES}
        ${catkin_LIBRARIES}
        ${Pangolin_LIBRARIES}

#        Kontiki
        )
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
#原文件构成的库
add_library(
        li_calib_lib
#        src/models/trajectory_manager/trajectory_manager.cpp
#        src/calib_lidar_imu/calib_lidar_imu_flow.cpp
        src/subscriber/cloud_subscriber.cpp
        src/subscriber/imu_subscriber.cpp
        src/sensor_data/imu_data.cpp
        src/models/registration/ndt_registration.cpp
        src/models/registration/icp_registration.cpp
#        src/models/registration/icp_registration.cpp
        src/other_data/lidar_odometry.cpp
        src/models/cloud_filter/voxel_filter.cpp
        src/models/cloud_filter/no_filter.cpp
#        src/models/inertial_initializer/inertial_initializer.cpp
        src/publisher/odometry_publisher.cpp
        src/publisher/cloud_publisher.cpp
        src/models/scan_adjust/distortion_adjust.cpp
        src/sensor_data/velocity_data.cpp
        src/sensor_data/gnss_data.cpp
        src/subscriber/gnss_subscriber.cpp
        src/publisher/imu_publisher.cpp
        src/data_pretreat/data_pretreat_flow.cpp
        src/subscriber/velocity_subscriber.cpp src/publisher/vel_publisher.cpp
        src/publisher/vel_publisher.cpp
        src/sensor_data/pos_vel_data.cpp
        src/publisher/pos_vel_publisher.cpp
        src/init_orientation/init_orientation_flow.cpp
        src/init_orientation/init_orientation.cpp
        src/sensor_data/pose_data.cpp
        src/subscriber/odometry_subscriber.cpp
        src/sensor_data/key_frame.cpp
        src/back_end_optimization/back_end_optimization.cpp
        src/back_end_optimization/back_end_optimization_flow.cpp
        src/publisher/tf_broadcaster.cpp
        src/subscriber/tf_listener.cpp
        src/publisher/key_frame_publisher.cpp
        src/subscriber/key_frame_subscriber.cpp
        src/models/cloud_filter/box_filter.cpp
        src/models/hand_eye_calibration/hand_eye_calibration.cpp
)
target_link_libraries(li_calib_lib ${thirdparty_libraries} ${ALL_TARGET_LIBRARIES})

file(GLOB_RECURSE ALL_SRCS "*.cpp")
file(GLOB_RECURSE NODE_SRCS "src/apps/*_node.cpp")
#list(REMOVE_ITEM ALL_SRCS ${NODE_SRCS})
add_executable(data_pretreat_node src/app/data_pretreat_node.cpp)
add_dependencies(data_pretreat_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(data_pretreat_node li_calib_lib)

add_executable(init_orientation_node src/app/init_orientation_node.cpp)
add_dependencies(init_orientation_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(init_orientation_node li_calib_lib)

add_executable(back_end_optimization_node src/app/back_end_optimization_node.cpp)
add_dependencies(back_end_optimization_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(back_end_optimization_node li_calib_lib)


#add_executable(calib_lidar_imu_node  src/app/calib_lidar_imu_node.cpp include/lidar_imu_calib/subscriber/odometry_subscriber.hpp)
#add_dependencies(calib_lidar_imu_node ${catkin_EXPORTED_TARGETS})
#target_link_libraries(calib_lidar_imu_node li_calib_lib)

#install(DIRECTORY
#        include/
#        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#        FILES_MATCHING PATTERN "*.h"
#        PATTERN ".svn" EXCLUDE
#        )
#
### Mark other directories for installation:
#
#install(TARGETS
#        calib_lidar_imu_node
#        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#        )
#install(DIRECTORY
#        launch/
#        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#        FILES_MATCHING PATTERN "*.launch"
#        )
#install(DIRECTORY
#        config/
#        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
#        FILES_MATCHING PATTERN "*.yaml"
#        )
#install(DIRECTORY
#        rviz/
#        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
#        FILES_MATCHING PATTERN "*.rviz"
#        )
## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/lidar_imu_calib.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/lidar_imu_calib_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_lidar_imu_calib.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
