cmake_minimum_required(VERSION 2.8.3)
project(handover_moveit)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  baxter_core_msgs
  roscpp
  moveit_ros_planning_interface
  rosbag
  tf2
  cv_bridge
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
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
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
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

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
#   baxter_core_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
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
#  LIBRARIES handover_moveit
#  CATKIN_DEPENDS baxter_core_msgs roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  image_transport/include
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/handover_moveit.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide

# handover source files
set(HandoverSrc
        src/HandoverController.cpp include/HandoverController.h
        src/msg_reader/PoseMsgReader.cpp include/msg_reader/PoseMsgReader.h
        src/head/HeadHandCoordinationEnable.cpp include/head/HeadHandCoordinationEnable.h
        src/Gripper.cpp include/Gripper.h
        src/Scene.cpp include/Scene.h
        src/RightArm.cpp include/RightArm.h
)
# head source files
set(HeadSrc
        src/head/HeadPan.cpp include/head/HeadPan.h
        src/head/HeadHandCoordination.cpp include/head/HeadHandCoordination.h
        src/head/HeadState.cpp include/head/HeadState.h
        src/head/HeadPanCommand.cpp include/head/HeadPanCommand.h
        src/head/Screen.cpp include/head/Screen.h
        src/head/FaceImage.cpp include/head/FaceImage.h
        src/head/Head.cpp include/head/Head.h
        src/head/Look.cpp include/head/Look.h)
# reuse object files
add_library(handover_objects OBJECT ${HandoverSrc})
add_library(head_objects OBJECT ${HeadSrc})
add_library(release_handover_objects OBJECT src/ReleaseController.cpp include/ReleaseController.h)

#add_executable(moveit_tutorial_node src/nodes/moveit_tutorial_node.cpp src/msg_reader/PoseMsgReader.cpp include/msg_reader/PoseMsgReader.h)
add_executable(give_node src/nodes/give_node.cpp $<TARGET_OBJECTS:handover_objects>)
add_executable(take_object_back_and_get_ready_to_give_node src/nodes/take_object_back_and_get_ready_to_give_node.cpp $<TARGET_OBJECTS:handover_objects>)
add_executable(back_to_ready_to_take_object_node src/nodes/back_to_ready_to_take_object_node.cpp $<TARGET_OBJECTS:handover_objects>)
add_executable(get_almost_ready_node src/nodes/get_almost_ready_node.cpp $<TARGET_OBJECTS:handover_objects>)
add_executable(test_node src/nodes/test/test_node.cpp $<TARGET_OBJECTS:handover_objects>)
add_executable(gripper_test_node src/nodes/test/gripper_test_node.cpp src/Gripper.cpp include/Gripper.h)
add_executable(head_hand_coordination_node src/nodes/head/head_hand_coordination_node.cpp $<TARGET_OBJECTS:head_objects>)
add_executable(face_node src/nodes/head/face_node.cpp include/head/Face.h src/head/Face.cpp $<TARGET_OBJECTS:head_objects>)
add_executable(face_image_node src/nodes/test/face_image_node.cpp include/head/FaceImage.h src/head/FaceImage.cpp)
add_executable(release_node src/nodes/test/test_release_node.cpp src/Gripper.cpp include/Gripper.h src/ReleaseController.cpp include/ReleaseController.h)
add_executable(test_scene src/nodes/test/test_scene.cpp src/Scene.cpp include/Scene.h)

# different releases
add_executable(rigid_release_node src/nodes/experiment/rigid_release_node.cpp $<TARGET_OBJECTS:handover_objects> $<TARGET_OBJECTS:release_handover_objects>)
add_executable(passive_release_node src/nodes/experiment/passive_release_node.cpp $<TARGET_OBJECTS:handover_objects> $<TARGET_OBJECTS:release_handover_objects>)
add_executable(proactive_release_node src/nodes/experiment/proactive_release_node.cpp $<TARGET_OBJECTS:handover_objects> $<TARGET_OBJECTS:release_handover_objects> include/ProactiveReleaseController.h src/DataStream.cpp include/DataStream.h)

# experiment
add_executable(look_at_participants_node src/nodes/experiment/look_at_participants_node.cpp $<TARGET_OBJECTS:handover_objects> $<TARGET_OBJECTS:head_objects>)

add_executable(test_force_data src/nodes/test/test_force_data.cpp include/DataStream.h)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(test_scene ${catkin_LIBRARIES})
#target_link_libraries(moveit_tutorial_node ${catkin_LIBRARIES})
target_link_libraries(give_node ${catkin_LIBRARIES})
target_link_libraries(take_object_back_and_get_ready_to_give_node ${catkin_LIBRARIES})
target_link_libraries(back_to_ready_to_take_object_node ${catkin_LIBRARIES})
target_link_libraries(get_almost_ready_node ${catkin_LIBRARIES})
target_link_libraries(test_node ${catkin_LIBRARIES})
target_link_libraries(gripper_test_node ${catkin_LIBRARIES})
target_link_libraries(head_hand_coordination_node ${catkin_LIBRARIES})
target_link_libraries(face_node ${catkin_LIBRARIES})
target_link_libraries(face_image_node ${catkin_LIBRARIES})
target_link_libraries(release_node ${catkin_LIBRARIES})
# different releases
target_link_libraries(rigid_release_node ${catkin_LIBRARIES})
target_link_libraries(passive_release_node ${catkin_LIBRARIES})
target_link_libraries(proactive_release_node ${catkin_LIBRARIES})
#experiment
target_link_libraries(look_at_participants_node ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_handover_moveit.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
