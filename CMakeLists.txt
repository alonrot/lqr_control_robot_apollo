cmake_minimum_required(VERSION 2.8.3)
project(pole_balancing_apollo)

# this command is required for autocomplete in vim
SET(CMAKE_EXPORT_COMPILE_COMMANDS 1)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# this sets all the required paths 
include("${CMAKE_SOURCE_DIR}/cmake/sl_cmake_global/LAB.cmake")

## non-sl dependance
# FIND_PACKAGE(Eigen3 3.2.8 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  rostest
  eigen_conversions
  std_msgs
  geometry_msgs
  visualization_msgs
  vicon_subscriber
  rosrt
  yaml_cpp
  tf
  Eigen
)

# we do this because SL libraries come with undefined symbol
# and can not easily be linked against. Non-sl dependant
# libraries can link against ${non_sl_catkin_LIBRARIES} without being
# annoyed by this
set(non_sl_catkin_LIBRARIES ${catkin_LIBRARIES})
set(non_sl_catkin_INCLUDE_DIRS ${catkin_INCLUDE_DIRS})

## sl dependance
find_package(catkin REQUIRED COMPONENTS
  utilities
  lwpr
  sl
  apollo
)

add_message_files(
  FILES
  Controller.msg
  CostValue.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


set(catkin_LIBRARIES ${non_sl_catkin_LIBRARIES} ${catkin_LIBRARIES})
set(catkin_INCLUDE_DIRS ${non_sl_catkin_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pole_balancing_apollo
  CATKIN_DEPENDS utilities lwpr sl apollo roscpp std_msgs rosrt message_runtime
)

find_package(xenomai_ros QUIET)
if(Xenomai_FOUND)
  add_xenomai_flags()
  include_directories( ${Xenomai_INCLUDE_DIR} )
endif()

include_directories(${catkin_INCLUDE_DIRS})
include_directories(include)

#################################
########### LIBRARIES ###########
#################################

# Needed for some of the tests:
add_library(fake_sensor src/tools/fake_sensor.cpp)
add_library(subscriber_pole_angle src/pole_angle_measurement/subscriber_pole_angle.cpp)
add_library(parameters src/pole_balancing_apollo/parameters.cpp)
add_library(feedback_control src/pole_balancing_apollo/feedback_control.cpp)
target_link_libraries(feedback_control ${non_sl_catkin_LIBRARIES})
add_library(read_write_interface tests/src/read_write_interface.cpp)
add_library(print_this src/tools/PrintThis.cpp)
add_library(print_math src/tools/PrintMath.cpp)

# This library needs to be generated apart:
add_library(apollo_interface src/pole_balancing_apollo/apollo_interface.cpp)
target_link_libraries(apollo_interface parameters SLtask SLcommon apollo_task apollo utility ${LAB_STD_LIBS} ${catkin_LIBRARIES})

#############################
########### NODES ###########
#############################

# Absolute path to nodes/config_nodes, accessible to all the nodes:
get_filename_component(CONFIG_NODES_PATH nodes/config_nodes/ ABSOLUTE)

# executable for broadcasting angle topic in ros. Assumes vicon_bridge running
add_executable(publisher_pole_angle_vicon nodes/publisher_pole_angle_vicon.cpp)
target_link_libraries(publisher_pole_angle_vicon rosrt vicon_subscriber)

# executable for broadcasting imaginary angle topic, for testing purpose 
# add_executable(publisher_fake_pole_angle nodes/publisher_fake_pole_angle.cpp)
# target_link_libraries(publisher_fake_pole_angle ${non_sl_catkin_LIBRARIES})

# subscribe to pole angle and broadcast marker arrays suitable for display in rviz
add_executable(publisher_pole_rviz_display nodes/publisher_pole_rviz_display.cpp)
target_link_libraries(publisher_pole_rviz_display ${non_sl_catkin_LIBRARIES})

#############################
########### TESTS ###########
#############################

  # executable for broadcasting angle topic in ros, test: broadcast 0 continuously
  add_executable(test_pole_angle_broadcast tests/test_pole_angle_broadcast.cpp)
  target_link_libraries(test_pole_angle_broadcast rosrt vicon_subscriber)

  # executable for testing the FakeSensor class:
  add_executable(test_fake_sensor tests/test_fake_sensor.cpp)
  target_link_libraries(test_fake_sensor fake_sensor rosrt)

  # executable for testing the FakeSensor class:
  add_executable(test_record_SL_data tests/test_record_SL_data.cpp)
  target_link_libraries(test_record_SL_data parameters read_write_interface ${non_sl_catkin_LIBRARIES})

  # executable for testing the real-time publisher:
  add_executable(test_rt_publisher tests/test_rt_publisher.cpp)
  target_link_libraries(test_rt_publisher rosrt)

  # executable for testing the real-time capabilities of PrintThis:
  add_executable(test_rt_print_this tests/test_rt_print_this.cpp)
  target_link_libraries(test_rt_print_this print_this rosrt)

# executable for testing the real-time capabilities of PrintThis:
  add_executable(test_rt_print_math tests/test_rt_print_math.cpp)
  target_link_libraries(test_rt_print_math print_math print_this rosrt)

  catkin_add_gtest(pole_balancing_ut   
    tests/main_test.cpp
    # tests/test_feedback_control.cpp  
    src/pole_balancing_apollo/parameters.cpp
    tests/test_parameters.cpp ) 
  # target_link_libraries(pole_balancing_ut feedback_control)
  get_filename_component(TEST_TMP_PATH tests/tmp ABSOLUTE)
  set_target_properties(pole_balancing_ut PROPERTIES  COMPILE_DEFINITIONS YAML_CONFIG_TEST_FILE="${TEST_TMP_PATH}/test_yaml_config.yaml")

#############################################
########### APOLLO POLE BALANCING ###########
#############################################

  # config folder:
  get_filename_component(CONFIG_FOLDER_PATH config/ ABSOLUTE)

  # pole_balancing_apollo sl-task
  add_library(pole_balancing_apollo src/pole_balancing_task.cpp
	                                  src/pole_balancing_apollo/parameters.cpp
	                                  src/pole_balancing_apollo/measurements.cpp 
	                                  src/pole_balancing_apollo/estimate_states.cpp 
	                                  src/pole_balancing_apollo/feedback_control.cpp
	                                  src/pole_balancing_apollo/StateMachineLearning.cpp
	                                  src/pole_angle_measurement/subscriber_pole_angle.cpp 
	                                  src/lqr_gains/ComputeLQRcost.cpp 
	                                  src/lqr_gains/GainsCollector.cpp 
	                                  src/tools/tools.cpp 
	                                  src/tools/fake_sensor.cpp 
	                                  src/tools/butter_filt.cpp
	                                  src/tools/PrintThis.cpp
	                                  src/tools/PrintMath.cpp)
  set_target_properties(pole_balancing_apollo PROPERTIES  LINK_FLAGS ${SL_IGNORE_UNDEF_SYMBOLS} 
                                                        COMPILE_DEFINITIONS YAML_CONFIG_FILE="${CONFIG_FOLDER_PATH}/task_parameters.yaml")
  target_link_libraries(pole_balancing_apollo SLtask SLcommon 
    apollo_task apollo apollo_interface utility ${LAB_STD_LIBS} ${catkin_LIBRARIES})


  # apollo_interface_debug_task test sl-task
  add_library(automatic_learning_task src/automatic_learning_task.cpp)
  add_dependencies(automatic_learning_task ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  set_target_properties(automatic_learning_task PROPERTIES  COMPILE_DEFINITIONS YAML_CONFIG_FILE="${CONFIG_FOLDER_PATH}/task_parameters.yaml")
  target_link_libraries(automatic_learning_task SLtask SLcommon 
    apollo_task apollo apollo_interface utility ${LAB_STD_LIBS} ${catkin_LIBRARIES})

  #message(FATAL_ERROR ${LAB_LIBDIR})
  install(TARGETS pole_balancing_apollo
    ARCHIVE DESTINATION ${LAB_LIBDIR}
    LIBRARY DESTINATION ${LAB_LIBDIR}
  )

