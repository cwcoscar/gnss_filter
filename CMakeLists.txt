cmake_minimum_required(VERSION 3.0.2)
project(gnss_filter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(DIRECTORY msg)
# add_message_files(
#   FILES
#   gnss_csv.msg
#   ndt_csv.msg
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
 CATKIN_DEPENDS 
 roscpp 
 rospy 
 std_msgs 
 message_runtime
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(gnss_csv_filter src/gnss_csv_filter.cpp)

target_link_libraries(gnss_csv_filter ${catkin_LIBRARIES})

add_dependencies(gnss_csv_filter
  ${catkin_EXPORTED_TARGETS}
)

add_executable(ndt_csv_filter src/ndt_csv_filter.cpp)

target_link_libraries(ndt_csv_filter ${catkin_LIBRARIES})

add_dependencies(ndt_csv_filter
  ${catkin_EXPORTED_TARGETS}
)

add_executable(gnss_raw_csv_filter src/gnss_raw_csv_filter.cpp)

target_link_libraries(gnss_raw_csv_filter ${catkin_LIBRARIES})

add_dependencies(gnss_raw_csv_filter
  ${catkin_EXPORTED_TARGETS}
)

#############
## Install ##
#############

install(TARGETS
  gnss_csv_filter
  ndt_csv_filter
  gnss_raw_csv_filter
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(PROGRAMS
  scripts/uwb_csv_recorder.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )

