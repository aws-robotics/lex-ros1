cmake_minimum_required(VERSION 2.8.3)
project(lex_node)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  lex_common_msgs
  aws_common
  aws_ros1_common
)

if(AWSSDK_FOUND)
  set(SERVICE lex)
  AWSSDK_DETERMINE_LIBS_TO_LINK(SERVICE OUTPUT)
  link_directories("${AWSSDK_LIB_DIR}")
endif()

add_definitions(-DUSE_IMPORT_EXPORT)

set(LEX_LIBRARY_TARGET ${PROJECT_NAME}_lib)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${LEX_LIBRARY_TARGET}
  CATKIN_DEPENDS roscpp std_msgs lex_common_msgs aws_ros1_common
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${AWSSDK_INCLUDE_DIR}
)

add_library(${LEX_LIBRARY_TARGET}
  src/lex_node.cpp
  src/lex_param_helper.cpp
)

target_link_libraries(${LEX_LIBRARY_TARGET}
  ${aws_common_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OUTPUT}
)

add_executable(${PROJECT_NAME} src/main.cpp)

target_link_libraries(${PROJECT_NAME} ${LEX_LIBRARY_TARGET})

add_dependencies(${LEX_LIBRARY_TARGET} ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} ${LEX_LIBRARY_TARGET}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(
  DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(GMock QUIET)
  if(GMOCK_FOUND)
    add_rostest_gmock(test_lex_node
      test/test_lex_node.test
      test/lex_node_test.cpp
    )
    target_include_directories(test_lex_node PRIVATE
      include
      ${catkin_INCLUDE_DIRS}
      ${GMOCK_INCLUDE_DIRS}
    )
    target_link_libraries(test_lex_node
      ${PROJECT_NAME}_lib
      ${catkin_LIBRARIES}
      ${GMOCK_BOTH_LIBRARIES}
    )
  else()
    include_directories(/usr/include/gmock /usr/src/gmock)
    add_library(${PROJECT_NAME}_libgmock SHARED /usr/src/gmock/src/gmock-all.cc)
    add_rostest_gtest(test_lex_node
      test/test_lex_node.test
      test/lex_node_test.cpp
    )
    target_include_directories(test_lex_node PRIVATE
      include
      ${catkin_INCLUDE_DIRS}
    )   
    target_link_libraries(test_lex_node
      ${PROJECT_NAME}_lib
      ${catkin_LIBRARIES}
      ${PROJECT_NAME}_libgmock
    )
  endif()
endif()
