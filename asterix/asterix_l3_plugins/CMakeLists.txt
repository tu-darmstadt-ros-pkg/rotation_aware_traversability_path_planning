cmake_minimum_required(VERSION 3.0.2)
project(asterix_l3_plugins)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS roscpp l3_plugins l3_footstep_planning_plugins)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES asterix_l3_plugins
  CATKIN_DEPENDS roscpp l3_plugins l3_footstep_planning_plugins
  #  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

set(HEADERS
  include/${PROJECT_NAME}/asterix_kinematics.h
  include/${PROJECT_NAME}/asterix_turning_state_generator.h
  )

set(SOURCES
  src/asterix_kinematics.cpp
  src/asterix_turning_state_generator.cpp
  )

add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(FILES l3_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(FILES l3_footstep_planning_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
  )

