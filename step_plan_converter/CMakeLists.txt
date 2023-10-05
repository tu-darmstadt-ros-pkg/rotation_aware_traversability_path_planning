cmake_minimum_required(VERSION 3.0.2)
project(step_plan_converter)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS roscpp l3_footstep_planning traversability_map_generator)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp l3_footstep_planning traversability_map_generator
  #  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/step_plan_converter.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(step_plan_converter_node src/step_plan_converter_node.cpp)

target_link_libraries(step_plan_converter_node ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} step_plan_converter_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
  )

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.launch"
  )
