cmake_minimum_required(VERSION 3.0.2)
project(traversability_rotation_planning_plugins)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS roscpp l3_footstep_planning_plugins hector_stability_metrics hector_math)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp l3_footstep_planning_plugins hector_stability_metrics hector_math
)

include_directories(include ${catkin_INCLUDE_DIRS} ${boost_INCLUDE_DIRS})

set(HEADERS
  include/${PROJECT_NAME}/std/heuristic/traversability_rotation_heuristic.h
  include/${PROJECT_NAME}/std/step_cost_estimator/angular_step_cost_estimator.h
  include/${PROJECT_NAME}/std/step_cost_estimator/traversability_step_cost_estimator.h)

set(SOURCES
  src/std/heuristic/traversability_rotation_heuristic.cpp
  src/std/step_cost_estimator/angular_step_cost_estimator.cpp
  src/std/step_cost_estimator/traversability_step_cost_estimator.cpp)

add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${boost_LIBRARIES} ${PCL_LIBRARIES})

install(FILES plugins.xml
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
